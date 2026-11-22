/*
 * bd1662.c -- BD1662 / BD16XX PPG (photoplethysmography) sensor driver
 *
 * This is a clean-room re-implementation written from the disassembly of the
 * vendor driver that ships inside the factory kernel image of the XinTaiQi
 * S16 / S16B Plus / W527 smartwatch (Unisoc UWS6152, Linux 4.14.98, arm64).
 *
 * There is no public source for this driver; every constant, register and ABI
 * string below was recovered from
 *     D:\nekoboot\out\factory_kernel.img
 * Symbols come from the image's embedded kallsyms
 *     (drivers/input/misc/bd1662.c -> .text 0x562f98 .. 0x563d4c,
 *      .init.text 0xc750b8 / 0xc9d0b0, .data 0xfc3148, .bss 0x13dd2b8).
 * See REVERSE_ENGINEERING.md for the per-offset evidence table.
 *
 * The chip exposes FOUR Android sensors through ONE input device named
 * "bd1662" plus a sysfs class "bd1662" whose "device" child carries the
 * "hrenable" (and "hrtest") attributes the ROM HAL depends on:
 *
 *     /sys/class/bd1662/device/hrenable      <- writable enable
 *     /dev/input/eventN  (name "bd1662")     <- data + finger motion
 *
 * Data path:  a kthread ("polling-mode-task") wakes up, reads the PPG result
 * registers 0x1c..0x1f through plain i2c_msg transfers, feeds a small
 * accumulation/"frame" filter and, when a frame completes (return code 1),
 * replays the accumulated buffer into the input device as
 * EV_REL/REL_X (HR/SpO2/VP sample frames), EV_ABS/ABS_X|Y|Z (accelerometer
 * samples handed over by the accelerometer driver) and EV_KEY/0
 * markers.
 *
 * NEKOBOOT PORT NOTES
 * -------------------
 * Comments tagged "nekoboot:" mark deviations from the factory code.  They are
 * limited to (a) NULL guards, (b) a bounded wait in the polling thread so that
 * kthread_stop() cannot deadlock, (c) naming the module-local i2c busy
 * counter, and (d) probe/remove bookkeeping the factory leaks.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/atomic.h>
#include <linux/pm.h>
#include <linux/sched.h>

#include "bd1662.h"

#define BD1662_TAG			"[bd1662] "
#define BD1662_INFO(fmt, args...)	printk(KERN_INFO BD1662_TAG fmt, ##args)
#define BD1662_ERR(fmt, args...)	printk(KERN_ERR  BD1662_TAG fmt, ##args)

/* ------------------------------------------------------------------------ */
/* driver state                                                             */
/* ------------------------------------------------------------------------ */

/*
 * factory layout (all in .bss, all file offsets = link offset):
 *   0x13dd2b8  g_bd1662_data        struct bd1662_data *
 *   0x13dd2c0  bp_data_up           u16 = 7000
 *   0x13dd2c2  bp_data_down         u16 = 1000
 *   0x13dd2c4  bp_difdata           u16 = 20
 *   0x13dd2c8  bd_hrs_data[36]      u16 sample ring
 *   0x13dd318  bd_start_flag        u8
 *   0x13dd319  data_count           u8
 *   0x13dd31a  reflect_abssum_data  u16
 */
static struct bd1662_data *g_bd1662_data;

static u16 bp_data_up;
static u16 bp_data_down;
static u16 bp_difdata;
static u16 bd_hrs_data[BD1662_HRS_DEPTH];
static u8  bd_start_flag;
static u8  data_count;
static u16 reflect_abssum_data;

/*
 * Accelerometer sample ring that BD_16XX_dataread() replays as EV_ABS events.
 * In the factory build these are *globals defined by the mir3da/DA217
 * accelerometer driver* (0x13dce60 gsensor_index, 0x13dce68 gsensor_x[128],
 * 0x13dcf68 gsensor_y[128], 0x13dd068 gsensor_z[128] - "B" symbols shared
 * between two translation units).
 *
 * nekoboot: our tree's mir3da port does not define them, so the port carries
 * its own copies.  They stay all-zero, which means the EV_ABS/EV_KEY frames
 * that the "finger motion detect" sensor consumes are emitted with value 0
 * until someone wires the accelerometer in.  To restore the factory behaviour
 * make these non-static and let mir3da write them (see REVERSE_ENGINEERING.md,
 * section "gsensor ring").
 */
static s16 gsensor_x[BD1662_GSENSOR_DEPTH];
static s16 gsensor_y[BD1662_GSENSOR_DEPTH];
static s16 gsensor_z[BD1662_GSENSOR_DEPTH];
static u32 gsensor_index;

/*
 * The factory serialises i2c transfers on bus 2 through a counter that it
 * *shares with the och1970 hall driver* (0x13dd3c8 och1970_i2c_count, bumped
 * with __ll_sc_atomic_add).  Our och1970 port does not export that symbol, so
 * the port keeps a module private counter with the same semantics.
 * factory: 0x5633f8 / 0x563428 / 0x563448
 */
static atomic_t bd1662_i2c_count = ATOMIC_INIT(0);

/* ------------------------------------------------------------------------ */
/* i2c primitives                                                           */
/* ------------------------------------------------------------------------ */

/*
 * bd1662_i2c_Read - factory 0x563398, exported symbol BD_16XX era helper.
 *
 * Builds a two-message i2c_msg array (write txbuf, then read rxbuf) and runs
 * i2c_transfer() on the client's adapter while holding the shared bus counter.
 * Returns the i2c_transfer() result: 2 on success, negative on failure, -1 if
 * the bus stayed busy for BD1662_I2C_BUSY_RETRIES attempts.
 */
static int bd1662_i2c_Read(u8 *txbuf, u16 txlen, u8 *rxbuf, u16 rxlen)
{
	struct bd1662_data *data = g_bd1662_data;
	struct i2c_msg msgs[2];
	int ret;
	int retry;

	if (!data || !data->client)
		return -ENODEV;			/* nekoboot: NULL guard */

	msgs[0].addr  = data->client->addr;
	msgs[0].flags = 0;
	msgs[0].len   = txlen;
	msgs[0].buf   = txbuf;

	msgs[1].addr  = data->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len   = rxlen;
	msgs[1].buf   = rxbuf;

	for (retry = 0; atomic_read(&bd1662_i2c_count); retry++) {
		if (retry >= BD1662_I2C_BUSY_RETRIES)
			return -1;
		usleep_range(BD1662_I2C_BUSY_SLEEP_US_MIN,
			     BD1662_I2C_BUSY_SLEEP_US_MAX);
	}

	atomic_inc(&bd1662_i2c_count);
	ret = i2c_transfer(data->client->adapter, msgs, 2);
	atomic_dec(&bd1662_i2c_count);

	return ret;
}

/*
 * bd1662_i2c_Write - factory 0x5634ec.
 * Single write message, same busy-counter protocol as the read helper.
 */
static int bd1662_i2c_Write(u8 *buf, u16 len)
{
	struct bd1662_data *data = g_bd1662_data;
	struct i2c_msg msg;
	int ret;
	int retry;

	if (!data || !data->client)
		return -ENODEV;			/* nekoboot: NULL guard */

	msg.addr  = data->client->addr;
	msg.flags = 0;
	msg.len   = len;
	msg.buf   = buf;

	for (retry = 0; atomic_read(&bd1662_i2c_count); retry++) {
		if (retry >= BD1662_I2C_BUSY_RETRIES)
			return -1;
		usleep_range(BD1662_I2C_BUSY_SLEEP_US_MIN,
			     BD1662_I2C_BUSY_SLEEP_US_MAX);
	}

	atomic_inc(&bd1662_i2c_count);
	ret = i2c_transfer(data->client->adapter, &msg, 1);
	atomic_dec(&bd1662_i2c_count);

	return ret;
}

/*
 * BD_I2C_Read - factory 0x563478.
 *
 * Writes one register byte and reads one byte back.
 * The factory signature carries a leading redundant i2c address
 * (BD_I2C_Read(0x38, reg, &ok), see 0x563c00); the port drops it because the
 * address already lives in the i2c_client.
 *
 * *ok is set to 1 on success, 0 on failure.  Returns the byte that was read
 * (0 when the transfer failed).
 */
static u8 BD_I2C_Read(u8 reg, u8 *ok)
{
	u8 txbuf;
	u8 rxbuf;
	int ret;

	txbuf = reg;
	rxbuf = 0;

	ret = bd1662_i2c_Read(&txbuf, 1, &rxbuf, 1);
	if (ret < 0) {
		if (ok)
			*ok = 0;
		return 0;
	}

	if (ok)
		*ok = 1;

	return rxbuf;
}

/*
 * BD_I2C_Write - factory 0x5635b4.
 *
 * Emits {reg, val} as one 2-byte i2c write and returns 1 on success, 0 on
 * failure (factory: "mvn w0,w0; lsr w0,w0,#31", i.e. ret >= 0).
 * The factory passes a redundant leading i2c address (0x38) which it never
 * uses; the port drops it.
 */
static int BD_I2C_Write(u8 reg, u8 val)
{
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;

	return (bd1662_i2c_Write(buf, 2) >= 0) ? 1 : 0;
}

/*
 * bd1662_test_read_and_write - factory 0x563608.
 *
 * Writes {0x05, 0x08}, waits 100 ms, reads register 0x05 back and reports
 * whether it still reads 0x08.  Used by the "hrtest" sysfs attribute (value
 * '2') as a live chip presence check.
 */
static int bd1662_test_read_and_write(void)
{
	u8 wbuf[2];
	u8 got;

	wbuf[0] = BD1662_CHIPID_REG;
	wbuf[1] = BD1662_VAL_CFG5_CHIPID;

	bd1662_i2c_Write(wbuf, 2);
	msleep(100);

	got = BD_I2C_Read(BD1662_CHIPID_REG, NULL);

	return (got == wbuf[1]) ? 1 : 0;
}

/* ------------------------------------------------------------------------ */
/* LED gpio                                                                 */
/* ------------------------------------------------------------------------ */

/*
 * LED_PWM_On / LED_PWM_Off - factory 0x563084 / 0x5630ac.
 * Both are exported symbols in the factory kallsyms.
 *   gpiod_set_raw_value(gpio_to_desc(data->led_gpio), 1) / (..., 0)
 */
void LED_PWM_On(void)
{
	struct bd1662_data *data = g_bd1662_data;

	if (!data)
		return;				/* nekoboot: NULL guard */

	gpiod_set_raw_value(gpio_to_desc(data->led_gpio), 1);
}

void LED_PWM_Off(void)
{
	struct bd1662_data *data = g_bd1662_data;

	if (!data)
		return;				/* nekoboot: NULL guard */

	gpiod_set_raw_value(gpio_to_desc(data->led_gpio), 0);
}

/* ------------------------------------------------------------------------ */
/* measurement algorithm block                                              */
/* ------------------------------------------------------------------------ */

/*
 * BD_Sensor_Init - factory 0x563a70.
 *
 * Clears bd_start_flag and pushes the five configuration bytes.  Returns
 * non-zero only if every write succeeded ("uxtb/and" chain over the five
 * return values).
 */
static u8 BD_Sensor_Init(void)
{
	u8 ok;

	bd_start_flag = 0;

	ok  = (u8)BD_I2C_Write(BD1662_REG_CFG1,  BD1662_VAL_CFG1);
	ok &= (u8)BD_I2C_Write(BD1662_REG_CFG4,  BD1662_VAL_CFG4);
	ok &= (u8)BD_I2C_Write(BD1662_REG_CFG5,  BD1662_VAL_CFG5);
	ok &= (u8)BD_I2C_Write(BD1662_REG_CFG14, BD1662_VAL_CFG14);
	ok &= (u8)BD_I2C_Write(BD1662_REG_CFG15, BD1662_VAL_CFG15);

	return ok;
}

/*
 * BD_Sensor_Start - factory 0x563b2c.
 *
 * Sets bd_start_flag *before* the write, resets the frame counters, writes
 * 0x00 = 0x01 (run) and switches the LED on when that succeeded.
 */
static u8 BD_Sensor_Start(void)
{
	u8 ok;

	data_count = 0;
	bd_start_flag = 1;
	reflect_abssum_data = 0;

	ok = (u8)BD_I2C_Write(BD1662_REG_CTRL0, BD1662_VAL_CTRL0_START);

	if (ok)
		LED_PWM_On();

	return ok;
}

/*
 * BD_Sensor_Close - factory 0x563b84.
 * Clears bd_start_flag, switches the LED off and writes 0x00 = 0x00 (stop).
 */
static u8 BD_Sensor_Close(void)
{
	u8 ok;

	bd_start_flag = 0;
	LED_PWM_Off();

	ok = (u8)BD_I2C_Write(BD1662_REG_CTRL0, BD1662_VAL_CTRL0_STOP);

	return ok;
}

/*
 * BD_Sensor_Data_Read - factory 0x563bbc.
 *
 * Reads the two 16-bit little-endian sample pairs:
 *     v1 = regs[0x1d] << 8 | regs[0x1c]
 *     v2 = regs[0x1f] << 8 | regs[0x1e]
 * accumulates |v1 - v2| into reflect_abssum_data, stores v1 into the sample
 * ring and v2 into out[30], then decides whether a frame is complete:
 *
 *     mode != 1 : complete once data_count > 0x1c (28)
 *     mode == 1 : complete once data_count > 2
 *
 * On completion out[31] = reflect_abssum_data >> 2, data_count and
 * reflect_abssum_data are cleared and the function returns 1.
 *
 * Return values (this is what BD_16XX_dataread() keys off):
 *     0  sensor not started (bd_start_flag != 1)
 *     1  frame complete - emit it
 *     2  frame still accumulating, or the trailing ack write failed
 *
 * NOTE: the four "ok" flags are ANDed together exactly as in the factory,
 * including the quirk that the first flag gates the second (0x563c2c).
 */
static u8 BD_Sensor_Data_Read(u8 mode, u16 *out)
{
	u8 ok = 0;
	u8 flag;
	u8 ret;
	u8 t;
	u16 v1;
	u16 v2;

	if (bd_start_flag != 1)
		return 0;

	t = BD_I2C_Read(BD1662_REG_DATA0_L, &flag);
	v1 = (u16)t;
	if (flag)
		ok = flag;

	t = BD_I2C_Read(BD1662_REG_DATA0_H, &flag);
	v1 |= ((u16)t) << 8;
	if (ok)
		ok = flag;

	t = BD_I2C_Read(BD1662_REG_DATA1_L, &flag);
	v2 = (u16)t;
	if (ok)
		ok = flag;

	t = BD_I2C_Read(BD1662_REG_DATA1_H, &flag);
	v2 |= ((u16)t) << 8;
	if (ok)
		ok = flag;

	if (v1 > v2) {
		u16 delta = (u16)(v1 - v2);

		reflect_abssum_data = (u16)(reflect_abssum_data +
					    (delta >> BD1662_ABS_ACC_SHIFT));
	}

	out[BD1662_HRS_SLOT_RAW] = v2;
	out[data_count] = v1;

	ret = 0;
	if (!ok)
		goto ack;

	if (mode == 1) {
		if (data_count <= BD1662_THRESHOLD_MODE1)
			goto next;
	} else {
		if (data_count <= BD1662_THRESHOLD_MODE0)
			goto next;
	}

	/* frame complete */
	ret = 1;
	out[BD1662_HRS_SLOT_REFLECT] =
		(u16)(reflect_abssum_data >> BD1662_REFLECT_SHIFT);
	data_count = 0;
	reflect_abssum_data = 0;
	goto ack;

next:
	data_count++;

ack:
	flag = (u8)BD_I2C_Write(BD1662_REG_ACK, BD1662_VAL_ACK);
	if (!flag || !ok)
		ret = 2;

	return ret;
}

/* ------------------------------------------------------------------------ */
/* input event path                                                         */
/* ------------------------------------------------------------------------ */

/*
 * BD_16XX_dataread - factory 0x56320c.
 *
 * Called by the polling thread.  Reads a frame into bd_hrs_data[] and, when a
 * frame completed, replays it into the input device:
 *
 *   input_event(dev, EV_REL, REL_Y,   gsensor_index)              [always]
 *   for i in 0..34:
 *       if (data->mode && gsensor_index > i)
 *           input_event(dev, EV_ABS, ABS_X, gsensor_x[i] << 2)
 *           input_event(dev, EV_ABS, ABS_Y, gsensor_y[i] << 2)
 *           input_event(dev, EV_ABS, ABS_Z, gsensor_z[i] << 2)
 *       input_event(dev, EV_REL, REL_X, bd_hrs_data[i] ? : 0xffff)
 *       input_event(dev, EV_SYN, 0, 0)
 *
 * The gsensor_index snapshot is consumed (zeroed) before the replay, and the
 * EV_KEY/0 marker in bd_hrs_data[] is replaced by 0xffff when it is zero.
 */
static void BD_16XX_dataread(void)
{
	struct bd1662_data *data = g_bd1662_data;
	struct input_dev *input;
	u32 count;
	int i;

	if (!data || !data->input_dev)
		return;				/* nekoboot: NULL guard */

	input = data->input_dev;

	if (BD_Sensor_Data_Read((u8)data->mode, bd_hrs_data) != 1)
		return;

	count = gsensor_index;
	gsensor_index = 0;

	input_event(input, EV_REL, REL_Y, (int)count);

	for (i = 0; i < BD1662_HRS_EMIT_COUNT; i++) {
		if (data->mode && count > (u32)i) {
			input_event(input, EV_ABS, ABS_X,
				    (int)(gsensor_x[i] << BD1662_GSENSOR_SCALE_SHIFT));
			input_event(input, EV_ABS, ABS_Y,
				    (int)(gsensor_y[i] << BD1662_GSENSOR_SCALE_SHIFT));
			input_event(input, EV_ABS, ABS_Z,
				    (int)(gsensor_z[i] << BD1662_GSENSOR_SCALE_SHIFT));
		}

		input_event(input, EV_REL, REL_X,
			    bd_hrs_data[i] ? (int)bd_hrs_data[i] : 0xffff);
		input_event(input, EV_SYN, 0, 0);
	}
}

/*
 * BD_16XX_polling_task - factory 0x56333c, kthread body.
 *
 *   while (!kthread_should_stop()) {
 *       wait_for_completion(&data->completion);
 *       BD_16XX_dataread();
 *       if (data->enable)
 *           complete(&data->completion);
 *       msleep(data->delay_ms);
 *   }
 *
 * nekoboot: the wait is bounded.  The factory's unbounded
 * wait_for_completion() deadlocks kthread_stop() whenever the sensor was
 * disabled first, because nothing ever calls complete() again.  While the
 * sensor is enabled complete() is called on every iteration, so the wait
 * returns immediately and the polling cadence is unchanged.
 */
static int BD_16XX_polling_task(void *arg)
{
	struct bd1662_data *data = g_bd1662_data;

	(void)arg;

	if (!data)
		return 0;			/* nekoboot: NULL guard */

	while (!kthread_should_stop()) {
		wait_for_completion_timeout(&data->completion,
					    msecs_to_jiffies(BD1662_WAIT_SLICE_MS));
		BD_16XX_dataread();
		if (data->enable)
			complete(&data->completion);
		msleep((unsigned int)data->delay_ms);
	}

	return 0;
}

/* ------------------------------------------------------------------------ */
/* setup helpers                                                            */
/* ------------------------------------------------------------------------ */

/*
 * bd1662_input_dev_init - factory 0x563714.
 *
 * Allocates and registers the input device that the HAL opens by name.
 * Capability/abs-param calls are reproduced in the factory's exact order,
 * including the three redundant input_set_abs_params() calls at the beginning
 * (the later ABS_X/ABS_Y calls overwrite them).
 *
 * Note: like the factory, dev->dev.parent is deliberately not set.
 */
static int bd1662_input_dev_init(struct bd1662_data *data)
{
	struct input_dev *input;
	int ret;

	input = input_allocate_device();
	if (!input) {
		printk(KERN_ERR "%s: can't allocate device!\n", __func__);
		return -ENOMEM;
	}

	input->name = BD1662_I2C_NAME;
	input->id.bustype = BUS_I2C;

	input_set_capability(input, EV_ABS, ABS_X);
	input_set_capability(input, EV_ABS, ABS_Y);
	input_set_capability(input, EV_ABS, ABS_Z);
	input_set_capability(input, EV_REL, REL_X);
	input_set_capability(input, EV_REL, REL_Y);

	input_set_abs_params(input, ABS_X, 0, 0xffff, 0, 0);
	input_set_abs_params(input, ABS_X, 0, 0xffff, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 0xffff, 0, 0);
	input_set_abs_params(input, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_Z, -32768, 32767, 0, 0);

	input_set_drvdata(input, data);

	ret = input_register_device(input);
	if (ret < 0) {
		printk(KERN_ERR "%s: can't register device!\n", __func__);
		input_free_device(input);
		return ret;
	}

	data->input_dev = input;

	return 0;
}

/*
 * bd1662_thread_init - factory 0x5638b0.
 *
 * init_completion(&data->completion) followed by
 * kthread_create(BD_16XX_polling_task, NULL, "polling-mode-task") and
 * wake_up_process() when kthread_create() did not fail.
 */
static int bd1662_thread_init(struct bd1662_data *data)
{
	struct task_struct *thread;

	init_completion(&data->completion);

	thread = kthread_create(BD_16XX_polling_task, NULL, BD1662_THREAD_NAME);
	if (!IS_ERR(thread))
		wake_up_process(thread);

	data->thread = thread;

	return 0;
}

/*
 * bd1662_led_init - factory 0x563920.
 *
 *   gpio = of_get_named_gpio_flags(client->dev.of_node,
 *                                  "bd166x,led-gpio", 0, NULL);
 *   data->led_gpio = gpio;
 *   if ((unsigned)gpio <= 0x1ff) {
 *       gpio_request(gpio, "hr_led_pin");
 *       gpiod_direction_output_raw(gpio_to_desc(gpio), 0);
 *   }
 *
 * The range test is on the *unsigned* value, which is how the factory rejects
 * negative errnos from of_get_named_gpio_flags().
 */
static void bd1662_led_init(struct bd1662_data *data)
{
	struct device_node *np;
	int gpio;

	if (!data || !data->client)
		return;				/* nekoboot: NULL guard */

	np = data->client->dev.of_node;
	gpio = of_get_named_gpio_flags(np, BD1662_DT_LED_GPIO, 0, NULL);
	data->led_gpio = gpio;

	if ((u32)gpio <= (u32)BD1662_GPIO_MAX) {
		if (gpio_request(gpio, BD1662_LED_GPIO_LABEL) == 0)
			data->led_gpio_requested = 1;
		gpiod_direction_output_raw(gpio_to_desc(data->led_gpio), 0);
	}
}

/*
 * BD_16XX_init_start - factory 0x5630d4.
 * Runs the init + start sequence, programs the algorithm constants and kicks
 * the polling thread through the completion.
 */
static int BD_16XX_init_start(struct bd1662_data *data)
{
	u8 inited;
	u8 started;

	inited = BD_Sensor_Init();
	started = BD_Sensor_Start();

	bp_data_up = BD1662_BP_DATA_UP;
	bp_data_down = BD1662_BP_DATA_DOWN;
	bp_difdata = BD1662_BP_DIFDATA;

	if (!data)
		return 0;			/* nekoboot: NULL guard */

	complete(&data->completion);

	return (inited != 0) && (started != 0);
}

/* BD_16XX_close - factory 0x5631f4 */
static int BD_16XX_close(void)
{
	BD_Sensor_Close();
	return 0;
}

/* ------------------------------------------------------------------------ */
/* sysfs ABI                                                                */
/* ------------------------------------------------------------------------ */

/*
 * hrenable_show - factory 0x562ff4.
 *   printk("gxy, func=%s, line=%d, bEnable = 0x%x\n", __func__, 387, enable);
 *   return snprintf(buf, 4096, "%d\n", enable);
 * The literal 387 is the vendor's __LINE__; it is kept so the log line matches
 * the factory byte for byte.
 */
static ssize_t hrenable_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct bd1662_data *data = g_bd1662_data;

	(void)dev;
	(void)attr;

	if (!data)				/* nekoboot: NULL guard */
		return snprintf(buf, 4096, "%d\n", 0);

	printk("gxy, func=%s, line=%d, bEnable = 0x%x\n",
	       __func__, 387, data->enable);

	return snprintf(buf, 4096, "%d\n", data->enable);
}

/*
 * hrtest_show - factory 0x563054.
 *   return snprintf(buf, 4096, "%s\n", "bd1662");
 */
static ssize_t hrtest_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	(void)dev;
	(void)attr;

	return snprintf(buf, 4096, "%s\n", BD1662_I2C_NAME);
}

/*
 * hrenable_store - factory 0x563148.
 *
 *   '0' / 0  -> if enabled: disable the sensor, then mode = 0
 *   '1' / 1  -> if disabled: init+start, enable = 1, mode = 0
 *   '2' / 2  -> if disabled: init+start, enable = 2, mode = 1
 *
 * Both the ASCII digit and the raw byte value are accepted, exactly as the
 * factory compares them.
 */
static ssize_t hrenable_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct bd1662_data *data = g_bd1662_data;

	(void)dev;
	(void)attr;

	if (!data)				/* nekoboot: NULL guard */
		return count;

	if (buf[0] == '0' || buf[0] == 0) {
		if (data->enable) {
			data->enable = 0;
			BD_16XX_close();
		}
		data->mode = 0;
	} else if (buf[0] == '1' || buf[0] == 1) {
		if (!data->enable) {
			BD_16XX_init_start(data);
			data->enable = 1;
		}
		data->mode = 0;
	} else if (buf[0] == '2' || buf[0] == 2) {
		if (!data->enable) {
			BD_16XX_init_start(data);
			data->enable = 2;
			data->mode = 1;
		}
	}

	return count;
}

/*
 * hrtest_store - factory 0x563694.
 *
 *   '0' / 0 -> LED off
 *   '1' / 1 -> LED on
 *   '2' / 2 -> bd1662_test_read_and_write()
 */
static ssize_t hrtest_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct bd1662_data *data = g_bd1662_data;

	(void)dev;
	(void)attr;

	if (!data)				/* nekoboot: NULL guard */
		return count;

	if (buf[0] == '0' || buf[0] == 0) {
		gpiod_set_raw_value(gpio_to_desc(data->led_gpio), 0);
	} else if (buf[0] == '1' || buf[0] == 1) {
		gpiod_set_raw_value(gpio_to_desc(data->led_gpio), 1);
	} else if (buf[0] == '2' || buf[0] == 2) {
		bd1662_test_read_and_write();
	}

	return count;
}

/*
 * Attributes and their modes, straight out of the factory .data:
 *   0xfc3230 dev_attr_hrtest    mode 0644 (0x1a4 at +0x08)
 *   0xfc3250 dev_attr_hrenable  mode 0644
 * The ROM HAL writes /sys/class/bd1662/device/hrenable; "hrtest" is the
 * vendor's factory-test hook (LED / chip id).
 */
static DEVICE_ATTR(hrenable, 0644, hrenable_show, hrenable_store);
static DEVICE_ATTR(hrtest,   0644, hrtest_show,   hrtest_store);

/* ------------------------------------------------------------------------ */
/* i2c driver                                                               */
/* ------------------------------------------------------------------------ */

static struct class *bd1662_class;
static struct device *bd1662_class_dev;

/*
 * bd1662_i2c_probe - factory 0x563984.
 *
 *   1. i2c_check_functionality(client->adapter, I2C_FUNC_I2C) -> -ENODEV
 *   2. kzalloc(sizeof(struct bd1662_data))                    -> -ENOMEM
 *   3. g_bd1662_data = data; data->client = client;
 *      i2c_set_clientdata(client, data)
 *   4. bd1662_input_dev_init / bd1662_thread_init / bd1662_led_init
 *   5. data->delay_ms = 50; data->enable = 0
 *   6. class_create(THIS_MODULE, "bd1662")
 *      device_create(class, NULL, 0, NULL, "device")
 *      device_create_file(dev, &dev_attr_hrtest)
 *      device_create_file(dev, &dev_attr_hrenable)
 *
 * The factory ignores the return value of the three setup helpers and never
 * checks class_create()/device_create(); the port keeps the same behaviour but
 * logs the failures (nekoboot).
 */
static int bd1662_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct bd1662_data *data;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	data = kzalloc(sizeof(struct bd1662_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	g_bd1662_data = data;
	data->client = client;
	i2c_set_clientdata(client, data);

	ret = bd1662_input_dev_init(data);
	if (ret)				/* nekoboot: factory ignores it */
		BD1662_ERR("input dev init failed: %d\n", ret);

	bd1662_thread_init(data);
	bd1662_led_init(data);

	data->delay_ms = BD1662_DEFAULT_DELAY_MS;
	data->enable = 0;

	bd1662_class = class_create(THIS_MODULE, BD1662_CLASS_NAME);
	if (IS_ERR(bd1662_class)) {
		BD1662_ERR("class_create(%s) failed\n", BD1662_CLASS_NAME);
		bd1662_class = NULL;
		return 0;			/* factory ignores this too */
	}

	bd1662_class_dev = device_create(bd1662_class, NULL, 0, NULL,
					 BD1662_CLASS_DEV_NAME);
	if (IS_ERR(bd1662_class_dev)) {
		BD1662_ERR("device_create(%s) failed\n",
			   BD1662_CLASS_DEV_NAME);
		bd1662_class_dev = NULL;
		return 0;
	}

	device_create_file(bd1662_class_dev, &dev_attr_hrtest);
	device_create_file(bd1662_class_dev, &dev_attr_hrenable);

	/* nekoboot: single bring-up log; the factory prints nothing here */
	BD1662_INFO("probed: i2c-%d addr 0x%02x, input \"%s\", class \"%s\"\n",
		    client->adapter->nr, client->addr,
		    BD1662_I2C_NAME, BD1662_CLASS_NAME);

	return 0;
}

/*
 * bd1662_i2c_remove - factory 0x562fa8.
 *
 *   if (data->thread) { kthread_stop(data->thread); data->thread = NULL; }
 *   input_unregister_device(data->input_dev);
 *   input_free_device(data->input_dev);
 *   kfree(data);
 *
 * nekoboot: the port stops the sensor first so the LED and the chip are left
 * in a defined state, and it tears the sysfs class down (the factory leaks
 * both); input_free_device() is dropped because
 * input_unregister_device() already releases the device.
 * The kthread_stop() deadlock the factory has is avoided by the bounded wait
 * in BD_16XX_polling_task().
 */
static int bd1662_i2c_remove(struct i2c_client *client)
{
	struct bd1662_data *data = g_bd1662_data;

	(void)client;

	if (!data)
		return 0;

	if (data->enable) {
		data->enable = 0;
		BD_16XX_close();
	}

	if (data->thread && !IS_ERR(data->thread)) {
		kthread_stop(data->thread);
		data->thread = NULL;
	}

	if (bd1662_class_dev) {
		device_remove_file(bd1662_class_dev, &dev_attr_hrtest);
		device_remove_file(bd1662_class_dev, &dev_attr_hrenable);
		device_destroy(bd1662_class, 0);
		bd1662_class_dev = NULL;
	}
	if (bd1662_class) {
		class_destroy(bd1662_class);
		bd1662_class = NULL;
	}

	if (data->led_gpio_requested) {
		gpio_free(data->led_gpio);
		data->led_gpio_requested = 0;
	}

	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}

	kfree(data);
	g_bd1662_data = NULL;

	return 0;
}

/* factory: bd1662_suspend 0x562f98 / bd1662_resume 0x562fa0 - both "return 0" */
static int bd1662_suspend(struct device *dev)
{
	(void)dev;
	return 0;
}

static int bd1662_resume(struct device *dev)
{
	(void)dev;
	return 0;
}

static SIMPLE_DEV_PM_OPS(bd1662_pm_ops, bd1662_suspend, bd1662_resume);

/* factory: bd1662_id at 0x939b18 == { "bd1662", 0 } + terminator */
static const struct i2c_device_id bd1662_id[] = {
	{ BD1662_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bd1662_id);

/* factory: bd1662_hr_of_match at 0x939988, compatible at +0x40 == 0x9399c8 */
static const struct of_device_id bd1662_hr_of_match[] = {
	{ .compatible = BD1662_COMPATIBLE },
	{ }
};
MODULE_DEVICE_TABLE(of, bd1662_hr_of_match);

static struct i2c_driver bd1662_driver = {
	.driver = {
		.name		= BD1662_I2C_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= bd1662_hr_of_match,
		.pm		= &bd1662_pm_ops,
	},
	.probe		= bd1662_i2c_probe,
	.remove		= bd1662_i2c_remove,
	.id_table	= bd1662_id,
};

/* factory: bd1662_i2c_init at 0xc750b8 == i2c_add_driver(&bd1662_driver) */
static int __init bd1662_i2c_init(void)
{
	BD1662_INFO("bd1662 ppg driver: init\n");

	return i2c_add_driver(&bd1662_driver);
}

/* factory: bd1662_i2c_exit at 0xc9d0b0 == i2c_del_driver(&bd1662_driver) */
static void __exit bd1662_i2c_exit(void)
{
	i2c_del_driver(&bd1662_driver);

	BD1662_INFO("bd1662 ppg driver: exit\n");
}

module_init(bd1662_i2c_init);
module_exit(bd1662_i2c_exit);

/*
 * THIS is what makes /sys/module/bd1662/version readable.
 *
 * The ROM reads that path.  In this tree include/linux/module.h only expands
 * MODULE_VERSION() to the flat "version=1.0" .modinfo string when MODULE is
 * defined; for a built-in driver (!MODULE && CONFIG_SYSFS) it instead emits
 *
 *     static struct module_version_attribute ___modver_attr = {
 *             .mattr = { .attr = { .name = "version", .mode = S_IRUGO },
 *                        .show = __modver_version_show },
 *             .module_name = KBUILD_MODNAME,      <- "bd1662"
 *             .version     = _version,            <- "1.0"
 *     };
 *     ... __attribute__((__section__("__modver"))) ...
 *
 * and kernel/params.c:module_versions()/param_sysfs_builtin() hangs that
 * attribute off /sys/module/<module_name>/.  This is an exact match for the
 * factory image, whose relocation table carries the quadruple
 *     ("version" -> 0xfc3280, __modver_version_show -> 0xfc32a8,
 *      "bd1662"  -> 0xfc32b0, "1.0" -> 0xfc32c8)
 * i.e. the factory driver was also built in and also used
 * MODULE_VERSION("1.0").  Nothing else is needed for that sysfs path.
 */
MODULE_VERSION("1.0");
MODULE_DESCRIPTION("BD1662 / BD16XX PPG (heart-rate, SpO2, VP) sensor driver");
MODULE_AUTHOR("XinTaiQi / BD16XX vendor code, reverse engineered");
MODULE_LICENSE("GPL");
