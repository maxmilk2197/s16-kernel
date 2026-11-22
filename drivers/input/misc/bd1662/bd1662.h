/*
 * bd1662.h -- BD1662 / BD16XX PPG (photoplethysmography) sensor
 *
 * Reverse-engineered from the factory kernel image
 *   D:\nekoboot\out\factory_kernel.img
 * by disassembling drivers/input/misc/bd1662.c (see REVERSE_ENGINEERING.md).
 *
 * The chip provides FOUR Android sensors through one input device and one
 * sysfs class:
 *
 *   android.sensor.HRN                     (65599)  -> EV_REL/REL_X frames
 *   android.sensor.spo2                    (65596)  -> same stream, HAL decodes
 *   android.sensor.vp                      (65598)  -> same stream, HAL decodes
 *   android.sensor.finger_motion_detect    (65602)  -> EV_ABS XYZ + EV_KEY
 *
 * The vendor sensor HAL (vendor/lib64/hw/sensors.ums312.so) opens the input
 * device BY NAME ("bd1662") and writes /sys/class/bd1662/device/hrenable.
 *
 * NEKOBOOT PORT NOTE
 * ------------------
 * "nekoboot" comments mark places where the port deviates from, or hardens,
 * the factory code; everything else mirrors the factory instruction for
 * instruction.
 */

#ifndef _BD1662_H_
#define _BD1662_H_

/* ------------------------------------------------------------------ identity */

/* i2c_device_id name and input_dev->name -- the HAL looks the input device up
 * by this exact string (SensorBase::openInput("bd1662")), so it must not
 * change.  factory: file offset 0xb84ec4 (".rodata" literal) */
#define BD1662_I2C_NAME			"bd1662"

/* DT compatible -- factory of_device_id at file offset 0x939988, .compatible
 * field at +0x40 */
#define BD1662_COMPATIBLE		"bd166x,bd166x_hr"

/* sysfs class + child device.  The ROM needs the FULL path
 * /sys/class/bd1662/device/hrenable, so the class must be "bd1662" and the
 * device node must literally be called "device".
 * factory: class_create(...,"bd1662") + device_create(..., "device")
 *          0x563a28 / 0x563a40 */
#define BD1662_CLASS_NAME		"bd1662"
#define BD1662_CLASS_DEV_NAME		"device"

/* DT property holding the LED gpio.  factory: 0x563938, 0xb84edd */
#define BD1662_DT_LED_GPIO		"bd166x,led-gpio"
/* gpio_request() label -- on the factory kernel this shows up in debugfs as
 * "gpio-161 |hr_led_pin".  factory: 0x56395c, 0xb84eed */
#define BD1662_LED_GPIO_LABEL		"hr_led_pin"

/* kthread name; the kernel truncates task->comm to 15 chars, which is why the
 * factory kernel shows a thread called "polling-mode-ta".
 * factory: 0x5638f4, 0xb84ecb */
#define BD1662_THREAD_NAME		"polling-mode-task"

/* ------------------------------------------------------------------- limits */

/* LED gpio acceptance test: the factory compares the (signed) result of
 * of_get_named_gpio_flags() against 0x1ff as UNSIGNED, so any negative errno
 * is rejected.  0x563950 */
#define BD1662_GPIO_MAX			0x1ff

/* per-transfer bus-busy retry count.  factory: 0x563410 (cmp w22, #0x33) */
#define BD1662_I2C_BUSY_RETRIES		0x33
/* factory: usleep_range(500, 1000) -> 0x563400/0x563404 */
#define BD1662_I2C_BUSY_SLEEP_US_MIN	500
#define BD1662_I2C_BUSY_SLEEP_US_MAX	1000

/* number of u16 samples the algorithm block accumulates and the number of
 * samples BD_16XX_dataread() replays into the input device.
 * bd_hrs_data[] spans file offsets 0x13dd2c8..0x13dd310 == 36 u16.
 * The replay loop runs i = 0..0x22 (35 iterations).  0x563318 */
#define BD1662_HRS_DEPTH		36
#define BD1662_HRS_EMIT_COUNT		0x23	/* 35 */
/* slot indices written directly by BD_Sensor_Data_Read() */
#define BD1662_HRS_SLOT_RAW		30	/* [x21,#60] */
#define BD1662_HRS_SLOT_REFLECT		31	/* [x21,#62] */

/* frame-complete thresholds.  factory: 0x563cdc (mode == 1 -> 2),
 * 0x563d04 (mode != 1 -> 0x1c) */
#define BD1662_THRESHOLD_MODE1		2
#define BD1662_THRESHOLD_MODE0		0x1c	/* 28 */

/* accel sample ring shared with the accelerometer driver in the factory build
 * (gsensor_x/y/z are int16[128] at 0x13dce68/0x13dcf68/0x13dd068 and
 * gsensor_index is a u32 at 0x13dce60).  BD_16XX_dataread() replays
 * gsensor_index samples as EV_ABS events, scaled by 4.  0x563260..0x5632d8 */
#define BD1662_GSENSOR_DEPTH		128
#define BD1662_GSENSOR_SCALE_SHIFT	2	/* lsl w3, w3, #2 */

/* default polling delay in ms.  factory: 0x563a04 (mov w1, #0x32) */
#define BD1662_DEFAULT_DELAY_MS		50

/* nekoboot: safety slice for the polling thread's wait.
 * The factory blocks in wait_for_completion() with no timeout, which makes
 * kthread_stop() deadlock whenever the sensor was disabled first
 * (nothing ever calls complete() again).  We keep the completion handshake but
 * put a bound on the wait; while the sensor is enabled complete() is called on
 * every iteration, so the polling cadence is bit-for-bit the same as the
 * factory's. */
#define BD1662_WAIT_SLICE_MS		200

/* algorithm tuning constants programmed by BD_16XX_init_start().
 * factory: 0x563104 (7000), 0x563118 (1000), 0x563128 (20) */
#define BD1662_BP_DATA_UP		7000
#define BD1662_BP_DATA_DOWN		1000
#define BD1662_BP_DIFDATA		20
/* reflect_abssum_data >>= 2 on frame completion.  0x563cf4 */
#define BD1662_REFLECT_SHIFT		2
/* (v1 - v2) >> 3 accumulated into reflect_abssum_data.  0x563cb0 */
#define BD1662_ABS_ACC_SHIFT		3

/* --------------------------------------------------------------- register map
 *
 * Only these registers were observed in the factory binary.  Every access goes
 * through BD_I2C_Read()/BD_I2C_Write(), which emit a plain
 * "write reg, read 1 byte" i2c_msg pair (no SMBus, no register-length byte).
 *
 *   reg   value        evidence (file offset of the caller)
 *   ----  -----------  ------------------------------------------------------
 *   0x00  0x01 start   0x563b4c/0x563b58 BD_Sensor_Start()
 *   0x00  0x00 stop    0x563b98/0x563ba4 BD_Sensor_Close()
 *   0x01  0x13         0x563a80/0x563a90 BD_Sensor_Init()
 *   0x02  0x00 ack     0x563d18/0x563d20 BD_Sensor_Data_Read() tail
 *   0x04  0x04         0x563a98/0x563aa4 BD_Sensor_Init()
 *   0x05  0x0f         0x563ab0/0x563acc BD_Sensor_Init()
 *   0x05  0x08 id?     0x56362c/0x563644 bd1662_test_read_and_write()
 *   0x0e  0x64         0x563ad8/0x563aec BD_Sensor_Init()
 *   0x0f  0x00         0x563af8/0x563b0c BD_Sensor_Init()
 *   0x1c  (read, lsb)  0x563c00/0x563c0c BD_Sensor_Data_Read()
 *   0x1d  (read, msb)  0x563c1c/0x563c24 BD_Sensor_Data_Read()
 *   0x1e  (read, lsb)  0x563c48/0x563c54 BD_Sensor_Data_Read()
 *   0x1f  (read, msb)  0x563c74/0x563c7c BD_Sensor_Data_Read()
 */
#define BD1662_REG_CTRL0		0x00	/* 1 = run, 0 = stop        */
#define BD1662_REG_CFG1			0x01	/* init 0x13                */
#define BD1662_REG_ACK			0x02	/* write 0 after each read  */
#define BD1662_REG_CFG4			0x04	/* init 0x04                */
#define BD1662_REG_CFG5			0x05	/* init 0x0f; also chip id  */
#define BD1662_REG_CFG14		0x0e	/* init 0x64 (100)          */
#define BD1662_REG_CFG15		0x0f	/* init 0x00                */
#define BD1662_REG_DATA0_L		0x1c	/* sample pair #1 low byte  */
#define BD1662_REG_DATA0_H		0x1d	/* sample pair #1 high byte */
#define BD1662_REG_DATA1_L		0x1e	/* sample pair #2 low byte  */
#define BD1662_REG_DATA1_H		0x1f	/* sample pair #2 high byte */

#define BD1662_VAL_CTRL0_START		0x01
#define BD1662_VAL_CTRL0_STOP		0x00
#define BD1662_VAL_CFG1			0x13
#define BD1662_VAL_CFG4			0x04
#define BD1662_VAL_CFG5			0x0f
#define BD1662_VAL_CFG14		0x64
#define BD1662_VAL_CFG15		0x00
#define BD1662_VAL_ACK			0x00
#define BD1662_VAL_CFG5_CHIPID		0x08	/* expected read-back of 0x05 */
#define BD1662_CHIPID_REG		0x05
#define BD1662_CHIPID_INVALID		0xff

/* ------------------------------------------------------------------ ABI bits
 *
 * input_set_capability() calls, factory 0x563770..0x5637b0:
 *   (EV_ABS, ABS_X) (EV_ABS, ABS_Y) (EV_ABS, ABS_Z)
 *   (EV_REL, REL_X) (EV_REL, REL_Y)
 *
 * input_set_abs_params() calls, factory 0x5637cc..0x563858 (six calls; the
 * first three are redundant because ABS_X/ABS_Y are overwritten later):
 *   (ABS_X,      0, 0xffff, 0, 0)
 *   (ABS_X,      0, 0xffff, 0, 0)
 *   (ABS_Y,      0, 0xffff, 0, 0)
 *   (ABS_X, -32768,  32767, 0, 0)
 *   (ABS_Y, -32768,  32767, 0, 0)
 *   (ABS_Z, -32768,  32767, 0, 0)
 *
 * input_dev->id.bustype = BUS_I2C (0x18).  factory 0x563764/0x563768
 */

/* --------------------------------------------------------------- data layout
 *
 * kzalloc(0x50) in bd1662_i2c_probe() (0x5639b4).  Offsets as used by the
 * factory binary:
 */
struct bd1662_data {
	int	led_gpio;	/* +0x00  LED gpio number               */
	int	enable;		/* +0x04  0=off 1=hr 2=hr+accel replay  */
	int	delay_ms;	/* +0x08  msleep() argument             */
	int	reserved0c;	/* +0x0c  never touched in the factory  */
	struct i2c_client *client;	/* +0x10                    */
	struct input_dev  *input_dev;	/* +0x18                    */
	struct task_struct *thread;	/* +0x20  polling kthread   */
	struct completion completion;	/* +0x28  0x20 bytes        */
	int	mode;		/* +0x48  0/1, from hrenable_store      */
	/* nekoboot: NOT part of the factory layout (the factory's object ends
	 * at 0x4c with tail padding).  Records whether gpio_request()
	 * succeeded, so bd1662_i2c_remove() never gpio_free()s a pin it does
	 * not own. */
	int	led_gpio_requested;	/* +0x4c */
};

#endif /* _BD1662_H_ */
