/* For standard android platform, also available for SPRD, Qualcomm, InfotMic
 *
 * mir3da_cust.c - Linux kernel modules for 3-Axis Accelerometer
 *
 * Copyright (C) 2011-2013 MiraMEMS Sensing Technology Co., Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ---------------------------------------------------------------------------
 * Ported to the Unisoc UWS6152 (ums312 / sharkl5) Android 4.14 SPRD vendor
 * tree (w527ice).  Platform glue changes relative to the MiraMEMS reference
 * drop:
 *
 *   - of_match_table matches "da,da217" (kept "da,da213" for other boards)
 *   - a class named "xr-gsensor" with a class device named "device" is
 *     created, exposing the "gsensor" (enable) and "delay_acc" (sample delay)
 *     attributes required by the ROM sensor HAL
 *     -> /sys/class/xr-gsensor/device/{gsensor,delay_acc}
 *   - the accelerometer input device is registered with the exact name
 *     "accelerometer" (EV_ABS / ABS_X, ABS_Y, ABS_Z, raw 12-bit signed LSBs,
 *     +/-2048 full scale)
 *   - a second input device named exactly "step_counter" is registered
 *     (EV_ABS / ABS_X, 0..65535) fed from the DA217 pedometer registers
 *   - the INT1 GPIO is looked up from the device tree using the property
 *     "irq1,irq-gpio" (falling back to "irq1,irq" used by the factory blob)
 *   - the vendor blobs this driver interoperates with are:
 *       /vendor/lib64/hw/sensors.ums312.so   (consumes the ABI above)
 *       factory boot image                   (reference implementation)
 * ---------------------------------------------------------------------------
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/syscalls.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include "mir3da_core.h"
#include "mir3da_cust.h"

#define PLATFORM_QUACOMM                0
#define PLATFORM_SPRD                   1
#define PLATFORM_ELSE                   8
#define TARGET_PLATFORM                 PLATFORM_SPRD

#define MIR3DA_INPUT_DEV_NAME           "accelerometer"   /* MUST match the HAL */
#define MIR3DA_STEP_INPUT_DEV_NAME      "step_counter"    /* MUST match the HAL */
#define MIR3DA_DRV_NAME                 "mir3da"
#define MIR3DA_MISC_NAME                MIR3DA_DRV_NAME
#define MIR3DA_CLASS_NAME               "xr-gsensor"      /* MUST match the HAL */
#define MIR3DA_CLASS_DEV_NAME           "device"          /* MUST match the HAL */

/* Two spellings appear in the wild for the INT1 GPIO property. */
#define MIR3DA_GPIO_INT1_PROP           "irq1,irq-gpio"
#define MIR3DA_GPIO_INT1_PROP_ALT       "irq1,irq"

/* Accelerometer input range: raw 12-bit signed LSBs scaled by 1024 LSB/g. */
#define MIR3DA_ABS_MIN                  (-2048)
#define MIR3DA_ABS_MAX                  (2048)
/* Step counter: plain unsigned 16-bit step total. */
#define MIR3DA_STEP_MIN                 (0)
#define MIR3DA_STEP_MAX                 (0xFFFF)

#define DELAY_INTERVAL_MAX              1000
#define DELAY_INTERVAL                  50
#define STEP_DELAY_INTERVAL             1000

static MIR_HANDLE                       mir_handle;
extern int                              Log_level;

#define MI_DATA(format, ...)            if(DEBUG_DATA&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_MSG(format, ...)             if(DEBUG_MSG&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_ERR(format, ...)             if(DEBUG_ERR&Log_level){printk(KERN_ERR MI_TAG format "\n", ## __VA_ARGS__);}
#define MI_FUN                          if(DEBUG_FUNC&Log_level){printk(KERN_ERR MI_TAG "%s is called, line: %d\n", __FUNCTION__,__LINE__);}
#define MI_ASSERT(expr)                 \
	if (!(expr)) {\
		printk(KERN_ERR "Assertion failed! %s,%d,%s,%s\n",\
			__FILE__, __LINE__, __func__, #expr);\
	}

#if MIR3DA_OFFSET_TEMP_SOLUTION
#if TARGET_PLATFORM == PLATFORM_QUACOMM
static char OffsetFolerName[] = "/persist/";
static const char OffsetFileName[] = "/persist/miraGSensorOffset.txt";
#elif TARGET_PLATFORM == PLATFORM_SPRD
static char OffsetFolerName[] = "/productinfo/";
static const char OffsetFileName[] = "/productinfo/miraGSensorOffset.txt";
#else
static char OffsetFolerName[] = "/data/misc/";
static const char OffsetFileName[] = "/data/misc/miraGSensorOffset.txt";
#endif

#define OFFSET_STRING_LEN               26
#endif /* MIR3DA_OFFSET_TEMP_SOLUTION */

struct mir3da_data {
	struct i2c_client       *mir3da_i2c_client;
	atomic_t                delay;
	atomic_t                enable;
	atomic_t                sc_enable;
	int                     sc_delay;
	short                   sc_last;
	struct input_dev        *input;
	struct input_dev        *sc_input;
	struct mutex            enable_mutex;
	struct delayed_work     work;
	struct delayed_work     sc_work;
	int                     irq1_gpio;
	int                     irq;
};

static struct class                     *mir3da_class;
static struct device                    *mir3da_class_dev;
static int                              direction_remap = MIR3DA_DIRECTION_NUM;
#if MIR3DA_OFFSET_TEMP_SOLUTION
int                                     bCaliResult = -1;
#endif

/* ------------------------------------------------------------------ */
/* I2C bus glue                                                        */
/* ------------------------------------------------------------------ */

static int get_address(PLAT_HANDLE handle)
{
	if (NULL == handle) {
		MI_ERR("chip init failed !\n");
		return -1;
	}

	return ((struct i2c_client *)handle)->addr;
}

static int i2c_smbus_read(PLAT_HANDLE handle, u8 addr, u8 *data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;
	int                 val;

	val = i2c_smbus_read_byte_data(client, addr);
	if (val < 0) {
		MI_ERR("i2c read failed, addr=0x%02x, res=%d", addr, val);
		return val;
	}
	*data = (u8)val;

	return res;
}

static int i2c_smbus_read_block(PLAT_HANDLE handle, u8 addr, u8 count, u8 *data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;

	res = i2c_smbus_read_i2c_block_data(client, addr, count, data);

	return res;
}

static int i2c_smbus_write(PLAT_HANDLE handle, u8 addr, u8 data)
{
	int                 res = 0;
	struct i2c_client   *client = (struct i2c_client *)handle;

	res = i2c_smbus_write_byte_data(client, addr, data);

	return res;
}

static void msdelay(int ms)
{
	mdelay(ms);
}

#if MIR3DA_OFFSET_TEMP_SOLUTION
/* Offset storage.  kernel_read()/kernel_write() perform the set_fs() dance
 * themselves, so no manual KERNEL_DS handling is needed.
 */
static int sensor_sync_write(u8 *off)
{
	char            buf[OFFSET_STRING_LEN];
	struct file     *filep;
	int             ret;

	snprintf(buf, sizeof(buf), "%x,%x,%x,%x,%x,%x,%x,%x,%x\n",
			off[0], off[1], off[2], off[3], off[4], off[5], off[6], off[7], off[8]);

	filep = filp_open(OffsetFileName, O_RDWR | O_CREAT, 0600);
	if (IS_ERR(filep)) {
		MI_ERR("write, filp_open %s error!!", OffsetFileName);
		return -1;
	}

	ret = kernel_write(filep, buf, strlen(buf), &filep->f_pos);
	filp_close(filep, NULL);

	return (ret < 0) ? -1 : 0;
}

static int sensor_sync_read(u8 *offset)
{
	int             off[MIR3DA_OFFSET_LEN] = {0};
	char            buf[OFFSET_STRING_LEN + 1];
	struct file     *filep;
	int             ret;
	int             i;

	memset(buf, 0, sizeof(buf));

	filep = filp_open(OffsetFileName, O_RDONLY, 0600);
	if (IS_ERR(filep)) {
		MI_ERR("read, filp_open %s error!!", OffsetFileName);
		return -1;
	}

	ret = kernel_read(filep, buf, OFFSET_STRING_LEN, &filep->f_pos);
	filp_close(filep, NULL);
	if (ret <= 0) {
		MI_ERR("read, empty cali file");
		return -1;
	}

	sscanf(buf, "%x,%x,%x,%x,%x,%x,%x,%x,%x",
			&off[0], &off[1], &off[2], &off[3], &off[4],
			&off[5], &off[6], &off[7], &off[8]);

	for (i = 0; i < MIR3DA_OFFSET_LEN; i++)
		offset[i] = (u8)off[i];

	return 0;
}

static int check_califolder_exist(void)
{
	struct  file *filep;

	filep = filp_open(OffsetFolerName, O_RDONLY, 0600);
	if (IS_ERR(filep)) {
		MI_ERR("%s read, filp_open %s error!!", __func__, OffsetFolerName);
		return 0;
	}

	filp_close(filep, NULL);

	return 1;
}

static int support_fast_auto_cali(void)
{
#if MIR3DA_SUPPORT_FAST_AUTO_CALI
	return 1;
#else
	return 0;
#endif
}
#endif /* MIR3DA_OFFSET_TEMP_SOLUTION */

#if MIR3DA_OFFSET_TEMP_SOLUTION
MIR_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block, i2c_smbus_write,
		sensor_sync_write, sensor_sync_read, check_califolder_exist, get_address,
		support_fast_auto_cali, msdelay, printk, sprintf);
#else
MIR_GENERAL_OPS_DECLARE(ops_handle, i2c_smbus_read, i2c_smbus_read_block, i2c_smbus_write,
		NULL, NULL, NULL, get_address, NULL, msdelay, printk, sprintf);
#endif

/* ------------------------------------------------------------------ */
/* DA217 pedometer / step counter                                      */
/* ------------------------------------------------------------------ */

/* Register values below reproduce the pedometer setup used by the factory
 * kernel driver for this board (extracted from the shipped boot image).
 */
static int mir3da_step_count_init(MIR_HANDLE handle)
{
	int     res = 0;

	res  = mir3da_register_mask_write(handle, DA217_REG_STEP_CFG0, 0xFF, DA217_STEP_CFG0_VALUE);
	res |= mir3da_register_mask_write(handle, DA217_REG_STEP_CFG1, 0xFF, DA217_STEP_CFG1_VALUE);
	res |= mir3da_register_mask_write(handle, DA217_REG_STEP_CFG2, 0xFF, DA217_STEP_CFG2_VALUE);
	res |= mir3da_register_mask_write(handle, DA217_REG_STEP_CFG3, 0xFF, DA217_STEP_CFG3_VALUE);
	res |= mir3da_register_mask_write(handle, DA217_REG_STEP_FILTER, 0xFF, DA217_STEP_FILTER_VALUE);

	return res;
}

static int mir3da_clear_step(MIR_HANDLE handle)
{
	return mir3da_register_mask_write(handle, DA217_REG_RESET_STEP, 0xFF, DA217_STEP_RESET_BIT);
}

static int mir3da_set_step_enable(MIR_HANDLE handle, int enable)
{
	unsigned char   value = enable ? DA217_STEP_ENABLE_BIT : 0x00;

	return mir3da_register_mask_write(handle, DA217_REG_STEP_FILTER, DA217_STEP_ENABLE_BIT, value);
}

/* Returns the accumulated step total in *count. */
static int mir3da_read_step(MIR_HANDLE handle, short *count)
{
	unsigned char   data[2] = {0};
	int             res;

	if (NULL == handle || NULL == count)
		return -1;

	res = mir3da_register_read_continuously(handle, DA217_REG_STEPS_MSB, 2, data);
	if (res) {
		MI_ERR("read_step failed, res=%d", res);
		return -1;
	}

	*count = (short)((data[0] << 8) | data[1]);

	return 0;
}

/* ------------------------------------------------------------------ */
/* workers                                                             */
/* ------------------------------------------------------------------ */

static void mir3da_work_func(struct work_struct *work)
{
	short   x = 0, y = 0, z = 0;
	struct mir3da_data *mir3da = container_of((struct delayed_work *)work, struct mir3da_data, work);
	int     map_para = 1;

	if (mir3da_read_data(mir3da->mir3da_i2c_client, &x, &y, &z) != 0) {
		MI_ERR("MIR3DA data read failed!\n");
	} else {
		map_para = mir3da_direction_remap(&x, &y, &z, direction_remap);

		if (bzstk)
			z = map_para * squareRoot(1024 * 1024 - (x) * (x) - (y) * (y));

		input_report_abs(mir3da->input, ABS_X, x);
		input_report_abs(mir3da->input, ABS_Y, y);
		input_report_abs(mir3da->input, ABS_Z, z);
		input_sync(mir3da->input);
	}

	schedule_delayed_work(&mir3da->work, msecs_to_jiffies(atomic_read(&mir3da->delay)));
}

static void mir3da_work_sc_func(struct work_struct *work)
{
	short   steps = 0;
	struct mir3da_data *mir3da = container_of((struct delayed_work *)work, struct mir3da_data, sc_work);

	if (mir3da_read_step(mir3da->mir3da_i2c_client, &steps) != 0)
		steps = mir3da->sc_last;        /* keep the last good value */
	else
		mir3da->sc_last = steps;

	input_report_abs(mir3da->sc_input, ABS_X, steps);
	input_sync(mir3da->sc_input);

	schedule_delayed_work(&mir3da->sc_work, msecs_to_jiffies(mir3da->sc_delay));
}

/* ------------------------------------------------------------------ */
/* misc / ioctl interface                                              */
/* ------------------------------------------------------------------ */

static int mir3da_misc_open(struct inode *inode, struct file *file)
{
	file->private_data = (struct i2c_client *)mir_handle;

	if (file->private_data == NULL) {
		MI_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int mir3da_misc_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long mir3da_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user     *argp = (void __user *)arg;
	int             err = 0;
	int             interval = 0;
	char            bEnable = 0;
	int             z_dir = 0;
	short           xyz[3] = {0};

	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct mir3da_data *mir3da;

	if (NULL == client)
		return -EFAULT;

	mir3da = (struct mir3da_data *)i2c_get_clientdata(client);
	if (NULL == mir3da)
		return -EFAULT;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
	case MIR3DA_ACC_IOCTL_GET_DELAY:
		interval = atomic_read(&mir3da->delay);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case MIR3DA_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > DELAY_INTERVAL_MAX)
			return -EINVAL;
		atomic_set(&mir3da->delay, (unsigned int)interval);
		break;

	case MIR3DA_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&bEnable, argp, sizeof(bEnable)))
			return -EFAULT;

		bEnable = (bEnable > 0) ? 1 : 0;

		err = mir3da_set_enable(client, bEnable);
		if (err < 0)
			return -EINVAL;

		if (bEnable)
			schedule_delayed_work(&mir3da->work, msecs_to_jiffies(atomic_read(&mir3da->delay)));
		else
			cancel_delayed_work_sync(&mir3da->work);
		atomic_set(&mir3da->enable, bEnable);
		break;

	case MIR3DA_ACC_IOCTL_GET_ENABLE:
		err = mir3da_get_enable(client, &bEnable);
		if (err < 0)
			return -EINVAL;

		if (copy_to_user(argp, &bEnable, sizeof(bEnable)))
			return -EFAULT;
		break;

#if MIR3DA_OFFSET_TEMP_SOLUTION
	case MIR3DA_ACC_IOCTL_CALIBRATION:
		if (copy_from_user(&z_dir, argp, sizeof(z_dir)))
			return -EFAULT;
		if (mir3da_calibrate(client, z_dir))
			return -EFAULT;

		if (copy_to_user(argp, &z_dir, sizeof(z_dir)))
			return -EFAULT;
		break;

	case MIR3DA_ACC_IOCTL_UPDATE_OFFSET:
		manual_load_cali_file(client);
		break;
#endif /* MIR3DA_OFFSET_TEMP_SOLUTION */

	case MIR3DA_ACC_IOCTL_GET_COOR_XYZ:
		if (mir3da_read_data(client, &xyz[0], &xyz[1], &xyz[2]))
			return -EFAULT;

		if (copy_to_user((void __user *)arg, xyz, sizeof(xyz)))
			return -EFAULT;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations mir3da_misc_fops = {
	.owner = THIS_MODULE,
	.open = mir3da_misc_open,
	.release = mir3da_misc_release,
	.unlocked_ioctl = mir3da_misc_ioctl,
};

static struct miscdevice misc_mir3da = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MIR3DA_MISC_NAME,
	.fops = &mir3da_misc_fops,
};

/* ------------------------------------------------------------------ */
/* helpers shared by the two attribute groups                          */
/* ------------------------------------------------------------------ */

static int mir3da_set_enable_work(struct mir3da_data *mir3da, int bEnable)
{
	int ret = 0;

	mutex_lock(&mir3da->enable_mutex);

	if (bEnable != atomic_read(&mir3da->enable)) {
		if (mir3da_set_enable(mir3da->mir3da_i2c_client, (char)bEnable) < 0) {
			ret = -EINVAL;
			goto out;
		}

		if (bEnable)
			schedule_delayed_work(&mir3da->work,
					msecs_to_jiffies(atomic_read(&mir3da->delay)));
		else
			cancel_delayed_work_sync(&mir3da->work);

		atomic_set(&mir3da->enable, bEnable);
	}

out:
	mutex_unlock(&mir3da->enable_mutex);
	return ret;
}

/* Accepts both ASCII ("0"/"1"/"123") and a raw binary byte, which is what
 * the SPRD sensor HAL has been observed to write.
 */
static int mir3da_parse_enable(const char *buf)
{
	unsigned char c = (unsigned char)buf[0];

	if (c == 0 || c == '0')
		return 0;
	if (c == 1 || c == '1')
		return 1;

	return (simple_strtoul(buf, NULL, 10) > 0) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* /sys/class/xr-gsensor/device/{gsensor,delay_acc}                    */
/* ------------------------------------------------------------------ */

static ssize_t mir3da_gsensor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);

	if (NULL == mir3da)
		return -EINVAL;

	return sprintf(buf, "%d\n", atomic_read(&mir3da->enable));
}

static ssize_t mir3da_gsensor_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);
	int bEnable;

	if (NULL == mir3da || NULL == buf)
		return -EINVAL;

	bEnable = mir3da_parse_enable(buf);

	MI_MSG("%s, line=%d, bEnable=%d", __func__, __LINE__, bEnable);

	if (mir3da_set_enable_work(mir3da, bEnable) < 0)
		return -EINVAL;

	return count;
}

static ssize_t mir3da_delay_acc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);

	if (NULL == mir3da)
		return -EINVAL;

	return sprintf(buf, "%d\n", atomic_read(&mir3da->delay));
}

/* delay_acc is the polling period in milliseconds (same unit the IOCTL and
 * the input-device "poll_delay" attribute use).  Both ASCII ("200") and a
 * raw little-endian binary int (0xC8 0x00 0x00 0x00) are accepted, because
 * SPRD sensor HALs have shipped with both conventions.
 */
static ssize_t mir3da_delay_acc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);
	unsigned long data = 0;

	if (NULL == mir3da || NULL == buf)
		return -EINVAL;

	data = simple_strtoul(buf, NULL, 10);

	if (data == 0 && count >= sizeof(int)) {
		int raw = 0;

		memcpy(&raw, buf, sizeof(raw));
		if (raw > 0)
			data = (unsigned long)raw;
	}

	if (data == 0)
		data = DELAY_INTERVAL;
	if (data > DELAY_INTERVAL_MAX)
		data = DELAY_INTERVAL_MAX;

	mir3da_set_odr(mir3da->mir3da_i2c_client, (int)data);
	atomic_set(&mir3da->delay, (unsigned int)data);

	return count;
}

static DEVICE_ATTR(gsensor, 0664, mir3da_gsensor_show, mir3da_gsensor_store);
static DEVICE_ATTR(delay_acc, 0664, mir3da_delay_acc_show, mir3da_delay_acc_store);

/* ------------------------------------------------------------------ */
/* /sys/class/xr-gsensor/device/scsensor  (step counter polling)       */
/* ------------------------------------------------------------------ */

static ssize_t mir3da_sc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);

	if (NULL == mir3da)
		return -EINVAL;

	return sprintf(buf, "%d\n", atomic_read(&mir3da->sc_enable));
}

static ssize_t mir3da_sc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);
	int bEnable;

	if (NULL == mir3da || NULL == buf)
		return -EINVAL;

	bEnable = mir3da_parse_enable(buf);

	if (bEnable != atomic_read(&mir3da->sc_enable)) {
		if (bEnable) {
			mir3da_set_step_enable(mir3da->mir3da_i2c_client, 1);
			schedule_delayed_work(&mir3da->sc_work,
					msecs_to_jiffies(mir3da->sc_delay));
		} else {
			cancel_delayed_work_sync(&mir3da->sc_work);
		}
		atomic_set(&mir3da->sc_enable, bEnable);
	}

	return count;
}

static DEVICE_ATTR(scsensor, 0664, mir3da_sc_show, mir3da_sc_store);

/* ------------------------------------------------------------------ */
/* legacy per-input-device attribute group                             */
/* ------------------------------------------------------------------ */

static ssize_t mir3da_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);
	int             ret;
	char            bEnable;

	if (NULL == mir3da)
		return -EINVAL;

	ret = mir3da_get_enable(mir3da->mir3da_i2c_client, &bEnable);
	if (ret < 0)
		ret = -EINVAL;
	else
		ret = sprintf(buf, "%d\n", bEnable);

	return ret;
}

static ssize_t mir3da_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);
	int bEnable;

	if (NULL == mir3da || NULL == buf)
		return -EINVAL;

	bEnable = mir3da_parse_enable(buf);

	if (mir3da_set_enable_work(mir3da, bEnable) < 0)
		return -EINVAL;

	return count;
}

static ssize_t mir3da_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);

	if (NULL == mir3da)
		return -EINVAL;

	return sprintf(buf, "%d\n", atomic_read(&mir3da->delay));
}

static ssize_t mir3da_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);
	unsigned long data = 0;

	if (NULL == mir3da)
		return -EINVAL;

	data = simple_strtoul(buf, NULL, 10);
	if (data > DELAY_INTERVAL_MAX)
		data = DELAY_INTERVAL_MAX;

	atomic_set(&mir3da->delay, (unsigned int)data);

	return count;
}

static ssize_t mir3da_axis_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int result;
	short x, y, z;
	int count = 0;
	struct mir3da_data *mir3da = dev_get_drvdata(dev);

	if (NULL == mir3da)
		return -EINVAL;

	result = mir3da_read_data(mir3da->mir3da_i2c_client, &x, &y, &z);
	if (result == 0)
		count += sprintf(buf + count, "x= %d;y=%d;z=%d\n", x, y, z);
	else
		count += sprintf(buf + count, "reading failed!");

	return count;
}

static ssize_t mir3da_reg_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int                 addr, data;
	int                 result;
	struct mir3da_data *mir3da = dev_get_drvdata(dev);

	if (NULL == mir3da)
		return -EINVAL;

	sscanf(buf, "0x%x, 0x%x\n", &addr, &data);

	result = mir3da_register_write(mir3da->mir3da_i2c_client, addr, data);

	MI_ASSERT(result == 0);

	return count;
}

static ssize_t mir3da_reg_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);

	if (NULL == mir3da)
		return -EINVAL;

	return mir3da_get_reg_data(mir3da->mir3da_i2c_client, buf);
}

#if MIR3DA_OFFSET_TEMP_SOLUTION
static ssize_t mir3da_calibrate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;

	ret = sprintf(buf, "%d\n", bCaliResult);
	return ret;
}

static ssize_t mir3da_calibrate_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	s8              z_dir = 0;
	struct mir3da_data *mir3da = dev_get_drvdata(dev);

	if (NULL == mir3da)
		return -EINVAL;

	z_dir = simple_strtol(buf, NULL, 10);
	bCaliResult = mir3da_calibrate(mir3da->mir3da_i2c_client, z_dir);

	return count;
}
#endif /* MIR3DA_OFFSET_TEMP_SOLUTION */

static ssize_t mir3da_log_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", Log_level);
}

static ssize_t mir3da_log_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	Log_level = simple_strtoul(buf, NULL, 10);

	return count;
}

static ssize_t mir3da_direction_remap_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", direction_remap);
}

static ssize_t mir3da_direction_remap_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long v = simple_strtoul(buf, NULL, 10);

	if (v < 8)
		direction_remap = (int)v;

	return count;
}

static ssize_t mir3da_primary_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mir3da_data *mir3da = dev_get_drvdata(dev);
	int x = 0, y = 0, z = 0;

	if (NULL == mir3da)
		return -EINVAL;

	mir3da_get_primary_offset(mir3da->mir3da_i2c_client, &x, &y, &z);

	return sprintf(buf, "x=%d ,y=%d ,z=%d\n", x, y, z);
}

static ssize_t mir3da_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s_%s\n", DRI_VER, CORE_VER);
}

static ssize_t mir3da_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", "MiraMEMS");
}

static DEVICE_ATTR(enable,          0664,  mir3da_enable_show,          mir3da_enable_store);
static DEVICE_ATTR(poll_delay,      0664,  mir3da_delay_show,           mir3da_delay_store);
static DEVICE_ATTR(axis_data,       S_IRUGO, mir3da_axis_data_show,     NULL);
static DEVICE_ATTR(reg_data,        0664,  mir3da_reg_data_show,        mir3da_reg_data_store);
static DEVICE_ATTR(log_level,       0664,  mir3da_log_level_show,       mir3da_log_level_store);
static DEVICE_ATTR(direction_remap, 0664,  mir3da_direction_remap_show, mir3da_direction_remap_store);
#if MIR3DA_OFFSET_TEMP_SOLUTION
static DEVICE_ATTR(calibrate_miraGSensor, 0664, mir3da_calibrate_show,  mir3da_calibrate_store);
#endif
static DEVICE_ATTR(primary_offset,  S_IRUGO, mir3da_primary_offset_show, NULL);
static DEVICE_ATTR(version,         S_IRUGO, mir3da_version_show,        NULL);
static DEVICE_ATTR(vendor,          S_IRUGO, mir3da_vendor_show,         NULL);

static struct attribute *mir3da_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_axis_data.attr,
	&dev_attr_reg_data.attr,
	&dev_attr_log_level.attr,
	&dev_attr_direction_remap.attr,
#if MIR3DA_OFFSET_TEMP_SOLUTION
	&dev_attr_calibrate_miraGSensor.attr,
#endif
	&dev_attr_primary_offset.attr,
	&dev_attr_version.attr,
	&dev_attr_vendor.attr,
	NULL
};

static const struct attribute_group mir3da_attr_group = {
	.attrs  = mir3da_attributes,
};

/* ------------------------------------------------------------------ */
/* power management                                                    */
/* ------------------------------------------------------------------ */

static int mir3da_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int result = 0;
	struct mir3da_data *mir3da = i2c_get_clientdata(client);

	MI_FUN;

	if (NULL == mir3da)
		return -EINVAL;

	result = mir3da_set_enable(mir3da->mir3da_i2c_client, false);
	if (result) {
		MI_ERR("%s: disable fail!!\n", __func__);
		return result;
	}

	cancel_delayed_work_sync(&mir3da->work);
	cancel_delayed_work_sync(&mir3da->sc_work);
	return result;
}

static int mir3da_resume(struct i2c_client *client)
{
	int result = 0;
	struct mir3da_data *mir3da = i2c_get_clientdata(client);
	int pre_enable;

	MI_FUN;

	if (NULL == mir3da)
		return -EINVAL;

	result = mir3da_chip_resume(mir3da->mir3da_i2c_client);
	if (result) {
		MI_ERR("chip resume fail!!\n");
		return result;
	}

	pre_enable = atomic_read(&mir3da->enable);
	result = mir3da_set_enable(mir3da->mir3da_i2c_client, pre_enable);
	if (result) {
		MI_ERR("%s: enable fail!!\n", __func__);
		return result;
	}

	if (pre_enable)
		schedule_delayed_work(&mir3da->work, msecs_to_jiffies(atomic_read(&mir3da->delay)));

	if (atomic_read(&mir3da->sc_enable))
		schedule_delayed_work(&mir3da->sc_work, msecs_to_jiffies(mir3da->sc_delay));

	return result;
}

static int mir3da_pm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	pm_message_t mesg = {0};

	mir3da_suspend(client, mesg);
	return 0;
}

static int mir3da_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	mir3da_resume(client);
	return 0;
}

static const struct dev_pm_ops mir3da_pm_ops = {
	.suspend = mir3da_pm_suspend,
	.resume = mir3da_pm_resume,
};

/* ------------------------------------------------------------------ */
/* INT1 GPIO (optional; polling is the primary data path)              */
/* ------------------------------------------------------------------ */

static irqreturn_t mir3da_irq1_handler(int irq, void *dev_id)
{
	struct mir3da_data *mir3da = dev_id;

	if (mir3da && atomic_read(&mir3da->enable))
		schedule_delayed_work(&mir3da->work, 0);

	return IRQ_HANDLED;
}

static int mir3da_int1_gpio_init(struct i2c_client *client, struct mir3da_data *mir3da)
{
	struct device_node  *np = client->dev.of_node;
	int                 gpio;
	int                 irq;
	int                 ret;
	enum of_gpio_flags  flags = 0;

	if (NULL == np)
		return -ENODEV;

	gpio = of_get_named_gpio_flags(np, MIR3DA_GPIO_INT1_PROP, 0, &flags);
	if (gpio < 0) {
		gpio = of_get_named_gpio_flags(np, MIR3DA_GPIO_INT1_PROP_ALT, 0, &flags);
		if (gpio < 0) {
			dev_info(&client->dev, "no %s / %s property, polling only\n",
					MIR3DA_GPIO_INT1_PROP, MIR3DA_GPIO_INT1_PROP_ALT);
			return -ENODEV;
		}
	}

	mir3da->irq1_gpio = gpio;
	dev_info(&client->dev, "INT1 gpio=%d flags=0x%x\n", gpio, (unsigned int)flags);

	ret = devm_gpio_request_one(&client->dev, gpio, GPIOF_IN, "mir3da_irq1");
	if (ret) {
		dev_warn(&client->dev, "gpio %d request failed (%d)\n", gpio, ret);
		return ret;
	}

	irq = gpio_to_irq(gpio);
	if (irq < 0) {
		dev_warn(&client->dev, "gpio %d has no irq (%d)\n", gpio, irq);
		return irq;
	}

	ret = devm_request_irq(&client->dev, irq, mir3da_irq1_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"mir3da_irq1", mir3da);
	if (ret) {
		dev_warn(&client->dev, "request_irq %d failed (%d)\n", irq, ret);
		return ret;
	}

	mir3da->irq = irq;
	dev_info(&client->dev, "INT1 irq=%d registered\n", irq);

	return 0;
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                      */
/* ------------------------------------------------------------------ */

static int mir3da_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int                 result;
	struct input_dev    *idev;
	struct input_dev    *sc_idev;
	struct i2c_adapter  *adapter;
	struct mir3da_data  *mir3da;
	unsigned char       chip_id = 0;
	struct device_node  *np = client->dev.of_node;
	u32                 layout = MIR3DA_DIRECTION_NUM;

	MI_MSG("%s: addr=0x%02x irq=%d\n", __func__, client->addr, client->irq);

	adapter = to_i2c_adapter(client->dev.parent);
	result = i2c_check_functionality(adapter,
			I2C_FUNC_SMBUS_BYTE |
			I2C_FUNC_SMBUS_BYTE_DATA);
	MI_ASSERT(result);
	if (!result) {
		MI_ERR("%s: i2c function not support!\n", __func__);
		return -EIO;
	}

	if (mir3da_install_general_ops(&ops_handle)) {
		MI_ERR("Install ops failed !\n");
		return -EINVAL;
	}

	mir3da = kzalloc(sizeof(struct mir3da_data), GFP_KERNEL);
	if (!mir3da) {
		result = -ENOMEM;
		goto err_out;
	}

	mutex_init(&mir3da->enable_mutex);
	mir3da->mir3da_i2c_client = client;
	mir3da->sc_delay = STEP_DELAY_INTERVAL;
	mir3da->sc_last = 0;
	i2c_set_clientdata(client, mir3da);

	i2c_smbus_read((PLAT_HANDLE)client, NSA_REG_WHO_AM_I, &chip_id);
	MI_MSG("mir3da chip_id=0x%x\n", chip_id);

	/* Initialize the MIR3DA chip */
	mir_handle = mir3da_core_init((PLAT_HANDLE)client);
	if (NULL == mir_handle) {
		MI_ERR("chip init failed !\n");
		result = -ENODEV;
		goto err_free;
	}

	/*
	 * nekoboot: DO NOT take the axis remap from the device tree.
	 *
	 * The factory kernel hard-codes MIR3DA_DIRECTION_NUM (6) and never reads
	 * the DT `layout` property.  This board's DT says layout = <7>, which
	 * permutes the axes differently and makes framework auto-rotate land 90
	 * degrees off.  Setting the sysfs attribute to 6 fixes it at runtime, but
	 * the driver used to reset it to 7 on every boot.
	 *
	 * Match the factory.  The `direction_remap` sysfs attribute remains as a
	 * runtime tuning knob (see mir3da_direction_remap_store).
	 */
	direction_remap = MIR3DA_DIRECTION_NUM;
	dev_info(&client->dev,
		 "nekoboot: direction_remap = %d (DT layout ignored, factory value)\n",
		 direction_remap);

	atomic_set(&mir3da->enable, 0);
	atomic_set(&mir3da->delay, DELAY_INTERVAL);
	atomic_set(&mir3da->sc_enable, 0);

	/* ---- accelerometer input device ---- */
	idev = input_allocate_device();
	if (!idev) {
		MI_ERR("alloc poll device failed!\n");
		result = -ENOMEM;
		goto err_free;
	}

	idev->name = MIR3DA_INPUT_DEV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->dev.parent = &client->dev;
	idev->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(idev, ABS_X, MIR3DA_ABS_MIN, MIR3DA_ABS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_Y, MIR3DA_ABS_MIN, MIR3DA_ABS_MAX, 0, 0);
	input_set_abs_params(idev, ABS_Z, MIR3DA_ABS_MIN, MIR3DA_ABS_MAX, 0, 0);

	input_set_drvdata(idev, mir3da);

	result = input_register_device(idev);
	if (result) {
		MI_ERR("register poll device failed!\n");
		input_free_device(idev);
		goto err_free;
	}

	mir3da->input = idev;
	INIT_DELAYED_WORK(&mir3da->work, mir3da_work_func);

	result = sysfs_create_group(&mir3da->input->dev.kobj, &mir3da_attr_group);
	if (result) {
		MI_ERR("create device file failed!\n");
		result = -EINVAL;
		goto err_unregister_input;
	}
	kobject_uevent(&mir3da->input->dev.kobj, KOBJ_ADD);

	/* ---- step counter input device ---- */
	sc_idev = input_allocate_device();
	if (!sc_idev) {
		MI_ERR("%s: can't allocate device!\n", __func__);
		result = -ENOMEM;
		goto err_remove_sysfs_group;
	}

	sc_idev->name = MIR3DA_STEP_INPUT_DEV_NAME;
	sc_idev->id.bustype = BUS_I2C;
	sc_idev->dev.parent = &client->dev;
	sc_idev->evbit[0] = BIT_MASK(EV_ABS);

	/* Plain monotonic step total.  The factory driver used EV_ABS/ABS_X
	 * with a 0..65535 range for this device; the ROM HAL only looks the
	 * device up by name, so we reproduce it exactly.
	 */
	input_set_abs_params(sc_idev, ABS_X, MIR3DA_STEP_MIN, MIR3DA_STEP_MAX, 0, 0);

	result = input_register_device(sc_idev);
	if (result) {
		MI_ERR("%s: can't register device!\n", __func__);
		input_free_device(sc_idev);
		goto err_remove_sysfs_group;
	}

	mir3da->sc_input = sc_idev;
	INIT_DELAYED_WORK(&mir3da->sc_work, mir3da_work_sc_func);

	/* ---- misc device ---- */
	result = misc_register(&misc_mir3da);
	if (result) {
		MI_ERR("%s: mir3da_dev register failed\n", __func__);
		goto err_unregister_sc;
	}

	/* ---- /sys/class/xr-gsensor/device ---- */
	if (NULL == mir3da_class) {
		mir3da_class = class_create(THIS_MODULE, MIR3DA_CLASS_NAME);
		if (IS_ERR(mir3da_class)) {
			mir3da_class = NULL;
			MI_ERR("Failed to create class(%s)!\n", MIR3DA_CLASS_NAME);
			result = -ENODEV;
			goto err_misc_deregister;
		}
	}

	if (NULL == mir3da_class_dev) {
		mir3da_class_dev = device_create(mir3da_class, NULL, 0, mir3da,
				MIR3DA_CLASS_DEV_NAME);
		if (IS_ERR(mir3da_class_dev)) {
			mir3da_class_dev = NULL;
			MI_ERR("Failed to create device(%s)!\n", MIR3DA_CLASS_DEV_NAME);
			result = -ENODEV;
			goto err_misc_deregister;
		}
	} else {
		dev_set_drvdata(mir3da_class_dev, mir3da);
	}

	result = device_create_file(mir3da_class_dev, &dev_attr_gsensor);
	if (result)
		MI_ERR("Failed to create device file(%s)!\n", dev_attr_gsensor.attr.name);

	result = device_create_file(mir3da_class_dev, &dev_attr_delay_acc);
	if (result)
		MI_ERR("Failed to create device file(%s)!\n", dev_attr_delay_acc.attr.name);

	result = device_create_file(mir3da_class_dev, &dev_attr_scsensor);
	if (result)
		MI_ERR("Failed to create device file(%s)!\n", dev_attr_scsensor.attr.name);

	/* ---- pedometer ---- */
	mir3da_step_count_init(client);
	mir3da_clear_step(client);
	mir3da_set_step_enable(client, 1);

	/* The ROM HAL looks the step device up by name only, so start feeding
	 * it right away; /sys/class/xr-gsensor/device/scsensor can stop it.
	 */
	atomic_set(&mir3da->sc_enable, 1);
	schedule_delayed_work(&mir3da->sc_work, msecs_to_jiffies(mir3da->sc_delay));

	/* ---- INT1 GPIO (non fatal) ---- */
	mir3da_int1_gpio_init(client, mir3da);

	MI_MSG("add mir3da i2c driver succeed\n");

	return 0;

err_misc_deregister:
	misc_deregister(&misc_mir3da);
err_unregister_sc:
	if (mir3da->sc_input) {
		input_unregister_device(mir3da->sc_input);
		mir3da->sc_input = NULL;
	}
err_remove_sysfs_group:
	sysfs_remove_group(&mir3da->input->dev.kobj, &mir3da_attr_group);
err_unregister_input:
	input_unregister_device(mir3da->input);
	mir3da->input = NULL;
err_free:
	i2c_set_clientdata(client, NULL);
	kfree(mir3da);
	mir_handle = NULL;
err_out:
	return result;
}

static int mir3da_remove(struct i2c_client *client)
{
	struct mir3da_data *mir3da = i2c_get_clientdata(client);

	MI_FUN;

	if (NULL == mir3da)
		return 0;

	mir3da_set_step_enable(mir3da->mir3da_i2c_client, 0);
	mir3da_set_enable(mir3da->mir3da_i2c_client, 0);

	cancel_delayed_work_sync(&mir3da->sc_work);
	cancel_delayed_work_sync(&mir3da->work);

	if (mir3da_class_dev) {
		device_remove_file(mir3da_class_dev, &dev_attr_gsensor);
		device_remove_file(mir3da_class_dev, &dev_attr_delay_acc);
		device_remove_file(mir3da_class_dev, &dev_attr_scsensor);
		device_destroy(mir3da_class, 0);
		mir3da_class_dev = NULL;
	}
	if (mir3da_class) {
		class_destroy(mir3da_class);
		mir3da_class = NULL;
	}

	misc_deregister(&misc_mir3da);

	if (mir3da->sc_input) {
		input_unregister_device(mir3da->sc_input);
		mir3da->sc_input = NULL;
	}
	if (mir3da->input) {
		sysfs_remove_group(&mir3da->input->dev.kobj, &mir3da_attr_group);
		input_unregister_device(mir3da->input);
		mir3da->input = NULL;
	}

	i2c_set_clientdata(client, NULL);
	kfree(mir3da);
	mir_handle = NULL;

	return 0;
}

static int mir3da_detect(struct i2c_client *new_client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = new_client->adapter;

	MI_MSG("adapter->NR = %d\n", adapter->nr);
	MI_MSG(">>> mir3da_detect, new_client->addr = 0x%x\n", new_client->addr);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	strlcpy(info->type, MIR3DA_DRV_NAME, I2C_NAME_SIZE);

	return 0;
}

static const struct i2c_device_id mir3da_id[] = {
	{ MIR3DA_DRV_NAME, 0 },
	{ "da217", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mir3da_id);

static const unsigned short normal_i2c[] = { 0x26, 0x27, I2C_CLIENT_END };

static const struct of_device_id mir3da_of_match[] = {
	{ .compatible = "da,da217", },
	{ .compatible = "da,da213", },
	{ }
};
MODULE_DEVICE_TABLE(of, mir3da_of_match);

static struct i2c_driver mir3da_driver = {
	.driver = {
		.name           = MIR3DA_DRV_NAME,
		.owner          = THIS_MODULE,
		.of_match_table = mir3da_of_match,
		.pm             = &mir3da_pm_ops
	},

	.probe          = mir3da_probe,
	.remove         = mir3da_remove,
	.id_table       = mir3da_id,
	.detect         = mir3da_detect,
	.address_list   = normal_i2c,
};

static int __init mir3da_init(void)
{
	int res;

	res = i2c_add_driver(&mir3da_driver);
	if (res < 0) {
		MI_ERR("add mir3da i2c driver failed\n");
		return -ENODEV;
	}
	MI_MSG("add mir3da i2c driver\n");
	return res;
}

static void __exit mir3da_exit(void)
{
	MI_FUN;

	MI_MSG("remove mir3da i2c driver.\n");
	i2c_del_driver(&mir3da_driver);

	/* mir3da_remove() already tore these down; clean up only leftovers. */
	if (mir3da_class_dev) {
		device_remove_file(mir3da_class_dev, &dev_attr_gsensor);
		device_remove_file(mir3da_class_dev, &dev_attr_delay_acc);
		device_remove_file(mir3da_class_dev, &dev_attr_scsensor);
		device_destroy(mir3da_class, 0);
		mir3da_class_dev = NULL;
	}
	if (mir3da_class) {
		class_destroy(mir3da_class);
		mir3da_class = NULL;
	}
}

MODULE_AUTHOR("MiraMEMS <lschen@miramems.com>");
MODULE_DESCRIPTION("MIR3DA / DA217 3-Axis Accelerometer driver (xr-gsensor ABI)");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

module_init(mir3da_init);
module_exit(mir3da_exit);
