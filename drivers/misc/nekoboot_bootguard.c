/*
 * nekoboot_bootguard.c -- if userspace never comes up, park in recovery.
 *
 * WHY
 *   A kernel *panic* leaves ro.boot.bootreason=kernel_panic behind, so a
 *   userspace script can notice it on the next boot and run `reboot recovery`.
 *   A kernel *hang* leaves nothing at all: no panic, no bootreason, and
 *   userspace never runs, so no userspace agent can help.  The only place that
 *   can still act is the kernel itself, and only if timers are still running.
 *
 * WHAT
 *   Arm a delayed work at late_initcall.  Userspace disarms it by writing to
 *   /proc/nekoboot_alive once it is genuinely up.  If the timer fires first we
 *   write the Android bootloader control block into the misc partition:
 *
 *       struct bootloader_message { char command[32]; char status[32];
 *                                   char recovery[768]; ... }
 *       command  = "boot-recovery"
 *       recovery = "recovery\n"
 *
 *   then reboot.  The bootloader reads misc before booting and starts recovery
 *   instead of the system -- the same path `adb reboot recovery` uses.
 *
 * CAVEATS
 *   - A hang with interrupts disabled will not run this timer either; only a
 *     hardware watchdog can catch that.
 *   - Keep the timeout generous.  Booting slowly is normal; a false positive
 *     drops the user into recovery for no reason.
 *   - If recovery itself cannot boot, this will bounce.  Disarm below.
 *
 * CONTROLS
 *   deadline_ms=  module param, default 120000 (2 minutes), 0 disables.
 *   echo 1 > /proc/nekoboot_alive   disarm (userspace is up)
 *   echo 0 > /proc/nekoboot_alive   re-arm (only useful for testing)
 */

#define pr_fmt(fmt) "nekoboot_bootguard: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/slab.h>		/* kzalloc / kfree */
#include <linux/jiffies.h>	/* msecs_to_jiffies */
#include <linux/errno.h>

#define BOOTGUARD_MISC_DEV   "/dev/block/by-name/misc"
#define BOOTGUARD_MISC_DEV2  "/dev/block/mmcblk0p4"
#define BOOTGUARD_BCB_SIZE   2048
#define BOOTGUARD_CMD_OFF    0
#define BOOTGUARD_REC_OFF    64

static unsigned int deadline_ms = 300000;
module_param(deadline_ms, uint, 0644);
MODULE_PARM_DESC(deadline_ms, "ms to wait for userspace before booting recovery (0 = off)");

/*
 * Belt and braces.
 *
 * The primary disarm path is userspace writing to /proc/nekoboot_alive.  That
 * alone is fragile: if KernelSU never runs the disarming script, EVERY healthy
 * boot would be dumped into recovery.  So before rescuing, also ask the kernel
 * whether userspace visibly came up.  Either signal means "do not rescue":
 *
 *   /dev/socket/zygote      zygote publishes its socket early in bring-up
 *   /data/system/packages.xml  PackageManager wrote it during boot
 *
 * A boot that hangs at the first screen has neither, so the rescue still fires
 * for the case this was built for.
 */
static bool userspace_seems_up(void)
{
	static const char *const signals[] = {
		"/dev/socket/zygote",
		"/data/system/packages.xml",
	};
	struct file *f;
	int i;

	for (i = 0; i < ARRAY_SIZE(signals); i++) {
		f = filp_open(signals[i], O_RDONLY, 0);
		if (!IS_ERR(f)) {
			filp_close(f, NULL);
			pr_info("userspace is up (%s exists) - not rescuing\n",
				signals[i]);
			return true;
		}
	}
	return false;
}

static atomic_t guard_armed = ATOMIC_INIT(0);
static struct delayed_work guard_work;

static int write_recovery_bcb(void)
{
	struct file *f;
	mm_segment_t oldfs;
	loff_t pos = 0;
	char *bcb;
	ssize_t n;
	int ret = -EIO;

	bcb = kzalloc(BOOTGUARD_BCB_SIZE, GFP_KERNEL);
	if (!bcb)
		return -ENOMEM;

	memcpy(bcb + BOOTGUARD_CMD_OFF, "boot-recovery", 13);
	memcpy(bcb + BOOTGUARD_REC_OFF, "recovery\n", 9);

	f = filp_open(BOOTGUARD_MISC_DEV, O_WRONLY | O_SYNC, 0);
	if (IS_ERR(f)) {
		pr_err("open %s failed (%ld), trying %s\n",
		       BOOTGUARD_MISC_DEV, PTR_ERR(f), BOOTGUARD_MISC_DEV2);
		f = filp_open(BOOTGUARD_MISC_DEV2, O_WRONLY | O_SYNC, 0);
	}
	if (IS_ERR(f)) {
		pr_err("open %s failed too (%ld)\n",
		       BOOTGUARD_MISC_DEV2, PTR_ERR(f));
		kfree(bcb);
		return -ENODEV;
	}

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	n = vfs_write(f, bcb, BOOTGUARD_BCB_SIZE, &pos);
	set_fs(oldfs);
	filp_close(f, NULL);

	if (n == BOOTGUARD_BCB_SIZE) {
		ret = 0;
		pr_emerg("wrote boot-recovery BCB to misc (%zd bytes)\n", n);
	} else {
		pr_err("short write to misc: %zd\n", n);
	}

	kfree(bcb);
	return ret;
}

static void guard_fn(struct work_struct *w)
{
	if (!atomic_read(&guard_armed))
		return;

	/*
	 * Second chance: userspace may have come up without anything writing to
	 * /proc/nekoboot_alive.  Never rescue a boot that clearly got somewhere.
	 */
	if (userspace_seems_up()) {
		atomic_set(&guard_armed, 0);
		return;
	}

	pr_emerg("userspace did not come up within %u ms -- booting recovery\n",
		 deadline_ms);

	if (write_recovery_bcb())
		pr_emerg("BCB write failed; rebooting anyway\n");

	/* give printk a moment to reach the console, then restart */
	mdelay(200);
	kernel_restart(NULL);
}

static ssize_t alive_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	char kbuf[8];

	if (count == 0 || count >= sizeof(kbuf))
		return -EINVAL;
	if (copy_from_user(kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';

	if (kbuf[0] == '1') {
		if (atomic_xchg(&guard_armed, 0)) {
			cancel_delayed_work_sync(&guard_work);
			pr_info("disarmed: userspace is up\n");
		}
	} else if (kbuf[0] == '0') {
		if (!atomic_xchg(&guard_armed, 1)) {
			pr_info("re-armed for %u ms\n", deadline_ms);
			schedule_delayed_work(&guard_work,
					      msecs_to_jiffies(deadline_ms));
		}
	}
	return count;
}

static ssize_t alive_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	char kbuf[4];
	int len;

	len = scnprintf(kbuf, sizeof(kbuf), "%d\n",
			atomic_read(&guard_armed) ? 0 : 1);
	return simple_read_from_buffer(buf, count, ppos, kbuf, len);
}

static const struct file_operations alive_fops = {
	.owner = THIS_MODULE,
	.write = alive_write,
	.read  = alive_read,
};

static int __init bootguard_init(void)
{
	if (deadline_ms == 0) {
		pr_info("disabled (deadline_ms=0)\n");
		return 0;
	}

	INIT_DELAYED_WORK(&guard_work, guard_fn);

	if (!proc_create("nekoboot_alive", 0644, NULL, &alive_fops)) {
		pr_err("cannot create /proc/nekoboot_alive\n");
		return -ENOMEM;
	}

	atomic_set(&guard_armed, 1);
	schedule_delayed_work(&guard_work, msecs_to_jiffies(deadline_ms));
	pr_info("armed: %u ms to reach userspace (/proc/nekoboot_alive)\n",
		deadline_ms);

	return 0;
}
late_initcall(bootguard_init);
