// SPDX-License-Identifier: GPL-2.0
/*
 * nekoboot_keyrescue.c - boot-animation-window key rescue
 * Target: S16B Plus watch (Unisoc UWS6152_1h10_go / ums312 / sharkl5,
 *         Linux 4.14.98 arm64, ReSukiSU/KernelSU + SUSFS)
 *
 * Physical keys (device DTBO gpio-keys overlay, see KEYRESCUE_DESIGN.md):
 *   side  key: KEY_F9    (linux,code 0x43 / 67, "Ai Key")
 *   power key: KEY_POWER (linux,code 0x74 / 116, "Power Key")
 *
 * While armed (from module_init - device_initcall level when built-in -
 * until userspace writes /proc/nekoboot_keyrescue_alive, or window_ms
 * expires):
 *   side  key x3  -> touch "disable" in /data/adb/modules/<lspd-prefix>*
 *                    (LSPosed framework) and reboot
 *   side  key >=5 -> touch "disable" in every /data/adb/modules/<id>
 *                    (ALL modules) and reboot
 *   power key >=5 -> write boot-recovery BCB into misc and reboot
 *
 * Pure input observer: the filter callback ALWAYS returns false.
 * drivers/input/input.c input_to_handler() compacts the shared vals[]
 * array when a filter returns true, which would swallow the event for
 * every later handle on that device - we never do that.
 *
 * Atomic-context discipline:
 *   - filter (dev->event_lock held, IRQs off): count only, spinlock+timer
 *   - settle timer (softirq): threshold decision, snapshot, queue work
 *   - work (process): filp_open/kernel_write/iterate_dir + kernel_restart
 *
 * Kernel 4.14 API: filp_open + kernel_write (no set_fs needed),
 * input_register_handler/input_register_handle/input_open_device,
 * setup_timer(mod_timer) legacy timer signature, iterate_dir.
 *
 * Build constraints: top Makefile carries bare -Werror -Wall -Wundef
 * -Wstrict-prototypes -std=gnu89 (Makefile:420 area), so declarations
 * stay at block tops, unused params get __maybe_unused, printk formats
 * are exact.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/limits.h>

#define KR_PROC_NAME       "nekoboot_keyrescue_alive"
#define KR_MODULES_ROOT    "/data/adb/modules"
#define KR_DISABLE_NAME    "disable"
#define KR_MISC_PRIMARY    "/dev/block/by-name/misc"
#define KR_MISC_FALLBACK   "/dev/block/mmcblk0p4"
#define KR_BCB_SIZE        2048
#define KR_MAX_PREFIXES    8
#define KR_MAX_MODULES     128
#define KR_NAME_LEN        (NAME_MAX + 1)

/* module parameters (built-in cmdline form: nekoboot_keyrescue.<name>=..) */
static unsigned int window_ms = 180000;
static unsigned int gap_ms = 3000;
static unsigned int side_key = KEY_F9;    /* DTBO "Ai Key", 0x43 */
static unsigned int power_key = KEY_POWER; /* DTBO "Power Key", 0x74 */
static char *lspd_prefixes = "lspd,zygisk_lsposed,riru_lsposed";
static bool quiet;

module_param(window_ms, uint, 0644);
MODULE_PARM_DESC(window_ms, "armed window in ms, 0 = until proc disarm only");
module_param(gap_ms, uint, 0644);
MODULE_PARM_DESC(gap_ms, "max silence between presses of one burst, ms");
module_param(side_key, uint, 0644);
MODULE_PARM_DESC(side_key, "keycode treated as side key (default 67 KEY_F9)");
module_param(power_key, uint, 0644);
MODULE_PARM_DESC(power_key, "keycode treated as power key (default 116)");
module_param(lspd_prefixes, charp, 0444);
MODULE_PARM_DESC(lspd_prefixes,
		 "comma-separated module-id prefixes for action A (LSPosed)");
module_param(quiet, bool, 0644);
MODULE_PARM_DESC(quiet, "suppress pr_info chatter (rescue milestones stay)");

#define kr_info(fmt, ...)						\
	do { if (!quiet) pr_info("KEYRESCUE: " fmt, ##__VA_ARGS__); } while (0)

/* state */
static atomic_t kr_armed = ATOMIC_INIT(1);
static atomic_t kr_action_queued = ATOMIC_INIT(0);

static DEFINE_SPINLOCK(kr_lock);
static unsigned long kr_last_press;
static unsigned int kr_burst_key;
static unsigned int kr_burst_count;

/* snapshot handed from settle timer to work */
static unsigned int kr_snap_key;
static unsigned int kr_snap_count;

static struct timer_list kr_settle_timer;
static struct timer_list kr_window_timer;
static struct work_struct kr_work;
static struct proc_dir_entry *kr_proc_entry;

static char *kr_prefix[KR_MAX_PREFIXES];
static int kr_prefix_count;

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */

static void kr_parse_prefixes(void)
{
	char *buf, *p, *tok;
	int i;

	for (i = 0; i < KR_MAX_PREFIXES; i++)
		kr_prefix[i] = NULL;
	kr_prefix_count = 0;

	buf = kstrdup(lspd_prefixes ? lspd_prefixes : "", GFP_KERNEL);
	if (!buf)
		return;

	p = buf;
	while ((tok = strsep(&p, ",")) != NULL &&
	       kr_prefix_count < KR_MAX_PREFIXES) {
		char *dup;

		if (!*tok)
			continue;
		dup = kstrdup(tok, GFP_KERNEL);
		if (dup)
			kr_prefix[kr_prefix_count++] = dup;
	}
	kfree(buf);
}

static int kr_name_matches(const char *name)
{
	int i;

	for (i = 0; i < kr_prefix_count; i++) {
		size_t len = strlen(kr_prefix[i]);

		if (strncmp(name, kr_prefix[i], len) == 0)
			return 1;
	}
	return 0;
}

/*
 * "touch <dir>/disable": create-if-missing, no truncation of existing
 * content, exactly the semantics Magisk/ksud expect (existence check).
 */
static int kr_touch_disable(const char *mod)
{
	char path[NAME_MAX + 40];
	struct file *f;
	int n;

	n = snprintf(path, sizeof(path), "%s/%s/%s",
		     KR_MODULES_ROOT, mod, KR_DISABLE_NAME);
	if (n < 0 || n >= (int)sizeof(path))
		return -ENAMETOOLONG;

	f = filp_open(path, O_WRONLY | O_CREAT, 0644);
	if (IS_ERR(f)) {
		long err = PTR_ERR(f);

		pr_err("KEYRESCUE: touch %s failed (err %ld)\n", path, err);
		return err;
	}
	filp_close(f, NULL);
	kr_info("touched %s\n", path);
	return 0;
}

struct kr_dir_ctx {
	struct dir_context dc;
	char (*names)[KR_NAME_LEN];
	int count;
	int max;
};

static int kr_filldir(struct dir_context *dc, const char *name, int namlen,
		      loff_t __maybe_unused offset, u64 __maybe_unused ino,
		      unsigned int d_type)
{
	struct kr_dir_ctx *c = container_of(dc, struct kr_dir_ctx, dc);

	if (c->count >= c->max)
		return 1; /* buffer full, stop iteration */
	if (namlen <= 0 || namlen >= KR_NAME_LEN)
		return 0;
	if (name[0] == '.' &&
	    (namlen == 1 || (namlen == 2 && name[1] == '.')))
		return 0;
	if (d_type != DT_DIR && d_type != DT_UNKNOWN)
		return 0;
	memcpy(c->names[c->count], name, namlen);
	c->names[c->count][namlen] = '\0';
	c->count++;
	return 0;
}

/*
 * Enumerate /data/adb/modules and touch disable in every directory
 * (all=0) or only in prefix-matching ones (all=1 means LSPosed mode).
 *
 * Return 0  -> enumeration succeeded, reboot caller should proceed
 *              (even when 0 modules matched: intent was disable+reboot)
 * Return <0 -> /data (or modules root) unreachable, caller must NOT
 *              reboot, retry stays possible
 */
static int kr_disable_modules(int lspd_only)
{
	struct kr_dir_ctx ctx = {
		.dc = { .actor = kr_filldir, .pos = 0 },
		.names = NULL,
		.count = 0,
		.max = KR_MAX_MODULES,
	};
	struct file *dir;
	char (*names)[KR_NAME_LEN];
	int i, ret, matched, touched;

	names = kzalloc(KR_MAX_MODULES * KR_NAME_LEN, GFP_KERNEL);
	if (!names)
		return -ENOMEM;
	ctx.names = names;

	dir = filp_open(KR_MODULES_ROOT, O_RDONLY | O_DIRECTORY, 0);
	if (IS_ERR(dir)) {
		long err = PTR_ERR(dir);

		pr_err("KEYRESCUE: open %s failed (err %ld), /data not mounted? aborting action, no reboot\n",
		       KR_MODULES_ROOT, err);
		kfree(names);
		return err;
	}

	ret = iterate_dir(dir, &ctx.dc);
	filp_close(dir, NULL);
	if (ret)
		kr_info("iterate_dir partial (err %d), using %d entries\n",
			ret, ctx.count);

	matched = 0;
	touched = 0;
	for (i = 0; i < ctx.count; i++) {
		if (lspd_only && !kr_name_matches(names[i]))
			continue;
		matched++;
		if (kr_touch_disable(names[i]) == 0)
			touched++;
	}
	kfree(names);

	if (matched == 0)
		kr_info("no matching modules (lspd_only=%d) - rebooting anyway, disable was a no-op\n",
			lspd_only);
	pr_emerg("KEYRESCUE: disabled %d/%d module dir(s) (lspd_only=%d)\n",
		 touched, matched, lspd_only);
	return 0;
}

/*
 * Write an Android bootloader control block asking for recovery.
 * Layout mirrors the existing nekoboot_bootguard.c (which is proven on
 * this exact device): 2048 zeroed bytes, "boot-recovery" at offset 0
 * (command[32]), "recovery\n" at offset 64 (recovery[768]).
 * Primary target: /dev/block/by-name/misc; fallback: /dev/block/mmcblk0p4.
 */
static int kr_write_recovery_bcb(void)
{
	struct file *f;
	void *bcb;
	loff_t pos = 0;
	ssize_t wret;

	bcb = kzalloc(KR_BCB_SIZE, GFP_KERNEL);
	if (!bcb)
		return -ENOMEM;

	memcpy(bcb, "boot-recovery", 13);
	memcpy((char *)bcb + 64, "recovery\n", 9);

	f = filp_open(KR_MISC_PRIMARY, O_WRONLY | O_SYNC, 0);
	if (IS_ERR(f)) {
		long err = PTR_ERR(f);

		pr_err("KEYRESCUE: open %s failed (err %ld), trying %s\n",
		       KR_MISC_PRIMARY, err, KR_MISC_FALLBACK);
		f = filp_open(KR_MISC_FALLBACK, O_WRONLY | O_SYNC, 0);
		if (IS_ERR(f)) {
			err = PTR_ERR(f);
			pr_err("KEYRESCUE: open %s failed too (err %ld), aborting, no reboot\n",
			       KR_MISC_FALLBACK, err);
			kfree(bcb);
			return err;
		}
	}

	wret = kernel_write(f, bcb, KR_BCB_SIZE, &pos);
	filp_close(f, NULL);
	kfree(bcb);

	if (wret != KR_BCB_SIZE) {
		pr_err("KEYRESCUE: BCB write incomplete (wret %zd/%d), aborting, no reboot\n",
		       wret, KR_BCB_SIZE);
		return wret < 0 ? (int)wret : -EIO;
	}

	pr_emerg("KEYRESCUE: BCB boot-recovery written to misc\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* rescue work (process context)                                      */
/* ------------------------------------------------------------------ */

static void kr_work_fn(struct work_struct __maybe_unused *work)
{
	unsigned int key = kr_snap_key;
	unsigned int count = kr_snap_count;
	int ret;

	pr_emerg("KEYRESCUE: settle: key=%u count=%u -> dispatching\n",
		 key, count);

	if (key == side_key) {
		if (count == 3) {
			ret = kr_disable_modules(1);
		} else if (count >= 5) {
			ret = kr_disable_modules(0);
		} else {
			kr_info("side key x%u: no action (x4 is deliberately inert; x1-2 ignored)\n",
				count);
			atomic_set(&kr_action_queued, 0);
			return;
		}
	} else if (key == power_key && count >= 5) {
		ret = kr_write_recovery_bcb();
	} else {
		kr_info("power key x%u: no action (need 5)\n", count);
		atomic_set(&kr_action_queued, 0);
		return;
	}

	if (ret != 0) {
		pr_err("KEYRESCUE: action aborted (err %d), NOT rebooting; retry allowed\n",
		       ret);
		atomic_set(&kr_action_queued, 0);
		return;
	}

	/* committed: give the log a moment, then reboot (bootguard pattern) */
	mdelay(200);
	kernel_restart(NULL);
}

/* ------------------------------------------------------------------ */
/* settle timer (softirq): threshold decision                          */
/* ------------------------------------------------------------------ */

static void kr_settle_fn(unsigned long __maybe_unused data)
{
	unsigned long flags;
	unsigned int key, count;

	if (!atomic_read(&kr_armed))
		return;
	if (atomic_read(&kr_action_queued))
		return;

	spin_lock_irqsave(&kr_lock, flags);
	key = kr_burst_key;
	count = kr_burst_count;
	kr_burst_key = 0;
	kr_burst_count = 0;
	spin_unlock_irqrestore(&kr_lock, flags);

	if (!count)
		return;

	if (key == side_key) {
		if (count == 3 || count >= 5) {
			kr_snap_key = key;
			kr_snap_count = count;
			atomic_set(&kr_action_queued, 1);
			if (!schedule_work(&kr_work))
				pr_err("KEYRESCUE: schedule_work failed\n");
		} else {
			kr_info("side key x%u: no action\n", count);
		}
	} else if (key == power_key) {
		if (count >= 5) {
			kr_snap_key = key;
			kr_snap_count = count;
			atomic_set(&kr_action_queued, 1);
			if (!schedule_work(&kr_work))
				pr_err("KEYRESCUE: schedule_work failed\n");
		} else {
			kr_info("power key x%u: no action\n", count);
		}
	}
}

static void kr_window_fn(unsigned long __maybe_unused data)
{
	if (atomic_xchg(&kr_armed, 0))
		pr_info("KEYRESCUE: %u ms window expired, self-disarmed\n",
			window_ms);
}

static void kr_arm_window(void)
{
	if (window_ms)
		mod_timer(&kr_window_timer,
			  jiffies + msecs_to_jiffies(window_ms));
}

/* ------------------------------------------------------------------ */
/* input handler (pure observer, never swallows events)               */
/* ------------------------------------------------------------------ */

static bool kr_filter(struct input_handle __maybe_unused *handle,
		      unsigned int type, unsigned int code, int value)
{
	unsigned long now = jiffies;
	unsigned long flags;
	int new_burst;

	if (!atomic_read(&kr_armed))
		return false;
	if (type != EV_KEY || value != 1)	/* only EV_KEY presses */
		return false;
	if (code != side_key && code != power_key)
		return false;
	if (atomic_read(&kr_action_queued))
		return false;

	spin_lock_irqsave(&kr_lock, flags);

	if (!kr_burst_count || kr_burst_key != code)
		new_burst = 1;	/* first press of a key switch */
	else if (time_after(now, kr_last_press + msecs_to_jiffies(gap_ms)))
		new_burst = 1;	/* previous press too old */
	else
		new_burst = 0;

	if (new_burst) {
		kr_burst_key = code;
		kr_burst_count = 1;
	} else {
		kr_burst_count++;
	}
	kr_last_press = now;

	spin_unlock_irqrestore(&kr_lock, flags);

	mod_timer(&kr_settle_timer, now + msecs_to_jiffies(gap_ms));

	/*
	 * ALWAYS false: input_to_handler() (input.c:100-118) removes
	 * events for which a filter returns true from the shared vals[]
	 * array passed to later handles. We observe, we never consume.
	 */
	return false;
}

static int kr_connect(struct input_handler *handler, struct input_dev *dev,
		      const struct input_device_id __maybe_unused *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "nekoboot_keyrescue";

	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;

	kr_info("watching input device %s\n", dev_name(&dev->dev));
	return 0;

err_unregister_handle:
	input_unregister_handle(handle);
err_free_handle:
	kfree(handle);
	return error;
}

static void kr_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id kr_ids[] = {
	{ .driver_info = 1 },	/* match all devices, filter by code */
	{ },			/* terminator */
};

static struct input_handler kr_handler = {
	.filter		= kr_filter,
	.connect	= kr_connect,
	.disconnect	= kr_disconnect,
	.name		= "nekoboot_keyrescue",
	.id_table	= kr_ids,
};

/* ------------------------------------------------------------------ */
/* /proc/nekoboot_keyrescue_alive                                      */
/* Semantics mirror /proc/nekoboot_alive of bootguard:                 */
/*   write '1' -> disarm (userspace alive)                             */
/*   write '0' -> re-arm (testing)                                     */
/*   read        -> "1" when disarmed/alive, "0" while armed           */
/* ------------------------------------------------------------------ */

static ssize_t kr_proc_write(struct file __maybe_unused *file,
			     const char __user *buf, size_t len,
			     loff_t __maybe_unused *off)
{
	char c;

	if (!len)
		return 0;
	if (copy_from_user(&c, buf, 1))
		return -EFAULT;

	if (c == '1') {
		if (atomic_xchg(&kr_armed, 0))
			pr_info("KEYRESCUE: disarmed by userspace (alive)\n");
	} else if (c == '0') {
		unsigned long flags;

		if (!atomic_xchg(&kr_armed, 1))
			pr_info("KEYRESCUE: re-armed by userspace\n");
		spin_lock_irqsave(&kr_lock, flags);
		kr_burst_key = 0;
		kr_burst_count = 0;
		spin_unlock_irqrestore(&kr_lock, flags);
		kr_arm_window();
	} else {
		return -EINVAL;
	}
	return len;
}

static ssize_t kr_proc_read(struct file __maybe_unused *file,
			    char __user *buf, size_t len, loff_t *off)
{
	char msg[8];
	int n;

	n = snprintf(msg, sizeof(msg), "%u\n",
		     atomic_read(&kr_armed) ? 0u : 1u);
	if (n < 0)
		return n;
	return simple_read_from_buffer(buf, len, off, msg, n);
}

static const struct file_operations kr_proc_fops = {
	.owner = THIS_MODULE,
	.write = kr_proc_write,
	.read  = kr_proc_read,
};

/* ------------------------------------------------------------------ */
/* init / exit                                                         */
/* ------------------------------------------------------------------ */

static int __init kr_init(void)
{
	int ret;

	kr_parse_prefixes();
	if (kr_prefix_count == 0)
		pr_err("KEYRESCUE: no valid lspd prefixes parsed, action A will match nothing\n");

	kr_proc_entry = proc_create(KR_PROC_NAME, 0644, NULL, &kr_proc_fops);
	if (!kr_proc_entry) {
		pr_err("KEYRESCUE: cannot create /proc/%s\n", KR_PROC_NAME);
		return -ENOMEM;
	}

	INIT_WORK(&kr_work, kr_work_fn);
	setup_timer(&kr_settle_timer, kr_settle_fn, 0);
	setup_timer(&kr_window_timer, kr_window_fn, 0);
	kr_arm_window();

	ret = input_register_handler(&kr_handler);
	if (ret) {
		pr_err("KEYRESCUE: input_register_handler failed: %d\n", ret);
		del_timer_sync(&kr_settle_timer);
		del_timer_sync(&kr_window_timer);
		proc_remove(kr_proc_entry);
		kr_proc_entry = NULL;
		return ret;
	}

	pr_info("KEYRESCUE: armed: window=%ums gap=%ums side=%u power=%u prefixes(%d)=\"%s\"\n",
		window_ms, gap_ms, side_key, power_key, kr_prefix_count,
		lspd_prefixes ? lspd_prefixes : "");
	return 0;
}
/*
 * module_init: works for both =y (runs at device_initcall level, input
 * core attaches our handler to already-registered devices at
 * input_register_handler time and auto-attaches to every later probe,
 * including deferred ones) and =m (late_initcall would never run for
 * a loadable module - tristate trap avoided).
 */
module_init(kr_init);

static void __exit kr_exit(void)
{
	int i;

	atomic_set(&kr_armed, 0);
	del_timer_sync(&kr_settle_timer);
	del_timer_sync(&kr_window_timer);
	cancel_work_sync(&kr_work);
	input_unregister_handler(&kr_handler);
	if (kr_proc_entry) {
		proc_remove(kr_proc_entry);
		kr_proc_entry = NULL;
	}
	for (i = 0; i < kr_prefix_count; i++)
		kfree(kr_prefix[i]);
}
module_exit(kr_exit);

MODULE_DESCRIPTION("nekoboot boot-window key rescue (S16B Plus)");
MODULE_LICENSE("GPL");
