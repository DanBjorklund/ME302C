/*
 * kernel/power/suspend.c - Suspend to RAM and standby functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 * Copyright (c) 2009 Rafael J. Wysocki <rjw@sisk.pl>, Novell Inc.
 *
 * This file is released under the GPLv2.
 */

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/syscalls.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/rtc.h>
#include <linux/ftrace.h>
#include <linux/workqueue.h>
#include <trace/events/power.h>
#include <linux/intel_mid_pm.h>

#include "power.h"

const char *const pm_states[PM_SUSPEND_MAX] = {
#ifdef CONFIG_EARLYSUSPEND
	[PM_SUSPEND_ON]		= "on",
#endif
	[PM_SUSPEND_STANDBY]	= "standby",
	[PM_SUSPEND_MEM]	= "mem",
};

static const struct platform_suspend_ops *suspend_ops;

static void do_suspend_sync(struct work_struct *work);
static DECLARE_WORK(suspend_sync_work, do_suspend_sync);
static DECLARE_COMPLETION(suspend_sync_complete);

static void do_suspend_sync(struct work_struct *work)
{
	sys_sync();
	complete(&suspend_sync_complete);
}

static bool check_sys_sync(void)
{
	while (!wait_for_completion_timeout(&suspend_sync_complete,
		HZ / 5)) {
		if (pm_wakeup_pending())
			return false;
		/* If sys_sync is doing, and no wakeup pending,
		 * we can try in loop to wait sys_sync() finish.
		 */
	}

	return true;
}

static bool suspend_sync(void)
{
	if (work_busy(&suspend_sync_work)) {
		/* When last sys_sync() work is still running,
		 * we need wait for it to be finished.
		 */
		if (!check_sys_sync())
			return false;
	}

	INIT_COMPLETION(suspend_sync_complete);
	schedule_work(&suspend_sync_work);

	return check_sys_sync();
}

/**
 * suspend_set_ops - Set the global suspend method table.
 * @ops: Suspend operations to use.
 */
void suspend_set_ops(const struct platform_suspend_ops *ops)
{
	lock_system_sleep();
	suspend_ops = ops;
	unlock_system_sleep();
}
EXPORT_SYMBOL_GPL(suspend_set_ops);

bool valid_state(suspend_state_t state)
{
	/*
	 * All states need lowlevel support and need to be valid to the lowlevel
	 * implementation, no valid callback implies that none are valid.
	 */
	return suspend_ops && suspend_ops->valid && suspend_ops->valid(state);
}

/**
 * suspend_valid_only_mem - Generic memory-only valid callback.
 *
 * Platform drivers that implement mem suspend only and only need to check for
 * that in their .valid() callback can use this instead of rolling their own
 * .valid() callback.
 */
int suspend_valid_only_mem(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}
EXPORT_SYMBOL_GPL(suspend_valid_only_mem);

static int suspend_test(int level)
{
#ifdef CONFIG_PM_DEBUG
	if (pm_test_level == level) {
		printk(KERN_INFO "suspend debug: Waiting for 5 seconds.\n");
		mdelay(5000);
		return 1;
	}
#endif /* !CONFIG_PM_DEBUG */
	return 0;
}

/**
 * suspend_prepare - Prepare for entering system sleep state.
 *
 * Common code run for every system sleep state that can be entered (except for
 * hibernation).  Run suspend notifiers, allocate the "suspend" console and
 * freeze processes.
 */
static int suspend_prepare(void)
{
	int error;

	if (!suspend_ops || !suspend_ops->enter)
		return -EPERM;

	pm_prepare_console();

	error = pm_notifier_call_chain(PM_SUSPEND_PREPARE);
	if (error)
		goto Finish;

	/* time stamp for start of process freezing */
	time_stamp_in_suspend_flow(PROC_FRZ, true);

	error = suspend_freeze_processes();

	/* time stamp for end of process freezing */
	time_stamp_in_suspend_flow(PROC_FRZ, false);

	if (!error)
		return 0;

	suspend_stats.failed_freeze++;
	dpm_save_failed_step(SUSPEND_FREEZE);
 Finish:
	pm_notifier_call_chain(PM_POST_SUSPEND);
	pm_restore_console();
	return error;
}

/* default implementation */
void __attribute__ ((weak)) arch_suspend_disable_irqs(void)
{
	local_irq_disable();
}

/* default implementation */
void __attribute__ ((weak)) arch_suspend_enable_irqs(void)
{
	local_irq_enable();
}

/**
 * suspend_enter - Make the system enter the given sleep state.
 * @state: System sleep state to enter.
 * @wakeup: Returns information that the sleep state should not be re-entered.
 *
 * This function should be called after devices have been suspended.
 */
static int suspend_enter(suspend_state_t state, bool *wakeup)
{
	int error;

	if (suspend_ops->prepare) {
		error = suspend_ops->prepare();
		if (error)
			goto Platform_finish;
	}

	error = dpm_suspend_end(PMSG_SUSPEND);
	if (error) {
		printk(KERN_ERR "PM: Some devices failed to power down\n");
		goto Platform_finish;
	}

	/* time stamp for end of device suspend */
	time_stamp_in_suspend_flow(DEV_SUS, false);

	if (suspend_ops->prepare_late) {
		error = suspend_ops->prepare_late();
		if (error)
			goto Platform_wake;
	}

	if (suspend_test(TEST_PLATFORM))
		goto Platform_wake;

	/* time stamp for start of disabling non-boot cpus */
	time_stamp_in_suspend_flow(NB_CPU_OFF, true);

	error = disable_nonboot_cpus();

	/* time stamp for end of disabling non-boot cpus */
	time_stamp_in_suspend_flow(NB_CPU_OFF, false);

	if (error || suspend_test(TEST_CPUS))
		goto Enable_cpus;

	arch_suspend_disable_irqs();
	BUG_ON(!irqs_disabled());

	error = syscore_suspend();
	if (!error) {
		*wakeup = pm_wakeup_pending();
		if (!(suspend_test(TEST_CORE) || *wakeup)) {
			error = suspend_ops->enter(state);
			events_check_enabled = false;
		}
		syscore_resume();
	}

	arch_suspend_enable_irqs();
	BUG_ON(irqs_disabled());

 Enable_cpus:
	/* time stamp for start of enabling non-boot cpus */
	time_stamp_in_suspend_flow(NB_CPU_ON, true);

	enable_nonboot_cpus();

	/* time stamp for end of enabling non-boot cpus */
	time_stamp_in_suspend_flow(NB_CPU_ON, false);

 Platform_wake:
	if (suspend_ops->wake)
		suspend_ops->wake();

	/* time stamp for start of device resume */
	time_stamp_in_suspend_flow(DEV_RES, true);

	dpm_resume_start(PMSG_RESUME);

 Platform_finish:
	if (suspend_ops->finish)
		suspend_ops->finish();

	return error;
}

/**
 * suspend_devices_and_enter - Suspend devices and enter system sleep state.
 * @state: System sleep state to enter.
 */
int suspend_devices_and_enter(suspend_state_t state)
{
	int error;
	bool wakeup = false;

	if (!suspend_ops)
		return -ENOSYS;

	trace_machine_suspend(state);
	if (suspend_ops->begin) {
		error = suspend_ops->begin(state);
		if (error)
			goto Close;
	}
	suspend_console();
	ftrace_stop();
	suspend_test_start();

	/* time stamp for start of device suspend */
	time_stamp_in_suspend_flow(DEV_SUS, true);

	error = dpm_suspend_start(PMSG_SUSPEND);
	if (error) {
		printk(KERN_ERR "PM: Some devices failed to suspend\n");
		goto Recover_platform;
	}
	suspend_test_finish("suspend devices");
	if (suspend_test(TEST_DEVICES))
		goto Recover_platform;

	do {
		error = suspend_enter(state, &wakeup);
	} while (!error && !wakeup
		&& suspend_ops->suspend_again && suspend_ops->suspend_again());

	if (wakeup)
		error = -EBUSY;

 Resume_devices:
	suspend_test_start();
	dpm_resume_end(PMSG_RESUME);

	/* time stamp for end of device resume */
	time_stamp_in_suspend_flow(DEV_RES, false);

	suspend_test_finish("resume devices");
	ftrace_start();
	resume_console();
 Close:
	if (suspend_ops->end)
		suspend_ops->end();
	trace_machine_suspend(PWR_EVENT_EXIT);
	return error;

 Recover_platform:
	if (suspend_ops->recover)
		suspend_ops->recover();
	goto Resume_devices;
}

/**
 * suspend_finish - Clean up before finishing the suspend sequence.
 *
 * Call platform code to clean up, restart processes, and free the console that
 * we've allocated. This routine is not called for hibernation.
 */
static void suspend_finish(void)
{
	/* time stamp for start of process unfreezing */
	time_stamp_in_suspend_flow(PROC_UNFRZ, true);

	suspend_thaw_processes();
        printk("PM: suspend_thaw_process done. \n");
	/* time stamp for end of process unfreezing */
	time_stamp_in_suspend_flow(PROC_UNFRZ, false);

	pm_notifier_call_chain(PM_POST_SUSPEND);
	printk("PM: pm_notifier_call_chain done. \n");
	pm_restore_console();
}

/**
 * enter_state - Do common work needed to enter system sleep state.
 * @state: System sleep state to enter.
 *
 * Make sure that no one else is trying to put the system into a sleep state.
 * Fail if that's not the case.  Otherwise, prepare for system suspend, make the
 * system enter the given sleep state and clean up after wakeup.
 */
static int enter_state(suspend_state_t state)
{
	int error;

	if (!valid_state(state))
		return -ENODEV;

	if (!mutex_trylock(&pm_mutex))
		return -EBUSY;

	printk(KERN_INFO "PM: Syncing filesystems ... ");
	if (!suspend_sync()) {
		printk(KERN_INFO "PM: Suspend aborted for filesystem syncing\n");
		error = -EBUSY;
		goto Unlock;
	}
	printk("done.\n");

	pr_debug("PM: Preparing system for %s sleep\n", pm_states[state]);
	error = suspend_prepare();
	if (error)
		goto Unlock;

	if (suspend_test(TEST_FREEZER))
		goto Finish;

	pr_debug("PM: Entering %s sleep\n", pm_states[state]);
	pm_restrict_gfp_mask();
	error = suspend_devices_and_enter(state);
	pm_restore_gfp_mask();

 Finish:
	pr_debug("PM: Finishing wakeup.\n");
	suspend_finish();
	printk("PM: suspend_finish() \n");
 Unlock:
	mutex_unlock(&pm_mutex);
	printk("PM: mutex_unlock -> pm_mutex \n");
	return error;
}

static void pm_suspend_marker(char *annotation)
{
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("PM: suspend %s %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		annotation, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}

/**
 * pm_suspend - Externally visible function for suspending the system.
 * @state: System sleep state to enter.
 *
 * Check if the value of @state represents one of the supported states,
 * execute enter_state() and update system suspend statistics.
 */
int pm_suspend(suspend_state_t state)
{
	int error;
	int s3_state = mid_state_to_sys_state(MID_S3_STATE);

	if (state <= PM_SUSPEND_ON || state >= PM_SUSPEND_MAX)
		return -EINVAL;

	/* time stamp for start of s3 entry */
	if (state == PM_SUSPEND_MEM)
		time_stamp_for_sleep_state_latency(s3_state, true, true);

	pm_suspend_marker("entry");
	error = enter_state(state);
	if (error) {
		suspend_stats.fail++;
		dpm_save_failed_errno(error);
	} else {
		suspend_stats.success++;
		/* time stamp for end of s3 exit */
		if (state == PM_SUSPEND_MEM)
			time_stamp_for_sleep_state_latency(s3_state,
							false, false);
	}
	pm_suspend_marker("exit");
	return error;
}
EXPORT_SYMBOL(pm_suspend);
