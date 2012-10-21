/*
 * DVFM Statistic Driver
 *
 * Copyright (C) 2007 Marvell Corporation
 * Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/notifier.h>
#include <linux/jiffies.h>
#include <linux/relay.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>
#include <mach/dvfm.h>

struct op_stats_type {
	unsigned int timestamp;
	unsigned int op_idx;
	unsigned int runtime;
	unsigned int idletime;
	unsigned int overflow;		/* high halfword is run, low is idle */
};

struct op_cycle_type {
	unsigned int op_idx;
	unsigned int runtime;
	unsigned int idletime;
	unsigned int gpu_runtime;
	unsigned int gpu_idletime;
	unsigned int vpu_runtime;
	unsigned int vpu_idletime;
	unsigned int count;
};
#define OP_NUM			20
/* Detail time cost on all OPs are stored in op_stats table */
static struct op_cycle_type op_ticks_array[OP_NUM];
static spinlock_t stats_lock = SPIN_LOCK_UNLOCKED;

extern unsigned int cur_op;
extern int mspm_op_num;
/* Interface under SYSFS */

enum {
	GPU,
	VPU,
	CPU,
};

/* Display duty cycles on all operating points */
static ssize_t duty_cycle_show(struct sys_device *sys_dev, 
							struct sysdev_attribute *attr, char *buf)
{
	int len, i;
	unsigned int total_ticks;

	total_ticks = 0;
	for (i = 0; i < mspm_op_num; i++) {
		total_ticks += op_ticks_array[i].runtime
				+ op_ticks_array[i].idletime;
		printk("%d,   %d\n", op_ticks_array[i].runtime, op_ticks_array[i].idletime);
	}

	if (total_ticks == 0) {
		len = sprintf(buf, "No OP change, no duty cycle info\n");
		return len;
	}
	len = sprintf(buf, "Duty cycle of operating point list:\n");
	for (i = 0; i < mspm_op_num; i++) {
		len += sprintf(buf + len, "op%d run:%u%% idle:%u%%\n",
			i, op_ticks_array[i].runtime * 100/ total_ticks,
			op_ticks_array[i].idletime * 100 / total_ticks);
	}
	return len;
}
SYSDEV_ATTR(duty_cycle, 0444, duty_cycle_show, NULL);

/* Display costed time on all operating points */
static ssize_t ticks_show(struct sys_device *sys_dev, struct sysdev_attribute *attr, char *buf)
{
	int len, i;
	len = sprintf(buf, "Ticks of operating point list:\n");
	for (i = 0; i < mspm_op_num; i++) {
		len += sprintf(buf + len, "op%d, run ticks:%u, idle ticks:%u "
			"run second:%u, idle second:%u\n",
			i, op_ticks_array[i].runtime,
			op_ticks_array[i].idletime,
			dvfm_driver->ticks_to_sec(op_ticks_array[i].runtime),
			dvfm_driver->ticks_to_sec(op_ticks_array[i].idletime));
	}
	return len;
}
SYSDEV_ATTR(ticks, 0444, ticks_show, NULL);

/* Display gpu costed time on all operating points */
static ssize_t gpu_ticks_show(struct sys_device *sys_dev, struct sysdev_attribute *attr, char *buf)
{
	int len, i;
	len = sprintf(buf, "Ticks of operating point list:\n");
	for (i = 0; i < mspm_op_num; i++) {
		len += sprintf(buf + len, "op%d, gpu run ticks:%u, gpu idle ticks:%u "
			"gpu run second:%u, gpu idle second:%u\n",
			i, op_ticks_array[i].gpu_runtime,
			op_ticks_array[i].gpu_idletime,
			dvfm_driver->ticks_to_sec(op_ticks_array[i].gpu_runtime),
			dvfm_driver->ticks_to_sec(op_ticks_array[i].gpu_idletime));
	}
	return len;
}
SYSDEV_ATTR(gpu_ticks, 0444, gpu_ticks_show, NULL);

/* Display gpu costed time on all operating points */
static ssize_t vpu_ticks_show(struct sys_device *sys_dev, struct sysdev_attribute *attr, char *buf)
{
	int len, i;
	len = sprintf(buf, "Ticks of operating point list:\n");
	for (i = 0; i < mspm_op_num; i++) {
		len += sprintf(buf + len, "op%d, vpu run ticks:%u, vpu idle ticks:%u "
			"vpu run second:%u, vpu idle second:%u\n",
			i, op_ticks_array[i].vpu_runtime,
			op_ticks_array[i].vpu_idletime,
			dvfm_driver->ticks_to_sec(op_ticks_array[i].vpu_runtime),
			dvfm_driver->ticks_to_sec(op_ticks_array[i].vpu_idletime));
	}
	return len;
}
SYSDEV_ATTR(vpu_ticks, 0444, vpu_ticks_show, NULL);


static struct attribute *dvfm_stats_attr[] = {
	&attr_duty_cycle.attr,
	&attr_ticks.attr,
	&attr_gpu_ticks.attr,
	&attr_vpu_ticks.attr,
};

static void update_op_cycle(int op_idx, unsigned int runtime,
				unsigned int idletime, int dev_id)
{
	if (dev_id == GPU) {
		op_ticks_array[op_idx].gpu_runtime += runtime;
		op_ticks_array[op_idx].gpu_idletime += idletime;
	} else if (dev_id == VPU) {
		op_ticks_array[op_idx].vpu_runtime += runtime;
	    op_ticks_array[op_idx].vpu_idletime += idletime;
	} else {
	op_ticks_array[op_idx].runtime += runtime;
	op_ticks_array[op_idx].idletime += idletime;
	}
	//printk("%d,   %d\n", op_ticks_array[op_idx].runtime, op_ticks_array[op_idx].idletime);
}

/*
 * Add this information into timeslot table.
 * This table records the time cost on different cpu state and different
 * operating points.
 */
int dvfm_add_timeslot(int op_idx, int cpu_state)
{
	static unsigned int prev_timestamp;
	unsigned int timestamp, time;
	unsigned long flags;

	spin_lock_irqsave(&stats_lock, flags);
	timestamp = dvfm_driver->read_time();
	time = (timestamp >= prev_timestamp) ? timestamp - prev_timestamp
		: 0xFFFFFFFF - prev_timestamp + timestamp;
	prev_timestamp = timestamp;
	if (cpu_state == CPU_STATE_IDLE) {
		update_op_cycle(op_idx, 0, time, CPU);
	} else {
		update_op_cycle(op_idx, time, 0, CPU);
	}
	spin_unlock_irqrestore(&stats_lock, flags);
	return 0;
}

/*start profile gpu/vpu run/idle time
dev_id:  1 vpu/0 gpu
State: 1 run/0 idle
Start: 2 init/ 1 start/0 stop
*/
void start_profile(int dev_id, unsigned int state, unsigned int start)
{

	static unsigned int prev_timestamp;
	unsigned int timestamp, time, i;
	unsigned long flags;

	if (start == 2) {
		prev_timestamp = dvfm_driver->read_time(); //for init write timestamp
		if (dev_id == GPU) {
			for (i = 0; i < mspm_op_num; i++) {
				op_ticks_array[i].gpu_runtime = 0;
				op_ticks_array[i].gpu_idletime = 0;
			}
		} else if (dev_id == VPU) {
			for (i = 0; i < mspm_op_num; i++) {
				op_ticks_array[i].vpu_runtime = 0;
				op_ticks_array[i].vpu_idletime = 0;
			}
		} else {
			for (i = 0; i < mspm_op_num; i++) {
				op_ticks_array[i].runtime = 0;
				op_ticks_array[i].idletime = 0;
			}
		}
		return;
	} else if (start == 0)
		return;

	spin_lock_irqsave(&stats_lock, flags);
	timestamp = dvfm_driver->read_time();
	time = (timestamp >= prev_timestamp) ? timestamp - prev_timestamp
		: 0xFFFFFFFF - prev_timestamp + timestamp;
	prev_timestamp = timestamp;
	if (state == 1) { //run
		update_op_cycle(cur_op, time, 0, dev_id);
	} else { //idle
		update_op_cycle(cur_op, 0, time, dev_id);
	}
	spin_unlock_irqrestore(&stats_lock, flags);
}
EXPORT_SYMBOL(start_profile);

static int stats_add(struct sys_device *sys_dev)
{
	int i, n, ret;
	n = ARRAY_SIZE(dvfm_stats_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj), dvfm_stats_attr[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int stats_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(dvfm_stats_attr);
	for (i = 0; i < n; i++) {
		sysfs_remove_file(&(sys_dev->kobj), dvfm_stats_attr[i]);
	}
	return 0;
}

static int stats_suspend(struct sys_device *sysdev, pm_message_t pmsg)
{
	return 0;
}

static int stats_resume(struct sys_device *sysdev)
{
	return 0;
}

static struct sysdev_driver dvfm_stats_driver = {
	.add		= stats_add,
	.remove		= stats_rm,
	.suspend	= stats_suspend,
	.resume		= stats_resume,
};

int __init dvfm_stats_init(void)
{
	int ret;

	memset(&op_ticks_array, 0, sizeof(struct op_cycle_type) * OP_NUM);
	ret = sysdev_driver_register(&cpu_sysdev_class, &dvfm_stats_driver);
	if (ret)
		printk(KERN_ERR "Can't register DVFM STATS in sysfs\n");
	return ret;
}

void __exit dvfm_stats_exit(void)
{
	sysdev_driver_unregister(&cpu_sysdev_class, &dvfm_stats_driver);
}

module_init(dvfm_stats_init);
module_exit(dvfm_stats_exit);

