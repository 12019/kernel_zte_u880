/*
 * linux/arch/arm/mach-mmp/cpufreq-pxa910.c
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/sysdev.h>
#include <linux/cpu.h>
#if defined(CONFIG_DVFM_PXA910)
#include <mach/dvfm.h>
#include <mach/pxa910_dvfm.h>
static int dvfm_dev_idx;
#endif

#define MAX_CORE_FREQS	50
#define MHZ_TO_KHZ	1000

/* Core frequencies table according to PP,
 in MHz */
static int core_freqs_table[MAX_CORE_FREQS];
static int num_pp;

//static int dvfm_dev_idx;

static unsigned int pxa910_freqs_num;
static struct cpufreq_frequency_table *pxa910_freqs_table;


static unsigned int pxa910_cpufreq_get(unsigned int cpu);

/* Interface under SYSFS */
/* Display duty cycles on all operating points */
static ssize_t all_supported_freqs_show(struct sys_device *sys_dev,
					struct sysdev_attribute *attr,
					char *buf)
{
	int len, i;

	len = sprintf(buf, "Supported CPU frequencies:\n");
	for (i = 0; i < num_pp; i++) {
		len += sprintf(&buf[len], "freq[%d] = %d MHz\n",
				i, core_freqs_table[i]);
	}

	return len;
}
SYSDEV_ATTR(supported_freqs, 0444, all_supported_freqs_show, NULL);

/* Display duty cycles on all operating points */
static ssize_t current_freq_show(struct sys_device *sys_dev,
		struct sysdev_attribute *attr, char *buf)
{
	int freq;

	freq = pxa910_cpufreq_get(0);
	return sprintf(buf, "Current frequency = %d\n", freq);
}
SYSDEV_ATTR(current_freq, 0444, current_freq_show, NULL);


static struct attribute *cpufreq_stats_attr[] = {
	&attr_supported_freqs.attr,
	&attr_current_freq.attr,
};

static int stats_add(struct sys_device *sys_dev)
{
	int i, n, ret;
	n = ARRAY_SIZE(cpufreq_stats_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj),
							cpufreq_stats_attr[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int stats_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(cpufreq_stats_attr);
	for (i = 0; i < n; i++)
		sysfs_remove_file(&(sys_dev->kobj), cpufreq_stats_attr[i]);

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

static struct sysdev_driver cpufreq_stats_driver = {
	.add		= stats_add,
	.remove		= stats_rm,
	.suspend	= stats_suspend,
	.resume		= stats_resume,
};

static int freq_limits_get(int *freq_table, int num_pp, int *max, int *min)
{
	int max_freq;
	int min_freq;
	int i;

	max_freq = 0;
	min_freq = 0x7FFFFFFF;

	for (i = 0; i < num_pp; i++) {
		if (max_freq < freq_table[i])
			max_freq = freq_table[i];

		if (min_freq > freq_table[i])
			min_freq = freq_table[i];
	}

	*max = max_freq;
	*min = min_freq;

	return 0;
}

static int setup_freqs_table(struct cpufreq_policy *policy,
			     int *freqs_table,
			     int num)
{
	struct cpufreq_frequency_table *table;
	int i;

	table = kzalloc((num + 1) * sizeof(*table), GFP_KERNEL);
	if (table == NULL)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		table[i].index = i;
		table[i].frequency = freqs_table[i] * MHZ_TO_KHZ;
	}
	table[num].index = i;
	table[num].frequency = CPUFREQ_TABLE_END;

	pxa910_freqs_num = num;
	pxa910_freqs_table = table;

	return cpufreq_frequency_table_cpuinfo(policy, table);
}

#if 0
static int freq_constraint_set(int *freqs_table,
				int num_pp,
				int required_freq_khz)
{
	int freq_khz;
	int max_pp = num_pp - 1;
	int op_point;
	int ret = -EINVAL;

	printk(KERN_DEBUG "cpufreq set: Freq=%d KHz\n", required_freq_khz);

	/* Set the constraints acquired from the target frequency */
	for (op_point = 0; op_point < max_pp; op_point++) {
		freq_khz = freqs_table[op_point] * MHZ_TO_KHZ;
		if (freq_khz < required_freq_khz) {
			/* Set DVFM constraint on the operating point */
			ret = dvfm_disable_op(op_point, dvfm_dev_idx);
		} else {
			ret = dvfm_enable_op(op_point, dvfm_dev_idx);
		}
	}

	return op_point;
}
#endif

static int pxa910_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, pxa910_freqs_table);
}

static unsigned int pxa910_cpufreq_get(unsigned int cpu)
{
	if (cpu != 0)
		return -EINVAL;

	return dvfm_current_core_freq_get() * MHZ_TO_KHZ;
}

static int pxa910_cpufreq_target(struct cpufreq_policy *policy,
			unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	unsigned int op_point;
	unsigned int freq_khz = 0;

	freqs.old = policy->cur;
	freqs.cpu = policy->cpu;

	pr_debug("CPU frequency from %d MHz to %d MHz%s\n",
			freqs.old / MHZ_TO_KHZ, freqs.new / MHZ_TO_KHZ,
			(freqs.old == freqs.new) ? " (skipped)" : "");

	if (policy->cpu != 0)
		return -EINVAL;

	/* Set the required constraints as derived from the */
	/* target freq */
	//freq_constraint_set(core_freqs_table, num_pp, target_freq);
	for (op_point = 0; op_point < pxa910_freqs_num; op_point ++) {
		freq_khz = pxa910_freqs_table[op_point].frequency;
		if (target_freq <= freq_khz) {
			target_freq = op_point;
			break;
		}
	}

	freqs.new = freq_khz;
	/* notify cpufreq - the actual frequency change occurs */
	/* about 1mSec after cpufreq sets the DVFM constraints */
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	dvfm_request_op(target_freq);

	freqs.new = pxa910_cpufreq_get(policy->cpu);
	/* wait for the actual frequency change to happen */
	/* nanosleep(1000);*/
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static unsigned int pxa910_cpufreq_getavg(struct cpufreq_policy *policy, unsigned int cpu)
{
	int freq;
	freq = core_freqs_table[cur_op];
	return freq*MHZ_TO_KHZ;
}

static void set_constraint(unsigned int min, unsigned int max)
{
	unsigned int i;

	for(i = min; i <= max; i++)
		dvfm_disable_op(i, dvfm_dev_idx);
}

static void unset_constraint(unsigned int min, unsigned int max)
{
	unsigned int i;

	for(i = min; i <= max; i++)
		dvfm_enable_op(i, dvfm_dev_idx);
}

static void pxa910_cpufreq_constraint(struct cpufreq_policy *data,
					struct cpufreq_policy *policy)
{
	unsigned int old_min = 0, old_max = 0;
	unsigned int new_min = 0, new_max = 0;
	unsigned int freq_khz = 0;
	unsigned int i = 0;

	for (i = 0; i < pxa910_freqs_num; i++) {
		freq_khz = pxa910_freqs_table[i].frequency;
		if (freq_khz == data->min)
			old_min = i;
		if (freq_khz == data->max)
			old_max = i;
		if (freq_khz == policy->min)
			new_min = i;
		if (freq_khz == policy->max)
			new_max = i;
	}

	if(old_min < new_min)
		set_constraint(old_min, new_min - 1);
	if(old_min > new_min)
		unset_constraint(new_min, old_min - 1);

	if(old_max > new_max)
		set_constraint(new_max + 1, old_max);
	if(old_max < new_max)
		unset_constraint(old_max + 1, new_max);
}

static __init int pxa910_cpufreq_init(struct cpufreq_policy *policy)
{
	int current_freq_khz;
	int ret = 0;
	int max_freq = 0;
	int min_freq = 0;

#ifdef CONFIG_DVFM_PXA910
	//ret = dvfm_register("cpufreq", &dvfm_dev_idx);
	//if (ret)
		//pr_err("failed to register cpufreq");

	ret = dvfm_core_freqs_table_get(core_freqs_table,
					&num_pp, MAX_CORE_FREQS);
	if (ret)
		pr_err("failed to get cpu frequency table");


	ret = freq_limits_get(core_freqs_table, num_pp, &max_freq, &min_freq);
	if (ret)
		pr_err("failed to get cpu frequency limits");

	/* set to 0 to indicate that no user configured */
	/* cpufreq until now */
	current_freq_khz = dvfm_current_core_freq_get() * MHZ_TO_KHZ;

	pr_debug("max_freq=%d, min_freq=%d", max_freq, min_freq);

	/* set default policy and cpuinfo */
	policy->cpuinfo.min_freq = min_freq * MHZ_TO_KHZ;
	policy->cpuinfo.max_freq = max_freq * MHZ_TO_KHZ;
	policy->cpuinfo.transition_latency = 1000; /* 1mSec latency */

	policy->cur = policy->min = policy->max = current_freq_khz;

	/* setup cpuinfo frequency table */
	ret = setup_freqs_table(policy, core_freqs_table, num_pp);
	if (ret) {
		pr_err("failed to setup frequency table\n");
		return ret;
	}
#endif/* CONFIG_DVFM_PXA910 */
	cpufreq_frequency_table_get_attr(pxa910_freqs_table, policy->cpu);
	pr_info("CPUFREQ support for PXA910 initialized\n");
	return 0;
}

static struct freq_attr* pxa_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver pxa910_cpufreq_driver = {
	.verify		= pxa910_cpufreq_verify,
	.target		= pxa910_cpufreq_target,
	.init		= pxa910_cpufreq_init,
	.get		= pxa910_cpufreq_get,
	.getavg		= pxa910_cpufreq_getavg,
	.constraint	= pxa910_cpufreq_constraint,
	.name		= "pxa910-cpufreq",
	.attr		= pxa_freq_attr,
};

static int __init cpufreq_init(void)
{
	int ret;
#ifdef CONFIG_DVFM
	dvfm_register("cpufreq", &dvfm_dev_idx);
#endif
	ret = sysdev_driver_register(&cpu_sysdev_class, &cpufreq_stats_driver);
	if (ret)
		printk(KERN_ERR "Can't register cpufreq STATS in sysfs\n");

	return cpufreq_register_driver(&pxa910_cpufreq_driver);
}
module_init(cpufreq_init);

static void __exit cpufreq_exit(void)
{
//#ifdef CONFIG_DVFM_PXA910
//	dvfm_unregister("cpufreq", &dvfm_dev_idx);
//#endif/* CONFIG_DVFM_PXA910 */
#ifdef CONFIG_DVFM
	dvfm_unregister("cpufreq", &dvfm_dev_idx);
#endif
	sysdev_driver_unregister(&cpu_sysdev_class, &cpufreq_stats_driver);

	cpufreq_unregister_driver(&pxa910_cpufreq_driver);
}
module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for PXA3xx");
MODULE_LICENSE("GPL");
