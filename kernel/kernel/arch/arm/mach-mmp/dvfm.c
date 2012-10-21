/*
 * DVFM Abstract Layer
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007-2009 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/spinlock.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/kobject.h>
#include <linux/list.h>
#include <linux/notifier.h>

#include <asm/atomic.h>

#include <mach/dvfm.h>
#include <linux/earlysuspend.h>


#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
struct wake_lock constraint_wakelock;
static int wakelock_count = 0;
static spinlock_t wakelock_lock = SPIN_LOCK_UNLOCKED;
#endif

#define MAX_DEVNAME_LEN	32

extern struct info_head clock_trace_list;
extern int mspm_op_num;

/* This structure is used to dump device name list */
struct name_list {
	int	id;
	char	name[MAX_DEVNAME_LEN];
};

static ATOMIC_NOTIFIER_HEAD(dvfm_freq_notifier_list);

/*This variable records devices which disable low power modes*/
unsigned long lowpower_devices[8];

/*This variable records devices which disable global dvfm*/
unsigned long dvfm_devices[8];

/* This list links log of dvfm operation */
static struct info_head dvfm_trace_list = {
	.list = LIST_HEAD_INIT(dvfm_trace_list.list),
	.lock = RW_LOCK_UNLOCKED,
	.device = {0, 0, 0, 0, 0, 0, 0, 0},
};

/* This idx is used for user debug */
static int dvfm_dev_idx;

struct dvfm_driver *dvfm_driver = NULL;
struct info_head *dvfm_op_list = NULL;

unsigned int cur_op;			/* current operating point */
unsigned int def_op;			/* default operating point */
unsigned int op_nums = 0;		/* number of operating point */

static atomic_t lp_count = ATOMIC_INIT(0);	/* number of blocking lowpower mode */

extern struct sysdev_class cpu_sysdev_class;

int dvfm_find_op(int index, struct op_info **op)
{
	struct op_info *p = NULL;
	unsigned long flags;

	read_lock_irqsave(&dvfm_op_list->lock, flags);
	if (list_empty(&dvfm_op_list->list)) {
		read_unlock_irqrestore(&dvfm_op_list->lock, flags);
		return -ENOENT;
	}
	list_for_each_entry(p, &dvfm_op_list->list, list) {
		if (p->index == index) {
			*op = p;
			read_unlock_irqrestore(&dvfm_op_list->lock, flags);
			return 0;
		}
	}
	read_unlock_irqrestore(&dvfm_op_list->lock, flags);
	return -ENOENT;
}

/* Display current operating point */
static ssize_t op_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	struct op_info *op = NULL;
	int len = 0, req_op;

	req_op = cur_op;

	if (dvfm_driver->dump) {
		if (!dvfm_find_op(req_op, &op)) {
			len = dvfm_driver->dump(dvfm_driver->priv, op, buf);
		}
	}

	return len;
}

/* Set current operating point */
static ssize_t op_store(struct sys_device *sys_dev,
			struct sysdev_attribute *attr,
			const char *buf, size_t len)
{
	int new_op;
	if(sscanf(buf, "%u", &new_op) != 1){
		printk("Value set fail!Please retry!\n");
		return len;
	}
	dvfm_request_op(new_op);
	return len;
}
static SYSDEV_ATTR(op, 0644, op_show, op_store);

/* Dump all operating point */
static ssize_t ops_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	struct op_info *entry = NULL;
	int len = 0;
	char *p = NULL;

	if (!dvfm_driver->dump)
		return 0;
	read_lock(&dvfm_op_list->lock);
	if (!list_empty(&dvfm_op_list->list)) {
		list_for_each_entry(entry, &dvfm_op_list->list, list) {
			p = buf + len;
			len += dvfm_driver->dump(dvfm_driver->priv, entry, p);
			len += sprintf(buf + len, "\n");
		}
	}
	read_unlock(&dvfm_op_list->lock);

	return len;
}
SYSDEV_ATTR(ops, 0444, ops_show, NULL);

/* Dump all enabled operating point */
static ssize_t enable_op_show(struct sys_device *sys_dev,
				struct sysdev_attribute *attr, char *buf)
{
	struct op_info *entry = NULL;
	int len = 0;
	char *p = NULL;

	if (!dvfm_driver->dump)
		return 0;
	read_lock(&dvfm_op_list->lock);
	if (!list_empty(&dvfm_op_list->list)) {
		list_for_each_entry(entry, &dvfm_op_list->list, list) {
			if (find_first_bit(entry->device, DVFM_MAX_CLIENT)\
					== DVFM_MAX_CLIENT) {
				p = buf + len;
				len += dvfm_driver->dump(dvfm_driver->priv, entry, p);
			}
		}
	}
	read_unlock(&dvfm_op_list->lock);

	return len;
}

static int savePowerFlags=0;
extern int get_charger_status(void);

static ssize_t enable_op_store(struct sys_device *sys_dev,
				struct sysdev_attribute *attr,
				const char *buf, size_t len)
{
	int op, level;

	if(sscanf(buf, "%u,%u", &op, &level) != 2){
		printk("Value set fail!Please retry!\n");
		return len;
	}
	
   #ifdef CONFIG_PXA_806M
		if (level) 
		{  
		    if((savePowerFlags==1)&&(op==3))
		    {
		     printk("savepower mode ,not to enable op3\n");
		    }
			else
			{
			 printk("enable op=%d==================\n",op);
			 #if 1
			  if(1==get_charger_status()&&(op==0||op==1))
			  	{
			  	 printk("charger catn't enable op0,op1\n");
			  	}
			  else
			  #endif
			  	{
		dvfm_enable_op(op, dvfm_dev_idx);
			  	}
			}
		} 
		else
		{
		   
		    printk("disable op=%d==================\n",op);
			dvfm_disable_op(op, dvfm_dev_idx);
		}
		return len;
	#else
		if(level) 
		{  
			if(op!=3)
			{
			 
			  #if 1 //with charger fix 624M HZ
			  if(1==get_charger_status()&&(op==0||op==1))
			  	{
			  	  printk("charger catn't enable op0,op1\n");
			  	}
			  else
			   #endif
			  	{
			      dvfm_enable_op(op, dvfm_dev_idx);  
			  	}
			}	
		} 
		else
		{    if(op!=3)
			 {
		dvfm_disable_op(op, dvfm_dev_idx);
			 }
		}
	return len;
	#endif
}
SYSDEV_ATTR(enable_op, 0644, enable_op_show, enable_op_store);


static ssize_t savepower_store(struct sys_device *sys_dev,
				struct sysdev_attribute *attr,
				const char *buf, size_t len)
{
	int op, level;

	if(sscanf(buf, "%u,%u", &op, &level) != 2){
		printk("Value set fail!Please retry!\n");
		return len;
	}
	//printk("op:level=%d,%d\n",op,level);
	//printk("dvfm_driver->get_max_op()=%d\n",dvfm_driver->get_max_op());
	
  #ifdef CONFIG_PXA_806M
	if(op!=3)
	{
	 printk("op is not max op!Please retry!\n");
	 return len;
	}
	if (level) 
	{   
	    dvfm_enable_op(3, dvfm_dev_idx);	 
		savePowerFlags=0;
	} 
	else
	{
		dvfm_disable_op(3, dvfm_dev_idx);
	    savePowerFlags=1;
	}
	return len;
 #else
   return len;
 #endif
 
}

static ssize_t savepower_show(struct sys_device *sys_dev,
				struct sysdev_attribute *attr, char *buf)

{
	int len=0;
	
    #ifdef CONFIG_PXA_806M
	len=sprintf(buf, "savePowerFlags,supportsavepower = %d,%d\n", savePowerFlags,1);
	#else
	len=sprintf(buf, "savePowerFlags,supportsavepower = %d,%d\n", savePowerFlags,0);
	#endif
    printk("len=%d\n",len);
	return len;
}

SYSDEV_ATTR(savepower, 0644, savepower_show, savepower_store);

int get_save_power_mode(void)
{
 return savePowerFlags;
}

EXPORT_SYMBOL(get_save_power_mode);


void fixed_cpu_624M(void)
{
  dvfm_disable_op(0, dvfm_dev_idx);
  dvfm_disable_op(1, dvfm_dev_idx);

}

void free_cpu_624M(void)
{
  dvfm_enable_op(0, dvfm_dev_idx);
  dvfm_enable_op(1, dvfm_dev_idx);

}

/*
 * Dump blocked device on specified OP.
 * And dump the device list that is tracked.
 */
static ssize_t trace_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	struct op_info *op_entry = NULL;
	struct dvfm_trace_info *entry = NULL;
	int len = 0, i;
	/* copy of constraint in operating point */
	unsigned long blocked[8];

	for (i = 0; i < op_nums; i++) {
		memset(blocked, 0, sizeof(blocked));

		read_lock(&dvfm_op_list->lock);
		/* op list shouldn't be empty because op_nums is valid */
		list_for_each_entry(op_entry, &dvfm_op_list->list, list) {
			if (op_entry->index == i)
				memcpy(blocked, (op_entry->device),\
					sizeof(blocked));
		}
		read_unlock(&dvfm_op_list->lock);

		/* If no device is blocking this OP, continue to check next OP. */
		if (find_first_bit(blocked, DVFM_MAX_CLIENT) == DVFM_MAX_CLIENT)
			continue;

		len += sprintf(buf + len, "Blocked devices on OP%d:", i);

		read_lock(&dvfm_trace_list.lock);
		list_for_each_entry(entry, &dvfm_trace_list.list, list) {
			if (test_bit(entry->index, blocked))
				len += sprintf(buf + len, "%s, ", entry->name);
		}
		read_unlock(&dvfm_trace_list.lock);

		len += sprintf(buf + len, "\n");
	}
	if (len == 0)
		len += sprintf(buf + len, "None device block OP\n");
	len += sprintf(buf + len, "Trace device list:\n");

	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		len += sprintf(buf + len, "%s, ", entry->name);
	}
	read_unlock(&dvfm_trace_list.lock);

	len += sprintf(buf + len, "\n");
	return len;
}
SYSDEV_ATTR(trace, 0444, trace_show, NULL);

static ssize_t clock_trace_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	struct clock_trace_info	*p __maybe_unused = NULL;
	int len = 0;

#ifdef CONFIG_PXA910_CLOCK_TRACE
	len += sprintf(buf + len, "CLOCK ON:\n");
	list_for_each_entry(p, &clock_trace_list.list, list) {
		if (test_bit(p->index, clock_trace_list.device)) {
			if (p->dev_id)
				len += sprintf(buf + len, "%s, ", p->dev_id);
			else if (p->con_id)
				len += sprintf(buf + len, "%s, ", p->con_id);
		}
	}
	len += sprintf(buf + len, "\n");
#endif
	return len;
}
SYSDEV_ATTR(clocktrace, 0444, clock_trace_show, NULL);

static ssize_t dvfm_trace_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	struct dvfm_trace_info *entry = NULL;
	int len = 0;
	/* copy of constraint in global dvfm */
	unsigned long blocked[8];

	memset(blocked, 0, sizeof(blocked));

	read_lock(&dvfm_op_list->lock);
	memcpy(blocked, dvfm_devices,sizeof(blocked));
	read_unlock(&dvfm_op_list->lock);

	/* No device blocked dvfm */
	if (find_first_bit(blocked, DVFM_MAX_CLIENT) == DVFM_MAX_CLIENT){
		len += sprintf(buf + len, "None device block global dvfm\n");
		return len;
	}

	len += sprintf(buf + len, "Blocked devices on global dvfm:\n");
	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		if (test_bit(entry->index, blocked))
			len += sprintf(buf + len, "%s, ", entry->name);
	}
	read_unlock(&dvfm_trace_list.lock);
	len += sprintf(buf + len, "\n");

	return len;
}
SYSDEV_ATTR(dvfmtrace, 0444, dvfm_trace_show, NULL);

static ssize_t lowpower_trace_show(struct sys_device *sys_dev,
			struct sysdev_attribute *attr, char *buf)
{
	struct dvfm_trace_info *entry = NULL;
	int len = 0;
	/* copy of constraint in global dvfm */
	unsigned long blocked[8];

	memset(blocked, 0, sizeof(blocked));

	read_lock(&dvfm_op_list->lock);
	memcpy(blocked, lowpower_devices,sizeof(blocked));
	read_unlock(&dvfm_op_list->lock);

	/* No device blocked dvfm */
	if (find_first_bit(blocked, DVFM_MAX_CLIENT) == DVFM_MAX_CLIENT){
		len += sprintf(buf + len, "None device block global dvfm\n");
		return len;
	}

	len += sprintf(buf + len, "Blocked devices on global dvfm:\n");
	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		if (test_bit(entry->index, blocked))
			len += sprintf(buf + len, "%s, ", entry->name);
	}
	read_unlock(&dvfm_trace_list.lock);
	len += sprintf(buf + len, "\n");

	return len;
}

static ssize_t lowpower_trace_store(struct sys_device *sys_dev,
				struct sysdev_attribute *attr,
				const char *buf, size_t len)
{
	int lowpower_enable;

	if (sscanf(buf, "%u", &lowpower_enable) == 1) {
		if (lowpower_enable) {
			dvfm_enable_lowpower(dvfm_dev_idx);
		} else
			dvfm_disable_lowpower(dvfm_dev_idx);
	}
	else
		pr_info("sscanf got nothing in lowpower_trace_store\n");
	return len;
}
SYSDEV_ATTR(lowpowertrace, 0644, lowpower_trace_show, lowpower_trace_store);

static struct attribute *dvfm_attr[] = {
	&attr_op.attr,
	&attr_ops.attr,
	&attr_enable_op.attr,
	&attr_trace.attr,
	&attr_clocktrace.attr,
	&attr_dvfmtrace.attr,
	&attr_lowpowertrace.attr,
	&attr_savepower.attr,
};

int dvfm_op_count(void)
{
	int ret = -EINVAL;

	if (dvfm_driver && dvfm_driver->count)
		ret = dvfm_driver->count(dvfm_driver->priv, dvfm_op_list);
	return ret;
}
EXPORT_SYMBOL(dvfm_op_count);

int dvfm_get_op(struct op_info **p)
{
	int req_op;

	req_op = cur_op;

	if (dvfm_find_op(req_op, p))
		return -EINVAL;
	return req_op;
}
EXPORT_SYMBOL(dvfm_get_op);

int dvfm_get_defop(void)
{
	return def_op;
}
EXPORT_SYMBOL(dvfm_get_defop);

int dvfm_get_opinfo(int index, struct op_info **p)
{
	if (dvfm_find_op(index, p))
		return -EINVAL;
	return 0;
}
EXPORT_SYMBOL(dvfm_get_opinfo);

int dvfm_set_op(struct dvfm_freqs *freqs, unsigned int new,
		unsigned int relation)
{
	int ret = -EINVAL;

	/* check whether dvfm is enabled */
	if (!dvfm_driver || !dvfm_driver->count)
		return -EINVAL;
	if (dvfm_driver->set)
		ret = dvfm_driver->set(dvfm_driver->priv, freqs, new, relation);
	return ret;
}

/* Request operating point. System may set higher frequency because of
 * device constraint.
 */
int dvfm_request_op(int index)
{
	int ret = -EFAULT;

	/* check whether dvfm is enabled */
	if (!dvfm_driver || !dvfm_driver->count)
		return -EINVAL;
	if (dvfm_driver->request_set)
		ret = dvfm_driver->request_set(dvfm_driver->priv, index);
	return ret;
}
EXPORT_SYMBOL(dvfm_request_op);

/*
 * Device remove the constraint on OP.
 */
int dvfm_enable_op(int index, int dev_idx)
{
	struct op_info *p = NULL;
	int num;
	unsigned long flags;

	/* check whether dvfm is enabled */
	if (!dvfm_driver || !dvfm_driver->count)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	num = dvfm_driver->count(dvfm_driver->priv, dvfm_op_list);
	if (num <= index)
		return -ENOENT;
	if (!dvfm_find_op(index, &p)) {
		write_lock_irqsave(&dvfm_op_list->lock, flags);
		/* remove device ID */
		clear_bit(dev_idx, p->device);
		write_unlock_irqrestore(&dvfm_op_list->lock, flags);
		dvfm_driver->enable_op(dvfm_driver->priv, index, RELATION_LOW);
	}
	return 0;
}

/*
 * Device set constraint on OP
 */
int dvfm_disable_op(int index, int dev_idx)
{
	struct op_info *p = NULL;
	int num;
	unsigned long flags;

	/* check whether dvfm is enabled */
	if (!dvfm_driver || !dvfm_driver->count)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	num = dvfm_driver->count(dvfm_driver->priv, dvfm_op_list);
	if (num <= index)
		return -ENOENT;
	if (!dvfm_find_op(index, &p)) {
		if(cur_op == index && index == dvfm_driver->get_max_op()){
			printk("Can't disable cur_op if it's currently the hightest op!\n");
			return 0;
		}
		write_lock_irqsave(&dvfm_op_list->lock, flags);
		/* set device ID */
		set_bit(dev_idx, p->device);
		write_unlock_irqrestore(&dvfm_op_list->lock, flags);
		dvfm_driver->disable_op(dvfm_driver->priv, index, RELATION_LOW);
	}
	return 0;
}
EXPORT_SYMBOL(dvfm_enable_op);
EXPORT_SYMBOL(dvfm_disable_op);

int dvfm_enable_op_name(char *name, int dev_idx)
{
	struct op_info *p = NULL;
	int index;
	unsigned long flags;
	unsigned long irqflags;

	if (!dvfm_driver || !dvfm_driver->name || !name)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	list_for_each_entry(p, &dvfm_op_list->list, list) {
		if (!strcmp(dvfm_driver->name(dvfm_driver->priv, p), name)) {
			index = p->index;
			write_lock_irqsave(&dvfm_op_list->lock, flags);
			clear_bit(dev_idx, p->device);
			write_unlock_irqrestore(&dvfm_op_list->lock, flags);

#if defined(CONFIG_WAKELOCK) && defined(CONFIG_DVFM_PXA910)
			if (!strcmp(name, "sys_sleep") || !strcmp(name, "apps_sleep")
				|| !strcmp(name, "apps_idle")) {
				spin_lock_irqsave(&wakelock_lock, irqflags);
				wakelock_count--;
				if (!wakelock_count)
					wake_unlock(&constraint_wakelock);
				spin_unlock_irqrestore(&wakelock_lock, irqflags);
			}
#endif

			dvfm_driver->enable_op(dvfm_driver->priv,
					index, RELATION_LOW);
			break;
		}
	}
	return 0;
}

int dvfm_disable_op_name(char *name, int dev_idx)
{
	struct op_info *p = NULL;
	int index;
	unsigned long flags;
	unsigned long irqflags;

	if (!dvfm_driver || !dvfm_driver->name || !name)
		return -EINVAL;
	/* only registered device can invoke DVFM operation */
	if ((dev_idx >= DVFM_MAX_CLIENT) || dev_idx < 0)
		return -ENOENT;
	list_for_each_entry(p, &dvfm_op_list->list, list) {
		if (!strcmp(dvfm_driver->name(dvfm_driver->priv, p), name)) {
			index = p->index;
			write_lock_irqsave(&dvfm_op_list->lock, flags);
			set_bit(dev_idx, p->device);
			write_unlock_irqrestore(&dvfm_op_list->lock, flags);

#if defined(CONFIG_WAKELOCK) && defined(CONFIG_DVFM_PXA910)
			if (!strcmp(name, "sys_sleep") || !strcmp(name, "apps_sleep")
				|| !strcmp(name, "apps_idle")) {
				spin_lock_irqsave(&wakelock_lock, irqflags);
				 if (!wakelock_count)
					wake_lock(&constraint_wakelock);
				 wakelock_count++;
				spin_unlock_irqrestore(&wakelock_lock, irqflags);
			}
#endif

			dvfm_driver->disable_op(dvfm_driver->priv,
					index, RELATION_LOW);
			break;
		}
	}
	return 0;
}
EXPORT_SYMBOL(dvfm_enable_op_name);
EXPORT_SYMBOL(dvfm_disable_op_name);

void dvfm_enable_global(int dev_idx)
{
	clear_bit(dev_idx, dvfm_devices);
}

void dvfm_disable_global(int dev_idx)
{
	set_bit(dev_idx, dvfm_devices);
}

/* Only enable those safe operating point */
int dvfm_enable(int dev_idx)
{
	if (!dvfm_driver || !dvfm_driver->count || !dvfm_driver->enable_dvfm)
		return -ENOENT;
	return dvfm_driver->enable_dvfm(dvfm_driver->priv, dev_idx);
}

/* return whether the result is zero */
int dvfm_disable(int dev_idx)
{
	if (!dvfm_driver || !dvfm_driver->count || !dvfm_driver->disable_dvfm)
		return -ENOENT;
	return dvfm_driver->disable_dvfm(dvfm_driver->priv, dev_idx);
}

/*disable other ops to disable frequency change operation*/
int dvfm_disable_frequncy_change(int dev_idx)
{
	int i;
	for (i=0; i < mspm_op_num; i++) {
		if (i == cur_op)
			continue;
		dvfm_disable_op(i, dev_idx);
	}

	return 0;
}

/*enable other ops to disable frequency change operation*/
int dvfm_enable_frequncy_change(int dev_idx)
{
	int i;
	for (i=0; i < mspm_op_num; i++) {
		if (i == cur_op)
			continue;
		dvfm_enable_op(i, dev_idx);
	}

	return 0;
}

/* return whether the result is zero */
int dvfm_enable_pm(void)
{
	return atomic_inc_and_test(&lp_count);
}

/* return whether the result is zero */
int dvfm_disable_pm(void)
{
	return atomic_dec_and_test(&lp_count);
}

void dvfm_enable_lowpower(int dev_idx)
{
	unsigned long irqflags;
	if (!dvfm_driver || !dvfm_driver->name)
		return;
	spin_lock_irqsave(&wakelock_lock, irqflags);
	clear_bit(dev_idx, lowpower_devices);
	wakelock_count--;
	if (!wakelock_count)
		wake_unlock(&constraint_wakelock);
	spin_unlock_irqrestore(&wakelock_lock, irqflags);
}
EXPORT_SYMBOL(dvfm_enable_lowpower);

void dvfm_disable_lowpower(int dev_idx)
{
	unsigned long irqflags;
	if (!dvfm_driver || !dvfm_driver->name)
		return;
	spin_lock_irqsave(&wakelock_lock, irqflags);
	set_bit(dev_idx, lowpower_devices);
	if (!wakelock_count)
		wake_lock(&constraint_wakelock);
	wakelock_count++;
	spin_unlock_irqrestore(&wakelock_lock, irqflags);
}
EXPORT_SYMBOL(dvfm_disable_lowpower);

int dvfm_notifier_frequency(struct dvfm_freqs *freqs, unsigned int state)
{
	int ret;

	switch (state) {
	case DVFM_FREQ_PRECHANGE:
		ret = atomic_notifier_call_chain(&dvfm_freq_notifier_list,
					DVFM_FREQ_PRECHANGE, freqs);
		if (ret != NOTIFY_DONE)
			pr_debug("Failure in device driver before "
				"switching frequency\n");
		break;
	case DVFM_FREQ_POSTCHANGE:
		ret = atomic_notifier_call_chain(&dvfm_freq_notifier_list,
					DVFM_FREQ_POSTCHANGE, freqs);
		if (ret != NOTIFY_DONE)
			pr_debug("Failure in device driver after "
				"switching frequency\n");
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

int dvfm_register_notifier(struct notifier_block *nb, unsigned int list)
{
	int ret;

	switch (list) {
	case DVFM_FREQUENCY_NOTIFIER:
		ret = atomic_notifier_chain_register(
				&dvfm_freq_notifier_list, nb);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(dvfm_register_notifier);

int dvfm_unregister_notifier(struct notifier_block *nb, unsigned int list)
{
	int ret;

	switch (list) {
	case DVFM_FREQUENCY_NOTIFIER:
		ret = atomic_notifier_chain_unregister(
				&dvfm_freq_notifier_list, nb);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(dvfm_unregister_notifier);

/*
 * add device into trace list
 * return device index
 */
static int add_device(char *name)
{
	struct dvfm_trace_info	*entry = NULL, *new = NULL;
	int min;

	min = find_first_zero_bit(dvfm_trace_list.device, DVFM_MAX_CLIENT);
	/* client list is full */
	if (min == DVFM_MAX_CLIENT)
		return -EINVAL;

	/* If device trace table is NULL */
	new = kzalloc(sizeof(struct dvfm_trace_info), GFP_ATOMIC);
	if (new == NULL)
		goto out_mem;
	/* add new item */
	if (strlen(name) > MAX_DEVNAME_LEN)
		return -EINVAL;
	strcpy(new->name, name);
	new->index = min;
	/* insert the new item in increasing order */
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		if (entry->index > min) {
			list_add_tail(&(new->list), &(entry->list));
			goto inserted;
		}
	}
	list_add_tail(&(new->list), &(dvfm_trace_list.list));
inserted:
	set_bit(min, dvfm_trace_list.device);

	return min;
out_mem:
	return -ENOMEM;
}

/*
 * Query the device number that registered in DVFM
 */
int dvfm_query_device_num(void)
{
	int count = 0;
	struct dvfm_trace_info *entry = NULL;

	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		count++;
	}
	read_unlock(&dvfm_trace_list.lock);
	return count;
}
EXPORT_SYMBOL(dvfm_query_device_num);

/*
 * Query all device name that registered in DVFM
 */
int dvfm_query_device_list(void *mem, int len)
{
	int count = 0, size;
	struct dvfm_trace_info *entry = NULL;
	struct name_list *p = (struct name_list *)mem;

	count = dvfm_query_device_num();
	size = sizeof(struct name_list);
	if (len < count * size)
		return -ENOMEM;

	read_lock(&dvfm_trace_list.lock);
	list_for_each_entry(entry, &dvfm_trace_list.list, list) {
		p->id = entry->index;
		strcpy(p->name, entry->name);
		p++;
	}
	read_unlock(&dvfm_trace_list.lock);
	return 0;
}
EXPORT_SYMBOL(dvfm_query_device_list);

/*
 * Device driver register itself to DVFM before any operation.
 * The number of registered device is limited in 32.
 */
int dvfm_register(char *name, int *id)
{
	struct dvfm_trace_info	*p = NULL;
	int len, idx;

	if (name == NULL)
		return -EINVAL;

	/* device name is stricted in 32 bytes */
	len = strlen(name);
	if (len > DVFM_MAX_NAME)
		len = DVFM_MAX_NAME;
	write_lock(&dvfm_trace_list.lock);
	list_for_each_entry(p, &dvfm_trace_list.list, list) {
		if (!strcmp(name, p->name)) {
			/*
			 * Find device in device trace table
			 * Skip to allocate new ID
			 */
			*id = p->index;
			goto out;
		}
	}
	idx = add_device(name);
	if (idx < 0)
		goto out_num;
	*id = idx;
out:
	write_unlock(&dvfm_trace_list.lock);
	return 0;
out_num:
	write_unlock(&dvfm_trace_list.lock);
	return -EINVAL;
}
EXPORT_SYMBOL(dvfm_register);

/*
 * Release the device and free the device index.
 */
int dvfm_unregister(char *name, int *id)
{
	struct dvfm_trace_info	*p = NULL;
	int len, num, i;

	if (!dvfm_driver || !dvfm_driver->count || (name == NULL))
		return -EINVAL;

	/* device name is stricted in 32 bytes */
	len = strlen(name);
	if (len > DVFM_MAX_NAME)
		len = DVFM_MAX_NAME;

	num = dvfm_driver->count(dvfm_driver->priv, dvfm_op_list);

	write_lock(&dvfm_trace_list.lock);
	if (list_empty(&dvfm_trace_list.list))
		goto out;
	list_for_each_entry(p, &dvfm_trace_list.list, list) {
		if (!strncmp(name, p->name, len)) {
			/* remove all dvfm constraint on this device. */
			for (i = 0; i < num; i++)
				dvfm_enable_op(i, p->index);
			/* clear the device index */
			clear_bit(p->index, dvfm_trace_list.device);
			*id = -1;
			list_del(&p->list);
			kfree(p);
			break;
		}
	}
	write_unlock(&dvfm_trace_list.lock);
	return 0;
out:
	write_unlock(&dvfm_trace_list.lock);
	return -ENOENT;
}
EXPORT_SYMBOL(dvfm_unregister);

/*
 * Get current operating frequency of the core
 */
int dvfm_current_core_freq_get(void)
{
	if (dvfm_driver->current_core_freq_get != NULL) {
		return dvfm_driver->current_core_freq_get(dvfm_driver->priv);
	} else {
		/* not supported */
		return -1;
	}
}
EXPORT_SYMBOL(dvfm_current_core_freq_get);

int dvfm_core_freqs_table_get(int *freqs_table, int *size, int table_sz)
{
	if (dvfm_driver->core_freqs_table_get != NULL) {
		return dvfm_driver->core_freqs_table_get(dvfm_driver->priv,
							freqs_table,
							size,
							table_sz);
	} else {
		/* not supported */
		return -1;
	}
}
EXPORT_SYMBOL(dvfm_core_freqs_table_get);

static int dvfm_add(struct sys_device *sys_dev)
{
	int i, n;
	int ret;

	n = ARRAY_SIZE(dvfm_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj), dvfm_attr[i]);
		if (ret)
			return -EIO;
	}
	return 0;
}

static int dvfm_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(dvfm_attr);
	for (i = 0; i < n; i++) {
		sysfs_remove_file(&(sys_dev->kobj), dvfm_attr[i]);
	}
	return 0;
}

static int dvfm_suspend(struct sys_device *sysdev, pm_message_t pmsg)
{
    fixed_cpu_624M();
	return 0;
}

static int dvfm_resume(struct sys_device *sysdev)
{
	return 0;
}

static struct sysdev_driver dvfm_sysdev_driver = {
	.add		= dvfm_add,
	.remove		= dvfm_rm,
	.suspend	= dvfm_suspend,
	.resume		= dvfm_resume,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void dvfm_early_suspend_handler(struct early_suspend *h)
{
 
 printk("dvfm_early_suspend_handler========================\n");
}

void dvfm_late_resume_handler(struct early_suspend *h)
{
  printk("dvfm_late_resume_handler========================\n");
  if(0==get_charger_status())
  	{
  dvfm_enable_op(0, dvfm_dev_idx);
  dvfm_enable_op(1, dvfm_dev_idx);
  	}
  
  #ifdef CONFIG_PXA_806M
  dvfm_enable_op(3, dvfm_dev_idx);
  #endif
}

static struct early_suspend dvfm_android_suspend = {
	.level = 98,
	.suspend = dvfm_early_suspend_handler,
	.resume =  dvfm_late_resume_handler,
};

#endif


int dvfm_register_driver(struct dvfm_driver *driver_data, struct info_head *op_list)
{
	int ret;
	if (!driver_data || !driver_data->set)
		return -EINVAL;
	if (dvfm_driver)
		return -EBUSY;
	dvfm_driver = driver_data;

	if (!op_list)
		return -EINVAL;
	dvfm_op_list = op_list;
	#ifdef CONFIG_PXA910_DVFM_STATS
	       dvfm_add_timeslot(cur_op, CPU_STATE_RUN);
	#endif
	
    #ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&dvfm_android_suspend);
	#endif
	/* enable_op need to invoke dvfm operation */
	dvfm_register("User", &dvfm_dev_idx);
	ret = sysdev_driver_register(&cpu_sysdev_class, &dvfm_sysdev_driver);
	return ret;
}

int dvfm_unregister_driver(struct dvfm_driver *driver)
{
    
    #ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&dvfm_android_suspend);
    #endif
	sysdev_driver_unregister(&cpu_sysdev_class, &dvfm_sysdev_driver);
	dvfm_unregister("User", &dvfm_dev_idx);
	dvfm_driver = NULL;
	return 0;
}

MODULE_DESCRIPTION("Basic DVFM support");
MODULE_LICENSE("GPL");

