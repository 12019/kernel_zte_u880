/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
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
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#ifdef CONFIG_DVFM
#include <mach/dvfm.h>
#endif

//#define TS_KEY_REPORT 

//#define POLL_IN_INT
#define FTM_KEY_REPORT

#ifdef FTM_KEY_REPORT
static int ts_key_left = 102;
static int ts_key_middle = 139;
static int ts_key_right = 158;	
#endif

struct synaptics_ts_data
{
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct hrtimer resume_timer;
	//
	struct timer_list watchdog_timer;
	struct work_struct  watchdog_work;
	struct work_struct  work;
	uint16_t max[2];
	struct early_suspend early_suspend;
#ifdef CONFIG_DVFM
	int dvfm_dev_idx;
	struct dvfm_lock dvfm_lock;
	struct timer_list dvfm_timer;
	struct work_struct unset_dvfm_work;
	struct work_struct set_dvfm_work;
	unsigned int dvfm_timeout_val;
#endif

};

// algorithm for ts shaking begin
#define NR_SAMPHISTLEN  4

/* To keep things simple (avoiding division) we ensure that
 * SUM(weight) = power-of-two. Also we must know how to approximate
 * measurements when we have less than NR_SAMPHISTLEN samples.
 */
static const unsigned char weight [NR_SAMPHISTLEN - 1][NR_SAMPHISTLEN + 1] =
{
    /* The last element is pow2(SUM(0..3)) */
	{ 5, 3, 0, 0, 3 },  /* When we have 2 samples ... */
	{ 8, 5, 3, 0, 4 },  /* When we have 3 samples ... */
	{ 6, 4, 3, 3, 4 },  /* When we have 4 samples ... */
};

struct ts_sample {
	int		x;
	int		y;
	unsigned int	pressure;
};

struct ts_hist {
    int x;
    int y;
    unsigned int p;
};

struct tslib_dejitter {
    int delta;
	int delta_x;
	int delta_y;
    int x;
    int y;
    int down;
    int nr;
    int head;
    struct ts_hist hist[NR_SAMPHISTLEN];
};

static struct tslib_dejitter *ts_djt;
// algorithm for ts shaking end

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

struct synaptics_ts_data *g_ts;
static struct workqueue_struct *synaptics_wq;
static struct i2c_driver synaptics_ts_driver;

static int ts_debug = 0;    //

#ifdef TS_KEY_REPORT
const char ts_keys_size_synaptics[] = "0x01:102:160:1567:300:150:0x01:139:500:1567:300:150:0x01:158:830:1567:300:150";
struct attribute ts_key_report_attr_synaptics = {
        .name = "virtualkeys.synaptics-rmi-touchscreen",
        .mode = S_IRWXUGO,
};
 
static struct attribute *def_attrs_synaptics[] = {
        &ts_key_report_attr_synaptics,
        NULL,
};
 
void ts_key_report_synaptics_release(struct kobject *kobject)
{
        return;
}
 
ssize_t ts_key_report_synaptics_show(struct kobject *kobject, struct attribute *attr,char *buf)
{
        sprintf(buf,"%s\n",ts_keys_size_synaptics);
        return strlen(ts_keys_size_synaptics)+2;
}
 
ssize_t ts_key_report_synaptics_store(struct kobject *kobject,struct attribute *attr,const char *buf, size_t count)
{
        return count;
}
 
struct sysfs_ops ts_key_report_sysops_synaptics =
{
        .show = ts_key_report_synaptics_show,
        .store = ts_key_report_synaptics_store,
};
 
struct kobj_type ktype_synaptics = 
{
        .release = ts_key_report_synaptics_release,
        .sysfs_ops=&ts_key_report_sysops_synaptics,
        .default_attrs=def_attrs_synaptics,
};
 
struct kobject kobj_synaptics;
static void ts_key_report_synaptics_init(void)
{
	int ret = 0;
        ret = kobject_init_and_add(&kobj_synaptics,&ktype_synaptics,NULL,"board_properties");
	if(ret)
		printk(KERN_ERR "ts_key_report_init: Unable to init and add the kobject\n");
}
#endif

#if defined(CONFIG_DVFM)
static void set_dvfm_constraint(struct synaptics_ts_data *p)
{
      // printk("set_dvfm_constraint\n");
	spin_lock_irqsave(&p->dvfm_lock.lock, p->dvfm_lock.flags);
	dvfm_disable_op_name("apps_idle", p->dvfm_dev_idx);
	dvfm_disable_op_name("apps_sleep", p->dvfm_dev_idx);
	dvfm_disable_op_name("sys_sleep", p->dvfm_dev_idx);
	dvfm_disable_op_name("156MHz", p->dvfm_dev_idx);
	dvfm_disable_op_name("312MHz", p->dvfm_dev_idx);
	spin_unlock_irqrestore(&p->dvfm_lock.lock, p->dvfm_lock.flags);
}

static void unset_dvfm_constraint(struct synaptics_ts_data *p)
{
       // printk("unset_dvfm_constraint\n");
	spin_lock_irqsave(&p->dvfm_lock.lock, p->dvfm_lock.flags);
	dvfm_enable_op_name("apps_idle", p->dvfm_dev_idx);
	dvfm_enable_op_name("apps_sleep", p->dvfm_dev_idx);
	dvfm_enable_op_name("sys_sleep", p->dvfm_dev_idx);
	dvfm_enable_op_name("156MHz", p->dvfm_dev_idx);
	dvfm_enable_op_name("312MHz", p->dvfm_dev_idx);
	spin_unlock_irqrestore(&p->dvfm_lock.lock, p->dvfm_lock.flags);
}

static void unset_dvfm_handler(struct work_struct *work)
{
	struct synaptics_ts_data *p_work =  
		container_of(work, struct synaptics_ts_data, unset_dvfm_work);
	unset_dvfm_constraint(p_work);
}

static void set_dvfm_handler(struct work_struct *work)
{
	struct synaptics_ts_data *p_work =
		container_of(work, struct synaptics_ts_data, set_dvfm_work);
	set_dvfm_constraint(p_work);
}

static void dvfm_timer_handler(unsigned long data)
{
	struct synaptics_ts_data *p = (struct synaptics_ts_data *)data;
	schedule_work(&p->unset_dvfm_work);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
static void dvfm_timer_handler(unsigned long data) {}
#endif


static int synaptics_i2c_read(struct i2c_client *client, int reg, u8 * buf, int count)
{
    int rc;
    int ret = 0;

    buf[0] = 0xff;
	buf[1] = reg >> 8;
    rc = i2c_master_send(client, buf, 2);
	if (rc != 2)
	{
		dev_err(&client->dev, "synaptics_i2c_read FAILED: failed of page select %d\n", rc);
		ret = -1;
		goto tp_i2c_rd_exit;
	}
	buf[0] = 0xff & reg;
	rc = i2c_master_send(client, buf, 1);
    if (rc != 1)
    {
        dev_err(&client->dev, "synaptics_i2c_read FAILED: read of register %d\n", reg);
        ret = -1;
        goto tp_i2c_rd_exit;
    }
    rc = i2c_master_recv(client, buf, count);
    if (rc != count)
    {
        dev_err(&client->dev, "synaptics_i2c_read FAILED: read %d bytes from reg %d\n", count, reg);
        ret = -1;
    }

  tp_i2c_rd_exit:
    return ret;
}
static int synaptics_i2c_write(struct i2c_client *client, int reg, u8 data)
{
    u8 buf[2];
    int rc;
    int ret = 0;
	
    buf[0] = 0xff;
    buf[1] = reg >> 8;
    rc = i2c_master_send(client, buf, 2);
    if (rc != 2)
    {
        dev_err(&client->dev, "synaptics_i2c_write FAILED: writing to reg %d\n", reg);
        ret = -1;
    }
	buf[0] = 0xff & reg;
    buf[1] = data;
    rc = i2c_master_send(client, buf, 2);
    if (rc != 2)
    {
        dev_err(&client->dev, "synaptics_i2c_write FAILED: writing to reg %d\n", reg);
        ret = -1;
    }
	return ret;
}
#ifdef CONFIG_PROC_FS
#define	TOUCH_PROC_FILE	"driver/pm860x_touch"
static struct proc_dir_entry *synapatics_touch_proc_file;

//
static ssize_t proc_read_val(struct file *file,
    char __user *buffer, size_t count, loff_t *offset)
{
	uint8_t buf[16];
	uint8_t irq_status[3];
	int finger,ret;
	unsigned int gpio_plr_val, gpio_pdr_val, gpio_rer_val, gpio_fer_val, gpio_edr_val;
	void *REG_BASE;
	ssize_t len = 0;
	char buffer_synap[800];
	
	finger=0;//initializing the status
	ret = synaptics_i2c_read(g_ts->client, 0x0020, irq_status, 3);
	len += sprintf(buffer_synap+len, "synaptics_ts_work_func device control %x , irq_status %x %x \n",
		irq_status[0], irq_status[1], irq_status[2]);
	ret = synaptics_i2c_read(g_ts->client, 0x0014, buf, 16);

    if (ret < 0)
    {
		len += sprintf(buffer_synap+len, "synaptics_ts_work_func: i2c_transfer failed\n");
		ret = synaptics_i2c_write(g_ts->client, 0x0020, 0x01);      /* deep sleep *//*cl value need change*/
		if (ret < 0)
			len += sprintf(buffer_synap+len, "synaptics go to deep sleep: synaptics_i2c_write failed\n");
		msleep(10);
		ret = synaptics_i2c_write(g_ts->client, 0x0020, 0x00); /*cl set nomal operation*/
		if (ret < 0)
			len += sprintf(buffer_synap+len, "synaptics set nomal operation: synaptics_i2c_write failed\n");
	}
    else
    {
		len += sprintf(buffer_synap+len, "synaptics_ts_work_func:"
			"%x %x %x %x %x %x %x %x %x"
				" %x %x %x %x %x %x, ret %d\n",
				       buf[0], buf[1], buf[2], buf[3],
				       buf[4], buf[5], buf[6], buf[7],
					buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], ret);/**/
	}
	REG_BASE = ioremap_nocache(0xd4019004, 0xa8);
	if (REG_BASE == NULL) {
		goto failed;
	}
	gpio_plr_val = readl(REG_BASE + 0x00);
	gpio_pdr_val = readl(REG_BASE + 0x0c);
	gpio_rer_val = readl(REG_BASE + 0x30);
	gpio_fer_val = readl(REG_BASE + 0x3c);
	gpio_edr_val = readl(REG_BASE + 0x48);
	len += sprintf(buffer_synap+len, "GPIO49 status plr: %x pdr: %x rer: %x fer: %x edr: %x \n", 
		gpio_plr_val, gpio_pdr_val, gpio_rer_val, gpio_fer_val, gpio_edr_val);

	iounmap(REG_BASE);
	
failed:
	return simple_read_from_buffer(buffer, count, offset, buffer_synap, len);
}

static ssize_t proc_write_val(struct file *filp,
				     const char *buff, size_t len,
				     loff_t * off)
{
	char messages[256];
	uint8_t buf[16];
	
	int reg, val;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
    //printk(KERN_INFO "%s\n", messages);
    
	if ('a' == messages[0]) {
		sscanf(&messages[1], "%x %x", &reg, &val );
		if(val > 13)
		{
			val = 13;
		}else if(val <= 0)
		{
			val = 1;
		}
		synaptics_i2c_read(g_ts->client, reg, buf, val);
		printk("sscanf reg:0x%x val:0x%x", reg, val );
		printk("synaptics_ts_work_func: %x %x %x %x %x %x %x %x %x %x %x %x %x\n",
				       buf[0], buf[1], buf[2], buf[3],
				       buf[4], buf[5], buf[6], buf[7],
        buf[8], buf[9], buf[10], buf[11], buf[12]);
		
		return len;
	} else if ('w' == messages[0]) {
		sscanf(&messages[1], "%x %x", &reg, &val );
		printk("sscanf reg:0x%x val:0x%x", reg, val );
		synaptics_i2c_write(g_ts->client, reg, val);
		return len;
	} else if ('d' == messages[0]) {
		ts_debug = 1;
		return len;
	} else if('n' == messages[0]) {
		ts_debug = 0;
		return len;
	} else if('r' == messages[0]) {
		synaptics_i2c_write(g_ts->client, 0x0021, 0x00);
		msleep(10);
		synaptics_i2c_write(g_ts->client, 0x0020, 0x01);
		msleep(10);
		synaptics_i2c_write(g_ts->client, 0x0020, 0x00);
		msleep(10);
		synaptics_i2c_write(g_ts->client, 0x0021, 0x07);
	}

	return len;
}

static struct file_operations synapatics_touch_proc_ops = {
	.read = proc_read_val,
	.write = proc_write_val,
};

static void create_synapatics_touch_proc_file(void)
{
	synapatics_touch_proc_file =
	    create_proc_entry(TOUCH_PROC_FILE, 0777, NULL);
	if (synapatics_touch_proc_file) {
		synapatics_touch_proc_file->proc_fops = &synapatics_touch_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_synapatics_touch_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(TOUCH_PROC_FILE, &proc_root);
}

#endif

#ifdef FTM_KEY_REPORT
static int ts_key_report(int x, int y, int pen_down)
{
	static int keycode = 0;

	if(pen_down){
		if(y > 1530 && !keycode)
		{
		    if(x > 84 && x < 253)
		    {
                keycode = ts_key_left;
		    }
			else if(x > 415 && x < 561)
			{
			    keycode = ts_key_middle;
		    }
			else if(x > 747 && x < 909)
			{
			    keycode = ts_key_right;
		    }
			else
			{
				return 0;
			}
			//printk(KERN_INFO "--- ts_key x=%d\n", keycode);
			input_report_key(g_ts->input_dev, keycode, pen_down);
			input_sync(g_ts->input_dev);
			return 1;
		}
		else
			return 0;
	}
	else
	{
		if(keycode)
		{
			//printk(KERN_INFO "--- ts_key pen_up x=%d\n", keycode);
			input_report_key(g_ts->input_dev, keycode, pen_down);
			input_sync(g_ts->input_dev);
			keycode = 0;
			return 1;
		}
		else
			return 0;
	}
        
	return 0;
}
#endif
//
static void watchdog_timer_handler(unsigned long data)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)data;
	//printk("%s in\n", __FUNCTION__);
	
	schedule_work(&ts->watchdog_work);
	mod_timer(&ts->watchdog_timer, jiffies + msecs_to_jiffies(3000));
}

static void reset_watchdog_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, watchdog_work);
	uint8_t buf[16];
	int int_pin_val = 0;
	//Read intr pin, if low read data.
	int_pin_val = GPLR(49);
	int_pin_val = int_pin_val & 0x20000;
	
	//printk("%s %d\n", __FUNCTION__, int_pin_val);
	if(int_pin_val == 0)
	{
		printk("%s ...\n", __FUNCTION__);
		synaptics_i2c_read(ts->client, 0x0014, buf, 16);    //read data empty
	}
}

// algorithm for ts shaking begin
static int sqr (int x)
{
	return x * x;
}

static void average (struct tslib_dejitter *djt, struct ts_sample *samp)
{
	const unsigned char *w;
	int sn = djt->head;
	int i, x = 0, y = 0;
	unsigned int p = 0;

	w = weight [djt->nr - 2];

	for (i = 0; i < djt->nr; i++) {
		x += djt->hist [sn].x * w [i];
		y += djt->hist [sn].y * w [i];
		p += djt->hist [sn].p * w [i];
		sn = (sn - 1) & (NR_SAMPHISTLEN - 1);
	}

	samp->x = x >> w [NR_SAMPHISTLEN];
	samp->y = y >> w [NR_SAMPHISTLEN];
	samp->pressure = p >> w [NR_SAMPHISTLEN];
	
	//printk("average----------------> %d %d %d\n",
	//    samp->x, samp->y, samp->pressure);
}

static int djt_filter(struct tslib_dejitter *djt, struct ts_sample *samp)
{
    struct ts_sample *s, samp_data;//, pre_samp; not used 
	
    samp_data = *samp;
	s = &samp_data;
	
	{
        if (s->pressure == 0) {
            /*
             * Pen was released. Reset the state and
             * forget all history events.
             */
            djt->nr = 0;
            *samp = *s;
            goto out;
        }

        /* If the pen moves too fast, reset the backlog. */
        if (djt->nr) {
            int prev = (djt->head - 1) & (NR_SAMPHISTLEN - 1);
            if (sqr (s->x - djt->hist [prev].x) +
                sqr (s->y - djt->hist [prev].y) > djt->delta) {
                //printk("pen moves too fast\n");
                djt->nr = 0;
            }
        }

        djt->hist[djt->head].x = s->x;
        djt->hist[djt->head].y = s->y;
        djt->hist[djt->head].p = s->pressure;
        if (djt->nr < NR_SAMPHISTLEN)
            djt->nr++;

        /* We'll pass through the very first sample since
         * we can't average it (no history yet).
         */
        if (djt->nr == 1)
            *samp = *s;
        else {
            average (djt, samp);
        }
		
        djt->head = (djt->head + 1) & (NR_SAMPHISTLEN - 1);
    }
out:
    return 0;
}

static void simple_filter(int *x, int *y, int finger)
{
	static int pre_x = 0, pre_y = 0;
	struct ts_sample samp;
	
	samp.pressure = finger;
	samp.x = *x;
	samp.y = *y;
	djt_filter(ts_djt, &samp);
	*x = samp.x;
	*y = samp.y;/**/
	
	if(abs(*x - pre_x) <= 10 && abs(*y - pre_y) <= 16)
	{
		*x = pre_x;
		*y = pre_y;
	}
	pre_x = *x;
	pre_y = *y;
}
// algorithm for ts shaking end

static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret, x, y, z, finger, wx, wy, x2, y2, wx2, wy2, z2, finger2, pressure, pressure2;
	__s8  gesture, flick_y, flick_x;  
	uint8_t buf[16];
	//uint8_t irq_status[2];
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	finger=0;//initializing the status

	ret = synaptics_i2c_read(ts->client, 0x0014, buf, 16);

    if (ret < 0)
    {
		printk(KERN_ERR "synaptics_ts_work_func: i2c_transfer failed\n");
		ret = synaptics_i2c_write(ts->client, 0x0021, 0x00);    //disable int
		msleep(10);
		ret = synaptics_i2c_write(ts->client, 0x0020, 0x01);      /* deep sleep *//*cl value need change*/
		if (ret < 0)
			printk(KERN_ERR "synaptics go to deep sleep: synaptics_i2c_write failed\n");
		msleep(10);
		ret = synaptics_i2c_write(ts->client, 0x0020, 0x00); /*cl set nomal operation*/
		msleep(10);
		ret = synaptics_i2c_write(ts->client, 0x0021, 0x07);    //reenable int
		if (ret < 0)
			printk(KERN_ERR "synaptics set nomal operation: synaptics_i2c_write failed\n");
	}
    else
    {
		/*printk(KERN_WARNING "synaptics_ts_work_func:"
		"%x %x %x %x %x %x %x %x %x"
				       " %x %x %x %x %x %x, ret %d\n",
				       buf[0], buf[1], buf[2], buf[3],
				       buf[4], buf[5], buf[6], buf[7],
        buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], ret);*/
		x = (uint16_t) buf[2] << 4| (buf[4] & 0x0f) ; 
		y = (uint16_t) buf[3] << 4| ((buf[4] & 0xf0) >> 4); 
		pressure = buf[6];
		wx = buf[5] & 0x0f;
		wy = buf[5] >> 4;
		z = buf[6];
		finger = buf[1] & 0x3;

		x2 = (uint16_t) buf[7] << 4| (buf[9] & 0x0f) ;  
		y2 = (uint16_t) buf[8] << 4| ((buf[9] & 0xf0) >> 4); 

		pressure2 = buf[11]; 
		wx2 = buf[10] & 0x0f;
		wy2 = buf[10] >> 4;
		z2 = buf[11];
		
		finger2 = buf[1] & 0xc; 
		gesture = buf[12];

		flick_x = buf[14];
		flick_y = buf[15];
		
		if(ts_debug)
			printk("cl: x=%d,y=%d,z=%d,wx=%d,wy=%d,x2=%d,y2=%d,z2=%d,wx2=%d,wy2=%d,\n",
				x, y, z, wx, wy, x2, y2, z2, wx2, wy2);
		
#ifdef FTM_KEY_REPORT
		if(ts_key_report(x, y, finger))
		{
			return;
		}
#endif
		// algorithm for ts shaking
		simple_filter(&x, &y, finger);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, finger? max(wx, wy) : 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(ts->input_dev);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, finger2? max(wx2, wy2) : 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x2);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y2);
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);
#if 0
		if(finger)
        {   
        	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 1);
			input_report_abs(ts->input_dev, ABS_PRESSURE, 255);
			input_report_abs(ts->input_dev, ABS_X, x);
			input_report_abs(ts->input_dev, ABS_Y, y);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 255);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_mt_sync(ts->input_dev);
			
			if(finger2)
			{
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 1);
				input_report_abs(ts->input_dev, ABS_PRESSURE, 255);
				input_report_abs(ts->input_dev, ABS_X, x2);
				input_report_abs(ts->input_dev, ABS_Y, y2);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x2);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y2);
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 255);
				input_report_key(ts->input_dev, BTN_2, 1);
				input_mt_sync(ts->input_dev);
			}
			input_sync(ts->input_dev);
		}
		else
		{
			input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
			input_report_key(ts->input_dev, BTN_TOUCH, 0);
			input_report_key(ts->input_dev, BTN_2, 0);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
			input_sync(ts->input_dev);
		}
#endif
	}
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	//printk("synaptics_ts_irq_handler\n");
	
	//
	mod_timer(&ts->watchdog_timer, jiffies + msecs_to_jiffies(3000));
	if (!work_pending(&ts->work))
		queue_work(synaptics_wq, &ts->work);

#if defined(CONFIG_DVFM)
	if (mod_timer(&ts->dvfm_timer, jiffies + ts->dvfm_timeout_val)) {
		/* timer is already active */ 
	} else {
	        schedule_work(&ts->set_dvfm_work);
	}
#endif

	return IRQ_HANDLED;
}

static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	uint8_t buf1[9], buf[16];
	int ret = 0;
	uint16_t max_x, max_y;
	
	msleep(250);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL)
    {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	client->driver = &synaptics_ts_driver;
	//pdata = client->dev.platform_data;
    //printk("cl:%s, ts->client->addr=%x\n", __FUNCTION__, ts->client->addr);
	{
		int retry = 3;
        while (retry-- > 0)
        {

            ret = synaptics_i2c_read(ts->client, 0x0078, buf1, 9);
			printk("cl: synaptics_i2c_read, %c, %d,%d,%d,%d,%d,%d,%d,%d\n",
				buf1[0],buf1[1],buf1[2],buf1[3],buf1[4],buf1[5],buf1[6],buf1[7],buf1[8]);
			if (ret >= 0)
				break;
			msleep(10);

		}
		if (retry < 0)
		{
			ret = -1;
			goto err_detect_failed;
		}
	}
	g_ts = ts;
	synaptics_i2c_write(ts->client, 0x0021, 0x00);
	synaptics_i2c_read(ts->client, 0x0014, buf, 16);    //read data empty
	ret = synaptics_i2c_write(ts->client, 0x0020, 0x00);    // cl set nomal operation
	ret = synaptics_i2c_read(ts->client, 0x002D, buf1, 2);

	ts->max[0] = max_x = 999;
	ts->max[1] = max_y = 1490;

	printk("%s, max_x=%d, max_y=%d\n", __FUNCTION__, max_x, max_y);
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	ts->input_dev->phys = "synaptics-rmi-touchscreen/input0";

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(ABS_X, ts->input_dev->absbit);
	set_bit(ABS_Y, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(ABS_PRESSURE, ts->input_dev->absbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	
#ifdef FTM_KEY_REPORT
	__set_bit(102, ts->input_dev->keybit);
	__set_bit(139, ts->input_dev->keybit);
	__set_bit(158, ts->input_dev->keybit);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_X, 0, max_x, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_Y, 0, max_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	ret = input_register_device(ts->input_dev);
    if (ret)
    {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
#if defined(CONFIG_DVFM)
	ts->dvfm_dev_idx = -1;
	ts->dvfm_lock.lock = SPIN_LOCK_UNLOCKED;
	if (!dvfm_register("synaptics_i2c_rmi", &ts->dvfm_dev_idx)) {
		INIT_WORK(&ts->unset_dvfm_work, unset_dvfm_handler);
		INIT_WORK(&ts->set_dvfm_work, set_dvfm_handler);
		init_timer(&ts->dvfm_timer);
		ts->dvfm_timer.function = dvfm_timer_handler;
		ts->dvfm_timer.data = (unsigned long)ts;
		ts->dvfm_timeout_val = HZ * 2;
	}
#endif
	// algorithm for ts shaking begin
	ts_djt = kzalloc(sizeof(struct tslib_dejitter), GFP_KERNEL);
	if (ts_djt == NULL)
		return -ENOMEM;

	ts_djt->head = 0;
	ts_djt->nr =0;
	ts_djt->delta = 40;
	ts_djt->delta = sqr (ts_djt->delta);
	// algorithm for ts shaking end

	//
	INIT_WORK(&ts->watchdog_work, reset_watchdog_work);
	setup_timer(&ts->watchdog_timer, watchdog_timer_handler, (unsigned long) ts);
	mod_timer(&ts->watchdog_timer, jiffies + msecs_to_jiffies(3000));
	
	// printk("%s, client->irq=%d\n", __FUNCTION__, client->irq);

	ret = request_irq(client->irq, synaptics_ts_irq_handler, IRQF_DISABLED | IRQF_TRIGGER_FALLING, "synaptics_touch", ts);

	if(ret == 0)
	{
		ret = synaptics_i2c_write(ts->client, 0x0021, 0x07);  /* enable abs int */
		if (ret)
			free_irq(client->irq, ts);
	}
	else
	{
		dev_err(&client->dev, "synaptics_touch request_irq failed\n");
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

#ifdef CONFIG_PROC_FS
	create_synapatics_touch_proc_file();
#endif
	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in interrupt mode\n", ts->input_dev->name);

#ifdef TS_KEY_REPORT
	ts_key_report_synaptics_init();
#endif
	synaptics_i2c_read(ts->client, 0x0014, buf, 16);    //read data empty again
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
//err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	free_irq(client->irq, ts);
	
#if defined(CONFIG_DVFM)
	if( ts->dvfm_dev_idx >= 0 ) {
		del_timer(&ts->dvfm_timer);
		dvfm_unregister("synaptics_i2c_rmi", &ts->dvfm_dev_idx);
		ts->dvfm_dev_idx = -1;
	}
#endif

	input_unregister_device(ts->input_dev);
	kfree(ts);
#ifdef CONFIG_PROC_FS
	remove_synapatics_touch_proc_file();
#endif
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	printk("%s\n", __FUNCTION__);

	ret = synaptics_i2c_write(ts->client, 0x0021, 0);     /* disable interrupt */
	if (ret < 0)
	    printk(KERN_ERR "synaptics_ts_suspend: synaptics_i2c_write failed\n");

	ret = synaptics_i2c_write(client, 0x0020, 0x01);      /* deep sleep *//*cl value need change*/
	if (ret < 0)
	    printk(KERN_ERR "synaptics_ts_suspend: synaptics_i2c_write failed\n");

	disable_irq(client->irq);

	ret = cancel_work_sync(&ts->work);
	if (ret) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);

	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	uint8_t buf[16];
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	printk("%s\n", __FUNCTION__);

	enable_irq(client->irq);

	ret = synaptics_i2c_write(ts->client, 0x0020, 0x00); /*cl set nomal operation*/
	synaptics_i2c_read(ts->client, 0x0014, buf, 16);    //read data empty
	synaptics_i2c_write(ts->client, 0x0021, 0x07);    /* enable abs int */
	//synaptics_i2c_write(ts->client, 0x0031, 0x7F); /*cl set 2D gesture enable*/
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_I2C_RMI_NAME, 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= SYNAPTICS_I2C_RMI_NAME,
	},
};

static int __devinit synaptics_ts_init(void)
{
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq)
		return -ENOMEM;
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
