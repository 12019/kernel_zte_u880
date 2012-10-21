/*
 * Touchscreen driver for Marvell 88PM860x
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * 	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/mfd/88pm860x.h>

#include <linux/delay.h>
#include <linux/earlysuspend.h> 
#include <linux/proc_fs.h>
#include <asm/uaccess.h> 
#include <linux/wait.h>   
#include <linux/freezer.h>    
#ifdef CONFIG_DVFM
#include <mach/dvfm.h>
#endif


#define MEAS_LEN                (8)
#define ACCURATE_BIT            (12)

/* touch register */
#define MEAS_EN3		(0x52)

#define MEAS_TSIX_1		(0x8D)
#define MEAS_TSIX_2		(0x8E)
#define MEAS_TSIY_1		(0x8F)
#define MEAS_TSIY_2		(0x90)
#define MEAS_TSIZ1_1		(0x91)
#define MEAS_TSIZ1_2		(0x92)
#define MEAS_TSIZ2_1		(0x93)
#define MEAS_TSIZ2_2		(0x94)

/* bit definitions of touch */
#define MEAS_PD_EN		(1 << 3)
#define MEAS_TSIX_EN		(1 << 4)
#define MEAS_TSIY_EN		(1 << 5)
#define MEAS_TSIZ1_EN		(1 << 6)
#define MEAS_TSIZ2_EN		(1 << 7)


#define INVALID_XY		0xFFF
#define VALID_MIN_XY	0x0
#define VALID_MAX_XY    0xFFF
#define VALID_MAX_DIF   20

#ifdef CONFIG_PXA_U802
#define TS_POINT_LEFT		250
#define TS_POINT_RIGHT		3900
#define TS_POINT_TOP		4095
#define TS_POINT_BOTTOM		450

#define LCD_POINT_LEFT		1
#define LCD_POINT_RIGHT		240
#define LCD_POINT_TOP		1
#define LCD_POINT_BOTTOM	320
#elif defined CONFIG_PXA_U810
#define TS_POINT_LEFT		1050
#define TS_POINT_RIGHT		3150
#define TS_POINT_TOP		3550
#define TS_POINT_BOTTOM		960

#define LCD_POINT_LEFT		1
#define LCD_POINT_RIGHT		320
#define LCD_POINT_TOP		1
#define LCD_POINT_BOTTOM	480
#endif

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

struct tslib_dejitter *ts_djt;

struct pm860x_touch {
	struct input_dev *idev;
	struct i2c_client *i2c;
	struct pm860x_chip *chip;
	int irq;
        int res_x;              /* resistor of Xplate */
};

struct pm860x_ts_eval {
	u16		X_pre;
	u16		Y_pre;
	u32		error_on_measure;
};

struct pm860x_ts_data {
	//spinlock_t			ts_lock;
	struct task_struct		*thread;
	//int				suspended;
	int				pen_state;
	int				key_state;
	//int				use_count;

	//struct task_struct		*thread;
 	wait_queue_head_t		ts_wait_queue;
 	//struct completion		thread_init;
 	//struct completion		thread_exit;
 	#ifdef CONFIG_DVFM
	int dvfm_dev_idx;
	struct dvfm_lock dvfm_lock;
	struct timer_list dvfm_timer;
	struct work_struct dvfm_work;
	unsigned int dvfm_timeout_val;
         #endif

};

enum {
	TSI_PEN_UNKNOW = 0,
	TSI_PEN_DOWN = 1,
	TSI_PEN_UP = 2,
	TSI_KEY_DOWN = 3,
	TSI_KEY_UP =4
};

#define TS_KEY_REPORT

#ifdef TS_KEY_REPORT
static int ts_key_left = 139;	
static int ts_key_middle = 102;
static int ts_key_right = 158;	
static int is_recovery_mode = 0;    
static struct input_dev *keypad_input_dev = NULL;    
extern struct input_dev *get_keyinput_dev(void);
extern char * envp_init[];
#endif

static int ts2lcd_param[6];    
static int ts_debug = 0;    

//static struct work_struct ts_wq;    
static struct pm860x_touch *pm860x_touch_data;
static struct pm860x_ts_data *pm860x_data;
static int calparm[7]={1, 0, 0, 0, 1, 0, 1};

//static void ts_handler_do_work(struct work_struct *work);
static int ts_handler_thread(void *d);

#ifdef CONFIG_DVFM
static void set_dvfm_constraint(struct pm860x_ts_data *p)
{
        //printk("set_dvfm_constraint\n");
	spin_lock_irqsave(&p->dvfm_lock.lock, p->dvfm_lock.flags);
	dvfm_disable_op_name("apps_idle", p->dvfm_dev_idx);
	dvfm_disable_op_name("apps_sleep", p->dvfm_dev_idx);
	dvfm_disable_op_name("sys_sleep", p->dvfm_dev_idx);
	dvfm_disable_op_name("156MHz", p->dvfm_dev_idx);
	dvfm_disable_op_name("312MHz", p->dvfm_dev_idx);
	spin_unlock_irqrestore(&p->dvfm_lock.lock, p->dvfm_lock.flags);
}

static void unset_dvfm_constraint(struct pm860x_ts_data *p)
{
        //printk("unset_dvfm_constraint\n");
	spin_lock_irqsave(&p->dvfm_lock.lock, p->dvfm_lock.flags);
	dvfm_enable_op_name("apps_idle", p->dvfm_dev_idx);
	dvfm_enable_op_name("apps_sleep", p->dvfm_dev_idx);
	dvfm_enable_op_name("sys_sleep", p->dvfm_dev_idx);
	dvfm_enable_op_name("156MHz", p->dvfm_dev_idx);
	dvfm_enable_op_name("312MHz", p->dvfm_dev_idx);
	spin_unlock_irqrestore(&p->dvfm_lock.lock, p->dvfm_lock.flags);
}

static void dvfm_work_handler(struct work_struct *work)
{
	struct pm860x_ts_data *p_work =  
		container_of(work, struct pm860x_ts_data, dvfm_work);
	unset_dvfm_constraint(p_work);
}

static void dvfm_timer_handler(unsigned long data)
{
	struct pm860x_ts_data *p = (struct pm860x_ts_data *)data;
	schedule_work(&p->dvfm_work);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
static void dvfm_timer_handler(unsigned long data) {}
#endif

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

static int pm860x_filter(struct tslib_dejitter *djt, struct ts_sample *samp)
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

#ifdef CONFIG_PXA_U802
const char ts_keys_size[] = "0x01:139:40:340:80:40:0x01:102:120:340:80:40:0x01:158:200:340:80:40";
#elif defined CONFIG_PXA_U810
const char ts_keys_size[] = "0x01:139:50:540:100:80:0x01:102:160:540:100:80:0x01:158:270:540:100:80";
#endif

struct attribute ts_key_report_attr = {
        .name = "virtualkeys.88pm860x-touch",
        .mode = S_IRWXUGO,
};
 
static struct attribute *def_attrs[] = {
        &ts_key_report_attr,
        NULL,
};
 
void ts_key_report_release(struct kobject *kobject)
{
        return;
}
 
ssize_t ts_key_report_show(struct kobject *kobject, struct attribute *attr,char *buf)
{
        sprintf(buf,"%s\n",ts_keys_size);
        return strlen(ts_keys_size)+2;
}
 
ssize_t ts_key_report_store(struct kobject *kobject,struct attribute *attr,const char *buf, size_t count)
{
        return count;
}
 
struct sysfs_ops ts_key_report_sysops =
{
        .show = ts_key_report_show,
        .store = ts_key_report_store,
};
 
struct kobj_type ktype = 
{
        .release = ts_key_report_release,
        .sysfs_ops=&ts_key_report_sysops,
        .default_attrs=def_attrs,
};
 
struct kobject kobj;
static void ts_key_report_init(void)
{
        int ret = 0;
        ret = kobject_init_and_add(&kobj,&ktype,NULL,"board_properties");
        if(ret)
                printk(KERN_ERR "ts_key_report_init: Unable to init and add the kobject\n");
}
//#endif

#ifdef CONFIG_PROC_FS
#define	PM860X_TOUCH_PROC_FILE	"driver/pm860x_touch"
static struct proc_dir_entry *pm860x_touch_proc_file;

static ssize_t pm860x_touch_proc_read(struct file *filp,
				    char *buffer, size_t length,
				    loff_t * offset)
{
	int i;
	for(i = 0; i < 7; i++)
	{
		printk(KERN_INFO "pm860x_touch calparam %d = %d\n", i, calparm[i]);
	}
	return 0;
}

static ssize_t pm860x_touch_proc_write(struct file *filp,
				     const char *buff, size_t len,
				     loff_t * off)
{
	char messages[256], calbuf[7][12];

	int i, j, k, pam, mx, my, xr, xl, yt, yb;    

	if (copy_from_user(messages, buff, len))
		return -EFAULT;
    //printk(KERN_INFO "%s\n", messages);
    
    /*
      :d 1 print ts point original && after calibate,
      :n 0 default not print. 
    */
	if ('d' == messages[0]) {
		ts_debug = 1;
		return len;
	} else if ('n' == messages[0]) {
		ts_debug = 0;
		return len;
	}
	
	for(j = 0; j < 7; j++)
	{
	    for(k = 0; k < 12; k++)
	    {
           calbuf[j][k] = 0;
	    }
	}
	j = 6;
	k = 0;
	for(i = len - 2; i >= 0; i--)
	{
	    if(' ' != messages[i] && j >= 0)
	    {
	        calbuf[j][k] = messages[i];
			k++;
	    }
		else if(k < 12)
		{
		    j--;
		    k = 0;
		}
		else
		{
			break;
		}
	}
	
	for(j = 0; j < 7; j++)
	{
	    calparm[j] = 0;
	    for(k = 0; k < 12; k++)
	    {
	        
	        if('0' <= calbuf[j][k] && calbuf[j][k] <= '9')
	        {
	            pam = 1;
	            for(i = 0; i < k; i++)
	            {
	               pam = pam * 10;
	            }
	            calparm[j] = (calbuf[j][k] - '0') * pam+ calparm[j];
	        }
		    else if('-' == calbuf[j][k])
		    {
		        calparm[j] = -calparm[j];
		        break;
		    }
			else
			{
			    break;
			}
	    }
		//printk(KERN_INFO "pm860x_touch calparam %d = %d\n", j, calparm[j]);
	}

	if(0 == calparm[6])
	{
	    calparm[6] = 1;
	}
	

    mx = my = 50;
	xl = (mx * calparm[0] + my * calparm[1] + calparm[2]) / calparm[6];
	yt = (mx * calparm[3] + my * calparm[4] + calparm[5]) / calparm[6];
	mx = LCD_POINT_RIGHT - 50;
	my = LCD_POINT_BOTTOM - 50;
	xr = (mx * calparm[0] + my * calparm[1] + calparm[2]) / calparm[6];
	yb = (mx * calparm[3] + my * calparm[4] + calparm[5]) / calparm[6];
	if(abs(50 - xl) > 70 || abs(LCD_POINT_RIGHT - 50 - xr) > 70 
		|| abs(50 - yt) > 70 || abs(LCD_POINT_BOTTOM - 50 - yb) > 70)
	{
	    calparm[0] = 1;
		calparm[1] = 0;
		calparm[2] = 0;
		calparm[3] = 0;
		calparm[4] = 1;
		calparm[5] = 0;
	    calparm[6] = 1;
		printk(KERN_INFO "cal parameter is invalid! load default parameter...\n");
	}
	
	return len;
}

static struct file_operations pm860x_touch_proc_ops = {
	.read = pm860x_touch_proc_read,
	.write = pm860x_touch_proc_write,
};

static void create_pm860x_touch_proc_file(void)
{
	pm860x_touch_proc_file =
	    create_proc_entry(PM860X_TOUCH_PROC_FILE, 0777, NULL);
	if (pm860x_touch_proc_file) {
		pm860x_touch_proc_file->proc_fops = &pm860x_touch_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_pm860x_touch_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(PM860X_TOUCH_PROC_FILE, &proc_root);
}
#endif

void calibrate(int *x, int *y)    
{
    int mx, my;
	//ts point to lcd point
	mx = *x;
	my = *y;
	mx = (ts2lcd_param[0] * mx + ts2lcd_param[4]) / ts2lcd_param[2];
	my = (ts2lcd_param[1] * my + ts2lcd_param[5]) / ts2lcd_param[3];
	//calibrate point
	*x = (mx * calparm[0] + my * calparm[1] + calparm[2]) / calparm[6];
	*y = (mx * calparm[3] + my * calparm[4] + calparm[5]) / calparm[6];
}


/*************************************************************************
* return ==-2  - pen up decided
* return ==-1  - bad measurement; restored from previous; not for report
* return == 0  - bad measurement; restored from previous
* return == 1  - good measure for reporting
* return == 7  - bad measure not restored but approximated
*/    
static int pm860x_evaluate_point(struct pm860x_ts_eval* buf, int* pX, int* pY, u32 cntr)
{
    int fRet = 1;

  if(buf != NULL) /* NULL: No evaluation required The XY data not modified (transparent) */
  {
    int dif_X, dif_Y;
	int xy_range_valid;
	int tmpX, tmpY;

	dif_X = dif_Y = 0;

	if(cntr == 0) {
		/* Init on the first measurement evaluation */
		buf->X_pre = INVALID_XY;
		buf->Y_pre = INVALID_XY;
		buf->error_on_measure = 0;
	}

	xy_range_valid = (*pX > VALID_MIN_XY) && (*pX < VALID_MAX_XY) &&
	                 (*pY > VALID_MIN_XY) && (*pY < VALID_MAX_XY);

	if(!xy_range_valid) {
        fRet = -1;  /* the measuremet has failed, retun -1 or 0 */
    }else {
		 /* Still no "previous" data; cannot evaluate, just update "previous" */
		if((buf->X_pre == INVALID_XY) || (buf->Y_pre == INVALID_XY))
			cntr = 0;

		if(cntr != 0){
 			dif_X = abs( *pX - buf->X_pre );
			dif_Y = abs( *pY - buf->Y_pre );
			fRet = (dif_X < VALID_MAX_DIF) && (dif_Y < VALID_MAX_DIF);
			if(fRet==0) {
				if (cntr == 1) { /* prev_0 was not good, drop it, report current_1 */
					buf->X_pre = *pX;
					buf->Y_pre = *pY;
					return fRet;
				}else if((buf->error_on_measure+1) == cntr) {
					/* 2 consequtive errors. Obviously real "jump" => re-sync */
					buf->X_pre = *pX;
					buf->Y_pre = *pY;
					buf->error_on_measure = 0;
					fRet = 1;
				}
			}
		}
	}
	/*------------------------------------------------------------------
	* If pen goes up at time of the measurement the last value is incorrect.
	* To exclude it lets report about PREVIOUS but not current.
	* The last will be dropped out "automatically".
	* If cntr==0 we still haven't the previous so return the current.
	*/
    if (fRet > 0) {
		if(cntr == 0){
			buf->X_pre = *pX;
			buf->Y_pre = *pY;
			fRet = 0;
		}else{
			tmpX = buf->X_pre;
			tmpY = buf->Y_pre;
			buf->X_pre = *pX;
			buf->Y_pre = *pY;
			*pX = tmpX;
			*pY = tmpY;
		}
	}else {
		if(cntr != 0){
			//printk(KERN_DEBUG "--->TSI skip BAD point{pre_d_new}  X{%x_%x_%x}  Y{%x_%x_%x} cntr_%u\n",
			//								 buf->X_pre, dif_X, *pX,  buf->Y_pre, dif_Y, *pY, cntr);
			*pX = buf->X_pre;
			*pY = buf->Y_pre;
			buf->error_on_measure = cntr;
		}
	}

  }/*if buf==NULL: No evaluation required */
  
	return fRet;
}

#ifdef TS_KEY_REPORT
static int ts_key_code_specify(void)
{
    unsigned int i;
    //extern char * envp_init[];
    
    for (i = 0; envp_init[i]; i++) 
    {
        //printk(KERN_NOTICE "envp_init[%d]: \"%s\"\n", i, envp_init[i]);
        if(!strncmp(envp_init[i],"bootrd=",7))
        {
            printk(KERN_NOTICE "bootrd=\n");
            if(!strncmp(envp_init[i]+7,"/dev/mtdblock6",14))
            {
            	ts_key_left = 0x160;    //KEY_OK 0x160
            	ts_key_middle = 102;    //KEY_home
            	ts_key_right = 158;     //KEY_back
            	is_recovery_mode = 1;    //only for recovery key
                printk(KERN_NOTICE "ts key code:recovery mode\n");
            }

//#ifdef CONFIG_PXA_U802
//            else// if(!strncmp(envp_init[i]+7,"/dev/mtdblock1",14))
//            {
//            	ts_key_left = 102;    //KEY_home
//            	ts_key_middle = 139;    //KEY_menu
//            	ts_key_right = 158;     //KEY_back
//                printk(KERN_NOTICE "ts key code:normal mode\n");
//            }
//#elif defined CONFIG_PXA_U810
            else// if(!strncmp(envp_init[i]+7,"/dev/mtdblock1",14))
            {
//            	ts_key_left = 139;    //KEY_menu
//            	ts_key_middle = 102;    //KEY_home
 //           	ts_key_right = 158;     //KEY_back
            	is_recovery_mode = 0;   
                printk(KERN_NOTICE "ts key code:normal mode\n");
            }
//#endif

            return 0;
        }
    }

//    ts_key_left = 139;    //KEY_menu
//    ts_key_middle = 102;    //KEY_home
//    ts_key_right = 158;     //KEY_back
    is_recovery_mode = 0;    
    printk(KERN_NOTICE "ts key code:normal mode\n");
    return 0;
}

#ifdef CONFIG_PXA_U802
static int ts_key_report(int x, int y, int pen_down)
{
//    struct pm860x_touch *touch = pm860x_touch_data;
	static int keycode = 0;
	
	if(keypad_input_dev == NULL)   
		keypad_input_dev = get_keyinput_dev();
	
	if(pen_down){
		if(y > 320)
		{
		    if(x < 80)    
		    {
                keycode = ts_key_left;   
		    }
			else if(x < 160)   
			{
			    keycode = ts_key_middle;    
		    }
			else
			{
			    keycode = ts_key_right;
		    }
			input_report_key(keypad_input_dev, keycode, pen_down);
			input_sync(keypad_input_dev);
			//input_report_key(touch->idev, keycode, pen_down);
			return 1;
		}
		else
			return 0;
	}
	else
	{
		if(keycode)
		{
			input_report_key(keypad_input_dev, keycode, pen_down);
			input_sync(keypad_input_dev);
			//input_report_key(touch->idev, keycode, pen_down);
			keycode = 0;
			return 1;
		}
		else
			return 0;
	}

	return 0;
}
#elif defined CONFIG_PXA_U810
static int ts_key_report(int x, int y, int pen_down)
{
//    struct pm860x_touch *touch = pm860x_touch_data;
	static int keycode = 0;

	if(keypad_input_dev == NULL)    
		keypad_input_dev = get_keyinput_dev();

	if(pen_down){
		if(y > 490)
		{
		    if(x < 106)   
		    {
                keycode = ts_key_left;    
		    }
			else if(x < 212)   
			{
			    keycode = ts_key_middle;    
		    }
			else
			{
			    keycode = ts_key_right;
		    }
			input_report_key(keypad_input_dev, keycode, pen_down);
			input_sync(keypad_input_dev);
			//input_report_key(touch->idev, keycode, pen_down);
			return 1;
		}
		else
			return 0;
	}
	else
	{
		if(keycode)
		{
			
			input_report_key(keypad_input_dev, keycode, pen_down);
			input_sync(keypad_input_dev);
			//input_report_key(touch->idev, keycode, pen_down);
			keycode = 0;
			return 1;
		}
		else
			return 0;
	}
        
	return 0;
}
#endif
#endif

/*    
static void pm860x_enable_pen_down_irq(int on)
{
    struct pm860x_touch *touch = pm860x_touch_data;
	int x;
        
    x = pm860x_reg_read(touch->i2c, 0x08);
	if(on)
	{
		pm860x_reg_write(touch->i2c, 0x08, x | 0x2);
	}
	else
	{
		pm860x_reg_write(touch->i2c, 0x08, x & 0xfd);
	}
}
*/
//static void ts_handler_do_work(struct work_struct *work)

static int ts_handler_thread(void *d)
{
	struct pm860x_touch *touch = pm860x_touch_data;
	struct pm860x_chip *chip = touch->chip;
	unsigned char buf[MEAS_LEN];
	int x, y, pen_down;
	int ret;

	u32 cntr = 0;
	struct pm860x_ts_eval     eval_buf = {0};
	struct pm860x_ts_eval  *p_eval_buf = &eval_buf; /* if NULL no evaluation applied */
	struct task_struct *tsk = current;
	struct sched_param param = { .sched_priority = 2 };
	struct pm860x_ts_data *ts_data = d;
	struct ts_sample samp;
	static int pre_x, pre_y;
	
	DEFINE_WAIT(ts_wait);
	/* set up thread context */

	ts_data->thread = current;
	daemonize("ts_handler_thread");

	/* improve micco_ts_thread priority */
	sched_setscheduler(tsk, SCHED_FIFO, &param);
	
	//for(;;)
	while(1)
    {
		if (!ts_data->thread)
			break;
		
		if (TSI_PEN_UP == ts_data->pen_state) {
			prepare_to_wait(&ts_data->ts_wait_queue, &ts_wait, TASK_INTERRUPTIBLE);

			if (TSI_PEN_UP == ts_data->pen_state)
				schedule();

			finish_wait(&ts_data->ts_wait_queue, &ts_wait);
		}
		try_to_freeze();

		ret = pm860x_bulk_read(touch->i2c, MEAS_TSIX_1, MEAS_LEN, buf);
		if (ret < 0)
			continue;//goto out;

		pen_down = buf[1] & (1 << 6);
		x = ((buf[0] & 0xFF) << 4) | (buf[1] & 0x0F);
		y = ((buf[2] & 0xFF) << 4) | (buf[3] & 0x0F);
		//z1 = ((buf[4] & 0xFF) << 4) | (buf[5] & 0x0F);
		//z2 = ((buf[6] & 0xFF) << 4) | (buf[7] & 0x0F);
		//if(ts_debug)
		//	printk(KERN_INFO "---pm860x_touch_interrupt [%d, %d].pen_down=%d \n", x, y, pen_down);
#if 0        
		//printk(KERN_INFO "---pm860x_touch_interrupt x=%d y=%d pen_down=%d \n", x, y, pen_down);

        if (pen_down) {
			//if(x > 1050)
				x = x - 1050;
			//else
			//{
			//    goto out;
			//}

			//if(y < 3450)
				y = 3550 - y;
			//else
			//{
			//    goto out;
			//}
	 	    //printk(KERN_INFO "--------pm860x_touch_interrupt x=%d y=%d pen_down=%d \n", x, y, pen_down);
        }

		calibrate(&x, &y);   
        if(ts_key_report(x, y, pen_down))
        {
            goto out;
        }
#endif        
		/*if (pen_down)
		{
			calibrate(&x, &y);
		}
		
		samp.pressure = pen_down;
		samp.x = x;
		samp.y = y;
		pm860x_filter(ts_djt, &samp);
		x = samp.x;
		y = samp.y;
		
		if(cntr == 0)
		{
			pre_x = x;
			pre_y = y;
		}
		if(abs(x - pre_x) <= 3 && abs(y - pre_y) <= 4)
		{
			x = pre_x;
			y = pre_y;
		}
		pre_x = x;
		pre_y = y;*/
		
        if (pen_down) {
			ts_data->pen_state = TSI_PEN_DOWN;
			//x = x - 1050;
			//y = 3550 - y;
			calibrate(&x, &y);
			if(cntr == 0)
			{
				pre_x = x;
				pre_y = y;
			}
			
			if(ts_debug)
				printk(KERN_INFO "pen down at [%d, %d].\n", x, y);
        
			if(0 < pm860x_evaluate_point(p_eval_buf, &x, &y, cntr)) {
#ifdef TS_KEY_REPORT
				if(1 == is_recovery_mode){
				if(cntr < 2 && ts_data->key_state == TSI_KEY_UP)
				if(ts_key_report(x, y, pen_down))
				{
					ts_data->key_state = TSI_KEY_DOWN;
					continue;//break;
				}
				}
#endif
				samp.pressure = pen_down;
				samp.x = x;
				samp.y = y;
				pm860x_filter(ts_djt, &samp);
				x = samp.x;
				y = samp.y;/**/
				if(abs(x - pre_x) <= 3 && abs(y - pre_y) <= 4)
				{
					x = pre_x;
					y = pre_y;
				}
				pre_x = x;
				pre_y = y;
				if(ts_debug)
					printk(KERN_INFO "--------[%d, %d].\n", x, y);
				
				input_report_abs(touch->idev, ABS_X, x);
				input_report_abs(touch->idev, ABS_Y, y);
				input_report_abs(touch->idev, ABS_PRESSURE, 255);//input_report_abs(touch->idev, ABS_PRESSURE, rt);
				input_report_key(touch->idev, BTN_TOUCH, 1);
				input_sync(touch->idev);
				dev_dbg(chip->dev, "pen down at [%d, %d].\n", x, y);
			}
			else if(x <= 0 || y <= 0)  
			{
					continue;
			}

			msleep(5);
			cntr++;
		} else {
#ifdef TS_KEY_REPORT
			if(1 == is_recovery_mode){
			if(ts_data->key_state == TSI_KEY_DOWN)
			if(ts_key_report(x, y, pen_down))
			{
				ts_data->key_state = TSI_KEY_UP;
				continue;//break;
			}
			}
#endif
			input_report_abs(touch->idev, ABS_PRESSURE, 0);
			input_report_key(touch->idev, BTN_TOUCH, 0);
			input_sync(touch->idev);
			//msleep(10);
			dev_dbg(chip->dev, "pen release\n");
			cntr = 0;
			ts_data->pen_state = TSI_PEN_UP;
			//pm860x_enable_pen_down_irq(1);//goto out;
		}
		//input_sync(touch->idev);
    }

//out:
	//pm860x_enable_pen_down_irq(1);
	return 0;
}

static irqreturn_t pm860x_touch_handler(int irq, void *data)
{
	//pm860x_enable_pen_down_irq(0);
	//schedule_work(&ts_wq);
        #ifdef CONFIG_DVFM
	if (mod_timer(&pm860x_data->dvfm_timer, jiffies + pm860x_data->dvfm_timeout_val)) {
		/* timer is already active */ 
	} else {
		set_dvfm_constraint(pm860x_data);
	}
        #endif

	if (TSI_PEN_UP == pm860x_data->pen_state)
		wake_up_interruptible(&pm860x_data->ts_wait_queue);
        return IRQ_HANDLED;
}

#if 0
static irqreturn_t pm860x_touch_handler(int irq, void *data)
{
        struct pm860x_touch *touch = data;
	struct pm860x_chip *chip = touch->chip;
	unsigned char buf[MEAS_LEN];
	int x, y, pen_down;
	int z1, z2, rt = 0;
	int ret;

	ret = pm860x_bulk_read(touch->i2c, MEAS_TSIX_1, MEAS_LEN, buf);
	if (ret < 0)
		goto out;

	pen_down = buf[1] & (1 << 6);
	x = ((buf[0] & 0xFF) << 4) | (buf[1] & 0x0F);
	y = ((buf[2] & 0xFF) << 4) | (buf[3] & 0x0F);
        z1 = ((buf[4] & 0xFF) << 4) | (buf[5] & 0x0F);
        z2 = ((buf[6] & 0xFF) << 4) | (buf[7] & 0x0F);

    //printk(KERN_INFO "pm860x_touch_interrupt x=%d y=%dz1=%d z2=%d\n", x, y, z1, z2);
        if (pen_down) {
                if ((x != 0) && (z1 != 0) && (touch->res_x != 0)) {
                        rt = z2 / z1 - 1;
			rt = (rt * touch->res_x * x) >> ACCURATE_BIT;
			dev_dbg(chip->dev, "z1:%d, z2:%d, rt:%d\n",
				z1, z2, rt);
		}
		input_report_abs(touch->idev, ABS_X, x);
		input_report_abs(touch->idev, ABS_Y, y);
		input_report_abs(touch->idev, ABS_PRESSURE, rt);
		input_report_key(touch->idev, BTN_TOUCH, 1);
		dev_dbg(chip->dev, "pen down at [%d, %d].\n", x, y);
	} else {
		input_report_abs(touch->idev, ABS_PRESSURE, 0);
		input_report_key(touch->idev, BTN_TOUCH, 0);
		dev_dbg(chip->dev, "pen release\n");
	}
	input_sync(touch->idev);

out:
        return IRQ_HANDLED;
}
#endif

static int pm860x_touch_open(struct input_dev *dev)
{
	struct pm860x_touch *touch = input_get_drvdata(dev);
	int data, ret;

	data = MEAS_PD_EN | MEAS_TSIX_EN | MEAS_TSIY_EN;
		//| MEAS_TSIZ1_EN | MEAS_TSIZ2_EN;
	ret = pm860x_set_bits(touch->i2c, MEAS_EN3, data, data);
	if (ret < 0)
		goto out;
	return 0;
out:
	return ret;
}

static void pm860x_touch_close(struct input_dev *dev)
{
	struct pm860x_touch *touch = input_get_drvdata(dev);
	int data;

	data = MEAS_PD_EN | MEAS_TSIX_EN | MEAS_TSIY_EN
                | MEAS_TSIZ1_EN | MEAS_TSIZ2_EN;
        pm860x_set_bits(touch->i2c, MEAS_EN3, data, 0);
}

static void pm860x_touch_sleep_early_suspend(struct early_suspend *h)
{
        struct pm860x_touch *touch = pm860x_touch_data;
        int ret, i = 0, data;
    data = MEAS_PD_EN | MEAS_TSIX_EN | MEAS_TSIY_EN
                | MEAS_TSIZ1_EN | MEAS_TSIZ2_EN;
        
      sleep_retry:
        ret = pm860x_set_bits(touch->i2c, MEAS_EN3, data, 0);;
        if (ret < 0) {
                if (i < 50) {
                        msleep(5);
                        i++;
                        printk(KERN_WARNING
                               "88pm860x_touch can't enter sleep,retry %d\n",
                               i);
                        goto sleep_retry;
                }
                printk(KERN_WARNING "88pm860x_touch can't enter sleep\n");
                return;
        } else {
                printk(KERN_INFO "88pm860x_touch enter sleep mode.\n");
        }
}

static void pm860x_touch_normal_late_resume(struct early_suspend *h)
{
    struct pm860x_touch *touch = pm860x_touch_data;
        int ret, i = 0, data;

    data = MEAS_PD_EN | MEAS_TSIX_EN | MEAS_TSIY_EN;
                //| MEAS_TSIZ1_EN | MEAS_TSIZ2_EN;
      reset_retry:
        ret = pm860x_set_bits(touch->i2c, MEAS_EN3, data, data);
        if (ret < 0) {
                if (i < 50) {
                        msleep(5);
                        i++;
                        printk(KERN_WARNING
                               "88pm860x_touch reset failed,retry %d\n", i);
                        goto reset_retry;
                }
                printk(KERN_WARNING "88pm860x_touch reset failed\n");
                return;
        } else {
                printk(KERN_INFO "88pm860x_touch reset successful.\n");
        }
}

static struct early_suspend pm860x_touch_early_suspend_desc = {
        .level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
        .suspend = pm860x_touch_sleep_early_suspend,
        .resume = pm860x_touch_normal_late_resume,
};

static int __devinit pm860x_touch_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_platform_data *pm860x_pdata =		\
				pdev->dev.parent->platform_data;
	struct pm860x_touch_pdata *pdata = NULL;
	struct pm860x_touch *touch;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		return -EINVAL;
	}

	if (!pm860x_pdata) {
		dev_err(&pdev->dev, "platform data is missing\n");
		return -EINVAL;
	}

	pdata = pm860x_pdata->touch;
	if (!pdata) {
		dev_err(&pdev->dev, "touchscreen data is missing\n");
		return -EINVAL;
	}

	touch = kzalloc(sizeof(struct pm860x_touch), GFP_KERNEL);
	if (touch == NULL)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, touch);

	touch->idev = input_allocate_device();
	if (touch->idev == NULL) {
		dev_err(&pdev->dev, "Failed to allocate input device!\n");
		ret = -ENOMEM;
		goto out;
	}

	touch->idev->name = "88pm860x-touch";
	touch->idev->phys = "88pm860x/input0";
	touch->idev->id.bustype = BUS_I2C;
	touch->idev->dev.parent = &pdev->dev;
	touch->idev->open = pm860x_touch_open;
	touch->idev->close = pm860x_touch_close;
	touch->chip = chip;
	touch->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	touch->irq = irq + chip->irq_base;
	touch->res_x = pdata->res_x;
	input_set_drvdata(touch->idev, touch);

	pm860x_data = kzalloc(sizeof(struct pm860x_ts_data), GFP_KERNEL);
	pm860x_data->pen_state = TSI_PEN_UP;
#ifdef TS_KEY_REPORT
	pm860x_data->key_state = TSI_KEY_UP;
#endif
	init_waitqueue_head(&pm860x_data->ts_wait_queue);
         #ifdef CONFIG_DVFM
	pm860x_data->dvfm_dev_idx = -1;
	pm860x_data->dvfm_lock.lock = SPIN_LOCK_UNLOCKED;
	if (!dvfm_register("88pm860x-ts", &pm860x_data->dvfm_dev_idx)) {
		INIT_WORK(&pm860x_data->dvfm_work, dvfm_work_handler);
		init_timer(&pm860x_data->dvfm_timer);
		pm860x_data->dvfm_timer.function = dvfm_timer_handler;
		pm860x_data->dvfm_timer.data = (unsigned long)pm860x_data;
		pm860x_data->dvfm_timeout_val = HZ * 2;
	}
         #endif

	ret = request_threaded_irq(touch->irq, NULL, pm860x_touch_handler,
				   IRQF_ONESHOT, "touch", touch);
	if (ret < 0)
		goto out_irq;

	__set_bit(EV_ABS, touch->idev->evbit);
	__set_bit(ABS_X, touch->idev->absbit);
	__set_bit(ABS_Y, touch->idev->absbit);
	__set_bit(ABS_PRESSURE, touch->idev->absbit);
	__set_bit(EV_SYN, touch->idev->evbit);
	__set_bit(EV_KEY, touch->idev->evbit);
	__set_bit(BTN_TOUCH, touch->idev->keybit);


#ifdef TS_KEY_REPORT
	__set_bit(102, touch->idev->keybit);
	__set_bit(139, touch->idev->keybit);
	__set_bit(158, touch->idev->keybit);
	__set_bit(352, touch->idev->keybit);
#endif
	//__set_bit(EV_REP, touch->idev->evbit);

#ifdef CONFIG_PXA_U802
	input_set_abs_params(touch->idev, ABS_X, 0, 240, 0, 0);
	input_set_abs_params(touch->idev, ABS_Y, 0, 320, 0, 0);
#elif defined CONFIG_PXA_U810
	input_set_abs_params(touch->idev, ABS_X, 0, 320, 0, 0);
	input_set_abs_params(touch->idev, ABS_Y, 0, 480, 0, 0);
#endif
	//input_set_abs_params(touch->idev, ABS_X, 0, 2100, 0, 0);
	//input_set_abs_params(touch->idev, ABS_Y, 0, 2590, 0, 0);
	input_set_abs_params(touch->idev, ABS_PRESSURE, 0, 1 << ACCURATE_BIT,
                                0, 0);

    pm860x_touch_data = touch;
	//INIT_WORK(&ts_wq, ts_handler_do_work);
#ifdef TS_KEY_REPORT
	ts_key_code_specify();	
#endif
	ts_key_report_init();    

	register_early_suspend(&pm860x_touch_early_suspend_desc);
#if 0
	input_set_abs_params(touch->idev, ABS_X, 0, 1 << ACCURATE_BIT, 0, 0);
	input_set_abs_params(touch->idev, ABS_Y, 0, 1 << ACCURATE_BIT, 0, 0);
	input_set_abs_params(touch->idev, ABS_PRESSURE, 0, 1 << ACCURATE_BIT,
                                0, 0);
#endif

#ifdef	CONFIG_PROC_FS 
	create_pm860x_touch_proc_file();
#endif

	ts_djt = kzalloc(sizeof(struct tslib_dejitter), GFP_KERNEL);
	if (ts_djt == NULL)
		return -ENOMEM;

	ts_djt->head = 0;
	ts_djt->nr =0;
	ts_djt->delta = 10;
	ts_djt->delta = sqr (ts_djt->delta);
	
	ts2lcd_param[0] = LCD_POINT_RIGHT - LCD_POINT_LEFT;
	ts2lcd_param[1] = LCD_POINT_TOP - LCD_POINT_BOTTOM;
	ts2lcd_param[2] = TS_POINT_RIGHT - TS_POINT_LEFT;
	ts2lcd_param[3] = TS_POINT_TOP - TS_POINT_BOTTOM;
	ts2lcd_param[4] = LCD_POINT_LEFT * ts2lcd_param[2] - (LCD_POINT_RIGHT - LCD_POINT_LEFT) * TS_POINT_LEFT;
	ts2lcd_param[5] = LCD_POINT_BOTTOM * ts2lcd_param[3] - (LCD_POINT_TOP - LCD_POINT_BOTTOM) * TS_POINT_BOTTOM;

	kernel_thread(ts_handler_thread, pm860x_data, 0);

	ret = input_register_device(touch->idev);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to register touch!\n");
		goto out_rg;
	}

	platform_set_drvdata(pdev, touch);
	return 0;
out_rg:
	free_irq(touch->irq, touch);
out_irq:
	input_free_device(touch->idev);
out:
	kfree(touch);
	return ret;
}

static int __devexit pm860x_touch_remove(struct platform_device *pdev)
{
	struct pm860x_touch *touch = platform_get_drvdata(pdev);
     #ifdef CONFIG_DVFM
	if( pm860x_data->dvfm_dev_idx >= 0 ) {
		del_timer(&pm860x_data->dvfm_timer);
		dvfm_unregister("88pm860x-ts", &pm860x_data->dvfm_dev_idx);
		pm860x_data->dvfm_dev_idx = -1;
	}
       #endif

	input_unregister_device(touch->idev);
	free_irq(touch->irq, touch);
	platform_set_drvdata(pdev, NULL);
	kfree(touch);

#ifdef	CONFIG_PROC_FS 
	remove_pm860x_touch_proc_file();
#endif

	return 0;
}

static struct platform_driver pm860x_touch_driver = {
	.driver	= {
		.name	= "88pm860x-touch",
		.owner	= THIS_MODULE,
	},
	.probe	= pm860x_touch_probe,
	.remove	= __devexit_p(pm860x_touch_remove),
};

static int __init pm860x_touch_init(void)
{
	return platform_driver_register(&pm860x_touch_driver);
}
module_init(pm860x_touch_init);

static void __exit pm860x_touch_exit(void)
{
	platform_driver_unregister(&pm860x_touch_driver);
}
module_exit(pm860x_touch_exit);

MODULE_DESCRIPTION("Touchscreen driver for Marvell Semiconductor 88PM860x");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:88pm860x-touch");

