/*
 *  lis3lv02d.c - ST LIS3LV02DL accelerometer driver
 *
 *  Copyright (C) 2007-2008 Yan Burman
 *  Copyright (C) 2008 Eric Piel
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/freezer.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/sensor-input.h>
#include <asm/atomic.h>
#include "lis3lv02d.h"
#include <mach/gpio.h>
#include <mach/gsensor.h>
#include <linux/miscdevice.h>
#include "../i2c/busses/i2c_gpio_pxa.h"
#include <linux/mfd/88pm860x.h>
#include <mach/pxa910_pm.h>

#define LIS3LV02D_I2C_DEBUG    0
#if LIS3LV02D_I2C_DEBUG
#define LIS3LV02D_I2C_DBG(fmt, args...)    printk("Lis3lv02d_i2c : " fmt "\n", ## args)
#else if
#define LIS3LV02D_I2C_DBG(fmt, args...)    do {} while (0)
#endif

#define LIS3LV02D_ID 0x38
#define MISC_IOCTL_READ 1

static struct i2c_client *g_client;
static unsigned int platform;

static struct sensor_axis axis;
static struct sensor_axis lis3lv02d_axis_conversion[] = 
{
	{2, -1, -3},
	{-1, -2, 3},
};


static struct lis3lv02d_struct *lis3lv02d;

static int int_mod = 0; //0:real-time 1:interupt
static int interval = 1000;
static spinlock_t gsensor_i2c_lock = SPIN_LOCK_UNLOCKED;

struct delayed_work gsensor_work;
struct workqueue_struct *gsensor_wqueue;


static lis302dl_device_t lis302dl_device = 
{
	.name 	= "lis302dl_device",
	.rf_count	= ATOMIC_INIT(0),
};

int gsensor_i2c_write_byte_data(u8 command, u8 value)
{
//	char data;
	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&gsensor_i2c_lock, flags);

    err = gpio_request(18, "VIRTUAL-SCL");
    if(err) 
	{
    	LIS3LV02D_I2C_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }
	err = gpio_request(19, "VIRTUAL-SDA");
    if(err) 
	{
        LIS3LV02D_I2C_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3LV02D_ID |I2C_WR))
	{
		LIS3LV02D_I2C_DBG("ERROR %s %d", __FUNCTION__, 1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
		
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		LIS3LV02D_I2C_DBG("ERROR %s %d", __FUNCTION__, 2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(value))
	{
		LIS3LV02D_I2C_DBG("ERROR %s %d", __FUNCTION__, 2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
		
	COMMON_I2C_Stop();
	COMMON_I2C_mutx_up();

	gpio_free(18);
	gpio_free(19);	
	spin_unlock_irqrestore(&gsensor_i2c_lock, flags);

	return 0;
}

u8 gsensor_i2c_read_byte_data(u8 command)
{
	char data;
	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&gsensor_i2c_lock, flags);

    err = gpio_request(18, "VIRTUAL-SCL");
    if(err) 
	{
    	LIS3LV02D_I2C_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }
		
	err = gpio_request(19, "VIRTUAL-SDA");
    if(err) 
	{
        LIS3LV02D_I2C_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3LV02D_ID |I2C_WR))
	{
		LIS3LV02D_I2C_DBG("ERROR %s %d", __FUNCTION__, 1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		LIS3LV02D_I2C_DBG("ERROR %s %d", __FUNCTION__, 2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
	
	COMMON_I2C_Restart();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3LV02D_ID |I2C_RD))
	{
		LIS3LV02D_I2C_DBG("ERROR %s %d", __FUNCTION__, 1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}

	data = COMMON_I2C_Receive_Byte(1);

	COMMON_I2C_Stop();
	COMMON_I2C_mutx_up();
	
	gpio_free(18);
	gpio_free(19);	
    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);


	return data;
}

static int lis3lv02d_write(u8 reg, u8 val)
{
	int ret;
	int status;
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

//	if (g_client == NULL)	/* No global client pointer? */
//		return -1;
	ret = gsensor_i2c_write_byte_data(reg, val);

	if(ret == 0)
	{
		status = 0;
	}
	else
	{
		status = -EIO;
	}

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
	return status;
}

static int lis3lv02d_read(u8 reg, u8 *pval)
{
	int ret;
	int status;
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

//	if (g_client == NULL)	/* No global client pointer? */
//		return -1;
	ret = gsensor_i2c_read_byte_data(reg);
	if (ret >= 0) 
	{
		*pval = ret;
		status = 0;
	} 
	else 
	{
		status = -EIO;
	}

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
	return status;
}


static s16 lis3lv02d_read_16(int reg)
{
	u8 lo, hi;
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

	lis3lv02d_read(reg, &lo);
	lis3lv02d_read(reg + 1, &hi);
	LIS3LV02D_I2C_DBG("In %s, lo = 0x%x, hi = 0x%x", __FUNCTION__, lo, hi);
	/* In "12 bit right justified" mode, bit 6, bit 7, bit 8 = bit 5 */
	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
	return (s16) ((hi << 8) | lo);
}


static inline int lis3lv02d_i2c_get_axis(s8 axis, int hw_values[3])
{
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);
	
	if (axis > 0)
		return hw_values[axis - 1];
	else
		return -hw_values[-axis - 1];
}


void lis3lv02d_get_xyz(u8 *x, u8 *y, u8 *z)
{
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);
/*
	int ret = 0;
	u8 datax = 0, datay = 0, dataz = 0;
	

	ret = lis3lv02d_read(OUTX, &datax);
	if(ret != 0)
	{
		LIS3LV02D_I2C_DBG("lis3lv02d_read x fail");
		return;
	}

	ret = lis3lv02d_read(OUTY, &datay);
	if(ret != 0)
	{
		LIS3LV02D_I2C_DBG("lis3lv02d_read y fail");
		return;
	}

	ret = lis3lv02d_read(OUTZ, &dataz);
	if(ret != 0)
	{
		LIS3LV02D_I2C_DBG("lis3lv02d_read z fail");
		return;
	}

	*x = datax;
	*y = datay;
	*z = dataz;
	
	LIS3LV02D_I2C_DBG("In %s, x = 0x%x, y = 0x%x, z = 0x%x", __FUNCTION__, *x, *y, *z);
*/





	int position[3];

	position[0] = lis3lv02d_read_16(OUTX_L) / 8;;
	position[1] = lis3lv02d_read_16(OUTY_L) / 8;
	position[2] = lis3lv02d_read_16(OUTZ_L) / 8;

	*x = lis3lv02d_i2c_get_axis(axis.x, position);
	*y = lis3lv02d_i2c_get_axis(axis.y, position);
	*z = lis3lv02d_i2c_get_axis(axis.z, position);

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
}
EXPORT_SYMBOL(lis3lv02d_get_xyz);



static void lis302dl_wake_up_int(unsigned long data)
{
//	lis302_status_t *status_t;
	LIS3LV02D_I2C_DBG("\n\n\nStart %s", __FUNCTION__);
/*
	spin_lock(&lis302dl_device.list_lock);
	list_for_each_entry(status_t, &lis302dl_device.open_list, list)
	{
		atomic_set(&status_t->g_rdy, 1);
	}
	spin_unlock(&lis302dl_device.list_lock);

	wake_up_interruptible(&lis302dl_device.lis302_wait);
*/
	u8 x = 0, y = 0, z = 0;
	lis3lv02d_get_xyz(&x, &y, &z);

	LIS3LV02D_I2C_DBG("***interval = %d***", interval);
	if(!int_mod && (lis302dl_device.switch_on == SWITCH_ON))
		mod_timer(&lis302dl_device.timer, jiffies + msecs_to_jiffies(interval));
	LIS3LV02D_I2C_DBG("End %s",__FUNCTION__);
}



static void lis302_init(void)
{
	u8 ctrl_reg1;	// ctrl_reg2, ctrl_reg3, ff_wu_cfg, ff_wu_ths, ff_wu_duration;
	/*Add code for single/double click */
//	u8 click_cfg,click_ths_yx,click_ths_z;
	//u8 click_timelimit,click_latency,click_window;
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);
#ifdef CONFIG_lis302_400HZ 
	LIS3LV02D_I2C_DBG("config lis302 400HZ");
	//ctrl_reg1:1100 0111 The selected date rate will be 400hz
	lis3lv02d_read(Ctrl_Reg1, &ctrl_reg1);
	ctrl_reg1 |= DATE_RATE_400HZ;
	lis3lv02d_write(Ctrl_Reg1, ctrl_reg1);
#endif	

	// ctrl_reg1
//	lis3lv02d_read(Ctrl_Reg1, &ctrl_reg1);
	ctrl_reg1 = LIS302_POWER_UP;
	lis3lv02d_write(Ctrl_Reg1, ctrl_reg1);
//	lis302dl_device.switch_on = SWITCH_ON;


/*
	// ctrl_reg2
	lis3lv02d_read(Ctrl_Reg2, &ctrl_reg2);
	ctrl_reg2 |= HP_FF_WU1_ENABLE;
	lis3lv02d_write(Ctrl_Reg2, ctrl_reg2);

	// ctrl_reg3
//	lis3lv02d_read(Ctrl_Reg3, &ctrl_reg3);
//	LIS3LV02D_I2C_DBG("0x22 reg value = 0x%x", ctrl_reg3);
	ctrl_reg3 = (LIS302DL_FF_WU_1);	// FF_WU 
	//ctrl_reg3 |= ( 1 << 7 ); // FF_WU
	lis3lv02d_write(Ctrl_Reg3, ctrl_reg3);

	// FF_WU_CFG
	//lis302_read( FF_WU_CFG, & ff_wu_cfg );
//	ff_wu_cfg = (u8)XYZ_LIE;
	ff_wu_cfg = 0x2a;
	// OR combination, LIR
	lis3lv02d_write(FF_WU_CFG, ff_wu_cfg);

//	ff_wu_ths = (u8)(THRESH_OLD | THRESH_DCRM);
	ff_wu_ths = 0x03;
	lis3lv02d_write(FF_WU_THS, ff_wu_ths);

//	ff_wu_duration = (u8)DURATION;
	ff_wu_duration = 0;
	lis3lv02d_write(FF_WU_DURATION, ff_wu_duration);
	
	click_cfg = (u8)LIS302DL_SINGLE_CLICK;
	lis3lv02d_write(CLICK_CFG, click_cfg);

	click_ths_yx = (u8)LIS302DL_CLICK_THS_YX;
	lis3lv02d_write(CLICK_THS_Y_X, click_ths_yx);

	click_ths_z = (u8)LIS302DL_CLICK_THS_Z;
	lis3lv02d_write(CLICK_THS_Z, click_ths_z);	
*/
	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
}









static inline void lis3lv02d_power_off(void)
{
	/* disable X,Y,Z axis and power down */
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

	lis3lv02d_write(CTRL_REG1, 0x00);

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
}

static void lis3lv02d_power_on(void)
{
	/* full-scale choose as +/- 2g*/
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);
	
//	lis3lv02d_write(CTRL_REG1,
//		CTRL1_DR|CTRL1_PD|CTRL1_Xen|CTRL1_Yen|CTRL1_Zen);
	/* disable all interrupt */
//	lis3lv02d_write(FF_WU_CFG_1, 0);

	lis302_init();

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
}




static void lis3lv02d_report(struct input_dev *idev)
{
/*
	int x, y, z;
	u8 datax, datay, dataz;
	s8 status;
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

	struct sensor_input_dev *sensor;
	sensor = dev_get_drvdata(&idev->dev);

	lis3lv02d_read(STATUS_REG, &status);
	if((status & 0x7) == 0x7) 
	{
		lis3lv02d_get_xyz(&datax, &datay, &dataz);
		x = datax;
		y = datay;
		z = dataz;
		LIS3LV02D_I2C_DBG("In %s, x = %d, y = %d, z = %d", __FUNCTION__, x, y, z);

		input_report_abs(idev, ABS_X, x);
		input_report_abs(idev, ABS_Y, y);
		input_report_abs(idev, ABS_Z, z);
		input_sync(idev);

		sensor->x = x;
		sensor->y = y;
		sensor->z = z;
	} 
	else 
	{
		sensor->x = 0;
		sensor->y = 0;
		sensor->z = 0;
	}

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
*/



//#if 0
/* old */
	LIS3LV02D_I2C_DBG("\n\n\nStart %s", __FUNCTION__);
	int x, y, z;
    int x_new_version, y_new_version;
	int position[3];
	s8 status;

	struct sensor_input_dev* sensor;
	sensor = dev_get_drvdata(&idev->dev);

	lis3lv02d_read(STATUS_REG, &status);
	if ((status & 0x7) == 0x7) 
	{
		position[0] = lis3lv02d_read_16(OUTX_L) / 8;;
		position[1] = lis3lv02d_read_16(OUTY_L) / 8;
		position[2] = lis3lv02d_read_16(OUTZ_L) / 8;

//		LIS3LV02D_I2C_DBG("position[0] = %d, position[1] = %d, position[2] = %d", position[0], position[1], position[2]);

		x = lis3lv02d_i2c_get_axis(axis.x, position);
		y = lis3lv02d_i2c_get_axis(axis.y, position);
		z = lis3lv02d_i2c_get_axis(axis.z, position);
                
#ifdef CONFIG_PXA_U810
        int U810_hardware_version = pm860x_get_boardID();
        if (U810_hardware_version >= 2)
		{
			x_new_version = -y;
			y_new_version = x;
		}
		else
		{
			x_new_version = x;
			y_new_version = y;
		}
		input_report_abs(idev, ABS_X, x_new_version);
		input_report_abs(idev, ABS_Y, y_new_version);
		input_report_abs(idev, ABS_Z, (-z));

#elif defined CONFIG_PXA_U802
		input_report_abs(idev, ABS_X, (-x));
		input_report_abs(idev, ABS_Y, y);
		input_report_abs(idev, ABS_Z, z);

#elif defined CONFIG_PXA_U880
		input_report_abs(idev, ABS_X, (-y));
		input_report_abs(idev, ABS_Y, x);
		input_report_abs(idev, ABS_Z, (-z));
#endif

		input_sync(idev);

		sensor->x = x;
		sensor->y = y;
		sensor->z = z;
		//printk(KERN_DEBUG "lis3lv02d: x 0x%x, y 0x%x, z 0x%x\n", x, y, z);
	} 
	else 
	{
		sensor->x = 0;
		sensor->y = 0;
		sensor->z = 0;
	}
	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
//#endif
}


static void lis3lv02d_gsensor_work(struct work_struct *work)
{
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

	u8 x = 0, y = 0, z = 0;
	lis3lv02d_get_xyz(&x, &y, &z);

	LIS3LV02D_I2C_DBG("End %s\n\n\n", __FUNCTION__);
}

static irqreturn_t lis3lv02d_isr(int irq, void *dev_id)
{
//	wake_lock_timeout(&alsps_suspend_lock, 2 * HZ);
//	LIS3LV02D_I2C_DBG("*************** Come into isr ***************");

	queue_delayed_work(gsensor_wqueue, &gsensor_work, msecs_to_jiffies(10));

	return IRQ_HANDLED;
}

#ifdef	CONFIG_PROC_FS
#define	LIS331DL_PROC_FILE	"driver/lis3lv02d"
static struct proc_dir_entry *lis3lv02d_proc_file;
static int index;

static ssize_t lis3lv02d_proc_read(struct file *filp,
				   char *buffer, size_t length,
				   loff_t * offset)
{
	u8 reg_val;
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

	if ((index < 0) || (index > 0x3f))
		return 0;

	lis3lv02d_read(index, &reg_val);
	LIS3LV02D_I2C_DBG("register 0x%x: 0x%x", index, reg_val);
	return 0;
}

static ssize_t lis3lv02d_proc_write(struct file *filp,
				    const char *buff, size_t len,
				    loff_t * off)
{
	u8 reg_val;
	char messages[256], vol[256];
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages + 1, len - 1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} else {
		/* set the register value */
		reg_val = (int) simple_strtoul(messages, NULL, 16);
		lis3lv02d_write(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations lis3lv02d_proc_ops = 
{
	.read = lis3lv02d_proc_read,
	.write = lis3lv02d_proc_write,
};

static void create_lis3lv02d_proc_file(void)
{
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);
	
	lis3lv02d_proc_file =
	    create_proc_entry(LIS331DL_PROC_FILE, 0644, NULL);
	if (lis3lv02d_proc_file) {
		lis3lv02d_proc_file->proc_fops = &lis3lv02d_proc_ops;
	} else
		LIS3LV02D_I2C_DBG("proc file create failed!");
}


static void remove_lis3lv02d_proc_file(void)
{
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);
	
	remove_proc_entry(LIS331DL_PROC_FILE, NULL);
}
#endif

#ifdef	CONFIG_PM
static int lis3lv02d_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
//	printk("****** lis3lv02d_i2c_suspend ******\n");
	lis3lv02d_power_off();
	return 0;
}

static int lis3lv02d_i2c_resume(struct i2c_client *client)
{
//	printk("****** lis3lv02d_i2c_resume ******\n");
	lis3lv02d_power_on();
	return 0;
}
#else
#define	lis3lv02d_i2c_suspend		NULL
#define	lis3lv02d_i2c_resume		NULL
#endif

/*********************************************/
static int misc_read(int *data)
{
	LIS3LV02D_I2C_DBG("\n\n\nStart %s", __FUNCTION__);

	int x, y, z;
	int position[3];
	s8 status;

	lis3lv02d_read(STATUS_REG, &status);
	if ((status & 0x7) == 0x7) 
	{

		position[0] = lis3lv02d_read_16(OUTX_L) / 8;;
		position[1] = lis3lv02d_read_16(OUTY_L) / 8;
		position[2] = lis3lv02d_read_16(OUTZ_L) / 8;

		LIS3LV02D_I2C_DBG("position[0] = %d, position[1] = %d, position[2] = %d", position[0], position[1], position[2]);

		x = lis3lv02d_i2c_get_axis(axis.x, position);
		y = lis3lv02d_i2c_get_axis(axis.y, position);
		z = lis3lv02d_i2c_get_axis(axis.z, position);

		LIS3LV02D_I2C_DBG("In %s, x = %d, y = %d, z = %d", __FUNCTION__, x, y, z);

#ifdef CONFIG_PXA_U810
        int U810_hardware_version = pm860x_get_boardID();
        if (U810_hardware_version == 2)
        { 
        	int xn, yn;
            xn = -y;
            yn = x;  
            *data = xn;
            *(data + 1) = yn;
            *(data + 2) = -z;
        }
        if(U810_hardware_version == 1)
        {
     		*data = x;
		  	*(data + 1) = y;
		  	*(data + 2) = z;
        }

#elif defined CONFIG_PXA_U802
     	*data = -x;
		*(data + 1) = y;
		*(data + 2) = -z;
#elif defined CONFIG_PXA_U880
		*data = -y;
		*(data + 1) = x;
		*(data + 2) = -z;
#endif
	} 

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);

	return 0;
}

static int lis3lv02d_misc_open(struct inode *inode, struct file *file)
{
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);
	
	lis302_init();

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);

	return nonseekable_open(inode, file);
}

static int lis3lv02d_misc_release(struct inode *inode, struct file *file)
{
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);

	lis3lv02d_write(CTRL_REG1, 0x00);

	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
}


static int
lis3lv02d_misc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
//	char sData[SENSOR_DATA_SIZE];/* for GETDATA */
	int rwbuf[3] = {0, 0, 0};		/* for READ/WRITE */
//	char mode;					/* for SET_MODE*/
//	short value[12];			/* for SET_YPR */
//	short delay;				/* for GET_DELAY */
//	int status;					/* for OPEN/CLOSE_STATUS */
	int ret = -1;				/* Return value. */
	/*AKMDBG("%s (0x%08X).", __func__, cmd);*/
	LIS3LV02D_I2C_DBG("Start %s", __FUNCTION__);
	switch(cmd) 
	{
		case MISC_IOCTL_READ:
			if(argp == NULL) 
			{
				LIS3LV02D_I2C_DBG("invalid argument.");
				return -EINVAL;
			}
			if(copy_from_user(&rwbuf, argp, sizeof(rwbuf))) 
			{
				LIS3LV02D_I2C_DBG("copy_from_user failed.");
				return -EFAULT;
			}

			LIS3LV02D_I2C_DBG("IOCTL_READ");
			ret = misc_read(&rwbuf[0]);
			if(ret < 0) 
			{
				return ret;
			}
			LIS3LV02D_I2C_DBG("In %s, read rwbuf[0] = %d, rwbuf[1] = %d, rwbuf[2] = %d", __FUNCTION__, rwbuf[0], rwbuf[1], rwbuf[2]);

			if(copy_to_user(argp, &rwbuf, 3 * sizeof(int))) 
			{
				LIS3LV02D_I2C_DBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		default:
			LIS3LV02D_I2C_DBG("In %s, default, return", __FUNCTION__);
			return -EINVAL;
	}
	LIS3LV02D_I2C_DBG("End %s", __FUNCTION__);
	return 0;
}


static struct file_operations lis3lv02d_misc_fops = 
{
	.owner = THIS_MODULE,
	.open = lis3lv02d_misc_open,
	.release = lis3lv02d_misc_release,
	.ioctl = lis3lv02d_misc_ioctl,
};

static struct miscdevice lis3lv02d_misc_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lis3lv02d_misc_dev",
	.fops = &lis3lv02d_misc_fops,
};

static ssize_t accel_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	unsigned char chip_id = gsensor_i2c_read_byte_data(0x0F);
	sprintf(page, "0x%x\n", chip_id);
}

static void create_accel_proc_file(void)
{
	struct proc_dir_entry *accel_proc_file = create_proc_entry("driver/accel_chip_id", 0644, NULL);

	if (accel_proc_file) 
	{
		accel_proc_file->read_proc = accel_read_proc;
	} 
	else
		printk(KERN_INFO "accel proc file create failed!\n");
}


static int lis3lv02d_i2c_probe(struct platform_device *pdev)
{
	int chip_id;

//	struct gsensor_platform_data *pdata;
	int ret;
//	u8 ctrl_reg1;
	
	if(!pdev->dev.platform_data)
	{
		dev_err(&pdev->dev, "platform data missing\n");
	    return -ENODEV;
	}
//	pdata = pdev->dev.platform_data;

//	if(pdev->dev.platform_data)
//		platform = (*(int *)pdev->dev.platform_data);
	platform = 0;
	axis = lis3lv02d_axis_conversion[platform];

	LIS3LV02D_I2C_DBG("******** lis3lv02d:platform:%d ********", platform);

/*
	if(!pdata) 
	{
		dev_err(&pdev->dev,"platform data missing\n");
	    return -ENODEV;
	}

	if(!pdata->irq)
	{
		LIS3LV02D_I2C_DBG("pdata->irq is null, return!");
		return -ENODEV;
	}
*/
//	LIS3LV02D_I2C_DBG("********lis3lv02d:platform********\n");

	chip_id = gsensor_i2c_read_byte_data(0x0F);
	LIS3LV02D_I2C_DBG("chip_id = 0x%x", chip_id);

	if(chip_id < 0) 
	{
		LIS3LV02D_I2C_DBG(KERN_WARNING "lis3lv02d unavailable!\n");
		g_client = NULL;
		return -ENXIO;
	} 
/*
	else 
	{
		if(chip_id == LIS_DOUBLE_ID)
			LIS3LV02D_I2C_DBG(KERN_INFO "g-sensor: lis3lv02d(chip id:0x%02x) detected.\n", chip_id);
		else if(chip_id == LIS_SINGLE_ID)
			LIS3LV02D_I2C_DBG(KERN_INFO "g-sensor: lis331dl(chip id:0x%02x) detected.\n", chip_id);
	}


	lis3lv02d = (struct lis3lv02d_struct *)kzalloc(sizeof(struct lis3lv02d_struct), GFP_KERNEL);
	if(!lis3lv02d) 
	{
		LIS3LV02D_I2C_DBG("lis3lv02d: kzalloc memory failed!\n");
		return -1;
	}


	INIT_DELAYED_WORK(&gsensor_work, lis3lv02d_gsensor_work);
	gsensor_wqueue = create_singlethread_workqueue("gsensor");
	if(!gsensor_wqueue) 
	{
	    LIS3LV02D_I2C_DBG("failed to create work queue.\n");
		goto workqueue_failed;
	}

// set irq
	lis3lv02d->irq = pdata->irq;
	 
	ret = request_irq(lis3lv02d->irq, lis3lv02d_isr, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "lis3lv02d", NULL);
	if(ret)
	{
		LIS3LV02D_I2C_DBG("lis3lv02d request irq failed!");
		goto failed;
	}
*/
//	INIT_WORK(&lis302dl_device.lis302_work, lis302_int_routine);
//	init_waitqueue_head(&lis302dl_device.lis302_wait);
//	INIT_LIST_HEAD(&lis302dl_device.open_list);
//	init_MUTEX(&lis302dl_device.mutex);
//	spin_lock_init(&lis302dl_device.list_lock);

//	lis302dl_device.switch_on = SWITCH_OFF;
//	setup_timer(&lis302dl_device.timer, lis302dl_wake_up_int, 0);


/*
	lis3lv02d_read(Ctrl_Reg1, &ctrl_reg1);
	ctrl_reg1 |= LIS302_POWER_UP;
	lis3lv02d_write(Ctrl_Reg1, ctrl_reg1);
	lis302dl_device.switch_on = SWITCH_ON;
*/
//	lis302_init();

//	if(!int_mod)
//		mod_timer(&lis302dl_device.timer, jiffies + msecs_to_jiffies(10*interval));

	sensor_input_add(INPUT_G_SENSOR, "accelerometer", lis3lv02d_report, NULL, lis3lv02d_power_on, lis3lv02d_power_off);

	ret = misc_register(&lis3lv02d_misc_device);
	if(ret) 
	{
		LIS3LV02D_I2C_DBG("lis3lv02d_probe: lis3lv02d_misc_device register failed\n");
		goto misc_failed;
	}
	
	create_accel_proc_file();

#ifdef	CONFIG_PROC_FS
//	create_lis3lv02d_proc_file();
#endif

	return 0;

misc_failed:
	misc_deregister(&lis3lv02d_misc_device);
//workqueue_failed:
//	free_irq(lis3lv02d->irq, NULL);
failed:
	kfree(lis3lv02d);
	return -1;
}

static int lis3lv02d_i2c_remove(struct i2c_client *client __maybe_unused)
{
//	sensor_input_del("lis3lv02d");
	misc_deregister(&lis3lv02d_misc_device);
#ifdef	CONFIG_PROC_FS
//	remove_lis3lv02d_proc_file();
#endif
	return 0;
}

#if 0
static const struct i2c_device_id lis3lv02d_i2c_id[] = 
{
	{ "lis3lv02d", 0 },
	{ }
};

#endif

static struct platform_driver lis3lv02d_device_driver = 
{
	.driver = 
	{
		.name	= "lis3lv02d",
		.owner 	= THIS_MODULE,
	},
//	.id_table 	= lis3lv02d_i2c_id,
	.probe		= lis3lv02d_i2c_probe,
	.remove		= lis3lv02d_i2c_remove,
//	.suspend	= lis3lv02d_i2c_suspend,
//	.resume		= lis3lv02d_i2c_resume,
};

static int __init lis3lv02d_i2c_init(void)
{
//	return i2c_add_driver(&lis3lv02d_i2c_driver);
	return platform_driver_register(&lis3lv02d_device_driver);
}

static void __exit lis3lv02d_i2c_exit(void)
{
//	i2c_del_driver(&lis3lv02d_i2c_driver);
	platform_driver_unregister(&lis3lv02d_device_driver);
}

MODULE_DESCRIPTION
    ("ST LIS3LV02Dx three-axis digital accelerometer (I2C) driver");
MODULE_AUTHOR("Bin Yang <bin.yang@marvell.com>");
MODULE_LICENSE("GPL");

module_init(lis3lv02d_i2c_init);
module_exit(lis3lv02d_i2c_exit);
