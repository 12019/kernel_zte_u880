/*
 *  ak8973_i2c.c - AKM AK8973 compass driver
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
#include <mach/devices.h>
#include <mach/gpio.h>
#include <mach/compass.h>
#include "../i2c/busses/i2c_gpio_pxa.h"

#define AK8973_I2C_DEBUG    1
#if AK8973_I2C_DEBUG
#define AK8973_I2C_DBG(fmt, args...)    printk("ak8973_i2c : " fmt "\n", ## args)
#else if
#define AK8973_I2C_DBG(fmt, args...)    do {} while (0)
#endif

#define AK8973_ID 0x3A

//static struct i2c_client *g_client;
//static unsigned int platform;
//static struct sensor_axis axis;
/*
static struct sensor_axis ak8973_axis_conversion[] = 
{
	{2, -1, -3},
	{-1, -2, 3},
};
*/

static int int_mod = 0; //0:real-time 1:interupt
static int interval = 2000;
static spinlock_t compass_i2c_lock = SPIN_LOCK_UNLOCKED;

static struct ak8973_struct *ak8973;

struct delayed_work compass_work;
struct workqueue_struct *compass_wqueue;

static ak8973_device_t ak8973_device = 
{
	.name 		= "ak8973_device",
	.rf_count	= ATOMIC_INIT(0),
};

int compass_i2c_write_byte_data(u8 command, u8 value)
{
//	char data;
	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&compass_i2c_lock, flags);

    err = gpio_request(SCL_GPIO_NUM, "VIRTUAL-SCL");
    if(err) 
	{
    	AK8973_I2C_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }
	err = gpio_request(SDA_GPIO_NUM, "VIRTUAL-SDA");
    if(err) 
	{
        AK8973_I2C_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(AK8973_ID |I2C_WR))
	{
		AK8973_I2C_DBG("ERROR %s %d", __FUNCTION__, 1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}
		
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		AK8973_I2C_DBG("ERROR %s %d", __FUNCTION__, 2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(value))
	{
		AK8973_I2C_DBG("ERROR %s %d", __FUNCTION__, 2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}
		
	COMMON_I2C_Stop();
	COMMON_I2C_mutx_up();

	gpio_free(SCL_GPIO_NUM);
	gpio_free(SDA_GPIO_NUM);	
	spin_unlock_irqrestore(&compass_i2c_lock, flags);

	return 0;
}

int compass_i2c_read_byte_data(u8 command, u8 *data)
{
//	char reg_data;
	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&compass_i2c_lock, flags);

    err = gpio_request(SCL_GPIO_NUM, "VIRTUAL-SCL");
    if(err) 
	{
    	AK8973_I2C_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }
		
	err = gpio_request(SDA_GPIO_NUM, "VIRTUAL-SDA");
    if(err) 
	{
        AK8973_I2C_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(AK8973_ID |I2C_WR))
	{
		AK8973_I2C_DBG("ERROR %s %d", __FUNCTION__, 1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		AK8973_I2C_DBG("ERROR %s %d", __FUNCTION__, 2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}
	
	COMMON_I2C_Restart();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(AK8973_ID | I2C_RD))
	{
		AK8973_I2C_DBG("ERROR %s %d", __FUNCTION__, 1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}

	*data = COMMON_I2C_Receive_Byte(1);

	COMMON_I2C_Stop();
	COMMON_I2C_mutx_up();
	
	gpio_free(SCL_GPIO_NUM);
	gpio_free(SDA_GPIO_NUM);	
    spin_unlock_irqrestore(&compass_i2c_lock, flags);

	return 0;
}

static int ak8973_write(u8 reg, u8 val)
{
	int ret;
	int status;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

//	if (g_client == NULL)	/* No global client pointer? */
//		return -1;
	ret = compass_i2c_write_byte_data(reg, val);
	if(0 == ret)
	{
		status = 0;
	}
	else
	{
		status = -EIO;
	}

	AK8973_I2C_DBG("End %s", __FUNCTION__);
	return status;
}

static int ak8973_read(u8 reg, u8 *pval)
{
	int ret;
	int status;
	u8 data = 0;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

//	if (g_client == NULL)	/* No global client pointer? */
//		return -1;
	ret = compass_i2c_read_byte_data(reg, &data);
	if (0 == ret) 
	{
		*pval = data;
		status = 0;
	} 
	else 
	{
		status = -EIO;
	}

	AK8973_I2C_DBG("End %s", __FUNCTION__);
	return status;
}

/*
static s16 ak8973_read_16(int reg)
{
	u8 lo, hi;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

	ak8973_read(reg, &lo);
	ak8973_read(reg + 1, &hi);
	AK8973_I2C_DBG("In %s, lo = 0x%x, hi = 0x%x", __FUNCTION__, lo, hi);
	// In "12 bit right justified" mode, bit 6, bit 7, bit 8 = bit 5
	AK8973_I2C_DBG("End %s", __FUNCTION__);
	return (s16) ((hi << 8) | lo);
}


static inline int ak8973_i2c_get_axis(s8 axis, int hw_values[3])
{
	AK8973_I2C_DBG("Start %s", __FUNCTION__);
	
	if (axis > 0)
		return hw_values[axis - 1];
	else
		return -hw_values[-axis - 1];
}
*/

void ak8973_get_xyz(u8 *x, u8 *y, u8 *z)
{
//	int position[3];
	int ret = 0;
	u8 datax = 0, datay = 0, dataz = 0;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);
/*
	position[0] = lis3lv02d_read_16(OUTX_L) / 8;
	position[1] = lis3lv02d_read_16(OUTY_L) / 8;
	position[2] = lis3lv02d_read_16(OUTZ_L) / 8;

	*x = lis3lv02d_i2c_get_axis(axis.x, position);
	*y = lis3lv02d_i2c_get_axis(axis.y, position);
	*z = lis3lv02d_i2c_get_axis(axis.z, position);
*/

	ret = ak8973_read(H1X_REG, &datax);
	if(ret != 0)
	{
		AK8973_I2C_DBG("ak8973_read x fail");
		return;
	}

	ret = ak8973_read(H1Y_REG, &datay);
	if(ret != 0)
	{
		AK8973_I2C_DBG("ak8973_read y fail");
		return;
	}

	ret = ak8973_read(H1Z_REG, &dataz);
	if(ret != 0)
	{
		AK8973_I2C_DBG("ak8973_read z fail");
		return;
	}

	*x = datax;
	*y = datay;
	*z = dataz;
	
	AK8973_I2C_DBG("In %s, x = 0x%x, y = 0x%x, z = 0x%x", __FUNCTION__, *x, *y, *z);
	AK8973_I2C_DBG("End %s", __FUNCTION__);
}


static inline void ak8973_power_off(void)
{
	/* disable X,Y,Z axis and power down */
	AK8973_I2C_DBG("Start %s", __FUNCTION__);
	
	ak8973_write(MS1_REG, POWER_DOWN_MODE);

	AK8973_I2C_DBG("End %s", __FUNCTION__);
}

static int ak8973_measure_mode_on(void)
{
	/* full-scale choose as +/- 2g*/
	AK8973_I2C_DBG("Start %s", __FUNCTION__);
	int ret = 0;
	
	ret = ak8973_write(MS1_REG, SENSOR_MEASUREMENT_MODE);
	if(0 != ret)
	{
		AK8973_I2C_DBG("MS1_REG write fail");
		return -1;
	}

	AK8973_I2C_DBG("End %s", __FUNCTION__);

	return 0;
}

static int ak8973_check_status(void)
{
	AK8973_I2C_DBG("Start %s", __FUNCTION__);
	int ret = 0;
	u8 data = 0;

	ret = ak8973_read(ST_REG, &data);
	if(ret != 0)
	{
		AK8973_I2C_DBG("ak8973_read status fail");
		return -1;
	}

	AK8973_I2C_DBG("Status reg = 0x%x", data);

	if(0 == (data & INT_HIGH))
	{
		AK8973_I2C_DBG("Now int bit is 0.");
		AK8973_I2C_DBG("End %s", __FUNCTION__);
		return INT_LOW;
	}
	else if(1 == (data & INT_HIGH))
	{
		AK8973_I2C_DBG("Now int bit is 1.");
		AK8973_I2C_DBG("End %s", __FUNCTION__);
		return INT_HIGH;
	}
	else
	{
	}

	AK8973_I2C_DBG("End %s", __FUNCTION__);

	return -1;
}

static int ak8973_reset(void)
{
	unsigned int AXIS_RST;
	int err = 0;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

	AXIS_RST = GPIO_EXT1(13);

	err = gpio_request(AXIS_RST, "AK8973 RESET");
	if(err)
	{
	 	AK8973_I2C_DBG("failed to request GPIO for AK8973 RESET");
		return -1;
	}

	gpio_direction_output(AXIS_RST, 0);
	msleep(100);
	gpio_set_value(AXIS_RST, 0);
	msleep(100);
	gpio_set_value(AXIS_RST, 1);
	msleep(100);

	gpio_free(AXIS_RST);

	AK8973_I2C_DBG("End %s", __FUNCTION__);

	return 0;
}

static void ak8973_wake_up_int(unsigned long data)
{
//	lis302_status_t *status_t;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);
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
	int ret = 0;
/*
	status = ak8973_check_status();
	if(INT_HIGH == status)
	{
		AK8973_I2C_DBG("Now int bit is high, and can not be set measurement mode, return");
		return;
	}
	if(-1 == status)
	{
		AK8973_I2C_DBG("Check status fail, return");
		return;
	}
*/

	ret = ak8973_measure_mode_on();
	if(-1 == ret)
	{
		AK8973_I2C_DBG("ak8973_measure_mode_on fail");
		return;
	}

	mdelay(100);

/*
	status = ak8973_check_status();
	if(INT_LOW == status)
	{
		AK8973_I2C_DBG("Now int bit is low, there is no data to be read, return");
		return;
	}
	if(-1 == status)
	{
		AK8973_I2C_DBG("Check status fail, return");
		return;
	}
*/
	ak8973_get_xyz(&x, &y, &z);

	AK8973_I2C_DBG("***interval = %d***", interval);
//	if(!int_mod && (ak8973_device.switch_on == SWITCH_ON))
	if(!int_mod)
		mod_timer(&ak8973_device.timer, jiffies + msecs_to_jiffies(interval));
	AK8973_I2C_DBG("End %s\n\n\n",__FUNCTION__);
}



static void ak8973_init(void)
{
	int ret = 0;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

	ret = ak8973_write(HXDA_REG, 0x81);
	if(ret != 0)
	{
		AK8973_I2C_DBG("HXDA_REG write fail");
		return;
	}

	ret = ak8973_write(HYDA_REG, 0x81);
	if(ret != 0)
	{
		AK8973_I2C_DBG("HYDA_REG write fail");
		return;
	}

	ret = ak8973_write(HZDA_REG, 0x81);
	if(ret != 0)
	{
		AK8973_I2C_DBG("HZDA_REG write fail");
		return;
	}

	ret = ak8973_write(HXGA_REG, 0x05);
	if(ret != 0)
	{
		AK8973_I2C_DBG("HXGA_REG write fail");
		return;
	}

	ret = ak8973_write(HYGA_REG, 0x05);
	if(ret != 0)
	{
		AK8973_I2C_DBG("HYGA_REG write fail");
		return;
	}

	ret = ak8973_write(HZGA_REG, 0x05);
	if(ret != 0)
	{
		AK8973_I2C_DBG("HZGA_REG write fail");
		return;
	}

	AK8973_I2C_DBG("End %s", __FUNCTION__);
}


static void ak8973_compass_work(struct work_struct *work)
{
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

	u8 x = 0, y = 0, z = 0;

	ak8973_get_xyz(&x, &y, &z);

	AK8973_I2C_DBG("End %s\n\n\n", __FUNCTION__);
}

static irqreturn_t ak8973_isr(int irq, void *dev_id)
{
	queue_delayed_work(compass_wqueue, &compass_work, msecs_to_jiffies(10));

	return IRQ_HANDLED;
}







//EXPORT_SYMBOL(ak8973_get_xyz);

static void ak8973_report(struct input_dev *idev)
{
	int x, y, z;
//	int position[3];
	u8 datax, datay, dataz;
//	s8 status;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

	struct sensor_input_dev *sensor;
	sensor = dev_get_drvdata(&idev->dev);

	if(-1 == ak8973_measure_mode_on())
	{
		AK8973_I2C_DBG("Open measure mode fail, return!");
		return;
	}
	mdelay(100);
/*
		position[0] = lis3lv02d_read_16(OutX) / 8;
		position[1] = lis3lv02d_read_16(OutY) / 8;
		position[2] = lis3lv02d_read_16(OutZ) / 8;

		x = lis3lv02d_i2c_get_axis(axis.x, position);
		y = lis3lv02d_i2c_get_axis(axis.y, position);
		z = lis3lv02d_i2c_get_axis(axis.z, position);
*/
	ak8973_get_xyz(&datax, &datay, &dataz);
		
	x = datax;
	y = datay;
	z = dataz;
	AK8973_I2C_DBG("In %s, x = 0x%x, y = 0x%x, z = 0x%x", __FUNCTION__, x, y, z);

	input_report_abs(idev, ABS_X, x);
	input_report_abs(idev, ABS_Y, y);
	input_report_abs(idev, ABS_Z, z);
	input_sync(idev);

	sensor->x = x;
	sensor->y = y;
	sensor->z = z;

	AK8973_I2C_DBG("End %s", __FUNCTION__);
}

#ifdef	CONFIG_PROC_FS
#define	AK8973_PROC_FILE	"driver/ak8973"
static struct proc_dir_entry *ak8973_proc_file;
static int index;

static ssize_t ak8973_proc_read(struct file *filp,
				   char *buffer, size_t length,
				   loff_t * offset)
{
	u8 reg_val;
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

	if ((index < 0) || (index > 0x3f))
		return 0;

	ak8973_read(index, &reg_val);
	AK8973_I2C_DBG("register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t ak8973_proc_write(struct file *filp,
				    const char *buff, size_t len,
				    loff_t * off)
{
	u8 reg_val;
	char messages[256], vol[256];
	AK8973_I2C_DBG("Start %s", __FUNCTION__);

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) 
	{
		/* set the register index */
		memcpy(vol, messages + 1, len - 1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} 
	else 
	{
		/* set the register value */
		reg_val = (int) simple_strtoul(messages, NULL, 16);
		ak8973_write(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations ak8973_proc_ops = 
{
	.read = ak8973_proc_read,
	.write = ak8973_proc_write,
};

static void create_ak8973_proc_file(void)
{
	AK8973_I2C_DBG("Start %s", __FUNCTION__);
	
	ak8973_proc_file = create_proc_entry(AK8973_PROC_FILE, 0644, NULL);
	if (ak8973_proc_file) 
	{
		ak8973_proc_file->proc_fops = &ak8973_proc_ops;
	} 
	else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_ak8973_proc_file(void)
{
	AK8973_I2C_DBG("Start %s", __FUNCTION__);
	
	remove_proc_entry(AK8973_PROC_FILE, NULL);
}

#endif

#ifdef	CONFIG_PM
static int ak8973_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int ak8973_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define	ak8973_i2c_suspend		NULL
#define	ak8973_i2c_resume		NULL
#endif

static int ak8973_i2c_probe(struct platform_device *pdev)
{
//	int chip_id;

	struct compass_platform_data *pdata;
	int ret;
	
	pdata = pdev->dev.platform_data;
	if(!pdata) 
	{
		dev_err(&pdev->dev, "platform data missing\n");
	    return -ENODEV;
	}

	AK8973_I2C_DBG("********ak8973:platform********\n");


//	INIT_WORK(&lis302dl_device.lis302_work, lis302_int_routine);
//	init_waitqueue_head(&lis302dl_device.lis302_wait);
//	INIT_LIST_HEAD(&lis302dl_device.open_list);
//	init_MUTEX(&lis302dl_device.mutex);
//	spin_lock_init(&lis302dl_device.list_lock);

//	ak8973_device.switch_on = SWITCH_OFF;
//	setup_timer(&ak8973_device.timer, ak8973_wake_up_int, 0);

//	ak8973_device.switch_on = SWITCH_ON;



	ak8973 = (struct ak8973_struct *)kzalloc(sizeof(struct ak8973_struct), GFP_KERNEL);
	if (!ak8973) 
	{
		AK8973_I2C_DBG("kzalloc memory failed!\n");
		return -1;
	}

	INIT_DELAYED_WORK(&compass_work, ak8973_compass_work);
	compass_wqueue = create_singlethread_workqueue("compass");
	if(!compass_wqueue) 
	{
	    AK8973_I2C_DBG("failed to create work queue.\n");
		goto workqueue_failed;
	}

	ak8973->irq = pdata->irq;
	
/* set irq */
	ret = request_irq(ak8973->irq, ak8973_isr, IRQF_TRIGGER_RISING, "ak8973", NULL);
	if(ret)
	{
		AK8973_I2C_DBG("ak8973 request irq failed!");
		goto failed;
	}




	ret = ak8973_reset();
	if(-1 == ret)
	{
		AK8973_I2C_DBG("ak8973_reset fail");
		return -ENODEV;
	}

	ak8973_init();





/* set measurement mode, let ak8973 to read data and then generate an interrupt, this will be called by framework after there is an app 
	ret = ak8973_measure_mode_on();
	if(-1 == ret)
	{
		AK8973_I2C_DBG("ak8973_measure_mode_on fail");
		return -ENODEV;
	}
*/






//	if(!int_mod)
//		mod_timer(&ak8973_device.timer, jiffies + msecs_to_jiffies(interval));

//	sensor_input_add(INPUT_G_SENSOR, "lis3lv02d", lis3lv02d_report, NULL, lis3lv02d_power_on, lis3lv02d_power_off);

#ifdef	CONFIG_PROC_FS
//	create_ak8973_proc_file();
#endif
	return 0;


workqueue_failed:
	free_irq(ak8973->irq, NULL);
failed:
	kfree(ak8973);
	return -1;
}

static int ak8973_i2c_remove(struct i2c_client *client __maybe_unused)
{
	sensor_input_del("ak8973");
#ifdef	CONFIG_PROC_FS
	remove_ak8973_proc_file();
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

static struct platform_driver ak8973_device_driver = 
{
	.driver = 
	{
		.name	= "ak8973",
		.owner 	= THIS_MODULE,
	},
//	.id_table 	= lis3lv02d_i2c_id,
	.probe		= ak8973_i2c_probe,
	.remove		= ak8973_i2c_remove,
	.suspend	= ak8973_i2c_suspend,
	.resume		= ak8973_i2c_resume,
};

static int __init ak8973_i2c_init(void)
{
//	return i2c_add_driver(&lis3lv02d_i2c_driver);
	return platform_driver_register(&ak8973_device_driver);
}

static void __exit ak8973_i2c_exit(void)
{
//	i2c_del_driver(&lis3lv02d_i2c_driver);
	platform_driver_unregister(&ak8973_device_driver);
}

MODULE_DESCRIPTION("AKM AK8973 compass (I2C) driver");
MODULE_AUTHOR("Mozart");
MODULE_LICENSE("GPL");

module_init(ak8973_i2c_init);
module_exit(ak8973_i2c_exit);
