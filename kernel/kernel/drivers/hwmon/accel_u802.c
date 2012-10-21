#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/sensor-input.h>
#include <mach/gsensor.h>
#include <mach/gpio.h>
#include <mach/pxa910_pm.h>
#include <linux/i2c.h>
#include "accel.h"

#define ST_CHIP_ID_REG			0x0F
#define ADI_CHIP_ID_REG			0

#define ST_CHIP_ID				0x3B
#define ADI_CHIP_ID				0xE5

#define LIS3LV02D_ADDR 			0x38
#define ADXL34X_ADDR 			0xA6

#define MISC_IOCTL_READ 		3
#define MISC_IOCTL_GET_CHIP_ID 	1

#define CONTROL_ACCEL_DEBUG    	0

#if CONTROL_ACCEL_DEBUG
#define ACCEL_DBG(fmt, args...)    printk("accel : " fmt "\n", ## args)
#else if
#define ACCEL_DBG(fmt, args...)    do {} while (0)
#endif

unsigned char chip_id = 0;
struct i2c_client *st_accel_client = NULL;
struct i2c_client *adi_accel_client = NULL;

/********************** ST operations ***********************/
static struct sensor_axis axis;
static struct sensor_axis lis3lv02d_axis_conversion[] = 
{
	{2, -1, -3},
	{-1, -2, 3},
};

int st_gsensor_i2c_write_byte_data(u8 command, u8 value)
{
	int ret;

	if(st_accel_client == NULL)
		return -1;

	ret = i2c_smbus_write_byte_data(st_accel_client, command, value);
	if(ret == 0)
		ret = 0;
	else
		ret = -EIO;

	return ret;
}

u8 st_gsensor_i2c_read_byte_data(u8 command)
{
	int ret;
	
	if (st_accel_client == NULL)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(st_accel_client, command);

	return ret;
}

static int lis3lv02d_write(u8 reg, u8 val)
{
	ACCEL_DBG("come into %s", __FUNCTION__);
	int ret;
	int status;

	ret = st_gsensor_i2c_write_byte_data(reg, val);

	if(ret == 0)
	{
		status = 0;
	}
	else
	{
		status = -EIO;
	}

	return status;
}

static int lis3lv02d_read(u8 reg, u8 *pval)
{
	int ret;
	int status;

	ret = st_gsensor_i2c_read_byte_data(reg);
	if (ret >= 0) 
	{
		*pval = ret;
		status = 0;
	} 
	else 
	{
		status = -EIO;
	}

	return status;
}

static s16 lis3lv02d_read_16(int reg)
{
	u8 lo, hi;

	lis3lv02d_read(reg, &lo);
	lis3lv02d_read(reg + 1, &hi);

	return (s16) ((hi << 8) | lo);
}

static inline int lis3lv02d_i2c_get_axis(s8 axis, int hw_values[3])
{
	if(axis > 0)
		return hw_values[axis - 1];
	else
		return -hw_values[-axis - 1];
}

static void lis302_init(void)
{
	ACCEL_DBG("come into %s", __FUNCTION__);
	u8 ctrl_reg1;

	ctrl_reg1 = LIS302_POWER_UP;
	if(lis3lv02d_write(Ctrl_Reg1, ctrl_reg1) < 0)
	{
		ACCEL_DBG("In %s, write fail", __FUNCTION__);
	}
}

static inline void lis3lv02d_power_off(void)
{
	lis3lv02d_write(CTRL_REG1, 0x00);
}

static void lis3lv02d_power_on(void)
{
	ACCEL_DBG("come into %s", __FUNCTION__);
	lis302_init();
}

static void lis3lv02d_report(struct input_dev *idev)
{
	ACCEL_DBG("come into %s", __FUNCTION__);
	int x, y, z;
	int x_new_version, y_new_version;

	int position[3];
	s8 status;

	struct sensor_input_dev* sensor;
	sensor = dev_get_drvdata(&idev->dev);

	lis3lv02d_read(STATUS_REG, &status);
	if ((status & 0x7) == 0x7) 
	{
		position[0] = lis3lv02d_read_16(OUTX_L) / 8;
		position[1] = lis3lv02d_read_16(OUTY_L) / 8;
		position[2] = lis3lv02d_read_16(OUTZ_L) / 8;

		x = lis3lv02d_i2c_get_axis(axis.x, position);
		y = lis3lv02d_i2c_get_axis(axis.y, position);
		z = lis3lv02d_i2c_get_axis(axis.z, position);
        ACCEL_DBG("lis3lv02d x = %d, y = %d, z = %d", x, y, z);

		input_report_abs(idev, ABS_X, (-x));
		input_report_abs(idev, ABS_Y, y);
		input_report_abs(idev, ABS_Z, z);
		input_sync(idev);
	}
}

static int lis3lv02d_misc_read(int *data)
{
	int x, y, z;
	int position[3];
	s8 status;

	lis3lv02d_read(STATUS_REG, &status);
	if((status & 0x7) == 0x7) 
	{
		position[0] = lis3lv02d_read_16(OUTX_L) / 8;;
		position[1] = lis3lv02d_read_16(OUTY_L) / 8;
		position[2] = lis3lv02d_read_16(OUTZ_L) / 8;

		x = lis3lv02d_i2c_get_axis(axis.x, position);
		y = lis3lv02d_i2c_get_axis(axis.y, position);
		z = lis3lv02d_i2c_get_axis(axis.z, position);

     	*data = -x;
		*(data + 1) = y;
		*(data + 2) = -z;
	} 

	return 0;
}

static int lis3lv02d_misc_open(struct inode *inode, struct file *file)
{	
	lis302_init();
	return nonseekable_open(inode, file);
}

static int lis3lv02d_misc_release(struct inode *inode, struct file *file)
{
	lis3lv02d_write(CTRL_REG1, 0x00);
}

static int
lis3lv02d_misc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int rwbuf[3] = {0, 0, 0};		/* for READ/WRITE */
	int ret = -1;				/* Return value. */

	switch(cmd) 
	{
		case MISC_IOCTL_READ:
			if(argp == NULL) 
			{
				return -EINVAL;
			}
			if(copy_from_user(&rwbuf, argp, sizeof(rwbuf))) 
			{
				return -EFAULT;
			}
			ret = lis3lv02d_misc_read(&rwbuf[0]);
			if(ret < 0) 
			{
				return ret;
			}
			if(copy_to_user(argp, &rwbuf, 3 * sizeof(int))) 
			{
				return -EFAULT;
			}
			break;

		case MISC_IOCTL_GET_CHIP_ID:
			if(copy_to_user(argp, &chip_id, sizeof(chip_id))) 
			{
				return -EFAULT;
			}
			break;

		default:
			return -EINVAL;
	}

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
	.name = "accel_misc_dev",
	.fops = &lis3lv02d_misc_fops,
};

/********************** ADI operations ***********************/
int adi_gsensor_i2c_write_byte_data(u8 command, u8 value)
{
	int ret;

	if (adi_accel_client == NULL)
		return -1;

	ret = i2c_smbus_write_byte_data(adi_accel_client, command, value);
	if(ret == 0)
		ret = 0;
	else
		ret = -EIO;

	return ret;
}

u8 adi_gsensor_i2c_read_byte_data(u8 command)
{
	int ret;
	
	if (adi_accel_client == NULL)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(adi_accel_client, command);

	return ret;
}

static int adxl34x_write(u8 reg, u8 val)
{
	int ret;
	int status;

	ret = adi_gsensor_i2c_write_byte_data(reg, val);

	if(ret == 0)
	{
		status = 0;
	}
	else
	{
		status = -EIO;
	}

	return status;
}

static int adxl34x_read(u8 reg, u8 *pval)
{
	int ret;
	int status;

	ret = adi_gsensor_i2c_read_byte_data(reg);
	if (ret >= 0) 
	{
		*pval = ret;
		status = 0;
	} 
	else 
	{
		status = -EIO;
	}

	return status;
}

static s16 adxl34x_read_16(int reg)
{
	u8 lo, hi;

	adxl34x_read(reg, &lo);
	adxl34x_read(reg + 1, &hi);

	return (s16) ((hi << 8) | lo);
}

static void adxl34x_power_on(void)
{
	printk("****** come into %s ******\n", __FUNCTION__);
	u8 reg_value = 0;
	adxl34x_read(POWER_CTL, &reg_value);
	reg_value = reg_value | 0x08;
	adxl34x_write(POWER_CTL, reg_value);
	printk("****** end %s ******\n", __FUNCTION__);
}

static inline void adxl34x_power_off(void)
{
	printk("****** come into %s ******\n", __FUNCTION__);
	u8 reg_value = 0;
	adxl34x_read(POWER_CTL, &reg_value);
	reg_value = reg_value & 0xF7;
	adxl34x_write(POWER_CTL, reg_value);
	printk("****** end %s ******\n", __FUNCTION__);
}

static void adxl34x_report(struct input_dev *idev)
{
	int x, y, z;

	x = adxl34x_read_16(DATAX0);
	y = adxl34x_read_16(DATAY0);
	z = adxl34x_read_16(DATAZ0);

	ACCEL_DBG("adxl34x x = %d, y = %d, z = %d", x, y, z);

	input_report_abs(idev, ABS_X, x);
	input_report_abs(idev, ABS_Y, y);
	input_report_abs(idev, ABS_Z, z);

	input_sync(idev);
}

static int adxl34x_misc_read(int *data)
{
	int x, y, z;

	x = adxl34x_read_16(DATAX0);
	y = adxl34x_read_16(DATAY0);
	z = adxl34x_read_16(DATAZ0);

	*data = x;
	*(data + 1) = y;
	*(data + 2) = z;

	return 0;
}

static int adxl34x_misc_open(struct inode *inode, struct file *file)
{
	ACCEL_DBG("Come into %s", __FUNCTION__);
	adxl34x_power_on();

	return nonseekable_open(inode, file);
}

static int adxl34x_misc_release(struct inode *inode, struct file *file)
{
	adxl34x_power_off();
}

static int
adxl34x_misc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	ACCEL_DBG("Come into %s, cmd = %d", __FUNCTION__, cmd);
	void __user *argp = (void __user *)arg;
	int rwbuf[3] = {0, 0, 0};		/* for READ/WRITE */
//	u8 chip_id = 0;
	int ret = -1;				/* Return value. */

	switch(cmd) 
	{
		case MISC_IOCTL_READ:
			if(argp == NULL) 
			{
				return -EINVAL;
			}
			if(copy_from_user(&rwbuf, argp, sizeof(rwbuf))) 
			{
				return -EFAULT;
			}

			ret = adxl34x_misc_read(&rwbuf[0]);
			if(ret < 0) 
			{
				return ret;
			}

			if(copy_to_user(argp, &rwbuf, 3 * sizeof(int))) 
			{
				return -EFAULT;
			}
			break;

		case MISC_IOCTL_GET_CHIP_ID:
			ACCEL_DBG("Now it is MISC_IOCTL_GET_CHIP_ID, chip_id = 0x%x", chip_id);
			if(copy_to_user(argp, &chip_id, sizeof(chip_id))) 
			{
				return -EFAULT;
			}
			break;

		default:
			return -EINVAL;
	}

	return 0;
}

static struct file_operations adxl34x_misc_fops = 
{
	.owner = THIS_MODULE,
	.open = adxl34x_misc_open,
	.release = adxl34x_misc_release,
	.ioctl = adxl34x_misc_ioctl,
};

static struct miscdevice adxl34x_misc_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "accel_misc_dev",
	.fops = &adxl34x_misc_fops,
};


static ssize_t accel_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
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

static int st_accel_init(void)
{
	ACCEL_DBG("It is ST accel IC");
	int ret = 0;

	axis = lis3lv02d_axis_conversion[0];

	sensor_input_add(INPUT_G_SENSOR, "accelerometer", lis3lv02d_report, NULL, lis3lv02d_power_on, lis3lv02d_power_off);

	ret = misc_register(&lis3lv02d_misc_device);
	if(ret) 
	{
		ACCEL_DBG("lis3lv02d_probe: lis3lv02d_misc_device register failed\n");
		goto misc_failed;
	}
	
	create_accel_proc_file();

	return 0;

misc_failed:
	misc_deregister(&lis3lv02d_misc_device);
	return -1;
}

static int adi_accel_init(void)
{
	ACCEL_DBG("It is ADI accel IC");
	int ret = 0;

	sensor_input_add(INPUT_G_SENSOR, "accelerometer", adxl34x_report, NULL, adxl34x_power_on, adxl34x_power_off);

	ret = misc_register(&adxl34x_misc_device);
	if(ret) 
	{
		ACCEL_DBG("adxl34x_probe: adxl34x_misc_device register failed\n");
		goto misc_failed;
	}
	
	create_accel_proc_file();

	return 0;

misc_failed:
	misc_deregister(&adxl34x_misc_device);
	return -1;
}

static int accel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if(client == NULL)
	{
		ACCEL_DBG("Client is NUll!\n");
	    return -ENODEV;
	}
/*
	chip_id = st_gsensor_i2c_read_byte_data(ST_CHIP_ID_REG);
	ACCEL_DBG("chip_id = 0x%x", chip_id);
	if(chip_id != ST_CHIP_ID)
	{
		ACCEL_DBG(KERN_WARNING "It is not ST vendor!\n");
		chip_id = adi_gsensor_i2c_read_byte_data(ADI_CHIP_ID_REG);
		if(chip_id != ADI_CHIP_ID)
		{
			ACCEL_DBG(KERN_WARNING "It is not ADI vendor!\n");
			return -ENXIO;
		}
	}
*/
	chip_id = ST_CHIP_ID;
	ACCEL_DBG("********* chip id = 0x%x *********", chip_id);

	if(ST_CHIP_ID == chip_id)
	{
		st_accel_client = client;
		if(st_accel_init() < 0)
		{
			ACCEL_DBG("ST accel init fail\n");
			return -ENXIO;
		}
	}

	if(ADI_CHIP_ID == chip_id)
	{
		adi_accel_client = client;
		if(adi_accel_init() < 0)
		{
			ACCEL_DBG("ADI accel init fail\n");
			return -ENXIO;
		}
	}

	return 0;
}

static int accel_remove(void)
{
	ACCEL_DBG("Come into %s", __FUNCTION__);
	if(ST_CHIP_ID == chip_id)
	{
		misc_deregister(&lis3lv02d_misc_device);
	}
	if(ADI_CHIP_ID == chip_id)
	{
		misc_deregister(&adxl34x_misc_device);
	}
	return 0;
}

static const struct i2c_device_id accel_id[] = 
{
	{ "accel", 0 },
	{ }
};

static struct i2c_driver accel_device_driver = 
{
	.driver = 
	{
		.name = "accel",
	},
	.probe		= accel_probe,
	.remove		= accel_remove,
	.id_table   = accel_id,
};

static int __init accel_init(void)
{
	int ret;

	ret = i2c_add_driver(&accel_device_driver);
	if(ret) 
		ACCEL_DBG("Driver registration failed, module not inserted.\n");
	else
		ACCEL_DBG("i2c_add_driver() OK!\n");
	
	return ret;
}

static void __exit accel_exit(void)
{
	i2c_del_driver(&accel_device_driver);
}

MODULE_DESCRIPTION("ST LIS3LV02Dx & ADI ADXL345 three-axis digital accelerometer I2C driver");
MODULE_AUTHOR("Mozart");
MODULE_LICENSE("GPL");
module_init(accel_init);
module_exit(accel_exit);
