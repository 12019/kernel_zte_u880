#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
//#include <linux/irq.h>
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
#ifdef CONFIG_PXA_U802
#include <linux/i2c.h>
#endif
#include "accel.h"
#include "../i2c/busses/i2c_gpio_pxa.h"

#define ST_CHIP_ID_REG			0x0F
#define ADI_CHIP_ID_REG			0

#define ST_CHIP_ID				0x3B
#define ADI_CHIP_ID				0xE5
#define ST_LIS3DH_CHIP_ID		0x33

#define LIS3LV02D_ADDR 			0x38
#define ADXL34X_ADDR 			0xA6
#define LIS3DH_ADDR 			0x30

#define MISC_IOCTL_READ 		3
#define MISC_IOCTL_GET_CHIP_ID 	1

#define CONTROL_ACCEL_DEBUG    	0

#if CONTROL_ACCEL_DEBUG
#define ACCEL_DBG(fmt, args...)    printk("accel : " fmt "\n", ## args)
#else if
#define ACCEL_DBG(fmt, args...)    do {} while (0)
#endif

unsigned char chip_id = 0;
static spinlock_t gsensor_i2c_lock = SPIN_LOCK_UNLOCKED;

#ifdef CONFIG_PXA_U802
int boardid = 0;
struct i2c_client *st_accel_client = NULL;
struct i2c_client *adi_accel_client = NULL;
#endif

/********************** ST operations ***********************/
static struct sensor_axis axis;
static struct sensor_axis lis3lv02d_axis_conversion[] = 
{
	{2, -1, -3},
	{-1, -2, 3},
};

int st_gsensor_i2c_write_byte_data(u8 command, u8 value)
{
#ifdef CONFIG_PXA_U802
	if(ZTE_HWVERSION2 == boardid)
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
#endif

	ACCEL_DBG("come into %s", __FUNCTION__);
	unsigned long flags;
	int err = 0;

    spin_lock_irqsave(&gsensor_i2c_lock, flags);

    err = gpio_request(18, "VIRTUAL-SCL");
    if(err) 
	{
        return -1;
    }
	err = gpio_request(19, "VIRTUAL-SDA");
    if(err) 
	{
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

#ifdef CONFIG_PXA_U812
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3DH_ADDR |I2C_WR))
#else
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3LV02D_ADDR |I2C_WR))
#endif
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
		
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(value))
	{
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

u8 st_gsensor_i2c_read_byte_data(u8 command)
{
#ifdef CONFIG_PXA_U802
	if(ZTE_HWVERSION2 == boardid)
	{
		int ret;
	
		if (st_accel_client == NULL)
			return -EINVAL;

		ret = i2c_smbus_read_byte_data(st_accel_client, command);

		return ret;
	}
#endif

	ACCEL_DBG("come into %s", __FUNCTION__);
	char data;
	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&gsensor_i2c_lock, flags);

    err = gpio_request(18, "VIRTUAL-SCL");
    if(err) 
	{
        return -1;
    }
		
	err = gpio_request(19, "VIRTUAL-SDA");
    if(err) 
	{
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

#ifdef CONFIG_PXA_U812
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3DH_ADDR |I2C_WR))
#else
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3LV02D_ADDR |I2C_WR))
#endif
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
	
	COMMON_I2C_Restart();

#ifdef CONFIG_PXA_U812
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3DH_ADDR |I2C_RD))
#else
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(LIS3LV02D_ADDR |I2C_RD))
#endif
	{
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
	u8 value = 0;
	s8 x = 0, y = 0, z = 0;
	int u812_x, u812_y, u812_z;
	int x_new_version, y_new_version;

	int position[3];
	u8 status;

	struct sensor_input_dev* sensor;
	sensor = dev_get_drvdata(&idev->dev);

	lis3lv02d_read(STATUS_REG, &status);
	if ((status & 0x7) == 0x7) 
	{
#ifdef CONFIG_PXA_U812
		position[0] = lis3lv02d_read_16(OUTX_L) / 8;
		position[1] = lis3lv02d_read_16(OUTY_L) / 8;
		position[2] = lis3lv02d_read_16(OUTZ_L) / 8;

		u812_x = lis3lv02d_i2c_get_axis(axis.x, position);
		u812_y = lis3lv02d_i2c_get_axis(axis.y, position);
		u812_z = lis3lv02d_i2c_get_axis(axis.z, position);
#else
		lis3lv02d_read(0x29, &value);
		x = (s8)value;
		lis3lv02d_read(0x2B, &value);
		y = (s8)value;
		lis3lv02d_read(0x2D, &value);
		z = (s8)value;
#endif

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
		input_report_abs(idev, ABS_X, (-y));
		input_report_abs(idev, ABS_Y, (-x));
		input_report_abs(idev, ABS_Z, (-z));

#elif defined CONFIG_PXA_U880
		input_report_abs(idev, ABS_X, x);
		input_report_abs(idev, ABS_Y, y);
		input_report_abs(idev, ABS_Z, z);

#elif defined CONFIG_PXA_U812
		input_report_abs(idev, ABS_X, u812_y);
		input_report_abs(idev, ABS_Y, (-u812_x));
		input_report_abs(idev, ABS_Z, (-u812_z));
#endif

		input_sync(idev);
	}
}


static int lis3lv02d_misc_read(int *data)
{
	s8 x = 0, y = 0, z = 0;
	u8 status = 0;
	u8 value = 0;

	lis3lv02d_read(STATUS_REG, &status);
	if((status & 0x7) == 0x7) 
	{
/*
		position[0] = lis3lv02d_read_16(OUTX_L) / 8;
		position[1] = lis3lv02d_read_16(OUTY_L) / 8;
		position[2] = lis3lv02d_read_16(OUTZ_L) / 8;

		x = lis3lv02d_i2c_get_axis(axis.x, position);
		y = lis3lv02d_i2c_get_axis(axis.y, position);
		z = lis3lv02d_i2c_get_axis(axis.z, position);
*/
		lis3lv02d_read(0x29, &value);
		x = (s8)value;
		lis3lv02d_read(0x2B, &value);
		y = (s8)value;
		lis3lv02d_read(0x2D, &value);
		z = (s8)value;

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
		*data = -y;
		*(data + 1) = -x;
		*(data + 2) = -z;
#elif defined CONFIG_PXA_U880
		*data = x;
		*(data + 1) = y;
		*(data + 2) = z;
/*
#elif defined CONFIG_PXA_U812
		*data = y;
		*(data + 1) = -x;
		*(data + 2) = -z;
*/
#endif
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
#ifdef CONFIG_PXA_U802
	if(ZTE_HWVERSION2 == boardid)
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
#endif

	unsigned long flags;
	int err = 0;

    spin_lock_irqsave(&gsensor_i2c_lock, flags);

    err = gpio_request(18, "VIRTUAL-SCL");
    if(err) 
	{
        return -1;
    }
	err = gpio_request(19, "VIRTUAL-SDA");
    if(err) 
	{
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(ADXL34X_ADDR |I2C_WR))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
		
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(value))
	{
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

u8 adi_gsensor_i2c_read_byte_data(u8 command)
{
#ifdef CONFIG_PXA_U802
	if(ZTE_HWVERSION2 == boardid)
	{
		int ret;
	
		if (adi_accel_client == NULL)
			return -EINVAL;

		ret = i2c_smbus_read_byte_data(adi_accel_client, command);

		return ret;
	}
#endif

	char data;
	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&gsensor_i2c_lock, flags);

    err = gpio_request(18, "VIRTUAL-SCL");
    if(err) 
	{
        return -1;
    }
		
	err = gpio_request(19, "VIRTUAL-SDA");
    if(err) 
	{
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(ADXL34X_ADDR |I2C_WR))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
	
	COMMON_I2C_Restart();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(ADXL34X_ADDR |I2C_RD))
	{
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

u16 adi_gsensor_i2c_read_word_data(u8 command)
{
	u8 lo = 0, hi = 0;
	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&gsensor_i2c_lock, flags);

    err = gpio_request(18, "VIRTUAL-SCL");
    if(err) 
	{
        return -1;
    }
		
	err = gpio_request(19, "VIRTUAL-SDA");
    if(err) 
	{
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(ADXL34X_ADDR | I2C_WR))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}
	
	COMMON_I2C_Restart();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(ADXL34X_ADDR | I2C_RD))
	{
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);
		return -1;
	}

	lo = COMMON_I2C_Receive_Byte(0);
	hi = COMMON_I2C_Receive_Byte(1);

	COMMON_I2C_Stop();
	COMMON_I2C_mutx_up();
	
	gpio_free(18);
	gpio_free(19);	
    spin_unlock_irqrestore(&gsensor_i2c_lock, flags);

	return ((hi << 8) | lo);
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
/*
	u8 lo, hi;

	adxl34x_read(reg, &lo);
	adxl34x_read(reg + 1, &hi);

	return (s16) ((hi << 8) | lo);
*/
#ifdef CONFIG_PXA_U802
	if(ZTE_HWVERSION2 == boardid)
	{
		unsigned char dest[2];
	
		if (adi_accel_client == NULL)
			return -EINVAL;

		i2c_smbus_read_i2c_block_data(adi_accel_client, reg, 2, dest);

		return (s16)((dest[1] << 8) | dest[0]);
	}
#endif

	return (s16) adi_gsensor_i2c_read_word_data(reg);
}

static void adxl34x_power_on(void)
{
	printk("****** come into %s ******\n", __FUNCTION__);
	u8 reg_value = 0;

	adxl34x_read(POWER_CTL, &reg_value);
	reg_value = reg_value | 0x08;
	adxl34x_write(POWER_CTL, reg_value);
/*
	adxl34x_read(BW_RATE, &reg_value);
	reg_value = reg_value | 0x0C;
	adxl34x_write(BW_RATE, reg_value);
*/
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
	s16 x, y, z;
	u8 status = 0;

	adxl34x_read(INT_SOURCE, &status);

	if(0x80 == (status & 0x80))
	{
	x = adxl34x_read_16(DATAX0);
	y = adxl34x_read_16(DATAY0);
	z = adxl34x_read_16(DATAZ0);

//		printk("adxl34x x = %d ... 0x%x,       y = %d ... 0x%x,       z = %d ... 0x%x\n\n", x, x, y, y, z, z);

#ifdef CONFIG_PXA_U802
	input_report_abs(idev, ABS_X, (-y));
	input_report_abs(idev, ABS_Y, (-x));
	input_report_abs(idev, ABS_Z, (-z));
#elif defined CONFIG_PXA_U880
	input_report_abs(idev, ABS_X, x);
	input_report_abs(idev, ABS_Y, y);
	input_report_abs(idev, ABS_Z, z);
#endif

	input_sync(idev);
}
}


static int adxl34x_misc_read(int *data)
{
	int x, y, z;
	u8 status = 0;

	adxl34x_read(INT_SOURCE, &status);

	if(0x80 == (status & 0x80))
	{
	x = adxl34x_read_16(DATAX0);
	y = adxl34x_read_16(DATAY0);
	z = adxl34x_read_16(DATAZ0);

	*data = x;
	*(data + 1) = y;
	*(data + 2) = z;

	return 0;
}

	return -1;
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







static int accel_probe(struct platform_device *pdev)
{
	ACCEL_DBG("Come into %s", __FUNCTION__);
	if(!pdev->dev.platform_data)
	{
		dev_err(&pdev->dev, "platform data missing\n");
	    return -ENODEV;
	}

	chip_id = st_gsensor_i2c_read_byte_data(ST_CHIP_ID_REG);
	ACCEL_DBG("********* chip id = 0x%x *********", chip_id);
	if((chip_id != ST_CHIP_ID) && (chip_id != ST_LIS3DH_CHIP_ID)) 
	{
		ACCEL_DBG(KERN_WARNING "It is not ST vendor!\n");
		chip_id = adi_gsensor_i2c_read_byte_data(ADI_CHIP_ID_REG);
		if(chip_id != ADI_CHIP_ID) 
		{
			ACCEL_DBG(KERN_WARNING "It is not ADI vendor!\n");
			return -ENXIO;
		}
	}

	if((ST_CHIP_ID == chip_id) || (ST_LIS3DH_CHIP_ID == chip_id))
	{
		if(st_accel_init() < 0)
		{
			ACCEL_DBG("ST accel init fail\n");
			return -ENXIO;
		}
	}

	if(ADI_CHIP_ID == chip_id)
	{
		if(adi_accel_init() < 0)
		{
			ACCEL_DBG("ADI accel init fail\n");
			return -ENXIO;
		}
	}

	return 0;
}

#ifdef CONFIG_PXA_U802
static int st_i2c_accel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ACCEL_DBG("Come into %s", __FUNCTION__);
	u8 chipid = 0;
	if(client == NULL)
	{
		ACCEL_DBG("Client is NUll!\n");
	    return -ENODEV;
	}
	st_accel_client = client;

	chipid = st_gsensor_i2c_read_byte_data(ST_CHIP_ID_REG);
	ACCEL_DBG("********* st chip id = 0x%x *********", chipid);
	if(chipid != ST_CHIP_ID)
	{
		ACCEL_DBG("It is not ST vendor!\n");
		return 0;
	}
	ACCEL_DBG("It is ST vendor!\n");
	chip_id = chipid;
	if(st_accel_init() < 0)
	{
		ACCEL_DBG("ST accel init fail\n");
		return -ENXIO;
	}

	return 0;
}

static int adi_i2c_accel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ACCEL_DBG("Come into %s", __FUNCTION__);
	u8 chipid = 0;
	if(client == NULL)
	{
		ACCEL_DBG("Client is NUll!\n");
	    return -ENODEV;
	}
	adi_accel_client = client;

	chipid = adi_gsensor_i2c_read_byte_data(ADI_CHIP_ID_REG);
	ACCEL_DBG("********* adi chip id = 0x%x *********", chipid);
	if(chipid != ADI_CHIP_ID)
	{
		ACCEL_DBG("It is not ADI vendor!\n");
		return 0;
	}
	ACCEL_DBG("It is ADI vendor!\n");
	chip_id = chipid;
	if(adi_accel_init() < 0)
	{
		ACCEL_DBG("ADI accel init fail\n");
		return -ENXIO;
	}

	return 0;
}
#endif

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








static struct platform_driver accel_device_driver = 
{
	.driver = 
	{
		.name	= "accel_vir_i2c",
		.owner 	= THIS_MODULE,
	},
	.probe		= accel_probe,
	.remove		= accel_remove,
};

/****** standard i2c start ******/
#ifdef CONFIG_PXA_U802
static const struct i2c_device_id st_accel_id[] = 
{
	{ "st_accel_i2c", 0 },
	{ }
};
static struct i2c_driver st_i2c_accel_device_driver = 
{
	.driver = 
	{
		.name = "st_accel_i2c",
	},
	.probe		= st_i2c_accel_probe,
	.remove		= accel_remove,
	.id_table   = st_accel_id,
};

static const struct i2c_device_id adi_accel_id[] = 
{
	{ "adi_accel_i2c", 0 },
	{ }
};
static struct i2c_driver adi_i2c_accel_device_driver = 
{
	.driver = 
	{
		.name = "adi_accel_i2c",
	},
	.probe		= adi_i2c_accel_probe,
	.remove		= accel_remove,
	.id_table   = adi_accel_id,
};
#endif
/****** standard i2c end ******/

static int __init accel_init(void)
{
	ACCEL_DBG("Come into %s", __FUNCTION__);
#ifdef CONFIG_PXA_U802
	boardid = pm860x_get_boardID();
	ACCEL_DBG("boardid = %d", boardid);
	if(ZTE_HWVERSION1 == boardid)
	{
		return platform_driver_register(&accel_device_driver);
	}
	else
	{
		i2c_add_driver(&st_i2c_accel_device_driver);
		return i2c_add_driver(&adi_i2c_accel_device_driver);
	}
#endif

	return platform_driver_register(&accel_device_driver);
}

static void __exit accel_exit(void)
{
#ifdef CONFIG_PXA_U802
	if(ZTE_HWVERSION1 == boardid)
	{
		platform_driver_unregister(&accel_device_driver);
	}
	else
	{
		i2c_del_driver(&st_i2c_accel_device_driver);
		i2c_del_driver(&adi_i2c_accel_device_driver);
	}
#endif

#if (defined CONFIG_PXA_U810 || defined CONFIG_PXA_U880)
	platform_driver_unregister(&accel_device_driver);
#endif
}

MODULE_DESCRIPTION("ST LIS3LV02Dx & ADI ADXL345 three-axis digital accelerometer driver");
MODULE_AUTHOR("Mozart");
MODULE_LICENSE("GPL");
module_init(accel_init);
module_exit(accel_exit);
