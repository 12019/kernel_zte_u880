/* 
 * drivers/i2c/chips/akm8973.c - akm8973 compass driver
 *
 * Copyright (C) 2008-2009 HTC Corporation.
 * Author: viral wang <viralwang@gmail.com>
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

/*
 * Revised by AKM 2010/03/25
 * 
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/akm8973.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/sensor-input.h>
#include <linux/proc_fs.h>
#include "../busses/i2c_gpio_pxa.h"

#define AKM8973_DEBUG		1
#define AKM8973_DEBUG_MSG	0
#define AKM8973_DEBUG_FUNC	0
#define AKM8973_DEBUG_DATA	0
#define MAX_FAILURE_COUNT	3
#define AKM8973_RETRY_COUNT	10
#define AKM8973_DEFAULT_DELAY	100

#if AKM8973_DEBUG_MSG
#define AKMDBG(format, ...)	printk(KERN_INFO "AKM8973 " format "\n", ## __VA_ARGS__)
#else
#define AKMDBG(format, ...)
#endif

#if AKM8973_DEBUG_FUNC
#define AKMFUNC(func) printk(KERN_INFO "AKM8973 " func " is called\n")
#else
#define AKMFUNC(func)
#endif

#define AK8973_ID 0x3A

//static struct i2c_client *this_client;
//static struct platform_device *this_pdev;
struct akm8973_data *akm;

struct akm8973_data 
{
	struct input_dev *input_dev_mag;
	struct input_dev *input_dev_ori;
	struct input_dev *input_dev_tem;
	struct work_struct work;
	struct early_suspend akm_early_suspend;
};

/* Addresses to scan -- protected by sense_data_mutex */
static char sense_data[SENSOR_DATA_SIZE];
static struct mutex sense_data_mutex;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t data_ready;
static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t t_flag;
static atomic_t mv_flag;

static int failure_count = 0;

static short akmd_delay = AKM8973_DEFAULT_DELAY;

static atomic_t suspend_flag = ATOMIC_INIT(0);

static struct akm8973_platform_data *pdata;

static spinlock_t compass_i2c_lock = SPIN_LOCK_UNLOCKED;

int compass_i2c_write_byte_data(char command, char value)
{
/*
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
#if AKM8973_DEBUG_DATA
	int i;
#endif
#ifdef AKM8973_DEBUG
// Caller should check parameter validity.
	if ((txData == NULL) || (length < 2)) {
		return -EINVAL;
	}
#endif	
	for (loop_i = 0; loop_i < AKM8973_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= AKM8973_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n", __func__, AKM8973_RETRY_COUNT);
		return -EIO;
	}
#if AKM8973_DEBUG_DATA
	printk(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
	for (i = 0; i < (length-1); i++) {
		printk(KERN_INFO " %02x", txData[i + 1]);
	}
	printk(KERN_INFO "\n");
#endif
	return 0;
*/
	AKMDBG("Start %s", __FUNCTION__);

	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&compass_i2c_lock, flags);

    err = gpio_request(SCL_GPIO_NUM, "VIRTUAL-SCL");
    if(err) 
	{
    	AKMDBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }
	err = gpio_request(SDA_GPIO_NUM, "VIRTUAL-SDA");
    if(err) 
	{
        AKMDBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(AK8973_ID |I2C_WR))
	{
		AKMDBG("ERROR %s %d", __FUNCTION__, 1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}
		
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		AKMDBG("ERROR %s %d", __FUNCTION__, 2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(value))
	{
		AKMDBG("ERROR %s %d", __FUNCTION__, 2);
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

	AKMDBG("End %s", __FUNCTION__);
	return 0;
}

int compass_i2c_read_byte_data(char command, char *data)
{
/*
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
#if AKM8973_DEBUG_DATA
	int i;
	char addr = rxData[0];
#endif
#ifdef AKM8973_DEBUG
// Caller should check parameter validity.
	if ((rxData == NULL) || (length < 1)) {
		return -EINVAL;
	}
#endif
	for (loop_i = 0; loop_i < AKM8973_RETRY_COUNT; loop_i++) 
	{
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0) 
		{
			break;
		}
		mdelay(10);
	}

	if (loop_i >= AKM8973_RETRY_COUNT) 
	{
		printk(KERN_ERR "%s retry over %d\n", __func__, AKM8973_RETRY_COUNT);
		return -EIO;
	}
#if AKM8973_DEBUG_DATA
	printk(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
	for (i = 0; i < length; i++) {
		printk(KERN_INFO " %02x", rxData[i]);
	}
    printk(KERN_INFO "\n");
#endif
	return 0;
*/
	AKMDBG("Start %s", __FUNCTION__);

	unsigned long flags;
	int err = 0;
	 
    spin_lock_irqsave(&compass_i2c_lock, flags);

    err = gpio_request(SCL_GPIO_NUM, "VIRTUAL-SCL");
    if(err) 
	{
    	AKMDBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }
		
	err = gpio_request(SDA_GPIO_NUM, "VIRTUAL-SDA");
    if(err) 
	{
        AKMDBG("failed to request GPIO20 for USB-UART\n");
        return -1;
    }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(AK8973_ID |I2C_WR))
	{
		AKMDBG("ERROR %s %d", __FUNCTION__, 1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		AKMDBG("ERROR %s %d", __FUNCTION__, 2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	    spin_unlock_irqrestore(&compass_i2c_lock, flags);
		return -1;
	}
	
	COMMON_I2C_Restart();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(AK8973_ID | I2C_RD))
	{
		AKMDBG("ERROR %s %d", __FUNCTION__, 1);
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

	AKMDBG("End %s", __FUNCTION__);
	return 0;
}

static int AKI2C_TxData(char *txData, int length)
{
	AKMDBG("Start %s", __FUNCTION__);

	int ret;
	int i = 0;
	char reg, val;

	reg = *txData;
	while(i < (length - 1))
	{
		val = *(txData + i + 1);

		ret = compass_i2c_write_byte_data((reg + i), val);
		if(0 != ret)
		{
			AKMDBG("In %s, fail to write data to reg 0x%x", __FUNCTION__, (reg + i));
			return -EIO;
		}
		udelay(100);
		i++;
	}
	
	AKMDBG("End %s", __FUNCTION__);
	return 0;
}

static int AKI2C_RxData(char *rxData, int length)
{
	AKMDBG("Start %s", __FUNCTION__);

	int ret;
	int i = 0;
	char reg, val;

	reg = *rxData;

	AKMDBG("In %s, reg = 0x%x, length = 0x%x", __FUNCTION__, reg, length);

	while(i < length)
	{
		ret = compass_i2c_read_byte_data((reg + i), &val);

		AKMDBG("In %s, buffer[i + 1] = 0x%x", __FUNCTION__, val);

		if(0 == ret) 
		{
			*(rxData + i + 1) = val;
		} 
		else 
		{
			AKMDBG("In %s, fail to read data from 0x%x", __FUNCTION__, (reg + i));
			return -EIO;
		}
		udelay(100);
		i++;
	}

	AKMDBG("End %s", __FUNCTION__);
	return 0;
}

static void AKECS_Reset(void)
{
	AKMDBG("Start %s", __FUNCTION__);
#ifdef CONFIG_PXA_U810
	gpio_set_value(pdata->gpio_RST, 0);
	udelay(120);
	gpio_set_value(pdata->gpio_RST, 1);
#endif
	AKMDBG("End %s", __FUNCTION__);
}

static int AKECS_SetMode_Measure(void)
{
	AKMDBG("Start %s", __FUNCTION__);

	char buffer[2];

	atomic_set(&data_ready, 0);

	/* Set measure mode */
	buffer[0] = AK8973_REG_MS1;
	buffer[1] = AK8973_MODE_MEASURE;

	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

//qmm
static int AKECS_read_data_test(void)
{
	AKMDBG("Start %s", __FUNCTION__);
	int ret = 0;
	char buffer[5] = {0, 0, 0, 0, 0};

	buffer[0] = AK8973_REG_TMPS;

	AKECS_Reset();

	if(0 != AKECS_SetMode_Measure())
	{
		AKMDBG("In %s, AKECS_SetMode_Measure fail");
		return -1;
	}

	ret = AKI2C_RxData(buffer, 4);
	if(0 != ret)
	{
		AKMDBG("In %s, AKI2C_RxData fail");
		return -1;
	}
	AKMDBG("reg 0xC1 = 0x%x, reg 0xC2 = 0x%x, reg 0xC3 = 0x%x, reg 0xC4 = 0x%x", buffer[1], buffer[2], buffer[3], buffer[4]);
	return 0;
}

static int AKECS_SetMode_E2PRead(void)
{
	AKMDBG("Start %s", __FUNCTION__);

	char buffer[2];
	
	/* Set measure mode */
	buffer[0] = AK8973_REG_MS1;
	buffer[1] = AK8973_MODE_E2P_READ;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_PowerDown(void)
{
	AKMDBG("Start %s", __FUNCTION__);

	char buffer[2];
	int ret;

	/* Set powerdown mode */
	buffer[0] = AK8973_REG_MS1;
	buffer[1] = AK8973_MODE_POWERDOWN;
	/* Set data */
	ret = AKI2C_TxData(buffer, 2);
	if (ret < 0) 
	{
		AKMDBG("txdata fail");
		return ret;
	}

	/* Dummy read for clearing INT pin */
	buffer[0] = AK8973_REG_TMPS;
	/* Read data */
	ret = AKI2C_RxData(buffer, 1);
	if (ret < 0) 
	{
		AKMDBG("rxdata fail");
		return ret;
	}

	AKMDBG("End %s", __FUNCTION__);
	return ret;
}

static int AKECS_SetMode(char mode)
{
	AKMDBG("Start %s", __FUNCTION__);

	int ret;

	switch (mode) 
	{
		case AK8973_MODE_MEASURE:
			ret = AKECS_SetMode_Measure();
			break;
		case AK8973_MODE_E2P_READ:
//			printk("111111111111111111\n");
			ret = AKECS_SetMode_E2PRead();
			break;
		case AK8973_MODE_POWERDOWN:
			ret = AKECS_SetMode_PowerDown();
			break;
		default:
			AKMDBG("%s: Unknown mode(%d)", __func__, mode);
			return -EINVAL;
	}
	/* wait at least 300us after changing mode */
	udelay(300);

	AKMDBG("End %s", __FUNCTION__);
	return ret;
}

static int AKECS_CheckDevice(void)
{
	AKMDBG("Start %s", __FUNCTION__);

	int ret;
	ret = AKECS_SetMode_PowerDown();
	if (ret < 0) 
	{
		AKMDBG(KERN_ERR "AKM8973 akm8973_probe: set power down mode error\n");
	}
	AKMDBG("End %s", __FUNCTION__);
	return ret;
}

static int AKECS_GetData(char *rbuf, int size)
{
	AKMDBG("Start %s", __FUNCTION__);
#ifdef AKM8973_DEBUG
	/* This function is not exposed, so parameters 
	 should be checked internally.*/
	if ((rbuf == NULL) || (size < SENSOR_DATA_SIZE)) 
	{
		return -EINVAL;
	}
#endif
	wait_event_interruptible_timeout(data_ready_wq,
									 atomic_read(&data_ready), 1000);
	if (!atomic_read(&data_ready)) 
	{
		AKMDBG("%s: data_ready is not set.", __func__);
		if (!atomic_read(&suspend_flag)) 
		{
			AKMDBG("%s: suspend_flag is not set.", __func__);
			failure_count++;
			if (failure_count >= MAX_FAILURE_COUNT) 
			{
				AKMDBG(KERN_ERR
				       "AKM8973 AKECS_GetData: successive %d failure.\n",
				       failure_count);
				atomic_set(&open_flag, -1);
				wake_up(&open_wq);
				failure_count = 0;
			}
		}
		return -1;
	}
	
	mutex_lock(&sense_data_mutex);
	memcpy(rbuf, sense_data, size);
	atomic_set(&data_ready, 0);
	mutex_unlock(&sense_data_mutex);
	
	failure_count = 0;

	AKMDBG("End %s", __FUNCTION__);
	return 0;
}

static void AKECS_SetYPR(short *rbuf)
{
	AKMDBG("Start %s", __FUNCTION__);
//	printk("\nStart %s\n", __FUNCTION__);

//	struct akm8973_data *data = i2c_get_clientdata(this_client);
	struct akm8973_data *data = akm;
#if 0
#if AKM8973_DEBUG_DATA
	AKMDBG("AKM8973 %s:", __func__);
	AKMDBG("  yaw =%6d, pitch =%6d, roll =%6d",
		   rbuf[0], rbuf[1], rbuf[2]);
	AKMDBG("  tmp =%6d, m_stat =%6d, g_stat =%6d",
		   rbuf[3], rbuf[4], rbuf[5]);
	AKMDBG("  Acceleration[LSB]: %6d,%6d,%6d",
	       rbuf[6], rbuf[7], rbuf[8]);
	AKMDBG("  Geomagnetism[LSB]: %6d,%6d,%6d",
	       rbuf[9], rbuf[10], rbuf[11]);
#endif
#endif
	/* Report magnetic sensor information */
	if (atomic_read(&m_flag)) 
	{
//		printk("###### Now report ORI ######, ABS_RX = %d, ABS_RY = %d, ABS_RZ = %d, ABS_RUDDER = %d\n", rbuf[0], rbuf[1], rbuf[2], rbuf[4]);
		input_report_abs(data->input_dev_ori, ABS_RX, rbuf[0]);
		input_report_abs(data->input_dev_ori, ABS_RY, rbuf[1]);
		input_report_abs(data->input_dev_ori, ABS_RZ, rbuf[2]);

//		input_report_abs(data->input_dev, ABS_WHEEL, rbuf[5]);
	}
#if 0	
	/* Report acceleration sensor information */
	if (atomic_read(&a_flag)) 
	{
		printk("###### Now report ACC ######\n");
		input_report_abs(data->input_dev, ABS_X, rbuf[6]);
		input_report_abs(data->input_dev, ABS_Y, rbuf[7]);
		input_report_abs(data->input_dev, ABS_Z, rbuf[8]);
		input_report_abs(data->input_dev, ABS_WHEEL, rbuf[5]);
	}
#endif
	/* Report temperature information */
	if (atomic_read(&t_flag)) 
	{
		input_report_abs(data->input_dev_tem, ABS_THROTTLE, rbuf[3]);
	}

	if (atomic_read(&mv_flag)) 
	{
//		printk("###### Now report MAG ######, ABS_HAT0X = %d, ABS_HAT0Y = %d, ABS_BRAKE = %d\n", rbuf[9], rbuf[10], rbuf[11]);
		input_report_abs(data->input_dev_mag, ABS_HAT0X, rbuf[9]);
		input_report_abs(data->input_dev_mag, ABS_HAT0Y, rbuf[10]);
		input_report_abs(data->input_dev_mag, ABS_BRAKE, rbuf[11]);

		input_report_abs(data->input_dev_mag, ABS_RUDDER, rbuf[4]);
	}

	input_sync(data->input_dev_ori);
	input_sync(data->input_dev_mag);
	input_sync(data->input_dev_tem);

	AKMDBG("End %s", __FUNCTION__);
}

static int AKECS_GetOpenStatus(void)
{
	AKMDBG("Start %s", __FUNCTION__);

	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{
	AKMDBG("Start %s", __FUNCTION__);

	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));
	return atomic_read(&open_flag);
}

static void AKECS_CloseDone(void)
{
	AKMDBG("Start %s", __FUNCTION__);
	atomic_set(&m_flag, 1);
	atomic_set(&a_flag, 1);
	atomic_set(&t_flag, 1);
	atomic_set(&mv_flag, 1);
	AKMDBG("End %s", __FUNCTION__);
}

/***** akm_aot functions ***************************************/
static int akm_aot_open(struct inode *inode, struct file *file)
{
	AKMDBG("Start %s", __FUNCTION__);

	int ret = -1;

//	AKMFUNC("akm_aot_open");
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) 
	{
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) 
		{
			atomic_set(&reserve_open_flag, 1);
			wake_up(&open_wq);
			ret = 0;
		}
	}
	AKMDBG("akm_aot_open (%d,%d,%d)",
		   atomic_read(&open_count),
		   atomic_read(&open_flag), 
		   atomic_read(&reserve_open_flag));

	AKMDBG("End %s", __FUNCTION__);
	return ret;
}

static int akm_aot_release(struct inode *inode, struct file *file)
{
	AKMDBG("Start %s", __FUNCTION__);
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq);
	AKMDBG("End %s", __FUNCTION__);
	return 0;
}

static int
akm_aot_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	AKMDBG("Start %s", __FUNCTION__);
//	printk("Start %s\n", __FUNCTION__);

	void __user *argp = (void __user *)arg;
	short flag;
	
	switch (cmd) 
	{
		case ECS_IOCTL_APP_SET_MFLAG:
		case ECS_IOCTL_APP_SET_AFLAG:
		case ECS_IOCTL_APP_SET_TFLAG:
		case ECS_IOCTL_APP_SET_MVFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag))) 
			{
				return -EFAULT;
			}
			if (flag < 0 || flag > 1) 
			{
				return -EINVAL;
			}
			break;
		case ECS_IOCTL_APP_SET_DELAY:
			if (copy_from_user(&flag, argp, sizeof(flag))) 
			{
//				printk("In kernel, ECS_IOCTL_APP_SET_DELAY fail\n");
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	switch (cmd) 
	{
		case ECS_IOCTL_APP_SET_MFLAG:
			atomic_set(&m_flag, flag);
			AKMDBG("MFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_MFLAG:
			flag = atomic_read(&m_flag);
			break;
		case ECS_IOCTL_APP_SET_AFLAG:
			atomic_set(&a_flag, flag);
			AKMDBG("AFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_AFLAG:
			flag = atomic_read(&a_flag);
			break;
		case ECS_IOCTL_APP_SET_TFLAG:
			atomic_set(&t_flag, flag);
			AKMDBG("TFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_TFLAG:
			flag = atomic_read(&t_flag);
			break;
		case ECS_IOCTL_APP_SET_MVFLAG:
			atomic_set(&mv_flag, flag);
			AKMDBG("MVFLAG is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_MVFLAG:
			flag = atomic_read(&mv_flag);
			break;
		case ECS_IOCTL_APP_SET_DELAY:
			akmd_delay = flag;
			AKMDBG("Delay is set to %d", flag);
			break;
		case ECS_IOCTL_APP_GET_DELAY:
			flag = akmd_delay;
			break;
		default:
			return -ENOTTY;
	}
	
	switch (cmd) 
	{
		case ECS_IOCTL_APP_GET_MFLAG:
		case ECS_IOCTL_APP_GET_AFLAG:
		case ECS_IOCTL_APP_GET_TFLAG:
		case ECS_IOCTL_APP_GET_MVFLAG:
		case ECS_IOCTL_APP_GET_DELAY:
			if (copy_to_user(argp, &flag, sizeof(flag))) 
			{
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	AKMDBG("End %s", __FUNCTION__);
	return 0;
}

/***** akmd functions ********************************************/
static int akmd_open(struct inode *inode, struct file *file)
{
	AKMDBG("Start %s", __FUNCTION__);
	return nonseekable_open(inode, file);
}

static int akmd_release(struct inode *inode, struct file *file)
{
	AKMDBG("Start %s", __FUNCTION__);
	AKECS_CloseDone();
	return 0;
}

static int
akmd_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		   unsigned long arg)
{
	AKMDBG("Start %s", __FUNCTION__);
//	printk("****** Start %s, cmd = %d ******\n", __FUNCTION__, cmd);

	void __user *argp = (void __user *)arg;
	
	/* NOTE: In this function the size of "char" should be 1-byte. */
	char sData[SENSOR_DATA_SIZE];/* for GETDATA */
	char rwbuf[RWBUF_SIZE];		/* for READ/WRITE */
	char mode;					/* for SET_MODE*/
	short value[12];			/* for SET_YPR */
	short delay;				/* for GET_DELAY */
	int status;					/* for OPEN/CLOSE_STATUS */
	int ret = -1;				/* Return value. */
	/*AKMDBG("%s (0x%08X).", __func__, cmd);*/
	
	switch (cmd) 
	{
		case ECS_IOCTL_WRITE:
//			printk("****** aaaaaaaaaaaaaaaaaaa ******\n");
		case ECS_IOCTL_READ:
//			printk("****** bbbbbbbbbbbbbbbbbbb ******\n");
			if (argp == NULL) 
			{
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) 
			{
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_SET_MODE:
//			printk("****** ccccccccccccccccccccc ******\n");
			if (argp == NULL) 
			{
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&mode, argp, sizeof(mode))) 
			{
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_SET_YPR:
//			printk("****** dddddddddddddddddddddd ******\n");
			if (argp == NULL) 
			{
				
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&value, argp, sizeof(value))) 
			{
//				printk("****** 22222222222222 ******\n");
				AKMDBG("copy_from_user failed.");
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	switch (cmd) 
	{
		case ECS_IOCTL_WRITE:
//			printk("****** eeeeeeeeeeeeeeeeeeeeeee ******\n");
			AKMDBG("IOCTL_WRITE");
			if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE-1))) 
			{
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
			if (ret < 0) 
			{
				return ret;
			}
			break;
		case ECS_IOCTL_READ:
//			printk("****** ffffffffffffffffffffff ******\n");
			AKMDBG("IOCTL_READ");
			if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE-1))) 
			{
				AKMDBG("invalid argument.");
				return -EINVAL;
			}
			ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
			if (ret < 0) 
			{
				return ret;
			}
			AKMDBG("In akmd_ioctl, rwbuf[0] = 0x%x, rwbuf[1] = 0x%x", rwbuf[0], rwbuf[1]);
			AKMDBG("********* In akmd_ioctl, rwbuf[2] = 0x%x, rwbuf[3] = 0x%x, rwbuf[4] = 0x%x, rwbuf[5] = 0x%x, rwbuf[6] = 0x%x *********", 
					rwbuf[2], rwbuf[3], rwbuf[4], rwbuf[5], rwbuf[6]);
			break;
		case ECS_IOCTL_RESET:
//			printk("****** ggggggggggggggggggggggg ******\n");
			AKMDBG("IOCTL_RESET");
			AKECS_Reset();
			break;
		case ECS_IOCTL_SET_MODE:
//			printk("****** hhhhhhhhhhhhhhhhhhhhhh ******\n");
			AKMDBG("IOCTL_SET_MODE");
			ret = AKECS_SetMode(mode);
			if (ret < 0) 
			{
				return ret;
			}
			break;
		case ECS_IOCTL_GETDATA:
//			printk("****** iiiiiiiiiiiiiiiiiiiiiii ******\n");
			AKMDBG("IOCTL_GET_DATA");
			ret = AKECS_GetData(sData, SENSOR_DATA_SIZE);
			if (ret < 0) 
			{
				return ret;
			}
			break;
		case ECS_IOCTL_SET_YPR:
//			printk("****** jjjjjjjjjjjjjjjjjj ******\n");
			AKECS_SetYPR(value);
			break;
		case ECS_IOCTL_GET_OPEN_STATUS:
//			printk("****** kkkkkkkkkkkkkkkkkk ******\n");
			AKMDBG("IOCTL_GET_OPEN_STATUS");
			status = AKECS_GetOpenStatus();
			AKMDBG("AKECS_GetOpenStatus returned (%d)", status);
			break;
		case ECS_IOCTL_GET_CLOSE_STATUS:
//			printk("****** lllllllllllllllllllll ******\n");
			AKMDBG("IOCTL_GET_CLOSE_STATUS");
			status = AKECS_GetCloseStatus();
			AKMDBG("AKECS_GetCloseStatus returned (%d)", status);
			break;
		case ECS_IOCTL_GET_DELAY:
//			printk("****** mmmmmmmmmmmmmmmmmmmmmmmm ******\n");
			AKMDBG("IOCTL_GET_DELAY");
			delay = akmd_delay;
			break;
		default:
			return -ENOTTY;
	}
	
	switch (cmd) 
	{
		case ECS_IOCTL_READ:
//			printk("****** nnnnnnnnnnnnnnnnnnnnnn ******\n");
			if (copy_to_user(argp, &rwbuf, rwbuf[0] + 2)) 
			{
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GETDATA:
//			printk("****** oooooooooooooooooooooo ******\n");
			if (copy_to_user(argp, &sData, sizeof(sData))) 
			{
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_OPEN_STATUS:
//			printk("****** ppppppppppppppppppp ******\n");
		case ECS_IOCTL_GET_CLOSE_STATUS:
//			printk("****** qqqqqqqqqqqqqqqqqqqqq ******\n");
			if (copy_to_user(argp, &status, sizeof(status))) 
			{
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_DELAY:
//			printk("****** rrrrrrrrrrrrrrrrrrrrr ******\n");
			if (copy_to_user(argp, &delay, sizeof(delay))) 
			{
				AKMDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		default:
			break;
	}
	
	AKMDBG("End %s", __FUNCTION__);
	return 0;
}
/*
static void akm8973_work_func(struct work_struct *work)
{
	char buffer[SENSOR_DATA_SIZE];
	int ret;
	
	memset(buffer, 0, SENSOR_DATA_SIZE);
	buffer[0] = AK8973_REG_ST;
	ret = AKI2C_RxData(buffer, SENSOR_DATA_SIZE);
	if (ret < 0) 
	{
		printk(KERN_ERR "AKM8973 akm8973_work_func: I2C failed\n");
		return;
	}
	// Check ST bit
	if ((buffer[0] & 0x01) != 0x01) 
	{
		printk(KERN_ERR "AKM8973 akm8973_work_func: ST is not set\n");
		return;
	}
	
	mutex_lock(&sense_data_mutex);
	memcpy(sense_data, buffer, SENSOR_DATA_SIZE);
	atomic_set(&data_ready, 1);
	wake_up(&data_ready_wq);
	mutex_unlock(&sense_data_mutex);
	
	enable_irq(this_client->irq);
	
	AKMFUNC("akm8973_work_func");
}

static irqreturn_t akm8973_interrupt(int irq, void *dev_id)
{
	struct akm8973_data *data = dev_id;
	AKMFUNC("akm8973_interrupt");
	disable_irq(this_client->irq);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}
*/
static void akm8973_early_suspend(struct early_suspend *handler)
{
	AKMDBG("Start %s", __FUNCTION__);
	atomic_set(&suspend_flag, 1);
	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
	wake_up(&open_wq);
//	disable_irq(this_client->irq);
	AKMDBG("suspended with flag=%d", 
	       atomic_read(&reserve_open_flag));
	AKMDBG("End %s", __FUNCTION__);
}

static void akm8973_early_resume(struct early_suspend *handler)
{
	AKMDBG("Start %s", __FUNCTION__);
//	enable_irq(this_client->irq);
	atomic_set(&suspend_flag, 0);
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
	wake_up(&open_wq);
	AKMDBG("resumed with flag=%d", 
	       atomic_read(&reserve_open_flag));
	AKMDBG("End %s", __FUNCTION__);
}

/*********************************************/
static struct file_operations akmd_fops = 
{
	.owner = THIS_MODULE,
	.open = akmd_open,
	.release = akmd_release,
	.ioctl = akmd_ioctl,
};

static struct file_operations akm_aot_fops = 
{
	.owner = THIS_MODULE,
	.open = akm_aot_open,
	.release = akm_aot_release,
	.ioctl = akm_aot_ioctl,
};

static struct miscdevice akmd_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm_dev",
	.fops = &akmd_fops,
};

static struct miscdevice akm_aot_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm_aot",
	.fops = &akm_aot_fops,
};

/*********************************************/
static char read_temp_test(void)
{
	char buffer[2] = {0, 0};
	int ret = 0;

	buffer[0] = AK8973_REG_TMPS;

	AKECS_Reset();

	if(0 != AKECS_SetMode_Measure())
	{
		AKMDBG("In %s, AKECS_SetMode_Measure fail");
		return -1;
	}

	mdelay(10);

	ret = AKI2C_RxData(buffer, 1);
	if(0 != ret)
	{
		AKMDBG("In %s, AKI2C_RxData fail");
		return -1;
	}
//	AKMDBG("reg 0xC1 = 0x%x, reg 0xC2 = 0x%x, reg 0xC3 = 0x%x, reg 0xC4 = 0x%x", buffer[1], buffer[2], buffer[3], buffer[4]);
	return buffer[1];
}

static ssize_t compass_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	unsigned char temp_test = read_temp_test();
	sprintf(page, "0x%x\n", temp_test);
}

static void create_compass_proc_file(void)
{
	struct proc_dir_entry *compass_proc_file = create_proc_entry("driver/compass_temp_test", 0644, NULL);

	if (compass_proc_file) 
	{
		compass_proc_file->read_proc = compass_read_proc;
	} 
	else
		printk(KERN_INFO "compass proc file create failed!\n");
}


int akm8973_probe(struct platform_device *pdev)
{
//	struct akm8973_data *akm;
	int err = 0;
	
	AKMDBG("Start %s", __FUNCTION__);
/*
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk(KERN_ERR "AKM8973 akm8973_probe: check_functionality failed.\n");
		err = -ENODEV;
		goto exit0;
	}
*/	
	/* Allocate memory for driver data */
	akm = kzalloc(sizeof(struct akm8973_data), GFP_KERNEL);
	if (!akm) 
	{
		AKMDBG("AKM8973 akm8973_probe: memory allocation failed.");
		err = -ENOMEM;
		goto exit1;
	}

//	INIT_WORK(&akm->work, akm8973_work_func);
//	i2c_set_clientdata(client, akm);
//	(&pdev->dev)->driver_data = akm;
	
	/* Check platform data*/
	if (pdev->dev.platform_data == NULL) 
	{
		AKMDBG("AKM8973 akm8973_probe: platform data is NULL");
		err = -ENOMEM;
		goto exit2;
	}	
	/* Copy to global variable */
	pdata = pdev->dev.platform_data;
//	this_client = client;
//	this_pdev = pdev;
	
	/* Check connection */
	err = AKECS_CheckDevice();
	if (err < 0) 
	{
		AKMDBG("AKM8973 akm8973_probe: set power down mode error\n");
		goto exit3;
	}
	
	/* IRQ */
/*	err = request_irq(client->irq, akm8973_interrupt, IRQ_TYPE_EDGE_RISING,
					  "akm8973_INT", akm);
	if (err < 0) 
	{
		printk(KERN_ERR "AKM8973 akm8973_probe: request irq failed\n");
		goto exit4;
	}
*/	
	/* Declare input device */
	akm->input_dev_ori = input_allocate_device();
	if (!akm->input_dev_ori) 
	{
		err = -ENOMEM;
		AKMDBG("AKM8973 akm8973_probe: Failed to allocate input_dev_ori\n");
		goto exit5;
	}

	akm->input_dev_mag = input_allocate_device();
	if (!akm->input_dev_mag) 
	{
		err = -ENOMEM;
		AKMDBG("AKM8973 akm8973_probe: Failed to allocate input_dev_mag\n");
		goto exit5;
	}

	akm->input_dev_tem = input_allocate_device();
	if (!akm->input_dev_tem) 
	{
		err = -ENOMEM;
		AKMDBG("AKM8973 akm8973_probe: Failed to allocate input_dev_tem\n");
		goto exit5;
	}

	/* Setup input device */
	set_bit(EV_ABS, akm->input_dev_ori->evbit);
	set_bit(EV_ABS, akm->input_dev_mag->evbit);
	set_bit(EV_ABS, akm->input_dev_tem->evbit);
	/* yaw (0, 360) */
	input_set_abs_params(akm->input_dev_ori, ABS_RX, 0, 23040, 0, 0);
	/* pitch (-180, 180) */
	input_set_abs_params(akm->input_dev_ori, ABS_RY, -11520, 11520, 0, 0);
	/* roll (-90, 90) */
	input_set_abs_params(akm->input_dev_ori, ABS_RZ, -5760, 5760, 0, 0);
#if 0
	/* x-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_X, -5760, 5760, 0, 0);
	/* y-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_Y, -5760, 5760, 0, 0);
	/* z-axis acceleration (720 x 8G) */
	input_set_abs_params(akm->input_dev, ABS_Z, -5760, 5760, 0, 0);
#endif
	/* temparature */
	input_set_abs_params(akm->input_dev_tem, ABS_THROTTLE, -30, 85, 0, 0);

	/* status of magnetic sensor */
	input_set_abs_params(akm->input_dev_mag, ABS_RUDDER, -32768, 3, 0, 0);
	/* status of acceleration sensor */
//	input_set_abs_params(akm->input_dev, ABS_WHEEL, -32768, 3, 0, 0);
	/* x-axis of raw magnetic vector (-128, 127) */
	input_set_abs_params(akm->input_dev_mag, ABS_HAT0X, -2048, 2032, 0, 0);
	/* y-axis of raw magnetic vector (-128, 127) */
	input_set_abs_params(akm->input_dev_mag, ABS_HAT0Y, -2048, 2032, 0, 0);
	/* z-axis of raw magnetic vector (-128, 127) */
	input_set_abs_params(akm->input_dev_mag, ABS_BRAKE, -2048, 2032, 0, 0);
	/* Set name */
	akm->input_dev_ori->name = "orientation";
	akm->input_dev_mag->name = "magnetic";
	akm->input_dev_tem->name = "temperature";

//	sensor_input_add(INPUT_ORIENTATION_SENSOR, "compass", AKECS_SetYPR, NULL, NULL, AKECS_SetMode_PowerDown, akm->input_dev);

	/* Register */
	err = input_register_device(akm->input_dev_ori);
	if (err) 
	{
		AKMDBG("AKM8973 akm8973_probe: Unable to register input_dev_ori\n");
		goto exit6;
	}
	err = input_register_device(akm->input_dev_mag);
	if (err) 
	{
		AKMDBG("AKM8973 akm8973_probe: Unable to register input_dev_mag\n");
		goto exit6;
	}
	err = input_register_device(akm->input_dev_tem);
	if (err) 
	{
		AKMDBG("AKM8973 akm8973_probe: Unable to register input_dev_tem\n");
		goto exit6;
	}


	err = misc_register(&akmd_device);
	if (err) 
	{
		AKMDBG("AKM8973 akm8973_probe: akmd_device register failed\n");
		goto exit7;
	}
	
	err = misc_register(&akm_aot_device);
	if (err) 
	{
		AKMDBG("AKM8973 akm8973_probe: akm_aot_device register failed\n");
		goto exit8;
	}
	
	mutex_init(&sense_data_mutex);
	
	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);
	
	/* As default, report all information */
	atomic_set(&m_flag, 0);
	atomic_set(&a_flag, 0);
	atomic_set(&t_flag, 0);
	atomic_set(&mv_flag, 0);

	akm->akm_early_suspend.suspend = akm8973_early_suspend;
	akm->akm_early_suspend.resume = akm8973_early_resume;
	register_early_suspend(&akm->akm_early_suspend);
	
/*
//qmm
	int i = 0;
	while(i < 99)
	{
		if(0 != AKECS_read_data_test())
		{
			AKMDBG("In %s, AKECS_read_data_test fail");
		}
		mdelay(2000);
	}
*/

	create_compass_proc_file();

	AKMDBG("successfully probed.");
	return 0;
	
exit8:
	misc_deregister(&akmd_device);
exit7:
	input_unregister_device(akm->input_dev_ori);
	input_unregister_device(akm->input_dev_mag);
	input_unregister_device(akm->input_dev_tem);
exit6:
	input_free_device(akm->input_dev_ori);
	input_free_device(akm->input_dev_mag);
	input_free_device(akm->input_dev_tem);
exit5:
//	free_irq(client->irq, akm);
exit4:
exit3:
exit2:
	kfree(akm);
exit1:
exit0:
	return err;
}

static int akm8973_remove(struct platform_device *pdev)
{
	AKMDBG("Start %s", __FUNCTION__);
//	struct akm8973_data *akm = (&pdev->dev)->driver_data;
//	AKMFUNC("akm8973_remove");
	unregister_early_suspend(&akm->akm_early_suspend);
	misc_deregister(&akm_aot_device);
	misc_deregister(&akmd_device);
	input_unregister_device(akm->input_dev_ori);
	input_unregister_device(akm->input_dev_mag);
	input_unregister_device(akm->input_dev_tem);
//	free_irq(client->irq, akm);
	kfree(akm);
	AKMDBG("successfully removed.");
	return 0;
}

#if 0
static const struct i2c_device_id akm8973_id[] = 
{
	{AKM8973_I2C_NAME, 0 },
	{ }
};
#endif

//static struct i2c_driver akm8973_driver = 
static struct platform_driver akm8973_driver =
{
	.probe		= akm8973_probe,
	.remove 	= akm8973_remove,
//	.id_table	= akm8973_id,
	.driver = 
	{
		.name = AKM8973_I2C_NAME,
	},
};

static int __init akm8973_init(void)
{
	AKMDBG("AKM8973 compass driver: initialize");
	return platform_driver_register(&akm8973_driver);
}

static void __exit akm8973_exit(void)
{
	AKMDBG("AKM8973 compass driver: release");
	platform_driver_unregister(&akm8973_driver);
}

module_init(akm8973_init);
module_exit(akm8973_exit);

MODULE_AUTHOR("Mozart");
MODULE_DESCRIPTION("AKM8973 compass driver");
MODULE_LICENSE("GPL");

