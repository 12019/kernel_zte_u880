/*
 * driver/sensors/alsps.c 
 *
 * Support CAPELLA CM36xx Short Distance Proximity Sensor with 
 * Ambient Light Sensor
 *
 * Copyright (C) 2008 Borqs, Ltd.
 * Author: Ma Yilei <yilei.ma@borqs.com>
 *         Ru Yi <yi.ru@borqs.com>
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
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <asm/signal.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/completion.h>
#include <linux/sensors.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <mach/alsps.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <asm/atomic.h>
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/sensor-input.h>
#include <linux/i2c.h>
#include "../../i2c/busses/i2c_gpio_pxa.h"

#define TSL2771_DEBUG    0
#if TSL2771_DEBUG
#define TSL2771_DBG(fmt, args...)    printk("TSL2771 : " fmt "\n", ## args)
#else if
#define TSL2771_DBG(fmt, args...)    do {} while (0)
#endif

//qiu
__s32 proximity_changed = 1;
#define OPENED 1
#define CLOSED 0
#define TSL2771_IOCTL_GET_CHIP_ID 	1

//__s32 ambient_status = ALS_CLOSED;
enum
{
	SUSPEND,
	RESUME
};
struct delayed_work psensor_work;
struct workqueue_struct *psensor_wqueue;
struct delayed_work psensor_work_read_data;
struct workqueue_struct *psensor_wqueue_read_data;
u16 psensor_data = 0;

static int ambient_status = CLOSED;
static int proximity_status = CLOSED;
static unsigned char chip_id = 0;

static struct wake_lock alsps_suspend_lock;

struct alsps_struct {
	int irq;
	int adc_event;
	int (*init)(void);
	int (*get_gpio_value)(void);
	void (*setpower)(int enable);
	int (*chip_verify) (int *, int);

	wait_queue_head_t proximity_workq_head;
	wait_queue_head_t ambient_workq_head;
	struct semaphore sem_ambient_interval;
	struct delayed_work ambient_workq;

	
	/* proximity sensor user count */
	int proximity_count;	
	/* ambient light sensor user count */
	int ambient_count;	
	/* ambient light interval */
	int ambient_interval;

	/* alsps sensor switch stat */
	int alsps_switch;	
	/* alsps sensor suspend stat. 1:0 */
	int alsps_suspend;

	struct i2c_client *alsps_client; 	
};

//added by huangxin
static struct input_function
{
	struct input_dev *idev;
};

static struct input_function *in_am;
static struct input_function *in_pro;

static struct alsps_struct *alsps;

#define ALSPS_PROXIMITY_DATA_READY 	0
#define ALSPS_AMBIENT_DATA_READY 	1
#define ALSPS_PROXIMITY_FIRST_POLL	2
#define ALSPS_AMBIENT_FIRST_POLL	3

#define DEFAULT_AMBIENT_INTERVAL	500	
//-----------------------------------------------

#define TSL2271ID 0x72
#define TAOS_DEVICE_ID		"tritonFN"
#define TAOS_ID_NAME_SIZE	10
#define TAOS_MAX_DEVICE_REGS	32

// TRITON register offsets
#define TAOS_TRITON_CNTRL 		0x00
#define TAOS_TRITON_ALS_TIME 		0X01
#define TAOS_TRITON_PRX_TIME		0x02
#define TAOS_TRITON_WAIT_TIME		0x03
#define TAOS_TRITON_ALS_MINTHRESHLO	0X04
#define TAOS_TRITON_ALS_MINTHRESHHI 	0X05
#define TAOS_TRITON_ALS_MAXTHRESHLO	0X06
#define TAOS_TRITON_ALS_MAXTHRESHHI	0X07
#define TAOS_TRITON_PRX_MINTHRESHLO 	0X08
#define TAOS_TRITON_PRX_MINTHRESHHI 	0X09
#define TAOS_TRITON_PRX_MAXTHRESHLO 	0X0A
#define TAOS_TRITON_PRX_MAXTHRESHHI 	0X0B
#define TAOS_TRITON_INTERRUPT		0x0C
#define TAOS_TRITON_PRX_CFG		0x0D
#define TAOS_TRITON_PRX_COUNT		0x0E
#define TAOS_TRITON_GAIN		0x0F
#define TAOS_TRITON_REVID		0x11
#define TAOS_TRITON_CHIPID      	0x12
#define TAOS_TRITON_STATUS		0x13
#define TAOS_TRITON_ALS_CHAN0LO		0x14
#define TAOS_TRITON_ALS_CHAN0HI		0x15
#define TAOS_TRITON_ALS_CHAN1LO		0x16
#define TAOS_TRITON_ALS_CHAN1HI		0x17
#define TAOS_TRITON_PRX_LO		0x18
#define TAOS_TRITON_PRX_HI		0x19
#define TAOS_TRITON_TEST_STATUS		0x1F

// Triton cmd reg masks
#define TAOS_TRITON_CMD_REG		0X80
#define TAOS_TRITON_CMD_BYTE_RW		0x00
#define TAOS_TRITON_CMD_WORD_BLK_RW	0x20
#define TAOS_TRITON_CMD_SPL_FN		0x60
#define TAOS_TRITON_CMD_PROX_INTCLR	0X05
#define TAOS_TRITON_CMD_ALS_INTCLR	0X06
#define TAOS_TRITON_CMD_PROXALS_INTCLR 	0X07
#define TAOS_TRITON_CMD_TST_REG		0X08
#define TAOS_TRITON_CMD_USER_REG	0X09

// Triton cntrl reg masks
#define TAOS_TRITON_CNTL_PROX_INT_ENBL	0X20
#define TAOS_TRITON_CNTL_ALS_INT_ENBL	0X10
#define TAOS_TRITON_CNTL_WAIT_TMR_ENBL	0X08
#define TAOS_TRITON_CNTL_PROX_DET_ENBL	0X04
#define TAOS_TRITON_CNTL_ADC_ENBL	0x02
#define TAOS_TRITON_CNTL_PWRON		0x01

// Triton status reg masks
#define TAOS_TRITON_STATUS_ADCVALID	0x01
#define TAOS_TRITON_STATUS_PRXVALID	0x02
#define TAOS_TRITON_STATUS_ADCINTR	0x10
#define TAOS_TRITON_STATUS_PRXINTR	0x20

// lux constants
#define	TAOS_MAX_LUX			65535000
#define TAOS_SCALE_MILLILUX		3
#define TAOS_FILTER_DEPTH		3

// ioctls

#define TAOS_IOCTL_MAGIC        	0XCF
#define TAOS_IOCTL_ALS_ON       	 1
#define TAOS_IOCTL_ALS_OFF      	 2
#define TAOS_IOCTL_ALS_DATA     	3
#define TAOS_IOCTL_ALS_CALIBRATE	4
#define TAOS_IOCTL_CONFIG_GET   	5
#define TAOS_IOCTL_CONFIG_SET		6
#define TAOS_IOCTL_PROX_ON		7
#define TAOS_IOCTL_PROX_OFF		8
#define TAOS_IOCTL_PROX_DATA		9
#define TAOS_IOCTL_PROX_EVENT       10
#define TAOS_IOCTL_PROX_CALIBRATE	11
#define TAOS_IOCTL_PROX_CAL_GET		12
#define TAOS_IOCTL_PROX_CAL_SET     13

// per-device data
struct taos_data {
	unsigned int addr;
	char taos_id;
	char taos_name[TAOS_ID_NAME_SIZE];
	struct semaphore update_lock;
	char valid;
	unsigned long last_updated;
} *taos_datap;

// lux time scale
struct time_scale_factor  {
	u16	numerator;
	u16	denominator;
	u16	saturation;
};

// device configuration
struct taos_cfg {
        u32     calibrate_target;
        u16     als_time;
        u16     scale_factor;
        u16     gain_trim;
        u8      filter_history;
        u8      filter_count;
        u8      gain;
	u16	prox_threshold;
};

// proximity data
struct taos_prox_info {
        u16 prox_clear;
        u16 prox_data;
        int prox_event;
};

// lux data
struct lux_data {
	u16	ratio;
	u16	clear;
	u16	ir;
};


// forward declarations
static int taos_ioctl(unsigned int cmd, char *arg) ;
static int taos_get_lux(void);
static int taos_lux_filter(int raw_lux);
static int taos_prox_poll(struct taos_prox_info *prxp);

// device configuration
struct taos_cfg *taos_cfgp;

// initial config parameters
static u32 calibrate_target_param = 300000;
static u16 als_time_param = 300;
static u16 scale_factor_param = 1;
static u16 gain_trim_param = 512;
static u8 filter_history_param = 3;
static u8 filter_count_param = 1;
static u8 gain_param = 1;
static u16 prox_threshold_param_hi = 0;
static u16 prox_threshold_param_lo = 0;
static u16 standard_prox_threshold_param_hi = 600;
static u16 standard_prox_threshold_param_lo = 550;
static int aaa = 0;
static int prox_adjust = 0;

//static int prox_max_value = 0;
//static int prox_min_value = 1023;
//static int prox_thres_value = 0;
// prox info
struct taos_prox_info prox_cal_info[20];
struct taos_prox_info prox_cur_info;
struct taos_prox_info *prox_cur_infop = &prox_cur_info;
static u8 prox_history = 0;

// device reg init values
u8 taos_triton_reg_init[16] = {0x00,0xFF,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0X00,0X00,0X00,0X00};

struct time_scale_factor TritonTime = {1, 0, 0};
struct time_scale_factor *lux_timep = &TritonTime;

// gain table
u8 taos_triton_gain_table[] = {1, 8, 16, 120};


struct lux_data TritonFN_lux_data[] = {
        { 9830,  8320,  15360 },
        { 12452, 10554, 22797 },
        { 14746, 6234,  11430 },
        { 17695, 3968,  6400  },
        { 0,     0,     0     }
};
struct lux_data *lux_tablep = TritonFN_lux_data;
static int lux_history[TAOS_FILTER_DEPTH] = {-ENODATA, -ENODATA, -ENODATA};

static spinlock_t alsps_i2c_lock=SPIN_LOCK_UNLOCKED;


/*i2c_smbus_write_byte_data*/
int taos_i2c_write_byte_data(u8 command, u8 value)
{
#if 0
	    unsigned long flags = 0;
            int err;
            spin_lock_irqsave(&alsps_i2c_lock, flags);
	    err = gpio_request(SCL_GPIO_NUM, "VIRTUAL-SCL");
            if(err)
            {
            TSL2771_DBG("failed to request GPIO20 for USB-UART\n");
            return -1;
            }

            err = gpio_request(SDA_GPIO_NUM, "VIRTUAL-SDA");
            if(err)
            {
            TSL2771_DBG("failed to request GPIO20 for USB-UART\n");
            return -1;
            }

	
		COMMON_I2C_mutx_down();
		COMMON_I2C_Reset();
		COMMON_I2C_Start();
//qiu
//		TSL2771_DBG("In write, command is 0x%x, value is 0x%x\n", command, value);

		if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(TSL2271ID |I2C_WR))
		{
			TSL2771_DBG("ERROR %s %d",__FUNCTION__,1);
			COMMON_I2C_Stop();
			COMMON_I2C_mutx_up();
	              spin_unlock_irqrestore(&alsps_i2c_lock, flags);
			return -1;
		}
		
		if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
		{
			TSL2771_DBG("ERROR %s %d",__FUNCTION__,2);
			COMMON_I2C_Stop();
			COMMON_I2C_mutx_up();
	                spin_unlock_irqrestore(&alsps_i2c_lock, flags);
			return -1;
		}

		if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(value))
		{
			TSL2771_DBG("ERROR %s %d",__FUNCTION__,2);
			COMMON_I2C_Stop();
			COMMON_I2C_mutx_up();
	                spin_unlock_irqrestore(&alsps_i2c_lock, flags);
			return -1;
		}
		
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();

        gpio_free(SCL_GPIO_NUM);
        gpio_free(SDA_GPIO_NUM);
	    spin_unlock_irqrestore(&alsps_i2c_lock, flags);

		return 0;
#endif

	int ret;

	if (alsps->alsps_client == NULL)
		return -1;

	ret = i2c_smbus_write_byte_data(alsps->alsps_client, command, value);
	if(ret == 0)
		ret = 0;
	else
		ret = -EIO;

	return ret;
}

u8 taos_i2c_read_byte_data(u8 command)
{
#if 0
	char data = 0;
	unsigned long flags;

    int err;
    spin_lock_irqsave(&alsps_i2c_lock, flags);

    err = gpio_request(SCL_GPIO_NUM, "VIRTUAL-SCL");
    if (err)
    {
        TSL2771_DBG("failed to request GPIO20 for USB-UART\n");
        return -1;
     }

     err = gpio_request(SDA_GPIO_NUM, "VIRTUAL-SDA");
     if (err)
     {
         TSL2771_DBG("failed to request GPIO20 for USB-UART\n");
         return -1;
      }
	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(TSL2271ID |I2C_WR))
	{
		TSL2771_DBG("ERROR %s %d",__FUNCTION__,1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	        spin_unlock_irqrestore(&alsps_i2c_lock, flags);
		return -1;
	}
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		TSL2771_DBG("ERROR %s %d",__FUNCTION__,2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	        spin_unlock_irqrestore(&alsps_i2c_lock, flags);
		return -1;
	}
	
	COMMON_I2C_Restart();

	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(TSL2271ID |I2C_RD))
	{
		TSL2771_DBG("ERROR %s %d",__FUNCTION__,1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
	        spin_unlock_irqrestore(&alsps_i2c_lock, flags);
		return -1;
	}

	data = COMMON_I2C_Receive_Byte(1);

	COMMON_I2C_Stop();
	COMMON_I2C_mutx_up();
        
        gpio_free(SCL_GPIO_NUM);
        gpio_free(SDA_GPIO_NUM);
        spin_unlock_irqrestore(&alsps_i2c_lock, flags);

	return data;
#endif

	int ret;
	
	if (alsps->alsps_client == NULL)
		return -EINVAL;

	ret = i2c_smbus_read_byte_data(alsps->alsps_client, command);

	return ret;
}

int taos_i2c_write_special_data(u8 command)
{
	   unsigned long flags = 0;
           int err;
           spin_lock_irqsave(&alsps_i2c_lock, flags);
           err = gpio_request(SCL_GPIO_NUM, "VIRTUAL-SCL");
            if(err)
            {
            TSL2771_DBG("failed to request GPIO20 for USB-UART\n");
            return -1;
            }

            err = gpio_request(SDA_GPIO_NUM, "VIRTUAL-SDA");
            if(err)
            {
            TSL2771_DBG("failed to request GPIO20 for USB-UART\n");
            return -1;
            }

	COMMON_I2C_mutx_down();
	COMMON_I2C_Reset();
	COMMON_I2C_Start();
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(TSL2271ID |I2C_WR))
	{
		TSL2771_DBG("ERROR %s %d",__FUNCTION__,1);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
                spin_unlock_irqrestore(&alsps_i2c_lock, flags);
		return -1;
	}
	
	if(COMMON_I2C_ERROR == COMMON_I2C_Send_Byte(command))
	{
		TSL2771_DBG("ERROR %s %d",__FUNCTION__,2);
		COMMON_I2C_Stop();
		COMMON_I2C_mutx_up();
                spin_unlock_irqrestore(&alsps_i2c_lock, flags);
		return -1;
	}

	COMMON_I2C_Stop();
	COMMON_I2C_mutx_up();

        gpio_free(SCL_GPIO_NUM);
        gpio_free(SDA_GPIO_NUM);
	spin_unlock_irqrestore(&alsps_i2c_lock, flags);

	return 0;
}

static int taos_ioctl(unsigned int cmd, char *arg) 
{
	int prox_sum = 0, prox_mean = 0, prox_max = 0;
	int lux_val = 0, ret = 0, i = 0, tmp = 0;
	u16 gain_trim_val = 0;
	u8 itime = 0, reg_val = 0, reg_cntrl = 0;

	TSL2771_DBG("asensor,cmd=%d",cmd);

	switch (cmd)
	{
		case TAOS_IOCTL_ALS_ON:
            TSL2771_DBG("TAOS_IOCTL_ALS_ON");
			for (i = 0; i < TAOS_FILTER_DEPTH; i++)
			{
				lux_history[i] = -ENODATA;
			}
#if 0
			for (i = 0; i < sizeof(taos_triton_reg_init); i++)
			{
				if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i)), 
					taos_triton_reg_init[i])) < 0) 
				{
                    TSL2771_DBG("write TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL+i) Failed \n");
		            return (ret);
		        }
			}
#endif

			itime = (((taos_cfgp->als_time/50) * 18) - 1);
			
			itime = (~itime);
			if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), itime))<0)
			{
                          TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                         return (ret);
	        }
			if((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x03), 0xF2))) < 0)
			{
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

/*
			if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_INTERRUPT), 0x03))<0)
			{
                          TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                          return (ret);
	        }


			if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_MINTHRESHLO), 0))<0)
		{
                         TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                          return (ret);
	        }
			if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_MINTHRESHHI), 0))<0)
			{
                          TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                          return (ret);
	        }
			if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_MAXTHRESHLO), 0))<0)
			{
                         TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                         return (ret);
	        }
			if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_MAXTHRESHHI), 0))<0)
			{
                        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                        return (ret);
	        }
*/
			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN);
                        TSL2771_DBG("TAOS: i2c read reg_val = %x\n", reg_val);
			reg_val = reg_val & 0xFC;
                        TSL2771_DBG("TAOS: new reg_val = %x\n", reg_val);

			if ((ret=taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_GAIN), reg_val))<0)
			{
                         TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                         return (ret);
	        }



			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
			if (reg_val & TAOS_TRITON_CNTL_PWRON)
			{
				reg_cntrl = TAOS_TRITON_CNTL_ADC_ENBL;
			}
			else
			{
				reg_cntrl = (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON);
			}

//			if((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), 0x8|reg_val|reg_cntrl|0x10))<0)
			if((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_val | 0x0B))<0)
			{
				TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
				return (ret);
			}
			TSL2771_DBG("^^^^^^^^^^^^^^^^^^^^^^^^^^ 0x%x ^^^^^^^^^^^^^^^^^^^^^^^\n", taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL));
//			TSL2771_DBG("open ALS reg_val = 0x%x\n",reg_val);
			return (ret);
			break;

		case TAOS_IOCTL_ALS_OFF:
                 
                       TSL2771_DBG("TAOS_IOCTL_ALS_OFF\n");
                       for (i = 0; i < TAOS_FILTER_DEPTH; i++)
		       {
            	       lux_history[i] = -ENODATA;
		       }

			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
			if (reg_val & TAOS_TRITON_CNTL_PROX_DET_ENBL)
			{
				reg_cntrl = reg_val & (~TAOS_TRITON_CNTL_ADC_ENBL);
			}
			else
			{
				reg_cntrl = reg_val & (~(TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON));
			}

//			if ((ret=taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_cntrl))<0)
			if ((ret=taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL), reg_val & 0xFD))<0)
			{
                        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_off\n");
                        return (ret);
            }

//			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
//			TSL2771_DBG("close ALS reg_val = 0x%x\n",reg_val);
			return (ret);
                break;
			
		case TAOS_IOCTL_ALS_DATA:
                        TSL2771_DBG("TAOS_IOCTL_ALS_DATA\n");
			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
                        if ((reg_val & (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
				!= (TAOS_TRITON_CNTL_ADC_ENBL | TAOS_TRITON_CNTL_PWRON))
                        return -ENODATA;

			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS);
			if ((reg_val & TAOS_TRITON_STATUS_ADCVALID) != TAOS_TRITON_STATUS_ADCVALID)
				return -ENODATA;

			if ((lux_val = taos_get_lux()) < 0)
			   TSL2771_DBG( "TAOS: call to taos_get_lux() returned error %d in ioctl als_data\n", lux_val);
			
			return (lux_val);
			break;
			
		case TAOS_IOCTL_ALS_CALIBRATE:
                        TSL2771_DBG("TAOS_IOCTL_ALS_CALIBRATE\n");
			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
			if ((reg_val & 0x07) != 0x07)
				return -ENODATA;
			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_STATUS);
			if ((reg_val & 0x01) != 0x01)
                          return -ENODATA;

                        if ((lux_val = taos_get_lux()) < 0) 
			{
                        TSL2771_DBG( "TAOS: call to lux_val() returned error %d in ioctl als_data\n", lux_val);
                        return (lux_val);
			}

			gain_trim_val = (u16)(((taos_cfgp->calibrate_target) * 512)/lux_val);
			taos_cfgp->gain_trim = (int)gain_trim_val;
			return ((int)gain_trim_val);
			break;
			
		case TAOS_IOCTL_CONFIG_GET:
            TSL2771_DBG("TAOS_IOCTL_CONFIG_GET\n");
			ret = copy_to_user((struct taos_cfg *)arg, taos_cfgp, sizeof(struct taos_cfg));
			if (ret)
			{
				TSL2771_DBG( "TAOS: copy_to_user failed in ioctl config_get\n");
				return -ENODATA;
			}
			return (ret);
			break;
			
               case TAOS_IOCTL_CONFIG_SET:
                       TSL2771_DBG("TAOS_IOCTL_CONFIG_SET\n");
                       ret = copy_from_user(taos_cfgp, (struct taos_cfg *)arg, sizeof(struct taos_cfg));
			if (ret) 
			{
				TSL2771_DBG( "TAOS: copy_from_user failed in ioctl config_set\n");
                                return -ENODATA;
			}
    		if(taos_cfgp->als_time < 50)
	               	taos_cfgp->als_time = 50;
			if(taos_cfgp->als_time > 650)
		            taos_cfgp->als_time = 650;
			tmp = (taos_cfgp->als_time + 25)/50;
    		taos_cfgp->als_time = tmp*50;
            	break;
				
		case TAOS_IOCTL_PROX_ON:

			TSL2771_DBG("TAOS_IOCTL_PROX_ON\n");
/*
			if((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x01), 0xF6))) < 0) 
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}
*/
			if((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x02), 0xFF))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

            if((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x03), 0xF2))) < 0)
			{
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

//qiu-threshold
#if 0
			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x08), 0))) < 0)
			{
                        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                        return (ret);
			}
                        TSL2771_DBG("========> 3 is 0x%x!!!!!!!!!!", taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x08));

			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x09), 0))) < 0)
			{
                        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                        return (ret);
			}
                        TSL2771_DBG("========> 4 is 0x%x!!!!!!!!!!", 
                        taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x09));
                        //0x40
			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0A), 0x40))) < 0)
			{
                        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                        return (ret);
			}
                        TSL2771_DBG("========> 5 is 0x%x!!!!!!!!!!", taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0A));
                        //0x2F
			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0B), 0x18))) < 0)
			{
                        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                        return (ret);
			}
			TSL2771_DBG("========> 6 is 0x%x!!!!!!!!!!", taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0B));
#endif
			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0C);
            if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0C), reg_val|0x30))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}
            TSL2771_DBG("========> 7 is 0x%x!!!!!!!!!!", taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0C));

            if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0D), 0x00))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}
            TSL2771_DBG("========> 8 is 0x%x!!!!!!!!!!", taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0D));
            
            if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0E), 0x06))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}
            TSL2771_DBG("========> 9 is 0x%x!!!!!!!!!!", taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0E));
//6.10
			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0F);
			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0F), reg_val | 0x20))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}
            TSL2771_DBG("========> 10 is 0x%x!!!!!!!!!!", taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0F));

			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
			TSL2771_DBG("*#*#*contrl reg_val = 0x%x#*#*#\n", reg_val);
//			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x00), (0x2D))) < 0))
			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x00), (0x0D | reg_val))) < 0))
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
               	return (ret);
            }
			break;
						
        case TAOS_IOCTL_PROX_OFF:

			TSL2771_DBG("TAOS_IOCTL_PROX_OFF\n");

			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x00), (reg_val & 0xFB)))) < 0)
			{
                return (ret);
            }

			itime = (((taos_cfgp->als_time/50) * 18) - 1);
			
			itime = (~itime);
			if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), itime))<0)
			{
                          TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                         return (ret);
	        }
			if((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x03), 0xF2))) < 0)
			{
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}


//6.26
/*
			itime = (((taos_cfgp->als_time/50) * 18) - 1);
			itime = (~itime);
			if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), itime))<0)
			{
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl als_on\n");
                return (ret);
	        }
*/
//			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
//			TSL2771_DBG("***close PROX reg_val = 0x%x***\n", reg_val);
            break;
				
		case TAOS_IOCTL_PROX_DATA:
			// TBD - remove this call to prox_poll when kernel timer implemented
			if ((ret = taos_prox_poll(prox_cur_infop)) < 0) 
			{
				//TSL2771_DBG( "TAOS: call to prox_poll failed in ioctl prox_data\n");
				return (ret);
			}
            ret = copy_to_user((struct taos_prox_info *)arg, prox_cur_infop, sizeof(struct taos_prox_info));
            if (ret) 
			{
               // TSL2771_DBG( "TAOS: copy_to_user failed in ioctl prox_data\n");
                return -ENODATA;
            }
            return (ret);
			
            break;
			
        case TAOS_IOCTL_PROX_EVENT:
            // TBD - remove this call to prox_poll when kernel timer implemented
            if ((ret = taos_prox_poll(prox_cur_infop)) < 0) 
			{
//qiu
			//	TSL2771_DBG("TAOS: call to prox_poll failed in ioctl prox_event\n");

                    TSL2771_DBG( "TAOS: call to prox_poll failed in ioctl prox_event\n");
                    return (ret);
            }
 			return (prox_cur_infop->prox_event);
            break;
			
		case TAOS_IOCTL_PROX_CALIBRATE:
			// TBD - stop polling kernel timer
			prox_sum = 0;
			prox_max = 0;
			for (i = 0; i < 20; i++)
			{
                if ((ret = taos_prox_poll(&prox_cal_info[i])) < 0) 
				{
                    TSL2771_DBG( "TAOS: call to prox_poll failed in ioctl prox_calibrate\n");
	                return (ret);
            	}
				prox_sum += prox_cal_info[i].prox_data;
				if (prox_cal_info[i].prox_data > prox_max)
					prox_max = prox_cal_info[i].prox_data;
				mdelay(100);
			}
			prox_mean = prox_sum/20;
			taos_cfgp->prox_threshold = ((((prox_max - prox_mean) * 170) + 50)/100) + prox_mean;
			// TBD - restart polling kernel timer
			break;
		case TAOS_IOCTL_PROX_CAL_GET:
			return (taos_cfgp->prox_threshold);
			break;
			
		case TAOS_IOCTL_PROX_CAL_SET:
			if ((arg < 0) || (arg > 65535))
				return -EINVAL;
			else
				taos_cfgp->prox_threshold = (u16)arg;
			break;
			
		default:
			return -EINVAL;
			break;
	}
	return (ret);
}

// read/calculate lux value
static int taos_get_lux(void)
{
    u16 raw_clear = 0, raw_ir = 0, raw_lux = 0;
	u32 lux = 0;
	u32 ratio = 0;
	u8 dev_gain = 0;
	struct lux_data *p;
	u8 chdata[4];
	int tmp = 0, i = 0;

	for (i = 0; i < 4; i++) 
	{
		chdata[i] = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i));
	}

//	TSL2771_DBG("chdata[0]= %d chdata[1]= %d chdata[2]= %d chdata[3]= %d \n",chdata[0],chdata[1],chdata[2],chdata[3]);
	tmp = (taos_cfgp->als_time + 25)/50;
    TritonTime.numerator = 1;
    TritonTime.denominator = tmp;

    tmp = 300 * taos_cfgp->als_time;
    if(tmp > 65535)
            tmp = 65535;
    TritonTime.saturation = tmp;
	raw_clear = chdata[1];
	raw_clear <<= 8;
	raw_clear |= chdata[0];
	raw_ir    = chdata[3];
	raw_ir    <<= 8;
	raw_ir    |= chdata[2];
	if(raw_ir > raw_clear) 
	{
		raw_lux = raw_ir;
		raw_ir = raw_clear;
		raw_clear = raw_lux;
	}
	raw_clear *= taos_cfgp->scale_factor;	
	raw_ir *= taos_cfgp->scale_factor;
	dev_gain = taos_triton_gain_table[taos_cfgp->gain & 0x3];
    if(raw_clear >= lux_timep->saturation)
            return(TAOS_MAX_LUX);
    if(raw_ir >= lux_timep->saturation)
            return(TAOS_MAX_LUX);
    if(raw_clear == 0)
            return(0);
    if(dev_gain == 0 || dev_gain > 127) 
	{
		TSL2771_DBG( "TAOS: dev_gain = 0 or > 127 in taos_get_lux()\n");
                return -1;
	}
	
    if(lux_timep->denominator == 0)
	{
		TSL2771_DBG( "TAOS: lux_timep->denominator = 0 in taos_get_lux()\n");
                return -1;
	}
	ratio = (raw_ir<<15)/raw_clear;
	for (p = lux_tablep; p->ratio && p->ratio < ratio; p++);
        if(!p->ratio)
                return 0;
	lux = ((raw_clear*(p->clear)) - (raw_ir*(p->ir)));
	lux = ((lux + (lux_timep->denominator >>1))/lux_timep->denominator) * lux_timep->numerator;
	lux = (lux + (dev_gain >> 1))/dev_gain;
	lux >>= TAOS_SCALE_MILLILUX;
        if(lux > TAOS_MAX_LUX)
                lux = TAOS_MAX_LUX;

	return(lux);
}

static int taos_lux_filter(int lux)
{
        static u8 middle[] = {1,0,2,0,0,2,0,1};
        int index = 0;

        lux_history[2] = lux_history[1];
        lux_history[1] = lux_history[0];
        lux_history[0] = lux;
        if((lux_history[2] < 0) || (lux_history[1] < 0) || (lux_history[0] < 0))
			return -ENODATA;
        index = 0;
        if( lux_history[0] > lux_history[1] ) index += 4;
        if( lux_history[1] > lux_history[2] ) index += 2;
        if( lux_history[0] > lux_history[2] ) index++;

//		TSL2771_DBG("In taos_lux_filter, lux_history[middle[index]] = %d", lux_history[middle[index]]);
        return(lux_history[middle[index]]);
}

// proximity poll
static int taos_prox_poll(struct taos_prox_info *prxp)
{
    u16 status = 0;
    int i = 0, ret = 0, wait_count = 0, event = 0, actual_event = 1;
    u8 chdata[6] = {0, 0, 0, 0, 0, 0};

    status = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x13);

	while((status & 0x22) != 0x22)
	{
       	status = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x13);
        wait_count++;
        if (wait_count > 10) 
		{
//qiu
			TSL2771_DBG("***status = 0x%x***\n", status);

                return -ENODATA;
        }
        mdelay(10);
    }

//qiu
	TSL2771_DBG("***status = 0x%x***\n", status);

    for (i = 0; i < 6; i++) 
	{
		chdata[i] = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i));
    }
    prxp->prox_clear = chdata[1];
    prxp->prox_clear <<= 8;
    prxp->prox_clear |= chdata[0];
    if (prxp->prox_clear > 3240)
    	return -ENODATA;

	TSL2771_DBG("psensor_data = 0x%x", psensor_data);
	TSL2771_DBG("taos_cfgp->prox_threshold = 0x%x", taos_cfgp->prox_threshold);

    if (psensor_data > taos_cfgp->prox_threshold)
	{
		TSL2771_DBG("The command will be suspend\n");
        prxp->prox_event = SUSPEND;
	}
    else 
	{
		TSL2771_DBG("The command will be resume\n");
		prxp->prox_event = RESUME;
	}

    return (ret);
}

//------------------------------------------------

struct alsps_stat {
	struct list_head list;
	unsigned long status;
};
static LIST_HEAD(proximity_process_list);
static LIST_HEAD(ambient_process_list);
static struct semaphore sem_ambient_list;

static unsigned int previous_lumen = 0;
//static unsigned int current_lumen = 0;

static int tsl2771_proximity_read();
static int tsl2771_ambient_read();

/* 
 * Proximity sensor operations 
 */
static void proximity_handler(void)
{
	struct alsps_stat *ptr;
	TSL2771_DBG("Come into proximity_handler!\n");

	if (alsps->proximity_count > 0)
	{
		list_for_each_entry(ptr, &proximity_process_list, list) 
		{
			set_bit(ALSPS_PROXIMITY_DATA_READY, &ptr->status);
		}
		//TSL2771_DBG("In proximity_handler, wake up workq!\n");
                TSL2771_DBG("proximity start Read the data\n");
//                tsl2771_proximity_read();
	//	wake_up(&alsps->proximity_workq_head);
	}
}

static void als_threshold_modify(void)
{
	TSL2771_DBG("******come into %s******", __FUNCTION__);
	u8 adc0_l, adc0_h, als_threshold_lo_lo, als_threshold_lo_hi, als_threshold_hi_lo, als_threshold_hi_hi;
	u16 ch0, als_threshold_hi, als_threshold_lo;

	adc0_l = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_CHAN0LO);
	adc0_h = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_CHAN0HI);
	ch0 = adc0_l + adc0_h * 256;

	TSL2771_DBG("******ch0 = %d******", ch0);

	if(ch0 < 26)
	{
		als_threshold_hi = 50;
		als_threshold_lo = 0;
	}
	else
	{
		als_threshold_hi = ch0 + ch0/5;
		als_threshold_lo = ch0 - ch0/5;
	}

	als_threshold_hi_hi = als_threshold_hi >> 8;
	als_threshold_hi_lo = als_threshold_hi & 0x00FF;
	als_threshold_lo_hi = als_threshold_lo >> 8;
	als_threshold_lo_lo = als_threshold_lo & 0x00FF;
	
	if(taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | 0x04), als_threshold_lo_lo) < 0)
	{
		TSL2771_DBG("error 1");
		return;
	}
	if(taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | 0x05), als_threshold_lo_hi) < 0)
	{
		TSL2771_DBG("error 2");
		return;
	}
	if(taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | 0x06), als_threshold_hi_lo) < 0)
	{
		TSL2771_DBG("error 3");
		return;
	}
	if(taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | 0x07), als_threshold_hi_hi) < 0)
	{
		TSL2771_DBG("error 4");
		return;
	}

	TSL2771_DBG("******End %s******", __FUNCTION__);
}


static void ambient_handler(void)
{
	TSL2771_DBG("**********come into %s*********", __FUNCTION__);
	struct alsps_stat *ptr;

	if (!alsps) 
	{
		TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
		als_threshold_modify();
		return;
	}

	if(alsps->alsps_switch & AMBIENT_ENABLE)
	{
		down(&sem_ambient_list);

		list_for_each_entry(ptr, &ambient_process_list, list) 
		{
			set_bit(ALSPS_AMBIENT_DATA_READY, &ptr->status);
		}
		up(&sem_ambient_list);
                //===>will add read
                TSL2771_DBG("ambient start read the data\n");
                tsl2771_ambient_read();
	//	wake_up(&alsps->ambient_workq_head);
	}
}


static void tsl2771_psensor_work(struct work_struct *work)
{
	u8 i = 0;
	TSL2771_DBG("*********come into isr*********\n");
        int level;
        level = __gpio_get_value(11);
        TSL2771_DBG("In delay work, electric level is %d\n", level);
//0701
	u8 chdata[6] = {0, 0, 0, 0, 0, 0};
	u16 value;
	for (i = 0; i < 6; i++) 
	{
		chdata[i] = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | (TAOS_TRITON_ALS_CHAN0LO + i));
        }
    value = chdata[5];
    value <<= 8;
    value |= chdata[4];
	TSL2771_DBG("value ============================= 0x%x\n", value);
	psensor_data = value;
        
     /* added by HuangXin */
     

	i = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x13);
	TSL2771_DBG("status register is 0x%x", i);

       

	if((i & 0x22) == 0x22)
	{
		TSL2771_DBG("\n******This is PROX interrupt******");
		goto prox_handle;
	}
	else if((i & 0x11) == 0x11)
	{
		TSL2771_DBG("\n######This is ALS interrupt######");
		goto als_handle;
	}
	else
	{
		TSL2771_DBG("Error interrupt, return");
		taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		return;
	}
	

prox_handle:
	if (alsps->alsps_switch & PROXIMITY_ENABLE)
	{

		TSL2771_DBG("################ alsps->alsps_suspend is %d #################\n", alsps->alsps_suspend);
//6.19

//7.10
		if(alsps->alsps_suspend == 0)
		{
			if(value < taos_cfgp->prox_threshold)
			{
				TSL2771_DBG("\n\n\n######!!!!!!!!!!!!Read error data!!!!!!!!!!\n\n");
				taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
				return;
			}
		}
		
		if(alsps->alsps_suspend == 1)
		{
			if(value > taos_cfgp->prox_threshold)
			{

				TSL2771_DBG("P sensor suspend222222, now change threshold\n");
				if ((taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x08), 0x40)) < 0)
				{
    				TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
    				return;
				}			
				if ((taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x09), 0x18)) < 0)
				{
        			TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        			return;
				}
				if ((taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0A), 0xFF)) < 0)
				{
        			TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        			return;
				}
				if ((taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0B), 0xFF)) < 0)
				{
        			TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        			return;
				}
				TSL2771_DBG("In suspend, threshold change222222 ok\n");


				TSL2771_DBG("Suspend two times, return\n");
				taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
				return;
			}
 		}
               
		proximity_handler();//tmp delete by huangxin
//		wake_lock_timeout(&alsps_suspend_lock, HZ/2);
	} 
	return;



als_handle:
		ambient_handler();
}


static irqreturn_t proximity_isr(int irq, void * dev_id)
{
	TSL2771_DBG("TSL2771: proximity_isr start\n");
        int level = 0;
        level = __gpio_get_value(11);
        TSL2771_DBG("TSL2771: ISR NOW Electric level is %d\n", level);
        wake_lock_timeout(&alsps_suspend_lock, 2 * HZ);
           
	queue_delayed_work(psensor_wqueue, &psensor_work, msecs_to_jiffies(1));

	return IRQ_HANDLED;
}


//6.10
struct timer_list alsps_timer_after_proximity_close;
enum
{
	TIMER_ON,
	TIMER_OFF
};
int timer_status = TIMER_OFF;
static void alsps_after_proximity_close_timer_handle(void)
{
	TSL2771_DBG("Start %s", __FUNCTION__);
	timer_status = TIMER_OFF;
	TSL2771_DBG("End %s", __FUNCTION__);
	return;
}


//static int tsl2771_proximity_read()
static void proximity_report(struct input_dev *idev)
{
	struct input_event data;
	struct sensor_input_dev* sensor;
	sensor = dev_get_drvdata(&idev->dev);
	unsigned char reg18 = 0, reg19 = 0;
	unsigned char status = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);

	if((status & 0x4) == 0x4) 
	{
		get_monotonic_time(&(data.time));
        data.type = EV_ABS;
        data.code = ABS_DISTANCE;

		data.value = i2c_smbus_read_word_data(alsps->alsps_client, 0x18 | 0x80);

		TSL2771_DBG("******** Now report pro data = %d ********", data.value);
/*
		if(data.value < prox_min_value || data.value > prox_max_value)
		{
			if(data.value < prox_min_value)
				prox_min_value = data.value;
			if(data.value > prox_max_value)
				prox_max_value = data.value;

			if(prox_min_value < 400)
			{
				if(prox_max_value < 1000)
				{
					prox_thres_value = prox_min_value + 160;
				}
				else
				{
					prox_thres_value = prox_max_value - 150;
				}
			}
			else
			{
				if(prox_max_value < 1000)
				{
					prox_thres_value = prox_min_value + 260;
					if(prox_thres_value > 1023)
						prox_thres_value = 1023;
				}
				else
				{
					prox_thres_value = 1000;
				}
			}
			TSL2771_DBG("prox_thres_value = %d\n", prox_thres_value);
		}
*/
		if(data.value >= prox_threshold_param_hi)
		{
//			TSL2771_DBG("report 3");
        	input_report_abs(idev, ABS_DISTANCE, 3);
		}
		if(data.value < prox_threshold_param_hi)
		{
//			TSL2771_DBG("report 100");
        	input_report_abs(idev, ABS_DISTANCE, 100);
		}
        input_sync(idev);
	}
	return;
}

static void ambient_report(struct input_dev *idev)
{
	struct sensor_input_dev* sensor;
	sensor = dev_get_drvdata(&idev->dev);
	struct input_event data;

	unsigned char status = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);

	if ((status & 0x2) == 0x2) 
	{
        data.value = taos_get_lux();
//		TSL2771_DBG("####### Now report ami data #######, data = %d", data.value);
		if(aaa == 0)
		{
			input_report_abs(idev, ABS_PRESSURE, data.value);
			aaa = 1;
		}
		if(aaa == 1)
		{
			input_report_abs(idev, ABS_PRESSURE, (data.value + 1));
			aaa = 0;
		}
        input_sync(idev);
	} 
	return;
}

static ssize_t proximity_read(struct file *filep, char __user *buff, size_t size, loff_t *offset)
{
//qiu
	int ret;
	struct input_event  data;
	struct alsps_stat *ptr;

	TSL2771_DBG("Start %s", __FUNCTION__);
	
	if (!alsps) 
	{
		TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
		return -1;
	}

	ptr = (struct alsps_stat *)(filep->private_data);

	ret = sizeof(struct input_event);
	if (alsps->alsps_switch & PROXIMITY_ENABLE)
	{
		get_monotonic_time(&(data.time));
		data.type = EV_ABS;
		data.code = ABS_DISTANCE;
		data.value = taos_ioctl(TAOS_IOCTL_PROX_EVENT, NULL);
		
		TSL2771_DBG("proximity data = %d", data.value);
		
		clear_bit(ALSPS_PROXIMITY_DATA_READY, &ptr->status);
             //   input_report_abs(alsps->idev, ABS_DISTANCE, data.value);

		if (0 != copy_to_user(buff, (const void *)&data, 
				      sizeof(struct input_event))) 
		{
			ret = -ENOMEM;
		}

	}
	TSL2771_DBG("End %s", __FUNCTION__);

	return ret;
}

static unsigned int proximity_poll(struct file *filep, struct poll_table_struct *wait)
{
//qiu
	unsigned int mask = 0;
	struct alsps_stat *ptr;

	TSL2771_DBG("Start %s", __FUNCTION__);

	if (!alsps) 
	{
		TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
		return -1;
	}

	mask = 0;
	ptr = (struct alsps_stat *)filep->private_data;
	poll_wait(filep, &alsps->proximity_workq_head, wait);
	if (test_bit(ALSPS_PROXIMITY_FIRST_POLL, &ptr->status))
	{
		mask |= (POLLIN | POLLRDNORM);
		clear_bit(ALSPS_PROXIMITY_FIRST_POLL, &ptr->status);
	}
	if (test_bit(ALSPS_PROXIMITY_DATA_READY, &ptr->status))
	{
		mask |= (POLLIN | POLLRDNORM);
		TSL2771_DBG("*************proximity data ready*************\n");
	}

	TSL2771_DBG("End %s", __FUNCTION__);
	
	return mask;
}

//static int proximity_open(struct inode * inode, struct file * filep)
static void proximity_power_on(void)
{
	TSL2771_DBG("Start %s", __FUNCTION__);
	taos_ioctl(TAOS_IOCTL_PROX_ON, NULL); 
	return;
}

//static int proximity_close(struct inode * inode, struct file *filep)
static void proximity_power_off(void)
{
	TSL2771_DBG("Start %s", __FUNCTION__);
	taos_ioctl(TAOS_IOCTL_PROX_OFF, NULL);
	return;
}


static int tsl2771_ambient_read()
{
	int ret=0;
        struct input_event  data;
        char buff[512];
        memset(buff, 0, 512);

        TSL2771_DBG("Start %s", __FUNCTION__);

        if (alsps->alsps_switch & AMBIENT_ENABLE)
        {
                ret = sizeof(struct input_event);
              //  ptr = (struct alsps_stat *)(filep->private_data);
               // get_monotonic_time(&(data.time));
               // data.type = EV_LED;  //EV_ABS
               // data.code = LED_MISC; //ABS_MISC
                data.value = taos_ioctl(TAOS_IOCTL_ALS_DATA, NULL);
                TSL2771_DBG("ambient data = %d\n",data.value);
                if (in_am->idev)
                {
                   TSL2771_DBG("####### Now report ami data #######, data = %d", data.value);
                   input_event(in_am->idev, EV_LED, LED_MISC, data.value);
                   input_sync(in_am->idev);
                }
                else
                {	
	           TSL2771_DBG("ambient input device is null\n");
                }
/*
               // clear_bit(ALSPS_AMBIENT_DATA_READY, &ptr->status);
                if (0 != copy_to_user(buff, (const void *)&data, sizeof(struct input_event)))
                {
                        ret = -ENOMEM;
                }
*/
        }
        als_threshold_modify();
        taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
        TSL2771_DBG("End %s", __FUNCTION__);
}

static ssize_t ambient_read(struct file *filep, char __user *buff, size_t size, loff_t *offset)
{
	int ret=0;
	struct input_event  data;
	struct alsps_stat *ptr;
	
	TSL2771_DBG("Start %s", __FUNCTION__);

	if (alsps->alsps_switch & AMBIENT_ENABLE)
	{
		ret = sizeof(struct input_event);
		ptr = (struct alsps_stat *)(filep->private_data);
		get_monotonic_time(&(data.time));
		data.type = EV_LED;  //EV_ABS
		data.code = LED_MISC; //ABS_MISC
		data.value = taos_ioctl(TAOS_IOCTL_ALS_DATA, NULL);
		TSL2771_DBG("ambient data = %d\n",data.value);

		clear_bit(ALSPS_AMBIENT_DATA_READY, &ptr->status);
		if (0 != copy_to_user(buff, (const void *)&data, sizeof(struct input_event))) 
		{
			ret = -ENOMEM;
		}
	}
	als_threshold_modify();
	taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
	TSL2771_DBG("End %s", __FUNCTION__);

	return ret;
}

static unsigned int ambient_poll(struct file *filep, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct alsps_stat *ptr;

	TSL2771_DBG("Start %s", __FUNCTION__);

	if (!alsps) 
	{
		TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
		return -1;
	}
	mask = 0;
	ptr = (struct alsps_stat *)filep->private_data;

	poll_wait(filep, &alsps->ambient_workq_head, wait);
	if (test_bit(ALSPS_AMBIENT_FIRST_POLL, &ptr->status))
	{
		mask |= (POLLIN | POLLRDNORM);
		clear_bit(ALSPS_AMBIENT_FIRST_POLL, &ptr->status);
	}
	if (test_bit(ALSPS_AMBIENT_DATA_READY, &ptr->status))
	{
		mask |= (POLLIN | POLLRDNORM);
	}

	TSL2771_DBG("End %s", __FUNCTION__);
	return mask;
}

//static int ambient_open(struct inode * inode, struct file * filep)
static void ambient_power_on(void)
{
	TSL2771_DBG("Start %s", __FUNCTION__);
	taos_ioctl(TAOS_IOCTL_ALS_ON, NULL);
	return;
}


//static int ambient_close(struct inode * inode, struct file *filep)
static void ambient_power_off(void)
{
	TSL2771_DBG("Start %s", __FUNCTION__);
	taos_ioctl(TAOS_IOCTL_ALS_OFF, NULL);
	return;
}

/*
 * sysfs operations
 */ 
static int alsps_get_property(struct sensors_dev *sdev,
			      enum sensors_property property,
			      union sensors_propval *val)
{

	int value = 0;

	switch (property) {
	case SENSORS_PROP_INTERVAL:
		if (sdev->id & SENSORS_LIGHT)
		{
			val->intval = taos_cfgp->als_time;//alsps->ambient_interval;	
		}
		else
		{
			val->intval = 100;
		}
		break;

	case SENSORS_PROP_MAXRANGE:
		if (sdev->id & SENSORS_LIGHT)
		{
			val->intval = 1200;//15000;	
		}
		else
		{
			val->intval = 1;
		}
		break;
	
	case SENSORS_PROP_RESOLUTION:
		if (sdev->id & SENSORS_LIGHT)
		{
			val->intval = 50;	
		}
		else
		{
			val->intval = 1;
		}
		break;

	case SENSORS_PROP_VERSION:
		val->intval = 1;	
		break;

	case SENSORS_PROP_CURRENT:
		if (sdev->id & SENSORS_LIGHT)
		{
			val->intval = 3;	
		}
		else
		{
			val->intval = 5;
		}
		break;

	case SENSORS_PROP_SWITCH:
		if (sdev->id & SENSORS_PROXIMITY) 
		{
			value = PROXIMITY_ENABLE;
		}	
		else if (sdev->id & SENSORS_LIGHT) 
		{
			value = AMBIENT_ENABLE;	
		}
		else 
		{
			TSL2771_DBG("alsps: invailed sensor");
			return -EINVAL;
		}
//qiu
		//TSL2771_DBG("sdev->id = %d", sdev->id);
		//TSL2771_DBG("value = %d", value);
		//TSL2771_DBG("alsps->alsps_switch = %d", alsps->alsps_switch);

		val->intval = (alsps->alsps_switch & value) ? 1 : 0;
		//TSL2771_DBG("PROXIMITY_ENABLE = %d",val->intval);
		break;

	case SENSORS_PROP_VENDOR:
		sprintf(val->strval, "Mozart");
		break;

	default:
		return -EINVAL;
	}

//	TSL2771_DBG("End %s", __FUNCTION__);
	return 0;
}

static int alsps_put_property(struct sensors_dev *sdev,
			      enum sensors_property property,
			      union sensors_propval *val)
{
	//TSL2771_DBG("put2222222222222222222222222222222\n");

	int value = 0;

//	TSL2771_DBG("Start %s", __FUNCTION__);
	//TSL2771_DBG("put property = %d",property);
	if (!alsps) 
	{
		TSL2771_DBG("alsps: not init alsps sensor,should not be here\n");
		return -EINVAL;
	}

	switch (property)
	{
	case SENSORS_PROP_SWITCH:
		if (sdev->id & SENSORS_PROXIMITY) 
		{
			value = PROXIMITY_ENABLE;
		}	
		else if (sdev->id & SENSORS_LIGHT) 
		{
			value = AMBIENT_ENABLE;	
		}
		else 
		{
			TSL2771_DBG("alsps: invailed sensor");
			return -EINVAL;
		}
		if (val->intval) 
		{
			alsps->alsps_switch |= value;
			if (value == AMBIENT_ENABLE)
			{
				//TSL2771_DBG("set AMBIENT_ENABLE\n");
				//taos_ioctl(TAOS_IOCTL_ALS_ON, NULL);
				//TSL2771_DBG("Set alsps_timer_ambient,timer interval = %d ms",taos_cfgp->als_time);
//				mod_timer(&alsps_timer_ambient, jiffies + msecs_to_jiffies(taos_cfgp->als_time));
			}
			else
			{
				//taos_ioctl(TAOS_IOCTL_PROX_ON, NULL);
//				mod_timer(&alsps_timer_proximity, jiffies + msecs_to_jiffies(1000));
			}
//			msleep(50);
		} 
		else
		{
			alsps->alsps_switch &= ~value;
			if (value == AMBIENT_ENABLE)
			{
				//taos_ioctl(TAOS_IOCTL_ALS_OFF, NULL);
			}
			else
			{
				//taos_ioctl(TAOS_IOCTL_PROX_OFF, NULL);
			}
		}
		break;

	case SENSORS_PROP_INTERVAL:
		if ((sdev->id & SENSORS_LIGHT) && 
				(val->intval > DEFAULT_AMBIENT_INTERVAL))
		{
			down(&alsps->sem_ambient_interval);
			alsps->ambient_interval = val->intval;
			taos_cfgp->als_time = val->intval;
			up(&alsps->sem_ambient_interval);
		}
		break;	

	default:
		return -EINVAL;
	}
	
//	TSL2771_DBG("End %s", __FUNCTION__);
	return 0;
}

/* added by Huangxin */
static int proximity_ioctl(struct inode *inode, struct file *file,
                          unsigned int cmd, unsigned long arg)
{

}

static int ambient_ioctl(struct inode *inode, struct file *file,
                          unsigned int cmd, unsigned long arg)
{

        struct alsps_stat *ptr;
        int enable_ret = 0;

        TSL2771_DBG("Start %s", __FUNCTION__);

        enable_ret = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
        TSL2771_DBG("In ambient_open111, enable_ret is 0x%x\n", enable_ret);
        char flag = enable_ret & 0x04;

        if (!alsps)
        {
                TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
                return -1;
        }

        ptr = kzalloc(sizeof(struct alsps_stat), GFP_KERNEL);
        if (ptr == NULL)
        {
                TSL2771_DBG("alsps: ambient open kzalloc failed!\n");
                return -ENOMEM;
        }
        
        if (!alsps->ambient_count)
        {

                if(flag == 0)
                {
                        if(timer_status == TIMER_OFF)
                        {
                           taos_ioctl(TAOS_IOCTL_ALS_ON, NULL);
                           alsps->alsps_switch |= AMBIENT_ENABLE;
                           TSL2771_DBG("Ambient open successfully\n");
                        }
                        else
                        {
                                TSL2771_DBG("Timer is on, do not open ambient\n");
                                del_timer(&alsps_timer_after_proximity_close);
                        }
                }

        }
     
        clear_bit(ALSPS_AMBIENT_DATA_READY, &ptr->status);
        set_bit(ALSPS_AMBIENT_FIRST_POLL, &ptr->status);
        file->private_data = ptr;

        down(&sem_ambient_list);
        list_add(&ptr->list, &ambient_process_list);
        ++alsps->ambient_count;
        up(&sem_ambient_list);

        TSL2771_DBG("End %s", __FUNCTION__);
        return 0;
}

static enum sensors_property alsps_properties[] = {
	SENSORS_PROP_INTERVAL,
	SENSORS_PROP_MAXRANGE,		/* read only */
	SENSORS_PROP_RESOLUTION,	/* read only */
	SENSORS_PROP_VERSION,		/* read only */ 
	SENSORS_PROP_CURRENT,		/* read only */
	SENSORS_PROP_SWITCH,
	SENSORS_PROP_VENDOR,		/* read only */
};

static struct file_operations proximity_fops = {
	.owner = THIS_MODULE,
//	.open = proximity_open,
	.read = proximity_read,
	.poll = proximity_poll,
//	.release = proximity_close,
        .ioctl = proximity_ioctl,
};

static struct file_operations ambient_fops = {
	.owner = THIS_MODULE,
//	.open = ambient_open,
	.read = ambient_read,
	.poll = ambient_poll,
//	.release = ambient_close,
        .ioctl = proximity_ioctl,
};

static struct sensors_dev proximity_dev = {
	.id = SENSORS_PROXIMITY,
	.name = "psensor",
	.num_properties = ARRAY_SIZE(alsps_properties),
	.properties = alsps_properties,
	.get_property = alsps_get_property,
	.put_property = alsps_put_property,
	.fops =	&proximity_fops,
};

static struct sensors_dev ambient_dev = {
	.id = SENSORS_LIGHT,
	.name = "lsensor",
	.num_properties = ARRAY_SIZE(alsps_properties),
	.properties = alsps_properties,
	.get_property = alsps_get_property,
	.put_property = alsps_put_property,
	.fops =	&ambient_fops,
};

/* tmp added by Huangxin */
#if 0
static void change_threshold()
{
         
	TSL2771_DBG("Start %s\n",__FUNCTION__);
        int ret = 0;
        u16 status = 0;
 
        if (!alsps) 
        {
                TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
                return;
        }
       
        alsps->alsps_suspend = 1;

        if((status & 0x22) == 0x22)
        {
                TSL2771_DBG("P sensor suspend, now change threshold\n");
                if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x08), 0x40))) < 0)
                {
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return;
                }
                if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x09), 0x19))) < 0)
                {
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return;
                }
                if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0A), 0xFF))) < 0)
                {
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return;
                }
                if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0B), 0xFF))) < 0)
                {
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return;
                }
                TSL2771_DBG("In suspend, threshold change ok\n");
        }
        else
                TSL2771_DBG("System suspend, do not change threshold\n");

        taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);

        TSL2771_DBG("End %s\n",__FUNCTION__);
}
#endif 
static void alsps_early_suspend(struct early_suspend *h)
{
	TSL2771_DBG("Start %s\n",__FUNCTION__);
	if (!alsps) 
	{
		TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
		return;
	}

	alsps->alsps_suspend = 1;
//qiu
	int ret = 0;
	u16 status = 0;

	status = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x13);
	if((status & 0x22) == 0x22)
	{
		TSL2771_DBG("P sensor suspend, now change threshold\n");
		if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x08), 0x40))) < 0)
		{
    		TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
    		return;
		}			
		if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x09), 0x2F))) < 0)
		{
        	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        	return;
		}
		if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0A), 0xFF))) < 0)
		{
        	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        	return;
		}
		if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0B), 0xFF))) < 0)
		{
        	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        	return;
		}
		TSL2771_DBG("In suspend, threshold change ok\n");
	}
	else
		TSL2771_DBG("System suspend, do not change threshold\n");

	taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);

	TSL2771_DBG("End %s\n",__FUNCTION__);
}
#if 0
static void change_away()
{
     TSL2771_DBG("Start %s\n",__FUNCTION__);

        if (!alsps)
        {
                TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
                return;
        }
      alsps->alsps_suspend = 0;

        int ret = 0;
        if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x08), 0))) < 0)
        {
        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        return;
        }
        if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x09), 0))) < 0)
        {
        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        return;
        }
        if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0A), 0x40))) < 0)
        {
        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        return;
        }
        if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0B), 0x19))) < 0)
        {
        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        return;
        }
        TSL2771_DBG("In resume, threshold change ok\n");

        taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);

        TSL2771_DBG("End %s\n",__FUNCTION__);

}
#endif 
static void alsps_late_resume(struct early_suspend *h)
{
	TSL2771_DBG("Start %s\n",__FUNCTION__);

	if (!alsps) 
	{
		TSL2771_DBG("alsps: null pointer! at %s: %d\n", __FILE__, __LINE__);
		return;
	}

	if (alsps->alsps_switch & AMBIENT_ENABLE)
	{
		previous_lumen = 0;
	}
	
	alsps->alsps_suspend = 0;
//	mod_timer(&alsps_timer_asenser_resume, jiffies + msecs_to_jiffies(2000));
	
//qiu

	int ret = 0;
	if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x08), 0))) < 0)
	{
    	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
    	return;
	}			
	if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x09), 0))) < 0)
	{
        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        return;
	}
	if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0A), 0x40))) < 0)
	{
        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        return;
	}
	if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0B), 0x18))) < 0)
	{
        TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
        return;
	}
	TSL2771_DBG("In resume, threshold change ok\n");

	taos_i2c_write_special_data(TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);

	TSL2771_DBG("End %s\n",__FUNCTION__);
}

static struct early_suspend alsps_early_suspend_desc = {
	.level = 1,
	.suspend = alsps_early_suspend,
	.resume = alsps_late_resume,
};


static void tsl2771_sensor_work_read_data(struct work_struct *work)
{
	u8 reg18 = 0, reg19 = 0;
	int data = 0;
	int lux = 0;

	reg18 = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x18);
	reg19 = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x19);
	lux = taos_get_lux();
	data = (reg19 << 8) | reg18;

	TSL2771_DBG("********************* The current proximity data = 0x%x, %d **********************\n", data, data);
 	TSL2771_DBG("--------------------- The current ambient lux = %d----------------------\n", lux);
	queue_delayed_work(psensor_wqueue_read_data, &psensor_work_read_data, msecs_to_jiffies(2000));
}

static void power_on_prox_test(void)
{
	int ret = 0;
	u8 reg_val = 0;
			if((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x01), 0xFF))) < 0) 
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

			if((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x02), 0xFF))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

            if((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x03), 0xFF))) < 0)
			{
                TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0C);
            if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0C), reg_val|0x30))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

            if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0D), 0x00))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

            if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0E), 0x06))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}
//6.10
			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x0F);
			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x0F), reg_val | 0x20))) < 0)
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
                return (ret);
			}

			reg_val = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
			TSL2771_DBG("*#*#*contrl reg_val = 0x%x#*#*#\n", reg_val);
			if ((ret = (taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x00), (0x0D | reg_val))) < 0))
			{
            	TSL2771_DBG( "TAOS: i2c_smbus_write_byte_data failed in ioctl prox_on\n");
               	return (ret);
            }
}

static void alsps_get_prox_threshold(void)
{
	int prox_mean = 0, prox_max = 0, prox_value = 0, prox_now = 0, i;
	u8 bbb = 0;
	
	power_on_prox_test();
	mdelay(100);
	
	for(i = 0; i < 32; i++)
	{
		prox_now = i2c_smbus_read_word_data(alsps->alsps_client, 0x18 | 0x80);
		TSL2771_DBG("current distence = %d", prox_now);
		prox_max = (prox_now > prox_max) ? prox_now : prox_max;
		prox_value += prox_now;
		mdelay(80);
	}
	TSL2771_DBG("-------------------- prox_max = %d --------------------", prox_max);
	prox_mean = prox_value >> 5;
	TSL2771_DBG("-------------------- prox_mean = %d --------------------", prox_mean);

	if(prox_mean <= 200)
	{
		prox_threshold_param_hi = prox_mean + 300;
	}
	else if(prox_mean > 200 && prox_mean <= 400)
	{
		prox_threshold_param_hi = prox_mean + 600;
	}
	else if(prox_mean > 400)
	{
		prox_threshold_param_hi = 1000;
	}
	else
	{
//		prox_threshold_param_hi = standard_prox_threshold_param_hi;
	}
	
	TSL2771_DBG("-------------------- prox_threshold_param_hi = %d", prox_threshold_param_hi);




	bbb = (((taos_cfgp->als_time/50) * 18) - 1);
	bbb = (~bbb);
	taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG | TAOS_TRITON_ALS_TIME), bbb);
	taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|0x03), 0xF2);
}





static int tsl2771_misc_open(struct inode *inode, struct file *file)
{
	TSL2771_DBG("%s", __FUNCTION__);
	return nonseekable_open(inode, file);
}

static int tsl2771_misc_release(struct inode *inode, struct file *file)
{
}

static int
tsl2771_misc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	TSL2771_DBG("%s, cmd = %d", __FUNCTION__, cmd);
	void __user *argp = (void __user *)arg;
	int rwbuf[3] = {0, 0, 0};		/* for READ/WRITE */
	int ret = -1;				/* Return value. */

	switch(cmd) 
	{
		case TSL2771_IOCTL_GET_CHIP_ID:
			TSL2771_DBG("Now it is IOCTL_GET_CHIP_ID, chip_id = 0x%x", chip_id);
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

static struct file_operations tsl2771_misc_fops = 
{
	.owner = THIS_MODULE,
	.open = tsl2771_misc_open,
	.release = tsl2771_misc_release,
	.ioctl = tsl2771_misc_ioctl,
};

static struct miscdevice tsl2771_misc_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tsl2771_misc_dev",
	.fops = &tsl2771_misc_fops,
};

static ssize_t alsps_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	prox_adjust = 1;
	alsps_get_prox_threshold();
	sprintf(page, "%d\n", prox_threshold_param_hi);
	prox_adjust = 0;
}

static ssize_t alsps_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	int value = 0;
	sscanf(buff, "%d", &value);
	prox_threshold_param_hi = value;
	printk("prox_threshold_param_hi = %d\n", prox_threshold_param_hi);
}

static void create_alsps_proc_file(void)
{
	struct proc_dir_entry *alsps_proc_file = create_proc_entry("driver/tsl2771_threshold", 0644, NULL);

	if (alsps_proc_file) 
	{
		alsps_proc_file->read_proc = alsps_read_proc;
		alsps_proc_file->write_proc = (write_proc_t *)alsps_write_proc;
	} 
	else
		printk(KERN_INFO "alsps proc file create failed!\n");
}

static int __devinit alsps_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	TSL2771_DBG("Start %s",__FUNCTION__);

//	struct tsl2771_alsps_platform_data *pdata = NULL;
	int ret = 0;
    u8 read_init_data = 0;
    int level = 0;         
    int i = 0;

	if(client == NULL)
	{
		TSL2771_DBG("Client is NUll!\n");
		goto failed;
	}
	
	alsps = (struct alsps_struct *)kzalloc(sizeof(struct alsps_struct), GFP_KERNEL);
	if (!alsps) 
	{
		TSL2771_DBG("alsps: kzalloc memory failed!\n");
		return -1;
	}
	alsps->alsps_client = client;

	chip_id = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | 0x12);
	TSL2771_DBG("chip id = 0x%x", chip_id);
	if((chip_id != 0x09) && (chip_id != 0))
	{
		TSL2771_DBG("~~~~~~~~~~~~~~~~~~~ Not taos vendor\nintersil vendor ~~~~~~~~~~~~~~~~~~~~~~\n");
		return -ENOMEM;
	}
/*
	pdata = client->dev.platform_data;
	if (!pdata) 
	{
		dev_err(&client->dev, "platform data missing\n");
	    return -ENODEV;
	}
*/



	init_MUTEX(&sem_ambient_list);
	init_MUTEX(&alsps->sem_ambient_interval);
	init_waitqueue_head(&alsps->proximity_workq_head);
	init_waitqueue_head(&alsps->ambient_workq_head);
//	INIT_DELAYED_WORK(&alsps->ambient_workq, ambient_delayed_worker);

	alsps->proximity_count = 0;
	alsps->ambient_count = 0;
	alsps->ambient_interval = DEFAULT_AMBIENT_INTERVAL; 
	alsps->alsps_switch = 0;
	alsps->alsps_suspend = 0;

//	alsps->irq = pdata->irq;
        
	/*set irq*/
        /* IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING */
//	ret = request_irq(alsps->irq, proximity_isr,
//			  IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "alsps", NULL);
	
	if (ret)
	{
		TSL2771_DBG("alsps: proximity request irq failed!\n");
		goto failed;
	}


	ret = sensors_dev_register(&proximity_dev);
	if (ret) 
	{
		TSL2771_DBG( "alsps: proximity unable to register device.\n");
		goto failed_register_dev_proximity;
	}
	ret = sensors_dev_register(&ambient_dev);
	if (ret)
	{
		TSL2771_DBG( "alsps: ambient unable to register device.\n");
		goto failed_register_dev_ambient;
	}
	dev_set_drvdata(&client->dev, alsps);

//	register_early_suspend(&alsps_early_suspend_desc);
	
//	COMMON_I2C_Init();
    
	taos_datap = kmalloc(sizeof(struct taos_data), GFP_KERNEL);
	if (!taos_datap) 
	{	
		TSL2771_DBG("TAOS: kmalloc for struct taos_data failed in taos_init()\n");
		return -ENOMEM;
	}
	memset(taos_datap, 0, sizeof(struct taos_data));
	

	for (i = 0; i < sizeof(taos_triton_reg_init); i++)
	{
		if ((ret = taos_i2c_write_byte_data((TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i)), taos_triton_reg_init[i])) < 0) 
		{
			return (ret);
		}
        read_init_data = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG|(TAOS_TRITON_CNTRL +i));
	}
        
	strlcpy(taos_datap->taos_name, TAOS_DEVICE_ID, TAOS_ID_NAME_SIZE);
	taos_datap->valid = 0;
	init_MUTEX(&taos_datap->update_lock);

	if (!(taos_cfgp = kmalloc(sizeof(struct taos_cfg), GFP_KERNEL)))
	{
		TSL2771_DBG( "TAOS: kmalloc for struct taos_cfg failed in taos_probe()\n");
		return -ENOMEM;
	}
	
	taos_cfgp->calibrate_target = calibrate_target_param;
	taos_cfgp->als_time = als_time_param;
	taos_cfgp->scale_factor = scale_factor_param;
	taos_cfgp->gain_trim = gain_trim_param;
	taos_cfgp->filter_history = filter_history_param;
	taos_cfgp->filter_count = filter_count_param;
	taos_cfgp->gain = gain_param;
//	taos_cfgp->prox_threshold = prox_threshold_param_hi;
    setup_timer(&alsps_timer_after_proximity_close,
                          alsps_after_proximity_close_timer_handle,
                          0);

	sensor_input_add(INPUT_PROXIMITY_SENSOR, "proximity", proximity_report, NULL, proximity_power_on, proximity_power_off);
	sensor_input_add(INPUT_AMBIENT_SENSOR, "ambient", ambient_report, NULL, ambient_power_on, ambient_power_off);
//qiu
	INIT_DELAYED_WORK(&psensor_work, tsl2771_psensor_work);
	psensor_wqueue = create_singlethread_workqueue("psensor");
	if (!psensor_wqueue) 
	{
		goto workqueue_failed;
	}

	alsps_get_prox_threshold();
	
	ret = misc_register(&tsl2771_misc_device);
	if(ret) 
	{
		TSL2771_DBG("tsl2771_misc_device register failed\n");
		return -ENOMEM;
	}

	create_alsps_proc_file();

	return 0;
workqueue_failed:
failed_register_dev_ambient:
	sensors_dev_unregister(&proximity_dev);
failed_register_dev_proximity:
//	free_irq(alsps->irq, NULL);
failed:
	kfree(alsps);
	return ret;
/* added by Huangxin2011_02_15 */
out_reg_am:
        input_free_device(in_am->idev);
out_reg_pro:
        input_free_device(in_pro->idev);
out_am:
        kfree(in_am);
out_pro:
        kfree(in_pro);
        return -1;
}

static int __devexit alsps_remove(struct i2c_client *client)
{
/*
	if (alsps) {
		free_irq(alsps->irq, NULL);
	}
*/
#ifdef CONFIG_PROC_FS
	remove_proc_entry("alsps", NULL);
#endif
	sensors_dev_unregister(&ambient_dev);
	sensors_dev_unregister(&proximity_dev);
//	unregister_early_suspend(&alsps_early_suspend_desc);
	memset(alsps, 0, sizeof(struct alsps_struct));
	return 0;
}

static int tsl2771_suspend(struct platform_device *device, pm_message_t state)
{
	TSL2771_DBG("****** ambient_suspend ******\n");

	unsigned char status = taos_i2c_read_byte_data(TAOS_TRITON_CMD_REG | TAOS_TRITON_CNTRL);
	status = status & 0x06;
	if(status == 0x06)
	{
		ambient_status = OPENED;
		proximity_status = OPENED;
	}
	else if(status == 0x04)
	{
		ambient_status = CLOSED;
		proximity_status = OPENED;
	}
	else if(status == 0x02)
	{
		ambient_status = OPENED;
		proximity_status = CLOSED;
	}
	else
	{
		ambient_status = CLOSED;
		proximity_status = CLOSED;
	}
	
	taos_ioctl(TAOS_IOCTL_ALS_OFF, NULL);
	taos_ioctl(TAOS_IOCTL_PROX_OFF, NULL);
	return 0;
}

static int tsl2771_resume(struct platform_device *device)
{
	TSL2771_DBG("****** ambient_resume ******\n");
	
	if(ambient_status == OPENED)
	{
		taos_ioctl(TAOS_IOCTL_ALS_ON, NULL);
	}
	if(proximity_status == OPENED)
	{
		taos_ioctl(TAOS_IOCTL_PROX_ON, NULL);
	}

	return 0;
}

#if 0
static struct platform_driver alsps_device_driver = 
{
	.probe = alsps_probe,
	.remove = alsps_remove,
	.suspend = tsl2771_suspend,
	.resume	= tsl2771_resume,
	.driver = 
	{
		.name = "alsps",
		.owner 	= THIS_MODULE,
	},
};
#endif

static const struct i2c_device_id alsps_id[] = 
{
	{"alsps", 0},
	{}
};


static struct i2c_driver alsps_driver = 
{
	.driver = 
	{
		.owner = THIS_MODULE,
		.name = "alsps",
	},
	.probe 	= alsps_probe,
	.remove = alsps_remove,
	.suspend = tsl2771_suspend,
	.resume = tsl2771_resume,
	.id_table = alsps_id,
};


static int __init alsps_init(void)
{
/*
	wake_lock_init(&alsps_suspend_lock, WAKE_LOCK_SUSPEND, "alsps");
	return platform_driver_register(&alsps_device_driver);
*/
	int ret;

	ret = i2c_add_driver(&alsps_driver);
	if (ret) 
		TSL2771_DBG("Driver registration failed, module not inserted.\n");
	else
		TSL2771_DBG("i2c_add_driver() OK!\n");
	
	return ret;
}

static void  __exit alsps_exit(void)
{
/*
	platform_driver_unregister(&alsps_device_driver);
	wake_lock_destroy(&alsps_suspend_lock);
*/
	i2c_del_driver(&alsps_driver);
//	sensors_dev_unregister(&lis302dl);
}

module_init(alsps_init);
module_exit(alsps_exit);
MODULE_AUTHOR("Mozart");
MODULE_DESCRIPTION("Capella tsl2771 short distance proximity and ambient light sensor");
MODULE_LICENSE("GPL");
