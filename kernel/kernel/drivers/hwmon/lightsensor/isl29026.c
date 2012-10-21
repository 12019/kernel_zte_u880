#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/sensor-input.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include "isl29026.h"

#define ISL29026_DEBUG    0
#if ISL29026_DEBUG
#define ISL29026_DBG(fmt, args...)    printk("ISL29026:   " fmt "\n", ## args)
#else
#define ISL29026_DBG(fmt, args...)    do {} while(0)
#endif

#define OPENED 1
#define CLOSED 0
#define ISL29026_IOCTL_GET_CHIP_ID 	1

static struct i2c_client *isl29026_client = NULL;
static u16 prox_threshold = 0;
static int ambient_status = CLOSED;
static int proximity_status = CLOSED;
static unsigned char chip_id = 0;
static int aaa = 0;
static int prox_adjust = 0;

static void ambient_power_on(void)
{
	unsigned char reg_value = 0;
	reg_value = i2c_smbus_read_byte_data(isl29026_client, CFG_REG);
	reg_value |= 0x06;
	i2c_smbus_write_byte_data(isl29026_client, CFG_REG, reg_value);
	return;
}

static void ambient_power_off(void)
{
	unsigned char reg_value = 0;
	reg_value = i2c_smbus_read_byte_data(isl29026_client, CFG_REG);
	reg_value &= 0xFB;
	i2c_smbus_write_byte_data(isl29026_client, CFG_REG, reg_value);
	return;
}

static void ambient_report(struct input_dev *idev)
{
	struct input_event data;
	unsigned char value_low = 0;
	unsigned char value_high = 0;
	
	value_low = i2c_smbus_read_byte_data(isl29026_client, ALS_DATA_LOW);
	value_high = i2c_smbus_read_byte_data(isl29026_client, ALS_DATA_HIGH);
	
	data.value = ((value_high & 0x0F) << 8) | value_low;

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
			
//	ISL29026_DBG("####### Now report ami data #######, data = %d", data.value);
	
    input_sync(idev);
    return;
}

static void proximity_power_on(void)
{
	ISL29026_DBG("%s", __FUNCTION__);
	unsigned char reg_value = 0;
	reg_value = i2c_smbus_read_byte_data(isl29026_client, CFG_REG);
	reg_value |= 0xB0;
	i2c_smbus_write_byte_data(isl29026_client, CFG_REG, reg_value);
	return;
}

static void proximity_power_off(void)
{
	ISL29026_DBG("%s", __FUNCTION__);
	unsigned char reg_value = 0;
	reg_value = i2c_smbus_read_byte_data(isl29026_client, CFG_REG);
	reg_value &= 0x7F;
	i2c_smbus_write_byte_data(isl29026_client, CFG_REG, reg_value);
	return;
}

static void proximity_report(struct input_dev *idev)
{
	struct input_event data;
	unsigned char reg_value = 0;
	
	data.type = EV_ABS;
    data.code = ABS_DISTANCE;

    reg_value = i2c_smbus_read_byte_data(isl29026_client, CFG_REG);
	ISL29026_DBG("cfg = %d", reg_value);
	if(0 == (reg_value & 0x80))
		proximity_power_on();
	
	data.value = i2c_smbus_read_byte_data(isl29026_client, PROX_DATA);
	ISL29026_DBG("******** Now report pro data = %d ********", data.value);
	if(data.value >= prox_threshold)
	{
		ISL29026_DBG("report 3");
        input_report_abs(idev, ABS_DISTANCE, 3);
	}
	if(data.value < prox_threshold)
	{
		ISL29026_DBG("report 100");
        input_report_abs(idev, ABS_DISTANCE, 100);
	}
    input_sync(idev);

    return;
}

static void get_prox_threshold(void)
{
	int prox_mean = 0, prox_max = 0, prox_value = 0, prox_now = 0, i;

	if(0 == prox_adjust)
	{
		proximity_power_on();
		mdelay(100);
	}
	
	for(i = 0; i < 32; i++)
	{
		prox_now = i2c_smbus_read_byte_data(isl29026_client, PROX_DATA);
		ISL29026_DBG("current distence = %d", prox_now);
		prox_max = (prox_now > prox_max) ? prox_now : prox_max;
		prox_value += prox_now;
		mdelay(80);
	}
	ISL29026_DBG("-------------------- prox_max = %d --------------------", prox_max);
	prox_mean = prox_value >> 5;
	ISL29026_DBG("-------------------- prox_mean = %d --------------------", prox_mean);

	prox_threshold = prox_max + 55;
	
	ISL29026_DBG("-------------------- prox_threshold = %d", prox_threshold);

	if(0 == prox_adjust)
	{
		proximity_power_off();
	}
	
	return;
}



static int isl29026_misc_open(struct inode *inode, struct file *file)
{
	ISL29026_DBG("%s", __FUNCTION__);
	return nonseekable_open(inode, file);
}

static int isl29026_misc_release(struct inode *inode, struct file *file)
{
}

static int
isl29026_misc_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	ISL29026_DBG("%s, cmd = %d", __FUNCTION__, cmd);
	void __user *argp = (void __user *)arg;
	int rwbuf[3] = {0, 0, 0};		/* for READ/WRITE */
	int ret = -1;				/* Return value. */

	switch(cmd) 
	{
		case ISL29026_IOCTL_GET_CHIP_ID:
			ISL29026_DBG("Now it is IOCTL_GET_CHIP_ID, chip_id = 0x%x", chip_id);
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

static struct file_operations isl29026_misc_fops = 
{
	.owner = THIS_MODULE,
	.open = isl29026_misc_open,
	.release = isl29026_misc_release,
	.ioctl = isl29026_misc_ioctl,
};

static struct miscdevice isl29026_misc_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "isl29026_misc_dev",
	.fops = &isl29026_misc_fops,
};

static ssize_t read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	prox_adjust = 1;
	get_prox_threshold();
	sprintf(page, "%d\n", prox_threshold);
	prox_adjust = 0;
}

static ssize_t write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	int value = 0;
	sscanf(buff, "%d", &value);
	prox_threshold = value;
	printk("prox_threshold = %d\n", prox_threshold);
}

static void create_alsps_proc_file(void)
{
	struct proc_dir_entry *proc_file = create_proc_entry("driver/isl29026_threshold", 0644, NULL);

	if (proc_file) 
	{
		proc_file->read_proc = read_proc;
		proc_file->write_proc = (write_proc_t *)write_proc;
	} 
	else
		printk(KERN_INFO "alsps proc file create failed!\n");
}

static int __devinit isl29026_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	ISL29026_DBG("Start %s",__FUNCTION__);
	int ret = 0;

	if(client == NULL)
	{
		ISL29026_DBG("Client is NUll!\n");
		return -ENOMEM; 
	}
	isl29026_client = client;
	
	chip_id = i2c_smbus_read_byte_data(isl29026_client, CHIPID_REG);
	printk("chip_id = 0x%x\n", chip_id);
	if(chip_id != 0xBF)
	{
		printk("****************** Not intersil vendor\ntaos vendor ******************\n");
		return -ENOMEM;
	}

	sensor_input_add(INPUT_PROXIMITY_SENSOR, "proximity", proximity_report, NULL, proximity_power_on, proximity_power_off);
	sensor_input_add(INPUT_AMBIENT_SENSOR, "ambient", ambient_report, NULL, ambient_power_on, ambient_power_off);

	ret = misc_register(&isl29026_misc_device);
	if(ret) 
	{
		ISL29026_DBG("isl29026_misc_device register failed\n");
		return -ENOMEM;
	}
	
	get_prox_threshold();

	create_alsps_proc_file();

	return 0;
}

static int __devexit isl29026_remove(struct i2c_client *client)
{
	misc_deregister(&isl29026_misc_device);
	isl29026_client = NULL;
	return 0;
}

static int isl29026_suspend(struct i2c_client *client)
{
	ISL29026_DBG("****** ambient_suspend ******\n");

	unsigned char status = i2c_smbus_read_byte_data(isl29026_client, CFG_REG);
	status = status & 0x84;
	if(status == 0x84)
	{
		ambient_status = OPENED;
		proximity_status = OPENED;
	}
	else if(status == 0x04)
	{
		ambient_status = OPENED;
		proximity_status = CLOSED;
	}
	else if(status == 0x80)
	{
		ambient_status = CLOSED;
		proximity_status = OPENED;
	}
	else
	{
		ambient_status = CLOSED;
		proximity_status = CLOSED;
	}
	
	proximity_power_off();
	ambient_power_off();
	
	return 0;
}

static int isl29026_resume(struct i2c_client *client)
{
	ISL29026_DBG("****** ambient_resume ******\n");
	
	if(ambient_status == OPENED)
	{
		ambient_power_on();
	}
	if(proximity_status == OPENED)
	{
		proximity_power_on();
	}

	return 0;
}

static const struct i2c_device_id isl29026_id[] = 
{
	{"isl29026", 0},
	{}
};

static struct i2c_driver isl29026_driver = 
{
	.driver = 
	{
		.owner = THIS_MODULE,
		.name = "isl29026",
	},
	.probe 	= isl29026_probe,
	.remove = isl29026_remove,
	.suspend = isl29026_suspend,
	.resume = isl29026_resume,
	.id_table = isl29026_id,
};

static int __init isl29026_init(void)
{
	int ret;

	ret = i2c_add_driver(&isl29026_driver);
	if (ret) 
		ISL29026_DBG("Driver registration failed, module not inserted.\n");
	else
		ISL29026_DBG("i2c_add_driver() OK!\n");
	
	return ret;
}

static void  __exit isl29026_exit(void)
{
	i2c_del_driver(&isl29026_driver);
}

module_init(isl29026_init);
module_exit(isl29026_exit);
MODULE_AUTHOR("Mozart");
MODULE_DESCRIPTION("Intersil ISL29026 short distance proximity and ambient light sensor");
MODULE_LICENSE("GPL");
