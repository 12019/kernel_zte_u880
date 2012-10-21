/*
 *  Copyright (C) 2008-2009 Foxconn K.H. Fan
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <mach/pxa910-aec-fm2010.h>
//#include <mach/pxa-regs.h>
//#include <mach/mfp-pxa9xx.h>
//#include <mach/u900_d9035_board.h>
#include <linux/proc_fs.h>
#include <plat/mfp.h>


#define TPA2018_SOC_PROC

// major & minor define area
#define tpa_MAJOR       18                   /* dynamic major by default */
#define tpa_MINOR       5

#ifdef tpa_MAJOR
static int tpa_major =   tpa_MAJOR;
#else
static int tpa_major =   0;
#endif

#ifdef tpa_MINOR
static int tpa_minor =   tpa_MINOR;
#else
static int tpa_minor =   0;
#endif

static int tpa_nr_dev = 1;
static int g_mode = AEC_NORMAL_MODE;
static unsigned int tpa_count = 0;

#if 0
static struct device ghost_device = {
	.bus_id    = "tpa_param",
};
#endif

struct i2c_client *gtpaClient = NULL;

static  struct tpa_reg  tpa2018_param[]=
{
	{0x01,0xc3,},
	{0x02,0x05,},//3//	
	{0x03,0x10,},//release time is 
	{0x04,0x00,}, //hold time diabled
	{0x05,0x0c,},	//fixed gain : 12db
	{0x06,0x7e,},//
/* turn off agc __modified by liujin 20101224 begin */	
	//{0x07,0xc2,},//compression ration 4:1
	{0x07,0xc0,}, //compression ration 1:1
/* modified by liujin 20101224 end */	
};

static int tpa_reg_read(char reg, char *data)
{
	char ret;

	//printk("enter tpa_reg_read\n");
	
	ret = i2c_smbus_read_byte_data(gtpaClient, reg);

	if (ret< 0) 
	{
		printk("Micco read error!\n");
		return -EIO;
	}
	*data = ret;
	
	return 0;
}

/*
	send two byte to the tpa chip's memory, the memory is 2bytes align;
	return 0, success
	-1, fails
*/
int tpa_reg_write(char reg, char data)
{
	int ret;

       // printk("enter tpa_reg_write\n");

	ret = i2c_smbus_write_byte_data(gtpaClient, reg, data);
		
	if (ret != 0)
	{
        		printk("tpa tpa_mem_write fail, ret=%d\n", ret);
		return -EIO;
    }
	return 0;
}

static int tpa_suspend(struct i2c_client * tpa, pm_message_t state)
{
#if 0
//add by xiameng 20101203 for  tpa2018 leak current 0.35mA	
	if (gpio_request(MFP_PIN_GPIO6, "AUDIO_PA_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO6);
		return -EIO;
	}

	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 0);
	gpio_free(MFP_PIN_GPIO6);
//add by xiameng 20101203 for  tpa2018 leak current 0.35mA	end 
#endif 
	return 0;
}

static  int tpa_resume(struct i2c_client * tpa)
{
#if 0
//add by xiameng 20101203 for  tpa2018 leak current 0.35mA	
	if (gpio_request(MFP_PIN_GPIO6, "AUDIO_PA_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO6);
		return -EIO;
	}

	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 1);
	gpio_free(MFP_PIN_GPIO6);
//add by xiameng 20101203 for  tpa2018 leak current 0.35mA	end 
#endif
	return 0;
}

void tpa2018_Power_control(int bflag)
{
      	printk("tpa2018_Power_control()  bflag=%d.\n",bflag);
		
	if (gpio_request(MFP_PIN_GPIO6, "AUDIO_PA_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO6);
		return -EIO;
	}
	if (bflag)
	{
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 1);
	        mdelay(5);
	}
	else
	{
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 0);
	        mdelay(5);
	}
	gpio_free(MFP_PIN_GPIO6);
}

int tpa2018_init_client(void)		
{
	char address;
	char data;
	char org_data;
	int i, ret;
	
//	printk("enter tpa2018_init_client\n");
	
	for(i=0; i < (sizeof(tpa2018_param)/sizeof(struct tpa_reg)); i++)
	{
		address = tpa2018_param[i].reg;
		data = tpa2018_param[i].val;
		
		ret = tpa_reg_write(address, data);
	}
	
	return ret;
}

int tpa2018_init(void)
{
       int enflag = 1;
       tpa2018_Power_control(enflag);
       tpa2018_init_client();	   
	return 0;
}

int tpa2018_sleep(void)//disabel the SPK_EN
{
	char	data, reg;
	int ret;

        reg= 0x1;
	ret = tpa_reg_read(reg, &data);
	if (ret != 0)
	{
        	printk("tpa_reg_read fail, ret=%d\n", ret);
		return -EIO;
   	 }

	data &= 0xbf;//disable the SPK_EN

	tpa_reg_write(reg,  data);
	return 0;
}
int tpa2018_wakeup(void) //enable the SPK_EN
{
	char	data, reg;
	int ret;

        reg= 0x1;
	ret = tpa_reg_read(reg, &data);
	if (ret != 0)
	{
        	printk("tpa_reg_read fail, ret=%d\n", ret);
		return -EIO;
   	 }

	data |= 0x40; 
	tpa_reg_write(reg,  data);
	
	return 0;
}
int tpa2018_reset(void)
{	
	return 0;	
}
int tpa2018_set_mode(int mode)
{
	return 0;
}
/* codec register dump */
#ifdef TPA2018_SOC_PROC
static ssize_t tpa2018_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	int 			t,status;
	char 			i,org_data;
	char 			reg;
	char address;
	struct aec_platform_data *pdata;

	t = scnprintf(next, size, "Tpa2018 regs: \n");
	size -= t;
	next += t;

	for(i=0; i < (sizeof(tpa2018_param)/sizeof(struct tpa_reg)); i++)
		{			
			address = tpa2018_param[i].reg;		
			tpa_reg_read(address, &org_data);
			t = scnprintf(next, size, "[0x%02x]=0x%02x  \n", address, org_data);
			size -= t;
			next += t;
		}

	*eof = 1;
	return count - size;
}

static int tpa2018_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[4096];
	char *buf = kbuf;
	char	i, reg, reg2;
	char cmd;

	if (count >= 4096)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	sscanf(buf, "%c 0x%x 0x%x", &cmd, &reg, &i);


	if ('r' == cmd) {
		if (reg > 0x7 || (reg < 1)) {
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		tpa_reg_read(reg , &i);
		printk(KERN_INFO "0x[%2x]=0x%2x\n", i, reg);
	} else if ('w' == cmd) {
		if (reg > 0x7 || (reg < 1)){
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		tpa_reg_write(reg, i);
		tpa_reg_read(reg, &reg2);
		printk(KERN_INFO
			"write 0x%2x to 0x[%2x], read back 0x%2x\n",
			i, reg, reg2);
	} else {
		printk(KERN_ERR "unknow opt!\n");
		goto error;
	}
	
	return count;
	
error:
	printk(KERN_INFO "r/w index(0x%%2x) value(0x%%2x)\n");
	return count;
}
#endif

//static DEVICE_ATTR(tpa_reg, 0644, tpa_reg_show, tpa_reg_store);
static struct device_attribute dev_attr_tpa_reg = {
	.attr = {	
		.name = "tpa2018",
		.owner=THIS_MODULE,
		.mode =S_IRUGO | S_IWUGO,
	},
};
static int tpa_probe(struct i2c_client *client)
{
	struct aec_platform_data *pdata;
	int ret;
#ifdef TPA2018_SOC_PROC
	struct proc_dir_entry *tpa2018_proc_entry;
#endif

	printk("tpa: tpa_probe\n");

	pdata = client->dev.platform_data;
	
	gtpaClient = client;
	
      if(NULL != pdata->init)
		pdata->init();	  
  
	  
	ret = device_create_file(&gtpaClient->dev, &dev_attr_tpa_reg);
	if (ret < 0)
		printk(KERN_WARNING "tpa: failed to add tpa chip sysfs entries\n");

#if 0
	if(device_register(&ghost_device) < 0) {
		printk("[tpa]device_register error\n");	    
	}
#endif
	
	if(NULL != pdata->sleep)
		pdata->sleep();
	

#ifdef TPA2018_SOC_PROC
	tpa2018_proc_entry = create_proc_entry("driver/tpa2018", 0, NULL);
	if (tpa2018_proc_entry) {
		tpa2018_proc_entry->data = gtpaClient;
		tpa2018_proc_entry->read_proc = tpa2018_proc_read;
		tpa2018_proc_entry->write_proc = tpa2018_proc_write;
	}
#endif

	
	return ret;
}

static int tpa_remove(struct i2c_client *client)
{  
	//printk("tpa: tpa_remove\n");	
	tpa2018_Power_control(0);
//	device_unregister(&ghost_device);
	device_remove_file(&gtpaClient->dev, &dev_attr_tpa_reg); 	
	gtpaClient = NULL;
	return 0;
}
static const struct i2c_device_id tpa2018_id[] = {
	{ "tpa2018", 0 },
	{ }
};
/* corgi i2c codec control layer */
struct i2c_driver tpa_i2c_driver = {
	.driver = {
		.name = "tpa2018",		
	},
	.probe      = tpa_probe,
	.remove     = tpa_remove,
	.id_table	= tpa2018_id,
	.suspend    = tpa_suspend,
	.resume     = tpa_resume,	
};

// device structure
struct tpa_dev {	
	struct cdev cdev;
};

struct tpa_dev *tpa_device;	

static int __init tpa_init(void)
{
	int result = -1;
	
	//printk(KERN_EMERG "tpa: tpa_init start ...\n");
	
	result = i2c_add_driver(&tpa_i2c_driver);
	if (result) {
		printk(KERN_ERR "tpa: can't add i2c driver");							
	} 
	return result;		
}

static void __exit tpa_exit(void)
{		
	i2c_del_driver(&tpa_i2c_driver);
}

module_init(tpa_init);
module_exit(tpa_exit);

MODULE_AUTHOR("K.H. Fan <kh.fan@foxconn.com>");
MODULE_DESCRIPTION("Foxconn manafacture tpa");
MODULE_LICENSE("GPL");

