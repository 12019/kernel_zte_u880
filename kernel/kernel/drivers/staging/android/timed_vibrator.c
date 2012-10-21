/*
 *	drivers/staging/android/timed_vibrator.c
 *
 *	vibrator driver for Android
 *
 *	Copyright (C) 2009, Marvell Corporation (xjian@Marvell.com)
 *	Author: Angela Wan <jwan@marvell.com>
 *		     Mike Lockwood <lockwood@android.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */
 
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
//#include <mach/resource.h>
#include <linux/mfd/88pm860x.h>
#include "timed_output.h"

struct timed_output_dev vibrator_dev;
static int vibrator_time;
struct vibrator_data *pdata;

static int work_init=0;
static struct delayed_work	vibrator_work;




static void vibrator_enable(struct timed_output_dev *dev, int value)
{
      // printk("value=======================%dms\n",value);
	pdata->set_vibrator(pdata->max_v);
	if(value>10&&work_init)
	{
	  schedule_delayed_work(&vibrator_work, msecs_to_jiffies(value));
	  vibrator_time = value;
	}
	else
	{
	mdelay(value);
	vibrator_time = value;
	pdata->set_vibrator(pdata->min_v);
}

}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	return vibrator_time;
}



void vibrator_func_work(void)  
{
	pdata->set_vibrator(pdata->min_v);	
}

static __devinit int pm860x_vibrator_probe(struct platform_device *pdev) {
	int ret;
	
	pdata = pdev->dev.platform_data;
	vibrator_dev.name = "vibrator";
	vibrator_dev.enable = vibrator_enable;
	vibrator_dev.get_time = vibrator_get_time;
	
	INIT_DELAYED_WORK(&vibrator_work, vibrator_func_work);
	work_init=1;	
	
	ret = timed_output_dev_register(&vibrator_dev);

	return ret;
}

static __devexit int pm860x_vibrator_remove(struct platform_device *pdev)
{
	timed_output_dev_unregister(&vibrator_dev);

	return 0;
}


static struct platform_driver vibrator_driver = {
	.probe		= pm860x_vibrator_probe,
	.remove		= __devexit_p(pm860x_vibrator_remove),
	.driver		= {
		.name		= "vibrator",
		.owner		= THIS_MODULE,
	},
};

static int __init vibrator_init(void)
{
	return platform_driver_register(&vibrator_driver);
}

static void __exit vibrator_exit(void)
{
	platform_driver_unregister(&vibrator_driver);
}

module_init(vibrator_init);
module_exit(vibrator_exit);
MODULE_LICENSE("GPL");

