/*
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/mfd/88pm860x.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>



#if (defined CONFIG_PXA_U880)
#define PWM_CR1     0xFE01A400
#define PWM_DCR1   0xFE01A404
#define PWM_PR1     0xFE01A408
#define  PXA910_PWM0 0xFE01500C
#define  PXA910_PWM1 0xFE015010
#define  PXA910_PWM2 0xFE015014
#define  PXA910_PWM3 0xFE015018

volatile static int PWM_brightness = 50;
#endif



#define MAX_BRIGHTNESS          (0xFF)
#define MIN_BRIGHTNESS		(0)

#define CURRENT_MASK		(0x1F << 1)

struct pm860x_backlight_data {
	struct pm860x_chip *chip;
	struct i2c_client *i2c;
	int	current_brightness;
        int     port;
        int     pwm;
        int     iset;
        struct early_suspend    early_suspend;
};

static int enable_bl = 1;
static int current_brightness = 102;
static int delay_time;

static struct backlight_device *bl_dev;
static struct work_struct bl_wq;

static void bl_handler_do_work(struct work_struct *work);
static int pm860x_backlight_set(struct backlight_device *bl, int brightness);


static void bl_handler_do_work(struct work_struct *work)
{
        msleep(delay_time);
#if (defined CONFIG_PXA_U880)
	__raw_writel(0x3,PXA910_PWM0);
	__raw_writel(0x3,PXA910_PWM1);
	__raw_writel(0x22,PWM_DCR1);//duty clce
	__raw_writel(0x148,PWM_PR1);//prioed
#endif
    pm860x_backlight_set(bl_dev, current_brightness);
#if (defined CONFIG_PXA_U880)
	__raw_writel(0x2,0xFE01E1A8); 
#endif
}
#if (defined CONFIG_PXA_U880)
static void bl_init(void)
{
	__raw_writel(0x3,PXA910_PWM0);
	__raw_writel(0x3,PXA910_PWM1);
	__raw_writel(1,PWM_CR1);//prescal=0
	__raw_writel(0x22,PWM_DCR1);//duty clce
	__raw_writel(0x148,PWM_PR1);//prioed
	__raw_writel(0x2,0xFE01E1A8); 
}
#endif
void lcd_power_on(int on, int ms)
{

    if(on)
    {
        enable_bl = 1;
         #if (defined CONFIG_PXA_U880)
               // delay_time = ms;
        	//schedule_work(&bl_wq);
         printk("Exit %s,PWMbrightness=%d\n", __FUNCTION__,current_brightness);
         pm860x_backlight_set(bl_dev, current_brightness);
        #else
                delay_time = ms;
        schedule_work(&bl_wq);
        #endif
    }
        else
        {
            current_brightness = 0;
            pm860x_backlight_set(bl_dev, current_brightness);
            enable_bl = 0;
        }

}
EXPORT_SYMBOL(lcd_power_on);

static inline int wled_a(int port)
{
	int ret;

	ret = ((port - PM8606_BACKLIGHT1) << 1) + 2;
	return ret;
}

static inline int wled_b(int port)
{
	int ret;

	ret = ((port - PM8606_BACKLIGHT1) << 1) + 3;
	return ret;
}

/* WLED2 & WLED3 share the same IDC */
static inline int wled_idc(int port)
{
	int ret;

	switch (port) {
	case PM8606_BACKLIGHT1:
	case PM8606_BACKLIGHT2:
		ret = ((port - PM8606_BACKLIGHT1) << 1) + 3;
		break;
	case PM8606_BACKLIGHT3:
	default:
		ret = ((port - PM8606_BACKLIGHT2) << 1) + 3;
		break;
	}
	return ret;
}

static int pm860x_backlight_set(struct backlight_device *bl, int brightness)
{
	struct pm860x_backlight_data *data = bl_get_data(bl);
	struct pm860x_chip *chip = data->chip;
        unsigned char value;
        int ret;
  
    if(!enable_bl)
    {
        current_brightness = brightness;
  #if  (defined CONFIG_PXA_U880)
    PWM_brightness= brightness;
  #endif
                
        return 0;
    }

        if (brightness > MAX_BRIGHTNESS)
                value = MAX_BRIGHTNESS;
        else
		value = brightness;

    #if  (defined CONFIG_PXA_U880)
     __raw_writel(value,PWM_DCR1);//duty clce
	data->current_brightness = value;
	PWM_brightness=value;
   #endif
	ret = pm860x_reg_write(data->i2c, wled_a(data->port), value);
	if (ret < 0)
		goto out;

	if ((data->current_brightness == 0) && brightness) {
		if (data->iset) {
			ret = pm860x_set_bits(data->i2c, wled_idc(data->port),
					      CURRENT_MASK, data->iset);
			if (ret < 0)
				goto out;
		}
		if (data->pwm) {
			ret = pm860x_set_bits(data->i2c, PM8606_PWM,
					      PM8606_PWM_FREQ_MASK, data->pwm);
			if (ret < 0)
				goto out;
		}
		if (brightness == MAX_BRIGHTNESS) {
			/* set WLED_ON bit as 100% */
			ret = pm860x_set_bits(data->i2c, wled_b(data->port),
					      PM8606_WLED_ON, PM8606_WLED_ON);
		}
	} else {
		if (brightness == MAX_BRIGHTNESS) {
			/* set WLED_ON bit as 100% */
			ret = pm860x_set_bits(data->i2c, wled_b(data->port),
					      PM8606_WLED_ON, PM8606_WLED_ON);
		} else {
			/* clear WLED_ON bit since it's not 100% */
			ret = pm860x_set_bits(data->i2c, wled_b(data->port),
					      PM8606_WLED_ON, 0);
		}
	}
	if (ret < 0)
		goto out;

	dev_dbg(chip->dev, "set brightness %d\n", value);
	data->current_brightness = value;
	return 0;
out:
	dev_dbg(chip->dev, "set brightness %d failure with return "
		"value:%d\n", value, ret);
	return ret;
}

static int pm860x_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	return pm860x_backlight_set(bl, brightness);
}

static int pm860x_backlight_get_brightness(struct backlight_device *bl)
{
	struct pm860x_backlight_data *data = bl_get_data(bl);
	struct pm860x_chip *chip = data->chip;
	int ret;
	   #if defined(U810_DEBUG)

	    #endif
	    #if  (defined CONFIG_PXA_U880)
	data->current_brightness = PWM_brightness;
	return data->current_brightness;
	#endif
	ret = pm860x_reg_read(data->i2c, wled_a(data->port));
	if (ret < 0)
		goto out;
	data->current_brightness = ret;
	dev_dbg(chip->dev, "get brightness %d\n", data->current_brightness);
	return data->current_brightness;
out:
	return -EINVAL;
}

static struct backlight_ops pm860x_backlight_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= pm860x_backlight_update_status,
	.get_brightness	= pm860x_backlight_get_brightness,
};

static int __check_device(struct pm860x_backlight_pdata *pdata, char *name)
{
	struct pm860x_backlight_pdata *p = pdata;
	int ret = -EINVAL;

	while (p && p->id) {
		if ((p->id != PM8606_ID_BACKLIGHT) || (p->flags < 0))
			break;

		if (!strncmp(name, pm860x_backlight_name[p->flags],
			MFD_NAME_SIZE)) {
			ret = (int)p->flags;
			break;
		}
		p++;
	}
        return ret;
}

#ifdef CONFIG_PM
static void pm860x_backlight_suspend(struct early_suspend *h)
{
        struct pm860x_backlight_data *data = container_of(h, struct pm860x_backlight_data, early_suspend);
        unsigned char value;
        int ret;

	    #if  (defined CONFIG_PXA_U880)
	  // data->current_brightness = 0;
           __raw_writel(0,PWM_DCR1);//duty clce
	   #endif

#ifdef CONFIG_PXA_U880
		ret = pm860x_reg_read(data->i2c, PM8606_RGB1B);
		ret = ret & 0x1F;
		if(ret)
			goto bl_suspend;
		
		ret = pm860x_reg_read(data->i2c, PM8606_RGB1C);
		ret = ret & 0x1F;
		if(ret)
			goto bl_suspend;
#endif

        /* Disable reference OSC */
        ret = pm860x_reg_read(data->i2c, PM8606_MISC);
        if (ret < 0)
                goto bl_suspend;
        if (ret & PM8606_MISC_OSC_EN) {
                value = ret & (~PM8606_MISC_OSC_EN);
                ret = pm860x_reg_write(data->i2c, PM8606_MISC, value);
                if (ret < 0)
                        goto bl_suspend;
        }
        /* Disable reference VSYS */
        ret = pm860x_reg_read(data->i2c, PM8606_VSYS);
        if (ret < 0)
                goto bl_suspend;
        if (ret & PM8606_VSYS_EN) {
                value = ret & (~PM8606_VSYS_EN);
                ret = pm860x_reg_write(data->i2c, PM8606_VSYS, value);
                if (ret < 0)
                        goto bl_suspend;
        }

bl_suspend:
        return;
}

static void pm860x_backlight_resume(struct early_suspend *h)
{
        struct pm860x_backlight_data *data = container_of(h, struct pm860x_backlight_data, early_suspend);
        unsigned char value;
        int ret;

        /* Enable reference VSYS */
        ret = pm860x_reg_read(data->i2c, PM8606_VSYS);
        if (ret < 0)
                goto bl_resume;
        if ((ret & PM8606_VSYS_EN) == 0) {
                value = ret | PM8606_VSYS_EN;
                ret = pm860x_reg_write(data->i2c, PM8606_VSYS, value);
                if (ret < 0)
                        goto bl_resume;
        }
        /* Enable reference OSC */
        ret = pm860x_reg_read(data->i2c, PM8606_MISC);
        if (ret < 0)
                goto bl_resume;
        if ((ret & PM8606_MISC_OSC_EN) == 0) {
                value = ret | PM8606_MISC_OSC_EN;
                ret = pm860x_reg_write(data->i2c, PM8606_MISC, value);
                if (ret < 0)
                        goto bl_resume;
        }

bl_resume:
        return;
}
#endif

static int pm860x_backlight_probe(struct platform_device *pdev)
{
        struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_platform_data *pm860x_pdata;
	struct pm860x_backlight_pdata *pdata = NULL;
	struct pm860x_backlight_data *data;
	struct backlight_device *bl;
	struct resource *res;
	unsigned char value;
	char name[MFD_NAME_SIZE];
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "No I/O resource!\n");
		return -EINVAL;
	}

	if (pdev->dev.parent->platform_data) {
		pm860x_pdata = pdev->dev.parent->platform_data;
		pdata = pm860x_pdata->backlight;
	}
	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data isn't assigned to "
			"backlight\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct pm860x_backlight_data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;
	memset(name, 0, MFD_NAME_SIZE);
	strncpy(name, res->name, MFD_NAME_SIZE-1);
	data->chip = chip;
	data->i2c = (chip->id == CHIP_PM8606) ? chip->client	\
			: chip->companion;
	data->current_brightness = MAX_BRIGHTNESS;
	data->pwm = pdata->pwm;
	data->iset = pdata->iset;
        data->port = __check_device(pdata, name);
        if (data->port < 0) {
                dev_err(&pdev->dev, "wrong platform data is assigned");
                kfree(data);
                return -EINVAL;
        }

	bl = backlight_device_register(name, &pdev->dev, data,
					&pm860x_backlight_ops);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		kfree(data);
		return PTR_ERR(bl);
	}
	bl->props.max_brightness = MAX_BRIGHTNESS;
	bl->props.brightness = 102;

	platform_set_drvdata(pdev, bl);

    #if  (defined CONFIG_PXA_U880)
	        bl->props.brightness = 34;
		bl_init();
     #endif

	ret = pm860x_set_bits(data->i2c, wled_idc(data->port),
		CURRENT_MASK, data->iset);
	if (ret < 0)
		goto out;

	/* Enable reference VSYS */
	ret = pm860x_reg_read(data->i2c, PM8606_VSYS);
	if (ret < 0)
		goto out;
	if ((ret & PM8606_VSYS_EN) == 0) {
		value = ret | PM8606_VSYS_EN;
		ret = pm860x_reg_write(data->i2c, PM8606_VSYS, value);
		if (ret < 0)
			goto out;
	}
	/* Enable reference OSC */
	ret = pm860x_reg_read(data->i2c, PM8606_MISC);
	if (ret < 0)
		goto out;
	if ((ret & PM8606_MISC_OSC_EN) == 0) {
		value = ret | PM8606_MISC_OSC_EN;
		ret = pm860x_reg_write(data->i2c, PM8606_MISC, value);
		if (ret < 0)
			goto out;
	}
	/* read current backlight */
        ret = pm860x_backlight_get_brightness(bl);
        if (ret < 0)
                goto out;
    
    bl_dev = bl;
  #if  (defined CONFIG_PXA_U880)
  #else
    INIT_WORK(&bl_wq, bl_handler_do_work);
        backlight_update_status(bl);
  #endif

#ifdef CONFIG_PM
        data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
        data->early_suspend.suspend = pm860x_backlight_suspend,
        data->early_suspend.resume = pm860x_backlight_resume,
        register_early_suspend(&data->early_suspend);
#endif
        return 0;
out:
        kfree(data);
	return ret;
}

static int pm860x_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pm860x_backlight_data *data = bl_get_data(bl);

	backlight_device_unregister(bl);
	kfree(data);
	return 0;
}

static struct platform_driver pm860x_backlight_driver = {
	.driver		= {
		.name	= "88pm860x-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pm860x_backlight_probe,
	.remove		= pm860x_backlight_remove,
};

static int __init pm860x_backlight_init(void)
{
	return platform_driver_register(&pm860x_backlight_driver);
}
module_init(pm860x_backlight_init);

static void __exit pm860x_backlight_exit(void)
{
	platform_driver_unregister(&pm860x_backlight_driver);
}
module_exit(pm860x_backlight_exit);

MODULE_DESCRIPTION("Backlight Driver for Marvell Semiconductor 88PM8606");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:88pm860x-backlight");
