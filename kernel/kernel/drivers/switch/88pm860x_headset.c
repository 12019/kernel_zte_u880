/*
 *	drivers/switch/88pm860x_headset.c
 *
 *	headset & hook detect driver for pm8607
 *
 *	Copyright (C) 2010, Marvell Corporation (xjian@Marvell.com)
 *	Author: Raul Xiong <xjian@marvell.com>
 *				 Mike Lockwood <lockwood@android.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm860x.h>
#include <linux/mfd/88pm860x-headset.h>
#include <linux/init.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/slab.h>

struct pm860x_headset_info {
	struct pm860x_chip	*chip;
	struct device		*dev;
	struct i2c_client	*i2c;
	int			irq_headset;
	int			irq_hook;
	int			headset_flag;
	struct work_struct	work_headset, work_hook;
	struct delayed_work	delayed_work;
	struct headset_switch_data *psw_data_headset, *psw_data_hook;
};

struct headset_switch_data {
	struct switch_dev	sdev;
	const char		*name_on;
	const char		*name_off;
	const char		*state_on;
	const char		*state_off;
	int 			irq;
	int state;
};

/* on TD_DKB, the headset jack circuit logic is opposite to the
 * design of levante spec, so if headset status is connected in
 * levante spec, it's actually disconnected. */

/* 88PM860x gives us an interrupt when headset is plug-in/unplug */
static irqreturn_t pm860x_headset_handler(int irq, void *data)
{
	struct pm860x_headset_info *info = data;

	/* headset interrupt */
	if (irq == info->irq_headset) {
		if (info->psw_data_headset != NULL) {
			queue_work(info->chip->monitor_wqueue, &info->work_headset);
		}
	} else {
	/* hook interrupt */
		if (info->psw_data_hook != NULL) {
			queue_work(info->chip->monitor_wqueue, &info->work_hook);
		}
	}

	return IRQ_HANDLED;
}

static void headset_switch_work(struct work_struct *work)
{
	struct pm860x_headset_info	*info =
		container_of(work, struct pm860x_headset_info, work_headset);
	struct headset_switch_data	*switch_data;
	unsigned char value;

	if (info == NULL) {
		pr_debug("Invalid headset info!\n");
		return;
	}
	switch_data = info->psw_data_headset;
	if (switch_data == NULL) {
		pr_debug("Invalid headset switch data!\n");
		return;
	}

	pm860x_reg_write(info->i2c, 0xd3, 0x3);	

	value = (unsigned char)pm860x_reg_read(info->i2c, PM8607_STATUS_1);
	value &= PM8607_STATUS_HEADSET;
	/* on TD_DKB, the headset jack circuit logic is opposite to the
	 * design of levante spec, so if headset status is connected in
	 * levante spec, it's actually disconnected. */
//	value = info->headset_flag? !value : value;
	value = info->headset_flag? value : !value;
	/* headset detected */
	if (value) {
		switch_data->state = PM860X_HEADSET_ADD;
	
		/* enable MIC bias to enable hook detection, we must enable mic bias first
		* otherwise we may get false hook detection */
		pm860x_set_bits(info->i2c, PM8607_AUDIO_REG_BASE + PM8607_AUDIO_ADC_ANALOG_PROGRAM1, PM8607_ADC_EN_MIC2_BIAS, 0x60);
		/* enable MIC detection to detect hook press*/
		pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 1);

		/* we need to wait some time before the status register goes stable */
		msleep(1500);		

		value = (unsigned char)pm860x_reg_read(info->i2c, PM8607_STATUS_1);
		
		printk("register 0x01  is 0x%x  .\n",value);

		/* Levante issue: use hook status to detect MIC, if detected hook, it's
		 * without MIC, headphone; otherwise, it's headset. */
		value &= PM8607_STATUS_HOOK;
		if (value)
			switch_data->state = PM860X_HEADPHONE_ADD;

		/* unmask hook interrupt only if the headset has a Mic */
		if (switch_data->state == PM860X_HEADSET_ADD) {
			pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_HOOK, PM8607_INT_EN_HOOK);
		} else {
			/* disable MIC/hook detection if headset does not have a Mic */
			pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 0);
	        }

	} 
	else {
		/* headset removed disable MIC/hook detection when headset is */
		pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 0);
		/* disable hook interrupt */
		pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_HOOK, 0);
		/* disable mic bias */
		pm860x_set_bits(info->i2c, PM8607_AUDIO_REG_BASE + PM8607_AUDIO_ADC_ANALOG_PROGRAM1, PM8607_ADC_EN_MIC2_BIAS, 0);

		switch_data->state = PM860X_HEADSET_REMOVE;
	}

	pr_info("headset_switch_work to %d \n", switch_data->state);
	switch_set_state(&switch_data->sdev, switch_data->state);
}

static void hook_switch_work(struct work_struct *work)
{
	struct pm860x_headset_info	*info =
		container_of(work, struct pm860x_headset_info, work_hook);
	struct headset_switch_data	*switch_data;
	unsigned char value;

	if (info == NULL) {
		pr_debug("Invalid headset info!\n");
		return;
		}
	switch_data = info->psw_data_hook;
	if (switch_data == NULL) {
		pr_debug("Invalid hook switch data!\n");
		return;
	}

	value = (unsigned char)pm860x_reg_read(info->i2c, PM8607_STATUS_1);
	value &= PM8607_STATUS_HOOK;

	/* hook pressed */
	if (value) {
		switch_data->state = PM860X_HOOKSWITCH_PRESSED;
	} else {
	/* hook released */
		switch_data->state = PM860X_HOOKSWITCH_RELEASED;
	}

	pr_info("hook state switch to %d \n", switch_data->state);
	switch_set_state(&switch_data->sdev, switch_data->state);
}

static ssize_t switch_headset_print_state(struct switch_dev *sdev, char *buf)
{
	struct headset_switch_data	*switch_data =
		container_of(sdev, struct headset_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static int headset_switch_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_headset_info *info;
	struct pm860x_platform_data *pm860x_pdata;
	struct gpio_switch_platform_data *pdata_headset = pdev->dev.platform_data;
	struct gpio_switch_platform_data *pdata_hook = pdata_headset + 1;
	struct headset_switch_data *switch_data_headset, *switch_data_hook;
	int irq_headset, irq_hook, ret = 0;

	if (pdev->dev.parent->platform_data) {
		pm860x_pdata = pdev->dev.parent->platform_data;
	} else {
		pr_debug("Invalid pm860x platform data!\n");
		return -EINVAL;
	}

	if (pdata_headset == NULL || pdata_hook == NULL) {
		pr_debug("Invalid gpio switch platform data!\n");
		return -EBUSY;
	}

	irq_headset = platform_get_irq(pdev, 0);
	if (irq_headset < 0) {
		dev_err(&pdev->dev, "No IRQ resource for headset!\n");
		return -EINVAL;
	}
	irq_hook = platform_get_irq(pdev, 1);
	if (irq_hook < 0) {
		dev_err(&pdev->dev, "No IRQ resource for hook!\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct pm860x_headset_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->chip = chip;
	info->dev = &pdev->dev;
	info->irq_headset = irq_headset + chip->irq_base;
	info->irq_hook = irq_hook + chip->irq_base;
	info->headset_flag = pm860x_pdata->headset_flag;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;

	switch_data_headset = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data_headset)
		return -ENOMEM;
	switch_data_hook = kzalloc(sizeof(struct headset_switch_data), GFP_KERNEL);
	if (!switch_data_hook)
		return -ENOMEM;

	switch_data_headset->sdev.name = pdata_headset->name;
	switch_data_headset->name_on = pdata_headset->name_on;
	switch_data_headset->name_off = pdata_headset->name_off;
	switch_data_headset->state_on = pdata_headset->state_on;
	switch_data_headset->state_off = pdata_headset->state_off;
	switch_data_headset->sdev.print_state = switch_headset_print_state;
	info->psw_data_headset = switch_data_headset;

	switch_data_hook->sdev.name = pdata_hook->name;
	switch_data_hook->name_on = pdata_hook->name_on;
	switch_data_hook->name_off = pdata_hook->name_off;
	switch_data_hook->state_on = pdata_hook->state_on;
	switch_data_hook->state_off = pdata_hook->state_off;
	switch_data_hook->sdev.print_state = switch_headset_print_state;
	info->psw_data_hook = switch_data_hook;

	ret = switch_dev_register(&switch_data_headset->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
	ret = switch_dev_register(&switch_data_hook->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = request_threaded_irq(info->irq_headset, NULL, pm860x_headset_handler,
				   IRQF_ONESHOT, "headset", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq_headset, ret);
		goto out_irq_headset;
	}
	ret = request_threaded_irq(info->irq_hook, NULL, pm860x_headset_handler,
				   IRQF_ONESHOT, "hook", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq_hook, ret);
		goto out_irq_hook;
	}

	platform_set_drvdata(pdev, info);

	/* set hook detection debounce time to 24ms, it's the best setting we experienced */
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_BTN_DBNC, 0x10);

	//pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_PERIOD, 0x04);
	
	/* set headset period to continuous detection */
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_PERIOD, 0x06);

	/* set MIC detection parameter: MIC period set to 250msec */
	pm860x_reg_write(info->i2c, PM8607_MIC_DECTION, 0xDC);

	/* mask hook interrupt since we don't want the first false hook press down detection
	when inserting a headset without Mic */
	pm860x_set_bits(info->i2c, PM8607_INT_MASK_3, PM8607_INT_EN_HOOK, 0);

	/* enable headset detection */
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_EN_HS_DET, 1);

	INIT_WORK(&info->work_headset, headset_switch_work);
	INIT_WORK(&info->work_hook, hook_switch_work);

	/* Perform initial detection */
	headset_switch_work(&info->work_headset);
	hook_switch_work(&info->work_hook);

	return 0;

err_switch_dev_register:
	kfree(switch_data_headset);
	kfree(switch_data_hook);

out_irq_hook:
	free_irq(info->irq_headset, info);
out_irq_headset:
	kfree(info);
	return ret;
}

static int __devexit headset_switch_remove(struct platform_device *pdev)
{
	struct pm860x_headset_info *info = platform_get_drvdata(pdev);
	struct headset_switch_data *switch_data_headset = info->psw_data_headset;
	struct headset_switch_data *switch_data_hook = info->psw_data_hook;

	/* disable headset detection */
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_EN_HS_DET, 0);

	cancel_work_sync(&info->work_headset);
	cancel_work_sync(&info->work_hook);

	free_irq(info->irq_hook, info);
	free_irq(info->irq_headset, info);

	switch_dev_unregister(&switch_data_hook->sdev);
	switch_dev_unregister(&switch_data_headset->sdev);

	kfree(switch_data_hook);
	kfree(switch_data_headset);
	kfree(info);

	return 0;
}

static int headset_switch_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pm860x_headset_info *info = platform_get_drvdata(pdev);
	struct headset_switch_data *switch_data = info->psw_data_headset;

	/* disable MIC/HOOK detection when headset is connected; no operation is
	 * needed for headphone */
	if (switch_data->state == PM860X_HEADSET_ADD) {
		pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 0);
	}
	
	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_EN_HS_DET, 0);


	return 0;
}


static int headset_switch_resume(struct platform_device *pdev)
{
	struct pm860x_headset_info *info = platform_get_drvdata(pdev);
	struct headset_switch_data *switch_data = info->psw_data_headset;


	/* enable MIC/HOOK detection when headset is connected. */
	if (switch_data->state == PM860X_HEADSET_ADD) {
		pm860x_set_bits(info->i2c, PM8607_MIC_DECTION, PM8607_MIC_DET_EN_MIC_DET, 1);
	}

	pm860x_set_bits(info->i2c, PM8607_HEADSET_DECTION, PM8607_HEADSET_EN_HS_DET, 1);

	return 0;
}


static struct platform_driver headset_switch_driver = {
	.probe		= headset_switch_probe,
	.remove		= __devexit_p(headset_switch_remove),
	.suspend	= headset_switch_suspend,
	.resume 	= headset_switch_resume,
	.driver		= {
		.name	= "88pm860x-headset",
		.owner	= THIS_MODULE,
	},
};

static int __init headset_switch_init(void)
{
	return platform_driver_register(&headset_switch_driver);
}
module_init(headset_switch_init);

static void __exit headset_switch_exit(void)
{
	platform_driver_unregister(&headset_switch_driver);
}
module_exit(headset_switch_exit);

MODULE_DESCRIPTION("Marvell 88PM860x Headset driver");
MODULE_AUTHOR("Raul Xiong <xjian@marvell.com>");
MODULE_LICENSE("GPL");
