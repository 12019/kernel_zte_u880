/*
 * Real Time Clock driver for Marvell 88PM860x PMIC
 *
 * Copyright (c) 2010 Marvell International Ltd.
 * Author:	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/mfd/88pm860x.h>
#include <mach/regs-rtc.h>
#include <mach/regs-apbc.h>

static int irq_1hz = -1, irq_alrm = -1;
static struct rtc_time rtc_alarm;
static DEFINE_SPINLOCK(mmp_rtc_lock);


#define RTC_DEF_DIVIDER		(32768 - 1)
#define RTC_DEF_TRIM		0

#define VRTC_CALIBRATION
static DEFINE_SPINLOCK(pxa_rtc_lock);
static void __iomem *rtc_base;


struct pm860x_rtc_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct rtc_device	*rtc_dev;
	struct device		*dev;

	struct delayed_work	calib_work;
	int			irq;
	int			vrtc;
};

#define REG0_ADDR		0xB0
#define REG1_ADDR		0xB2
#define REG2_ADDR		0xB4
#define REG3_ADDR		0xB6

#define REG0_DATA		0xB1
#define REG1_DATA		0xB3
#define REG2_DATA		0xB5
#define REG3_DATA		0xB7

/* bit definitions of Measurement Enable Register 2 (0x51) */
#define MEAS2_VRTC		(1 << 0)

/* bit definitions of RTC Register 1 (0xA0) */
#define ALARM_EN		(1 << 3)
#define ALARM_WAKEUP		(1 << 4)
#define ALARM			(1 << 5)
#define RTC1_USE_XO		(1 << 7)

#define VRTC_CALIB_INTERVAL	(HZ * 60 * 10)		/* 10 minutes */

#define RTC_DEBUG    1
#if RTC_DEBUG
#define RTC_DBG(fmt, args...)    printk("RTC : " fmt "\n", ## args)
#else if
#define RTC_DBG(fmt, args...)    do {} while (0)
#endif
static int pm860x_rtc_set_time(struct device *dev, struct rtc_time *tm);


static irqreturn_t rtc_update_handler(int irq, void *data)
{
	struct pm860x_rtc_info *info = (struct pm860x_rtc_info *)data;
	int mask;

	mask = ALARM | ALARM_WAKEUP;
	pm860x_set_bits(info->i2c, PM8607_RTC1, mask | ALARM_EN, mask);
	rtc_update_irq(info->rtc_dev, 1, RTC_AF);
	return IRQ_HANDLED;
}

static void wait_2_cycles(void)
{
	/* wait for 2 cycles of 32.768KHz */
	udelay(64);
}

static int pm860x_rtc_alarm_irq_enable(struct device *dev,unsigned int enabled)
{
	struct pm860x_rtc_info *info = dev_get_drvdata(dev);

	if(enabled) {
		pm860x_set_bits(info->i2c, PM8607_RTC1, ALARM, ALARM);
	} else {
		pm860x_set_bits(info->i2c, PM8607_RTC1, ALARM, 0);
	}
	return 0;
}

/*
 * Calculate the next alarm time given the requested alarm time mask
 * and the current time.
 */
static void rtc_next_alarm_time(struct rtc_time *next, struct rtc_time *now,
				struct rtc_time *alrm)
{
	unsigned long next_time;
	unsigned long now_time;

	next->tm_year = now->tm_year;
	next->tm_mon = now->tm_mon;
	next->tm_mday = now->tm_mday;
	next->tm_hour = alrm->tm_hour;
	next->tm_min = alrm->tm_min;
	next->tm_sec = alrm->tm_sec;

	rtc_tm_to_time(now, &now_time);
	rtc_tm_to_time(next, &next_time);

	if (next_time < now_time) {
		/* Advance one day */
		next_time += 60 * 60 * 24;
		rtc_time_to_tm(next_time, next);
	}
}

static int pm860x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct pm860x_rtc_info *info = dev_get_drvdata(dev);
	unsigned char buf[8];
	unsigned long ticks, base, data;
	int ret=0;

	pm860x_page_bulk_read(info->i2c, REG0_ADDR, 8, buf);
	dev_dbg(info->dev, "%x-%x-%x-%x-%x-%x-%x-%x\n", buf[0], buf[1],
		buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	base = (buf[1] << 24) | (buf[3] << 16) | (buf[5] << 8) | buf[7];

	/* load 32-bit read-only counter */
	pm860x_bulk_read(info->i2c, PM8607_RTC_COUNTER1, 4, buf);
	data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	ticks = base + data;
	dev_dbg(info->dev, "get base:0x%lx, RO count:0x%lx, ticks:0x%lx\n",
		base, data, ticks);

	rtc_time_to_tm(ticks, tm);
	if((tm->tm_year < 70) ||(ticks >= 0x7FFFFFFF) /*(tm->tm_year > 138)*/) {
		tm->tm_year = 70;
		tm->tm_mon = 0;
		tm->tm_mday = 1;
		tm->tm_hour = 0;
		tm->tm_min = 0;
		tm->tm_sec = 0;
		ret = pm860x_rtc_set_time(dev,tm);
		if (ret < 0) {
			dev_err(dev, "Failed to set initial time.\n");
			
		}
	}
        RTC_DBG("rtc tm Get time year=%u, mon=%u, day=%u, week=%u, hour=%u, min=%u, sec=%u",
            tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	RCNR=ticks;
	
	return 0;
}

static int pm860x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct pm860x_rtc_info *info = dev_get_drvdata(dev);
	unsigned char buf[4];
	unsigned long ticks, base, data;

	rtc_tm_to_time(tm, &ticks);
	if((tm->tm_year < 70) ||(ticks >= 0x7FFFFFFF) /*(tm->tm_year > 138)*/) {
		dev_dbg(info->dev, "set time %d out of range, please set time between 1970 to 2038.\n",
			1900+tm->tm_year);
		return -EINVAL;
	}
RTC_DBG("Set time year=%u, mon=%u, day=%u, week=%u, hour=%u, min=%u, sec=%u",
	 tm->tm_year+1900,tm->tm_mon+1, tm->tm_mday, tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);
	RCNR=ticks;

	/* load 32-bit read-only counter */
	pm860x_bulk_read(info->i2c, PM8607_RTC_COUNTER1, 4, buf);
	data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	base = ticks - data;
	dev_dbg(info->dev, "set base:0x%lx, RO count:0x%lx, ticks:0x%lx\n",
		base, data, ticks);

	pm860x_page_reg_write(info->i2c, REG0_DATA, (base >> 24) & 0xFF);
	pm860x_page_reg_write(info->i2c, REG1_DATA, (base >> 16) & 0xFF);
	pm860x_page_reg_write(info->i2c, REG2_DATA, (base >> 8) & 0xFF);
	pm860x_page_reg_write(info->i2c, REG3_DATA, base & 0xFF);
	return 0;
}

static int pm860x_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pm860x_rtc_info *info = dev_get_drvdata(dev);
	unsigned char buf[8];
	unsigned long ticks, base, data;
	int ret;

	pm860x_page_bulk_read(info->i2c, REG0_ADDR, 8, buf);
	dev_dbg(info->dev, "%x-%x-%x-%x-%x-%x-%x-%x\n", buf[0], buf[1],
		buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
	base = (buf[1] << 24) | (buf[3] << 16) | (buf[5] << 8) | buf[7];

	pm860x_bulk_read(info->i2c, PM8607_RTC_EXPIRE1, 4, buf);
	data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
	ticks = base + data;
	dev_dbg(info->dev, "get base:0x%lx, RO count:0x%lx, ticks:0x%lx\n",
		base, data, ticks);

	rtc_time_to_tm(ticks, &alrm->time);
	ret = pm860x_reg_read(info->i2c, PM8607_RTC1);
	alrm->enabled = (ret & ALARM_EN) ? 1 : 0;
	alrm->pending = (ret & (ALARM | ALARM_WAKEUP)) ? 1 : 0;
	return 0;
}

static int pm860x_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct pm860x_rtc_info *info = dev_get_drvdata(dev);
	struct rtc_time now_tm, alarm_tm;
	unsigned long ticks, base, data;
	unsigned char buf[8];
	struct rtc_time now;
       unsigned long  time;
	unsigned long reg;

        printk("alrm->enabled...... = %d\n", alrm->enabled);
        RTC_DBG("Set alarm time year=%u, mon=%u, day=%u, week=%u, hour=%u, min=%u, sec=%u",
	 alrm->time.tm_year+1900,alrm->time.tm_mon+1, alrm->time.tm_mday, alrm->time.tm_wday, alrm->time.tm_hour, alrm->time.tm_min, alrm->time.tm_sec);

	int mask;
       if (alrm->enabled == POWEROFF_WAKEUP_ENABLE||alrm->enabled == POWEROFF_WAKEUP_DISABLE)
       {
		pm860x_set_bits(info->i2c, PM8607_RTC1, ALARM_EN, 0);

		pm860x_page_bulk_read(info->i2c, REG0_ADDR, 8, buf);
		dev_dbg(info->dev, "%x-%x-%x-%x-%x-%x-%x-%x\n", buf[0], buf[1],
		buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
		base = (buf[1] << 24) | (buf[3] << 16) | (buf[5] << 8) | buf[7];

		/* load 32-bit read-only counter */
		pm860x_bulk_read(info->i2c, PM8607_RTC_COUNTER1, 4, buf);
		data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
		ticks = base + data;
		dev_dbg(info->dev, "get base:0x%lx, RO count:0x%lx, ticks:0x%lx\n",
		base, data, ticks);

#if 0
		rtc_time_to_tm(ticks, &now_tm);
		rtc_next_alarm_time(&alarm_tm, &now_tm, &alrm->time);
		/* get new ticks for alarm in 24 hours */
		rtc_tm_to_time(&alarm_tm, &ticks);
#else
		rtc_tm_to_time(&alrm->time, &ticks);
#endif

		data = ticks - base;

		buf[0] = data & 0xff;
		buf[1] = (data >> 8) & 0xff;
		buf[2] = (data >> 16) & 0xff;
		buf[3] = (data >> 24) & 0xff;
		pm860x_bulk_write(info->i2c, PM8607_RTC_EXPIRE1, 4, buf);

		if(alrm->enabled==POWEROFF_WAKEUP_ENABLE){
			mask = ALARM | ALARM_WAKEUP | ALARM_EN;
			pm860x_set_bits(info->i2c, PM8607_RTC1, mask, mask);
		} else {
			mask = ALARM | ALARM_WAKEUP | ALARM_EN;
			pm860x_set_bits(info->i2c, PM8607_RTC1, mask, ALARM | ALARM_WAKEUP);
		}
	
       }
	else  if (alrm->enabled == RTC_WAKEUP_ENABLE||alrm->enabled == RTC_WAKEUP_DISABLE)
	{
	      pm860x_rtc_read_time(dev, &now);
		rtc_tm_to_time(&now, &time);
		RCNR = time;
	      rtc_tm_to_time(&alrm->time, &reg);
		spin_lock_irq(&pxa_rtc_lock);
		RTSR = RTSR & (RTSR_HZE|RTSR_ALE|RTSR_AL);


		if (alrm->enabled == RTC_WAKEUP_ENABLE)
		{
			printk("RTC_WAKEUP_ENABLE\n");
			#if 1
			RTAR = reg;
			RTSR |= RTSR_ALE;
			#endif	
		}
		else
		{
			printk("RTC_WAKEUP_DISABLE\n");
			RTAR = 0;
			RTSR &= ~RTSR_ALE;
		}
		spin_unlock_irq(&pxa_rtc_lock);
		wait_2_cycles();
	}
	return 0;
}

static const struct rtc_class_ops pm860x_rtc_ops = {
	.read_time	= pm860x_rtc_read_time,
	.set_time	= pm860x_rtc_set_time,
	.read_alarm	= pm860x_rtc_read_alarm,
	.set_alarm	= pm860x_rtc_set_alarm,
	.alarm_irq_enable = pm860x_rtc_alarm_irq_enable,
};

#ifdef VRTC_CALIBRATION
static void calibrate_vrtc_work(struct work_struct *work)
{
	struct pm860x_rtc_info *info = container_of(work,
		struct pm860x_rtc_info, calib_work.work);
	unsigned char buf[2];
	unsigned int sum, data, mean, vrtc_set;
	int i;

	for (i = 0, sum = 0; i < 16; i++) {
		msleep(100);
		pm860x_bulk_read(info->i2c, PM8607_VRTC_MEAS1, 2, buf);
		data = (buf[0] << 4) | buf[1];
		data = (data * 5400) >> 12;	/* convert to mv */
		sum += data;
	}
	mean = sum >> 4;
	vrtc_set = 2700 + (info->vrtc & 0x3) * 200;
	dev_dbg(info->dev, "mean:%d, vrtc_set:%d\n", mean, vrtc_set);

	sum = pm860x_reg_read(info->i2c, PM8607_RTC_MISC1);
	data = sum & 0x3;
	if ((mean + 200) < vrtc_set) {
		/* try higher voltage */
		if (++data == 4)
			goto out;
		data = (sum & 0xf8) | (data & 0x3);
		pm860x_reg_write(info->i2c, PM8607_RTC_MISC1, data);
	} else if ((mean - 200) > vrtc_set) {
		/* try lower voltage */
		if (data-- == 0)
			goto out;
		data = (sum & 0xf8) | (data & 0x3);
		pm860x_reg_write(info->i2c, PM8607_RTC_MISC1, data);
	} else
		goto out;
	dev_dbg(info->dev, "set 0x%x to RTC_MISC1\n", data);
	/* trigger next calibration since VRTC is updated */
	queue_delayed_work(info->chip->monitor_wqueue, &info->calib_work,
			   VRTC_CALIB_INTERVAL);
	return;
out:
	/* disable measurement */
	pm860x_set_bits(info->i2c, PM8607_MEAS_EN2, MEAS2_VRTC, 0);
	dev_dbg(info->dev, "finish VRTC calibration\n");
	return;
}
#endif

static int rtc_update_alarm(struct rtc_time *alrm)
{
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time;
	int ret;


	rtc_tm_to_time(alrm, &time);
	do {
		now = RCNR;
		printk("rtc_update_alarm:RCNR=%ld\n",RCNR);
		printk("rtc_update_alarm:alalrm time=%ld\n",time);

		rtc_time_to_tm(now, &now_tm);
		rtc_next_alarm_time(&alarm_tm, &now_tm, alrm);
		ret = rtc_tm_to_time(&alarm_tm, &time);
		if (ret != 0)
			break;

		RTSR = RTSR & (RTSR_HZE|RTSR_ALE|RTSR_AL);
		RTAR = time;
	} while (now != RCNR);

	return ret;
}
static inline int rtc_periodic_alarm(struct rtc_time *tm)
{
	return  (tm->tm_year == -1) ||
		((unsigned)tm->tm_mon >= 12) ||
		((unsigned)(tm->tm_mday - 1) >= 31) ||
		((unsigned)tm->tm_hour > 23) ||
		((unsigned)tm->tm_min > 59) ||
		((unsigned)tm->tm_sec > 59);
}

static irqreturn_t mmp_rtc_interrupt(int irq, void *dev_id)
{

	struct platform_device *pdev = to_platform_device(dev_id);
	//struct rtc_device *rtc = platform_get_drvdata(pdev);
	struct pm860x_rtc_info *info = platform_get_drvdata(pdev);

	unsigned int rtsr;
	unsigned long events = 0;

	spin_lock(&mmp_rtc_lock);

	rtsr = RTSR;
	/* clear interrupt sources */
	RTSR = 0;
	RTSR = (RTSR_AL | RTSR_HZ) & (rtsr >> 2);

	/* clear alarm interrupt if it has occurred */
	if (rtsr & RTSR_AL)
		rtsr &= ~RTSR_ALE;
	RTSR = rtsr & (RTSR_ALE | RTSR_HZE);

	/* update irq data & counter */
	if (rtsr & RTSR_AL)
		events |= RTC_AF | RTC_IRQF;
	if (rtsr & RTSR_HZ)
		events |= RTC_UF | RTC_IRQF;

	rtc_update_irq(info->rtc_dev, 1, events);

	//if (rtsr & RTSR_AL && rtc_periodic_alarm(&rtc_alarm))
	//	rtc_update_alarm(&rtc_alarm);

	spin_unlock(&mmp_rtc_lock);

	return IRQ_HANDLED;
}


static int __devinit pm860x_rtc_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_platform_data *pm860x_pdata;
	struct pm860x_rtc_pdata *pdata = NULL;
	struct pm860x_rtc_info *info;
	struct device *dev = &pdev->dev;
	struct rtc_time tm;
	unsigned long ticks = 0;


	int ret;
		RTC_DBG("pm860x_rtc_probe:enter\n");

	info = kzalloc(sizeof(struct pm860x_rtc_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->irq = platform_get_irq(pdev, 0);
	if (info->irq < 0) {
		RTC_DBG("pm860x_rtc_probe:error0\n");
		dev_err(&pdev->dev, "No IRQ resource!\n");
		ret = -EINVAL;
		goto out;
	}

	irq_1hz =IRQ_PXA168_RTC_INT;
	irq_alrm =IRQ_PXA168_RTC_ALARM;
	
        *(volatile u32  *)APBC_PXA910_RTC=0x83;//enable rtc clk

	#if 1
	ret = request_irq(irq_1hz, mmp_rtc_interrupt, IRQF_DISABLED,
				"rtc 1Hz", dev);
	if (ret) {
		printk("IRQ %d already in use.\n", irq_1hz);
		irq_1hz = irq_alrm = -1;
		ret = -ENXIO;
		goto out;
	}
	disable_irq(irq_1hz);

	ret = request_irq(irq_alrm, mmp_rtc_interrupt, IRQF_DISABLED,
				"rtc Alrm", dev);
	if (ret) {
		dev_err(dev, "IRQ %d already in use.\n", irq_alrm);
		irq_alrm = -1;
		ret = -ENXIO;
		goto out;
	}
	//disable_irq(irq_alrm);
#endif



        
	info->chip = chip;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	info->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, info);

	/* set addresses of 32-bit base value for RTC time */
	pm860x_page_reg_write(info->i2c, REG0_ADDR, REG0_DATA);
	pm860x_page_reg_write(info->i2c, REG1_ADDR, REG1_DATA);
	pm860x_page_reg_write(info->i2c, REG2_ADDR, REG2_DATA);
	pm860x_page_reg_write(info->i2c, REG3_ADDR, REG3_DATA);

      //0xd4010000-rtc base addr
	rtc_base = ioremap(0xd4010000, 0x100);
	 printk("rtc_base=0x%x\n",rtc_base); 
	if (rtc_base == NULL) {
	     printk("failed to remap I/O memory\n");
		ret = -ENXIO;
		goto out_rtc;
	}

	if (RTTR == 0) {
		RTTR = RTC_DEF_DIVIDER + (RTC_DEF_TRIM << 16);
		dev_warn(&pdev->dev, "warning: initializing default clock divider/trim value\n");
		/* The current RTC value probably doesn't make sense either */
		RCNR = 0;
	}

	ret = pm860x_rtc_read_time(&pdev->dev,&tm);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read initial time.\n");
		goto out_rtc;
	}
	if((tm.tm_year < 70) || (tm.tm_year > 138)) {
		tm.tm_year = 70;
		tm.tm_mon = 0;
		tm.tm_mday = 1;
		tm.tm_hour = 0;
		tm.tm_min = 0;
		tm.tm_sec = 0;
		ret = pm860x_rtc_set_time(&pdev->dev,&tm);
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to set initial time.\n");
			goto out_rtc;
		}
	}
        rtc_tm_to_time(&tm, &ticks);
	RCNR = ticks;
	dev_dbg(info->dev, "%s, ticks:0x%lx, RCNR:0x%x\n", __func__,ticks,RCNR);
	device_init_wakeup(&pdev->dev, 1);
	info->rtc_dev = rtc_device_register("88pm860x-rtc", &pdev->dev,
					    &pm860x_rtc_ops, THIS_MODULE);
	ret = PTR_ERR(info->rtc_dev);
	if (IS_ERR(info->rtc_dev)) {
				RTC_DBG("pm860x_rtc_probe:error2\n");

		dev_err(&pdev->dev, "Failed to register RTC device: %d\n", ret);
		goto out_rtc;
	}

	ret = request_threaded_irq(info->irq, NULL, rtc_update_handler,
				   IRQF_ONESHOT, "rtc", info);
	if (ret < 0) {
				RTC_DBG("pm860x_rtc_probe:error1\n");

		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, ret);
		goto out;
	}
	
	/*
	 * enable internal XO instead of internal 3.25MHz clock since it can
	 * free running in PMIC power-down state.
	 */
	pm860x_set_bits(info->i2c, PM8607_RTC1, RTC1_USE_XO, RTC1_USE_XO);

#ifdef VRTC_CALIBRATION
	info->vrtc = 1;			/* By default, VRTC is 2.9V */
	if (pdev->dev.parent->platform_data) {
		pm860x_pdata = pdev->dev.parent->platform_data;
		pdata = pm860x_pdata->rtc;
		if (pdata)
			info->vrtc = pdata->vrtc & 0x3;
	}
	pm860x_set_bits(info->i2c, PM8607_MEAS_EN2, MEAS2_VRTC, MEAS2_VRTC);

	/* calibrate VRTC */
	INIT_DELAYED_WORK(&info->calib_work, calibrate_vrtc_work);
	queue_delayed_work(chip->monitor_wqueue, &info->calib_work,
			   VRTC_CALIB_INTERVAL);
#endif	/* VRTC_CALIBRATION */
	return 0;
out_rtc:
	free_irq(info->irq, info);
out:
	kfree(info);
	return ret;
}

static int __devexit pm860x_rtc_remove(struct platform_device *pdev)
{
	struct pm860x_rtc_info *info = platform_get_drvdata(pdev);

#ifdef VRTC_CALIBRATION
	flush_workqueue(info->chip->monitor_wqueue);
	/* disable measurement */
	pm860x_set_bits(info->i2c, PM8607_MEAS_EN2, MEAS2_VRTC, 0);
#endif	/* VRTC_CALIBRATION */

	platform_set_drvdata(pdev, NULL);
	rtc_device_unregister(info->rtc_dev);
	free_irq(info->irq, info);
	kfree(info);
	iounmap(rtc_base);
	return 0;
}

static struct platform_driver pm860x_rtc_driver = {
	.driver		= {
		.name	= "88pm860x-rtc",
		.owner	= THIS_MODULE,
	},
	.probe		= pm860x_rtc_probe,
	.remove		= __devexit_p(pm860x_rtc_remove),
};

static int __init pm860x_rtc_init(void)
{
	return platform_driver_register(&pm860x_rtc_driver);
}
module_init(pm860x_rtc_init);

static void __exit pm860x_rtc_exit(void)
{
	platform_driver_unregister(&pm860x_rtc_driver);
}
module_exit(pm860x_rtc_exit);

MODULE_DESCRIPTION("Marvell 88PM860x RTC driver");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
