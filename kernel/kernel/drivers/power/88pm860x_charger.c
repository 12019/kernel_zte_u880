/*
 * Battery driver for Marvell 88PM860x PMIC
 *
 * Copyright (c) 2009-2010 Marvell International Ltd.
 * Author:	Jett Zhou <jtzhou@marvell.com>
 *		Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/mfd/88pm860x.h>
#include <asm/div64.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>


/* bit definitions of Status Query Interface 2 */
#define STATUS2_CHG		(1 << 2)

/* bit definitions of Reset Out Register */
#define RESET_SW_PD		(1 << 7)

/* bit definitions of PreReg 1 */
#define PREREG1_1500MA		(0x0E)
#define PM8606_PREREG_CURLIM_SET_450V   (0x4 << 0)
#define PM8606_PREREG_CURLIM_SET_540V   (0x5 << 0)
#define PM8606_PREREG_CURLIM_SET_630V   (0x6 << 0)
#define PM8606_PREREG_CURLIM_SET_810V   (0x7 << 0)

#define PREREG1_VSYS_4_5V	(3 << 4)

/* bit definitions of Charger Control 1 Register */
#define CC1_MODE_OFF		(0)
#define CC1_MODE_PRECHARGE	(1)
#define CC1_MODE_FASTCHARGE	(2)
#define CC1_MODE_PULSECHARGE	(3)
#define CC1_ITERM_20MA		(0 << 2)
#define CC1_ITERM_70MA		(0x2<<2)

#define CC1_VFCHG_4_2V		(0x9 << 4)

/* bit definitions of Charger Control 2 Register */
#define CC2_ICHG_500MA		(8)
#define CC2_ICHG_550MA      (0x0A)
#define CC2_ICHG_650MA      (0x0C)
#define CC2_ICHG_1000MA		(0x13)
#define CC2_ICHG_700MA		(0xE)

/* bit definitions of Charger Control 3 Register */
#define CC3_180MIN_TIMEOUT	(0x6 << 4)
#define CC3_270MIN_TIMEOUT	(0x7 << 4)
#define CC3_360MIN_TIMEOUT	(0xA << 4)
#define CC3_DISABLE_TIMEOUT	(0xF << 4)

/* bit definitions of Charger Control 4 Register */
#define CC4_IPRE_40MA		(7)
#define CC4_VPCHG_3_2V		(3 << 4)
#define CC4_IFCHG_MON_EN	(1 << 6)
#define CC4_BTEMP_MON_EN	(1 << 7)

/* bit definitions of Charger Control 6 Register */
#define CC6_BAT_OV_EN		(1 << 2)
#define CC6_BAT_UV_EN		(1 << 3)
#define CC6_UV_VBAT_SET		(0x3 << 6)/*2.8v*/

/* bit definitions of Charger Control 7 Register */
#define CC7_BAT_REM_EN		(1 << 3)
#define CC7_IFSM_EN		(1 << 7)

/* bit definitions of Measurement Enable 1 Register */
#define MEAS1_VBAT		(1 << 0)

/* bit definitions of Measurement Enable 3 Register */
#define MEAS3_IBAT_EN		(1 << 0)
#define MEAS3_CC_EN		(1 << 2)

#define FSM_INIT		0
#define FSM_DISCHARGE		1
#define FSM_PRECHARGE		2
#define FSM_FASTCHARGE		3
#define FSM_POWEROFF		4

#define VCHG_TEC_MATAIN_TH 		5700
#define VCHG_TEC_LOW_TH 	5000
#define VCHG_C_UPP_TH 		8500
#define VCHG_ALL_LOW_TH 	3000
#define VCHG_ALL_UPP_TH 	6290	
#define PM8607_VCHG_UPP_TH		0x64
#define PM8607_VCHG_LOW_TH		0x5c
#define PRECHARGE_THRESHOLD	3100
#define POWEROFF_THRESHOLD	3380
#define LOWBAT_THRESHOLD	3540
#define CHARGE_THRESHOLD	4050
#define DISCHARGE_THRESHOLD	4130
/* Temperature is reverse from GPADC1 voltage*/
#define GPADC1_DEGREE_MIN	1462
#define GPADC1_DEGREE_MAX	15
#define DEGREE_MIN		-25
#define DEGREE_MAX		100
extern u8 ovchprotect;
extern u8 low_bat_on;
static u8 ac_usb=0;
static u8 charge_done=0;
static u8 low_bat_threshold=0;
extern u8 chg_full_en;
extern int charger_flag;
extern u8 full_excep;
static int excep_flag = 0;
static int usb_online_flag = 0;
/*over-temperature on PM8606 setting*/
#define OVER_TEMP_FLAG		1<<6
#define OVTEMP_AUTORECOVER	1<<3
/*over-voltage protect on vchg setting mv*/
#define VCHG_NORMAL_LOW		4200
#define VCHG_NORMAL_CHECK  5800

#define VCHG_OVP_LOW		5000
extern int vchg_normal_high;
enum enum_charger_type type_backup=USB_CHARGER;
static struct delayed_work	monitor_charger_type;

struct pm860x_charger_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct i2c_client	*i2c_8606;
	struct device		*dev;

	struct power_supply	usb;
	struct power_supply	ac;
	struct mutex		lock;
	int			irq_nums;
	int			irq[7];
	unsigned		state : 3;	/* fsm state */
	unsigned		charge_type : 2;
	unsigned		online : 1;/* pc usb*/
	unsigned		ac_online : 1;
	unsigned		present : 1;	/* battery present */
	unsigned 		allowed : 1;
	unsigned		bc_short;	/*1 disable 0 enable */
};

static char *pm860x_supplied_to[] = {
	"battery",
};
#define loop 20
static struct pm860x_charger_info *ginfo=NULL;
struct pm860x_charger_info *g_chinfo=NULL;

extern u8 ovchprotect;
 extern struct delayed_work	charger_init_detect_work;
extern int volt_timer_count;
extern int volt_loop[loop];
extern int version_880;
extern unsigned char zte_prodtestmod;
static void set_charger_type_func(void);

static int stop_charge(struct pm860x_charger_info *info, int vbatt);
static int set_charging_fsm(struct pm860x_charger_info *info);
static void set_vbatt_threshold(struct pm860x_charger_info *info,
				int min, int max);
static void set_batt_temp_threshold(struct pm860x_charger_info *info,
				int min, int max);
static void set_vchg_threshold(struct pm860x_charger_info *info,
				int min, int max);
static int measure_vchg(struct pm860x_charger_info *info, int *data);
static struct wake_lock lowbat_wakeup;
static struct wake_lock usb_ac_wakelock;
static struct wake_lock charger_remove_wakelock;


extern void fixed_cpu_624M(void);
extern void free_cpu_624M(void);
static int isFiex_624M=0;
static int ac_usb_wakelock_count=0;
static int set_type_en = 0;
struct timer_list	set_charger_timer;	

void pm860x_charger_shutdown(void)
{
	if(pm860x_reg_read(ginfo->i2c, PM8607_CHG_CTRL1)&(0x03))
	{
	pm860x_set_bits(ginfo->i2c, PM8607_CHG_CTRL1, 3, CC1_MODE_OFF);
		printk("PM8607_CHG_CTRL1 set \n");
	}
	else
	{
		printk("PM8607_CHG_CTRL1 not set \n");
	}

}
void pm860x_set_charger_type(enum enum_charger_type type )
{
	struct power_supply *psy;
	union power_supply_propval data;
	struct pm860x_charger_info *info = ginfo;
	int ret = -EINVAL;
	int vbatt;

	psy = power_supply_get_by_name(pm860x_supplied_to[0]);
	if (!psy)
	{
		
		type_backup=type;
		if(set_type_en == 0)
		{
		INIT_DELAYED_WORK(&monitor_charger_type, set_charger_type_func);
			set_type_en = 1;
		}
 		schedule_delayed_work(&monitor_charger_type, 3*HZ);
		
		goto out;
	}

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &data);
	if (ret)
		goto out;
	vbatt = data.intval / 1000;
	if((vbatt)>=4140)
		chg_full_en = 1;
	else
		chg_full_en = 0;

	//printk(KERN_ALERT"charger type:%s detect...\n",(!type) ? "usb" : "ac");
	printk(KERN_ALERT"charger type:%d detect...\n",type);

	if(version_880==0)
	{
			if(zte_prodtestmod)
			{
				printk(" su guo song androidboot.bsp enter \n");
				return;
			}
	}
	mutex_lock(&info->lock);
	switch(type)
	{
		case USB_CHARGER:
			info->charge_type = USB_CHARGER;
			info->online = 1;
			info->ac_online = 0;			
			ac_usb = 1;
			break;
		case AC_STANDARD_CHARGER:
			info->charge_type = AC_STANDARD_CHARGER;
			info->online = 0;
			info->ac_online = 1;
			ac_usb = 1;
			break;
		case AC_OTHER_CHARGER:
			info->charge_type = AC_OTHER_CHARGER;
			info->online = 0;
			info->ac_online = 1;
			ac_usb = 1;
			break;
		default:
			ac_usb = 0;
			break;

	}
	if(usb_online_flag ==0)
		{
			info->online = 0;
			info->ac_online = 0;
			ac_usb = 0;
		charge_done=0;
	volt_timer_count = 0;
	chg_full_en = 0;
	ovchprotect = 0;
	full_excep = 0;
	usb_online_flag = 0;
		}

	if(ac_usb)
		{

	    if(ac_usb_wakelock_count==0)
		{
	        wake_lock(&usb_ac_wakelock);
			ac_usb_wakelock_count=1;
		}
		 #if 1
		  if(isFiex_624M==0)
		  {
		    printk("set 624M HZ\n");
		    fixed_cpu_624M();
		    isFiex_624M=1;
		  // wake_lock(&usb_ac_wakelock);
		  }
		  #endif
		}
	else
		{
		  #if 1
		  if(isFiex_624M==1)
		  {
		   printk("free 624M HZ\n");
		   free_cpu_624M();
		   isFiex_624M=0;
		   // wake_unlock(&usb_ac_wakelock);

		   }
		  #endif
		}

	if (info->state == FSM_FASTCHARGE&&ac_usb == 1){
		/* stop charging */
		//printk("pm860x_set_charger_type:FSM_FASTCHARGE\n");
		ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3,
				      CC1_MODE_OFF);
		if (ret < 0)
			goto out;
		//printk("pm860x_set_charger_type:info->charge_type=%d\n",info->charge_type);

		/*set chg current*/
		switch (info->charge_type) {
			case USB_CHARGER:
				ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f,
						      CC2_ICHG_500MA);
				break;
			case AC_STANDARD_CHARGER:
				ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f,
						      CC2_ICHG_550MA);
				break;
			case AC_OTHER_CHARGER:
				ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f,
						      CC2_ICHG_550MA);
				break;
			default:
				ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f,
						      CC2_ICHG_500MA);
				break;
		}
			/* trigger fastcharge */
	    	//printk("end pm860x_set_charger_type\n");
	if(info->online)
	ret = pm860x_reg_write(info->i2c_8606, PM8606_PREREGULATORA,
			       PM8606_PREREG_CURLIM_SET_450V | PREREG1_VSYS_4_5V);
	else if(info->ac_online)
	ret = pm860x_reg_write(info->i2c_8606, PM8606_PREREGULATORA,
			       PM8606_PREREG_CURLIM_SET_540V | PREREG1_VSYS_4_5V);
	else
	ret = pm860x_reg_write(info->i2c_8606, PM8606_PREREGULATORA,
			       PM8606_PREREG_CURLIM_SET_450V | PREREG1_VSYS_4_5V);

		ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3,
					  CC1_MODE_FASTCHARGE);
	}

	power_supply_changed(&info->usb);
	mutex_unlock(&info->lock);
	
out:
	return;
}

extern void usb_switch_uart(void);
extern void uart_switch_usb(void);
extern  u8 chg_full ;


void handle_charger_init_detect_work(void)
{
       struct pm860x_charger_info *info = ginfo;
	int ret;
	       printk("enter handle_charger_init_detect_work\n");
		mutex_lock(&info->lock);
		ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
		if (ret < 0) {
			mutex_unlock(&info->lock);
			return;
		}
		if (ret & STATUS2_CHG) {
			//printk(" pm860x_charger_handler:STATUS2_CHG\n");
			
			if(ac_usb_wakelock_count==0)
			  {
			     wake_lock(&usb_ac_wakelock);
			     ac_usb_wakelock_count=1;
			  }
			 uart_switch_usb();

			info->online = 1;
			info->allowed = 1;
			usb_online_flag = 1;
			/* GPADC1 (battery temperature) */
			set_batt_temp_threshold(info, GPADC1_DEGREE_MIN, GPADC1_DEGREE_MAX);
		} else {
			//printk(" pm860x_charger_handler:no CHG\n");
			chg_full = 0;
			if(ac_usb_wakelock_count==1)
			{
			  wake_unlock(&usb_ac_wakelock);
			  ac_usb_wakelock_count=0;
			}
			usb_switch_uart();
			info->online = 0;
			info->ac_online = 0;
			info->allowed = 0;
			ac_usb = 0;
			charge_done=0;
			usb_online_flag = 0;
		}
		mutex_unlock(&info->lock);
		dev_dbg(info->dev, "%s, USB:%s, Allowed:%d\n", __func__,
			(info->online) ? "online" : "N/A", info->allowed);

		set_charging_fsm(info);
	
 
}

irqreturn_t pm860x_charger_handler(int irq, void *data)
{
	struct pm860x_charger_info *info = ginfo;
	int ret,i=0;
         printk("enter pm860x_charger_handler\n");
	
	if(ginfo==NULL)
	{
	   schedule_delayed_work(&charger_init_detect_work, msecs_to_jiffies(3000));
	}
	else
	{
	mutex_lock(&info->lock);
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (ret < 0) {
		mutex_unlock(&info->lock);
		goto out;
	}
	if (ret & STATUS2_CHG) {
			//printk(" pm860x_charger_handler:STATUS2_CHG\n");
		  if(ac_usb_wakelock_count==0)
		  {
		   wake_lock(&usb_ac_wakelock);
		   ac_usb_wakelock_count=1;
		  }
		 uart_switch_usb();

		info->online = 1;
		info->allowed = 1;
		/* GPADC1 (battery temperature) */
		set_batt_temp_threshold(info, GPADC1_DEGREE_MIN, GPADC1_DEGREE_MAX);

		usb_online_flag = 1;
		charger_flag = 1;
				
	} else {
			//printk(" pm860x_charger_handler:no CHG\n");
		wake_lock_timeout(&charger_remove_wakelock,3*HZ);
		chg_full = 0;
	 
		if(ac_usb_wakelock_count==1)
		  {
		    wake_unlock(&usb_ac_wakelock);
		    ac_usb_wakelock_count=0;
		   }
		usb_switch_uart();
		info->online = 0;
		info->ac_online = 0;
		info->allowed = 0;
		ac_usb = 0;
		charge_done=0;
	volt_timer_count = 0;
	chg_full_en = 0;
	ovchprotect = 0;
	charger_flag = 1;
	full_excep = 0;
	usb_online_flag = 0;
	
   	for (i=0; i<loop; i++)
        {
		volt_loop[i]=0;
        }
		
	}
	mutex_unlock(&info->lock);
	dev_dbg(info->dev, "%s, USB:%s, Allowed:%d\n", __func__,
		(info->online) ? "online" : "N/A", info->allowed);

	set_charging_fsm(info);
	}
         printk("end pm860x_charger_handler\n");
	low_bat_threshold = 0;
out:
	return IRQ_HANDLED;
}

static irqreturn_t pm860x_temp_handler(int irq, void *data)
{
	struct pm860x_charger_info *info = data;
	struct power_supply *psy;
	union power_supply_propval val;
	int temp, ret;
	  printk("enter pm860x_temp_handler\n");

	info->allowed = 1;
	goto out;
      //  printk("enter pm860x_temp_handler\n");
	/* clear GPADC1 threshold (battery temperature) */
	pm860x_reg_write(info->i2c, PM8607_GPADC1_LOWTH, 0x0);
	pm860x_reg_write(info->i2c, PM8607_GPADC1_HIGHTH, 0xff);

	psy = power_supply_get_by_name(pm860x_supplied_to[0]);
	if (!psy)
		goto out;
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		goto out;
	temp = val.intval;

	mutex_lock(&info->lock);
	if (temp > DEGREE_MIN && temp < DEGREE_MAX)
		info->allowed = 1;
	else
		info->allowed = 1;
	dev_dbg(info->dev, "%s, Allowed:%d\n", __func__, info->allowed);
	mutex_unlock(&info->lock);

	set_charging_fsm(info);
out:
	return IRQ_HANDLED;
}

static irqreturn_t pm860x_exception_handler(int irq, void *data)
{
	struct pm860x_charger_info *info = data;
	int ret =0;
        printk("enter pm860x_exception_handler\n");
	struct power_supply *psy;
	union power_supply_propval vdata;
	//int ret = -EINVAL;
	psy = power_supply_get_by_name(pm860x_supplied_to[0]);
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vdata);

	mutex_lock(&info->lock);
	//info->allowed = 0;
	mutex_unlock(&info->lock);
	dev_dbg(info->dev, "%s, irq:%d\n", __func__, irq);
	printk("%s, irq:%d\n", __func__, irq);
	if(vdata.intval/1000 >=4140)
	{
		full_excep = 1;
		info->allowed = 0;
	}
	else
	{
	
		full_excep = 0;
		info->allowed = 1;
		excep_flag = 1;
		
	}
	set_charging_fsm(info);
	return IRQ_HANDLED;
}

static irqreturn_t pm860x_done_handler(int irq, void *data)
{
	struct pm860x_charger_info *info = data;
        printk("enter pm860x_done_handler  info -> state ====%d\n",info->state);
	

	mutex_lock(&info->lock);
	if (info->state == FSM_PRECHARGE||info->state==FSM_FASTCHARGE)
		info->allowed = 1;
	else
		info->allowed = 0;
	if (info->state==FSM_FASTCHARGE)
		charge_done=1;
	dev_dbg(info->dev, "%s, Allowed:%d\n", __func__, info->allowed);
	mutex_unlock(&info->lock);

	set_charging_fsm(info);
	return IRQ_HANDLED;
}

static irqreturn_t pm860x_vbattery_handler(int irq, void *data)
{
	struct pm860x_charger_info *info = data;
        printk("enter pm860x_vbattery_handler\n");

	struct power_supply *psy;
	union power_supply_propval vdata;
	int vbatt;
	int ret = -EINVAL;
	psy = power_supply_get_by_name(pm860x_supplied_to[0]);
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vdata);
	vbatt = vdata.intval / 1000;

	
	if(info->online==0&&info->ac_online==0)
	{
		if(vbatt>=POWEROFF_THRESHOLD)
			low_bat_on = 0;
		low_bat_threshold = 1;
	}
	wake_lock_timeout(&lowbat_wakeup,msecs_to_jiffies(3000));	

	mutex_lock(&info->lock);
	if (info->present && (info->online||info->ac_online))
		info->allowed = 1;
	else
		info->allowed = 0;
	mutex_unlock(&info->lock);

	dev_dbg(info->dev, "%s, Allowed:%d\n", __func__, info->allowed);

	set_charging_fsm(info);
	/*
	 * We only need two thresholds for battery voltage. One is discharging,
	 * and the other is power-off.
	 * If CHARGE_THRESHOLD is triggered, we should try to charge or -not-
	 * care. Since fsm will start charging according to conditions. We only
	 * need to set POWEROFF threshold at here.
	 */
	 #if 0
	 if(info->online==0&&info->ac_online==0)
	 {
		if(low_bat_on)
		{
		   set_vbatt_threshold(info, POWEROFF_THRESHOLD, 0);
		}
		else
		{
		  set_vbatt_threshold(info, LOWBAT_THRESHOLD, 0);
		}
	 }
	 #endif
	 	
	 set_vbatt_threshold(info, 0, 0);

	return IRQ_HANDLED;
}

static irqreturn_t pm860x_vchg_handler(int irq, void *data)
{
	struct pm860x_charger_info *info = data;
	int vchg = 0,status=0;
        printk("enter pm860x_vchg_handler\n");
	measure_vchg(info,&vchg);
        printk("pm860x_vchg_handler:vchg=%d\n",vchg);
	
	//disable_irq(info->irq[6]);
	/*disbale irq*/
	//pm860x_reg_write(info->i2c, 0x07,pm860x_reg_read(info->i2c, 0x07)&0xfd);
	mutex_lock(&info->lock);
	//pm860x_reg_write(info->i2c, 0x04,pm860x_reg_read(info->i2c, 0x04)&0xfd);
	/*clear irq flag*/
	if (info->present){
		if(!info->online && !info->ac_online){
			/* check if over-temp on pm8606 or not*/
			status = pm860x_reg_read(info->i2c_8606, PM8606_FLAGS);
			if(status&OVER_TEMP_FLAG){
				pm860x_set_bits(info->i2c_8606, PM8606_FLAGS,
					OVER_TEMP_FLAG, OVER_TEMP_FLAG);/*clear flag*/
				pm860x_set_bits(info->i2c_8606, PM8606_VSYS,
					OVTEMP_AUTORECOVER, OVTEMP_AUTORECOVER);
				dev_dbg(info->dev, "%s,pm8606 over-temp occure\n",__func__);
			}
		}
		if(vchg > VCHG_NORMAL_CHECK){
			set_vchg_threshold(info, VCHG_OVP_LOW, 0);
			info->allowed = 0;
			ovchprotect = 1;
			dev_dbg(info->dev, "%s,pm8607 over-vchg occure,vchg = %dmv\n",
				__func__,vchg);
		} else/* if(vchg < VCHG_OVP_LOW)*/{
			set_vchg_threshold(info, 3000, vchg_normal_high);
			info->allowed = 1;
			ovchprotect = 0;
			dev_dbg(info->dev, "%s,pm8607 over-vchg recover,vchg = %dmv\n",
				__func__,vchg);
		}
	}
	mutex_unlock(&info->lock);

	dev_dbg(info->dev, "%s, Allowed:%d\n", __func__, info->allowed);

	set_charging_fsm(info);
	/*enable irq*/
	//pm860x_reg_write(info->i2c, 0x07,pm860x_reg_read(info->i2c, 0x07)|0x02);
	//enable_irq(info->irq[6]);

	return IRQ_HANDLED;
}


static int  usb_online_init=0;

static int pm860x_usb_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct pm860x_charger_info *info = dev_get_drvdata(psy->dev->parent);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(ac_usb == 1)
		{
		   if(usb_online_init==0)
		   	{
		          val->intval = 0;
		   	}
		   else
		   	{
		val->intval = info->online;
		   	}
		}
		else
		val->intval = 0;
		//printk("pm860x_usb_get_prop:online=%d\n", info->online);
		break;
	default:
		return -ENODEV;
	}
	return 0;
}

static enum power_supply_property pm860x_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int pm860x_ac_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct pm860x_charger_info *info = dev_get_drvdata(psy->dev->parent);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(ac_usb == 1)
		val->intval = info->ac_online;
		else
		val->intval = 0;
		//printk("pm860x_ac_get_prop:online=%d\n", info->ac_online);
		break;
	default:
		return -ENODEV;
	}
	return 0;
}

int get_charger_status(void)
{
  return ac_usb;
}

static enum power_supply_property pm860x_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int measure_vchg(struct pm860x_charger_info *info, int *data)
{
	unsigned char buf[2];
	int ret=0;

	ret = pm860x_bulk_read(info->i2c, PM8607_VCHG_MEAS1, 2, buf);
	if (ret < 0)
		return ret;

	*data = ((buf[0] & 0xff) << 4) | (buf[1] & 0x0f);
	/* V_BATT_MEAS(mV) = value * 5 * 1.8 * 1000 / (2^12) */
	*data = ((*data & 0xfff) * 9 * 125) >> 9;

	dev_dbg(info->dev, "%s,vchg:%dmv\n",__func__,*data);

	return ret;
}

static void set_vchg_threshold(struct pm860x_charger_info *info,
				int min, int max)
{
	int data;

	/* (tmp << 8) * / 5 / 1800 */
	if (min <= 0)
		data = 0;
	else
		data = (min << 5) / 1125;
	pm860x_reg_write(info->i2c, PM8607_VCHG_LOWTH, data);
	dev_dbg(info->dev, "VCHG_LOWTH:%dmv,0x%x\n", min,data);

	if (max <= 0)
		data = 0xff;
	else
		data = (max << 5) / 1125;
	pm860x_reg_write(info->i2c, PM8607_VCHG_HIGHTH, data);
	dev_dbg(info->dev, "VCHG_HIGHTH:%dmv,0x%x\n", max,data);

}

static void set_vbatt_threshold(struct pm860x_charger_info *info,
				int min, int max)
{
	int data;

	/* (tmp << 8) * / 3 / 1800 */
	if (min <= 0)
		data = 0;
	else
		data = (min << 5) / 675;
	pm860x_reg_write(info->i2c, PM8607_VBAT_LOWTH, data);
	dev_dbg(info->dev, "VBAT_LOWTH:%dmv,0x%x\n", min,data);

	if (max <= 0)
		data = 0xff;
	else
		data = (max << 5) / 675;
	pm860x_reg_write(info->i2c, PM8607_VBAT_HIGHTH, data);
	dev_dbg(info->dev, "VBAT_HIGHTH:%dmv,0x%x\n", max,data);

}
static void set_batt_temp_threshold(struct pm860x_charger_info *info,
				int min, int max){
	int data;
       return;
	/* (tmp << 8) * / 1800 */
	if (min <= 0)
		data = 0;
	else
		data = (min << 8) / 1800;
	pm860x_reg_write(info->i2c, PM8607_GPADC1_HIGHTH, data);
	dev_dbg(info->dev, "PM8607_GPADC1_HIGHTH:%dmv,0x%x\n", min,data);

	if (max <= 0)
		data = 0xff;
	else
		data = (max << 8) / 1800;
	pm860x_reg_write(info->i2c, PM8607_GPADC1_LOWTH, data);
	dev_dbg(info->dev, "PM8607_GPADC1_LOWTH:%dmv,0x%x\n", max,data);
}

static int start_precharge(struct pm860x_charger_info *info)
{
	int ret;
	if(version_880==0)
	{
		if(zte_prodtestmod)
			return 0;
	}
       //  printk("enter start_precharge\n");
	dev_dbg(info->dev, "Start Pre-charging!\n");
	set_vbatt_threshold(info, 0, 0);
	/* GPADC1 (battery temperature) */
	set_batt_temp_threshold(info, GPADC1_DEGREE_MIN, GPADC1_DEGREE_MAX);

	ret = pm860x_reg_write(info->i2c_8606, PM8606_PREREGULATORA,
			       PREREG1_1500MA | PREREG1_VSYS_4_5V);
	if (ret < 0)
		goto out;
	/* stop charging */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3,
			      CC1_MODE_OFF);
	if (ret < 0)
		goto out;
	/* set 270 minutes timeout */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL3, (0xf << 4),
			      CC3_270MIN_TIMEOUT);
	if (ret < 0)
		goto out;
	/* set precharge current, termination voltage, IBAT & TBAT monitor */
	ret = pm860x_reg_write(info->i2c, PM8607_CHG_CTRL4,
			       CC4_IPRE_40MA | CC4_VPCHG_3_2V | CC4_IFCHG_MON_EN
			       | CC4_BTEMP_MON_EN);
	if (ret < 0)
		goto out;
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL7,
			      CC7_BAT_REM_EN | CC7_IFSM_EN,
			      /*CC7_BAT_REM_EN |*/ CC7_IFSM_EN);
	if (ret < 0)
		goto out;
	/* trigger precharge */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3,
			      CC1_MODE_PRECHARGE);
	        // printk("end start_precharge\n");

out:
	return ret;
}

static int start_fastcharge(struct pm860x_charger_info *info)
{
	unsigned char buf[6];
	int ret;
	//printk(" enter start_fastcharge\n");
	if(version_880==0)
	{
		if(zte_prodtestmod)
			return 0;
	}
	//dev_dbg(info->dev, "Start Fast-charging!\n");
	set_vbatt_threshold(info, 0, 0);
	set_vchg_threshold(info, /*VCHG_NORMAL_LOW*/3000, vchg_normal_high);
	/* GPADC1 (battery temperature) */
	set_batt_temp_threshold(info, GPADC1_DEGREE_MIN, GPADC1_DEGREE_MAX);
	pm860x_calc_resistor();
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3,
			      CC1_MODE_OFF);
	if (ret < 0)
	{
		//printk("start_fastcharge out\n");
		goto out;
	}
	if(info->online)
	ret = pm860x_reg_write(info->i2c_8606, PM8606_PREREGULATORA,
			       PM8606_PREREG_CURLIM_SET_450V | PREREG1_VSYS_4_5V);
	else if(info->ac_online)
	ret = pm860x_reg_write(info->i2c_8606, PM8606_PREREGULATORA,
			       PM8606_PREREG_CURLIM_SET_540V | PREREG1_VSYS_4_5V);
	else
	ret = pm860x_reg_write(info->i2c_8606, PM8606_PREREGULATORA,
			       PM8606_PREREG_CURLIM_SET_450V | PREREG1_VSYS_4_5V);
				       
	if (ret < 0)
	{
	//printk("start_fastcharge out000\n");
	  goto out;
	}
	//printk("  start_fastcharge0000\n");

	/* set fastcharge termination current & voltage, disable charging */
	ret = pm860x_reg_write(info->i2c, PM8607_CHG_CTRL1,
			       CC1_MODE_OFF | CC1_ITERM_70MA | CC1_VFCHG_4_2V);
	if (ret < 0)
	{
	 // printk("start_fastcharge out111\n");
	  goto out;
	}
		//printk("  start_fastcharge1111\n");
		//printk("  info->charge_type=%d\n",info->charge_type);

	switch (info->charge_type) {
	case USB_CHARGER:
		ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f,
				      CC2_ICHG_500MA);
		break;
	case AC_STANDARD_CHARGER:
		ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f,
				      CC2_ICHG_550MA);
		break;
	case AC_OTHER_CHARGER:
		ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f,
				      CC2_ICHG_550MA);
		break;
	default:
		ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f,
				      CC2_ICHG_500MA);

		break;
	}
	if (ret < 0)
	{
	// printk("start_fastcharge out222\n");
	  goto out;
	}
	//printk("  start_fastcharge2222\n");

	/* set 270 minutes timeout */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL3, (0xf << 4),
			      CC3_270MIN_TIMEOUT);
	if (ret < 0)
	{
		//printk("start_fastcharge out333\n");
		goto out;
	}
		//printk("  start_fastcharge333\n");

	/* set IBAT & TBAT monitor */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL4,
			      CC4_IFCHG_MON_EN | CC4_BTEMP_MON_EN,
			      CC4_IFCHG_MON_EN | CC4_BTEMP_MON_EN);
	if (ret < 0)
	{
		//printk("start_fastcharge out444\n");
		goto out;
	}
		//printk("  start_fastcharge4444\n");

	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL6,
			      CC6_BAT_OV_EN | CC6_BAT_UV_EN | CC6_UV_VBAT_SET,
			      CC6_BAT_OV_EN | CC6_BAT_UV_EN | CC6_UV_VBAT_SET);
	if (ret < 0)
	{
		//printk("start_fastcharge out555\n");
		goto out;
	}
		//printk("  start_fastcharge5555\n");

	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL7,
			      CC7_BAT_REM_EN | CC7_IFSM_EN,
			      /*CC7_BAT_REM_EN |*/ CC7_IFSM_EN);
	if (ret < 0)
	{
		//printk("start_fastcharge out666\n");
		goto out;
	}
		//printk(" end start_fastcharge\n");
/*hw fix workaround: disable BC_SHORT by setting in testpage,
	only occur before sanremo C1*/
	if((info->chip->chip_version <= PM8607_C1D_VERSION) && !info->bc_short){
		info->bc_short = 1;/* disable bc_short mechanism*/
		buf[0] = buf[2] = 0x0;
		buf[1] = 0x60;
		buf[3] = 0xff;
		buf[4] = 0x9f;
		buf[5] = 0xfd;
		/*
		buf[0] = buf[1] = buf[2] = 0x0;
		buf[3] = buf[4] = 0xff;
		buf[5] = 0xfd;
		*///bc_short for power_down
		pm860x_page_bulk_write(info->i2c, 0xC8, 6, buf);
		pm860x_page_reg_write(info->i2c, 0xCF, 0x02);
	}
	/* trigger fastcharge */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3,
			      CC1_MODE_FASTCHARGE);
	low_bat_on = 1;
	//printk(" end start_fastcharge\n");

out:
	return ret;
}

static int stop_charge(struct pm860x_charger_info *info, int vbatt)
{

	printk("stop_charge\n");
	dev_dbg(info->dev, "Stop charging!\n");
        pm860x_set_bits(info->i2c, PM8607_CHG_CTRL5, 1<<2,1<<2);
		
	pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3, CC1_MODE_OFF);
	if (vbatt > CHARGE_THRESHOLD && (info->online||info->ac_online)) 
        {
              //  printk("set CHARGE_THRESHOLD\n");
		set_vbatt_threshold(info, CHARGE_THRESHOLD, 0);
	} else
        {
        	#if 0
		 if(low_bat_on)
                 {
		set_vbatt_threshold(info, POWEROFF_THRESHOLD, 0);
                 }
		 else
                 {
		 set_vbatt_threshold(info, LOWBAT_THRESHOLD, 0);
                 }
		 #endif
	}
	/*hw fix workaround: enable bc_short again after fast charge finished*/
	
	if((info->chip->chip_version <= PM8607_C1D_VERSION) && info->bc_short)
         {
		info->bc_short = 0;/* enable bc_short mechanism*/
		msleep(2);
		pm860x_page_reg_write(info->i2c, 0xCF, 0x0);
	}
	return 0;
}

static int power_off(struct pm860x_charger_info *info)
{
	return 0;
	dev_dbg(info->dev, "Power-off system!\n");
	/* disable VBATT measure */
	pm860x_set_bits(info->i2c, PM8607_MEAS_EN1, MEAS1_VBAT, 0);

	set_vbatt_threshold(info, 0, 0);
	return pm860x_set_bits(info->i2c, PM8607_RESET_OUT, RESET_SW_PD,
			       RESET_SW_PD);
}

static int set_charging_fsm(struct pm860x_charger_info *info)
{
	struct power_supply *psy;
	union power_supply_propval data;
	unsigned char fsm_state[][16] = { "init", "discharge", "precharge",
					  "fastcharge", "poweroff", };
	int ret = -EINVAL;
	int vbatt;
	printk(" enter set_charging_fsm\n");

	psy = power_supply_get_by_name(pm860x_supplied_to[0]);
	if (!psy)
		goto out;
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &data);
	if (ret)
		goto out;
	vbatt = data.intval / 1000;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_PRESENT, &data);
	if (ret) {
		goto out;
	}
	mutex_lock(&info->lock);
	info->present = data.intval;
	//printk("  set_charging_fsm11111\n");

	dev_dbg(info->dev, "Entering FSM:%s, USB:%s, AC:%s, Battery:%s, Allowed:%d\n",
		&fsm_state[info->state][0],
		(info->online) ? "online" : "N/A",
		(info->ac_online) ? "online" : "N/A",
		(info->present) ? "present" : "N/A", info->allowed);
	dev_dbg(info->dev, "V_BATT:%d(mV)\n", vbatt);
	//printk("  set_charging_fsm222\n");

	switch (info->state) {
	case FSM_INIT:
		// printk("set_charging_fsm:FSM_INIT,online:ac_online:present:allowed:vbatt=%d,%d,%d,%d,%d\n",
			//info->online,info->ac_online,info->present ,info->allowed,vbatt);
		if ((info->online||info->ac_online)&& info->present && info->allowed && !ovchprotect) {

			if (vbatt < PRECHARGE_THRESHOLD) {
				info->state = FSM_PRECHARGE;
				start_precharge(info);
			} else{
				//printk("  set_charging_fsm:start_fastcharge\n");
				info->state = FSM_FASTCHARGE;
				start_fastcharge(info);
			}
		} else {
			if (vbatt < POWEROFF_THRESHOLD) {
				info->state = FSM_DISCHARGE;
				//power_off(info);//pay attenation
			} else {
				info->state = FSM_DISCHARGE;
				//printk("  set_charging_fsm:stop_charge11\n");
				stop_charge(info, vbatt);
			}
		}
		break;
	case FSM_PRECHARGE:
             // printk("set_charging_fsm:FSM_PRECHARGE,online:ac_online:present:allowed:vbatt=%d,%d,%d,%d,%d\n",
			//info->online,info->ac_online,info->present ,info->allowed,vbatt);
		if ((info->online||info->ac_online) && info->present && info->allowed && !ovchprotect) {
			if (vbatt > PRECHARGE_THRESHOLD) {
				info->state = FSM_FASTCHARGE;
				start_fastcharge(info);
			}
		} else {
			info->state = FSM_DISCHARGE;
			//printk("  set_charging_fsm:stop_charge22\n");
			stop_charge(info, vbatt);
		}
		break;
	case FSM_FASTCHARGE:
		printk("set_charging_fsm:FSM_FASTCHARGE,online:ac_online:present:allowed:vbatt=%d,%d,%d,%d,%d\n",
			info->online,info->ac_online,info->present ,info->allowed,vbatt);
		if ((info->online||info->ac_online) && info->present && info->allowed && !ovchprotect) {
			if (vbatt < PRECHARGE_THRESHOLD) {
				info->state = FSM_PRECHARGE;
				start_precharge(info);
			} else if ((vbatt > DISCHARGE_THRESHOLD)&&(charge_done==1)) {
				info->state = FSM_DISCHARGE;
				printk("  set_charging_fsm:stop_charge33\n");
				chg_full = 1;
				charge_done=0;
				stop_charge(info, vbatt);
			}else{
			    //stop_charge(info, vbatt);
			    pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3, CC1_MODE_OFF);
				      info->state = FSM_FASTCHARGE;
					printk("enter excep charger \n");
			             start_fastcharge(info);
			}
		
		} else {
			info->state = FSM_DISCHARGE;
			printk("  set_charging_fsm:FSM_DISCHARGE\n");
			stop_charge(info, vbatt);
		}
		break;
	case FSM_DISCHARGE:
		//printk("set_charging_fsm:FSM_DISCHARGE,online:ac_online:present:allowed:vbatt=%d,%d,%d,%d,%d\n",
		//	info->online,info->ac_online,info->present ,info->allowed,vbatt);
		if ((info->online||info->ac_online) && info->present && info->allowed && !ovchprotect) 
		{
			//printk("  set_charging_fsm:FSM_DISCHARGE000\n");

			if (vbatt < PRECHARGE_THRESHOLD) {
				//printk("  set_charging_fsm:FSM_DISCHARGE1111\n");

				info->state = FSM_PRECHARGE;
				start_precharge(info);
			} 
			else /* if (vbatt < DISCHARGE_THRESHOLD) */
			{
					//printk("  set_charging_fsm:FSM_DISCHARGE to start_fastcharge \n");

				info->state = FSM_FASTCHARGE;
				start_fastcharge(info);
			}
			
			
		}
		else 
		{
			if (vbatt < POWEROFF_THRESHOLD) 
			{
				info->state = FSM_DISCHARGE;
				//power_off(info);
			} 
			else if (vbatt > CHARGE_THRESHOLD && (info->online||info->ac_online))
			{
				set_vbatt_threshold(info, CHARGE_THRESHOLD, 0);
			}
			else
			{
			 set_vbatt_threshold(info, 0, 0);
			}
			#if 0
			else
			{
			    if(low_bat_on)
			     {
				set_vbatt_threshold(info, POWEROFF_THRESHOLD, 0);
			     }
			     else
			      {
				set_vbatt_threshold(info, LOWBAT_THRESHOLD, 0);
			      }
			}
			#endif
		}
		break;
	case FSM_POWEROFF:
		//printk("  set_charging_fsm:FSM_POWEROFF\n");

		dev_warn(info->dev, "Error occurs! System should be off!\n");
		//power_off(info);	/* power-off again */
		break;
	default:
		//printk("  set_charging_fsm:default\n");

		dev_warn(info->dev, "FSM meets wrong state:%d\n", info->state);
		break;
	}
	dev_dbg(info->dev, "Out FSM:%s, USB:%s, AC:%s, Battery:%s, Allowed:%d\n",
		&fsm_state[info->state][0],
		(info->online) ? "online" : "N/A",
		(info->ac_online) ? "online" : "N/A",
		(info->present) ? "present" : "N/A", info->allowed);
	mutex_unlock(&info->lock);
	power_supply_changed(&info->usb);
	printk(" end set_charging_fsm\n");

	return 0;
out:
	return ret;
}
#if 1

static u16 pm8607_read_volt_meas_val(u8 measRegister)
{
	u16 meas_val;
	u8 reg_value[2];

	/* Read two registers, the alignment will be done as follows:
	 * Register 1 - bits 7:0 => 8 MSB bits <11:4> of measurement value
	 * Register 2 - bits 3:0 => 4 LSB bits <3:0> of measurement value
	 */
	if (pm860x_bulk_read(ginfo->i2c, measRegister, 2, reg_value)
	    >= 0) {
		meas_val = ((reg_value[0] << 4) | (reg_value[1] & 0x0F));
	} else {
		return 0;
	}
	return meas_val;
}

static void read_vchg(u16 * vchrg)
{
        u32 meas_val;
	meas_val = pm8607_read_volt_meas_val(PM8607_VCHG_MEAS1);
	/* voltage in mili volt */
	//*vchrg = (u16) (((u32) meas_val * 5 * 18 * 1000) >> 12) / 10;
	*vchrg = (u16) (((u32) meas_val * 5 * 18 * 1000) >> 13) / 5;
}
static void set_vchg_th(u16 lower, u16 upper)
{
	u8 val;
	/* the upper vchg is translte to lower value
	 * and stored into the LOW_TH register
	 */
	val = ((upper * 0xFF) / 9000);
	pm860x_reg_write(ginfo->i2c, PM8607_VCHG_UPP_TH, val);
	/* the lower vchg is translte to higher value
	 * and stored into the UPP_TH register
	 */
	val = ((lower * 0xFF) / 9000);
	pm860x_reg_write(ginfo->i2c, PM8607_VCHG_LOW_TH, val);
}

static irqreturn_t pm860x_vcharger_handler(int irq, void *data)
{
	
	struct pm860x_charger_info *info = ginfo;
	u16 vchg_mv;
	/*Tbat pin is different among different type of batteries, so ignor it temporarily */
	//return CHARGE_ALLOW;
	read_vchg(&vchg_mv);
	printk("enter pm860x_vcharger_handler\n");
	/* extra low temperature (tbat < -17C) - phone is off */
	if (vchg_mv > VCHG_TEC_MATAIN_TH ) {
		set_vchg_th(VCHG_TEC_LOW_TH, VCHG_C_UPP_TH);
		/* notify user space - low temperature charge not allow */
		printk("--->get_tbat_state[LOW] low temperature "
			 "charge not allow\n");
		/* normal temperature (tbat > 2C && tbat < 42C)- charge allow */
		ovchprotect = 1;
	} 
	else
	{
		if (vchg_mv >4000 )
		set_vchg_th(VCHG_ALL_LOW_TH, VCHG_ALL_UPP_TH);
		printk("--->get_vchg_state[LOW] low temperature "
	 "CHARGE_ALLOW\n");
		ovchprotect = 0;
	}
	
	set_charging_fsm(info);
	return IRQ_HANDLED;
}
#endif
static int pm860x_init_charger(struct pm860x_charger_info *info)
{
	int ret ;
printk("enter pm860x_init_charger\n");

	mutex_lock(&info->lock);
	info->state = FSM_INIT;
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (ret < 0)
		goto out;
	if (ret & STATUS2_CHG) {
		info->online = 1;
		info->allowed = 1;
		usb_online_flag = 1;
	} else {
		info->online = 0;
		info->ac_online = 0;
		info->allowed = 0;
		usb_online_flag = 0;
	}
	info->charge_type = USB_CHARGER;
	mutex_unlock(&info->lock);
	/* GPADC1 (battery temperature) */
	set_batt_temp_threshold(info, GPADC1_DEGREE_MIN, GPADC1_DEGREE_MAX);
	wake_lock_init(&lowbat_wakeup, WAKE_LOCK_SUSPEND, "unknown_wakeups");

	set_charging_fsm(info);
printk("end pm860x_init_charger\n");

	if(version_880==0)
	{
		if(zte_prodtestmod)
			{
				printk(" int charger enter \n");
				pm860x_set_bits(info->i2c, PM8607_CHG_CTRL1, 3, CC1_MODE_OFF);
				
			}
	}

	return 0;
out:
	return ret;
}

#ifdef	CONFIG_PROC_FS
#define PM860X_POWER_REG_NUM		0xef
#define	PM860X_POWER_PROC_FILE	"driver/pm860x_charger"
static struct proc_dir_entry *pm860x_power_proc_file;
static int index;

static ssize_t pm860x_power_proc_read(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	u8 reg_val;
	int len=0;
	struct pm860x_charger_info *info = ginfo;

	if (index == 0xffff) {
		int i;
		printk(KERN_INFO "pm8607:sanremo reg dump\n");
		for (i = 0; i < PM860X_POWER_REG_NUM; i++) {
			reg_val = pm860x_reg_read(info->i2c, i);
			printk(KERN_INFO "[0x%02x]=0x%02x\n", i, reg_val);
		}
		return 0;
	}
	if((index < 0) || (index > PM860X_POWER_REG_NUM))
		return 0;
	reg_val = pm860x_reg_read(info->i2c, index);
	len = sprintf(page, "0x%x:0x%x\n",index,reg_val);

	return len;
}

static ssize_t pm860x_power_proc_write(struct file *filp,
				       const char *buff, size_t len,
				       loff_t * off)
{
	u8 reg_val;
	char messages[256], vol[256];
	int value = 0;
	u8 temp1,temp2,temp3;
	struct pm860x_charger_info *info = ginfo;

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages + 1, len - 1);
		index = (int) simple_strtoul(vol, NULL, 16);
		printk("index=0x%x\n", index);
	} else if('d' == messages[0]){
		temp1 = pm860x_page_reg_read(info->i2c,0xD4);
		temp2 = pm860x_page_reg_read(info->i2c,0xD5);
		temp3 = pm860x_page_reg_read(info->i2c,0xD7);
		value = temp1 + (temp2<<8) + (temp3<<16);
		printk("d4=0x%x, d5=0x%x,d7=0x%x value:0x%x,value:%d\n", temp1,temp2,temp3,value,value);
	} else {
		/* set the register value */
		reg_val = (int) simple_strtoul(messages, NULL, 16);
		pm860x_reg_write(info->i2c, index, reg_val & 0xFF);
	}

	return len;
}

static void create_pm860x_power_proc_file(void)
{
	pm860x_power_proc_file =
	    create_proc_entry(PM860X_POWER_PROC_FILE, 0644, NULL);
	if (pm860x_power_proc_file) {
		pm860x_power_proc_file->read_proc = pm860x_power_proc_read;
		pm860x_power_proc_file->write_proc = (write_proc_t  *)pm860x_power_proc_write;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_pm860x_power_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(PM860X_POWER_PROC_FILE, &proc_root);
}

#endif

struct timer_list	escape_online_timer;	
void escape_online_function(unsigned long h)
{
  printk("escape_online_function\n");
  usb_online_init=1;
}
static void set_charger_type_func(void)
{
	pm860x_set_charger_type(type_backup);
}

static __devinit int pm860x_charger_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_charger_info *info;
	int ret, i, j, count;

	info = kzalloc(sizeof(struct pm860x_charger_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	count = pdev->num_resources;
	for (i = 0, j = 0; i < count; i++) {
		info->irq[j] = platform_get_irq(pdev, i);
		if (info->irq[j] < 0)
			continue;
		j++;
	}
	ginfo = info;
	g_chinfo = ginfo;
	info->irq_nums = j;

	info->chip = chip;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	info->i2c_8606 = (chip->id == CHIP_PM8607) ? chip->companion
			: chip->client;
	if (!info->i2c_8606) {
		dev_err(&pdev->dev, "Missed I2C address of 88PM8606!\n");
		ret = -EINVAL;
		goto out;
	}
	info->dev = &pdev->dev;

#if !defined(CONFIG_USB_VBUS_88PM860X)
	ret = request_threaded_irq(info->irq[0], NULL,
				   pm860x_charger_handler,
				   IRQF_ONESHOT, "usb supply detect", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq[0], ret);
		goto out;
	}
#endif
	ret = request_threaded_irq(info->irq[1], NULL,
				   pm860x_done_handler,
				   IRQF_ONESHOT, "charge done", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq[1], ret);
		goto out_irq1;
	}
	ret = request_threaded_irq(info->irq[2], NULL,
				   pm860x_exception_handler,
				   IRQF_ONESHOT, "charge timeout", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq[2], ret);
		goto out_irq2;
	}
	
	ret = request_threaded_irq(info->irq[3], NULL,
				   pm860x_exception_handler,
				   IRQF_ONESHOT, "charge fault", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq[3], ret);
		goto out_irq3;
	}
	
	ret = request_threaded_irq(info->irq[4], NULL,
				   pm860x_temp_handler,
				   IRQF_ONESHOT, "temperature", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq[4], ret);
		goto out_irq4;
	}
	ret = request_threaded_irq(info->irq[5], NULL,
				   pm860x_vbattery_handler,
				   IRQF_ONESHOT, "vbatt", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq[5], ret);
		goto out_irq5;
	}
	ret = request_threaded_irq(info->irq[6], NULL,
				   pm860x_vchg_handler,
				   IRQF_ONESHOT, "vchg", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq[6], ret);
		goto out_irq6;
	}
	if (info->irq_nums <= 6) {
		dev_err(chip->dev, "IRQ numbers aren't matched\n");
		goto out_nums;
	}
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);

	info->usb.name = "usb";
	info->usb.type = POWER_SUPPLY_TYPE_USB;
	info->usb.supplied_to = pm860x_supplied_to;
	info->usb.num_supplicants = ARRAY_SIZE(pm860x_supplied_to);
	info->usb.properties = pm860x_usb_props;
	info->usb.num_properties = ARRAY_SIZE(pm860x_usb_props);
	info->usb.get_property = pm860x_usb_get_prop;
	ret = power_supply_register(&pdev->dev, &info->usb);
	if (ret)
		goto out_nums;

	info->ac.name = "ac";
	info->ac.type = POWER_SUPPLY_TYPE_MAINS;
	info->ac.supplied_to = pm860x_supplied_to;
	info->ac.num_supplicants = ARRAY_SIZE(pm860x_supplied_to);
	info->ac.properties = pm860x_ac_props;
	info->ac.num_properties = ARRAY_SIZE(pm860x_ac_props);
	info->ac.get_property = pm860x_ac_get_prop;
	ret = power_supply_register(&pdev->dev, &info->ac);
	if (ret)
		goto out_nums;

	pm860x_init_charger(info);
#ifdef	CONFIG_PROC_FS
	create_pm860x_power_proc_file();
#endif
       setup_timer(&escape_online_timer, escape_online_function, 0);
	mod_timer(&escape_online_timer, jiffies + msecs_to_jiffies(5000));
	wake_lock_init(&usb_ac_wakelock, WAKE_LOCK_SUSPEND, "usb_ac_wakelock");
	wake_lock_init(&charger_remove_wakelock, WAKE_LOCK_SUSPEND, "charger_remove_wakelock");

	
	return 0;

out_nums:
	free_irq(info->irq[6], info);
out_irq6:
	free_irq(info->irq[5], info);
out_irq5:
	free_irq(info->irq[4], info);
out_irq4:
	free_irq(info->irq[3], info);
out_irq3:
	free_irq(info->irq[2], info);
out_irq2:
	free_irq(info->irq[1], info);
out_irq1:
#if !defined(CONFIG_USB_VBUS_88PM860X)
	free_irq(info->irq[0], info);
#endif
out:
	kfree(info);
	return ret;
}

static int __devexit pm860x_charger_remove(struct platform_device *pdev)
{
	struct pm860x_charger_info *info = platform_get_drvdata(pdev);
	int i;

	platform_set_drvdata(pdev, NULL);
	power_supply_unregister(&info->usb);
#if !defined(CONFIG_USB_VBUS_88PM860X)
	free_irq(info->irq[0], info);
#endif
	for (i = 1; i < info->irq_nums; i++)
		free_irq(info->irq[i], info);
	kfree(info);
#ifdef	CONFIG_PROC_FS
	remove_pm860x_power_proc_file();
#endif
	
	wake_lock_destroy(&lowbat_wakeup);
	return 0;
}

#ifdef CONFIG_PM
static int pm860x_charger_suspend(struct device *dev)
{
        struct power_supply *psy;
	union power_supply_propval data;
	int ret = -EINVAL;
	int vbatt;
	int retval;
	printk("pm860x_charger_suspend enter\n");

	struct pm860x_charger_info *info = dev_get_drvdata(dev);
	retval = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (retval & STATUS2_CHG) 
		return 0;

	mutex_lock(&info->lock);
	info->allowed = 0;
	mutex_unlock(&info->lock);

	set_charging_fsm(info);
	psy = power_supply_get_by_name(pm860x_supplied_to[0]);
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &data);
	vbatt =data.intval /1000;

	if(low_bat_on == 1)
	{
	   if((vbatt>=LOWBAT_THRESHOLD)&&(low_bat_threshold == 0))
	   	{
	set_vbatt_threshold(info,LOWBAT_THRESHOLD,0);
	   	}
	   else
	   	{
	   	   set_vbatt_threshold(info,POWEROFF_THRESHOLD,0);
	   	}
	}
	else 
	{
	set_vbatt_threshold(info,POWEROFF_THRESHOLD,0);
	}
	return 0;
}

static int pm860x_charger_resume(struct device *dev)
{
	struct power_supply *psy;
	union power_supply_propval data;
	int ret = -EINVAL;
	#if 0
	psy = power_supply_get_by_name(pm860x_supplied_to[0]);
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &data);
	if(data.intval<=3340)
		low_bat_on = 1;
	else 
		low_bat_on = 0;
	#endif
	//wake_lock_timeout(&lowbat_wakeup,3000);
	return 0;
}

static struct dev_pm_ops pm860x_charger_pm_ops = {
	.suspend	= pm860x_charger_suspend,
	.resume		= pm860x_charger_resume,
};
#endif

static struct platform_driver pm860x_charger_driver = {
	.driver		= {
		.name	= "88pm860x-charger",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm860x_charger_pm_ops,
#endif
	},
	.probe		= pm860x_charger_probe,
	.remove		= __devexit_p(pm860x_charger_remove),
	.shutdown      = pm860x_charger_shutdown,
};

static int __init pm860x_charger_init(void)
{
	return platform_driver_register(&pm860x_charger_driver);
}
module_init(pm860x_charger_init);

static void __exit pm860x_charger_exit(void)
{
	platform_driver_unregister(&pm860x_charger_driver);
}
module_exit(pm860x_charger_exit);

MODULE_DESCRIPTION("Marvell 88PM860x Battery driver");
MODULE_LICENSE("GPL");
