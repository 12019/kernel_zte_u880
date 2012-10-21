/*
 * Battery driver for Marvell 88PM860x
 *
 * Copyright (c) 2009-2010 Marvell International Ltd.
 *	Gang Wu <gang.wu@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
 
#define CONFIG_PM860x_FG
#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/mfd/88pm860x.h>
#include <linux/mfd/88pm860x-power.h>
#include <linux/mfd/88pm860x_soc.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <mach/pxa910_pm.h>



enum charge_state_t {
	NO_CHARGE,
	PRE_CHARGE,
	FAST_CHARGE,
	PULSE_CHARGE  
};

enum tbat_state_t {
	CHARGE_NOT_ALLOW,
	CHARGE_ALLOW,
	PHONE_OFF
};

enum chg_state_t {
	VCHARGE_NOT_ALLOW,
	VCHARGE_ALLOW,
	VPHONE_OFF
};
enum charge_enable_state_t {
	CHARGER_DISABLE,
	BATTERY_NO_CHARGER,
	CHARGER_NO_BATTERY,
	CHARGER_AND_BATTERY,
};

struct pm860x_power_info {
	struct pm860x_chip *chip;
	struct i2c_client *i2c_pm8606;
	struct i2c_client *i2c_pm8607;

	struct power_supply ac;
	struct power_supply usb;
	struct power_supply battery;
	int irq_base;
	unsigned ac_online:1;
	unsigned usb_online:1;
	unsigned bat_online:1;

	struct delayed_work monitor_work;	/* monitor battery */

	enum charge_state_t charge_state;
	enum enum_charger_type charge_type;
	struct timer_list charger_timer;
	unsigned char pm8607_id;
	struct work_struct charger_info_dump_work;
};

DEFINE_MUTEX(charger_input_lock);
static u8 chg_full = 0;   
static u8 chg_failed = 0;  
//static u8 chg_matain = 0;
static u8 low_bat_on = 0;
static u8 ovchprotect = 0;
/*set only in charger_fsm*/
static u8 low_suspend = 0;
struct pm860x_power_info *info;
static struct wake_lock lowbat_wakeup;
static u8 board_id = 0;

#define loop 20
static int volt_index=-1;

static int volt_loop[loop]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int pm860x_get_board(void);

#ifdef CONFIG_PM860x_FG
enum ocv_mode_type {
        ACTIVE_OCV_MODE         = 0,
        SLEEP_OCV_MODE          = 1,
};

u32 q_chg = 0, q_dischg = 0, q_max = 1500, soc_start = 0, r_bat = 300;
u32 ccnt_pos[2] = { 0, 0 }, ccnt_neg[2] = {0, 0}, ccnt_spos = 0, ccnt_sneg = 0;
u64 ccnt_pos_acc = 0, ccnt_neg_acc = 0;
#endif
#if 0       
#define OCV_NUM 100
                                          
u32 soc_ocv[OCV_NUM][2] =
    { {4135, 99}, {4118, 98}, {4106, 97}, {4094, 96}, {4083, 95},
{4072, 94}, {4059, 93}, {4051, 92}, {4041, 91}, {4033, 90},
{4025, 89}, {4016, 88}, {4007, 87}, {3999, 86}, {3989, 85},
{3978, 84}, {3970, 83}, {3963, 82}, {3953, 81}, {3944, 80},
{3937, 79}, {3931, 78}, {3922, 77}, {3917, 76}, {3909, 75},
{3904, 74}, {3898, 73}, {3892, 72}, {3885, 71}, {3878, 70},
{3872, 69}, {3864, 68}, {3860, 67}, {3851, 66}, {3845, 65},
{3841, 64}, {3833, 63}, {3826, 62}, {3819, 61}, {3811, 60},
{3802, 59}, {3797, 58}, {3791, 57}, {3784, 56}, {3777, 55},
{3772, 54}, {3767, 53}, {3763, 52}, {3758, 51}, {3754, 50},
{3751, 49}, {3749, 48}, {3744, 47}, {3742, 46}, {3738, 45},
{3735, 44}, {3733, 43}, {3731, 42}, {3728, 41}, {3725, 40},
{3714, 34}, {3713, 33}, {3711, 32}, {3710, 31}, {3708, 30},
{3705, 29}, {3705, 28}, {3700, 27}, {3698, 26}, {3697, 25},
{3693, 24}, {3691, 23}, {3685, 22}, {3681, 21}, {3677, 20},
{3672, 19}, {3665, 18}, {3656, 17}, {3647, 16}, {3637, 15},
{3625, 14}, {3619, 13}, {3616, 12}, {3615, 11}, {3611, 10},
{3607, 9}, {3601, 8}, {3597, 7}, {3585, 6}, {3556, 5},
{3503, 4}, {3412, 3}, {3311, 2}, {3102, 1}, {2900, 0}
};


u32 soc_ocv[OCV_NUM][2] =                     
    { {4211, 99}, {4156, 98}, {4130, 97}, {4119, 96},
{4109, 95}, {4100, 94}, {4090, 93}, {4081, 92}, {4073, 91},
{4065, 90}, {4057, 89}, {4048, 88}, {4039, 87}, {4031, 86},
{4024, 85}, {4017, 84}, {4009, 83}, {4002, 82}, {3995, 81},
{3988, 80}, {3981, 79}, {3974, 78}, {3968, 77}, {3961, 76},
{3955, 75}, {3949, 74}, {3943, 73}, {3937, 72}, {3931, 71},
{3925, 70}, {3919, 69}, {3913, 68}, {3908, 67}, {3902, 66},
{3896, 65}, {3890, 64}, {3884, 63}, {3877, 62}, {3870, 61},
{3863, 60}, {3856, 59}, {3849, 58}, {3843, 57}, {3836, 56},
{3831, 55}, {3826, 54}, {3822, 53}, {3817, 52}, {3814, 51},
{3810, 50}, {3807, 49}, {3804, 48}, {3800, 47}, {3797, 46},
{3795, 45}, {3793, 44}, {3790, 43}, {3788, 42}, {3786, 41},
{3784, 40}, {3782, 39}, {3780, 38}, {3778, 37}, {3777, 36},
{3775, 35}, {3773, 34}, {3772, 33}, {3769, 32}, {3767, 31},
{3764, 30}, {3759, 29}, {3755, 28}, {3751, 27}, {3747, 26},
{3743, 25}, {3739, 24}, {3735, 23}, {3732, 22}, {3729, 21},
{3725, 20}, {3720, 19}, {3714, 18}, {3709, 17}, {3704, 16},
{3698, 15}, {3691, 14}, {3682, 13}, {3678, 12}, {3675, 11},
{3672, 10}, {3667, 9}, {3660, 8}, {3642, 7}, {3608, 6},{3580, 5},
{3555, 4}, {3530, 3}, {3490, 2},{3450, 1}, {3350, 0}

};
#endif

int chg_from_dischged = 0;
int need_calcu_rbat = 1;
static u8 ac_usb=0;
static u32 soc_ocv_info[100][2];
static int need_get_bat_info;
static int chg_done = 0;



static int pm860x_check_trim(void)
{
	static u8 trim_val, val;

	if (!trim_val) {
		/* ---> enter test page */
		pm860x_reg_write(info->i2c_pm8607, 0xFA, 0x00);
		pm860x_reg_write(info->i2c_pm8607, 0xFB, 0x00);
		pm860x_reg_write(info->i2c_pm8607, 0xFF, 0x00);
		val =
		    pm860x_reg_read(info->i2c_pm8607,
				    PM8607_TEST_PAGE1_D6);
		trim_val = ((val & 0xF0) >> 4);	/* lsb */
		val =
		    pm860x_reg_read(info->i2c_pm8607,
				    PM8607_TEST_PAGE1_D7);
		trim_val |= ((val & 0x03) << 4);	/* msb */
		/* <--- exit test page */
		pm860x_reg_write(info->i2c_pm8607, 0xFE, 0x00);
		pm860x_reg_write(info->i2c_pm8607, 0xFC, 0x00);

		pr_debug("pm860x_probe:triming value for B0[%d]\n",
			 trim_val);
		pr_debug("pm860x_probe:triming {57 is bad value,"
			 "31-36 is good}\n");
	}
	if (trim_val == 57) {
		pr_debug("pm860x_probe:!!!*** BAD TRIMING[%d] ***!!!! \n",
			 trim_val);
	}
	return trim_val;
}

static u16 pm8607_read_volt_meas_val(u8 measRegister)
{
	u16 meas_val;
	u8 reg_value[2];

	/* Read two registers, the alignment will be done as follows:
	 * Register 1 - bits 7:0 => 8 MSB bits <11:4> of measurement value
	 * Register 2 - bits 3:0 => 4 LSB bits <3:0> of measurement value
	 */
	if (pm860x_bulk_read(info->i2c_pm8607, measRegister, 2, reg_value)
	    == 0) {
		meas_val = ((reg_value[0] << 4) | (reg_value[1] & 0x0F));
	} else {
		return 0;
	}
	return meas_val;
}

static u16 pm8607_read_curr_meas_val(u8 measRegister)
{
	u16 meas_val = 0;
	u8 reg_value[2], val;

	/* Read two registers, the alignment will be done as follows: */
	if (pm860x_bulk_read(info->i2c_pm8607, measRegister, 2, reg_value)
	    == 0) {
		val = pm860x_reg_read(info->i2c_pm8607, PM8607_IBAT_MEAS1);
		meas_val = (val << 8);
		val = pm860x_reg_read(info->i2c_pm8607, PM8607_IBAT_MEAS2);
		meas_val |= val;
	} else {
		return 0;
	}

	return meas_val;
}

void read_vbat(u16 * vbat)
{
	u32 meas_val;
	meas_val = pm8607_read_volt_meas_val(PM8607_VBAT_MEAS1);
	/* voltage in mili volt */
        *vbat = (u16) (((u32) meas_val * 3 * 18 * 1000) >> 12) / 10;
}

void read_boardid(u16 * vbat)
{
	u32 meas_val;
	meas_val = pm8607_read_volt_meas_val(PM8607_GPADC2_MEAS1);
	/* voltage in mili volt */
        *vbat = (u16) (((u32) meas_val * 3 * 18 * 1000) >> 12) / 10;
}

void read_vbat_sleep(u16 * vbat)
{
        u32 meas_val=0;
        u8 val[5],reg,i;
        reg=0x14;
        /*VBAT_Sleep:MSB to LSB: 0x18.7:6, 0x17.7:6, 0x16.7:6, 0x15.7:6, 0x14.7:4*/
        for(i=0;i<5;i++)
        {
                val[i] = pm860x_reg_read(info->i2c_pm8607, reg);
                reg++;
        }
        meas_val |= (val[0]&0xF0)>>4;
        for(i=1;i<5;i++)
        {
                meas_val |= ((val[i]&0xC0)>>6)<<(4+(i-1)*2);
        }
        /* voltage in mili volt */
        *vbat = (u16) (((u32) meas_val * 3 * 18 * 1000) >> 12) / 10;
}

static void read_vchg(u16 * vchrg)
{
        u32 meas_val;
	meas_val = pm8607_read_volt_meas_val(PM8607_VCHG_MEAS1);
	/* voltage in mili volt */
	//*vchrg = (u16) (((u32) meas_val * 5 * 18 * 1000) >> 12) / 10;
	*vchrg = (u16) (((u32) meas_val * 5 * 18 * 1000) >> 13) / 5;
}

static void read_vsys(u16 * vsys)
{
	u32 meas_val;
	meas_val = pm8607_read_volt_meas_val(PM8607_VSYS_MEAS1);
	/* voltage in mili volt */
	*vsys = (u16) (((u32) meas_val * 3 * 18 * 1000) >> 12) / 10;
}

static void read_vbat_avg(u16 * vbat_avg)
{
	u32 meas_val;
	u8 val;
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_VBAT_AVG);
	meas_val = val;
	*vbat_avg = (u16) (((u32) meas_val * 3 * 18 * 1000) >> 8) / 10;
}

void read_ibat(u16 * ibat, u8 * sign)
{
	u16 meas_val;
	u32 val;
	meas_val = pm8607_read_curr_meas_val(PM8607_IBAT_MEAS1);
	/* negative number */
	if (meas_val & 0x8000) {
		*sign = 1;
		meas_val = ~meas_val;
		/* turn off the sign bit */
		meas_val &= 0x7FFF;
		meas_val += 1;
		/* positve number */
	} else {
		*sign = 0;
	}
	/* ibat in mA -  * 0.125 */
	val = (meas_val >> 3);
	*ibat = val;
}

static void read_tbat(u16 * tbat)
{
	u32 meas_val;
	meas_val = pm8607_read_volt_meas_val(PM8607_TBAT_MEAS1);
	*tbat = (u16) (((u32) meas_val * 18 * 1000) >> 12) / 10;
}

static void read_tint(u16 * tint)
{
	u32 meas_val, tint_mv;
	meas_val = pm8607_read_volt_meas_val(PM8607_TINT_MEAS1);
	tint_mv = (u16) (((u32) meas_val * 18 * 1000) >> 12) / 10;
	*tint = (tint_mv - 884) * 1000 / 3611;
}

static void set_vbat_th(u16 lower, u16 upper)
{
	u8 val;
	val = ((lower * 0xFF) / 5400);
	pm860x_reg_write(info->i2c_pm8607, PM8607_VBAT_LOW_TH, val);
	val = ((upper * 0xFF) / 5400);
	pm860x_reg_write(info->i2c_pm8607, PM8607_VBAT_UPP_TH, val);
}

static void set_tbat_th(u16 lower, u16 upper)
{
	u8 val;
	/* the upper temprture is translte to lower value
	 * and stored into the LOW_TH register
	 */
	val = ((upper * 0xFF) / 1800);
	pm860x_reg_write(info->i2c_pm8607, PM8607_GPADC1_LOW_TH, val);
	/* the lower temprture is translte to higher value
	 * and stored into the UPP_TH register
	 */
	val = ((lower * 0xFF) / 1800);
	pm860x_reg_write(info->i2c_pm8607, PM8607_GPADC1_UPP_TH, val);
}

static void set_vchg_th(u16 lower, u16 upper)
{
	u8 val;
	/* the upper vchg is translte to lower value
	 * and stored into the LOW_TH register
	 */
	val = ((upper * 0xFF) / 9000);
	pm860x_reg_write(info->i2c_pm8607, PM8607_VCHG_UPP_TH, val);
	/* the lower vchg is translte to higher value
	 * and stored into the UPP_TH register
	 */
	val = ((lower * 0xFF) / 9000);
	pm860x_reg_write(info->i2c_pm8607, PM8607_VCHG_LOW_TH, val);
}

#ifdef CONFIG_PM860x_FG

/*************************************************************************
 * event processing:
 * event: insert new battery.
 * 	1. clear: q_chg, q_dischg, cc
 * 	2. calculate: ocv, soc(update to soc_start)
 * 	notes: supposed the new battery has same characteristic with old one.
 * event: system begin to do CI fast charge.
 * 	1. calculate: the battery internal resistor(updated to r_bat)
 * event: CC interrupt occur.
 * 	1. update the cc
 * event: the charging done.
 * 	1. if it is a complete full charging, updated the q_max*(charge efficiency)
 * 	2. clear cc (assume q_rm == q_max once the charging done), init the soc_start
 *
 * the q_rm calculation:
 * 	q_rm = q_max * soc_start + q_chg*(charge efficiency) -q_dischg
 *
 * notes:
 * 	update cc: update ccnt_pos_acc, ccnt_neg_cc,
 * 	q_chg, q_dischg wich the CC reading
 *
 *
 *************************************************************************/

/* get the battery internal resistor (mOh) */
/* which is called when system is doing CI fast charging */
u32 pm860x_fg_get_rbat(void)
{
	u8 chg_ctrl2_saved, val, ibat_discharge;
	int vbat1 = 0, vbat2 = 0, ibat1 = 0, ibat2 = 0;
	u16 vbat, ibat;
	u32 rbat;
	int i;

	chg_ctrl2_saved =
            pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL2);

        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL2);
        val = (val & 0xE0) | 0x09;      /* Set fast charging current to 500mA */
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL2, val);
        msleep(1000);
        for (i = 0; i < 10; i++) {
		read_vbat(&vbat);
		vbat1 += vbat;
		read_ibat(&ibat, &ibat_discharge);
		if (ibat_discharge) {
			ibat1 = ibat1 - ibat;
		} else {
			ibat1 = ibat1 + ibat;
		}
        }

        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL2);
        val = (val & 0xE0) | 0x01;      /* Set fast charging current to 100mA */
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL2, val);
        msleep(1000);
        for (i = 0; i < 10; i++) {
		read_vbat(&vbat);
		vbat2 += vbat;
		read_ibat(&ibat, &ibat_discharge);
		if (ibat_discharge) {
			ibat2 = ibat2 - ibat;
		} else {
			ibat2 = ibat2 + ibat;
		}
	}

        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL2,
                         chg_ctrl2_saved);
        if(vbat1 > vbat2 && ibat1 > ibat2){
        rbat = 1000 * (vbat1 - vbat2) / (ibat1 - ibat2);
                /*if the gap between two measurement of rbat is two large, discard it */
                if(rbat>2*r_bat || r_bat>2*rbat )
                        rbat=r_bat;
                else
                        r_bat=rbat;
        }
        else
                rbat=r_bat;
        pr_debug("vbat1:%d mV,vbat2:%d mV,ibat1 :%d mA, ibat2 :%d mA\n",
                 vbat1, vbat2, ibat1, ibat2);
        pr_debug("------>rbat = %d mOh\n", rbat);
	return rbat;
}

/* updated the coulomb counter and discharge & charge quant */
void pm860x_fg_get_cc(void)
{
	int i;
	u8 cc_avg_sel;
	u8 val1, val2;
	u32 temp1, temp2 = 0;
	cc_avg_sel = 4 << 3;

	for (i = 0; i < 6; i++) {
		if (i < 2) {	/* CCNT_POS */
			pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ,
					 i | cc_avg_sel);
			val1 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT1);
			val2 =
			    pm860x_reg_read(info->i2c_pm8607,
                                            PM8607_CCNT2);
                        temp1 =
                            (val2 << (i << 4)) + (val1 << ((i << 4) + 8));
                        ccnt_pos_acc = ccnt_pos_acc + temp1;
                }

		if (i >= 2 && i < 4) {	/* CCNT_NEG */
			pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ,
					 i | cc_avg_sel);
			val1 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT1);
			val2 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT2);
			temp1 =
			    (val2 << ((i - 2) << 4)) +
			    (val1 << (((i - 2) << 4) + 8));
                        temp2 = temp2 + temp1;
                        if (i == 3) {
                                temp2 = (~temp2) + 1;
                                ccnt_neg_acc += temp2;
                        }
                }

		if (i == 4) {	/* CCNT_SPOS */
			pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ,
					 i | cc_avg_sel);
			val1 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT1);
			val2 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT2);
			temp1 = val2 + (val1 << 8);
			ccnt_spos = ccnt_spos + temp1;
		}

		if (i == 5) {	/* CCNT_SNEG */
			pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ,
					 i | cc_avg_sel);
			val1 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT1);
			val2 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT2);
			temp1 = val2 + (val1 << 8);
                        ccnt_sneg = ccnt_sneg + temp1;
                }
        }
        /* According to sanremo spec, Charge[mAh]=reading[lsb]*1.6954*1e(-8),to
        improve accuracy of Charge, we involved a magic value 0x48D11F as a factor,
        the formula is like this :Charge[mAh]=reading[lsb]*1.6954*1e(-8)*2^48/2^48=
        reading[lsb]*0x48D11F>>48 */
        q_chg = (ccnt_pos_acc * 0x48D11F) >> 48;
        q_dischg = (ccnt_neg_acc * 0x48D11F) >> 48;

}

/* reset the coulomb counter and discharge & charge quant */
void pm860x_fg_clear_cc(void)
{
	int i;
	u8 val1, val2;
	u8 cc_avg_sel;

	cc_avg_sel = 8 << 3;
	for (i = 0; i < 6; i++) {
		if (i < 2) {	/* CCNT_POS */
			pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ,
					 i | cc_avg_sel);
			val1 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT1);
			val2 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT2);
		}

		if (i >= 2 && i < 4) {	/* CCNT_NEG */
			pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ,
					 i | cc_avg_sel);
			val1 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT1);
			val2 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT2);
		}

		if (i == 4) {	/* CCNT_SPOS */
			pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ,
					 i | cc_avg_sel);
			val1 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT1);
			val2 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT2);
		}

		if (i == 5) {	/* CCNT_SNEG */
			pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ,
					 i | cc_avg_sel);
			val1 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT1);
			val2 =
			    pm860x_reg_read(info->i2c_pm8607,
					    PM8607_CCNT2);
		}

	}

	ccnt_neg_acc = 0;
	ccnt_sneg = 0;
	q_chg = 0;

	ccnt_pos_acc = 0;
	ccnt_spos = 0;
	q_dischg = 0;

}

/* get the ocv of current battery */
u32 pm860x_fg_get_ocv(void)
{
	u8 ibat_discharge;
	u16 ibat, vbat;
	s32 vbat_mean, vbat_sum = 0;
	s32 ibat_mean, ibat_sum = 0;
	u32 ocv;
	int i,volt_sum=0;
	volt_index = (volt_index+1)%loop;
	for (i = 0; i < 10; i++) {
		read_vbat(&vbat);
		vbat_sum += vbat;
		read_ibat(&ibat, &ibat_discharge);
		if (ibat_discharge) {
			ibat_sum = ibat_sum - ibat;
		} else {
			ibat_sum = ibat_sum + ibat;
		}
	}
	vbat_mean = vbat_sum / 10;
	ibat_mean = ibat_sum / 10;

	volt_loop[volt_index] =vbat_mean;
	
	for (i=0; i<loop; i++)
       {
		if(volt_loop[i] <= 0)
			break;
		volt_sum += volt_loop[i];
	}
	if(i!=0)
	ocv = volt_sum/i;
	else
	ocv = vbat_mean;

//	ocv = vbat_mean - ibat_mean * (s32)r_bat / 1000; 
    //ocv = vbat_mean;
	/*pr_debug("vbat_mean:%d mV, vbat_sum:%d mV,ibat_mean=%d mA ibat_sum :%d mA\n", vbat_mean,vbat_sum,ibat_mean,ibat_sum);
	   pr_debug("ocv:%d mV, rbat=%d mOh\n", ocv,r_bat); */
	return ocv;
}

/* get the soc of current battery */
u32 pm860x_fg_get_soc(enum ocv_mode_type type)
{
        u32 ocv=0, soc = 0;
        int i;
        if(type == SLEEP_OCV_MODE)
        {
                read_vbat_sleep((u16*)&ocv);
        } else {
        ocv = pm860x_fg_get_ocv();
        }
	if (ovchprotect == 1||chg_full==1)
		return 100;
        for (i = 0; i < OCV_NUM-1; i++) {
                if (ocv >= soc_ocv[i][0]) {
                                soc = soc_ocv[i][1];
                                break;
                } else if ((ocv < soc_ocv[i][0]) && ((ocv >= soc_ocv[i + 1][0]))) {
                                soc = soc_ocv[i + 1][1];
                                break;
                } else if (ocv < soc_ocv[OCV_NUM-1][0]) {
                        soc = 0;
                        break;
                }
        }
        printk(KERN_INFO"ocv:%d, soc:%d\n",ocv,soc);
        return soc;
}

/* get the fule gauge of current battery */
/* supposed require specific ibat */
u32 pm860x_fg_get_qrm(void)
{
	u32 soc_end = 0, q_rm = 0;
        pm860x_fg_get_cc();
        if (((q_max * (soc_start - soc_end) / 100) + q_chg) > q_dischg) {
                q_rm =
                    (q_max * (soc_start - soc_end) / 100) + q_chg - q_dischg;
        } else {
                q_rm = 0;
        }

        /*pr_debug("soc_start:%d soc_end:%d\n", soc_start, soc_end);
        pr_debug("q_rm:%d q_max:%d,percent:%d \%\n", q_rm, q_max,(q_rm * 100 / q_max));*/
        return q_rm;
}

int pm860x_fg_get_qrm_percent(void)
{
	u32 val, ret;
	val = pm860x_fg_get_qrm();
	ret = val * 100 / q_max;
	return ret;
}

u32 pm860x_fg_gather_bat_info(void)
{
	int i, k;
	static int last = 0;
	static u32 q_t1 = 0, q_t2 = 0;
	u32 ocv_t = 0;
	static u32 ocv_t1 = 0, ocv_t2 = 0;
	for (i = 0; i < 100; i++) {
		if (soc_ocv_info[i][1] == 0) {
			pm860x_fg_get_cc();
			ocv_t = pm860x_fg_get_ocv();
			if (q_dischg < q_max * (i + 1) / 100) {
				q_t1 = q_dischg;
				ocv_t1 = ocv_t;
			} else if (q_dischg == q_max * (i + 1) / 100) {
				soc_ocv_info[i][0] = ocv_t;
				soc_ocv_info[i][1] = 100 - (i + 1);
				pr_debug
				    ("record data1:[ocv:%d index:%d]-----\n",
				     soc_ocv_info[i][0],
				     soc_ocv_info[i][1]);
			} else {
				q_t2 = q_dischg;
				ocv_t2 = ocv_t;
				if (ocv_t1 >= ocv_t2 && q_t2 > q_t1) {
					k = (ocv_t1 - ocv_t2) / (q_t2 -
								 q_t1);
					ocv_t =
					    ocv_t1 -
					    (q_max * (i + 1) / 100 -
					     q_t1) * k;
				} else {
					if (ocv_t1 == 0)
						ocv_t = ocv_t2;
					else
						ocv_t =
						    (ocv_t1 + ocv_t2) / 2;
				}
				soc_ocv_info[i][0] = ocv_t;
				soc_ocv_info[i][1] = 100 - (i + 1);
				pr_debug
				    ("ocv_t1:%d, ocv_t2:%d q_t1:%d q_t2:%d\n",
				     ocv_t1, ocv_t2, q_t1, q_t2);
				pr_debug
				    ("record data2:[ocv:%d index:%d]-----\n",
				     soc_ocv_info[i][0],
				     soc_ocv_info[i][1]);
			}
			break;

		} else {
			if (i == 100)
				break;
		}
	}
	if ((i) % 5 == 0) {
		if (last != i) {
			last = i;
			for (i = 0; i < 100; i += 5) {
				pr_debug
				    ("{%d,%d},{%d,%d},{%d,%d},{%d,%d},{%d,%d}, \n",
				     soc_ocv_info[i][0],
				     soc_ocv_info[i][1],
				     soc_ocv_info[i + 1][0],
				     soc_ocv_info[i + 1][1],
				     soc_ocv_info[i + 2][0],
				     soc_ocv_info[i + 2][1],
				     soc_ocv_info[i + 3][0],
				     soc_ocv_info[i + 3][1],
				     soc_ocv_info[i + 4][0],
				     soc_ocv_info[i + 4][1]);
			}
		}

	}
	return 0;

}

/* later, the init value should be updated from nvm */
void pm860x_fg_poweron_init(void)
{
        q_max = q_max;
        r_bat = r_bat;
        soc_start = pm860x_fg_get_soc(ACTIVE_OCV_MODE);
        pr_debug("--->pm860x_fg_poweron_init:soc_start:%d\n", soc_start);
}


#endif

void charger_timer_start(u16 delay)
{
	if (mod_timer(&info->charger_timer, jiffies + delay)) {
		pr_debug("charger_timer_start:timer is active already \n");
	}
}

void charger_timer_stop(void)
{
	del_timer(&info->charger_timer);
	pr_debug("charger_timer_stop\n");
}

static void stop_charging(void)
{
	u8 val;
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL5);
        val = val | (1<<2);
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL5, val);

	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = val & CHARGER_MODE_MASK;

	if ((val == CHARGER_MODE_PRE_CHARGE) ||
	    (val == CHARGER_MODE_FAST_CHARGE) || (val == CHARGER_MODE_PULSE_CHARGE)) {  
		pr_debug("--->pm860x_stop_charging \n");
		/* this is done auto by Levante -
		 *  we need it in case we want to stop manualy
		 */
		val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_CHARGE_OFF;
		pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	} else {
		pr_debug("--->pm860x_stop_charging(doing nothing "
			 "already stopped)\n");
	}

	return;
}

static void start_pre_charging(void)
{
	u8 val;
	u16 vbat;

	pr_debug("--->pm860x_start_pre-charging \n");

	/* interrupt are working fine */
	/* set pre-regulator to 1500mA & Vsys to 4.5v */
	if(info->charge_type == AC_OTHER_CHARGER||info->charge_type == AC_STANDARD_CHARGER)
	{
		 pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
			 PM8606_PREREG_VSYS_SET |
		 PM8606_PREREG_CURLIM_SET_810V);
	}
	else
	{
		 pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
		 PM8606_PREREG_VSYS_SET |
			 PM8606_PREREG_CURLIM_SET_450V);
	}

	/* make sure we stop charge */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_CHARGE_OFF;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	/* 0xFx Charger Timeout = disable */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL3);
	val |= CHG_TIMER_SET_DISABLE;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL3, val);

	/* Ipre = 40ma , Vpre term 3.2v,IBAT&BTEMP monitor enable */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL4);
	val = (val & ~IPRE_SET_MASK) | IPRE_SET_75MA;
	val = (val & ~VPCHG_SET_MASK) | VPCHG_SET_3P2V;
	val = val | IFCHG_MON_EN | BTEMP_MON_EN;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL4, val);

	/*
	 * Charger Initialization
	 * Only valid for SR B0- Write to SanRemo register 0x4D, CHG_CTRL6
	 * the value 0x01 The field BD_MSK='01' (BAT TEMP is active,
	 * BAT_ID is masked)Only valid for SR B0- Write to SanRemo
	 * register 0x4E, CHG_CTRL7 the value 0x88 The field ILIM_LONGTMREN='0'
	 * (ILIM_TMR_SET multiplied by factor of 128) The field IFSM_EN='1'
	 * (enables charge current thermal control ) Total ILIM timer set is
	 * 512mSec (ILIM_TMR_SET='00' and ILIM_LONGTMREN='1')BAT_REM_EN=1;
	 */
	if (info->pm8607_id >= PM8607_B0_ID) {
		val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL7);
//		val |= BAT_REM_EN | IFSM_EN;
		pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL7, val);
	}

	/*for Levante B0 and previous -
	 * work in continues mode
	 * to avoid false VBAT measurement (~2705mV)
	 */
	/* should be fixed in C0 */
	if (info->pm8607_id >= PM8607_B0_ID) {
		val =
		    pm860x_reg_read(info->i2c_pm8607, PM8607_MEAS_ENABLE3);
		val |=
		    (PM8607_MEAS_EN3_COULOMB_COUNTER |
		     PM8607_MEAS_EN3_IBAT);
		pm860x_reg_write(info->i2c_pm8607, PM8607_MEAS_ENABLE3,
				 val);
        }
#ifdef CONFIG_PM860x_FG
        read_vbat(&vbat);
        if (vbat < 3100) {
                chg_from_dischged = 1;
                pr_debug("---->chg_from_dischged: %d\n",
                         chg_from_dischged);
	}
#endif
	/* start pre-charge */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_PRE_CHARGE;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	pr_debug("start_pre_charging:[0x48]: 0x%x\n", val);
	//chg_matain = 0;
	//charger_timer_start(200);

}

static void start_pulse_charging(void)  
{
        u8 val;
        u16 vbat;

        pr_debug("--->pm860x_start_pulse-charging \n");
        
        /* interrupt are working fine */
        /* set pre-regulator to 1500mA & Vsys to 4.5v */
		if(info->charge_type == USB_CHARGER)
		{
			 pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
				 PM8606_PREREG_VSYS_SET |
				 PM8606_PREREG_CURLIM_SET_450V);
		}
		else
		{
			 pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
			 PM8606_PREREG_VSYS_SET |
			 PM8606_PREREG_CURLIM_SET_810V);
		}

        /*columb counter */   
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CC_READ);
        val = (val & ~CC_AVG_SEL_MASK) | CC_AVG_SEL_4;
        pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ, val);

        
        val = pm860x_reg_read(info->i2c_pm8606, PM8606_PREREGULATORB);
        val = val | 0x1;
        pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORB,val);

        /*IFSM | enable ILMIT | 4.2V | Iterm = 60ma| fast */
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
       // val = (val & ~VFCHG_SET_MASK) | VFCHG_SET_4P2V|ITERM_SET_60MA;
        val = (val & ~ITERM_SET_MASK) | VFCHG_SET_4P4V|ITERM_SET_20MA; 
        val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_CHARGE_OFF;
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL2);
         val = (val & ~BB_PREG_OFF) | TOFFMAX_SET| BB_LRSW_EN; 
         pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL2, val);

        /* IBAT is monitored | BTEMP_MON_EN enable */
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL4);
        val |= IFCHG_MON_EN | BTEMP_MON_EN;
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL4, val);
        /* enable BAT_DET by GPADC1|BC_OV_VBAT_EN=1|BC_UV_VBAT_EN=1 */
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL6);
        val = (val & ~BD_MSK_MASK) | BD_MSK_GPDAC1;
        val = val | BC_OV_VBAT_EN | BC_UV_VBAT_EN;
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL6, val);

//pulse bank radio
        /* 0xFx Charger Timeout = disable */
       // val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL3);
        //val |= CHG_TIMER_SET_DISABLE;                  
       // val |=CHG_TIMER_SET_45MIN;
       // pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL3, val);

        /*
         * Charger Initialization
         * Only valid for SR B0- Write to SanRemo register 0x4D, CHG_CTRL6
         * the value 0x01 The field BD_MSK='01' (BAT TEMP is active,
         * BAT_ID is masked)Only valid for SR B0- Write to SanRemo
         * register 0x4E, CHG_CTRL7 the value 0x88 The field ILIM_LONGTMREN='0'
         * (ILIM_TMR_SET multiplied by factor of 128) The field IFSM_EN='1'
         * (enables charge current thermal control ) Total ILIM timer set is
         * 512mSec (ILIM_TMR_SET='00' and ILIM_LONGTMREN='1')BAT_REM_EN=1;
         */

        if (info->pm8607_id >= PM8607_B0_ID) {
                val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL7);
                //val |= BAT_REM_EN | IFSM_EN;
                pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL7, val);
        }

        /*for Levante B0 and previous -
         * work in continues mode
         * to avoid false VBAT measurement (~2705mV)
         */
        /* should be fixed in C0 */
        if (info->pm8607_id >= PM8607_B0_ID) {
                val =
                    pm860x_reg_read(info->i2c_pm8607, PM8607_MEAS_ENABLE3);
                val |=
                    (PM8607_MEAS_EN3_COULOMB_COUNTER |
                     PM8607_MEAS_EN3_IBAT);
                pm860x_reg_write(info->i2c_pm8607, PM8607_MEAS_ENABLE3,
                                 val);
        }
#ifdef CONFIG_PM860x_FG
        read_vbat(&vbat);
        if (vbat > 4300) {    
                chg_from_dischged = 1;
                pr_debug("---->chg_from_dischged: %d\n",
                         chg_from_dischged);
        }
#endif
        /* start pre-charge */
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
        val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_PULSE_CHARGE;
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
        pr_debug("start_pulse_charging:[0x48]: 0x%x\n", val);

        //charger_timer_start(200);
}

static void start_charging(void)
{
	u8 val;
	u16 vbat;

	pr_debug("--->pm860x_start_charging \n");
	/* interrupt are working fine */
	/*set pre-regulator to 1500mA & Vsys to 4.5v */
	if(info->charge_type == AC_OTHER_CHARGER||info->charge_type == AC_STANDARD_CHARGER)
	{
		 pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
			 PM8606_PREREG_VSYS_SET |
		 PM8606_PREREG_CURLIM_SET_810V);
	}
	else
	{
		 pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
		 PM8606_PREREG_VSYS_SET |
			 PM8606_PREREG_CURLIM_SET_450V);
	}
	/*columb counter */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CC_READ);
	val = (val & ~CC_AVG_SEL_MASK) | CC_AVG_SEL_4;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ, val);

	/*IFSM | enable ILMIT | 4.2V | Iterm = 20ma| fast */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = (val & ~VFCHG_SET_MASK) | VFCHG_SET_4P2V;
	val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_CHARGE_OFF;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	/*PREREG_OFF=0|LRSW_EN = 0|ICHG_SET= 500ma or 1000ma or 700ma */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL2);
        if(info->charge_type == USB_CHARGER)
		val = (val & ~ICHG_SET_MASK) | ICHG_SET_300MA;
		// val = (val & ~ICHG_SET_MASK) | ICHG_SET_500MA;
        else if(info->charge_type == AC_STANDARD_CHARGER)
		val = (val & ~ICHG_SET_MASK) | ICHG_SET_700MA;  
        else if(info->charge_type == AC_OTHER_CHARGER)
		val = (val & ~ICHG_SET_MASK) | ICHG_SET_700MA;  //ICHG_SET_750MA
		//val = (val & ~ICHG_SET_MASK) | ICHG_SET_1000MA;
		 val = (val & ~BB_PREG_OFF) | TOFFMAX_SET| BB_LRSW_EN;  
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL2, val);

        /* Charger Timeout disabled */
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL3);
	val = (val & ~CHG_TIMER_SET_MASK) | CHG_TIMER_SET_MASK;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL3, val);

	/* IBAT is monitored | BTEMP_MON_EN enable */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL4);
	val |= IFCHG_MON_EN | BTEMP_MON_EN;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL4, val);

	/* enable BAT_DET by GPADC1|BC_OV_VBAT_EN=1|BC_UV_VBAT_EN=1 
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL6);
	val = (val & ~BD_MSK_MASK) | BD_MSK_GPDAC1;
	val = val | BC_OV_VBAT_EN | BC_UV_VBAT_EN;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL6, val);*/

	/*
	 * Charger Initialization
	 * Only valid for SR B0- Write to SanRemo register 0x4D, CHG_CTRL6
	 * the value 0x01 The field BD_MSK='01' (BAT TEMP is active,
	 * BAT_ID is masked)Only valid for SR B0- Write to SanRemo
	 * register 0x4E, CHG_CTRL7 the value 0x88 The field ILIM_LONGTMREN='0'
	 * (ILIM_TMR_SET multiplied by factor of 128) The field IFSM_EN='1'
	 * (enables charge current thermal control ) Total ILIM timer set is
	 * 512mSec (ILIM_TMR_SET='00' and ILIM_LONGTMREN='1')BAT_REM_EN=1;
	 */
	if (info->pm8607_id >= PM8607_B0_ID) {
		val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL7);
		//val |= BAT_REM_EN | ILIM_LONGTMREN | IFSM_EN;
		val |=  ILIM_LONGTMREN | IFSM_EN;
		pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL7, val);
	}
	/*0x9a IFSM | enable ILMIT | 4.2V | Iterm = 20ma| fast */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = (val & ~CHARGER_MODE_MASK);
	//val = (val & ~ITERM_SET_MASK) | ITERM_SET_20MA;
	val = (val & ~ITERM_SET_MASK) | ITERM_SET_60MA; 
	//val = (val & ~VFCHG_SET_MASK) | VFCHG_SET_4P4V;   //VFCHG_SET_4P2V   
	val = (val & ~VFCHG_SET_MASK) | VFCHG_SET_4P2V;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_FAST_CHARGE;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	pr_debug("start_charging:[0x48]=0x%02x\n", val);

#ifdef CONFIG_PM860x_FG
        read_vbat(&vbat);
        if (vbat >= 3800 && vbat <= 4100) {  //4150) {  
                if (need_calcu_rbat) {
                        pr_debug
                            ("start_charging: rbat----------------------->\n");
			r_bat = pm860x_fg_get_rbat();
		}
	}
#endif
	//chg_matain = 0;
	charger_timer_start(200);
	return;
}

static void start_matain_charging(void)
{
	u8 val;
	u16 vbat;

	pr_debug("--->pm860x_start_charging \n");
	/* interrupt are working fine */
	/*set pre-regulator to 1500mA & Vsys to 4.5v */
	if(info->charge_type == USB_CHARGER)
	{
		 pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
			 PM8606_PREREG_VSYS_SET |
			 PM8606_PREREG_CURLIM_SET_450V);
	}
	else
	{
		 pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
		 PM8606_PREREG_VSYS_SET |
		 PM8606_PREREG_CURLIM_SET_810V);
	}
	/*columb counter */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CC_READ);
	val = (val & ~CC_AVG_SEL_MASK) | CC_AVG_SEL_4;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ, val);

	/*IFSM | enable ILMIT | 4.2V | Iterm = 20ma| fast */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = (val & ~VFCHG_SET_MASK) | VFCHG_SET_4P2V;
	val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_CHARGE_OFF;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	/*PREREG_OFF=0|LRSW_EN = 0|ICHG_SET= 500ma or 1000ma or 700ma */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL2);
        if(info->charge_type == USB_CHARGER)
		val = (val & ~ICHG_SET_MASK) | ICHG_SET_300MA;
		// val = (val & ~ICHG_SET_MASK) | ICHG_SET_500MA;
        else if(info->charge_type == AC_STANDARD_CHARGER)
		val = (val & ~ICHG_SET_MASK) | ICHG_SET_700MA;  
        else if(info->charge_type == AC_OTHER_CHARGER)
		val = (val & ~ICHG_SET_MASK) | ICHG_SET_700MA;  //ICHG_SET_750MA
		//val = (val & ~ICHG_SET_MASK) | ICHG_SET_1000MA;
		 val = (val & ~BB_PREG_OFF) | TOFFMAX_SET| BB_LRSW_EN;  
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL2, val);

        /* Charger Timeout disabled */
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL3);
	val = (val & ~CHG_TIMER_SET_MASK) | CHG_TIMER_SET_MASK;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL3, val);

	/* IBAT is monitored | BTEMP_MON_EN enable */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL4);
	val |= IFCHG_MON_EN | BTEMP_MON_EN;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL4, val);

	/* enable BAT_DET by GPADC1|BC_OV_VBAT_EN=1|BC_UV_VBAT_EN=1 */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL6);
	val = (val & ~BD_MSK_MASK) | BD_MSK_GPDAC1;
	val = val | BC_OV_VBAT_EN | BC_UV_VBAT_EN;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL6, val);

	/*
	 * Charger Initialization
	 * Only valid for SR B0- Write to SanRemo register 0x4D, CHG_CTRL6
	 * the value 0x01 The field BD_MSK='01' (BAT TEMP is active,
	 * BAT_ID is masked)Only valid for SR B0- Write to SanRemo
	 * register 0x4E, CHG_CTRL7 the value 0x88 The field ILIM_LONGTMREN='0'
	 * (ILIM_TMR_SET multiplied by factor of 128) The field IFSM_EN='1'
	 * (enables charge current thermal control ) Total ILIM timer set is
	 * 512mSec (ILIM_TMR_SET='00' and ILIM_LONGTMREN='1')BAT_REM_EN=1;
	 */
	if (info->pm8607_id >= PM8607_B0_ID) {
		val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL7);
		//val |= BAT_REM_EN | ILIM_LONGTMREN | IFSM_EN;
		val |=  ILIM_LONGTMREN | IFSM_EN;
		pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL7, val);
	}
	/*0x9a IFSM | enable ILMIT | 4.2V | Iterm = 20ma| fast */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = (val & ~CHARGER_MODE_MASK);
	//val = (val & ~ITERM_SET_MASK) | ITERM_SET_20MA;
	val = (val & ~ITERM_SET_MASK) | ITERM_SET_60MA; 
	//val = (val & ~VFCHG_SET_MASK) | VFCHG_SET_4P4V;   //VFCHG_SET_4P2V   
	val = (val & ~VFCHG_SET_MASK) | VFCHG_SET_4P3V;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_FAST_CHARGE;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);

	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
	pr_debug("start_charging:[0x48]=0x%02x\n", val);

#ifdef CONFIG_PM860x_FG
        read_vbat(&vbat);
        if (vbat >= 3800 && vbat <= 4100) {  //4150) {  
                if (need_calcu_rbat) {
                        pr_debug
                            ("start_charging: rbat----------------------->\n");
			r_bat = pm860x_fg_get_rbat();
		}
	}
#endif
	//chg_matain = 1;
	charger_timer_start(200);
	return;
}

static enum charge_enable_state_t get_charge_state(void)
{
	u8 val;

	val = pm860x_reg_read(info->i2c_pm8607, PM8607_STATUS_2);

	if (pm860x_check_trim() == 57) {
		/*panic("Chipset fault for charger only-repalce \
		   //pm860x!!!"); */
	}

	if ((val & PM8607_BAT_STATUS) && (val & PM8607_CHG_STATUS)) {
		pr_debug("--->get_charge_state-detect battery "
			 "and charger \n");
		return CHARGER_AND_BATTERY;
	} else if (val & PM8607_BAT_STATUS) {
		pr_debug("--->get_charge_state-detect battery but "
			 "no charger\n");
		return CHARGER_DISABLE;
        } else if (val & PM8607_CHG_STATUS) {
                pr_debug("--->get_charge_state-detect charger but "
                         "no battery\n");
                return CHARGER_AND_BATTERY;
                //return CHARGER_NO_BATTERY;
        } else {
                pr_debug("--->get_charge_state - no battery no charger\n");
                return CHARGER_DISABLE;
	}

	pr_debug("--->get_charge_state- charger disable \n");
	return CHARGER_DISABLE;
}

static enum chg_state_t get_vchg_state(void)
{
	u16 vchg_mv;
	enum chg_state_t chgstate = VCHARGE_ALLOW;

	/*Tbat pin is different among different type of batteries, so ignor it temporarily */
	//return CHARGE_ALLOW;
	read_vchg(&vchg_mv);

	/* extra low temperature (tbat < -17C) - phone is off */
	if (vchg_mv > VCHG_TEC_MATAIN_TH ) {
		set_vchg_th(VCHG_TEC_LOW_TH, VCHG_C_UPP_TH);
		chgstate = VCHARGE_NOT_ALLOW;
		/* notify user space - low temperature charge not allow */
		pr_debug("--->get_tbat_state[LOW] low temperature "
			 "charge not allow\n");
		/* normal temperature (tbat > 2C && tbat < 42C)- charge allow */
		chg_full = 1;
		ovchprotect = 1;
	} 
	else
	{
		chgstate = VCHARGE_ALLOW;
		if (vchg_mv >4000 )
		set_vchg_th(VCHG_ALL_LOW_TH, VCHG_ALL_UPP_TH);
		pr_debug("--->get_vchg_state[LOW] low temperature "
	 "CHARGE_ALLOW\n");
		chg_full = 0;
		ovchprotect = 0;
	}
	return chgstate;
}
static enum tbat_state_t get_tbat_state(void)
{
	u16 tbat_mv;
	enum tbat_state_t tstate = CHARGE_NOT_ALLOW;

	/*Tbat pin is different among different type of batteries, so ignor it temporarily */
	return CHARGE_ALLOW;
	read_tbat(&tbat_mv);

	/** The higher the mV the lower the temperature and vise versa **/

	/* extra low temperature (tbat < -17C) - phone is off */
	if (tbat_mv > TBAT_TEC_DPONIT_TH) {
		set_tbat_th(TBAT_TEC_MIN, TBAT_TEC_UPP_TH);
		tstate = PHONE_OFF;
		pr_debug("--->get_tbat_state[MIN] very low temperature "
			 "phone should be off\n");
		/* low temperature (tbat > -17C && tbat < 2C) - charge not allow */
	} else if (tbat_mv < TBAT_TEC_DPONIT_TH
		   && tbat_mv > TBAT_C_DPONIT_TH) {
		set_tbat_th(TBAT_TEC_LOW_TH, TBAT_C_UPP_TH);
		tstate = CHARGE_NOT_ALLOW;
		/* notify user space - low temperature charge not allow */
		pr_debug("--->get_tbat_state[LOW] low temperature "
			 "charge not allow\n");
		/* normal temperature (tbat > 2C && tbat < 42C)- charge allow */
	} else if (tbat_mv < TBAT_C_DPONIT_TH &&
		   tbat_mv > TBAT_H_DPONIT_TH) {
		set_tbat_th(TBAT_C_LOW_TH, TBAT_H_UPP_TH);
		tstate = CHARGE_ALLOW;
		pr_debug("--->get_tbat_state[NORMAL] charge allow\n");
		/* high temperature (tbat > 42C && tbat < 57C)- charge not allow */
	} else if (tbat_mv < TBAT_H_DPONIT_TH
		   && tbat_mv > TBAT_EH_DPONIT_TH) {
		set_tbat_th(TBAT_H_LOW_TH, TBAT_EH_UPP_TH);
		tstate = CHARGE_NOT_ALLOW;
		/* notify user space - high temperature charge not allow */
		pr_debug("--->get_tbat_state[HIGH] high temperature "
			 "charge not allow\n");
		/* extra high temperature (tbat > 57C) - phone off */
	} else if (tbat_mv < TBAT_EH_DPONIT_TH) {
		set_tbat_th(TBAT_EH_LOW_TH, TBAT_EH_MAX);
		tstate = PHONE_OFF;
		/* notify user space - Extra High temperature - phone off */
		pr_debug
		    ("--->get_tbat_state[MAX] Extra High temperature - "
		     "phone should be off\n");
	}
	return tstate;
}
extern void usb_switch_uart(void);
extern void uart_switch_usb(void);

static void charger_fsm(unsigned int event)
{
        u16 vbat;
        enum charge_enable_state_t charger_status;
        enum tbat_state_t temperature_status;
        enum chg_state_t chgvoltage_status;

        mutex_lock(&charger_input_lock);
        /* gather charger data*/
                        charger_status  = get_charge_state();
	if (charger_status == BATTERY_NO_CHARGER){
                   
		  printk("charger_fsm:BATTERY_NO_CHARGER\n");
                                info->bat_online = 1;
                info->usb_online = 0;
                info->ac_online = 0;
		ac_usb = 0;
				usb_switch_uart();
	}else if((charger_status == CHARGER_AND_BATTERY)){
		    printk("charger_fsm:CHARGER_AND_BATTERY\n");
                                info->bat_online = 1;
                                info->usb_online = 1;
				     uart_switch_usb();
	}else if(charger_status == CHARGER_NO_BATTERY){
		    printk("charger_fsm:CHARGER_NO_BATTERY\n");
                info->bat_online = 0;
                info->usb_online = 1;
				  uart_switch_usb();
                        }
		else
		{
		usb_switch_uart();
		    printk("charger_fsm:CHARGER_DISABLE\n");
		ac_usb = 0;
                        }
                temperature_status = get_tbat_state();
	       chgvoltage_status = get_vchg_state();

	read_vbat(&vbat);

#ifdef CONFIG_PM860x_FG
	if (charger_status == CHARGER_DISABLE ||
	    charger_status == BATTERY_NO_CHARGER) {
		chg_from_dischged = 0;
		if(low_suspend == 0)
		set_vbat_th(1000,7100);
	}
	if ((event == PMIC_EVENT_INIT)) {
		pm860x_fg_poweron_init();
	}

	if (event & PMIC_EVENT_BAT_DET) {
                if (charger_status == CHARGER_AND_BATTERY ||
                    charger_status == BATTERY_NO_CHARGER) {
                        pm860x_fg_clear_cc();
                        soc_start = pm860x_fg_get_soc(ACTIVE_OCV_MODE);
                }
        }

	if (event & PMIC_EVENT_CHARG_COMP) {
		pr_debug("PMIC_EVENT_CHARG_COMP\n");
		info->charge_state = NO_CHARGE;
		read_vbat(&vbat);
		if (vbat > 4180 && charger_status == CHARGER_AND_BATTERY) {
			/*      charging done from fast charging */
			if (chg_from_dischged) {
				pm860x_fg_get_cc();
				q_max = q_chg;
				need_get_bat_info = 1;
				pr_debug
				    ("q_max:%d,need_get_bat_info=%d------>\n",
				     q_max, need_get_bat_info);
			}
			pm860x_fg_clear_cc();
			pr_debug
			    ("CSM[FULL] app should display full battery\n");
			
			soc_start = 100;
		}

	}

	if (event & PMIC_EVENT_CC) {
		pm860x_fg_get_cc();
		pr_debug("--------PMIC_EVENT_CC:-------\n");
	}
#endif

	pr_debug("--->charger_fsm-vbat[%d]mV,chargerSts[%d], "
		 "tempSts[%d],charge_state[%d]\n", vbat, charger_status,
		 temperature_status,info->charge_state);

	if (event & (PMIC_EVENT_IOVER |
		     PMIC_EVENT_TINT |
		     PMIC_EVENT_VBATMON |
		     PMIC_EVENT_VSYS |
		     PMIC_EVENT_CAHRG_FAIL)) {
		stop_charging();
		info->charge_state = NO_CHARGE;
		pr_debug("charger_fsm[all other event] stop charge:event:%x\n",
			 event);
	} else
	    if ((/*charger_status == BATTERY_NO_CHARGER//
		 ||*/ charger_status == CHARGER_AND_BATTERY)
		&& (temperature_status == CHARGE_ALLOW)&&(chgvoltage_status==VCHARGE_ALLOW)) {
		/* Battery is empty, do: pre-charge */
		/*
		 * if battery < 3100mv then pre-charge and
		 *  wait till we reach 3200mV
		 */
		 chg_failed=0; 
		if (vbat <= VBAT_TS_DISCHARGE_DPOINT) {
			if (info->charge_state != NO_CHARGE) {
				/* if we are charging
				 * then stop it before switching.
				 */
				stop_charging();
				info->charge_state = NO_CHARGE;
				pr_debug("charger_fsm[PRECH] "
					 "stop charge\n");
			}
      chg_full=0; 
			set_vbat_th(VBAT_TS_DISCHARGE_LOW,
				    VBAT_TS_DISCHARGE_UPP);

			if (charger_status == CHARGER_AND_BATTERY) {
				start_pre_charging();
				info->charge_state = PRE_CHARGE;
				/*
				 * notify user space - app should block
				 * all phone functionalties since we are
				 * low on battery
				 */
			} else {
				pr_debug
				    ("charger_fsm[PRECH] not start_pre_charging\n");
			}

		} else if (vbat > VBAT_TS_DISCHARGE_DPOINT &&
			   vbat <= VBAT_TS_LOWBAT_DPOINT) {
			/* Battery is low, do: fast-charge */
			/* if we came from pre-charge stop it before switching */
			if (info->charge_state != NO_CHARGE) {
				stop_charging();
				info->charge_state = NO_CHARGE;
				pr_debug("charger_fsm[LOW] stop charge\n");
			}
      chg_full=0; 
			set_vbat_th(VBAT_TS_LOWBAT_LOW,
				    VBAT_TS_LOWBAT_UPP);
			/* start fast charge only if we are not in it already */
			if (info->charge_state != FAST_CHARGE) {
				if (charger_status == CHARGER_AND_BATTERY) {
					start_charging();
					info->charge_state = FAST_CHARGE;
					/* notify user space - app should block all
					 * phone functionalties since we are still
					 * low on battery
					 */
				} else {
					pr_debug
					    ("charger_fsm[LOW] not start charging\n");
				}
			}
		} else if
		    (vbat > VBAT_TS_LOWBAT_DPOINT
		     && vbat <= VBAT_TS_PULSE_DPOINT) {
			/*NORMAL - fast-charge */
			/*couldn't be , but for safety:
			 * if we came from pre-charge stop
			 * it before switching.*/
			if (info->charge_state != NO_CHARGE) {
				stop_charging();
				info->charge_state = NO_CHARGE;
				pr_debug
				    ("charger_fsm[NORMAL] stop charge from pre-charge\n");
			}
      chg_full=0; 
			set_vbat_th(VBAT_TS_NORMAL_LOW,
				    VBAT_TS_PULSE_UPP);

			/*start fast charge only if we are not in it already. */
			if (info->charge_state != FAST_CHARGE) {
				if (charger_status == CHARGER_AND_BATTERY) {
					start_charging();
					info->charge_state = FAST_CHARGE;
					/* notify user space -
					 * app should display charger icon */
					pr_debug
					    ("[NORMAL] app should display charger icon\n");
				} else
					pr_debug
					    ("CSM[NORMAL] not start charging\n");
			}
		} 
		 else if (vbat > VBAT_TS_PULSE_DPOINT) {
			/* Battery is full, do: no-charge */
			stop_charging();
			//start_matain_charging();//start_pulse_charging();
			chg_full=1; 
			pr_debug("--->charger_fsm[FULL] stop charge\n");
			info->charge_state = NO_CHARGE;
			set_vbat_th(VBAT_TS_FULL_LOW, VBAT_TS_FULL_UPP);
			/* notify user space -
			 * app should display full battery
			 */
			pr_debug
			    ("CSM[FULL] app should display full battery\n");
		}
	} else {
		/* charger or battery are not detected or
		 * charger is disable or
		 * temperature is not in range
		 */
		 
		stop_charging();
		pr_debug("charger_fsm[charg or bat or temp not] "
			 "stop charge\n");
		info->charge_state = NO_CHARGE;
          chg_full=0;   
          chg_failed=0; 
		/* "discharge bat" or "low bat" state */
		if ((vbat < VBAT_TS_DISCHARGE_DPOINT) ||
		    (vbat > VBAT_TS_DISCHARGE_DPOINT &&
		     vbat < VBAT_TS_LOWBAT_DPOINT)) {
			pr_debug("CSM - vbat = [%d]mV notify user "
				 "and enter S3 \n", vbat);
			/* notify user space */
			/* enter S3 */
			/*normal vbat */
		} else if (vbat > VBAT_TS_LOWBAT_DPOINT &&
			        vbat < VBAT_TS_PULSE_DPOINT) { //vbat < VBAT_TS_NORMAL_DPOINT) {  
			pr_debug("CSM[NORMAL]vbat=[%d]mV normal working "
				 "voltage(COM can work)\n", vbat);
			/* if COM side is in reset mode, release it */
		}
		if(ovchprotect==1)
        		chg_full = 1;
	}
	mutex_unlock(&charger_input_lock);
	/*update power supply status*/
	schedule_delayed_work(&info->monitor_work, 5 * HZ);

}

static void charger_timer_handler(unsigned long data)
{
	charger_timer_start(3000);
	schedule_work(&info->charger_info_dump_work);
}

void pm860x_charger_info_dump(struct work_struct *work)
{
	u16 ibat, vbat, tbat, tint, vsys, vchrg, vbat_avg;
	u8 sign;

	/* read the battery level and decide if need
	 * charge also need to read temperature*/
	read_vbat(&vbat);
	read_vsys(&vsys);
	read_vchg(&vchrg);
	read_vbat_avg(&vbat_avg);

	read_ibat(&ibat, &sign);
	read_tbat(&tbat);
	read_tint(&tint);
	if (sign)
		pr_debug("--->pm860x_charger_worker ibat[-%d]mA \n", ibat);
	else
		pr_debug("--->pm860x_charger_worker ibat[%d]mA \n", ibat);

	pr_debug("--->Vbat =[%d]mV tbat[%d]mV tint[%d]C,ChrgSts[%d]\n",
		 vbat, tbat, tint, info->charge_state);
	pr_debug("--->Vsys =[%d]mV vchrg[%d]mV vbatavg[%d]mV\n",
		 vsys, vchrg, vbat_avg);
	sign = pm860x_reg_read(info->i2c_pm8607, 0x48);
	pr_debug("[0x48]=0x%02x\n", sign);
	{
#if 1
		u32 ocv = 0;
		pm860x_fg_get_cc();
		pr_debug("--->q_chg = %d mAh\n", q_chg);
		pr_debug("--->q_dischg = %d mAh\n", q_dischg);
		ocv = pm860x_fg_get_ocv();
		pr_debug("ocv:%d mV, rbat=%d mOh\n", ocv, r_bat);
		pm860x_fg_get_qrm();
#endif
	}
	if (need_get_bat_info)
		pm860x_fg_gather_bat_info();
}

static void init_charger_timer(void)
{
	init_timer(&info->charger_timer);
	info->charger_timer.function = charger_timer_handler;
	info->charger_timer.data = (unsigned long) NULL;
}

irqreturn_t pm8607_charger_handler(int irq, void *data)
{

	unsigned int event = 0;
	int irq_val = 0;
	if(info==NULL)
		return 0;
	irq_val = irq - info->chip->irq_base;

	switch (irq_val) {
	case PM8607_IRQ_CHG:
		pr_debug("--->pm860x_charger::PM8607_IRQ_CHG\n");
		event |= (PMIC_EVENT_CHDET);
		break;
	case PM8607_IRQ_BAT:
		pr_debug("--->pm860x_charger::PM8607_IRQ_BAT\n");
		event |= PMIC_EVENT_BAT_DET;
		break;
	case PM8607_IRQ_VBAT:
		pr_debug("--->pm860x_charger::PM8607_IRQ_VBAT\n");
		event |= PMIC_EVENT_VBAT_TS;
		break;
	case PM8607_IRQ_VCHG:
		pr_debug("--->pm860x_charger::PM8607_IRQ_VCHG\n");
		event |= PMIC_EVENT_VCHG_TS;
		//chg_failed=1; 
		break;
	case PM8607_IRQ_VSYS:
		pr_debug("--->pm860x_charger::PM8607_IRQ_VSYS\n");
		event |= PMIC_EVENT_VSYS;
		chg_failed=1; 
		break;
	case PM8607_IRQ_TINT:
		pr_debug("--->pm860x_charger::PM8607_IRQ_TINT\n");
		event |= PMIC_EVENT_TINT;
		chg_failed=1; 
		break;
	case PM8607_IRQ_GPADC1:
		pr_debug("--->pm860x_charger::PM8607_IRQ_GPADC1\n");
		event |= PMIC_EVENT_TBAT;
		break;
	case PM8607_IRQ_CHG_FAIL:
		pr_debug("--->pm860x_charger::PM8607_IRQ_CHG_FAIL\n");
		event |= PMIC_EVENT_CAHRG_FAIL;
		chg_failed=1; 
		break;
	case PM8607_IRQ_CHG_DONE:
		pr_debug("--->pm860x_charger::PM8607_IRQ_CHG_DONE\n");
		event |= PMIC_EVENT_CHARG_COMP;
		break;
	case PM8607_IRQ_CHG_FAULT:
		pr_debug("--->pm860x_charger::PM8607_IRQ_CHG_FAULT\n");
		event |= PMIC_EVENT_IOVER;
		break;
	case PM8607_IRQ_CC:
		pr_debug("--->pm860x_charger::PM8607_IRQ_CC\n");
		event |= PMIC_EVENT_CC;
		break;
	default:
		pr_debug("--->pm860x_charger::unknown irq:%d\n", irq_val);
		break;
	}
	charger_fsm(event);

	return 0;
}

EXPORT_SYMBOL(pm8607_charger_handler);




static int pm860x_ac_get_prop(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct pm860x_power_info *info = dev_get_drvdata(psy->dev->parent);
	int ret = 0;
	u16 data;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(ac_usb == 1)
		val->intval = info->ac_online;
		else
		val->intval = 0;

		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (info->ac_online) {
			read_vchg(&data);
			val->intval = data;
			break;
		}
		ret = -ENODATA;
		break;
	default:
		ret = -ENODEV;
		break;
	}
	return ret;
}

static enum power_supply_property pm860x_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int pm860x_usb_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct pm860x_power_info *info = dev_get_drvdata(psy->dev->parent);
	int ret = 0;
	u16 data;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(ac_usb == 1)
		val->intval = info->usb_online;
		else
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (info->usb_online) {
			read_vchg(&data);
			val->intval = data;	/* unit is mV */
			break;
		}
		ret = -ENODATA;
		break;
	default:
		ret = -ENODEV;
		break;
	}
	return ret;
}

static enum power_supply_property pm860x_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int pm860x_bat_get_prop(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	u8 tmp;
	u16 data;
	int value,ret=0;
	struct pm860x_power_info *info = dev_get_drvdata(psy->dev->parent);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!info->bat_online) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		}
		tmp = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
		if (tmp & 0x3){
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
       info->usb_online = 1; 
       }
    else if(chg_full){    
       val->intval = POWER_SUPPLY_STATUS_FULL;
       info->usb_online = 1; 
       }
    else if (chg_failed){
       val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
       info->usb_online = 1; 
       }
    else {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
       info->ac_online = 0;
       info->usb_online = 0;
       }
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = info->bat_online;
                break;
  case POWER_SUPPLY_PROP_CAPACITY:
      //value = pm860x_fg_get_qrm_percent(); 
        value = pm860x_fg_get_soc(ACTIVE_OCV_MODE);
                if (value < 0)
                        val->intval = 0;
                else if (value > 100)
                        val->intval = 100;
                else
			val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:     
        value = pm860x_fg_get_soc(ACTIVE_OCV_MODE);
                if (value < 1)
                        val->intval = 1;
                else if (value > 100)
                        val->intval = 100;
                else
      val->intval = value;
    break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		read_vbat(&data);
		val->intval = (data * 1000);	/*uV */
	if(data<=3310)
		low_bat_on = 1;
	else 
		low_bat_on = 0;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		tmp = pm860x_reg_read(info->i2c_pm8607, PM8607_IBAT_MEAS2);
		value = tmp & 0x3f;
		tmp = pm860x_reg_read(info->i2c_pm8607, PM8607_IBAT_MEAS1);
		value += tmp << 6;
		val->intval = value;	/*uA */
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 250;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = info->bat_online;
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static enum power_supply_property pm860x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

#define REQUEST_IRQ(_irq, _name)					\
do {									\
	ret = request_threaded_irq(chip->irq_base + _irq, NULL,		\
				    pm8607_charger_handler,		\
				    IRQF_ONESHOT, _name, info);		\
	if (ret)							\
		dev_err(chip->dev, "Failed to request IRQ #%d: %d\n",	\
			_irq, ret);					\
} while (0)

static __devinit int pm860x_init_charger(struct pm860x_chip *chip,
					 struct pm860x_power_info *info)
{
	int ret;
	enum charge_enable_state_t charger_status;
	u8 val;

	info->charge_state = NO_CHARGE;
	info->charge_type = USB_CHARGER;
	info->ac_online = 0;
	info->usb_online = 0;
	info->bat_online = 0;
        charger_status = get_charge_state();
        if (charger_status == BATTERY_NO_CHARGER){
                info->bat_online = 1;
                info->ac_online = 0;
                info->usb_online = 0;
        }else if((charger_status == CHARGER_AND_BATTERY)){
                info->bat_online = 1;
                info->usb_online = 1;
	}else if((charger_status == CHARGER_NO_BATTERY)){
		info->ac_online = 1;
	}

	/*REQUEST_IRQ(PM8607_IRQ_CHG, "chg-detection"); */
	REQUEST_IRQ(PM8607_IRQ_BAT, "bat-detection");
	REQUEST_IRQ(PM8607_IRQ_VBAT, "vbat-out-range");
	REQUEST_IRQ(PM8607_IRQ_VCHG, "vchg-out-range"); 
	REQUEST_IRQ(PM8607_IRQ_VSYS, "vsys-out-range");
	REQUEST_IRQ(PM8607_IRQ_TINT, "pm8607-temp-out-range");
	REQUEST_IRQ(PM8607_IRQ_GPADC1, "batt-temp-out-range");
	REQUEST_IRQ(PM8607_IRQ_CHG_FAIL, "charger-time-out");
	REQUEST_IRQ(PM8607_IRQ_CHG_DONE, "charger-done");
	REQUEST_IRQ(PM8607_IRQ_CHG_FAULT, "charger-fault");
	REQUEST_IRQ(PM8607_IRQ_CC, "charger-cc");
	wake_lock_init(&lowbat_wakeup, WAKE_LOCK_SUSPEND, "unknown_wakeups");
	INIT_WORK(&info->charger_info_dump_work, pm860x_charger_info_dump);
	init_charger_timer();

	/*
	 * Enable interrupt:
	 * the over current on charger reverse,
	 * the VBAT less then VBATMON,
	 * the Battery over temperature,
	 * the Charger detection/removal.
	 */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_MEAS_ENABLE1);
	val |= (PM8607_MEAS_EN1_VSYS | PM8607_MEAS_EN1_VCHG |
		PM8607_MEAS_EN1_VBAT | PM8607_MEAS_EN1_TINT |
		PM8607_MEAS_EN1_TBAT);
	pm860x_reg_write(info->i2c_pm8607, PM8607_MEAS_ENABLE1, val);

	val = pm860x_reg_read(info->i2c_pm8607, PM8607_MEAS_ENABLE3);
        val |= (PM8607_MEAS_EN3_IBAT | PM8607_MEAS_EN3_BAT_DET_EN_B0);
        pm860x_reg_write(info->i2c_pm8607, PM8607_MEAS_ENABLE3, val);

        /* setting MEAS_OFF_TIME=0x1b20 ,GPADC low power setting */
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_MEAS_OFF_TIME1);
        val |= 0x80 | PM8607_MEASOFFTIME1_MEAS_EN_SLP;
        pm860x_reg_write(info->i2c_pm8607, PM8607_MEAS_OFF_TIME1, val);
        pm860x_reg_write(info->i2c_pm8607, PM8607_MEAS_OFF_TIME2, 0x6c);

	/* hw issue fix from sanremo C0 unexpected BAT interrupt when chip-sleep */
	val = pm860x_reg_read(info->i2c_pm8607, 0x3a);
	val = (val & ~0x0F) | 0x2;
	pm860x_reg_write(info->i2c_pm8607, 0x3a, val);

	/* make sure GPADC is on */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_GPADC_MISC1);
	val |= (PM8607_GPADC_MISC1_GPFSM_EN);
	pm860x_reg_write(info->i2c_pm8607, PM8607_GPADC_MISC1, val);

	pm860x_reg_write(chip->client, PM8607_MEAS_EN1, 
	pm860x_reg_read(chip->client, PM8607_MEAS_EN1)|0x40); 

	/* enable BAT detect by BTEMP pin,GPADC1 */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL6);
	//val |= BD_MSK_GPDAC1;
	val &=~BD_MSK_MASK;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL6, val);

	/* current src level (90=50uA/20=15uA)for battery temperature */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_GP_BIAS2);
	val = (val & ~GP1_BIAS_SET_MASK) | GP1_BIAS_SET_15UA;
	pm860x_reg_write(info->i2c_pm8607, PM8607_GP_BIAS2, val);

	/* IBAT AVG for Ibat 2 comp. calc see read_ibat */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CC_READ);
	val |= CC_AVG_SEL_4;
	pm860x_reg_write(info->i2c_pm8607, PM8607_CC_READ, val);

	/* enable CC and ibat measurement */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_MEAS_ENABLE3);
	//val |= (PM8607_MEAS_EN3_COULOMB_COUNTER | PM8607_MEAS_EN3_IBAT);
	val |= (PM8607_MEAS_EN3_IBAT);
	val |= PM8607_MEAS_EN3_COULOMB_COUNTER;
	pm860x_reg_write(info->i2c_pm8607, PM8607_MEAS_ENABLE3, val);


	/*set VSYS_LOW_TH =2.6v for precharge feature in over discharge status */
	val = ((2600 * 0xFF) / 5400);
	pm860x_reg_write(info->i2c_pm8607, PM8607_VSYS_LOW_TH, val);

	/*reset VCHG_ON_CNT_SEL =16s default value */
	val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL5);
        val = (val & ~VCHG_ON_CNT_SEL_MASK) | VCHG_ON_CNT_SEL_16SEC;
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL5, val);
        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL7);
        val &= ~BAT_REM_EN;
        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL7, val);

        /*IBAT calibration:because there is a gap between IBAT value measured
        in real user case by multimeter and sanremo IBAT value by ADC, these register
        is internal test page register, so there is not description on SPEC */
        /* entry test page----> */
        i2c_smbus_write_byte(info->i2c_pm8607, 0xfa);
        i2c_smbus_write_byte(info->i2c_pm8607, 0xfb);
        i2c_smbus_write_byte(info->i2c_pm8607, 0xff);

	pm860x_reg_write(info->i2c_pm8607, 0xD4, 0xfe);
	pm860x_reg_write(info->i2c_pm8607, 0xD5, 0xd1);
	val = pm860x_reg_read(info->i2c_pm8607, 0xD7);
	val = (val & 0xF) | (0xb << 4);
        pm860x_reg_write(info->i2c_pm8607, 0xD7, val);

        val = pm860x_reg_read(info->i2c_pm8607, 0xD4);

        i2c_smbus_write_byte(info->i2c_pm8607, 0xfe);
        i2c_smbus_write_byte(info->i2c_pm8607, 0xfc);
        /* <--- exit test page */
	board_id = pm860x_get_board();

        /* activate charging state machine -
           this will set the TBAT & VBAT treshold */
	charger_fsm(PMIC_EVENT_INIT);
	printk("u810/802 board id ++++= %d\n",board_id);
	return 0;
}

#define FREE_IRQ(_irq)							\
do {									\
	free_irq(chip->irq_base + _irq, info);				\
} while (0)


static __devexit int pm860x_deinit_charger(struct pm860x_power_info *info)
{
	struct pm860x_chip *chip = info->chip;
	FREE_IRQ(PM8607_IRQ_CHG);
	FREE_IRQ(PM8607_IRQ_BAT);
	FREE_IRQ(PM8607_IRQ_VBAT);
	FREE_IRQ(PM8607_IRQ_VCHG);
	FREE_IRQ(PM8607_IRQ_VSYS);
	FREE_IRQ(PM8607_IRQ_TINT);
	FREE_IRQ(PM8607_IRQ_GPADC1);
	FREE_IRQ(PM8607_IRQ_CHG_FAIL);
	FREE_IRQ(PM8607_IRQ_CHG_DONE);
	FREE_IRQ(PM8607_IRQ_CHG_FAULT);
	wake_lock_destroy(&lowbat_wakeup);
	del_timer(&info->charger_timer);

	return 0;
}

#ifdef	CONFIG_PROC_FS
#define PM860X_POWER_REG_NUM		0xef
#define	PM860X_POWER_PROC_FILE	"driver/pm860x_power"
static struct proc_dir_entry *pm860x_power_proc_file;
static int index;

static ssize_t pm860x_power_proc_read(struct file *filp,
				      char *buffer, size_t length,
				      loff_t * offset)
{
	u8 reg_val;

	if (index == 0xffff) {
		int i;
		printk(KERN_INFO "pm8607:sanremo reg dump\n");
		for (i = 0; i < PM860X_POWER_REG_NUM; i++) {
			reg_val = pm860x_reg_read(info->i2c_pm8607, i);
			printk(KERN_INFO "[0x%02x]=0x%02x\n", i, reg_val);
		}
		printk(KERN_INFO "pm8606:portofino reg dump\n");
		for (i = 0; i < 0x1f; i++) {
			reg_val = pm860x_reg_read(info->i2c_pm8606, i);
			printk(KERN_INFO "[0x%02x]=0x%02x\n", i, reg_val);
		}
		return 0;
	}
	if ((index < 0) || (index > PM860X_POWER_REG_NUM))
		return 0;

	reg_val = pm860x_reg_read(info->i2c_pm8607, index);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	pm860x_fg_get_cc();
	return 0;
}

static ssize_t pm860x_power_proc_write(struct file *filp,
				       const char *buff, size_t len,
				       loff_t * off)
{
	u8 reg_val;
	char messages[256], vol[256];
	int value = 0, hi_value = 0;

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages + 1, len - 1);
		index = (int) simple_strtoul(vol, NULL, 16);
		printk("index=0x%x\n", index);
	} else if ('c' == messages[0]) {
		if ('1' == messages[1]) {
			charger_timer_start(200);
		} else if ('0' == messages[1]) {
			charger_timer_stop();
		}
	} else if ('r' == messages[0]) {
		printk("--->fsm:TEST PAGE start");
		/* ---> enter test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfa);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfb);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xff);

		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD4);
		printk(KERN_INFO "[0xD4]=0x%02x\n", reg_val);
		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD5);
		printk(KERN_INFO "[0xD5]=0x%02x\n", reg_val);
		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD7);
		printk(KERN_INFO "[0xD7]=0x%02x\n", reg_val);

		/* <--- exit test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfe);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfc);
		printk("--->fsm:TEST PAGE end");
	} else if ('w' == messages[0] && '+' == messages[1]) {
		printk("--->fsm:TEST PAGE start");
		/* ---> enter test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfa);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfb);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xff);

		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD4);
		printk(KERN_INFO "[0xD4]=0x%02x\n", reg_val);
		value = reg_val;
		value += 1;
		pm860x_reg_write(info->i2c_pm8607, 0xD4, value);


		/* <--- exit test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfe);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfc);
		printk("--->fsm:TEST PAGE end");
	} else if ('w' == messages[0] && '-' == messages[1]) {
		printk("--->fsm:TEST PAGE start");
		/* ---> enter test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfa);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfb);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xff);

		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD4);
		printk(KERN_INFO "[0xD4]=0x%02x\n", reg_val);
		value = reg_val;
		value -= 1;
		pm860x_reg_write(info->i2c_pm8607, 0xD4, value);


		/* <--- exit test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfe);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfc);
		printk("--->fsm:TEST PAGE end");
	} else if ('w' == messages[0] && 'p' == messages[1]) {

		printk("--->fsm:TEST PAGE start");
		/* ---> enter test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfa);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfb);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xff);

		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD5);
		value = reg_val;
		value += 16;
		if (value >= 256) {
			reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD7);
			value = value % 256;
			hi_value = (reg_val >> (4 + 1)) << 4;
			hi_value = (hi_value & 0xF0) | (reg_val & 0xF);
			printk(KERN_INFO "hi_value=0x%02x\n", hi_value);
			pm860x_reg_write(info->i2c_pm8607, 0xD7, hi_value);
			reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD7);
			printk(KERN_INFO "[0xD7]=0x%02x\n", reg_val);
		}

		pm860x_reg_write(info->i2c_pm8607, 0xD5, value);
		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD5);
		printk(KERN_INFO "[0xD5]=0x%02x\n", reg_val);


		/* <--- exit test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfe);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfc);
		printk("--->fsm:TEST PAGE end");
	} else if ('w' == messages[0] && 'm' == messages[1]) {
		printk("--->fsm:TEST PAGE start");
		/* ---> enter test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfa);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfb);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xff);

		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD5);
		value = reg_val;
		value -= 16;
		if (value < 0) {
			reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD7);
			value = (value + 256) % 256;
			hi_value = ((reg_val >> 4) - 1) << 4;
			hi_value = (hi_value & 0xF0) | (reg_val & 0xF);

			printk(KERN_INFO "hi_value=0x%02x\n", hi_value);
			pm860x_reg_write(info->i2c_pm8607, 0xD7, hi_value);
			reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD7);
			printk(KERN_INFO "[0xD7]=0x%02x\n", reg_val);

		}

		pm860x_reg_write(info->i2c_pm8607, 0xD5, value);
		reg_val = pm860x_reg_read(info->i2c_pm8607, 0xD5);
		printk(KERN_INFO "[0xD5]=0x%02x\n", reg_val);


		/* <--- exit test page */
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfe);
		i2c_smbus_write_byte(info->i2c_pm8607, 0xfc);
		printk("--->fsm:TEST PAGE end");

	} else {
		/* set the register value */
		reg_val = (int) simple_strtoul(messages, NULL, 16);
		pm860x_reg_write(info->i2c_pm8607, index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations pm860x_power_proc_ops = {
	.read = pm860x_power_proc_read,
	.write = pm860x_power_proc_write,
};

static void create_pm860x_power_proc_file(void)
{
	pm860x_power_proc_file =
	    create_proc_entry(PM860X_POWER_PROC_FILE, 0644, NULL);
	if (pm860x_power_proc_file) {
		pm860x_power_proc_file->proc_fops = &pm860x_power_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_pm860x_power_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(PM860X_POWER_PROC_FILE, &proc_root);
}

#endif
void pm860x_set_charger_type(enum enum_charger_type type )
{
	unsigned char val;
	switch(type)
	{
		case USB_CHARGER:
			info->charge_type = USB_CHARGER;
			info->usb_online = 1;
			info->ac_online = 0;
			ac_usb = 1;
			pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
			 PM8606_PREREG_VSYS_SET |
			 PM8606_PREREG_CURLIM_SET_450V);			
			break;
		case AC_STANDARD_CHARGER:
			info->charge_type = AC_STANDARD_CHARGER;
			info->usb_online = 0;
			info->ac_online = 1;
			ac_usb = 1;
		         pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
		 PM8606_PREREG_VSYS_SET |
		 PM8606_PREREG_CURLIM_SET_810V);
			break;
		case AC_OTHER_CHARGER:
			info->charge_type = AC_OTHER_CHARGER;
			info->usb_online = 0;
			info->ac_online = 1;
			ac_usb = 1;
		         pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
		 PM8606_PREREG_VSYS_SET |
		 PM8606_PREREG_CURLIM_SET_810V);
			break;
		case NO_CHARGER:
			info->charge_type = NO_CHARGER;
			info->usb_online = 0;
			info->ac_online = 0;
			ac_usb = 0;
			pm860x_reg_write(info->i2c_pm8606, PM8606_PREREGULATORA,
			 PM8606_PREREG_VSYS_SET |
			 PM8606_PREREG_CURLIM_SET_450V);
			break;
		default:
			break;

	}

	if (get_charge_state() == CHARGER_AND_BATTERY){
		val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
		if((val & CHARGER_MODE_MASK)==CHARGER_MODE_PRE_CHARGE)
			pr_debug("--------CHARGER_MODE_PRE_CHARGE:-------\n");
		else if((val & CHARGER_MODE_MASK)==CHARGER_MODE_FAST_CHARGE){
			val &= ~CHARGER_MODE_MASK;
			pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1,val);
			/*set chg current*/
			val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL2);
                        if(type==USB_CHARGER)
			val = (val & ~ICHG_SET_MASK) | ICHG_SET_300MA;
						//val = (val & ~ICHG_SET_MASK) | ICHG_SET_500MA;
                        else if(type==AC_STANDARD_CHARGER)
			val = (val & ~ICHG_SET_MASK) | ICHG_SET_700MA;  //ICHG_SET_750MA    
						// val = (val & ~ICHG_SET_MASK) | ICHG_SET_500MA;
                        else if(type==AC_OTHER_CHARGER)
			val = (val & ~ICHG_SET_MASK) | ICHG_SET_700MA;  //ICHG_SET_750MA   
						//val = (val & ~ICHG_SET_MASK) | ICHG_SET_1000MA;
						val = (val & ~BB_PREG_OFF) | TOFFMAX_SET| BB_LRSW_EN;  
                        pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL2, val);

                        val = pm860x_reg_read(info->i2c_pm8607, PM8607_CHG_CTRL1);
			val = (val & ~CHARGER_MODE_MASK) | CHARGER_MODE_FAST_CHARGE;
			pm860x_reg_write(info->i2c_pm8607, PM8607_CHG_CTRL1, val);
		}
	}
	/*update power supply status*/
	schedule_delayed_work(&info->monitor_work, 5 * HZ);

}

int pm860x_set_vibrator(unsigned char value)
{               
      printk("set_vibrator----------------=0x%x\n",value);
        if(value == 0) {                
        pm860x_reg_write(info->i2c_pm8606,PM8606_VIBRATORA, 0x00);//disable LDO,1.22V default,          
        pm860x_reg_write(info->i2c_pm8607,PM8607_VIBRATOR_PWM, 0x00);//0%       
        
        } 
      else {            
        pm860x_reg_write(info->i2c_pm8606,PM8606_VIBRATORA, 0x01|((value&0x7)<<1));//enable LDO 
        pm860x_reg_write(info->i2c_pm8607,PM8607_VIBRATOR_PWM, 0xFF);//62%      
      
        }       
          return 0;
}
EXPORT_SYMBOL(pm860x_set_vibrator);

int pm860x_get_board(void)
{           
      u16 data=0,i=0,sum=0;
     for(i=0;i<10;i++)
      {
      	read_boardid(&data);
	sum+=data;
     }
	sum =sum/10;
    if(sum<2500)
    return 1;
    else
    return 2;
	  
}
int pm860x_get_boardID(void)
{           
	return board_id;	  
}

//EXPORT_SYMBOL(pm860x_get_boardID);

static void battery_monitor_work(struct work_struct *work)
{
        power_supply_changed(&info->battery);
	//if(info->bat_online)
		schedule_delayed_work(&info->monitor_work, 5 * HZ);
}

static __devinit int pm860x_power_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_platform_data *pm860x_pdata;
	struct pm860x_power_pdata *pdata = NULL;
	int ret;

	if (pdev->dev.parent->platform_data) {
		pm860x_pdata = pdev->dev.parent->platform_data;
		pdata = pm860x_pdata->power;
	}

	if (!pdata) {
		dev_err(&pdev->dev, "platform data isn't assigned to "
			"power supply\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct pm860x_power_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->chip = chip;
	info->i2c_pm8606 = (chip->id == CHIP_PM8607) ?
	    chip->companion : chip->client;
	info->i2c_pm8607 = (chip->id == CHIP_PM8607) ?
            chip->client : chip->companion;

        platform_set_drvdata(pdev, info);
        INIT_DELAYED_WORK(&info->monitor_work, battery_monitor_work);
        pm860x_init_charger(chip, info);

        info->ac.name = "ac";
        info->ac.type = POWER_SUPPLY_TYPE_MAINS;
	info->ac.properties = pm860x_ac_props;
	info->ac.num_properties = ARRAY_SIZE(pm860x_ac_props);
	info->ac.get_property = pm860x_ac_get_prop;
	ret = power_supply_register(&pdev->dev, &info->ac);
	if (ret)
		goto out;
	info->ac.dev->parent = &pdev->dev;

	info->usb.name = "usb";
	info->usb.type = POWER_SUPPLY_TYPE_USB;
	info->usb.properties = pm860x_usb_props;
	info->usb.num_properties = ARRAY_SIZE(pm860x_usb_props);
	info->usb.get_property = pm860x_usb_get_prop;
	ret = power_supply_register(&pdev->dev, &info->usb);
	if (ret)
		goto out_usb;
	info->usb.dev->parent = &pdev->dev;

	info->battery.name = "battery";
	info->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	info->battery.properties = pm860x_battery_props;
	info->battery.num_properties = ARRAY_SIZE(pm860x_battery_props);
	info->battery.get_property = pm860x_bat_get_prop;
	ret = power_supply_register(&pdev->dev, &info->battery);
	if (ret)
		goto out_battery;
	info->battery.dev->parent = &pdev->dev;

        info->pm8607_id =
            pm860x_reg_read(info->i2c_pm8607, PM8607_CHIP_ID);

#ifdef  CONFIG_PROC_FS
        create_pm860x_power_proc_file();
#endif

	return 0;

      out_battery:
	power_supply_unregister(&info->battery);
      out_usb:
	power_supply_unregister(&info->ac);
      out:
	kfree(info);
	return ret;
}

static __devexit int pm860x_power_remove(struct platform_device *pdev)
{
	struct pm860x_power_info *info = platform_get_drvdata(pdev);

	if (info) {
		power_supply_unregister(&info->ac);
		power_supply_unregister(&info->usb);
		power_supply_unregister(&info->battery);
		cancel_delayed_work_sync(&info->monitor_work);
		pm860x_deinit_charger(info);
		kfree(info);
#ifdef	CONFIG_PROC_FS
		remove_pm860x_power_proc_file();
#endif
	}
        return 0;
}

#ifdef  CONFIG_PM
static int pm860x_power_suspend(struct platform_device *pdev, pm_message_t state)
{
	low_suspend = 1;
	if(low_bat_on == 0)
	set_vbat_th(3340,5400);
	else 
	set_vbat_th(3240,5400);

        return 0;
}

static int pm860x_power_resume(struct platform_device *pdev)
{
	u16 data =0;
	low_suspend = 0;
	schedule_delayed_work(&info->monitor_work, 5 * HZ);
	        read_vbat(&data);
        	if(data<=3340)
		low_bat_on = 1;
	else 
		low_bat_on = 0;
	if(low_bat_on ==1)
        wake_lock_timeout(&lowbat_wakeup,3000);

	volt_index = (volt_index+1)%loop;
	volt_loop[volt_index] = data;
        if(info->pm8607_id == PM8607_C1D_ID)
        {
                pm860x_fg_clear_cc();
                soc_start = pm860x_fg_get_soc(ACTIVE_OCV_MODE);
        }
        pr_debug("----->pm860x_power_resume,soc_start:%d\n",soc_start);
        return 0;
}
#else
#define pm860x_battery_suspend  NULL
#define pm860x_battery_resume   NULL
#endif

static struct platform_driver pm860x_power_driver = {
        .probe = pm860x_power_probe,
        .remove = __devexit_p(pm860x_power_remove),
        .suspend    = pm860x_power_suspend,
        .resume     = pm860x_power_resume,
        .driver = {
                   .name = "88pm860x-power",
                   .owner = THIS_MODULE,
		   },
};

static int __init pm860x_power_init(void)
{
	return platform_driver_register(&pm860x_power_driver);
}

module_init(pm860x_power_init);

static void __exit pm860x_power_exit(void)
{
	platform_driver_unregister(&pm860x_power_driver);
}

module_exit(pm860x_power_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Power supply driver for 88pm860x");
MODULE_ALIAS("platform:88pm860x-power");
