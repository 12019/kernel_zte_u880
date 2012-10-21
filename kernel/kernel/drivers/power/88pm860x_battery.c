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
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/mfd/88pm860x.h>
//#include <linux/mfd/88pm860x_soc.h>
#include <linux/delay.h>
#include <linux/i2c.h>

/* bit definitions of Status Query Interface 2 */
#define STATUS2_CHG			(1 << 2)
#define STATUS2_BAT			(1 << 3)
#define STATUS2_VBUS			(1 << 4)

/* bit definitions of Measurement Enable 1 Register */
#define MEAS1_TINT			(1 << 3)
#define MEAS1_GP1			(1 << 5)
#define MEAS1_GP2			(1 << 6)


/* bit definitions of Measurement Enable 3 Register */
#define MEAS3_IBAT			(1 << 0)
#define MEAS3_BAT_DET			(1 << 1)
#define MEAS3_CC			(1 << 2)

/* bit definitions of Measurement Off Time Register */
#define MEAS_OFF_SLEEP_EN		(1 << 1)

/* bit definitions of Charger Control 6 Register */
#define CC6_BAT_DET_GPADC1		1

/* bit definitions of Coulomb Counter Reading Register */
#define CCNT_AVG_SEL			(4 << 3)

/* bit definition of GPADC1 bias set*/
#define GP2_BIAS_SET_15UA		(0x2 << 4)

/*bit definition of GPADC fsm on*/
#define PM8607_GPADC_MISC1_GPFSM_EN (1<<0)

static u8 board_id = 0;
#define OCV_NUM 101

extern u32 array_soc[OCV_NUM][2];
extern int para_on,para_off;

#define CCNT_POS1			0
#define CCNT_POS2			1
#define CCNT_NEG1			2
#define CCNT_NEG2			3
#define CCNT_SPOS			4
#define CCNT_SNEG			5

/* OCV -- Open Circuit Voltage */
#define OCV_MODE_ACTIVE			0
#define OCV_MODE_SLEEP			1

#define VBATT_RESISTOR_MIN		3800
#define VBATT_RESISTOR_MAX		4100

#define MONITOR_INTERVAL		(HZ * 60)
static struct delayed_work	monitor_batteryvolt_work;
static int flag = 1;
int charger_flag = 1;
u8 full_excep = 0;
static int char_on = 0,soc_chg_on=0,soc_chg_off=0;
static unsigned int ncount = 0;

struct pm860x_battery_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct device		*dev;
	struct workqueue_struct	*monitor_wqueue;

	struct power_supply	battery;
	struct delayed_work	monitor_work;
	struct delayed_work	changed_work;
	struct mutex		lock;
	unsigned int		irq_base;
	int			status;
	int			irq_cc;
	int			irq_batt;
	int			max_capacity;
	int			resistor;	/* Battery Internal Resistor */
	int			start_soc;
	unsigned		present : 1;
	unsigned		temp_type : 1;	/* TINT or TBAT */
};

struct ccnt {
	unsigned long long int	pos;
	unsigned long long int	neg;
	unsigned int		spos;
	unsigned int		sneg;

	int			total_chg;	/* mAh(3.6C) */
	int			total_dischg;	/* mAh(3.6C) */
};

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

extern struct pm860x_charger_info *g_chinfo;

/*
 * State of Charge.
 * The first number is mAh(=3.6C), and the second number is percent point.
 */
 #if 0
int array_soc[][2] = {  {4170, 100},
			{4154, 99}, {4136, 98}, {4122, 97}, {4107, 96},
			{4102, 95}, {4088, 94}, {4081, 93}, {4070, 92},
			{4060, 91}, {4053, 90}, {4044, 89}, {4035, 88},
			{4028, 87}, {4019, 86}, {4013, 85}, {4006, 84},
			{3995, 83}, {3987, 82}, {3982, 81}, {3976, 80},
			{3968, 79}, {3962, 78}, {3954, 77}, {3946, 76},
			{3941, 75}, {3934, 74}, {3929, 73}, {3922, 72},
			{3916, 71}, {3910, 70}, {3904, 69}, {3898, 68},
			{3892, 67}, {3887, 66}, {3880, 65}, {3874, 64},
			{3868, 63}, {3862, 62}, {3854, 61}, {3849, 60},
			{3843, 59}, {3840, 58}, {3833, 57}, {3829, 56},
			{3824, 55}, {3818, 54}, {3815, 53}, {3810, 52},
			{3808, 51}, {3804, 50}, {3801, 49}, {3798, 48},
			{3796, 47}, {3792, 46}, {3789, 45}, {3785, 44},
			{3784, 43}, {3782, 42}, {3780, 41}, {3777, 40},
			{3776, 39}, {3774, 38}, {3772, 37}, {3771, 36},
			{3769, 35}, {3768, 34}, {3764, 33}, {3763, 32},
			{3760, 31}, {3760, 30}, {3754, 29}, {3750, 28},
			{3749, 27}, {3744, 26}, {3740, 25}, {3734, 24},
			{3732, 23}, {3728, 22}, {3726, 21}, {3720, 20},
			{3716, 19}, {3709, 18}, {3703, 17}, {3698, 16},
			{3692, 15}, {3683, 14}, {3675, 13}, {3670, 12},
			{3665, 11}, {3661, 10}, {3657, 9},  {3649, 8},
			{3637, 7},  {3609, 6},  {3562, 5},  {3510, 4},
			{3429, 3},  {3312, 2},  {3012, 1},  {2900, 0} };

	int array_soc[][2] =					  
		{ {4160, 99}, {4146, 98}, {4130, 97}, {4119, 96},
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
/*
 * Battery Temp (GPADC1)
 * The first number is mV, and the second number is degree.
 */
static int array_batt_temp[][2] = { {15, 100}, {46, 60}, {49, 57},   {53, 55},
				    {73, 45},  {81, 42}, {87, 40},   {338, 5},
				    {386, 2},  {423, 0}, {868, -15}, {962, -17},
				    {1125, -20}, {1462, -25}, };

static struct ccnt ccnt_data;
static struct pm860x_battery_info *ginfo;


#define loop 20
static int volt_index=-1;

int volt_loop[loop]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int volt_value = 0;
static int volt_pre = 0; 
/****
***These definition are for sanremo Ibat,vbat calibratio
****/
struct sanremo_nvram
{
	unsigned char d4value;
	unsigned char d5value;
	unsigned char d7value;
	char ibat_offset;
	char vbat_offset;
	unsigned short  vbat_slope_low;
	unsigned short  vbat_slope_high;
};
static struct sanremo_nvram sanremo_nvram_data;

#define __raw_writel(v,p) (*(unsigned long *)(p) = (v))
/*battery current calibration ibat remain.*/
static int ibat_offset_remain;
/*sanremo registers temp value,need to restore later */
static u8 temp_reg_0x4C, temp_reg_0x48, temp_reg_0x49;
/*for battery calibration sys interface use*/
static u8 temp_4c_1,temp_4c_2,temp_4c_3;
static u8 temp_48_1,temp_48_2,temp_48_3;
static u8 temp1,temp2,temp3;
static u16 vol_input;
 u8 chg_full = 0;  
u8 ovchprotect = 0;
u8 low_bat_on = 1;
u8 chg_full_en = 0;

/*define some var for battery calibration feature*/
struct sanremo_cali
{
	unsigned int vbat_slope_low;
	unsigned int vbat_slope_high;
	char vbat_offset;
	char ibat_offset;
};

static int calc_ccnt(struct pm860x_battery_info *info, struct ccnt *ccnt);
static int calc_soc(struct pm860x_battery_info *info, int state, int *soc);
static int clear_ccnt(struct pm860x_battery_info *info, struct ccnt *ccnt);
static int measure_current(struct pm860x_battery_info *info, int *data);

static irqreturn_t pm860x_coulomb_handler(int irq, void *data)
{
	struct pm860x_battery_info *info = data;
        printk("pm860x_coulomb_handler\n");

	calc_ccnt(info, &ccnt_data);
	return IRQ_HANDLED;
}

static irqreturn_t pm860x_batt_handler(int irq, void *data)
{
	struct pm860x_battery_info *info = data;
	int ret;
        printk("pm860x_batt_handler\n");
	mutex_lock(&info->lock);
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (ret & STATUS2_BAT) {
		info->present = 1;
		info->temp_type = PM860X_TEMP_TBAT;
	} else {
	        #if 0
		info->present = 0;
		info->temp_type = PM860X_TEMP_TINT;
		#endif
		info->present = 1;
		info->temp_type = PM860X_TEMP_TBAT;
	}
	mutex_unlock(&info->lock);
	/* clear ccnt since battery is attached or dettached */
	clear_ccnt(info, &ccnt_data);
	return IRQ_HANDLED;
}

static void pm860x_battery_work(struct work_struct *work)
{
	struct pm860x_battery_info *info = container_of(work,
		struct pm860x_battery_info, monitor_work.work);

	power_supply_changed(&info->battery);
	queue_delayed_work(info->chip->monitor_wqueue, &info->monitor_work,
			   MONITOR_INTERVAL);
}

static void pm860x_changed_work(struct work_struct *work)
{
	struct pm860x_battery_info *info = container_of(work,
		struct pm860x_battery_info, changed_work.work);
	int ret, data,soc_data;
	int tmp;
        printk("pm860x_changed_work\n");
	calc_soc(info,OCV_MODE_ACTIVE,&soc_data);
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (0/*ret & STATUS2_BAT*/) {
		ret = measure_current(info, &data);
		if (ret < 0)
			goto out;
		mutex_lock(&info->lock);
		info->present = 1;
		if (data > 0)
			info->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			info->status = POWER_SUPPLY_STATUS_DISCHARGING;
		mutex_unlock(&info->lock);
	} else {
	      #if 0
	         ret = measure_current(info, &data);
		if (ret < 0)
			goto out;
		#endif
		tmp = pm860x_reg_read(info->i2c, PM8607_CHG_CTRL1);
		ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
		if (ret < 0)
			goto out;
		mutex_lock(&info->lock);
		info->present = 1;
		printk("tmp=0x%x\n",tmp);
		if (tmp & 0x3)
		{
				printk("pm860x_changed_work:charging\n");
                        if(soc_data>=100)
                        {
                            info->status = POWER_SUPPLY_STATUS_FULL;
                        }
			else
			{
			    info->status = POWER_SUPPLY_STATUS_CHARGING;
			}
			//chg_full = 0;
                 }
		else
		{
			printk("pm860x_changed_work: discharging\n");

			if(ovchprotect == 1||chg_full==1||chg_full_en==1||full_excep == 1)
			{
			info->status = POWER_SUPPLY_STATUS_FULL;
			}
			else
			{
			info->status = POWER_SUPPLY_STATUS_DISCHARGING;
			
			}
		}
		#if 0
		if (data > 0)
			info->status = POWER_SUPPLY_STATUS_CHARGING;
		else
			info->status = POWER_SUPPLY_STATUS_DISCHARGING;
		#endif
		mutex_unlock(&info->lock);
		
	      #if 0
		mutex_lock(&info->lock);
		info->present = 0;
		info->status = POWER_SUPPLY_STATUS_UNKNOWN;
		mutex_unlock(&info->lock);
		#endif
	}
	power_supply_changed(&info->battery);

out:
	return;
}

static u16 pm8607_read_volt_meas_val(struct pm860x_battery_info *info,u8 measRegister)
{
	u16 meas_val;
	u8 reg_value[2];

	/* Read two registers, the alignment will be done as follows:
	 * Register 1 - bits 7:0 => 8 MSB bits <11:4> of measurement value
	 * Register 2 - bits 3:0 => 4 LSB bits <3:0> of measurement value
	 */
	if (pm860x_bulk_read(info->i2c, measRegister, 2, reg_value)
	    >= 0) {
		meas_val = ((reg_value[0] << 4) | (reg_value[1] & 0x0F));
	} else {
		return 0;
	}
	return meas_val;
}

#if 0
void read_boardid(u16 * vbat)
{
	u32 meas_val;
	meas_val = pm8607_read_volt_meas_val(ginfo,PM8607_GPADC2_MEAS1);
	/* voltage in mili volt */
        *vbat = (u16) (((u32) meas_val * 3 * 18 * 1000) >> 12) / 10;
}

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
#endif

/* Calculate start_soc */
static void pm860x_init_battery(struct pm860x_battery_info *info)
{
	int ret, data;

	/* measure enable on GPADC1 */
	data = MEAS1_GP1;
	if (info->temp_type == PM860X_TEMP_TINT)
		data |= MEAS1_TINT|MEAS1_GP2;
	ret = pm860x_set_bits(info->i2c, PM8607_MEAS_EN1, data, data);
	if (ret)
		goto out;

	/* measure enable on IBAT, BAT_DET, CC. IBAT is depend on CC. */
	data = MEAS3_IBAT | MEAS3_BAT_DET | MEAS3_CC;
	ret = pm860x_set_bits(info->i2c, PM8607_MEAS_EN3, data, data);
	if (ret)
		goto out;

	/* measure disable CC in sleep time  */
	ret = pm860x_reg_write(info->i2c, PM8607_MEAS_OFF_TIME1, 0x82);
	if (ret)
		goto out;
	ret = pm860x_reg_write(info->i2c, PM8607_MEAS_OFF_TIME2, 0x6c);
	if (ret)
		goto out;

	ret = pm860x_set_bits(info->i2c, PM8607_GPADC_MISC1, 1 << 0,
			      PM8607_GPADC_MISC1_GPFSM_EN);
	if (ret < 0)
		goto out;

	/* detect battery via GPADC1 */
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL6,
			      CC6_BAT_DET_GPADC1, CC6_BAT_DET_GPADC1);
	if (ret)
		goto out;

	ret = pm860x_set_bits(info->i2c, PM8607_CCNT, 7 << 3,
			      CCNT_AVG_SEL);
	if (ret < 0)
		goto out;

	/* set GPADC1 bias*/
	ret = pm860x_set_bits(info->i2c, PM8607_GP_BIAS2, 0xF << 4,
			      GP2_BIAS_SET_15UA);
	if (ret < 0)
		goto out;

    #if 0//marvell revert
	/* reset 0xD0~0xD7 in test page, avoid the four LSB of 0xD7 broken */
	ret = pm860x_page_reg_write(info->i2c, 0xE1, 0x0);
	if (ret < 0)
		goto out;
   #endif

	/* hw issue fix from sanremo C0 unexpected BAT interrupt when chip-sleep */
	ret = pm860x_set_bits(info->i2c, 0x3a, 0xF, 0x2);
	if (ret < 0)
		goto out;

	/* check whether battery present) */
	mutex_lock(&info->lock);
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if (ret < 0) {
		mutex_unlock(&info->lock);
		goto out;
	}
	if (ret & STATUS2_BAT) {
		info->present = 1;
		info->temp_type = PM860X_TEMP_TBAT;
	} else {
	        #if 0
		info->present = 0;
		info->temp_type = PM860X_TEMP_TINT;
		#endif
		info->present = 1;
		info->temp_type = PM860X_TEMP_TBAT;
	}
	mutex_unlock(&info->lock);

	/*set default battery calibration data*/
	sanremo_nvram_data.ibat_offset = 0;
	sanremo_nvram_data.vbat_offset = 0;
	sanremo_nvram_data.vbat_slope_high = 1000;
	sanremo_nvram_data.vbat_slope_low = 1000;

	calc_soc(info, OCV_MODE_ACTIVE, &info->start_soc);
	//board_id = pm860x_get_board();

out:
	return;
}

/*
 * register 1 bit[7:0] -- bit[11:4] of measured value of voltage
 * register 0 bit[3:0] -- bit[3:0] of measured value of voltage
 */
static int measure_12bit_voltage(struct pm860x_battery_info *info,
				 int offset, int *data)
{
	unsigned char buf[2];
	int ret;

	if (!data)
		return -EINVAL;

	ret = pm860x_bulk_read(info->i2c, offset, 2, buf);
	if (ret < 0)
		return ret;

	*data = ((buf[0] & 0xff) << 4) | (buf[1] & 0x0f);
	/* V_MEAS(mV) = data * 1.8 * 1000 / (2^12) */
	*data = ((*data & 0xfff) * 9 * 25) >> 9;
	return 0;
}

#if 0
static int measure_8bit_voltage(struct pm860x_battery_info *info,
				int offset, int *data)
{
	int ret;

	if (!data)
		return -EINVAL;

	ret = pm860x_reg_read(info->i2c, offset);
	if (ret < 0)
		return ret;

	/* V_MEAS(mV) = data * 1.8 * 1000 / (2^8) */
	*data = ((ret & 0xff) * 9 * 25) >> 5;
	return 0;
}
#endif

static int measure_vbatt(struct pm860x_battery_info *info, int state, int *data)
{
	unsigned char buf[5];
	int ret=0,vbat_base;

	switch (state) {
	case OCV_MODE_ACTIVE:
		ret = measure_12bit_voltage(info, PM8607_VBAT_MEAS1, data);
		if (ret)
			return ret;
		/* V_BATT_MEAS(mV) = value * 3 * 1.8 * 1000 / (2^12) */
		*data *= 3;
		vbat_base =  3700 - sanremo_nvram_data.vbat_offset;
		if(*data <= 3700)
			*data = (3700 * 1000  - (vbat_base - *data) * sanremo_nvram_data.vbat_slope_low)/1000;
		else
			*data = (3700 * 1000  + (*data - vbat_base) * sanremo_nvram_data.vbat_slope_high)/1000;

		break;
	case OCV_MODE_SLEEP:
		/*
		 * voltage value of VBATT in sleep mode is saved in different
		 * registers.
		 * bit[11:10] -- bit[7:6] of LDO9(0x18)
		 * bit[9:8] -- bit[7:6] of LDO8(0x17)
		 * bit[7:6] -- bit[7:6] of LDO7(0x16)
		 * bit[5:4] -- bit[7:6] of LDO6(0x15)
		 * bit[3:0] -- bit[7:4] of LDO5(0x14)
		 */
		ret = pm860x_bulk_read(info->i2c, PM8607_LDO5, 5, buf);
		if (ret < 0)
			return ret;
		ret = ((buf[4] >> 6) << 10) | ((buf[3] >> 6) << 8)
			| ((buf[2] >> 6) << 6) | ((buf[1] >> 6) << 4)
			| (buf[0] >> 4);
		/* V_BATT_MEAS(mV) = data * 3 * 1.8 * 1000 / (2^12) */
		*data = (u16) (((u32) ret * 3 * 18 * 1000) >> 12) / 10;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


void read_vbat(struct pm860x_battery_info *info,int * vbat)
{
	u32 meas_val;
	int ret;
	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);  
	meas_val = pm8607_read_volt_meas_val(info,PM8607_VBAT_MEAS1);
	/* voltage in mili volt */
	    if(ret & 0x04)
	 *vbat =(int) (((u32) meas_val * para_on * 18) >> 12) / 10; 
	    else
	*vbat =(int) (((u32) meas_val * para_off * 18) >> 12) / 10; // (int) (((u32) meas_val * 3 * 18 * 1000) >> 12) / 10;

}
static int measure_temp(struct pm860x_battery_info *info, int *data)
{
	int ret, i, temp;

	if (!data)
		return -EINVAL;
	if (info->temp_type == PM860X_TEMP_TINT) {
		ret = measure_12bit_voltage(info, PM8607_TINT_MEAS1, data);
		if (ret)
			return ret;
		*data = (*data - 884) * 1000 / 3611;
	} else {
		ret = measure_12bit_voltage(info, PM8607_GPADC1_MEAS1, data);
		if (ret)
			return ret;
		for (i = 0; i < ARRAY_SIZE(array_batt_temp); i++) {
			if (*data < array_batt_temp[i][0]) {
				if (i == 0)
					temp = array_batt_temp[0][1];
				else
					temp = (array_batt_temp[i - 1][1]
						+ array_batt_temp[i][1]) >> 1;
				break;
			}
		}
		if (i >= ARRAY_SIZE(array_batt_temp))
			temp = array_batt_temp[i - 1][1];
		*data = temp;
	}
	return 0;
}

/*
 * Return value is signed data.
 * Negative value means discharging, and positive value means charging.
 */
static int measure_current(struct pm860x_battery_info *info, int *data)
{
	unsigned char buf[2];
	short s;
	int ret;

	if (!data)
		return -EINVAL;

	ret = pm860x_bulk_read(info->i2c, PM8607_IBAT_MEAS1, 2, buf);
	if (ret < 0)
		return ret;

	s = ((buf[0] & 0xff) << 8) | (buf[1] & 0xff);
	/* current(mA) = value * 0.125 */
	*data = s >> 3;
	*data += sanremo_nvram_data.ibat_offset;
	return 0;
}

static int set_charger_current(struct pm860x_battery_info *info, int data,
			       int *old)
{
	int ret;

	if (data < 50 || data > 1600 || !old)
		return -EINVAL;

	data = ((data - 50) / 50) & 0x1f;
	*old = pm860x_reg_read(info->i2c, PM8607_CHG_CTRL2);
	*old = (*old & 0x1f) * 50 + 50;
	ret = pm860x_set_bits(info->i2c, PM8607_CHG_CTRL2, 0x1f, data);
	if (ret < 0)
		return ret;
	return 0;
}

static int calc_resistor(struct pm860x_battery_info *info)
{
	int ret, i, data;
	int vbatt_sum1, vbatt_sum2, chg_current;
	int ibatt_sum1, ibatt_sum2;
	
	ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data);
	if (ret)
		goto out;
	if (data < VBATT_RESISTOR_MIN || data > VBATT_RESISTOR_MAX)
		goto out;

	/* current is saved */
	if (set_charger_current(info, 500, &chg_current))
		goto out;
	msleep(1000);

	for (i = 0, vbatt_sum1 = 0, ibatt_sum1 = 0; i < 10; i++) {
		ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data);
		if (ret)
			goto out_meas;
		vbatt_sum1 += data;
		ret = measure_current(info, &data);
		if (ret)
			goto out_meas;

		if (data < 0)
			ibatt_sum1 = ibatt_sum1 - data;	/* discharging */
		else
			ibatt_sum1 = ibatt_sum1 + data;	/* charging */
	}

	if (set_charger_current(info, 100, &ret))
		goto out_meas;
	msleep(1000);

	for (i = 0, vbatt_sum2 = 0, ibatt_sum2 = 0; i < 10; i++) {
		ret = measure_vbatt(info, OCV_MODE_ACTIVE, &data);
		if (ret)
			goto out_meas;
		vbatt_sum2 += data;
		ret = measure_current(info, &data);
		if (ret)
			goto out_meas;

		if (data < 0)
			ibatt_sum2 = ibatt_sum2 - data;	/* discharging */
		else
			ibatt_sum2 = ibatt_sum2 + data;	/* charging */
	}

	/* restore current setting */
	if (set_charger_current(info, chg_current, &ret))
		goto out_meas;

	if ((vbatt_sum1 > vbatt_sum2) && (ibatt_sum1 > ibatt_sum2)
		&& (ibatt_sum2 > 0)) {
		/* calculate resistor in discharging case */
		data = 1000 * (vbatt_sum1 - vbatt_sum2)
			/ (ibatt_sum1 - ibatt_sum2);
		if ((data - info->resistor > 0)
			&& (data - info->resistor < info->resistor))
			info->resistor = data;
		if ((info->resistor - data > 0)
			&& (info->resistor - data < data))
			info->resistor = data;
	}
	return 0;

out_meas:
	set_charger_current(info, chg_current, &ret);
out:
	return -EINVAL;
}

int pm860x_calc_resistor(void)
{
	return calc_resistor(ginfo);
}
EXPORT_SYMBOL(pm860x_calc_resistor);

static int read_ccnt(struct pm860x_battery_info *info, int offset,
		     int *ccnt)
{
	unsigned char buf[2];
	int ret;

	if (!ccnt)
		return -EINVAL;

	ret = pm860x_set_bits(info->i2c, PM8607_CCNT, 7, offset & 7);
	if (ret < 0)
		goto out;
	ret = pm860x_bulk_read(info->i2c, PM8607_CCNT_MEAS1, 2, buf);
	if (ret < 0)
		goto out;
	*ccnt = ((buf[0] & 0xff) << 8) | (buf[1] & 0xff);
	return 0;
out:
	return ret;
}

static int calc_ccnt(struct pm860x_battery_info *info, struct ccnt *ccnt)
{
	unsigned int sum;
	int ret, data;

	ret = read_ccnt(info, CCNT_POS1, &data);
	if (ret)
		goto out;
	sum = data & 0xffff;
	ret = read_ccnt(info, CCNT_POS2, &data);
	if (ret)
		goto out;
	sum |= (data & 0xffff) << 16;
	ccnt->pos += sum;

	ret = read_ccnt(info, CCNT_NEG1, &data);
	if (ret)
		goto out;
	sum = data & 0xffff;
	ret = read_ccnt(info, CCNT_NEG2, &data);
	if (ret)
		goto out;
	sum |= (data & 0xffff) << 16;
	sum = ~sum + 1;		/* since it's negative */
	ccnt->neg += sum;

	ret = read_ccnt(info, CCNT_SPOS, &data);
	if (ret)
		goto out;
	ccnt->spos += data;
	ret = read_ccnt(info, CCNT_SNEG, &data);
	if (ret)
		goto out;

	/*
	 * charge(mAh)  = count * 1.6984 * 1e(-8)
	 *		= count * 16984 * 1.024 * 1.024 * 1.024 / (2 ^ 40)
	 *		= count * 18236 / (2 ^ 40)
	 */
	ccnt->total_chg = (int)((ccnt->pos * 18236) >> 40);
	ccnt->total_dischg = (int)((ccnt->neg * 18236) >> 40);
	return 0;
out:
	return ret;
}

static int clear_ccnt(struct pm860x_battery_info *info, struct ccnt *ccnt)
{
	int data;

	memset(ccnt, 0, sizeof(struct ccnt));
	/* read to clear ccnt */
	read_ccnt(info, CCNT_POS1, &data);
	read_ccnt(info, CCNT_POS2, &data);
	read_ccnt(info, CCNT_NEG1, &data);
	read_ccnt(info, CCNT_NEG2, &data);
	read_ccnt(info, CCNT_SPOS, &data);
	read_ccnt(info, CCNT_SNEG, &data);
	return 0;
}

/* Calculate Open Circuit Voltage */
static int calc_ocv(struct pm860x_battery_info *info, int *ocv)
{
	int ret, i, data;
	int vbatt_avg, vbatt_sum, ibatt_avg, ibatt_sum;
	int volt_sum=0;
	//volt_index = (volt_index+1)%loop;

	if (!ocv)
		return -EINVAL;

	for (i = 0, ibatt_sum = 0, vbatt_sum = 0; i < 10; i++) {
		read_vbat(info,&data);
		vbatt_sum += data;
		ret = measure_current(info, &data);
		if (ret)
		{
		//  printk("calc_ocv:ret=%d\n",ret);
			goto out;
		}
		ibatt_sum += data;
	}
	vbatt_avg = vbatt_sum / 10;
	ibatt_avg = ibatt_sum / 10;

	#if 0   
	volt_loop[volt_index] =vbatt_avg;
	
	for (i=0; i<loop; i++)
       {
		if(volt_loop[i] <= 0)
			break;
		volt_sum += volt_loop[i];
	}
	if(i!=0)
	*ocv = volt_sum/i;
	else
	*ocv = vbatt_avg;
       #endif
	*ocv = vbatt_avg;
	#if 0
	mutex_lock(&info->lock);
	if (info->present)
		*ocv = vbatt_avg - ibatt_avg * info->resistor / 1000;
	else
		*ocv = vbatt_avg;
	mutex_unlock(&info->lock);
	#endif
	dev_dbg(info->dev, "VBAT average:%d, OCV:%d\n", vbatt_avg, *ocv);
	return 0;
out:
	return ret;
}

int volt_timer_count=0;
static int  tmp_for_low_volt=0;
extern void display_active_wakelock(void);

void battery_volt_read_work(void)  
{
	int volt_sum = 0;
	int volt = 0;
	int i = 0;
	int interval = HZ * 15;
	int tmp_volt=0;
	int ret;
	int num=0;
      struct pm860x_battery_info *info = ginfo;
      calc_ocv(info, &volt);
	tmp_for_low_volt=volt;
     
	static int log_count = 0;
	volt_index = (volt_index+1)%loop;
      //ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
      if( 0 )
      	{
		if(0 == volt_pre)
		{
		volt_loop[volt_index] = volt;
		}
		else if(volt > volt_pre)
		{
		volt_loop[volt_index] = volt_pre+1;
		}
		else if(volt < volt_pre)
		{
		volt_loop[volt_index] = volt_pre-1;
		}
		else
		{
		volt_loop[volt_index] = volt_pre;
		}
		volt_pre = volt_loop[volt_index];
      	}
	else
	{
	
		  if(volt_timer_count<10)
		  {
		  	volt_loop[volt_index] =volt;
			volt_timer_count+=1;
//		         printk("volt_timer_count=%d\n",volt_timer_count);
			if(volt_timer_count==6)//power on and voltage stable
			{
			   	for (i=0; i<loop; i++)
                                {
					if(volt_loop[i] <= 0)
						break;
					volt_loop[i]=volt;

	                         }
//				printk("volt is stable and i=%d\n",i);
			}
		  }
		 else  
		  {
			    // if(volt_index>=1)
			     	
					if(volt>=volt_value/*volt_loop[volt_index-1]*/) 
					{
//					   printk("voltage is high than ave\n");
					    volt_loop[volt_index]=volt_value+(volt-volt_value)/3;//volt_loop[volt_index-1];
					}
					else
					{
					    volt_loop[volt_index] =volt_value-(volt_value-volt)/3;
					}
			     	
				
		  }
	}


	for (i=0; i<loop; i++)
       {
		if(volt_loop[i] > 0)
		{
		volt_sum += volt_loop[i];
		num++;
		}
	}
#if 0
	printk("volt0 %d mv,volt1 %d mv,volt2 %d mv,volt3 %d mv,volt4 %d mv,volt5 %d mv, loop %d\n",

		volt_loop[0],volt_loop[1],
		volt_loop[2],volt_loop[3],
		volt_loop[4],volt_loop[5],
		volt_index);

#endif
      if ((num!= 0)&&(volt>3500))
	volt_value = volt_sum/num;
	else
	volt_value = volt;


	if((log_count++)==5)
	{
	  	printk("tmp volt=%d\n",volt);
	  printk("aval volt=%d\n",volt_value);
		log_count = 0;
		display_active_wakelock();
	}


	//printk("====battery_volt_now: %d mV====\n",volt_value);
      // if(charger_is_online()==1)
         {
		if(volt_value < 3430)
		{
		interval = HZ * 5;
		}
         }
	schedule_delayed_work(&monitor_batteryvolt_work, interval);
}
//zhnghua_linear process end

/* Calculate State of Charge (percent points) */
static int calc_soc(struct pm860x_battery_info *info, int state, int *soc)
{
	int i, ocv, count, ret = -EINVAL;

	if (!soc)
		return -EINVAL;

	switch (state) {
	case OCV_MODE_ACTIVE:
		ret = calc_ocv(info, &ocv);
		ocv = volt_value;
		//printk("calc_soc:ret=%d\n",ret);
		break;
	case OCV_MODE_SLEEP:
		ret = measure_vbatt(info, OCV_MODE_SLEEP, &ocv);
		break;
	}
	if (ret)
		goto out;
	
		//printk("ovchprotect:chg_full=%d:%d:%d\n",ovchprotect,chg_full,chg_full_en);
		if (ovchprotect == 1||chg_full==1||chg_full_en==1||full_excep == 1)
		{
		  *soc=100;
		  return 0;
		}
		
	flag = 1;
	count = ARRAY_SIZE(array_soc);
        	for (i = 0; i < count-1; i++) {
		if (ocv >= array_soc[i][0]) {
			*soc = array_soc[i][1];
			break;
                } else if ((ocv < array_soc[i][0]) && ((ocv >= array_soc[i + 1][0]))) {
                                *soc = array_soc[i + 1][1];
                                break;
                } else if (ocv < array_soc[count-1][0]) {
                        *soc = 0;
                        break;
		}
	}

	if(pm860x_reg_read(info->i2c, PM8607_CHG_CTRL1)&(0x03))
	{
	
		if(ocv>4140)
			*soc = 100;
			char_on = 1;
		soc_chg_on = *soc;
		ncount = 0;
		
	}
	else
	{

		if(char_on == 1)
		{
			
			if((ncount++)>3)
			char_on = 0;

			
			*soc = soc_chg_on;
		}
		

		if(ocv>4120&&char_on == 0)
			*soc = 100;


	}
	
	if((*soc ==100)&&(charger_flag ==1))
	{
		charger_flag = 0;
		power_supply_changed(&g_chinfo->usb);
		
	}
      //  printk(KERN_INFO"ocv:%d, soc:%d\n",ocv,*soc);
	return 0;
out:
	return ret;
}

static int calc_capacity(struct pm860x_battery_info *info, int *cap)
{
	int ret, data;

	ret = calc_ccnt(info, &ccnt_data);
	if (ret)
		goto out;

	data = info->max_capacity * info->start_soc / 100;
	if (ccnt_data.total_dischg - ccnt_data.total_chg < data)
		*cap = data + ccnt_data.total_chg - ccnt_data.total_dischg;
	else
		*cap = data;
	*cap = *cap * 100 / info->max_capacity;

	return 0;
out:
	return ret;
}

static void pm860x_external_power_changed(struct power_supply *psy)
{
	struct pm860x_battery_info *info;

	info = container_of(psy, struct pm860x_battery_info, battery);
	queue_delayed_work(info->chip->monitor_wqueue,
			   &info->changed_work, HZ / 2);
	return;
}

static int pm860x_batt_get_prop(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct pm860x_battery_info *info = dev_get_drvdata(psy->dev->parent);
	int data, ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		mutex_lock(&info->lock);
		val->intval = info->status;
		mutex_unlock(&info->lock);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		mutex_lock(&info->lock);
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		mutex_unlock(&info->lock);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		mutex_lock(&info->lock);
		val->intval = info->present;
		mutex_unlock(&info->lock);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = calc_soc(info,OCV_MODE_ACTIVE,&data);
		if (ret)
			goto out;
		if(data < 0)
			data=0;
		else if(data > 100)
			data = 100;
		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* return Open Circuit Voltage (not measured voltage) */
		ret = calc_ocv(info, &data);
		if (ret)
			goto out;
		data=volt_value;

		data*=1000;
		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = measure_current(info, &data);
		if (ret)
			goto out;
		val->intval = data;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = measure_temp(info, &data);
		if (ret)
			goto out;
		data=100;
		val->intval = data;
		break;
	default:
		return -ENODEV;
	}
	return 0;
out:
	return ret;
}

static enum power_supply_property pm860x_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

/*RF has leak current when battery calibration*/
int disable_rf(void)
{
	__raw_writel(0xb0c0, 0xfe01e298); /*disable RF*/
	__raw_writel(0xb0c0, 0xfe01e29c);
	__raw_writel(0xb0c0, 0xfe01e2a0);
	__raw_writel(0xb0c0, 0xfe01e2a4);
	__raw_writel(0xb881, 0xfe01e2a8);
	__raw_writel(0xb881, 0xfe01e2ac);
	__raw_writel(0xb881, 0xfe01e2b0);
	__raw_writel(0xa880, 0xfe01e304);
	__raw_writel(0x1085, 0xfe01e308);
	__raw_writel(0x1085, 0xfe01e30c);
	__raw_writel(0x1085, 0xfe01e310);
	__raw_writel(0x1085, 0xfe01e314);
	__raw_writel(0x1085, 0xfe01e318);
	__raw_writel(0x1085, 0xfe01e31c);
	return 0;
}
int ibat_offset_d4_trim1(void)
{
	int vbat;
	u8  temp;
	struct pm860x_battery_info *info = ginfo;

	/*restore default battery calibration paras before calibration*/
	sanremo_nvram_data.ibat_offset = 0;
	sanremo_nvram_data.vbat_offset = 0;
	sanremo_nvram_data.vbat_slope_high = 1000;
	sanremo_nvram_data.vbat_slope_low = 1000;

	disable_rf();
	msleep(1);
	temp_reg_0x4C = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
	/*Force system to be supplied from charger*/
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, (temp_reg_0x4C & 0xF0) | 0x8);
	temp_reg_0x48 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
	/*Charge off*/
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1,( temp_reg_0x48 & 0xFC));
	measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat);
	if((4100 > vbat) && (vbat > 3700))
	{
		temp =(14 - ((vbat - 3200)/100)) << 4;
		/*Set Portofino preregulator output.*/
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,temp | 0x0E);
	}
	return 0;
}

int ibat_offset_d4_trim2(int i_offset)
{
	int ibat;
	u8 temp_d4, ibat_trim=0, value;
	struct pm860x_battery_info *info = ginfo;

	while(1)
	{
		measure_current(ginfo, &ibat);
		temp_d4 = pm860x_page_reg_read(info->i2c,0xD4);
		printk("init  d4  = 0x%x ,ibat = %d\n", temp_d4,ibat);
		msleep(100);
		if(i_offset == ibat)
			return ibat_offset_remain = 0;
		else if(i_offset > ibat){
			ibat_trim = (i_offset - ibat)>>1;
			value = temp_d4 + ibat_trim;
			pm860x_page_reg_write(info->i2c, 0xD4 , value);
			msleep(100);
		}
		else{
			ibat_trim = (ibat - i_offset)>>1;
			value = temp_d4 - ibat_trim;
			pm860x_page_reg_write(info->i2c, 0xD4 , value);
			msleep(100);
		}
		measure_current(ginfo, &ibat);
		printk("sys read ibat2 = %d\n",ibat);
		if(ibat < 2 && ibat > -2){
			ibat_offset_remain = i_offset - ibat ;
			printk("ibat_offset_remain = %d \n", ibat_offset_remain);
			printk("trim value d4  = 0x%x \n", value);
			return ibat_offset_remain;	//D4 trim value
		}
	}
}

int ibat_gain_d5d7_trim1(void)
{
	int ibat1,ibat2;
	int ibat_delta;
	int vbat,ret;
	struct pm860x_battery_info *info = ginfo;

	/* disable chg done interrupt */
	pm860x_set_bits(info->i2c, 0x08, 1<<6,0<<6);

	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if(/*ret & STATUS2_BAT && */ret & STATUS2_CHG)
	{
		measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat);
		printk("vbat = %d \n", vbat);
		if((4100 > vbat) && (vbat > 3700))
		{
			while(1)
			{
				/*Set Portofino preregulator output to 4.5V.*/
				pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,0x0E);
				temp_reg_0x48 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
				pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_reg_0x48|0x2);
				temp_reg_0x49 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL2);
				/*Set fast charge current to 300mA*/
				pm860x_reg_write(info->i2c,PM8607_CHG_CTRL2, 0x5);

				msleep(1000);
				while(1)
				{
					msleep(100);/*Wait 100mS*/
					measure_current(ginfo, &ibat1);
					msleep(100);/*Wait 100mS*/
					measure_current(ginfo, &ibat2);

					if(ibat1 == ibat2)
						return ibat_offset_remain;
					else if(ibat1 < ibat2)
						ibat_delta = ibat2 - ibat1;
					else
						ibat_delta = ibat1 - ibat2;
					if(ibat_delta <= 3)
						return ibat_offset_remain;
				}
			}
		}
	}
	return ibat_offset_remain;
}

int pm860x_set_vibrator(unsigned char value)
{    
      struct pm860x_battery_info *info = ginfo;

    //  printk("set_vibrator----------------=0x%x\n",value);
        if(value == 0) {                
        pm860x_reg_write(info->chip->companion,PM8606_VIBRATORA, 0x00);//disable LDO,1.22V default,          
        pm860x_reg_write(info->i2c,PM8607_VIBRATOR_PWM, 0x00);//0%       
        
        } 
      else {            
        pm860x_reg_write(info->chip->companion,PM8606_VIBRATORA, 0x01|((value&0x7)<<1));//enable LDO 
        pm860x_reg_write(info->i2c,PM8607_VIBRATOR_PWM, 0xFF);//62%      
      
        }       
          return 0;
}
EXPORT_SYMBOL(pm860x_set_vibrator);

int ibat_gain_d5d7_trim2(u16 i_trim)
{
	u8  temp;
	u16 ibat_gain;
	u16 temp_i=0;
	u8 gain2;
	int vbat,ibat=0,ret;
	struct pm860x_battery_info *info = ginfo;

	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_2);
	if(/*ret & STATUS2_BAT &&*/ ret & STATUS2_CHG)
	{
		measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat);
		printk("%s:vbat=%d\n",__func__,vbat);
		if((4100 > vbat) && (vbat > 3700))
		{
			msleep(100);
			measure_current(ginfo, &ibat);
			temp = pm860x_page_reg_read(info->i2c,0xD7);
			ibat_gain = (u16)temp;
			ibat_gain &=0xF0;
			gain2 = pm860x_page_reg_read(info->i2c,0xD5);
			ibat_gain = (ibat_gain << 4) |gain2;

			if(ibat_offset_remain < 0)
				ibat = ibat + (ibat_offset_remain & 0xFFFF);
			else
				ibat = ibat - (ibat_offset_remain & 0xFFFF);
			if(ibat <= i_trim){
				temp_i = 256*(i_trim - ibat)/34;

				if(ibat_gain <= temp_i){
					temp_i = temp_i - ibat_gain;
					ibat_gain = 0xFFF - temp_i;
				}
				else
					ibat_gain = ibat_gain + temp_i;
			}
			else{
				temp_i = 256*(ibat - i_trim)/34;
				ibat_gain = ibat_gain - temp_i;
			}

			temp = ibat_gain & 0xFF;
			pm860x_page_reg_write(info->i2c, 0xD5 , temp);
			temp = pm860x_page_reg_read(info->i2c,0xD7);
			temp = (temp & 0xF)  ;
			temp |= ((ibat_gain & 0xF00) >>4);
			pm860x_page_reg_write(info->i2c, 0xD7 , temp);
			msleep(100);
			temp= pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS1);
			temp_i = temp << 8;
			temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS2);
			temp_i = temp_i | temp;
			if(temp_i <= (i_trim<<3)){
				while(temp_i <= (i_trim>>3)){
					ibat_gain = ibat_gain +1;
					temp = ibat_gain & 0xFF;
					pm860x_page_reg_write(info->i2c, 0xD5 , temp);
					temp = (ibat_gain & 0xF00) >>4;
					temp = pm860x_page_reg_read(info->i2c,0xD7);
					temp = (temp & 0xF);
					temp |= ((ibat_gain & 0xF00) >>4);
					pm860x_page_reg_write(info->i2c, 0xD7 , temp);
					msleep(100);
					temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS1);
					temp_i = temp << 8;
					temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS2);
					temp_i = temp_i | temp;
				}
			}
			else{
				while(temp_i > (i_trim<<3)){
					if(ibat_gain >= 1)
						ibat_gain = ibat_gain -1;
					else
						ibat_gain = 0xFFF;

					temp = ibat_gain & 0xFF;

					pm860x_page_reg_write(info->i2c, 0xD5 , temp);
					temp = pm860x_page_reg_read(info->i2c,0xD7);
					temp = (temp & 0xF)  ;
					temp |= ((ibat_gain & 0xF00) >>4);
					pm860x_page_reg_write(info->i2c, 0xD7 , temp);
					msleep(100);
					temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS1);
					temp_i = temp << 8;
					temp = pm860x_reg_read(info->i2c,PM8607_IBAT_MEAS2);
					temp_i = temp_i | temp;
				}
			}
			pm860x_reg_write(info->i2c,PM8607_CHG_CTRL2, temp_reg_0x49);//Restore ox49 register

			measure_current(ginfo, &ibat);
			printk("final ibat  = %d \n", ibat);
			temp = pm860x_page_reg_read(info->i2c,0xD7);
			printk("final d7 = 0x%x \n",temp);
			temp = pm860x_page_reg_read(info->i2c,0xD5);
			printk("final d5 = 0x%x \n",temp);
			printk("Ibat_gain = %d \n", ibat_gain);
			printk("Ibat_gain= %d\n", ibat_gain);
			/* enable chg done interrupt */
			pm860x_set_bits(info->i2c, 0x08, 1<<6,1<<6);
			return 1;
		} else{
			pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_reg_0x48);//Restore ox48 register
			pm860x_reg_write(info->i2c,PM8607_CHG_CTRL2, temp_reg_0x49);//Restore ox4C register
			printk("Currently can not do Ibat trim!\n");
		}
	}
	else
		printk("Unknow PMIC!Can't do Ibat trim!\n");
		/* enable chg done interrupt */
		pm860x_set_bits(info->i2c, 0x08, 1<<6,1<<6);

	return 1;
}

static ssize_t battery_cali_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u8 val;
	val = ibat_gain_d5d7_trim1();
	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t battery_cali_store_attrs(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	u16 input;
	u16 value;
	char *last = NULL;

	input = simple_strtoul(buf, &last, 0);
	value = ibat_gain_d5d7_trim2(input);
	printk("input value = %d\n",input);
	return count;
}

static ssize_t battery_info_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u8 val=0;
	printk("%s \n",__func__);
	ibat_offset_d4_trim1();
	return snprintf(buf, PAGE_SIZE, "%d\n", val);

}
static ssize_t battery_info_store_attrs(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	int input;
	char *last = NULL;

	input = simple_strtoul(buf, &last, 0);
	printk("%s, input=%d \n",__func__,input);
	ibat_offset_d4_trim2(input);

	return count;
}

static ssize_t battery_trimreturn_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u8 temp1,temp2,temp3;
	u32 value;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	printk("%s \n",__func__);

	temp1 = pm860x_page_reg_read(info->i2c,0xD4);
	temp2 = pm860x_page_reg_read(info->i2c,0xD5);
	temp3 = pm860x_page_reg_read(info->i2c,0xD7);
	value = temp1 + (temp2<<8) + (temp3<<16);
	printk("d4=0x%x, d5=0x%x,d7=0x%x \n", temp1,temp2,temp3);
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}
static ssize_t battery_trimreturn_store_attrs(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	return count;
}
static ssize_t battery_vol_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u16 value;
	u8 i;
	int vbat1 = 0;
	int vbat2 = 0;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	for (i = 0; i < 10; i++) {
		measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat2);
		vbat1 += vbat2;
	}
	value = vbat1/10;
	printk("%s,value = %d \n",__func__,value);
	if( 3195 < vol_input && vol_input < 3205){
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, temp_4c_1);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48_1);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,temp1);
	}
	if( 3695 < vol_input && vol_input < 3705){
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, temp_4c_2);
        pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48_2);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,temp2);
        }
	if( 4195  < vol_input && vol_input < 4205){
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, temp_4c_3);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48_2);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,temp3);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}
static ssize_t trim_ibat_remain_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ibat_offset_remain);
}

static ssize_t battery_vol_store_attrs(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	char *last = NULL;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	vol_input = simple_strtoul(buf, &last, 0);
	printk("%s,vol input = %d \n", __func__,vol_input);
	disable_rf();
	if( 3195 < vol_input && vol_input < 3205){
		temp_4c_1 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5,(temp_4c_1 &0xF0)|0x8);//force supply from charger
		temp_48_1 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48_1&0xFC);
		temp1 = pm860x_reg_read(info->chip->companion, PM8606_PREREGULATORA);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,(0xF0)|0x0E);
	}
	if(3695< vol_input && vol_input < 3705){
		temp_4c_2 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
        pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5,(temp_4c_2 &0xF0)|0x8);
		temp_48_2 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
        pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48_2&0xFC);
		temp2 = pm860x_reg_read(info->chip->companion, PM8606_PREREGULATORA);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,(0xA0)|0x0E);

	}
	if( 4195 < vol_input && vol_input < 4205){
		temp_4c_3 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
		pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, (temp_4c_3&0xF0)|0x8);
		temp_48_3 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
        pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48_3&0xFC);
		temp3 = pm860x_reg_read(info->chip->companion, PM8606_PREREGULATORA);
		pm860x_reg_write(info->chip->companion, PM8606_PREREGULATORA,(0x50)|0x0E);
	}
	return count;
}

static ssize_t calibration_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	int len=0;
	len = sprintf(buf, "Battery Max Capacity: %d(mAh)\n", info->max_capacity);
	len += sprintf(buf + len, "Battery Internal Resistor: %d(omh)\n",
			info->resistor);
	len += sprintf(buf + len, "d4:0x%x, d5:0x%x, d7:0x%x\n",
		sanremo_nvram_data.d4value,sanremo_nvram_data.d5value,
		sanremo_nvram_data.d7value);
	len += sprintf(buf + len, "ibat_offset: %d, vbat_offset: %d\n",
		sanremo_nvram_data.ibat_offset,sanremo_nvram_data.vbat_offset);
	len += sprintf(buf + len, "vbat_slope_low %d, vbat_slope_high :%d\n",
		sanremo_nvram_data.vbat_slope_low,sanremo_nvram_data.vbat_slope_high);
	return len;
}

static ssize_t calibration_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	char *s, *sub, flag = 0;
	unsigned long data = 0;
	unsigned long d7_trim = 0,d7_temp = 0 ;
	long value;

	for (s = (char *)buf; s;) {
		sub = strsep(&s, " \t");
		if (!sub)
			break;
		if (flag == 'c') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong max capacity is "
					"assigned!\n");
			mutex_lock(&info->lock);
			info->max_capacity = data;
			mutex_unlock(&info->lock);
			break;
		}
		if (flag == 'r') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong resistor is assigned!\n");
			mutex_lock(&info->lock);
			info->resistor = data;
			mutex_unlock(&info->lock);
			break;
		}
		if (flag == '1') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong d4value is assigned!\n");
			sanremo_nvram_data.d4value = data;
			break;
		}
		if (flag == '2') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong d5value is assigned!\n");
			sanremo_nvram_data.d5value = data;
			break;
		}
		if (flag == '3') {
			flag = 0;
			if (strict_strtoul(sub, 10, &d7_trim))
				dev_warn(dev, "Wrong d7value is assigned!\n");
				d7_temp = pm860x_page_reg_read(info->i2c,0xD7);
			d7_temp = d7_temp&0x0F;
			sanremo_nvram_data.d7value = (d7_trim&0xf0)|d7_temp; //D7 lsb 4bit don't change
			pm860x_page_reg_write(info->i2c, 0xD4 , sanremo_nvram_data.d4value);
			pm860x_page_reg_write(info->i2c, 0xD5 , sanremo_nvram_data.d5value);
			pm860x_page_reg_write(info->i2c, 0xD7 , sanremo_nvram_data.d7value);
			break;
		}
		if (flag == '4') {
			flag = 0;
			if (strict_strtol(sub, 10, &value))
				dev_warn(dev, "Wrong ibat_offset is assigned!\n");
			sanremo_nvram_data.ibat_offset = value;
			break;
		}
		if (flag == '5') {
			flag = 0;
			if (strict_strtol(sub, 10, &value))
				dev_warn(dev, "Wrong vbat_offsetis assigned!\n");
			sanremo_nvram_data.vbat_offset = value;
			break;
		}
		if (flag == '6') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong vbat_slope_low is assigned!\n");
			sanremo_nvram_data.vbat_slope_low = data;
			break;
		}
		if (flag == '7') {
			flag = 0;
			if (strict_strtoul(sub, 10, &data))
				dev_warn(dev, "Wrong vbat_slope_high is assigned!\n");
			sanremo_nvram_data.vbat_slope_high = data;
			break;
		}
		if (!strcasecmp(sub, "d4value")) {
			flag = '1';
			continue;
		}
		if (!strcasecmp(sub, "d5value")) {
			flag = '2';
			continue;
		}
		if (!strcasecmp(sub, "d7value")) {
			flag = '3';
			continue;
		}
		if (!strcasecmp(sub, "ibat_offset")) {
			flag = '4';
			continue;
		}
		if (!strcasecmp(sub, "vbat_offset")) {
			flag = '5';
			continue;
		}
		if (!strcasecmp(sub, "vbat_slope_low")) {
			flag = '6';
			continue;
		}
		if (!strcasecmp(sub, "vbat_slope_high")) {
			flag = '7';
			continue;
		}
	}
	return count;
}

static ssize_t calibartion_vbat_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	u16 value;
	u8 i;
	int vbat1 = 0;
	int vbat2 = 0;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);

	temp_4c_1 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL5);
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5,(temp_4c_1 &0xF0)|0x8);//force supply from charger
	temp_48_1 = pm860x_reg_read(info->i2c,PM8607_CHG_CTRL1);
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48_1&0xFC);
	/*for read vbat when power supplied from charger*/
	msleep(500);
	for (i = 0; i < 10; i++) {
		measure_vbatt(ginfo, OCV_MODE_ACTIVE, &vbat2);
		vbat1 += vbat2;
	}
	value = vbat1/10;

	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL5, temp_4c_1);
	pm860x_reg_write(info->i2c,PM8607_CHG_CTRL1, temp_48_1);

	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static ssize_t calibartion_ibat_show_attrs(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	int value;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	measure_current(info, &value);
	return snprintf(buf, PAGE_SIZE, "%d\n", value);
}

static DEVICE_ATTR(trim_d5d7, S_IRUSR|S_IWUSR, battery_cali_show_attrs, battery_cali_store_attrs);
static DEVICE_ATTR(trim_d4, S_IRUSR|S_IWUSR, battery_info_show_attrs, battery_info_store_attrs);
static DEVICE_ATTR(trim_vol, S_IRUSR|S_IWUSR, battery_vol_show_attrs, battery_vol_store_attrs);
static DEVICE_ATTR(trim_return, S_IRUSR|S_IWUSR, battery_trimreturn_show_attrs, battery_trimreturn_store_attrs);
static DEVICE_ATTR(trim_ibat_remain, S_IRUSR, trim_ibat_remain_show_attrs, NULL);
static DEVICE_ATTR(calibration, S_IRUSR|S_IWUSR, calibration_show, calibration_store);
static DEVICE_ATTR(calibration_vbat, S_IRUSR, calibartion_vbat_show_attrs, NULL);
static DEVICE_ATTR(calibration_ibat, S_IRUSR, calibartion_ibat_show_attrs, NULL);



static struct attribute *battery_attributes[] = {
	&dev_attr_trim_d5d7.attr,
	&dev_attr_trim_d4.attr,
	&dev_attr_trim_vol.attr,
	&dev_attr_trim_return.attr,
	&dev_attr_trim_ibat_remain.attr,
	&dev_attr_calibration.attr,
	&dev_attr_calibration_vbat.attr,
	&dev_attr_calibration_ibat.attr,
	NULL,
};
static struct attribute_group battery_attr_group = {
	.attrs = battery_attributes,
};

#if 0
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

void read_boardid(u16 * vbat)
{
	u32 meas_val;
	meas_val = pm8607_read_volt_meas_val(PM8607_GPADC2_MEAS1);
	/* voltage in mili volt */
        *vbat = (u16) (((u32) meas_val * 3 * 18 * 1000) >> 12) / 10;
}
#endif



static __devinit int pm860x_battery_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_battery_info *info;
	int ret;

	info = kzalloc(sizeof(struct pm860x_battery_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->irq_cc = platform_get_irq(pdev, 0);
	if (info->irq_cc < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		ret = -EINVAL;
		goto out;
	}
	info->irq_batt = platform_get_irq(pdev, 1);
	if (info->irq_batt < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		ret = -EINVAL;
		goto out;
	}

	info->chip = chip;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	info->dev = &pdev->dev;
	info->status = POWER_SUPPLY_STATUS_UNKNOWN;
	ginfo = info;

	ret = request_threaded_irq(info->irq_cc, NULL, pm860x_coulomb_handler,
				   IRQF_ONESHOT, "coulomb", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq_cc, ret);
		goto out;
	}
	ret = request_threaded_irq(info->irq_batt, NULL, pm860x_batt_handler,
				   IRQF_ONESHOT, "battery", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq_batt, ret);
		goto out_coulomb;
	}

	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);

	info->battery.name = "battery";
	info->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	info->battery.properties = pm860x_batt_props;
	info->battery.num_properties = ARRAY_SIZE(pm860x_batt_props);
	info->battery.get_property = pm860x_batt_get_prop;
	info->battery.external_power_changed = pm860x_external_power_changed;
	info->max_capacity = 1500;	/* set default capacity */
	info->resistor = 300;		/* set default internal resistor */
	ret = power_supply_register(&pdev->dev, &info->battery);
	if (ret)
		goto out_attr;
	info->battery.dev->parent = &pdev->dev;

	pm860x_init_battery(info);

	INIT_DELAYED_WORK(&info->monitor_work, pm860x_battery_work);
	INIT_DELAYED_WORK(&info->changed_work, pm860x_changed_work);
	info->monitor_wqueue = create_singlethread_workqueue("battery-monitor");
	if (!info->monitor_wqueue) {
		ret = -ESRCH;
		goto out_work;
	}

	INIT_DELAYED_WORK(&monitor_batteryvolt_work, battery_volt_read_work);
      battery_volt_read_work();
	  
	queue_delayed_work(chip->monitor_wqueue, &info->monitor_work,
			   MONITOR_INTERVAL);

	ret=sysfs_create_group(&pdev->dev.kobj, &battery_attr_group);
	if (ret < 0) {
		goto out_work;
	}

	return 0;

out_work:
	power_supply_unregister(&info->battery);
out_attr:
	device_remove_file(&pdev->dev, &dev_attr_calibration);
	free_irq(info->irq_batt, info);
out_coulomb:
	free_irq(info->irq_cc, info);
out:
	kfree(info);
	return ret;
}

static int __devexit pm860x_battery_remove(struct platform_device *pdev)
{
	struct pm860x_battery_info *info = platform_get_drvdata(pdev);

	flush_workqueue(info->chip->monitor_wqueue);
	destroy_workqueue(info->monitor_wqueue);
	platform_set_drvdata(pdev, NULL);
	power_supply_unregister(&info->battery);
	device_remove_file(info->dev, &dev_attr_calibration);
	free_irq(info->irq_batt, info);
	free_irq(info->irq_cc, info);
	kfree(info);
	return 0;
}

#ifdef CONFIG_PM
static int pm860x_battery_suspend(struct device *dev)
{
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	calc_ccnt(info, &ccnt_data);
	return 0;
}

static int pm860x_battery_resume(struct device *dev)
{
	int i = 0;
	struct pm860x_battery_info *info = dev_get_drvdata(dev);
	if(info->chip->chip_version >= PM8607_C1D_VERSION){
		calc_soc(info, OCV_MODE_ACTIVE, &info->start_soc);
		/*when sanremo sleep fail, the vbat_sleep will be 0, so fix it*/
		if(info->start_soc < 0){
			calc_soc(info, OCV_MODE_ACTIVE, &info->start_soc);
		}
		clear_ccnt(info, &ccnt_data);
		dev_dbg(dev,"resume:soc:%d\n",info->start_soc);
	}
	#if 0
	volt_timer_count = 0;
	
   	for (i=0; i<loop; i++)
        {
		volt_loop[i]=0;
        }
	#endif
	battery_volt_read_work();
	//schedule_delayed_work(&monitor_batteryvolt_work, 0);
	return 0;
}

static struct dev_pm_ops pm860x_battery_pm_ops = {
	.suspend	= pm860x_battery_suspend,
	.resume		= pm860x_battery_resume,
};
#endif

static struct platform_driver pm860x_battery_driver = {
	.driver		= {
		.name	= "88pm860x-battery",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm860x_battery_pm_ops,
#endif
	},
	.probe		= pm860x_battery_probe,
	.remove		= __devexit_p(pm860x_battery_remove),
};

static int __init pm860x_battery_init(void)
{
	return platform_driver_register(&pm860x_battery_driver);
}
module_init(pm860x_battery_init);

static void __exit pm860x_battery_exit(void)
{
	platform_driver_unregister(&pm860x_battery_driver);
}
module_exit(pm860x_battery_exit);

MODULE_DESCRIPTION("Marvell 88PM860x Battery driver");
MODULE_LICENSE("GPL");
