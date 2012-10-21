/*
 * AT42XMT140/ATMXT224 Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version. 
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/earlysuspend.h>
#include <asm/uaccess.h>
#include <mach/gpio.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/atmel_xmt140_ts.h>
#ifdef CONFIG_DVFM
#include <mach/dvfm.h>
#endif


/* Version */
#define XMT140_VER_20			0x20
#define XMT140_VER_21			0x21
#define XMT140_VER_22			0x22

/* Slave addresses */
#define XMT140_APP_LOW		0x4a
#define XMT140_APP_HIGH		0x4b
#define XMT140_BOOT_LOW		0x24
#define XMT140_BOOT_HIGH		0x25

/* Firmware */
#define XMT140_FW_NAME		"atmel_xmt140.fw"

/* Registers */
#define XMT140_FAMILY_ID		0x00
#define XMT140_VARIANT_ID		0x01
#define XMT140_VERSION		0x02
#define XMT140_BUILD			0x03
#define XMT140_MATRIX_X_SIZE		0x04
#define XMT140_MATRIX_Y_SIZE		0x05
#define XMT140_OBJECT_NUM		0x06
#define XMT140_OBJECT_START		0x07

#define XMT140_OBJECT_SIZE		6

/* Object types */
#define XMT140_DEBUG_DIAGNOSTIC	37
#define XMT140_GEN_MESSAGE		5
#define XMT140_GEN_COMMAND		6
#define XMT140_GEN_POWER		7
#define XMT140_GEN_ACQUIRE		8
#define XMT140_TOUCH_MULTI		9
#define XMT140_TOUCH_KEYARRAY		15
#define XMT140_TOUCH_PROXIMITY	23
#define XMT140_PROCI_GRIPFACE		20
#define XMT140_PROCG_NOISE		22
#define XMT140_PROCI_ONETOUCH		24
#define XMT140_PROCI_TWOTOUCH		27
#define XMT140_SPT_COMMSCONFIG	18	/* firmware ver 21 over */
#define XMT140_SPT_GPIOPWM		19
#define XMT140_SPT_SELFTEST		25
#define XMT140_SPT_CTECONFIG		28
#define XMT140_SPT_USERDATA		38	/* firmware ver 21 over */

/* XMT140_GEN_COMMAND field */
#define XMT140_COMMAND_RESET		0
#define XMT140_COMMAND_BACKUPNV	1
#define XMT140_COMMAND_CALIBRATE	2
#define XMT140_COMMAND_REPORTALL	3
#define XMT140_COMMAND_DIAGNOSTIC	5

/* XMT140_GEN_POWER field */
#define XMT140_POWER_IDLEACQINT	0
#define XMT140_POWER_ACTVACQINT	1
#define XMT140_POWER_ACTV2IDLETO	2

/* XMT140_GEN_ACQUIRE field */
#define XMT140_ACQUIRE_CHRGTIME	0
#define XMT140_ACQUIRE_TCHDRIFT	2
#define XMT140_ACQUIRE_DRIFTST	3
#define XMT140_ACQUIRE_TCHAUTOCAL	4
#define XMT140_ACQUIRE_SYNC		5
#define XMT140_ACQUIRE_ATCHCALST	6
#define XMT140_ACQUIRE_ATCHCALSTHR	7

/* XMT140_TOUCH_MULTI field */
#define XMT140_TOUCH_CTRL		0
#define XMT140_TOUCH_XORIGIN		1
#define XMT140_TOUCH_YORIGIN		2
#define XMT140_TOUCH_XSIZE		3
#define XMT140_TOUCH_YSIZE		4
#define XMT140_TOUCH_BLEN		6
#define XMT140_TOUCH_TCHTHR		7
#define XMT140_TOUCH_TCHDI		8
#define XMT140_TOUCH_ORIENT		9
#define XMT140_TOUCH_MOVHYSTI		11
#define XMT140_TOUCH_MOVHYSTN		12
#define XMT140_TOUCH_NUMTOUCH		14
#define XMT140_TOUCH_MRGHYST		15
#define XMT140_TOUCH_MRGTHR		16
#define XMT140_TOUCH_AMPHYST		17
#define XMT140_TOUCH_XRANGE_LSB	18
#define XMT140_TOUCH_XRANGE_MSB	19
#define XMT140_TOUCH_YRANGE_LSB	20
#define XMT140_TOUCH_YRANGE_MSB	21
#define XMT140_TOUCH_XLOCLIP		22
#define XMT140_TOUCH_XHICLIP		23
#define XMT140_TOUCH_YLOCLIP		24
#define XMT140_TOUCH_YHICLIP		25
#define XMT140_TOUCH_XEDGECTRL	26
#define XMT140_TOUCH_XEDGEDIST	27
#define XMT140_TOUCH_YEDGECTRL	28
#define XMT140_TOUCH_YEDGEDIST	29
#define XMT140_TOUCH_JUMPLIMIT	30	/* firmware ver 22 over */

/* XMT140_PROCI_GRIPFACE field */
#define XMT140_GRIPFACE_CTRL		0
#define XMT140_GRIPFACE_XLOGRIP	1
#define XMT140_GRIPFACE_XHIGRIP	2
#define XMT140_GRIPFACE_YLOGRIP	3
#define XMT140_GRIPFACE_YHIGRIP	4
#define XMT140_GRIPFACE_MAXTCHS	5
#define XMT140_GRIPFACE_SZTHR1	7
#define XMT140_GRIPFACE_SZTHR2	8
#define XMT140_GRIPFACE_SHPTHR1	9
#define XMT140_GRIPFACE_SHPTHR2	10
#define XMT140_GRIPFACE_SUPEXTTO	11

/* XMT140_PROCI_NOISE field */
#define XMT140_NOISE_CTRL		0
#define XMT140_NOISE_OUTFLEN		1
#define XMT140_NOISE_GCAFUL_LSB	3
#define XMT140_NOISE_GCAFUL_MSB	4
#define XMT140_NOISE_GCAFLL_LSB	5
#define XMT140_NOISE_GCAFLL_MSB	6
#define XMT140_NOISE_ACTVGCAFVALID	7
#define XMT140_NOISE_NOISETHR		8
#define XMT140_NOISE_FREQHOPSCALE	10
#define XMT140_NOISE_FREQ0		11
#define XMT140_NOISE_FREQ1		12
#define XMT140_NOISE_FREQ2		13
#define XMT140_NOISE_FREQ3		14
#define XMT140_NOISE_FREQ4		15
#define XMT140_NOISE_IDLEGCAFVALID	16

/* XMT140_SPT_COMMSCONFIG */
#define XMT140_COMMS_CTRL		0
#define XMT140_COMMS_CMD		1

/* XMT140_SPT_CTECONFIG field */
#define XMT140_CTE_CTRL		0
#define XMT140_CTE_CMD		1
#define XMT140_CTE_MODE		2
#define XMT140_CTE_IDLEGCAFDEPTH	3
#define XMT140_CTE_ACTVGCAFDEPTH	4
#define XMT140_CTE_VOLTAGE		5	/* firmware ver 21 over */

#define XMT140_VOLTAGE_DEFAULT	2700000
#define XMT140_VOLTAGE_STEP		10000

/* Define for XMT140_GEN_COMMAND */
#define XMT140_BOOT_VALUE		0xa5
#define XMT140_BACKUP_VALUE		0x55
#define XMT140_BACKUP_TIME		25	/* msec */
#define XMT140_RESET_TIME		65	/* msec */

#define XMT140_FWRESET_TIME		175	/* msec */

/* Command to unlock bootloader */
#define XMT140_UNLOCK_CMD_MSB		0xaa
#define XMT140_UNLOCK_CMD_LSB		0xdc

/* Bootloader mode status */
#define XMT140_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define XMT140_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define XMT140_FRAME_CRC_CHECK	0x02
#define XMT140_FRAME_CRC_FAIL		0x03
#define XMT140_FRAME_CRC_PASS		0x04
#define XMT140_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define XMT140_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define XMT140_SUPPRESS		(1 << 1)
#define XMT140_AMP			(1 << 2)
#define XMT140_VECTOR			(1 << 3)
#define XMT140_MOVE			(1 << 4)
#define XMT140_RELEASE		(1 << 5)
#define XMT140_PRESS			(1 << 6)
#define XMT140_DETECT			(1 << 7)

/* Touchscreen absolute values */
#define XMT140_MAX_XC			0x3ff
#define XMT140_MAX_YC			0x3ff
#define XMT140_MAX_AREA		0xff

#define XMT140_MAX_FINGER		10

#define TS_KEY_REPORT
//#define FTM_KEY_REPORT

#ifdef FTM_KEY_REPORT
static int ts_key_left = 102;	//
static int ts_key_middle = 139;
static int ts_key_right = 158;	
#endif

/* Initial register values recommended from chip vendor */
static const u8 init_vals_ver_20[] = {
	/* XMT140_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_GEN_POWER(7) */
	0x20, 0xff, 0x32,
	/* XMT140_GEN_ACQUIRE(8) */
	0x08, 0x05, 0x05, 0x00, 0x00, 0x00, 0x05, 0x14,
	/* XMT140_TOUCH_MULTI(9) */
	0x00, 0x00, 0x00, 0x11, 0x0a, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x01, 0x01, 0x0e, 0x0a, 0x0a, 0x0a, 0x0a, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x64,
	/* XMT140_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* XMT140_SPT_GPIOPWM(19) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00,
	/* XMT140_PROCI_GRIPFACE(20) */
	0x00, 0x64, 0x64, 0x64, 0x64, 0x00, 0x00, 0x1e, 0x14, 0x04,
	0x1e, 0x00,
	/* XMT140_PROCG_NOISE(22) */
	0x05, 0x00, 0x00, 0x19, 0x00, 0xe7, 0xff, 0x04, 0x32, 0x00,
	0x01, 0x0a, 0x0f, 0x14, 0x00, 0x00, 0xe8,
	/* XMT140_TOUCH_PROXIMITY(23) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00,
	/* XMT140_PROCI_ONETOUCH(24) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_SPT_SELFTEST(25) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* XMT140_PROCI_TWOTOUCH(27) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x00, 0x04, 0x08,
};

static const u8 init_vals_ver_21[] = {
	/* XMT140_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_GEN_POWER(7) */
	50, 20, 50,
	/* XMT140_GEN_ACQUIRE(8) */
	10, 0, 5, 20, 0, 0, 5, 45, 0, 0,//10, 25,//zlx_0614
	//10, 0, 5, 20, 0, 0, 5, 45, 0, 0,
	/* XMT140_TOUCH_MULTI(9) */
	0x83, 0x00, 0x02, 14, 9, 0x00, 16, 80, 0x02, 0x00,
	0x00, 3, 1, 48, 2, 10, 50, 10, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0, 0,
	/* XMT140_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* XMT140_SPT_GPIOPWM(19) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_PROCI_GRIPFACE(20) */
	//5, 0, 0, 0, 0, 0, 0x00, 70, 35, 4,////palm open
	0, 0, 0, 0, 0, 0, 0x00, 70, 35, 4,
	30, 10,
	/* XMT140_PROCG_NOISE(22) */
	0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 60, 0x00,
	0x00, 6, 11, 16, 19, 21, 0x00,
	/* XMT140_TOUCH_PROXIMITY(23) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_PROCI_ONETOUCH(24) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_SPT_SELFTEST(25) */
	0x03, 0x00, 0x1C, 0x25, 0xe8, 0x1c, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* XMT140_PROCI_TWOTOUCH(27) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x02, 16, 32, 10,
};

static const u8 init_vals_ver_22[] = {
	/* XMT140_GEN_COMMAND(6) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_GEN_POWER(7) */
	0x20, 0x10, 0x32,
	/* XMT140_GEN_ACQUIRE(8) */
	0x0a, 0x00, 0x05, 0x00, 0x00, 0x00, 0x09, 0x23,
	/* XMT140_TOUCH_MULTI(9) */
	0x00, 0x00, 0x00, 0x13, 0x0b, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x00, 0x01, 0x01, 0x0e, 0x0a, 0x0a, 0x0a, 0x0a, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* XMT140_TOUCH_KEYARRAY(15) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	/* XMT140_SPT_GPIOPWM(19) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_PROCI_GRIPFACE(20) */
	0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x28, 0x04,
	0x0f, 0x0a,
	/* XMT140_PROCG_NOISE(22) */
	0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x23, 0x00,
	0x00, 0x05, 0x0f, 0x19, 0x23, 0x2d, 0x03,
	/* XMT140_TOUCH_PROXIMITY(23) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_PROCI_ONETOUCH(24) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_SPT_SELFTEST(25) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	/* XMT140_PROCI_TWOTOUCH(27) */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	/* XMT140_SPT_CTECONFIG(28) */
	0x00, 0x00, 0x00, 0x08, 0x10, 0x00,
};

struct atmel_xmt140_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct atmel_xmt140_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct atmel_xmt140_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct atmel_xmt140_finger {
	int status;
	int x;
	int y;
	int area;
};

/* Each client has this additional data */
struct atmel_xmt140_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct atmel_xmt140_platform_data *pdata;
	struct atmel_xmt140_object *object_table;
	struct atmel_xmt140_info info;
	struct atmel_xmt140_finger finger[XMT140_MAX_FINGER];
	unsigned int irq;
	//for ts watchdog, 
	struct timer_list watchdog_timer;
	struct work_struct  watchdog_work;
	struct workqueue_struct *wq;
	struct work_struct  work;
	struct early_suspend    early_suspend;
	 #ifdef CONFIG_DVFM
	int dvfm_dev_idx;
	struct dvfm_lock dvfm_lock;
	struct timer_list dvfm_timer;
	struct work_struct dvfm_work;
	unsigned int dvfm_timeout_val;
         #endif

};

// algorithm for ts shaking begin
#define NR_SAMPHISTLEN  4

/* To keep things simple (avoiding division) we ensure that
 * SUM(weight) = power-of-two. Also we must know how to approximate
 * measurements when we have less than NR_SAMPHISTLEN samples.
 */
static const unsigned char weight [NR_SAMPHISTLEN - 1][NR_SAMPHISTLEN + 1] =
{
    /* The last element is pow2(SUM(0..3)) */
	{ 5, 3, 0, 0, 3 },  /* When we have 2 samples ... */
	{ 8, 5, 3, 0, 4 },  /* When we have 3 samples ... */
	{ 6, 4, 3, 3, 4 },  /* When we have 4 samples ... */
};

struct ts_sample {
	int		x;
	int		y;
	unsigned int	pressure;
};

struct ts_hist {
    int x;
    int y;
    unsigned int p;
};

struct tslib_dejitter {
    int delta;
	int delta_x;
	int delta_y;
    int x;
    int y;
    int down;
    int nr;
    int head;
    struct ts_hist hist[NR_SAMPHISTLEN];
};

static struct tslib_dejitter *ts_djt;
// algorithm for ts shaking end

static struct atmel_xmt140_data *g_data;
u8 max_reportid=9;//
u8 min_reportid=2;
u8 pre_data[3];
unsigned int  pre_time = 0;
unsigned int  now_time  = 0;

unsigned int  boot_time  = 0;//
unsigned int  Is_boot=0;

#ifdef CONFIG_DVFM
static void set_dvfm_constraint(struct atmel_xmt140_data *p)
{
        printk("set_dvfm_constraint\n");
	spin_lock_irqsave(&p->dvfm_lock.lock, p->dvfm_lock.flags);
	dvfm_disable_op_name("apps_idle", p->dvfm_dev_idx);
	dvfm_disable_op_name("apps_sleep", p->dvfm_dev_idx);
	dvfm_disable_op_name("sys_sleep", p->dvfm_dev_idx);
	dvfm_disable_op_name("156MHz", p->dvfm_dev_idx);
	dvfm_disable_op_name("312MHz", p->dvfm_dev_idx);
	spin_unlock_irqrestore(&p->dvfm_lock.lock, p->dvfm_lock.flags);
}

static void unset_dvfm_constraint(struct atmel_xmt140_data *p)
{
        printk("unset_dvfm_constraint\n");
	spin_lock_irqsave(&p->dvfm_lock.lock, p->dvfm_lock.flags);
	dvfm_enable_op_name("apps_idle", p->dvfm_dev_idx);
	dvfm_enable_op_name("apps_sleep", p->dvfm_dev_idx);
	dvfm_enable_op_name("sys_sleep", p->dvfm_dev_idx);
	dvfm_enable_op_name("156MHz", p->dvfm_dev_idx);
	dvfm_enable_op_name("312MHz", p->dvfm_dev_idx);
	spin_unlock_irqrestore(&p->dvfm_lock.lock, p->dvfm_lock.flags);
}

static void dvfm_work_handler(struct work_struct *work)
{
	struct atmel_xmt140_data *p_work =  
		container_of(work, struct atmel_xmt140_data, dvfm_work);
	unset_dvfm_constraint(p_work);
}

static void dvfm_timer_handler(unsigned long data)
{
	struct atmel_xmt140_data *p = (struct atmel_xmt140_data *)data;
	schedule_work(&p->dvfm_work);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
static void dvfm_timer_handler(unsigned long data) {}
#endif
static bool cal_check_flag = 0;
static unsigned int qt_time_point=0;
static unsigned int qt_time_diff=0;
static unsigned int qt_timer_state =0;
static int ts_debug;
static bool atmel_xmt140_object_readable(unsigned int type)
{
	switch (type) {
	case XMT140_GEN_MESSAGE:
	case XMT140_GEN_COMMAND:
	case XMT140_GEN_POWER:
	case XMT140_GEN_ACQUIRE:
	case XMT140_TOUCH_MULTI:
	case XMT140_TOUCH_KEYARRAY:
	case XMT140_TOUCH_PROXIMITY:
	case XMT140_PROCI_GRIPFACE:
	case XMT140_PROCG_NOISE:
	case XMT140_PROCI_ONETOUCH:
	case XMT140_PROCI_TWOTOUCH:
	case XMT140_SPT_COMMSCONFIG:
	case XMT140_SPT_GPIOPWM:
	case XMT140_SPT_SELFTEST:
	case XMT140_SPT_CTECONFIG:
	case XMT140_SPT_USERDATA:
		return true;
	default:
		return false;
	}
}

static bool atmel_xmt140_object_writable(unsigned int type)
{
	switch (type) {
	case XMT140_GEN_COMMAND: //6
	case XMT140_GEN_POWER: //7
	case XMT140_GEN_ACQUIRE: //8
	case XMT140_TOUCH_MULTI: //9
	case XMT140_TOUCH_KEYARRAY: //15
	case XMT140_TOUCH_PROXIMITY: //23
	case XMT140_PROCI_GRIPFACE: //20
	case XMT140_PROCG_NOISE: //22
	case XMT140_PROCI_ONETOUCH: //24
	case XMT140_PROCI_TWOTOUCH: //27
	case XMT140_SPT_GPIOPWM: //19
	case XMT140_SPT_SELFTEST: //25
	case XMT140_SPT_CTECONFIG: //28
		return true;
	default:
		return false;
	}
}

static void atmel_xmt140_dump_message(struct device *dev,
				  struct atmel_xmt140_message *message)
{
	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->message[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->message[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->message[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->message[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->message[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->message[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);
}

static int atmel_xmt140_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case XMT140_WAITING_BOOTLOAD_CMD:
	case XMT140_WAITING_FRAME_DATA:
		val &= ~XMT140_BOOT_STATUS_MASK;
		break;
	case XMT140_FRAME_CRC_PASS:
		if (val == XMT140_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int atmel_xmt140_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = XMT140_UNLOCK_CMD_LSB;
	buf[1] = XMT140_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int atmel_xmt140_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __atmel_xmt140_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int atmel_xmt140_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __atmel_xmt140_read_reg(client, reg, 1, val);
}

static int atmel_xmt140_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3], i = 5;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	while(i)
	{
		if (i2c_master_send(client, buf, 3) == 3) {
			break;
		}
		dev_err(&client->dev, "%s: i2c send failed, try one more\n", __func__);
		i--;
	}
	
	if(i == 0)
	{
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int atmel_xmt140_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __atmel_xmt140_read_reg(client, reg, XMT140_OBJECT_SIZE,
				   object_buf);
}

static struct atmel_xmt140_object *
atmel_xmt140_get_object(struct atmel_xmt140_data *data, u8 type)
{
	struct atmel_xmt140_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int atmel_xmt140_read_message(struct atmel_xmt140_data *data,
				 struct atmel_xmt140_message *message)
{
	struct atmel_xmt140_object *object;
	u16 reg;

	object = atmel_xmt140_get_object(data, XMT140_GEN_MESSAGE);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __atmel_xmt140_read_reg(data->client, reg,
			sizeof(struct atmel_xmt140_message), message);
}

static int atmel_xmt140_read_object(struct atmel_xmt140_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct atmel_xmt140_object *object;
	u16 reg;

	object = atmel_xmt140_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __atmel_xmt140_read_reg(data->client, reg + offset, 1, val);
}

static int atmel_xmt140_write_object(struct atmel_xmt140_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct atmel_xmt140_object *object;
	u16 reg;

	object = atmel_xmt140_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return atmel_xmt140_write_reg(data->client, reg + offset, val);
}
#ifdef TS_KEY_REPORT
const char ts_keys_size[] = "0x01:102:54:510:54:15:0x01:139:160:510:54:15:0x01:158:276:510:54:15";

struct attribute ts_key_report_attr = {
        .name = "virtualkeys.atmel_xmt140_ts",
        .mode = S_IRWXUGO,
};
 
static struct attribute *def_attrs[] = {
        &ts_key_report_attr,
        NULL,
};
 
void ts_key_report_release(struct kobject *kobject)
{
        return;
}
 
ssize_t ts_key_report_show(struct kobject *kobject, struct attribute *attr,char *buf)
{
        sprintf(buf,"%s\n",ts_keys_size);
        return strlen(ts_keys_size)+2;
}
 
ssize_t ts_key_report_store(struct kobject *kobject,struct attribute *attr,const char *buf, size_t count)
{
        return count;
}
 
struct sysfs_ops ts_key_report_sysops =
{
        .show = ts_key_report_show,
        .store = ts_key_report_store,
};
 
struct kobj_type ktype = 
{
        .release = ts_key_report_release,
        .sysfs_ops=&ts_key_report_sysops,
        .default_attrs=def_attrs,
};
 
struct kobject kobj;
static void ts_key_report_init(void)
{
        int ret = 0;
        ret = kobject_init_and_add(&kobj,&ktype,NULL,"board_properties");
        if(ret)
                printk(KERN_ERR "ts_key_report_init: Unable to init and add the kobject\n");
}
#endif

#ifdef CONFIG_PROC_FS
#define	TOUCH_PROC_FILE	"driver/pm860x_touch"
static struct proc_dir_entry *atmel_touch_proc_file;

//
static ssize_t proc_read_val(struct file *file,
    char __user *buffer, size_t count, loff_t *offset)
{
	struct atmel_xmt140_info *info = &g_data->info;
	unsigned int gpio_plr_val, gpio_pdr_val, gpio_rer_val, gpio_fer_val, gpio_edr_val;
	void *REG_BASE;
	ssize_t len = 0;
	char buffer_synap[800];
	

	len += sprintf(buffer_synap+len, 
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	len += sprintf(buffer_synap+len, 
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);
	
	REG_BASE = ioremap_nocache(0xd4019004, 0xa8);
	if (REG_BASE == NULL) {
		goto failed;
	}
	gpio_plr_val = readl(REG_BASE + 0x00);
	gpio_pdr_val = readl(REG_BASE + 0x0c);
	gpio_rer_val = readl(REG_BASE + 0x30);
	gpio_fer_val = readl(REG_BASE + 0x3c);
	gpio_edr_val = readl(REG_BASE + 0x48);
	len += sprintf(buffer_synap+len, "GPIO49 status plr: %x pdr: %x rer: %x fer: %x edr: %x \n", 
		gpio_plr_val, gpio_pdr_val, gpio_rer_val, gpio_fer_val, gpio_edr_val);

	iounmap(REG_BASE);
	
failed:
	return simple_read_from_buffer(buffer, count, offset, buffer_synap, len);
}

static ssize_t proc_write_val(struct file *filp,
				     const char *buff, size_t len,
				     loff_t * off)
{
	char messages[256];

	int object, item, val;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
    //printk(KERN_INFO "%s\n", messages);

    if ('p' == messages[0]) {
		sscanf(&messages[1], "%d %d %d", &object, &item, &val );
		printk("sscanf object:%d item:%d val:%d", object, item, val );
		atmel_xmt140_write_object(g_data, object, item, val);
		return len;
	} else if ('d' == messages[0]) {
		ts_debug = 1;
		return len;
	} else if('n' == messages[0]) {
		ts_debug = 0;
		return len;
	} else if('r' == messages[0]) {
		atmel_xmt140_write_object(g_data, XMT140_TOUCH_MULTI, XMT140_TOUCH_CTRL, 0);
		atmel_xmt140_write_object(g_data, XMT140_GEN_COMMAND, XMT140_COMMAND_RESET, 1);
		msleep(XMT140_RESET_TIME);
		atmel_xmt140_write_object(g_data, XMT140_TOUCH_MULTI, XMT140_TOUCH_CTRL, 0x83);
		return len;
	}

	return len;
}

static struct file_operations synapatics_touch_proc_ops = {
	.read = proc_read_val,
	.write = proc_write_val,
};

static void create_atmel_touch_proc_file(void)
{
	atmel_touch_proc_file =
	    create_proc_entry(TOUCH_PROC_FILE, 0777, NULL);
	if (atmel_touch_proc_file) {
		atmel_touch_proc_file->proc_fops = &synapatics_touch_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_atmel_touch_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(TOUCH_PROC_FILE, &proc_root);
}

#endif

static int atmel_xmt140_read_diagnostic(u16 read_addr, u8 *buffer, u8 size)
{
	struct i2c_client *client = g_data->client;
	struct atmel_xmt140_object *object;
	u16 reg;
	u8 buf[2];

	object = atmel_xmt140_get_object(g_data, XMT140_DEBUG_DIAGNOSTIC);
	if (!object)
		return -EINVAL;

	reg = object->start_address + read_addr;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "i2c send failed\n");
		return -EIO;
	}

	if (i2c_master_recv(client, buffer, size) != size) {
		dev_err(&client->dev, "i2c recv failed\n");
		return -EIO;
	}

	return 0;
}

static void release_all_fingers(struct atmel_xmt140_data *data)
{
	struct atmel_xmt140_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int id;
	for ( id= 0; id<XMT140_MAX_FINGER; ++id )
	{
	    if (!finger[id].status)
	        continue;

	    finger[id].status = 0;

	    input_report_abs(input_dev, ABS_MT_POSITION_X, finger[id].x);
	    input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[id].y);
	    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[id].area);
	    input_mt_sync(input_dev);
	}

	input_sync(input_dev);
}

void calibrate_chip(struct atmel_xmt140_data *data)
{
	//uint8_t atchcalst, atchcalsthr;
    int error;
	if(!cal_check_flag)
	{
	    atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 6, 0);
		atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 7, 0);
	}
    /* send calibration command to the chip */
    error = atmel_xmt140_write_object(data, XMT140_GEN_COMMAND,
                XMT140_COMMAND_CALIBRATE, 1);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }
    else
    {
        /* set flag to show we must still confirm if calibration was good or bad */
		cal_check_flag = 1;
        release_all_fingers(data);
    }

}

static int check_abs_time(void)
{
    qt_time_diff = jiffies_to_msecs(jiffies) - qt_time_point;
    if(qt_time_diff >0)
        return 1;
    else
        return 0;
}

void check_chip_calibration(struct atmel_xmt140_data *data)
{
    u8 data_buffer[100] = { 0 };
    u16 check_tsp = 0;
    int try_ctr = 0;
    int tch_ch = 0, atch_ch = 0;
    int i, error, x_line_limit;

    /* we have had the first touchscreen or face suppression message
    * after a calibration - check the sensor state and try to confirm if
    * cal was good or bad */

    /* get touch flags from the chip using the diagnostic object */
    /* write command to command processor to get touch flags - 0xF3 Command required to do this */
    error = atmel_xmt140_write_object(data, XMT140_GEN_COMMAND,
        	XMT140_COMMAND_DIAGNOSTIC, 0xF3);
    if (error < 0)
    {
        printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
    }

    atmel_xmt140_read_diagnostic(0, data_buffer, 2 );

    while(!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)))
    {
        /* wait for data to be valid  */
        if(try_ctr > 10) //0318 hugh 100-> 10
        {
            /* Failed! */
            printk(KERN_ERR "[TSP] Diagnostic Data did not update!!\n");
            qt_timer_state = 0;//0430 hugh

            /* soft reset */
            atmel_xmt140_write_object(data, XMT140_GEN_COMMAND,
                	XMT140_COMMAND_RESET, 1);

            /* wait for soft reset */
            msleep(100);
            calibrate_chip(data);
            break;
        }
        msleep(2); //0318 hugh  3-> 2
        try_ctr++; /* timeout counter */

        atmel_xmt140_read_diagnostic(0, data_buffer, 2 );
//        printk("[TSP] Waiting for diagnostic data to update, try %d\n", try_ctr);
    }

    /* data is ready - read the detection flags */
    atmel_xmt140_read_diagnostic(0, data_buffer, 82);

    /* data array is 20 x 16 bits for each set of flags, 2 byte header, 40 bytes for touch flags 40 bytes for antitouch flags*/

    /* count up the channels/bits if we recived the data properly */
    if((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))
    {
        x_line_limit = 14;

        if(x_line_limit > 20)
        {
            /* hard limit at 20 so we don't over-index the array */
            x_line_limit = 20;
        }

        /* double the limit as the array is in bytes not words */
        x_line_limit = x_line_limit << 1;

        for(i = 0; i < x_line_limit; i++)
        {
            check_tsp = data_buffer[2+i];
            while(check_tsp)
            {
                if(check_tsp & 0x1)
                {
                    tch_ch++;
                }
                check_tsp = check_tsp >>1;
            }

            check_tsp = data_buffer[42+i];
            while(check_tsp)
            {
                if(check_tsp & 0x1)
                {
                    atch_ch++;
                }
                check_tsp = check_tsp >>1;
            }
        }

        /* print how many channels we counted */
//        if(atch_ch>0) //
		//{
		//	printk("[TSP] Flags Counted channels: t:%d a:%d \n", tch_ch, atch_ch);
		//}

        /* send page up command so we can detect when data updates next time,
			 * page byte will sit at 1 until we next send F3 command */
        error = atmel_xmt140_write_object(data, XMT140_GEN_COMMAND,
        	XMT140_COMMAND_DIAGNOSTIC, 0x01);
        if (error < 0)
        {
            printk(KERN_ERR "[TSP] error %s: write_object\n", __func__);
        }

        /* process counters and decide if we must re-calibrate or if cal was good */
        if((tch_ch>0) && (atch_ch == 0))  //jwlee change.
        {
            /* cal was good - don't need to check any more */
            //hugh 0312
           #if 0
            if(!check_abs_time())
                qt_time_diff=1001;
			
	  printk(KERN_DEBUG "qt_time_diff = %d\n",qt_time_diff);
	  
            if(qt_timer_state == 1)
            {
                if(qt_time_diff > 1000)
                {
                    printk(KERN_DEBUG "[TSP] calibration was good\n");
                    qt_timer_state =0;
                    cal_check_flag = 0;
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 0, 10);
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 1, 0);
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 2, 5);
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 3, 20);
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 4, 0);
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 5, 0);
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 6, 0);//
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 7, 1);
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 8, 10);
					atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 9, 25);
					//atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 7, 1);
					//atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 8, 0);
					//atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 9, 0);
                    /* Write normal acquisition config back to the chip. */
                    //error = ATMEL_XMT140_Acquisition_Config_Init(data);
                    //if(error<0)
                   // {
                   //     printk(KERN_ERR "[TSP] fail to initialize the Acqusition config\n");
                   // }
                }

            }
            else
            {
            	qt_timer_state=1;
            	qt_time_point = jiffies_to_msecs(jiffies);
            	cal_check_flag=1;
            }
	#endif
        }
        else if(atch_ch >= 1)// && (tch_ch + atch_ch) >= 8)		//jwlee add 0325//
        {
            printk(KERN_DEBUG "[TSP] calibration was bad\n");

            /* cal was bad - must recalibrate and check afterwards */
            calibrate_chip(data);
            qt_timer_state=0;
	    pre_data[0] =0;
        }
        else
        {
    //        printk(KERN_DEBUG "[TSP] calibration was not decided yet\n");
            /* we cannot confirm if good or bad - we must wait for next touch  message to confirm */
            /* Reset the 100ms timer */
            qt_timer_state=0;//0430 hugh 1 --> 0
            qt_time_point = jiffies_to_msecs(jiffies);
        }
    }
}

// algorithm for ts shaking begin
static int sqr (int x)
{
	return x * x;
}

static void average (struct tslib_dejitter *djt, struct ts_sample *samp)
{
	const unsigned char *w;
	int sn = djt->head;
	int i, x = 0, y = 0;
	unsigned int p = 0;

	w = weight [djt->nr - 2];

	for (i = 0; i < djt->nr; i++) {
		x += djt->hist [sn].x * w [i];
		y += djt->hist [sn].y * w [i];
		p += djt->hist [sn].p * w [i];
		sn = (sn - 1) & (NR_SAMPHISTLEN - 1);
	}

	samp->x = x >> w [NR_SAMPHISTLEN];
	samp->y = y >> w [NR_SAMPHISTLEN];
	samp->pressure = p >> w [NR_SAMPHISTLEN];
	
	//printk("average----------------> %d %d %d\n",
	//    samp->x, samp->y, samp->pressure);
}

static int djt_filter(struct tslib_dejitter *djt, struct ts_sample *samp)
{
    struct ts_sample *s, samp_data;//, pre_samp; not used 
	
    samp_data = *samp;
	s = &samp_data;
	
	{
        if (s->pressure == 0) {
            /*
             * Pen was released. Reset the state and
             * forget all history events.
             */
            djt->nr = 0;
            *samp = *s;
            goto out;
        }

        /* If the pen moves too fast, reset the backlog. */
        if (djt->nr) {
            int prev = (djt->head - 1) & (NR_SAMPHISTLEN - 1);
            if (sqr (s->x - djt->hist [prev].x) +
                sqr (s->y - djt->hist [prev].y) > djt->delta) {
                //printk("pen moves too fast\n");
                djt->nr = 0;
            }
        }

        djt->hist[djt->head].x = s->x;
        djt->hist[djt->head].y = s->y;
        djt->hist[djt->head].p = s->pressure;
        if (djt->nr < NR_SAMPHISTLEN)
            djt->nr++;

        /* We'll pass through the very first sample since
         * we can't average it (no history yet).
         */
        if (djt->nr == 1)
            *samp = *s;
        else {
            average (djt, samp);
        }
		
        djt->head = (djt->head + 1) & (NR_SAMPHISTLEN - 1);
    }
out:
    return 0;
}

static void simple_filter(int *x, int *y, int finger)
{
	static int pre_x = 0, pre_y = 0;
	struct ts_sample samp;
	
	samp.pressure = finger;
	samp.x = *x;
	samp.y = *y;
	djt_filter(ts_djt, &samp);
	*x = samp.x;
	*y = samp.y;/**/
	
	if(abs(*x - pre_x) <= 3 && abs(*y - pre_y) <= 4)
	{
		*x = pre_x;
		*y = pre_y;
	}
	pre_x = *x;
	pre_y = *y;
}
// algorithm for ts shaking end

#ifdef FTM_KEY_REPORT
static int ts_key_report(int x, int y, int pen_down)
{
//    struct pm860x_touch *touch = pm860x_touch_data;
	static int keycode = 0;

	if(pen_down){
		if(y > 495 && !keycode)
		{
		    if(x > 27 && x < 81)
		    {
                keycode = ts_key_left;
		    }
			else if(x > 133 && x < 187)
			{
			    keycode = ts_key_middle;
		    }
			else if(x > 249 && x < 303)
			{
			    keycode = ts_key_right;
		    }
			else
			{
				return 0;
			}
			//printk(KERN_INFO "--- ts_key x=%d\n", keycode);
			input_report_key(g_data->input_dev, keycode, pen_down);
			input_sync(g_data->input_dev);
			return 1;
		}
		else
			return 0;
	}
	else
	{
		if(keycode)
		{
			//printk(KERN_INFO "--- ts_key pen_up x=%d\n", keycode);
			input_report_key(g_data->input_dev, keycode, pen_down);
			input_sync(g_data->input_dev);
			keycode = 0;
			return 1;
		}
		else
			return 0;
	}
        
	return 0;
}
#endif

static void atmel_xmt140_input_report(struct atmel_xmt140_data *data, int single_id)
{
	struct atmel_xmt140_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;
#if 1
	
#ifdef FTM_KEY_REPORT
	if(ts_key_report(finger[single_id].x, finger[single_id].y, finger[single_id].status != XMT140_RELEASE ? finger[single_id].area : 0))
	{
		return;
	}
#endif

	for (id = 0; id < XMT140_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;
		// algorithm for ts shaking
		simple_filter(&finger[id].x, &finger[id].y, finger[id].status != XMT140_RELEASE ? 1 : 0);

		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
				finger[id].status != XMT140_RELEASE ?
				finger[id].area : 0);
		input_report_abs(input_dev, ABS_MT_POSITION_X,
				finger[id].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y,
				finger[id].y);
		input_mt_sync(input_dev);

		if (finger[id].status == XMT140_RELEASE)
			finger[id].status = 0;
		else
			finger_num++;
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

	if (status != XMT140_RELEASE) {
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
	}

	input_sync(input_dev);
#else
	if(status == XMT140_RELEASE)
	{
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_sync(input_dev);
 	}
	else
	{
		for (id = 0; id < XMT140_MAX_FINGER; id++) {
			if (!finger[id].status)
				continue;

 			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[id].area);
			input_report_abs(input_dev, ABS_MT_POSITION_X, finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[id].y);
			input_mt_sync(input_dev);

			if (finger[id].status == XMT140_RELEASE)
				finger[id].status = 0;
			else
				finger_num++;
		}

		input_report_key(input_dev, BTN_TOUCH, finger_num > 0);
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
		input_report_key(input_dev, BTN_TOUCH, 1);
		input_sync(input_dev);
	}
	
#endif
}

static void confirm_calibration(struct atmel_xmt140_data*data)//
{
	cal_check_flag = 0;
	pre_data[0] = 2;
	atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 6, 0);//
	atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 7, 1);
	atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 8, 0);
	atmel_xmt140_write_object(data, XMT140_GEN_ACQUIRE, 9, 0);
	printk(KERN_INFO "calibration confirm\n");
}

//for ts watchdog, 
static void watchdog_timer_handler(unsigned long data)
{
	struct atmel_xmt140_data *ts_data = (struct atmel_xmt140_data *)data;
	//printk("%s in\n", __FUNCTION__);
	
	schedule_work(&ts_data->watchdog_work);
	mod_timer(&ts_data->watchdog_timer, jiffies + msecs_to_jiffies(3000));
}

static void reset_watchdog_work(struct work_struct *work)
{
	struct atmel_xmt140_data *data = container_of(work, struct atmel_xmt140_data, watchdog_work);
	struct atmel_xmt140_message message;
	int int_pin_val = 0;
	
	//Read intr pin, if low read data.
	int_pin_val = GPLR(49);
	int_pin_val = int_pin_val & 0x20000;
	
	//printk("%s %d\n", __FUNCTION__, int_pin_val);
	if(int_pin_val == 0)
	{
		printk("%s ...\n", __FUNCTION__);
		atmel_xmt140_read_message(data, &message);    //read data empty
		release_all_fingers(data);
	}
}

static void atmel_xmt140_input_touchevent(struct atmel_xmt140_data *data,
				      struct atmel_xmt140_message *message, int id)
{
	struct atmel_xmt140_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;
	
	
	
	/* Check the touch is present on the screen */
	if (!(status & XMT140_DETECT)) {
		if (status & XMT140_RELEASE) {
			dev_dbg(dev, "[%d] released\n", id);
			finger[id].status = XMT140_RELEASE;
			if((pre_data[0] == 1)&& (id == 0))
			{
				now_time = jiffies_to_msecs(jiffies);
				printk("---now_time = %u, pre_time = %u\n",now_time,pre_time);
				if(Is_boot) //
				{
					if(now_time -boot_time > 60000)
					{
						Is_boot = 0;
					}
						
				}
				else
				{
					if(((finger[id].x-pre_data[1]>100) || (finger[id].y-pre_data[2]>100)) &&(now_time -pre_time > 250))
					{
						confirm_calibration(data);
					}	
					else
					{
						pre_data[0] = 0;
					}
				}
			}
			atmel_xmt140_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (XMT140_PRESS | XMT140_MOVE)))
		return;

	x = (message->message[1] << 2) | ((message->message[3] & ~0x3f) >> 6);
	y = (message->message[2] << 2) | ((message->message[3] & ~0xf3) >> 2);
	area = message->message[4];

	if(ts_debug)
		printk("--------[%d] %s x: %d, y: %d, area: %d\n", id,
		status & XMT140_MOVE ? "moved" : "pressed",
		x, y, area);
	
	dev_dbg(dev, "[%d] %s x: %d, y: %d, area: %d\n", id,
		status & XMT140_MOVE ? "moved" : "pressed",
		x, y, area);

	finger[id].status = status & XMT140_MOVE ?
				XMT140_MOVE : XMT140_PRESS;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = area;

	if(((pre_data[0] == 0)&& (id == 0))&&(finger[1].status != XMT140_DETECT))
	{
		pre_data[0]=1;//
		pre_data[1] = finger[id].x;
		pre_data[2] = finger[id].y;
		
		pre_time =jiffies_to_msecs(jiffies);
		printk("----pre_time = %u\n",pre_time);
	}
	else if((pre_data[0] == 1) && (finger[1].status == XMT140_DETECT))
	{
		pre_data[0]=0;//
	}
	
	
	
	atmel_xmt140_input_report(data, id);
}
static void atmel_xmt140_worker(struct work_struct *work)
{
	struct atmel_xmt140_data *data = container_of(work, struct atmel_xmt140_data, work);
	struct atmel_xmt140_message message;
	//struct atmel_xmt140_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	//u8 max_reportid;
	//u8 min_reportid;
	
	//printk("-------------atmel_xmt140_interrupt\n");

	do {
		if (atmel_xmt140_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;
		
		//if(ts_debug)
		//	printk("---reportid: 0x%x status: 0x%x\n", message.reportid, message.message[0]);
		if(ts_debug)
		{
			//printk("---reportid: 0x%x status: 0x%x msg1:0x%x msg2:0x%x msg3:0x%x msg4:0x%x msg5:0x%x msg6:0x%x\n", message.reportid, message.message[0], 
			//	message.message[1], message.message[2], message.message[3], 
			//	message.message[4], message.message[5], message.message[6]);
		}
		
		if(cal_check_flag)//
		{
			    check_chip_calibration(data);
		}
		
		if(12 == reportid)//zlx_0614
		{
			if((message.message[1]&0x01) == 0)
			{
				printk("---zlx--- reportid=12  so release!!\n");
				data->finger[2].status = XMT140_RELEASE;
				data->finger[3].status = XMT140_RELEASE;
				atmel_xmt140_input_report(data, 2);
				atmel_xmt140_input_report(data, 3);
			}
		}
		else
		{
			/* whether reportid is thing of XMT140_TOUCH_MULTI */
		//	object = atmel_xmt140_get_object(data, XMT140_TOUCH_MULTI);  //
		//	if (!object)
		//		goto end;

		//	max_reportid = object->max_reportid;
		//	min_reportid = max_reportid - object->num_report_ids + 1;

             //   printk("---max_reportid: 0x%x min_reportid: 0x%x\n", max_reportid, min_reportid);
			
			id = reportid - min_reportid;

			if (reportid >= min_reportid && reportid <= max_reportid)
				atmel_xmt140_input_touchevent(data, &message, id);
			else
				atmel_xmt140_dump_message(dev, &message);
		}
	} while (reportid != 0xff);

end:
/*	if(cal_check_flag)
	{
	    check_chip_calibration(data);
	}*/
	return;
}

static irqreturn_t atmel_xmt140_interrupt(int irq, void *dev_id)
{
	struct atmel_xmt140_data *data = dev_id;

	//for ts watchdog, 
	mod_timer(&data->watchdog_timer, jiffies + msecs_to_jiffies(3000));
	
        #ifdef CONFIG_DVFM
	if (mod_timer(&g_data->dvfm_timer, jiffies + g_data->dvfm_timeout_val)) {
		/* timer is already active */ 
	} else {
		set_dvfm_constraint(g_data);
	}
        #endif
	
	if (!work_pending(&data->work))
		queue_work(data->wq, &data->work);

	return IRQ_HANDLED;
}

static int atmel_xmt140_check_reg_init(struct atmel_xmt140_data *data)
{
	struct atmel_xmt140_object *object;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, j;
	u8 version = data->info.version;
	u8 *init_vals;

	switch (version) {
	case XMT140_VER_20:
		init_vals = (u8 *)init_vals_ver_20;
		break;
	case XMT140_VER_21:
		init_vals = (u8 *)init_vals_ver_21;
		break;
	case XMT140_VER_22:
		init_vals = (u8 *)init_vals_ver_22;
		break;
	default:
		dev_err(dev, "Firmware version %d doesn't support\n", version);
		return -EINVAL;
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!atmel_xmt140_object_writable(object->type))
			continue;

		for (j = 0; j < object->size + 1; j++)
			atmel_xmt140_write_object(data, object->type, j,
					init_vals[index + j]);

		index += object->size + 1;
	}

	return 0;
}

static int atmel_xmt140_check_matrix_size(struct atmel_xmt140_data *data)
{
	const struct atmel_xmt140_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	int mode = -1;
	int error;
	u8 val;

	dev_dbg(dev, "Number of X lines: %d\n", pdata->x_line);
	dev_dbg(dev, "Number of Y lines: %d\n", pdata->y_line);

	switch (pdata->x_line) {
	case 0 ... 15:
		if (pdata->y_line <= 14)
			mode = 2;//for 830 ts, added by 20110530
		break;
	case 16:
		if (pdata->y_line <= 12)
			mode = 1;
		if (pdata->y_line == 13 || pdata->y_line == 14)
			mode = 0;
		break;
	case 17:
		if (pdata->y_line <= 11)
			mode = 2;
		if (pdata->y_line == 12 || pdata->y_line == 13)
			mode = 1;
		break;
	case 18:
		if (pdata->y_line <= 10)
			mode = 3;
		if (pdata->y_line == 11 || pdata->y_line == 12)
			mode = 2;
		break;
	case 19:
		if (pdata->y_line <= 9)
			mode = 4;
		if (pdata->y_line == 10 || pdata->y_line == 11)
			mode = 3;
		break;
	case 20:
		mode = 4;
	}

	if (mode < 0) {
		dev_err(dev, "Invalid X/Y lines\n");
		return -EINVAL;
	}

	error = atmel_xmt140_read_object(data, XMT140_SPT_CTECONFIG,
				XMT140_CTE_MODE, &val);
	if (error)
		return error;

	if (mode == val)
		return 0;

	/* Change the CTE configuration */
	atmel_xmt140_write_object(data, XMT140_SPT_CTECONFIG,
			XMT140_CTE_CTRL, 1);  
	atmel_xmt140_write_object(data, XMT140_SPT_CTECONFIG,
			XMT140_CTE_MODE, mode);
	atmel_xmt140_write_object(data, XMT140_SPT_CTECONFIG,
			XMT140_CTE_IDLEGCAFDEPTH, 16);//zlx_0614
	atmel_xmt140_write_object(data, XMT140_SPT_CTECONFIG,
			XMT140_CTE_ACTVGCAFDEPTH, 32);//zlx_0614
	
	atmel_xmt140_write_object(data, XMT140_SPT_CTECONFIG,
			XMT140_CTE_CTRL, 0);

	return 0;
}

static int atmel_xmt140_make_highchg(struct atmel_xmt140_data *data)
{
	struct device *dev = &data->client->dev;
	int count = 10;
	int error;
	u8 val;

	/* Read dummy message to make high CHG pin */
	do {
		error = atmel_xmt140_read_object(data, XMT140_GEN_MESSAGE, 0, &val);
		if (error)
			return error;
	} while ((val != 0xff) && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared val =0x%x\n", val);
		//return -EBUSY;
	}

	return 0;
}

static void atmel_xmt140_handle_pdata(struct atmel_xmt140_data *data)
{
	const struct atmel_xmt140_platform_data *pdata = data->pdata;
	u8 voltage;

	/* Set touchscreen lines */
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI, XMT140_TOUCH_XSIZE,
			pdata->x_line);
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI, XMT140_TOUCH_YSIZE,
			pdata->y_line);

	/* Set touchscreen orient */
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI, XMT140_TOUCH_ORIENT,
			pdata->orient);

	/* Set touchscreen burst length */
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI,
			XMT140_TOUCH_BLEN, pdata->blen);

	/* Set touchscreen threshold */
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI,
			XMT140_TOUCH_TCHTHR, 55);//pdata->threshold);

	/* Set touchscreen resolution */
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI,
			XMT140_TOUCH_XRANGE_LSB, (pdata->x_size - 1) & 0xff);
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI,
			XMT140_TOUCH_XRANGE_MSB, (pdata->x_size - 1) >> 8);
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI,
			XMT140_TOUCH_YRANGE_LSB, (pdata->y_size - 1) & 0xff);
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI,
			XMT140_TOUCH_YRANGE_MSB, (pdata->y_size - 1) >> 8);

	/* Set touchscreen voltage */
	if (data->info.version >= XMT140_VER_21 && pdata->voltage) {
		if (pdata->voltage < XMT140_VOLTAGE_DEFAULT) {
			voltage = (XMT140_VOLTAGE_DEFAULT - pdata->voltage) /
				XMT140_VOLTAGE_STEP;
			voltage = 0xff - voltage + 1;
		} else
			voltage = (pdata->voltage - XMT140_VOLTAGE_DEFAULT) /
				XMT140_VOLTAGE_STEP;

		atmel_xmt140_write_object(data, XMT140_SPT_CTECONFIG,
				XMT140_CTE_VOLTAGE, voltage);
	}
}

static int atmel_xmt140_get_info(struct atmel_xmt140_data *data)
{
	struct i2c_client *client = data->client;
	struct atmel_xmt140_info *info = &data->info;
	int error;
	u8 val;

	error = atmel_xmt140_read_reg(client, XMT140_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = atmel_xmt140_read_reg(client, XMT140_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = atmel_xmt140_read_reg(client, XMT140_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = atmel_xmt140_read_reg(client, XMT140_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = atmel_xmt140_read_reg(client, XMT140_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int atmel_xmt140_get_object_table(struct atmel_xmt140_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[XMT140_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct atmel_xmt140_object *object = data->object_table + i;

		reg = XMT140_OBJECT_START + XMT140_OBJECT_SIZE * i;
		error = atmel_xmt140_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];


	//	max_reportid = object->max_reportid; //
	//	min_reportid = max_reportid - object->num_report_ids + 1;
		
		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
			object->max_reportid = reportid;
		}
		printk("--------object type:%d, size:%d, instances:%d, ids:%d, max_reportid:%d\n",
			object->type, object->size, object->instances, object->num_report_ids,
			object->max_reportid);
	}

	return 0;
}

static int atmel_xmt140_initialize(struct atmel_xmt140_data *data)
{
	struct i2c_client *client = data->client;
	struct atmel_xmt140_info *info = &data->info;
	int error;
	u8 val;
	
	pre_data[0] = 0;//
	
	error = atmel_xmt140_get_info(data);
	if (error)
		return error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct atmel_xmt140_data),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = atmel_xmt140_get_object_table(data);
	if (error)
		return error;

	/* Check register init values */
	error = atmel_xmt140_check_reg_init(data);
	if (error)
		return error;

	/* Check X/Y matrix size */
	error = atmel_xmt140_check_matrix_size(data);
	if (error)
		return error;
	
	error = atmel_xmt140_make_highchg(data);
	if (error)
		return error;

	atmel_xmt140_handle_pdata(data);

	/* Backup to memory */
	atmel_xmt140_write_object(data, XMT140_GEN_COMMAND,
			XMT140_COMMAND_BACKUPNV,
			XMT140_BACKUP_VALUE);
	msleep(XMT140_BACKUP_TIME);
	
	/* Update matrix size at info struct */
	error = atmel_xmt140_read_reg(client, XMT140_MATRIX_X_SIZE, &val);
	if (error)
		return error;
	info->matrix_xsize = val;

	error = atmel_xmt140_read_reg(client, XMT140_MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	info->matrix_ysize = val;
	
	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	calibrate_chip(data);
	return 0;
}

static ssize_t atmel_xmt140_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct atmel_xmt140_data *data = dev_get_drvdata(dev);
	struct atmel_xmt140_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += sprintf(buf + count,
				"Object Table Element %d(Type %d)\n",
				i + 1, object->type);

		if (!atmel_xmt140_object_readable(object->type)) {
			count += sprintf(buf + count, "\n");
			continue;
		}

		for (j = 0; j < object->size + 1; j++) {
			error = atmel_xmt140_read_object(data,
						object->type, j, &val);
			if (error)
				return error;

			count += sprintf(buf + count,
					"  Byte %d: 0x%x (%d)\n", j, val, val);
		}

		count += sprintf(buf + count, "\n");
	}

	return count;
}

static int atmel_xmt140_load_fw(struct device *dev, const char *fn)
{
	struct atmel_xmt140_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	atmel_xmt140_write_object(data, XMT140_GEN_COMMAND,
			XMT140_COMMAND_RESET, XMT140_BOOT_VALUE);
	msleep(XMT140_RESET_TIME);

	/* Change to slave address of bootloader */
	if (client->addr == XMT140_APP_LOW)
		client->addr = XMT140_BOOT_LOW;
	else
		client->addr = XMT140_BOOT_HIGH;

	ret = atmel_xmt140_check_bootloader(client, XMT140_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	atmel_xmt140_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = atmel_xmt140_check_bootloader(client,
						XMT140_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		atmel_xmt140_fw_write(client, fw->data + pos, frame_size);

		ret = atmel_xmt140_check_bootloader(client,
						XMT140_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);

	/* Change to slave address of application */
	if (client->addr == XMT140_BOOT_LOW)
		client->addr = XMT140_APP_LOW;
	else
		client->addr = XMT140_APP_HIGH;

	return ret;
}

static ssize_t atmel_xmt140_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct atmel_xmt140_data *data = dev_get_drvdata(dev);
	unsigned int version;
	int error;

	if (sscanf(buf, "%u", &version) != 1) {
		dev_err(dev, "Invalid values\n");
		return -EINVAL;
	}

	if (data->info.version < XMT140_VER_21 || version < XMT140_VER_21) {
		dev_err(dev, "FW update supported starting with version 21\n");
		return -EINVAL;
	}

	disable_irq(data->irq);

	error = atmel_xmt140_load_fw(dev, XMT140_FW_NAME);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_dbg(dev, "The firmware update succeeded\n");

		/* Wait for reset */
		msleep(XMT140_FWRESET_TIME);

		kfree(data->object_table);
		data->object_table = NULL;

		atmel_xmt140_initialize(data);
	}

	enable_irq(data->irq);

	return count;
}

static DEVICE_ATTR(object, 0444, atmel_xmt140_object_show, NULL);
static DEVICE_ATTR(update_fw, 0664, NULL, atmel_xmt140_update_fw_store);

static struct attribute *atmel_xmt140_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	NULL
};

static const struct attribute_group atmel_xmt140_attr_group = {
	.attrs = atmel_xmt140_attrs,
};

static void atmel_xmt140_start(struct atmel_xmt140_data *data)
{
	/* Touch enable */
	atmel_xmt140_write_object(data,
			XMT140_TOUCH_MULTI, XMT140_TOUCH_CTRL, 0x83);
}

static void atmel_xmt140_stop(struct atmel_xmt140_data *data)
{
	/* Touch disable */
	atmel_xmt140_write_object(data,
			XMT140_TOUCH_MULTI, XMT140_TOUCH_CTRL, 0);
}

static int atmel_xmt140_input_open(struct input_dev *dev)
{
	struct atmel_xmt140_data *data = input_get_drvdata(dev);

	atmel_xmt140_start(data);

	return 0;
}

static void atmel_xmt140_input_close(struct input_dev *dev)
{
	struct atmel_xmt140_data *data = input_get_drvdata(dev);

	atmel_xmt140_stop(data);
}

#ifdef CONFIG_PM
static void atmel_xmt140_suspend(struct early_suspend *h)
{
	struct atmel_xmt140_data *data = container_of(h, struct atmel_xmt140_data, early_suspend);
	struct input_dev *input_dev = data->input_dev;

	mutex_lock(&input_dev->mutex);
	
	atmel_xmt140_write_object(data, XMT140_GEN_POWER, XMT140_POWER_IDLEACQINT, 0);
	atmel_xmt140_write_object(data, XMT140_GEN_POWER, XMT140_POWER_ACTVACQINT, 0);

	//if (input_dev->users)
	//	atmel_xmt140_stop(data);

	qt_timer_state = 0;
	release_all_fingers(data);
	mutex_unlock(&input_dev->mutex);

	return;
}

static void atmel_xmt140_resume(struct early_suspend *h)
{
	struct atmel_xmt140_data *data = container_of(h, struct atmel_xmt140_data, early_suspend);
	struct input_dev *input_dev = data->input_dev;

	printk("atmel_xmt140_resume......\n");
	
	cal_check_flag = 0;//
	pre_data[0] = 0;
	/* Soft reset */
	atmel_xmt140_write_object(data, XMT140_GEN_COMMAND,
			XMT140_COMMAND_RESET, 1);

	msleep(XMT140_RESET_TIME);

	mutex_lock(&input_dev->mutex);
	
	atmel_xmt140_write_object(data, XMT140_GEN_POWER, XMT140_POWER_IDLEACQINT, 32);
	atmel_xmt140_write_object(data, XMT140_GEN_POWER, XMT140_POWER_ACTVACQINT, 255);

	//if (input_dev->users)
	//	atmel_xmt140_start(data);
	calibrate_chip(data);
	check_chip_calibration(data);

	mutex_unlock(&input_dev->mutex);

	return;
}
#else
#define atmel_xmt140_suspend	NULL
#define atmel_xmt140_resume		NULL
#endif

static int __devinit atmel_xmt140_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct atmel_xmt140_data *data;
	struct input_dev *input_dev;
	int error;

	if (!client->dev.platform_data)
		return -EINVAL;

	data = kzalloc(sizeof(struct atmel_xmt140_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	Is_boot=1;//
	boot_time =  jiffies_to_msecs(jiffies);
	
	input_dev->name = "atmel_xmt140_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = atmel_xmt140_input_open;
	input_dev->close = atmel_xmt140_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	
#ifdef FTM_KEY_REPORT
	__set_bit(102, input_dev->keybit);
	__set_bit(139, input_dev->keybit);
	__set_bit(158, input_dev->keybit);
#endif

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, XMT140_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_X, 0, 320, 0, 0);
    input_set_abs_params(input_dev, ABS_Y, 0, 480, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, 320, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, 480, 0, 0);

	input_set_drvdata(input_dev, data);

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = client->dev.platform_data;
	data->irq = client->irq;

	i2c_set_clientdata(client, data);

	error = atmel_xmt140_initialize(data);
	if (error)
		goto err_free_object;

	   g_data = data;

	#ifdef CONFIG_DVFM
	g_data->dvfm_dev_idx = -1;
	g_data->dvfm_lock.lock = SPIN_LOCK_UNLOCKED;
	if (!dvfm_register("atmel_xmt140_ts", &g_data->dvfm_dev_idx)) {
	INIT_WORK(&g_data->dvfm_work, dvfm_work_handler);
	init_timer(&g_data->dvfm_timer);
	g_data->dvfm_timer.function = dvfm_timer_handler;
	g_data->dvfm_timer.data = (unsigned long)g_data;
	g_data->dvfm_timeout_val = HZ * 2;
	}
	#endif

	// algorithm for ts shaking begin
	ts_djt = kzalloc(sizeof(struct tslib_dejitter), GFP_KERNEL);
	if (ts_djt == NULL)
		return -ENOMEM;

	ts_djt->head = 0;
	ts_djt->nr =0;
	ts_djt->delta = 10;
	ts_djt->delta = sqr (ts_djt->delta);
	// algorithm for ts shaking end
	
	//for ts watchdog, 
	INIT_WORK(&data->watchdog_work, reset_watchdog_work);
	setup_timer(&data->watchdog_timer, watchdog_timer_handler, (unsigned long) data);
	mod_timer(&data->watchdog_timer, jiffies + msecs_to_jiffies(3000));
	
	error = request_threaded_irq(client->irq, NULL, atmel_xmt140_interrupt,
			IRQF_DISABLED | IRQF_TRIGGER_FALLING, client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}

	error = input_register_device(input_dev);
	if (error)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &atmel_xmt140_attr_group);
	if (error)
		goto err_unregister_device;
	
	
	INIT_WORK(&data->work, atmel_xmt140_worker);
	data->wq = create_singlethread_workqueue("atmel_xmt140");
	if (!data->wq) {
        dev_err(&client->dev, "%s:failed to create work queue.\n", __func__);
		goto error_create_workqueue;
	}
	
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI, XMT140_TOUCH_CTRL, 0);
	
	/* Soft reset */
	atmel_xmt140_write_object(data, XMT140_GEN_COMMAND, XMT140_COMMAND_RESET, 1);
	msleep(XMT140_RESET_TIME);

	/* Start */
	atmel_xmt140_write_object(data, XMT140_TOUCH_MULTI, XMT140_TOUCH_CTRL, 0x83);

	calibrate_chip(data);
	check_chip_calibration(data);
	
#ifdef CONFIG_PROC_FS
	create_atmel_touch_proc_file();
#endif
#ifdef TS_KEY_REPORT
	ts_key_report_init();
#endif

#ifdef CONFIG_PM
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	data->early_suspend.suspend = atmel_xmt140_suspend,
	data->early_suspend.resume = atmel_xmt140_resume,
	register_early_suspend(&data->early_suspend);
#endif

	return 0;

error_create_workqueue:
	destroy_workqueue(data->wq);
err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
//error_create_workqueue:
//	destroy_workqueue(data->wq);
	return error;
}

static int __devexit atmel_xmt140_remove(struct i2c_client *client)
{
	struct atmel_xmt140_data *data = i2c_get_clientdata(client);
       #ifdef CONFIG_DVFM
	if( g_data->dvfm_dev_idx >= 0 ) {
		del_timer(&g_data->dvfm_timer);
		dvfm_unregister("atmel_xmt140_ts", &g_data->dvfm_dev_idx);
		g_data->dvfm_dev_idx = -1;
	}
       #endif

	sysfs_remove_group(&client->dev.kobj, &atmel_xmt140_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);
	
#ifdef CONFIG_PROC_FS
	remove_atmel_touch_proc_file();
#endif

	destroy_workqueue(data->wq);

	return 0;
}


static const struct i2c_device_id atmel_xmt140_id[] = {
	{ "atmel_xmt140_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, atmel_xmt140_id);

static struct i2c_driver atmel_xmt140_driver = {
	.driver = {
		.name	= "atmel_xmt140_ts",
		.owner	= THIS_MODULE,
	},
	.probe		= atmel_xmt140_probe,
	.remove		= __devexit_p(atmel_xmt140_remove),
	.id_table	= atmel_xmt140_id,
};

static int __init atmel_xmt140_init(void)
{
	return i2c_add_driver(&atmel_xmt140_driver);
}

static void __exit atmel_xmt140_exit(void)
{
	i2c_del_driver(&atmel_xmt140_driver);
}

module_init(atmel_xmt140_init);
module_exit(atmel_xmt140_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("AT42XMT140/ATMXT224 Touchscreen driver");
MODULE_LICENSE("GPL");
