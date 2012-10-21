/*** ADI ***/
#include <linux/adxl34x.h>
/* ADXL345/6 Register Map */
#define DEVID		0x00	/* R   Device ID */
#define THRESH_TAP	0x1D	/* R/W Tap threshold */
#define OFSX		0x1E	/* R/W X-axis offset */
#define OFSY		0x1F	/* R/W Y-axis offset */
#define OFSZ		0x20	/* R/W Z-axis offset */
#define DUR		0x21	/* R/W Tap duration */
#define LATENT		0x22	/* R/W Tap latency */
#define WINDOW		0x23	/* R/W Tap window */
#define THRESH_ACT	0x24	/* R/W Activity threshold */
#define THRESH_INACT	0x25	/* R/W Inactivity threshold */
#define TIME_INACT	0x26	/* R/W Inactivity time */
#define ACT_INACT_CTL	0x27	/* R/W Axis enable control for activity and */
				/* inactivity detection */
#define THRESH_FF	0x28	/* R/W Free-fall threshold */
#define TIME_FF		0x29	/* R/W Free-fall time */
#define TAP_AXES	0x2A	/* R/W Axis control for tap/double tap */
#define ACT_TAP_STATUS	0x2B	/* R   Source of tap/double tap */
#define BW_RATE		0x2C	/* R/W Data rate and power mode control */
#define POWER_CTL	0x2D	/* R/W Power saving features control */
#define INT_ENABLE	0x2E	/* R/W Interrupt enable control */
#define INT_MAP		0x2F	/* R/W Interrupt mapping control */
#define INT_SOURCE	0x30	/* R   Source of interrupts */
#define DATA_FORMAT	0x31	/* R/W Data format control */
#define DATAX0		0x32	/* R   X-Axis Data 0 */
#define DATAX1		0x33	/* R   X-Axis Data 1 */
#define DATAY0		0x34	/* R   Y-Axis Data 0 */
#define DATAY1		0x35	/* R   Y-Axis Data 1 */
#define DATAZ0		0x36	/* R   Z-Axis Data 0 */
#define DATAZ1		0x37	/* R   Z-Axis Data 1 */
#define FIFO_CTL	0x38	/* R/W FIFO control */
#define FIFO_STATUS	0x39	/* R   FIFO status */
#define TAP_SIGN	0x3A	/* R   Sign and source for tap/double tap */
/* Orientation ADXL346 only */
#define ORIENT_CONF	0x3B	/* R/W Orientation configuration */
#define ORIENT		0x3C	/* R   Orientation status */

/* INT_ENABLE/INT_MAP/INT_SOURCE Bits */
#define DATA_READY	(1 << 7)
#define SINGLE_TAP	(1 << 6)
#define DOUBLE_TAP	(1 << 5)
#define ACTIVITY	(1 << 4)
#define INACTIVITY	(1 << 3)
#define FREE_FALL	(1 << 2)
#define WATERMARK	(1 << 1)
#define OVERRUN		(1 << 0)

/* ACT_INACT_CONTROL Bits */
#define ACT_ACDC	(1 << 7)
#define ACT_X_EN	(1 << 6)
#define ACT_Y_EN	(1 << 5)
#define ACT_Z_EN	(1 << 4)
#define INACT_ACDC	(1 << 3)
#define INACT_X_EN	(1 << 2)
#define INACT_Y_EN	(1 << 1)
#define INACT_Z_EN	(1 << 0)

/* TAP_AXES Bits */
#define SUPPRESS	(1 << 3)
#define TAP_X_EN	(1 << 2)
#define TAP_Y_EN	(1 << 1)
#define TAP_Z_EN	(1 << 0)

/* ACT_TAP_STATUS Bits */
#define ACT_X_SRC	(1 << 6)
#define ACT_Y_SRC	(1 << 5)
#define ACT_Z_SRC	(1 << 4)
#define ASLEEP		(1 << 3)
#define TAP_X_SRC	(1 << 2)
#define TAP_Y_SRC	(1 << 1)
#define TAP_Z_SRC	(1 << 0)

/* BW_RATE Bits */
#define LOW_POWER	(1 << 4)
#define RATE(x)		((x) & 0xF)

/* POWER_CTL Bits */
#define PCTL_LINK	(1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE	(1 << 3)
#define PCTL_SLEEP	(1 << 2)
#define PCTL_WAKEUP(x)	((x) & 0x3)

/* DATA_FORMAT Bits */
#define SELF_TEST	(1 << 7)
#define SPI		(1 << 6)
#define INT_INVERT	(1 << 5)
#define FULL_RES	(1 << 3)
#define JUSTIFY		(1 << 2)
#define RANGE(x)	((x) & 0x3)
#define RANGE_PM_2g	0
#define RANGE_PM_4g	1
#define RANGE_PM_8g	2
#define RANGE_PM_16g	3

/*
 * Maximum value our axis may get in full res mode for the input device
 * (signed 13 bits)
 */
#define ADXL_FULLRES_MAX_VAL 4096

/*
 * Maximum value our axis may get in fixed res mode for the input device
 * (signed 10 bits)
 */
#define ADXL_FIXEDRES_MAX_VAL 512

/* FIFO_CTL Bits */
#define FIFO_MODE(x)	(((x) & 0x3) << 6)
#define FIFO_BYPASS	0
#define FIFO_FIFO	1
#define FIFO_STREAM	2
#define FIFO_TRIGGER	3
#define TRIGGER		(1 << 5)
#define SAMPLES(x)	((x) & 0x1F)

/* FIFO_STATUS Bits */
#define FIFO_TRIG	(1 << 7)
#define ENTRIES(x)	((x) & 0x3F)

/* TAP_SIGN Bits ADXL346 only */
#define XSIGN		(1 << 6)
#define YSIGN		(1 << 5)
#define ZSIGN		(1 << 4)
#define XTAP		(1 << 3)
#define YTAP		(1 << 2)
#define ZTAP		(1 << 1)

/* ORIENT_CONF ADXL346 only */
#define ORIENT_DEADZONE(x)	(((x) & 0x7) << 4)
#define ORIENT_DIVISOR(x)	((x) & 0x7)

/* ORIENT ADXL346 only */
#define ADXL346_2D_VALID		(1 << 6)
#define ADXL346_2D_ORIENT(x)		(((x) & 0x3) >> 4)
#define ADXL346_3D_VALID		(1 << 3)
#define ADXL346_3D_ORIENT(x)		((x) & 0x7)
#define ADXL346_2D_PORTRAIT_POS		0	/* +X */
#define ADXL346_2D_PORTRAIT_NEG		1	/* -X */
#define ADXL346_2D_LANDSCAPE_POS	2	/* +Y */
#define ADXL346_2D_LANDSCAPE_NEG	3	/* -Y */

#define ADXL346_3D_FRONT		3	/* +X */
#define ADXL346_3D_BACK			4	/* -X */
#define ADXL346_3D_RIGHT		2	/* +Y */
#define ADXL346_3D_LEFT			5	/* -Y */
#define ADXL346_3D_TOP			1	/* +Z */
#define ADXL346_3D_BOTTOM		6	/* -Z */







/*** ST ***/
#include <linux/lis3lv02d.h>
/* 2-byte registers */
#define LIS_DOUBLE_ID	0x3A /* LIS3LV02D[LQ] */
/* 1-byte registers */
#define LIS_SINGLE_ID	0x3B /* LIS[32]02DL and others */

enum lis3_reg {
//	WHO_AM_I	= 0x0F,
	OFFSET_X	= 0x16,
	OFFSET_Y	= 0x17,
	OFFSET_Z	= 0x18,
	GAIN_X		= 0x19,
	GAIN_Y		= 0x1A,
	GAIN_Z		= 0x1B,
	CTRL_REG1	= 0x20,
	CTRL_REG2	= 0x21,
	CTRL_REG3	= 0x22,
	HP_FILTER_RESET	= 0x23,
	STATUS_REG	= 0x27,
	OUTX_L		= 0x28,
	OUTX_H		= 0x29,
	OUTX		= 0x29,
	OUTY_L		= 0x2A,
	OUTY_H		= 0x2B,
	OUTY		= 0x2B,
	OUTZ_L		= 0x2C,
	OUTZ_H		= 0x2D,
	OUTZ		= 0x2D,
};
/*
enum lis302d_reg {
	FF_WU_CFG_1	= 0x30,
	FF_WU_SRC_1	= 0x31,
	FF_WU_THS_1	= 0x32,
	FF_WU_DURATION_1 = 0x33,
	FF_WU_CFG_2	= 0x34,
	FF_WU_SRC_2	= 0x35,
	FF_WU_THS_2	= 0x36,
	FF_WU_DURATION_2 = 0x37,
//	CLICK_CFG	= 0x38,
//	CLICK_SRC	= 0x39,
	CLICK_THSY_X	= 0x3B,
	CLICK_THSZ	= 0x3C,
//	CLICK_TIMELIMIT	= 0x3D,
	CLICK_LATENCY	= 0x3E,
	CLICK_WINDOW	= 0x3F,
};

enum lis3lv02d_reg {
//	FF_WU_CFG	= 0x30,
//	FF_WU_SRC	= 0x31,
	FF_WU_ACK	= 0x32,
	FF_WU_THS_L	= 0x34,
	FF_WU_THS_H	= 0x35,
//	FF_WU_DURATION	= 0x36,
	DD_CFG		= 0x38,
	DD_SRC		= 0x39,
	DD_ACK		= 0x3A,
	DD_THSI_L	= 0x3C,
	DD_THSI_H	= 0x3D,
	DD_THSE_L	= 0x3E,
	DD_THSE_H	= 0x3F,
};
*/
enum lis331dl_ctrl1{
	CTRL1_STM	= 0x08,
	CTRL1_STP	= 0x10,
	CTRL1_FS	= 0x20,
	CTRL1_PD	= 0x40,
	CTRL1_DR	= 0x80,
};
enum lis331dl_ctrl2 {
	CTRL2_HPCOEff1	= 0x01,
	CTRL2_HPCOEff2	= 0x02,
	CTRL2_HPFF_WU1	= 0x04,
	CTRL2_HPFF_WU2	= 0x08,
	CTRL2_FDS	= 0x10,
	CTRL2_RESERVE	= 0x20,
	CTRL2_RBOOT	= 0x40,
	CTRL2_SPI	= 0x80,
};
enum lis331dl_ctrl3 {
	CTRL3_L1CFG0	= 0x01,
	CTRL3_L1CFG1	= 0x02,
	CTRL3_L1CFG2	= 0x04,
	CTRL3_L2CFG0	= 0x08,
	CTRL3_L2CFG1	= 0x10,
	CTRL3_L2CFG2	= 0x20,
	CTRL2_PP_OD	= 0x40,
	CTRL2_IHL	= 0x80,
};

enum lis3lv02d_ctrl1 {
	CTRL1_Xen	= 0x01,
	CTRL1_Yen	= 0x02,
	CTRL1_Zen	= 0x04,
	CTRL1_ST	= 0x08,
	CTRL1_DF0	= 0x10,
	CTRL1_DF1	= 0x20,
	CTRL1_PD0	= 0x40,
	CTRL1_PD1	= 0x80,
};
enum lis3lv02d_ctrl2 {
	CTRL2_DAS	= 0x01,
	CTRL2_SIM	= 0x02,
	CTRL2_DRDY	= 0x04,
	CTRL2_IEN	= 0x08,
	CTRL2_BOOT	= 0x10,
	CTRL2_BLE	= 0x20,
	CTRL2_BDU	= 0x40, /* Block Data Update */
	CTRL2_FS	= 0x80, /* Full Scale selection */
};

enum lis302d_ctrl2 {
	HP_FF_WU2	= 0x08,
	HP_FF_WU1	= 0x04,
};

enum lis3lv02d_ctrl3 {
	CTRL3_CFS0	= 0x01,
	CTRL3_CFS1	= 0x02,
	CTRL3_FDS	= 0x10,
	CTRL3_HPFF	= 0x20,
	CTRL3_HPDD	= 0x40,
	CTRL3_ECK	= 0x80,
};

enum lis3lv02d_status_reg {
	STATUS_XDA	= 0x01,
	STATUS_YDA	= 0x02,
	STATUS_ZDA	= 0x04,
	STATUS_XYZDA	= 0x08,
	STATUS_XOR	= 0x10,
	STATUS_YOR	= 0x20,
	STATUS_ZOR	= 0x40,
	STATUS_XYZOR	= 0x80,
};

enum lis3lv02d_ff_wu_cfg {
	FF_WU_CFG_XLIE	= 0x01,
	FF_WU_CFG_XHIE	= 0x02,
	FF_WU_CFG_YLIE	= 0x04,
	FF_WU_CFG_YHIE	= 0x08,
	FF_WU_CFG_ZLIE	= 0x10,
	FF_WU_CFG_ZHIE	= 0x20,
	FF_WU_CFG_LIR	= 0x40,
	FF_WU_CFG_AOI	= 0x80,
};

enum lis3lv02d_ff_wu_src {
	FF_WU_SRC_XL	= 0x01,
	FF_WU_SRC_XH	= 0x02,
	FF_WU_SRC_YL	= 0x04,
	FF_WU_SRC_YH	= 0x08,
	FF_WU_SRC_ZL	= 0x10,
	FF_WU_SRC_ZH	= 0x20,
	FF_WU_SRC_IA	= 0x40,
};

enum lis3lv02d_dd_cfg {
	DD_CFG_XLIE	= 0x01,
	DD_CFG_XHIE	= 0x02,
	DD_CFG_YLIE	= 0x04,
	DD_CFG_YHIE	= 0x08,
	DD_CFG_ZLIE	= 0x10,
	DD_CFG_ZHIE	= 0x20,
	DD_CFG_LIR	= 0x40,
	DD_CFG_IEND	= 0x80,
};

enum lis3lv02d_dd_src {
	DD_SRC_XL	= 0x01,
	DD_SRC_XH	= 0x02,
	DD_SRC_YL	= 0x04,
	DD_SRC_YH	= 0x08,
	DD_SRC_ZL	= 0x10,
	DD_SRC_ZH	= 0x20,
	DD_SRC_IA	= 0x40,
};

struct axis_conversion {
	s8	x;
	s8	y;
	s8	z;
};

struct lis3lv02d {
	void			*bus_priv; /* used by the bus layer only */
	int (*init) (struct lis3lv02d *lis3);
	int (*write) (struct lis3lv02d *lis3, int reg, u8 val);
	int (*read) (struct lis3lv02d *lis3, int reg, u8 *ret);

	u8			whoami;    /* 3Ah: 2-byte registries, 3Bh: 1-byte registries */
	s16 (*read_data) (struct lis3lv02d *lis3, int reg);
	int			mdps_max_val;

	struct input_polled_dev	*idev;     /* input device */
	struct platform_device	*pdev;     /* platform device */
	atomic_t		count;     /* interrupt count after last read */
	int			xcalib;    /* calibrated null value for x */
	int			ycalib;    /* calibrated null value for y */
	int			zcalib;    /* calibrated null value for z */
	struct axis_conversion	ac;        /* hw -> logical axis */

	u32			irq;       /* IRQ number */
	struct fasync_struct	*async_queue; /* queue for the misc device */
	wait_queue_head_t	misc_wait; /* Wait queue for the misc device */
	unsigned long		misc_opened; /* bit0: whether the device is open */

	struct lis3lv02d_platform_data *pdata;	/* for passing board config */
};

int lis3lv02d_init_device(struct lis3lv02d *lis3);
int lis3lv02d_joystick_enable(void);
void lis3lv02d_joystick_disable(void);
void lis3lv02d_poweroff(struct lis3lv02d *lis3);
void lis3lv02d_poweron(struct lis3lv02d *lis3);
int lis3lv02d_remove_fs(struct lis3lv02d *lis3);

extern struct lis3lv02d lis3_dev;
