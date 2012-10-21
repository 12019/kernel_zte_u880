/*
 * A V4L2 driver for OmniVision OV3640 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <linux/i2c.h>
#include <mach/hardware.h>
#include <mach/camera.h>
#include <mach/mfp.h>

#include <linux/clk.h>
//#include <mach/pxa910.h>
#include "pxa910_camera.h"

MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov3640 sensors");
MODULE_LICENSE("GPL");

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */

#define QXGA_WIDTH	2048
#define QXGA_HEIGHT	1536
#define UXGA_WIDTH	1600
#define UXGA_HEIGHT     1200
#define XGA_WIDTH	1024
#define XGA_HEIGHT      768
#define D1_WIDTH	720
#define D1_HEIGHT      480
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/*
 * Our nominal (default) frame rate.
 */
#define OV3640_FRAME_RATE 30

/*
 * The 3640 sits on i2c with ID 0x42
 */
#define OV3640_I2C_ADDR 0x42

/* Registers */
#define REG_GAIN        0x00    /* Gain lower 8 bits (rest in vref) */
#define REG_BLUE        0x01    /* blue gain */
#define REG_RED         0x02    /* red gain */
#define REG_VREF        0x03    /* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1        0x04    /* Control 1 */
#define  COM1_CCIR656     0x40  /* CCIR656 enable */
#define REG_BAVE        0x05    /* U/B Average level */
#define REG_GbAVE       0x06    /* Y/Gb Average level */
#define REG_AECHH       0x07    /* AEC MS 5 bits */
#define REG_RAVE        0x08    /* V/R Average level */
#define REG_COM2        0x09    /* Control 2 */
#define  COM2_SSLEEP      0x10  /* Soft sleep mode */
#define REG_COM3        0x0c    /* Control 3 */
#define  COM3_SWAP        0x40    /* Byte swap */
#define  COM3_SCALEEN     0x08    /* Enable scaling */
#define  COM3_DCWEN       0x04    /* Enable downsamp/crop/window */
#define REG_COM4        0x0d    /* Control 4 */
#define REG_COM5        0x0e    /* All "reserved" */
#define REG_COM6        0x0f    /* Control 6 */
#define REG_AECH        0x10    /* More bits of AEC value */
#define   CLK_EXT         0x40    /* Use external clock directly */
#define   CLK_SCALE       0x3f    /* Mask for internal clock scale */
#define REG_COM7        0x12    /* Control 7 */
#define   COM7_RESET      0x80    /* Register reset */
#define   COM7_FMT_MASK   0x38
#define   COM7_FMT_VGA    0x00
#define   COM7_FMT_CIF    0x20    /* CIF format */
#define   COM7_FMT_QVGA   0x10    /* QVGA format */
#define   COM7_FMT_QCIF   0x08    /* QCIF format */
#define   COM7_RGB        0x04    /* bits 0 and 2 - RGB format */
#define   COM7_YUV        0x00    /* YUV */
#define   COM7_BAYER      0x01    /* Bayer format */
#define   COM7_PBAYER     0x05    /* "Processed bayer" */
#define REG_COM8        0x13    /* Control 8 */
#define   COM8_FASTAEC    0x80    /* Enable fast AGC/AEC */
#define   COM8_AECSTEP    0x40    /* Unlimited AEC step size */
#define   COM8_BFILT      0x20    /* Band filter enable */
#define   COM8_AGC        0x04    /* Auto gain enable */
#define   COM8_AWB        0x02    /* White balance enable */
#define   COM8_AEC        0x01    /* Auto exposure enable */
#define REG_COM9        0x14    /* Control 9  - gain ceiling */
#define REG_COM10       0x15    /* Control 10 */
#define   COM10_HSYNC     0x40    /* HSYNC instead of HREF */
#define   COM10_PCLK_HB   0x20    /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08    /* Reverse HREF */
#define   COM10_VS_LEAD   0x04    /* VSYNC on clock leading edge */
#define   COM10_VS_NEG    0x02    /* VSYNC negative */
#define   COM10_HS_NEG    0x01    /* HSYNC negative */
#define REG_HSTART      0x17    /* Horiz start high bits */
#define REG_HSTOP       0x18    /* Horiz stop high bits */
#define REG_VSTART      0x19    /* Vert start high bits */
#define REG_VSTOP       0x1a    /* Vert stop high bits */
#define REG_PSHFT       0x1b    /* Pixel delay after HREF */
#define REG_MIDH        0x1c    /* Manuf. ID high */
#define REG_MIDL        0x1d    /* Manuf. ID low */
#define REG_MVFP        0x1e    /* Mirror / vflip */
#define   MVFP_MIRROR     0x20    /* Mirror image */
#define   MVFP_FLIP       0x10    /* Vertical flip */

#define REG_AEW         0x24    /* AGC upper limit */
#define REG_AEB         0x25    /* AGC lower limit */
#define REG_VPT         0x26    /* AGC/AEC fast mode op region */
#define REG_HSYST       0x30    /* HSYNC rising edge delay */
#define REG_HSYEN       0x31    /* HSYNC falling edge delay */
#define REG_HREF        0x32    /* HREF pieces */
#define REG_TSLB        0x3a    /* lots of stuff */
#define   TSLB_YLAST      0x04    /* UYVY or VYUY - see com13 */
#define REG_COM11       0x3b    /* Control 11 */
#define   COM11_NIGHT     0x80    /* NIght mode enable */
#define   COM11_NMFR      0x60    /* Two bit NM frame rate */
#define   COM11_HZAUTO    0x10    /* Auto detect 50/60 Hz */
#define   COM11_50HZ      0x08    /* Manual 50Hz select */
#define   COM11_EXP       0x02
#define REG_COM12       0x3c    /* Control 12 */
#define   COM12_HREF      0x80    /* HREF always */
#define REG_COM13       0x3d    /* Control 13 */
#define   COM13_GAMMA     0x80    /* Gamma enable */
#define   COM13_UVSAT     0x40    /* UV saturation auto adjustment */
#define   COM13_UVSWAP    0x01    /* V before U - w/TSLB */
#define REG_COM14       0x3e    /* Control 14 */
#define   COM14_DCWEN     0x10    /* DCW/PCLK-scale enable */
#define REG_EDGE        0x3f    /* Edge enhancement factor */
#define REG_COM15       0x40    /* Control 15 */
#define   COM15_R10F0     0x00    /* Data range 10 to F0 */
#define   COM15_R01FE     0x80    /*            01 to FE */
#define   COM15_R00FF     0xc0    /*            00 to FF */
#define   COM15_RGB565    0x10    /* RGB565 output */
#define   COM15_RGB555    0x30    /* RGB555 output */
#define REG_COM16       0x41    /* Control 16 */
#define   COM16_AWBGAIN   0x08    /* AWB gain enable */
#define REG_COM17       0x42    /* Control 17 */
#define   COM17_AECWIN    0xc0    /* AEC window - must match COM4 */
#define   COM17_CBAR      0x08    /* DSP Color bar */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58

#define REG_MVFP0   0x3023    /* Vertical flip */
#define REG_MVFP1   0x307c    /* Vertical flip */
#define REG_MVFP2   0x3090    /* Vertical flip */
#define REG_BRIGHT0	0x3302	/* Brightness */
#define REG_BRIGHT1 0x3355
#define REG_BRIGHT2 0x3354
#define REG_BRIGHT3 0x335e
#define REG_SAT0 0x3358
#define REG_SAT1 0x3359
#define REG_HUE0	0x3340
#define REG_HUE1 	0x3341
#define REG_HUE2 	0x3342
#define REG_HUE3 	0x3343
#define REG_HUE4	0x3344
#define REG_HUE5 	0x3345
#define REG_HUE6 	0x3346
#define REG_HUE7 	0x3347
#define REG_HUE8 	0x3348
#define REG_HUE9 	0x3349
#define REG_HUE10 	0x333f
#define REG_CONTRAS0	0x335c	/* Contrast control */
#define REG_CONTRAS1	0x335d	/* Contrast control */
#define REG_WB0	0x332b	/* white balance */
#define REG_WB1 0x33a7
#define REG_WB2 0x33a8
#define REG_WB3 0x33a9
#define REG_EXPOSURE0	0x3047	/* exporure */
#define REG_EXPOSURE1 	0x3018
#define REG_EXPOSURE2 	0x3019
#define REG_EXPOSURE3 	0x301a


#define REG_GFIX	0x69	/* Fix gain control */

#define REG_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */

#define REG_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */

#define REG_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define REG_HAECC2	0xa0	/* Hist AEC/AGC control 2 */

#define REG_BD50MAX	0xa5	/* 50hz banding step limit */
#define REG_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define REG_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define REG_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define REG_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define REG_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define REG_BD60MAX	0xab	/* 60hz banding step limit */


/*for OV3640 porting*/
#define REG_CLKRC	0x3011
#define REG_PIDH        0x300a
#define REG_PIDL        0x300b
#define REG_SYS		0x3012
#define SYS_RESET	0x80
/*
 * Information we maintain about a known sensor.
 */
struct ov3640_format_struct;  /* coming later */
struct ov3640_info {
	struct ov3640_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */

struct regval_list {
	u16 reg_num;
	unsigned char value;
};


/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 * RGB656 and YUV422 come from OV; RGB444 is homebrewed.
 *
 * IMPORTANT RULE: the first entry must be for COM7, see ov3640_s_fmt for why.
 */

/*TODO - ov3640_fmt_yuv422_qxga can't work. configuration should be correct*/
//mipi supported // 10 fps
static struct regval_list ov3640_fmt_yuv422_qxga[] = {
	{0x304d, 0x45}, /*Rev2A */
	{0x3087, 0x16}, /*Rev2A */
	{0x30aa, 0x45}, /*Rev2A */
	{0x30b0, 0xff}, /*Rev2A */
	{0x30b1, 0xff},
	{0x30b2, 0x10},
	{0x30d7, 0x10}, /*Rev2A */

	{0x309e, 0x00}, /*terry */
	{0x3602, 0x26}, /*2a ;SOL/EOL on */
	{0x3603, 0x4D}, /*ecc  */
	{0x364c, 0x04}, /*ecc  */
	{0x360c, 0x12}, /*virtual channel 0 */
	{0x361e, 0x00},
	{0x361f, 0x11}, /*pclk_period, terry */
	{0x3633, 0x32}, /*terry, increase hs_prepare */
	{0x3629, 0x3c}, /*terry, increase clk_prepare */
	{0x300e, 0x39}, /*7.5fps terry */
	{0x300f, 0xa1}, /*terry */

	{0x3010, 0xa1}, /*high mipi spd, 81 ;terry */ //ENABLE_TWO_LANES
	//0x3010, 0x}a2, /*low down mipi spd, 81 ;terry */ //ENABLE_TWO_LANES

	{0x3011, 0x01},
	{0x304c, 0x81}, /*Rev2A */

	{0x3018, 0x38}, /*aec */
	{0x3019, 0x30}, /*06142007 */
	{0x301a, 0x61}, /*06142007 */
	{0x307d, 0x00}, /*aec isp 06142007 */
	{0x3087, 0x02}, /*06142007 */
	{0x3082, 0x20}, /*06142007 */

	{0x303c, 0x08}, /*aec weight */
	{0x303d, 0x18},
	{0x303e, 0x06},
	{0x303F, 0x0c},
	{0x3030, 0x62},   
	{0x3031, 0x26},
	{0x3032, 0xe6},
	{0x3033, 0x6e},
	{0x3034, 0xea},
	{0x3035, 0xae},
	{0x3036, 0xa6},
	{0x3037, 0x6a},

	{0x3015, 0x12}, /*07182007 8x gain, auto 1/2 */
	{0x3014, 0x04}, /*06142007 auto frame off */
	{0x3013, 0xf7}, /*07182007 */

	{0x3104, 0x02},
	{0x3105, 0xfd},
	{0x3106, 0x00},
	{0x3107, 0xff},
	{0x3308, 0xa5},
	{0x3316, 0xff},
	{0x3317, 0x00},
	{0x3087, 0x02},
	{0x3082, 0x20},
	{0x3300, 0x13},
	{0x3301, 0xd6},
	{0x3302, 0xef},

	{0x30b8, 0x20}, /*10 */
	{0x30b9, 0x17}, /*18 */
	{0x30ba, 0x04}, /*00 */
	{0x30bb, 0x08}, /*1f */

	{0x3100, 0x02}, /*YUV */ /*Format */
	{0x3304, 0x00},
	{0x3400, 0x00},
	{0x3404, 0x02}, /*terry - output format: YUV422PACKED(UYVY)*/

	{0x3020, 0x01}, /*QXGA *//*Size */
	{0x3021, 0x1d},
	{0x3022, 0x00},
	{0x3023, 0x0a},
	{0x3024, 0x08},
	{0x3025, 0x18},
	{0x3026, 0x06},
	{0x3027, 0x0c},
	{0x335f, 0x68},
	{0x3360, 0x18},
	{0x3361, 0x0c},
	{0x3362, 0x68},
	{0x3363, 0x08},
	{0x3364, 0x04},
	{0x3403, 0x42},
	{0x3088, 0x08},
	{0x3089, 0x00},
	{0x308a, 0x06},
	{0x308b, 0x00},

	{0x3507, 0x06},
	{0x350a, 0x4f},
	{0x3600, 0xc4},
	{0xffff, 0xff},
};

static struct regval_list ov3640_fmt_yuv422_d1[] = {
    {0x3012, 0x80},  
    {0x304d, 0x45},  /*Rev2A */
    {0x3087, 0x16},  /*Rev2A */
    {0x30aa, 0x45},  /*Rev2A */
    {0x30b0, 0xff},  /*Rev2A */
    {0x30b1, 0xff}, 
    {0x30b2, 0x10}, 
    {0x30d7, 0x10},  /*Rev2A */

    {0x309e, 0x00},  /*terry */
    {0x3602, 0x26},  /*2a ;SOL/EOL on */
    {0x3603, 0x4D},  /*ecc    */
    {0x364c, 0x04},  /*ecc    */
    {0x360c, 0x12},  /*virtual channel 0 */
    {0x361e, 0x00}, 
    {0x361f, 0x11},  /*pclk_period, terry */
    {0x3633, 0x32},  /*terry, increase hs_prepare */
    {0x3629, 0x3c},  /*terry, increase clk_prepare */
    {0x300e, 0x33},  /*15fps terry */
    {0x300f, 0xa1},  /*terry */

    {0x3010, 0xa2},  /*mipi spd, 81 ;terry */

    {0x3011, 0x01},
    {0x304c, 0x82},  /*Rev2A */

    {0x3018, 0x48},  /*aec *//*0x3018~0x301a, exposure*/
    {0x3019, 0x40},  /*06142007 */
    {0x301a, 0x81},  /*06142007 */
    {0x307d, 0x00},  /*aec isp 06142007 */
    {0x3087, 0x02},  /*06142007 */
    {0x3082, 0x20},  /*06142007 */

    {0x303c, 0x08},  /*aec weight */
    {0x303d, 0x18}, 
    {0x303e, 0x06}, 
    {0x303F, 0x0c}, 
    {0x3030, 0x62}, 
    {0x3031, 0x26}, 
    {0x3032, 0xe6}, 
    {0x3033, 0x6e}, 
    {0x3034, 0xea}, 
    {0x3035, 0xae}, 
    {0x3036, 0xa6}, 
    {0x3037, 0x6a}, 

    {0x3015, 0x12},  /*07182007 8x gain, auto 1/2 */
    {0x3014, 0x04},  /*06142007 auto frame off */
    {0x3013, 0xf7},  /*07182007 */

    {0x3104, 0x02}, 
    {0x3105, 0xfd}, 
    {0x3106, 0x00}, 
    {0x3107, 0xff}, 
    {0x3308, 0xa5}, 
    {0x3316, 0xff}, 
    {0x3317, 0x00}, 
    {0x3087, 0x02}, 
    {0x3082, 0x20}, 
    {0x3300, 0x13}, 
    {0x3301, 0xd6}, 
    {0x3302, 0xef}, 

    {0x30b8, 0x20},  /*10 */
    {0x30b9, 0x17},  /*18 */
    {0x30ba, 0x04},  /*00 */
    {0x30bb, 0x08},  /*1f */

    {0x3100, 0x02},  /*YUV */
    {0x3304, 0x00}, 
    {0x3400, 0x00}, 
    {0x3404, 0x02},  /*terry - output format: YUV422PACKED(UYVY)*/

    {0x3302, 0xef},  /* */
    {0x3020, 0x01}, 
    {0x3021, 0x1d}, 
    {0x3022, 0x00}, 
    {0x3023, 0x0a}, 
    {0x3024, 0x08}, 
    {0x3025, 0x18}, 
    {0x3026, 0x06}, 
    {0x3027, 0x0c}, 
    {0x335f, 0x68}, 
    {0x3360, 0x18}, 
    {0x3361, 0x0c}, 
    {0x3362, 0x34}, 
    {0x3363, 0x08}, 
    {0x3364, 0x06}, 
    {0x3403, 0x42}, 
    {0x3088, 0x02},
    {0x3089, 0xD0}, 
    {0x308a, 0x01}, 
    {0x308b, 0xE0}, 

    {0x3507, 0x06}, 
    {0x350a, 0x4f}, 
    {0x3600, 0xc4}, 
    //
    //gamma
    {0x331b, 0x08 },
    {0x331c, 0x16 },
    {0x331d, 0x2d },
    {0x331e, 0x54 },
    {0x331f, 0x66 },
    {0x3320, 0x73 },
    {0x3321, 0x80 },
    {0x3322, 0x8c },
    {0x3323, 0x95 },
    {0x3324, 0x9d },
    {0x3325, 0xac },
    {0x3326, 0xb8 },
    {0x3327, 0xcc },
    {0x3328, 0xdd },
    {0x3329, 0xee },
    {0x332a, 0x1d },
    //color matrix
    {0x3340, 0x20 },
    {0x3341, 0x64 },
    {0x3342, 0x08 },
    {0x3343, 0x30 },
    {0x3344, 0x90 },
    {0x3345, 0xc0 },
    {0x3346, 0xa0 },
    {0x3347, 0x98 },
    {0x3348, 0x08 },
    {0x3349, 0x98 },
    {0x333f, 0x02 },


    {0x3012, 0x10},
    {0x3023, 0x06},
    {0x3026, 0x03},
    {0x3027, 0x04},
    {0x302a, 0x03},
    {0x302b, 0x10},
    {0x3075, 0x24},
    {0x300d, 0x01},
    {0x30d7, 0x80},
    {0x3069, 0x04},
    {0x303e, 0x00},
    {0x303f, 0xc0},
    {0x3302, 0xef},
    {0x335f, 0x34},
    {0x3360, 0x0c},
    {0x3361, 0x04},
    {0x3362, 0x34},
    {0x3363, 0x08},
    {0x3364, 0x04},
    {0x3403, 0x42},
    {0x3088, 0x02},
    {0x3089, 0xD0},
    {0x308a, 0x01},
    {0x308b, 0xE0},

    {0xffff, 0x00ff}        
};

static struct regval_list ov3640_fmt_yuv422_vga[] = {

	{0x3012, 0x80},
	{0x304d, 0x45}, // /*Rev2A */
	{0x3087, 0x16}, // /*Rev2A */
	{0x30aa, 0x45}, // /*Rev2A */
	{0x30b0, 0xff}, // /*Rev2A */
	{0x30b1, 0xff}, //
	{0x30b2, 0x10}, //
	{0x30d7, 0x10}, // /*Rev2A */

	{0x309e, 0x00}, // /*terry */
	{0x3602, 0x26}, // /*2a ;SOL/EOL on */
	{0x3603, 0x4D}, // /*ecc	*/
	{0x364c, 0x04}, // /*ecc	*/
	{0x360c, 0x12}, // /*virtual channel 0 */
	{0x361e, 0x00}, //
	{0x361f, 0x11}, // /*pclk_period, terry */
	{0x3633, 0x32}, // /*terry, increase hs_prepare */
	{0x3629, 0x3c}, // /*terry, increase clk_prepare */
	{0x300e, 0x39}, // /*15fps terry */
	{0x300f, 0xa1}, // /*terry */

	{0x3010, 0xa2}, // /*mipi spd, 81 ;terry */

	{0x3011, 0x00}, //
	{0x304c, 0x84}, // /*Rev2A */

	{0x3018, 0x48}, // /*aec *//*0x3018~0x301a, exposure*/
	{0x3019, 0x40}, // /*06142007 */
	{0x301a, 0x81}, // /*06142007 */
	{0x307d, 0x00}, // /*aec isp 06142007 */
	{0x3087, 0x02}, // /*06142007 */
	{0x3082, 0x20}, // /*06142007 */

	{0x303c, 0x08}, // /*aec weight */
	{0x303d, 0x18}, //
	{0x303e, 0x06}, //
	{0x303F, 0x0c}, //
	{0x3030, 0x62}, //
	{0x3031, 0x26}, //
	{0x3032, 0xe6}, //
	{0x3033, 0x6e}, //
	{0x3034, 0xea}, //
	{0x3035, 0xae}, //
	{0x3036, 0xa6}, //
	{0x3037, 0x6a}, //

	{0x3015, 0x12}, // /*07182007 8x gain, auto 1/2 */
	{0x3014, 0x04}, // /*06142007 auto frame off */
	{0x3013, 0xf7}, // /*07182007 */

	{0x3104, 0x02}, //
	{0x3105, 0xfd}, //
	{0x3106, 0x00}, //
	{0x3107, 0xff}, //
	{0x3308, 0xa5}, //
	{0x3316, 0xff}, //
	{0x3317, 0x00}, //
	{0x3087, 0x02}, //
	{0x3082, 0x20}, //
	{0x3300, 0x13}, //
	{0x3301, 0xd6}, //
	{0x3302, 0xef}, //

	{0x30b8, 0x20}, // /*10 */
	{0x30b9, 0x17}, // /*18 */
	{0x30ba, 0x04}, // /*00 */
	{0x30bb, 0x08}, // /*1f */

	{0x3100, 0x02}, // /*YUV */
	{0x3304, 0x00}, //
	{0x3400, 0x00}, //
	{0x3404, 0x02}, // /*terry - output format: YUV422PACKED(UYVY)*/

	{0x3302, 0xef}, // /* */
	{0x3020, 0x01}, //
	{0x3021, 0x1d}, //
	{0x3022, 0x00}, //
	{0x3023, 0x0a}, //
	{0x3024, 0x08}, //
	{0x3025, 0x18}, //
	{0x3026, 0x06}, //
	{0x3027, 0x0c}, //
	{0x335f, 0x68}, //
	{0x3360, 0x18}, //
	{0x3361, 0x0c}, //
	{0x3362, 0x12}, //
	{0x3363, 0x88}, //
	{0x3364, 0xe4}, //
	{0x3403, 0x42}, //
	{0x3088, 0x02}, //
	{0x3089, 0x80}, //
	{0x308a, 0x01}, //
	{0x308b, 0xe0}, //

	{0x3507, 0x06}, //
	{0x350a, 0x4f}, //
	{0x3600, 0xc4}, //

	//gamma
	{0x331b, 0x08 },
	{0x331c, 0x16 },
	{0x331d, 0x2d },
	{0x331e, 0x54 },
	{0x331f, 0x66 },
	{0x3320, 0x73 },
	{0x3321, 0x80 },
	{0x3322, 0x8c },
	{0x3323, 0x95 },
	{0x3324, 0x9d },
	{0x3325, 0xac },
	{0x3326, 0xb8 },
	{0x3327, 0xcc },
	{0x3328, 0xdd },
	{0x3329, 0xee },
	{0x332a, 0x1d },
	//color matrix
	{0x3340, 0x20 },
	{0x3341, 0x64 },
	{0x3342, 0x08 },
	{0x3343, 0x30 },
	{0x3344, 0x90 },
	{0x3345, 0xc0 },
	{0x3346, 0xa0 },
	{0x3347, 0x98 },
	{0x3348, 0x08 },
	{0x3349, 0x98 },
	{0x333f, 0x02 },

	{0xffff, 0x00ff}        /* End of file marker (0xFFFF)*/
};

static struct regval_list ov3640_fmt_yuv422_qvga[] = {
	{0x3002, 0x06 }, {0x3003, 0x1F }, {0x3001, 0x12 }, {0x304d, 0x45 },
	{0x30aa, 0x45 }, {0x30B0, 0xff }, {0x30B1, 0xff }, {0x30B2, 0x10 },
	{0x30d7, 0x10 }, {0x3047, 0x00 }, {0x3018, 0x60 }, {0x3019, 0x58 },
	{0x301A, 0xa1 }, {0x3087, 0x02 }, {0x3082, 0x20 }, {0x303C, 0x08 },
	{0x303d, 0x18 }, {0x303e, 0x06 },
	{0x303f, 0x0c }, {0x3030, 0x62 }, {0x3031, 0x26 }, {0x3032, 0xe6 },
	{0x3033, 0x6e }, {0x3034, 0xea }, {0x3035, 0xae }, {0x3036, 0xa6 },
	{0x3037, 0x6a }, {0x3015, 0x12 }, {0x3013, 0xfd }, {0x3104, 0x02 },
	{0x3105, 0xfd }, {0x3106, 0x00 }, {0x3107, 0xff }, {0x3308, 0xa5 },
	{0x3316, 0xff }, {0x3317, 0x00 }, {0x3087, 0x02 }, {0x3082, 0x20 },
	{0x3300, 0x13 }, {0x3301, 0xd6 }, {0x3302, 0xef }, {0x30B8, 0x20 },
	{0x30B9, 0x17 }, {0x30BA, 0x04 }, {0x30BB, 0x08 }, {0x3507, 0x06 },
	{0x350a, 0x4f }, {0x3600, 0xc4 }, {0x332B, 0x00 }, {0x332D, 0x45 },
	{0x332D, 0x60 }, {0x332F, 0x03 },
	{0x3100, 0x02 }, {0x3304, 0xfc }, {0x3400, 0x00 }, {0x3404, 0x02 }, /* YUV422 */
	{0x3601, 0x01 }, {0x302a, 0x06 }, {0x302b, 0x20 },
	{0x300E, 0x32 }, {0x300F, 0x21 }, {0x3010, 0x21 }, {0x3011, 0x01 }, /* QXGA PLL setting*/
	{0x304c, 0x81 },
	{0x3602, 0x22 }, {0x361E, 0x00 }, {0x3622, 0x18 }, {0x3623, 0x69 }, /* CSI setting */
	{0x3626, 0x00 }, {0x3627, 0xf0 }, {0x3628, 0x00 }, {0x3629, 0x26 },
	{0x362A, 0x00 }, {0x362B, 0x5f }, {0x362C, 0xd0 }, {0x362D, 0x3c },
	{0x3632, 0x10 }, {0x3633, 0x28 }, {0x3603, 0x4d }, {0x364C, 0x04 },
	{0x309e, 0x00 },
	{0x3020, 0x01 }, {0x3021, 0x1d }, {0x3022, 0x00 }, {0x3023, 0x0a }, /* crop window setting*/
	{0x3024, 0x08 }, {0x3025, 0x18 }, {0x3026, 0x06 }, {0x3027, 0x0c },
	{0x335f, 0x68 }, {0x3360, 0x18 }, {0x3361, 0x0c },
	{0x3362, 0x01 }, {0x3363, 0x48 }, {0x3364, 0xf4 }, {0x3403, 0x42 },  /* QVGA */
	{0x3088, 0x01 }, {0x3089, 0x40 }, {0x308a, 0x00 }, {0x308b, 0xf0 },
	{0x3355, 0x04 }, {0x3354, 0x01 }, {0x335e, 0x28 },      /* brightness */
	{0x3355, 0x04 }, {0x335c, 0x20 }, {0x335d, 0x20 },      /* contrast */
	{0xffff, 0xff },
};

/*TODO - ov3640_fmt_yuv422_qcif can't work*/
static struct regval_list ov3640_fmt_yuv422_qcif[] = {
	//QVGA
	{0x3012, 0x80}, //
	{0x304d, 0x45}, // /*Rev2A */
	{0x3087, 0x16}, // /*Rev2A */
	{0x30aa, 0x45}, // /*Rev2A */
	{0x30b0, 0xff}, // /*Rev2A */
	{0x30b1, 0xff}, //
	{0x30b2, 0x10}, //
	{0x30d7, 0x10}, // /*Rev2A */

	{0x309e, 0x00}, // /*terry */
	{0x3602, 0x26}, // /*2a ;SOL/EOL on */
	{0x3603, 0x4D}, // /*ecc  */
	{0x364c, 0x04}, // /*ecc  */
	{0x360c, 0x12}, // /*virtual channel 0 */
	{0x361e, 0x00}, //
	{0x361f, 0x11}, // /*pclk_period, terry */
	{0x3633, 0x32}, // /*terry, increase hs_prepare */
	{0x3629, 0x3c}, // /*terry, increase clk_prepare */
	{0x300e, 0x39}, // /*15fps terry */
	{0x300f, 0xa1}, // /*terry */

	{0x3010, 0xa2}, // /*mipi spd, 81 ;terry */

	{0x3011, 0x00}, //
	{0x304c, 0x84}, // /*Rev2A */

	{0x3018, 0x38}, // /*aec */
	{0x3019, 0x30}, // /*06142007 */
	{0x301a, 0x61}, // /*06142007 */
	{0x307d, 0x00}, // /*aec isp 06142007 */
	{0x3087, 0x02}, // /*06142007 */
	{0x3082, 0x20}, // /*06142007 */

	{0x303c, 0x08}, // /*aec weight */
	{0x303d, 0x18}, //
	{0x303e, 0x06}, //
	{0x303F, 0x0c}, //
	{0x3030, 0x62}, //
	{0x3031, 0x26}, //
	{0x3032, 0xe6}, //
	{0x3033, 0x6e}, //
	{0x3034, 0xea}, //
	{0x3035, 0xae}, //
	{0x3036, 0xa6}, //
	{0x3037, 0x6a}, //

	{0x3015, 0x12}, // /*07182007 8x gain, auto 1/2 */
	{0x3014, 0x04}, // /*06142007 auto frame off */
	{0x3013, 0xf7}, // /*07182007 */

	{0x3104, 0x02}, //
	{0x3105, 0xfd}, //
	{0x3106, 0x00}, //
	{0x3107, 0xff}, //
	{0x3308, 0xa5}, //
	{0x3316, 0xff}, //
	{0x3317, 0x00}, //
	{0x3087, 0x02}, //
	{0x3082, 0x20}, //
	{0x3300, 0x13}, //
	{0x3301, 0xd6}, //
	{0x3302, 0xef}, //

	{0x30b8, 0x20}, // /*10 */
	{0x30b9, 0x17}, // /*18 */
	{0x30ba, 0x04}, // /*00 */
	{0x30bb, 0x08}, // /*1f */

	{0x3100, 0x02}, // /*YUV */
	{0x3304, 0x00}, //
	{0x3400, 0x00}, //
	{0x3404, 0x02}, // /*terry - output format: YUV422PACKED(UYVY)*/

	{0x3302, 0xef}, // /* */
	{0x3020, 0x01}, //
	{0x3021, 0x1d}, //
	{0x3022, 0x00}, //
	{0x3023, 0x0a}, //
	{0x3024, 0x08}, //
	{0x3025, 0x18}, //
	{0x3026, 0x06}, //
	{0x3027, 0x0c}, //

	{0x335f, 0x68}, //
	{0x3360, 0x18}, //
	{0x3361, 0x0c}, //

	{0x3362, 0x00}, // QVGA
	{0x3363, 0xb8}, // QVGA
	{0x3364, 0x94}, // QVGA

	{0x3403, 0x42}, // QVGA

	{0x3088, 0x00}, // QVGA
	{0x3089, 0xb0}, // QVGA
	{0x308a, 0x00}, // QVGA
	{0x308b, 0x90}, // QVGA

	{0x3507, 0x06}, //
	{0x350a, 0x4f}, //
	{0x3600, 0xc4}, //
	{0xffff, 0xff},

};

static struct regval_list ov3640_fmt_jpeg_vga[] = {
	{0x3002, 0x06 }, {0x3003, 0x1F }, {0x3001, 0x12 }, {0x304d, 0x45 },
	{0x30aa, 0x45 }, {0x30B0, 0xff }, {0x30B1, 0xff }, {0x30B2, 0x10 },
	{0x30d7, 0x10 }, {0x3047, 0x00 }, {0x3018, 0x60 }, {0x3019, 0x58 },
	{0x301A, 0xa1 }, {0x303C, 0x08 }, {0x303d, 0x18 }, {0x303e, 0x06 },
	{0x303f, 0x0c }, {0x3030, 0x62 }, {0x3031, 0x26 }, {0x3032, 0xe6 },
	{0x3033, 0x6e }, {0x3034, 0xea }, {0x3035, 0xae }, {0x3036, 0xa6 },
	{0x3037, 0x6a }, {0x3015, 0x12 }, {0x3013, 0xfd }, {0x3104, 0x02 },
	{0x3105, 0xfd }, {0x3106, 0x00 }, {0x3107, 0xff }, {0x3308, 0xa5 },
	{0x3316, 0xff }, {0x3317, 0x00 }, {0x3087, 0x02 }, {0x3082, 0x20 },
	{0x3300, 0x13 }, {0x3301, 0xd6 }, {0x3302, 0xef }, {0x30B8, 0x20 },
	{0x30B9, 0x17 }, {0x30BA, 0x04 }, {0x30BB, 0x08 }, {0x3507, 0x06 },
	{0x350a, 0x4f }, {0x3600, 0xc4 }, {0x332B, 0x00 }, {0x332D, 0x45 },
	{0x332D, 0x60 }, {0x332F, 0x03 },
	{0x3100, 0x32 }, {0x3304, 0xfc }, {0x3404, 0x22 }, {0x3500, 0x00 }, /* JPEG */
	{0x3610, 0x0c },
	{0x300E, 0x32 }, {0x300F, 0x21 }, {0x3010, 0x21 }, {0x3011, 0x01 }, /* QXGA PLL setting*/
	{0x304c, 0x81 },
	{0x3602, 0x22 }, {0x361E, 0x00 }, {0x3622, 0x18 }, {0x3623, 0x69 }, /* CSI setting */
	{0x3626, 0x00 }, {0x3627, 0xf0 }, {0x3628, 0x00 }, {0x3629, 0x26 },
	{0x362A, 0x00 }, {0x362B, 0x5f }, {0x362C, 0xd0 }, {0x362D, 0x3c },
	{0x3632, 0x10 }, {0x3633, 0x28 }, {0x3603, 0x4d }, {0x364C, 0x04 },
	{0x309e, 0x00 },
	{0x3020, 0x01 }, {0x3021, 0x1d }, {0x3022, 0x00 }, {0x3023, 0x0a }, /* crop window setting*/
	{0x3024, 0x08 }, {0x3025, 0x18 }, {0x3026, 0x06 }, {0x3027, 0x0c },
	{0x335f, 0x68 }, {0x3360, 0x18 }, {0x3361, 0x0c },
	{0x3362, 0x12 }, {0x3363, 0x88 }, {0x3364, 0xe4 }, {0x3403, 0x42 },  /* VGA */
	{0x3088, 0x02 }, {0x3089, 0x80 }, {0x308a, 0x01 }, {0x308b, 0xe0 },

	{0x3355, 0x04 }, {0x3354, 0x01 }, {0x335e, 0x28 },      /* brightness */
	{0x3355, 0x04 }, {0x335c, 0x20 }, {0x335d, 0x20 },      /* contrast */
	{0xffff, 0x00ff}        /* End of file marker (0xFFFF)*/

};

/*TODO - ov3640_fmt_jpeg_qxga can't work*/
static struct regval_list ov3640_fmt_jpeg_qxga[] = {
	//2048x1536 JPG@15fps

	{0x3012, 0x80},
	{0x304d, 0x45}, //Rev2A
	{0x30a7, 0x5e}, //Rev2C mi
	{0x3087, 0x16}, //Rev2A
	{0x309C, 0x1a}, //Rev2C 
	{0x30a2, 0xe4}, //Rev2C 
	{0x30aa, 0x42}, //Rev2C 
	{0x30b0, 0xff}, //Rev2A
	{0x30b1, 0xff},
	{0x30b2, 0x10},
	{0x30d7, 0x10},//Rev2A

	{0x309e, 0x00}, //terry
	{0x3602, 0x26}, //2a //SOL/EOL on
	{0x3603, 0x4D}, //ecc 
	{0x364c, 0x04}, //ecc 
	{0x360c, 0x12}, //virtual channel 0
	{0x361e, 0x00},
	{0x361f, 0x11}, //pclk_period, terry
	{0x3633, 0x32}, //terry, increase hs_prepare
	{0x3629, 0x3c}, //terry, increase clk_prepare
	{0x300e, 0x39}, //15fps terry
	{0x300f, 0xa1}, //terry
	{0x3010, 0xa2}, //high mipi spd, 2lane
	{0x3011, 0x00},
	{0x304c, 0x84}, //Rev2A

	{0x30d9, 0x0d}, //Rev2C
	{0x30db, 0x08}, //Rev2C
	{0x3016, 0x82}, //Rev2C

	{0x3018, 0x38}, //aec
	{0x3019, 0x30}, //06142007
	{0x301a, 0x61}, //06142007
	{0x307d, 0x00}, //aec isp 06142007
	{0x3087, 0x02}, //06142007
	{0x3082, 0x20}, //06142007

	{0x303c, 0x08}, //aec weight
	{0x303d, 0x18},
	{0x303e, 0x06},
	{0x303F, 0x0c},
	{0x3030, 0x62},	
	{0x3031, 0x26},
	{0x3032, 0xe6},
	{0x3033, 0x6e},
	{0x3034, 0xea},
	{0x3035, 0xae},
	{0x3036, 0xa6},
	{0x3037, 0x6a},

	{0x3015, 0x12}, //07182007 8x gain, auto 1/2
	{0x3014, 0x04}, //06142007 auto frame off
	{0x3013, 0xf7}, //07182007

	{0x3104, 0x02},
	{0x3105, 0xfd},
	{0x3106, 0x00},
	{0x3107, 0xff},

	{0x3300, 0x13},
	{0x3301, 0xde},
	{0x3302, 0xef},

	{0x3308, 0xa5},
	{0x3316, 0xff},
	{0x3317, 0x00},
	{0x3312, 0x26},
	{0x3314, 0x42},
	{0x3313, 0x2b},
	{0x3315, 0x42},
	{0x3310, 0xd0},
	{0x3311, 0xbd},
	{0x330c, 0x18},
	{0x330d, 0x18},
	{0x330e, 0x56},
	{0x330f, 0x5c},
	{0x330b, 0x1c},
	{0x3306, 0x5c},
	{0x3307, 0x11},

	{0x336a, 0x52}, //052207
	{0x3370, 0x46},
	{0x3376, 0x38},

	{0x30b8, 0x20}, //10
	{0x30b9, 0x17}, //18
	{0x30ba, 0x04}, //00
	{0x30bb, 0x08}, //1f

	//Format
	{0x3100, 0x32}, //JPG
	{0x3304, 0x00},
	{0x3400, 0x02},
	{0x3404, 0x22},
	{0x3500, 0x00}, //
	{0x3610, 0x80}, //0c

	//Size QXGA
	{0x3302, 0xcf},
	{0x3088, 0x08},
	{0x3089, 0x00},
	{0x308a, 0x06},
	{0x308b, 0x00},

	{0x3507, 0x06},
	{0x350a, 0x4f},
	{0x3600, 0xc4},

	{0xffff, 0xff},
};

static struct regval_list ov3640_fmt_rgb565[] = {
	{ REG_COM7, COM7_RGB },	/* Selects RGB mode */
	{ REG_RGB444, 0 },	/* No RGB444 please */
	{ REG_COM1, 0x0 },
	{ REG_COM15, COM15_RGB565 },
	{ REG_COM9, 0x38 }, 	/* 16x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0xb3 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0xb3 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x3d }, 	/* "matrix coefficient 4" */
	{ 0x53, 0xa7 }, 	/* "matrix coefficient 5" */
	{ 0x54, 0xe4 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT },
	{ 0xff, 0xff },
};

static struct regval_list ov3640_fmt_rgb444[] = {
	{ REG_COM7, COM7_RGB },	/* Selects RGB mode */
	{ REG_RGB444, R444_ENABLE },	/* Enable xxxxrrrr ggggbbbb */
	{ REG_COM1, 0x40 },	/* Magic reserved bit */
	{ REG_COM15, COM15_R01FE|COM15_RGB565 }, /* Data range needed? */
	{ REG_COM9, 0x38 }, 	/* 16x gain ceiling; 0x8 is reserved bit */
	{ 0x4f, 0xb3 }, 	/* "matrix coefficient 1" */
	{ 0x50, 0xb3 }, 	/* "matrix coefficient 2" */
	{ 0x51, 0    },		/* vb */
	{ 0x52, 0x3d }, 	/* "matrix coefficient 4" */
	{ 0x53, 0xa7 }, 	/* "matrix coefficient 5" */
	{ 0x54, 0xe4 }, 	/* "matrix coefficient 6" */
	{ REG_COM13, COM13_GAMMA|COM13_UVSAT|0x2 },  /* Magic rsvd bit */
	{ 0xff, 0xff },
};

static struct regval_list ov3640_fmt_raw[] = {
	{ REG_COM7, COM7_BAYER },
	{ REG_COM13, 0x08 }, /* No gamma, magic rsvd bit */
	{ REG_COM16, 0x3d }, /* Edge enhancement, denoise */
	{ REG_REG76, 0xe1 }, /* Pix correction, magic rsvd */
	{ 0xff, 0xff },
};

/*
 * Low-level register I/O.
 */
/*issue that OV sensor must write then read. 3640 register is 16bit!!!*/
static int ov3640_read(struct i2c_client *c, u16 reg,
		unsigned char *value)
{
	u8 data;
	u8 address[2];
	address[0] = reg>>8;
	address[1] = reg;         
	i2c_smbus_write_byte_data(c,address[0],address[1]);
	data = i2c_smbus_read_byte(c);
	*value = data;
	return 0;
}


static int ov3640_write(struct i2c_client *c, u16 reg,
		unsigned char value)
{
	u8 data[3];
	data[0] = reg>>8;
	data[1] = reg;
	data[2]=  value;
	i2c_master_send(c, data, 3);
	if (reg == REG_SYS && (value & SYS_RESET))
		msleep(2);  /* Wait for reset to run */
	return 0;
}


/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov3640_write_array(struct i2c_client *c, struct regval_list *vals)
{
	int i = 0;
	while (vals->reg_num != 0xffff || vals->value != 0xff) {
		int ret = ov3640_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		if (i == 0)
			mdelay(5);
		i++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */
static void ov3640_reset(struct i2c_client *client)
{
	ov3640_write(client, REG_SYS, SYS_RESET);
	msleep(1);
}

static int ov3640_detect(struct i2c_client *client)
{
	unsigned char v;
	int ret;

	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov3640_read(client, REG_PIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x36)
		return -ENODEV;
	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct ov3640_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
	int bpp;   /* bits per pixel */
} ov3640_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= ov3640_fmt_yuv422_qxga,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },	//TODO
		.bpp		= 16,
	},
	{
		.desc		= "YUYV422 planar",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.regs 		= ov3640_fmt_yuv422_qxga,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 16,
	},
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.regs           = ov3640_fmt_yuv422_qxga,
		.cmatrix        = { 128, -128, 0, -34, -94, 128 },
		.bpp            = 12,
	},
	{
		.desc           = "JFIF JPEG",
		.pixelformat    = V4L2_PIX_FMT_JPEG,
		.regs           = ov3640_fmt_jpeg_qxga,
		.cmatrix        = { 128, -128, 0, -34, -94, 128 },
		.bpp            = 16,
	},
	{
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.regs		= ov3640_fmt_rgb444,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 16,
	},
	{
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.regs		= ov3640_fmt_rgb565,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 16,
	},
	{
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.regs 		= ov3640_fmt_raw,
		.cmatrix	= { 0, 0, 0, 0, 0, 0 },
		.bpp		= 8,
	},
};
#define N_OV3640_FMTS ARRAY_SIZE(ov3640_formats)

/*TODO - also can use ccic size register 0x34 to do same thing for cropping...anyway, sensor doing it is better?
  0x3020~0x3027*/
static struct ov3640_win_size {
	int	width;
	int	height;
	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	/* h/vref stuff */
} ov3640_win_sizes[] = {
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.com7_bit	= COM7_FMT_VGA, /* see comment above */
		.hstart		= 456,		/* Empirically determined */
		.hstop		=  24,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
		//		.regs 		= ov3640_qcif_regs,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.com7_bit	= COM7_FMT_QVGA,
		.hstart		= 164,		/* Empirically determined */
		.hstop		=  20,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
	},
#if 0
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
		.com7_bit	= COM7_FMT_CIF,
		.hstart		= 170,		/* Empirically determined */
		.hstop		=  90,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
	},
#endif
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.com7_bit	= COM7_FMT_VGA,
		.hstart		= 158,		/* These values from */
		.hstop		=  14,		/* Omnivision */
		.vstart		=  10,
		.vstop		= 490,
		.regs 		= NULL,
	},
	/* D1 */
	{
		.width          = D1_WIDTH,
		.height         = D1_HEIGHT,
		.com7_bit       = 0,
		.hstart         = 158,          /* These values from */
		.hstop          =  14,          /* Omnivision */
		.vstart         =  10,
		.vstop          = 490,
		.regs           = NULL,
	},
	/* QXGA */
	{
		.width          = QXGA_WIDTH,
		.height         = QXGA_HEIGHT,
		.com7_bit       = 0,
		.hstart         = 158,          /* These values from */
		.hstop          =  14,          /* Omnivision */
		.vstart         =  10,
		.vstop          = 490,
		.regs           = NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(ov3640_win_sizes))

#if 0
/*
 * Store a set of start/stop values into the camera.	//TODO - not used for 3640?
 */
static int ov3640_set_hw(struct i2c_client *client, int hstart, int hstop,
		int vstart, int vstop)
{
	int ret;
	unsigned char v;
	/*
	 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
	 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
	 * a mystery "edge offset" value in the top two bits of href.
	 */
	ret =  ov3640_write(client, REG_HSTART, (hstart >> 3) & 0xff);
	ret += ov3640_write(client, REG_HSTOP, (hstop >> 3) & 0xff);
	ret += ov3640_read(client, REG_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += ov3640_write(client, REG_HREF, v);
	/*
	 * Vertical: similar arrangement, but only 10 bits.
	 */
	ret += ov3640_write(client, REG_VSTART, (vstart >> 2) & 0xff);
	ret += ov3640_write(client, REG_VSTOP, (vstop >> 2) & 0xff);
	ret += ov3640_read(client, REG_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += ov3640_write(client, REG_VREF, v);
	return ret;
}
#endif

static int ov3640_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct ov3640_format_struct *ofmt;

	if (fmt->index >= N_OV3640_FMTS)
		return -EINVAL;

	ofmt = ov3640_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int ov3640_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct ov3640_format_struct **ret_fmt,
		struct ov3640_win_size **ret_wsize)
{
	int index;
	struct ov3640_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	for (index = 0; index < N_OV3640_FMTS; index++)
		if (ov3640_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_OV3640_FMTS){
		printk("unsupported format!\n");
		return -EINVAL;
	}
	if (ret_fmt != NULL)
		*ret_fmt = ov3640_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
		return -EINVAL;
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = ov3640_win_sizes; wsize < ov3640_win_sizes + N_WIN_SIZES;
			wsize++)
		if (pix->width <= wsize->width && pix->height <= wsize->height)
			break;
	if (wsize >= ov3640_win_sizes + N_WIN_SIZES){
		printk("size exceed and set as QXGA!\n");
		wsize--;   /* Take the smallest one */
	}
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
#if 0
	pix->width = wsize->width;
	pix->height = wsize->height;
#endif
	pix->bytesperline = pix->width*ov3640_formats[index].bpp/8;
	pix->sizeimage = pix->height*pix->bytesperline;
	printk("ov3640_try_fmt: pix->width is %d, pix->height is %d wsize %d\n", pix->width, pix->height, wsize->width);

	if (ret_fmt == NULL)
		return 0;
	switch (pix->pixelformat)
	{
		case V4L2_PIX_FMT_YUYV: 
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV420: 
			switch (pix->width)
			{
				case QXGA_WIDTH:
					(*ret_fmt)->regs = ov3640_fmt_yuv422_qxga;
					break;
				case D1_WIDTH:
					(*ret_fmt)->regs = ov3640_fmt_yuv422_d1;
					break;
				case VGA_WIDTH:
					(*ret_fmt)->regs = ov3640_fmt_yuv422_vga;
					break;
				case QVGA_WIDTH:
					(*ret_fmt)->regs = ov3640_fmt_yuv422_qvga;
					break;
				case QCIF_WIDTH:
					(*ret_fmt)->regs = ov3640_fmt_yuv422_qcif;
					break;
				default:
					printk("unsupported size!\n");
					return -EINVAL;
			}
			break;
		case V4L2_PIX_FMT_JPEG:
			switch (pix->width)
			{
				case QXGA_WIDTH:
					(*ret_fmt)->regs = ov3640_fmt_jpeg_qxga;
					break;
				case VGA_WIDTH:
					(*ret_fmt)->regs = ov3640_fmt_jpeg_vga;
					break;
				default:
					(*ret_fmt)->regs = ov3640_fmt_jpeg_qxga;
					break;
			}
			break;
		default:
			printk("unsupported format!\n");
			break;
	}	
	return 0;
}

/*
 * Set a format.
 */
static int ov3640_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
	struct ov3640_format_struct *ovfmt;
	struct ov3640_win_size *wsize;
	ret = ov3640_try_fmt(c, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;
	ov3640_write_array(c, ovfmt->regs);
	//	ov3640_set_hw();	//TODO
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov3640_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	unsigned char clkrc;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	ret = ov3640_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = OV3640_FRAME_RATE;
	if ((clkrc & CLK_EXT) == 0 && (clkrc & CLK_SCALE) > 1)
		cp->timeperframe.denominator /= (clkrc & CLK_SCALE);
	return 0;
}

static int ov3640_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	unsigned char clkrc;
	int ret, div;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;
	/*
	 * CLKRC has a reserved bit, so let's preserve it.
	 */
	ret = ov3640_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator*OV3640_FRAME_RATE)/tpf->denominator;
	if (div == 0)
		div = 1;
	else if (div > CLK_SCALE)
		div = CLK_SCALE;
	clkrc = (clkrc & 0x80) | div;
	tpf->numerator = 1;
	tpf->denominator = OV3640_FRAME_RATE/div;
	return ov3640_write(c, REG_CLKRC, clkrc);
}

static int ov3640_s_input(struct i2c_client *c, int *id)
{
	return 0;
}

/*
 * Code for dealing with controls.
 */


/*TODO - need to port below register codes for 3640...maybe not used*/

#if 0
static int ov3640_store_cmatrix(struct i2c_client *client,
		int matrix[CMATRIX_LEN])
{
	int i, ret;
	unsigned char signbits;

	/*
	 * Weird crap seems to exist in the upper part of
	 * the sign bits register, so let's preserve it.
	 */
	ret = ov3640_read(client, REG_CMATRIX_SIGN, &signbits);
	signbits &= 0xc0;

	for (i = 0; i < CMATRIX_LEN; i++) {
		unsigned char raw;

		if (matrix[i] < 0) {
			signbits |= (1 << i);
			if (matrix[i] < -255)
				raw = 0xff;
			else
				raw = (-1 * matrix[i]) & 0xff;
		}
		else {
			if (matrix[i] > 255)
				raw = 0xff;
			else
				raw = matrix[i] & 0xff;
		}
		ret += ov3640_write(client, REG_CMATRIX_BASE + i, raw);
	}
	ret += ov3640_write(client, REG_CMATRIX_SIGN, signbits);
	return ret;
}
#endif

/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */

int g_brightness;
int g_hue;
int g_saturation;
int g_contrast;
int g_vflip;
int g_hflip;
int g_whitebalance;
int g_exposure;

static int ov3640_t_sat(struct i2c_client *client, int value)
{

	int ret = 0;
	unsigned char regval;

	ov3640_read(client, REG_BRIGHT1, &regval);
	regval |= 0x02;
	switch (value)
	{
		case 0:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x10);
			ret = ov3640_write(client, REG_SAT1, 0x10);
		break;

		case 1:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x30);
			ret = ov3640_write(client, REG_SAT1, 0x30);
		break;

		case 2:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x40);
			ret = ov3640_write(client, REG_SAT1, 0x40);
		break;

		case 3:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x50);
			ret = ov3640_write(client, REG_SAT1, 0x50);
		break;

		case 4:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x70);
			ret = ov3640_write(client, REG_SAT1, 0x70);
		break;
	}
	g_saturation = value;
	return ret;
}

static int ov3640_q_sat(struct i2c_client *client, __s32 *value)
{
	*value = g_saturation;
	return 0;
}

static int ov3640_t_hue(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;
	ov3640_read(client, REG_HUE10, &regval);
	switch (value)
	{
		case 1:/*90 degree*/
			ret = ov3640_write(client, REG_HUE0, 0x20);
			ret = ov3640_write(client, REG_HUE1, 0x64);
			ret = ov3640_write(client, REG_HUE2, 0x08);
			ret = ov3640_write(client, REG_HUE3, 0x7f);
			ret = ov3640_write(client, REG_HUE4, 0x78);
			ret = ov3640_write(client, REG_HUE5, 0x06);
			ret = ov3640_write(client, REG_HUE6, 0x3d);
			ret = ov3640_write(client, REG_HUE7, 0xb6);
			ret = ov3640_write(client, REG_HUE8, 0xf3);
			ret = ov3640_write(client, REG_HUE9, 0xc8);
			regval &= ~0x2;
			ret = ov3640_write(client, REG_HUE10, regval);
		break;

		case 2:/*180 degree*/
			ret = ov3640_write(client, REG_HUE0, 0x20);
			ret = ov3640_write(client, REG_HUE1, 0x64);
			ret = ov3640_write(client, REG_HUE2, 0x08);
			ret = ov3640_write(client, REG_HUE3, 0x30);
			ret = ov3640_write(client, REG_HUE4, 0x90);
			ret = ov3640_write(client, REG_HUE5, 0xc0);
			ret = ov3640_write(client, REG_HUE6, 0xa0);
			ret = ov3640_write(client, REG_HUE7, 0x98);
			ret = ov3640_write(client, REG_HUE8, 0x08);
			ret = ov3640_write(client, REG_HUE9, 0x60);
			regval &= ~0x2;
			ret = ov3640_write(client, REG_HUE10,regval);
		break;

		case 3:/*-90 degree*/
			ret = ov3640_write(client, REG_HUE0, 0x20);
			ret = ov3640_write(client, REG_HUE1, 0x64);
			ret = ov3640_write(client, REG_HUE2, 0x08);
			ret = ov3640_write(client, REG_HUE3, 0x7f);
			ret = ov3640_write(client, REG_HUE4, 0x78);
			ret = ov3640_write(client, REG_HUE5, 0x06);
			ret = ov3640_write(client, REG_HUE6, 0x3d);
			ret = ov3640_write(client, REG_HUE7, 0xb6);
			ret = ov3640_write(client, REG_HUE8, 0xf3);
			ret = ov3640_write(client, REG_HUE9, 0x30);
			regval |= 0x2;
			ret = ov3640_write(client, REG_HUE10,regval);
		break;

		case 0:/*0 degree*/
		default:
			ret = ov3640_write(client, REG_HUE0, 0x20);
			ret = ov3640_write(client, REG_HUE1, 0x64);
			ret = ov3640_write(client, REG_HUE2, 0x08);
			ret = ov3640_write(client, REG_HUE3, 0x30);
			ret = ov3640_write(client, REG_HUE4, 0x90);
			ret = ov3640_write(client, REG_HUE5, 0xc0);
			ret = ov3640_write(client, REG_HUE6, 0xa0);
			ret = ov3640_write(client, REG_HUE7, 0x98);
			ret = ov3640_write(client, REG_HUE8, 0x08);
			ret = ov3640_write(client, REG_HUE9, 0x98);
			regval |= 0x2;
			ret = ov3640_write(client, REG_HUE10,regval);
		break;
	}
	g_hue = value;
	return ret;
}


static int ov3640_q_hue(struct i2c_client *client, __s32 *value)
{
	*value = g_hue;
	return 0;
}

static int ov3640_t_brightness(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval, regvalue;

	ov3640_read(client, REG_BRIGHT1, &regval);//55
	regval |= 0x04;
	ov3640_read(client, REG_BRIGHT2, &regvalue);//54
	switch (value)
	{
		case 0:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue |= 0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x20);
		break;

		case 1:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue |= 0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x10);
		break;

		case 3:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x10);
		break;

		case 4:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x20);
		break;

		case 2:
		default:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x00);
		break;
	}
	g_brightness = value;
	return ret;
}

static int ov3640_q_brightness(struct i2c_client *client, __s32 *value)
{
	*value = g_brightness;
	return 0;
}

static int ov3640_t_contrast(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval0, regval1;

	ov3640_read(client, REG_BRIGHT1, &regval0);//55
	ov3640_read(client, REG_BRIGHT2, &regval1);//54
	regval1 &= ~0x04;
	regval0 |= 0x04;

	switch (value)
	{
		case 0:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x08);
			ret = ov3640_write(client, REG_CONTRAS1, 0x08);
		break;

		case 1:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x18);
			ret = ov3640_write(client, REG_CONTRAS1, 0x18);
		break;

		case 3:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x40);
			ret = ov3640_write(client, REG_CONTRAS1, 0x40);
		break;

		case 4:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x80);
			ret = ov3640_write(client, REG_CONTRAS1, 0x80);
		break;

		case 2:
		default:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x20);
			ret = ov3640_write(client, REG_CONTRAS1, 0x20);
		break;
	}
	g_brightness = value;
	return ret;
}

static int ov3640_q_contrast(struct i2c_client *client, __s32 *value)
{
	*value = g_contrast;
	return 0;
}

static int ov3640_q_hflip(struct i2c_client *client, __s32 *value)
{
	*value = g_hflip;
	return 0;
}


static int ov3640_t_hflip(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval1, regval2;

	ov3640_read(client, REG_MVFP1, &regval1);
	ov3640_read(client, REG_MVFP2, &regval2);
	if (value){
		regval1 |= 0x02;
		regval2 |= 0x08;
		ret = ov3640_write(client, REG_MVFP1, regval1);
		ret = ov3640_write(client, REG_MVFP2, regval2);
	} else {
		regval1 &= ~0x02;
		regval2 &= ~0x08;
		ret = ov3640_write(client, REG_MVFP1, regval1);
		ret = ov3640_write(client, REG_MVFP2, regval2);
	}
	g_hflip = value;
	return ret;
}



static int ov3640_q_vflip(struct i2c_client *client, __s32 *value)
{
	*value = g_vflip;
	return 0;
}


static int ov3640_t_vflip(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;
	ov3640_read(client, REG_MVFP1, &regval);
	if (value){
		ret = ov3640_write(client, REG_MVFP0, 0x9);
		regval |= 0x1;
		ret = ov3640_write(client, REG_MVFP1, regval);
	} else {
		ret = ov3640_write(client, REG_MVFP0, 0xa);
		regval &= ~0x1;
		ret = ov3640_write(client, REG_MVFP1, regval);
	}
	g_vflip = value;
	return ret;
}

static int ov3640_t_whitebalance(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;

	ov3640_read(client, REG_WB0, &regval);
	switch (value)
	{
		case 0:/*sunny*/
			regval |= 0x08;
			ret = ov3640_write(client, REG_WB0, regval);
			ret = ov3640_write(client, REG_WB1, 0x5e);
			ret = ov3640_write(client, REG_WB2, 0x40);
			ret = ov3640_write(client, REG_WB3, 0x46);
		break;

		case 1:/*cloudy*/
			regval |= 0x08;
			ret = ov3640_write(client, REG_WB0, regval);
			ret = ov3640_write(client, REG_WB1, 0x68);
			ret = ov3640_write(client, REG_WB2, 0x40);
			ret = ov3640_write(client, REG_WB3, 0x4e);
		break;

		case 3:/*office*/
			regval |= 0x08;
			ret = ov3640_write(client, REG_WB0, regval);
			ret = ov3640_write(client, REG_WB1, 0x52);
			ret = ov3640_write(client, REG_WB2, 0x40);
			ret = ov3640_write(client, REG_WB3, 0x58);
		break;

		case 4:/*home*/
			regval |= 0x08;
			ret = ov3640_write(client, REG_WB0, regval);
			ret = ov3640_write(client, REG_WB1, 0x44);
			ret = ov3640_write(client, REG_WB2, 0x40);
			ret = ov3640_write(client, REG_WB3, 0x70);
		break;

		case 2:
		default:
			regval &= ~0x08;
			ret = ov3640_write(client, REG_WB0, regval);
		break;
	}
	g_whitebalance = value;
	return ret;
}

static int ov3640_q_whitebalance(struct i2c_client *client, __s32 *value)
{
	*value = g_whitebalance;
	return 0;
}

static int ov3640_t_exposure(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval0;

	ov3640_read(client, REG_EXPOSURE0, &regval0);
	regval0 &= ~0x80;
	ov3640_write(client, REG_EXPOSURE0, regval0);
	switch (value)
	{
		case 0:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x10);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x08);
			ret = ov3640_write(client, REG_EXPOSURE3, 0x21);
		break;

		case 1:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x20);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x18);
			ret = ov3640_write(client, REG_EXPOSURE3, 0x41);
		break;

		case 3:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x48);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x40);
			ret = ov3640_write(client, REG_EXPOSURE3, 0x81);
		break;

		case 4:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x60);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x58);
			ret = ov3640_write(client, REG_EXPOSURE3, 0xa1);
		break;

		case 2:
		default:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x28);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x20);
			ret = ov3640_write(client, REG_EXPOSURE3, 0x51);
		break;
	}
	g_exposure = value;
	return ret;
}

static int ov3640_q_exposure(struct i2c_client *client, __s32 *value)
{
	*value = g_exposure;
	return 0;
}


static struct ov3640_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} ov3640_controls[] =
{
	{
		.qc = {
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_brightness,
		.query = ov3640_q_brightness,
	},
	{
		.qc = {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,   /* XXX ov3640 spec */
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_contrast,
		.query = ov3640_q_contrast,
	},
	{
		.qc = {
			.id = V4L2_CID_SATURATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Saturation",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_sat,
		.query = ov3640_q_sat,
	},
	{
		.qc = {
			.id = V4L2_CID_HUE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "HUE",
			.minimum = 0,
			.maximum = 3,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_hue,
		.query = ov3640_q_hue,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Vertical flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov3640_t_vflip,
		.query = ov3640_q_vflip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Horizontal mirror",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov3640_t_hflip,
		.query = ov3640_q_hflip,
	},
	{
		.qc = {
			.id = V4L2_CID_DO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "white balance",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_whitebalance,
		.query = ov3640_q_whitebalance,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_exposure,
		.query = ov3640_q_exposure,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov3640_controls))

static struct ov3640_control *ov3640_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov3640_controls[i].qc.id == id)
			return ov3640_controls + i;
	return NULL;
}


static int ov3640_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct ov3640_control *ctrl = ov3640_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int ov3640_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov3640_control *octrl = ov3640_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int ov3640_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov3640_control *octrl = ov3640_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret =  octrl->tweak(client, ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}




int ccic_sensor_attach(struct i2c_client *client);


/*
 * Basic i2c stuff.
 */
static int __devinit ov3640_probe(struct i2c_client *client, const struct i2c_device_id * i2c_id)
{
	int ret = 0;
	struct ov3640_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;

	ccic_set_clock_mipi();

	pdata->power_on(1, 1);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct ov3640_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &ov3640_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);
	/*
	 * Make sure it's an ov3640
	 */
	ret = ov3640_detect(client);
	if (ret)
		goto out_free;
	printk(KERN_NOTICE "OmniVision ov3640 sensor detected\n");
	ret = ccic_sensor_attach(client);
	if (ret)
		goto out_free;

out_free:
	pdata->power_on(0, 1);
	ccic_disable_clock();

	if(ret)
		kfree(info);
	return ret;
}


static int ov3640_remove(struct i2c_client *client)
{
	return 0;	//TODO
}


static int ov3640_streamon(struct i2c_client *client)
{
	unsigned char val;
	ov3640_read(client, 0x3086, &val);
	val &= ~0x03;
	ov3640_write(client, 0x3086, val);
	return 0;
}

static int ov3640_streamoff(struct i2c_client *client)
{
	unsigned char val;
	ov3640_read(client, 0x3086, &val);
	val |= 0x03;
	ov3640_write(client, 0x3086, val);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov3640_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov3640_read(client, (u16)reg->reg, (unsigned char *)&(reg->val));
}

static int ov3640_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov3640_write(client, (u16)reg->reg, (unsigned char)reg->val);
}
#endif
static int ov3640_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_OV3640, 0);

		case VIDIOC_INT_RESET:
			ov3640_reset(client);
			return 0;
#if 0
		case VIDIOC_INT_INIT:
			return 0;//ov3640_init(client);		//TODO - should get 3640 default register values
#endif

		case VIDIOC_ENUM_FMT:
			return ov3640_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_TRY_FMT:
			return ov3640_try_fmt(client, (struct v4l2_format *) arg, NULL, NULL);
		case VIDIOC_S_FMT:
			return ov3640_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_QUERYCTRL:
			return ov3640_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return ov3640_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return ov3640_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return ov3640_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return ov3640_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return ov3640_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return ov3640_streamon(client);
		case VIDIOC_STREAMOFF:
			return ov3640_streamoff(client);
#ifdef CONFIG_VIDEO_ADV_DEBUG
		case VIDIOC_DBG_G_REGISTER:
			return ov3640_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return ov3640_s_register(client, (struct v4l2_dbg_register *) arg);
#endif
	}
	return -EINVAL;
}

static struct i2c_device_id ov3640_idtable[] = {
	{ "ov3640", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov3640_idtable);

static struct i2c_driver ov3640_driver = {
	.driver = {
		.name	= "ov3640",
	},
	.id_table       = ov3640_idtable,
	.command	= ov3640_command,
	.probe		= ov3640_probe,
	.remove		= ov3640_remove,
};


/*
 * Module initialization
 */
static int __init ov3640_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision ov3640 sensor driver, at your service\n");
	return i2c_add_driver(&ov3640_driver);
}

static void __exit ov3640_mod_exit(void)
{
	i2c_del_driver(&ov3640_driver);
}

late_initcall(ov3640_mod_init);
//module_init(ov3640_mod_init);
module_exit(ov3640_mod_exit);

