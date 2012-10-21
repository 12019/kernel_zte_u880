/*
 * A V4L2 driver for OmniVision MT9D113 cameras.
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
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <mach/camera.h>
#include <mach/mfp.h>
#include <linux/clk.h>

#include <linux/platform_device.h>
#include <mach/pxa910.h>
#include <mach/mfp-pxa910.h>

#include "pxa910_camera.h"

MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("A low-level driver for OmniVision mt9d113 sensors");
MODULE_LICENSE("GPL");

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */


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
#define MT9D113_FRAME_RATE 30

/*
 * The 3640 sits on i2c with ID 0x42
 */
#define MT9D113_I2C_ADDR 0x3c

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


#define CMATRIX_LEN 6
/*for MT9D113 porting*/
#define REG_CLKRC	0x0010
#define REG_SYS		0x001A
#define SYS_RESET	        0x0001
#define MAXARRAYLENGTH	      3000
/*
 * Information we maintain about a known sensor.
 */
struct mt9d113_format_struct;  /* coming later */
struct mt9d113_info {
	struct mt9d113_format_struct *fmt;  /* Current format */
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
	u16 value;//char to int
};

static int fmt_cnt=0;

/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 * RGB656 and YUV422 come from OV; RGB444 is homebrewed.
 *
 * IMPORTANT RULE: the first entry must be for COM7, see mt9d113_s_fmt for why.
 */

/*TODO - mt9d113_fmt_yuv422_qxga can't work. configuration should be correct*/

static struct regval_list  mt9d113_fmt_yuv[]  = {
{0x0014, 0x2545},     //PLL Control: BYPASS PLL = 9541
{0x001A, 0x0251},         // RESET_AND_MISC_CONTROL STATE=Sensor Reset, 1            
//Delay = 50                                            STATE=Sensor Reset, 0            
{0x001A, 0x0050},         // RESET_AND_MISC_CONTROL                                  
//Delay = 50                                                                             //BYPASS PLL??
{0x0014, 0x21F9},     //PLL Control: BYPASS PLL = 8697                                 //   
{0x0010, 0x1969},     //PLL Dividers = 784                                             //
{0x0012, 0x1FF9},     //PLL P Dividers = 247                                           //
{0x0014, 0x21FB},     //PLL Control: PLL_ENABLE on = 8699                              //
{0x0014, 0x20FB},     //PLL Control: SEL_LOCK_DET on = 8443                            //
//DELAY = 10            //Allow PLL to lock                                              //                                          
{0x0014, 0x20FA},     //PLL Control: PLL_BYPASS off = 8442                             //
{0x001A, 0x0050},     //RESET_AND_MISC_CONTROL BITFIELD = 0x001A, 0x0008, 0x0000},     //PLL//
{0x0018, 0x4028},     //STANDBY_CONTROL        BITFIELD = 0x0018, 0x0001, 0x0000},     //PLL//
{0x321C, 0x0003},     //By Pass TxFIFO = 3                                             
{0x98C, 0x2703},     //Output Width (A)
{0x990, 0x0280},     //      = 640
{0x98C, 0x2705},     //Output Height (A)
{0x990, 0x01E0},     //      = 480
{0x98C, 0x2707},     //Output Width (B)
{0x990, 0x0640},     //      = 1600
{0x98C, 0x2709},     //Output Height (B)
{0x990, 0x04B0},     //      = 1200
{0x98C, 0x270D},     //Row Start (A)
{0x990, 0x000},     //      = 0
{0x98C, 0x270F},     //Column Start (A)
{0x990, 0x000},     //      = 0
{0x98C, 0x2711},     //Row End (A)
{0x990, 0x4BD},     //      = 1213
{0x98C, 0x2713},     //Column End (A)
{0x990, 0x64D},     //      = 1613
{0x98C, 0x2715},     //Row Speed (A)
{0x990, 0x0111},     //      = 273
{0x98C, 0x2717},     //Read Mode (A)
{0x990, 0x046C},     //      = 1132
{0x98C, 0x2719},     //sensor_fine_correction (A)
{0x990, 0x005A},     //      = 90
{0x98C, 0x271B},     //sensor_fine_IT_min (A)
{0x990, 0x01BE},     //      = 446
{0x98C, 0x271D},     //sensor_fine_IT_max_margin (A)
{0x990, 0x0131},     //      = 305
{0x98C, 0x271F},     //Frame Lines (A)
{0x990, 0x02B3},     //      = 691
{0x98C, 0x2721},     //Line Length (A)
{0x990, 0x0B3D},     //      = 2877
{0x98C, 0x2723},     //Row Start (B)
{0x990, 0x004},     //      = 4
{0x98C, 0x2725},     //Column Start (B)
{0x990, 0x004},     //      = 4
{0x98C, 0x2727},     //Row End (B)
{0x990, 0x4BB},     //      = 1211
{0x98C, 0x2729},     //Column End (B)
{0x990, 0x64B},     //      = 1611
{0x98C, 0x272B},     //Row Speed (B)
{0x990, 0x0111},     //      = 273
{0x98C, 0x272D},     //Read Mode (B)
{0x990, 0x0024},     //      = 36
{0x98C, 0x272F},     //sensor_fine_correction (B)
{0x990, 0x003A},     //      = 58
{0x98C, 0x2731},     //sensor_fine_IT_min (B)
{0x990, 0x00F6},     //      = 246
{0x98C, 0x2733},     //sensor_fine_IT_max_margin (B)
{0x990, 0x008B},     //      = 139
{0x98C, 0x2735},     //Frame Lines (B)
{0x990, 0x050D},     //      = 1293
{0x98C, 0x2737},     //Line Length (B)
{0x990, 0x0B3D},     //      = 2877
{0x98C, 0x2739},     //Crop_X0 (A)
{0x990, 0x0000},     //      = 0
{0x98C, 0x273B},     //Crop_X1 (A)
{0x990, 0x031F},     //      = 799
{0x98C, 0x273D},     //Crop_Y0 (A)
{0x990, 0x0000},     //      = 0
{0x98C, 0x273F},     //Crop_Y1 (A)
{0x990, 0x0257},     //      = 599
{0x98C, 0x2747},     //Crop_X0 (B)
{0x990, 0x0000},     //      = 0
{0x98C, 0x2749},     //Crop_X1 (B)
{0x990, 0x063F},     //      = 1599
{0x98C, 0x274B},     //Crop_Y0 (B)
{0x990, 0x0000},     //      = 0
{0x98C, 0x274D},     //Crop_Y1 (B)
{0x990, 0x04AF},     //      = 1199
{0x98C, 0x222D},     //R9 Step
{0x990, 0x0046},     //      = 70
{0x98C, 0xA408},     //search_f1_50
{0x990, 0x10},     //      = 16
{0x98C, 0xA409},     //search_f2_50
{0x990, 0x12},     //      = 18
{0x98C, 0xA40A},     //search_f1_60
{0x990, 0x13},     //      = 19
{0x98C, 0xA40B},     //search_f2_60
{0x990, 0x15},     //      = 21
{0x98C, 0x2411},     //R9_Step_60 (A)
{0x990, 0x0046},     //      = 70
{0x98C, 0x2413},     //R9_Step_50 (A)
{0x990, 0x0053},     //      = 83
{0x98C, 0x2415},     //R9_Step_60 (B)
{0x990, 0x0046},     //      = 70
{0x98C, 0x2417},     //R9_Step_50 (B)
{0x990, 0x0053},     //      = 83
{0x98C, 0xA404},     //FD Mode
{0x990, 0x10},     //      = 16
{0x98C, 0xA40D},     //Stat_min
{0x990, 0x02},     //      = 2
{0x98C, 0xA40E},     //Stat_max
{0x990, 0x03},     //      = 3
{0x98C, 0xA410},     //Min_amplitude
{0x990, 0x0A},     //      = 10
{0x98C, 0xA103},     //Refresh Sequencer Mode
{0x990, 0x06},     //      = 6
//DELAY=10//, TIMEOUT=100  // wait for command to be processed
{0x98C, 0xA103},     //Refresh Sequencer
{0x990, 0x05},     //      = 5


//-lens
{0x3658, 0x00F0},      // P_RD_P0Q0                   
{0x365A, 0x1EC9},      // P_RD_P0Q1                   
{0x365C, 0x47D1},      // P_RD_P0Q2                   
{0x365E, 0xA9AE},      // P_RD_P0Q3                   
{0x3660, 0xB0B1},      // P_RD_P0Q4                   
{0x3680, 0x4ACC},      // P_RD_P1Q0                   
{0x3682, 0x1D4F},      // P_RD_P1Q1                   
{0x3684, 0x7CAE},      // P_RD_P1Q2                   
{0x3686, 0x6DB0},      // P_RD_P1Q3                   
{0x3688, 0x08B3},      // P_RD_P1Q4                   
{0x36A8, 0x4AD2},      // P_RD_P2Q0                   
{0x36AA, 0x680E},      // P_RD_P2Q1                   
{0x36AC, 0x32F4},      // P_RD_P2Q2                   
{0x36AE, 0xB754},      // P_RD_P2Q3                   
{0x36B0, 0x8198},      // P_RD_P2Q4                   
{0x36D0, 0x4CB0},      // P_RD_P3Q0                   
{0x36D2, 0x6011},      // P_RD_P3Q1                   
{0x36D4, 0x1314},      // P_RD_P3Q2                   
{0x36D6, 0x9716},      // P_RD_P3Q3                   
{0x36D8, 0xB517},      // P_RD_P3Q4                   
{0x36F8, 0xC492},      // P_RD_P4Q0                   
{0x36FA, 0xD074},      // P_RD_P4Q1                   
{0x36FC, 0x8099},      // P_RD_P4Q2                   
{0x36FE, 0x5A78},      // P_RD_P4Q3                   
{0x3700, 0x6F9B},      // P_RD_P4Q4                   
{0x364E, 0x0890},      // P_GR_P0Q0                   
{0x3650, 0xC2A8},      // P_GR_P0Q1                   
{0x3652, 0x4DF1},      // P_GR_P0Q2                   
{0x3654, 0x86CF},      // P_GR_P0Q3                   
{0x3656, 0xD692},      // P_GR_P0Q4                   
{0x3676, 0x120C},      // P_GR_P1Q0                   
{0x3678, 0xD14F},      // P_GR_P1Q1                   
{0x367A, 0x4E4F},      // P_GR_P1Q2                   
{0x367C, 0x36D2},      // P_GR_P1Q3                   
{0x367E, 0x02D3},      // P_GR_P1Q4                   
{0x369E, 0x67B2},      // P_GR_P2Q0                   
{0x36A0, 0xA251},      // P_GR_P2Q1                   
{0x36A2, 0xB273},      // P_GR_P2Q2                   
{0x36A4, 0x75F2},      // P_GR_P2Q3                   
{0x36A6, 0xDB95},      // P_GR_P2Q4                   
{0x36C6, 0x3F90},      // P_GR_P3Q0                   
{0x36C8, 0x1C31},      // P_GR_P3Q1                   
{0x36CA, 0x1E54},      // P_GR_P3Q2                   
{0x36CC, 0x9CB5},      // P_GR_P3Q3                   
{0x36CE, 0xC257},      // P_GR_P3Q4                   
{0x36EE, 0xE3D3},      // P_GR_P4Q0                   
{0x36F0, 0xDF70},      // P_GR_P4Q1                   
{0x36F2, 0x8118},      // P_GR_P4Q2                   
{0x36F4, 0x2F97},      // P_GR_P4Q3                   
{0x36F6, 0x0CBB},      // P_GR_P4Q4                   
{0x3662, 0x00B0},      // P_BL_P0Q0                   
{0x3664, 0x9808},      // P_BL_P0Q1                   
{0x3666, 0x1FB1},      // P_BL_P0Q2                   
{0x3668, 0xC3AD},      // P_BL_P0Q3                   
{0x366A, 0xD331},      // P_BL_P0Q4                   
{0x368A, 0x148C},      // P_BL_P1Q0                   
{0x368C, 0xA3CF},      // P_BL_P1Q1                   
{0x368E, 0x154E},      // P_BL_P1Q2                   
{0x3690, 0x59B2},      // P_BL_P1Q3                   
{0x3692, 0x7092},      // P_BL_P1Q4                   
{0x36B2, 0x6232},      // P_BL_P2Q0                   
{0x36B4, 0xC850},      // P_BL_P2Q1                   
{0x36B6, 0x2B72},      // P_BL_P2Q2                   
{0x36B8, 0xE292},      // P_BL_P2Q3                   
{0x36BA, 0xC357},      // P_BL_P2Q4                   
{0x36DA, 0x13B0},      // P_BL_P3Q0                   
{0x36DC, 0x4591},      // P_BL_P3Q1                   
{0x36DE, 0x6673},      // P_BL_P3Q2                   
{0x36E0, 0xBD75},      // P_BL_P3Q3                   
{0x36E2, 0x8317},      // P_BL_P3Q4                   
{0x3702, 0xE973},      // P_BL_P4Q0                   
{0x3704, 0x9873},      // P_BL_P4Q1                   
{0x3706, 0xC9F8},      // P_BL_P4Q2                   
{0x3708, 0x10B8},      // P_BL_P4Q3                   
{0x370A, 0x5F9B},      // P_BL_P4Q4                   
{0x366C, 0x01D0},      // P_GB_P0Q0                   
{0x366E, 0xD469},      // P_GB_P0Q1                   
{0x3670, 0x3F91},      // P_GB_P0Q2                   
{0x3672, 0xB88E},      // P_GB_P0Q3                   
{0x3674, 0xD252},      // P_GB_P0Q4                   
{0x3694, 0x302C},      // P_GB_P1Q0                   
{0x3696, 0x49CF},      // P_GB_P1Q1                   
{0x3698, 0xEFAB},      // P_GB_P1Q2                   
{0x369A, 0x34ED},      // P_GB_P1Q3                   
{0x369C, 0x20F3},      // P_GB_P1Q4                   
{0x36BC, 0x75D2},      // P_GB_P2Q0                   
{0x36BE, 0xB970},      // P_GB_P2Q1                   
{0x36C0, 0x8AD4},      // P_GB_P2Q2                   
{0x36C2, 0x3230},      // P_GB_P2Q3                   
{0x36C4, 0xDD94},      // P_GB_P2Q4                   
{0x36E4, 0x0631},      // P_GB_P3Q0                   
{0x36E6, 0x4C91},      // P_GB_P3Q1                   
{0x36E8, 0x1134},      // P_GB_P3Q2                   
{0x36EA, 0xFFD5},      // P_GB_P3Q3                   
{0x36EC, 0xBA97},      // P_GB_P3Q4                   
{0x370C, 0x8994},      // P_GB_P4Q0                   
{0x370E, 0xB972},      // P_GB_P4Q1                   
{0x3710, 0xCDD7},      // P_GB_P4Q2                   
{0x3712, 0x49F7},      // P_GB_P4Q3                   
{0x3714, 0x7BFA},      // P_GB_P4Q4                   
{0x3644, 0x033C},      // POLY_ORIGIN_C               
{0x3642, 0x0240},      // POLY_ORIGIN_R               
{0x3210, 0x01B8},      // COLOR_PIPELINE_CONTROL 

//GAMMA TABLE                                                                
{0x098C, 0xAB37},      // MCU_ADDRESS [HG_GAMMA_MORPH_CTRL]                 
{0x0990, 0x0003},      // MCU_DATA_0                               rev3     
{0x098C, 0xAB3C},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_0]                  
{0x0990, 0x0000},      // MCU_DATA_0                                        
{0x098C, 0xAB3D},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_1]                  
{0x0990, 0x000A},      // MCU_DATA_0                                        
{0x098C, 0xAB3E},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_2]                  
{0x0990, 0x001D},      // MCU_DATA_0                                        
{0x098C, 0xAB3F},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_3]                  
{0x0990, 0x0037},      // MCU_DATA_0                                        
{0x098C, 0xAB40},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_4]                  
{0x0990, 0x0058},      // MCU_DATA_0                                        
{0x098C, 0xAB41},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_5]                  
{0x0990, 0x0071},      // MCU_DATA_0                                        
{0x098C, 0xAB42},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_6]                  
{0x0990, 0x0086},      // MCU_DATA_0                                        
{0x098C, 0xAB43},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_7]                  
{0x0990, 0x0098},      // MCU_DATA_0                                        
{0x098C, 0xAB44},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_8]                  
{0x0990, 0x00A7},      // MCU_DATA_0                                        
{0x098C, 0xAB45},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_9]                  
{0x0990, 0x00B5},      // MCU_DATA_0                                        
{0x098C, 0xAB46},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_10]                 
{0x0990, 0x00C0},      // MCU_DATA_0                                        
{0x098C, 0xAB47},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_11]                 
{0x0990, 0x00CB},      // MCU_DATA_0                                        
{0x098C, 0xAB48},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_12]                 
{0x0990, 0x00D4},      // MCU_DATA_0                                        
{0x098C, 0xAB49},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_13]                 
{0x0990, 0x00DD},      // MCU_DATA_0                                        
{0x098C, 0xAB4A},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_14]                 
{0x0990, 0x00E4},      // MCU_DATA_0                                        
{0x098C, 0xAB4B},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_15]                 
{0x0990, 0x00EC},      // MCU_DATA_0                                        
{0x098C, 0xAB4C},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_16]                 
{0x0990, 0x00F3},      // MCU_DATA_0                                        
{0x098C, 0xAB4D},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_17]                 
{0x0990, 0x00F9},      // MCU_DATA_0                                        
{0x098C, 0xAB4E},      // MCU_ADDRESS [HG_GAMMA_TABLE_A_18]                 
{0x0990, 0x00FF},      // MCU_DATA_0                                        
{0x098C, 0xAB4F},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_0]                  
{0x0990, 0x0000},      // MCU_DATA_0                                        
{0x098C, 0xAB50},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_1]                  
{0x0990, 0x000A},      // MCU_DATA_0                                        
{0x098C, 0xAB51},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_2]                  
{0x0990, 0x001D},      // MCU_DATA_0                                        
{0x098C, 0xAB52},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_3]                  
{0x0990, 0x0037},      // MCU_DATA_0                                        
{0x098C, 0xAB53},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_4]                  
{0x0990, 0x0058},      // MCU_DATA_0                                        
{0x098C, 0xAB54},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_5]                  
{0x0990, 0x0071},      // MCU_DATA_0                                        
{0x098C, 0xAB55},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_6]                  
{0x0990, 0x0086},      // MCU_DATA_0                                        
{0x098C, 0xAB56},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_7]                  
{0x0990, 0x0098},      // MCU_DATA_0                                        
{0x098C, 0xAB57},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_8]                  
{0x0990, 0x00A7},      // MCU_DATA_0                                        
{0x098C, 0xAB58},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_9]                  
{0x0990, 0x00B5},      // MCU_DATA_0                                        
{0x098C, 0xAB59},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_10]                 
{0x0990, 0x00C0},      // MCU_DATA_0                                        
{0x098C, 0xAB5A},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_11]                 
{0x0990, 0x00CB},      // MCU_DATA_0                                        
{0x098C, 0xAB5B},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_12]                 
{0x0990, 0x00D4},      // MCU_DATA_0                                        
{0x098C, 0xAB5C},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_13]                 
{0x0990, 0x00DD},      // MCU_DATA_0                                        
{0x098C, 0xAB5D},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_14]                 
{0x0990, 0x00E4},      // MCU_DATA_0                                        
{0x098C, 0xAB5E},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_15]                 
{0x0990, 0x00EC},      // MCU_DATA_0                                        
{0x098C, 0xAB5F},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_16]                 
{0x0990, 0x00F3},      // MCU_DATA_0                                        
{0x098C, 0xAB60},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_17]                 
{0x0990, 0x00F9},      // MCU_DATA_0                                        
{0x098C, 0xAB61},      // MCU_ADDRESS [HG_GAMMA_TABLE_B_18]                 
{0x0990, 0x00FF},      // MCU_DATA_0                                        
{0x098C, 0x2B62},      // MCU_ADDRESS [HG_FTB_START_BM]                     
{0x0990, 0xFFFF},      // MCU_DATA_0                                 0x6978 
{0x098C, 0x2B64},      // MCU_ADDRESS [HG_FTB_STOP_BM]                      
{0x0990, 0xFFFF},      // MCU_DATA_0                                 0x7530 
{0x323E,0xC52C},    // AWB_CONFIG1  
{0x3244,0x0304},    // AWB_CONFIG4                  
{0x098C, 0x2306},      // MCU_ADDRESS [AWB_CCM_L_0]                         
{0x0990, 0x00F7},      // MCU_DATA_0                                        
{0x098C, 0x2308},      // MCU_ADDRESS [AWB_CCM_L_1]                         
{0x0990, 0xFFDA},      // MCU_DATA_0                                        
{0x098C, 0x230A},      // MCU_ADDRESS [AWB_CCM_L_2]                         
{0x0990, 0xFFE6},      // MCU_DATA_0                                        
{0x098C, 0x230C},      // MCU_ADDRESS [AWB_CCM_L_3]                         
{0x0990, 0xFFD1},      // MCU_DATA_0                                        
{0x098C, 0x230E},      // MCU_ADDRESS [AWB_CCM_L_4]                         
{0x0990, 0x0197},      // MCU_DATA_0                                        
{0x098C, 0x2310},      // MCU_ADDRESS [AWB_CCM_L_5]                         
{0x0990, 0xFFC8},      // MCU_DATA_0                                        
{0x098C, 0x2312},      // MCU_ADDRESS [AWB_CCM_L_6]                         
{0x0990, 0x0046},      // MCU_DATA_0                                        
{0x098C, 0x2314},      // MCU_ADDRESS [AWB_CCM_L_7]                         
{0x0990, 0xFF67},      // MCU_DATA_0                                        
{0x098C, 0x2316},      // MCU_ADDRESS [AWB_CCM_L_8]                         
{0x0990, 0x0179},      // MCU_DATA_0                                        
{0x098C, 0x2318},      // MCU_ADDRESS [AWB_CCM_L_9]                         
{0x0990, 0x0028},      // MCU_DATA_0                                        
{0x098C, 0x231A},      // MCU_ADDRESS [AWB_CCM_L_10]                        
{0x0990, 0x0036},      // MCU_DATA_0                                        
{0x098C, 0x231C},      // MCU_ADDRESS [AWB_CCM_RL_0]                        
{0x0990, 0x0074},      // MCU_DATA_0                                        
{0x098C, 0x231E},      // MCU_ADDRESS [AWB_CCM_RL_1]                        
{0x0990, 0xFF1A},      // MCU_DATA_0                                        
{0x098C, 0x2320},      // MCU_ADDRESS [AWB_CCM_RL_2]                        
{0x0990, 0x00C6},      // MCU_DATA_0                                        
{0x098C, 0x2322},      // MCU_ADDRESS [AWB_CCM_RL_3]                        
{0x0990, 0x0019},      // MCU_DATA_0                                        
{0x098C, 0x2324},      // MCU_ADDRESS [AWB_CCM_RL_4]                        
{0x0990, 0x003B},      // MCU_DATA_0                                        
{0x098C, 0x2326},      // MCU_ADDRESS [AWB_CCM_RL_5]                        
{0x0990, 0xFFBD},      // MCU_DATA_0                                        
{0x098C, 0x2328},      // MCU_ADDRESS [AWB_CCM_RL_6]                        
{0x0990, 0xFFC9},      // MCU_DATA_0                                        
{0x098C, 0x232A},      // MCU_ADDRESS [AWB_CCM_RL_7]                        
{0x0990, 0x006F},      // MCU_DATA_0                                        
{0x098C, 0x232C},      // MCU_ADDRESS [AWB_CCM_RL_8]                        
{0x0990, 0xFFE6},      // MCU_DATA_0                                        
{0x098C, 0x232E},      // MCU_ADDRESS [AWB_CCM_RL_9]                        
{0x0990, 0x0001},      // MCU_DATA_0                                        
{0x098C, 0x2330},      // MCU_ADDRESS [AWB_CCM_RL_10]                       
{0x0990, 0xFFF2},      // MCU_DATA_0                                        
{0x098C, 0xA348},      // MCU_ADDRESS [AWB_GAIN_BUFFER_SPEED]               
{0x0990, 0x0006},      // MCU_DATA_0                                        
{0x098C, 0xA349},      // MCU_ADDRESS [AWB_JUMP_DIVISOR]                    
{0x0990, 0x0001},      // MCU_DATA_0                                        
{0x098C, 0xA34A},      // MCU_ADDRESS [AWB_GAIN_MIN]                        
{0x0990, 0x0059},      // MCU_DATA_0                                        
{0x098C, 0xA34B},      // MCU_ADDRESS [AWB_GAIN_MAX]                        
{0x0990, 0x00C8},      // MCU_DATA_0                                        
{0x098C, 0xA351},      // MCU_ADDRESS [AWB_CCM_POSITION_MIN]                
{0x0990, 0x0000},      // MCU_DATA_0                                        
{0x098C, 0xA352},      // MCU_ADDRESS [AWB_CCM_POSITION_MAX]                
{0x0990, 0x007F},      // MCU_DATA_0                                        
{0x098C, 0xA355},      // MCU_ADDRESS [AWB_MODE]                            
{0x0990, 0x0002},      // MCU_DATA_0                                        
{0x098C, 0xA34C},      // MCU_ADDRESS [AWB_GAINMIN_B]                       
{0x0990, 0x0059},      // MCU_DATA_0                                        
{0x098C, 0xA34D},      // MCU_ADDRESS [AWB_GAINMAX_B]                       
{0x0990, 0x00A6},      // MCU_DATA_0                                        
{0x098C, 0xA354},      // MCU_ADDRESS [AWB_SATURATION]   READ ONLY & variabl
{0x0990, 0x0040},      // MCU_DATA_0                                        
{0x098C, 0xA35D},      // MCU_ADDRESS [AWB_STEADY_BGAIN_OUT_MIN]            
{0x0990, 0x0078},      // MCU_DATA_0                                        
{0x098C, 0xA35E},      // MCU_ADDRESS [AWB_STEADY_BGAIN_OUT_MAX]            
{0x0990, 0x0086},      // MCU_DATA_0                                        
{0x098C, 0xA35F},      // MCU_ADDRESS [AWB_STEADY_BGAIN_IN_MIN]             
{0x0990, 0x007E},      // MCU_DATA_0                                        
{0x098C, 0xA360},      // MCU_ADDRESS [AWB_STEADY_BGAIN_IN_MAX]             
{0x0990, 0x0082},      // MCU_DATA_0                                        
{0x098C, 0xA363},      // MCU_ADDRESS [AWB_TG_MIN0]                         
{0x0990, 0x00C0},      // MCU_DATA_0                                        
{0x098C, 0xA364},      // MCU_ADDRESS [AWB_TG_MAX0]                         
{0x0990, 0x00F6},      // MCU_DATA_0                                        
{0x098C, 0xA302},      // MCU_ADDRESS [AWB_WINDOW_POS]                      
{0x0990, 0x0000},      // MCU_DATA_0                                        
{0x098C, 0xA303},      // MCU_ADDRESS [AWB_WINDOW_SIZE]                     
{0x0990, 0x00EF},      // MCU_DATA_0                                        
{0x098C, 0xA366},      // MCU_ADDRESS [AWB_KR_L]                            
{0x0990, 0x007C},      // MCU_DATA_0                                        
{0x098C, 0xA367},      // MCU_ADDRESS [AWB_KG_L]                            
{0x0990, 0x0080},      // MCU_DATA_0                                        
{0x098C, 0xA368},      // MCU_ADDRESS [AWB_KB_L]                            
{0x0990, 0x0079},      // MCU_DATA_0                                        
{0x098C, 0xA36A},      // MCU_ADDRESS [AWB_KG_R]                            
{0x0990, 0x0080},      // MCU_DATA_0                                        
{0x098C, 0xA369},      // MCU_ADDRESS [AWB_KR_R]                            
{0x0990, 0x007C},      // MCU_DATA_0                                        
{0x098C, 0xA36B},      // MCU_ADDRESS [AWB_KB_R]                            
{0x0990, 0x007E},      // MCU_DATA_0                                        
{0x327A, 0x002E},      // BLACK_LEVEL_1ST_RED  

//Errata for Rev2: issue02                                               
{0x3084, 0x240C},      // RESERVED_CORE_3084                              
{0x3092, 0x0A4C},      // RESERVED_CORE_3092                              
{0x3094, 0x4C4C},      // RESERVED_CORE_3094                              
{0x3096, 0x4C54},      // RESERVED_CORE_3096                              
//CCM AE AWB HG                                                          
{0x098C, 0xA109},      // MCU_ADDRESS [SEQ_AE_FASTBUFF]                     
{0x0990, 0x0006},      // MCU_DATA_0                                        
{0x098C, 0xA10A},      // MCU_ADDRESS [SEQ_AE_FASTSTEP]                     
{0x0990, 0x0001},      // MCU_DATA_0                                        
{0x098C, 0xA10B},      // MCU_ADDRESS [SEQ_AWB_CONTBUFF]                    
{0x0990, 0x0006},      // MCU_DATA_0                                        
{0x098C, 0xA10C},      // MCU_ADDRESS [SEQ_AWB_CONTSTEP]                    
{0x0990, 0x0001},      // MCU_DATA_0                                        
{0x098C, 0xA115},      // MCU_ADDRESS [SEQ_CAP_MODE]  //set capture rate                      
{0x0990, 0x0072},      // MCU_DATA_0                                        
{0x098C, 0xA117},      // MCU_ADDRESS [SEQ_PREVIEW_0_AE]                    
{0x0990, 0x0002},      // MCU_DATA_0                                        
{0x098C, 0xA11D},      // MCU_ADDRESS [SEQ_PREVIEW_1_AE]                    
{0x0990, 0x0002},      // MCU_DATA_0                                        
{0x098C, 0xA123},      // MCU_ADDRESS [SEQ_PREVIEW_2_AE]                    
{0x0990, 0x0002},      // MCU_DATA_0                                        
{0x098C, 0xA129},      // MCU_ADDRESS [SEQ_PREVIEW_3_AE]                    
{0x0990, 0x0002},      // MCU_DATA_0                                        
{0x098C, 0xA207},      // MCU_ADDRESS [AE_GATE]                             
{0x0990, 0x000A},      // MCU_DATA_0                                        
{0x098C, 0xA20D},      // MCU_ADDRESS [AE_MIN_VIRTGAIN]                     
{0x0990, 0x0010},      // MCU_DATA_0                                        
{0x098C, 0xA20C},      // MCU_ADDRESS [AE_MAX_INDEX]  //set fps(0x02-0x24)  smaller, higer fps                     
{0x0990, 0x0012},      // MCU_DATA_0   24                                     
{0x098C, 0xA20E},      // MCU_ADDRESS [AE_MAX_VIRTGAIN]                     
{0x0990, 0x0090},      // MCU_DATA_0                                        
{0x098C, 0x2212},      // MCU_ADDRESS [AE_MAX_DGAIN_AE1]                    
{0x0990, 0x0140},      // MCU_DATA_0                                        
{0x098C, 0x2240},      // MCU_ADDRESS [AE_FINE_IT_MIN]                      
{0x0990, 0x013F},      // MCU_DATA_0                                        
{0x098C, 0x2242},      // MCU_ADDRESS [AE_FINE_IT_MAX_MARGIN]               
{0x0990, 0x00AB},      // MCU_DATA_0                                        
{0x098C, 0xA24F},      // MCU_ADDRESS [AE_BASETARGET]   //set brightness                       
{0x0990, 0x0046},      // MCU_DATA_0       50                               
{0x098C, 0x2304},      // MCU_ADDRESS [AWB_WAKEUPLINE]                      
{0x0990, 0x01F4},      // MCU_DATA_0                                        
{0x098C, 0x2356},      // MCU_ADDRESS [AWB_GAINR_BUF]                       
{0x0990, 0x0000},      // MCU_DATA_0                                        
{0x098C, 0x2358},      // MCU_ADDRESS [AWB_GAINB_BUF]                       
{0x0990, 0x0000},      // MCU_DATA_0                                        
{0x098C, 0x275F},      // MCU_ADDRESS [MODE_COMMONMODESETTINGS_BRIGHT_COLOR_
{0x0990, 0x0596},      // MCU_DATA_0                                        
{0x098C, 0x2761},      // MCU_ADDRESS [MODE_COMMONMODESETTINGS_DARK_COLOR_KI
{0x0990, 0x0094},      // MCU_DATA_0                                        
{0x098C, 0xA765},      // MCU_ADDRESS [MODE_COMMONMODESETTINGS_FILTER_MODE] 
{0x0990, 0x0006},      // MCU_DATA_0                                        
{0x098C, 0xAB04},      // MCU_ADDRESS [HG_MAX_DLEVEL]                       
{0x0990, 0x0023},      // MCU_DATA_0                                        
{0x098C, 0xAB06},      // MCU_ADDRESS [HG_PERCENT]                          
{0x0990, 0x0008},      // MCU_DATA_0                                        
{0x098C, 0xAB07},      // MCU_ADDRESS [HG_GATEPERCENT]                      
{0x0990, 0x0002},      // MCU_DATA_0                                        
{0x098C, 0xAB20},      // MCU_ADDRESS [HG_LL_SAT1]                          
{0x0990, 0x0053},      // MCU_DATA_0                     43            rev3 
{0x098C, 0xAB22},      // MCU_ADDRESS [HG_LL_APCORR1]  //sharpness: default(4/5)                       
{0x0990, 0x0007},      // MCU_DATA_0                                        
{0x098C, 0xAB23},      // MCU_ADDRESS [HG_23]                             
{0x0990, 0x0006},      // MCU_DATA_0         0x0004 -> 0x0006             
{0x098C, 0xAB24},      // MCU_ADDRESS [HG_LL_SAT2]                         
{0x0990, 0x0020},      // MCU_DATA_0   0x003A -> 0x0020                    
{0x098C, 0xAB25},      // MCU_ADDRESS [HG_LL_INTERPTHRESH2]                
{0x0990, 0x0070},      // MCU_DATA_0                                       
{0x098C, 0xAB26}, 	                                                     
{0x0990, 0x0000}, 	                                                     
{0x098C, 0xAB27},      // MCU_ADDRESS [HG_LL_APTHRESH2]                    
{0x0990, 0x0050},      // MCU_DATA_0     0x0000 -> 0x0030                  
{0x098C, 0x2B28},      // MCU_ADDRESS [HG_LL_BRIGHTNESSSTART]              
{0x0990, 0x0F80},      // MCU_DATA_0     0x1130 -> 0x1000                  
{0x098C, 0x2B2A},      // MCU_ADDRESS [HG_LL_BRIGHTNESSSTOP]               
{0x0990, 0x4F00},      // MCU_DATA_0     0x4268 -> 0x4f00                  
{0x098C, 0xAB30},      // MCU_ADDRESS [HG_NR_STOP_R]                        
{0x0990, 0x001E},      // MCU_DATA_0                                        
{0x098C, 0xAB31},      // MCU_ADDRESS [HG_NR_STOP_G]                        
{0x0990, 0x001E},      // MCU_DATA_0                                        
{0x098C, 0xAB32},      // MCU_ADDRESS [HG_NR_STOP_B]                        
{0x0990, 0x001E},      // MCU_DATA_0                                        
{0x098C, 0xAB33},      // MCU_ADDRESS [HG_NR_STOP_OL]                       
{0x0990, 0x001E},      // MCU_DATA_0                                        
{0x098C, 0xAB34},      // MCU_ADDRESS [HG_NR_GAINSTART]                     
{0x0990, 0x0008},      // MCU_DATA_0                                        
{0x098C, 0xAB35},      // MCU_ADDRESS [HG_NR_GAINSTOP]                      
{0x0990, 0x0080},      // MCU_DATA_0                                        
{0x098C, 0xAB37},      // MCU_ADDRESS [HG_GAMMA_MORPH_CTRL]                 
{0x0990, 0x0003},      // MCU_DATA_0                                        
{0x098C, 0x2B38},      // MCU_ADDRESS [HG_GAMMASTARTMORPH]                  
{0x0990, 0x1E14},      // MCU_DATA_0                                        
{0x098C, 0x2B3A},      // MCU_ADDRESS [HG_GAMMASTOPMORPH]                   
{0x0990, 0x6590},      // MCU_DATA_0	
// [Basic Init]19: LOAD=SOC2030_patch
// [SOC2030_patch]3: {0x098C, 0x415 	
{0x098C, 0x0415},
// [SOC2030_patch]4: REG_BURST=0x990, 0xF601, 0x42C1, 0x326, 0x11F6, 0x143, 0xC104, 0x260A, 0xCC04
{0x0990, 0xF601},
{0x0992, 0x42C1},
{0x0994, 0x0326},
{0x0996, 0x11F6},
{0x0998, 0x0143},
{0x099A, 0xC104},
{0x099C, 0x260A},
{0x099E, 0xCC04},
// [SOC2030_patch]5: {0x098C, 0x425 	
{0x098C, 0x0425},
// [SOC2030_patch]6: REG_BURST=0x990, 0x33BD, 0xA362, 0xBD04, 0x3339, 0xC6FF, 0xF701, 0x6439, 0xFE01
{0x0990, 0x33BD},
{0x0992, 0xA362},
{0x0994, 0xBD04},
{0x0996, 0x3339},
{0x0998, 0xC6FF},
{0x099A, 0xF701},
{0x099C, 0x6439},
{0x099E, 0xFE01},
// [SOC2030_patch]7: {0x098C, 0x435 	
 {0x098C, 0x0435},
// [SOC2030_patch]8: REG_BURST=0x990, 0x6918, 0xCE03, 0x25CC, 0x13, 0xBDC2, 0xB8CC, 0x489, 0xFD03
{0x0990, 0x6918},
{0x0992, 0xCE03},
{0x0994, 0x25CC},
{0x0996, 0x0013},
{0x0998, 0xBDC2},
{0x099A, 0xB8CC},
{0x099C, 0x0489},
{0x099E, 0xFD03},
// [SOC2030_patch]9: {0x098C, 0x445 	
 {0x098C, 0x0445},
// [SOC2030_patch]10: REG_BURST=0x990, 0x27CC, 0x325, 0xFD01, 0x69FE, 0x2BD, 0x18CE, 0x339, 0xCC00
{0x0990, 0x27CC},
{0x0992, 0x0325},
{0x0994, 0xFD01},
{0x0996, 0x69FE},
{0x0998, 0x02BD},
{0x099A, 0x18CE},
{0x099C, 0x0339},
{0x099E, 0xCC00},
// [SOC2030_patch]11: {0x098C, 0x455 	
 {0x098C, 0x0455},
// [SOC2030_patch]12: REG_BURST=0x990, 0x11BD, 0xC2B8, 0xCC04, 0xC8FD, 0x347, 0xCC03, 0x39FD, 0x2BD
{0x0990, 0x11BD},
{0x0992, 0xC2B8},
{0x0994, 0xCC04},
{0x0996, 0xC8FD},
{0x0998, 0x0347},
{0x099A, 0xCC03},
{0x099C, 0x39FD},
{0x099E, 0x02BD},
// [SOC2030_patch]13: {0x098C, 0x465 	
{0x098C, 0x0465},
// [SOC2030_patch]14: REG_BURST=0x990, 0xDE00, 0x18CE, 0xC2, 0xCC00, 0x37BD, 0xC2B8, 0xCC04, 0xEFDD
{0x0990, 0xDE00},
{0x0992, 0x18CE},
{0x0994, 0x00C2},
{0x0996, 0xCC00},
{0x0998, 0x37BD},
{0x099A, 0xC2B8},
{0x099C, 0xCC04},
{0x099E, 0xEFDD},
// [SOC2030_patch]15: {0x098C, 0x475 	
{0x098C, 0x0475},
// [SOC2030_patch]16: REG_BURST=0x990, 0xE6CC, 0xC2, 0xDD00, 0xC601, 0xF701, 0x64C6, 0x3F7, 0x165
{0x0990, 0xE6CC},
{0x0992, 0x00C2},
{0x0994, 0xDD00},
{0x0996, 0xC601},
{0x0998, 0xF701},
{0x099A, 0x64C6},
{0x099C, 0x03F7},
{0x099E, 0x0165},
// [SOC2030_patch]17: {0x098C, 0x485 	
{0x098C, 0x0485},
// [SOC2030_patch]18: REG_BURST=0x990, 0x7F01, 0x6639, 0x3C3C, 0x3C34, 0xCC32, 0x3EBD, 0xA558, 0x30ED
{0x0990, 0x7F01},
{0x0992, 0x6639},
{0x0994, 0x3C3C},
{0x0996, 0x3C34},
{0x0998, 0xCC32},
{0x099A, 0x3EBD},
{0x099C, 0xA558},
{0x099E, 0x30ED},
// [SOC2030_patch]19: {0x098C, 0x495 	
 {0x098C, 0x0495},
// [SOC2030_patch]20: REG_BURST=0x990, 0x4BD, 0xB2D7, 0x30E7, 0x6CC, 0x323E, 0xED00, 0xEC04, 0xBDA5
{0x0990, 0x04BD},
{0x0992, 0xB2D7},
{0x0994, 0x30E7},
{0x0996, 0x06CC},
{0x0998, 0x323E},
{0x099A, 0xED00},
{0x099C, 0xEC04},
{0x099E, 0xBDA5},
// [SOC2030_patch]21: {0x098C, 0x4A5 	
{0x098C, 0x04A5},
// [SOC2030_patch]22: REG_BURST=0x990, 0x44CC, 0x3244, 0xBDA5, 0x585F, 0x30ED, 0x2CC, 0x3244, 0xED00
{0x0990, 0x44CC},
{0x0992, 0x3244},
{0x0994, 0xBDA5},
{0x0996, 0x585F},
{0x0998, 0x30ED},
{0x099A, 0x02CC},
{0x099C, 0x3244},
{0x099E, 0xED00},
// [SOC2030_patch]23: {0x098C, 0x4B5 	
 {0x098C, 0x04B5},
// [SOC2030_patch]24: REG_BURST=0x990, 0xF601, 0xD54F, 0xEA03, 0xAA02, 0xBDA5, 0x4430, 0xE606, 0x3838
{0x0990, 0xF601},
{0x0992, 0xD54F},
{0x0994, 0xEA03},
{0x0996, 0xAA02},
{0x0998, 0xBDA5},
{0x099A, 0x4430},
{0x099C, 0xE606},
{0x099E, 0x3838},
// [SOC2030_patch]25: {0x098C, 0x4C5 	
{0x098C, 0x04C5},
// [SOC2030_patch]26: REG_BURST=0x990, 0x3831, 0x39BD, 0xD661, 0xF602, 0xF4C1, 0x126, 0xBFE, 0x2BD
{0x0990, 0x3831},
{0x0992, 0x39BD},
{0x0994, 0xD661},
{0x0996, 0xF602},
{0x0998, 0xF4C1},
{0x099A, 0x0126},
{0x099C, 0x0BFE},
{0x099E, 0x02BD},
// [SOC2030_patch]27: {0x098C, 0x4D5 	
{0x098C, 0x04D5},
// [SOC2030_patch]28: REG_BURST=0x990, 0xEE10, 0xFC02, 0xF5AD, 0x39, 0xF602, 0xF4C1, 0x226, 0xAFE
{0x0990, 0xEE10},
{0x0992, 0xFC02},
{0x0994, 0xF5AD},
{0x0996, 0x0039},
{0x0998, 0xF602},
{0x099A, 0xF4C1},
{0x099C, 0x0226},
{0x099E, 0x0AFE},
// [SOC2030_patch]29: {0x098C, 0x4E5 	
{0x098C, 0x04E5},
// [SOC2030_patch]30: REG_BURST=0x990, 0x2BD, 0xEE10, 0xFC02, 0xF7AD, 0x39, 0x3CBD, 0xB059, 0xCC00
{0x0990, 0x02BD},
{0x0992, 0xEE10},
{0x0994, 0xFC02},
{0x0996, 0xF7AD},
{0x0998, 0x0039},
{0x099A, 0x3CBD},
{0x099C, 0xB059},
{0x099E, 0xCC00},
// [SOC2030_patch]31: {0x098C, 0x4F5 	
{0x098C, 0x04F5},
// [SOC2030_patch]32: REG_BURST=0x990, 0x28BD, 0xA558, 0x8300, 0x27, 0xBCC, 0x26, 0x30ED, 0xC6
{0x0990, 0x28BD},
{0x0992, 0xA558},
{0x0994, 0x8300},
{0x0996, 0x0027},
{0x0998, 0x0BCC},
{0x099A, 0x0026},
{0x099C, 0x30ED},
{0x099E, 0x00C6},
// [SOC2030_patch]33: {0x098C, 0x505 	
{0x098C, 0x0505},
// [SOC2030_patch]34: REG_BURST=0x990, 0x3BD, 0xA544, 0x3839
{0x0990, 0x03BD},
{0x0992, 0xA544},
{0x0994, 0x3839},
// [SOC2030_patch]35: FIELD_WR=MON_ARG1, 0X0415     
{0x098C, 0x2006},      // MCU_ADDRESS [MON_ARG1]
{0x0990, 0x0415},      // MCU_DATA_0
// [SOC2030_patch]36: FIELD_WR=MON_CMD, 1     
{0x098C, 0xA005},      // MCU_ADDRESS [MON_CMD]
{0x0990, 0x0001},      // MCU_DATA_0

{0x098C,	0xA103},      // MCU_ADDRESS                                                         
{0x0990,	0x0005},      // MCU_DATA_0                    
//DELAY=100		                                             
{0x098C, 0xA103},      // MCU_ADDRESS [SEQ_CMD]           
{0x0990, 0x0006},      // MCU_DATA_0  

{0xffff, 0x00ff},
};


static struct regval_list  mt9d113_fmt_capture[]  = {
{0x098C, 0xA115},        
{0x0990, 0x0002},          
{0x098C, 0xA103},          
{0x0990, 0x0002}, 

{0xffff,0x00ff},
};
/*
 * Low-level register I/O.
 */
/*issue that OV sensor must write then read. 3640 register is 16bit!!!*/


static
inline int read_sensor_reg_16bit(struct i2c_client *c, u16 addr, u16 *pvalue)
{

	u8 data[2];        
	
	if( c == NULL )	/* Not initialize the client */	
	{
		printk("not initialize the client\n");
		return -1;
	}
	data[0] = addr>>8;	
	data[1] = addr;

	i2c_master_send(c, data, 2);

	i2c_master_recv(c, data, 2)	;
	*pvalue=(data[0]<<8)|data[1];
	//printk("mt9d113: read reg [%04x]=%02x\n", addr, *pvalue);
	return 0;
}
static
inline int write_sensor_reg_16bit(struct i2c_client *c, u16 addr, u16 *value)
{
	u8 data[4];
	
	if( c == NULL )	/* Not initialize the client */	
	{
		printk("not initialize the client\n");
		return -1;
	}
	data[0] = addr>>8;
	data[1] = addr;	
	data[2]=  *value>>8;
	data[3]= *value;

	//printk("i2c address= %x\n",c->addr);
	
	i2c_master_send(c, data, 4);
	return 0;
}


static int mt9d113_read(struct i2c_client *c, u16 reg,u16 *value)
{
	int ret;

	ret = read_sensor_reg_16bit(c, reg, value);
	
	return ret;
}

static int mt9d113_write(struct i2c_client *c, u16 reg, u16 value)
{
	int ret;

	ret = write_sensor_reg_16bit(c, reg, &value);
	
	return ret;
}


/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int mt9d113_write_array(struct i2c_client *c,struct regval_list *vals)
{
	int i = 0;
	while (vals->reg_num != 0xffff ) {

	int ret = mt9d113_write(c, vals->reg_num, vals->value);
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
static void mt9d113_reset(struct i2c_client *client)
{
//printk("mt9d113_reset\n");

	mt9d113_write(client, REG_SYS, SYS_RESET);
	msleep(1);
}

static int mt9d113_detect(struct i2c_client *client)
{
	u16 v;
	int ret;
	printk("mt9d113_detect \n");
	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	ret = read_sensor_reg_16bit(client, 0x0000, &v);
	if (ret < 0)
		return ret;
	if (v != 0x2580)
		return -ENODEV;
	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct mt9d113_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
	int bpp;   /* bits per pixel */
} mt9d113_formats[] = {
#if 1  
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= mt9d113_fmt_yuv,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },	//TODO
		.bpp		= 16,
	},
	{
		.desc		= "YUYV422 planar",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.regs 		= mt9d113_fmt_yuv,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 16,
	},
	#endif
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.regs           = mt9d113_fmt_yuv,
		.cmatrix        = { 128, -128, 0, -34, -94, 128 },
		.bpp            = 12,
	},
	
};
#define N_MT9D113_FMTS ARRAY_SIZE(mt9d113_formats)

/*TODO - also can use ccic size register 0x34 to do same thing for cropping...anyway, sensor doing it is better?
  0x3020~0x3027*/
static struct mt9d113_win_size {
	int	width;
	int	height;
	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	/* h/vref stuff */
} mt9d113_win_sizes[] = {
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.com7_bit	= 0,//COM7_FMT_VGA, /* see comment above */
		.hstart		= 456,		/* Empirically determined */
		.hstop		=  24,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
		//		.regs 		= mt9d113_qcif_regs,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.com7_bit	=0,// COM7_FMT_QVGA,
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
		.width          = UXGA_WIDTH,
		.height         = UXGA_HEIGHT,
		.com7_bit       = 0,
		.hstart         = 158,          /* These values from */
		.hstop          =  14,          /* Omnivision */
		.vstart         =  10,
		.vstop          = 490,
		.regs           = NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(mt9d113_win_sizes))

#if 0
/*
 * Store a set of start/stop values into the camera.	//TODO - not used for 3640?
 */
static int mt9d113_set_hw(struct i2c_client *client, int hstart, int hstop,
		int vstart, int vstop)
{
	int ret;
	unsigned char v;
	/*
	 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
	 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
	 * a mystery "edge offset" value in the top two bits of href.
	 */
	ret =  mt9d113_write(client, REG_HSTART, (hstart >> 3) & 0xff);
	ret += mt9d113_write(client, REG_HSTOP, (hstop >> 3) & 0xff);
	ret += mt9d113_read(client, REG_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += mt9d113_write(client, REG_HREF, v);
	/*
	 * Vertical: similar arrangement, but only 10 bits.
	 */
	ret += mt9d113_write(client, REG_VSTART, (vstart >> 2) & 0xff);
	ret += mt9d113_write(client, REG_VSTOP, (vstop >> 2) & 0xff);
	ret += mt9d113_read(client, REG_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += mt9d113_write(client, REG_VREF, v);
	return ret;
}
#endif


static int mt9d113_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct mt9d113_format_struct *ofmt;
	//printk("mt9d113_enum_fmt \n");
	if (fmt->index >= N_MT9D113_FMTS)
		return -EINVAL;

	ofmt = mt9d113_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int mt9d113_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct mt9d113_format_struct **ret_fmt,
		struct mt9d113_win_size **ret_wsize)
{
	int index;
	struct mt9d113_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	//printk("mt9d113_try_fmt \n");

	
	for (index = 0; index < N_MT9D113_FMTS; index++)
		if (mt9d113_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_MT9D113_FMTS){
		printk("unsupported format!\n");
		return -EINVAL;
	}
	if (ret_fmt != NULL)
		*ret_fmt = mt9d113_formats + index;
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
	for (wsize = mt9d113_win_sizes; wsize < mt9d113_win_sizes + N_WIN_SIZES;
			wsize++)
		if (pix->width <= wsize->width && pix->height <= wsize->height)
			break;
	if (wsize >= mt9d113_win_sizes + N_WIN_SIZES){
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
	pix->bytesperline = pix->width*mt9d113_formats[index].bpp/8;
	pix->sizeimage = pix->height*pix->bytesperline;
	//printk("mt9d113_try_fmt: pix->width is %d, pix->height is %d wsize %d\n", pix->width, pix->height, wsize->width);

	if (ret_fmt == NULL)
		return 0;
	switch (pix->pixelformat)
	{
		case V4L2_PIX_FMT_YUYV: 
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV420: 
			switch (pix->width)
			{
				case UXGA_WIDTH:
					(*ret_fmt)->regs = mt9d113_fmt_capture;
					break;
		                 default:
					(*ret_fmt)->regs = mt9d113_fmt_yuv;
					break;
			//	default:
				//	printk("unsupported size!\n");
				//	return -EINVAL;
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
static int mt9d113_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
	
	struct mt9d113_format_struct *ovfmt;
	struct mt9d113_win_size *wsize;
	//printk("mt9d113_s_fmt \n");

	ret = mt9d113_try_fmt(c, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;
	fmt_cnt++;
	if(fmt_cnt<2)
	{ 
	    mt9d113_write(c, 0x301A, 0x1218);// RESET_REGISTER
            msleep(3);
            mt9d113_write(c, 0x301A, 0x121C);// RESET_REGISTER
            msleep(10);


    //        mt9d113_write(c, 0x001A, 0x0051);// RESET_AND_MISC_CONTROL
       //     msleep(1);
          //  mt9d113_write(c, 0x001A, 0x0050);// RESET_AND_MISC_CONTROL

           // mt9d113_write(c, 0x0014, 0x2545); //PLL Control: BYPASS PLL = 9541
	   // mt9d113_write(c, 0x0010, 0x0110);//PLL Dividers = 272
          //  mt9d113_write(c, 0x0012, 0x1FF7);   //PLL P Dividers = 8183
         //   mt9d113_write(c, 0x0014, 0x2547);   //PLL Control: PLL_ENABLE on = 9543
        //    mt9d113_write(c, 0x0014, 0x2447);//PLL Control: SEL_LOCK_DET on = 9287
         
	//   msleep(1);               // Allow PLL to lock
       //     mt9d113_write(c, 0x0014, 0x2047);//PLL Control: PLL_BYPASS off = 8263
       //     mt9d113_write(c, 0x0014, 0x2046);//PLL Control: = 8262

           // mt9d113_write(c, 0x001E, 0x0777);//songhaijun

     //       msleep(1);
         
	 //   mt9d113_write(c, 0x001A, 0x0050);		 	// RESET_AND_MISC_CONTROL
	//    mt9d113_write(c, 0x0018, 0x4028); 	// STANDBY_CONTROL
	//    msleep(100);
	    mt9d113_write_array(c, ovfmt->regs);
	}
	//	mt9d113_set_hw();	//TODO
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int mt9d113_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	u16 clkrc;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	ret = mt9d113_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = MT9D113_FRAME_RATE;
	//if ((clkrc & CLK_EXT) == 0 && (clkrc & CLK_SCALE) > 1)
	//	cp->timeperframe.denominator /= (clkrc & CLK_SCALE);
	return 0;
}

static int mt9d113_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	u16 clkrc;
	int ret, div;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;
	/*
	 * CLKRC has a reserved bit, so let's preserve it.
	 */
	ret = mt9d113_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator*MT9D113_FRAME_RATE)/tpf->denominator;
	if (div == 0)
		div = 1;
//	else if (div > CLK_SCALE)
	//	div = CLK_SCALE;
	clkrc = (clkrc & 0x80) | div;
	tpf->numerator = 1;
	tpf->denominator = MT9D113_FRAME_RATE/div;
	return mt9d113_write(c, REG_CLKRC, clkrc);
}

static int mt9d113_s_input(struct i2c_client *c, int *id)
{
     printk("mt9d113_s_input \n");

	return 0;
}



/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */
#if 0
int g_brightness;
int g_hue;
int g_saturation;
int g_contrast;
int g_vflip;
int g_hflip;
int g_whitebalance;
int g_exposure;

static int mt9d113_t_sat(struct i2c_client *client, int value)
{

	int ret = 0;
	unsigned char regval;

	mt9d113_read(client, REG_BRIGHT1, &regval);
	regval |= 0x02;
	switch (value)
	{
		case 0:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			ret = mt9d113_write(client, REG_SAT0, 0x10);
			ret = mt9d113_write(client, REG_SAT1, 0x10);
		break;

		case 1:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			ret = mt9d113_write(client, REG_SAT0, 0x30);
			ret = mt9d113_write(client, REG_SAT1, 0x30);
		break;

		case 2:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			ret = mt9d113_write(client, REG_SAT0, 0x40);
			ret = mt9d113_write(client, REG_SAT1, 0x40);
		break;

		case 3:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			ret = mt9d113_write(client, REG_SAT0, 0x50);
			ret = mt9d113_write(client, REG_SAT1, 0x50);
		break;

		case 4:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			ret = mt9d113_write(client, REG_SAT0, 0x70);
			ret = mt9d113_write(client, REG_SAT1, 0x70);
		break;
	}
	g_saturation = value;
	return ret;
}

static int mt9d113_q_sat(struct i2c_client *client, __s32 *value)
{
	*value = g_saturation;
	return 0;
}

static int mt9d113_t_hue(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;
	mt9d113_read(client, REG_HUE10, &regval);
	switch (value)
	{
		case 1:/*90 degree*/
			ret = mt9d113_write(client, REG_HUE0, 0x20);
			ret = mt9d113_write(client, REG_HUE1, 0x64);
			ret = mt9d113_write(client, REG_HUE2, 0x08);
			ret = mt9d113_write(client, REG_HUE3, 0x7f);
			ret = mt9d113_write(client, REG_HUE4, 0x78);
			ret = mt9d113_write(client, REG_HUE5, 0x06);
			ret = mt9d113_write(client, REG_HUE6, 0x3d);
			ret = mt9d113_write(client, REG_HUE7, 0xb6);
			ret = mt9d113_write(client, REG_HUE8, 0xf3);
			ret = mt9d113_write(client, REG_HUE9, 0xc8);
			regval &= ~0x2;
			ret = mt9d113_write(client, REG_HUE10, regval);
		break;

		case 2:/*180 degree*/
			ret = mt9d113_write(client, REG_HUE0, 0x20);
			ret = mt9d113_write(client, REG_HUE1, 0x64);
			ret = mt9d113_write(client, REG_HUE2, 0x08);
			ret = mt9d113_write(client, REG_HUE3, 0x30);
			ret = mt9d113_write(client, REG_HUE4, 0x90);
			ret = mt9d113_write(client, REG_HUE5, 0xc0);
			ret = mt9d113_write(client, REG_HUE6, 0xa0);
			ret = mt9d113_write(client, REG_HUE7, 0x98);
			ret = mt9d113_write(client, REG_HUE8, 0x08);
			ret = mt9d113_write(client, REG_HUE9, 0x60);
			regval &= ~0x2;
			ret = mt9d113_write(client, REG_HUE10,regval);
		break;

		case 3:/*-90 degree*/
			ret = mt9d113_write(client, REG_HUE0, 0x20);
			ret = mt9d113_write(client, REG_HUE1, 0x64);
			ret = mt9d113_write(client, REG_HUE2, 0x08);
			ret = mt9d113_write(client, REG_HUE3, 0x7f);
			ret = mt9d113_write(client, REG_HUE4, 0x78);
			ret = mt9d113_write(client, REG_HUE5, 0x06);
			ret = mt9d113_write(client, REG_HUE6, 0x3d);
			ret = mt9d113_write(client, REG_HUE7, 0xb6);
			ret = mt9d113_write(client, REG_HUE8, 0xf3);
			ret = mt9d113_write(client, REG_HUE9, 0x30);
			regval |= 0x2;
			ret = mt9d113_write(client, REG_HUE10,regval);
		break;

		case 0:/*0 degree*/
		default:
			ret = mt9d113_write(client, REG_HUE0, 0x20);
			ret = mt9d113_write(client, REG_HUE1, 0x64);
			ret = mt9d113_write(client, REG_HUE2, 0x08);
			ret = mt9d113_write(client, REG_HUE3, 0x30);
			ret = mt9d113_write(client, REG_HUE4, 0x90);
			ret = mt9d113_write(client, REG_HUE5, 0xc0);
			ret = mt9d113_write(client, REG_HUE6, 0xa0);
			ret = mt9d113_write(client, REG_HUE7, 0x98);
			ret = mt9d113_write(client, REG_HUE8, 0x08);
			ret = mt9d113_write(client, REG_HUE9, 0x98);
			regval |= 0x2;
			ret = mt9d113_write(client, REG_HUE10,regval);
		break;
	}
	g_hue = value;
	return ret;
}


static int mt9d113_q_hue(struct i2c_client *client, __s32 *value)
{
	*value = g_hue;
	return 0;
}

static int mt9d113_t_brightness(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval, regvalue;

	mt9d113_read(client, REG_BRIGHT1, &regval);//55
	regval |= 0x04;
	mt9d113_read(client, REG_BRIGHT2, &regvalue);//54
	switch (value)
	{
		case 0:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			regvalue |= 0x08;
			ret = mt9d113_write(client, REG_BRIGHT2, regvalue);
			ret = mt9d113_write(client, REG_BRIGHT3, 0x20);
		break;

		case 1:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			regvalue |= 0x08;
			ret = mt9d113_write(client, REG_BRIGHT2, regvalue);
			ret = mt9d113_write(client, REG_BRIGHT3, 0x10);
		break;

		case 3:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = mt9d113_write(client, REG_BRIGHT2, regvalue);
			ret = mt9d113_write(client, REG_BRIGHT3, 0x10);
		break;

		case 4:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = mt9d113_write(client, REG_BRIGHT2, regvalue);
			ret = mt9d113_write(client, REG_BRIGHT3, 0x20);
		break;

		case 2:
		default:
			ret = mt9d113_write(client, REG_BRIGHT0, 0xef);
			ret = mt9d113_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = mt9d113_write(client, REG_BRIGHT2, regvalue);
			ret = mt9d113_write(client, REG_BRIGHT3, 0x00);
		break;
	}
	g_brightness = value;
	return ret;
}

static int mt9d113_q_brightness(struct i2c_client *client, __s32 *value)
{
	*value = g_brightness;
	return 0;
}

static int mt9d113_t_contrast(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval0, regval1;

	mt9d113_read(client, REG_BRIGHT1, &regval0);//55
	mt9d113_read(client, REG_BRIGHT2, &regval1);//54
	regval1 &= ~0x04;
	regval0 |= 0x04;

	switch (value)
	{
		case 0:
			ret = mt9d113_write(client, REG_BRIGHT1, regval0);
			ret = mt9d113_write(client, REG_BRIGHT2, regval1);
			ret = mt9d113_write(client, REG_CONTRAS0, 0x08);
			ret = mt9d113_write(client, REG_CONTRAS1, 0x08);
		break;

		case 1:
			ret = mt9d113_write(client, REG_BRIGHT1, regval0);
			ret = mt9d113_write(client, REG_BRIGHT2, regval1);
			ret = mt9d113_write(client, REG_CONTRAS0, 0x18);
			ret = mt9d113_write(client, REG_CONTRAS1, 0x18);
		break;

		case 3:
			ret = mt9d113_write(client, REG_BRIGHT1, regval0);
			ret = mt9d113_write(client, REG_BRIGHT2, regval1);
			ret = mt9d113_write(client, REG_CONTRAS0, 0x40);
			ret = mt9d113_write(client, REG_CONTRAS1, 0x40);
		break;

		case 4:
			ret = mt9d113_write(client, REG_BRIGHT1, regval0);
			ret = mt9d113_write(client, REG_BRIGHT2, regval1);
			ret = mt9d113_write(client, REG_CONTRAS0, 0x80);
			ret = mt9d113_write(client, REG_CONTRAS1, 0x80);
		break;

		case 2:
		default:
			ret = mt9d113_write(client, REG_BRIGHT1, regval0);
			ret = mt9d113_write(client, REG_BRIGHT2, regval1);
			ret = mt9d113_write(client, REG_CONTRAS0, 0x20);
			ret = mt9d113_write(client, REG_CONTRAS1, 0x20);
		break;
	}
	g_brightness = value;
	return ret;
}

static int mt9d113_q_contrast(struct i2c_client *client, __s32 *value)
{
	*value = g_contrast;
	return 0;
}

static int mt9d113_q_hflip(struct i2c_client *client, __s32 *value)
{
	*value = g_hflip;
	return 0;
}


static int mt9d113_t_hflip(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval1, regval2;

	mt9d113_read(client, REG_MVFP1, &regval1);
	mt9d113_read(client, REG_MVFP2, &regval2);
	if (value){
		regval1 |= 0x02;
		regval2 |= 0x08;
		ret = mt9d113_write(client, REG_MVFP1, regval1);
		ret = mt9d113_write(client, REG_MVFP2, regval2);
	} else {
		regval1 &= ~0x02;
		regval2 &= ~0x08;
		ret = mt9d113_write(client, REG_MVFP1, regval1);
		ret = mt9d113_write(client, REG_MVFP2, regval2);
	}
	g_hflip = value;
	return ret;
}



static int mt9d113_q_vflip(struct i2c_client *client, __s32 *value)
{
	*value = g_vflip;
	return 0;
}


static int mt9d113_t_vflip(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;
	mt9d113_read(client, REG_MVFP1, &regval);
	if (value){
		ret = mt9d113_write(client, REG_MVFP0, 0x9);
		regval |= 0x1;
		ret = mt9d113_write(client, REG_MVFP1, regval);
	} else {
		ret = mt9d113_write(client, REG_MVFP0, 0xa);
		regval &= ~0x1;
		ret = mt9d113_write(client, REG_MVFP1, regval);
	}
	g_vflip = value;
	return ret;
}

static int mt9d113_t_whitebalance(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;

	mt9d113_read(client, REG_WB0, &regval);
	switch (value)
	{
		case 0:/*sunny*/
			regval |= 0x08;
			ret = mt9d113_write(client, REG_WB0, regval);
			ret = mt9d113_write(client, REG_WB1, 0x5e);
			ret = mt9d113_write(client, REG_WB2, 0x40);
			ret = mt9d113_write(client, REG_WB3, 0x46);
		break;

		case 1:/*cloudy*/
			regval |= 0x08;
			ret = mt9d113_write(client, REG_WB0, regval);
			ret = mt9d113_write(client, REG_WB1, 0x68);
			ret = mt9d113_write(client, REG_WB2, 0x40);
			ret = mt9d113_write(client, REG_WB3, 0x4e);
		break;

		case 3:/*office*/
			regval |= 0x08;
			ret = mt9d113_write(client, REG_WB0, regval);
			ret = mt9d113_write(client, REG_WB1, 0x52);
			ret = mt9d113_write(client, REG_WB2, 0x40);
			ret = mt9d113_write(client, REG_WB3, 0x58);
		break;

		case 4:/*home*/
			regval |= 0x08;
			ret = mt9d113_write(client, REG_WB0, regval);
			ret = mt9d113_write(client, REG_WB1, 0x44);
			ret = mt9d113_write(client, REG_WB2, 0x40);
			ret = mt9d113_write(client, REG_WB3, 0x70);
		break;

		case 2:
		default:
			regval &= ~0x08;
			ret = mt9d113_write(client, REG_WB0, regval);
		break;
	}
	g_whitebalance = value;
	return ret;
}

static int mt9d113_q_whitebalance(struct i2c_client *client, __s32 *value)
{
	*value = g_whitebalance;
	return 0;
}

static int mt9d113_t_exposure(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval0;

	mt9d113_read(client, REG_EXPOSURE0, &regval0);
	regval0 &= ~0x80;
	mt9d113_write(client, REG_EXPOSURE0, regval0);
	switch (value)
	{
		case 0:
			ret = mt9d113_write(client, REG_EXPOSURE1, 0x10);
			ret = mt9d113_write(client, REG_EXPOSURE2, 0x08);
			ret = mt9d113_write(client, REG_EXPOSURE3, 0x21);
		break;

		case 1:
			ret = mt9d113_write(client, REG_EXPOSURE1, 0x20);
			ret = mt9d113_write(client, REG_EXPOSURE2, 0x18);
			ret = mt9d113_write(client, REG_EXPOSURE3, 0x41);
		break;

		case 3:
			ret = mt9d113_write(client, REG_EXPOSURE1, 0x48);
			ret = mt9d113_write(client, REG_EXPOSURE2, 0x40);
			ret = mt9d113_write(client, REG_EXPOSURE3, 0x81);
		break;

		case 4:
			ret = mt9d113_write(client, REG_EXPOSURE1, 0x60);
			ret = mt9d113_write(client, REG_EXPOSURE2, 0x58);
			ret = mt9d113_write(client, REG_EXPOSURE3, 0xa1);
		break;

		case 2:
		default:
			ret = mt9d113_write(client, REG_EXPOSURE1, 0x28);
			ret = mt9d113_write(client, REG_EXPOSURE2, 0x20);
			ret = mt9d113_write(client, REG_EXPOSURE3, 0x51);
		break;
	}
	g_exposure = value;
	return ret;
}

static int mt9d113_q_exposure(struct i2c_client *client, __s32 *value)
{
	*value = g_exposure;
	return 0;
}

#endif

static struct mt9d113_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} mt9d113_controls[] =
{
#if 0
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
		.tweak = mt9d113_t_brightness,
		.query = mt9d113_q_brightness,
	},
	{
		.qc = {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,   /* XXX mt9d113 spec */
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = mt9d113_t_contrast,
		.query = mt9d113_q_contrast,
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
		.tweak = mt9d113_t_sat,
		.query = mt9d113_q_sat,
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
		.tweak = mt9d113_t_hue,
		.query = mt9d113_q_hue,
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
		.tweak = mt9d113_t_vflip,
		.query = mt9d113_q_vflip,
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
		.tweak = mt9d113_t_hflip,
		.query = mt9d113_q_hflip,
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
		.tweak = mt9d113_t_whitebalance,
		.query = mt9d113_q_whitebalance,
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
		.tweak = mt9d113_t_exposure,
		.query = mt9d113_q_exposure,
	},
	#endif
};
#define N_CONTROLS (ARRAY_SIZE(mt9d113_controls))

static struct mt9d113_control *mt9d113_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (mt9d113_controls[i].qc.id == id)
			return mt9d113_controls + i;
	return NULL;
}


static int mt9d113_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct mt9d113_control *ctrl = mt9d113_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int mt9d113_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct mt9d113_control *octrl = mt9d113_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int mt9d113_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct mt9d113_control *octrl = mt9d113_find_control(ctrl->id);
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
static int __devinit mt9d113_probe(struct i2c_client *client, const struct i2c_device_id * i2c_id)
{
	int ret = 0;
	struct mt9d113_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;
	printk("mt9d113_probe \n");
	ccic_set_clock_parallel();

	pdata->power_on(1, 1);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct mt9d113_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &mt9d113_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);
	/*
	 * Make sure it's an mt9d113
	 */
	
	ret = mt9d113_detect(client);
	if (ret)
		goto out_free;
	printk(KERN_NOTICE "OmniVision mt9d113 sensor detected\n");
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


static int mt9d113_remove(struct i2c_client *client)
{
	return 0;	//TODO
}


static int mt9d113_streamon(struct i2c_client *client)
{
	u16 val;
	mt9d113_read(client, 0x0018, &val);
	val &= ~0x07;
	mt9d113_write(client, 0x0018, val);
	
	mt9d113_write(client, 0x098C, 0xA103);
	mt9d113_write(client, 0x0990, 0x0006);
	
	return 0;
}

static int mt9d113_streamoff(struct i2c_client *client)
{
	u16 val;
	fmt_cnt=0;
	mt9d113_read(client, 0x0018,  &val);
	val |= 0x07;
	mt9d113_write(client, 0x0018, val);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9d113_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return mt9d113_read(client, (u16)reg->reg, (u16 *)&(reg->val));
}

static int mt9d113_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return mt9d113_write(client, (u16)reg->reg, (u16)reg->val);
}
#endif
static int mt9d113_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
//printk("mt9d113_command:cmd=%u\n",cmd);
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_MT9D113, 0);

		case VIDIOC_INT_RESET:
			mt9d113_reset(client);
			return 0;
#if 0 
		case VIDIOC_INT_INIT:
			return mt9d113_init(client);		//TODO - should get 3640 default register values
#endif

		case VIDIOC_ENUM_FMT:
			return mt9d113_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_TRY_FMT:
			return mt9d113_try_fmt(client, (struct v4l2_format *) arg, NULL, NULL);
		case VIDIOC_S_FMT:
			return mt9d113_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_QUERYCTRL:
			return mt9d113_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return mt9d113_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return mt9d113_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return mt9d113_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return mt9d113_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return mt9d113_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return mt9d113_streamon(client);
		case VIDIOC_STREAMOFF:
			return mt9d113_streamoff(client);
#ifdef CONFIG_VIDEO_ADV_DEBUG
		case VIDIOC_DBG_G_REGISTER:
			return mt9d113_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return mt9d113_s_register(client, (struct v4l2_dbg_register *) arg);
#endif
	}
	return -EINVAL;
}

static struct i2c_device_id mt9d113_idtable[] = {
	{ "mt9d113", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mt9d113_idtable);

static struct i2c_driver mt9d113_driver = {
	.driver = {
		.name	= "mt9d113",
	},
	.id_table       = mt9d113_idtable,
	.command	= mt9d113_command,
	.probe		= mt9d113_probe,
	.remove		= mt9d113_remove,
};


/*
 * Module initialization
 */
static int __init mt9d113_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision mt9d113 sensor driver, at your service\n");
	return i2c_add_driver(&mt9d113_driver);
}

static void __exit mt9d113_mod_exit(void)
{
	i2c_del_driver(&mt9d113_driver);
}

late_initcall(mt9d113_mod_init);
//module_init(mt9d113_mod_init);
module_exit(mt9d113_mod_exit);

