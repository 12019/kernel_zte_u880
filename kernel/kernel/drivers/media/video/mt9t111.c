/*
 * A V4L2 driver for OmniVision MT9T111 cameras.
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
MODULE_DESCRIPTION("A low-level driver for OmniVision mt9t111 sensors");
MODULE_LICENSE("GPL");

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define QXGA_WIDTH	2048
#define QXGA_HEIGHT	1536
#define UXGA_WIDTH	1600
#define UXGA_HEIGHT   1200
#define XGA_WIDTH	1024
#define XGA_HEIGHT      768
#define D1_WIDTH	        720
#define D1_HEIGHT        480
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH 	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define QCIF_HEIGHT	144

/*
 * Our nominal (default) frame rate.
 */
#define MT9T111_FRAME_RATE 30

/*
 * The 3640 sits on i2c with ID 0x42
 */
#define MT9T111_I2C_ADDR 0x3c



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
/*for MT9T111 porting*/
#define REG_CLKRC	0x0010
#define REG_SYS		0x001A
#define SYS_RESET	        0x0001
#define MAXARRAYLENGTH	      3000
#define CAPTURE_MODE_PREVIEW    0
#define CAPTURE_MODE_STILL    1
#define FALSE    -1
/*
 * Information we maintain about a known sensor.
 */
struct mt9t111_format_struct;  /* coming later */
struct mt9t111_info {
	struct mt9t111_format_struct *fmt;  /* Current format */
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

//static int fmt_cnt=0;

/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 * RGB656 and YUV422 come from OV; RGB444 is homebrewed.
 *
 * IMPORTANT RULE: the first entry must be for COM7, see mt9t111_s_fmt for why.
 */
/*TODO - mt9t111_fmt_yuv422_qxga can't work. configuration should be correct*/
//lyw_5242
static struct regval_list  mt9t111_fmt_yuv[]  = {
    {0x0018  ,  0x4028},    //STANDBY_CONTROL_AND_STATUS
//enable parallel interface    disable MIPI
    {0x001A  ,  0x0218},    //RESET_AND_MISC_CONTROL
    {0x001A  ,  0x0218},    //RESET_AND_MISC_CONTROL
    {0x001E  ,  0x0703},    //PAD_SLEW_PAD_CONFIG
//optimal power consumption
    {0x3084 , 0x2406},    //RESERVED_CORE_3084 DAC_LD_4_5  
    {0x3092 , 0x0A46},    //RESERVED_CORE_3092 DAC_LD_18_19
    {0x3094 , 0x4646},    //RESERVED_CORE_3094 DAC_LD_20_21
    {0x3096 , 0x4649},    //RESERVED_CORE_3096 DAC_LD_22_23
//power consumption reduction by disable JPEG CLK
//    {0x0016   , 0x02DF},//},    //0x0400},    //CLOCKS_CONTROL  林狼!!
//Disabel adaptive clock
    {0x098E ,   0x68A0},    //MCU_ADDRESS [PRI_A_CONFIG_JPEG_OB_TX_CONTROL_VAR]
    {0x0990 ,   0x082E},    //MCU_DATA_0
    {0x098E ,   0x6CA0},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR]
    {0x0990 ,   0x082E},    //MCU_DATA_0
//invert PCLK back to Rev2 mode
    {0x3C20,  0x0001},      // TX_SS_CONTROL
//temp noise   ,
    {0x316C  ,  0x350F},    //0x0406           
//AFM driver & I2C initialize
//master I2C enable
    {0x0614  ,  0x0000},    //SECOND_SCL_SDA_PD
   
//[timing set] ,
    {0x098E  ,  0x6800},    //Output Width (A) 
    {0x0990,  640},         //      = 800     
    {0x098E,  0x6802},      //Output Height (A)
    {0x0990,  480},        //      = 600     
    {0x098E  ,  0xE88E},    //JPEG (A)         
{0x0990, 0x00},      //      = 0
    {0x098E  ,  0x68A0},    //Adaptive Output Clock (A)
{0x0990, 0x082D },      // MCU_DATA_0
    {0x098E  ,  0x4802},    //Row Start (A)            
{0x0990, 0x000},      //      = 0
    {0x098E  ,  0x4804},    //Column Start (A)         
{0x0990, 0x000},      //      = 0
    {0x098E  ,  0x4806},    //Row End (A)              
{0x0990, 0x60D},      //      = 1549
    {0x098E  ,  0x4808},    //Column End (A)           
{0x0990, 0x80D},      //      = 2061
    {0x098E  ,  0x480A},    //Row Speed (A)            
    {0x0990  ,  0x0111},    //     = 273              
    {0x098E  ,  0x480C},    //Read Mode (A)            
    {0x0990  ,  0x046C},    //     = 1132             
    {0x098E  ,  0x480F},    //Fine Correction (A)      
    {0x0990  ,  0x00CC},    //     = 204              
    {0x098E  ,  0x4811},    //Fine IT Min (A)          
    {0x0990  ,  0x0381},    //     = 897              
    {0x098E  ,  0x4813},    //Fine IT Max Margin (A)   
    {0x0990  ,  0x024F},    //     = 591              
    {0x098E  ,  0x481D},    //Base Frame Lines (A)     
{0x0990, 0x035B},      //      = 859
    {0x098E  ,  0x481F},    //Min Line Length (A)      
    {0x0990  ,  0x05D0},    //     = 1488             
    {0x098E  ,  0x4825},    //Line Length (A)          
{0x0990, 0x07F2},      //      = 2034
    {0x098E  ,  0x482B},    //Contex Width (A)         
    {0x0990  ,  0x0408},    //     = 1032             
    {0x098E  ,  0x482D},    //Context Height (A)       
    {0x0990  ,  0x0308},    //     = 776              
    {0x098E  ,  0x6C00},    //Output Width (B)         
    {0x0990  ,  0x0800},    //     = 2048             
    {0x098E  ,  0x6C02},    //Output Height (B)        
    {0x0990  ,  0x0600},    //     = 1536             
    {0x098E  ,  0xEC8E},    //JPEG (B)                 
{0x0990, 0x00},      //      = 0
    {0x098E  ,  0x6CA0},    //Adaptive Output Clock (B)
{0x0990, 0x082D },      // MCU_DATA_0
    {0x098E  ,  0x484A},    //Row Start (B)         
{0x0990, 0x004},      //      = 4
    {0x098E  ,  0x484C},    //Column Start (B)      
{0x0990, 0x004},      //      = 4
    {0x098E  ,  0x484E},    //Row End (B)           
{0x0990, 0x60B},      //      = 1547
    {0x098E  ,  0x4850},    //Column End (B)        
{0x0990, 0x80B},      //      = 2059
    {0x098E  ,  0x4852},    //Row Speed (B)         
    {0x0990  ,  0x0111},    //     = 273           
    {0x098E  ,  0x4854},    //Read Mode (B)         
    {0x0990  ,  0x0024},    //     = 36            
    {0x098E  ,  0x4857},    //Fine Correction (B)   
    {0x0990  ,  0x008C},    //     = 140           
    {0x098E  ,  0x4859},    //Fine IT Min (B)       
    {0x0990  ,  0x01F1},    //     = 497           
    {0x098E  ,  0x485B},    //Fine IT Max Margin (B)
    {0x0990  ,  0x00FF},    //     = 255           
    {0x098E  ,  0x4865},    //Base Frame Lines (B)  
{0x0990, 0x065D},      //      = 1629
    {0x098E  ,  0x4867},    //Min Line Length (B)   
    {0x0990  ,  0x0378},    //     = 888           
    {0x098E  ,  0x486D},    //Line Length (B)       
{0x0990, 0x0F69},      //      = 3945
    {0x098E  ,  0x4873},    //Contex Width (B)      
    {0x0990  ,  0x0808},    //     = 2056          
    {0x098E  ,  0x4875},    //Context Height (B)    
    {0x0990  ,  0x0608},    //     = 1544          
    {0x098E  ,  0xC8A5},    //search_f1_50          
{0x0990, 0x26},      //      = 38
    {0x098E  ,  0xC8A6},    //search_f2_50          
{0x0990, 0x28},      //      = 40
    {0x098E  ,  0xC8A7},    //search_f1_60          
{0x0990, 0x2E},      //      = 46
    {0x098E  ,  0xC8A8},    //search_f2_60          
{0x0990, 0x30},      //      = 48
    {0x098E  ,  0xC844},    //period_50Hz (A)       
{0x0990, 0xEC},      //      = 236
    {0x098E  ,  0xC92F},    //period_50Hz (A MSB)   
{0x0990, 0x00},      //      = 0
    {0x098E  ,  0xC845},    //period_60Hz (A)       
{0x0990, 0xC5},      //      = 197
    {0x098E  ,  0xC92D},    //period_60Hz (A MSB)   
{0x0990, 0x00},      //      = 0
    {0x098E  ,  0xC88C},    //period_50Hz (B)       
{0x0990, 0x7A},      //      = 122
    {0x098E  ,  0xC930},    //period_50Hz (B) MSB   
{0x0990, 0x00},      //      = 0
    {0x098E  ,  0xC88D},    //period_60Hz (B)       
{0x0990, 0x65},      //      = 101
{0x098E  ,  0xC92E},    //period_60Hz (B) MSB   
{0x0990, 0x00},      //      = 0
{0x098E, 0xB825},      //FD Window Height
{0x0990, 0x05},      //      = 5
{0x098E  ,  0xA009},    //Stat_min              
{0x0990, 0x02},      //      = 2
{0x098E  ,  0xA00A},    //Stat_max              
{0x0990, 0x03},      //      = 3
{0x098E  ,  0xA00C},    //Min_amplitude         
{0x0990, 0x0A},      //      = 10
{0x098E  ,  0x4846},    //RX FIFO Watermark (A) 
{0x0990, 0x0080},      //      = 128
 {0x098E  ,  0x68AA},    //TX FIFO Watermark (A) 
//{0x0990, 0x02E8},	//		= 744

{0x0990, 0x02C4},	//		= 744		__changed_jpeg__	
{0x098E, 0x488E},	   //RX FIFO Watermark (B)
{0x0990, 0x0080},	   //	   = 128
{0x098E, 0x6CAA},	   //TX FIFO Watermark (B)
//{0x0990, 0x008A}, 	 // 	 = 138
{0x0990, 0x0400},	   //	   = 138		__changed_jpeg__
{0x098E  ,  0x6815},    //Max FD Zone 50 Hz     
{0x0990, 0x0007},      //      = 4
{0x098E  ,  0x6817},    //Max FD Zone 60 Hz     
{0x0990, 0x0009},      //      = 5
{0x098E  ,  0x682D},    //AE Target FD Zone     
{0x0990, 0x0007},      //      = 4
	////////////////////////////////////////////////////////
	
	
//	{0x31E0, 0x0003},	//PIX_DEF_ID

//	{0x3C20, 0x0000},	//TX_SS_CONTROL

//  Lens register settings for MI-3130SOC (MT9T111) REV3
	//lsc 85%
	{0x3640, 0x01B0},
	{0x3642, 0x6EA9},
	{0x3644, 0x1D11},
	{0x3646, 0x910B},
	{0x3648, 0xA450},

	{0x364A, 0x0130},
	{0x364C, 0x7BCA},
	{0x364E, 0x2AD1},
	{0x3650, 0x23AB},
	{0x3652, 0x0FAD},
	{0x3654, 0x0110},
	{0x3656, 0x630B},
	{0x3658, 0x1991},
	{0x365A, 0xBAEC},
	{0x365C, 0xDECF},
	{0x365E, 0x7BEF},
	{0x3660, 0xFA0A},
	{0x3662, 0x0A71},
	{0x3664, 0x01A8},
	{0x3666, 0x888F},
	{0x3680, 0xC4AB},
	{0x3682, 0x948D},
	{0x3684, 0x056C},
	{0x3686, 0x6C2D},
	{0x3688, 0xEF4E},
	{0x368A, 0xDAE8},
	{0x368C, 0x206F},
	{0x368E, 0xB08F},
	{0x3690, 0x830E},
	{0x3692, 0x2450},
	{0x3694, 0x8D4C},
	{0x3696, 0x1E2C},
	{0x3698, 0xB4CC},
	{0x369A, 0x8AAE},
	{0x369C, 0xABCE},
	{0x369E, 0xFFEA},
	{0x36A0, 0x2A2F},
	{0x36A2, 0x8A0F},
	{0x36A4, 0xC3CE},
	{0x36A6, 0x610F},
	{0x36C0, 0x73F1},
	{0x36C2, 0x8D4F},
	{0x36C4, 0xE891},
	{0x36C6, 0x4410},
	{0x36C8, 0x1493},
	{0x36CA, 0x0992},
	{0x36CC, 0xC54D},
	{0x36CE, 0x2051},
	{0x36D0, 0xF151},
	{0x36D2, 0xFDB3},
	{0x36D4, 0x1432},
	{0x36D6, 0xFBAF},
	{0x36D8, 0x94B3},
	{0x36DA, 0x68B1},
	{0x36DC, 0x43F4},
	{0x36DE, 0x6A71},
	{0x36E0, 0xAF6E},
	{0x36E2, 0x3251},
	{0x36E4, 0xAB0E},
	{0x36E6, 0xCF53},
	{0x3700, 0x236E},
	{0x3702, 0x81D0},
	{0x3704, 0x0D11},
	{0x3706, 0x3B92},
	{0x3708, 0x8634},
	{0x370A, 0x632D},
	{0x370C, 0x0410},
	{0x370E, 0x5C90},
	{0x3710, 0xCE30},
	{0x3712, 0xC4F3},
	{0x3714, 0x4E6F},
	{0x3716, 0xA210},
	{0x3718, 0xA991},
	{0x371A, 0x66D2},
	{0x371C, 0x530C},
	{0x371E, 0x40EF},
	{0x3720, 0x72EE},
	{0x3722, 0x2BB1},
	{0x3724, 0xEED1},
	{0x3726, 0xC114},
	{0x3740, 0x85D2},
	{0x3742, 0x0BD2},
	{0x3744, 0xDC54},
	{0x3746, 0xAAF4},
	{0x3748, 0x0838},
	{0x374A, 0xA112},
	{0x374C, 0x3A30},
	{0x374E, 0x83B5},
	{0x3750, 0xA553},
	{0x3752, 0x1478},
	{0x3754, 0xA133},
	{0x3756, 0x00D3},
	{0x3758, 0x5B54},
	{0x375A, 0xA515},
	{0x375C, 0x51F6},
	{0x375E, 0xC691},
	{0x3760, 0x73D0},
	{0x3762, 0xF635},
	{0x3764, 0xCAF2},
	{0x3766, 0x3DD8},
	{0x3782, 0x02C0},
	{0x3784, 0x0450},
	{0x3210, 0x01B8},	//COLOR_PIPELINE_CONTROL

{0x098E, 0xC913},     // MCU_ADDRESS [CAM1_STAT_BRIGHTNESS_METRIC_PREDIVIDER]
{0x0990, 0x000A},     // MCU_DATA_0
{0x098E, 0x686B},     // MCU_ADDRESS [PRI_A_CONFIG_LL_START_BRIGHTNESS]
{0x0990, 0x05DC},     // MCU_DATA_0
{0x098E, 0x686D},     // MCU_ADDRESS [PRI_A_CONFIG_LL_STOP_BRIGHTNESS]
{0x0990, 0x0BB8},     // MCU_DATA_0
{0x098E, 0x6C6B},     // MCU_ADDRESS [PRI_B_CONFIG_LL_START_BRIGHTNESS]
{0x0990, 0x05DC},     // MCU_DATA_0
{0x098E, 0x6C6D},     // MCU_ADDRESS [PRI_B_CONFIG_LL_STOP_BRIGHTNESS]
{0x0990, 0x0BB8},     // MCU_DATA_0
{0x098E, 0x3439},     // MCU_ADDRESS [AS_ASSTART_BRIGHTNESS]
{0x0990, 0x05DC},     // MCU_DATA_0
{0x098E, 0x343B},     // MCU_ADDRESS [AS_ASSTOP_BRIGHTNESS]
{0x0990, 0x0BB8},     // MCU_DATA_0
{0x098E, 0x4926},     // MCU_ADDRESS [CAM1_LL_START_GAMMA_BM]
{0x0990, 0x0001},     // MCU_DATA_0
{0x098E, 0x4928},     // MCU_ADDRESS [CAM1_LL_MID_GAMMA_BM]
{0x0990, 0x0002},     // MCU_DATA_0
{0x098E, 0x492A},     // MCU_ADDRESS [CAM1_LL_STOP_GAMMA_BM]
{0x0990, 0x0656},     // MCU_DATA_0
{0x098E, 0x4D26},     // MCU_ADDRESS [CAM2_LL_START_GAMMA_BM]
{0x0990, 0x0001},     // MCU_DATA_0
{0x098E, 0x4D28},     // MCU_ADDRESS [CAM2_LL_MID_GAMMA_BM]
{0x0990, 0x0002},     // MCU_DATA_0
{0x098E, 0x4D2A},     // MCU_ADDRESS [CAM2_LL_STOP_GAMMA_BM]
{0x0990, 0x0656},     // MCU_DATA_0
{0x33F4, 0x040B},     // KERNEL_CONFIG
{0x098E, 0xC916},     // MCU_ADDRESS [CAM1_LL_LL_START_0]
{0x0990, 0x0014},     // MCU_DATA_0
{0x098E, 0xC919},     // MCU_ADDRESS [CAM1_LL_LL_STOP_0]
{0x0990, 0x0028},     // MCU_DATA_0
{0x098E, 0xC917},     // MCU_ADDRESS [CAM1_LL_LL_START_1]
{0x0990, 0x0004},     // MCU_DATA_0
{0x098E, 0xC918},     // MCU_ADDRESS [CAM1_LL_LL_START_2]
{0x0990, 0x0000},     // MCU_DATA_0
{0x098E, 0xC91A},     // MCU_ADDRESS [CAM1_LL_LL_STOP_1]
{0x0990, 0x0001},     // MCU_DATA_0
{0x098E, 0xC91B},     // MCU_ADDRESS [CAM1_LL_LL_STOP_2]
{0x0990, 0x0009},     // MCU_DATA_0
{0x326C, 0x0C00},     // APERTURE_PARAMETERS_2D
{0x098E, 0x494B},     // MCU_ADDRESS [CAM1_LL_EXT_START_GAIN_METRIC]
{0x0990, 0x0042},     // MCU_DATA_0
{0x098E, 0x494D},     // MCU_ADDRESS [CAM1_LL_EXT_STOP_GAIN_METRIC]
{0x0990, 0x012C},     // MCU_DATA_0
{0x098E, 0xC91E},     // MCU_ADDRESS [CAM1_LL_NR_START_0]
{0x0990, 0x0012},     // MCU_DATA_0
{0x098E, 0xC91F},     // MCU_ADDRESS [CAM1_LL_NR_START_1]
{0x0990, 0x000A},     // MCU_DATA_0
{0x098E, 0xC920},     // MCU_ADDRESS [CAM1_LL_NR_START_2]
{0x0990, 0x0012},     // MCU_DATA_0
{0x098E, 0xC921},     // MCU_ADDRESS [CAM1_LL_NR_START_3]
{0x0990, 0x000A},     // MCU_DATA_0
{0x098E, 0xC922},     // MCU_ADDRESS [CAM1_LL_NR_STOP_0]
{0x0990, 0x0026},     // MCU_DATA_0
{0x098E, 0xC923},     // MCU_ADDRESS [CAM1_LL_NR_STOP_1]
{0x0990, 0x001E},     // MCU_DATA_0
{0x098E, 0xC924},     // MCU_ADDRESS [CAM1_LL_NR_STOP_2]
{0x0990, 0x0026},     // MCU_DATA_0
{0x098E, 0xC925},     // MCU_ADDRESS [CAM1_LL_NR_STOP_3]
{0x0990, 0x0026},     // MCU_DATA_0
{0x098E, 0xBC02},     // MCU_ADDRESS [LL_MODE]
{0x0990, 0x0003},     // MCU_DATA_0
{0x098E, 0xBC05},     // MCU_ADDRESS [LL_CLUSTER_DC_TH]
{0x0990, 0x000E},     // MCU_DATA_0
{0x316C, 0x350F},     // DAC_TXLO
{0x098E, 0xC950},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_START_1]
{0x0990, 0x0064},     // MCU_DATA_0
{0x098E, 0xC94F},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_START_0]
{0x0990, 0x0038},     // MCU_DATA_0
{0x098E, 0xC952},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_START_3]
{0x0990, 0x0064},     // MCU_DATA_0
{0x098E, 0xC951},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_START_2]
{0x0990, 0x0051},     // MCU_DATA_0
{0x098E, 0xC954},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_START_5]
{0x0990, 0x0010},     // MCU_DATA_0
{0x098E, 0xC953},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_START_4]
{0x0990, 0x0020},     // MCU_DATA_0
{0x098E, 0xC956},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_START_7]
{0x0990, 0x0010},     // MCU_DATA_0
{0x098E, 0xC955},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_START_6]
{0x0990, 0x0020},     // MCU_DATA_0
{0x098E, 0xC958},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_STOP_1]
{0x0990, 0x0020},     // MCU_DATA_0
{0x098E, 0xC957},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_STOP_0]
{0x0990, 0x0014},     // MCU_DATA_0
{0x098E, 0xC95A},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_STOP_3]
{0x0990, 0x001D},     // MCU_DATA_0
{0x098E, 0xC959},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_STOP_2]
{0x0990, 0x0020},     // MCU_DATA_0
{0x098E, 0xC95C},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_STOP_5]
{0x0990, 0x000C},     // MCU_DATA_0
{0x098E, 0xC95B},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_STOP_4]
{0x0990, 0x0008},     // MCU_DATA_0
{0x098E, 0xC95E},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_STOP_7]
{0x0990, 0x000C},     // MCU_DATA_0
{0x098E, 0xC95D},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_THRESHOLDS_STOP_6]
{0x0990, 0x0008},     // MCU_DATA_0
{0x098E, 0xC95F},     // MCU_ADDRESS [CAM1_LL_EXT_GRB_WINDOW_PERCENT]
{0x0990, 0x0064},     // MCU_DATA_0
{0x098E, 0x48DC},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_0]
{0x0990, 0x004D},     // MCU_DATA_0
{0x098E, 0x48DE},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_1]
{0x0990, 0x0096},     // MCU_DATA_0
{0x098E, 0x48E0},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_2]
{0x0990, 0x001D},     // MCU_DATA_0
{0x098E, 0x48E2},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_3]
{0x0990, 0x004D},     // MCU_DATA_0
{0x098E, 0x48E4},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_4]
{0x0990, 0x0096},     // MCU_DATA_0
{0x098E, 0x48E6},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_5]
{0x0990, 0x001D},     // MCU_DATA_0
{0x098E, 0x48E8},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_6]
{0x0990, 0x004D},     // MCU_DATA_0
{0x098E, 0x48EA},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_7]
{0x0990, 0x0096},     // MCU_DATA_0
{0x098E, 0x48EC},     // MCU_ADDRESS [CAM1_AWB_LL_CCM_8]
{0x0990, 0x001D},     // MCU_DATA_0
{0x098E, 0xDC2A},     // MCU_ADDRESS [SYS_DELTA_GAIN]
{0x0990, 0x000B},     // MCU_DATA_0
{0x098E, 0xDC2B},     // MCU_ADDRESS [SYS_DELTA_THRESH]
{0x0990, 0x0017},     // MCU_DATA_0

//gamm 0.55
{0x098E, 0xBC0B},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_0]
{0x0990, 0x0000},	// MCU_DATA_0
{0x098E, 0xBC0C},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_1]
{0x0990, 0x0006},	// MCU_DATA_0
{0x098E, 0xBC0D},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_2]
{0x0990, 0x0013},	// MCU_DATA_0
{0x098E, 0xBC0E},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_3]
{0x0990, 0x0027},	// MCU_DATA_0
{0x098E, 0xBC0F},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_4]
{0x0990, 0x0044},	// MCU_DATA_0
{0x098E, 0xBC10},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_5]
{0x0990, 0x005B},	// MCU_DATA_0
{0x098E, 0xBC11},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_6]
{0x0990, 0x0070},	// MCU_DATA_0
{0x098E, 0xBC12},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_7]
{0x0990, 0x0084},	// MCU_DATA_0
{0x098E, 0xBC13},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_8]
{0x0990, 0x0096},	// MCU_DATA_0
{0x098E, 0xBC14},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_9]
{0x0990, 0x00A5},	// MCU_DATA_0
{0x098E, 0xBC15},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_10]
{0x0990, 0x00B3},	// MCU_DATA_0
{0x098E, 0xBC16},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_11]
{0x0990, 0x00BF},	// MCU_DATA_0
{0x098E, 0xBC17},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_12]
{0x0990, 0x00CB},	// MCU_DATA_0
{0x098E, 0xBC18},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_13]
{0x0990, 0x00D5},	// MCU_DATA_0
{0x098E, 0xBC19},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_14]
{0x0990, 0x00DF},	// MCU_DATA_0
{0x098E, 0xBC1A},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_15]
{0x0990, 0x00E8},	// MCU_DATA_0
{0x098E, 0xBC1B},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_16]
{0x0990, 0x00F0},	// MCU_DATA_0
{0x098E, 0xBC1C},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_17]
{0x0990, 0x00F8},	// MCU_DATA_0
{0x098E, 0xBC1D},	// MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_18]
{0x0990, 0x00FF},	// MCU_DATA_0

	{0x098E, 0xBC1E},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_0]
	{0x0990, 0x0000},	// MCU_DATA_0
	{0x098E, 0xBC1F},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_1]
	{0x0990, 0x0006},	// MCU_DATA_0
	{0x098E, 0xBC20},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_2]
	{0x0990, 0x0013},	// MCU_DATA_0
	{0x098E, 0xBC21},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_3]
	{0x0990, 0x0027},	// MCU_DATA_0
	{0x098E, 0xBC22},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_4]
	{0x0990, 0x0044},	// MCU_DATA_0
	{0x098E, 0xBC23},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_5]
	{0x0990, 0x005B},	// MCU_DATA_0
	{0x098E, 0xBC24},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_6]
	{0x0990, 0x0070},	// MCU_DATA_0
	{0x098E, 0xBC25},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_7]
	{0x0990, 0x0084},	// MCU_DATA_0
	{0x098E, 0xBC26},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_8]
	{0x0990, 0x0096},	// MCU_DATA_0
	{0x098E, 0xBC27},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_9]
	{0x0990, 0x00A5},	// MCU_DATA_0
	{0x098E, 0xBC28},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_10]
	{0x0990, 0x00B3},	// MCU_DATA_0
	{0x098E, 0xBC29},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_11]
	{0x0990, 0x00BF},	// MCU_DATA_0
	{0x098E, 0xBC2A},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_12]
	{0x0990, 0x00CB},	// MCU_DATA_0
	{0x098E, 0xBC2B},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_13]
	{0x0990, 0x00D5},	// MCU_DATA_0
	{0x098E, 0xBC2C},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_14]
	{0x0990, 0x00DF},	// MCU_DATA_0
	{0x098E, 0xBC2D},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_15]
	{0x0990, 0x00E8},	// MCU_DATA_0
	{0x098E, 0xBC2E},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_16]
	{0x0990, 0x00F0},	// MCU_DATA_0
	{0x098E, 0xBC2F},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_17]
	{0x0990, 0x00F8},	// MCU_DATA_0
	{0x098E, 0xBC30},	// MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_18]
	{0x0990, 0x00FF},	// MCU_DATA_0
	
	{0x098E, 0xBC31},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_0]
	{0x0990, 0x0000},	// MCU_DATA_0
	{0x098E, 0xBC32},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_1]
	{0x0990, 0x0006},	// MCU_DATA_0
	{0x098E, 0xBC33},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_2]
	{0x0990, 0x0013},	// MCU_DATA_0
	{0x098E, 0xBC34},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_3]
	{0x0990, 0x0027},	// MCU_DATA_0
	{0x098E, 0xBC35},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_4]
	{0x0990, 0x0044},	// MCU_DATA_0
	{0x098E, 0xBC36},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_5]
	{0x0990, 0x005B},	// MCU_DATA_0
	{0x098E, 0xBC37},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_6]
	{0x0990, 0x0070},	// MCU_DATA_0
	{0x098E, 0xBC38},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_7]
	{0x0990, 0x0084},	// MCU_DATA_0
	{0x098E, 0xBC39},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_8]
	{0x0990, 0x0096},	// MCU_DATA_0
	{0x098E, 0xBC3A},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_9]
	{0x0990, 0x00A5},	// MCU_DATA_0
	{0x098E, 0xBC3B},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_10]
	{0x0990, 0x00B3},	// MCU_DATA_0
	{0x098E, 0xBC3C},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_11]
	{0x0990, 0x00BF},	// MCU_DATA_0
	{0x098E, 0xBC3D},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_12]
	{0x0990, 0x00CB},	// MCU_DATA_0
	{0x098E, 0xBC3E},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_13]
	{0x0990, 0x00D5},	// MCU_DATA_0
	{0x098E, 0xBC3F},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_14]
	{0x0990, 0x00DF},	// MCU_DATA_0
	{0x098E, 0xBC40},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_15]
	{0x0990, 0x00E8},	// MCU_DATA_0
	{0x098E, 0xBC41},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_16]
	{0x0990, 0x00F0},	// MCU_DATA_0
	{0x098E, 0xBC42},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_17]
	{0x0990, 0x00F8},	// MCU_DATA_0
	{0x098E, 0xBC43},	// MCU_ADDRESS [LL_GAMMA_NRCURVE_18]
	{0x0990, 0x00FF},	// MCU_DATA_0
//gamm end

{0x098E, 0x6865},     // MCU_ADDRESS [PRI_A_CONFIG_LL_ALGO_ENTER]
{0x0990, 0x00E0},     // MCU_DATA_0
{0x098E, 0x6867},     // MCU_ADDRESS [PRI_A_CONFIG_LL_ALGO_RUN]
{0x0990, 0x00F4},     // MCU_DATA_0
{0x098E, 0x8400},     // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006},     // MCU_DATA_0
{0x098E, 0xBC4A},     // MCU_ADDRESS [LL_TONAL_CURVE_HIGH]
{0x0990, 0x007F},     // MCU_DATA_0
{0x098E, 0xBC4B},     // MCU_ADDRESS [LL_TONAL_CURVE_MED]
{0x0990, 0x007F},     // MCU_DATA_0
{0x098E, 0xBC4C},     // MCU_ADDRESS [LL_TONAL_CURVE_LOW]
{0x0990, 0x007F},     // MCU_DATA_0
{0x3542, 0x0010},     // TONAL_X0
{0x3544, 0x0030},     // TONAL_X1
{0x3546, 0x0040},     // TONAL_X2
{0x3548, 0x0080},     // TONAL_X3
{0x354A, 0x0100},     // TONAL_X4
{0x354C, 0x0200},     // TONAL_X5
{0x354E, 0x0300},     // TONAL_X6
{0x3550, 0x0010},     // TONAL_Y0
{0x3552, 0x0030},     // TONAL_Y1
{0x3554, 0x0040},     // TONAL_Y2
{0x3556, 0x0080},     // TONAL_Y3
{0x3558, 0x012C},     // TONAL_Y4
{0x355A, 0x0320},     // TONAL_Y5
{0x355C, 0x03E8},     // TONAL_Y6
{0x3560, 0x0040},     // RECIPROCAL_OF_X0_MINUS_ZERO
{0x3562, 0x0020},     // RECIPROCAL_OF_X1_MINUS_X0
{0x3564, 0x0040},     // RECIPROCAL_OF_X2_MINUS_X1
{0x3566, 0x0010},     // RECIPROCAL_OF_X3_MINUS_X2
{0x3568, 0x0008},     // RECIPROCAL_OF_X4_MINUS_X3
{0x356A, 0x0004},     // RECIPROCAL_OF_X5_MINUS_X4
{0x356C, 0x0004},     // RECIPROCAL_OF_X6_MINUS_X5
{0x356E, 0x0004},     // RECIPROCAL_OF_400_MINUS_X6
{0x098E, 0x3C4D},     // MCU_ADDRESS [LL_START_GAMMA_FTB]
{0x0990, 0x0DAC},     // MCU_DATA_0
{0x098E, 0x3C4F},     // MCU_ADDRESS [LL_STOP_GAMMA_FTB]
{0x0990, 0x148A},     // MCU_DATA_0
{0x098E, 0xC911},     // MCU_ADDRESS [CAM1_STAT_LUMA_THRESH_HIGH]
{0x0990, 0x00C8},     // MCU_DATA_0
{0x098E, 0xC8F4},     // MCU_ADDRESS [CAM1_AWB_AWB_XSCALE]
{0x0990, 0x0004},     // MCU_DATA_0
{0x098E, 0xC8F5},     // MCU_ADDRESS [CAM1_AWB_AWB_YSCALE]
{0x0990, 0x0002},     // MCU_DATA_0
{0x098E, 0x48F6},     // MCU_ADDRESS [CAM1_AWB_AWB_WEIGHTS_0]
{0x0990, 0x3B4D},     // MCU_DATA_0
{0x098E, 0x48F8},     // MCU_ADDRESS [CAM1_AWB_AWB_WEIGHTS_1]
{0x0990, 0x6380},     // MCU_DATA_0
{0x098E, 0x48FA},     // MCU_ADDRESS [CAM1_AWB_AWB_WEIGHTS_2]
{0x0990, 0x9B18},     // MCU_DATA_0
{0x098E, 0x48FC},     // MCU_ADDRESS [CAM1_AWB_AWB_WEIGHTS_3]
{0x0990, 0x5D51},     // MCU_DATA_0
{0x098E, 0x48FE},     // MCU_ADDRESS [CAM1_AWB_AWB_WEIGHTS_4]
{0x0990, 0xEDE8},     // MCU_DATA_0
{0x098E, 0x4900},     // MCU_ADDRESS [CAM1_AWB_AWB_WEIGHTS_5]
{0x0990, 0xE515},     // MCU_DATA_0
{0x098E, 0x4902},     // MCU_ADDRESS [CAM1_AWB_AWB_WEIGHTS_6]
{0x0990, 0xBFF4},     // MCU_DATA_0
{0x098E, 0x4904},     // MCU_ADDRESS [CAM1_AWB_AWB_WEIGHTS_7]
{0x0990, 0x001E},     // MCU_DATA_0
{0x098E, 0x4906},     // MCU_ADDRESS [CAM1_AWB_AWB_XSHIFT_PRE_ADJ]
{0x0990, 0x0026},     // MCU_DATA_0
{0x098E, 0x4908},     // MCU_ADDRESS [CAM1_AWB_AWB_YSHIFT_PRE_ADJ]
{0x0990, 0x0033},     // MCU_DATA_0
{0x098E, 0xE84A},     // MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_L]
{0x0990, 0x0073},	//0x0083// MCU_DATA_0
{0x098E, 0xE84D},     // MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_R]
{0x0990, 0x0080},	//0x0083 // MCU_DATA_0
{0x098E, 0xE84C},     // MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_L]
{0x0990, 0x009D},	//0x0080 // MCU_DATA_0
{0x098E, 0xE84F},     // MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_R]
{0x0990, 0x0084},	//0x0080// MCU_DATA_0
	
{0x098E, 0x8400},     // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006},     // MCU_DATA_0
{0x098E, 0x48B0},     // MCU_ADDRESS [CAM1_AWB_CCM_L_0]
{0x0990, 0x0180},     // MCU_DATA_0
{0x098E, 0x48B2},     // MCU_ADDRESS [CAM1_AWB_CCM_L_1]
{0x0990, 0xFF7A},     // MCU_DATA_0
{0x098E, 0x48B4},     // MCU_ADDRESS [CAM1_AWB_CCM_L_2]
{0x0990, 0x0018},     // MCU_DATA_0
{0x098E, 0x48B6},     // MCU_ADDRESS [CAM1_AWB_CCM_L_3]
{0x0990, 0xFFCA},     // MCU_DATA_0
{0x098E, 0x48B8},     // MCU_ADDRESS [CAM1_AWB_CCM_L_4]
{0x0990, 0x017C},     // MCU_DATA_0
{0x098E, 0x48BA},     // MCU_ADDRESS [CAM1_AWB_CCM_L_5]
{0x0990, 0xFFCC},     // MCU_DATA_0
{0x098E, 0x48BC},     // MCU_ADDRESS [CAM1_AWB_CCM_L_6]
{0x0990, 0x000C},     // MCU_DATA_0
{0x098E, 0x48BE},     // MCU_ADDRESS [CAM1_AWB_CCM_L_7]
{0x0990, 0xFF1F},     // MCU_DATA_0
{0x098E, 0x48C0},     // MCU_ADDRESS [CAM1_AWB_CCM_L_8]
{0x0990, 0x01E8},     // MCU_DATA_0
{0x098E, 0x48C2},     // MCU_ADDRESS [CAM1_AWB_CCM_L_9]
{0x0990, 0x0020},     // MCU_DATA_0
{0x098E, 0x48C4},     // MCU_ADDRESS [CAM1_AWB_CCM_L_10]
{0x0990, 0x0044},     // MCU_DATA_0
{0x098E, 0x48C6},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_0]
{0x0990, 0x0079},     // MCU_DATA_0
{0x098E, 0x48C8},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_1]
{0x0990, 0xFFAD},     // MCU_DATA_0
{0x098E, 0x48CA},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_2]
{0x0990, 0xFFE2},     // MCU_DATA_0
{0x098E, 0x48CC},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_3]
{0x0990, 0x0033},     // MCU_DATA_0
{0x098E, 0x48CE},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_4]
{0x0990, 0x002A},     // MCU_DATA_0
{0x098E, 0x48D0},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_5]
{0x0990, 0xFFAA},     // MCU_DATA_0
{0x098E, 0x48D2},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_6]
{0x0990, 0x0017},     // MCU_DATA_0
{0x098E, 0x48D4},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_7]
{0x0990, 0x004B},     // MCU_DATA_0
{0x098E, 0x48D6},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_8]
{0x0990, 0xFFA5},     // MCU_DATA_0
{0x098E, 0x48D8},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_9]
{0x0990, 0x0015},     // MCU_DATA_0
{0x098E, 0x48DA},     // MCU_ADDRESS [CAM1_AWB_CCM_RL_10]
{0x0990, 0xFFE2},     // MCU_DATA_0
{0x35A2, 0x0014},     // DARK_COLOR_KILL_CONTROLS
{0x098E, 0xC949},     // MCU_ADDRESS [CAM1_SYS_DARK_COLOR_KILL]
{0x0990, 0x0024},     // MCU_DATA_0
{0x35A4, 0x0596},     // BRIGHT_COLOR_KILL_CONTROLS
{0x098E, 0xC94A},     // MCU_ADDRESS [CAM1_SYS_BRIGHT_COLORKILL]
{0x0990, 0x0062},     // MCU_DATA_0
{0x098E, 0xC948},     // MCU_ADDRESS [CAM1_SYS_UV_COLOR_BOOST]
{0x0990, 0x0006},     // MCU_DATA_0
{0x098E, 0xC914},     // MCU_ADDRESS [CAM1_LL_START_DESATURATION]
{0x0990, 0x0000},     // MCU_DATA_0
{0x098E, 0xC915},     // MCU_ADDRESS [CAM1_LL_END_DESATURATION]
{0x0990, 0x00FF},     // MCU_DATA_0
{0x098E, 0xE86F},     // MCU_ADDRESS [PRI_A_CONFIG_LL_START_SATURATION]
{0x0990, 0x0020},   
{0x098E, 0xE870},     // MCU_ADDRESS [PRI_A_CONFIG_LL_END_SATURATION]
{0x0990, 0x0010},	//0x003C// MCU_DATA_0
	
{0x098E, 0xEC6F},     // MCU_ADDRESS [PRI_B_CONFIG_LL_START_SATURATION]
{0x0990, 0x0020},  
{0x098E, 0xEC70},     // MCU_ADDRESS [PRI_B_CONFIG_LL_END_SATURATION]
{0x0990, 0x0010},   
{0x098E, 0xE883},     // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
{0x0990, 0x0000},     // MCU_DATA_0
{0x098E, 0xEC83},     // MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
{0x0990, 0x0000},     // MCU_DATA_0
{0x098E, 0x8400},     // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006},     // MCU_DATA_0
{0x098E, 0xE885},     // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CR]
{0x0990, 0x001E},     // MCU_DATA_0
{0x098E, 0xE886},     // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CB]
{0x0990, 0x00D8},     // MCU_DATA_0
{0x098E, 0xEC85},     // MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CR]
{0x0990, 0x001E},     // MCU_DATA_0
{0x098E, 0xEC86},     // MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CB]
{0x0990, 0x00D8},     // MCU_DATA_0
{0x098E, 0xE884},     // MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SOLARIZATION_TH]
{0x0990, 0x005C},     // MCU_DATA_0
{0x098E, 0xEC84},     // MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SOLARIZATION_TH]
{0x0990, 0x005C},     // MCU_DATA_0
{0x098E, 0x490A},     // MCU_ADDRESS [CAM1_AS_INTEG_SCALE_FIRST_PASS]
{0x0990, 0x0666},     // MCU_DATA_0
{0x098E, 0x490C},     // MCU_ADDRESS [CAM1_AS_MIN_INT_TIME_FIRST_PASS]
{0x0990, 0x0140},     // MCU_DATA_0
{0x098E, 0x6857},     // MCU_ADDRESS [PRI_A_CONFIG_IS_FEATURE_THRESHOLD]
{0x0990, 0x0014},     // MCU_DATA_0
{0x098E, 0x685C},     // MCU_ADDRESS [PRI_A_CONFIG_IS_BLUR_INPUT_PARAMETER]
{0x0990, 0x0005},     // MCU_DATA_0
{0x098E, 0x490E},     // MCU_ADDRESS [CAM1_AS_MAX_DIGITAL_GAIN_ALLOWED]
{0x0990, 0x00A4},     // MCU_DATA_0
{0x098E, 0xB43D},     // MCU_ADDRESS [AS_START_ASVALUES_0]
{0x0990, 0x0031},     // MCU_DATA_0
{0x098E, 0xB43E},     // MCU_ADDRESS [AS_START_ASVALUES_1]
{0x0990, 0x001B},     // MCU_DATA_0
{0x098E, 0xB43F},     // MCU_ADDRESS [AS_START_ASVALUES_2]
{0x0990, 0x0028},     // MCU_DATA_0
{0x098E, 0xB440},     // MCU_ADDRESS [AS_START_ASVALUES_3]
{0x0990, 0x0003},     // MCU_DATA_0
{0x098E, 0xB441},     // MCU_ADDRESS [AS_STOP_ASVALUES_0]
{0x0990, 0x00CD},     // MCU_DATA_0
{0x098E, 0xB442},     // MCU_ADDRESS [AS_STOP_ASVALUES_1]
{0x0990, 0x0064},     // MCU_DATA_0
{0x098E, 0xB443},     // MCU_ADDRESS [AS_STOP_ASVALUES_2]
{0x0990, 0x000F},     // MCU_DATA_0
{0x098E, 0xB444},     // MCU_ADDRESS [AS_STOP_ASVALUES_3]
{0x0990, 0x0007},     // MCU_DATA_0
{0x098E, 0x300D},     // MCU_ADDRESS [AF_FILTERS]
{0x0990, 0x000F},     // MCU_DATA_0
{0x098E, 0x3017},     // MCU_ADDRESS [AF_THRESHOLDS]
{0x0990, 0x0F0F},     // MCU_DATA_0
{0x098E, 0x8400},     // MCU_ADDRESS [SEQ_CMD]
{0x0990, 0x0006},     // MCU_DATA_0
{0x098E, 0xE81F},     // MCU_ADDRESS [PRI_A_CONFIG_AE_RULE_BASE_TARGET]
{0x0990, 0x0020},     // MCU_DATA_0

{0x098E, 0x68A0},     // MCU_ADDRESS [PRI_A_CONFIG_JPEG_OB_TX_CONTROL_VAR]
{0x0990, 0x082E},     // MCU_DATA_0
{0x098E, 0x6CA0},     // MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR]
{0x0990, 0x082E},     // MCU_DATA_0
{0x098E, 0x70A0},     // MCU_ADDRESS [SEC_A_CONFIG_JPEG_OB_TX_CONTROL_VAR]
{0x0990, 0x082E},     // MCU_DATA_0
{0x098E, 0x74A0},     // MCU_ADDRESS [SEC_B_CONFIG_JPEG_OB_TX_CONTROL_VAR]
{0x0990, 0x082E},     // MCU_DATA_0
{0x3C52, 0x082E},     // OB_TX_CONTROL
{0x098E, 0x488E},     // MCU_ADDRESS [CAM1_CTX_B_RX_FIFO_TRIGGER_MARK]
{0x0990, 0x0020},     // MCU_DATA_0
{0x098E, 0xECAC},     // MCU_ADDRESS [PRI_B_CONFIG_IO_OB_MANUAL_FLAG]
{0x0990, 0x0000},     // MCU_DATA_0

//---///[Optimal power consumption]
{0x3084, 0x2409},     // DAC_LD_4_5
{0x3092, 0x0A49},     // DAC_LD_18_19
{0x3094, 0x4949},     // DAC_LD_20_21
{0x3096, 0x4950},     // DAC_LD_22_23

//---///[REV3_patch]
{0x0982, 0x0000},     // ACCESS_CTL_STAT
{0x098A, 0x0CFB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x3C3C},
{0x0992, 0x3C3C},
{0x0994, 0x3C3C},
{0x0996, 0x5F4F},
{0x0998, 0x30ED},
{0x099A, 0x0AED},
{0x099C, 0x08BD},
{0x099E, 0x61D5},
{0x098A, 0x0D0B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xCE04},
{0x0992, 0xCD1F},
{0x0994, 0x1702},
{0x0996, 0x11CC},
{0x0998, 0x332E},
{0x099A, 0x30ED},
{0x099C, 0x02CC},
{0x099E, 0xFFFD},
{0x098A, 0x0D1B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xED00},
{0x0992, 0xCC00},
{0x0994, 0x02BD},
{0x0996, 0x706D},
{0x0998, 0x18DE},
{0x099A, 0x1F18},
{0x099C, 0x1F8E},
{0x099E, 0x0110},
{0x098A, 0x0D2B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xCC3C},
{0x0992, 0x5230},
{0x0994, 0xED00},
{0x0996, 0x18EC},
{0x0998, 0xA0C4},
{0x099A, 0xFDBD},
{0x099C, 0x7021},
{0x099E, 0x201E},
{0x098A, 0x0D3B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xCC3C},
{0x0992, 0x5230},
{0x0994, 0xED00},
{0x0996, 0xDE1F},
{0x0998, 0xECA0},
{0x099A, 0xBD70},
{0x099C, 0x21CC},
{0x099E, 0x3C52},
{0x098A, 0x0D4B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x30ED},
{0x0992, 0x02CC},
{0x0994, 0xFFFC},
{0x0996, 0xED00},
{0x0998, 0xCC00},
{0x099A, 0x02BD},
{0x099C, 0x706D},
{0x099E, 0xFC04},
{0x098A, 0x0D5B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xE11A},
{0x0992, 0x8300},
{0x0994, 0x0127},
{0x0996, 0x201A},
{0x0998, 0x8300},
{0x099A, 0x0427},
{0x099C, 0x221A},
{0x099E, 0x8300},
{0x098A, 0x0D6B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0827},
{0x0992, 0x241A},
{0x0994, 0x8300},
{0x0996, 0x1027},
{0x0998, 0x261A},
{0x099A, 0x8300},
{0x099C, 0x2027},
{0x099E, 0x281A},
{0x098A, 0x0D7B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x8300},
{0x0992, 0x4027},
{0x0994, 0x2A20},
{0x0996, 0x2ECC},
{0x0998, 0x001E},
{0x099A, 0x30ED},
{0x099C, 0x0A20},
{0x099E, 0x26CC},
{0x098A, 0x0D8B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0022},
{0x0992, 0x30ED},
{0x0994, 0x0A20},
{0x0996, 0x1ECC},
{0x0998, 0x0021},
{0x099A, 0x30ED},
{0x099C, 0x0A20},
{0x099E, 0x16CC},
{0x098A, 0x0D9B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0020},
{0x0992, 0x30ED},
{0x0994, 0x0A20},
{0x0996, 0x0ECC},
{0x0998, 0x002A},
{0x099A, 0x30ED},
{0x099C, 0x0A20},
{0x099E, 0x06CC},
{0x098A, 0x0DAB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x002B},
{0x0992, 0x30ED},
{0x0994, 0x0ACC},
{0x0996, 0x3400},
{0x0998, 0x30ED},
{0x099A, 0x0034},
{0x099C, 0xBD6F},
{0x099E, 0xD184},
{0x098A, 0x0DBB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0330},
{0x0992, 0xED07},
{0x0994, 0xA60C},
{0x0996, 0x4848},
{0x0998, 0x5FED},
{0x099A, 0x05EC},
{0x099C, 0x07EA},
{0x099E, 0x06AA},
{0x098A, 0x0DCB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0531},
{0x0992, 0xBD70},
{0x0994, 0x21DE},
{0x0996, 0x1F1F},
{0x0998, 0x8E01},
{0x099A, 0x08EC},
{0x099C, 0x9B05},
{0x099E, 0x30ED},
{0x098A, 0x0DDB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0820},
{0x0992, 0x3BDE},
{0x0994, 0x1FEC},
{0x0996, 0x0783},
{0x0998, 0x0040},
{0x099A, 0x2628},
{0x099C, 0x7F30},
{0x099E, 0xC4CC},
{0x098A, 0x0DEB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x3C68},
{0x0992, 0xBD6F},
{0x0994, 0xD1FD},
{0x0996, 0x30C5},
{0x0998, 0xCC01},
{0x099A, 0xF4FD},
{0x099C, 0x30C7},
{0x099E, 0xC640},
{0x098A, 0x0DFB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xF730},
{0x0992, 0xC4CC},
{0x0994, 0x0190},
{0x0996, 0xFD30},
{0x0998, 0xC501},
{0x099A, 0x0101},
{0x099C, 0xFC30},
{0x099E, 0xC230},
{0x098A, 0x0E0B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xED08},
{0x0992, 0x200A},
{0x0994, 0xCC3C},
{0x0996, 0x68BD},
{0x0998, 0x6FD1},
{0x099A, 0x0530},
{0x099C, 0xED08},
{0x099E, 0xCC34},
{0x098A, 0x0E1B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x08ED},
{0x0992, 0x00EC},
{0x0994, 0x08BD},
{0x0996, 0x7021},
{0x0998, 0x30C6},
{0x099A, 0x0C3A},
{0x099C, 0x3539},
{0x099E, 0x373C},
{0x098A, 0x0E2B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x3C3C},
{0x0992, 0x34DE},
{0x0994, 0x2FEE},
{0x0996, 0x0EAD},
{0x0998, 0x007D},
{0x099A, 0x13EF},
{0x099C, 0x277C},
{0x099E, 0xCE13},
{0x098A, 0x0E3B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xE01E},
{0x0992, 0x0510},
{0x0994, 0x60E6},
{0x0996, 0x0E4F},
{0x0998, 0xC313},
{0x099A, 0xF08F},
{0x099C, 0xE600},
{0x099E, 0x30E1},
{0x098A, 0x0E4B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0722},
{0x0992, 0x16F6},
{0x0994, 0x13EE},
{0x0996, 0x4FC3},
{0x0998, 0x13F3},
{0x099A, 0x8FE6},
{0x099C, 0x0030},
{0x099E, 0xE107},
{0x098A, 0x0E5B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x2507},
{0x0992, 0xF613},
{0x0994, 0xEEC1},
{0x0996, 0x0325},
{0x0998, 0x3C7F},
{0x099A, 0x13EE},
{0x099C, 0xF613},
{0x099E, 0xEFE7},
{0x098A, 0x0E6B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x06CC},
{0x0992, 0x13F0},
{0x0994, 0xED04},
{0x0996, 0xCC13},
{0x0998, 0xF320},
{0x099A, 0x0F7C},
{0x099C, 0x13EE},
{0x099E, 0xEC04},
{0x098A, 0x0E7B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xC300},
{0x0992, 0x01ED},
{0x0994, 0x04EC},
{0x0996, 0x02C3},
{0x0998, 0x0001},
{0x099A, 0xED02},
{0x099C, 0xF613},
{0x099E, 0xEEE1},
{0x098A, 0x0E8B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0624},
{0x0992, 0x12EE},
{0x0994, 0x04E6},
{0x0996, 0x0030},
{0x0998, 0xE107},
{0x099A, 0x22DF},
{0x099C, 0xEE02},
{0x099E, 0xE600},
{0x098A, 0x0E9B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x30E1},
{0x0992, 0x0725},
{0x0994, 0xD6DE},
{0x0996, 0x49EE},
{0x0998, 0x08AD},
{0x099A, 0x00CC},
{0x099C, 0x13F6},
{0x099E, 0x30ED},
{0x098A, 0x0EAB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x00DE},
{0x0992, 0x2FEE},
{0x0994, 0x10CC},
{0x0996, 0x13FA},
{0x0998, 0xAD00},
{0x099A, 0x3838},
{0x099C, 0x3838},
{0x099E, 0x3937},
{0x098A, 0x0EBB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x363C},
{0x0992, 0x3C3C},
{0x0994, 0x5F4F},
{0x0996, 0x30ED},
{0x0998, 0x04EC},
{0x099A, 0x06ED},
{0x099C, 0x008F},
{0x099E, 0xC300},
{0x098A, 0x0ECB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x04BD},
{0x0992, 0x0F43},
{0x0994, 0x30EC},
{0x0996, 0x04BD},
{0x0998, 0x0F76},
{0x099A, 0x30ED},
{0x099C, 0x0238},
{0x099E, 0x3838},
{0x098A, 0x0EDB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x3839},
{0x0992, 0x373C},
{0x0994, 0x3C3C},
{0x0996, 0x3C30},
{0x0998, 0xE608},
{0x099A, 0x2712},
{0x099C, 0xC101},
{0x099E, 0x2713},
{0x098A, 0x0EEB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xC102},
{0x0992, 0x2714},
{0x0994, 0xC103},
{0x0996, 0x2715},
{0x0998, 0xC104},
{0x099A, 0x2716},
{0x099C, 0x2019},
{0x099E, 0xCC30},
{0x098A, 0x0EFB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x5E20},
{0x0992, 0x12CC},
{0x0994, 0x305A},
{0x0996, 0x200D},
{0x0998, 0xCC30},
{0x099A, 0x5620},
{0x099C, 0x08CC},
{0x099E, 0x305C},
{0x098A, 0x0F0B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x2003},
{0x0992, 0xCC30},
{0x0994, 0x58ED},
{0x0996, 0x065F},
{0x0998, 0x4FED},
{0x099A, 0x04EC},
{0x099C, 0x0BED},
{0x099E, 0x008F},
{0x098A, 0x0F1B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xC300},
{0x0992, 0x04BD},
{0x0994, 0x0F43},
{0x0996, 0x30EC},
{0x0998, 0x048A},
{0x099A, 0x02ED},
{0x099C, 0x02EC},
{0x099E, 0x06ED},
{0x098A, 0x0F2B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x008F},
{0x0992, 0xC300},
{0x0994, 0x02DE},
{0x0996, 0x0EAD},
{0x0998, 0x0030},
{0x099A, 0xEC04},
{0x099C, 0xBD0F},
{0x099E, 0x7630},
{0x098A, 0x0F3B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xED02},
{0x0992, 0x3838},
{0x0994, 0x3838},
{0x0996, 0x3139},
{0x0998, 0x3736},
{0x099A, 0x30EC},
{0x099C, 0x041A},
{0x099E, 0x8300},
{0x098A, 0x0F4B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x4025},
{0x0992, 0x22EC},
{0x0994, 0x041A},
{0x0996, 0x8300},
{0x0998, 0x8024},
{0x099A, 0x0504},
{0x099C, 0xCA40},
{0x099E, 0x2015},
{0x098A, 0x0F5B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xEC04},
{0x0992, 0x1A83},
{0x0994, 0x0100},
{0x0996, 0x2406},
{0x0998, 0x0404},
{0x099A, 0xCA80},
{0x099C, 0x2007},
{0x099E, 0xEC04},
{0x098A, 0x0F6B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0404},
{0x0992, 0x04CA},
{0x0994, 0xC0EE},
{0x0996, 0x00ED},
{0x0998, 0x0038},
{0x099A, 0x3937},
{0x099C, 0x363C},
{0x099E, 0x301F},
{0x098A, 0x0F7B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0340},
{0x0992, 0x0E1F},
{0x0994, 0x0380},
{0x0996, 0x0AEC},
{0x0998, 0x02C4},
{0x099A, 0x3F4F},
{0x099C, 0x0505},
{0x099E, 0x0520},
{0x098A, 0x0F8B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x1B1F},
{0x0992, 0x0380},
{0x0994, 0x09EC},
{0x0996, 0x02C4},
{0x0998, 0x3F4F},
{0x099A, 0x0505},
{0x099C, 0x200E},
{0x099E, 0x1F03},
{0x098A, 0x0F9B},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x4008},
{0x0992, 0xEC02},
{0x0994, 0xC43F},
{0x0996, 0x4F05},
{0x0998, 0x2002},
{0x099A, 0xEC02},
{0x099C, 0xED00},
{0x099E, 0x3838},
{0x098A, 0x8FAB},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0039},     // MCU_DATA_0
{0x098A, 0x1000},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xCC10},
{0x0992, 0x09BD},
{0x0994, 0x4224},
{0x0996, 0x7E10},
{0x0998, 0x09C6},
{0x099A, 0x01F7},
{0x099C, 0x018A},
{0x099E, 0xC609},
{0x098A, 0x1010},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xF701},
{0x0992, 0x8BDE},
{0x0994, 0x3F18},
{0x0996, 0xCE0B},
{0x0998, 0xF3CC},
{0x099A, 0x0011},
{0x099C, 0xBDD7},
{0x099E, 0x00CC},
{0x098A, 0x1020},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0BF3},
{0x0992, 0xDD3F},
{0x0994, 0xDE35},
{0x0996, 0x18CE},
{0x0998, 0x0C05},
{0x099A, 0xCC00},
{0x099C, 0x3FBD},
{0x099E, 0xD700},
{0x098A, 0x1030},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xCC0C},
{0x0992, 0x05DD},
{0x0994, 0x35DE},
{0x0996, 0x4718},
{0x0998, 0xCE0C},
{0x099A, 0x45CC},
{0x099C, 0x0015},
{0x099E, 0xBDD7},
{0x098A, 0x1040},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x00CC},
{0x0992, 0x0C45},
{0x0994, 0xDD47},
{0x0996, 0xFE00},
{0x0998, 0x3318},
{0x099A, 0xCE0C},
{0x099C, 0x5BCC},
{0x099E, 0x0009},
{0x098A, 0x1050},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xBDD7},
{0x0992, 0x00CC},
{0x0994, 0x0C5B},
{0x0996, 0xFD00},
{0x0998, 0x33DE},
{0x099A, 0x3118},
{0x099C, 0xCE0C},
{0x099E, 0x65CC},
{0x098A, 0x1060},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0029},
{0x0992, 0xBDD7},
{0x0994, 0x00CC},
{0x0996, 0x0C65},
{0x0998, 0xDD31},
{0x099A, 0xDE39},
{0x099C, 0x18CE},
{0x099E, 0x0C8F},
{0x098A, 0x1070},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xCC00},
{0x0992, 0x23BD},
{0x0994, 0xD700},
{0x0996, 0xCC0C},
{0x0998, 0x8FDD},
{0x099A, 0x39DE},
{0x099C, 0x4918},
{0x099E, 0xCE0C},
{0x098A, 0x1080},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xB3CC},
{0x0992, 0x000D},
{0x0994, 0xBDD7},
{0x0996, 0x00CC},
{0x0998, 0x0CB3},
{0x099A, 0xDD49},
{0x099C, 0xFC04},
{0x099E, 0xC2FD},
{0x098A, 0x1090},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0BF1},
{0x0992, 0x18FE},
{0x0994, 0x0BF1},
{0x0996, 0xCDEE},
{0x0998, 0x1518},
{0x099A, 0xCE0C},
{0x099C, 0xC1CC},
{0x099E, 0x0029},
{0x098A, 0x10A0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xBDD7},
{0x0992, 0x00FE},
{0x0994, 0x0BF1},
{0x0996, 0xCC0C},
{0x0998, 0xC1ED},
{0x099A, 0x15CC},
{0x099C, 0x11A5},
{0x099E, 0xFD0B},
{0x098A, 0x10B0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xFFCC},
{0x0992, 0x0CFB},
{0x0994, 0xFD0C},
{0x0996, 0x21CC},
{0x0998, 0x128F},
{0x099A, 0xFD0C},
{0x099C, 0x53CC},
{0x099E, 0x114E},
{0x098A, 0x10C0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xFD0C},
{0x0992, 0x5DCC},
{0x0994, 0x10E2},
{0x0996, 0xFD0C},
{0x0998, 0x6FCC},
{0x099A, 0x0EDD},
{0x099C, 0xFD0C},
{0x099E, 0xD7CC},
{0x098A, 0x10D0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0EBA},
{0x0992, 0xFD0C},
{0x0994, 0xE9CC},
{0x0996, 0x1350},
{0x0998, 0xFD0C},
{0x099A, 0x9BCC},
{0x099C, 0x0E29},
{0x099E, 0xFD0C},
{0x098A, 0x10E0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xBF39},
{0x0992, 0x373C},
{0x0994, 0x3CDE},
{0x0996, 0x1DEC},
{0x0998, 0x0C5F},
{0x099A, 0x8402},
{0x099C, 0x4416},
{0x099E, 0x4FF7},
{0x098A, 0x10F0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0CEB},
{0x0992, 0xE60B},
{0x0994, 0xC407},
{0x0996, 0xF70C},
{0x0998, 0xEC7F},
{0x099A, 0x30C4},
{0x099C, 0xEC25},
{0x099E, 0xFD30},
{0x098A, 0x1100},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xC5FC},
{0x0992, 0x06D6},
{0x0994, 0xFD30},
{0x0996, 0xC701},
{0x0998, 0xFC30},
{0x099A, 0xC0FD},
{0x099C, 0x0BED},
{0x099E, 0xFC30},
{0x098A, 0x1110},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xC2FD},
{0x0992, 0x0BEF},
{0x0994, 0xFC04},
{0x0996, 0xC283},
{0x0998, 0xFFFF},
{0x099A, 0x2728},
{0x099C, 0xDE06},
{0x099E, 0xEC22},
{0x098A, 0x1120},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x8322},
{0x0992, 0x0026},
{0x0994, 0x1FCC},
{0x0996, 0x3064},
{0x0998, 0x30ED},
{0x099A, 0x008F},
{0x099C, 0xC300},
{0x099E, 0x02DE},
{0x098A, 0x1130},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0CAD},
{0x0992, 0x0030},
{0x0994, 0x1D02},
{0x0996, 0x01CC},
{0x0998, 0x3064},
{0x099A, 0xED00},
{0x099C, 0x8FC3},
{0x099E, 0x0002},
{0x098A, 0x1140},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xDE0E},
{0x0992, 0xAD00},
{0x0994, 0x30E6},
{0x0996, 0x04BD},
{0x0998, 0x5203},
{0x099A, 0x3838},
{0x099C, 0x3139},
{0x099E, 0x3C3C},
{0x098A, 0x1150},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x3C21},
{0x0992, 0x01CC},
{0x0994, 0x0018},
{0x0996, 0xBD6F},
{0x0998, 0xD1C5},
{0x099A, 0x0426},
{0x099C, 0xF5DC},
{0x099E, 0x2530},
{0x098A, 0x1160},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xED04},
{0x0992, 0x2012},
{0x0994, 0xEE04},
{0x0996, 0x3C18},
{0x0998, 0x38E6},
{0x099A, 0x2118},
{0x099C, 0xE7BE},
{0x099E, 0x30EE},
{0x098A, 0x1170},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x04EC},
{0x0992, 0x1D30},
{0x0994, 0xED04},
{0x0996, 0xEC04},
{0x0998, 0x26EA},
{0x099A, 0xCC00},
{0x099C, 0x1AED},
{0x099E, 0x02CC},
{0x098A, 0x1180},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xFBFF},
{0x0992, 0xED00},
{0x0994, 0xCC04},
{0x0996, 0x00BD},
{0x0998, 0x706D},
{0x099A, 0xCC00},
{0x099C, 0x1A30},
{0x099E, 0xED02},
{0x098A, 0x1190},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xCCFB},
{0x0992, 0xFFED},
{0x0994, 0x005F},
{0x0996, 0x4FBD},
{0x0998, 0x706D},
{0x099A, 0x5FBD},
{0x099C, 0x5B17},
{0x099E, 0xBD55},
{0x098A, 0x11A0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x8B38},
{0x0992, 0x3838},
{0x0994, 0x393C},
{0x0996, 0x3CC6},
{0x0998, 0x40F7},
{0x099A, 0x30C4},
{0x099C, 0xFC0B},
{0x099E, 0xEDFD},
{0x098A, 0x11B0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x30C0},
{0x0992, 0xFC0B},
{0x0994, 0xEFFD},
{0x0996, 0x30C2},
{0x0998, 0xDE1D},
{0x099A, 0xEC25},
{0x099C, 0xFD30},
{0x099E, 0xC501},
{0x098A, 0x11C0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0101},
{0x0992, 0xFC30},
{0x0994, 0xC2FD},
{0x0996, 0x06D6},
{0x0998, 0xEC0C},
{0x099A, 0x5F84},
{0x099C, 0x0244},
{0x099E, 0x164F},
{0x098A, 0x11D0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x30E7},
{0x0992, 0x03F1},
{0x0994, 0x0CEB},
{0x0996, 0x2715},
{0x0998, 0xF10C},
{0x099A, 0xEB23},
{0x099C, 0x09FC},
{0x099E, 0x06D6},
{0x098A, 0x11E0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x04FD},
{0x0992, 0x06D6},
{0x0994, 0x2007},
{0x0996, 0xFC06},
{0x0998, 0xD605},
{0x099A, 0xFD06},
{0x099C, 0xD6DE},
{0x099E, 0x1DE6},
{0x098A, 0x11F0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0BC4},
{0x0992, 0x0730},
{0x0994, 0xE702},
{0x0996, 0xF10C},
{0x0998, 0xEC27},
{0x099A, 0x2C7D},
{0x099C, 0x0CEC},
{0x099E, 0x2727},
{0x098A, 0x1200},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x5D27},
{0x0992, 0x247F},
{0x0994, 0x30C4},
{0x0996, 0xFC06},
{0x0998, 0xD6FD},
{0x099A, 0x30C5},
{0x099C, 0xF60C},
{0x099E, 0xEC4F},
{0x098A, 0x1210},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xFD30},
{0x0992, 0xC7C6},
{0x0994, 0x40F7},
{0x0996, 0x30C4},
{0x0998, 0xE602},
{0x099A, 0x4FFD},
{0x099C, 0x30C5},
{0x099E, 0x0101},
{0x098A, 0x1220},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x01FC},
{0x0992, 0x30C2},
{0x0994, 0xFD06},
{0x0996, 0xD67D},
{0x0998, 0x06CB},
{0x099A, 0x272E},
{0x099C, 0xC640},
{0x099E, 0xF730},
{0x098A, 0x1230},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xC4FC},
{0x0992, 0x06C1},
{0x0994, 0x04F3},
{0x0996, 0x06D6},
{0x0998, 0xED00},
{0x099A, 0x5F6D},
{0x099C, 0x002A},
{0x099E, 0x0153},
{0x098A, 0x1240},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x17FD},
{0x0992, 0x30C0},
{0x0994, 0xEC00},
{0x0996, 0xFD30},
{0x0998, 0xC2FC},
{0x099A, 0x06C1},
{0x099C, 0xFD30},
{0x099E, 0xC501},
{0x098A, 0x1250},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x0101},
{0x0992, 0xFC30},
{0x0994, 0xC2FD},
{0x0996, 0x06C7},
{0x0998, 0x2022},
{0x099A, 0x7F30},
{0x099C, 0xC4DE},
{0x099E, 0x1DEC},
{0x098A, 0x1260},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x25FD},
{0x0992, 0x30C5},
{0x0994, 0xFC06},
{0x0996, 0xD6FD},
{0x0998, 0x30C7},
{0x099A, 0x01FC},
{0x099C, 0x30C0},
{0x099E, 0xFD06},
{0x098A, 0x1270},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xD0FC},
{0x0992, 0x30C2},
{0x0994, 0xFD06},
{0x0996, 0xD2EC},
{0x0998, 0x25FD},
{0x099A, 0x06C3},
{0x099C, 0xBD95},
{0x099E, 0x3CDE},
{0x098A, 0x1280},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x3FEE},
{0x0992, 0x10AD},
{0x0994, 0x00DE},
{0x0996, 0x1DFC},
{0x0998, 0x06CC},
{0x099A, 0xED3E},
{0x099C, 0x3838},
{0x099E, 0x3930},
{0x098A, 0x1290},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x8FC3},
{0x0992, 0xFFEC},
{0x0994, 0x8F35},
{0x0996, 0xBDAD},
{0x0998, 0x15DE},
{0x099A, 0x198F},
{0x099C, 0xC301},
{0x099E, 0x4B8F},
{0x098A, 0x12A0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xEC00},
{0x0992, 0xFD05},
{0x0994, 0x0EEC},
{0x0996, 0x02FD},
{0x0998, 0x0510},
{0x099A, 0x8FC3},
{0x099C, 0xFFCB},
{0x099E, 0x8FE6},
{0x098A, 0x12B0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x00F7},
{0x0992, 0x0514},
{0x0994, 0xE603},
{0x0996, 0xF705},
{0x0998, 0x15FC},
{0x099A, 0x055B},
{0x099C, 0xFD05},
{0x099E, 0x12DE},
{0x098A, 0x12C0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x37EE},
{0x0992, 0x08AD},
{0x0994, 0x00F6},
{0x0996, 0x0516},
{0x0998, 0x4F30},
{0x099A, 0xED04},
{0x099C, 0xDE1F},
{0x099E, 0xEC6B},
{0x098A, 0x12D0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xFD05},
{0x0992, 0x0EEC},
{0x0994, 0x6DFD},
{0x0996, 0x0510},
{0x0998, 0xDE19},
{0x099A, 0x8FC3},
{0x099C, 0x0117},
{0x099E, 0x8FE6},
{0x098A, 0x12E0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x00F7},
{0x0992, 0x0514},
{0x0994, 0xE603},
{0x0996, 0xF705},
{0x0998, 0x15FC},
{0x099A, 0x0559},
{0x099C, 0xFD05},
{0x099E, 0x12DE},
{0x098A, 0x12F0},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x37EE},
{0x0992, 0x08AD},
{0x0994, 0x00F6},
{0x0996, 0x0516},
{0x0998, 0x4F30},
{0x099A, 0xED06},
{0x099C, 0xDE1F},
{0x099E, 0xEC6B},
{0x098A, 0x1300},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xFD05},
{0x0992, 0x0EEC},
{0x0994, 0x6DFD},
{0x0996, 0x0510},
{0x0998, 0xDE19},
{0x099A, 0x8FC3},
{0x099C, 0x0118},
{0x099E, 0x8FE6},
{0x098A, 0x1310},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x00F7},
{0x0992, 0x0514},
{0x0994, 0xE603},
{0x0996, 0xF705},
{0x0998, 0x15FC},
{0x099A, 0x0559},
{0x099C, 0xFD05},
{0x099E, 0x12DE},
{0x098A, 0x1320},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x37EE},
{0x0992, 0x08AD},
{0x0994, 0x00F6},
{0x0996, 0x0516},
{0x0998, 0x4F30},
{0x099A, 0xED08},
{0x099C, 0xCC32},
{0x099E, 0x8EED},
{0x098A, 0x1330},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x00EC},
{0x0992, 0x04BD},
{0x0994, 0x7021},
{0x0996, 0xCC32},
{0x0998, 0x6C30},
{0x099A, 0xED02},
{0x099C, 0xCCF8},
{0x099E, 0x00ED},
{0x098A, 0x1340},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x00A6},
{0x0992, 0x07E3},
{0x0994, 0x0884},
{0x0996, 0x07BD},
{0x0998, 0x706D},
{0x099A, 0x30C6},
{0x099C, 0x143A},
{0x099E, 0x3539},
{0x098A, 0x1350},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x3CBD},
{0x0992, 0x776D},
{0x0994, 0xCC32},
{0x0996, 0x5C30},
{0x0998, 0xED00},
{0x099A, 0xFC13},
{0x099C, 0x8683},
{0x099E, 0x0001},
{0x098A, 0x1360},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0xBD70},
{0x0992, 0x21CC},
{0x0994, 0x325E},
{0x0996, 0x30ED},
{0x0998, 0x00FC},
{0x099A, 0x1388},
{0x099C, 0x8300},
{0x099E, 0x01BD},
{0x098A, 0x1370},     // PHYSICAL_ADDR_ACCESS
{0x0990, 0x7021},
{0x0992, 0x3839},
{0x098E, 0x0010},     // MCU_ADDRESS [MON_ADDR]
{0x0990, 0x1000},     // MCU_DATA_0
{0x098E, 0x0003},     // MCU_ADDRESS [MON_ALGO]
{0x0990, 0x0004},     // MCU_DATA_0



//  POLL  MON_PATCH_0 =>  0x01
{0x098E, 0x4815},     // MCU_ADDRESS
{0x0990, 0x0004},     // MCU_DATA_0
{0x098E, 0x485D},     // MCU_ADDRESS
{0x0990, 0x0004},     // MCU_DATA_0


{0x0018, 0x0028},     // STANDBY_CONTROL_AND_STATUS
	
	{0x098E, 0x6833},	//MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MIN_VIRT_DGAIN]
	{0x0990, 0x0040},	//MCU_DATA_0
	{0x098E, 0x6835},	//MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MAX_VIRT_DGAIN]
	{0x0990, 0x00e0},	//MCU_DATA_0
	
	{0x098E, 0x6837},	//MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MIN_VIRT_AGAIN]
	{0x0990, 0x0020},	//MCU_DATA_0
	{0x098E, 0x6839},	//MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MAX_VIRT_AGAIN]
	{0x0990, 0x012C},	//MCU_DATA_0
	
	{0x098E, 0x682F},	 //MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_TARGET_AGAIN]
	{0x0990, 0x0040},	 //MCU_DATA_0
	
	
//  POLL  SEQ_STATE =>  0x03

{0x098E, 0x6C9B},     // MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_WIDTH_VAR]
{0x0990, 0x0400},     // MCU_DATA_0
{0x098E, 0x6C9D},     // MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_HEIGHT_VAR]
{0x0990, 0x07D0},     // MCU_DATA_0

//-------------------JPEG capture设置
//[JPEG422 capure]
//    {0x098E, 0x6C07},    //MCU_ADDRESS [PRI_B_OUTPUT_FORMAT]
//    {0x0990, 0x0001},    //MCU_DATA_0
//    {0x098E, 0x6C09},    //MCU_ADDRESS [PRI_B_OUTPUT_FORMAT_ORDER]
// /   {0x0990, 0x0000},    //MCU_DATA_0
//    {0x098E, 0xEC8E},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_JP_MODE]
//    {0x0990, 0x0001},    //MCU_DATA_0
//    {0x098E, 0xEC8F},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_FORMAT]
  //  {0x0990, 0x0002},    //MCU_DATA_0
    //{0x098E, 0x6CA0},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR]
//    {0x0990, 0x083D},    //0x082D},    //MCU_DATA_0
//    {0x098E, 0xEC9F},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_CONTROL_VAR]
//    {0x0990, 0x0001},    //MCU_DATA_0
//[test mode] 图片设置代码
//    {0x3070, 0x0001},
 

//DELAY=10 //
//{0x098E, 0x8400},      // MCU_ADDRESS [SEQ_CMD]S
//{0x0990, 0x0006},      // MCU_DATA_0           
//DELAY=350 //
    {0xffff,0x00ff},
};
static struct regval_list  mt9t111_fmt_jpeg_setting[]  = {
//[JPEG422 capure]
	{0x098E, 0x6C07},    //MCU_ADDRESS [PRI_B_OUTPUT_FORMAT]
	{0x0990, 0x0001},    //MCU_DATA_0
	{0x098E, 0x6C09},    //MCU_ADDRESS [PRI_B_OUTPUT_FORMAT_ORDER]
	{0x0990, 0x0000},    //MCU_DATA_0
	{0x098E, 0xEC8E},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_JP_MODE]
	{0x0990, 0x0001},    //MCU_DATA_0
	{0x098E, 0xEC8F},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_FORMAT]
	{0x0990, 0x0002},    //MCU_DATA_0
	{0x098E, 0x6CA0},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR]
	{0x0990, 0x083D},    //0x082D},    //MCU_DATA_0
	{0x098E, 0xEC9F},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_CONTROL_VAR]
	{0x0990, 0x0001},    //MCU_DATA_0
	{0xffff,0x00ff},
};

//176*144
static struct regval_list  mt9t111_fmt_yuv422_qcif[]  = {
{0x098E,  0x6800},      //Output Width (A) 
{0x0990,  176},         //      = 176     
{0x098E,  0x6802},      //Output Height (A)
{0x0990,  144},        //      = 144     

{0x098E, 0x4846},      //RX FIFO Watermark (A)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x68AA},      //TX FIFO Watermark (A)
{0x0990, 0x023E},      //      = 574
{0x098E, 0x6815},      //Max FD Zone 50 Hz
{0x0990, 0x0007},      //      = 4
{0x098E, 0x6817},      //Max FD Zone 60 Hz
{0x0990, 0x0009},      //      = 5
{0x098E, 0x682D},      //AE Target FD Zone
{0x0990, 0x0007},      //      = 4
{0x098E, 0x488E},      //RX FIFO Watermark (B)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x6CAA},      //TX FIFO Watermark (B)
//{0x0990, 0x008A},      //      = 138
{0x0990, 0x0400},      //      = 138		__changed_jpeg__
{0x098E, 0x8400},      //Refresh Sequencer Mode
{0x0990, 0x06},      //      = 6
{0x098E, 0x8400},      //Refresh Sequencer
{0x0990, 0x05},      //      = 5

{0xffff,0x00ff},
};

static struct regval_list  mt9t111_fmt_yuv422_qcif_rotate[]  = {
	{0x098E,  0x6800},      //Output Width (A) 
	{0x0990,  144},         //      = 234     
	{0x098E,  0x6802},      //Output Height (A)
	{0x0990,  176},        //      = 176     
 	//{0x098E  ,  0x4802},    //Row Start (A)            
  	//{0x0990  ,  0x0000},    //     = 0                
   	//{0x098E  ,  0x4804},    //Column Start (A)         
   	//{0x0990  ,  454},    //     = 454                
   	//{0x098E  ,  0x4806},    //Row End (A)              
   	//{0x0990  ,  1549},    //     = 1549             
    //	{0x098E  ,  0x4808},    //Column End (A)           
    	//{0x0990  ,  1606},    //     = 1606        

#if 0
{ 0x98E, 0x481D},	//Base Frame Lines (A)
{0x990, 0x0786},	//      = 1926

{0x98E, 0x6815},	//Max FD Zone 50 Hz
{0x990, 0x0008},	//      = 8
{0x98E, 0x6817},	//Max FD Zone 60 Hz
{0x990, 0x000A},	//      = 10
{ 0x98E, 0x682D},	//AE Target FD Zone
{0x990, 0x0008},	//      = 8

{0x098E, 0x8400},      //Refresh Sequencer Mode
{0x0990, 0x06},      //      = 6
{0x098E, 0x8400},      //Refresh Sequencer
{0x0990, 0x05},      //      = 5
#endif
#if 1
{0x98E, 0x4802},     //Row Start (A)
{0x990, 0x000},     //      = 0
{0x98E, 0x4804},     //Column Start (A)
{0x990, 0x000},     //      = 0
{0x98E, 0x4806},     //Row End (A)
{0x990, 0x60D},     //      = 1549
{0x98E, 0x4808},     //Column End (A)
{0x990, 0x80D},     //      = 2061
{0x98E, 0x480A},     //Row Speed (A)
{0x990, 0x0111},     //      = 273
{0x98E, 0x480C},     //Read Mode (A)
{0x990, 0x046C},     //      = 1132
{0x98E, 0x480F},     //Fine Correction (A)
{0x990, 0x00CC},     //      = 204
{0x98E, 0x4811},     //Fine IT Min (A)
{0x990, 0x0381},     //      = 897
{0x98E, 0x4813},     //Fine IT Max Margin (A)
{0x990, 0x024F},     //      = 591

{0x98E, 0x481D},     //Base Frame Lines (A)
{0x990, 0x05D5},     //      = 1493

{0x98E, 0x481F},     //Min Line Length (A)
{0x990, 0x05D0},     //      = 1488
{0x98E, 0x4825},     //Line Length (A)
{0x990, 0x07F2},     //      = 2034
{0x98E, 0x482B},     //Contex Width (A)
{0x990, 0x0408},     //      = 1032
{0x98E, 0x482D},     //Context Height (A)
{0x990, 0x0308},     //      = 776
{0x98E, 0x6C00},     //Output Width (B)
{0x990, 0x0800},     //      = 2048
{0x98E, 0x6C02},     //Output Height (B)
{0x990, 0x0600},     //      = 1536
{0x98E, 0xEC8E},     //JPEG (B)
{0x990, 0x00},     //      = 0
//{0x98E, 0x6CA0},     //Adaptive Output Clock (B)
//BITFIELD = 0x990, 0x0040, 0x0000},     //      = 0
{0x98E, 0x484A},     //Row Start (B)
{0x990, 0x004},     //      = 4
{0x98E, 0x484C},     //Column Start (B)
{0x990, 0x004},     //      = 4
{0x98E, 0x484E},     //Row End (B)
{0x990, 0x60B},     //      = 1547
{0x98E, 0x4850},     //Column End (B)
{0x990, 0x80B},     //      = 2059
{0x98E, 0x4852},     //Row Speed (B)
{0x990, 0x0111},     //      = 273
{0x98E, 0x4854},     //Read Mode (B)
{0x990, 0x0024},     //      = 36
{0x98E, 0x4857},     //Fine Correction (B)
{0x990, 0x008C},     //      = 140
{0x98E, 0x4859},     //Fine IT Min (B)
{0x990, 0x01F1},     //      = 497
{0x98E, 0x485B},     //Fine IT Max Margin (B)
{0x990, 0x00FF},     //      = 255
{0x98E, 0x4865},     //Base Frame Lines (B)
{0x990, 0x065D},     //      = 1629
{0x98E, 0x4867},     //Min Line Length (B)
{0x990, 0x0378},     //      = 888
{0x98E, 0x486D},     //Line Length (B)
{0x990, 0x0F69},     //      = 3945
{0x98E, 0x4873},     //Contex Width (B)
{0x990, 0x0808},     //      = 2056
{0x98E, 0x4875},     //Context Height (B)
{0x990, 0x0608},     //      = 1544
{0x98E, 0xC8A5},     //search_f1_50
{0x990, 0x26},     //      = 38
{0x98E, 0xC8A6},     //search_f2_50
{0x990, 0x28},     //      = 40
{0x98E, 0xC8A7},     //search_f1_60
{0x990, 0x2E},     //      = 46
{0x98E, 0xC8A8},     //search_f2_60
{0x990, 0x30},     //      = 48
{0x98E, 0xC844},     //period_50Hz (A)
{0x990, 0xEC},     //      = 236
{0x98E, 0xC92F},     //period_50Hz (A MSB)
{0x990, 0x00},     //      = 0
{0x98E, 0xC845},     //period_60Hz (A)
{0x990, 0xC5},     //      = 197
{0x98E, 0xC92D},     //period_60Hz (A MSB)
{0x990, 0x00},     //      = 0
{0x98E, 0xC88C},     //period_50Hz (B)
{0x990, 0x7A},     //      = 122
{0x98E, 0xC930},     //period_50Hz (B) MSB
{0x990, 0x00},     //      = 0
{0x98E, 0xC88D},     //period_60Hz (B)
{0x990, 0x65},     //      = 101
{0x98E, 0xC92E},     //period_60Hz (B) MSB
{0x990, 0x00},     //      = 0
{0x98E, 0xB825},     //FD Window Height
{0x990, 0x05},     //      = 5
{0x98E, 0xA009},     //Stat_min
{0x990, 0x02},     //      = 2
{0x98E, 0xA00A},     //Stat_max
{0x990, 0x03},     //      = 3
{0x98E, 0xA00C},     //Min_amplitude
{0x990, 0x0A},     //      = 10
{0x98E, 0x4846},     //RX FIFO Watermark (A)
{0x990, 0x0080},     //      = 128
{0x98E, 0x68AA},     //TX FIFO Watermark (A)
{0x990, 0x02B2},     //      = 690
{0x98E, 0x6815},     //Max FD Zone 50 Hz
{0x990, 0x0007},     //      = 6
{0x98E, 0x6817},     //Max FD Zone 60 Hz
{0x990, 0x0009},     //      = 8
{0x98E, 0x682D},     //AE Target FD Zone
{0x990, 0x0007},     //      = 6
{0x98E, 0x488E},     //RX FIFO Watermark (B)
{0x990, 0x0080},     //      = 128
{0x98E, 0x6CAA},     //TX FIFO Watermark (B)
{0x990, 0x008A},     //      = 138
#endif
//{0x98E, 0x5c0c},     //Refresh Sequencer
//{0x990, 0x0274},     //      = 5

{0x98E, 0x8400},     //Refresh Sequencer Mode
{0x990, 0x06},     //      = 6
{0x98E, 0x8400},     //Refresh Sequencer
{0x990, 0x05},     //      = 5

{0xffff,0x00ff},
};

//320*240
static struct regval_list  mt9t111_fmt_yuv422_qvga[]  = {
{0x098E,  0x6800},      //Output Width (A) 
{0x0990,  320},         //      = 320     
{0x098E,  0x6802},      //Output Height (A)
{0x0990,  240},        //      = 240     
{0x098E, 0x4846},      //RX FIFO Watermark (A)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x68AA},      //TX FIFO Watermark (A)
//{0x0990, 0x02E8},      //      = 744
{0x0990, 0x02C4},      //      = 744		__changed_jpeg__
{0x098E, 0x6815},      //Max FD Zone 50 Hz
{0x0990, 0x0007},      //      = 4
{0x098E, 0x6817},      //Max FD Zone 60 Hz
{0x0990, 0x0009},      //      = 5
{0x098E, 0x682D},      //AE Target FD Zone
{0x0990, 0x0007},      //      = 4
{0x098E, 0x488E},      //RX FIFO Watermark (B)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x6CAA},      //TX FIFO Watermark (B)
//{0x0990, 0x008A}, 	 // 	 = 138
{0x0990, 0x0400},	   //	   = 138		__changed_jpeg__
{0x098E, 0x8400},      //Refresh Sequencer Mode
{0x0990, 0x06},      //      = 6
{0x098E, 0x8400},      //Refresh Sequencer
{0x0990, 0x05},      //      = 5

{0xffff,0x00ff},
};

static struct regval_list  mt9t111_fmt_yuv422_cif[]  = {
{0x098E,  0x6800},      //Output Width (A) 
{0x0990,  352},         //      = 176     
{0x098E,  0x6802},      //Output Height (A)
{0x0990,  288},        //      = 144     
{0x098E, 0x4846},      //RX FIFO Watermark (A)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x68AA},      //TX FIFO Watermark (A)
{0x0990, 0x02DE},      //      = 734
{0x098E, 0x6815},      //Max FD Zone 50 Hz
{0x0990, 0x0007},      //      = 4
{0x098E, 0x6817},      //Max FD Zone 60 Hz
{0x0990, 0x0009},      //      = 5
{0x098E, 0x682D},      //AE Target FD Zone
{0x0990, 0x0007},      //      = 4
{0x098E, 0x488E},      //RX FIFO Watermark (B)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x6CAA},      //TX FIFO Watermark (B)
//{0x0990, 0x008A}, 	 // 	 = 138
{0x0990, 0x0400},	   //	   = 138		__changed_jpeg__
{0x098E, 0x8400},      //Refresh Sequencer Mode
{0x0990, 0x06},      //      = 6
{0x098E, 0x8400},      //Refresh Sequencer
{0x0990, 0x05},      //      = 5

{0xffff,0x00ff},
};

//640*480
static struct regval_list  mt9t111_fmt_yuv422_vga[]  = {
{0x098E,  0x6800},      //Output Width (A) 
{0x0990,  640},         //      = 176     
{0x098E,  0x6802},      //Output Height (A)
{0x0990,  480},        //      = 144     

{0x098E, 0x4846},      //RX FIFO Watermark (A)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x68AA},      //TX FIFO Watermark (A)
{0x0990, 0x022E},      //      = 558
{0x098E, 0x6815},      //Max FD Zone 50 Hz
{0x0990, 0x0007},      //      = 4
{0x098E, 0x6817},      //Max FD Zone 60 Hz
{0x0990, 0x0009},      //      = 5
{0x098E, 0x682D},      //AE Target FD Zone
{0x0990, 0x0007},      //      = 4
{0x098E, 0x488E},      //RX FIFO Watermark (B)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x6CAA},      //TX FIFO Watermark (B)
//{0x0990, 0x008A}, 	 // 	 = 138
{0x0990, 0x0400},	   //	   = 138		__changed_jpeg__
{0x098E, 0x8400},      //Refresh Sequencer Mode
{0x0990, 0x06},      //      = 6
{0x098E, 0x8400},      //Refresh Sequencer
{0x0990, 0x05},      //      = 5

{0xffff,0x00ff},
};

//720*480
static struct regval_list  mt9t111_fmt_yuv422_d1[]  = {
{0x098E,  0x6800},      //Output Width (A) 
{0x0990,  720},         //      = 720     
{0x098E,  0x6802},      //Output Height (A)
{0x0990,  480},        //      = 480     
{0x098E, 0x4846},      //RX FIFO Watermark (A)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x68AA},      //TX FIFO Watermark (A)
{0x0990, 0x023E},      //      = 574
{0x098E, 0x6815},      //Max FD Zone 50 Hz
{0x0990, 0x0007},      //      = 4
{0x098E, 0x6817},      //Max FD Zone 60 Hz
{0x0990, 0x0009},      //      = 5
{0x098E, 0x682D},      //AE Target FD Zone
{0x0990, 0x0007},      //      = 4
{0x098E, 0x488E},      //RX FIFO Watermark (B)
{0x0990, 0x0080},      //      = 128
{0x098E, 0x6CAA},      //TX FIFO Watermark (B)
//{0x0990, 0x008A}, 	 // 	 = 138
{0x0990, 0x0400},	   //	   = 138		__changed_jpeg__
{0x098E, 0x8400},      //Refresh Sequencer Mode
{0x0990, 0x06},      //      = 6
{0x098E, 0x8400},      //Refresh Sequencer
{0x0990, 0x05},      //      = 5

{0xffff,0x00ff},
};

//1024*768
static struct regval_list  mt9t111_fmt_yuv422_xga[]  = {
{0x098E,  0x6800},      //Output Width (A) 
{0x0990,  1024},         //      = 1024     
{0x098E,  0x6802},      //Output Height (A)
{0x0990,  768},        //      = 768     

{0xffff,0x00ff},
};
//1600*1200
static struct regval_list  mt9t111_fmt_yuv422_uxga[]  = {
{0x098E,  0x6800},      //Output Width (A) 
{0x0990,  1600},         //      = 1600     
{0x098E,  0x6802},      //Output Height (A)
{0x0990,  1200},        //      = 1200     

{0xffff,0x00ff},
};
//2048*1536
static struct regval_list  mt9t111_fmt_yuv422_qxga[]  = {
{0x098E,  0x6800},      //Output Width (A) 
{0x0990,  2048},         //      = 2048     
{0x098E,  0x6802},      //Output Height (A)
{0x0990,  1536},        //      = 1536     

{0xffff,0x00ff},
};

//320*240
static struct regval_list  mt9t111_fmt_jpeg_qvga[]  = {
{0x098E,  0x6C00},      //Output Width (B)         
{0x0990,  320},      //      = 320             
{0x098E,  0x6C02},      //Output Height (B)        
{0x0990,  240},      //      = 240             

{0xffff,0x00ff},
};

//640*480
static struct regval_list  mt9t111_fmt_jpeg_vga[]  = {
{0x098E,  0x6C00},      //Output Width (B)         
{0x0990,  640},      //      = 640             
{0x098E,  0x6C02},      //Output Height (B)        
{0x0990,  480},      //      = 480             

{0xffff,0x00ff},
};

//1024*768
static struct regval_list  mt9t111_fmt_jpeg_xga[]  = {
{0x098E,  0x6C00},      //Output Width (B)         
{0x0990,  1024},      //      = 1024             
{0x098E,  0x6C02},      //Output Height (B)        
{0x0990,  768},      //      = 768             

{0xffff,0x00ff},
};
//1600*1200
static struct regval_list  mt9t111_fmt_jpeg_uxga[]  = {
{0x098E,  0x6C00},      //Output Width (B)         
{0x0990,  1600},      //      = 1600             
{0x098E,  0x6C02},      //Output Height (B)        
{0x0990,  1200},      //      = 1200             

{0xffff,0x00ff},
};


//2048*1536.
static struct regval_list mt9t111_fmt_jpeg_qxga[]  = {
{0x098E,  0x6C00},      //Output Width (B)         
{0x0990,  0x0800},      //      = 2048             
{0x098E,  0x6C02},      //Output Height (B)        
{0x0990,  0x0600},      //      = 1536             

{0xffff,0x00ff},
};
/*
 * Low-level register I/O.
 */
/*issue that OV sensor must write then read. 3640 register is 16bit!!!*/

static int capture_mode = CAPTURE_MODE_PREVIEW;
static int init_nums = 0, is_init_reg = 1;    
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
	//printk("mt9t111: read reg [%04x]=%02x\n", addr, *pvalue);
	return 0;
}
static
inline int write_sensor_reg_16bit(struct i2c_client *c, u16 addr, u16 *value)
{
	u8 data[4];
	int ret;
	
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
	
	ret = i2c_master_send(c, data, 4);
	return ret;
}


static int mt9t111_read(struct i2c_client *c, u16 reg,u16 *value)
{
	int ret;

	ret = read_sensor_reg_16bit(c, reg, value);
	
	return ret;
}

static int mt9t111_write(struct i2c_client *c, u16 reg, u16 value)
{
	int ret;

	ret = write_sensor_reg_16bit(c, reg, &value);
	
	return ret;
}


/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int mt9t111_write_array(struct i2c_client *c,struct regval_list *vals)
{
	int i = 0;
	while (vals->reg_num != 0xffff ) {

	int ret = mt9t111_write(c, vals->reg_num, vals->value);
	//if (ret < 0)
		//return ret;
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
static void mt9t111_reset(struct i2c_client *client)
{
//printk("mt9t111_reset\n");

	//mt9t111_write(client, REG_SYS, SYS_RESET);
	//msleep(1);
}

static int mt9t111_detect(struct i2c_client *client)
{
	u16 v;
	int ret;
	printk("mt9t111_detect \n");
	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	ret = read_sensor_reg_16bit(client, 0x0000, &v);
	printk("mt9t111_detect:chip_id= %x\n",v);	
	if (ret < 0)
		return ret;
	if (v != 0x2680)
		return -ENODEV;
	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct mt9t111_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
	int bpp;   /* bits per pixel */
} mt9t111_formats[] = {
#if 1  
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= mt9t111_fmt_yuv,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },	//TODO
		.bpp		= 16,
	},
	{
		.desc		= "YUYV422 planar",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.regs 		= mt9t111_fmt_yuv,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 16,
	},
	#endif
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.regs           = mt9t111_fmt_yuv,
		.cmatrix        = { 128, -128, 0, -34, -94, 128 },
		.bpp            = 12,
	},
	{
		.desc           = "JFIF JPEG",
		.pixelformat    = V4L2_PIX_FMT_JPEG,
		.regs           = mt9t111_fmt_jpeg_qxga,
		.cmatrix        = { 128, -128, 0, -34, -94, 128 },
		.bpp            = 16,
	},
	
};
#define N_MT9T111_FMTS ARRAY_SIZE(mt9t111_formats)

/*TODO - also can use ccic size register 0x34 to do same thing for cropping...anyway, sensor doing it is better?
  0x3020~0x3027*/
static struct mt9t111_win_size {
	int	width;
	int	height;
} mt9t111_win_sizes[] = {
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
	},
#if 1  
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
	},
#endif
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
	},
	/* D1 */
	{
		.width          = D1_WIDTH,
		.height         = D1_HEIGHT,
	},
	/* QXGA */
	{
		.width          = UXGA_WIDTH,
		.height         = UXGA_HEIGHT,
	},
	/* QXGA */
	{
		.width          = QXGA_WIDTH,
		.height         = QXGA_HEIGHT,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(mt9t111_win_sizes))
/* capture jpeg size */
static struct mt9t111_win_size mt9t111_win_sizes_jpeg[] = {
	/* full */
	{
		.width = QXGA_WIDTH,
		.height = QXGA_HEIGHT,
	},
	{
		.width = UXGA_WIDTH,
		.height = UXGA_HEIGHT,
	},
	{
		.width = 1024,
		.height = 768,
	},
//	{
//		.width = 640,
//		.height = 480,
//	},
//	{
//		.width = 320,
//		.height = 240,
//	},
};



static int mt9t111_querycap(struct i2c_client *c, struct v4l2_capability *argp)
{
	if(!argp){
		printk(KERN_ERR" argp is NULL %s %d \n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	strcpy(argp->driver, "mt9t111");
	strcpy(argp->card, "TD/TTC");
	return 0;
}

static int mt9t111_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct mt9t111_format_struct *ofmt;
	//printk("mt9t111_enum_fmt \n");
	if (fmt->index >= N_MT9T111_FMTS)
		return -EINVAL;

	ofmt = mt9t111_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}
static int mt9t111_enum_fmsize(struct i2c_client *c, struct v4l2_frmsizeenum *argp)
{
	struct v4l2_frmsizeenum frmsize;

	if (copy_from_user(&frmsize, argp, sizeof(frmsize)))
		   return -EFAULT;

	if (frmsize.pixel_format == V4L2_PIX_FMT_YUV420||
		frmsize.pixel_format == V4L2_PIX_FMT_YUYV ||
		frmsize.pixel_format == V4L2_PIX_FMT_YUV422P){
		if (frmsize.index >= (ARRAY_SIZE(mt9t111_win_sizes)))
{
		    return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = mt9t111_win_sizes[frmsize.index].height;
		frmsize.discrete.width = mt9t111_win_sizes[frmsize.index].width;
	}else if(frmsize.pixel_format == V4L2_PIX_FMT_JPEG){
		if (frmsize.index >= ARRAY_SIZE(mt9t111_win_sizes_jpeg)){
			   return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = mt9t111_win_sizes_jpeg[frmsize.index].height;
		frmsize.discrete.width = mt9t111_win_sizes_jpeg[frmsize.index].width;

	}else
	   return -EINVAL;

	if (copy_to_user(argp, &frmsize, sizeof(frmsize)))
		   return -EFAULT;
	return 0;
}

static int mt9t111_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct mt9t111_format_struct **ret_fmt,
		struct mt9t111_win_size **ret_wsize)
{
	int index;
	struct mt9t111_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	//printk("mt9t111_try_fmt \n");

	
	for (index = 0; index < N_MT9T111_FMTS; index++)
		if (mt9t111_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_MT9T111_FMTS){
		printk("unsupported format!\n");
		return -EINVAL;
	}
	if (ret_fmt != NULL)
		*ret_fmt = mt9t111_formats + index;
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
	for (wsize = mt9t111_win_sizes; wsize < mt9t111_win_sizes + N_WIN_SIZES;
			wsize++)
		if (pix->width <= wsize->width && pix->height <= wsize->height)
			break;
	if (wsize >= mt9t111_win_sizes + N_WIN_SIZES){
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
	pix->bytesperline = pix->width*mt9t111_formats[index].bpp/8;
	pix->sizeimage = pix->height*pix->bytesperline;
	printk("mt9t111_try_fmt: pix->width is %d, pix->height is %d wsize %d\n", pix->width, pix->height, wsize->width);

	if (ret_fmt == NULL)
		return 0;
	switch (pix->pixelformat)
	{
		case V4L2_PIX_FMT_YUYV: 
		//case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV420: 
			printk("try_fmt:V4L2_PIX_FMT_YUV\n");
			switch (pix->width)
			{
				case QXGA_WIDTH:
					printk("try_fmt:mt9t111_fmt_yuv422_qxga\n");
					(*ret_fmt)->regs = mt9t111_fmt_yuv422_qxga;
					break;
					
				case D1_WIDTH:
					printk("try_fmt:mt9t111_fmt_yuv422_d1\n");
					(*ret_fmt)->regs = mt9t111_fmt_yuv422_d1;
					break;
					
				case VGA_WIDTH:
					printk("try_fmt:mt9t111_fmt_yuv422_vga\n");
					(*ret_fmt)->regs = mt9t111_fmt_yuv422_vga;
					break;
					
				case QVGA_WIDTH:
					printk("try_fmt:mt9t111_fmt_yuv422_qvga\n");
					(*ret_fmt)->regs = mt9t111_fmt_yuv422_qvga;
					break;
					
				case CIF_WIDTH: 
					printk("try_fmt:mt9t111_fmt_yuv422_cif\n");
					(*ret_fmt)->regs = mt9t111_fmt_yuv422_cif;
					break;
					
				case QCIF_WIDTH:
					printk("try_fmt:mt9t111_fmt_yuv422_qcif\n");
					(*ret_fmt)->regs = mt9t111_fmt_yuv422_qcif;
					break;
					
				case 144:
					printk("try_fmt:mt9t111_fmt_yuv422_qcif_rotate\n");
					(*ret_fmt)->regs = mt9t111_fmt_yuv422_qcif_rotate;
					break;
					
				default:
					printk("unsupported size!\n");
					return -EINVAL;
				
			}
			break;
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_JPEG:
			printk("try_fmt:V4L2_PIX_FMT_JPEG\n");
			switch (pix->width)
			{
				case QXGA_WIDTH:
					printk("try_fmt:mt9t111_fmt_jpeg_qxga\n");
					(*ret_fmt)->regs = mt9t111_fmt_jpeg_qxga;
					break;
					
				case UXGA_WIDTH:
					printk("try_fmt:mt9t111_fmt_jpeg_uxga\n");
					(*ret_fmt)->regs = mt9t111_fmt_jpeg_uxga;
					break;
					
				case XGA_WIDTH:
					printk("try_fmt:mt9t111_fmt_jpeg_xga\n");
					(*ret_fmt)->regs = mt9t111_fmt_jpeg_xga;
					break;
					
				case VGA_WIDTH:
					printk("try_fmt:mt9t111_fmt_jpeg_vga\n");
					(*ret_fmt)->regs = mt9t111_fmt_jpeg_vga;
					break;
					
				case QVGA_WIDTH:
					printk("try_fmt:mt9t111_fmt_jpeg_qvga\n");
					(*ret_fmt)->regs = mt9t111_fmt_jpeg_qvga;
					break;
				default:
					printk("unsupported size!\n");
					return -EINVAL;
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
static int mt9t111_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
//	u16 reg_val = 0;
   // u16 times = 0;
	struct mt9t111_format_struct *ovfmt;
	struct mt9t111_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	
	struct sensor_platform_data *pdata;
	pdata = c->dev.platform_data;
	if(1 == is_init_reg)     
	{
		//mt9t111_write(c, 0x001A, 0x001D);// RESET_REGISTER
		mt9t111_write(c, 0x001A, 0x0019);// RESET_REGISTER  _don't need enable MIPI
		msleep(1);
		mt9t111_write(c, 0x001A, 0x0018);// RESET_REGISTER
		msleep(10);
		mt9t111_write(c, 0x001A, 0x0018);// RESET_REGISTER
		mt9t111_write(c, 0x0014,   0x2425);      //PLL_CONTROL
		mt9t111_write(c, 0x0014,   0x2425);      //PLL_CONTROL
		mt9t111_write(c, 0x0014,   0x2145);      //PLL_CONTROL
	
		mt9t111_write(c, 0x0010, 0x0C36);      //PLL Dividers = 3126
		mt9t111_write(c, 0x0012, 0x0070);      //PLL P Dividers = 112
		mt9t111_write(c, 0x002A, 0x7788);    // 0x7788 //PLL P Dividers 4-5-6 = 30600
		mt9t111_write(c, 0x001A, 0x218);      //Reset Misc. Control = 536
		mt9t111_write(c, 0x0014, 0x2545);      //PLL control: TEST_BYPASS on = 9541
		mt9t111_write(c, 0x0014, 0x2547);      //PLL control: PLL_ENABLE on = 9543
		mt9t111_write(c, 0x0014, 0x2447);      //PLL control: SEL_LOCK_DET on = 9287
		mt9t111_write(c, 0x0014, 0x2047);      //PLL control: TEST_BYPASS off = 8263
		msleep(10);////DELAY=10 
		mt9t111_write(c, 0x0014, 0x2046);      //PLL control: PLL_BYPASS off = 8262
		mt9t111_write(c, 0x0022, 0x0410);      //Reference clock count for 20 us = 1040
		mt9t111_write(c, 0x001E, 0x0777);      //Pad Slew Rate = 1911
		mt9t111_write(c, 0x0016, 0x0400);      //JPEG Clock = 1024

		mt9t111_write(c, 0x0018,  0x402D);      // STANDBY_CONTROL_AND_STATUS             
		mt9t111_write(c, 0x0018,  0x402C);      // STANDBY_CONTROL_AND_STATUS             
		msleep(50);
		mt9t111_write_array(c, mt9t111_fmt_yuv);


		mt9t111_write(c, 0x098E, 0x8400);     // MCU_ADDRESS [SEQ_CMD]
		mt9t111_write(c, 0x0990, 0x0006);      // MCU_DATA_0
		msleep(150); 
		mt9t111_write(c, 0x098E, 0x8400);     // MCU_ADDRESS [SEQ_CMD]
		mt9t111_write(c, 0x0990, 0x0005);      // MCU_DATA_0
		msleep(100);
		is_init_reg = 0;
	}
	ret = mt9t111_try_fmt(c, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	mt9t111_write_array(c, ovfmt->regs);
	msleep(100); 
	mt9t111_write_array(c, ovfmt->regs);
	msleep(200); 
	
	 if(pix->width == 144)
	{
		msleep(400);
	 }
	if(pix->width == QCIF_WIDTH ||pix->width == CIF_WIDTH )
	{
		mt9t111_write(c, 0x098E, 0x5C0C);     // MCU_ADDRESS [SEQ_CMD]
		mt9t111_write(c, 0x0990, 0x03AB);      // MCU_DATA_0
	}
	
	printk("mt9t111_s_fmt mt9t111_write_array end\n");
	
	switch (pix->pixelformat)
	{
		case V4L2_PIX_FMT_YUV422P:
			capture_mode = CAPTURE_MODE_STILL;
			break;
		case V4L2_PIX_FMT_YUYV: 
		case V4L2_PIX_FMT_YUV420: 
			capture_mode = CAPTURE_MODE_PREVIEW;
			break;
		
		case V4L2_PIX_FMT_JPEG:
			capture_mode = CAPTURE_MODE_STILL;
			mt9t111_write_array(c, mt9t111_fmt_jpeg_setting);
			break;
		default:
			capture_mode = CAPTURE_MODE_PREVIEW;
			printk("mt9t111_s_fmt unsupported format!\n");
			break;
		
	}
	
			
	mt9t111_write(c, 0x098E, 0x8400);     // MCU_ADDRESS [SEQ_CMD]
	mt9t111_write(c, 0x0990, 0x0006);      //0x006// MCU_DATA_0
	msleep(100); 
	//mt9t111_write(c, 0x098E, 0x8400);     // MCU_ADDRESS [SEQ_CMD]
	//mt9t111_write(c, 0x0990, 0x0005);      // MCU_DATA_0
	//msleep(200);
	 if(pix->width == 144)
	{
		mt9t111_write(c, 0x098E, 0x5C0C);     // MCU_ADDRESS [SEQ_CMD]
		mt9t111_write(c, 0x0990, 0x0274);      // MCU_DATA_0
	}
	  if(pix->width == 720)
	{
		mt9t111_write(c, 0x098E, 0x5C0E);     // MCU_ADDRESS [SEQ_CMD]
		mt9t111_write(c, 0x0990, 0x02aa);      // MCU_DATA_0
	}
	printk("mt9t111_s_fmt sensor mode = %d\n", capture_mode);
	
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int mt9t111_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	u16 clkrc;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	ret = mt9t111_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = MT9T111_FRAME_RATE;
	//if ((clkrc & CLK_EXT) == 0 && (clkrc & CLK_SCALE) > 1)
	//	cp->timeperframe.denominator /= (clkrc & CLK_SCALE);
	return 0;
}

static int mt9t111_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int mt9t111_s_input(struct i2c_client *c, int *id)
{
	is_init_reg = 1;   
     printk("mt9t111_s_input \n");

	return 0;
}





static struct mt9t111_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} mt9t111_controls[] =
{

};
#define N_CONTROLS (ARRAY_SIZE(mt9t111_controls))

static struct mt9t111_control *mt9t111_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (mt9t111_controls[i].qc.id == id)
			return mt9t111_controls + i;
	return NULL;
}


static int mt9t111_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct mt9t111_control *ctrl = mt9t111_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int mt9t111_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct mt9t111_control *octrl = mt9t111_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int mt9t111_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct mt9t111_control *octrl = mt9t111_find_control(ctrl->id);
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
static int __devinit mt9t111_probe(struct i2c_client *client, const struct i2c_device_id * i2c_id)
{
	int ret = 0;
	struct mt9t111_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;
	printk("mt9t111_probe \n");
	ccic_set_clock_parallel_52M();

	pdata->power_on(1, 1);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct mt9t111_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &mt9t111_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);
	/*
	 * Make sure it's an mt9t111
	 */
	
	ret = mt9t111_detect(client);
	if (ret)
		goto out_free;
	printk(KERN_NOTICE "OmniVision mt9t111 sensor detected\n");
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


static int mt9t111_remove(struct i2c_client *client)
{
	return 0;	//TODO
}


static int mt9t111_streamon(struct i2c_client *client)
{
    u16 reg_val = 0;
    u16 times = 0;
    u16 val;
	//fmt_cnt=0;
	printk("streamon: start......sensor mode = %d\n", capture_mode);
	mt9t111_read(client, 0x0018,  &val);
	val &= ~(0x0001);
	mt9t111_write(client, 0x0018, val);
    switch (capture_mode)
    {
    case CAPTURE_MODE_PREVIEW:
		printk("mt9t111_streamon:CAPTURE_MODE_PREVIEW\n");
        if (mt9t111_write(client, 0x098E, 0xEC05) < 0)
        {
            //return FALSE;
        }
        if (mt9t111_write(client, 0x0990, 0x0005) < 0)
        {
            //return FALSE;
        }
        if (mt9t111_write(client, 0x098E, 0x8400) < 0)
        {
            //return FALSE;
        }
        if (mt9t111_write(client, 0x0990, 0x0001) < 0)
        {
            //return FALSE;
        }
        msleep(10);
        
        while (reg_val != 3 && times != 2000)   // 3 :entery preview state
        {
            if (mt9t111_write(client, 0x098E, 0x8401) < 0)
            {
                //return FALSE;
            }

            if (mt9t111_read(client, 0x0990, &reg_val) < 0)
            {
                //return FALSE;
            }

            times++;
        }
        break;

    case CAPTURE_MODE_STILL:
		printk("mt9t111_streamon:CAPTURE_MODE_STILL\n");
        if (mt9t111_write(client, 0x098E, 0xEC05) < 0)
        {
        	printk("mt9t111_streamon 0xEC05:to still I2C Error try again\n");
			mt9t111_write(client, 0x098E, 0xEC05);
        }
        if (mt9t111_write(client, 0x0990, 0x0000) < 0)
        {
            printk("mt9t111_streamon 0x0000:to still I2C Error try again\n");
			mt9t111_write(client, 0x0990, 0x0000);
        }
        if (mt9t111_write(client, 0x098E, 0x8400) < 0)
        {
            printk("mt9t111_streamon 0x8400:to still I2C Error try again\n");
			mt9t111_write(client, 0x098E, 0x8400);
        }
        if (mt9t111_write(client, 0x0990, 0x0002) < 0)
        {
            printk("mt9t111_streamon 0x0002:to still I2C Error try again\n");
			mt9t111_write(client, 0x0990, 0x0002);
        }
        msleep(10);//10
#if 1
        while (reg_val != 7 && times != 10)
        {
            if (mt9t111_write(client, 0x098E, 0x8401) < 0)
            {
                printk("mt9t111_s_fmt:write to still I2C Error\n");
            }

            if (mt9t111_read(client, 0x0990, &reg_val) < 0)
            {
                printk("mt9t111_s_fmt: 0990 to still I2C Error\n");
            }
            times++;
        }


        if (reg_val != 7)
        {
            if (mt9t111_write(client, 0x098E, 0xEC05) < 0)
            {
                printk("mt9t111_s_fmt: 0xEC05 to still I2C Error\n");
            }
            if (mt9t111_write(client, 0x0990, 0x0000) < 0)
            {
                printk("mt9t111_s_fmt:0x0000 to still I2C Error\n");
            }
            if (mt9t111_write(client, 0x098E, 0x8400) < 0)
            {
                printk("mt9t111_s_fmt:0x8400 to still I2C Error\n");
            }
            if (mt9t111_write(client, 0x0990, 0x0002) < 0)
            {
                printk("mt9t111_s_fmt:0x0002 to still I2C Error\n");
            }
            msleep(500);
        }
#endif
        break;

    default:
		printk("mt9t111_streamon:unsupported sensor mode\n");
        return FALSE;
    }

	return 0;
}

static int mt9t111_streamoff(struct i2c_client *client)
{
	u16 val;
	//fmt_cnt=0;
	printk("streamoff: start......capture_mode= %d \n", capture_mode);
	//mt9t111_read(client, 0x0018,  &val);
	//val |= 0x0001;
	//mt9t111_write(client, 0x0018, val);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9t111_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return mt9t111_read(client, (u16)reg->reg, (u16 *)&(reg->val));
}

static int mt9t111_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return mt9t111_write(client, (u16)reg->reg, (u16)reg->val);
}
#endif
static int mt9t111_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
//printk("mt9t111_command:cmd=%u\n",cmd);
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_MT9T111, 0);

		case VIDIOC_INT_RESET:
			mt9t111_reset(client);
			return 0;
#if 0
		case VIDIOC_INT_INIT:
			return 0;//mt9t111_init(client);		//TODO - should get 3640 default register values
#endif
		case VIDIOC_QUERYCAP:
			return mt9t111_querycap(client, (struct v4l2_capability *) arg);

		case VIDIOC_ENUM_FMT:
			return mt9t111_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_ENUM_FRAMESIZES:
			return mt9t111_enum_fmsize(client, (struct v4l2_frmsizeenum *) arg);
		case VIDIOC_TRY_FMT:
			return mt9t111_try_fmt(client, (struct v4l2_format *) arg, NULL, NULL);
		case VIDIOC_S_FMT:
			return mt9t111_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_QUERYCTRL:
			return mt9t111_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return mt9t111_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return mt9t111_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return mt9t111_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return mt9t111_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return mt9t111_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return mt9t111_streamon(client);
		case VIDIOC_STREAMOFF:
			return mt9t111_streamoff(client);
#ifdef CONFIG_VIDEO_ADV_DEBUG
		case VIDIOC_DBG_G_REGISTER:
			return mt9t111_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return mt9t111_s_register(client, (struct v4l2_dbg_register *) arg);
#endif
	}
	return -EINVAL;
}

static struct i2c_device_id mt9t111_idtable[] = {
	{ "mt9t111", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mt9t111_idtable);

static struct i2c_driver mt9t111_driver = {
	.driver = {
		.name	= "mt9t111",
	},
	.id_table       = mt9t111_idtable,
	.command	= mt9t111_command,
	.probe		= mt9t111_probe,
	.remove		= mt9t111_remove,
};


/*
 * Module initialization
 */
static int __init mt9t111_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision mt9t111 sensor driver, at your service\n");
	return i2c_add_driver(&mt9t111_driver);
}

static void __exit mt9t111_mod_exit(void)
{
	i2c_del_driver(&mt9t111_driver);
}

late_initcall(mt9t111_mod_init);
//module_init(mt9t111_mod_init);
module_exit(mt9t111_mod_exit);

