/*
 * A V4L2 driver for OmniVision MT9V113 cameras.
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
MODULE_DESCRIPTION("A low-level driver for OmniVision mt9v113 sensors");
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
#define MT9V113_FRAME_RATE 30

/*
 * The 3640 sits on i2c with ID 0x42
 */
#define MT9V113_I2C_ADDR 0x3c



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
/*for MT9V113 porting*/
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
struct mt9v113_format_struct;  /* coming later */
struct mt9v113_info {
	struct mt9v113_format_struct *fmt;  /* Current format */
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
 * IMPORTANT RULE: the first entry must be for COM7, see mt9v113_s_fmt for why.
 */
/*TODO - mt9v113_fmt_yuv422_qxga can't work. configuration should be correct*/
//lyw_5242
static struct regval_list  mt9v113_fmt_yuv[]  = {
///     /timming setting/                             
    {0x98C, 0x2703},            //Output Width (A)    
    {0x990, 0x0280},            //      = 640     140    
    {0x98C, 0x2705},            //Output Height (A)   
    {0x990, 0x01e0},            //      = 480     f0    
    {0x98C, 0x2707},            //Output Width (B)    
    {0x990, 0x0280},            //      = 640         
    {0x98C, 0x2709},            //Output Height (B)   
    {0x990, 0x01E0},            //      = 480         
    {0x98C, 0x270D},            //Row Start (A)       
    {0x990, 0x004},             //      = 4            
    {0x98C, 0x270F},            //Column Start (A)    
    {0x990, 0x004},             //      = 4            
    {0x98C, 0x2711},            //Row End (A)         
    {0x990, 0x1EB},             //      = 491          
    {0x98C, 0x2713},            //Column End (A)      
    {0x990, 0x28B},             //      = 651          
    {0x98C, 0x2715},            //Row Speed (A)       
    {0x990, 0x0001},            //      = 1           
    {0x98C, 0x2717},            //Read Mode (A)       
    {0x990, 0x0027},            //      = 38          
    {0x98C, 0x2719},            //sensor_fine_correcti
    {0x990, 0x001A},            //      = 26          
    {0x98C, 0x271B},            //sensor_fine_IT_min (
    {0x990, 0x006B},            //      = 107         
    {0x98C, 0x271D},            //sensor_fine_IT_max_m
    {0x990, 0x006B},            //      = 107         
    {0x98C, 0x271F},            //Frame Lines (A)     
    {0x990, 0x0231},            //   0x02EA   = 746            
    {0x98C, 0x2721},            //Line Length (A)     
    {0x990, 0x044A},            //      = 1098     
    {0x98C, 0x2723},            //Row Start (B)       
    {0x990, 0x004},             //      = 4            
    {0x98C, 0x2725},            //Column Start (B)    
    {0x990, 0x004},             //      = 4            
    {0x98C, 0x2727},            //Row End (B)         
    {0x990, 0x1EB},             //      = 491          
    {0x98C, 0x2729},            //Column End (B)      
    {0x990, 0x28B},             //      = 651          
    {0x98C, 0x272B},            //Row Speed (B)       
    {0x990, 0x0001},            //      = 1           
    {0x98C, 0x272D},            //Read Mode (B)       
    {0x990, 0x0027},            //      = 38          
    {0x98C, 0x272F},            //sensor_fine_correcti
    {0x990, 0x001A},            //      = 26          
    {0x98C, 0x2731},            //sensor_fine_IT_min (
    {0x990, 0x006B},            //      = 107         
    {0x98C, 0x2733},            //sensor_fine_IT_max_m
    {0x990, 0x006B},            //      = 107         
    {0x98C, 0x2735},            //Frame Lines (B)     
    {0x990, 0x02EA},            //      = 746         
    {0x98C, 0x2737},            //Line Length (B)     
    {0x990, 0x044A},            //      = 1098        
    {0x98C, 0x2739},            //Crop_X0 (A)         
    {0x990, 0x0000},            //      = 0           
    {0x98C, 0x273B},            //Crop_X1 (A)         
    {0x990, 0x027F},            //      = 639         
    {0x98C, 0x273D},            //Crop_Y0 (A)         
    {0x990, 0x0000},            //      = 0           
    {0x98C, 0x273F},            //Crop_Y1 (A)         
    {0x990, 0x01DF},            //      = 479         
    {0x98C, 0x2747},            //Crop_X0 (B)         
    {0x990, 0x0000},            //      = 0           
    {0x98C, 0x2749},            //Crop_X1 (B)         
    {0x990, 0x027F},            //      = 639         
    {0x98C, 0x274B},            //Crop_Y0 (B)         
    {0x990, 0x0000},            //      = 0           
    {0x98C, 0x274D},            //Crop_Y1 (B)         
    {0x990, 0x01DF},            //      = 479         
    {0x98C, 0x222D},            //R9 Step             
    {0x990, 0x005B},            //      = 91          
    {0x98C, 0xA408},            //search_f1_50        
    {0x990, 0x15},              //      = 21            
    {0x98C, 0xA409},            //search_f2_50        
    {0x990, 0x18},              //      = 24            
    {0x98C, 0xA40A},            //search_f1_60        
    {0x990, 0x1A},              //      = 26            
    {0x98C, 0xA40B},            //search_f2_60        
    {0x990, 0x1D},              //      = 29            
    {0x98C, 0x2411},            //R9_Step_60 (A)      
    {0x990, 0x005B},            //      = 91          
    {0x98C, 0x2413},            //R9_Step_50 (A)      
    {0x990, 0x006D},            //      = 109         
    {0x98C, 0x2415},            //R9_Step_60 (B)      
    {0x990, 0x005B},            //      = 91          
    {0x98C, 0x2417},            //R9_Step_50 (B)      
    {0x990, 0x006D},            //      = 109         
    {0x98C, 0xA404},            //FD Mode             
    {0x990, 0x10},              //      = 16            
    {0x98C, 0xA40D},            //Stat_min            
    {0x990, 0x02},              //      = 2             
    {0x98C, 0xA40E},            //Stat_max            
    {0x990, 0x03},              //      = 3             
    {0x98C, 0xA410},            //Min_amplitude       
    {0x990, 0x0A},              //      = 10            
////optimize FPS and output fromat/                        
    {0x098C, 0xA20C},           // MCU_ADDRESS [AE_MAX_INDEX]      
    {0x0990, 0x000C},           // MCU_DATA_0                     
    {0x098C, 0xA215},           // MCU_ADDRESS [AE_INDEX_TH23]    
    {0x0990, 0x0008},           // MCU_DATA_0    
///lens shading/                                   
    {0x3658, 0x0030},           // P_RD_P0Q0              
    {0x365A, 0xB82B},           // P_RD_P0Q1                
    {0x365C, 0x42F2},           // P_RD_P0Q2               
    {0x365E, 0xD1F0},           // P_RD_P0Q3               
    {0x3660, 0xBB33},           // P_RD_P0Q4               
    {0x3680, 0x084C},           // P_RD_P1Q0               
    {0x3682, 0xA0AC},           // P_RD_P1Q1                
    {0x3684, 0xFD6F},           // P_RD_P1Q2               
    {0x3686, 0xF031},           // P_RD_P1Q3              
    {0x3688, 0xB92E},           // P_RD_P1Q4               
    {0x36A8, 0x0893},           // P_RD_P2Q0               
    {0x36AA, 0xF792},           // P_RD_P2Q1                
    {0x36AC, 0xB255},           // P_RD_P2Q2                
    {0x36AE, 0x0D17},           // P_RD_P2Q3                
    {0x36B0, 0x65B8},           // P_RD_P2Q4               
    {0x36D0, 0x88F0},           // P_RD_P3Q0               
    {0x36D2, 0x5D6E},           // P_RD_P3Q1                
    {0x36D4, 0xFC53},           // P_RD_P3Q2               
    {0x36D6, 0x91D7},           // P_RD_P3Q3                
    {0x36D8, 0x9719},           // P_RD_P3Q4               
    {0x36F8, 0xF7B3},           // P_RD_P4Q0               
    {0x36FA, 0x6FB4},           // P_RD_P4Q1               
    {0x36FC, 0xA1D7},           // P_RD_P4Q2                
    {0x36FE, 0x163A},           // P_RD_P4Q3               
    {0x3700, 0x4FBD},           // P_RD_P4Q4               
    {0x364E, 0x0030},           // P_GR_P0Q0              
    {0x3650, 0xF32C},           // P_GR_P0Q1               
    {0x3652, 0x41F2},           // P_GR_P0Q2              
    {0x3654, 0xDCF0},           // P_GR_P0Q3               
    {0x3656, 0x8194},           // P_GR_P0Q4              
    {0x3676, 0xDDEA},           // P_GR_P1Q0                
    {0x3678, 0x050E},           // P_GR_P1Q1              
    {0x367A, 0x14B0},           // P_GR_P1Q2               
    {0x367C, 0x9073},           // P_GR_P1Q3               
    {0x367E, 0x8193},           // P_GR_P1Q4              
    {0x369E, 0x04B3},           // P_GR_P2Q0               
    {0x36A0, 0x97D3},           // P_GR_P2Q1                
    {0x36A2, 0xCFD5},           // P_GR_P2Q2                
    {0x36A4, 0x2157},           // P_GR_P2Q3               
    {0x36A6, 0x5118},           // P_GR_P2Q4               
    {0x36C6, 0xC14E},           // P_GR_P3Q0               
    {0x36C8, 0x9C92},           // P_GR_P3Q1               
    {0x36CA, 0xB6B1},           // P_GR_P3Q2                
    {0x36CC, 0x86D7},           // P_GR_P3Q3                
    {0x36CE, 0xBCD9},           // P_GR_P3Q4                
    {0x36EE, 0xA454},           // P_GR_P4Q0               
    {0x36F0, 0x7CB5},           // P_GR_P4Q1               
    {0x36F2, 0xE1D7},           // P_GR_P4Q2               
    {0x36F4, 0x2B19},           // P_GR_P4Q3               
    {0x36F6, 0x5AFD},           // P_GR_P4Q4                
    {0x3662, 0x0030},           // P_BL_P0Q0              
    {0x3664, 0x89ED},           // P_BL_P0Q1               
    {0x3666, 0x3AB2},           // P_BL_P0Q2               
    {0x3668, 0xFFCF},           // P_BL_P0Q3               
    {0x366A, 0xEB93},           // P_BL_P0Q4               
    {0x368A, 0x654A},           // P_BL_P1Q0                
    {0x368C, 0xE6CE},           // P_BL_P1Q1               
    {0x368E, 0xB6F0},           // P_BL_P1Q2               
    {0x3690, 0x0531},           // P_BL_P1Q3              
    {0x3692, 0x6792},           // P_BL_P1Q4              
    {0x36B2, 0x6CD2},           // P_BL_P2Q0                
    {0x36B4, 0xA7F3},           // P_BL_P2Q1               
    {0x36B6, 0x8536},           // P_BL_P2Q2               
    {0x36B8, 0x36D7},           // P_BL_P2Q3               
    {0x36BA, 0x7978},           // P_BL_P2Q4               
    {0x36DA, 0x928C},           // P_BL_P3Q0                
    {0x36DC, 0x4392},           // P_BL_P3Q1               
    {0x36DE, 0x8FD3},           // P_BL_P3Q2                
    {0x36E0, 0x82B8},           // P_BL_P3Q3               
    {0x36E2, 0xC1F9},           // P_BL_P3Q4               
    {0x3702, 0xBCF3},           // P_BL_P4Q0               
    {0x3704, 0x4036},           // P_BL_P4Q1              
    {0x3706, 0x8DB6},           // P_BL_P4Q2               
    {0x3708, 0x42D7},           // P_BL_P4Q3               
    {0x370A, 0x3B9D},           // P_BL_P4Q4                
    {0x366C, 0x0150},           // P_GB_P0Q0               
    {0x366E, 0xC90C},           // P_GB_P0Q1               
    {0x3670, 0x4332},           // P_GB_P0Q2              
    {0x3672, 0xBD10},           // P_GB_P0Q3               
    {0x3674, 0xF2B3},           // P_GB_P0Q4               
    {0x3694, 0x8F2C},           // P_GB_P1Q0               
    {0x3696, 0x118E},           // P_GB_P1Q1              
    {0x3698, 0x494F},           // P_GB_P1Q2              
    {0x369A, 0x9B93},           // P_GB_P1Q3               
    {0x369C, 0xA753},           // P_GB_P1Q4               
    {0x36BC, 0x02B3},           // P_GB_P2Q0               
    {0x36BE, 0x9B13},           // P_GB_P2Q1               
    {0x36C0, 0xBDD5},           // P_GB_P2Q2                
    {0x36C2, 0x1CD7},           // P_GB_P2Q3                
    {0x36C4, 0x2538},           // P_GB_P2Q4               
    {0x36E4, 0x98CE},           // P_GB_P3Q0               
    {0x36E6, 0xDDD2},           // P_GB_P3Q1                
    {0x36E8, 0xD971},           // P_GB_P3Q2               
    {0x36EA, 0xEF76},           // P_GB_P3Q3               
    {0x36EC, 0xA279},           // P_GB_P3Q4               
    {0x370C, 0x8A94},           // P_GB_P4Q0               
    {0x370E, 0x06D6},           // P_GB_P4Q1               
    {0x3710, 0xF457},           // P_GB_P4Q2              
    {0x3712, 0x5DB9},           // P_GB_P4Q3               
    {0x3714, 0x6DFD},           // P_GB_P4Q4                
    {0x3644, 0x015C},           // POLY_ORIGIN_C           
    {0x3642, 0x00F4},           // POLY_ORIGIN_R          
    {0x3210, 0x09B8},           // COLOR_PIPELINE_CONTROL  
///saturation and AP/                                      
///saturation/                                             
    {0x098C, 0xAB20},           // MCU_ADDRESS [HG_LL_SAT1]         
    {0x0990, 0x0060},           // MCU_DATA_0                     
    {0x098C, 0xAB24},           // MCU_ADDRESS [HG_LL_SAT2]         
    {0x0990, 0x0040},           // MCU_DATA_0                     
    {0x098C, 0xAB22},           // MCU_ADDRESS [HG_LL_APCORR1]      
    {0x0990, 0x0004},           // MCU_DATA_0                     
    {0x098C, 0xAB26},           // MCU_ADDRESS [HG_LL_APCORR2]      
    {0x0990, 0x0001},           // MCU_DATA_0                     
    {0x098C, 0xAB27},           // MCU_ADDRESS [HG_LL_APTHRESH2]    
    {0x0990, 0x0008},           // MCU_DATA_0   
///denoise/                                                                
    {0x098C, 0xA20E},           // MCU_ADDRESS [AE_MAX_VIRTGAIN]                   
    {0x0990, 0x008C},           // MCU_DATA_0 

    {0x098C, 0x2717},           // MCU_ADDRESS [AE_MAX_VIRTGAIN]                   
    {0x0990, 0x0027},           // MCU_DATA_0 
    {0x098C, 0x272D},           // MCU_ADDRESS [AE_MAX_VIRTGAIN]                   
    {0x0990, 0x0027},           // MCU_DATA_0 

//AE stable/                                                               
    {0x098C, 0xAB1F},           // MCU_ADDRESS [HG_LLMODE]                          
    {0x0990, 0x00C8},           // MCU_DATA_0                                      
    {0x098C, 0x2B28},           // MCU_ADDRESS [HG_LL_BRIGHTNESSSTART]             
    {0x0990, 0x03E8},           // MCU_DATA_0                                     
    {0x098C, 0x2B2A},           // MCU_ADDRESS [HG_LL_BRIGHTNESSSTOP]               
    {0x0990, 0x07D0},           // MCU_DATA_0                                      
    {0x098C, 0x2B38},           // MCU_ADDRESS [HG_GAMMASTARTMORPH]                
    {0x0990, 0x03E8},           // MCU_DATA_0                                     
    {0x098C, 0x2B3A},           // MCU_ADDRESS [HG_GAMMASTOPMORPH]                  
    {0x0990, 0x07D0},           // MCU_DATA_0                                      
    {0x098C, 0xA244},           // MCU_ADDRESS [AE_DRTFEATURECTRL]                 
    {0x0990, 0x00B3},           // MCU_DATA_0                                      
    {0x098C, 0xA202},           // AE_WINDOW_POS [1]                                  
    {0x0990, 0x44},
    {0x098C, 0xA203},           // AE_WINDOW_SIZE [1]                                 
    {0x0990, 0x88},
    {0x098C, 0xA207},           //(2) AE_GATE                                         
    {0x0990, 0x10},
    {0x098C, 0xA24C},           //(1) AE_TARGETBUFFERSPEED                            
    {0x0990, 0x10},
    {0x098C, 0xA24F},           //(5) AE_BASETARGET                                   
    {0x0990, 0x42},
    {0x098C, 0xA109},           //(2) SEQ_AE_FASTBUFF                                 
    {0x0990, 0x1E},
    {0x098C, 0xA10A},           //(2) SEQ_AE_FASTSTEP                                 
    {0x0990, 0x02},
    {0x098C, 0xA20D},           //(1) AE_MIN_VIRTGAIN                                 
    {0x0990, 0x30},
    {0x098C, 0xA24A},           // AE_TARGETMIN [1]                                   
    {0x0990, 0x28},
    {0x098C, 0xA24B},           // AE_TARGETMAX [1]                                   
    {0x0990, 0x96},

//    {0x098C, 0xa766 },
//    {0x0990, 0x0003 },	// test pattern
    
    {0x098C, 0xA103},           // MCU_ADDRESS                                     
    {0x0990, 0x0005},           // MCU_DATA_0                                     

	{0xffff, 0x00ff}        /* End of file marker (0xFFFF)*/
};
static struct regval_list  mt9v113_fmt_jpeg_setting[]  = {
//[JPEG422 capure]
	{0x098c, 0x6C07},    //MCU_ADDRESS [PRI_B_OUTPUT_FORMAT]
	{0x0990, 0x0001},    //MCU_DATA_0
	{0x098c, 0x6C09},    //MCU_ADDRESS [PRI_B_OUTPUT_FORMAT_ORDER]
	{0x0990, 0x0000},    //MCU_DATA_0
	{0x098c, 0xEC8E},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_JP_MODE]
	{0x0990, 0x0001},    //MCU_DATA_0
	{0x098c, 0xEC8F},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_FORMAT]
	{0x0990, 0x0002},    //MCU_DATA_0
	{0x098c, 0x6CA0},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR]
	{0x0990, 0x083D},    //0x082D},    //MCU_DATA_0
	{0x098c, 0xEC9F},    //MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_CONTROL_VAR]
	{0x0990, 0x0001},    //MCU_DATA_0
	{0xffff,0x00ff},
};

//176*144
static struct regval_list  mt9v113_fmt_yuv422_qcif[]  = {
{0x098c,  0x2703},      //Output Width (A) 
{0x0990,  176},         //      = 176     
{0x098c,  0x2705},      //Output Height (A)
{0x0990,  144},        //      = 144     

{0xffff,0x00ff},
};

static struct regval_list  mt9v113_fmt_yuv422_qcif_rotate[]  = {
    {0x098c,  0x2703},      //Output Width (A) 
    {0x0990,  144},         //      = 144     
    {0x098c,  0x2705},      //Output Height (A)
    {0x0990,  176},        //      = 176     
    {0x98C, 0x270D},            //Row Start (A)       
    {0x990, 4},             //      = 4            
    {0x98C, 0x270F},            //Column Start (A)    
    {0x990, 165},             //      = 165            
    {0x98C, 0x2711},            //Row End (A)         
    {0x990, 491},             //      = 491          
    {0x98C, 0x2713},            //Column End (A)      
    {0x990, 492},             //      = 492
    
    {0x98C, 0x2739},            //Crop_X0 (A)         
    {0x990, 0x0000},            //      = 0           
    {0x98C, 0x273B},            //Crop_X1 (A)         
    {0x990, 319},            //      = 319         
    {0x98C, 0x273D},            //Crop_Y0 (A)         
    {0x990, 0x0000},            //      = 0           
    {0x98C, 0x273F},            //Crop_Y1 (A)         
    {0x990, 479},            //      = 479   
    {0xffff,0x00ff},
};

//320*240
static struct regval_list  mt9v113_fmt_yuv422_qvga[]  = {
{0x098c,  0x2703},      //Output Width (A) 
{0x0990,  320},         //      = 320     
{0x098c,  0x2705},      //Output Height (A)
{0x0990,  240},        //      = 240     

{0xffff,0x00ff},
};

//640*480
static struct regval_list  mt9v113_fmt_yuv422_vga[]  = {
{0x098c,  0x2703},      //Output Width (A) 
{0x0990,  640},         //      = 176     
{0x098c,  0x2705},      //Output Height (A)
{0x0990,  480},        //      = 144     

{0xffff,0x00ff},
};

//176*144
static struct regval_list  mt9v113_fmt_jpeg_qcif[]  = {
{0x098c,  0x2707},      //Output Width (B)         
{0x0990,  176},      //      = 176             
{0x098c,  0x2709},      //Output Height (B)        
{0x0990,  144},      //      = 144             

{0xffff,0x00ff},
};

//320*240
static struct regval_list  mt9v113_fmt_jpeg_qvga[]  = {
{0x098c,  0x2707},      //Output Width (B)         
{0x0990,  320},      //      = 320             
{0x098c,  0x2709},      //Output Height (B)        
{0x0990,  240},      //      = 240             

{0xffff,0x00ff},
};

//640*480
static struct regval_list  mt9v113_fmt_jpeg_vga[]  = {
{0x098c,  0x2707},      //Output Width (B)         
{0x0990,  640},      //      = 640             
{0x098c,  0x2709},      //Output Height (B)        
{0x0990,  480},      //      = 480             

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
	//printk("mt9v113: read reg [%04x]=%02x\n", addr, *pvalue);
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


static int mt9v113_read(struct i2c_client *c, u16 reg,u16 *value)
{
	int ret;

	ret = read_sensor_reg_16bit(c, reg, value);
	
	return ret;
}

static int mt9v113_write(struct i2c_client *c, u16 reg, u16 value)
{
	int ret;

	ret = write_sensor_reg_16bit(c, reg, &value);
	
	return ret;
}


/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int mt9v113_write_array(struct i2c_client *c,struct regval_list *vals)
{
	int i = 0;
	while (vals->reg_num != 0xffff ) {

	int ret = mt9v113_write(c, vals->reg_num, vals->value);
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
static void mt9v113_reset(struct i2c_client *client)
{
//printk("mt9v113_reset\n");

	//mt9v113_write(client, REG_SYS, SYS_RESET);
	//msleep(1);
}

static int mt9v113_detect(struct i2c_client *client)
{
	u16 v;
	int ret;
	printk("mt9v113_detect \n");
	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	ret = read_sensor_reg_16bit(client, 0x0000, &v);
	printk("mt9v113_detect:chip_id= %x\n",v);	
	if (ret < 0)
		return ret;
	if (v != 0x2280)
		return -ENODEV;
	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct mt9v113_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
	int bpp;   /* bits per pixel */
} mt9v113_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= mt9v113_fmt_yuv,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },	//TODO
		.bpp		= 16,
	},
	{
		.desc		= "YUYV422 planar",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.regs 		= mt9v113_fmt_yuv,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 16,
	},
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.regs           = mt9v113_fmt_yuv,
		.cmatrix        = { 128, -128, 0, -34, -94, 128 },
		.bpp            = 12,
	},
};
#define N_MT9V113_FMTS ARRAY_SIZE(mt9v113_formats)

/*TODO - also can use ccic size register 0x34 to do same thing for cropping...anyway, sensor doing it is better?
  0x3020~0x3027*/
static struct mt9v113_win_size {
	int	width;
	int	height;
} mt9v113_win_sizes[] = {
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
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(mt9v113_win_sizes))
/* capture jpeg size */
static struct mt9v113_win_size mt9v113_win_sizes_jpeg[] = {
	{
		.width = 640,
		.height = 480,
	},
	{
		.width = 320,
		.height = 240,
	},
};



static int mt9v113_querycap(struct i2c_client *c, struct v4l2_capability *argp)
{
	if(!argp){
		printk(KERN_ERR" argp is NULL %s %d \n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	strcpy(argp->driver, "mt9v113");
	strcpy(argp->card, "TD/TTC");
	return 0;
}

static int mt9v113_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct mt9v113_format_struct *ofmt;
	//printk("mt9v113_enum_fmt \n");
	if (fmt->index >= N_MT9V113_FMTS)
		return -EINVAL;

	ofmt = mt9v113_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}
static int mt9v113_enum_fmsize(struct i2c_client *c, struct v4l2_frmsizeenum *argp)
{
	struct v4l2_frmsizeenum frmsize;

	if (copy_from_user(&frmsize, argp, sizeof(frmsize)))
		   return -EFAULT;

	if (frmsize.pixel_format == V4L2_PIX_FMT_YUV420||
		frmsize.pixel_format == V4L2_PIX_FMT_YUYV ||
		frmsize.pixel_format == V4L2_PIX_FMT_YUV422P){
		if (frmsize.index >= (ARRAY_SIZE(mt9v113_win_sizes)))
{
		    return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = mt9v113_win_sizes[frmsize.index].height;
		frmsize.discrete.width = mt9v113_win_sizes[frmsize.index].width;
	}else if(frmsize.pixel_format == V4L2_PIX_FMT_JPEG){
		if (frmsize.index >= ARRAY_SIZE(mt9v113_win_sizes_jpeg)){
			   return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = mt9v113_win_sizes_jpeg[frmsize.index].height;
		frmsize.discrete.width = mt9v113_win_sizes_jpeg[frmsize.index].width;

	}else
	   return -EINVAL;

	if (copy_to_user(argp, &frmsize, sizeof(frmsize)))
		   return -EFAULT;
	return 0;
}

static int mt9v113_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct mt9v113_format_struct **ret_fmt,
		struct mt9v113_win_size **ret_wsize)
{
	int index;
	struct mt9v113_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	//printk("mt9v113_try_fmt \n");

	
	for (index = 0; index < N_MT9V113_FMTS; index++)
		if (mt9v113_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_MT9V113_FMTS){
		printk("unsupported format!\n");
		return -EINVAL;
	}
	if (ret_fmt != NULL)
		*ret_fmt = mt9v113_formats + index;
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
	for (wsize = mt9v113_win_sizes; wsize < mt9v113_win_sizes + N_WIN_SIZES;
			wsize++)
		if (pix->width <= wsize->width && pix->height <= wsize->height)
			break;
	if (wsize >= mt9v113_win_sizes + N_WIN_SIZES){
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
	pix->bytesperline = pix->width*mt9v113_formats[index].bpp/8;
	pix->sizeimage = pix->height*pix->bytesperline;
	printk("mt9v113_try_fmt: pix->width is %d, pix->height is %d wsize %d\n", pix->width, pix->height, wsize->width);

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
				case VGA_WIDTH:
					printk("try_fmt:mt9v113_fmt_yuv422_vga\n");
					(*ret_fmt)->regs = mt9v113_fmt_yuv422_vga;
					break;
					
				case QVGA_WIDTH:
					printk("try_fmt:mt9v113_fmt_yuv422_qvga\n");
					(*ret_fmt)->regs = mt9v113_fmt_yuv422_qvga;
					break;
					
				case QCIF_WIDTH:
					printk("try_fmt:mt9v113_fmt_yuv422_qcif\n");
					(*ret_fmt)->regs = mt9v113_fmt_yuv422_qcif;
					break;
					
				case 144:
					printk("try_fmt:mt9v113_fmt_yuv422_qcif_rotate\n");
					(*ret_fmt)->regs = mt9v113_fmt_yuv422_qcif_rotate;
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
				case QCIF_WIDTH:
					printk("try_fmt:mt9v113_fmt_jpeg_qcif\n");
					(*ret_fmt)->regs = mt9v113_fmt_jpeg_qcif;
					break;
				case VGA_WIDTH:
					printk("try_fmt:mt9v113_fmt_jpeg_vga\n");
					(*ret_fmt)->regs = mt9v113_fmt_jpeg_vga;
					break;
					
				case QVGA_WIDTH:
					printk("try_fmt:mt9v113_fmt_jpeg_qvga\n");
					(*ret_fmt)->regs = mt9v113_fmt_jpeg_qvga;
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
static int mt9v113_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
//	u16 reg_val = 0;
   // u16 times = 0;
	struct mt9v113_format_struct *ovfmt;
	struct mt9v113_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	
	struct sensor_platform_data *pdata;
	pdata = c->dev.platform_data;
	
	if(1 == is_init_reg)    
	{
		mt9v113_write(c, 0x301A, 0x1218);// RESET_REGISTER
		msleep(100);
		mt9v113_write(c, 0x301A, 0x121C);// RESET_REGISTER
		msleep(100);
		mt9v113_write(c, 0x001A, 0x0011);// RESET_AND_MISC_CONTROL
		msleep(10);
		mt9v113_write(c, 0x001A, 0x0010);// RESET_AND_MISC_CONTROL 
		msleep(10);
		mt9v113_write(c, 0x0018, 0x4028);// STANDBY_CONTROL 
		msleep(50);
		mt9v113_write(c, 0x001A, 0x0210);// RESET_AND_MISC_CONTROL 
		mt9v113_write(c, 0x001E, 0x0777);// PAD_SLEW  
	/////PLL enable/                                        
		mt9v113_write(c, 0x0016, 0x42DF);// CLOCKS_CONTROL 
		mt9v113_write(c, 0x0010, 0x0110);//PLL Dividers = 272 
		mt9v113_write(c, 0x0012, 0x0000);//PLL P Dividers = 0  
		msleep(10);
		mt9v113_write(c, 0x0014, 0x2147);// PLL_CONTROL
		msleep(10);
		mt9v113_write(c, 0x0014, 0x2047);// PLL_CONTROL
		msleep(10);
		mt9v113_write(c, 0x0014, 0xA046);
		mt9v113_write_array(c, mt9v113_fmt_yuv);
		msleep(50);
		is_init_reg = 0;
	}
	ret = mt9v113_try_fmt(c, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	mt9v113_write_array(c, ovfmt->regs);
	printk("mt9v113_s_fmt mt9v113_write_array end\n");
	
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
			//mt9v113_write_array(c, mt9v113_fmt_jpeg_setting);
			break;
		default:
			capture_mode = CAPTURE_MODE_PREVIEW;
			printk("mt9v113_s_fmt unsupported format!\n");
			break;
		
	}
	
			
	mt9v113_write(c, 0x098c, 0xa103);     // MCU_ADDRESS [SEQ_CMD]
	mt9v113_write(c, 0x0990, 0x0006);      // MCU_DATA_0
	msleep(100); 
	//mt9v113_write(c, 0x098c, 0xa103);     // MCU_ADDRESS [SEQ_CMD]
	//mt9v113_write(c, 0x0990, 0x0005);      // MCU_DATA_0
	//msleep(200);
	
	printk("mt9v113_s_fmt sensor mode = %d\n", capture_mode);
	
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int mt9v113_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	u16 clkrc;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	ret = mt9v113_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = MT9V113_FRAME_RATE;
	//if ((clkrc & CLK_EXT) == 0 && (clkrc & CLK_SCALE) > 1)
	//	cp->timeperframe.denominator /= (clkrc & CLK_SCALE);
	return 0;
}

static int mt9v113_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int mt9v113_s_input(struct i2c_client *c, int *id)
{
	is_init_reg = 1;     
     printk("mt9v113_s_input \n");

	return 0;
}





static struct mt9v113_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} mt9v113_controls[] =
{

};
#define N_CONTROLS (ARRAY_SIZE(mt9v113_controls))

static struct mt9v113_control *mt9v113_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (mt9v113_controls[i].qc.id == id)
			return mt9v113_controls + i;
	return NULL;
}


static int mt9v113_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct mt9v113_control *ctrl = mt9v113_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int mt9v113_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct mt9v113_control *octrl = mt9v113_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int mt9v113_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct mt9v113_control *octrl = mt9v113_find_control(ctrl->id);
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
static int __devinit mt9v113_probe(struct i2c_client *client, const struct i2c_device_id * i2c_id)
{
	int ret = 0;
	struct mt9v113_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;
	printk("mt9v113_probe \n");
	ccic_set_clock_parallel_26M();

	pdata->power_on(1, 1);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct mt9v113_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &mt9v113_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);
	/*
	 * Make sure it's an mt9v113
	 */
	
	ret = mt9v113_detect(client);
	if (ret)
		goto out_free;
	printk(KERN_NOTICE "OmniVision mt9v113 sensor detected\n");
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


static int mt9v113_remove(struct i2c_client *client)
{
	return 0;	//TODO
}


static int mt9v113_streamon(struct i2c_client *client)
{
    u16 reg_val = 0;
    u16 times = 0;
    u16 val;
	//fmt_cnt=0;
	printk("streamon: start......sensor mode = %d\n", capture_mode);
	mt9v113_read(client, 0x0018,  &val);
	val &= ~(0x0001);
	mt9v113_write(client, 0x0018, val);
    switch (capture_mode)
    {
    case CAPTURE_MODE_PREVIEW:
		printk("mt9v113_streamon:CAPTURE_MODE_PREVIEW\n");
        if (mt9v113_write(client, 0x098c, 0xa103) < 0)
        {
            //return FALSE;
        }
        if (mt9v113_write(client, 0x0990, 0x0001) < 0)
        {
            //return FALSE;
        }
        msleep(10);

        break;

    case CAPTURE_MODE_STILL:
		printk("mt9v113_streamon:CAPTURE_MODE_STILL\n");
        if (mt9v113_write(client, 0x098c, 0xa103) < 0)
        {
        	printk("mt9v113_streamon 0xEC05:to still I2C Error try again\n");
			mt9v113_write(client, 0x098c, 0xa103);
        }
        if (mt9v113_write(client, 0x0990, 0x0002) < 0)
        {
            printk("mt9v113_streamon 0x0000:to still I2C Error try again\n");
			mt9v113_write(client, 0x0990, 0x0002);
        }
        if (mt9v113_write(client, 0x098c, 0xA115) < 0)
        {
            printk("mt9v113_streamon 0xa103:to still I2C Error try again\n");
			mt9v113_write(client, 0x098c, 0xA115);
        }
        if (mt9v113_write(client, 0x0990, 0x0002) < 0)
        {
            printk("mt9v113_streamon 0x0002:to still I2C Error try again\n");
			mt9v113_write(client, 0x0990, 0x0002);
        }
        msleep(10);//10
        break;

    default:
		printk("mt9v113_streamon:unsupported sensor mode\n");
        return FALSE;
    }

	return 0;
}

static int mt9v113_streamoff(struct i2c_client *client)
{
	u16 val;
	//fmt_cnt=0;
	printk("streamoff: start......capture_mode= %d \n", capture_mode);
	//mt9v113_read(client, 0x0018,  &val);
	//val |= 0x0001;
	//mt9v113_write(client, 0x0018, val);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9v113_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return mt9v113_read(client, (u16)reg->reg, (u16 *)&(reg->val));
}

static int mt9v113_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return mt9v113_write(client, (u16)reg->reg, (u16)reg->val);
}
#endif
static int mt9v113_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
//printk("mt9v113_command:cmd=%u\n",cmd);
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_MT9V113, 0);

		case VIDIOC_INT_RESET:
			mt9v113_reset(client);
			return 0;
#if 0
		case VIDIOC_INT_INIT:
			return 0;//mt9v113_init(client);		//TODO - should get 3640 default register values
#endif
		case VIDIOC_QUERYCAP:
			return mt9v113_querycap(client, (struct v4l2_capability *) arg);

		case VIDIOC_ENUM_FMT:
			return mt9v113_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_ENUM_FRAMESIZES:
			return mt9v113_enum_fmsize(client, (struct v4l2_frmsizeenum *) arg);
		case VIDIOC_TRY_FMT:
			return mt9v113_try_fmt(client, (struct v4l2_format *) arg, NULL, NULL);
		case VIDIOC_S_FMT:
			return mt9v113_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_QUERYCTRL:
			return mt9v113_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return mt9v113_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return mt9v113_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return mt9v113_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return mt9v113_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return mt9v113_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return mt9v113_streamon(client);
		case VIDIOC_STREAMOFF:
			return mt9v113_streamoff(client);
#ifdef CONFIG_VIDEO_ADV_DEBUG
		case VIDIOC_DBG_G_REGISTER:
			return mt9v113_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return mt9v113_s_register(client, (struct v4l2_dbg_register *) arg);
#endif
	}
	return -EINVAL;
}

static struct i2c_device_id mt9v113_idtable[] = {
	{ "mt9v113", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mt9v113_idtable);

static struct i2c_driver mt9v113_driver = {
	.driver = {
		.name	= "mt9v113",
	},
	.id_table       = mt9v113_idtable,
	.command	= mt9v113_command,
	.probe		= mt9v113_probe,
	.remove		= mt9v113_remove,
};


/*
 * Module initialization
 */
static int __init mt9v113_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision mt9v113 sensor driver, at your service\n");
	return i2c_add_driver(&mt9v113_driver);
}

static void __exit mt9v113_mod_exit(void)
{
	i2c_del_driver(&mt9v113_driver);
}

late_initcall(mt9v113_mod_init);
//module_init(mt9v113_mod_init);
module_exit(mt9v113_mod_exit);

