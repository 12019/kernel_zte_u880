/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa910.h>
#include <mach/irqs.h>

#include <plat/generic.h>
#include <plat/i2c.h>

#include "../common.h"
#include "../../../../drivers/video/pxa910fb.h"

#define LCD_HX8347_ID 0x0047

#define DEBUG_LCD_LEAD
#ifdef DEBUG_LCD_LEAD
	#define DPRINTK(fmt, args...)	printk("<%s:%d> %s : " fmt "\n", __FILE__ , __LINE__ , __FUNCTION__ , ## args)
#else
	#define DPRINTK(fmt, args...)
#endif

#define SHOW_FUNCTION_ENTRY	DPRINTK("Begin +++")
#define SHOW_FUNCTION_EXIT	DPRINTK("End   ---")

extern int pxa910fb_smart_cmd_simple(struct pxa910fb_info *fbi, unsigned int *cmd, unsigned int num);
extern void smart_trigger_vsync(struct pxa910fb_info *fbi);
extern void pxa910fb_print_regs(struct pxa910fb_info *fbi);

extern void lcd_power_on(int on, int ms);

struct reg_value
{
	unsigned int reg;
	unsigned int value;
	unsigned int msleep;
};

struct reg_value hx8347_reg[] = {
	{0x00EA,0x0000,0},//PTBA[15:8]
	{0x00EB,0x0020,0},//PTBA[7:0]
	{0x00EC,0x000C,0},//STBA[15:8]
	{0x00ED,0x00C4,0},//STBA[7:0]
	{0x00E8,0x0038,0},//OPON[7:0]
	{0x00E9,0x0038,0},//10//OPON1[7:0]
	{0x00F1,0x0001,0},//OTPS1B
	{0x00F2,0x0010,0},//GEN
	{0x0027,0x00A3,0},

	//Gamma 2.2 Setting
	{0x0040,0x0000,0},//
	{0x0041,0x0000,0},
	{0x0042,0x0001,0},
	{0x0043,0x0012,0},
	{0x0044,0x0010,0},
	{0x0045,0x0026,0},
	{0x0046,0x0008,0},
	{0x0047,0x0054,0},
	{0x0048,0x0002,0},
	{0x0049,0x0015,0},
	{0x004a,0x0019,0},
	{0x004b,0x0019,0},
	{0x004c,0x0016,0},

	{0x0050,0x0019,0},
	{0x0051,0x002f,0},
	{0x0052,0x002d,0},
	{0x0053,0x003e,0},
	{0x0054,0x003f,0},
	{0x0055,0x003f,0},
	{0x0056,0x002b,0},
	{0x0057,0x0077,0},
	{0x0058,0x0009,0},
	{0x0059,0x0006,0},
	{0x005a,0x0006,0},
	{0x005b,0x000a,0},
	{0x005c,0x001d,0},
	{0x005d,0x00cc,0},


	//Power Voltage Setting
	{0x001b,0x001B,0},//VRH=4.65V
	{0x001A,0x0001,0},//BT (VGH~15V,VGL~-10V,DDVDH~5V)
	{0x0024,0x0039,0},//VMH(VCOM High voltage ~3.2V) 
	{0x0025,0x006E,0},//VML(VCOM Low voltage -1.2V) 
	{0x0023,0x0079,0},//for Flicker adjust //can reload from OTP 

	//Power on Setting
	{0x0018,0x0036,0}, //I/P_RADJ,N/P_RADJ, Normal mode 75Hz
	{0x0019,0x0001,0}, //OSC_EN='1', start Osc
	{0x0001,0x0000,0}, //DP_STB='0', out deep sleep
	{0x001F,0x0088,5}, //GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
	//Delayms(5);
	{0x001F,0x0080,5}, //GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0
	//Delayms(5);
	{0x001F,0x0090,5}, //GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0
	//Delayms(5);
	{0x001F,0x00D0,5}, //GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0, STB=0
	//Delayms(5);


	//262k/65k color selection
	{0x0017,0x0006,0}, //default 0x60 262k color // 0x50 65k color     
    //SET PANEL
	{0x0036,0x0000,0}, //SS_P, GS_P,REV_P,BGR_P
	//Display ON Setting
	{0x0028,0x0038,40}, //GON=1, DTE=1, D=1000
	//Delayms(40,0},
	{0x0028,0x003C,0}, //GON=1, DTE=1, D=1100

	//240x320 window setting
	{0x0002,0x0000,0}, 
	{0x0003,0x0000,0}, //Column Start
	{0x0004,0x0000,0}, 
	{0x0005,0x00EF,0}, //Column End

	{0x0006,0x0000,0},
	{0x0007,0x0000,0}, //Row Start
	{0x0008,0x0001,0}, 
	{0x0009,0x003F,0}, //Row End

	//{(0x0022); //Start GRAM write
	//CABC Control
	{0x003C,0x00F0,0}, //The backlight PWM pulse output duty is equal to (DBV[7:0]+1)/256 x CABC_duty.
	{0x003D,0x002C,0}, //BCTRL-d5; DD-d3; BL-d2; 1 ON; 0 OFF
	{0x003E,0x0001,0}, //C[1:0]
	{0x003F,0x0000,0}, //set the minimum brightness value of the display for CABC function.


};

struct reg_value hx8347_reg_windows[] = {

	{0x0002,0x0000,0}, 
	{0x0003,0x0000,0}, //Column Start
	{0x0004,0x0000,0}, 
	{0x0005,0x00EF,0}, //Column End

	{0x0006,0x0000,0},
	{0x0007,0x0000,0}, //Row Start
	{0x0008,0x0001,0}, 
	{0x0009,0x003F,0}, //Row End
	
};

struct reg_value hx8347_reg_standby[] = {
    {0x0007, 0x0131, 10},    
    {0x0007, 0x0130, 10},    
    {0x0007, 0x0000, 0},   
 //************* Power OFF sequence **************//
    {0x0010, 0x0080, 0},    
    {0x0011, 0x0000, 0},     
    {0x0012, 0x0000, 0},      
    {0x0013, 0x0000, 200},  
    {0x0010, 0x0082, 0},     
};

static int cs_gpio = 0;
 
static unsigned long tpo_lcd_gpio_pin_config[] = {
	GPIO81_LCD_RDB,
	GPIO82_LCD_A0,
	GPIO83_LCD_WRB,
	GPIO85_LCD_DD0,		
	GPIO86_LCD_DD1	,	
	GPIO87_LCD_DD2	,	
	GPIO88_LCD_DD3	,	
	GPIO89_LCD_DD4	,	
	GPIO90_LCD_DD5	,	
	GPIO91_LCD_DD6	,	
	GPIO92_LCD_DD7	,	
	GPIO93_LCD_DD8	,	
	GPIO94_LCD_DD9	,	
	GPIO95_LCD_DD10	,
	GPIO96_LCD_DD11	,
	GPIO97_LCD_DD12	,
	GPIO98_LCD_DD13	,
	GPIO100_LCD_DD14	,
	GPIO101_LCD_DD15	,
	GPIO102_LCD_DD16	,
	GPIO103_LCD_DD17	,
	GPIO104_LCD_DD18, 
	GPIO106_LCD_RST

};



#define	 LCD_SPU_DMA_CTRL0			0x0190
#define    CFG_GRA_ENA_MASK			0x00000100
#define    CFG_DMA_ENA_MASK			0x00000001
#define 	 LCD_SPU_SMPN_CTRL			0x0188
extern int    LCD_POWERON;
extern volatile u32 sleep_dma_flag;
extern atomic_t global_op_count ;
static void lcd_hx8347_init(struct pxa910fb_info *fbi)
{
	int num = ARRAY_SIZE(hx8347_reg);
	unsigned int reg, value;
	unsigned int high, low;
	int i = 0;
	u32 dma_flag;
       mutex_lock(&fbi->lcd_init);


	dma_flag = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
	/* disable all DMA to send commands */
	writel(dma_flag & (~(CFG_GRA_ENA_MASK | CFG_DMA_ENA_MASK)), fbi->reg_base + LCD_SPU_DMA_CTRL0);
	//writel(0x333305D1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
        writel(0x222204D3, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	while(num--)
	{
		high =( hx8347_reg[i].reg &0xFF00)<<2;	
		low =( hx8347_reg[i].reg &0xFF)<<1;		
		reg = high|low | 0x01000000;			
		
		high =( hx8347_reg[i].value &0xFF00)<<2;		
		low =( hx8347_reg[i].value &0xFF)<<1;		
		value = high|low | 0x81000000;		

		//gpio_set_value(cs_gpio, 0);
		udelay(1);
		pxa910fb_smart_cmd_simple(fbi, &reg, 1);/*write cmd*/
		udelay(1);

		pxa910fb_smart_cmd_simple(fbi, &value, 1);/*write data*/
		//gpio_set_value(cs_gpio, 1);
		udelay(1);
		if(hx8347_reg[i].msleep)
			msleep(hx8347_reg[i].msleep);
		i++;
	}
	
	//writel(0x333305C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	writel(0x222204C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	writel(dma_flag|sleep_dma_flag, fbi->reg_base + LCD_SPU_DMA_CTRL0);

       LCD_POWERON=1;
	fbi->active=1;
       mutex_unlock(&fbi->lcd_init);
}
void gpio_reconfgiure()
{
gpio_free(cs_gpio);
mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
}
static void lcd_hx8347_trans_dma(struct pxa910fb_info *fbi)
{
	unsigned int reg, value;
	unsigned int high, low;
	int i = 0;
	int delay = 0;
	u32 dma_flag;
	
	unsigned long flags;
	int num =8 /*ARRAY_SIZE(hx8347_reg_windows)*/;
	//static DEFINE_SPINLOCK(trigger_lock);
	
	mutex_lock(&fbi->lcd_init);
	dma_flag = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
	/* disable all DMA to send commands */
	writel(dma_flag & (~(CFG_GRA_ENA_MASK | CFG_DMA_ENA_MASK)), fbi->reg_base + LCD_SPU_DMA_CTRL0);
	//writel(0x222205D1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
        writel(0x222204D3, fbi->reg_base + LCD_SPU_SMPN_CTRL);  
	while(num--)
	{
		high =( hx8347_reg_windows[i].reg &0xFF00)<<2;	
		low =( hx8347_reg_windows[i].reg &0xFF)<<1;		
		reg = high|low | 0x01000000;			
		
		high =( hx8347_reg_windows[i].value &0xFF00)<<2;		
		low =( hx8347_reg_windows[i].value &0xFF)<<1;		
		value = high|low | 0x81000000;		

              	//gpio_set_value(cs_gpio, 0);//low
		//udelay(1);
		//pxa910fb_smart_cmd_simple(fbi, &reg, 1);/*write cmd*/
		writel(reg, fbi->reg_base + LCD_SPU_SLV_PORT);
		//udelay(1);

		//pxa910fb_smart_cmd_simple(fbi, &value, 1);/*write data*/
		writel(value, fbi->reg_base + LCD_SPU_SLV_PORT);
		//gpio_set_value(cs_gpio, 1);//low
		//udelay(1);
		for(delay=0;delay<10;delay++);
		//if(hx8347_reg_windows[i].msleep)
			//msleep(hx8347_reg_windows[i].msleep);
		i++;
	}
	udelay(1);
	//gpio_set_value(cs_gpio, 0);
	low = 0x22<<1;		
	reg = low | 0x01000000;		
	pxa910fb_smart_cmd_simple(fbi, &reg, 1);/*write cmd*/

	writel(0x222204C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	//writel(dma_flag/*| CFG_ARBFAST_ENA(1) | CFG_NOBLENDING(1)*/, fbi->reg_base + LCD_SPU_DMA_CTRL0);
     	writel((dma_flag |CFG_GRA_ENA_MASK ), fbi->reg_base + LCD_SPU_DMA_CTRL0);	

	smart_trigger_vsync(fbi);
	//mdelay(15);
	msleep(30);
	//gpio_set_value(cs_gpio, 1);
	//spin_unlock_irqrestore(&trigger_lock,flags );
        mutex_unlock(&fbi->lcd_init);
	
}

static void lcd_hx8347_standby(struct pxa910fb_info *fbi)
{
	int num = ARRAY_SIZE(hx8347_reg_standby);
	unsigned int reg, value;
	unsigned int high, low;
	int i = 0;

	u32 dma_flag;

	dma_flag = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
	/* disable all DMA to send commands */
	writel(dma_flag & (~(CFG_GRA_ENA_MASK | CFG_DMA_ENA_MASK)), fbi->reg_base + LCD_SPU_DMA_CTRL0);
	writel(0x333305D1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	while(num--)
	{
		high =( hx8347_reg_standby[i].reg &0xFF00)<<2;	
		low =( hx8347_reg_standby[i].reg &0xFF)<<1;		
		reg = high|low | 0x01000000;			
		
		high =( hx8347_reg_standby[i].value &0xFF00)<<2;		
		low =( hx8347_reg_standby[i].value &0xFF)<<1;		
		value = high|low | 0x81000000;		

		gpio_set_value(cs_gpio, 0);
		//udelay(1);
		pxa910fb_smart_cmd_simple(fbi, &reg, 1);/*write cmd*/
		//udelay(1);

		pxa910fb_smart_cmd_simple(fbi, &value, 1);/*write data*/
		gpio_set_value(cs_gpio, 1);
		//udelay(1);
		if(hx8347_reg_standby[i].msleep)
			msleep(hx8347_reg_standby[i].msleep);
		i++;
	}
	writel(0x333305C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	writel(dma_flag, fbi->reg_base + LCD_SPU_DMA_CTRL0);
}

static unsigned int is_hx8347_lcd(struct pxa910fb_info *fbi)
{	
	unsigned int reg, value;
	unsigned int high, low;
	
	u32 dma_flag;

	dma_flag = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
	/* disable all DMA to send commands */
	writel(dma_flag & (~(CFG_GRA_ENA_MASK | CFG_DMA_ENA_MASK)), fbi->reg_base + LCD_SPU_DMA_CTRL0);
	writel(0x555505D1, fbi->reg_base + LCD_SPU_SMPN_CTRL);	
	//gpio_set_value(cs_gpio, 0);//low
	reg = 0x01000000;							/*write cmd*/	
	pxa910fb_smart_cmd_simple(fbi, &reg, 1);	
	udelay(1);
	reg = 0x82000000;							/*read data*/	
	pxa910fb_smart_cmd_simple(fbi, &reg, 1);	
	udelay(1);
	value = readl(fbi->reg_base + LCD_SPU_ISA_RSDATA);
	high =0xFF00 & value>>2;
	low = 0xFF & value >>1;
	value = low | high;
	//gpio_set_value(cs_gpio, 1);//low
	writel(0x555505C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);	
	writel(dma_flag, fbi->reg_base + LCD_SPU_DMA_CTRL0);
	return value;
}

static int pxa910_setup_gpio(void)
{
	int err;
	
	cs_gpio = mfp_to_gpio(MFP_PIN_GPIO104);
	err = gpio_request(cs_gpio, "LCD CS");
	if (err) {
		printk(KERN_ERR"failed to request GPIO for TPO LCD CS\n");
		return -1;
	}
	gpio_direction_output(cs_gpio, 1);
	udelay(20);

	return 0;
}

static int hx8347_lcd_power(struct pxa910fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int err = 0;
	mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
	int static poweron=0;
	printk("Enter %s,on=%dd\n", __FUNCTION__,on);
	
	//pxa910_setup_gpio();
	writel(0x40000008, fbi->reg_base + LCD_CFG_SCLK_DIV);
	if (on) 
	{
		if (spi_gpio_reset != -1) {
			err = gpio_request(spi_gpio_reset, "TPO_LCD_RESET");
			if (err) {
				printk("failed to request GPIO for TPO LCD RESET\n");
				return 1;
			}
			gpio_direction_output(spi_gpio_reset, 1);
			msleep(10);
			//gpio_set_value(spi_gpio_reset, 1);
			//msleep(10);
			gpio_set_value(spi_gpio_reset, 0);
			udelay(20);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(200);
			gpio_free(spi_gpio_reset);
                 /*
			gpio_direction_output(spi_gpio_reset, 1);
			msleep(10);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(10);
			gpio_set_value(spi_gpio_reset, 0);
			msleep(60);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(100);
			gpio_free(spi_gpio_reset);   */
		}
		#if  0
                if(LCD_HX8347_ID !=is_hx8347_lcd(fbi))
			printk("LCD9325 Init fault!!!!!!!!\n");
		else
			printk("LCD9325 Init success!!!!!!!!\n");
		#endif
		lcd_hx8347_init(fbi);
		lcd_power_on(1, 50);
	} else {

		lcd_power_on(0, 0);	
		mutex_lock(&fbi->lcd_init);
		LCD_POWERON=0;
		sleep_dma_flag = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
		writel(sleep_dma_flag & (~(CFG_GRA_ENA_MASK | CFG_DMA_ENA_MASK)), fbi->reg_base + LCD_SPU_DMA_CTRL0);
		mutex_unlock(&fbi->lcd_init);
		err = gpio_request(spi_gpio_reset, "TPO_LCD_RESET");
		if (err) {
			printk("failed to request LCD RESET gpio\n");
			return 1;
		}
		gpio_set_value(spi_gpio_reset, 0);
		gpio_free(spi_gpio_reset);
		
              
	}

	return 0;
}

#define LCD_SCLK (312000000UL)

static struct fb_videomode hx8347_video_modes[] = {
	/* hx8347 HVGA mode info */
	[0] = {
		.pixclock       = 152070,
		.refresh        = 60,
		.xres           = 240,
		.yres           = 320,
		.hsync_len      = 5,
		.left_margin    = 4,
		.right_margin   = 4,
		.vsync_len      = 2,
		.upper_margin   = 2,
		.lower_margin   = 2,
		.sync           = 0,
	},
};

static struct pxa910fb_mach_info u810_smart_lcd_info = {
        .id                     = "Base",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = ARRAY_SIZE(hx8347_video_modes),
        .modes          	= hx8347_video_modes,
        .pix_fmt                = PIX_FMT_RGB565/*PIX_FMT_RGB565*/,
        .io_pin_allocation_mode = PIN_MODE_SMART_18_SPI,
        .dumb_mode              = DUMB_MODE_RGB666,
        .panel_rgb_reverse_lanes= 0,
        .gpio_output_data       = 1,
        .gpio_output_mask       = 0xff,
        .invert_composite_blank = 0,
        .invert_pix_val_ena     = 0,
        .invert_pixclock        = 0,
        .invert_vsync           = 0,
        .invert_hsync           = 0,
        .panel_rbswap		= 1,
        .active                 = 1,
        .enable_lcd             = 1,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset	= mfp_to_gpio(MFP_PIN_GPIO106),
	.max_fb_size		= 640 * 960 * 4,
	.pxa910fb_lcd_power	= hx8347_lcd_power,
	.display_interface	= SMART_PANEL,		
	.lcd_xxx_trans_dma= lcd_hx8347_trans_dma,
};

static struct pxa910fb_mach_info u810_smart_lcd_ovly_info = {
        .id                     = "Ovly",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = ARRAY_SIZE(hx8347_video_modes),
        .modes                  = hx8347_video_modes,
        .pix_fmt                = PIX_FMT_RGB565,
        .dumb_mode              = DUMB_MODE_RGB666,
        .panel_rgb_reverse_lanes= 0,
        .panel_rbswap		= 1,
        .enable_lcd             = 1,
        .spi_gpio_cs            = -1,
	.spi_gpio_reset		= mfp_to_gpio(MFP_PIN_GPIO106),
	.max_fb_size		= 1280 * 720 * 4,
	.pxa910fb_lcd_power	= hx8347_lcd_power,
	.display_interface	= SMART_PANEL,
};

void u810_add_lcd_hx8347(void)
{
	struct pxa910fb_mach_info *fb = &u810_smart_lcd_info, *ovly = &u810_smart_lcd_ovly_info;
	//pxa910_setup_gpio();
	pxa910_add_fb(fb);
	pxa910_add_fb_ovly(ovly);
} 
