/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */
/* ========================================================================================
when         who        what, where, why                                  comment tag
--------     ----       -----------------------------                --------------------------
==========================================================================================*/

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
#include <mach/pxa910fb.h>
static void (*spi_send)(struct pxa910fb_info *, void *, int , unsigned int );
 void u880_add_lcd_HX8369(void);
 unsigned int IS_HX8369_LCD(void);
static unsigned long tpo_lcd_gpio_pin_config[] = {
	GPIO81_LCD_FCLK,		
	GPIO82_LCD_LCLK,		
	GPIO83_LCD_PCLK,		
	GPIO84_LCD_DENA,		
	GPIO85_LCD_DD0,		
	GPIO86_LCD_DD1,	
	GPIO87_LCD_DD2,	
	GPIO88_LCD_DD3,		
	GPIO89_LCD_DD4,		
	GPIO90_LCD_DD5	,	
	GPIO91_LCD_DD6,		
	GPIO92_LCD_DD7,		
	GPIO93_LCD_DD8,		
	GPIO94_LCD_DD9,		
	GPIO95_LCD_DD10	,
	GPIO96_LCD_DD11,
	GPIO97_LCD_DD12	,
	GPIO98_LCD_DD13	,
	GPIO100_LCD_DD14,	
	GPIO101_LCD_DD15,	
	GPIO102_LCD_DD16,
	GPIO103_LCD_DD17,	
	GPIO104_GPIO104,  // Data out
	GPIO105_GPIO105,  // Data in
	GPIO106_GPIO106,  // Reset
	GPIO107_GPIO107,  // CS
	GPIO108_GPIO108,  // SCLK
};

extern void lcd_power_on(int on, int ms);
static int clk_gpio;
static int d_gpio;
static int d_gpio_in;    
static int cs_gpio;

static unsigned char  SPI_ReadData(void);
#define LCD_DELAYT_TIME  (10)

static void SPI_WriteReg(unsigned char SPI_COMMD)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	unsigned long flags;
	static DEFINE_SPINLOCK(trigger_lock);
	
	SBuffer=SPI_COMMD;

	spin_lock_irqsave(&trigger_lock,flags );
	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<9;BitCounter++)
	{
		SBit = SBuffer&0x0100;
		gpio_direction_output(clk_gpio, 0);
		if(SBit)
			gpio_direction_output(d_gpio, 1);
		else
			gpio_direction_output(d_gpio, 0);
		udelay(LCD_DELAYT_TIME);
		gpio_direction_output(clk_gpio, 1);
		udelay(LCD_DELAYT_TIME);
		SBuffer = SBuffer<<1;
	}
	//gpio_direction_output(cs_gpio, 1);
	spin_unlock_irqrestore(&trigger_lock,flags );
}

static void SPI_WriteRegWithOutos(unsigned char SPI_COMMD)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	unsigned long flags;
	//static DEFINE_SPINLOCK(trigger_lock);
	
	SBuffer=SPI_COMMD;

	//spin_lock_irqsave(&trigger_lock,flags );
	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<9;BitCounter++)
	{
		SBit = SBuffer&0x0100;
		gpio_direction_output(clk_gpio, 0);
		if(SBit)
			gpio_direction_output(d_gpio, 1);
		else
			gpio_direction_output(d_gpio, 0);
		udelay(LCD_DELAYT_TIME);
		gpio_direction_output(clk_gpio, 1);
		udelay(LCD_DELAYT_TIME);
		SBuffer = SBuffer<<1;
	}
	//gpio_direction_output(cs_gpio, 1);
	//spin_unlock_irqrestore(&trigger_lock,flags );
}
//***********************************************
//***********************************************
static void SPI_WriteData(unsigned char SPI_DATA)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	unsigned long flags;
	static DEFINE_SPINLOCK(trigger_lock);
	
	SBuffer = SPI_DATA | 0x0100;
	
	spin_lock_irqsave(&trigger_lock,flags );
	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<9;BitCounter++)
	{
		SBit = SBuffer&0x0100;
		gpio_direction_output(clk_gpio, 0);
		if(SBit)
			gpio_direction_output(d_gpio, 1);
		else
			gpio_direction_output(d_gpio, 0);
		udelay(LCD_DELAYT_TIME);
		gpio_direction_output(clk_gpio, 1);
	      udelay(LCD_DELAYT_TIME);
		SBuffer = SBuffer<<1;
	}
	//gpio_direction_output(cs_gpio, 1);
	spin_unlock_irqrestore(&trigger_lock,flags );//UNLOCK
}

static void SPI_WriteDataWithOutos(unsigned char SPI_DATA)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	unsigned long flags;
	//static DEFINE_SPINLOCK(trigger_lock);
	
	SBuffer = SPI_DATA | 0x0100;
	
	//spin_lock_irqsave(&trigger_lock,flags );
	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<9;BitCounter++)
	{
		SBit = SBuffer&0x0100;
		gpio_direction_output(clk_gpio, 0);
		if(SBit)
			gpio_direction_output(d_gpio, 1);
		else
			gpio_direction_output(d_gpio, 0);
		udelay(LCD_DELAYT_TIME);
		gpio_direction_output(clk_gpio, 1);
	      udelay(LCD_DELAYT_TIME);
		SBuffer = SBuffer<<1;
	}
	//gpio_direction_output(cs_gpio, 1);
	//spin_unlock_irqrestore(&trigger_lock,flags );//UNLOCK
}
static unsigned char  SPI_ReadData(void)
{
	unsigned short SBit=0;
	unsigned char BitCounter=0;
	unsigned char SpiData=0;

	gpio_direction_input(d_gpio_in);
	gpio_direction_output(cs_gpio, 0);
	gpio_direction_output(clk_gpio, 1);
	for(BitCounter = 0; BitCounter < 8; BitCounter++)
	{
		gpio_direction_output(clk_gpio, 0);
		udelay(20);
		SBit = gpio_get_value(d_gpio_in);
		gpio_direction_output(clk_gpio, 1);
		udelay(20);
		if(0 != SBit)
		{
		SpiData|= (1 << (7 - BitCounter));
		}
	}
	//gpio_direction_output(cs_gpio, 1);
	return SpiData;
	
}
#if  1
static void lcd_panel_sleep(void)
{
	
	
       SPI_WriteReg(0x10);
	msleep(200);
       SPI_WriteReg(0x28);
}
	
static void lcd_panel_wakeup(void)
{

       SPI_WriteReg(0x11);
	msleep(200);
       SPI_WriteReg(0x29);
}
#endif	
static void Xinli_Hx8369_hydis_init(void)
{	

       printk("Enter %s\n", __FUNCTION__);
        SPI_WriteReg(0xB9);  // SET password
        SPI_WriteData(0xFF); 
        SPI_WriteData(0x83); 
        SPI_WriteData(0x69); 
 
        SPI_WriteReg(0xB1);  //Set Power 
        SPI_WriteData(0x85);
        SPI_WriteData(0x00);
        SPI_WriteData(0x34);
        SPI_WriteData(0x07);
        SPI_WriteData(0x00);
        SPI_WriteData(0x0F);
        SPI_WriteData(0x0F);
        #if   0/*0614*/
        SPI_WriteData(0x2A);
        SPI_WriteData(0x32);
        SPI_WriteData(0x3F);
        SPI_WriteData(0x3F);
        #else
        SPI_WriteData(0x28);
        SPI_WriteData(0x30);
        SPI_WriteData(0x3F);
        SPI_WriteData(0x3F);
        #endif
        SPI_WriteData(0x01); //update VBIAS
        SPI_WriteData(0x3A);
        SPI_WriteData(0x01);
        SPI_WriteData(0xE6);
        SPI_WriteData(0xE6);
        SPI_WriteData(0xE6);
        SPI_WriteData(0xE6);
        SPI_WriteData(0xE6);
 
 
 
        SPI_WriteReg(0xB2);  // SET Display  480x800
        SPI_WriteData(0x00); 
        SPI_WriteData(0x23); 
        SPI_WriteData(0x03); 
        SPI_WriteData(0x03); 
        SPI_WriteData(0x70); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0xFF); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x03); 
        SPI_WriteData(0x03); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x01); 


        SPI_WriteReg(0xB4);  // SET Display  480x800
        SPI_WriteData(0x00); 
        SPI_WriteData(0x18); 
        SPI_WriteData(0x80);
        SPI_WriteData(0x06); 
        SPI_WriteData(0x02); 
 
 
 
        SPI_WriteReg(0xB6);  // SET VCOM
        SPI_WriteData(0x42);  // Update VCOM
        SPI_WriteData(0x42); 
 
 
 
        SPI_WriteReg(0xD5); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x03); //04
        SPI_WriteData(0x03); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x01); 
        SPI_WriteData(0x04); //05
        SPI_WriteData(0x28); 
        SPI_WriteData(0x70); 
        SPI_WriteData(0x11); //01
        SPI_WriteData(0x13); //03
        SPI_WriteData(0x00); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x40); 
        SPI_WriteData(0x06); 
        SPI_WriteData(0x51); 
        SPI_WriteData(0x07); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x00); 
        SPI_WriteData(0x41); 
        SPI_WriteData(0x06); 
        SPI_WriteData(0x50); 
        SPI_WriteData(0x07); 
        SPI_WriteData(0x07); 
        SPI_WriteData(0x0F); 
        SPI_WriteData(0x04); 
        SPI_WriteData(0x00); 
 
 
 
        SPI_WriteReg(0xE0);
        SPI_WriteData(0x00);
        SPI_WriteData(0x13);
        SPI_WriteData(0x19);
        SPI_WriteData(0x38);
        SPI_WriteData(0x3D);
        SPI_WriteData(0x3F);
        SPI_WriteData(0x28);
        SPI_WriteData(0x46);
        SPI_WriteData(0x07);
        SPI_WriteData(0x0D);
        SPI_WriteData(0x0E);
        SPI_WriteData(0x12);
        SPI_WriteData(0x15);
        SPI_WriteData(0x12);
        SPI_WriteData(0x14);
        SPI_WriteData(0x0F);
        SPI_WriteData(0x17);
        SPI_WriteData(0x00);
        SPI_WriteData(0x13);
        SPI_WriteData(0x19);
        SPI_WriteData(0x38);
        SPI_WriteData(0x3D);
        SPI_WriteData(0x3F);
        SPI_WriteData(0x28);
        SPI_WriteData(0x46);
        SPI_WriteData(0x07);
        SPI_WriteData(0x0D);
        SPI_WriteData(0x0E);
        SPI_WriteData(0x12);
        SPI_WriteData(0x15);
        SPI_WriteData(0x12);
        SPI_WriteData(0x14);
        SPI_WriteData(0x0F);
        SPI_WriteData(0x17);
 

         msleep(10);

 
        SPI_WriteReg(0xC1); 
        SPI_WriteData(0x01); 
//R 
        SPI_WriteData(0x04); 
        SPI_WriteData(0x0a); 
        SPI_WriteData(0x13); 
        SPI_WriteData(0x1A); 
        SPI_WriteData(0x21); 
        SPI_WriteData(0x29); 
        SPI_WriteData(0x31); 
        SPI_WriteData(0x37); 
        SPI_WriteData(0x3E); 
        SPI_WriteData(0x47); 
        SPI_WriteData(0x4F); 
        SPI_WriteData(0x56); 
        SPI_WriteData(0x5E); 
        SPI_WriteData(0x65); 
        SPI_WriteData(0x6E); 
        SPI_WriteData(0x78); 
        SPI_WriteData(0x80); 
        SPI_WriteData(0x88); 
        SPI_WriteData(0x8F); 
        SPI_WriteData(0x98); 
        SPI_WriteData(0xA0); 
        SPI_WriteData(0xA8); 
        SPI_WriteData(0xB1); 
        SPI_WriteData(0xBA); 
        SPI_WriteData(0xC3); 
        SPI_WriteData(0xCB); 
        SPI_WriteData(0xD3); 
        SPI_WriteData(0xDB); 
        SPI_WriteData(0xE4); 
        SPI_WriteData(0xEB); 
        SPI_WriteData(0xF3); 
        SPI_WriteData(0xFA); 
        SPI_WriteData(0xFF); 
        SPI_WriteData(0x20);
        SPI_WriteData(0x35); 
        SPI_WriteData(0xE6); 
        SPI_WriteData(0x2D); 
        SPI_WriteData(0x8C); 
        SPI_WriteData(0x29); 
        SPI_WriteData(0xE3); 
        SPI_WriteData(0x2F); 
        SPI_WriteData(0xC0); 
//G 
       SPI_WriteData(0x04); 
        SPI_WriteData(0x0a); 
        SPI_WriteData(0x13); 
        SPI_WriteData(0x1A); 
        SPI_WriteData(0x21); 
        SPI_WriteData(0x29); 
        SPI_WriteData(0x31); 
        SPI_WriteData(0x37); 
        SPI_WriteData(0x3E); 
        SPI_WriteData(0x47); 
        SPI_WriteData(0x4F); 
        SPI_WriteData(0x56); 
        SPI_WriteData(0x5E); 
        SPI_WriteData(0x65); 
        SPI_WriteData(0x6E); 
        SPI_WriteData(0x78); 
        SPI_WriteData(0x80); 
        SPI_WriteData(0x88); 
        SPI_WriteData(0x8F); 
        SPI_WriteData(0x98); 
        SPI_WriteData(0xA0); 
        SPI_WriteData(0xA8); 
        SPI_WriteData(0xB1); 
        SPI_WriteData(0xBA); 
        SPI_WriteData(0xC3); 
        SPI_WriteData(0xCB); 
        SPI_WriteData(0xD3); 
        SPI_WriteData(0xDB); 
        SPI_WriteData(0xE4); 
        SPI_WriteData(0xEB); 
        SPI_WriteData(0xF3); 
        SPI_WriteData(0xFA); 
        SPI_WriteData(0xFF); 
        SPI_WriteData(0x20);
        SPI_WriteData(0x35); 
        SPI_WriteData(0xE6); 
        SPI_WriteData(0x2D); 
        SPI_WriteData(0x8C); 
        SPI_WriteData(0x29); 
        SPI_WriteData(0xE3); 
        SPI_WriteData(0x2F); 
        SPI_WriteData(0xC0); 
//B 
       SPI_WriteData(0x04); 
        SPI_WriteData(0x0a); 
        SPI_WriteData(0x13); 
        SPI_WriteData(0x1A); 
        SPI_WriteData(0x21); 
        SPI_WriteData(0x29); 
        SPI_WriteData(0x31); 
        SPI_WriteData(0x37); 
        SPI_WriteData(0x3E); 
        SPI_WriteData(0x47); 
        SPI_WriteData(0x4F); 
        SPI_WriteData(0x56); 
        SPI_WriteData(0x5E); 
        SPI_WriteData(0x65); 
        SPI_WriteData(0x6E); 
        SPI_WriteData(0x78); 
        SPI_WriteData(0x80); 
        SPI_WriteData(0x88); 
        SPI_WriteData(0x8F); 
        SPI_WriteData(0x98); 
        SPI_WriteData(0xA0); 
        SPI_WriteData(0xA8); 
        SPI_WriteData(0xB1); 
        SPI_WriteData(0xBA); 
        SPI_WriteData(0xC3); 
        SPI_WriteData(0xCB); 
        SPI_WriteData(0xD3); 
        SPI_WriteData(0xDB); 
        SPI_WriteData(0xE4); 
        SPI_WriteData(0xEB); 
        SPI_WriteData(0xF3); 
        SPI_WriteData(0xFA); 
        SPI_WriteData(0xFF); 
        SPI_WriteData(0x20);
        SPI_WriteData(0x35); 
        SPI_WriteData(0xE6); 
        SPI_WriteData(0x2D); 
        SPI_WriteData(0x8C); 
        SPI_WriteData(0x29); 
        SPI_WriteData(0xE3); 
        SPI_WriteData(0x2F); 
        SPI_WriteData(0xC0); 
 

        msleep(10);
#if  1
        SPI_WriteReg(0xB3); //
	SPI_WriteData(0x07); //
	
        SPI_WriteReg(0x3A); 
        SPI_WriteData(0x60); 
 #endif
        SPI_WriteReg(0xcc); 
        SPI_WriteData(0x0A); 
 
        SPI_WriteReg(0x11);  
        msleep(120);
		
        SPI_WriteReg(0x29); 
	 SPI_WriteReg(0x2C);
	gpio_direction_output(cs_gpio, 1);
	printk("Exit %s\n", __FUNCTION__);
}

static int lcd_panel_gpio_spi_init(void)
{
	int err;
	printk("Enter %s,\n", __FUNCTION__);
	cs_gpio = mfp_to_gpio(MFP_PIN_GPIO107);
	err = gpio_request(cs_gpio, "LCD CS");
	if (err) {
	printk("failed to request GPIO for TPO LCD CS\n");
	return -1;
	}
	
	clk_gpio = mfp_to_gpio(MFP_PIN_GPIO108);
	err = gpio_request(clk_gpio, "LCD CLK");
	if (err) {
	printk("failed to request GPIO for TPO LCD CLK\n");
	return -1;
	}

	d_gpio = mfp_to_gpio(MFP_PIN_GPIO104);
	err = gpio_request(d_gpio, "LCD data");
	if (err) {
	printk("failed to request GPIO for TPO LCD Data\n");
	return -1;
	}

	d_gpio_in = mfp_to_gpio(MFP_PIN_GPIO105);
	err = gpio_request(d_gpio_in, "LCD data in");
	if (err) {
	printk("failed to request GPIO for LCD Data in\n");
	return -1;
	}

	return 0;
}
static int Hx8369_lcd_Hydis_power(struct pxa910fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{

       #if   1
	int err = 0;
	printk("Enter %s ,spi_gpio_reset=%d,on=%d\n", __FUNCTION__,spi_gpio_reset,on);

	mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
	lcd_panel_gpio_spi_init();
     
	if (on)
	{
             		if (spi_gpio_reset != -1) {
			 err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
			if (err) {
				printk("failed to request GPIO for TPO LCD RESET\n");
				return 1;
			}
			gpio_direction_output(spi_gpio_reset, 0);
			msleep(30);
			gpio_direction_output(spi_gpio_reset, 1);
			msleep(100);
			gpio_free(spi_gpio_reset);
		   }
		// IS_HX8369_LCD();
               //lcd_panel_wakeup();
               Xinli_Hx8369_hydis_init();
	       //msleep(100);
	       lcd_power_on(1, 50);
	   //gpio_direction_output(bl_gpio, 1);
	}
	else
	{
	lcd_power_on(0, 0);
	//gpio_direction_output(bl_gpio, 0);
	//lcd_panel_sleep();
			err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
			if (err) {
			printk("failed to request LCD RESET gpio\n");
			return 1;
			}
		gpio_direction_output(spi_gpio_reset, 0);
		gpio_free(spi_gpio_reset);
		

	}
	
	gpio_direction_output(cs_gpio, 0);
	gpio_direction_output(clk_gpio, 0);
	gpio_direction_output(d_gpio, 0);
	gpio_free(d_gpio_in);
	gpio_free(cs_gpio);
	gpio_free(clk_gpio);
	gpio_free(d_gpio);

	printk("Exit %s ,spi_gpio_reset=%d,on=%d\n", __FUNCTION__,spi_gpio_reset,on);
	return 0;
      #else
	int err = 0;
	printk("Enter %s ,spi_gpio_reset=%d,on=%d\n", __FUNCTION__,spi_gpio_reset,on);

	mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
	lcd_panel_gpio_spi_init();
     
	if (on)
	{
             lcd_panel_wakeup();
	       lcd_power_on(1, 50);
	}
	else
	{
	lcd_power_on(0, 0);
	lcd_panel_sleep();		
	}
	
	gpio_direction_output(cs_gpio, 0);
	gpio_direction_output(clk_gpio, 0);
	gpio_direction_output(d_gpio, 0);
	gpio_free(d_gpio_in);
	gpio_free(cs_gpio);
	gpio_free(clk_gpio);
	gpio_free(d_gpio);

	printk("Exit %s ,spi_gpio_reset=%d,on=%d\n", __FUNCTION__,spi_gpio_reset,on);
	return 0;
#endif
}

static struct fb_videomode LeadHx8369_video_modes[] = {
	/* Lead HVGA mode info */
        #if   0
	[0] = {
		.pixclock       = 41216,
		.refresh        = 60,
		.xres           = 480,
		.yres           = 800,
		.hsync_len      = 3*2,
		.left_margin    = 3*2,
		.right_margin   = 3*2,
		.vsync_len      = 2*2,
		.upper_margin   = 2*2,
		.lower_margin   = 2*2,
		.sync           = 0,
	},
        #endif
	#if   1
	[0] = {
		.pixclock       = 40345,
		.refresh        = 60,
		.xres           = 480,
		.yres           = 800,
		.hsync_len      = 10,///3*2,
		.left_margin    = 10,///3*2,
		.right_margin   =10, ///3*2,
		.vsync_len      = 4,///2*2,
		.upper_margin   =3, ///2*2,
		.lower_margin   =3, ///2*2,
		.sync           = 0,
	},
	#endif
};

/* SPI Control Register. */
#define     CFG_SCLKCNT(div)                    (div<<24)  /* 0xFF~0x2 */
#define     CFG_RXBITS(rx)                      ((rx - 1)<<16)   /* 0x1F~0x1 */
#define     CFG_TXBITS(tx)                      ((tx - 1)<<8)    /* 0x1F~0x1, 0x1: 2bits ... 0x1F: 32bits */
#define     CFG_SPI_ENA(spi)                    (spi<<3)
#define     CFG_SPI_SEL(spi)                    (spi<<2)   /* 1: port1; 0: port0 */
#define     CFG_SPI_3W4WB(wire)                 (wire<<1)  /* 1: 3-wire; 0: 4-wire */

static struct pxa910fb_mach_info u880_lcd_info __initdata = {
	.id                     = "Base",
	.modes                  = LeadHx8369_video_modes,
	.num_modes              = ARRAY_SIZE(LeadHx8369_video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
	.spi_ctrl               = CFG_SCLKCNT(16) | CFG_TXBITS(9) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(0) | CFG_SPI_ENA(1),
	.spi_gpio_cs            = -1,
	.spi_gpio_reset		= mfp_to_gpio(MFP_PIN_GPIO106),
	.panel_rbswap		= 0,
	.pxa910fb_lcd_power	= Hx8369_lcd_Hydis_power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

static struct pxa910fb_mach_info u880_lcd_ovly_info __initdata = {
	.id                     = "Ovly",
	.modes                  = LeadHx8369_video_modes,
	.num_modes              = ARRAY_SIZE(LeadHx8369_video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
        .spi_ctrl               = CFG_SCLKCNT(16) | CFG_TXBITS(9) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(0) | CFG_SPI_ENA(1),
        .spi_gpio_cs            = -1,
        .spi_gpio_reset         = mfp_to_gpio(MFP_PIN_GPIO106),
	.panel_rbswap		= 1,
        .pxa910fb_lcd_power     =  Hx8369_lcd_Hydis_power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

 void u880_add_lcd_HX8369_HYIDIS(void)
{
	struct pxa910fb_mach_info *fb = &u880_lcd_info, *ovly = &u880_lcd_ovly_info;
	spi_send = pxa910fb_spi_send;
	pxa910_add_fb(fb);
	pxa910_add_fb_ovly(ovly);
}

unsigned int IS_HX8369_LCD_HYIDIS(void)
	{
	#if  0
	 unsigned long int ID=0;
	 unsigned int RegDataL=0;
	 unsigned int RegDataH=0;
	 unsigned int RegDataM=0;
	 #if  0
	SPI_WriteRegWithOutos(0xB9);   //Set_EXTC
	SPI_WriteDataWithOutos(0xFF);          //
	SPI_WriteDataWithOutos(0x83);          //
	SPI_WriteDataWithOutos(0x63);          //

	SPI_WriteRegWithOutos(0xC3);   
	SPI_WriteDataWithOutos(0x83);          
	SPI_WriteDataWithOutos(0x88);          
	SPI_WriteDataWithOutos(0x63);        
	
	//msleep(200);
	mdelay(120);
	#endif
	//SPI_WriteRegWithOutos(0xDA);
	//RegDataL=SPI_ReadData();
	SPI_WriteRegWithOutos(0xDB);
	RegDataM=SPI_ReadData();
	//SPI_WriteRegWithOutos(0xDC);
	RegDataH=SPI_ReadData();
       gpio_direction_output(cs_gpio, 1);
	ID=(RegDataL<<8)|RegDataH;
	printk("Enter %s,0x%lx,0x%lx\n", __FUNCTION__,RegDataM,RegDataH);
	if(0x8363==ID)
	{
	return 1;
	}
	else
	{
	return 1;
	}
	#else
	u32     isr;

	//isr = readl(0xFE20B000 + 0xf8);
	void * reg_base;
	reg_base = ioremap_nocache(0xd420b000, 0x1ec);
	isr = readl(reg_base + 0xf8);
       iounmap(reg_base);
	printk("Enter %s,ID=0x%x\n", __FUNCTION__,isr);
	if(isr==0x5A5A5A03)
	{
           return 1;
	}
	else
	{
	    return 0;
	}
	#endif
}

