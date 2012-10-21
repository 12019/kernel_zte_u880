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


static struct pxa910fb_info *g_fbi;
static unsigned int g_spi_gpio_cs;
static unsigned int g_spi_gpio_reset;
static int g_on;

static struct work_struct lcd_poweron_wq;

static void lcd_power_do_work(struct work_struct *work);

extern void lcd_power_on(int on, int ms);

static int clk_gpio;
static int d_gpio;
static int d_gpio_in;
static int cs_gpio;
#define  LCD_DELAY_TIME   (10)
static void LCDSPI_InitCMD(unsigned char SPI_COMMD)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	
	SBuffer=SPI_COMMD;
	gpio_direction_output(cs_gpio, 0);	//Set_CS(0); //CLR CS
	udelay(LCD_DELAY_TIME);
	for(BitCounter=0;BitCounter<9;BitCounter++)
	{
		SBit = SBuffer&0x100;
		if(SBit)
			gpio_direction_output(d_gpio, 1);//Set_SDA(1);
		else
			gpio_direction_output(d_gpio, 0);//Set_SDA(0);
			
		udelay(LCD_DELAY_TIME);
		gpio_direction_output(clk_gpio, 0);//Set_SCK(0); //CLR SCL
		udelay(LCD_DELAY_TIME);
		gpio_direction_output(clk_gpio, 1);//Set_SCK(1); //SET SCL
		udelay(LCD_DELAY_TIME);
		SBuffer = SBuffer<<1;
	}
	gpio_direction_output(cs_gpio, 1);//Set_CS(1); //SET CS
}
//***********************************************
//***********************************************
static void LCDSPI_InitDAT(unsigned char SPI_DATA)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	
	SBuffer=SPI_DATA | 0x100;
	gpio_direction_output(cs_gpio, 0);//Set_CS(0); //CLR CS
	udelay(LCD_DELAY_TIME);
	for(BitCounter=0;BitCounter<9;BitCounter++)
	{
		SBit = SBuffer&0x100;
		if(SBit)
			gpio_direction_output(d_gpio, 1);//Set_SDA(1);
		else
			gpio_direction_output(d_gpio, 0);//Set_SDA(0);
			
		udelay(LCD_DELAY_TIME);
		gpio_direction_output(clk_gpio, 0);//Set_SCK(0); //CLR SCL
		udelay(LCD_DELAY_TIME);
		gpio_direction_output(clk_gpio, 1);//Set_SCK(1); //SET SCL
		udelay(LCD_DELAY_TIME);
		SBuffer = SBuffer<<1;
	}
	gpio_direction_output(cs_gpio, 1);//Set_CS(1); //SET CS
}

static void lcd_panel_sleep(void)
{
	LCDSPI_InitDAT(0x28);
	msleep(20);  
	LCDSPI_InitDAT(0x10);
	msleep(100); 
}

static void lcd_panel_wakeup(void)
{
	LCDSPI_InitDAT(0x11);
	msleep(120);  
	LCDSPI_InitDAT(0x29);	
	msleep(40);  
}

static void lcd_xinli_init(void)
{
LCDSPI_InitCMD(0x11);
msleep(150);
LCDSPI_InitCMD(0xB0);//{setc, [107], W, 0x000B0}
LCDSPI_InitDAT(0x00);//{setp, [104], W, 0x00000}

LCDSPI_InitCMD(0xB3);
LCDSPI_InitDAT(0x02);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x00);

LCDSPI_InitCMD(0xB4);
LCDSPI_InitDAT(0x00);

LCDSPI_InitCMD(0xC0);
//LCDSPI_InitDAT(0x12);
LCDSPI_InitDAT(0x16);
LCDSPI_InitDAT(0x3B);//480
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x02);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x01);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x54);

LCDSPI_InitCMD(0xC1);
LCDSPI_InitDAT(0x08);
LCDSPI_InitDAT(0x12);//CLOCK
LCDSPI_InitDAT(0x08);
LCDSPI_InitDAT(0x08);

LCDSPI_InitCMD(0xC4);
LCDSPI_InitDAT(0x11);
LCDSPI_InitDAT(0x07);
LCDSPI_InitDAT(0x03);
LCDSPI_InitDAT(0x03);

LCDSPI_InitCMD(0xC6);
LCDSPI_InitDAT(0x1A);


LCDSPI_InitCMD(0xC8);//GAMMA
LCDSPI_InitDAT(0x04);
LCDSPI_InitDAT(0x0C);
LCDSPI_InitDAT(0x0A);
LCDSPI_InitDAT(0xfC);
LCDSPI_InitDAT(0x06);
LCDSPI_InitDAT(0x08);
LCDSPI_InitDAT(0x0f);
LCDSPI_InitDAT(0x07);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x32);

LCDSPI_InitDAT(0x07);
LCDSPI_InitDAT(0x0f);
LCDSPI_InitDAT(0x08);
LCDSPI_InitDAT(0xf6);//43/55
LCDSPI_InitDAT(0x0C);
LCDSPI_InitDAT(0x0A);
LCDSPI_InitDAT(0x0C);
LCDSPI_InitDAT(0x04);

LCDSPI_InitDAT(0x32);
LCDSPI_InitDAT(0x00);

LCDSPI_InitCMD(0x2A);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x01);
LCDSPI_InitDAT(0x3F);//320

LCDSPI_InitCMD(0x2B);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x01);
LCDSPI_InitDAT(0xDF);//480

LCDSPI_InitCMD(0x35);
LCDSPI_InitDAT(0x00);

LCDSPI_InitCMD(0x36);
//LCDSPI_InitDAT(0x80);
LCDSPI_InitDAT(0x00);

LCDSPI_InitCMD(0x3A);
LCDSPI_InitDAT(0x66);

LCDSPI_InitCMD(0x44);
LCDSPI_InitDAT(0x00);
LCDSPI_InitDAT(0x01);

msleep(100);


LCDSPI_InitCMD(0xD0);
LCDSPI_InitDAT(0x07);
LCDSPI_InitDAT(0x07);
LCDSPI_InitDAT(0x1E);
LCDSPI_InitDAT(0x33);


LCDSPI_InitCMD(0xD1);
LCDSPI_InitDAT(0x03);
LCDSPI_InitDAT(0x3C);//VCM40
LCDSPI_InitDAT(0x14);//VDV

LCDSPI_InitCMD(0xD2);
LCDSPI_InitDAT(0x03);
LCDSPI_InitDAT(0x04);//0X24
LCDSPI_InitDAT(0x04);
LCDSPI_InitCMD(0x29);
mdelay(10);  

LCDSPI_InitCMD(0xB4);
LCDSPI_InitDAT(0x11);
msleep(120);  
LCDSPI_InitCMD(0x2C);
mdelay(20);  
}

static int lcd_panel_gpio_spi_init(void)
{
    int err;
	
	cs_gpio = mfp_to_gpio(MFP_PIN_GPIO107);
	err = gpio_request(cs_gpio, "LCD CS");
	if (err) {
		printk(KERN_ERR"failed to request GPIO for TPO LCD CS\n");
		return -1;
	}
	gpio_direction_output(cs_gpio, 1);
	udelay(20);

	clk_gpio = mfp_to_gpio(MFP_PIN_GPIO108);
	err = gpio_request(clk_gpio, "LCD CLK");
	if (err) {
		printk(KERN_ERR"failed to request GPIO for TPO LCD CLK\n");
		return -1;
	}

	d_gpio = mfp_to_gpio(MFP_PIN_GPIO104);
	err = gpio_request(d_gpio, "LCD data");
	if (err) {
		printk(KERN_ERR"failed to request GPIO for TPO LCD Data\n");
		return -1;
	}

    d_gpio_in = mfp_to_gpio(MFP_PIN_GPIO105);
	err = gpio_request(d_gpio_in, "LCD data in");
	if (err) {
		printk(KERN_ERR"failed to request GPIO for TPO LCD Data\n");
		return -1;
	}
	return 0;
}

static void xinli_lcd_power_on(struct pxa910fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int err = 0;

	mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
	lcd_panel_gpio_spi_init();
	if (on)
	{
		if (spi_gpio_reset != -1) {
			err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
			if (err) {
				printk("failed to request GPIO for TPO LCD RESET\n");
				return;
			}
			gpio_direction_output(spi_gpio_reset, 0);
			msleep(20);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(20);
			gpio_free(spi_gpio_reset);
		}
		lcd_xinli_init();
          	lcd_power_on(1, 0);
	}
	else
	{
                lcd_power_on(0, 0);
		err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
		if (err) {
			printk("failed to request LCD RESET gpio\n");
			return;
		}
		gpio_set_value(spi_gpio_reset, 0);
		gpio_free(spi_gpio_reset);
		
		
	}
	
	gpio_direction_output(cs_gpio, 0);
	gpio_direction_output(clk_gpio, 0);
	gpio_direction_output(d_gpio, 0);
	gpio_free(d_gpio_in);
	gpio_free(cs_gpio);
	gpio_free(clk_gpio);
	gpio_free(d_gpio);
}


static struct fb_videomode xinli_video_modes[] = {
	[0] = {
		.pixclock       = 93717,
		.refresh        = 60,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 10,
		.left_margin    = 20,
		.right_margin   = 10,
		.vsync_len      = 2,
		.upper_margin   = 2,
		.lower_margin   = 10,
		.sync           = 0,
	},
};

/* SPI Control Register. */
#define     CFG_SCLKCNT(div)                    (div<<24)  /* 0xFF~0x2 */
#define     CFG_RXBITS(rx)                      ((rx - 1)<<16)   /* 0x1F~0x1 */
#define     CFG_TXBITS(tx)                      ((tx - 1)<<8)    /* 0x1F~0x1, 0x1: 2bits ... 0x1F: 32bits */
#define     CFG_SPI_ENA(spi)                    (spi<<3)
#define     CFG_SPI_SEL(spi)                    (spi<<2)   /* 1: port1; 0: port0 */
#define     CFG_SPI_3W4WB(wire)                 (wire<<1)  /* 1: 3-wire; 0: 4-wire */

static struct pxa910fb_mach_info u830_lcd_info __initdata = {
	.id                     = "Base",
	.modes                  = xinli_video_modes,
	.num_modes              = ARRAY_SIZE(xinli_video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
	.spi_ctrl               = CFG_SCLKCNT(16) | CFG_TXBITS(9) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(0) | CFG_SPI_ENA(1),
	.spi_gpio_cs            = -1,
	.spi_gpio_reset		= mfp_to_gpio(MFP_PIN_GPIO106),
	.panel_rbswap		= 1,
	.pxa910fb_lcd_power	= xinli_lcd_power_on,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

static struct pxa910fb_mach_info u830_lcd_ovly_info __initdata = {
	.id                     = "Ovly",
	.modes                  = xinli_video_modes,
	.num_modes              = ARRAY_SIZE(xinli_video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
        .spi_ctrl               = CFG_SCLKCNT(16) | CFG_TXBITS(9) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(0) | CFG_SPI_ENA(1),
        .spi_gpio_cs            = -1,
        .spi_gpio_reset         = mfp_to_gpio(MFP_PIN_GPIO106),
	.panel_rbswap		= 1,
        .pxa910fb_lcd_power     = xinli_lcd_power_on,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

void u830_add_lcd_xinli(void)
{
	struct pxa910fb_mach_info *fb = &u830_lcd_info, *ovly = &u830_lcd_ovly_info;
	pxa910_add_fb(fb);
	pxa910_add_fb_ovly(ovly);
}



