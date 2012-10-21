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

static void P1152_WriteReg(unsigned char SPI_COMMD)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	
	SBuffer=SPI_COMMD;
	gpio_direction_output(cs_gpio, 0);	//Set_CS(0); //CLR CS
	udelay(20);
	for(BitCounter=0;BitCounter<9;BitCounter++)
	{
		SBit = SBuffer&0x100;
		if(SBit)
			gpio_direction_output(d_gpio, 1);//Set_SDA(1);
		else
			gpio_direction_output(d_gpio, 0);//Set_SDA(0);
			
		udelay(20);
		gpio_direction_output(clk_gpio, 0);//Set_SCK(0); //CLR SCL
		udelay(20);
		gpio_direction_output(clk_gpio, 1);//Set_SCK(1); //SET SCL
		udelay(20);
		SBuffer = SBuffer<<1;
	}
	gpio_direction_output(cs_gpio, 1);//Set_CS(1); //SET CS
}
//***********************************************
//***********************************************
static void P1152_WriteData(unsigned char SPI_DATA)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	
	SBuffer=SPI_DATA | 0x100;
	gpio_direction_output(cs_gpio, 0);//Set_CS(0); //CLR CS
	udelay(20);
	for(BitCounter=0;BitCounter<9;BitCounter++)
	{
		SBit = SBuffer&0x100;
		if(SBit)
			gpio_direction_output(d_gpio, 1);//Set_SDA(1);
		else
			gpio_direction_output(d_gpio, 0);//Set_SDA(0);
			
		udelay(20);
		gpio_direction_output(clk_gpio, 0);//Set_SCK(0); //CLR SCL
		udelay(20);
		gpio_direction_output(clk_gpio, 1);//Set_SCK(1); //SET SCL
		udelay(20);
		SBuffer = SBuffer<<1;
	}
	gpio_direction_output(cs_gpio, 1);//Set_CS(1); //SET CS
}

static void lcd_panel_sleep(void)
{
	P1152_WriteReg(0x10);
}

static void lcd_panel_wakeup(void)
{
	P1152_WriteReg( 0x11);
	mdelay(50);
	P1152_WriteReg( 0x29);
}

static void lcd_xinli_init(void)
{
//************* Start Initial Sequence **********//
    P1152_WriteReg(0xB0);
    P1152_WriteData(0x04);
	
    P1152_WriteReg(0xC6);
    P1152_WriteData(0x1C);

    P1152_WriteReg(0x36);
    P1152_WriteData(0xD0);

    P1152_WriteReg(0x3A);
    P1152_WriteData(0x66);
 
    P1152_WriteReg(0xB4);    
    P1152_WriteData(0x00); 
 
    P1152_WriteReg(0xC0);
    P1152_WriteData(0x03);//0013
    P1152_WriteData(0xDF);//480
    P1152_WriteData(0x40);
    P1152_WriteData(0x12);
    P1152_WriteData(0x00);
    P1152_WriteData(0x01);
    P1152_WriteData(0x00);
    P1152_WriteData(0x43);
 
 
    P1152_WriteReg(0xC1);//frame frequency
    P1152_WriteData(0x07);//BCn,DIVn[1:0
    P1152_WriteData(0x28);//RTNn[4:0] 
    P1152_WriteData(0x04);// BPn[7:0]
    P1152_WriteData(0x04);// FPn[7:0]
    P1152_WriteData(0x00);
 
 
 
    P1152_WriteReg(0xC4);
    P1152_WriteData(0x63);
    P1152_WriteData(0x00);
    P1152_WriteData(0x06);
    P1152_WriteData(0x06);
 
     P1152_WriteReg(0xC8);//Gamma
    P1152_WriteData(0x06);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x16);
    P1152_WriteData(0x24);//26
    P1152_WriteData(0x30);//32 
    P1152_WriteData(0x48);
    P1152_WriteData(0x3d);
    P1152_WriteData(0x28);
    P1152_WriteData(0x20);
    P1152_WriteData(0x14);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x04);
 
    P1152_WriteData(0x06);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x16);
    P1152_WriteData(0x24);
    P1152_WriteData(0x30);
    P1152_WriteData(0x48);
    P1152_WriteData(0x3d);
    P1152_WriteData(0x28);
    P1152_WriteData(0x20);
    P1152_WriteData(0x14);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x04);
 
 
 
    P1152_WriteReg(0xC9);//Gamma
    P1152_WriteData(0x06);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x16);
    P1152_WriteData(0x24);//26
    P1152_WriteData(0x30);//32 
    P1152_WriteData(0x48);
    P1152_WriteData(0x3d);
    P1152_WriteData(0x28);
    P1152_WriteData(0x20);
    P1152_WriteData(0x14);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x04);
 
    P1152_WriteData(0x06);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x16);
    P1152_WriteData(0x24);
    P1152_WriteData(0x30);
    P1152_WriteData(0x48);
    P1152_WriteData(0x3d);
    P1152_WriteData(0x28);
    P1152_WriteData(0x20);
    P1152_WriteData(0x14);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x04);
 
 
 
    P1152_WriteReg(0xCA);//Gamma
    P1152_WriteData(0x06);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x16);
    P1152_WriteData(0x24);//26
    P1152_WriteData(0x30);//32 
    P1152_WriteData(0x48);
    P1152_WriteData(0x3d);
    P1152_WriteData(0x28);
    P1152_WriteData(0x20);
    P1152_WriteData(0x14);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x04);
 
    P1152_WriteData(0x06);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x16);
    P1152_WriteData(0x24);
    P1152_WriteData(0x30);
    P1152_WriteData(0x48);
    P1152_WriteData(0x3d);
    P1152_WriteData(0x28);
    P1152_WriteData(0x20);
    P1152_WriteData(0x14);
    P1152_WriteData(0x0c);
    P1152_WriteData(0x04);
 

    P1152_WriteReg(0xD0);
    P1152_WriteData(0x95);
    P1152_WriteData(0x0a);
    P1152_WriteData(0x08);
    P1152_WriteData(0x10);
    P1152_WriteData(0x39);
 
 
    P1152_WriteReg(0xD1);
    P1152_WriteData(0x02);
    P1152_WriteData(0x28);
    P1152_WriteData(0x28);
    P1152_WriteData(0x40);
 
    P1152_WriteReg(0xE1);    
    P1152_WriteData(0x00);    
    P1152_WriteData(0x00);    
    P1152_WriteData(0x00);    
    P1152_WriteData(0x00);    
    P1152_WriteData(0x00);   
    P1152_WriteData(0x00);   
 
    P1152_WriteReg(0xE2);    
    P1152_WriteData(0x80);    
 
    P1152_WriteReg(0x2A);    
    P1152_WriteData(0x00);    
    P1152_WriteData(0x00);    
    P1152_WriteData(0x01);    
    P1152_WriteData(0x3F);    
 
    P1152_WriteReg(0x2B);    
    P1152_WriteData(0x00);    
    P1152_WriteData(0x00);    
    P1152_WriteData(0x01);    
    P1152_WriteData(0xDF);    
 
    P1152_WriteReg(0xB0);    
    P1152_WriteData(0x03);  
 
    P1152_WriteReg(0x11);
 
    msleep(120);
 
    P1152_WriteReg(0x29);
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
	//static int is_firston;
	
	mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
	
	if (on)
	{
		if (spi_gpio_reset != -1) {
			err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
			if (err) {
				printk("failed to request GPIO for TPO LCD RESET\n");
				return;
			}
			gpio_direction_output(spi_gpio_reset, 0);
			msleep(100);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(100);
			gpio_free(spi_gpio_reset);
		}
		lcd_panel_gpio_spi_init();
		
		//if(!is_firston)
		//{
		//    lcd_xinli_init();
		//	lcd_power_on(1, 1);
		//}
		lcd_xinli_init();
		lcd_power_on(1, 0);
		//is_firston = 0;
		
	}
	else
	{
	    lcd_panel_gpio_spi_init();
		lcd_panel_sleep();
        lcd_power_on(0, 0);		
		err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
		if (err) {
			printk("failed to request LCD RESET gpio\n");
			return;
		}
		gpio_set_value(spi_gpio_reset, 0);
		gpio_free(spi_gpio_reset);
		
		
	}
	
//out:
	gpio_direction_output(cs_gpio, 0);
	gpio_direction_output(clk_gpio, 0);
	gpio_direction_output(d_gpio, 0);
	gpio_free(d_gpio_in);
	gpio_free(cs_gpio);
	gpio_free(clk_gpio);
	gpio_free(d_gpio);
}

static void lcd_power_do_work(struct work_struct *work)
{
	msleep(100);
    xinli_lcd_power_on(g_fbi, g_spi_gpio_cs, g_spi_gpio_reset, g_on);
}

static int xinli_lcd_power(struct pxa910fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
    g_fbi = fbi;
	g_spi_gpio_cs = spi_gpio_cs;
	g_spi_gpio_reset = spi_gpio_reset;
	g_on = on;

	schedule_work(&lcd_poweron_wq);

	return 0;
}

static struct fb_videomode xinli_video_modes[] = {

	[0] = {
		.pixclock       = 80000,//90000,
		.refresh        = 60,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 2,//10,
		.left_margin    = 10,//20,
		.right_margin   = 40,//10,
		.vsync_len      = 1,//2//,///2,
		.upper_margin   = 4,//8,///8,
		.lower_margin   = 4,//8,///8,
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

static struct pxa910fb_mach_info u810_lcd_info __initdata = {
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
	.pxa910fb_lcd_power	= xinli_lcd_power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

static struct pxa910fb_mach_info u810_lcd_ovly_info __initdata = {
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
        .pxa910fb_lcd_power     = xinli_lcd_power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

void u810_add_lcd_xinli(void)
{
	struct pxa910fb_mach_info *fb = &u810_lcd_info, *ovly = &u810_lcd_ovly_info;
	
    INIT_WORK(&lcd_poweron_wq, lcd_power_do_work);
	
	spi_send = pxa910fb_spi_send;
	pxa910_add_fb(fb);
	pxa910_add_fb_ovly(ovly);
}




