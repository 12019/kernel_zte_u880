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

extern void lcd_power_on(int on, int ms);
extern void u810_add_lcd_xinli(void);

static int clk_gpio;
static int d_gpio;
static int d_gpio_in;    
static int cs_gpio;

static void ILI9481_WriteReg(unsigned char SPI_COMMD)
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
static void ILI9481_WriteData(unsigned char SPI_DATA)
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
	ILI9481_WriteReg(0x10);
}

static void lcd_panel_wakeup(void)
{
	ILI9481_WriteReg( 0x11);
	mdelay(50);
	ILI9481_WriteReg( 0x29);
}

static void lcd_lead_init(void)
{
	//************* Start Initial Sequence **********//
	ILI9481_WriteReg( 0x11);
	mdelay(20);//Delay(10*20);
	
	//CPT+ILI9481
	ILI9481_WriteReg(0xC6);
	ILI9481_WriteData(0x9a);//ILI9481_WriteData(0x9B);///5B for lcd effect

	ILI9481_WriteReg(0xD0);
	ILI9481_WriteData(0x07);
	ILI9481_WriteData(0x41);//ILI9481_WriteData( 0x41);for lcd effect
	ILI9481_WriteData(0x1A);
	
	ILI9481_WriteReg(0xD1);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x0B);//10
	ILI9481_WriteData(0x11);
	
	ILI9481_WriteReg(0xD2);
	ILI9481_WriteData(0x01);
	ILI9481_WriteData(0x11);
	
	ILI9481_WriteReg(0xC0);
	ILI9481_WriteData(0x10);
	ILI9481_WriteData(0x3B);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x02);
	ILI9481_WriteData(0x11);
	
	ILI9481_WriteReg(0xC5);
	ILI9481_WriteData(0x02);//
	
	ILI9481_WriteReg(0xC8);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x66);
	ILI9481_WriteData(0x15);
	ILI9481_WriteData(0x24);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x08);
	ILI9481_WriteData(0x26);
	ILI9481_WriteData(0x11);
	ILI9481_WriteData(0x77);
	ILI9481_WriteData(0x42);
	ILI9481_WriteData(0x08);
	ILI9481_WriteData(0x00);
	
	ILI9481_WriteReg(0x36);
	ILI9481_WriteData(0xDA);//ILI9481_WriteData( 0x0A);rotated for u810 lcd screen
	ILI9481_WriteReg(0x3A);
	ILI9481_WriteData(0x66);
	ILI9481_WriteReg(0x0C);
	ILI9481_WriteData(0x66);
	ILI9481_WriteReg(0x2A);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x01);
	ILI9481_WriteData(0x3F);
	ILI9481_WriteReg(0x2B);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x01);
	ILI9481_WriteData(0xE0);
	mdelay(120);//Delay(10*120);
	ILI9481_WriteReg(0xB4);
	ILI9481_WriteData(0x10);

	ILI9481_WriteReg(0x29);
	ILI9481_WriteReg(0x2C);
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
	//gpio_direction_output(cs_gpio, 1);
	//udelay(20);

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
		printk(KERN_ERR"failed to request GPIO for LCD Data in\n");
		return -1;
	}

	return 0;
}

static int lead_lcd_power(struct pxa910fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
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
				return 1;
			}
			gpio_direction_output(spi_gpio_reset, 0);
			msleep(100);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(100);
			gpio_free(spi_gpio_reset);
		}
		lcd_lead_init();
		lcd_power_on(1, 50);
		
	}
	else
	{
	    //lcd_panel_gpio_spi_init();
		//lcd_panel_sleep();
        lcd_power_on(0, 0);		
		err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
		if (err) {
			printk("failed to request LCD RESET gpio\n");
			return 1;
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

	return 0;
}

static struct fb_videomode lead_video_modes[] = {
	/* Lead HVGA mode info */
	[0] = {
		.pixclock       = 122070,
		.refresh        = 60,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 3,
		.left_margin    = 3,
		.right_margin   = 3,
		.vsync_len      = 2,
		.upper_margin   = 2,
		.lower_margin   = 4,
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
	.modes                  = lead_video_modes,
	.num_modes              = ARRAY_SIZE(lead_video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
	.spi_ctrl               = CFG_SCLKCNT(16) | CFG_TXBITS(9) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(0) | CFG_SPI_ENA(1),
	.spi_gpio_cs            = -1,
	.spi_gpio_reset		= mfp_to_gpio(MFP_PIN_GPIO106),
	.panel_rbswap		= 1,
	.pxa910fb_lcd_power	= lead_lcd_power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

static struct pxa910fb_mach_info u810_lcd_ovly_info __initdata = {
	.id                     = "Ovly",
	.modes                  = lead_video_modes,
	.num_modes              = ARRAY_SIZE(lead_video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
        .spi_ctrl               = CFG_SCLKCNT(16) | CFG_TXBITS(9) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(0) | CFG_SPI_ENA(1),
        .spi_gpio_cs            = -1,
        .spi_gpio_reset         = mfp_to_gpio(MFP_PIN_GPIO106),
	.panel_rbswap		= 1,
        .pxa910fb_lcd_power     = lead_lcd_power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

static void u810_add_lcd_ili9481(void)
{
	struct pxa910fb_mach_info *fb = &u810_lcd_info, *ovly = &u810_lcd_ovly_info;

	spi_send = pxa910fb_spi_send;
	pxa910_add_fb(fb);
	pxa910_add_fb_ovly(ovly);
}


static int is_ili9481_lcd(void)
{
    int i;
	unsigned short SBit,SBuffer,LcdCodeId[6];
	unsigned char BitCounter;

//	ILI9481_WriteReg(0xC6);
//	ILI9481_WriteData(0x00);
	ILI9481_WriteReg(0xB0);    
	ILI9481_WriteData(0x00);

	SBuffer = 0xBF;
	gpio_direction_input(d_gpio_in);
	
	gpio_direction_output(cs_gpio, 0);	//Set_CS(0); //CLR CS
	udelay(20);
	//send command
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

    //read ili9481 LcdCodeId
	for(i = 0; i < 6; i++)
	{
	    LcdCodeId[i] = 0;
        for(BitCounter = 0; BitCounter < 8; BitCounter++)
        {
            gpio_direction_output(clk_gpio, 0);//Set_SCK(0); //CLR SCL
            udelay(20);
			SBit = gpio_get_value(d_gpio_in);
            gpio_direction_output(clk_gpio, 1);//Set_SCK(1); //SET SCL
			udelay(20);
			if(0 != SBit)
			{
			    LcdCodeId[i] |= (1 << (7 - BitCounter));
			}
        }
        printk(KERN_INFO "+++++++++lcd_code is 0x%x %d\n", LcdCodeId[i], LcdCodeId[i]);
    }
    
	gpio_direction_output(cs_gpio, 1);//Set_CS(1); //SET CS

//       ILI9481_WriteReg(0xC6);
//	ILI9481_WriteData(0x9a);

	if((LcdCodeId[2] == 0x4a && LcdCodeId[3] == 0x40) || (LcdCodeId[3] == 0x94 && LcdCodeId[4] == 0x81))
	{
		return 1;
	}
	else if((LcdCodeId[2] == 0x22) && (LcdCodeId[3] == 0x15))
	{
		return 0;
	}

	return 1;
}

void __init u810_add_lcd_lead(void)
{
//  mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
    lcd_panel_gpio_spi_init();
	
    if(is_ili9481_lcd())
    {
    	u810_add_lcd_ili9481();
		printk("+++++++++u810_add_lcd_lead\n");
    }
	else
	{
		u810_add_lcd_xinli();
		printk("+++++++++u810_add_lcd_xinli\n");
	}

	gpio_free(cs_gpio);
	gpio_free(clk_gpio);
	gpio_free(d_gpio);
	gpio_free(d_gpio_in);
}



