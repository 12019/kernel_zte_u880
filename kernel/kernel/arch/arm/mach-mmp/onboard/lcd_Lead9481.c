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
extern void lcd_power_on(int on, int ms);
extern void u830_add_lcd_xinli(void);
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
	GPIO90_LCD_DD5,	
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


static int clk_gpio;
static int d_gpio;
static int d_gpio_in;    
static int cs_gpio;
#define  LCD_DELAY_TIME   (10)
static void ILI9481_WriteReg(unsigned char SPI_COMMD)
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
static void ILI9481_WriteData(unsigned char SPI_DATA)
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
#if   0
	ILI9481_WriteReg( 0x11);
msleep(120);
	ILI9481_WriteReg(0xC6);
ILI9481_WriteData(0x9A);
	ILI9481_WriteReg(0xD0);
	ILI9481_WriteData(0x07);
ILI9481_WriteData(0x47);
	ILI9481_WriteData(0x1D);
	ILI9481_WriteReg(0xD1);
	ILI9481_WriteData(0x00);
ILI9481_WriteData(0x3F);//10
ILI9481_WriteData(0x14);
	ILI9481_WriteReg(0xD2);
	ILI9481_WriteData(0x01);
ILI9481_WriteData(0x11);
	ILI9481_WriteReg(0xC0);
ILI9481_WriteData(0x00);//04
	ILI9481_WriteData(0x3B);
	ILI9481_WriteData(0x00);
	ILI9481_WriteData(0x02);
	ILI9481_WriteData(0x11);
	ILI9481_WriteReg(0xC5);
ILI9481_WriteData(0x03);
	ILI9481_WriteReg(0xC8);
	ILI9481_WriteData(0x00);
ILI9481_WriteData(0x53);
	ILI9481_WriteData(0x17);
ILI9481_WriteData(0x45);
ILI9481_WriteData(0x0A);
ILI9481_WriteData(0x1A);
ILI9481_WriteData(0x06);
ILI9481_WriteData(0x42);
	ILI9481_WriteData(0x77);
ILI9481_WriteData(0x54);
ILI9481_WriteData(0x12);
	ILI9481_WriteData(0x0C);
	ILI9481_WriteReg(0x36);
ILI9481_WriteData(0xDA); //
	ILI9481_WriteReg(0x3A);
ILI9481_WriteData(0x66); //18bit
ILI9481_WriteReg(0x2A);
ILI9481_WriteData(0x00);
ILI9481_WriteData(0x00);
ILI9481_WriteData(0x01);
ILI9481_WriteData(0x3F);
ILI9481_WriteReg(0x2B);
ILI9481_WriteData(0x00);
ILI9481_WriteData(0x00);
ILI9481_WriteData(0x01);
ILI9481_WriteData(0xDF);
msleep(120);
	ILI9481_WriteReg(0xB4);
ILI9481_WriteData(0x10);  //
ILI9481_WriteReg(0x29);   //display_on
ILI9481_WriteReg(0x21);   //Enter_invert_mode//0x20
ILI9481_WriteReg(0x2C);   //Write_memory_start
#endif
#if   0

       ILI9481_WriteReg(0x11);
        msleep(120);
        ILI9481_WriteReg(0xC6);
        ILI9481_WriteData(0x9B);

        ILI9481_WriteReg(0xD0);
        ILI9481_WriteData(0x07);
        ILI9481_WriteData(0x47);
        ILI9481_WriteData(0x1D);
 
        ILI9481_WriteReg(0xD1);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x3F);//10
        ILI9481_WriteData(0x14);
 
        ILI9481_WriteReg(0xD2);
        ILI9481_WriteData(0x01);
        ILI9481_WriteData(0x11);
 
        ILI9481_WriteReg(0xC0);
        ILI9481_WriteData(0x00);//04
        ILI9481_WriteData(0x3B);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x02);
        ILI9481_WriteData(0x11);
 
        ILI9481_WriteReg(0xC5);
        ILI9481_WriteData(0x03);
 
        ILI9481_WriteReg(0xC8);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x53);
        ILI9481_WriteData(0x17);
        ILI9481_WriteData(0x45);
        ILI9481_WriteData(0x0A);
        ILI9481_WriteData(0x1A);
        ILI9481_WriteData(0x06);
        ILI9481_WriteData(0x42);
        ILI9481_WriteData(0x77);
        ILI9481_WriteData(0x54);
        ILI9481_WriteData(0x12);
        ILI9481_WriteData(0x0C);
 
        ILI9481_WriteReg(0x36);
        ILI9481_WriteData(0x09); // 

        ILI9481_WriteReg(0x3A);
        ILI9481_WriteData(0x66); //18bit
        ILI9481_WriteReg(0x2A);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x01);
        ILI9481_WriteData(0x3F);
        ILI9481_WriteReg(0x2B);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x01);
        ILI9481_WriteData(0xDF);

        ILI9481_WriteReg(0xE4);
        ILI9481_WriteData(0xA0);

        ILI9481_WriteReg(0xF0);
        ILI9481_WriteData(0x01);

        ILI9481_WriteReg(0xF3);
        ILI9481_WriteData(0x02);
        ILI9481_WriteData(0x1A);

        ILI9481_WriteReg(0xF7);
        ILI9481_WriteData(0x80);
        msleep(120);
        ILI9481_WriteReg(0xB4);
        ILI9481_WriteData(0x11);  

        ILI9481_WriteReg(0x29);   //display_on
        ILI9481_WriteReg(0x21); //Enter_invert_mode//0x20
        ILI9481_WriteReg(0x2C);   //Write_memory_start
       msleep(100);
	   #endif

	   #if   1
	  ILI9481_WriteReg(0x11);
        msleep(120);
        ILI9481_WriteReg(0xC6);
        ILI9481_WriteData(0x9A);

        ILI9481_WriteReg(0xD0);
        ILI9481_WriteData(0x07);
        ILI9481_WriteData(0x47);
        ILI9481_WriteData(0x1D);
 
        ILI9481_WriteReg(0xD1);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x3F);//10
        ILI9481_WriteData(0x14);
 
        ILI9481_WriteReg(0xD2);
        ILI9481_WriteData(0x01);
        ILI9481_WriteData(0x11);
 
        ILI9481_WriteReg(0xC0);
        ILI9481_WriteData(0x00);//04
        ILI9481_WriteData(0x3B);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x02);
        ILI9481_WriteData(0x11);
 
        ILI9481_WriteReg(0xC5);
        ILI9481_WriteData(0x03);
 
        ILI9481_WriteReg(0xC8);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x53);
        ILI9481_WriteData(0x17);
        ILI9481_WriteData(0x45);
        ILI9481_WriteData(0x0A);
        ILI9481_WriteData(0x1A);
        ILI9481_WriteData(0x06);
        ILI9481_WriteData(0x42);
        ILI9481_WriteData(0x77);
        ILI9481_WriteData(0x54);
        ILI9481_WriteData(0x12);
        ILI9481_WriteData(0x0C);
 
        ILI9481_WriteReg(0x36);
        ILI9481_WriteData(0xDA); 
        ILI9481_WriteReg(0x3A);
        ILI9481_WriteData(0x66); //18bit
        ILI9481_WriteReg(0x2A);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x01);
        ILI9481_WriteData(0x3F);
        ILI9481_WriteReg(0x2B);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x00);
        ILI9481_WriteData(0x01);
        ILI9481_WriteData(0xDF);

         #if   0
        ILI9481_WriteReg(0xE4);
        ILI9481_WriteData(0xA0);
       
        ILI9481_WriteReg(0xF0);
        ILI9481_WriteData(0x01);

       
        ILI9481_WriteReg(0xF3);
        ILI9481_WriteData(0x40);
        ILI9481_WriteData(0x0A);
	 #endif
        #if 0
        ILI9481_WriteReg(0xF7);
        ILI9481_WriteData(0x80);
	 #endif
        msleep(120);
        ILI9481_WriteReg(0xB4);
        ILI9481_WriteData(0x10); 

        ILI9481_WriteReg(0x29);   //display_on
        ILI9481_WriteReg(0x21); 
//Enter_invert_mode//0x20
        ILI9481_WriteReg(0x2C);   //Write_memory_start
       msleep(100); 
	   #endif
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
			msleep(20);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(100);
			gpio_free(spi_gpio_reset);
		}
		lcd_lead_init();
                lcd_power_on(1, 50);
	}
	else
	{
                lcd_power_on(0, 0);
		err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
		if (err) {
			printk("failed to request LCD RESET gpio\n");
			return 1;
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

	return 0;
}

static struct fb_videomode lead_video_modes[] = {
	/* Lead HVGA mode info */
	[0] = {
		.pixclock       = 103808,
		.refresh        = 60,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 3,
		.left_margin    = 3,
		.right_margin   = 3,
		.vsync_len      = 3,
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
     #if    0
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
	#else
	u32     isr;
	void * reg_base;
	reg_base = ioremap_nocache(0xd420b000, 0x1ec);
	isr = readl(reg_base + 0xf8);
	iounmap(reg_base);
	printk("Enter %s,ID=0x%x\n", __FUNCTION__,isr);
	if(isr==0x5A5A5A02)
	{
	return 1;
	}
	else
	{
	return 0;
	}
	#endif
}

void __init u810_add_lcd_lead(void)
{
	lcd_panel_gpio_spi_init();

	if(is_ili9481_lcd())
	{
		u810_add_lcd_ili9481();
		printk("+++++++++u810_add_lcd_lead\n");
	}
	else
	{
		u830_add_lcd_xinli();
		printk("+++++++++u810_add_lcd_xinli\n");
	}

	gpio_free(cs_gpio);
	gpio_free(clk_gpio);
	gpio_free(d_gpio);
	gpio_free(d_gpio_in);
}



