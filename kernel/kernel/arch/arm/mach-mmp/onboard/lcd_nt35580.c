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

static int IS_NT35580_LCD(void);
extern  void u880_add_lcd_HX8363A(void);
extern  void u880_add_lcd_HX8063A(void);
extern unsigned int IS_HX8363A_LCD(void);
extern unsigned int IS_HX8063A_LCD(void);
extern unsigned int IS_HX8369_LG_LCD(void);
extern unsigned int IS_HX8369_LCD_HYIDIS(void);
extern void u880_add_lcd_HX8369_LG(void);
extern void u880_add_lcd_HX8369_HYIDIS(void);

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
	//GPIO51_GPIO51,
	//GPIO51_PWM,
};


extern void lcd_power_on(int on, int ms);
static int clk_gpio;
static int d_gpio;
static int d_gpio_in;    
static int cs_gpio;

int  lcd_type=0;
#define LCD_DELAYT_TIME  (5)

static void NT35580_WriteReg(unsigned short SPI_COMMD)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	unsigned long flags;
	static DEFINE_SPINLOCK(trigger_lock);
	
	SBuffer=(0x2000|((SPI_COMMD>>8)&0x00FF));

	spin_lock_irqsave(&trigger_lock,flags );
	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<16;BitCounter++)
	{
		SBit = SBuffer&0x8000;
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
	gpio_direction_output(cs_gpio, 1);
	
	udelay(LCD_DELAYT_TIME);
	SBuffer=(SPI_COMMD&0x00FF);
	gpio_direction_output(cs_gpio, 0);
			
	for(BitCounter=0;BitCounter<16;BitCounter++)
	{
		SBit = SBuffer&0x8000;
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
	gpio_direction_output(cs_gpio, 1);
	spin_unlock_irqrestore(&trigger_lock,flags );
}
//***********************************************
//***********************************************
static void NT35580_WriteRegData(unsigned short SPI_COMMD,unsigned char SPI_DATA)
{
	unsigned short SBit,SBuffer;
	unsigned char BitCounter;
	unsigned long flags;
	static DEFINE_SPINLOCK(trigger_lock);
	
	SBuffer=(0x2000|((SPI_COMMD>>8)&0x00FF));
	
	spin_lock_irqsave(&trigger_lock,flags );
	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<16;BitCounter++)
	{
		SBit = SBuffer&0x8000;
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
	gpio_direction_output(cs_gpio, 1);
	spin_unlock_irqrestore(&trigger_lock,flags );
	
       udelay(LCD_DELAYT_TIME);
	SBuffer=(SPI_COMMD&0x00FF);

	spin_lock_irqsave(&trigger_lock,flags );
	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<16;BitCounter++)
	{
		SBit = SBuffer&0x8000;
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
	gpio_direction_output(cs_gpio, 1);
	spin_unlock_irqrestore(&trigger_lock,flags );	
	
	SBuffer=(0x4000|SPI_DATA);
	 udelay(LCD_DELAYT_TIME);
	spin_lock_irqsave(&trigger_lock,flags );//LOCK
	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<16;BitCounter++)
	{
		SBit = SBuffer&0x8000;
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
	gpio_direction_output(cs_gpio, 1);
	spin_unlock_irqrestore(&trigger_lock,flags );//UNLOCK
}

static unsigned char  NT35580_ReadRegData(unsigned short SPI_COMMD)
{
	unsigned short SBit,SBuffer=0;
	unsigned char BitCounter=0;
	unsigned char SpiData=0;

	SBuffer=(0x2000|((SPI_COMMD>>8)&0x00FF));
	gpio_direction_output(cs_gpio, 0);	
	for(BitCounter=0;BitCounter<16;BitCounter++)
{
		SBit = SBuffer&0x8000;
		gpio_direction_output(clk_gpio, 0);
		if(SBit)
			gpio_direction_output(d_gpio, 1);
		else
			gpio_direction_output(d_gpio, 0);
		udelay(20);
		gpio_direction_output(clk_gpio, 1);
		udelay(20);
		SBuffer = SBuffer<<1;
}
	gpio_direction_output(cs_gpio, 1);
	SBuffer=(SPI_COMMD&0x00FF);
	gpio_direction_output(cs_gpio, 0);

	for(BitCounter=0;BitCounter<16;BitCounter++)
{
		SBit = SBuffer&0x8000;
		gpio_direction_output(clk_gpio, 0);
		if(SBit)
			gpio_direction_output(d_gpio, 1);
		else
			gpio_direction_output(d_gpio, 0);
		udelay(20);
		gpio_direction_output(clk_gpio, 1);
		udelay(20);
		SBuffer = SBuffer<<1;
	}
	gpio_direction_output(cs_gpio, 1);
	
	SBuffer=0xC0;

	gpio_direction_output(cs_gpio, 0);
	for(BitCounter=0;BitCounter<8;BitCounter++)
	{
		SBit = SBuffer&0x80;
		gpio_direction_output(clk_gpio, 0);
		if(SBit)
			gpio_direction_output(d_gpio, 1);
		else
			gpio_direction_output(d_gpio, 0);
		udelay(20);
		gpio_direction_output(clk_gpio, 1);
		udelay(20);
		SBuffer = SBuffer<<1;
	}
	
	gpio_direction_input(d_gpio_in);
	
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
	gpio_direction_output(cs_gpio, 1);
	return SpiData;
	
}
static void lcd_panel_sleep(void)
{
        #if  0
	NT35580_WriteRegData(0x1000,0x00);
	msleep(200);
       NT35580_WriteRegData(0x2800,0x00);
       #else
       NT35580_WriteReg(0x1000);
        msleep(200);
       #endif
}
	
static void lcd_panel_wakeup(void)
{

	NT35580_WriteReg(0x1100);
	msleep(200);
      NT35580_WriteReg(0x2900);
}
	
static void lcd_lead_nt35580_init(void)
{	


NT35580_WriteReg(0x1100);
msleep(120);
NT35580_WriteRegData(0x2e80,0x01);//0x01:Don't reload MTP   0x00:reload MTP
mdelay(10);

NT35580_WriteRegData(0x0680,0x21);
NT35580_WriteRegData(0xD380,0x04);
NT35580_WriteRegData(0xD480,0x5B);
NT35580_WriteRegData(0xD580,0x07);
NT35580_WriteRegData(0xD680,0x52);
NT35580_WriteRegData(0xD080,0x17);
NT35580_WriteRegData(0xD180,0x0D);
NT35580_WriteRegData(0xD280,0x04);
NT35580_WriteRegData(0xDC80,0x04);
NT35580_WriteRegData(0xD780,0x01);
NT35580_WriteRegData(0x2280,0x02);
NT35580_WriteRegData(0x2480,0x52);
NT35580_WriteRegData(0x2580,0x25);
NT35580_WriteRegData(0x2780,0x75);
NT35580_WriteRegData(0x3A00,0x66);

NT35580_WriteRegData(0x0180,0x02);
NT35580_WriteRegData(0x4080,0x00);
NT35580_WriteRegData(0x4180,0x05);
NT35580_WriteRegData(0x4280,0x15);
NT35580_WriteRegData(0x4380,0x27);
NT35580_WriteRegData(0x4480,0x19);
NT35580_WriteRegData(0x4580,0x2D);
NT35580_WriteRegData(0x4680,0x5E);
NT35580_WriteRegData(0x4780,0x43);
NT35580_WriteRegData(0x4880,0x1F);
NT35580_WriteRegData(0x4980,0x26);
NT35580_WriteRegData(0x4A80,0x9C);
NT35580_WriteRegData(0x4B80,0x1D);
NT35580_WriteRegData(0x4C80,0x41);
NT35580_WriteRegData(0x4D80,0x5A);
NT35580_WriteRegData(0x4E80,0x8B);
NT35580_WriteRegData(0x4F80,0x95);
NT35580_WriteRegData(0x5080,0x64);
NT35580_WriteRegData(0x5180,0x7F);
NT35580_WriteRegData(0x5880,0x00);
NT35580_WriteRegData(0x5980,0x1A);
NT35580_WriteRegData(0x5A80,0x68);
NT35580_WriteRegData(0x5B80,0x79);
NT35580_WriteRegData(0x5C80,0x2A);
NT35580_WriteRegData(0x5D80,0x43);
NT35580_WriteRegData(0x5E80,0x67);
NT35580_WriteRegData(0x5F80,0x65);
NT35580_WriteRegData(0x6080,0x17);
NT35580_WriteRegData(0x6180,0x1E);
NT35580_WriteRegData(0x6280,0xBF);
NT35580_WriteRegData(0x6380,0x20);
NT35580_WriteRegData(0x6480,0x51);
NT35580_WriteRegData(0x6580,0x66);
NT35580_WriteRegData(0x6680,0xD7);
NT35580_WriteRegData(0x6780,0xE9);
NT35580_WriteRegData(0x6880,0x79);
NT35580_WriteRegData(0x6980,0x7F);
NT35580_WriteRegData(0x7080,0x00);
NT35580_WriteRegData(0x7180,0x07);
NT35580_WriteRegData(0x7280,0x1A);
NT35580_WriteRegData(0x7380,0x30);
NT35580_WriteRegData(0x7480,0x1C);
NT35580_WriteRegData(0x7580,0x30);
NT35580_WriteRegData(0x7680,0x61);
NT35580_WriteRegData(0x7780,0x4F);
NT35580_WriteRegData(0x7880,0x20);
NT35580_WriteRegData(0x7980,0x27);
NT35580_WriteRegData(0x7A80,0xA5);
NT35580_WriteRegData(0x7B80,0x1E);
NT35580_WriteRegData(0x7C80,0x48);
NT35580_WriteRegData(0x7D80,0x5E);
NT35580_WriteRegData(0x7E80,0x87);
NT35580_WriteRegData(0x7F80,0xAD);
NT35580_WriteRegData(0x8080,0x6A);
NT35580_WriteRegData(0x8180,0x7F);
NT35580_WriteRegData(0x8880,0x00);
NT35580_WriteRegData(0x8980,0x15);
NT35580_WriteRegData(0x8A80,0x53);
NT35580_WriteRegData(0x8B80,0x76);
NT35580_WriteRegData(0x8C80,0x2D);
NT35580_WriteRegData(0x8D80,0x43);
NT35580_WriteRegData(0x8E80,0x66);
NT35580_WriteRegData(0x8F80,0x5C);
NT35580_WriteRegData(0x9080,0x18);
NT35580_WriteRegData(0x9180,0x1F);
NT35580_WriteRegData(0x9280,0xB2);
NT35580_WriteRegData(0x9380,0x1E);
NT35580_WriteRegData(0x9480,0x4E);
NT35580_WriteRegData(0x9580,0x64);
NT35580_WriteRegData(0x9680,0xCE);
NT35580_WriteRegData(0x9780,0xE3);
NT35580_WriteRegData(0x9880,0x78);
NT35580_WriteRegData(0x9980,0x7F);
NT35580_WriteRegData(0xA080,0x00);
NT35580_WriteRegData(0xA180,0x09);
NT35580_WriteRegData(0xA280,0x25);
NT35580_WriteRegData(0xA380,0x41);
NT35580_WriteRegData(0xA480,0x20);
NT35580_WriteRegData(0xA580,0x35);
NT35580_WriteRegData(0xA680,0x63);
NT35580_WriteRegData(0xA780,0x5D);
NT35580_WriteRegData(0xA880,0x1F);
NT35580_WriteRegData(0xA980,0x26);
NT35580_WriteRegData(0xAA80,0xAE);
NT35580_WriteRegData(0xAB80,0x19);
NT35580_WriteRegData(0xAC80,0x3D);
NT35580_WriteRegData(0xAD80,0x4B);
NT35580_WriteRegData(0xAE80,0xA7);
NT35580_WriteRegData(0xAF80,0xF8);
NT35580_WriteRegData(0xB080,0x7D);
NT35580_WriteRegData(0xB180,0x7F);
NT35580_WriteRegData(0xB880,0x00);
NT35580_WriteRegData(0xB980,0x02);
NT35580_WriteRegData(0xBA80,0x08);
NT35580_WriteRegData(0xBB80,0x59);
NT35580_WriteRegData(0xBC80,0x34);
NT35580_WriteRegData(0xBD80,0x4A);
NT35580_WriteRegData(0xBE80,0x6B);
NT35580_WriteRegData(0xBF80,0x53);
NT35580_WriteRegData(0xC080,0x16);
NT35580_WriteRegData(0xC180,0x1E);
NT35580_WriteRegData(0xC280,0xA4);
NT35580_WriteRegData(0xC380,0x1B);
NT35580_WriteRegData(0xC480,0x4B);
NT35580_WriteRegData(0xC580,0x62);
NT35580_WriteRegData(0xC680,0xBD);
NT35580_WriteRegData(0xC780,0xD7);
NT35580_WriteRegData(0xC880,0x75);
NT35580_WriteRegData(0xC980,0x7F);
NT35580_WriteRegData(0x3500,0x00);
//NT35580_WriteRegData(0x5300,0x2C);
NT35580_WriteRegData(0x3B00,0x00);//
NT35580_WriteRegData(0x3600,0x41);//
//NT35580_WriteRegData(0x3B00,0x28);//
NT35580_WriteReg(0x2900);
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
 static  int  Pown=0;
 extern  void lcd_panle_on(struct  pxa910fb_info *fbi );
static int LeadNt35580_Lcd_Power(struct pxa910fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
     int  err;
	printk("Enter %s,spi_gpio_reset=%d,on=%d\n", __FUNCTION__,spi_gpio_reset,on);

	mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
	lcd_panel_gpio_spi_init();
     
	if (on)
	{

              		if (spi_gpio_reset != -1)
				{
					err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
				}
				if (err) 
				{
				printk("failed to request GPIO for TPO LCD RESET\n");
				return 1;
				}
				gpio_direction_output(spi_gpio_reset, 0);
				msleep(10);
				gpio_set_value(spi_gpio_reset, 1);
				msleep(120);
				gpio_free(spi_gpio_reset);
		lcd_lead_nt35580_init();
	                         msleep(100);
	lcd_power_on(1, 50);
                               lcd_panle_on( fbi );

	}
	else
	{
	lcd_power_on(0, 0);
	//gpio_direction_output(bl_gpio, 0);
	lcd_panel_sleep();

	}
	
	gpio_direction_output(cs_gpio, 0);
	gpio_direction_output(clk_gpio, 0);
	gpio_direction_output(d_gpio, 0);
	gpio_free(d_gpio_in);
	gpio_free(cs_gpio);
	gpio_free(clk_gpio);
	gpio_free(d_gpio);
	//gpio_free(bl_gpio);
	printk("Exit %s,spi_gpio_reset=%d,on=%d\n", __FUNCTION__,spi_gpio_reset,on);
	return 0;
}

static struct fb_videomode lead_video_modes[] = {
	/* Lead HVGA mode info */
	[0] = {
		.pixclock       = 41013,
		.refresh        = 60,
		.xres           = 480,
		.yres           = 800,
		.hsync_len      = 3*2,
		.left_margin    = 3*2,
		.right_margin   = 3*2,
		.vsync_len      = 2*2,
		.upper_margin   = 2*2,
		.lower_margin   = 4*2,
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

static struct pxa910fb_mach_info u880_lcd_info __initdata = {
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
	.panel_rbswap		= 0,
	.pxa910fb_lcd_power	= LeadNt35580_Lcd_Power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

static struct pxa910fb_mach_info u880_lcd_ovly_info __initdata = {
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
        .pxa910fb_lcd_power     = LeadNt35580_Lcd_Power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

static void u880_add_lcd_NT35580(void)
{
	struct pxa910fb_mach_info *fb = &u880_lcd_info, *ovly = &u880_lcd_ovly_info;
	pxa910_add_fb(fb);
	pxa910_add_fb_ovly(ovly);
	}

static int IS_NT35580_LCD(void)
	{
       #if   0
	unsigned char RegDataL=0;
	unsigned char RegDataH=0;
	RegDataH=  NT35580_ReadRegData(0x1080);
	RegDataL=  NT35580_ReadRegData(0x1180);
	printk("Enter %s,ID=0x%x 0x%x\n", __FUNCTION__,RegDataH,RegDataL);
	if((0x55==RegDataH)&&(0x80==RegDataL))
	{
	return 1;
}
	else
	{
	return 0;
	}
	#else
	u32     isr;
	//isr = readl(0xFE20B000 + 0xf8);
	void * reg_base;
	reg_base = ioremap_nocache(0xd420b000, 0x1ec);
	isr = readl(reg_base + 0xf8);
       iounmap(reg_base);
	printk("Enter %s,ID=0x%x\n", __FUNCTION__,isr);
	if(isr==0x5A5A5A01)
	{
	return 1;
	}
	else
	{
	return 0;
	}
	#endif
}
void __init u880_add_lcd(void)
{
    printk("Enter %s\n", __FUNCTION__);

	if( IS_NT35580_LCD())
	{    
	lcd_type=1;
	u880_add_lcd_NT35580();
	return;
	}

	if(IS_HX8363A_LCD())
	{
	lcd_type=2;
	u880_add_lcd_HX8363A();
	return;
	}

	if(IS_HX8369_LCD_HYIDIS())
	{
	lcd_type=3;
	u880_add_lcd_HX8369_HYIDIS();
	return;
	}

	if(IS_HX8369_LG_LCD())
	{
	lcd_type=4;
	u880_add_lcd_HX8369_LG();
	return;
	}

       if(IS_HX8063A_LCD())
	{
	lcd_type=5;
	u880_add_lcd_HX8063A();
	return;
	}
        u880_add_lcd_HX8369_LG();
	printk("Enter %s  Error !!!\n", __FUNCTION__);

}



