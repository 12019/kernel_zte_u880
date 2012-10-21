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

#define LCD_ILI9325_ID 0x9325

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
extern void u810_add_lcd_hx8347(void);

extern void lcd_power_on(int on, int ms);
unsigned int  U802_lcd_type=0;
struct reg_value
{
	unsigned int reg;
	unsigned int value;
	unsigned int msleep;
};

struct reg_value ili9325_reg[] = {
//************* Start Initial Sequence **********//     
    {0x0001, 0x0100, 0},    
    {0x0002, 0x0700, 0},    
    {0x0003, 0x1030, 0},    
    {0x0004, 0x0000, 0},
    {0x0008, 0x0202, 0},     
    {0x0009, 0x0000, 0},    
    {0x000A, 0x0000, 0},    
    {0x000C, 0x0000, 0},     
    {0x000D, 0x0000, 0},    
    {0x000F, 0x0000, 0},       
  //*************Power On sequence ****************//     
    {0x0010, 0x0000, 0},      
    {0x0011, 0x0007, 0},       
    {0x0012, 0x0000, 0},      
    {0x0013, 0x0000, 0},      
    {0x0007, 0x0001, 200},   
    
    {0x0010, 0x1690, 0},      
    {0x0011, 0x0227, 50},     

    {0x0012, 0x009D, 50},  

    {0x0013, 0x1900, 0},      
    {0x0029, 0x0025, 0},      
    {0x002B, 0x000D, 50},    

    {0x0020, 0x0000, 0},      
    {0x0021, 0x0000, 0},  
// ----------- Adjust the Gamma Curve ----------//    
    {0x0030, 0x0007, 0},     
    {0x0031, 0x0200, 0},     
    {0x0032, 0x0303, 0},     
    {0x0035, 0x0206, 0},     
    {0x0036, 0x0008, 0},     
    {0x0037, 0x0404, 0},     
    {0x0038, 0x0705, 0},     
    {0x0039, 0x0007, 0},     
    {0x003C, 0x0602, 0},     
    {0x003D, 0x0008, 0}, 
//------------------ Set GRAM area ---------------//     
    {0x0050, 0x0000, 0},    
    {0x0051, 0x00EF, 0},    
    {0x0052, 0x0000, 0},    
    {0x0053, 0x013F, 0},    
    {0x0060, 0xA700, 0},    
    {0x0061, 0x0001, 0},   
    {0x006A, 0x0000, 0},    // set scrolling line 
//-------------- Partial Display Control ---------// 
    {0x0080, 0x0000, 0}, 
    {0x0081, 0x0000, 0}, 
    {0x0082, 0x0000, 0}, 
    {0x0083, 0x0000, 0}, 
    {0x0084, 0x0000, 0}, 
    {0x0085, 0x0000, 0}, 

//-------------- Panel Control -------------------// 
    {0x0090, 0x0010, 0}, 
    {0x0092, 0x0600, 0}, 

    {0x0007, 0x0133, 0},    // 262K color and display ON 
};

struct reg_value ili9325_reg_windows[] = {
//************* Start Initial Sequence **********//     
    {0x0050, 0x0000, 0},    
    {0x0051, 0x00EF, 0},    
    {0x0052, 0x0000, 0},    
    {0x0053, 0x013F, 0},    
    {0x0020, 0x0000, 0},     
    {0x0021, 0x0000, 0},           
};

struct reg_value ili9325_reg_standby[] = {
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

 int cs_gpio = 0;
 
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
volatile int    LCD_POWERON=0;
volatile u32 sleep_dma_flag=0;;
extern atomic_t global_op_count ;
static void lcd_ili9325_init(struct pxa910fb_info *fbi)
{
	int num = ARRAY_SIZE(ili9325_reg);
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
		high =( ili9325_reg[i].reg &0xFF00)<<2;	
		low =( ili9325_reg[i].reg &0xFF)<<1;		
		reg = high|low | 0x01000000;			
		
		high =( ili9325_reg[i].value &0xFF00)<<2;		
		low =( ili9325_reg[i].value &0xFF)<<1;		
		value = high|low | 0x81000000;		

		//gpio_set_value(cs_gpio, 0);
		udelay(1);
		pxa910fb_smart_cmd_simple(fbi, &reg, 1);/*write cmd*/
		udelay(1);

		pxa910fb_smart_cmd_simple(fbi, &value, 1);/*write data*/
		//gpio_set_value(cs_gpio, 1);
		udelay(1);
		if(ili9325_reg[i].msleep)
			msleep(ili9325_reg[i].msleep);
		i++;
	}
	
	//writel(0x333305C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	writel(0x222204C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	writel(dma_flag|sleep_dma_flag, fbi->reg_base + LCD_SPU_DMA_CTRL0);

       LCD_POWERON=1;
	fbi->active=1;
       mutex_unlock(&fbi->lcd_init);
}
/*
void gpio_reconfgiure()
{
gpio_free(cs_gpio);
mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
}
*/
volatile int dma_tiger=0;
static void lcd_ili9325_trans_dma(struct pxa910fb_info *fbi)
{
	unsigned int reg, value;
	unsigned int high, low;
	int i = 0;
	int delay = 0;
	u32 dma_flag;
	
	unsigned long flags;
	int num =6 /*ARRAY_SIZE(ili9325_reg_windows)*/;
	//static DEFINE_SPINLOCK(trigger_lock);
	
	mutex_lock(&fbi->lcd_init);
	dma_flag = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
	/* disable all DMA to send commands */
	writel(dma_flag & (~(CFG_GRA_ENA_MASK | CFG_DMA_ENA_MASK)), fbi->reg_base + LCD_SPU_DMA_CTRL0);
	//writel(0x222205D1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
        writel(0x222204D3, fbi->reg_base + LCD_SPU_SMPN_CTRL);  
        #if   1
        dma_tiger=1;
	while(num--)
	{
		high =( ili9325_reg_windows[i].reg &0xFF00)<<2;	
		low =( ili9325_reg_windows[i].reg &0xFF)<<1;		
		reg = high|low | 0x01000000;			
		
		high =( ili9325_reg_windows[i].value &0xFF00)<<2;		
		low =( ili9325_reg_windows[i].value &0xFF)<<1;		
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
		//for(delay=0;delay<10;delay++);
		//if(ili9325_reg_windows[i].msleep)
			//msleep(ili9325_reg_windows[i].msleep);
		i++;
	}
	//udelay(1);
	//gpio_set_value(cs_gpio, 0);
        #endif
	low = 0x22<<1;		
	reg = low | 0x01000000;		
	pxa910fb_smart_cmd_simple(fbi, &reg, 1);/*write cmd*/

	//writel(0x222204C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	//writel(dma_flag/*| CFG_ARBFAST_ENA(1) | CFG_NOBLENDING(1)*/, fbi->reg_base + LCD_SPU_DMA_CTRL0);
     	writel((dma_flag |CFG_GRA_ENA_MASK ), fbi->reg_base + LCD_SPU_DMA_CTRL0);	

	smart_trigger_vsync(fbi);
	//mdelay(15);
	msleep(20);
        writel(0x222204C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	//gpio_set_value(cs_gpio, 1);
	//spin_unlock_irqrestore(&trigger_lock,flags );
        mutex_unlock(&fbi->lcd_init);
	
}

static void lcd_ili9325_standby(struct pxa910fb_info *fbi)
{
	int num = ARRAY_SIZE(ili9325_reg_standby);
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
		high =( ili9325_reg_standby[i].reg &0xFF00)<<2;	
		low =( ili9325_reg_standby[i].reg &0xFF)<<1;		
		reg = high|low | 0x01000000;			
		
		high =( ili9325_reg_standby[i].value &0xFF00)<<2;		
		low =( ili9325_reg_standby[i].value &0xFF)<<1;		
		value = high|low | 0x81000000;		

		gpio_set_value(cs_gpio, 0);
		//udelay(1);
		pxa910fb_smart_cmd_simple(fbi, &reg, 1);/*write cmd*/
		//udelay(1);

		pxa910fb_smart_cmd_simple(fbi, &value, 1);/*write data*/
		gpio_set_value(cs_gpio, 1);
		//udelay(1);
		if(ili9325_reg_standby[i].msleep)
			msleep(ili9325_reg_standby[i].msleep);
		i++;
	}
	writel(0x333305C1, fbi->reg_base + LCD_SPU_SMPN_CTRL);
	writel(dma_flag, fbi->reg_base + LCD_SPU_DMA_CTRL0);
}

static unsigned int is_ili9325_lcd(void)
{	


	unsigned int reg, value;
	unsigned int high, low;
	
	u32 dma_flag;
	void * reg_base;
	reg_base = ioremap_nocache(0xd420b000, 0x1ec);


	dma_flag = readl(reg_base + LCD_SPU_DMA_CTRL0);
	/* disable all DMA to send commands */
	writel(dma_flag & (~(CFG_GRA_ENA_MASK | CFG_DMA_ENA_MASK)), reg_base + LCD_SPU_DMA_CTRL0);
	writel(0x555505D1, reg_base + LCD_SPU_SMPN_CTRL);	
	//gpio_set_value(cs_gpio, 0);//low
	reg = 0x01000000;							/*write cmd*/	
	writel(reg, reg_base + LCD_SPU_SLV_PORT);
	udelay(1);
	reg = 0x82000000;							/*read data*/	
	writel(reg, reg_base + LCD_SPU_SLV_PORT);
	udelay(1);
	value = readl(reg_base + LCD_SPU_ISA_RSDATA);
	high =0xFF00 & value>>2;
	low = 0xFF & value >>1;
	value = low | high;
	//gpio_set_value(cs_gpio, 1);//low
	writel(0x555505C1, reg_base + LCD_SPU_SMPN_CTRL);	
	writel(dma_flag, reg_base + LCD_SPU_DMA_CTRL0);

	 iounmap(reg_base);
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

static int ili9325_lcd_power(struct pxa910fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int err = 0;
	mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
	int static poweron=0;
	printk("Enter %s,on=%dd\n", __FUNCTION__,on);
	
	//pxa910_setup_gpio();
	writel(0x4000000A, fbi->reg_base + LCD_CFG_SCLK_DIV);
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
			gpio_set_value(spi_gpio_reset, 1);
			msleep(10);
			gpio_set_value(spi_gpio_reset, 0);
			msleep(60);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(100);
			gpio_free(spi_gpio_reset);
		}
		#if  0
                if(LCD_ILI9325_ID !=is_ili9325_lcd(fbi))
			printk("LCD9325 Init fault!!!!!!!!\n");
		else
			printk("LCD9325 Init success!!!!!!!!\n");
		#endif
		lcd_ili9325_init(fbi);
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

static struct fb_videomode ili9325_video_modes[] = {
	/* ili9325 HVGA mode info */
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
        .num_modes              = ARRAY_SIZE(ili9325_video_modes),
        .modes          	= ili9325_video_modes,
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
	.pxa910fb_lcd_power	= ili9325_lcd_power,
	.display_interface	= SMART_PANEL,
	.lcd_xxx_trans_dma = lcd_ili9325_trans_dma,
};

static struct pxa910fb_mach_info u810_smart_lcd_ovly_info = {
        .id                     = "Ovly",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = ARRAY_SIZE(ili9325_video_modes),
        .modes                  = ili9325_video_modes,
        .pix_fmt                = PIX_FMT_RGB565,
        .dumb_mode              = DUMB_MODE_RGB666,
        .panel_rgb_reverse_lanes= 0,
        .panel_rbswap		= 1,
        .enable_lcd             = 1,
        .spi_gpio_cs            = -1,
	.spi_gpio_reset		= mfp_to_gpio(MFP_PIN_GPIO106),
	.max_fb_size		= 1280 * 720 * 4,
	.pxa910fb_lcd_power	= ili9325_lcd_power,
	.display_interface	= SMART_PANEL,
};

void u810_add_lcd_ili9325(void)
{
    mfp_config(ARRAY_AND_SIZE(tpo_lcd_gpio_pin_config));
     if(0x47==is_ili9325_lcd())
     {
        U802_lcd_type=1;
	 u810_add_lcd_hx8347();
     }
     else
     {
	struct pxa910fb_mach_info *fb = &u810_smart_lcd_info, *ovly = &u810_smart_lcd_ovly_info;
	//pxa910_setup_gpio();
	U802_lcd_type=2;
	pxa910_add_fb(fb);
	pxa910_add_fb_ovly(ovly);}
} 
