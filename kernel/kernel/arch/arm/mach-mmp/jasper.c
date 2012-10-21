/*
 *  linux/arch/arm/mach-mmp/jasper.c
 *
 *  Support for the Marvell Jasper Development Platform.
 *
 *  Copyright (C) 2009-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/max8649.h>
#include <linux/mfd/max8925.h>
#include <linux/usb/otg.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp2.h>
#include <mach/irqs.h>
#include <mach/regs-mpmu.h>
#include <mach/tc358762.h>
#include <plat/pxa27x_keypad.h>

#include <plat/generic.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>

#include "common.h"

#include <mach/regs-icu.h>

static unsigned long jasper_pin_config[] __initdata = {
	/* UART1 */
	GPIO29_UART1_RXD,
	GPIO30_UART1_TXD,

	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* I2C */
	TWSI1_SCL,
	TWSI1_SDA,

	/* TWSI5 */
	GPIO99_TWSI5_SCL,
	GPIO100_TWSI5_SDA,

	/* TWSI6 */
	GPIO97_TWSI6_SCL,
	GPIO98_TWSI6_SDA,

	/*Keypad*/
	GPIO00_KP_MKIN0,
	GPIO01_KP_MKOUT0,
	GPIO02_KP_MKIN1,
	GPIO03_KP_MKOUT1,
	GPIO04_KP_MKIN2,
	GPIO05_KP_MKOUT2,

	/* DFI */
	GPIO168_DFI_D0,
	GPIO167_DFI_D1,
	GPIO166_DFI_D2,
	GPIO165_DFI_D3,
	GPIO107_DFI_D4,
	GPIO106_DFI_D5,
	GPIO105_DFI_D6,
	GPIO104_DFI_D7,
	GPIO111_DFI_D8,
	GPIO164_DFI_D9,
	GPIO163_DFI_D10,
	GPIO162_DFI_D11,
	GPIO161_DFI_D12,
	GPIO110_DFI_D13,
	GPIO109_DFI_D14,
	GPIO108_DFI_D15,
	GPIO143_ND_nCS0,
	GPIO144_ND_nCS1,
	GPIO147_ND_nWE,
	GPIO148_ND_nRE,
	GPIO150_ND_ALE,
	GPIO149_ND_CLE,
	GPIO112_ND_RDY0,
	GPIO160_ND_RDY1,
	GPIO154_SMC_IRQ,

	/* OTG */
	GPIO82_GPIO82,

	/* LCD */
	GPIO94_SPI_DCLK,
	GPIO95_SPI_CS0,
	GPIO96_SPI_DIN,
	GPIO83_LCD_RST,
	GPIO114_MN_CLK_OUT,
	CLK_REQ,

	/* PMIC */
	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,
	GPIO101_GPIO101 | MFP_PULL_HIGH,
};

#if defined(CONFIG_TC358762)
extern int tc358762_write32(u16 reg, u32 val);
extern int tc358762_write16(u16 reg, u16 val);
extern int tc358762_read16(u16 reg, u16 *pval);
extern int tc358762_read32(u16 reg, u32 *pval);

static int chip_reset(void)
{
	int gpio = mfp_to_gpio(GPIO83_LCD_RST);

	if (gpio_request(gpio, "lcd reset gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}

	gpio_direction_output(gpio, 0);
	mdelay(1);
	gpio_direction_output(gpio, 1);
	mdelay(1);

	gpio_free(gpio);

	return 0;
}

static int tc358762_init(void)
{
	u32 reg;
	u32 *vaddr;

	int gpio = mfp_to_gpio(GPIO89_LCD_5V_BIAS);
	if (gpio_request(gpio, "lcd bias 5v gpio")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio);
		return -1;
	}
	gpio_direction_output(gpio, 1);
	mdelay(1);
	gpio_free(gpio);

	/*Enable the M/N_CLK_OUT*/
	reg = __raw_readl(MPMU_ACGR);
	reg |= 1<<9;
	__raw_writel(reg, MPMU_ACGR);

	/* Set the M/N value and re-enable the clock gate register*/
	__raw_writel(0x00020001, MPMU_GPCR);

	vaddr = ioremap(0xd401e160, 4);	
	reg = readl(vaddr);
	reg |= 0x4000;
	writel(reg, vaddr);
	iounmap(vaddr);

	chip_reset();

	return 0;
}

#define TC358762_CHIP_ID        0x4A0
static void dsi_set_tc358762(struct pxa168fb_info *fbi)
{
	int status;
	u16 chip_id;
	struct fb_var_screeninfo *var = &(fbi->fb_info->var);
	/* 
	 * tc358762 should have been powered on
	 * tc358762_write32(0x47c, 0x0);
	 */
	status = tc358762_read16(TC358762_CHIP_ID, &chip_id);
	if ((status < 0) || (chip_id != 0x6200)) {
		printk(KERN_WARNING "tc358762 unavailable!\n");
		return;
	} else {
		printk(KERN_INFO "tc358762(chip id:0x%02x) detected.\n", chip_id);
	}

	tc358762_write16(0x0210, 0x7); /* 2 Lane*/
	mdelay(10);

	/* Set PLL to 117 MHz */
	tc358762_write32(0x0470, 0xb8230000);	/* FIXME */
	mdelay(10);

	/* Configure the Pixel Clock Divider (for 32 MHz pixel clock) */
	tc358762_write32(0x0464, 0x400);
	mdelay(10);

	/* Start the DSI RX function */
	tc358762_write32(0x0204, 0x1);
	mdelay(10);
	/* Analog Timer register programming (lane 0) */
	tc358762_write32(0x0144, 0x0);/*Set to zero (per bring up procedure doc)*/
	mdelay(10);

	/* Analog Timer register programming (lane 1) */
	tc358762_write32(0x0148, 0x0);/*Set to zero (per bring up procedure doc)*/
	mdelay(10);

	/*Set asserting period for the duration between LP-00 detect and
	  High Speed data reception for each lane. (This is for LANE 0)
	  Set between 85ns + 6*UI and 145 ns + 10*UI based on HSBCLK*/
	tc358762_write32(0x0164, 0x0a);
	mdelay(10);

	/*Set asserting period for the duration between LP-00 detect and
	  High Speed data reception for each lane. (This is for LANE 1)
	  Set between 85ns + 6*UI and 145 ns + 10*UI based on HSBCLK*/
	tc358762_write32(0x0168, 0x0a);		/* FIXME */
	mdelay(10);

	/*Setup DPI register for the color depth of the LCD panel being used - 24bpp output*/
	tc358762_write32(0x0420, 0x150);
	mdelay(10);

	/*Setup SPI control register
	  The top 16 bits are the SPIRCMR register
	  The bottom 16 bits are the SPICTRL register*/
	tc358762_write32(0x0450, 0x3FF0000);/*Address of SPIRCMR/SPICTRL registers*/
	mdelay(10);

	/*Setup specific Burst or Non Burst mode
	  No code setup for now as these registers are not applicable to non-burst mode*/
	/*Start the DSI-RX PPI function*/
	tc358762_write32(0x0104, 0x1);/*Address of STARTPPI registers*/
	mdelay(10);

	/*Set up platform specific timings for high speed mode operation*/
	/*CLW_DLYCNTRL = 20pS*/
	tc358762_write32(0x0020, 0x2102);/*Address of CLW_DPHYCONTRX register*/
	mdelay(10);

	/*D0W_DLYCNTRL = 10pS*/
	tc358762_write32(0x0024, 0x2002);/*Address of D0W_DPHYCONTRX register*/
	mdelay(10);

	/*D1W_DLYCNTRL = 10pS*/
	tc358762_write32(0x0028, 0x2002);/*Address of D1W_DPHYCONTRX register*/
	mdelay(10);

	/*Start the DSI RX function (docs imply this needs to be set again)*/
	tc358762_write32(0x0204, 0x1);/*Address of STARTDSI register*/
	mdelay(10);

	/*EXTRA TOSHIBA CODE*/

	/* FIXME - MUST write as 32bit rather than 16bit. otherwise bit[1] can't be set. the value will be 0x150 */
	tc358762_write32(0x0420, 0x152);
	mdelay(10);

	tc358762_write16(0x0424, var->hsync_len);/*HSR and HBFR*/
	mdelay(10);
	tc358762_write16(0x0426, var->left_margin);/*HSR and HBFR*/
	mdelay(10);
	tc358762_write16(0x0428, var->xres);/*HDISPR*/
	mdelay(10);

	tc358762_write16(0x042A, var->right_margin);/*HFPR*/
	mdelay(10);

	tc358762_write16(0x042C, var->vsync_len);/*VSR and VBFR*/
	mdelay(10);

	tc358762_write16(0x042E, var->upper_margin);/*VSR and VBFR*/
	mdelay(10);

	tc358762_write16(0x0430, var->yres);/*VDISPR and VFPR*/
	mdelay(10);

	tc358762_write16(0x0432, var->lower_margin);/*VDISPR and VFPR*/
	mdelay(10);

}

static void dsi_dump_tc358762(struct pxa168fb_info *fbi)
{
	u16 val_16;
	u32 val_32;

	tc358762_read32(0x47c, &val_32);
	printk("%s: tc358762 - 0x47c = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0210, &val_32);
	printk("%s: tc358762 - 0x0210 = 0x%x\n", __func__, val_32);

	tc358762_read32(0x0470, &val_32);
	printk("%s: tc358762 - 0x0470 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0464, &val_32);
	printk("%s: tc358762 - 0x0464 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0204, &val_32);	/* should be 16bit according to spec */
	printk("%s: tc358762 - 0x0204 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0144, &val_32);
	printk("%s: tc358762 - 0x0144 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0148, &val_32);
	printk("%s: tc358762 - 0x0148 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0164, &val_32);
	printk("%s: tc358762 - 0x0164 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0168, &val_32);
	printk("%s: tc358762 - 0x0168 = 0x%x\n", __func__, val_32);

	tc358762_read32(0x0420, &val_32);	/* should be 16bit according to spec */
	printk("%s: tc358762 - 0x0420 = 0x%x\n", __func__, val_32);

	tc358762_read32(0x0450, &val_32);
	printk("%s: tc358762 - 0x0450 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0104, &val_32);
	printk("%s: tc358762 - 0x0104 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0020, &val_32);
	printk("%s: tc358762 - 0x0020 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0024, &val_32);
	printk("%s: tc358762 - 0x0024 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0028, &val_32);
	printk("%s: tc358762 - 0x0028 = 0x%x\n", __func__, val_32);
	tc358762_read32(0x0204, &val_32);
	printk("%s: tc358762 - 0x0204 = 0x%x\n", __func__, val_32);

	tc358762_read32(0x0420, &val_32);
	printk("%s: tc358762 - 0x0420 = 0x%x\n", __func__, val_32);

	tc358762_read16(0x0424, &val_16);
	printk("%s: tc358762 - 0x0424 = 0x%x\n", __func__, val_16);
	tc358762_read16(0x0426, &val_16);
	printk("%s: tc358762 - 0x0426 = 0x%x\n", __func__, val_16);
	tc358762_read16(0x0428, &val_16);
	printk("%s: tc358762 - 0x0428 = 0x%x\n", __func__, val_16);
	tc358762_read16(0x042A, &val_16);
	printk("%s: tc358762 - 0x042A = 0x%x\n", __func__, val_16);
	tc358762_read16(0x042C, &val_16);
	printk("%s: tc358762 - 0x042C = 0x%x\n", __func__, val_16);
	tc358762_read16(0x042E, &val_16);
	printk("%s: tc358762 - 0x042E = 0x%x\n", __func__, val_16);
	tc358762_read16(0x0430, &val_16);
	printk("%s: tc358762 - 0x0430 = 0x%x\n", __func__, val_16);
	tc358762_read16(0x0432, &val_16);
	printk("%s: tc358762 - 0x0432 = 0x%x\n", __func__, val_16);
}

static struct tc358762_platform_data tc358762_data = {
	.platform_init = tc358762_init,
};
#endif

#ifdef CONFIG_FB_SECOND_PANEL_DSI
unsigned char dsi_cmd_sharp [][30] = {
	{ 9, 0,0,0,0, 	0xF1,   0x5A,   0x5A,   0x00,   0x00 },
	{ 9, 0,0,0,0, 	0xFC,   0x5A,   0x5A,   0x00,   0x00 },
	{ 10, 0,0,0,0, 	0xB7,   0x00,   0x11,   0x11,   0x00,   0x00 },
	{ 9, 0,0,0,0, 	0xB8,   0x2D,   0x21,   0x00,   0x00 },
	{ 9, 0,0,0,0, 	0xB9,   0x00,   0x06,   0x00,   0x00 },
	{ 11, 0,0,0,0, 	0x2A,   0x00,   0x00,   0x01,   0xDF,   0x00,   0x00 },
	{ 11, 0,0,0,0, 	0x2B,   0x00,   0x00,   0x02,   0x7F,   0x00,   0x00 },
	{ 0x04, 0x05,   0x11,   0x00,   0x00 },
	{ 0x01 },

	{ 17, 0,0,0,0,  0xF4,   0x00,   0x23,   0x00,   0x64,   0x5C,   0x00,   0x64,
		0x5C,   0x00,   0x00,   0x00,   0x00 },
	{ 17, 0,0,0,0,  0xF5,   0x00,   0x00,   0x0E,   0x00,   0x04,   0x02,   0x03,
		0x03,   0x03,   0x03,   0x00,   0x00 },
	{ 10, 0,0,0,0,  0xEE,   0x32,   0x29,   0x00,   0x00,   0x00 },
	{ 16, 0,0,0,0,  0xF2,   0x00,   0x30,   0x88,   0x88,   0x57,   0x57,   0x10,
		0x00,   0x04,   0x00,   0x00 },
	{ 19, 0,0,0,0,  0xF3,   0x00,   0x10,   0x25,   0x01,   0x2D,   0x2D,   0x24,
		0x2D,   0x10,   0x12,   0x12,   0x73,   0x00,   0x00 },
	{ 13, 0,0,0,0,  0xF6,   0x21,   0xAE,   0xBF,   0x62,   0x22,   0x62,   0x00,
		0x00 },
	{ 24, 0,0,0,0,  0xF7,   0x00,   0x01,   0x00,   0xF2,   0x0A,   0x0A,   0x0A,
		0x30,   0x0A,   0x00,   0x0F,   0x00,   0x0F,   0x00,   0x4B,	0x00,
		0x8C,   0x00,   0x00 },
	{ 24, 0,0,0,0,  0xF8,   0x00,   0x01,   0x00,   0xF2,   0x0A,   0x0A,   0x0A,
		0x30,   0x0A,   0x00,   0x0F,   0x00,   0x0F,   0x00,   0x4B,   0x00,
		0x8C,   0x00,   0x00 },
	{ 29, 0,0,0,0,  0xF9,   0x11,   0x10,   0x0F,   0x00,   0x01,   0x02,   0x04,
		0x05,   0x08,   0x00,   0x0A,   0x00,   0x00,   0x00,   0x0F,   0x10,
		0x11,   0x00,   0x00,   0xC3,   0xFF,   0x7F,   0x00,   0x00 },
	{ 0x02 },

	{ 0x04, 0x05,   0x29,   0x00,   0x00 },
	{ 0xFF },
};

static void dsi_set_sharp(struct pxa168fb_info *fbi)
{
	int i;
	for (i = 0; i < 21; i++) {
		if (dsi_cmd_sharp[i][0] == 0x01) {
			msleep(5);
			continue;
		} else if (dsi_cmd_sharp[i][0] == 0x02) {
			msleep(120);
			continue;
		} else if (dsi_cmd_sharp[i][0] == 0xFF) {
			break;
		} else {
			pxa168fb_dsi_send(fbi, dsi_cmd_sharp[i]);
		}
	}
}
#endif

static u16 tpo_spi_cmdon[] = {
	0x080F,
	0x0C5F,
	0x1017,
	0x1420,
	0x1808,
	0x1c20,
	0x2020,
	0x2420,
	0x2820,
	0x2c20,
	0x3020,
	0x3420,
	0x3810,
	0x3c10,
	0x4010,
	0x4415,
	0x48aa,
	0x4cff,
	0x5086,
	0x548d,
	0x58d4,
	0x5cfb,
	0x602e,
	0x645a,
	0x6889,
	0x6cfe,
	0x705a,
	0x749b,
	0x78c5,
	0x7cff,
	0x80f0,
	0x84f0,
	0x8808,
};

static u16 tpo_spi_cmdoff[] = {
	0x0c5e,		//standby
};

static void tpo_lcd_power(struct pxa168fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int err = 0;
	if (on) {
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
		pxa168fb_spi_send(fbi, tpo_spi_cmdon, ARRAY_SIZE(tpo_spi_cmdon), spi_gpio_cs);
	} else
		pxa168fb_spi_send(fbi, tpo_spi_cmdoff, ARRAY_SIZE(tpo_spi_cmdoff), spi_gpio_cs);
}

/* SPI Control Register. */
#define     CFG_SCLKCNT(div)                    (div<<24)  /* 0xFF~0x2 */
#define     CFG_RXBITS(rx)                      ((rx - 1)<<16)   /* 0x1F~0x1 */
#define     CFG_TXBITS(tx)                      ((tx - 1)<<8)    /* 0x1F~0x1, 0x1: 2bits ... 0x1F: 32bits */
#define     CFG_SPI_ENA(spi)                    (spi<<3)
#define     CFG_SPI_SEL(spi)                    (spi<<2)   /* 1: port1; 0: port0 */
#define     CFG_SPI_3W4WB(wire)                 (wire<<1)  /* 1: 3-wire; 0: 4-wire */

static struct fb_videomode video_modes[] = {
	[0] = {
		.pixclock	= 30120,
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.hsync_len	= 4,
		.left_margin	= 212,
		.right_margin	= 40,
		.vsync_len	= 4,
		.upper_margin	= 31,
		.lower_margin	= 10,
		.sync		= FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
	},
};

static struct pxa168fb_mach_info mmp2_mipi_lcd_info __initdata = {
	.id			= "GFX Layer",
	.num_modes		= ARRAY_SIZE(video_modes),
	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB565,
	/* 
	 * don't care about io_pin_allocation_mode and dumb_mode
	 * since the panel is hard connected with lcd panel path and dsi1 output
	 */
	.panel_rgb_reverse_lanes= 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.invert_pixclock        = 0,
	.panel_rbswap           = 0,
	.active			= 1,
	.enable_lcd             = 1,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset         = -1,
	.spi_ctrl               = CFG_SCLKCNT(2) | CFG_TXBITS(16) | CFG_SPI_SEL(0) | CFG_SPI_3W4WB(1) | CFG_SPI_ENA(1),
	.max_fb_size		= 800 * 480 * 8 + 4096,
	.display_interface	= DSI2DPI,
	.dsi2dpi_set		= dsi_set_tc358762,
	.pxa168fb_lcd_power     = tpo_lcd_power,
	.vdma_enable		= 1,
};

static struct pxa168fb_mach_info mmp2_mipi_lcd_ovly_info __initdata = {
	.id			= "Video Layer",
	.num_modes		= ARRAY_SIZE(video_modes),
	.modes			= video_modes,
	.pix_fmt		= PIX_FMT_RGB565,
	.panel_rgb_reverse_lanes= 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.invert_pixclock        = 0,
	.panel_rbswap           = 0,
	.active			= 1,
	.enable_lcd             = 1,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset         = -1,
	.max_fb_size            = 800 * 480 * 8 + 4096,
};

#ifdef CONFIG_FB_SECOND_PANEL_DSI
static struct fb_videomode video_modes_2[] = {
	[0] = {
		.pixclock	= 49622,
		.refresh	= 60,
		.xres		= 480,
		.yres		= 640,
		.hsync_len	= 8,
		.left_margin	= 8,
		.right_margin	= 16,
		.vsync_len	= 2,
		.upper_margin	= 6,
		.lower_margin	= 8,
		.sync		= FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
	},
};

static struct pxa168fb_mach_info mmp2_mipi_lcd2_info __initdata = {
	.id			= "GFX Layer - 2",
	.num_modes		= ARRAY_SIZE(video_modes_2),
	.modes			= video_modes_2,
	.pix_fmt		= PIX_FMT_RGB565,
	/* 
	 * don't care about io_pin_allocation_mode and dumb_mode
	 * since the panel is hard connected with lcd tv path and dsi2 output
	 */
	.panel_rgb_reverse_lanes= 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.invert_pixclock        = 0,
	.panel_rbswap           = 0,
	.active			= 1,
	.enable_lcd             = 1,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset         = -1,
	.max_fb_size		= 640 * 480 * 8 + 4096,
	.display_interface	= DSI,
	.dsi_set		= dsi_set_sharp,
	.vdma_enable		= 1,
};

static struct pxa168fb_mach_info mmp2_mipi_lcd2_ovly_info __initdata = {
	.id			= "Video Layer - 2",
	.num_modes		= ARRAY_SIZE(video_modes_2),
	.modes			= video_modes_2,
	.pix_fmt		= PIX_FMT_RGB565,
	.panel_rgb_reverse_lanes= 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.invert_pixclock        = 0,
	.panel_rbswap           = 0,
	.active			= 1,
	.enable_lcd             = 1,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset         = -1,
	.max_fb_size            = 640 * 480 * 8 + 4096,
};
#endif

#ifdef CONFIG_FB_SECOND_PANEL_HDMI
static struct fb_videomode tv_video_modes[] = {
	[0] = {
		.pixclock	= 6734,
		.refresh	= 60,
		.xres		= 1920,
		.yres		= 1080,
		.hsync_len	= 44,
		.left_margin	= 88,
		.right_margin	= 148,
		.vsync_len	= 5,
		.upper_margin	= 4,
		.lower_margin	= 36,
		.sync		= FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
	},
	[1] = {
		.pixclock	= 13149,
		.refresh	= 60,
		.xres		= 1280,
		.yres		= 720,
		.hsync_len	= 40,
		.left_margin	= 110,
		.right_margin	= 220,
		.vsync_len	= 5,
		.upper_margin	= 5,
		.lower_margin	= 20,
		.sync		= FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
	},
};

static struct pxa168fb_mach_info mmp2_tv_hdmi_info __initdata = {
	.id			= "GFX Layer - TV",
	.num_modes		= ARRAY_SIZE(tv_video_modes),
	.modes			= tv_video_modes,
	.pix_fmt		= PIX_FMT_RGB565,
	/* 
	 * don't care about io_pin_allocation_mode and dumb_mode
	 * since the hdmi monitor is hard connected with lcd tv path and hdmi output
	 */
	.panel_rgb_reverse_lanes= 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.invert_pixclock        = 0,
	.panel_rbswap           = 1,
	.active			= 1,
	.enable_lcd             = 1,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset         = -1,
	.max_fb_size		= 1920 * 1080 * 4 + 4096,
	.vdma_enable		= 1,
};

static struct pxa168fb_mach_info mmp2_tv_hdmi_ovly_info __initdata = {
	.id			= "Video Layer - TV",
	.num_modes		= ARRAY_SIZE(tv_video_modes),
	.modes			= tv_video_modes,
	.pix_fmt		= PIX_FMT_RGB565,
	.panel_rgb_reverse_lanes= 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.invert_pixclock        = 0,
	.panel_rbswap           = 1,
	.active			= 1,
	.enable_lcd             = 1,
	.spi_gpio_cs            = -1,
	.spi_gpio_reset         = -1,
	.max_fb_size            = 1920 * 1080 * 4 + 4096,
};
#endif

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};

static struct regulator_consumer_supply max8649_supply[] = {
	REGULATOR_SUPPLY("v_core", NULL),
};

static struct regulator_init_data max8649_init_data = {
	.constraints	= {
		.name		= "SD1",
		.min_uV		= 750000,
		.max_uV		= 1380000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &max8649_supply[0],
};

static struct max8649_platform_data jasper_max8649_info = {
	.mode		= 2,	/* VID1 = 1, VID0 = 0 */
	.extclk		= 0,
	.ramp_timing	= MAX8649_RAMP_32MV,
	.regulator	= &max8649_init_data,
};

static struct regulator_consumer_supply regulator_supply[] = {
	[MAX8925_ID_SD1]	= REGULATOR_SUPPLY("v_sd1", NULL),
	[MAX8925_ID_SD2]	= REGULATOR_SUPPLY("v_sd2", NULL),
	[MAX8925_ID_SD3]	= REGULATOR_SUPPLY("v_sd3", NULL),
	[MAX8925_ID_LDO1]	= REGULATOR_SUPPLY("v_ldo1", NULL),
	[MAX8925_ID_LDO2]	= REGULATOR_SUPPLY("v_ldo2", NULL),
	[MAX8925_ID_LDO3]	= REGULATOR_SUPPLY("v_ldo3", NULL),
	[MAX8925_ID_LDO4]	= REGULATOR_SUPPLY("v_ldo4", NULL),
	[MAX8925_ID_LDO5]	= REGULATOR_SUPPLY("v_ldo5", NULL),
	[MAX8925_ID_LDO6]	= REGULATOR_SUPPLY("v_ldo6", NULL),
	[MAX8925_ID_LDO7]	= REGULATOR_SUPPLY("v_ldo7", NULL),
	[MAX8925_ID_LDO8]	= REGULATOR_SUPPLY("v_ldo8", NULL),
	[MAX8925_ID_LDO9]	= REGULATOR_SUPPLY("v_ldo9", NULL),
	[MAX8925_ID_LDO10]	= REGULATOR_SUPPLY("v_ldo10", NULL),
	[MAX8925_ID_LDO11]	= REGULATOR_SUPPLY("v_ldo11", NULL),
	[MAX8925_ID_LDO12]	= REGULATOR_SUPPLY("v_ldo12", NULL),
	[MAX8925_ID_LDO13]	= REGULATOR_SUPPLY("v_ldo13", NULL),
	[MAX8925_ID_LDO14]	= REGULATOR_SUPPLY("v_ldo14", NULL),
	[MAX8925_ID_LDO15]	= REGULATOR_SUPPLY("v_ldo15", NULL),
	[MAX8925_ID_LDO16]	= REGULATOR_SUPPLY("v_ldo16", NULL),
	[MAX8925_ID_LDO17]	= REGULATOR_SUPPLY("v_ldo17", NULL),
	[MAX8925_ID_LDO18]	= REGULATOR_SUPPLY("v_ldo18", NULL),
	[MAX8925_ID_LDO19]	= REGULATOR_SUPPLY("v_ldo19", NULL),
	[MAX8925_ID_LDO20]	= REGULATOR_SUPPLY("v_ldo20", NULL),
};

#define REG_INIT(_name, _min, _max, _always, _boot)		\
{								\
	.constraints = {					\
		.name		= __stringify(_name),		\
		.min_uV		= _min,				\
		.max_uV		= _max,				\
		.always_on	= _always,			\
		.boot_on	= _boot,			\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,	\
	},							\
	.num_consumer_supplies	= 1,				\
	.consumer_supplies	= &regulator_supply[MAX8925_ID_##_name], \
}

static struct regulator_init_data regulator_data[] = {
	[MAX8925_ID_SD1] = REG_INIT(SD1, 637500, 1425000, 0, 0),
	[MAX8925_ID_SD2] = REG_INIT(SD2, 650000, 2225000, 1, 1),
	[MAX8925_ID_SD3] = REG_INIT(SD3, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO1] = REG_INIT(LDO1, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO2] = REG_INIT(LDO2, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO3] = REG_INIT(LDO3, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO4] = REG_INIT(LDO4, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO5] = REG_INIT(LDO5, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO6] = REG_INIT(LDO6, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO7] = REG_INIT(LDO7, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO8] = REG_INIT(LDO8, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO9] = REG_INIT(LDO9, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO10] = REG_INIT(LDO10, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO11] = REG_INIT(LDO11, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO12] = REG_INIT(LDO12, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO13] = REG_INIT(LDO13, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO14] = REG_INIT(LDO14, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO15] = REG_INIT(LDO15, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO16] = REG_INIT(LDO16, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO17] = REG_INIT(LDO17, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO18] = REG_INIT(LDO18, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO19] = REG_INIT(LDO19, 750000, 3900000, 0, 0),
	[MAX8925_ID_LDO20] = REG_INIT(LDO20, 750000, 3900000, 1, 1),
};

static struct max8925_backlight_pdata jasper_backlight_data = {
	.dual_string	= 0,
};

static int jasper_set_charger(int enable)
{
	unsigned int vbus_en;

	vbus_en = mfp_to_gpio(MFP_PIN_GPIO82);
	if (gpio_request(vbus_en, "Enable VBUS")) {
		pr_err("Request GPIO82 failure for Enabling VBUS\n");
		return -EIO;
	}
	if (enable)
		gpio_direction_output(vbus_en, 0);
	else
		gpio_direction_output(vbus_en, 1);
	msleep(1);
	gpio_free(vbus_en);
	
	return 0;
}

static struct max8925_power_pdata jasper_power_data = {
	.set_charger		= jasper_set_charger,
	.batt_detect		= 0,	/* can't detect battery by ID pin */
	.topoff_threshold	= MAX8925_TOPOFF_THR_10PER,
	.fast_charge		= MAX8925_FCHG_1000MA,
};

static struct max8925_platform_data jasper_max8925_info = {
	.backlight		= &jasper_backlight_data,
	.power			= &jasper_power_data,
	.irq_base		= IRQ_BOARD_START,

	.regulator[MAX8925_ID_SD1] = &regulator_data[MAX8925_ID_SD1],
	.regulator[MAX8925_ID_SD2] = &regulator_data[MAX8925_ID_SD2],
	.regulator[MAX8925_ID_SD3] = &regulator_data[MAX8925_ID_SD3],
	.regulator[MAX8925_ID_LDO1] = &regulator_data[MAX8925_ID_LDO1],
	.regulator[MAX8925_ID_LDO2] = &regulator_data[MAX8925_ID_LDO2],
	.regulator[MAX8925_ID_LDO3] = &regulator_data[MAX8925_ID_LDO3],
	.regulator[MAX8925_ID_LDO4] = &regulator_data[MAX8925_ID_LDO4],
	.regulator[MAX8925_ID_LDO5] = &regulator_data[MAX8925_ID_LDO5],
	.regulator[MAX8925_ID_LDO6] = &regulator_data[MAX8925_ID_LDO6],
	.regulator[MAX8925_ID_LDO7] = &regulator_data[MAX8925_ID_LDO7],
	.regulator[MAX8925_ID_LDO8] = &regulator_data[MAX8925_ID_LDO8],
	.regulator[MAX8925_ID_LDO9] = &regulator_data[MAX8925_ID_LDO9],
	.regulator[MAX8925_ID_LDO10] = &regulator_data[MAX8925_ID_LDO10],
	.regulator[MAX8925_ID_LDO11] = &regulator_data[MAX8925_ID_LDO11],
	.regulator[MAX8925_ID_LDO12] = &regulator_data[MAX8925_ID_LDO12],
	.regulator[MAX8925_ID_LDO13] = &regulator_data[MAX8925_ID_LDO13],
	.regulator[MAX8925_ID_LDO14] = &regulator_data[MAX8925_ID_LDO14],
	.regulator[MAX8925_ID_LDO15] = &regulator_data[MAX8925_ID_LDO15],
	.regulator[MAX8925_ID_LDO16] = &regulator_data[MAX8925_ID_LDO16],
	.regulator[MAX8925_ID_LDO17] = &regulator_data[MAX8925_ID_LDO17],
	.regulator[MAX8925_ID_LDO18] = &regulator_data[MAX8925_ID_LDO18],
	.regulator[MAX8925_ID_LDO19] = &regulator_data[MAX8925_ID_LDO19],
	.regulator[MAX8925_ID_LDO20] = &regulator_data[MAX8925_ID_LDO20],
};

static struct i2c_board_info jasper_twsi1_info[] = {
	[0] = {
		.type		= "max8649",
		.addr		= 0x60,
		.platform_data	= &jasper_max8649_info,
	},
	[1] = {
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP2_PMIC,
		.platform_data	= &jasper_max8925_info,
	},
};

static int gpio_usb_otg_pen = mfp_to_gpio(MFP_PIN_GPIO82);
static int jasper_vbus_status(unsigned int base)
{
	int status = VBUS_LOW;

#ifdef CONFIG_USB_OTG
	if (u2o_get(base, U2xOTGSC) & U2xOTGSC_BSV)
		status = VBUS_HIGH;
	else
		status = VBUS_LOW;
#endif
	return status;
}

static int jasper_otg_init(void)
{
	return 0;
}

static int jasper_u2o_vbus_set(int vbus_type)
{
	unsigned long flags;

	local_irq_save(flags);
	if (gpio_request(gpio_usb_otg_pen, "USB OTG Power Enable")) {
		pr_err("%s USB OTG PEN GPIO Request Failed\n", __func__);
		return -1;
	}
	pr_info("%s: enter\n", __func__);
	switch (vbus_type) {
	case VBUS_SRP:
		gpio_direction_output(gpio_usb_otg_pen, 1);
		udelay(10);
		gpio_direction_output(gpio_usb_otg_pen, 0);
		break;
	case VBUS_HIGH:
		gpio_direction_output(gpio_usb_otg_pen, 1);
		break;
	case VBUS_LOW:
		gpio_direction_output(gpio_usb_otg_pen, 0);
		break;
	}
	gpio_free(gpio_usb_otg_pen);
	local_irq_restore(flags);
	return 0;
}

static int jasper_u2o_vbus_set_ic(int function)
{
	pr_info("%s %d not implemented yet\n", __func__, function);
	return 0;
}

static struct otg_pmic_ops jasper_otg_ops = {
	.otg_vbus_init		= jasper_otg_init,
	.otg_set_vbus		= jasper_u2o_vbus_set,
	.otg_set_vbus_ic	= jasper_u2o_vbus_set_ic,
};

struct otg_pmic_ops *init_jasper_otg_ops(void)
{
	return &jasper_otg_ops;
}

static struct pxa_usb_plat_info jasper_u2o_info = {
	.phy_init	= mmp2_init_usb_phy,
	.init_pmic_ops	= init_jasper_otg_ops,
	.vbus_status	= jasper_vbus_status,
	.is_otg		= 1,
};

static struct resource u2o_resources[] = {
	/* reg base */
	[0] = {
		.start	= PXA168_U2O_REGBASE,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2o",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2ophy",
	},
	[2] = {
		.start	= IRQ_MMP2_USB_OTG,
		.end	= IRQ_MMP2_USB_OTG,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device u2o_device = {
	.name		= "pxa-u2o",
	.id		= -1,
	.dev		= {
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &jasper_u2o_info,
	},

	.num_resources	= ARRAY_SIZE(u2o_resources),
	.resource	= u2o_resources,
};

struct platform_device otg_device = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev		= {
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &jasper_u2o_info,
	},

	.num_resources	= ARRAY_SIZE(u2o_resources),
	.resource	= u2o_resources,
};

static struct mtd_partition jasper_nand_partitions[] = {
	[0] = {
		.name        = "Bootloader",
		.offset      = 0,
		.size        = 0x100000,
		.mask_flags  = MTD_WRITEABLE,
	},
	[1] = {
		.name        = "Reserve",
		.offset      = 0x100000,
		.size        = 0x080000,
	},
	[2] = {
		.name        = "Reserve",
		.offset      = 0x180000,
		.size        = 0x800000,
		.mask_flags  = MTD_WRITEABLE,
	},
	[3] = {
		.name        = "Kernel",
		.offset      = 0x980000,
		.size        = 0x300000,
		.mask_flags  = MTD_WRITEABLE,
	},
	[4] = {
		.name        = "system",
		.offset      = 0x0c80000,
		.size        = 0x7000000,
	},
	[5] = {
		.name        = "userdata",
		.offset      = 0x7c80000,
		.size        = 0x7000000,
	},
	[6] = {
		.name        = "filesystem",
		.offset      = 0x0ec80000,
		.size	     = MTDPART_SIZ_FULL,
	}
};

static struct pxa3xx_nand_platform_data jasper_nand_info;
static void __init jasper_init_flash(void)
{
	jasper_nand_info.pxa3xx_nand_mode = DMA_SUPPORT | ARBITER_ENABLE | NAKEDCMD_S,
	jasper_nand_info.parts[0] = jasper_nand_partitions;
	jasper_nand_info.nr_parts[0] = ARRAY_SIZE(jasper_nand_partitions);
	mmp2_add_nand(&jasper_nand_info);
}

static struct i2c_board_info jasper_twsi5_info[] =
{
#if defined(CONFIG_TC358762)
	{
		.type           = "tc358762",
		.addr           = 0x0b,
		.platform_data  = &tc358762_data,
	},
#endif
};

static unsigned int mmp2_matrix_key_map[] = {
	/* KEY(row, col, key_code) */
	KEY(0, 1, KEY_BACK),
	KEY(0, 2, KEY_VOLUMEUP),
	KEY(1, 0, KEY_SEARCH),
	KEY(1, 1, KEY_HOME),
	KEY(1, 2, KEY_VOLUMEDOWN),
	KEY(2, 0, KEY_CAMERA),
	KEY(2, 1, KEY_CAMERA),
};

static struct pxa27x_keypad_platform_data mmp2_keypad_info = {
	.matrix_key_rows        = 3,
	.matrix_key_cols        = 3,
	.matrix_key_map         = mmp2_matrix_key_map,
	.matrix_key_map_size    = ARRAY_SIZE(mmp2_matrix_key_map),
	.debounce_interval      = 30,
};

static void __init jasper_init(void)
{
	mfp_config(ARRAY_AND_SIZE(jasper_pin_config));

	/* on-chip devices */
	mmp2_add_uart(1);
	mmp2_add_uart(3);
	mmp2_add_rtc();
	mmp2_add_twsi(0, NULL, ARRAY_AND_SIZE(jasper_twsi1_info));
	mmp2_add_twsi(4, &pwri2c_info, ARRAY_AND_SIZE(jasper_twsi5_info));
	mmp2_add_imm();

	mmp2_add_keypad(&mmp2_keypad_info);

	/* off-chip devices */
	regulator_has_full_constraints();
	platform_device_register(&u2o_device);
	platform_device_register(&otg_device);
 	jasper_init_flash();
#ifdef CONFIG_FB_PXA168
	mmp2_add_fb(&mmp2_mipi_lcd_info);
	mmp2_add_fb_ovly(&mmp2_mipi_lcd_ovly_info);
#ifdef CONFIG_FB_SECOND_PANEL_DSI
	mmp2_add_fb_tv(&mmp2_mipi_lcd2_info);
	mmp2_add_fb_tv_ovly(&mmp2_mipi_lcd2_ovly_info);
#endif
#ifdef CONFIG_FB_SECOND_PANEL_HDMI
	mmp2_add_fb_tv(&mmp2_tv_hdmi_info);
	mmp2_add_fb_tv_ovly(&mmp2_tv_hdmi_ovly_info);
#endif
#endif
	mmp2_add_hdmi();

}

MACHINE_START(MARVELL_JASPER, "Jasper Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = mmp2_init_irq,
	.timer          = &mmp2_timer,
	.init_machine   = jasper_init,
MACHINE_END
