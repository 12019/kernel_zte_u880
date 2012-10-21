/*
 *  linux/arch/arm/mach-mmp/tavorevb.c
 *
 *  Support for the Marvell PXA910-based TavorEVB Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/smc91x.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/usb/otg.h>
#include <linux/delay.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa168.h>
#include <mach/pxa910.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/devices.h>
#include <mach/cputype.h>
#include <mach/camera.h>

#include <plat/generic.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>
#include "common.h"

/*
* tavorevb_keypad_type
* 0 - use tavorevb builtin keypad
* 1 - use zylonite keypad
* NOTE: default to 0 at the moment
*/
static int tavorevb_keypad_type = 1;

static int __init keypad_setup(char *type)
{
	tavorevb_keypad_type = simple_strtol(type, NULL, 0);
	return 1;
}
__setup("tavorevb_keypad_type", keypad_setup);

static unsigned long tavorevb_pin_config[] __initdata = {
	/* UART2 */
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,

	/* SMC */
	SM_nCS0_nCS0,
	SM_ADV_SM_ADV,
	SM_SCLK_SM_SCLK,
	SM_SCLK_SM_SCLK,
	SM_BE0_SM_BE0,
	SM_BE1_SM_BE1,

	/* IRDA */
	GPIO51_IRDA_SHDN,

	/* I2C */
	GPIO53_CI2C_SCL,
	GPIO54_CI2C_SDA,

	/* SSP1 (I2S) */
	GPIO24_SSP1_SDATA_IN,
	GPIO21_SSP1_BITCLK,
	GPIO20_SSP1_SYSCLK,
	GPIO22_SSP1_SYNC,
	GPIO23_SSP1_DATA_OUT,
	GPIO124_MN_CLK_OUT,
	GPIO123_CLK_REQ,

	/* DFI */
	DF_IO0_ND_IO0,
	DF_IO1_ND_IO1,
	DF_IO2_ND_IO2,
	DF_IO3_ND_IO3,
	DF_IO4_ND_IO4,
	DF_IO5_ND_IO5,
	DF_IO6_ND_IO6,
	DF_IO7_ND_IO7,
	DF_IO8_ND_IO8,
	DF_IO9_ND_IO9,
	DF_IO10_ND_IO10,
	DF_IO11_ND_IO11,
	DF_IO12_ND_IO12,
	DF_IO13_ND_IO13,
	DF_IO14_ND_IO14,
	DF_IO15_ND_IO15,
	DF_nCS0_SM_nCS2_nCS0,
	DF_ALE_SM_WEn_ND_ALE,
	DF_CLE_SM_OEn_ND_CLE,
	DF_WEn_DF_WEn,
	DF_REn_DF_REn,
	DF_RDY0_DF_RDY0,

	/*keypad*/
	GPIO00_KP_MKIN0,
	GPIO01_KP_MKOUT0,
	GPIO02_KP_MKIN1,
	GPIO03_KP_MKOUT1,
	GPIO04_KP_MKIN2,
	GPIO05_KP_MKOUT2,
	GPIO06_KP_MKIN3,
	GPIO07_KP_MKOUT3,
	GPIO08_KP_MKIN4,
	GPIO09_KP_MKOUT4,
	GPIO10_KP_MKIN5,
	GPIO11_KP_MKOUT5,
	GPIO12_KP_MKIN6,
	GPIO13_KP_MKOUT6,
	GPIO14_KP_MKIN7,
	GPIO15_KP_MKOUT7,
	GPIO16_KP_DKIN0,
	GPIO17_KP_DKIN1,
	GPIO18_KP_DKIN2,
	GPIO19_KP_DKIN3,

	/* LCD */
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
	GPIO95_LCD_DD10,
	GPIO96_LCD_DD11,
	GPIO97_LCD_DD12,
	GPIO98_LCD_DD13,
	GPIO100_LCD_DD14,
	GPIO101_LCD_DD15,
	GPIO102_LCD_DD16,
	//GPIO103_LCD_DD17,
	GPIO104_LCD_DD18,
	GPIO105_LCD_DD19,
	GPIO106_LCD_DD20,
	GPIO107_LCD_DD21,
	GPIO108_LCD_DD22,
	GPIO109_LCD_DD23,

	/*1wire*/
	GPIO106_1WIRE,

	/*CCIC/CAM*/
	GPIO67_CCIC_IN7,
	GPIO68_CCIC_IN6,
	GPIO69_CCIC_IN5,
	GPIO70_CCIC_IN4,
	GPIO71_CCIC_IN3,
	GPIO72_CCIC_IN2,
	GPIO73_CCIC_IN1,
	GPIO74_CCIC_IN0,
	GPIO75_CAM_HSYNC,
	GPIO76_CAM_VSYNC,
	GPIO77_CAM_MCLK,
	GPIO78_CAM_PCLK,

#if defined(CONFIG_MMC_PXA_SDH)
	/*MMC sdh*/
	MMC1_DAT7_MMC1_DAT7,
	MMC1_DAT6_MMC1_DAT6,
	MMC1_DAT5_MMC1_DAT5,
	MMC1_DAT4_MMC1_DAT4,
	MMC1_DAT3_MMC1_DAT3,
	MMC1_DAT2_MMC1_DAT2,
	MMC1_DAT1_MMC1_DAT1,
	MMC1_DAT0_MMC1_DAT0,
	MMC1_CMD_MMC1_CMD,
	MMC1_CLK_MMC1_CLK,
	MMC1_CD_MMC1_CD,
	MMC1_WP_MMC1_WP,
#endif
};

#if defined(CONFIG_PXA168_CAMERA)
/* sensor init */
static int sensor_power_onoff(int on, int sensor)
{
	/*
	 * sensor, 0, low resolution
	 * sensor, 1, high resolution
	 */

	unsigned int cam_hi_pwdn;
	unsigned int cam_lo_pwdn;

	if (cpu_is_pxa910()) {
		cam_hi_pwdn = mfp_to_gpio(MFP_PIN_GPIO80);
		cam_lo_pwdn = mfp_to_gpio(MFP_PIN_GPIO79);
	}

	if (gpio_request(cam_hi_pwdn, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_hi_pwdn);
		return -EIO;
	}

	if (gpio_request(cam_lo_pwdn, "CAM_EANBLE_LO_SENSOR")){
		gpio_free(cam_hi_pwdn);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", cam_lo_pwdn);
		return -EIO;
	}

	if (on) {
		if(sensor){
			gpio_direction_output(cam_hi_pwdn, 0);
			gpio_direction_output(cam_lo_pwdn, 1);
		}else{
			gpio_direction_output(cam_lo_pwdn, 0);
			gpio_direction_output(cam_hi_pwdn, 1);
		}
	} else {
		if (sensor)
			gpio_direction_output(cam_hi_pwdn, 1);
		else
			gpio_direction_output(cam_lo_pwdn, 1);

	}
	gpio_free(cam_hi_pwdn);
	gpio_free(cam_lo_pwdn);

	return 0;
}

static struct sensor_platform_data ov7660_sensor_data = {
        .id = SENSOR_LOW,
        .power_on = sensor_power_onoff,
};
static struct sensor_platform_data ov3640_sensor_data = {
        .id = SENSOR_HIGH,
        .power_on = sensor_power_onoff,
};
#endif

static struct smc91x_platdata tavorevb_smc91x_info = {
	.flags	= SMC91X_USE_16BIT | SMC91X_NOWAIT,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= SMC_CS1_PHYS_BASE + 0x300,
		.end	= SMC_CS1_PHYS_BASE + 0xfffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= gpio_to_irq(80),
		.end	= gpio_to_irq(80),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.dev		= {
		.platform_data = &tavorevb_smc91x_info,
	},
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

static unsigned int zylonite_matrix_key_map[] = {
		/* KEY(row, col, key_code) */
		KEY(0, 0, KEY_A), KEY(0, 1, KEY_B), KEY(0, 2, KEY_C),
		KEY(0, 5, KEY_D), KEY(1, 0, KEY_E), KEY(1, 1, KEY_F),
		KEY(1, 2, KEY_G), KEY(1, 5, KEY_H), KEY(2, 0, KEY_I),
		KEY(2, 1, KEY_J), KEY(2, 2, KEY_K), KEY(2, 5, KEY_L),
		KEY(3, 0, KEY_M), KEY(3, 1, KEY_N), KEY(3, 2, KEY_O),
		KEY(3, 5, KEY_P), KEY(5, 0, KEY_Q), KEY(5, 1, KEY_R),
		KEY(5, 2, KEY_S), KEY(5, 5, KEY_T), KEY(6, 0, KEY_U),
		KEY(6, 1, KEY_V), KEY(6, 2, KEY_W), KEY(6, 5, KEY_X),
		KEY(7, 1, KEY_Y), KEY(7, 2, KEY_Z),

		KEY(4, 4, KEY_0), KEY(1, 3, KEY_1), KEY(4, 1, KEY_2),
		KEY(1, 4, KEY_3), KEY(2, 3, KEY_4), KEY(4, 2, KEY_5),
		KEY(2, 4, KEY_6), KEY(3, 3, KEY_7), KEY(4, 3, KEY_8),
		KEY(3, 4, KEY_9),

		KEY(4, 5, KEY_SPACE),
		KEY(5, 3, KEY_KPASTERISK),      /* * */
		KEY(5, 4, KEY_KPDOT),           /* #" */

		KEY(0, 7, KEY_UP),
		KEY(1, 7, KEY_DOWN),
		KEY(2, 7, KEY_LEFT),
		KEY(3, 7, KEY_RIGHT),
		KEY(2, 6, KEY_HOME),
		KEY(3, 6, KEY_END),
		KEY(6, 4, KEY_DELETE),
		KEY(6, 6, KEY_BACK),
		KEY(6, 3, KEY_CAPSLOCK),/* KEY_LEFTSHIFT), */

		KEY(4, 6, KEY_ENTER),   /* scroll push */
		KEY(5, 7, KEY_ENTER),   /* keypad action */

		KEY(0, 4, KEY_EMAIL),
		KEY(5, 6, KEY_SEND),
		KEY(4, 0, KEY_CALENDAR),
		KEY(7, 6, KEY_RECORD),
		KEY(6, 7, KEY_VOLUMEUP),
		KEY(7, 7, KEY_VOLUMEDOWN),

		KEY(0, 6, KEY_F22),     /* soft1 */
		KEY(1, 6, KEY_F23),     /* soft2 */

		KEY(0, 3, KEY_AUX),     /* contact */
};

static unsigned int tavorevb_matrix_key_map[] = {
		/* KEY(row, col, key_code) */
		KEY(0, 4, KEY_A), KEY(0, 5, KEY_B), KEY(0, 6, KEY_C),
		KEY(0, 7, KEY_D), KEY(1, 4, KEY_E), KEY(1, 5, KEY_F),
		KEY(1, 6, KEY_G), KEY(1, 7, KEY_H), KEY(2, 4, KEY_I),
		KEY(2, 5, KEY_J), KEY(2, 6, KEY_K), KEY(2, 7, KEY_L),
		KEY(3, 4, KEY_M), KEY(3, 5, KEY_N), KEY(3, 6, KEY_O),
		KEY(3, 7, KEY_P), KEY(7, 3, KEY_Q), KEY(4, 5, KEY_R),
		KEY(4, 6, KEY_S), KEY(4, 7, KEY_T), KEY(5, 4, KEY_U),
		KEY(5, 5, KEY_V), KEY(5, 6, KEY_W), KEY(5, 7, KEY_X),
		KEY(6, 4, KEY_Y), KEY(6, 5, KEY_Z),

		KEY(0, 3, KEY_0), KEY(2, 0, KEY_1), KEY(2, 1, KEY_2),
		KEY(2, 2, KEY_3), KEY(2, 3, KEY_4), KEY(1, 0, KEY_5),
		KEY(1, 1, KEY_6), KEY(1, 2, KEY_7),
		KEY(1, 3, KEY_8), KEY(0, 2, KEY_9),

		KEY(6, 6, KEY_SPACE),
		KEY(0, 0, KEY_KPASTERISK),      /* * */
		KEY(0, 1, KEY_KPDOT),           /* #" */

		KEY(4, 1, KEY_UP),
		KEY(4, 3, KEY_DOWN),
		KEY(4, 0, KEY_LEFT),
		KEY(4, 2, KEY_RIGHT),
		KEY(6, 0, KEY_HOME),
		KEY(3, 2, KEY_END),
		KEY(6, 1, KEY_DELETE),
		KEY(5, 2, KEY_BACK),
		KEY(6, 3, KEY_CAPSLOCK),/* KEY_LEFTSHIFT), */

		KEY(6, 2, KEY_ENTER),   /* keypad action */

		KEY(7, 2, KEY_EMAIL),
		KEY(3, 1, KEY_SEND),
		KEY(7, 1, KEY_CALENDAR),
		KEY(5, 3, KEY_RECORD),
		KEY(5, 0, KEY_VOLUMEUP),
		KEY(5, 1, KEY_VOLUMEDOWN),

		KEY(3, 0, KEY_F22),	/* soft1 */
		KEY(3, 3, KEY_F23),	/* soft2 */

		KEY(7, 0, KEY_AUX),     /* contact */
};

static struct pxa27x_keypad_platform_data tavorevb_keypad_info = {
	.matrix_key_rows        = 8,
	.matrix_key_cols        = 8,
	.matrix_key_map         = tavorevb_matrix_key_map,
	.matrix_key_map_size    = ARRAY_SIZE(tavorevb_matrix_key_map),
	.debounce_interval      = 30,
};
static struct pxa27x_keypad_platform_data zylonite_keypad_info = {
	.matrix_key_rows        = 8,
	.matrix_key_cols        = 8,
	.matrix_key_map         = zylonite_matrix_key_map,
	.matrix_key_map_size    = ARRAY_SIZE(zylonite_matrix_key_map),
	.debounce_interval      = 30,
};


static struct fb_videomode video_modes[] = {
        /* sharp_ls037 QVGA mode info */
        [0] = {
                .pixclock       = 158000,
                .refresh        = 60,
                .xres           = 240,
                .yres           = 320,
                .hsync_len      = 4,
                .left_margin    = 39,
                .right_margin   = 39,
                .vsync_len      = 1,
                .upper_margin   = 1,
                .lower_margin   = 2,
                .sync		= FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
        },
        /* sharp_ls037 VGA mode info */
        [1] = {
                .pixclock       = 39700,
                .refresh        = 60,
                .xres           = 480,
                .yres           = 640,
                .hsync_len      = 8,
                .left_margin    = 81,
                .right_margin   = 81,
                .vsync_len      = 1,
                .upper_margin   = 2,
                .lower_margin   = 7,
                .sync           = 0,
        },
};

#define LCD_SCLK (312000000UL)
static struct pxa910fb_mach_info pxa168_tavorevb_lcd_info = {
        .id                     = "GFX Layer",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = ARRAY_SIZE(video_modes),
        .modes                  = video_modes,
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_DUMB_16_GPIO,
        .dumb_mode              = DUMB_MODE_RGB565,
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
        .spi_gpio_reset         = -1,
	.max_fb_size		= 1280 * 720 * 4,
};

static struct pxa910fb_mach_info pxa168_tavorevb_lcd_ovly_info = {
        .id                     = "Video Layer",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = ARRAY_SIZE(video_modes),
        .modes                  = video_modes,
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_DUMB_16_GPIO,
        .dumb_mode              = DUMB_MODE_RGB565,
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
        .spi_gpio_reset         = -1,
	.max_fb_size		= 1280 * 720 * 4,
};
#ifdef CONFIG_PXA168_SMART_LCD
static unsigned short pxa910_init_panel[] = {
	/* P-ON Init sequence */
	PXA910_MAKEUP_CMD(0x00),	/* OSC ON */
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x01),	/* SOURCE DRIVER SHIFT DIRECTION and display RAM setting */
	PXA910_MAKEUP_DATA(0x01),
	PXA910_MAKEUP_DATA(0x27),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x02),	/* LINE INV */
	PXA910_MAKEUP_DATA(0x02),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x03),	/* IF mode(1) */
	PXA910_MAKEUP_DATA(0x01), 	/* 8bit smart mode(8-8),high speed write mode */
	PXA910_MAKEUP_DATA(0x30),  
	PXA910_MAKEUP_CMD(0x07),
	PXA910_MAKEUP_CMD(0x00),	/* RAM Write Mode */
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x03),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x07),	/* DISPLAY Setting,  262K, fixed(NO scroll), no split screen */
	PXA910_MAKEUP_DATA(0x40),	/* 16/18/19 BPP */
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x08),	/* BP, FP Seting, BP=2H, FP=3H */
	PXA910_MAKEUP_DATA(0x03),
	PXA910_MAKEUP_DATA(0x02),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x0C),	/* IF mode(2), using internal clock & MPU */
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x0D),	/* Frame setting, 1Min. Frequence, 16CLK */
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x10),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x12),	/* Timing(1),ASW W=4CLK, ASW ST=1CLK */
	PXA910_MAKEUP_DATA(0x03),
	PXA910_MAKEUP_DATA(0x02),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x13),	/* Timing(2),OEV ST=0.5CLK, OEV ED=1CLK */
	PXA910_MAKEUP_DATA(0x01),
	PXA910_MAKEUP_DATA(0x02),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x14),	/* Timing(3), ASW HOLD=0.5CLK */
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x15),	/* Timing(4), CKV ST=0CLK, CKV ED=1CLK */
	PXA910_MAKEUP_DATA(0x20),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x1C),		
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x03),
	PXA910_MAKEUP_CMD(0x00),		
	PXA910_MAKEUP_DATA(0x04),
	PXA910_MAKEUP_DATA(0x03),
	PXA910_MAKEUP_CMD(0x03),
	PXA910_MAKEUP_CMD(0x01),		
	PXA910_MAKEUP_DATA(0x03),
	PXA910_MAKEUP_DATA(0x04),
	PXA910_MAKEUP_CMD(0x03),
	PXA910_MAKEUP_CMD(0x02),		
	PXA910_MAKEUP_DATA(0x04),
	PXA910_MAKEUP_DATA(0x03),
	PXA910_MAKEUP_CMD(0x03),		
	PXA910_MAKEUP_CMD(0x03),
	PXA910_MAKEUP_DATA(0x03),
	PXA910_MAKEUP_DATA(0x03),
	PXA910_MAKEUP_CMD(0x03),
	PXA910_MAKEUP_CMD(0x04),		
	PXA910_MAKEUP_DATA(0x01),
	PXA910_MAKEUP_DATA(0x01),
	PXA910_MAKEUP_CMD(0x03),
	PXA910_MAKEUP_CMD(0x05),		
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x04),
	PXA910_MAKEUP_CMD(0x02),		
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x04),
	PXA910_MAKEUP_CMD(0x03),		
	PXA910_MAKEUP_DATA(0x01),
	PXA910_MAKEUP_DATA(0x3F),	
	(PXA910_LCD_CMD_WAIT |	0),	

	/* DISP RAM setting: 240*320 */
	PXA910_MAKEUP_CMD(0x04),	/* HADDR, START 0 */
	PXA910_MAKEUP_CMD(0x06),	
	PXA910_MAKEUP_DATA(0x00), 
	PXA910_MAKEUP_DATA(0x00),	/* x1,3 */
	PXA910_MAKEUP_CMD(0x04),	/* HADDR,  END   4 */
	PXA910_MAKEUP_CMD(0x07), 
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0xEF), 	/* x2, 7 */
	PXA910_MAKEUP_CMD(0x04),	/* VADDR, START 8 */
	PXA910_MAKEUP_CMD(0x08),	
	PXA910_MAKEUP_DATA(0x00), 	/* y1, 10 */
	PXA910_MAKEUP_DATA(0x00), 	/* y1, 11 */
	PXA910_MAKEUP_CMD(0x04), 	/* VADDR, END 12 */
	PXA910_MAKEUP_CMD(0x09),
	PXA910_MAKEUP_DATA(0x01), 	/* y2, 14 */
	PXA910_MAKEUP_DATA(0x3F), 	/* y2, 15 */
	PXA910_MAKEUP_CMD(0x02),	/* RAM ADDR SETTING 16 */
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x00), 	/* x1, 19 */
	PXA910_MAKEUP_CMD(0x02),	/* RAM ADDR SETTING 20 */
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_DATA(0x00), 	/* y1, 22 */
	PXA910_MAKEUP_DATA(0x00), 	/* y1, 23 */
};


static unsigned short pxa910_turn_on_display[] = {
	/* Power-IC ON */
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x02),
	PXA910_MAKEUP_DATA(0x07),
	PXA910_MAKEUP_DATA(0x7D),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x03),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x05),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x04),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x05),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x15),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0xC0),
	PXA910_MAKEUP_DATA(0x10),	
	(PXA910_LCD_CMD_WAIT |	30),		

	/* DISP ON */
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x01),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0xFF),
	PXA910_MAKEUP_DATA(0xFE),
	(PXA910_LCD_CMD_WAIT |	150),		
};


static unsigned short pxa910_turn_off_display[] = {
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_CMD(0x1E),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x0A),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0xFF),
	PXA910_MAKEUP_DATA(0xEE),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0xF8),
	PXA910_MAKEUP_DATA(0x12),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0xE8),
	PXA910_MAKEUP_DATA(0x11),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0xC0),
	PXA910_MAKEUP_DATA(0x11),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0x40),
	PXA910_MAKEUP_DATA(0x11),
	PXA910_MAKEUP_CMD(0x01),
	PXA910_MAKEUP_CMD(0x00),
	PXA910_MAKEUP_DATA(0x00),
	PXA910_MAKEUP_DATA(0x10),	
};

static unsigned short pxa910_update_framedata[] = {
	/* write ram */
	PXA910_MAKEUP_CMD(0x02),
	PXA910_MAKEUP_CMD(0x02),
};

/*Clear Standby and Deep standby states*/
static unsigned short LCD_EX_STBY_CMD[] = {
        PXA910_MAKEUP_CMD(0x00), PXA910_MAKEUP_CMD(0x1D), PXA910_MAKEUP_DATA(0x00), PXA910_MAKEUP_DATA(0x05)
};

static unsigned int gpio82, gpio83, gpio103;
static void pxa910_setup_gpio(void)
{
	gpio82 = mfp_to_gpio(MFP_PIN_GPIO82);
	gpio83 = mfp_to_gpio(MFP_PIN_GPIO83);
	gpio103 = mfp_to_gpio(MFP_PIN_GPIO103);

	gpio_request(gpio82, "gpio82");
	gpio_request(gpio83, "gpio83");
	gpio_request(gpio103, "gpio103");
	gpio_request(109, "gpio109");

	gpio_direction_output(gpio82, 1);
	gpio_direction_output(gpio83, 1);
	gpio_direction_output(gpio103, 1);
	gpio_direction_output(109, 1);
}

static void ltm020d550_panel_exit_standby(struct pxa910fb_info *fbi)
{
	unsigned int tmp, *reg_base2, *reg_base3;
	int i, j;
	unsigned long io83[] = {GPIO83_LCD_WR};
	unsigned long lcd83[] = {GPIO83_LCD_PCLK};

	/*Set the following MFP as gpio temporally to bit-bang the LCD lines to get the LCD out of deep standby mode
		GPIO  82 - Toshiba RS line (Register Select)
		GPIO  83 - Toshiba WR line (write)
		GPIO 103 - Toshiba CS line (Chip Select)
	*/
	pxa910_setup_gpio();

	for(i = 0; i < 2; i++) {
		/* configure smart A0 as gpio to wake up panel */
		mfp_write(82, mfp_read(82) & (~7));

		/*set MFP83 to as GPIO PIN.
		 * Can not mfp_config pin 83 for no reason??? workaround with low level primitive */
		//mfp_config(io83, 1);
		mfp_write(83, mfp_read(83) & (~7));
		mdelay(2);
		/*Toggle the smart panel reset line (GPIO_109)*/
		gpio_set_value(109, 0);
		mdelay(1);
		gpio_set_value(109, 1);
		mdelay(1);
		/*To wake the panel up from DEEP STANDBY mode ,GPIO registerS  must be hit
		  three times with a 1 ms delay between each hit.
		 */

		for (j = 0; j < 4; j++) {
			/*use primitive API to set three pins to low for this panel request strict timing */
			GPCR(103) = GPIO_bit(103);
			GPCR(83) = GPIO_bit(82) | GPIO_bit(83);
			udelay(1);
			/*set three pins to high*/
			GPSR(83) = GPIO_bit(82) | GPIO_bit(83);
			GPSR(103) = GPIO_bit(103);
			mdelay(1);
		}
		/*set MFP83 as LCD controller signal PIN for cmd sending */
		//mfp_config(lcd83, 1);
		mfp_write(83, mfp_read(83) & (~7) | 1);
		mdelay(1);
		/*Keep CS signal LOW for now*/
		gpio_set_value(gpio103, 0);
		mdelay(5);
		/* Configure pin 82 as smart A0 of LCD function pin */
		mfp_write(82, mfp_read(82) & (~7) | 1);
		/*Write to an internal register in the smart panel to exit standby mode*/
		pxa910fb_smart_cmd(fbi, ARRAY_AND_SIZE(LCD_EX_STBY_CMD));
		mdelay(10);
	}
	/* Configure pin 81 as LCD function pin: smart RD */
	mfp_write(81, mfp_read(81) & (~7) | 1);
	mdelay(1);
}

static void ltm020d550_lcd_power(struct pxa910fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	if (on) {
		ltm020d550_panel_exit_standby(fbi);
		pxa910fb_smart_cmd(fbi, ARRAY_AND_SIZE(pxa910_init_panel));
		pxa910fb_smart_cmd(fbi, ARRAY_AND_SIZE(pxa910_turn_on_display));
		pxa910fb_smart_cmd(fbi, ARRAY_AND_SIZE(pxa910_update_framedata));
	} else {
		pxa910fb_smart_cmd(fbi, ARRAY_AND_SIZE(pxa910_turn_off_display));
	}
}

/*pxa910_tavorevb: toshiba smart panel - TODO*/
static struct fb_videomode pxa910_toshiba_ltm020d550_modes =
{
	//.flags          =	PXAFB_SMART_PANEL | PXAFB_STD_SMART_PANEL | PXAFB_AUTO_REFRESH,
	.pixclock       = 158000,
	.refresh        = 60,
	.xres           = 240,
	.yres           = 320,
};

#define LCD_SCLK (312000000UL)
static struct pxa910fb_mach_info pxa910_smart_lcd_info = {
        .id                     = "GFX Layer",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = 1,
        .modes          	= &pxa910_toshiba_ltm020d550_modes,
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_SMART_8_SPI_GPIO,
        .dumb_mode              = DUMB_MODE_RGB565,
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
        .spi_gpio_reset         = -1,
	.max_fb_size		= 640 * 960 * 4,
	.pxa910fb_lcd_power	= ltm020d550_lcd_power,
	.display_interface	= SMART_PANEL,
};
static struct pxa910fb_mach_info pxa910_smart_lcd_ovly_info = {
        .id                     = "Video Layer",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = ARRAY_SIZE(video_modes),
        .modes                  = video_modes,
        .pix_fmt                = PIX_FMT_RGB565,
        .panel_rgb_reverse_lanes= 0,
        .panel_rbswap		= 1,
        .enable_lcd             = 1,
	.max_fb_size		= 1280 * 720 * 4,
	.display_interface	= SMART_PANEL,
};

static unsigned long pxa910_tavorevb_smart_lcd_cfg[] __initdata = {
        /* LCD */
        GPIO103_LCD_CS,
        GPIO82_LCD_A0,
        GPIO83_LCD_WR,
	GPIO109_GPIO109,
};

/*pxa910_tavorevb: sharp dump panel*/
static int smart_lcd = 1;
static int __init smart_lcd_setup(char *__unused)
{
        smart_lcd = 1;
        return 1;
}
__setup("smart_lcd", smart_lcd_setup);

inline int is_smart_lcd(void)
{
        return smart_lcd;
}
#endif /* CONFIG_PXA168_SMART_LCD */

static struct i2c_pxa_platform_data i2c_info __initdata = {
	.use_pio		= 1,
};

static struct i2c_board_info i2c_board_info[] =
{
#if defined(CONFIG_PXA168_CAMERA)
        {
                .type           = "ov7660",
                .addr           = 0x21,
                .platform_data  = &ov7660_sensor_data,
        },
        {
                .type           = "ov3640",
                .addr           = 0x3C,
                .platform_data  = &ov3640_sensor_data,
        },
#endif
};

#if defined(CONFIG_MMC_PXA_SDH)
static struct pxasdh_platform_data tavorevb_sdh_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
};
#endif

static void __init tavorevb_init(void)
{

	mfp_config(ARRAY_AND_SIZE(tavorevb_pin_config));

	/* on-chip devices */
	pxa910_add_uart(1);
	pxa910_add_twsi(0, &i2c_info, ARRAY_AND_SIZE(i2c_board_info));
	pxa910_add_ire();
	pxa910_add_acipc();
	if (tavorevb_keypad_type == 1)
		pxa910_add_keypad(&zylonite_keypad_info);
	else
		pxa910_add_keypad(&tavorevb_keypad_info);
#if defined(CONFIG_MMC_PXA_SDH)
	pxa910_add_sdh(0, &tavorevb_sdh_platform_data);
#endif

#ifdef CONFIG_PXA168_SMART_LCD
	if(is_smart_lcd()){
		mfp_config(ARRAY_AND_SIZE(pxa910_tavorevb_smart_lcd_cfg));
		pxa910_add_fb(&pxa910_smart_lcd_info);
		pxa910_add_fb_ovly(&pxa910_smart_lcd_ovly_info);
	} else
#endif
	{
		pxa910_add_fb(&pxa168_tavorevb_lcd_info);
		pxa910_add_fb_ovly(&pxa168_tavorevb_lcd_ovly_info);
	}
#ifdef CONFIG_USB_GADGET
	platform_device_register(&pxa910_device_u2o);
#endif

#ifdef CONFIG_USB_OTG
	platform_device_register(&pxa910_device_u2ootg);
	platform_device_register(&pxa910_device_u2oehci);
#endif
#if defined(CONFIG_PXA910_CAMERA)
	pxa910_add_cam();
#endif

	pxa910_add_ssp(1);
	pxa910_add_freq();

	/* off-chip devices */
	platform_device_register(&smc91x_device);
}

MACHINE_START(TAVOREVB, "PXA910 Evaluation Board (aka TavorEVB)")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa910_init_irq,
	.timer          = &pxa910_timer,
	.init_machine   = tavorevb_init,
MACHINE_END
