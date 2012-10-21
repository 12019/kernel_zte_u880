/*
 * linux/arch/arm/mach-mmp/mmp2.c
 *
 * code name MMP2
 *
 * Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/devices.h>

#include <plat/pxa_u2o.h>

#include <linux/platform_device.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x9c)

static struct mfp_addr_map mmp2_addr_map[] __initdata = {
	MFP_ADDR(PMIC_INT, 0x2c4),

	/* LCD */
	MFP_ADDR_X(GPIO94, GPIO96, 0x1c0),
	MFP_ADDR(GPIO83, 0x194),
	MFP_ADDR(GPIO114, 0x164),
	MFP_ADDR(CLK_REQ, 0x160),

	/* TWSI5 */
	MFP_ADDR_X(GPIO99, GPIO100, 0x1d4),

	/* TWSI6 */
	MFP_ADDR_X(GPIO97, GPIO98, 0x1cc),

	/* KEYPAD */
	MFP_ADDR_X(GPIO0, GPIO5, 0x054),

	MFP_ADDR_END,
};

void mmp2_clear_pmic_int(void)
{
	unsigned long mfpr_pmic, data;

	mfpr_pmic = APB_VIRT_BASE + 0x1e000 + 0x2c4;
	data = __raw_readl(mfpr_pmic);
	__raw_writel(data | (1 << 6), mfpr_pmic);
	__raw_writel(data, mfpr_pmic);
}

static void __init mmp2_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_MMP2_GPIO);

	/* unmask GPIO edge detection for all 6 banks -- APMASKx */
	for (i = 0; i < 6; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_MMP2_GPIO, 0, 167, NULL);
}

void __init mmp2_init_irq(void)
{
	mmp2_init_icu();
	mmp2_init_gpio();
}

void __init mmp2_init_usb_phy(unsigned int base)
{
	pxa_usb_phy_init(base, USB_PHY_PXA910);
}

/* APB peripheral clocks */
static APBC_CLK(uart1, MMP2_UART1, 1, 26000000);
static APBC_CLK(uart2, MMP2_UART2, 1, 26000000);
static APBC_CLK(uart3, MMP2_UART3, 1, 26000000);
static APBC_CLK(uart4, MMP2_UART4, 1, 26000000);
static APBC_CLK(twsi1, MMP2_TWSI1, 0, 26000000);
static APBC_CLK(twsi2, MMP2_TWSI2, 0, 26000000);
static APBC_CLK(twsi3, MMP2_TWSI3, 0, 26000000);
static APBC_CLK(twsi4, MMP2_TWSI4, 0, 26000000);
static APBC_CLK(twsi5, MMP2_TWSI5, 0, 26000000);
static APBC_CLK(twsi6, MMP2_TWSI6, 0, 26000000);
static APBC_CLK(rtc, MMP2_RTC, 0, 32768);
static APBC_CLK(keypad, MMP2_KPC, 0, 32768);
static PSEUDO_CLK(iscclk, 0, 0, 0);     /* pseudo clock for imm */

static APMU_CLK(nand, NAND, 0xbf, 100000000);
static APMU_CLK(u2o, USBOTG, 0x1b, 480000000);
static APMU_CLK(lcd, LCD_CLK_RES_CTRL, 0xd123f, 400000000);	/* 400MHz, HCLK, CLK, AXICLK */
#ifdef CONFIG_FB_SECOND_PANEL_HDMI
static APMU_CLK(tv, LCD_CLK_RES_CTRL, 0xd723f, 400000000);	/* 400MHz, HCLK, CLK, AXICLK */
#endif
#ifdef CONFIG_FB_SECOND_PANEL_DSI
static APMU_CLK(lcd2, LCD2_CLK_RES_CTRL, 0xd123f, 400000000);	/* 400MHz, HCLK, CLK, AXICLK */
#endif

static struct clk_lookup mmp2_clkregs[] = {
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_uart2, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_uart3, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_uart4, "pxa2xx-uart.3", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa2xx-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi2, "pxa2xx-i2c.1", NULL),
	INIT_CLKREG(&clk_twsi3, "pxa2xx-i2c.2", NULL),
	INIT_CLKREG(&clk_twsi4, "pxa2xx-i2c.3", NULL),
	INIT_CLKREG(&clk_twsi5, "pxa2xx-i2c.4", NULL),
	INIT_CLKREG(&clk_twsi6, "pxa2xx-i2c.5", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_u2o, NULL, "USBCLK"),
	INIT_CLKREG(&clk_iscclk,  NULL, "ISCCLK"),
	INIT_CLKREG(&clk_lcd, "pxa168-fb.0", NULL),
#ifdef CONFIG_FB_SECOND_PANEL_HDMI
	INIT_CLKREG(&clk_tv, "pxa168-fb.1", NULL),
#endif
#ifdef CONFIG_FB_SECOND_PANEL_DSI
	INIT_CLKREG(&clk_lcd2, "pxa168-fb.1", NULL),
#endif
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_rtc, "mmp-rtc", NULL),

};

static int __init mmp2_init(void)
{
	if (cpu_is_mmp2()) {
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(mmp2_addr_map);
		pxa_init_dma(IRQ_MMP2_DMA_RIQ, 16);
		clks_register(ARRAY_AND_SIZE(mmp2_clkregs));
	}

	return 0;
}
postcore_initcall(mmp2_init);

/* on-chip devices */
MMP2_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4030000, 0x30, 4, 5);
MMP2_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4017000, 0x30, 20, 21);
MMP2_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4018000, 0x30, 22, 23);
MMP2_DEVICE(uart4, "pxa2xx-uart", 3, UART4, 0xd4016000, 0x30, 18, 19);
MMP2_DEVICE(twsi1, "pxa2xx-i2c", 0, TWSI1, 0xd4011000, 0x70);
MMP2_DEVICE(twsi2, "pxa2xx-i2c", 1, TWSI2, 0xd4031000, 0x70);
MMP2_DEVICE(twsi3, "pxa2xx-i2c", 2, TWSI3, 0xd4032000, 0x70);
MMP2_DEVICE(twsi4, "pxa2xx-i2c", 3, TWSI4, 0xd4033000, 0x70);
MMP2_DEVICE(twsi5, "pxa2xx-i2c", 4, TWSI5, 0xd4033800, 0x70);
MMP2_DEVICE(twsi6, "pxa2xx-i2c", 5, TWSI6, 0xd4034000, 0x70);
MMP2_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x100, 28, 29);
MMP2_DEVICE(fb, "pxa168-fb", 0, LCD, 0xd420b000, 0x500);
MMP2_DEVICE(fb_tv, "pxa168-fb", 1, LCD, 0xd420b000, 0x500);
MMP2_DEVICE(fb_ovly, "pxa168fb_ovly", 0, LCD, 0xd420b000, 0x500);
MMP2_DEVICE(fb_tv_ovly, "pxa168fb_ovly", 1, LCD, 0xd420b000, 0x500);
MMP2_DEVICE(hdmi, "pxa688-hdmi", -1, HDMI, 0xd420bc00, 0x2000);
MMP2_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);

#ifdef CONFIG_CPU_PXA910
static struct resource pxa910_resource_imm[] = {
        [0] = {
	                        .name   = "phy_sram",
	                        .start  = 0xd1000000 + SZ_64K ,
	                        .end    = 0xd1000000 + SZ_128K - 1,
	                        .flags  = IORESOURCE_MEM,
	                },
        [1] = {
	                        .name   = "imm_sram",
	                        .start  = 0xd1000000 + SZ_64K,
	                        .end    = 0xd1000000 + SZ_128K - 1,
	                        .flags  = IORESOURCE_MEM,
	                },
};

struct platform_device pxa910_device_imm = {
        .name           = "pxa3xx-imm",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(pxa910_resource_imm),
	.resource       = pxa910_resource_imm,
};
#endif

static struct resource mmp2_resource_imm[] = {
    [0] = {
		.name   = "phy_sram",
		.start  = 0xd1000000,
		.end    = 0xd1000000 + SZ_128K + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
    [1] = {
		.name   = "imm_sram",
		.start  = 0xd1000000,
		.end    = 0xd1000000 + SZ_128K + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device mmp2_device_imm = {
	.name           = "pxa3xx-imm",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(mmp2_resource_imm),
	.resource       = mmp2_resource_imm,
};

static struct resource mmp2_resource_rtc[] = {
	[0] = {
		.start  = 0xd4010000,
		.end    = 0xD40100ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_MMP2_RTC,
		.end    = IRQ_MMP2_RTC,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_1HZ",
	},

	[2] = {
		.start  = IRQ_MMP2_RTC_ALARM,
		.end    = IRQ_MMP2_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_ALARM",
	},

};

struct platform_device mmp2_device_rtc = {
       .name           = "mmp-rtc",
       .id             = -1,
       .resource       = mmp2_resource_rtc,
       .num_resources  = ARRAY_SIZE(mmp2_resource_rtc),
};
