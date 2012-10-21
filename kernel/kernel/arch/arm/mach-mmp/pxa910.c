/*
 *  linux/arch/arm/mach-mmp/pxa910.c
 *
 *  Code specific to PXA910
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>

#include <asm/mach/time.h>
#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/mfp.h>
#include <plat/i2c.h>
#include <mach/pxa910-squ.h>
#include <mach/devices.h>
#include <linux/usb/otg.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>
#include <mach/pxa910.h>

#include <asm/mach-types.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)
#define FAB_CTRL	(AXI_VIRT_BASE + 0x260)

static struct mfp_addr_map pxa910_mfp_addr_map[] __initdata =
{
	MFP_ADDR_X(GPIO0, GPIO54, 0xdc),
	MFP_ADDR_X(GPIO55, GPIO66, 0x2f0),
	MFP_ADDR_X(GPIO67, GPIO98, 0x1b8),
	MFP_ADDR_X(GPIO100, GPIO109, 0x238),
	MFP_ADDR_X(GPIO110, GPIO116, 0x298),

	MFP_ADDR(GPIO123, 0xcc),
	MFP_ADDR(GPIO124, 0xd0),

	MFP_ADDR(DF_IO0, 0x40),
	MFP_ADDR(DF_IO1, 0x3c),
	MFP_ADDR(DF_IO2, 0x38),
	MFP_ADDR(DF_IO3, 0x34),
	MFP_ADDR(DF_IO4, 0x30),
	MFP_ADDR(DF_IO5, 0x2c),
	MFP_ADDR(DF_IO6, 0x28),
	MFP_ADDR(DF_IO7, 0x24),
	MFP_ADDR(DF_IO8, 0x20),
	MFP_ADDR(DF_IO9, 0x1c),
	MFP_ADDR(DF_IO10, 0x18),
	MFP_ADDR(DF_IO11, 0x14),
	MFP_ADDR(DF_IO12, 0x10),
	MFP_ADDR(DF_IO13, 0xc),
	MFP_ADDR(DF_IO14, 0x8),
	MFP_ADDR(DF_IO15, 0x4),

	MFP_ADDR(DF_nCS0_SM_nCS2, 0x44),
	MFP_ADDR(DF_nCS1_SM_nCS3, 0x48),
	MFP_ADDR(SM_nCS0, 0x4c),
	MFP_ADDR(SM_nCS1, 0x50),
	MFP_ADDR(DF_WEn, 0x54),
	MFP_ADDR(DF_REn, 0x58),
	MFP_ADDR(DF_CLE_SM_OEn, 0x5c),
	MFP_ADDR(DF_ALE_SM_WEn, 0x60),
	MFP_ADDR(SM_SCLK, 0x64),
	MFP_ADDR(DF_RDY0, 0x68),
	MFP_ADDR(SM_BE0, 0x6c),
	MFP_ADDR(SM_BE1, 0x70),
	MFP_ADDR(SM_ADV, 0x74),
	MFP_ADDR(DF_RDY1, 0x78),
	MFP_ADDR(SM_ADVMUX, 0x7c),
	MFP_ADDR(SM_RDY, 0x80),

	MFP_ADDR_X(MMC1_DAT7, MMC1_WP, 0x84),

	MFP_ADDR_END,
};

static u32 gc_current_clk_rate_flag;

#define MCB_CNTRL5_OFF 0x550
#define MCB_GC_SW_BYPASS (1<<2)

static unsigned char __iomem *dmc_membase;
void gc_fc_ack_bypass(int bypass)
{
	unsigned int temp;

	temp = __raw_readl(dmc_membase + MCB_CNTRL5_OFF);
	if (bypass)
		__raw_writel(temp | MCB_GC_SW_BYPASS, dmc_membase + MCB_CNTRL5_OFF);
	else
		__raw_writel(temp & ~MCB_GC_SW_BYPASS, dmc_membase + MCB_CNTRL5_OFF);
}

static void gc500_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	u32 pll2freq, rate;
	static int gc_aclk_done = 0;

	if (clk->rate > 312000000) {
		/* Note: get_pll2_freq use MHz instead of Hz */
		rate = clk->rate*2/1000000;
		pll2freq = get_pll2_freq();
		if (unlikely(pll2freq != rate))
			printk(KERN_INFO "gc_clk2x will use %uMHz instead of %uMHz\n",
				pll2freq/2, rate/2);
		if (!gc_aclk_done) {
			gc_aclk_fc();
			gc_aclk_done = 1;
		}
	}
	__raw_writel(tmp | 0x38, clk->clk_rst);
	gc_fc_ack_bypass(0);
}

static void gc500_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	gc_fc_ack_bypass(1);
	__raw_writel(tmp & (~0x38), clk->clk_rst);
}

void gc_pwr(int power_on)
{
	if (power_on) {
		u32 tmp = __raw_readl(APMU_GC);

	tmp &= 0xc0;
		tmp |= gc_current_clk_rate_flag;
		__raw_writel(tmp | 0x1238, APMU_GC);	/* on1 */
		udelay(200); /* at least 200 us*/
		__raw_writel(tmp | 0x1638, APMU_GC);	/* on2 */
		__raw_writel(tmp | 0x163a, APMU_GC);	/* release function reset */
		udelay(100); /* at least 48 cycles */
		__raw_writel(tmp | 0x173f, APMU_GC);	/* aReset hReset and disable isolation */

	gc_fc_ack_bypass(0);
	} else {
	gc_fc_ack_bypass(1);
		__raw_writel(0x738, APMU_GC);	/* reset AXI/AHB/function */
		udelay(100);
		__raw_writel(0x638, APMU_GC);	/* enable isolation */
		__raw_writel(0x238, APMU_GC);	/* off2 */
		__raw_writel(0x038, APMU_GC);	/* off1 */

		__raw_writel(0x0, APMU_GC);	/* all clear for power */
	}
}
EXPORT_SYMBOL(gc_pwr);

struct gc_rate_table {
	unsigned long	rate;
	unsigned int	flag;
};

static struct gc_rate_table gc500_rates [] = {
	/* put highest rate at the top of the table */
	{
		.rate	=	403000000,
		.flag	=	APMU_GC_PLL2_DIV2,
	},
	{
		.rate	=	312000000,
		.flag	=	APMU_GC_312M,
	},
	{
		.rate	=	156000000,
		.flag	=	APMU_GC_156M,
	},
};

static int gc_lookaround_rate(unsigned long gc_clk2x, u32 *flag)
{
	int i;

	for (i=0; i<ARRAY_SIZE(gc500_rates); i++) {
		if (gc_clk2x >= gc500_rates[i].rate)
			break;
	}
	if (i==ARRAY_SIZE(gc500_rates)) i--;
	*flag = gc500_rates[i].flag;
	return gc500_rates[i].rate;
}

static int gc500_clk_setrate(struct clk *clk, unsigned long gc_clk2x)
{
	u32 tmp, flag;
	int rate;

	if (cpu_is_pxa918())
		rate = gc_lookaround_rate(312000000, &flag);
	else
		rate = gc_lookaround_rate(gc_clk2x, &flag);

	clk->rate = rate;
	__raw_writel(0xf, APMU_GC_PD);
	tmp = __raw_readl(clk->clk_rst);
	tmp &= ~0xc0;
	tmp |= flag;
	__raw_writel(tmp, clk->clk_rst);
	gc_current_clk_rate_flag = flag;
	return 0;
}

static unsigned long gc500_clk_getrate(struct clk *clk)
{
	return clk->rate;
}

struct clkops gc500_clk_ops = {
	.enable		= gc500_clk_enable,
	.disable	= gc500_clk_disable,
	.setrate	= gc500_clk_setrate,
	.getrate	= gc500_clk_getrate,
};

#define APMASK(i)      (GPIO_REGS_VIRT + BANK_OFF(i) + 0x09c)
static void __init pxa910_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_PXA910_GPIO);

	/* unmask GPIO edge detection for all 4 banks - APMASKx */
	for (i = 0; i < 4; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_PXA910_AP_GPIO, 0, 127, NULL);
}

void __init pxa910_init_irq(void)
{
	icu_init_irq();
	pxa910_init_gpio();
}

/* APB peripheral clocks */
static APBC_CLK(uart1, PXA910_UART0, 1, 14745600);
static APBC_CLK(uart2, PXA910_UART1, 1, 14745600);
static APBC_CLK(uart3, PXA910_UART2, 1, 14745600);
static APBC_CLK(twsi0, PXA910_TWSI0, 0, 33000000);
static APBC_CLK(twsi1, PXA910_TWSI1, 1, 33000000);
static APBC_CLK(pwm1, PXA910_PWM1, 1, 13000000);
static APBC_CLK(pwm2, PXA910_PWM2, 1, 13000000);
static APBC_CLK(pwm3, PXA910_PWM3, 1, 13000000);
static APBC_CLK(pwm4, PXA910_PWM4, 1, 13000000);
static APBC_CLK(keypad, PXA910_KPC, 0, 32000);
static APBC_CLK(ssp0,  PXA910_SSP0,  4, 3250000);
static APBC_CLK(ssp1,  PXA910_SSP1,  0, 0);
static APBC_CLK(rtc, PXA910_RTC, 0x8, 1);

static APMU_CLK_OPS(nand, NAND, 156000000, &nand_clk_ops);
static APMU_CLK_OPS(u2o, USB, 480000000, &u2o_clk_ops);	/* 480MHz, AXICLK */
static APMU_CLK_OPS(u2h, USB, 480000000, &u2h_clk_ops);	/* 480MHz, AXICLK */
static APMU_CLK_OPS(smc, SMC, 31200000, &smc_clk_ops);
//static PSEUDO_CLK(iscclk, 0, 0, 0);	/* pseudo clock for imm */
static APMU_CLK(lcd, LCD, 0x003f, 312000000);	/* 312MHz, HCLK, CLK, AXICLK */
static APMU_CLK(ire, IRE, 0x9, 0);
static APMU_CLK_OPS(gc, GC, 0, &gc500_clk_ops);
static APMU_CLK(sdh0, SDH0, 0x001b, 48000000);	/* 48MHz, CLK, AXICLK */
static APMU_CLK(sdh1, SDH1, 0x001b, 48000000);	/* 48MHz, CLK, AXICLK */
static APMU_CLK(sdh2, SDH2, 0x001b, 48000000);	/* 48MHz, CLK, AXICLK */
static APMU_CLK(ccic_rst, CCIC_RST, 0x0, 312000000);
static APMU_CLK(ccic_gate, CCIC_GATE, 0xfff, 0);

/* device and clock bindings */
static struct clk_lookup pxa910_clkregs[] = {
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_uart2, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_uart3, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_twsi0, "pxa2xx-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa2xx-i2c.1", NULL),
	INIT_CLKREG(&clk_pwm1, "pxa910-pwm.0", NULL),
	INIT_CLKREG(&clk_pwm2, "pxa910-pwm.1", NULL),
	INIT_CLKREG(&clk_pwm3, "pxa910-pwm.2", NULL),
	INIT_CLKREG(&clk_pwm4, "pxa910-pwm.3", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_lcd, "pxa168-fb", "LCDCLK"),
	INIT_CLKREG(&clk_lcd, "pxa910-fb", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", "NANDCLK"),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_ire, "pxa910-ire", NULL),
	INIT_CLKREG(&clk_ssp0,  "pxa910-ssp.0", NULL),
	INIT_CLKREG(&clk_ssp1,	"pxa910-ssp.1", NULL),
	INIT_CLKREG(&clk_sdh0, "pxa-sdh.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh1, "pxa-sdh.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "pxa-sdh.2", "PXA-SDHCLK"),
        INIT_CLKREG(&clk_ccic_rst, "pxa168-camera", "CCICRSTCLK"),
        INIT_CLKREG(&clk_ccic_gate, "pxa168-camera", "CCICGATECLK"),
        INIT_CLKREG(&clk_ccic_rst, "pxa910-camera", "CCICRSTCLK"),
        INIT_CLKREG(&clk_ccic_gate, "pxa910-camera", "CCICGATECLK"),
	INIT_CLKREG(&clk_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_u2h, NULL, "U2HCLK"),
	INIT_CLKREG(&clk_rtc, NULL, "MMP-RTC"),
	INIT_CLKREG(&clk_gc, NULL, "GCCLK"),
};

/* ACIPC clock is initialized by CP, enable the clock by default
 * and this clock is always enabled.
 */
static void pxa910_init_acipc_clock(void)
{
	__raw_writel(0x3, APBC_PXA910_ACIPC);
}
static void pxa910_init_ripc_clock(void)
{
        __raw_writel(0x2, APBC_PXA910_RIPC);
}
static int __init pxa910_init(void)
{
	if (cpu_is_pxa910()) {
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(pxa910_mfp_addr_map);
		pxa_init_dma(IRQ_PXA910_DMA_INT0, 32);
		pxa910_init_squ(2);

		pxa910_init_acipc_clock();
		pxa910_init_ripc_clock();
		/* Enable AXI write request for gc500 */
        __raw_writel(__raw_readl(FAB_CTRL) | 0x8, FAB_CTRL);

		clks_register(ARRAY_AND_SIZE(pxa910_clkregs));
	}

	dmc_membase = ioremap(0xb0000000, 0x00001000);
	return 0;
}
postcore_initcall(pxa910_init);

static int ttc_dkb_vbus_detect(void *func, int enable) {
	return 0;
}

static struct pxa_usb_plat_info pxa910_u2o_info = {
	.phy_init	= pxa_usb_phy_init,
	.is_otg		= 1,
	.in_single	= 1,
	.vbus_detect	= ttc_dkb_vbus_detect,
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
		.start	= IRQ_PXA910_USB1,
		.end	= IRQ_PXA910_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource u2ootg_resources[] = {
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
		.start	= IRQ_PXA910_USB1,
		.end	= IRQ_PXA910_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource ehci_u2o_resources[] = {
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
		.start	= IRQ_PXA910_USB1,
		.end	= IRQ_PXA910_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

#ifdef CONFIG_USB_GADGET
struct platform_device pxa910_device_u2o = {
	.name		= "pxa-u2o",
	.id		= -1,
	.dev		= {
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &pxa910_u2o_info,
	},

	.num_resources	= ARRAY_SIZE(u2o_resources),
	.resource	= u2o_resources,
};
#endif

#ifdef CONFIG_USB_OTG
struct platform_device pxa910_device_u2ootg = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev		= {
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &pxa910_u2o_info,
	},

	.num_resources	= ARRAY_SIZE(u2ootg_resources),
	.resource	= u2ootg_resources,
};


static u64 ehci_hcd_pxa_dmamask = DMA_BIT_MASK(32);
struct platform_device pxa910_device_u2oehci = {
	.name		= "pxau2o-ehci",
	.id		= -1,
	.dev		= {
		.dma_mask		= &ehci_hcd_pxa_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &pxa910_u2o_info,
	},

	.num_resources	= ARRAY_SIZE(ehci_u2o_resources),
	.resource	= ehci_u2o_resources,
};
#endif

int pxa910_init_usb_phy(unsigned int base)
{
	return pxa_usb_phy_init(base);
}

int pxa910_deinit_usb_phy(unsigned int base)
{
	return pxa_usb_phy_deinit(base);
}

/* on-chip devices */

/* NOTE: there are totally 3 UARTs on PXA910:
 *
 *   UART1   - Slow UART (can be used both by AP and CP)
 *   UART2/3 - Fast UART
 *
 * To be backward compatible with the legacy FFUART/BTUART/STUART sequence,
 * they are re-ordered as:
 *
 *   pxa910_device_uart1 - UART2 as FFUART
 *   pxa910_device_uart2 - UART3 as BTUART
 *
 * UART1 is not used by AP for the moment.
 */
PXA910_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4017000, 0x30, 21, 22);
PXA910_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4018000, 0x30, 23, 24);
PXA910_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4036000, 0x30, 4, 5);
PXA910_DEVICE(pwm1, "pxa910-pwm", 0, NONE, 0xd401a000, 0x10);
PXA910_DEVICE(pwm2, "pxa910-pwm", 1, NONE, 0xd401a400, 0x10);
PXA910_DEVICE(pwm3, "pxa910-pwm", 2, NONE, 0xd401a800, 0x10);
PXA910_DEVICE(pwm4, "pxa910-pwm", 3, NONE, 0xd401ac00, 0x10);
PXA910_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x80, 97, 99);
PXA910_DEVICE(ire, "pxa910-ire", -1, IRE, 0xd420C000, 0x90);
PXA910_DEVICE(ssp0, "pxa910-ssp", 0, SSP0, 0xd401b000, 0x90, 52, 53);
PXA910_DEVICE(ssp1, "pxa910-ssp", 1, SSP1, 0xd42a0c00, 0x90,  1, 2);
PXA910_DEVICE(ssp2, "pxa910-ssp", 2, SSP2, 0xd401C000, 0x90, 60, 61);
PXA910_DEVICE(twsi0, "pxa2xx-i2c", 0, TWSI0, 0xd4011000, 0x28);
PXA910_DEVICE(twsi1, "pxa2xx-i2c", 1, TWSI1, 0xd4037000, 0x28);
PXA910_DEVICE(fb, "pxa910-fb", -1, LCD, 0xd420b000, 0x1ec);
PXA910_DEVICE(fb_ovly, "pxa910fb_ovly", -1, LCD, 0xd420b000, 0x1ec);
PXA910_DEVICE(onenand, "onenand", -1, NONE, 0x80000000, 0x100000);
PXA910_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
PXA910_DEVICE(sdh0, "pxa-sdh", 0, MMC, 0xd4280000, 0x100);
PXA910_DEVICE(sdh1, "pxa-sdh", 1, MMC, 0xd4280800, 0x100);
PXA910_DEVICE(sdh2, "pxa-sdh", 2, MMC, 0xd4281000, 0x100);
PXA910_DEVICE(sdh3, "pxa-sdh", 3, MMC2, 0xd427f000, 0x100);
PXA910_DEVICE(cnm, "pxa-cnm", -1, CNM, 0xd420d000, 0x1000);
PXA910_DEVICE(camera, "pxa910-camera", -1, CCIC, 0xd420a000, 0xfff);

static struct resource pxa910_resource_rtc[] = {
	[0] = {
		.start  = 0xd4010000,
		.end    = 0xD40100ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_PXA168_RTC_INT,
		.end    = IRQ_PXA168_RTC_INT,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_1HZ",
	},

	[2] = {
		.start  = IRQ_PXA168_RTC_ALARM,
		.end    = IRQ_PXA168_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_ALARM",
	},

};

struct platform_device pxa910_device_rtc = {
       .name           = "mmp-rtc",
       .id             = -1,
       .resource       = pxa910_resource_rtc,
       .num_resources  = ARRAY_SIZE(pxa910_resource_rtc),
};


