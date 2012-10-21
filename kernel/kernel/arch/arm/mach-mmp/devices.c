/*
 * linux/arch/arm/mach-mmp/devices.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/devices.h>

#include <plat/pxa_u2o.h>

int __init pxa_register_device(struct pxa_device_desc *desc,
				void *data, size_t size)
{
	struct platform_device *pdev;
	struct resource res[2 + MAX_RESOURCE_DMA];
	int i, ret = 0, nres = 0;

	pdev = platform_device_alloc(desc->drv_name, desc->id);
	if (pdev == NULL)
		return -ENOMEM;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	memset(res, 0, sizeof(res));

	if (desc->start != -1ul && desc->size > 0) {
		res[nres].start	= desc->start;
		res[nres].end	= desc->start + desc->size - 1;
		res[nres].flags	= IORESOURCE_MEM;
		nres++;
	}

	if (desc->irq != NO_IRQ) {
		res[nres].start	= desc->irq;
		res[nres].end	= desc->irq;
		res[nres].flags	= IORESOURCE_IRQ;
		nres++;
	}

	for (i = 0; i < MAX_RESOURCE_DMA; i++, nres++) {
		if (desc->dma[i] == 0)
			break;

		res[nres].start	= desc->dma[i];
		res[nres].end	= desc->dma[i];
		res[nres].flags	= IORESOURCE_DMA;
	}

	ret = platform_device_add_resources(pdev, res, nres);
	if (ret) {
		platform_device_put(pdev);
		return ret;
	}

	if (data && size) {
		ret = platform_device_add_data(pdev, data, size);
		if (ret) {
			platform_device_put(pdev);
			return ret;
		}
	}

	return platform_device_add(pdev);
}

#ifdef CONFIG_CPU_PXA910
static struct resource pxa910_resource_acipc[] = {
		[0] = {
				.start  = 0xD401D000,
				.end    = 0xD401D0ff,
				.flags  = IORESOURCE_MEM,
		},
		[1] = {
				.start  = IRQ_PXA910_IPC_AP_DATAACK,
				.end    = IRQ_PXA910_IPC_AP_DATAACK,
				.flags  = IORESOURCE_IRQ,
				.name   = "IPC_AP_DATAACK",
		},
		[2] = {
				.start  = IRQ_PXA910_IPC_AP_SET_CMD,
				.end    = IRQ_PXA910_IPC_AP_SET_CMD,
				.flags  = IORESOURCE_IRQ,
				.name   = "IPC_AP_SET_CMD",
		},
        [3] = {
				.start  = IRQ_PXA910_IPC_AP_SET_MSG,
				.end    = IRQ_PXA910_IPC_AP_SET_MSG,
				.flags  = IORESOURCE_IRQ,
				.name   = "IPC_AP_SET_MSG",
		},
};

struct platform_device pxa910_device_acipc = {
       .name           = "pxa9xx-acipc",
       .id             = -1,
       .resource       = pxa910_resource_acipc,
       .num_resources  = ARRAY_SIZE(pxa910_resource_acipc),
};
#endif


#if defined (CONFIG_USB) || defined (CONFIG_USB_GADGET)

/*****************************************************************************
 * The registers read/write routines
 *****************************************************************************/

unsigned int u2o_get(unsigned int base, unsigned int offset)
{
	return readl(base + offset);
}

void u2o_set(unsigned int base, unsigned int offset, unsigned int value)
{
	volatile unsigned int reg;

	reg = readl(base + offset);
	reg |= value;
	writel(reg, base + offset);
	__raw_readl(base + offset);

}

void u2o_clear(unsigned int base, unsigned int offset, unsigned int value)
{
	volatile unsigned int reg;

	reg = readl(base + offset);
	reg &= ~value;
	writel(reg, base + offset);
	__raw_readl(base + offset);
}

void u2o_write(unsigned int base, unsigned int offset, unsigned int value)
{
	writel(value, base + offset);
	__raw_readl(base + offset);

}

/********************************************************************
 * USB 2.0 OTG controller
 */
int pxa_usb_phy_init(unsigned int base)
{	
	static int init_done;
	int count;

	if (init_done) {
		printk(KERN_DEBUG "re-init phy\n\n");
		/* return; */
	}

	/* Initialize the USB PHY power */
	if (cpu_is_pxa910()) {
		u2o_set(base, UTMI_CTRL, (1<<UTMI_CTRL_INPKT_DELAY_SOF_SHIFT)
			| (1<<UTMI_CTRL_PU_REF_SHIFT));
	}

	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);
	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);
	
	/* UTMI_PLL settings */
	u2o_clear(base, UTMI_PLL, UTMI_PLL_PLLVDD18_MASK
		| UTMI_PLL_PLLVDD12_MASK | UTMI_PLL_PLLCALI12_MASK
		| UTMI_PLL_FBDIV_MASK | UTMI_PLL_REFDIV_MASK
		| UTMI_PLL_ICP_MASK | UTMI_PLL_KVCO_MASK);

	u2o_set(base, UTMI_PLL, 0xee<<UTMI_PLL_FBDIV_SHIFT
		| 0xb<<UTMI_PLL_REFDIV_SHIFT | 3<<UTMI_PLL_PLLVDD18_SHIFT
		| 3<<UTMI_PLL_PLLVDD12_SHIFT | 3<<UTMI_PLL_PLLCALI12_SHIFT
		| 1<<UTMI_PLL_ICP_SHIFT | 3<<UTMI_PLL_KVCO_SHIFT);

	/* UTMI_TX */
	u2o_clear(base, UTMI_TX, UTMI_TX_REG_EXT_FS_RCAL_EN_MASK | UTMI_TX_TXVDD12_MASK
		| UTMI_TX_CK60_PHSEL_MASK | UTMI_TX_IMPCAL_VTH_MASK
		| UTMI_TX_REG_EXT_FS_RCAL_MASK | UTMI_TX_AMP_MASK);
	u2o_set(base, UTMI_TX, 3<<UTMI_TX_TXVDD12_SHIFT
		| 4<<UTMI_TX_CK60_PHSEL_SHIFT | 4<<UTMI_TX_IMPCAL_VTH_SHIFT
		| 8<<UTMI_TX_REG_EXT_FS_RCAL_SHIFT | 3<<UTMI_TX_AMP_SHIFT);

	/* UTMI_RX */
	u2o_clear(base, UTMI_RX, UTMI_RX_SQ_THRESH_MASK
		| UTMI_REG_SQ_LENGTH_MASK);
	if (cpu_is_pxa168())
		u2o_set(base, UTMI_RX, 7<<UTMI_RX_SQ_THRESH_SHIFT
			| 2<<UTMI_REG_SQ_LENGTH_SHIFT);
	else
		u2o_set(base, UTMI_RX, 0x7<<UTMI_RX_SQ_THRESH_SHIFT
			| 2<<UTMI_REG_SQ_LENGTH_SHIFT);

	/* UTMI_IVREF */
	if (cpu_is_pxa168())
		/* fixing Microsoft Altair board interface with NEC hub issue -
		 * Set UTMI_IVREF from 0x4a3 to 0x4bf */
		u2o_write(base, UTMI_IVREF, 0x4bf);

	/* calibrate */
	count = 10000;
	while(((u2o_get(base, UTMI_PLL) & PLL_READY)==0) && count--);
	if (count <= 0) printk("%s %d: calibrate timeout, UTMI_PLL %x\n", 
		__func__, __LINE__, u2o_get(base, UTMI_PLL));

	/* toggle VCOCAL_START bit of UTMI_PLL */
	udelay(200);
	u2o_set(base, UTMI_PLL, VCOCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_PLL, VCOCAL_START);

	/* toggle REG_RCAL_START bit of UTMI_TX */
	udelay(200);
	u2o_set(base, UTMI_TX, REG_RCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_TX, REG_RCAL_START);
	udelay(200);

	/* make sure phy is ready */
	count = 1000;
	while(((u2o_get(base, UTMI_PLL) & PLL_READY)==0) && count--);
	if (count <= 0) printk("%s %d: calibrate timeout, UTMI_PLL %x\n", 
		__func__, __LINE__, u2o_get(base, UTMI_PLL));

	if (cpu_is_pxa168()) {
		u2o_set(base, UTMI_RESERVE, 1<<5);
		u2o_write(base, UTMI_OTG_ADDON, 1);  /* Turn on UTMI PHY OTG extension */
	}

	init_done = 1;
	return 0;
}

int pxa_usb_phy_deinit(unsigned int base)
{
	if (cpu_is_pxa168())
		u2o_clear(base, UTMI_OTG_ADDON, UTMI_OTG_ADDON_OTG_ON);

	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_RXBUF_PDWN);
	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_TXBUF_PDWN);
	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_USB_CLK_EN);
	u2o_clear(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);
	u2o_clear(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);

	return 0;
}

static u64 u2o_dma_mask = ~(u32)0;
struct resource pxa168_u2o_resources[] = {
	/* regbase */
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
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2o = {
	.name		= "pxa-u2o",
	.id		= -1,
	.resource	= pxa168_u2o_resources,
	.num_resources	= ARRAY_SIZE(pxa168_u2o_resources),
	.dev		=  {
		.dma_mask	= &u2o_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};
#endif

/********************************************************************
 * USB 2.0 Dedicated Host controller
 */
static u64 ehci_hcd_pxa_dmamask = DMA_BIT_MASK(32);
static void ehci_hcd_pxa_device_release(struct device *dev)
{
        /* Keep this function empty. */
}

#ifdef CONFIG_USB_EHCI_PXA_U2H
static struct resource pxa168_u2h_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2H_REGBASE,
		.end	= PXA168_U2H_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2h",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2H_PHYBASE,
		.end	= PXA168_U2H_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2hphy",
	},
	[2] = {
		.start	= IRQ_PXA168_USB2,
		.end	= IRQ_PXA168_USB2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2h = {
        .name = "pxau2h-ehci",
        .id   = -1,
        .dev  = {
                .dma_mask = &ehci_hcd_pxa_dmamask,
                .coherent_dma_mask = DMA_BIT_MASK(32),
                .release = ehci_hcd_pxa_device_release,
        },

        .num_resources = ARRAY_SIZE(pxa168_u2h_resources),
        .resource      = pxa168_u2h_resources,
};
#endif

#if defined(CONFIG_USB_PXA_U2O) && defined(CONFIG_USB_OTG)
struct resource pxa168_u2ootg_resources[] = {
	/* regbase */
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
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct resource pxa168_u2oehci_resources[] = {
	/* regbase */
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
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2ootg = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev  = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},

	.num_resources	= ARRAY_SIZE(pxa168_u2ootg_resources),
	.resource      = pxa168_u2ootg_resources,
};

struct platform_device pxa168_device_u2oehci = {
	.name = "pxau2o-ehci",
	.id = -1,
	.dev		= {
		.dma_mask = &ehci_hcd_pxa_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.release = ehci_hcd_pxa_device_release,
	},

	.num_resources = ARRAY_SIZE(pxa168_u2o_resources),
	.resource      = pxa168_u2oehci_resources,
};

#endif

static struct resource pxa910_resource_freq[] = {
	[0] = {
		.name   = "pmum_regs",
		.start	= 0xd4050000,
		.end	= 0xd4051050,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name   = "pmua_regs",
		.start	= 0xd4282800,
		.end	= 0xd4282900,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa910_device_freq = {
	.name		= "pxa168-freq",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(pxa910_resource_freq),
	.resource	= pxa910_resource_freq,
};


