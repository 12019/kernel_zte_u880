/*
 *  linux/drivers/mmc/host/pxa_sdh.c - PXAxxx SD Host driver
 *
 *  Copyright (C) 2008-2009 Marvell International Ltd.
 *                Philip Rakity <prakity@marvell.com>
 *
 *  Based on linux/drivers/mmc/host/sdhci-pci.c - PXA MMC shim driver
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/dma-mapping.h>

#include <mach/mmc.h>
#include <mach/cputype.h>

#include "sdhci.h"

#define SD_RESP_6	0x1c		/* Command Response 6 */
#define SD_RESP_7	0x1e		/* Command Response 7 */

#define DRIVER_NAME	"pxa-sdh"
#define MAX_SLOTS	8

#define DEFAULT_NO_DYNAMIC_CLOCKING	0

struct sdhci_mmc_slot {
	struct pxasdh_platform_data *pdata;
	struct sdhci_host	*host;
	struct clk	*clk;
	struct resource	*res;
	u32	clkrate;
	u32	f_max;
	u8	width;
	u8	eightBitEnabled;
	u8	clockEnabled;
	u8	no_dynamic_SD_clocking;
};
#define DBG(f, x...) \
	pr_debug(DRIVER_NAME " [%s()]: " f, __func__,## x)

static void inline programFIFO (struct sdhci_host *host)
{
	struct sdhci_mmc_slot *slot = sdhci_priv(host);
	unsigned short tmp;

	tmp = readw(host->ioaddr + SD_FIFO_PARAM);
	DBG("ENTER %s SD_FIFO_PARAM = %04X\n", mmc_hostname(host->mmc), tmp);

	tmp &= ~CLK_GATE_SETTING_BITS;
	if (slot->no_dynamic_SD_clocking)
		tmp |= CLK_GATE_SETTING_BITS;

	writew(tmp, host->ioaddr + SD_FIFO_PARAM);
	DBG("EXIT: %s SD_FIFO_PARAM = %04X\n", mmc_hostname(host->mmc), tmp);
}

static int platform_supports_8_bit(struct sdhci_host *host)
{
	struct sdhci_mmc_slot *slot = sdhci_priv(host);

	return slot->width >= 8;
}

#define SD_CE_ATA_2		0xEA
#define MMC_CARD	(1<<12)
#define	MMC_WIDTH	(1<<8)

static void platform_clear_8_bit(struct sdhci_host *host)
{
	unsigned short tmp;
	struct sdhci_mmc_slot *slot = sdhci_priv(host);

	tmp = readw(host->ioaddr + SD_CE_ATA_2);
	tmp &= ~(MMC_CARD | MMC_WIDTH);
	writew(tmp, host->ioaddr + SD_CE_ATA_2);
	slot->eightBitEnabled = 0;
	DBG("EXIT: %s SD_CE_ATA_2 = %04X\n", mmc_hostname(host->mmc), tmp);
}

static void platform_set_8_bit(struct sdhci_host *host)
{
	unsigned short tmp;
	struct sdhci_mmc_slot *slot = sdhci_priv(host);

	tmp = readw(host->ioaddr + SD_CE_ATA_2);
	tmp |= MMC_CARD | MMC_WIDTH;
	writew(tmp, host->ioaddr + SD_CE_ATA_2);
	slot->eightBitEnabled = 1;
	DBG("EXIT: %s SD_CE_ATA_2 = %04X\n", mmc_hostname(host->mmc), tmp);
}

static int platform_init_after_reset (struct sdhci_host *host)
{
	struct sdhci_mmc_slot *slot = sdhci_priv(host);
	u16 tmp;

	programFIFO(host);

	if (platform_supports_8_bit(host)) {
		if (slot->eightBitEnabled)
			platform_set_8_bit(host);
		else
			platform_clear_8_bit(host);
	}
	if (slot->pdata && slot->pdata->sd_clock) {
		tmp = readw(host->ioaddr + SD_CLOCK_AND_BURST_SIZE_SETUP);
		tmp &= ~(SDCLK_DELAY_MASK << SDCLK_DELAY_SHIFT);
		tmp &= ~(SDCLK_SEL_MASK << SDCLK_SEL_SHIFT);
		tmp |= (slot->pdata->sdclk_delay & SDCLK_DELAY_MASK) <<
			SDCLK_DELAY_SHIFT;
		tmp |= (slot->pdata->sdclk_sel & SDCLK_SEL_MASK) <<
			SDCLK_SEL_SHIFT;
		writew(tmp, host->ioaddr + SD_CLOCK_AND_BURST_SIZE_SETUP);
	}

	DBG ("SD_CLOCK_AND_BURST_SIZE_SETUP to %04X\n", readw(host->ioaddr + SD_CLOCK_AND_BURST_SIZE_SETUP));
	return 0;
}

static void platform_specific_sdio (struct sdhci_host *host, int enable)
{
	struct sdhci_mmc_slot *slot = sdhci_priv(host);

	DBG("ENTER: %s enable = %d, no_dynamic_SD_clocking = %d\n",
		mmc_hostname(host->mmc), enable, slot->no_dynamic_SD_clocking);

	if (enable) {
		slot->no_dynamic_SD_clocking = 1;
		programFIFO(host);
	}
	else
	{
		slot->no_dynamic_SD_clocking = 0;
		programFIFO(host);
	}
	DBG("EXIT: %s enable = %d, no_dynamic_SD_clocking = %d\n",
		mmc_hostname(host->mmc), enable, slot->no_dynamic_SD_clocking);

}

static void platform_specific_reset (struct sdhci_host *host, u8 mask)
{
	if (mask & SDHCI_RESET_ALL) {
		DBG("%s: run %s %s\n", mmc_hostname(host->mmc), DRIVER_NAME, __FUNCTION__);
		platform_init_after_reset (host);
	}

}

static int platform_specific_get_ro(struct mmc_host *mmc)
{
	struct sdhci_mmc_slot *slot = NULL;
	struct sdhci_host *host;

	host = mmc_priv(mmc);
	slot = sdhci_priv(host);

	if (slot->pdata && slot->pdata->get_ro)
		return !!slot->pdata->get_ro(mmc_dev(mmc));
	/*
	 * Board doesn't support read only detection; let the mmc core
	 * decide what to do.
	 */
	return -ENOSYS;
}

static void enable_clock (struct sdhci_host *host)
{
	struct sdhci_mmc_slot *slot = sdhci_priv(host);

	DBG("ENTER: %s slot->clockEnabled = %d\n", mmc_hostname(host->mmc), slot->clockEnabled);

	if (slot->clockEnabled == 0) {
		clk_enable(slot->clk);
		slot->clockEnabled = 1;
	}
}


static void disable_clock (struct sdhci_host *host)
{
	struct sdhci_mmc_slot *slot = sdhci_priv(host);

	DBG("ENTER:\n");

	if (slot->clockEnabled) {
		clk_disable(slot->clk);
		slot->clockEnabled = 0;
	}
}

static void set_clock (struct sdhci_host *host, unsigned int clock)
{
	DBG("ENTER: clock = %d\n", clock);

	if (clock == 0)
		disable_clock(host);
	else
		enable_clock(host);
}

static unsigned int get_max_clock (struct sdhci_host *host)
{
	struct sdhci_mmc_slot *slot = sdhci_priv(host);
	unsigned int clkrate = slot->clkrate;

	DBG("EXIT: slot->clkrate = %u \n", clkrate);

	return clkrate;
}

static unsigned int get_f_max_clock (struct sdhci_host *host)
{
	struct sdhci_mmc_slot *slot = sdhci_priv(host);
	unsigned int f_max;

	f_max = slot->f_max;

	DBG("EXIT: %s f_max = %u\n", mmc_hostname(host->mmc), f_max);

	return f_max;
}

static inline u16 pxa168_readw(struct sdhci_host *host, int reg)
{
	u32 temp;
	if (reg == SDHCI_HOST_VERSION) {
		temp = readl (host->ioaddr + SDHCI_HOST_VERSION - 2) >> 16;
		return temp & 0xffff;
	}

	return readw(host->ioaddr + reg);
}


static struct sdhci_ops sdhci_mmc_ops = {
	.sd_readw = pxa168_readw,
	.platform_specific_reset = platform_specific_reset,
	.platform_specific_sdio = platform_specific_sdio,
	.get_max_clock = get_max_clock,
	.get_f_max_clock = get_f_max_clock,
	.set_clock = set_clock,
	.platform_supports_8_bit = platform_supports_8_bit,
	.platform_set_8_bit = platform_set_8_bit,
	.platform_clear_8_bit = platform_clear_8_bit,
};


static void sdhci_mmc_remove_slot(struct sdhci_mmc_slot *slot)
{
	int dead;
	u32 scratch;

	DBG("ENTER %s\n", mmc_hostname(slot->host->mmc));
	dead = 0;
	scratch = readl(slot->host->ioaddr + SDHCI_INT_STATUS);
	if (scratch == (u32)-1)
		dead = 1;

	sdhci_remove_host(slot->host, dead);

	free_irq(slot->host->irq, slot->host);
	if (slot->host->ioaddr)
		iounmap(slot->host->ioaddr);
	release_mem_region(slot->res->start, SZ_256);
	sdhci_free_host(slot->host);
}

static int pxa_sdh_probe(struct platform_device *pdev)
{
	struct sdhci_host *host = NULL;
	struct resource *r = NULL;
	int ret;
	int irq;
	struct sdhci_mmc_slot *slot = NULL;

	host = sdhci_alloc_host(&pdev->dev, sizeof(struct sdhci_mmc_slot));
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		goto out;
	}
	host->hw_name = "MMC";
	platform_set_drvdata(pdev, host);

	slot = sdhci_priv(host);
	slot->host = host;
	slot->pdata = pdev->dev.platform_data;
	slot->no_dynamic_SD_clocking = DEFAULT_NO_DYNAMIC_CLOCKING;
	slot->eightBitEnabled = 0;
	slot->clockEnabled = 0;
	slot->clk = clk_get(&pdev->dev, "PXA-SDHCLK");
	if (slot->clk == NULL) {
		ret = -ENXIO;
		goto err;
	}
	slot->clkrate = clk_get_rate(slot->clk);
	if (slot->pdata->max_speed)
		slot->f_max = slot->pdata->max_speed;
	else
		slot->f_max = slot->clkrate;

	if (slot->pdata->bus_width)
		slot->width = slot->pdata->bus_width;
	else
		slot->width = 4;

	DBG("%s: slot->width = %d\n", mmc_hostname(host->mmc), slot->width);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	irq = platform_get_irq(pdev, 0);
	DBG("platform_get_resource = %p\n", r);
	if (!r || irq < 0) {
		ret = -ENXIO;
		goto err;
	}
	host->irq = irq;

	r = request_mem_region(r->start, SZ_256, DRIVER_NAME);
	DBG("request_mem_region = %p\n", r);
	if (!r) {
		ret = -EBUSY;
		goto out;
	}
	host->ioaddr = ioremap(r->start, SZ_256);
	if (!host->ioaddr) {
		ret = -ENOMEM;
		goto out;
	}
	slot->res = r;

#if 0
	host->quirks = SDHCI_QUIRK_BROKEN_ADMA | SDHCI_QUIRK_RESET_AFTER_REQUEST
		| SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;
#else
        host->quirks = SDHCI_QUIRK_BROKEN_ADMA | SDHCI_QUIRK_BROKEN_TIMEOUT_VAL | SDHCI_QUIRK_BROKEN_CARD_DETECTION;   

#endif

	enable_clock(host);

	platform_init_after_reset(host);

	host->mmc->pm_caps = MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ |
		MMC_PM_SKIP_RESUME_PROBE;

	host->ops = &sdhci_mmc_ops;
	if (slot->pdata->get_ro)
		sdhci_mmc_ops.platform_specific_get_ro =
			&platform_specific_get_ro;

	ret = sdhci_add_host(host);
	if (ret)
		goto out;

#ifdef CONFIG_SD8XXX_RFKILL
	if (slot->pdata->pmmc)
		*slot->pdata->pmmc = host->mmc;
#endif

	DBG ("Exit %s\n", mmc_hostname(host->mmc));
	return 0;

 out:
	DBG ("%s ERROR EXIT\n", __FUNCTION__);
	if (host) {
		if (host->ioaddr)
			iounmap(host->ioaddr);
			sdhci_free_host(host);
	}
	if (r)
		release_mem_region(r->start, SZ_256);

	if (slot && slot->clk)
	{
		if (slot->clockEnabled)
			clk_disable(slot->clk);
		slot->clockEnabled = 0;
	}

err:
	return ret;
}

static int __devexit pxa_sdh_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = NULL;
	struct sdhci_mmc_slot *slot = NULL;

	DBG("%s: ENTER\n", __FUNCTION__);

	host = platform_get_drvdata(pdev);
	slot = sdhci_priv(host);

	sdhci_mmc_remove_slot(slot);
	platform_set_drvdata(pdev, NULL);
	clk_disable(slot->clk);

	return 0;
}

#ifdef CONFIG_PM
static int pxa_sdh_suspend(struct platform_device *dev, pm_message_t state)
{
	struct sdhci_host *host = NULL;
	struct sdhci_mmc_slot *slot = NULL;
	int ret = 0;

	host = platform_get_drvdata(dev);
	slot = sdhci_priv(host);

	DBG("%s: ENTER, %s\n", __FUNCTION__, mmc_hostname(host->mmc));

	ret = sdhci_suspend_host(host, state);
	if(ret)
		return ret;

	if (slot->pdata->lp_switch){
		mdelay(10); //fix 36mA wifi suspend current abnormal
		ret = slot->pdata->lp_switch(1, (int)host->mmc->card);
		if(ret){
			slot->pdata->lp_switch(0, (int)host->mmc->card);

			sdhci_resume_host(host);
			DBG("SDHCI switch gpio failed, resume sdhci.\n");
		}
	}

	DBG("%s: EXIT, %s, returns %d\n", __FUNCTION__,
		mmc_hostname(host->mmc), ret);
	return ret;
}

static int pxa_sdh_resume(struct platform_device *dev)
{
	struct sdhci_host *host = NULL;
	struct sdhci_mmc_slot *slot = NULL;
	int ret = 0;

	host = platform_get_drvdata(dev);
	slot = sdhci_priv(host);

	DBG("%s: ENTER, %s\n", __FUNCTION__, mmc_hostname(host->mmc));

	if (slot->pdata->lp_switch)
		slot->pdata->lp_switch(0, (int)host->mmc->card);

	ret = sdhci_resume_host(host);

	DBG("%s: EXIT, %s, returns %d\n", __FUNCTION__,
		mmc_hostname(host->mmc), ret);
	return ret;
}
#else
#define pxa_sdh_suspend	NULL
#define pxa_sdh_resume	NULL
#endif

static void pxa_sdh_shutdown(struct platform_device *dev)
{	
	struct sdhci_host *host = NULL;	
	struct sdhci_mmc_slot *slot = NULL;	

	host = platform_get_drvdata(dev);	
	slot = sdhci_priv(host);		
	if (slot->pdata->lp_switch)		
		slot->pdata->lp_switch(1, (int)host->mmc->card);
}

static struct platform_driver pxa_sdh_driver = {
	.probe		= pxa_sdh_probe,
	.remove		= pxa_sdh_remove,
	.suspend	= pxa_sdh_suspend,
	.resume		= pxa_sdh_resume,
	.shutdown = pxa_sdh_shutdown,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init pxa_sdh_init(void)
{
	return platform_driver_register(&pxa_sdh_driver);
}

static void __exit pxa_sdh_exit(void)
{
	platform_driver_unregister(&pxa_sdh_driver);
}

module_init(pxa_sdh_init);
module_exit(pxa_sdh_exit);

MODULE_AUTHOR("Philip Rakity <prakity@marvell.com>");
MODULE_DESCRIPTION("PXA SD Host Controller(MMC) Interface");
MODULE_LICENSE("GPL");
