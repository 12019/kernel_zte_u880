#ifndef ASMARM_ARCH_MMC_H
#define ASMARM_ARCH_MMC_H

#include <linux/mmc/host.h>
#include <linux/interrupt.h>

struct device;
struct mmc_host;

struct pxasdh_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	unsigned long detect_delay;		/* delay in jiffies before detecting cards after interrupt */
	int (*get_ro)(struct device *);
	unsigned int bus_width;
	unsigned int max_speed;

	/* SD_CLOCK_AND_BURST_SIZE_SETUP register */
	unsigned int sd_clock;			/* 1 for need to tuning sd clock */
	unsigned int sdclk_sel;
	unsigned int sdclk_delay;

#ifdef CONFIG_SD8XXX_RFKILL
	/*for sd8688-rfkill device*/
	struct mmc_host **pmmc;
#endif

	/* slot operations needed while going in/out low-power mode */
	int (*lp_switch)(unsigned int on, int with_card);
};
#endif
