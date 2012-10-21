/*
 * linux/arch/arm/mach-mmp/include/mach/system.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_SYSTEM_H
#define __ASM_MACH_SYSTEM_H

#include <mach/regs-mpmu.h>
#include <mach/regs-timers.h>
#include <mach/cputype.h>
#include <linux/mfd/88pm860x.h>

#define REG_RTC_BR0	(APB_VIRT_BASE + 0x010014)
#define REG_RTC_BR1	(APB_VIRT_BASE + 0x010018)

#define MPMU_APRR_WDTR	(1<<4)
#define MPMU_APRR_CPR		(1<<0)
#define MPMU_CPRR_DSPR		(1<<2)
#define MPMU_CPRR_BBR		(1<<3)

#define RTCSTATEFLAG		0xB6
#define RTCSTATEFLAGS		0xB6B6B6B6

typedef enum {
        REBOOT_NORMAL,
        //REBOOT_FACTORY,
        //REBOOT_FLASH,
        REBOOT_RECOVERY,
        REBOOT_FOTA,
        REBOOT_ASSERT,
        REBOOT_ALARM,	
       // REBOOT_DKB=0xfd,
        REBOOT_MAX
} reboot_setting_t;

extern void __iomem *reboot_pwr_ram_base;

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	u32 reg, backup;
	u32 watchdog_virt_base;
	int i;
	int match=0, count=0;
	char *p = (char *)reboot_pwr_ram_base;

	if (cpu_is_pxa168())
		watchdog_virt_base = TIMERS1_VIRT_BASE;
	else if (cpu_is_pxa910())
		watchdog_virt_base = CP_TIMERS2_VIRT_BASE;
	else
		return ;

	/* reset/enable WDT clock */
	writel(0x7, MPMU_WDTPCR);
	readl(MPMU_WDTPCR);
	writel(0x3, MPMU_WDTPCR);
	readl(MPMU_WDTPCR);
	
	do {
		writel(REBOOT_NORMAL, REG_RTC_BR0);
	} while (readl(REG_RTC_BR0) != REBOOT_NORMAL);
	
	do {
		writel(RTCSTATEFLAGS, REG_RTC_BR1);
	} while (readl(REG_RTC_BR1) != RTCSTATEFLAGS);
	
	if (cmd && !strcmp(cmd, "recovery")) {
		#if 0
		for (i = 0, backup = 0; i < 4; i++) {
			backup <<= 8;
			backup |= *(cmd + i);
		}
		#endif
		do {
			writel(REBOOT_RECOVERY, REG_RTC_BR0);
		} while (readl(REG_RTC_BR0) != REBOOT_RECOVERY);
		
		if(reboot_pwr_ram_base) {
			memset(p, RTCSTATEFLAG, 4);
			memset(p+4, REBOOT_RECOVERY, 1);
		}
	}
	else if (cmd && !strcmp(cmd, "alarm")) {
		do {
			writel(REBOOT_ALARM, REG_RTC_BR0);
		} while (readl(REG_RTC_BR0) != REBOOT_ALARM);
		
		if(reboot_pwr_ram_base) {
			memset(p, RTCSTATEFLAG, 4);
			memset(p+4, REBOOT_ALARM, 1);
		}
	}
	else {
		if(reboot_pwr_ram_base) {
			memset(p, RTCSTATEFLAG, 4);
			memset(p+4, REBOOT_NORMAL, 1);
		}
	}

	/* enable WDT reset */
	writel(0xbaba, watchdog_virt_base + TMR_WFAR);
	writel(0xeb10, watchdog_virt_base + TMR_WSAR);
	writel(0x3, watchdog_virt_base + TMR_WMER);

	/*hold CP first*/
	reg = readl(MPMU_APRR) | MPMU_APRR_CPR;
	writel(reg, MPMU_APRR);
	for (i=0; i<10; i++);
	/*CP reset MSA*/
	reg = readl(MPMU_CPRR) | MPMU_CPRR_DSPR | MPMU_CPRR_BBR;
	writel(reg, MPMU_CPRR);
	for (i=0; i<10; i++);
	/* negate hardware reset to the WDT after system reset */
	reg = readl(MPMU_APRR) | MPMU_APRR_WDTR;
	writel(reg, MPMU_APRR);

	/* clear previous WDT status */
	writel(0xbaba, watchdog_virt_base + TMR_WFAR);
	writel(0xeb10, watchdog_virt_base + TMR_WSAR);
	writel(0, watchdog_virt_base + TMR_WSR);

	match = readl(watchdog_virt_base + TMR_WMR);
	count = readl(watchdog_virt_base + TMR_WVR);

	if (match - count > 0x20) {
		/* set match counter */
		writel(0xbaba, watchdog_virt_base + TMR_WFAR);
		writel(0xeb10, watchdog_virt_base + TMR_WSAR);
		writel(0x20+count, watchdog_virt_base + TMR_WMR);
	}
	/*avoid reboot fail due to power off charge feature, set pmic 0x3e:0,
	simulate on-key detect event in rdinit script*/
	pm860x_codec_reg_write(0x3e,0x1);
}
#endif /* __ASM_MACH_SYSTEM_H */
