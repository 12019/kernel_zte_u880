/*
 * PXA910 Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/kobject.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/addr-map.h>
#include <mach/dma.h>
#include <mach/gpio.h>
#include <mach/pxa910-squ.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-icu.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-ciu.h>
#include <mach/regs-timers.h>
#include <mach/pxa168_pm.h>
#include <mach/pxa910_pm.h>
#include <mach/pxa910_dvfm.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/mfd/88pm860x.h>
#include <mach/mfp-pxa910.h>
#include <linux/wakelock.h>

static struct wake_lock system_wakeup;

#define TDI         0x3
#define TRST        0x2
#define TMS         0x6
#define TCK         0xa
#define TDITCK      0xb
#define TMSTCK      0xe
#define TDITMSTCK   0xf

int enable_deepidle = IDLE_CORE_INTIDLE | IDLE_CORE_EXTIDLE |
	IDLE_APPS_IDLE | IDLE_APPS_SLEEP | IDLE_SYS_SLEEP;
static unsigned long pm_state;
static int cur_lpmode = -1;
static unsigned int pm_irqs[] = {
	//IRQ_PXA168_KEYPAD,
	IRQ_PXA168_PMIC_INT,
	IRQ_PXA168_RTC_ALARM,
	IRQ_PXA168_TWSI0,
	IRQ_PXA910_IPC_AP_DATAACK,
	IRQ_PXA910_IPC_AP_SET_CMD,
	IRQ_PXA910_IPC_AP_SET_MSG,
	IRQ_PXA168_GPIOX,
};
static struct pxa910_peripheral_config_ops *wakeup_ops = NULL;
static int wakeup_detect;
enum {
	SUSPEND = 0,
	RESUME = 1,
};
/*whether in suspend or resume process*/
static int cur_state = -1;
/*flag used to exit suspend process*/
static int suspend_exit = 0;

extern void pxa910_cpu_disable_l2(void* param);
extern void pxa910_cpu_enable_l2(void* param);

/* pxa910_suspend_exit_set API is added as a supplicant to wake_lock,
 * for wake_lock is only checked before suspend procedure begin to
 * prevent system suspend. Any event during suspend procedure that
 * want to stop system entering suspend should call this API if it
 * has no other method to stop it.
 */
void pxa910_suspend_exit_set()
{
	if(cur_state == SUSPEND)
		suspend_exit = 1;
}

/*clr exit suspend flag.*/
void pxa910_suspend_exit_clr()
{
	if(cur_state == SUSPEND)
		suspend_exit = 0;
}

int pxa910_suspend_exit_status()
{
	return suspend_exit;
}

int pxa910_power_config_register(struct pxa910_peripheral_config_ops *ops)
{
	wakeup_ops = ops;
	return 0;
}

void pxa910_power_config_unregister(void)
{
	wakeup_ops = NULL;
}

static uint32_t setup_wakeup_sources(int state)
{
	uint32_t apcr, awucrm;

	awucrm = 0x0;
	//awucrm |= PMUM_GSM_WAKEUPWMX;
	//awucrm |= PMUM_WCDMA_WAKEUPX;
	//awucrm |= PMUM_GSM_WAKEUPWM;
	//awucrm |= PMUM_WCDMA_WAKEUPWM;
	awucrm |= PMUM_AP_ASYNC_INT;
	awucrm |= PMUM_AP_FULL_IDLE;
	//awucrm |= PMUM_SDH2;
	//awucrm |= PMUM_TRACKBALL;
	//awucrm |= PMUM_NEWROTARY;
	if (state != POWER_MODE_UDR) {
	awucrm |= PMUM_WDT;
		awucrm |= PMUM_SDH1;
		awucrm |= PMUM_KEYPRESS;
		awucrm |= PMUM_AP1_TIMER_2;
	}
	awucrm |= PMUM_RTC_ALARM;
	//awucrm |= PMUM_CP_TIMER_3;
	//awucrm |= PMUM_CP_TIMER_2;
	//awucrm |= PMUM_CP_TIMER_1;
	//awucrm |= PMUM_AP2_TIMER_3;
	//awucrm |= PMUM_AP2_TIMER_2;
	//awucrm |= PMUM_AP2_TIMER_1;
	//awucrm |= PMUM_AP1_TIMER_3;
	//awucrm |= PMUM_AP1_TIMER_2;
	awucrm |= PMUM_WAKEUP7;
	awucrm |= PMUM_WAKEUP6;
	//awucrm |= PMUM_WAKEUP5;
	awucrm |= PMUM_WAKEUP4;
	awucrm |= PMUM_WAKEUP3;
	awucrm |= PMUM_WAKEUP2;
	//awucrm |= PMUM_WAKEUP1;
	//awucrm |= PMUM_WAKEUP0;

	__raw_writel(awucrm, MPMU_AWUCRM);

	apcr = 0xff087fff;	/* enable all wake-up ports */

	return apcr;
}

void pxa910_pm_enter_lowpower_mode(int state)
{
	uint32_t idle_cfg, apcr;

	if (state == cur_lpmode)
		return;

	cur_lpmode = state;
	idle_cfg = __raw_readl(APMU_IDLE_CFG);
	apcr = __raw_readl(MPMU_APCR);

	apcr &= ~PMUM_DDRCORSD & ~PMUM_APBSD & ~PMUM_AXISD;
	apcr &= ~PMUM_VCTCXOSD;
	idle_cfg &= ~PMUA_MOH_IDLE;
	idle_cfg &= ~PMUA_MOH_PWRDWN;
	apcr &= ~PMUM_STBYEN;

	switch (state) {
	case POWER_MODE_UDR:
		apcr |= PMUM_STBYEN | PMUM_APBSD;	/* only shutdown APB in UDR */
		/* fall through */
	case POWER_MODE_SYS_SLEEP:
		apcr |= PMUM_SLPEN;			/* set the SLPEN bit */
		apcr |= PMUM_VCTCXOSD;			/* set VCTCXOSD */
		apcr &= setup_wakeup_sources(state);		/* set up wakeup sources */
		/* fall through */
	case POWER_MODE_APPS_SLEEP:
		apcr |= PMUM_DDRCORSD;			/* set DDRCORSD */
		/* fall through */
	case POWER_MODE_APPS_IDLE:
		apcr |= PMUM_AXISD;			/* set AXISDD bit */
		/* fall through */
	case POWER_MODE_CORE_EXTIDLE:
		idle_cfg |= PMUA_MOH_IDLE;		/* set the IDLE bit */
		idle_cfg |= PMUA_MOH_PWRDWN;		/* set the PWRDWN bit */
		idle_cfg |= 0x000f0000;			/* number of power switch to 4 */
		/* fall through */
	case POWER_MODE_CORE_INTIDLE:
		break;
	}

	/* program the memory controller hardware sleep type and auto wakeup */
	idle_cfg |= PMUA_MOH_DIS_MC_SW_REQ;
	idle_cfg |= PMUA_MOH_MC_WAKE_EN;
	__raw_writel(0x0, APMU_MC_HW_SLP_TYPE);		/* auto refresh */

	/* set DSPSD, DTCMSD, BBSD, MSASLPEN */
	apcr |= PMUM_DSPSD | PMUM_DTCMSD | PMUM_BBSD | PMUM_MSASLPEN;

	/*always set SLEPEN bit mainly for MSA*/
	apcr |= PMUM_SLPEN;

	/* finally write the registers back */
	__raw_writel(idle_cfg, APMU_IDLE_CFG);
	__raw_writel(apcr, MPMU_APCR);

	//__raw_writel(0x20807, APMU_PWR_STBL_TIMER);
	//__raw_writel(0x1, APMU_SRAM_PWR_DWN);

	//__raw_writel(0x0, APMU_MC_SW_SLP_TYPE);
	//__raw_writel(0x1, APMU_MC_SLP_REQ);

	//__raw_writel(0x0, MPMU_VRCR);
}

#define MAXTOKENS 80

static int tokenizer(char **tbuf, const char *userbuf, ssize_t n,
			char **tokptrs, int maxtoks)
{
	char *cp, *tok;
	char *whitespace = " \t\r\n";
	int ntoks = 0;
	cp = kmalloc(n + 1, GFP_KERNEL);
	if (!cp)
		return -ENOMEM;

	*tbuf = cp;
	memcpy(cp, userbuf, n);
	cp[n] = '\0';

	do {
		cp = cp + strspn(cp, whitespace);
		tok = strsep(&cp, whitespace);
		if ((*tok == '\0') || (ntoks == maxtoks))
			break;
		tokptrs[ntoks++] = tok;
	} while (cp);

	return ntoks;
}

static ssize_t deepidle_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int len = 0;

	if (enable_deepidle & IDLE_CORE_INTIDLE)
		len += sprintf(buf + len, "core_intidle ");
	if (enable_deepidle & IDLE_CORE_EXTIDLE)
		len += sprintf(buf + len, "core_extidle ");
	if (enable_deepidle & IDLE_APPS_IDLE)
		len += sprintf(buf + len, "apps_idle ");
	if (enable_deepidle & IDLE_APPS_SLEEP)
		len += sprintf(buf + len, "apps_sleep ");
	if (enable_deepidle & IDLE_SYS_SLEEP)
		len += sprintf(buf + len, "sys_sleep");
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "Usage: echo [set|unset] "
		"[core_intidle|core_extidle|apps_idle|apps_sleep|sys_sleep] "
		"> deepidle\n");
	return len;
}

static ssize_t deepidle_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int error = 0;
	char *tbuf = NULL;
	char *token[MAXTOKENS];
	int ntoks = tokenizer(&tbuf, buf, len, (char **)&token, MAXTOKENS);

	if (ntoks <= 0) {
		error = ntoks;
		goto out;
	}

	if (strcmp(token[0], "set") == 0) {
		if (strcmp(token[1], "core_intidle") == 0)
			enable_deepidle |= IDLE_CORE_INTIDLE;
		else if (strcmp(token[1], "core_extidle") == 0)
			enable_deepidle |= IDLE_CORE_EXTIDLE;
		else if (strcmp(token[1], "apps_idle") == 0)
			enable_deepidle |= IDLE_APPS_IDLE;
		else if (strcmp(token[1], "apps_sleep") == 0)
			enable_deepidle |= IDLE_APPS_SLEEP;
		else if (strcmp(token[1], "sys_sleep") == 0)
			enable_deepidle |= IDLE_SYS_SLEEP;
		else
			error = -EINVAL;
	} else if (strcmp(token[0], "unset") == 0) {
		if (strcmp(token[1], "core_intidle") == 0)
			enable_deepidle &= ~IDLE_CORE_INTIDLE;
		else if (strcmp(token[1], "core_extidle") == 0)
			enable_deepidle &= ~IDLE_CORE_EXTIDLE;
		else if (strcmp(token[1], "apps_idle") == 0)
			enable_deepidle &= ~IDLE_APPS_IDLE;
		else if (strcmp(token[1], "apps_sleep") == 0)
			enable_deepidle &= ~IDLE_APPS_SLEEP;
		else if (strcmp(token[1], "sys_sleep") == 0)
			enable_deepidle &= ~IDLE_SYS_SLEEP;
		else
			error = -EINVAL;
	} else {
		if (strcmp(token[0], "0") == 0)
			enable_deepidle = IDLE_ACTIVE;
		else
			error = -EINVAL;
	}
out:
	kfree(tbuf);
	return error ? error : len;
}

static struct kobj_attribute deepidle_attr = {
	.attr	= {
		.name = __stringify(deepidle),
		.mode = 0644,
	},
	.show	= deepidle_show,
	.store	= deepidle_store,
};

static ssize_t regdump_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int len = 0;
	unsigned int addr;
	uint32_t id_code, cache_type, L2_system_id, L2_cache_type, control,
		auxiliary_control, translation_table_base, domain_access_control,
		data_fault_status, intruction_fault_status, fault_address, fcse_pid,
		context_id, coprocessor_enable, control_configuration,
		privileged_mode_access, cpu_id_code_extension;

	/*len += sprintf(buf + len, "apcr 0x%x, cpcr: 0x%x, "
		"apidle 0x%x, cpidle 0x%x, "
		"apwake 0x%x, cpwake 0x%x\n",
		__raw_readl(MPMU_APCR), __raw_readl(MPMU_CPCR),
		__raw_readl(APMU_IDLE_CFG), __raw_readl(APMU_CP_IDLE_CFG),
		__raw_readl(MPMU_AWUCRM), __raw_readl(MPMU_CWUCRM));*/
	for (addr = MPMU_CPCR; addr <= (MPMU_CPCR+0x4c); addr += 4)
		len += sprintf(buf + len, "0x%x: 0x%x\n", addr, __raw_readl(addr));
	for (addr = MPMU_APCR; addr <= (MPMU_APCR+0x4c); addr += 4)
		len += sprintf(buf + len, "0x%x: 0x%x\n", addr, __raw_readl(addr));
	for (addr = (APMU_CCR-4); addr <= APMU_SMC_CLK_RES_CTRL; addr += 4)
		len += sprintf(buf + len, "0x%x: 0x%x\n", addr, __raw_readl(addr));
	__raw_writel(__raw_readl(APMU_CP_CCR) | 0x80000000, APMU_CP_CCR);
	__raw_writel(__raw_readl(APMU_CP_CCR) & ~0x80000000, APMU_CP_CCR);
	len += sprintf(buf + len, "\n");

	for (addr = CIU_CHIP_ID; addr <= (CIU_CHIP_ID+0x9c); addr += 4)
		len += sprintf(buf + len, "0x%x: 0x%x\n", addr, __raw_readl(addr));
	len += sprintf(buf + len, "\n");

	__asm__ __volatile__("mrc p15, 0, %0, c0,  c0, 0" :"=r"(id_code));
	__asm__ __volatile__("mrc p15, 0, %0, c0,  c0, 1" :"=r"(cache_type));
	__asm__ __volatile__("mrc p15, 1, %0, c0,  c0, 0" :"=r"(L2_system_id));
	__asm__ __volatile__("mrc p15, 1, %0, c0,  c0, 1" :"=r"(L2_cache_type));
	__asm__ __volatile__("mrc p15, 0, %0, c1,  c0, 0" :"=r"(control));
	__asm__ __volatile__("mrc p15, 0, %0, c1,  c0, 1" :"=r"(auxiliary_control));
	__asm__ __volatile__("mrc p15, 0, %0, c2,  c0, 0" :"=r"(translation_table_base));
	__asm__ __volatile__("mrc p15, 0, %0, c3,  c0, 0" :"=r"(domain_access_control));
	__asm__ __volatile__("mrc p15, 0, %0, c5,  c0, 0" :"=r"(data_fault_status));
	__asm__ __volatile__("mrc p15, 0, %0, c5,  c0, 1" :"=r"(intruction_fault_status));
	__asm__ __volatile__("mrc p15, 0, %0, c6,  c0, 0" :"=r"(fault_address));
	__asm__ __volatile__("mrc p15, 0, %0, c13, c0, 0" :"=r"(fcse_pid));
	__asm__ __volatile__("mrc p15, 0, %0, c13, c0, 1" :"=r"(context_id));
	__asm__ __volatile__("mrc p15, 0, %0, c15, c1, 0" :"=r"(coprocessor_enable));
	__asm__ __volatile__("mrc p15, 1, %0, c15, c1, 0" :"=r"(control_configuration));
	__asm__ __volatile__("mrc p15, 0, %0, c15, c9, 0" :"=r"(privileged_mode_access));
	__asm__ __volatile__("mrc p15, 1, %0, c15, c12,0" :"=r"(cpu_id_code_extension));
	len += sprintf(buf + len,
		"id_code: 0x%x\ncache_type: 0x%x\nL2_system_id: 0x%x\nL2_cache_type: 0x%x\ncontrol: 0x%x\n"
		"auxiliary_control: 0x%x\ntranslation_table_base: 0x%x\ndomain_access_control: 0x%x\n"
		"data_fault_status: 0x%x\nintruction_fault_status: 0x%x\nfault_address: 0x%x\nfcse_pid: 0x%x\n"
		"context_id: 0x%x\ncoprocessor_enable: 0x%x\ncontrol_configuration: 0x%x\n"
		"privileged_mode_access: 0x%x\ncpu_id_code_extension: 0x%x\n",
		id_code, cache_type, L2_system_id, L2_cache_type, control,
		auxiliary_control, translation_table_base, domain_access_control,
		data_fault_status, intruction_fault_status, fault_address, fcse_pid,
		context_id, coprocessor_enable, control_configuration,
		privileged_mode_access, cpu_id_code_extension);
	return len;
}

static ssize_t regdump_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	unsigned int addr, value;
	int error = 0;
	char *tbuf = NULL;
	char *token[MAXTOKENS];
	int ntoks = tokenizer(&tbuf, buf, len, (char **)&token, MAXTOKENS);

	if (ntoks <= 0) {
		error = ntoks;
		goto out;
	}

	addr = simple_strtoul(token[0], NULL, 0);
	if (addr < 0xd4000000 || addr > 0xd4400000) {
		error = -EINVAL;
		goto out;
	}
	value = simple_strtoul(token[1], NULL, 0);

	__raw_writel(value, addr+0x2a000000);
out:
	kfree(tbuf);
	return error ? error : len;
}

static struct kobj_attribute regdump_attr = {
	.attr	= {
		.name = __stringify(regdump),
		.mode = 0644,
	},
	.show	= regdump_show,
	.store	= regdump_store,
};

static struct attribute * g[] = {
	&deepidle_attr.attr,
	&regdump_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int peripheral_suspend_config(void)
{
	if (wakeup_ops->pin_lpm_config)
		wakeup_ops->pin_lpm_config();

	return 0;
}

static int peripheral_resume_config(void)
{
	if (wakeup_ops->pin_restore)
		wakeup_ops->pin_restore();

	return 0;
}

/* This function is used to set unit clock before system enters sleep.
 */
static void pxa910_pm_set_cken(void)
{
	/*
	 * turn off SMC periphral clock to save power in udr mode.
	 */
	__raw_writel(0x3, APMU_SMC_CLK_RES_CTRL);
}

/* This function is used to restore unit clock after system resumes.
 */
static void pxa910_pm_restore_cken(void)
{
	__raw_writel(0x1b, APMU_SMC_CLK_RES_CTRL);
}

void wakeup_reason(uint32_t  awucrs)
{

 	if(awucrs&0x1<<27)
	{
	 printk("GSM wakeup\n");
	}
	if(awucrs&0x1<<26)
	{
	 printk("wcdma wakeup\n");
	}
	if(awucrs&0x1<<21)
	{
	 printk("keypress wakeup\n");
	}
	if(awucrs&0x1<<18)
	{
	 printk("wdt wakeup\n");
	}
	if(awucrs&0x1<<17)
	{
	 printk("rtc alarm wakeup\n");
	}
	if(awucrs&0x1c000)
	{
	 printk("cp timer wakeup\n");
	}
	if(awucrs&0x3f00)
	{
	 printk("ap timer wakeup\n");
	}
	if(awucrs&0x80)
	{
	 printk("wakeup7 wakeup\n");
	}
	if(awucrs&0x40)
	{
	 printk("wakeup6 wakeup\n");
	}
	if(awucrs&0x20)
	{
	 printk("wakeup5 wakeup\n");
	}
	if(awucrs&0x10)
	{
	 printk("wakeup4 wakeup\n");
	}
	if(awucrs&0x8)
	{
	 printk("wakeup3 wakeup\n");
	}
	if(awucrs&0x4)
	{
	 printk("wakeup2 wakeup\n");
	}
	if(awucrs&0x2)
	{
	 printk("wakeup1 wakeup\n");
	}
	if(awucrs&0x1)
	{
	 printk("wakeup0 wakeup\n");
	}
}
static int pxa910_pm_enter(suspend_state_t state)
{
	int idle_cfg;
	uint32_t awucrs=0;
	uint32_t awucrs_wakeup= PMUM_WAKEUP7; //PMIC wakeup status
	uint32_t reg = 0;

	//pmic thread not completed,exit;otherwise system can't be waked up
	reg = __raw_readl(ICU_INT_CONF(IRQ_PXA168_PMIC_INT));
	if((reg & 0x3) == 0) return -EAGAIN;
	if(pxa910_suspend_exit_status()){
		pxa910_suspend_exit_clr();
		return -EAGAIN;
	}

	__raw_writel(0x0, MPMU_VRCR);

	peripheral_suspend_config();
	pxa910_pm_set_cken();

	idle_cfg = __raw_readl(APMU_IDLE_CFG);       // add it
	idle_cfg |= 0x60;
	__raw_writel(idle_cfg, APMU_IDLE_CFG);


	pxa910_cpu_disable_l2(0);

	cpu_do_idle();

	pxa910_cpu_enable_l2(0);

	/*enter resume process*/
	cur_state = RESUME;

	idle_cfg = __raw_readl(APMU_IDLE_CFG);
	idle_cfg &= (~0x60);
	__raw_writel(idle_cfg, APMU_IDLE_CFG);

	pxa910_pm_restore_cken();
	peripheral_resume_config();

	awucrs=__raw_readl(MPMU_AWUCRS);
        wakeup_reason(awucrs);
	if (awucrs & awucrs_wakeup) {
		printk("[%s]AWUCRS:%x\r\n",__func__, awucrs);
		wakeup_detect = 1;
		printk("WAKEUP DETECT\n");
	}

	return 0;
}

static void pxa910_pm_irq(unsigned int enable)
{
	struct irq_desc *desc = NULL;
	unsigned int i, j, flag=0;
	int reg;

	for (i = 0; i < 64; i++) {
		desc = irq_to_desc(i);
		flag = 0;
		for (j = 0; j < ARRAY_SIZE(pm_irqs); j++)
			if (i == pm_irqs[j]) {
				if(desc->action){
					if(enable)
						desc->action->flags &= ~IRQF_TIMER;
					else
						desc->action->flags |= IRQF_TIMER;
				}
				flag = 1;
				break;
			}
		if(flag)
			continue;
		reg = __raw_readl(ICU_INT_CONF(i));
		if(!(reg & ICU_INT_CONF_CP_INT)) {
			if(enable)
				desc->chip->unmask(i);
			else
				desc->chip->mask(i);
		}
	}
#if 1
	/*PMIC irq sources should be set IRQF_TIMER flag*/
	for(i = pm8607_irq_base; i < pm8607_irq_base + PM8607_MAX_IRQ; i++){
		desc = irq_to_desc(i);
		if(desc->action){
			if(enable)
				desc->action->flags &= ~IRQF_TIMER;
			else
				desc->action->flags |= IRQF_TIMER;
		}
	}
  #endif
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int pxa910_pm_prepare(void)
{
	/*enter suspend process*/
	cur_state = SUSPEND;
	pxa910_pm_enter_lowpower_mode(POWER_MODE_UDR);
	pxa910_pm_irq(0);
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void pxa910_pm_finish(void)
{
	pm_state = PM_SUSPEND_ON;
	pxa910_pm_irq(1);
	pxa910_pm_enter_lowpower_mode(POWER_MODE_CORE_INTIDLE);
	__raw_writel(0x1, MPMU_VRCR);
	pm860x_codec_reg_write(PM8607_AUDIO_REG_BASE + PM8607_AUDIO_SHORTS, 0xaa);
}

static void pxa910_pm_wake(void)
{
	if (wakeup_detect) {
		wake_lock_timeout(&system_wakeup, HZ * 2);
	}
	wakeup_detect = 0;
}

static int pxa910_pm_valid(suspend_state_t state)
{
	int ret = 1;

	if (state == PM_SUSPEND_STANDBY) {
		pm_state = PM_SUSPEND_STANDBY;
	} else if (state == PM_SUSPEND_MEM) {
		pm_state = PM_SUSPEND_MEM;
	} else {
		ret = 0;
	}
	return ret;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct platform_suspend_ops pxa910_pm_ops = {
	.valid		= pxa910_pm_valid,
	.prepare	= pxa910_pm_prepare,
	.enter		= pxa910_pm_enter,
	.finish		= pxa910_pm_finish,
	.wake		= pxa910_pm_wake,
};

static int __init pxa910_pm_init(void)
{
	if (!cpu_is_pxa910())
		return -EIO;
	if (sysfs_create_group(power_kobj, &attr_group))
		return -1;
	suspend_set_ops(&pxa910_pm_ops);

	__raw_writel(__raw_readl(APMU_SQU_CLK_GATE_CTRL) | BIT_30, APMU_SQU_CLK_GATE_CTRL);
	__raw_writel(__raw_readl(MPMU_FCCR) | BIT_28, MPMU_FCCR);

	wake_lock_init(&system_wakeup, WAKE_LOCK_SUSPEND, "system_wakeup_detect");

	return 0;
}

late_initcall(pxa910_pm_init);
