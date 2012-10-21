/*
 * PXA910 DVFM Driver
 *
 * Copyright (C) 2008 Marvell Corporation
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

#include <asm/io.h>

#include <mach/cputype.h>
#include <mach/hardware.h>
#include <mach/dvfm.h>
#include <mach/pxa168_pm.h>
#include <mach/pxa910_pm.h>
#include <mach/pxa910_dvfm.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>
#include <mach/regs-ciu.h>
#include <mach/mspm_prof.h>

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
extern struct wake_lock constraint_wakelock;
#endif

#define CONFIG_PXA910_DVFM_ASYNC_MODE 1
#define CONFIG_PXA910_AP_ALONE_MODE 1

struct pxa910_dvfm_info {
	uint32_t cpuid;
	unsigned char __iomem *pmum_base;
	unsigned char __iomem *pmua_base;
};

static struct info_head pxa910_dvfm_op_list = {
	.list = LIST_HEAD_INIT(pxa910_dvfm_op_list.list),
	.lock = RW_LOCK_UNLOCKED,
};

/* the operating point preferred by policy maker or user */
static int preferred_op;
/*mutex lock protecting frequency change */
static DEFINE_MUTEX(freqs_mutex);
/*mutex lock protecting low power modes*/
static DEFINE_MUTEX(low_power_mutex);

extern unsigned int cur_op;		/* current operating point */
extern unsigned int def_op;		/* default operating point */

extern int mspm_op_num;

static struct pxa910_md_opt pxa920_op_array[] = {
	/* core 78MHz ddr 78MHz bus 78MHz */
	/*{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,
		.pclk = 78,
		.pdclk = 39,
		.baclk = 39,
		.xpclk = 78,
		.dclk = 78,
		.aclk = 78,
		.lpj = 78*(500000)/HZ,
		.name = "78MHz",
	},*/
	/* core 156MHz ddr 104MHz bus 104MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225, //1200,
		.pclk = 156,
		.pdclk = 78,
		.baclk = 78,
		.xpclk = 156,
		.dclk = 104,
		.aclk = 104,
		.lpj = 156*500000/HZ,
		.name = "156MHz",
	},
	/* core 312MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,//1200,
		.pclk = 312,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 312*500000/HZ,
		.name = "312MHz",
	},
	/* core 624MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225, //1200,
		.pclk = 624,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 624*500000/HZ,
		.name = "624MHz",
	},
	/* core 797MHz ddr 199MHz bus 208MHz */
	/*{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1300,
		.pclk = 797,
		.pdclk = 199,
		.baclk = 199,
		.xpclk = 398,
		.dclk = 199,
		.aclk = 208,
		.lpj = 797*500000/HZ,
		.name = "797MHz",
	},*/
	/* core 801MHz ddr 200MHz bus 200MHz */
	/*{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1300,
		.pclk = 801,
		.pdclk = 200,
		.baclk = 200,
		.xpclk = 400,
		.dclk = 200,
		.aclk = 208,
		.lpj = 801*500000/HZ,
		.name = "801MHz",
	},*/
	#if CONFIG_PXA_806M
	/* core 806MHz ddr 201MHz bus 201MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1300,
		.pclk = 806,
		.pdclk = 201,
		.baclk = 201,
		.xpclk = 403,
		.dclk =   201,   
		.aclk = 208,
		.lpj = 806*500000/HZ,
		.name = "806MHz",
	},
	#endif
	/* core internal idle */
	{
		.power_mode = POWER_MODE_CORE_INTIDLE,
		.vcc_core = 1300,
		.name = "core_intidle",
	},
	/* core external idle */
	{
		.power_mode = POWER_MODE_CORE_EXTIDLE,
		.vcc_core = 1300,
		.name = "core_extidle",
	},
	/* application subsystem idle */
	{
		.power_mode = POWER_MODE_APPS_IDLE,
		.vcc_core = 1300,
		.name = "apps_idle",
	},
	/* application subsystem sleep */
	{
		.power_mode = POWER_MODE_APPS_SLEEP,
		.vcc_core = 1300,
		.name = "apps_sleep",
	},
	/* system sleep */
	{
		.power_mode = POWER_MODE_SYS_SLEEP,
		.vcc_core = 1300,
		.name = "sys_sleep",
	},
};

static struct pxa910_md_opt pxa910_op_array[] = {
	/* core 208MHz ddr 104MHz bus 104MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,
		.pclk = 208,
		.pdclk = 104,
		.baclk = 104,
		.xpclk = 104,
		.dclk = 104,
		.aclk = 104,
		.lpj = 208*500000/HZ,
		.name = "208MHz",
	},
	/* core 312MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,
		.pclk = 312,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 156,
		.dclk = 156,
		.aclk = 156,
		.lpj = 312*500000/HZ,
		.name = "312MHz",
	},
	/* core 624MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,
		.pclk = 624,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 624*500000/HZ,
		.name = "624MHz",
	},
	/* core 806MHz ddr 201MHz bus 201MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1300,
		.pclk = 806,
		.pdclk = 201,
		.baclk = 201,
		.xpclk = 403,
		.dclk = 201,
		.aclk = 208,
		.lpj = 806*500000/HZ,
		.name = "806MHz",
	},
	/* core internal idle */
	{
		.power_mode = POWER_MODE_CORE_INTIDLE,
		.vcc_core = 1300,
		.name = "core_intidle",
	},
	/* core external idle */
	{
		.power_mode = POWER_MODE_CORE_EXTIDLE,
		.vcc_core = 1300,
		.name = "core_extidle",
	},
	/* application subsystem idle */
	{
		.power_mode = POWER_MODE_APPS_IDLE,
		.vcc_core = 1300,
		.name = "apps_idle",
	},
	/* application subsystem sleep */
	{
		.power_mode = POWER_MODE_APPS_SLEEP,
		.vcc_core = 1300,
		.name = "apps_sleep",
	},
	/* system sleep */
	{
		.power_mode = POWER_MODE_SYS_SLEEP,
		.vcc_core = 1300,
		.name = "sys_sleep",
	},
};

static struct pxa910_md_opt pxa918_op_array[] = {
	/* core 78MHz ddr 78MHz bus 78MHz */
	/*{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,
		.pclk = 78,
		.pdclk = 39,
		.baclk = 39,
		.xpclk = 78,
		.dclk = 78,
		.aclk = 78,
		.lpj = 78*(500000)/HZ,
		.name = "78MHz",
	},*/
	/* core 156MHz ddr 104MHz bus 104MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,
		.pclk = 156,
		.pdclk = 78,
		.baclk = 78,
		.xpclk = 156,
		.dclk = 104,
		.aclk = 104,
		.lpj = 156*500000/HZ,
		.name = "156MHz",
	},
	/* core 312MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,
		.pclk = 312,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 312*500000/HZ,
		.name = "312MHz",
	},
	/* core 624MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1225,
		.pclk = 624,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 624*500000/HZ,
		.name = "624MHz",
	},
	/* core 797MHz ddr 199MHz bus 208MHz */
	/*{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1300,
		.pclk = 797,
		.pdclk = 199,
		.baclk = 199,
		.xpclk = 398,
		.dclk = 199,
		.aclk = 208,
		.lpj = 797*500000/HZ,
		.name = "797MHz",
	},*/
	/* core 801MHz ddr 200MHz bus 200MHz */
	/*{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1300,
		.pclk = 801,
		.pdclk = 200,
		.baclk = 200,
		.xpclk = 400,
		.dclk = 200,
		.aclk = 208,
		.lpj = 801*500000/HZ,
		.name = "801MHz",
	},*/
	/* core 806MHz ddr 201MHz bus 201MHz */
	/*{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1300,
		.pclk = 806,
		.pdclk = 201,
		.baclk = 201,
		.xpclk = 403,
		.dclk = 201,
		.aclk = 208,
		.lpj = 806*500000/HZ,
		.name = "806MHz",
	},*/
	/* core internal idle */
	{
		.power_mode = POWER_MODE_CORE_INTIDLE,
		.vcc_core = 1300,
		.name = "core_intidle",
	},
	/* core external idle */
	{
		.power_mode = POWER_MODE_CORE_EXTIDLE,
		.vcc_core = 1300,
		.name = "core_extidle",
	},
	/* application subsystem idle */
	{
		.power_mode = POWER_MODE_APPS_IDLE,
		.vcc_core = 1300,
		.name = "apps_idle",
	},
	/* application subsystem sleep */
	{
		.power_mode = POWER_MODE_APPS_SLEEP,
		.vcc_core = 1300,
		.name = "apps_sleep",
	},
	/* system sleep */
	{
		.power_mode = POWER_MODE_SYS_SLEEP,
		.vcc_core = 1300,
		.name = "sys_sleep",
	},
};

static int check_op(void *driver_data, struct dvfm_freqs *freqs,
		unsigned int new, unsigned int relation);

static int fc_lock_ref_cnt;

static void get_fc_lock(void)
{
	pmua_dm_cc dm_cc_ap;

	fc_lock_ref_cnt++;

	if (fc_lock_ref_cnt == 1) {
		int timeout = 100000;

		/* AP-CP FC mutual exclusion */
		dm_cc_ap.v = __raw_readl(APMU_CCSR);
		while (dm_cc_ap.b.sea_rd_status && timeout) {
			dm_cc_ap.v = __raw_readl(APMU_CCSR);
			timeout--;
		}
		if (timeout <= 0)
			printk("cp does not release its fc lock\n");
	}
}

static void put_fc_lock(void)
{
	pmua_cc cc_ap;

	fc_lock_ref_cnt--;

	if (fc_lock_ref_cnt < 0)
		printk("unmatched put_fc_lock\n");

	if (fc_lock_ref_cnt == 0) {
		/* write 1 to MOH_RD_ST_CLEAR to clear MOH_RD_STATUS */
		cc_ap.v = __raw_readl(APMU_CCR);
		cc_ap.b.core_rd_st_clear = 1;
		__raw_writel(cc_ap.v, APMU_CCR);
		cc_ap.b.core_rd_st_clear = 0;
		__raw_writel(cc_ap.v, APMU_CCR);
	}
}

static unsigned int to_refd_div(unsigned int refd)
{
	unsigned int div;
	switch (refd) {
	case 0x00: div = 1; break;
	case 0x10: div = 2; break;
	case 0x01: div = 3; break;
	default: div = refd + 2; break;
	}
	return div;
}

static unsigned int to_fbd_div(unsigned int fbd)
{
	return fbd + 2;
}

u32 get_pll2_freq(void)
{
	pmum_pll2cr pll2cr;

	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if ((pll2cr.b.ctrl == 1) && (pll2cr.b.en == 0))
		return 0;
	return 26 * to_fbd_div(pll2cr.b.pll2fbd) / to_refd_div(pll2cr.b.pll2refd);
}
EXPORT_SYMBOL(get_pll2_freq);

unsigned char __iomem *sram_last_page;

extern void freq_change(unsigned char __iomem *sram_last_page,
			    unsigned int value, unsigned int base_addr);
extern void freq_sram_start(void);
extern void freq_sram_end(void);

static unsigned int to_clk_src(unsigned int sel, u32 pll2freq)
{
	unsigned int clk = 0;

	switch (sel) {
	case 0: clk = 312; break;
	case 1: clk = 624; break;
	case 2: clk = pll2freq; break;
	case 3: clk = 26; break;
	default: printk("Wrong clock source\n"); break;
	}
	return clk;
}

static void get_current_op(struct pxa910_md_opt *cop)
{
	pmua_pllsel pllsel;
	pmua_dm_cc dm_cc_cp, dm_cc_ap;
	pmua_cc cc_cp;

	get_fc_lock();
	dm_cc_ap.v = __raw_readl(APMU_CCSR);
	dm_cc_cp.v = __raw_readl(APMU_CP_CCSR);
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_rd_st_clear = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);
	cc_cp.b.core_rd_st_clear = 0;
	__raw_writel(cc_cp.v, APMU_CP_CCR);
	pllsel.v = __raw_readl(APMU_PLL_SEL_STATUS);

	cop->pll2freq = get_pll2_freq();
	cop->ap_clk_src = to_clk_src(pllsel.b.apclksel, cop->pll2freq);
	cop->cp_clk_src = to_clk_src(pllsel.b.cpclksel, cop->pll2freq);
	cop->axi_clk_src = to_clk_src(pllsel.b.axiclksel, cop->pll2freq);
	cop->ddr_clk_src = to_clk_src(pllsel.b.ddrclksel, cop->pll2freq);
	cop->gc_clk_src = to_clk_src(pllsel.b.gcaclksel, cop->pll2freq);
	cop->pclk = cop->ap_clk_src/(dm_cc_ap.b.core_clk_div+1);
	cop->pdclk = cop->ap_clk_src/(dm_cc_ap.b.bus_mc_clk_div+1);
	cop->baclk = cop->ap_clk_src/(dm_cc_ap.b.biu_clk_div+1);
	cop->xpclk = cop->ap_clk_src/(dm_cc_ap.b.xp_clk_div+1);
	cop->cp_pclk = cop->cp_clk_src/(dm_cc_cp.b.core_clk_div+1);
	cop->cp_pdclk = cop->cp_clk_src/(dm_cc_cp.b.bus_mc_clk_div+1);
	cop->cp_baclk = cop->cp_clk_src/(dm_cc_cp.b.biu_clk_div+1);
	cop->cp_xpclk = cop->cp_clk_src/(dm_cc_cp.b.xp_clk_div+1);
	cop->dclk = cop->ddr_clk_src/(dm_cc_ap.b.ddr_clk_div+1)/2;
	cop->aclk = cop->axi_clk_src/(dm_cc_ap.b.bus_clk_div+1);

	if ((cop->cp_clk_src != 312))
		printk("Wrong cp clock source\n");
	if (dm_cc_ap.b.biu_clk_div)
		printk("biu_clk_div should be 0 for Ax stepping\n");
	put_fc_lock();
}

static u32 choose_clk_src(u32 clk)
{
	u32 choice = 0;

	if ((26 % clk) == 0) {
		choice |= CLK_SRC_VCTCXO;
	} else {
		if ((312/clk <= 8) && ((312 % clk) == 0))
			choice |= CLK_SRC_PLL1_312;
		if ((624/clk <= 8) && ((624 % clk) == 0))
			choice |= CLK_SRC_PLL1_624;
		/* we assume the clk can always be generated from PLL2 */
		choice |= CLK_SRC_PLL2;
	}
	return choice;
}

/*
 * Clock source selection principle:
 *   1. do not use PLL2 if possible
 *   2. use the same clock source for core and ddr if possible
 *   3. try to use the lowest PLL source for core/ddr/axi
 *
 * FIXME: the code does not follow the principle right now
 */
static void select_clk_src(struct pxa910_md_opt *top)
{
	u32 dclk2x = top->dclk*2;
	u32 ap_clk_choice, axi_clk_choice, ddr_clk_choice;

	ap_clk_choice = choose_clk_src(top->pclk);
	axi_clk_choice = choose_clk_src(top->aclk);
	ddr_clk_choice = choose_clk_src(dclk2x);

	if ((ap_clk_choice == 0) || (axi_clk_choice == 0) || (ddr_clk_choice == 0))
		printk("Wrong clock combination\n");

	top->pll2freq = top->ap_clk_src = top->axi_clk_src = top->ddr_clk_src = 0;

	if (ap_clk_choice & axi_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_624) {
		top->ap_clk_src = top->axi_clk_src = top->ddr_clk_src = 624;
	} else if (ap_clk_choice & axi_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_312) {
		top->ap_clk_src = top->axi_clk_src = top->ddr_clk_src = 312;
	} else if ((ap_clk_choice & CLK_SRC_PLL1_312) &&
		   (axi_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_624)) {
		top->ap_clk_src = 312; top->axi_clk_src = top->ddr_clk_src = 624;
	} else if ((ap_clk_choice & CLK_SRC_PLL1_624) &&
		   (axi_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_312)) {
		top->ap_clk_src = 624; top->axi_clk_src = top->ddr_clk_src = 312;
	} else if ((axi_clk_choice & CLK_SRC_PLL1_312) &&
		   (ap_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_624)) {
		top->axi_clk_src = 312; top->ap_clk_src = top->ddr_clk_src = 624;
	} else if ((axi_clk_choice & CLK_SRC_PLL1_624) &&
		   (ap_clk_choice & ddr_clk_choice & CLK_SRC_PLL1_312)) {
		top->axi_clk_src = 624; top->ap_clk_src = top->ddr_clk_src = 312;
	} else if ((ddr_clk_choice & CLK_SRC_PLL1_312) &&
		   (ap_clk_choice & axi_clk_choice & CLK_SRC_PLL1_624)) {
		top->ddr_clk_src = 312; top->ap_clk_src = top->axi_clk_src = 624;
	} else if ((ddr_clk_choice & CLK_SRC_PLL1_624) &&
		   (ap_clk_choice & axi_clk_choice & CLK_SRC_PLL1_312)) {
		top->ddr_clk_src = 624; top->ap_clk_src = top->axi_clk_src = 312;
	} else if (ap_clk_choice & CLK_SRC_PLL2) {
		top->pll2freq = top->pclk;
		top->ap_clk_src = top->pll2freq;
		if ((top->pll2freq/dclk2x <= 8) && (top->pll2freq % dclk2x) == 0)
			top->ddr_clk_src = top->pll2freq;
		else if (ddr_clk_choice & CLK_SRC_PLL1_312)
			top->ddr_clk_src = 312;
		else if (ddr_clk_choice & CLK_SRC_PLL1_624)
			top->ddr_clk_src = 624;
		if (top->ddr_clk_src == 0)
			top->ddr_clk_src = top->pll2freq;
		/* try to use lowest PLL source for axi bus */
		if (axi_clk_choice & CLK_SRC_PLL1_312)
			top->axi_clk_src = 312;
		else if (axi_clk_choice & CLK_SRC_PLL1_624)
			top->axi_clk_src = 624;
		else if ((top->pll2freq/top->aclk <= 8) && (top->pll2freq % top->aclk) == 0)
			top->axi_clk_src = top->pll2freq;
		if (top->axi_clk_src == 0)
			top->axi_clk_src = top->pll2freq;
	} else if (axi_clk_choice & CLK_SRC_PLL2) {
		top->pll2freq = top->aclk;
		top->axi_clk_src = top->pll2freq;
		if (ap_clk_choice & CLK_SRC_PLL1_312)
			top->ap_clk_src = 312;
		else if (ap_clk_choice & CLK_SRC_PLL1_624)
			top->ap_clk_src = 624;
		if ((top->pll2freq/dclk2x <= 8) && (top->pll2freq % dclk2x) == 0)
			top->ddr_clk_src = top->pll2freq;
		else if (ddr_clk_choice & CLK_SRC_PLL1_312)
			top->ddr_clk_src = 312;
		else if (ddr_clk_choice & CLK_SRC_PLL1_624)
			top->ddr_clk_src = 624;
	} else if (ddr_clk_choice & CLK_SRC_PLL2) {
		top->pll2freq = dclk2x;
		top->ddr_clk_src = top->pll2freq;
		if (ap_clk_choice & CLK_SRC_PLL1_312)
			top->ap_clk_src = 312;
		else if (ap_clk_choice & CLK_SRC_PLL1_624)
			top->ap_clk_src = 624;
		if (axi_clk_choice & CLK_SRC_PLL1_312)
			top->axi_clk_src = 312;
		else if (axi_clk_choice & CLK_SRC_PLL1_624)
			top->axi_clk_src = 624;
	} else if (ap_clk_choice & axi_clk_choice & ddr_clk_choice & CLK_SRC_VCTCXO) {
		top->ap_clk_src = top->axi_clk_src = top->ddr_clk_src = 26;
	}

	if ((top->ap_clk_src == 0) || (top->axi_clk_src == 0) || (top->ddr_clk_src == 0))
		printk("Wrong clock combination\n");
}

static void set_ap_clk_sel(struct pxa910_md_opt *top)
{
	pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);
	if (top->ap_clk_src == 312)
		fccr.b.mohclksel = 0;
	else if (top->ap_clk_src == 624)
		fccr.b.mohclksel = 1;
	else if (top->ap_clk_src == top->pll2freq)
		fccr.b.mohclksel = 2;
	else if (top->ap_clk_src == 26)
		fccr.b.mohclksel = 3;
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_axi_clk_sel(struct pxa910_md_opt *top)
{
	pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);
	if (top->axi_clk_src == 312) {
		fccr.b.axiclksel1 = 0;
		fccr.b.axiclksel0 = 0;
	} else if (top->axi_clk_src == 624) {
		fccr.b.axiclksel1 = 0;
		fccr.b.axiclksel0 = 1;
	} else if (top->axi_clk_src == top->pll2freq) {
		fccr.b.axiclksel1 = 1;
		fccr.b.axiclksel0 = 0;
	} else if (top->axi_clk_src == 26) {
		fccr.b.axiclksel1 = 1;
		fccr.b.axiclksel0 = 1;
	}
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_ddr_clk_sel(struct pxa910_md_opt *top)
{
	pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);
	if (top->ddr_clk_src == 312)
		fccr.b.ddrclksel = 0;
	else if (top->ddr_clk_src == 624)
		fccr.b.ddrclksel = 1;
	else if (top->ddr_clk_src == top->pll2freq)
		fccr.b.ddrclksel = 2;
	else if (top->ddr_clk_src == 26)
		fccr.b.ddrclksel = 3;
	__raw_writel(fccr.v, MPMU_FCCR);
}

static void set_clk_sel(struct pxa910_md_opt *top)
{
	pmum_fccr fccr;

	fccr.v = __raw_readl(MPMU_FCCR);

	if (top->ap_clk_src == 312)
		fccr.b.mohclksel = 0;
	else if (top->ap_clk_src == 624)
		fccr.b.mohclksel = 1;
	else if (top->ap_clk_src == top->pll2freq)
		fccr.b.mohclksel = 2;
	else if (top->ap_clk_src == 26)
		fccr.b.mohclksel = 3;

	if (top->axi_clk_src == 312) {
		fccr.b.axiclksel1 = 0;
		fccr.b.axiclksel0 = 0;
	} else if (top->axi_clk_src == 624) {
		fccr.b.axiclksel1 = 0;
		fccr.b.axiclksel0 = 1;
	} else if (top->axi_clk_src == top->pll2freq) {
		fccr.b.axiclksel1 = 1;
		fccr.b.axiclksel0 = 0;
	} else if (top->axi_clk_src == 26) {
		fccr.b.axiclksel1 = 1;
		fccr.b.axiclksel0 = 1;
	}

	if (top->ddr_clk_src == 312)
		fccr.b.ddrclksel = 0;
	else if (top->ddr_clk_src == 624)
		fccr.b.ddrclksel = 1;
	else if (top->ddr_clk_src == top->pll2freq)
		fccr.b.ddrclksel = 2;
	else if (top->ddr_clk_src == 26)
		fccr.b.ddrclksel = 3;

	__raw_writel(fccr.v, MPMU_FCCR);
}

static void enable_fc_intr(void)
{
	u32 fc_int_msk;

	fc_int_msk = __raw_readl(APMU_IMR);
	/* fc_int_msk |= (7<<3); */
	/*
	 * enable AP FC done interrupt for one step,
	 * while not use three interrupts by three steps
	 */
	fc_int_msk |= (1<<1);
	__raw_writel(fc_int_msk, APMU_IMR);
}

static void wait_for_fc_done(void)
{
	int timeout = 1000000;
	while (!((1<<1) & __raw_readl(APMU_ISR)) && timeout)
		timeout--;
	if (timeout <= 0)
		panic("AP frequency change timeout!\n");
	__raw_writel(0x0, APMU_ISR);
}
/*
static void wait_for_ap_fc_done(void)
{
	int timeout = 1000000;
	while (!((1<<3) & __raw_readl(APMU_ISR)) && timeout)
		timeout--;
	if(timeout <= 0)
		panic("AP core frequency change timeout!\n");
	__raw_writel(0x0, APMU_ISR);
}

static void wait_for_axi_fc_done(void)
{
	int timeout = 1000000;
	while (!((1<<5) & __raw_readl(APMU_ISR)) && timeout)
		timeout--;
	if(timeout <= 0)
		panic("AXI frequency change timeout!\n");
	__raw_writel(0x0, APMU_ISR);
}
*/
/*
static void wait_for_ddr_fc_done(void)
{
	int timeout = 1000000;
	while (!((1<<4) & __raw_readl(APMU_ISR)) && timeout)
		timeout--;
	if(timeout <= 0)
		panic("DDR frequency change timeout!\n");
	__raw_writel(0x0, APMU_ISR);
}
*/
static struct regulator *v_buck1 = NULL;

static int pxa910_dvfm_get_core_voltage(void)
{
	int vcc_volt;

	vcc_volt = regulator_get_voltage(v_buck1);
	return vcc_volt/1000;
}

static int pxa910_dvfm_set_core_voltage(int mV)
{
	int vcc_volt;

	if (regulator_set_voltage(v_buck1, mV*1000, mV*1000))
		return -EIO;
	vcc_volt = regulator_get_voltage(v_buck1);
	if (vcc_volt != mV*1000)
		return -EIO;
	return 0;
}

#define VOL_UP_FAIL	-1
#define VOL_DOWN_FAIL	-2


static void PMUcore2_fc_seq(struct pxa910_md_opt *cop,
	struct pxa910_md_opt *top, struct pxa910_md_opt *old)
{
	pmua_cc cc_ap, cc_cp;
	u32 dclk2x = top->dclk*2;
	int two_step = 0;

	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_allow_spd_chg = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);

	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 1;

	/* If PLL2 is involved, two-step frequency change is needed */
	if (old->pll2freq || top->pll2freq)
		two_step = 1;

	/* async5 async4 async3_1 async3 are always 1 */
	cc_ap.b.async5 = 1;
	cc_ap.b.async4 = 1;
	cc_ap.b.async3_1 = 1;
	cc_ap.b.async3 = 1;

	/* use async mode when doing core/axi frequency change */
	cc_ap.b.async2 = 1;
	cc_ap.b.async1 = 1;

	if ((cop->ap_clk_src != top->ap_clk_src) || (cop->pclk != top->pclk) ||
	    (cop->pdclk != top->pdclk) || (cop->baclk != top->baclk)) {
		if (two_step)
	        set_ap_clk_sel(top);
		cc_ap.b.core_clk_div = top->ap_clk_src / top->pclk - 1;
		cc_ap.b.bus_mc_clk_div = top->ap_clk_src / top->pdclk - 1;
		cc_ap.b.biu_clk_div = 0;
		cc_ap.b.xp_clk_div = top->ap_clk_src / top->xpclk - 1;
		cc_ap.b.core_freq_chg_req = 1;
		if (two_step) {
		__raw_writel(cc_ap.v, APMU_CCR);
			wait_for_fc_done();
		cc_ap.b.core_freq_chg_req = 0;
		}
	}

	if ((cop->axi_clk_src != top->axi_clk_src) || (cop->aclk != top->aclk)) {
		if (two_step)
		set_axi_clk_sel(top);
		cc_ap.b.bus_clk_div = top->axi_clk_src / top->aclk - 1;
		cc_ap.b.bus_freq_chg_req = 1;
		/*
		__raw_writel(cc_ap.v, APMU_CCR);
		wait_for_axi_fc_done();
		cc_ap.b.bus_freq_chg_req = 0;
		*/
	}

	/* set sync mode if possible when doing ddr frequency change */
	if ((top->ap_clk_src == top->ddr_clk_src) && (top->pdclk == top->dclk))
		cc_ap.b.async2 = 0;
	else
		cc_ap.b.async2 = 1;
	/* keep ASYNC mode for CP ddr access for all operating points */
	/*if ((cop->cp_clk_src == top->ddr_clk_src) && (cop->cp_pdclk == top->dclk))
		cc_ap.b.async1 = 0;
	else
		cc_ap.b.async1 = 1;*/

	if ((cop->ddr_clk_src != top->ddr_clk_src) || (cop->dclk != top->dclk) ||
	    (cc_ap.b.async2 == 0) || (cc_ap.b.async1 == 0)) {
		if (two_step)
	          set_ddr_clk_sel(top);
		cc_ap.b.ddr_clk_div = top->ddr_clk_src / dclk2x - 1;
		cc_ap.b.ddr_freq_chg_req = 1;
		/*__raw_writel(cc_ap.v, APMU_CCR);
		wait_for_ddr_fc_done();*/
		if (two_step) {
			freq_change(sram_last_page, cc_ap.v, APMU_CP_CCR);
		cc_ap.b.ddr_freq_chg_req = 0;
	}
	}

	/*
	 * set clk sources for pclk, aclk and ddrclk,
	 * and update FCCR at the same time
	 */
	if (!two_step) {
		set_clk_sel(top);
		freq_change(sram_last_page, cc_ap.v, APMU_CP_CCR);
	}

	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 0;
	__raw_writel(cc_ap.v, APMU_CCR);
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_allow_spd_chg = 0;
	__raw_writel(cc_cp.v, APMU_CP_CCR);

	__raw_writel(0x0, APMU_ISR);
}

/* Note that we fix gc aclk to PLL2/4 so make sure PLL2 is enabled previously */
void gc_aclk_fc(void)
{
	pmum_fccr fccr;
	unsigned int temp;

	/* set GC ACLK clock source to PLL2 */
	fccr.v = __raw_readl(MPMU_FCCR);
	fccr.b.gcaclksel = 0x2;
	__raw_writel(fccr.v, MPMU_FCCR);

	/* set GC ACLK clock divider to 4 */
	temp = __raw_readl(APMU_GC_CLK_RES_CTRL);
	temp |= (3 << 12);
	__raw_writel(temp, APMU_GC_CLK_RES_CTRL);

	/* GC ACLK dynamic clock change */
	temp |= (1 << 16);
	__raw_writel(temp, APMU_GC_CLK_RES_CTRL);
}

/* #####################Debug Function######################## */
static int dump_op(void *driver_data, struct op_info *p, char *buf)
{
	int len, count, x, i, max, sum;
	struct pxa910_md_opt *q = (struct pxa910_md_opt *)p->op;

	if (q == NULL)
		len = sprintf(buf, "Can't dump the op info\n");
	else {
		/* calculate how much bits is set in device word */
		max = DVFM_MAX_CLIENT >> 5;
		for (i = 0, sum = 0; i < max; i++) {
			x = p->device[i];
			for (count = 0; x; x = x & (x - 1), count++);
			sum += count;
		}
		len = sprintf(buf, "OP:%d name:%s [%s, %d]\n",
				p->index, q->name, (sum)?"Disabled"
				:"Enabled", sum);
		len += sprintf(buf + len, "pclk:%d pdclk:%d baclk:%d xpclk:%d "
				"dclk:%d aclk:%d vcc_core:%d\n",
				q->pclk, q->pdclk, q->baclk, q->xpclk,
				q->dclk, q->aclk, q->vcc_core);
	}
	return len;
}

static int dump_op_list(void *driver_data, struct info_head *op_table)
{
	struct op_info *p = NULL;
	struct list_head *list = NULL;
	struct pxa910_dvfm_info *info = driver_data;
	char buf[256];

	if (!op_table || list_empty(&op_table->list)) {
		printk(KERN_WARNING "op list is null\n");
		return -EINVAL;
	}
	memset(buf, 0, 256);
	list_for_each(list, &op_table->list) {
		p = list_entry(list, struct op_info, list);
		dump_op(info, p, buf);
	}
	return 0;
}

/* Get current setting, and record it in fv_info structure
 */
static int capture_op_info(void *driver_data, struct pxa910_md_opt *fv_info)
{
	if (!fv_info)
		return -EFAULT;
	memset(fv_info, 0, sizeof(struct pxa910_md_opt));
	get_current_op(fv_info);
	fv_info->vcc_core = pxa910_dvfm_get_core_voltage();
	return 0;
}

static int get_op_num(void *driver_data, struct info_head *op_table)
{
	struct list_head *entry = NULL;
	int num = 0;
	unsigned long flags;

	if (!op_table)
		goto out;
	read_lock_irqsave(&op_table->lock, flags);
	if (list_empty(&op_table->list)) {
		read_unlock_irqrestore(&op_table->lock, flags);
		goto out;
	}
	list_for_each(entry, &op_table->list) {
		num++;
	}
	read_unlock_irqrestore(&op_table->lock, flags);
out:
	return num;
}

static char *get_op_name(void *driver_data, struct op_info *p)
{
	struct pxa910_md_opt *q = NULL;
	if (p == NULL)
		return NULL;
	q = (struct pxa910_md_opt *)p->op;
	return q->name;
}

static void set_freq(void *driver_data, struct pxa910_md_opt *old,
		struct pxa910_md_opt *new)
{
	u32 temp = 0;
	struct pxa910_md_opt cop;

	/*
	 * Check LCD reset is released so that AXI2MC reset
	 * is de-asserted and ddr frequency change can be done.
	 * If not, issue warning message and return.
	 */
	if ((__raw_readl(APMU_LCD_CLK_RES_CTRL) & 0x1) != 0x1)
		panic("LCD reset is not released\n");

	/* Check CP reset is released */
	if (__raw_readl(MPMU_APRR) & 0x1) {
		//printk(KERN_WARNING "CP reset is not released, skip CP acknowledge\n");
		temp = __raw_readl(APMU_DEBUG);
		__raw_writel(temp | (1<<0) | (1<<3), APMU_DEBUG);
	}

	get_fc_lock();

	memcpy(&cop, old, sizeof(struct pxa910_md_opt));
	get_current_op(&cop);
	if ((cop.pclk != old->pclk) || (cop.pdclk != old->pdclk) ||
	    (cop.xpclk != old->xpclk) ||
	    (cop.aclk != old->aclk) || (cop.dclk != old->dclk) ||
	    (cop.ap_clk_src != old->ap_clk_src) ||
	    (cop.axi_clk_src != old->axi_clk_src) ||
	    (cop.ddr_clk_src != old->ddr_clk_src)) {
		printk("%d %d %d %d %d %d %d %d %d\n", old->pclk, old->pdclk, old->xpclk, old->baclk,
			old->aclk, old->dclk, old->ap_clk_src, old->axi_clk_src, old->ddr_clk_src);
		printk("%d %d %d %d %d %d %d %d %d\n", cop.pclk, cop.pdclk, cop.xpclk, cop.baclk,
			cop.aclk, cop.dclk, cop.ap_clk_src, cop.axi_clk_src, cop.ddr_clk_src);
		printk("%d %d %d %d %d %d %d %d %d\n", new->pclk, new->pdclk, new->xpclk, new->baclk,
			new->aclk, new->dclk, new->ap_clk_src, new->axi_clk_src, new->ddr_clk_src);
		dump_stack();
	}

	PMUcore2_fc_seq(&cop, new, old);

	get_current_op(&cop);
	if ((cop.pclk != new->pclk) || (cop.pdclk != new->pdclk) ||
	    (cop.xpclk != new->xpclk) ||
	    (cop.aclk != new->aclk) || (cop.dclk != new->dclk) ||
	    (cop.ap_clk_src != new->ap_clk_src) ||
	    (cop.axi_clk_src != new->axi_clk_src) ||
	    (cop.ddr_clk_src != new->ddr_clk_src))
		printk("unsuccessful frequency change!\n");

	put_fc_lock();

	/* Check CP reset is released */
	if (__raw_readl(MPMU_APRR) & 0x1)
		__raw_writel(temp, APMU_DEBUG);
}

/*static int update_freq(void *driver_data, struct dvfm_freqs *freqs)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *old = NULL, *new = NULL;
	struct op_info *p = NULL;
	unsigned long flags;
	int found = 0, new_op;
	int ret;
	static int err_cnt_up, err_cnt_down;

	write_lock_irqsave(&pxa910_dvfm_op_list.lock, flags);
	new_op = cur_op;
	if (!list_empty(&pxa910_dvfm_op_list.list)) {
		list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
			if (p->index == freqs->old) {
				found++;
				old = (struct pxa910_md_opt *)p->op;
			}
			if (p->index == freqs->new) {
				found++;
				new = (struct pxa910_md_opt *)p->op;
				new_op = p->index;
			}
			if (found == 2)
				break;
		}
	}
	write_unlock_irqrestore(&pxa910_dvfm_op_list.lock, flags);
	if (found != 2)
		return -EINVAL;

	ret = set_freq(info, old, new);
	if (ret == VOL_UP_FAIL) {
		err_cnt_up++;
		if ((err_cnt_up % 100) == 0)
			printk("voltage up failed %d times\n", err_cnt_up);
		goto err;
	}
	if (ret == VOL_DOWN_FAIL) {
		err_cnt_down++;
		if ((err_cnt_down % 100) == 0)
			printk("voltage down failed %d times\n", err_cnt_down);
	}
	cur_op = new_op;
	loops_per_jiffy = new->lpj;
err:
	return ret;
}*/

static int do_freq_notify(void *driver_data, struct dvfm_freqs *freqs)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md_old = (struct pxa910_md_opt *)((freqs->old_info).op);
	struct pxa910_md_opt *md_new = (struct pxa910_md_opt *)((freqs->new_info).op);
	int temp_new = freqs->new;
	int ret = 0;

	/*recheck freqs->new in case that another process disable op during voltage change*/
	ret = check_op(info, freqs, temp_new, RELATION_LOW);
	if(ret)
		return -EINVAL;
	else{
		if(temp_new != freqs->new)
			return -EAGAIN;
	dvfm_notifier_frequency(freqs, DVFM_FREQ_PRECHANGE);
		set_freq(info, md_old, md_new);
		cur_op = freqs->new;
		loops_per_jiffy = md_new->lpj;
	dvfm_notifier_frequency(freqs, DVFM_FREQ_POSTCHANGE);
	return ret;
}
}

static void do_lowpower_notify(void *driver_data, struct dvfm_freqs *freqs,
	unsigned int state)
{
	dvfm_notifier_frequency(freqs, DVFM_FREQ_PRECHANGE);
	pxa910_pm_enter_lowpower_mode(state);
	dvfm_notifier_frequency(freqs, DVFM_FREQ_POSTCHANGE);
}

/* Check whether any client blocks the current operating point */
static int block_client(struct op_info *info)
{
	int i;
	unsigned int ret = 0;
	for (i = 0; i < (DVFM_MAX_CLIENT >> 5); i++)
		ret |= info->device[i];
	return (int)ret;
}

static int check_op(void *driver_data, struct dvfm_freqs *freqs,
		unsigned int new, unsigned int relation)
{
	struct op_info *p = NULL;
	int index,temp_index = 0;
	unsigned long flags;

	if (!dvfm_find_op(new, &p)) {
		index = p->index;
	} else
		return -EINVAL;

	read_lock_irqsave(&pxa910_dvfm_op_list.lock, flags);
	if (relation == RELATION_LOW) {
		/* Set the lowest usable op that is higher than specifed one */
		/* Note: we assume bigger index number is more 'usable' */
		temp_index = MAX_OP_NUM;
		list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
			if ((p->index < mspm_op_num)&&!block_client(p) && (p->index >= index)) {
				if (p->index < temp_index){
					temp_index = p->index;
					freqs->new = p->index;
				}
			}
		}
	} else if (relation == RELATION_HIGH) {
		/* Set the highest usable op that is lower than specified one */
		temp_index = -1;
		list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
			if ((p->index < mspm_op_num)&&!block_client(p) && (p->index <= index)) {
				if (p->index > temp_index){
					temp_index = p->index;
					freqs->new = p->index;
				}
			}
		}
	} else if (relation == RELATION_STICK) {
		/* Set the specified frequency */
		list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
			if (!block_client(p) && (p->index == new)) {
				freqs->new = p->index;
				break;
			}
		}
	}
	read_unlock_irqrestore(&pxa910_dvfm_op_list.lock, flags);
	if (temp_index < 0||temp_index >= MAX_OP_NUM)
		return -EINVAL;
	return 0;
}

static int pxa910_set_op(void *driver_data, struct dvfm_freqs *freqs,
		unsigned int new, unsigned int relation)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md_old = NULL,*md_new = NULL;
	struct op_info *p = NULL;
	unsigned long flags;
	int ret = 0;

	/*low power mode op*/
	if(new >= mspm_op_num){
		mutex_lock(&low_power_mutex);
		ret = dvfm_find_op(new, &p);
		if(ret){
			mutex_unlock(&low_power_mutex);
			return ret;
		}
		md_new = (struct pxa910_md_opt *)(p->op);
		do_lowpower_notify(info, freqs, md_new->power_mode);
		mutex_unlock(&low_power_mutex);
	}
	/*active mode op*/
	else{
		if(find_first_bit(dvfm_devices, DVFM_MAX_CLIENT)
			!= DVFM_MAX_CLIENT)
			return ret;
		mutex_lock(&freqs_mutex);
		/*set cur_op as old op and get its information*/
		freqs->old = cur_op;
	ret = dvfm_find_op(freqs->old, &p);
	if (ret)
		goto out;
		md_old = (struct pxa910_md_opt *)(p->op);
		memcpy(&(freqs->old_info), p, sizeof(struct op_info));
		/*find op which is closest to new op*/
	ret = check_op(info, freqs, new, relation);
	if (ret)
		goto out;
	vol:
		ret = dvfm_find_op(freqs->new, &p);
		if(ret)
			goto out;
		md_new = (struct pxa910_md_opt *)(p->op);
		memcpy(&freqs->new_info, p, sizeof(struct op_info));
		/*quit if cur_op is identical to new op*/
		if (freqs->old_info.index == freqs->new_info.index)
			goto out;

		/*if new op's voltage higher than cur op's, increase the voltage before changing op*/
		if(md_new->vcc_core > md_old->vcc_core){
			if (pxa910_dvfm_set_core_voltage(md_new->vcc_core) < 0){
				ret = VOL_UP_FAIL;
				printk("voltage up failed, still at %d\n", md_old->vcc_core);
				goto out;
		}
	}

		local_fiq_disable();
		local_irq_save(flags);
		ret = do_freq_notify(info, freqs);
	local_irq_restore(flags);
	local_fiq_enable();
		if(ret == -EAGAIN)
			goto vol;

		/*if new op's voltage lower than cur op's, decrease the voltage after changing op*/
		if(md_new->vcc_core < md_old->vcc_core){
			if (pxa910_dvfm_set_core_voltage(md_new->vcc_core) < 0){
				ret = VOL_DOWN_FAIL;
				printk("voltage down failed, still at %d\n", md_old->vcc_core);
				goto out;
			}
		}
	out:
		mutex_unlock(&freqs_mutex);
	}
	return ret;
}

static int pxa910_request_op(void *driver_data, int index)
{
	struct dvfm_freqs freqs;
	struct op_info *info = NULL;
	struct pxa910_md_opt *md = NULL;
	unsigned long flags;
	int relation, ret;

	ret = dvfm_find_op(index, &info);
	if (ret)
		goto out;

	md = (struct pxa910_md_opt *)(info->op);
	switch (md->power_mode) {
	case POWER_MODE_CORE_INTIDLE:
	case POWER_MODE_CORE_EXTIDLE:
	case POWER_MODE_APPS_IDLE:
	case POWER_MODE_APPS_SLEEP:
	case POWER_MODE_SYS_SLEEP:
		relation = RELATION_STICK;
		ret = pxa910_set_op(driver_data, &freqs, index, relation);
		break;
	default:
		relation = RELATION_LOW;
		/* only use non-low power mode as preferred op */
		write_lock_irqsave(&pxa910_dvfm_op_list.lock, flags);
		preferred_op = index;
		write_unlock_irqrestore(&pxa910_dvfm_op_list.lock, flags);
		ret = pxa910_set_op(driver_data, &freqs, index, relation);
		break;
	}
out:
	return ret;
}

/*
 * The machine operation of dvfm_enable
 */
static int pxa910_enable_dvfm(void *driver_data, int dev_id)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md = NULL;
	struct op_info *p = NULL;
	int i, num;
	num = get_op_num(info, &pxa910_dvfm_op_list);
	for (i = 0; i < num; i++) {
		if (!dvfm_find_op(i, &p)) {
			md = (struct pxa910_md_opt *)p->op;
			dvfm_enable_op(i, dev_id);
		}
	}
	return 0;
}

/*
 * The mach operation of dvfm_disable
 */
static int pxa910_disable_dvfm(void *driver_data, int dev_id)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md = NULL;
	struct op_info *p = NULL;
	int i, num;
	num = get_op_num(info, &pxa910_dvfm_op_list);
	for (i = 0; i < num; i++) {
		if (!dvfm_find_op(i, &p)) {
			md = (struct pxa910_md_opt *)p->op;
			dvfm_disable_op(i, dev_id);
		}
	}
	return 0;
}

static int pxa910_enable_op(void *driver_data, int index, int relation)
{
	/*
	 * Restore preferred_op. Because this op is sugguested by policy maker
	 * or user.
	 */
	if (index > (mspm_op_num-1)) return 0;
	return pxa910_request_op(driver_data, preferred_op);
}

static int pxa910_disable_op(void *driver_data, int index, int relation)
{
	struct dvfm_freqs freqs;
	int ret = 0;
	if (index > (mspm_op_num-1)) return 0;
	if(cur_op == index)
		ret = pxa910_set_op(driver_data, &freqs, index, relation);
	if(ret)
		printk("Can't disable op %d!\n",index);
	return ret;
}

static int pxa910_core_freq_calc(int op_point)
{
	struct pxa910_md_opt *md = NULL;
	struct op_info *p = NULL;
	int freq = 0;

	if (!dvfm_find_op(op_point, &p)) {
		md = (struct pxa910_md_opt *)p->op;
		freq = md->pclk;
	} else {
		/* unknown */
		freq = -1;
	}

	return freq;
}

static int pxa910_core_current_freq_get(void *driver_data)
{
	return pxa910_core_freq_calc(cur_op);
}

static int pxa910_core_freqs_table_get(void *driver_data,
					int *freq_table,
					int *num_pp,
					int table_sz)
{
	int op_point;

	//*num_pp = get_op_num(info, &pxa910_dvfm_op_list);
	*num_pp = mspm_op_num;

	if (table_sz < *num_pp)
		return -1;

	for (op_point = 0; op_point < *num_pp; op_point++)
		freq_table[op_point] = pxa910_core_freq_calc(op_point);

	return 0;
}

static int pxa910_get_max_op(void)
{
	struct op_info *p=NULL;
	int index=-1,pclk=0;
	int temp_pclk;
	list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
		temp_pclk = ((struct pxa910_md_opt *)p->op)->pclk;
		if (p->index < mspm_op_num && !block_client(p) && temp_pclk > pclk) {
				pclk = temp_pclk;
				index = p->index;
		}
	}
	return index;
}

#ifdef CONFIG_PXA910_DVFM_STATS
/* Convert ticks from 32K timer to microseconds */
static unsigned int pxa910_ticks_to_usec(unsigned int ticks)
{
	return (ticks * 5 * 5 * 5 * 5 * 5 * 5) >> 9;
}

static unsigned int pxa910_ticks_to_sec(unsigned int ticks)
{
	return (ticks >> 15);
}
#else
#define pxa910_ticks_to_usec	NULL
#define pxa910_ticks_to_sec		NULL
#endif

static struct dvfm_driver pxa910_driver = {
	.count		= get_op_num,
	.set		= pxa910_set_op,
	.dump		= dump_op,
	.name		= get_op_name,
	.request_set	= pxa910_request_op,
	.enable_dvfm	= pxa910_enable_dvfm,
	.disable_dvfm	= pxa910_disable_dvfm,
	.enable_op	= pxa910_enable_op,
	.disable_op	= pxa910_disable_op,
        .ticks_to_usec  = pxa910_ticks_to_usec,
        .ticks_to_sec   = pxa910_ticks_to_sec,
        .read_time      = read_timer,
	.current_core_freq_get = pxa910_core_current_freq_get,
	.core_freqs_table_get = pxa910_core_freqs_table_get,
	.get_max_op	= pxa910_get_max_op,
};

/* Produce a operating point table */
static int op_init(struct pxa910_dvfm_info *driver_data, struct info_head *op_table)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md, *smd;
	int i, index;
	struct op_info *p = NULL, *q = NULL;
	struct pxa910_md_opt *proc_array = NULL;
	unsigned int proc_array_size;
	u32 temp;

	if (cpu_is_pxa918()) {
		printk("cpu_is_pxa918 \n");
		proc_array = pxa918_op_array;
		proc_array_size = ARRAY_SIZE(pxa918_op_array);
	}
	else if (cpu_is_pxa910_c910()) {
		printk("cpu_is_pxa910_c910 \n");
		proc_array = pxa910_op_array;
		proc_array_size = ARRAY_SIZE(pxa910_op_array);
	}
	else if (cpu_is_pxa910_c920()) {
		printk("cpu_is_pxa910_c920 \n");
		proc_array = pxa920_op_array;
		proc_array_size = ARRAY_SIZE(pxa920_op_array);
	}

	for (i = 0, index = 0; i < proc_array_size; i++) {
		/* Set index of operating point used in idle */
		if ((proc_array + i)->power_mode != POWER_MODE_ACTIVE)
			set_idle_op(index, (proc_array + i)->power_mode);

		/* calculate proper clock source for each OP */
		if ((proc_array + i)->power_mode == POWER_MODE_ACTIVE)
			select_clk_src(proc_array + i);

		if (!(p = (struct op_info *)kzalloc(sizeof(struct op_info),
				GFP_KERNEL)))
			return -ENOMEM;
		if (!(p->op = (struct pxa910_md_opt *)kzalloc(sizeof(struct pxa910_md_opt),
				GFP_KERNEL))){
			kfree(p);
			return -ENOMEM;
		}
		memcpy(p->op, proc_array + i, sizeof(struct pxa910_md_opt));
		p->index = index++;
		list_add_tail(&(p->list), &(op_table->list));
	}

	if (!(p = (struct op_info *)kzalloc(sizeof(struct op_info),
				GFP_KERNEL)))
			return -ENOMEM;
	if (!(p->op = (struct pxa910_md_opt *)kzalloc(sizeof(struct pxa910_md_opt),
				GFP_KERNEL))){
			kfree(p);
			return -ENOMEM;
	}
	md = (struct pxa910_md_opt *)p->op;
	if (capture_op_info(info, md)) {
		printk(KERN_WARNING "Failed to get current op setting\n");
	} else {
		def_op = 0x5a5a;	/* magic number */
		list_for_each_entry(q, &(op_table->list), list) {
			smd = (struct pxa910_md_opt *)q->op;
			if (md->pclk == smd->pclk && md->pdclk == smd->pdclk &&
			    /*md->baclk == smd->baclk &&*/ md->xpclk == smd->xpclk &&
			    md->dclk == smd->dclk /*&& md->aclk == smd->aclk*/) {
				def_op = q->index;
				break;
			}
		}
	}
	md->power_mode = POWER_MODE_ACTIVE;
	md->lpj = loops_per_jiffy;
	sprintf(md->name, "BOOT OP");
	select_clk_src(md);


	if (!(q = (struct op_info *)kzalloc(sizeof(struct op_info),
				GFP_KERNEL)))
			return -ENOMEM;
	if (!(q->op = (struct pxa910_md_opt *)kzalloc(sizeof(struct pxa910_md_opt),
				GFP_KERNEL)))
			return -ENOMEM;
	smd = (struct pxa910_md_opt *)q->op;
	memcpy(smd, md, sizeof(struct pxa910_md_opt));
	sprintf(smd->name, "CUSTOM OP");
	select_clk_src(smd);

	/* Add CUSTOM OP into op list */
	q->index = index++;
	list_add_tail(&q->list, &op_table->list);
	/* Add BOOT OP into op list */
	p->index = index++;
	preferred_op = p->index;
	list_add_tail(&p->list, &op_table->list);
	/* BOOT op */
	if (def_op == 0x5a5a) {
		cur_op = p->index;
		def_op = p->index;
	} else
		cur_op = def_op;
	pr_debug("%s, def_op:%d, cur_op:%d\n", __FUNCTION__, def_op, cur_op);

	op_nums = proc_array_size + 2;	/* set the operating point number */

	printk("Current Operating Point is %d\n", cur_op);
	dump_op_list(info, op_table);

	/* The speed change bits will be set before FC and cleared after FC */
	/*
	cc_cp.v = __raw_readl(APMU_CP_CCR);
	cc_cp.b.core_allow_spd_chg = 1;
	__raw_writel(cc_cp.v, APMU_CP_CCR);

	cc_ap.v = __raw_readl(APMU_CCR);
	cc_ap.b.core_allow_spd_chg = 1;
	__raw_writel(cc_ap.v, APMU_CCR);
	*/

	temp = __raw_readl(APMU_DEBUG);
	temp |= (1 << 22) | (1 << 21); /* AP_WFI_FC and CP_WFI_FC */
	__raw_writel(temp, APMU_DEBUG);

	enable_fc_intr();
	/*
	 * pll2 should be initialized by boot loader and if it is not, we just
	 * report a serious bug here.
	 */
	if (get_pll2_freq() == 0) {
		printk(KERN_ERR "pll2 is off, please turn it on first\n");
		//BUG();
	}

	return 0;
}

#ifdef CONFIG_PM
static int pxa910_freq_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int pxa910_freq_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define pxa910_freq_suspend    NULL
#define pxa910_freq_resume     NULL
#endif

#define SRAM_ADDR_END 0xd1020000

static int pxa910_freq_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct pxa910_dvfm_info *info = NULL;

	v_buck1 = regulator_get(NULL, "v_buck1");
	if (IS_ERR(v_buck1))
		goto err;

	/* map the last page of SRAM for ddr frequency change */
	sram_last_page = ioremap(SRAM_ADDR_END-PAGE_SIZE, PAGE_SIZE);
	if ((freq_sram_end-freq_sram_start) < PAGE_SIZE)
		memcpy(sram_last_page, freq_sram_start, freq_sram_end-freq_sram_start);
	else
		goto err;

	if (!(info = kzalloc(sizeof(struct pxa910_dvfm_info), GFP_KERNEL)))
		goto err;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmum_regs");
	if (!res) goto err;
	info->pmum_base = ioremap(res->start, res->end - res->start + 1);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmua_regs");
	if (!res) goto err;
	info->pmua_base = ioremap(res->start, res->end - res->start + 1);

	pxa910_driver.priv = info;

	rwlock_init(&pxa910_dvfm_op_list.lock);

	if (op_init(info, &pxa910_dvfm_op_list))
		goto err;

	wake_lock_init(&constraint_wakelock, WAKE_LOCK_SUSPEND, "constraint_wakeup");

	return dvfm_register_driver(&pxa910_driver, &pxa910_dvfm_op_list);
err:
	if(info) kfree(info);
	printk("pxa910_dvfm init failed\n");
	return -EIO;
}

static int pxa910_freq_remove(struct platform_device *pdev)
{
	regulator_put(v_buck1);
	kfree(pxa910_driver.priv);
	return dvfm_unregister_driver(&pxa910_driver);
}

static struct platform_driver pxa910_freq_driver = {
	.driver = {
		.name	= "pxa168-freq",
	},
	.probe		= pxa910_freq_probe,
	.remove		= pxa910_freq_remove,
#ifdef CONFIG_PM
	.suspend	= pxa910_freq_suspend,
	.resume		= pxa910_freq_resume,
#endif
};

static int __init pxa910_freq_init(void)
{
	if (!cpu_is_pxa910())
		return -EIO;

	return platform_driver_register(&pxa910_freq_driver);
}

static void __exit pxa910_freq_exit(void)
{
	platform_driver_unregister(&pxa910_freq_driver);
}

module_init(pxa910_freq_init);
module_exit(pxa910_freq_exit);
