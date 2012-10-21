/*
 *  linux/arch/arm/mach-mmp/flint.c
 *
 *  Support for the Marvell Flint Development Platform.
 *
 *  Copyright (C) 2009 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/smc91x.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/max8925.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp2.h>

#include "common.h"

static unsigned long flint_pin_config[] __initdata = {
	/* UART1 */
	GPIO45_UART1_RXD,
	GPIO46_UART1_TXD,

	/* UART2 */
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,

	/* UART3 */
	GPIO51_UART3_RXD,
	GPIO52_UART3_TXD,

	/* SMC */
	GPIO151_SMC_SCLK,
	GPIO145_SMC_nCS0,
	GPIO146_SMC_nCS1,
	GPIO152_SMC_BE0,
	GPIO153_SMC_BE1,
	GPIO154_SMC_IRQ,
	GPIO113_SMC_RDY,

	/*Ethernet*/
	GPIO155_GPIO155,

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

	/* PMIC */
	PMIC_PMIC_INT | MFP_LPM_EDGE_FALL,
};

static struct smc91x_platdata flint_smc91x_info = {
	.flags  = SMC91X_USE_16BIT | SMC91X_NOWAIT,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start  = SMC_CS1_PHYS_BASE + 0x300,
		.end    = SMC_CS1_PHYS_BASE + 0xfffff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = gpio_to_irq(155),
		.end    = gpio_to_irq(155),
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.dev            = {
		.platform_data = &flint_smc91x_info,
	},
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

#if defined(CONFIG_I2C_PXA) || defined(CONFIG_I2C_PXA_MODULE)
static struct regulator_consumer_supply regulator_supply[] = {
	[MAX8925_ID_SD1] = REGULATOR_SUPPLY("v_sd1", NULL),
	[MAX8925_ID_SD2] = REGULATOR_SUPPLY("v_sd2", NULL),
	[MAX8925_ID_SD3] = REGULATOR_SUPPLY("v_sd3", NULL),
	[MAX8925_ID_LDO1] = REGULATOR_SUPPLY("v_ldo1", NULL),
	[MAX8925_ID_LDO2] = REGULATOR_SUPPLY("v_ldo2", NULL),
	[MAX8925_ID_LDO3] = REGULATOR_SUPPLY("v_ldo3", NULL),
	[MAX8925_ID_LDO4] = REGULATOR_SUPPLY("v_ldo4", NULL),
	[MAX8925_ID_LDO5] = REGULATOR_SUPPLY("v_ldo5", NULL),
	[MAX8925_ID_LDO6] = REGULATOR_SUPPLY("v_ldo6", NULL),
	[MAX8925_ID_LDO7] = REGULATOR_SUPPLY("v_ldo7", NULL),
	[MAX8925_ID_LDO8] = REGULATOR_SUPPLY("v_ldo8", NULL),
	[MAX8925_ID_LDO9] = REGULATOR_SUPPLY("v_ldo9", NULL),
	[MAX8925_ID_LDO10] = REGULATOR_SUPPLY("v_ldo10", NULL),
	[MAX8925_ID_LDO11] = REGULATOR_SUPPLY("v_ldo11", NULL),
	[MAX8925_ID_LDO12] = REGULATOR_SUPPLY("v_ldo12", NULL),
	[MAX8925_ID_LDO13] = REGULATOR_SUPPLY("v_ldo13", NULL),
	[MAX8925_ID_LDO14] = REGULATOR_SUPPLY("v_ldo14", NULL),
	[MAX8925_ID_LDO15] = REGULATOR_SUPPLY("v_ldo15", NULL),
	[MAX8925_ID_LDO16] = REGULATOR_SUPPLY("v_ldo16", NULL),
	[MAX8925_ID_LDO17] = REGULATOR_SUPPLY("v_ldo17", NULL),
	[MAX8925_ID_LDO18] = REGULATOR_SUPPLY("v_ldo18", NULL),
	[MAX8925_ID_LDO19] = REGULATOR_SUPPLY("v_ldo19", NULL),
	[MAX8925_ID_LDO20] = REGULATOR_SUPPLY("v_ldo20", NULL),
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
	[MAX8925_ID_SD1] = REG_INIT(SD1, 637500, 1425000, 1, 1),
	[MAX8925_ID_SD2] = REG_INIT(SD2, 650000, 2225000, 1, 1),
	[MAX8925_ID_SD3] = REG_INIT(SD3, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO1] = REG_INIT(LDO1, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO2] = REG_INIT(LDO2, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO3] = REG_INIT(LDO3, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO4] = REG_INIT(LDO4, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO5] = REG_INIT(LDO5, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO6] = REG_INIT(LDO6, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO7] = REG_INIT(LDO7, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO8] = REG_INIT(LDO8, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO9] = REG_INIT(LDO9, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO10] = REG_INIT(LDO10, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO11] = REG_INIT(LDO11, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO12] = REG_INIT(LDO12, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO13] = REG_INIT(LDO13, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO14] = REG_INIT(LDO14, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO15] = REG_INIT(LDO15, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO16] = REG_INIT(LDO16, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO17] = REG_INIT(LDO17, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO18] = REG_INIT(LDO18, 650000, 2250000, 1, 1),
	[MAX8925_ID_LDO19] = REG_INIT(LDO19, 750000, 3900000, 1, 1),
	[MAX8925_ID_LDO20] = REG_INIT(LDO20, 750000, 3900000, 1, 1),
};

static struct max8925_backlight_pdata flint_backlight_data = {
	.dual_string	= 0,
};

static struct max8925_power_pdata flint_power_data = {
	.batt_detect		= 0,	/* can't detect battery by ID pin */
	.topoff_threshold	= MAX8925_TOPOFF_THR_10PER,
	.fast_charge		= MAX8925_FCHG_1000MA,
};

static struct max8925_platform_data flint_max8925_info = {
	.backlight		= &flint_backlight_data,
	.power			= &flint_power_data,
	.irq_base		= IRQ_BOARD_START,

	.regulator[MAX8925_ID_SD1]	= &regulator_data[MAX8925_ID_SD1],
	.regulator[MAX8925_ID_SD2]	= &regulator_data[MAX8925_ID_SD2],
	.regulator[MAX8925_ID_SD3]	= &regulator_data[MAX8925_ID_SD3],
	.regulator[MAX8925_ID_LDO1]	= &regulator_data[MAX8925_ID_LDO1],
	.regulator[MAX8925_ID_LDO2]	= &regulator_data[MAX8925_ID_LDO2],
	.regulator[MAX8925_ID_LDO3]	= &regulator_data[MAX8925_ID_LDO3],
	.regulator[MAX8925_ID_LDO4]	= &regulator_data[MAX8925_ID_LDO4],
	.regulator[MAX8925_ID_LDO5]	= &regulator_data[MAX8925_ID_LDO5],
	.regulator[MAX8925_ID_LDO6]	= &regulator_data[MAX8925_ID_LDO6],
	.regulator[MAX8925_ID_LDO7]	= &regulator_data[MAX8925_ID_LDO7],
	.regulator[MAX8925_ID_LDO8]	= &regulator_data[MAX8925_ID_LDO8],
	.regulator[MAX8925_ID_LDO9]	= &regulator_data[MAX8925_ID_LDO9],
	.regulator[MAX8925_ID_LDO10]	= &regulator_data[MAX8925_ID_LDO10],
	.regulator[MAX8925_ID_LDO11]	= &regulator_data[MAX8925_ID_LDO11],
	.regulator[MAX8925_ID_LDO12]	= &regulator_data[MAX8925_ID_LDO12],
	.regulator[MAX8925_ID_LDO13]	= &regulator_data[MAX8925_ID_LDO13],
	.regulator[MAX8925_ID_LDO14]	= &regulator_data[MAX8925_ID_LDO14],
	.regulator[MAX8925_ID_LDO15]	= &regulator_data[MAX8925_ID_LDO15],
	.regulator[MAX8925_ID_LDO16]	= &regulator_data[MAX8925_ID_LDO16],
	.regulator[MAX8925_ID_LDO17]	= &regulator_data[MAX8925_ID_LDO17],
	.regulator[MAX8925_ID_LDO18]	= &regulator_data[MAX8925_ID_LDO18],
	.regulator[MAX8925_ID_LDO19]	= &regulator_data[MAX8925_ID_LDO19],
	.regulator[MAX8925_ID_LDO20]	= &regulator_data[MAX8925_ID_LDO20],
};

static struct i2c_board_info flint_twsi1_info[] = {
	[0] = {
		.type		= "max8925",
		.addr		= 0x3c,
		.irq		= IRQ_MMP2_PMIC,
		.platform_data	= &flint_max8925_info,
	},
};
#endif

static void __init flint_init(void)
{
	mfp_config(ARRAY_AND_SIZE(flint_pin_config));

	/* on-chip devices */
	mmp2_add_uart(1);
	mmp2_add_uart(2);
	mmp2_add_uart(3);
	mmp2_add_twsi(0, NULL, ARRAY_AND_SIZE(flint_twsi1_info));

	/* off-chip devices */
	platform_device_register(&smc91x_device);
}

MACHINE_START(FLINT, "Flint Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = mmp2_init_irq,
	.timer          = &mmp2_timer,
	.init_machine   = flint_init,
MACHINE_END
