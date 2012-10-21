/*
 *  linux/arch/arm/mach-pxa/saarb.c
 *
 *  Support for the Marvell PXA950 Handheld Platform (aka SAARB)
 *
 *  Copyright (C) 2007-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mfd/88pm860x.h>
#include <linux/regulator/machine.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/pxa9xx.h>

#include <plat/i2c.h>

#include "devices.h"
#include "generic.h"

/* SAARB MFP configurations */
static mfp_cfg_t saarb_mfp_cfg[] __initdata = {
	/* UART */
	GPIO53_UART1_RXD,
	GPIO54_UART1_TXD,

	/* I2C */
	GPIO73_CI2C_SCL,
	GPIO74_CI2C_SDA,

	/* PMIC */
	PMIC_INT_GPIO83,
};

#if defined(CONFIG_I2C_PXA) || defined(CONFIG_I2C_PXA_MODULE)
static struct regulator_consumer_supply regulator_supply[] = {
	[PM8607_ID_BUCK1]	= REGULATOR_SUPPLY("v_buck1", NULL),
	[PM8607_ID_BUCK3]	= REGULATOR_SUPPLY("v_buck3", NULL),
	[PM8607_ID_LDO1]	= REGULATOR_SUPPLY("v_ldo1", NULL),
	[PM8607_ID_LDO2]	= REGULATOR_SUPPLY("v_ldo2", NULL),
	[PM8607_ID_LDO3]	= REGULATOR_SUPPLY("v_ldo3", NULL),
	[PM8607_ID_LDO4]	= REGULATOR_SUPPLY("v_ldo4", NULL),
	[PM8607_ID_LDO5]	= REGULATOR_SUPPLY("v_ldo5", NULL),
	[PM8607_ID_LDO6]	= REGULATOR_SUPPLY("v_ldo6", NULL),
	[PM8607_ID_LDO7]	= REGULATOR_SUPPLY("v_ldo7", NULL),
	[PM8607_ID_LDO8]	= REGULATOR_SUPPLY("v_ldo8", NULL),
	[PM8607_ID_LDO9]	= REGULATOR_SUPPLY("v_ldo9", NULL),
	[PM8607_ID_LDO10]	= REGULATOR_SUPPLY("v_ldo10", NULL),
	[PM8607_ID_LDO12]	= REGULATOR_SUPPLY("v_ldo12", NULL),
	[PM8607_ID_LDO14]	= REGULATOR_SUPPLY("v_ldo14", NULL),
};

#define REG_INIT(_name, _min, _max, _always, _boot)			\
{									\
	.constraints = {						\
		.name		= __stringify(_name),			\
		.min_uV		= _min,					\
		.max_uV		= _max,					\
		.always_on	= _always,				\
		.boot_on	= _boot,				\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,		\
	},								\
	.num_consumer_supplies	= 1,					\
	.consumer_supplies	= &regulator_supply[PM8607_ID_##_name],	\
}

static struct regulator_init_data regulator_data[] = {
	[PM8607_ID_BUCK1] = REG_INIT(BUCK1, 0, 1500000, 1, 1),
	[PM8607_ID_BUCK3] = REG_INIT(BUCK3, 0, 3000000, 1, 1),
	[PM8607_ID_LDO1] = REG_INIT(LDO1, 1200000, 2800000, 1, 1),
	[PM8607_ID_LDO2] = REG_INIT(LDO2, 1800000, 3300000, 1, 1),
	[PM8607_ID_LDO3] = REG_INIT(LDO3, 1800000, 3300000, 1, 1),
	[PM8607_ID_LDO4] = REG_INIT(LDO4, 1800000, 3300000, 1, 1),
	[PM8607_ID_LDO5] = REG_INIT(LDO5, 2900000, 3300000, 1, 1),
	[PM8607_ID_LDO6] = REG_INIT(LDO6, 1800000, 3300000, 1, 1),
	[PM8607_ID_LDO7] = REG_INIT(LDO7, 1800000, 2900000, 1, 1),
	[PM8607_ID_LDO8] = REG_INIT(LDO8, 1800000, 2900000, 1, 1),
	[PM8607_ID_LDO9] = REG_INIT(LDO9, 1800000, 3300000, 1, 1),
	[PM8607_ID_LDO10] = REG_INIT(LDO10, 1200000, 3300000, 1, 1),
	[PM8607_ID_LDO12] = REG_INIT(LDO12, 1200000, 3300000, 1, 1),
	[PM8607_ID_LDO14] = REG_INIT(LDO14, 1800000, 3300000, 1, 1),
};

static struct pm860x_touch_pdata saarb_touch = {
	.gpadc_prebias	= 1,
	.slot_cycle	= 1,
	.tsi_prebias	= 6,
	.pen_prebias	= 16,
	.pen_prechg	= 2,
	.res_x		= 300,
};

static struct pm860x_backlight_pdata saarb_backlight[] = {
	{
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(24),
		.flags	= PM8606_BACKLIGHT1,
	},
	{},
};

static struct pm860x_led_pdata saarb_led[] = {
	{
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_BLUE,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED2_BLUE,
	},
};

static struct pm860x_platform_data saarb_pm8607_info = {
	.touch				= &saarb_touch,
	.backlight			= &saarb_backlight,
	.led				= &saarb_led,
	.companion_addr			= 0x10,
	.irq_mode			= 0,
	.irq_base			= IRQ_BOARD_START,

	.i2c_port			= GI2C_PORT,
	.regulator[PM8607_ID_BUCK1]	= &regulator_data[PM8607_ID_BUCK1],
	.regulator[PM8607_ID_BUCK3]	= &regulator_data[PM8607_ID_BUCK3],
	.regulator[PM8607_ID_LDO1]	= &regulator_data[PM8607_ID_LDO1],
	.regulator[PM8607_ID_LDO2]	= &regulator_data[PM8607_ID_LDO2],
	.regulator[PM8607_ID_LDO3]	= &regulator_data[PM8607_ID_LDO3],
	.regulator[PM8607_ID_LDO4]	= &regulator_data[PM8607_ID_LDO4],
	.regulator[PM8607_ID_LDO5]	= &regulator_data[PM8607_ID_LDO5],
	.regulator[PM8607_ID_LDO6]	= &regulator_data[PM8607_ID_LDO6],
	.regulator[PM8607_ID_LDO7]	= &regulator_data[PM8607_ID_LDO7],
	.regulator[PM8607_ID_LDO8]	= &regulator_data[PM8607_ID_LDO8],
	.regulator[PM8607_ID_LDO9]	= &regulator_data[PM8607_ID_LDO9],
	.regulator[PM8607_ID_LDO10]	= &regulator_data[PM8607_ID_LDO10],
	.regulator[PM8607_ID_LDO12]	= &regulator_data[PM8607_ID_LDO12],
	.regulator[PM8607_ID_LDO14]	= &regulator_data[PM8607_ID_LDO14],
};

static struct i2c_board_info saarb_i2c_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &saarb_pm8607_info,
		.irq		= gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO83)),
	},
};

static void __init saarb_init_i2c(void)
{
	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(saarb_i2c_info));
}
#else
static inline void saarb_init_i2c(void) {}
#endif

static void __init saarb_init(void)
{
	/* initialize MFP configurations */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(saarb_mfp_cfg));

	pxa_set_ffuart_info(NULL);

	saarb_init_i2c();
}

MACHINE_START(SAARB, "PXA950 Handheld Platform (aka SAARB)")
	.phys_io        = 0x40000000,
	.boot_params    = 0xa0000100,
	.io_pg_offst    = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io         = pxa_map_io,
	.init_irq       = pxa3xx_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = saarb_init,
MACHINE_END
