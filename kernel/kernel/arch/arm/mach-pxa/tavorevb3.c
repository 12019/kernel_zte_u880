/*
 *  linux/arch/arm/mach-pxa/tavorevb3.c
 *
 *  Support for the Marvell EVB3 Development Platform.
 *
 *  Copyright:  (C) Copyright 2008-2010 Marvell International Ltd.
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
#include <linux/dma-mapping.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/pxa9xx.h>

#include <plat/i2c.h>
#include <plat/pxa_u2o.h>

#include "devices.h"
#include "generic.h"

static mfp_cfg_t evb3_mfp_cfg[] __initdata = {
	/* UART */
	GPIO53_UART1_TXD,
	GPIO54_UART1_RXD,

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

static struct pm860x_touch_pdata evb3_touch = {
	.gpadc_prebias	= 1,
	.slot_cycle	= 1,
	.tsi_prebias	= 6,
	.pen_prebias	= 16,
	.pen_prechg	= 2,
	.res_x		= 300,
};

static struct pm860x_backlight_pdata evb3_backlight[] = {
	{
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(24),
		.flags	= PM8606_BACKLIGHT1,
	},
	{},
};

static struct pm860x_led_pdata evb3_led[] = {
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

static struct pm860x_platform_data evb3_pm8607_info = {
	.touch				= &evb3_touch,
	.backlight			= &evb3_backlight,
	.led				= &evb3_led,
	.companion_addr			= 0x10,
	.irq_mode			= 0,
	.irq_base			= IRQ_BOARD_START,

	.i2c_port			= GI2C_PORT,
#if 0
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
#endif
};

static struct i2c_board_info evb3_i2c_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &evb3_pm8607_info,
		//.irq		= gpio_to_irq(mfp_to_gpio(MFP_PIN_GPIO83)),
	},
};

static void __init evb3_init_i2c(void)
{
	pxa_set_i2c_info(NULL);
	i2c_register_board_info(0, ARRAY_AND_SIZE(evb3_i2c_info));
}
#else
static inline void evb3_init_i2c(void) {}
#endif

static struct pxa_usb_plat_info evb3_u2o_info = {
	.phy_init	= pxa_usb_phy_init,
};

static struct resource u2o_resources[] = {
	/* reg base */
	[0] = {
		.start	= PXA935_U2O_REGBASE,
		.end	= PXA935_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2o",
	},
	/* phybase */
	[1] = {
		.start	= PXA935_U2O_PHYBASE,
		.end	= PXA935_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2ophy",
	},
	[2] = {
		.start	= IRQ_U2O,
		.end	= IRQ_U2O,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device u2o_device = {
	.name		= "pxa-u2o",
	.id		= -1,
	.dev		= {
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &evb3_u2o_info,
	},

	.num_resources	= ARRAY_SIZE(u2o_resources),
	.resource	= u2o_resources,
};

struct platform_device otg_device = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev		= {
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &evb3_u2o_info,
	},

	.num_resources	= ARRAY_SIZE(u2o_resources),
	.resource	= u2o_resources,
};

static void __init evb3_init(void)
{
	/* initialize MFP configurations */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(evb3_mfp_cfg));

	pxa_set_ffuart_info(NULL);

	evb3_init_i2c();
	platform_device_register(&u2o_device);
	platform_device_register(&otg_device);
}

MACHINE_START(TAVOREVB3, "PXA950 Evaluation Board (aka TavorEVB3)")
	.phys_io	= 0x40000000,
	.boot_params	= 0xa0000100,
	.map_io         = pxa_map_io,
	.init_irq       = pxa3xx_init_irq,
	.timer          = &pxa_timer,
	.init_machine   = evb3_init,
MACHINE_END
