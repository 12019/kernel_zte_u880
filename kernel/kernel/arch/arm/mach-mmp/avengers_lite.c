/*
 *  linux/arch/arm/mach-mmp/avengers_lite.c
 *
 *  Support for the Marvell PXA168-based Avengers lite Development Platform.
 *
 *  Copyright (C) 2009-2010 Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/irqs.h>


#include "common.h"
#include <linux/delay.h>

/* Avengers lite MFP configurations */
static unsigned long avengers_lite_pin_config_V16F[] __initdata = {
	/* DEBUG_UART */
	GPIO88_UART2_TXD,
	GPIO89_UART2_RXD,
};

static struct mtd_partition avengers_nand_partitions_0[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= 0x100000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "MassStorage0",
		.offset		= 0x100000,
		.size		= MTDPART_SIZ_FULL,
	}
};

static struct mtd_partition avengers_nand_partitions_1[] = {
	{
		.name		= "reserved",
		.offset		= 0,
		.size		= 0xA00000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "Kernel",
		.offset		= 0xA00000,
		.size		= 0x380000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "Kernel_recovery",
		.offset		= 0xD80000,
		.size		= 0x400000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "System",
		.offset		= 0x1180000,
		.size		= 0xEE80000,
	}, {
		.name		= "Userdata",
		.offset		= 0x10000000,
		.size		= 0x70000000,
	}, {
		.name		= "MassStorage1",
		.offset		= 0x80000000,
		.size		= MTDPART_SIZ_FULL,
	}
};

static struct pxa3xx_nand_platform_data avengers_nand_info;
static void __init avengers_init_flash(void)
{
	avengers_nand_info.pxa3xx_nand_mode = DMA_SUPPORT | ARBITER_ENABLE | NAKEDCMD_S,
	avengers_nand_info.parts[0] = avengers_nand_partitions_0;
	avengers_nand_info.nr_parts[0] = ARRAY_SIZE(avengers_nand_partitions_0);
	avengers_nand_info.parts[1] = avengers_nand_partitions_1;
	avengers_nand_info.nr_parts[1] = ARRAY_SIZE(avengers_nand_partitions_1);
	pxa168_add_nand(&avengers_nand_info);
}

static void __init avengers_lite_init(void)
{
	mfp_config(ARRAY_AND_SIZE(avengers_lite_pin_config_V16F));

	/* on-chip devices */
	pxa168_add_uart(2);
	avengers_init_flash();
}

MACHINE_START(AVENGERS_LITE, "PXA168 Avengers lite Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = avengers_lite_init,
MACHINE_END
