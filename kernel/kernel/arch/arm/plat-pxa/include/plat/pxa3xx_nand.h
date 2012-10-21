#ifndef __ASM_ARCH_PXA3XX_NAND_H
#define __ASM_ARCH_PXA3XX_NAND_H

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#define NUM_CHIP_SELECT		(2)
#define DMA_SUPPORT		(1)
#define POLLING_S		(1 << 1)
#define NAKEDCMD_S		(1 << 2)
/* the data flash bus is shared between the Static Memory
 * Controller and the Data Flash Controller,  the arbiter
 * controls the ownership of the bus
 */
#define ARBITER_ENABLE		(1 << 3)
struct pxa3xx_nand_platform_data {

	uint8_t pxa3xx_nand_mode;
	unsigned int RD_CNT_DEL;

	struct mtd_partition		*parts[NUM_CHIP_SELECT];
	unsigned int			nr_parts[NUM_CHIP_SELECT];
};

extern void pxa3xx_set_nand_info(struct pxa3xx_nand_platform_data *info);
#endif /* __ASM_ARCH_PXA3XX_NAND_H */
