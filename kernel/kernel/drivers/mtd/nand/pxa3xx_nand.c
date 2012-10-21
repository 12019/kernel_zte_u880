/*
 * drivers/mtd/nand/pxa3xx_nand.c
 *
 * Copyright © 2005 Intel Corporation
 * Copyright © 2006 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <mach/dma.h>
#include <plat/pxa3xx_nand.h>
#ifdef CONFIG_PXA3XX_BBM
#include <plat/pxa3xx_bbm.h>
#endif
#ifdef CONFIG_TRACEPOINTS
#define CREATE_TRACE_POINTS
#include "pxa3xx_nand_trace.h"
#endif
#include <asm/cacheflush.h>

#define	CHIP_DELAY_TIMEOUT	(100 * HZ/10)
#define NAND_STOP_DELAY		(2 * HZ/50)
#define PAGE_CHUNK_SIZE		(2048)
#define OOB_CHUNK_SIZE		(64)
#define BCH_THRESHOLD           (8)
#define CMD_POOL_SIZE           (5)
#undef PXA3XX_NAND_DEBUG
#ifdef PXA3XX_NAND_DEBUG
#define DBG_NAND(x)	do{x;}while(0)
#else
#define DBG_NAND(x)
#endif
/* the max buff size should be large than
 * the largest size of page of NAND flash
 * that currently controller support
 */
#define DMA_H_SIZE	(sizeof(struct pxa_dma_desc) * 2)
#define MAX_DATA_SZ	(PAGE_CHUNK_SIZE * 2)
#define MAX_OOB_SZ	(OOB_CHUNK_SIZE * 2)
#define MAX_BUFF_SIZE	(MAX_DATA_SZ + MAX_OOB_SZ)

/* registers and bit definitions */
#define NDCR		(0x00) /* Control register */
#define NDTR0CS0	(0x04) /* Timing Parameter 0 for CS0 */
#define NDTR1CS0	(0x0C) /* Timing Parameter 1 for CS0 */
#define NDSR		(0x14) /* Status Register */
#define NDPCR		(0x18) /* Page Count Register */
#define NDBDR0		(0x1C) /* Bad Block Register 0 */
#define NDBDR1		(0x20) /* Bad Block Register 1 */
#define NDREDEL		(0x24) /* Read Enable Return Delay Register */
#define NDECCCTRL	(0x28) /* ECC Control Register */
#define NDBZCNT		(0x2C) /* Timer for NDRnB0 and NDRnB1 */
#define NDDB		(0x40) /* Data Buffer */
#define NDCB0		(0x48) /* Command Buffer0 */
#define NDCB1		(0x4C) /* Command Buffer1 */
#define NDCB2		(0x50) /* Command Buffer2 */

#define NDCR_SPARE_EN		(0x1 << 31)
#define NDCR_ECC_EN		(0x1 << 30)
#define NDCR_DMA_EN		(0x1 << 29)
#define NDCR_ND_RUN		(0x1 << 28)
#define NDCR_DWIDTH_C		(0x1 << 27)
#define NDCR_DWIDTH_M		(0x1 << 26)
#define NDCR_PAGE_SZ_MASK	(0x3 << 24)
#define NDCR_PAGE_SZ(x)		(((x) << 24) & NDCR_PAGE_SZ_MASK)
#define NDCR_SEQ_DIS		(0x1 << 23)
#define NDCR_ND_STOP		(0x1 << 22)
#define NDCR_FORCE_CSX		(0x1 << 21)
#define NDCR_CLR_PG_CNT		(0x1 << 20)
#define NDCR_STOP_ON_UNCOR	(0x1 << 19)
#define NDCR_RD_ID_CNT_MASK	(0x7 << 16)
#define NDCR_RD_ID_CNT(x)	(((x) << 16) & NDCR_RD_ID_CNT_MASK)

#define NDCR_RA_START		(0x1 << 15)
#define NDCR_PG_PER_BLK_MASK	(0x3 << 13)
#define NDCR_PG_PER_BLK(x)	(((x) << 13) & NDCR_PG_PER_BLK_MASK)
#define NDCR_ND_ARB_EN		(0x1 << 12)
#define NDCR_RDYM               (0x1 << 11)
#define NDCR_CS0_PAGEDM         (0x1 << 10)
#define NDCR_CS1_PAGEDM         (0x1 << 9)
#define NDCR_CS0_CMDDM          (0x1 << 8)
#define NDCR_CS1_CMDDM          (0x1 << 7)
#define NDCR_CS0_BBDM           (0x1 << 6)
#define NDCR_CS1_BBDM           (0x1 << 5)
#define NDCR_UNCERRM            (0x1 << 4)
#define NDCR_CORERRM            (0x1 << 3)
#define NDCR_WRDREQM            (0x1 << 2)
#define NDCR_RDDREQM            (0x1 << 1)
#define NDCR_WRCMDREQM          (0x1)
#define NDCR_INT_MASK           (0xFFF)

/* Data Controller Timing Paramter x Register For CSx */
#define NDTR0_tADL(c)           (min_t(uint32_t, (c), 31) << 27)
#define NDTR0_SELCNTR           (0x1 << 26)
#define NDTR0_RD_CNT_DEL_MASK   (0xF << 22)
#define NDTR0_RD_CNT_DEL(x)     (((x) << 22) & NDTR0_RD_CNT_DEL_MASK)
#define NDTR0_tCH(c)            (min_t(uint32_t, (c), 7) << 19)
#define NDTR0_tCS(c)            (min_t(uint32_t, (c), 7) << 16)
#define NDTR0_tWH(c)            (min_t(uint32_t, (c), 7) << 11)
#define NDTR0_tWP(c)            (min_t(uint32_t, (c), 7) << 8)
#define NDTR0_sel_NRE_EDGE      (0x1 << 7)
#define NDTR0_ETRP              (0x1 << 6)
#define NDTR0_tRH(c)            (min_t(uint32_t, (c), 7) << 3)
#define NDTR0_tRP(c)            (min_t(uint32_t, (c), 7) << 0)

#define NDTR1_tR(c)             (min_t(uint32_t, (c), 65535) << 16)
#define NDTR1_WAIT_MODE         (0x1 << 15)
#define NDTR1_PRESCALE          (0x1 << 14)
#define NDTR1_tRHW(c)           (min_t(uint32_t, (c), 3) << 8)
#define NDTR1_tWHR(c)           (min_t(uint32_t, (c), 15) << 4)
#define NDTR1_tAR(c)            (min_t(uint32_t, (c), 15) << 0)

#define NDSR_MASK		(0xfff)
#define NDSR_ERR_CNT_MASK       (0x1F << 16)
#define NDSR_ERR_CNT(x)         (((x) << 16) & NDSR_ERR_CNT_MASK)
#define NDSR_RDY                (0x1 << 12)
#define NDSR_FLASH_RDY          (0x1 << 11)
#define NDSR_CS0_PAGED		(0x1 << 10)
#define NDSR_CS1_PAGED		(0x1 << 9)
#define NDSR_CS0_CMDD		(0x1 << 8)
#define NDSR_CS1_CMDD		(0x1 << 7)
#define NDSR_CS0_BBD		(0x1 << 6)
#define NDSR_CS1_BBD		(0x1 << 5)
#define NDSR_DBERR		(0x1 << 4)
#define NDSR_SBERR		(0x1 << 3)
#define NDSR_WRDREQ		(0x1 << 2)
#define NDSR_RDDREQ		(0x1 << 1)
#define NDSR_WRCMDREQ		(0x1)

#define NDCB0_CMD_XTYPE_MASK    (0x7 << 29)
#define NDCB0_CMD_XTYPE(x)      (((x) << 29) & NDCB0_CMD_XTYPE_MASK)
#define NDCB0_LEN_OVRD		(0x1 << 28)
#define NDCB0_ST_ROW_EN         (0x1 << 26)
#define NDCB0_AUTO_RS		(0x1 << 25)
#define NDCB0_CSEL		(0x1 << 24)
#define NDCB0_CMD_TYPE_MASK	(0x7 << 21)
#define NDCB0_CMD_TYPE(x)	(((x) << 21) & NDCB0_CMD_TYPE_MASK)
#define NDCB0_NC		(0x1 << 20)
#define NDCB0_DBC		(0x1 << 19)
#define NDCB0_ADDR_CYC_MASK	(0x7 << 16)
#define NDCB0_ADDR_CYC(x)	(((x) << 16) & NDCB0_ADDR_CYC_MASK)
#define NDCB0_CMD2_MASK		(0xff << 8)
#define NDCB0_CMD1_MASK		(0xff)
#define NDCB0_ADDR_CYC_SHIFT	(16)

/* ECC Control Register */
#define NDECCCTRL_ECC_SPARE_MSK (0xFF << 7)
#define NDECCCTRL_ECC_SPARE(x)  (((x) << 7) & NDECCCTRL_ECC_SPARE_MSK)
#define NDECCCTRL_ECC_THR_MSK   (0x3F << 1)
#define NDECCCTRL_ECC_THRESH(x) (((x) << 1) & NDECCCTRL_ECC_THR_MSK)
#define NDECCCTRL_BCH_EN        (0x1)

/* macros for registers read/write */
#define nand_writel(info, off, val)	\
	__raw_writel((val), (info)->mmio_base + (off))

#define nand_readl(info, off)		\
	__raw_readl((info)->mmio_base + (off))
#define get_mtd_by_info(info)		\
	(struct mtd_info *)((void *)info - sizeof(struct mtd_info))

/* error code and state */
enum {
	ERR_NONE	= 0,
	ERR_DMABUSERR	= -1,
	ERR_SENDCMD	= -2,
	ERR_DBERR	= -3,
	ERR_BBERR	= -4,
	ERR_SBERR	= -5,
};

enum {
	STATE_CMD_WAIT_DONE	= 1,
	STATE_DATA_PROCESSING	= (1 << 1),
	STATE_DATA_DONE		= (1 << 2),
	STATE_PAGE_DONE		= (1 << 3),
	STATE_CMD_DONE		= (1 << 4),
	STATE_READY		= (1 << 5),
	STATE_CMD_PREPARED	= (1 << 6),
	STATE_IS_WRITE		= (1 << 7),
};

#define STATE_MASK		(0x3f)
/* error code and state */
enum {
	ECC_NONE = 0,
	ECC_HAMMIN,
	ECC_BCH,
};

struct pxa3xx_nand_timing {
	uint32_t	tADL; /* Adress to Write Data delay */
	uint32_t	tCH;  /* Enable signal hold time */
	uint32_t	tCS;  /* Enable signal setup time */
	uint32_t	tWH;  /* ND_nWE high duration */
	uint32_t	tWP;  /* ND_nWE pulse time */
	uint32_t	tRH;  /* ND_nRE high duration */
	uint32_t	tRP;  /* ND_nRE pulse width */
	uint32_t	tR;   /* ND_nWE high to ND_nRE low for read */
	uint32_t	tRHW; /* delay for next command issue */
	uint32_t	tWHR; /* ND_nWE high to ND_nRE low for status read */
	uint32_t	tAR;  /* ND_ALE low to ND_nRE low delay */
};

struct pxa3xx_nand_cmdset {
	uint16_t	read1;
	uint16_t	read2;
	uint16_t	program;
	uint16_t	read_status;
	uint16_t	read_id;
	uint16_t	erase;
	uint16_t	reset;
	uint16_t	lock;
	uint16_t	unlock;
	uint16_t	lock_status;
};

struct pxa3xx_nand_flash {
	uint32_t	chip_id;
	uint16_t	page_per_block; /* Pages per block */
	uint16_t 	page_size;	/* Page size in bytes */
	uint8_t		flash_width;	/* Width of Flash memory (DWIDTH_M) */
	uint8_t 	dfc_width;	/* Width of flash controller(DWIDTH_C) */
	uint8_t		ecc_type;	/* Which ECC is applied */
	uint32_t	num_blocks;	/* Number of physical blocks in Flash */
	struct pxa3xx_nand_timing timing;	/* NAND Flash timing */
};

struct pxa3xx_nand_info {
	struct nand_chip	nand_chip;
	/* page size of attached chip */
	uint16_t		page_size;
	/* the page addr last time accessed */
	int			page_addr;
	uint8_t			chip_select;

	/* use HW ECC ? */
	/* 0:off, 1:Hammin ECC  2: BCH ECC */
	uint8_t			use_ecc;

	/* calculated from pxa3xx_nand_flash data */
	uint8_t			col_addr_cycles;
	uint8_t			row_addr_cycles;
	uint8_t			read_id_bytes;

	/* cached register value */
	uint32_t		reg_ndcr;
	uint32_t		ndtr0cs0;
	uint32_t		ndtr1cs0;

	void			*nand_data;
};

struct pxa3xx_nand {
	struct clk		*clk;
	void __iomem		*mmio_base;
	unsigned long		mmio_phys;
	struct nand_hw_control	controller;
	struct completion 	cmd_complete;
	struct platform_device	*pdev;

	/* DMA information */
	uint32_t		drcmr_dat;
	uint32_t		drcmr_cmd;
	int			data_dma_ch;
	dma_addr_t		data_offset;
	dma_addr_t 		dma_buff_phys;

	struct pxa3xx_nand_info *info[NUM_CHIP_SELECT];
	/* relate to the command */
	uint32_t		command;
	uint16_t		data_size;	/* data size in FIFO */
	uint16_t		oob_size;
	uint32_t		bad_count;
	unsigned char		*dma_buff;
	unsigned char		*data_buff;
	unsigned char		*oob_buff;
	uint32_t		buf_start;
	uint32_t		buf_count;
	uint8_t			total_cmds;
	uint8_t			mode;

	uint8_t			chip_select;
	uint8_t			use_ecc;
	uint32_t		RD_CNT_DEL;
	uint32_t		ndcb1[CMD_POOL_SIZE];
	uint32_t		ndcb2[CMD_POOL_SIZE];

	uint32_t		state;
	uint32_t		retcode;
	uint16_t		data_column;
	uint16_t		oob_column;
	uint8_t			cmd_seqs;
	uint8_t			wait_ready[CMD_POOL_SIZE];
	uint32_t		ndcb0[CMD_POOL_SIZE];
};

static uint8_t pxa3xx_nand_mode = DMA_SUPPORT | ARBITER_ENABLE | NAKEDCMD_S;
module_param(pxa3xx_nand_mode, byte, 0444);
MODULE_PARM_DESC(pxa3xx_nand_mode, "default pxa3xx_nand mode");

const static struct pxa3xx_nand_cmdset cmdset = {
	.read1		= 0x3000,
	.read2		= 0x0050,
	.program	= 0x1080,
	.read_status	= 0x0070,
	.read_id	= 0x0090,
	.erase		= 0xD060,
	.reset		= 0x00FF,
	.lock		= 0x002A,
	.unlock		= 0x2423,
	.lock_status	= 0x007A,
};

/**
 * The timing defined in the builtin_flash_types[0]
 * could be considered as the common timing which is used to
 * detect the chip id before we know how to optimize further
 */
static uint32_t nand_dump_times = 0;
int flash_type=0;
static struct pxa3xx_nand_flash __devinitdata builtin_flash_types[] = {
{ 0, 0, 0, 0, 0, 0, 0, { 0, 40, 80, 60, 100, 80, 100, 90000, 0, 400, 40, }, },
{ 0x46ec, 32, 512, 16, 16, ECC_HAMMIN, 4096, \
	{ 0, 10, 0, 20, 40, 30, 40, 11123, 0, 110, 10, }, },
{ 0xdaec, 64, 2048, 8, 8, ECC_HAMMIN, 2048, \
	{ 0, 10, 0, 20, 40, 30, 40, 25000, 0, 110, 10, }, },
{ 0xbcec, 64, 2048, 16, 16, ECC_HAMMIN, 4096, \
	//{ 100, 5, 21, 10, 21, 10, 21, 60, 100, 60, 10, }, },
	{ 100, 10, 25, 10, 25, 10, 25, 60, 100, 60, 10, }, },
{ 0xd3ec, 128, 2048, 8, 8, ECC_BCH, 4096, \
	{ 0, 10, 0, 20, 40, 30, 40, 11123, 0, 110, 10, }, },
{ 0xd7ec, 128, 4096, 8, 8, ECC_BCH, 8192, \
	{ 200, 10, 15, 10, 12, 10, 8, 60000, 20, 75, 10, }, },
{ 0xa12c, 64, 2048, 8, 8, ECC_HAMMIN, 1024, \
	{ 0, 10, 25, 15, 25, 15, 30, 25000, 0, 60, 10, }, },
{ 0xb12c, 64, 2048, 16, 16, ECC_HAMMIN, 1024, \
	{ 0, 10, 25, 15, 25, 15, 30, 25000, 0, 60, 10, }, },
{ 0xba2c, 64, 2048, 16, 16, ECC_HAMMIN, 2048, \
	{ 0, 10, 25, 15, 25, 15, 30, 25000, 0, 60, 10, }, },
{ 0xdc2c, 64, 2048, 8, 8, ECC_HAMMIN, 4096, \
	{ 0, 10, 25, 15, 25, 15, 30, 25000, 0, 60, 10, }, },
{ 0xcc2c, 64, 2048, 16, 16, ECC_HAMMIN, 4096, \
	{ 0, 10, 25, 15, 25, 15, 30, 25000, 0, 60, 10, }, },
{ 0x382c, 128, 4096, 8, 8, ECC_BCH, 2048, \
	{ 120, 8, 55, 15, 30, 30, 30, 25000, 60, 170, 15, }, },
{ 0xba20, 64, 2048, 16, 16, ECC_HAMMIN, 2048, \
	{ 0, 10, 35, 15, 25, 15, 25, 25000, 0, 60, 10, }, },
#ifdef CONFIG_PXA_U812
{ 0xbcad, 64, 2048, 16, 16, ECC_BCH, 4096, \
	{ 0, 5, 15, 10, 10, 10, 10, 0, 0, 80, 5, }, },
#else
{ 0xbcad, 64, 2048, 16, 16, ECC_HAMMIN, 4096, \
	{ 0, 5, 15, 10, 10, 10, 10, 0, 0, 80, 5, }, },
#endif
{ 0xbc2c, 64, 2048, 16, 16, ECC_HAMMIN, 4096, \
	{ 0, 10, 25, 15,  25, 15,  30, 25000,  100,  80, 10, }, },
};

#ifdef CONFIG_MTD_CMDLINE_PARTS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif
static const char *mtd_names[] = {"pxa3xx_nand-0", "pxa3xx_nand-1", NULL};

/* convert nano-seconds to nand flash controller clock cycles */
#define ns2cycle(ns, clk)       (int)(((ns) * (clk / 1000000) / 1000) + 1)

/* convert nand flash controller clock cycles to nano-seconds */
#define cycle2ns(cycle, clk)    (cycle * 1000 / (clk / 1000000))

#if defined(CONFIG_DVFM)
#include <mach/dvfm.h>
static int dvfm_dev_idx;

static void set_dvfm_constraint(void)
{
	/*Disable frequency change during nand transaction*/
	dvfm_disable_global(dvfm_dev_idx);
	/* Disable Low power mode */
	dvfm_disable_lowpower(dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	dvfm_enable_global(dvfm_dev_idx);
	/* Enable Low power mode*/
	dvfm_enable_lowpower(dvfm_dev_idx);
}

#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

static dma_addr_t map_addr(struct pxa3xx_nand *nand, void *buf,
			   size_t sz, int dir)
{
	struct device *dev = &nand->pdev->dev;
	/* if not cache aligned, don't use dma */
	if (((size_t)buf & 0x1f) || (sz & 0x1f))
		return ~0;

	if (buf >= high_memory) {
		struct page *page;

		if (((size_t) buf & PAGE_MASK) !=
		    ((size_t) (buf + sz - 1) & PAGE_MASK))
			return ~0;

		page = vmalloc_to_page(buf);
		if (!page)
			return ~0;

		if (dir == DMA_FROM_DEVICE)
			dmac_inv_range(buf,  buf + sz);
		else
			dmac_clean_range(buf, buf + sz);
		return dma_map_page(dev, page, (size_t)buf & (PAGE_SIZE - 1), sz, dir);
	}

	return dma_map_single(dev, buf, sz, dir);
}

static void unmap_addr(struct device *dev, dma_addr_t buf, void *orig_buf,
		       size_t sz, int dir)
{
	if (!buf)
		return;
	if (orig_buf >= high_memory)
		dma_unmap_page(dev, buf, sz, dir);
	else
		dma_unmap_single(dev, buf, sz, dir);
}

/*
 * This function shows the real timing when NAND controller
 * send signal to the NAND chip.
 */
static void show_real_timing(uint32_t ndtr0, uint32_t ndtr1, unsigned long nand_clk)
{
	uint32_t rtADL, rtCH, rtCS, rtWH, rtWP, rtRH, rtRP;
	uint32_t rtR, rtRHW, rtWHR, rtAR, tmp;

	rtCH = ((ndtr0 >> 19) & 0x7) + 1;
	rtCS = ((ndtr0 >> 16) & 0x7) + 1;
	rtWH = ((ndtr0 >> 11) & 0x7) + 1;
	rtWP = ((ndtr0 >> 8) & 0x7) + 1;
	rtADL= (ndtr0 >> 27) & 0x1f;
	rtRH = ((ndtr0 >> 3) & 0x7) + 1;
	rtRP = (ndtr0 & NDTR0_ETRP) ? ((0x8 | (ndtr0 & 0x7)) + 1)
			: ((ndtr0 & 0x7) + 1);
	rtRHW = (ndtr1 >> 8) & 0x3;
	rtWHR = (ndtr1 >> 4) & 0xf;
	rtAR = ndtr1 & 0xf;
	rtR = (ndtr1 >> 16) & 0xffff;

	if (ndtr1 & NDTR1_PRESCALE)
		rtR *= 16;

	rtR += rtCH + 2;
	switch(rtRHW) {
	case 0:
		rtRHW = 0;
		break;
	case 1:
		rtRHW = 16;
		break;
	case 2:
		rtRHW = 32;
		break;
	case 3:
		rtRHW = 48;
		break;
	}

	/*
	 * TWHR delay=max(tAR, max(0, tWHR-max(tWH, tCH)))
	 * TAR delay=max(tAR, max(0, tWHR-max(tWH, tCH))) + 2
	 */
	if (rtWH > rtCH)
		tmp = rtWH - 1;
	else
		tmp = rtCH - 1;
	if (rtADL != 0) {
		rtADL = rtADL - 3 - rtWP;
		rtADL = rtADL > 0 ? rtADL : 0;
		rtADL = rtADL + tmp + rtWP + 8;
	}
	if (rtWHR < tmp)
		rtWHR = rtAR;
	else {
		if (rtAR > (rtWHR - tmp))
			rtWHR = rtAR;
		else
			rtWHR = rtWHR - tmp;
	}
	rtAR = rtWHR + 2;
	printk("Shows real timing(ns):\n");
	if (ndtr0 & NDTR0_SELCNTR)
		printk("NDTR0 SELCNTR is set\n");
	else
		printk("NDTR0 SELCNTR is not set\n");
	if (ndtr0 & NDTR0_RD_CNT_DEL_MASK)
		printk("Read Strobe delay is %d\n",
				(ndtr0 & NDTR0_RD_CNT_DEL_MASK) >> 22);
	else
		printk("No Read Stobe delay\n");
	if (ndtr0 & NDTR0_sel_NRE_EDGE)
		printk("Controller is using falling edge to detect RE\n");
	else
		printk("Controller is using rising edge to detect RE\n");

	if (ndtr1 & NDTR1_WAIT_MODE)
		printk("NDTR1 wait mode is set\n");
	else
		printk("NDTR1 wait mode is not set\n");

	printk("TADL is %ld TCH is %ld TCS is %ld TWH is %ld TWP is %ld TRH is %ld "
		"TRP is %ld TR is %ld TRHW is %ld TWHR is %ld TAR is %ld\n",
		cycle2ns(rtADL, nand_clk), cycle2ns(rtCH, nand_clk),
		cycle2ns(rtCS, nand_clk), cycle2ns(rtWH, nand_clk),
		cycle2ns(rtWP, nand_clk), cycle2ns(rtRH, nand_clk),
		cycle2ns(rtRP, nand_clk), cycle2ns(rtR, nand_clk),
		cycle2ns(rtRHW, nand_clk), cycle2ns(rtWHR, nand_clk),
		cycle2ns(rtAR, nand_clk));
}

static void pxa3xx_nand_set_timing(struct pxa3xx_nand_info *info,
			   struct pxa3xx_nand_timing *t, int show_timing)
{
	struct pxa3xx_nand *nand = info->nand_data;
	unsigned long nand_clk;
	uint32_t ndtr0, ndtr1, tRP, tR, tRHW, tADL;

	nand_clk = clk_get_rate(nand->clk);
	ndtr0 = ndtr1 = 0;
	tRP = ns2cycle(t->tRP, nand_clk);
	tRP = (tRP > 0xf) ? 0xf : tRP;
	if (tRP > 0x7) {
		ndtr0 |= NDTR0_ETRP;
		tRP -= 0x7;
	}
	if (unlikely(info->page_size < PAGE_CHUNK_SIZE) || !(nand->mode & NAKEDCMD_S)) {
		tR = ns2cycle(t->tR, nand_clk);
		if (tR > 0xffff) {
			ndtr1 |= NDTR1_PRESCALE;
			tR /= 16;
		}
	}
	else
		tR = 0;
	if (t->tRHW > 0) {
		tRHW = ns2cycle(t->tRHW, nand_clk);
		if (tRHW < 16)
			tRHW = 1;
		else {
			if (tRHW < 32)
				tRHW = 2;
			else
				tRHW = 3;
		}
	}
	else
		tRHW = 0;
	tADL = (t->tADL > 0) ? ns2cycle(t->tADL, nand_clk) : 0;

	if (nand->RD_CNT_DEL > 0)
		ndtr0 |= NDTR0_SELCNTR
			| (NDTR0_RD_CNT_DEL(nand->RD_CNT_DEL - 1));

	ndtr0 |= NDTR0_tADL(tADL)
		| NDTR0_tCH(ns2cycle(t->tCH, nand_clk))
		| NDTR0_tCS(ns2cycle(t->tCS, nand_clk))
		| NDTR0_tWH(ns2cycle(t->tWH, nand_clk))
		| NDTR0_tWP(ns2cycle(t->tWP, nand_clk))
		| NDTR0_tRH(ns2cycle(t->tRH, nand_clk))
		| NDTR0_tRP(tRP)
		| NDTR0_SELCNTR;

	ndtr1 |= NDTR1_tR(tR)
		| NDTR1_tRHW(tRHW)
		| NDTR1_tWHR(ns2cycle(t->tWHR, nand_clk))
		| NDTR1_tAR(ns2cycle(t->tAR, nand_clk));

	nand_writel(nand, NDTR0CS0, ndtr0);
	nand_writel(nand, NDTR1CS0, ndtr1);
	nand_writel(nand, NDREDEL, 0x0);
	info->ndtr0cs0 = ndtr0;
	info->ndtr1cs0 = ndtr1;
	if (show_timing)
		show_real_timing(ndtr0, ndtr1, nand_clk);
}

static void pxa3xx_set_datasize(struct pxa3xx_nand_info *info)
{
	struct pxa3xx_nand *nand = info->nand_data;
	int oob_enable = info->reg_ndcr & NDCR_SPARE_EN;

	if (info->page_size >= PAGE_CHUNK_SIZE) {
		nand->data_size = PAGE_CHUNK_SIZE;
		if (!oob_enable) {
			nand->oob_size = 0;
			return;
		}

		switch (nand->use_ecc) {
		case ECC_HAMMIN:
			nand->oob_size = 40;
			break;
		case ECC_BCH:
			nand->oob_size = 32;
			break;
		default:
			nand->oob_size = 64;
			break;
		}
	}
	else {
		nand->data_size = 512;
		if (!oob_enable) {
			nand->oob_size = 0;
			return;
		}

		switch (nand->use_ecc) {
		case ECC_HAMMIN:
			nand->oob_size = 8;
			break;
		case ECC_BCH:
			printk("Don't support BCH on small page device!!!\n");
			BUG();
			break;
		default:
			nand->oob_size = 16;
			break;
		}
	}
}

/**
 * NOTE: it is a must to set ND_RUN firstly, then write
 * command buffer, otherwise, it does not work.
 * We enable all the interrupt at the same time, and
 * let pxa3xx_nand_irq to handle all logic.
 */
static void pxa3xx_nand_start(struct pxa3xx_nand *nand)
{
	struct pxa3xx_nand_info *info;
	uint32_t ndcr, ndeccctrl = 0;

	info = nand->info[nand->chip_select];
	ndcr = info->reg_ndcr;
	ndcr |= (nand->mode & DMA_SUPPORT) ? NDCR_DMA_EN : 0;
	ndcr |= (nand->mode & POLLING_S)? NDCR_INT_MASK : 0;
	ndcr |= (nand->mode & ARBITER_ENABLE) ? NDCR_ND_ARB_EN : 0;
	ndcr |= NDCR_ND_RUN;

	switch (nand->use_ecc) {
	case ECC_BCH:
		ndeccctrl |= NDECCCTRL_BCH_EN;
		ndeccctrl |= NDECCCTRL_ECC_THRESH(BCH_THRESHOLD);
	case ECC_HAMMIN:
		ndcr |= NDCR_ECC_EN;
	default:
		break;
	}

	/* clear status bits and run */
	DBG_NAND(printk("@@@ndcr set: %x, ndeccctrl set %x\n",
				ndcr, ndeccctrl));
	nand_writel(nand, NDCR, 0);
	nand_writel(nand, NDECCCTRL, ndeccctrl);
	nand_writel(nand, NDSR, NDSR_MASK);
	nand_writel(nand, NDCR, ndcr);
}

static void pxa3xx_nand_stop(struct pxa3xx_nand *nand)
{
	uint32_t ndcr;
	int timeout = NAND_STOP_DELAY;

	/* wait RUN bit in NDCR become 0 */
	do {
		/* clear status bits */
		nand_writel(nand, NDSR, NDSR_MASK);
		ndcr = nand_readl(nand, NDCR);
		udelay(1);
	} while ((ndcr & NDCR_ND_RUN) && (timeout -- > 0));

	if (timeout <= 0) {
		ndcr &= ~(NDCR_ND_RUN);
		nand_writel(nand, NDCR, ndcr);
	}
}

static void enable_int(struct pxa3xx_nand *nand, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = nand_readl(nand, NDCR);
	nand_writel(nand, NDCR, ndcr & ~int_mask);
}

static void disable_int(struct pxa3xx_nand *nand, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = nand_readl(nand, NDCR);
	nand_writel(nand, NDCR, ndcr | int_mask);
}

static void nand_error_dump(struct pxa3xx_nand *nand)
{
	int i;

	printk(KERN_ERR "NAND controller state wrong!!!\n");
	printk(KERN_ERR "command %x, state %x, current seqs %d, errcode %x, bad count %d\n",
			nand->command, nand->state, nand->cmd_seqs,
			nand->retcode, nand->bad_count);
	printk(KERN_ERR "Totally %d command for sending\n",
			nand->total_cmds);
	for (i = 0; i < nand->total_cmds; i ++)
		printk(KERN_ERR "==%d: NDCB0 %x, NDCB1 %x, NDCB2 %x\n",
				i, nand->ndcb0[i], nand->ndcb1[i],
				nand->ndcb2[i]);

	printk(KERN_ERR "\nRegister DUMPing ##############\n");
	printk(KERN_ERR "NDCR %x\n"
			"NDSR %x\n"
			"NDCB0 %x\n"
			"NDCB1 %x\n"
			"NDCB2 %x\n"
			"NDTR0CS0 %x\n"
			"NDTR1CS0 %x\n"
			"NDBDR0 %x\n"
			"NDBDR1 %x\n"
			"NDREDEL %x\n"
			"NDECCCTRL %x\n"
			"NDBZCNT %x\n\n",
			nand_readl(nand, NDCR),
			nand_readl(nand, NDSR),
			nand_readl(nand, NDCB0),
			nand_readl(nand, NDCB1),
			nand_readl(nand, NDCB2),
			nand_readl(nand, NDTR0CS0),
			nand_readl(nand, NDTR1CS0),
			nand_readl(nand, NDBDR0),
			nand_readl(nand, NDBDR1),
			nand_readl(nand, NDREDEL),
			nand_readl(nand, NDECCCTRL),
			nand_readl(nand, NDBZCNT));
}

static void handle_data_pio(struct pxa3xx_nand *nand)
{
	uint16_t data_size, oob_size;

	data_size = DIV_ROUND_UP(nand->data_size, 4);
	oob_size = DIV_ROUND_UP(nand->oob_size, 4);
	DBG_NAND(printk("data col %x, size %x, oob col %x size %x\n",
				nand->data_column, nand->data_size,
				nand->oob_column, nand->oob_size));
	if (nand->state & STATE_IS_WRITE) {
		__raw_writesl(nand->mmio_base + NDDB,
				nand->data_buff + nand->data_column, data_size);
		if (oob_size > 0)
			__raw_writesl(nand->mmio_base + NDDB,
				nand->oob_buff + nand->oob_column, oob_size);

	}
	else {
		__raw_readsl(nand->mmio_base + NDDB,
				nand->data_buff + nand->data_column, data_size);
		if (oob_size > 0)
			__raw_readsl(nand->mmio_base + NDDB,
				nand->oob_buff + nand->oob_column, oob_size);
	}

	nand->data_column += (data_size << 2);
	nand->oob_column += (oob_size << 2);
}

static void start_data_dma(struct pxa3xx_nand *nand, int dir_out)
{
	struct pxa_dma_desc *desc, *desc_oob;
	dma_addr_t data_desc_addr;
	unsigned int data_len = ALIGN(nand->data_size, 32);
	unsigned int oob_len = ALIGN(nand->oob_size, 32);

	/* ensure cache alignment */
	BUG_ON(nand->dma_buff_phys & 0x1f);
	desc = (struct pxa_dma_desc *)((void *)nand->dma_buff + MAX_BUFF_SIZE);
	data_desc_addr = (dma_addr_t)((void *)nand->dma_buff_phys + MAX_BUFF_SIZE);

	desc->ddadr = data_desc_addr + sizeof(struct pxa_dma_desc);
	desc_oob = desc + 1;
	desc_oob->ddadr = DDADR_STOP;
	desc_oob->dcmd = desc->dcmd = DCMD_WIDTH4 | DCMD_BURST32;
	if (!(nand->mode & POLLING_S)) {
		desc_oob->dcmd |= DCMD_ENDIRQEN;
		desc->dcmd |= DCMD_ENDIRQEN;
		disable_int(nand, NDCR_INT_MASK);
	}

	if (dir_out) {
		desc->dsadr = (volatile u32)nand->data_offset + nand->data_column;
		desc->dcmd |= DCMD_INCSRCADDR | DCMD_FLOWTRG | data_len;
		desc_oob->dsadr = nand->dma_buff_phys + MAX_DATA_SZ + nand->oob_column;
		desc_oob->dcmd |= DCMD_INCSRCADDR | DCMD_FLOWTRG | oob_len;
		desc_oob->dtadr = desc->dtadr = nand->mmio_phys + NDDB;
	} else {
		desc->dtadr = (volatile u32)nand->data_offset + nand->data_column;
		desc->dcmd |= DCMD_INCTRGADDR | DCMD_FLOWSRC | data_len;
		desc_oob->dtadr = nand->dma_buff_phys + MAX_DATA_SZ + nand->oob_column;
		desc_oob->dcmd |= DCMD_INCTRGADDR | DCMD_FLOWSRC | oob_len;
		desc_oob->dsadr = desc->dsadr = nand->mmio_phys + NDDB;
	}

	DBG_NAND(printk("DMA START:DMA dcmd %x, dsadr %x, dtadr %x, len %x %x\n",
				desc->dcmd, desc->dsadr, desc->dtadr, data_len, oob_len));
	DRCMR(nand->drcmr_dat) = DRCMR_MAPVLD | nand->data_dma_ch;
	DDADR(nand->data_dma_ch) = data_desc_addr;
	DCSR(nand->data_dma_ch) |= DCSR_RUN;
}

static inline void dma_complete(int channel, struct pxa3xx_nand *nand)
{
	uint32_t dcsr;

	dcsr = DCSR(channel);
	DCSR(channel) = dcsr;

	DBG_NAND(printk("DMA IRQ: dcsr %x\n", dcsr));
	if (dcsr & DCSR_BUSERR) {
		nand->retcode = ERR_DMABUSERR;
	}

	nand->data_column += nand->data_size;
	nand->oob_column += nand->oob_size;
}

static void pxa3xx_nand_data_dma_irq(int channel, void *data)
{
	struct pxa3xx_nand *nand = data;
	dma_complete(channel, nand);
	enable_int(nand, NDCR_INT_MASK);
	nand_writel(nand, NDSR, NDSR_WRDREQ | NDSR_RDDREQ);
}

static int pxa3xx_nand_transaction(struct pxa3xx_nand *nand)
{
	uint8_t cmd_seqs, cs;
	unsigned int status, is_completed = 0;
	unsigned int ready, cmd_done, page_done, badblock_detect;

	cs		= nand->chip_select;
	ready           = (cs) ? NDSR_RDY : NDSR_FLASH_RDY;
	cmd_done        = (cs) ? NDSR_CS1_CMDD : NDSR_CS0_CMDD;
	page_done       = (cs) ? NDSR_CS1_PAGED : NDSR_CS0_PAGED;
	badblock_detect = (cs) ? NDSR_CS1_BBD : NDSR_CS0_BBD;
	cmd_seqs	= nand->cmd_seqs;

	status = nand_readl(nand, NDSR);
	if (!status)
		return 0;
	DBG_NAND(printk("\t\t==cmd seqs %x, status %x, cs %x\n",
				cmd_seqs, status, cs));
	nand->bad_count = (status & NDSR_ERR_CNT_MASK) >> 16;
	if (status & badblock_detect) {
		nand->retcode = ERR_BBERR;
		is_completed = 1;
		goto IRQ_FORCE_EXIT;
	}
	if (status & NDSR_DBERR)
		nand->retcode = ERR_DBERR;
	if (status & NDSR_SBERR)
		nand->retcode = ERR_SBERR;
	if (status & (NDSR_RDDREQ | NDSR_WRDREQ)) {

		nand->state |= STATE_DATA_PROCESSING;
		/* whether use dma to transfer data */
		if (nand->mode & DMA_SUPPORT) {
			start_data_dma(nand, nand->state & STATE_IS_WRITE);
			if (nand->mode & POLLING_S) {
				while (!(DCSR(nand->data_dma_ch) & DCSR_STOPSTATE))
					;
				dma_complete(nand->data_dma_ch, nand);
			}
			else
				goto NORMAL_IRQ_EXIT;
		} else
			handle_data_pio(nand);

		nand->state |= STATE_DATA_DONE;
	}
	if (status & page_done)
		nand->state |= STATE_PAGE_DONE;
	if (status & ready) {
		nand->state |= STATE_READY;
		if (nand->wait_ready[cmd_seqs]) {
			if (!(nand->mode & POLLING_S))
				enable_int(nand, NDCR_WRCMDREQM | NDCR_CS0_CMDDM
						| NDCR_CS1_CMDDM);
			if (cmd_seqs == nand->total_cmds)
				is_completed = 1;
		}
	}
	if (status & cmd_done) {
		nand->state |= STATE_CMD_DONE;
		if (nand->wait_ready[cmd_seqs] && !(nand->state & STATE_READY)) {
			status &= ~cmd_done;
			if (!(nand->mode & POLLING_S))
				disable_int(nand, NDCR_CS0_CMDDM | NDCR_CS1_CMDDM);
		}
		if (cmd_seqs == nand->total_cmds && !nand->wait_ready[cmd_seqs])
			is_completed = 1;
	}

	if (status & NDSR_WRCMDREQ) {
		status &= ~NDSR_WRCMDREQ;
		if (nand->wait_ready[cmd_seqs] && !(nand->state & STATE_READY)) {
			if (!(nand->mode & POLLING_S))
				disable_int(nand, NDCR_WRCMDREQM);
			goto IRQ_FORCE_EXIT;
		}

		nand_writel(nand, NDSR, NDSR_WRCMDREQ);
		if (cmd_seqs < nand->total_cmds) {
			nand->cmd_seqs ++;
			nand->state &= ~STATE_MASK;
			nand->state |= STATE_CMD_WAIT_DONE;
			nand_writel(nand, NDCB0, nand->ndcb0[cmd_seqs]);
			nand_writel(nand, NDCB0, nand->ndcb1[cmd_seqs]);
			nand_writel(nand, NDCB0, nand->ndcb2[cmd_seqs]);
			if (nand->ndcb0[cmd_seqs] & NDCB0_LEN_OVRD) {
				nand_writel(nand, NDCB0, nand->data_size + nand->oob_size);
				DBG_NAND(printk("\tdata length 0x%x, ", nand->data_size + nand->oob_size));
			}
			else
				DBG_NAND(printk("\tdata length 0x0, "));
			DBG_NAND(printk("ndcb0 %x ndcb1 %x ndcb2 %x\n",
						nand->ndcb0[cmd_seqs],
						nand->ndcb1[cmd_seqs],
						nand->ndcb2[cmd_seqs]));
		}
		else
			is_completed = 1;
	}

IRQ_FORCE_EXIT:
	/* clear NDSR to let the controller exit the IRQ */
	nand_writel(nand, NDSR, status);
NORMAL_IRQ_EXIT:
	return is_completed;
}

static irqreturn_t pxa3xx_nand_irq(int irq, void *devid)
{
	struct pxa3xx_nand *nand = devid;
	int is_completed;

	is_completed = pxa3xx_nand_transaction(nand);
	if (is_completed)
		complete(&nand->cmd_complete);

	return IRQ_HANDLED;
}

static int pxa3xx_nand_polling(struct pxa3xx_nand *nand, unsigned long timeout)
{
	int i, ret = 0;

	for (i = 0; i < timeout; i++) {
		ret = pxa3xx_nand_transaction(nand);
		if (ret)
			break;
		udelay(1);
	}

	return ret;
}

static inline int is_buf_blank(const uint8_t *buf, size_t len)
{
	for (; len > 0; len--)
		if (*buf++ != 0xff)
			return 0;
	return 1;
}

static int prepare_command_pool(struct pxa3xx_nand *nand, int command,
		uint16_t column, int page_addr)
{
	uint16_t cmd;
	uint8_t	chunks = 0;
	int addr_cycle, exec_cmd, ndcb0, i;
	struct pxa3xx_nand_info *info = nand->info[nand->chip_select];
	struct mtd_info *mtd = get_mtd_by_info(info);
	struct nand_chip *chip = mtd->priv;

	ndcb0 = (nand->chip_select) ? NDCB0_CSEL : 0;;
	addr_cycle = 0;
	exec_cmd = 1;

	/* reset data and oob column point to handle data */
	nand->total_cmds	= 1;
	nand->use_ecc		= ECC_NONE;
	i = (uint32_t)(&nand->state) - (uint32_t)nand;
	memset(&nand->state, 0, sizeof(struct pxa3xx_nand) - i);

	switch (command) {
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_RNDOUT:
		nand->use_ecc = info->use_ecc;
		pxa3xx_set_datasize(info);
		chunks = info->page_size / nand->data_size;
		break;
	default:
		nand->mode &= ~DMA_SUPPORT;
		nand->buf_start = column;
		memset(nand->ndcb1, 0, CMD_POOL_SIZE * sizeof(uint32_t) * 2);
		break;
	}

	/* clear the command buffer */
	for (i = 0; i < CMD_POOL_SIZE; i ++)
		nand->ndcb0[i] = ndcb0;
	addr_cycle = NDCB0_ADDR_CYC(info->row_addr_cycles
			+ info->col_addr_cycles);

	switch (command) {
	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:
	case NAND_CMD_SEQIN:
		exec_cmd = 0;
		info->page_addr = page_addr;
		/* small page addr setting */
		if (unlikely(info->page_size < PAGE_CHUNK_SIZE))
			nand->ndcb1[0] = ((page_addr & 0xFFFFFF) << 8)
					| (column & 0xFF);
		else {
			nand->ndcb1[0] = ((page_addr & 0xFFFF) << 16)
					| (column & 0xFFFF);

			if (page_addr & 0xFF0000)
				nand->ndcb2[0] = (page_addr & 0xFF0000) >> 16;
		}

		for (i = 1; i <=chunks; i ++) {
			nand->ndcb1[i] = nand->ndcb1[0];
			nand->ndcb2[i] = nand->ndcb2[0];
		}
		nand->buf_count = mtd->writesize + mtd->oobsize;

		break;

	case NAND_CMD_RNDOUT:
		cmd  = cmdset.read1;
		nand->total_cmds = 3;
		nand->buf_start = column;
		nand->buf_count = nand->oob_size;
		nand->wait_ready[1] = 1;

		if (unlikely(info->page_size < PAGE_CHUNK_SIZE) || !(nand->mode & NAKEDCMD_S)) {
			if (unlikely(info->page_size < PAGE_CHUNK_SIZE))
				nand->ndcb0[0] |= NDCB0_CMD_TYPE(0)
						| addr_cycle
						| (cmd & NDCB0_CMD1_MASK);
			else
				nand->ndcb0[0] |= NDCB0_CMD_TYPE(0)
						| NDCB0_DBC
						| addr_cycle
						| cmd;

			if (nand->command == NAND_CMD_READOOB) {
				nand->use_ecc = ECC_NONE;
				nand->buf_start += mtd->writesize;
			}
			else
				nand->buf_count += mtd->writesize;
			break;
		}

		i = 0;
		nand->ndcb0[i ++] |= NDCB0_CMD_XTYPE(0x6)
					| NDCB0_CMD_TYPE(0)
					| NDCB0_DBC
					| NDCB0_NC
					| addr_cycle
					| cmd;
		/* we should wait RnB go high again
		 * before read out data*/
		nand->wait_ready[1] = 1;
		nand->buf_count += mtd->writesize;
		ndcb0 = nand->ndcb0[i]
			| NDCB0_CMD_XTYPE(0x5)
			| NDCB0_NC
			| addr_cycle;
		nand->total_cmds = chunks + i;
		for (; i <= nand->total_cmds; i ++)
			nand->ndcb0[i] = ndcb0;

		nand->ndcb0[nand->total_cmds - 1] &= ~NDCB0_NC;
		break;

	case NAND_CMD_PAGEPROG:
		if (nand->command == NAND_CMD_NONE) {
			exec_cmd = 0;
			break;
		}

		cmd = cmdset.program;
		nand->state |= STATE_IS_WRITE;
		if (unlikely(info->page_size < PAGE_CHUNK_SIZE) || !(nand->mode & NAKEDCMD_S))
			nand->ndcb0[0] |= NDCB0_CMD_TYPE(0x1)
					| NDCB0_AUTO_RS
					| NDCB0_ST_ROW_EN
					| NDCB0_DBC
					| cmd
					| addr_cycle;
		else {
			nand->total_cmds = chunks + 1;
			nand->ndcb0[0] |= NDCB0_CMD_XTYPE(0x4)
					| NDCB0_CMD_TYPE(0x1)
					| NDCB0_NC
					| NDCB0_AUTO_RS
					| (cmd & NDCB0_CMD1_MASK)
					| addr_cycle;

			for (i = 1; i < chunks; i ++)
				nand->ndcb0[i] |= NDCB0_CMD_XTYPE(0x5)
						| NDCB0_NC
						| NDCB0_AUTO_RS
						| NDCB0_CMD_TYPE(0x1)
						| addr_cycle;

			nand->ndcb0[chunks] |= NDCB0_CMD_XTYPE(0x3)
						| NDCB0_CMD_TYPE(0x1)
						| NDCB0_ST_ROW_EN
						| NDCB0_DBC
						| (cmd & NDCB0_CMD2_MASK)
						| NDCB0_CMD1_MASK
						| addr_cycle;
			/* we should wait for RnB goes high which
			 * indicate the data has been written succesfully*/
			nand->wait_ready[nand->total_cmds] = 1;
		}
		break;

	case NAND_CMD_READID:
		cmd = cmdset.read_id;
		nand->buf_count = info->read_id_bytes;
		nand->ndcb0[0] |= NDCB0_CMD_TYPE(3)
				| NDCB0_ADDR_CYC(1)
				| cmd;

		nand->data_size = 8;
		break;

	case NAND_CMD_STATUS:
		cmd = cmdset.read_status;
		nand->buf_count = 1;
		nand->data_buff = chip->buffers->databuf;
		nand->ndcb0[0] |= NDCB0_CMD_TYPE(4)
				| NDCB0_ADDR_CYC(1)
				| cmd;

		nand->data_size = 8;
		break;

	case NAND_CMD_ERASE1:
		cmd = cmdset.erase;
		nand->ndcb0[0] |= NDCB0_CMD_TYPE(2)
				| NDCB0_AUTO_RS
				| NDCB0_ADDR_CYC(3)
				| NDCB0_DBC
				| cmd;
		nand->ndcb1[0] = page_addr;

		break;
	case NAND_CMD_RESET:
		/* on some platform, it is stranger that when issue reset command,
		 * cmd done would not come till timeout cause irq exit.
		 * Force polling mode for reset command*/
		nand->mode |= POLLING_S;
		cmd = cmdset.reset;
		nand->ndcb0[0] |= NDCB0_CMD_TYPE(5)
				| cmd;

		break;

	case NAND_CMD_ERASE2:
		exec_cmd = 0;
		break;

	default:
		exec_cmd = 0;
		printk(KERN_ERR "pxa3xx-nand: non-supported command %x\n", command);
		break;
	}

	nand->command = command;
	return exec_cmd;
}

static void pxa3xx_nand_cmdfunc(struct mtd_info *mtd, unsigned command,
				int column, int page_addr)
{
	struct pxa3xx_nand_info *info= mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int ret, exec_cmd;
#ifdef CONFIG_PXA3XX_BBM
	struct pxa3xx_bbm *pxa3xx_bbm = mtd->bbm;
	loff_t addr;

	DBG_NAND(printk("pre relocated addr is %x\n", page_addr););
	if (pxa3xx_bbm && (command == NAND_CMD_READOOB
			|| command == NAND_CMD_READ0
			|| command == NAND_CMD_SEQIN
			|| command == NAND_CMD_ERASE1)) {

		addr = (loff_t)page_addr << mtd->writesize_shift;
		addr = pxa3xx_bbm->search(mtd, addr);
		page_addr = addr >> mtd->writesize_shift;
	}
#endif

	set_dvfm_constraint();

	/* if this is a x16 device ,then convert the input
	 * "byte" address into a "word" address appropriate
	 * for indexing a word-oriented device
	 */
	if (info->reg_ndcr & NDCR_DWIDTH_M)
		column /= 2;

	/* reset timing */
	if (nand->chip_select != info->chip_select) {
		nand->chip_select = info->chip_select;
		nand_writel(nand, NDTR0CS0, info->ndtr0cs0);
		nand_writel(nand, NDTR1CS0, info->ndtr1cs0);
	}

	DBG_NAND(printk("command %x, page %x\n", command, page_addr));
	exec_cmd = prepare_command_pool(nand, command, column, page_addr);
	if (exec_cmd) {
		init_completion(&nand->cmd_complete);
		nand->state |= STATE_CMD_PREPARED;
		pxa3xx_nand_start(nand);

		if (!(nand->mode & POLLING_S))
			ret = wait_for_completion_timeout(&nand->cmd_complete,
					CHIP_DELAY_TIMEOUT);
		else
			ret = pxa3xx_nand_polling(nand, CHIP_DELAY_TIMEOUT * 50);
		if (!ret) {
			printk(KERN_ERR "Wait time out!!!\n");
			nand_error_dump(nand);
		}
		/* Stop State Machine for next command cycle */
		pxa3xx_nand_stop(nand);
		disable_int(nand, NDCR_INT_MASK);
		nand->state &= ~STATE_CMD_PREPARED;
	}
	unset_dvfm_constraint();
}

static uint8_t pxa3xx_nand_read_byte(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	char retval = 0xFF;

	if (nand->buf_start < nand->buf_count)
		/* Has just send a new command? */
		retval = nand->data_buff[nand->buf_start++];

	return retval;
}

static u16 pxa3xx_nand_read_word(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	u16 retval = 0xFFFF;

	if (!(nand->buf_start & 0x01) && nand->buf_start < nand->buf_count) {
		retval = *((u16 *)(nand->data_buff+nand->buf_start));
		nand->buf_start += 2;
	}
	return retval;
}

static void pxa3xx_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int real_len = min_t(size_t, len, nand->buf_count - nand->buf_start);

	memcpy(buf, nand->data_buff + nand->buf_start, real_len);
	nand->buf_start += real_len;
}

static void pxa3xx_nand_write_buf(struct mtd_info *mtd,
		const uint8_t *buf, int len)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int real_len = min_t(size_t, len, nand->buf_count - nand->buf_start);

	memcpy(nand->data_buff + nand->buf_start, buf, real_len);
	nand->buf_start += real_len;
}

static int pxa3xx_nand_verify_buf(struct mtd_info *mtd,
		const uint8_t *buf, int len)
{
	return 0;
}

static void pxa3xx_nand_select_chip(struct mtd_info *mtd, int chip)
{
	return;
}

static int pxa3xx_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand= info->nand_data;

	if ((nand->command == NAND_CMD_PAGEPROG)
	    && (nand->mode & DMA_SUPPORT)) {
		unmap_addr(&nand->pdev->dev, nand->data_offset,
			   nand->data_buff, mtd->writesize, DMA_TO_DEVICE);
	}

	/* pxa3xx_nand_send_command has waited for command complete */
	if (this->state == FL_WRITING || this->state == FL_ERASING) {
		if (nand->retcode == ERR_NONE)
			return 0;
		else {
			/*
			 * any error make it return 0x01 which will tell
			 * the caller the erase and write fail
			 */
			return 0x01;
		}
	}

	return 0;
}

static void pxa3xx_nand_config_flash(struct pxa3xx_nand_info *info,
			    struct pxa3xx_nand_flash *f, int show_timing)
{
	struct pxa3xx_nand *nand = info->nand_data;
	struct platform_device *pdev = nand->pdev;
	uint32_t ndcr = 0;

	info->page_size = f->page_size;
	if ((info->page_size > PAGE_CHUNK_SIZE)
		       && !(nand->mode & NAKEDCMD_S)) {
		dev_err(&pdev->dev, "Your controller don't support 4k or"
				"larger page NAND for don't support naked command\n");
		BUG();
	}

	/* calculate flash information */
	info->use_ecc = f->ecc_type;
	//info->read_id_bytes = (f->page_size >= 2048) ? 4 : 2;
	info->read_id_bytes = 4;

	/* calculate addressing information */
	info->col_addr_cycles = (f->page_size >= 2048) ? 2 : 1;

	if (f->num_blocks * f->page_per_block > 65536)
		info->row_addr_cycles = 3;
	else
		info->row_addr_cycles = 2;

	ndcr |= (info->col_addr_cycles == 2) ? NDCR_RA_START : 0;
	ndcr |= (f->flash_width == 16) ? NDCR_DWIDTH_M : 0;
	ndcr |= (f->dfc_width == 16) ? NDCR_DWIDTH_C : 0;

	switch (f->page_per_block) {
	case 32:
		ndcr |= NDCR_PG_PER_BLK(0x0);
		break;
	case 128:
		ndcr |= NDCR_PG_PER_BLK(0x1);
		break;
	case 256:
		ndcr |= NDCR_PG_PER_BLK(0x3);
		break;
	case 64:
	default:
		ndcr |= NDCR_PG_PER_BLK(0x2);
		break;
	}

	switch (f->page_size) {
	case 512:
		ndcr |= NDCR_PAGE_SZ(0x0);
		break;
	case 2048:
	default:
		ndcr |= NDCR_PAGE_SZ(0x1);
		ndcr |= NDCR_FORCE_CSX;
		break;
	}

	ndcr |= NDCR_RD_ID_CNT(info->read_id_bytes);
	ndcr |= NDCR_SPARE_EN; /* enable spare by default */

	info->reg_ndcr = ndcr;

	pxa3xx_nand_set_timing(info, &f->timing, show_timing);
}

static struct nand_ecclayout hw_smallpage_ecclayout = {
	.eccbytes = 6,
	.eccpos = {8, 9, 10, 11, 12, 13 },
	.oobfree = { {2, 6} }
};

static struct nand_ecclayout nand_oob_64 = {
	.eccbytes = 24,
	.eccpos = {
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = { {2, 38} }
};

static struct nand_ecclayout bch_nand_oob_64 = {
	.eccbytes = 32,
	.eccpos = {
		32, 33, 34, 35, 36, 37, 38, 39,
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = { {2, 30} }
};

static struct nand_ecclayout nand_oob_128 = {
	.eccbytes = 48,
	.eccpos = {
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 94, 95,
		96, 97, 98, 99, 100, 101, 102, 103,
		104, 105, 106, 107, 108, 109, 110, 111,
		112, 113, 114, 115, 116, 117, 118, 119,
		120, 121, 122, 123, 124, 125, 126, 127},
	.oobfree = { {2, 78} }
};

static struct nand_ecclayout bch_nand_oob_128 = {
	.eccbytes = 64,
	.eccpos = {
		64, 65, 66, 67, 68, 69, 70, 71,
		72, 73, 74, 75, 76, 77, 78, 79,
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 94, 95,
		96, 97, 98, 99, 100, 101, 102, 103,
		104, 105, 106, 107, 108, 109, 110, 111,
		112, 113, 114, 115, 116, 117, 118, 119,
		120, 121, 122, 123, 124, 125, 126, 127},
	.oobfree = { {2, 62} }
};

static void free_cs_resource(struct pxa3xx_nand_info *info, uint8_t cs)
{
	struct pxa3xx_nand *nand;
	struct mtd_info *mtd;

	if (!info)
		return;

	nand = info->nand_data;
	nand->info[cs] = NULL;
	mtd = get_mtd_by_info(info);
	kfree(mtd);
}

static int pxa3xx_read_page(struct mtd_info *mtd, uint8_t *buf)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct nand_chip *chip = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	dma_addr_t mapped_addr = 0;
	int page_addr, buf_blank;

	page_addr = info->page_addr;
	nand->data_buff = (buf) ? buf : nand->dma_buff;
	nand->oob_buff = chip->oob_poi;
	nand->mode &= ~DMA_SUPPORT;
	nand->mode |= pxa3xx_nand_mode & DMA_SUPPORT;
	nand->data_offset = 0;
	if ((nand->mode & DMA_SUPPORT) && buf) {
		mapped_addr = map_addr(nand, (void *)buf,
				mtd->writesize, DMA_FROM_DEVICE);
		if (dma_mapping_error(&nand->pdev->dev, mapped_addr))
			nand->mode &= ~DMA_SUPPORT;
		else
			nand->data_offset = mapped_addr;
	}

	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_RNDOUT, 0, page_addr);
	if (nand->data_offset)
		unmap_addr(&nand->pdev->dev, nand->data_offset,
			   buf, mtd->writesize, DMA_FROM_DEVICE);
	switch (nand->retcode) {
	case ERR_SBERR:
		DBG_NAND(printk("###correctable error detected\n"););
		switch (nand->use_ecc) {
		case ECC_BCH:
			if (nand->bad_count > BCH_THRESHOLD)
				mtd->ecc_stats.corrected +=
					(nand->bad_count - BCH_THRESHOLD);
			break;

		case ECC_HAMMIN:
			mtd->ecc_stats.corrected ++;
			break;

		case ECC_NONE:
		default:
			break;
		}
		break;
	case ERR_DBERR:
		buf_blank = is_buf_blank(nand->data_buff, mtd->writesize);
		if (!buf_blank) {
			DBG_NAND(printk("###uncorrectable error!!!\n"));
			mtd->ecc_stats.failed++;
			if(nand_dump_times<2)
			{
				nand_dump_times++;
				nand_error_dump(nand);
			}	
		}
		break;
	case ERR_NONE:
		break;
	default:
		mtd->ecc_stats.failed++;
		break;
	}

	return 0;
}

static int pxa3xx_nand_read_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, uint8_t *buf, int page)
{
	pxa3xx_read_page(mtd, buf);

	return 0;
}

static int pxa3xx_nand_read_oob(struct mtd_info *mtd, struct nand_chip *chip,
		int page, int sndcmd)
{
	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_READOOB, 0, page);
	pxa3xx_read_page(mtd, NULL);
	return 0;
}

static void pxa3xx_nand_write_page_hwecc(struct mtd_info *mtd,
		struct nand_chip *chip, const uint8_t *buf)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;

	nand->mode &= ~DMA_SUPPORT;
	nand->mode |= pxa3xx_nand_mode & DMA_SUPPORT;
	nand->data_buff = (uint8_t *)buf;
	nand->oob_buff = chip->oob_poi;
	if (is_buf_blank(buf, mtd->writesize) &&
	    is_buf_blank(nand->oob_buff, nand->oob_size)) {
		nand->command = NAND_CMD_NONE;
		return;
	}

	if (nand->mode & DMA_SUPPORT) {
		nand->data_offset = map_addr(nand, (void *)buf,
				      mtd->writesize, DMA_TO_DEVICE);
		if (dma_mapping_error(&nand->pdev->dev, nand->data_offset))
			nand->mode &= ~DMA_SUPPORT;
	}
}

static int pxa3xx_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip,
			      int page)
{
	struct pxa3xx_nand_info *info= mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int status = 0;
	nand->mode = pxa3xx_nand_mode;
	/* Send command to program the OOB data */
	chip->cmdfunc(mtd, NAND_CMD_SEQIN, 0, page);
	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);

	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

static void pxa3xx_nand_erase_cmd(struct mtd_info *mtd, int page)
{
	/* Send commands to erase a block */
	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_ERASE1, -1, page);
}

static int pxa3xx_nand_sensing(struct pxa3xx_nand *nand)
{
	struct pxa3xx_nand_info *info = nand->info[nand->chip_select];
	struct mtd_info *mtd = get_mtd_by_info(info);
	/* use the common timing to make a try */
	pxa3xx_nand_config_flash(info, &builtin_flash_types[0], 0);
	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_RESET, 0, 0);
	if (nand->state & STATE_READY)
		return 1;
	else
		return 0;
}
static ssize_t show_flashid(struct device *dev, \
				struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "%d\n", flash_type);
}
static int __devinit pxa3xx_nand_scan_ident(struct mtd_info *mtd, int maxchips)
{
	struct pxa3xx_nand_flash *f;
	struct pxa3xx_nand_info *info = mtd->priv;
	struct nand_chip *chip = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	uint32_t id = -1;
	int i, ret;

	nand->chip_select = info->chip_select;
	ret = pxa3xx_nand_sensing(nand);
	if (!ret) {
		printk(KERN_INFO "There is no nand chip on cs %d!\n", info->chip_select);
		free_cs_resource(info, nand->chip_select);

		return -EINVAL;
	}

	nand->data_buff = chip->buffers->databuf;
	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_READID, 0, 0);
	
	id = *((uint32_t *)(nand->data_buff));
	flash_type = id;
	if (id != 0)
		printk(KERN_INFO "Detect a flash id %x on cs %d\n", id, nand->chip_select);
	else {
		printk(KERN_WARNING "Read out ID 0, potential timing set wrong!!\n");
		free_cs_resource(info, nand->chip_select);

		return -EINVAL;
	}
	id = *((uint16_t *)(nand->data_buff));

	for (i = 1; i < ARRAY_SIZE(builtin_flash_types); i++) {

		f = &builtin_flash_types[i];

		/* find the chip in default list */
		if (f->chip_id == id) {
			printk(KERN_WARNING "chip_id=%x,ecc_type=%x\n",f->chip_id, f->ecc_type);		
			//flash_type = f->chip_id;
			pxa3xx_nand_config_flash(info, f, 1);
			chip->cellinfo = nand->data_buff[2];
			mtd->writesize = f->page_size;
			mtd->writesize_shift = ffs(mtd->writesize) - 1;
			mtd->writesize_mask = (1 << mtd->writesize_shift) - 1;
			mtd->oobsize = mtd->writesize / 32;
			mtd->erasesize = f->page_size * f->page_per_block;
			mtd->erasesize_shift = ffs(mtd->erasesize) - 1;
			mtd->erasesize_mask = (1 << mtd->erasesize_shift) - 1;

			mtd->name = mtd_names[nand->chip_select];
			break;
		}
	}

	if (i == ARRAY_SIZE(builtin_flash_types)) {
		printk(KERN_ERR "ERROR!! flash not defined!!!\n");
		free_cs_resource(info, nand->chip_select);

		return -EINVAL;
	}

	chip->ecc.mode		= NAND_ECC_HW;
	chip->ecc.size		= f->page_size;
	switch (f->page_size) {
	case 512:
		chip->ecc.layout = &hw_smallpage_ecclayout;
		break;
	case 2048:
		chip->ecc.layout = (f->ecc_type == ECC_BCH) ?
				   &bch_nand_oob_64 : &nand_oob_64;
		break;
	case 4096:
		chip->ecc.layout = (f->ecc_type == ECC_BCH) ?
				   &bch_nand_oob_128 : &nand_oob_128;
		break;
	default:
		BUG();
	}

	chip->chipsize 		= (uint64_t)f->num_blocks 	* \
				  f->page_per_block 		* \
				  f->page_size;

	chip->chip_shift 	= ffs(chip->chipsize) - 1;
	mtd->size 		= chip->chipsize;

	/* Calculate the address shift from the page size */
	chip->page_shift = ffs(mtd->writesize) - 1;
	chip->pagemask = mtd_div_by_ws(chip->chipsize, mtd) - 1;
	chip->numchips		= 1;
	chip->bbt_erase_shift = chip->phys_erase_shift = ffs(mtd->erasesize) - 1;

	/* Set the bad block position */
	chip->badblockpos = mtd->writesize > 512 ?
		NAND_LARGE_BADBLOCK_POS : NAND_SMALL_BADBLOCK_POS;

	chip->options = (f->flash_width == 16) ? NAND_BUSWIDTH_16: 0;
	chip->options |= NAND_NO_AUTOINCR | NAND_NO_READRDY | NAND_USE_FLASH_BBT;
#ifdef CONFIG_PXA3XX_BBM
	chip->options |= BBT_RELOCATION_IFBAD;
#endif

	return 0;
}

static int __devinit alloc_nand_resource(struct platform_device *pdev)
{
	struct pxa3xx_nand_info *info;
	struct pxa3xx_nand *nand;
	struct nand_chip *chip;
	struct mtd_info *mtd;
	struct resource *r;
	int ret, irq;
	uint8_t cs;

	nand = kzalloc(sizeof(struct pxa3xx_nand), GFP_KERNEL);
	if (!nand) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	nand->pdev = pdev;
	nand->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(nand->clk)) {
		dev_err(&pdev->dev, "failed to get nand clock\n");
		ret = PTR_ERR(nand->clk);
		ret = -ENXIO;
		goto fail_alloc;
	}
	clk_enable(nand->clk);

	r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no resource defined for data DMA\n");
		ret = -ENXIO;
		goto fail_put_clk;
	}
	nand->drcmr_dat = r->start;

	r = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (r == NULL) {
		dev_err(&pdev->dev, "no resource defined for command DMA\n");
		ret = -ENXIO;
		goto fail_put_clk;
	}
	nand->drcmr_cmd = r->start;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		ret = -ENODEV;
		goto fail_put_clk;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		ret = -EBUSY;
		goto fail_put_clk;
	}

	nand->mmio_base = ioremap(r->start, resource_size(r));
	if (nand->mmio_base == NULL) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto fail_free_res;
	}
	nand->mmio_phys = r->start;

	/* initialize all interrupts to be disabled */
	disable_int(nand, NDCR_INT_MASK);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		ret = -ENXIO;
		goto fail_put_clk;
	}

	ret = request_irq(irq, pxa3xx_nand_irq, IRQF_DISABLED,
			  pdev->name, nand);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		ret = -ENXIO;
		goto fail_free_irq;
	}

	platform_set_drvdata(pdev, nand);
	for (cs = 0; cs < NUM_CHIP_SELECT; cs ++) {
		mtd = kzalloc(sizeof(struct mtd_info) + sizeof(struct pxa3xx_nand_info)
			      + sizeof(*chip->buffers),	GFP_KERNEL);
		if (!mtd) {
			dev_err(&pdev->dev, "failed to allocate memory\n");
			ret = -ENOMEM;
			goto fail_free_irq;
		}

		info = (struct pxa3xx_nand_info *)(&mtd[1]);
		info->nand_data = nand;
		info->chip_select = cs;
		mtd->priv = info;
		mtd->owner = THIS_MODULE;
		nand->info[cs] = info;

		chip = (struct nand_chip *)(&mtd[1]);
		chip->controller        = &nand->controller;
		chip->buffers		= (struct nand_buffers *)(&info[1]);
		chip->ecc.read_page	= pxa3xx_nand_read_page_hwecc;
		chip->ecc.write_page	= pxa3xx_nand_write_page_hwecc;
		chip->ecc.read_oob      = pxa3xx_nand_read_oob;
		chip->ecc.write_oob	= pxa3xx_nand_write_oob;
		chip->waitfunc		= pxa3xx_nand_waitfunc;
		chip->select_chip	= pxa3xx_nand_select_chip;
		chip->cmdfunc		= pxa3xx_nand_cmdfunc;
		chip->read_word		= pxa3xx_nand_read_word;
		chip->read_byte		= pxa3xx_nand_read_byte;
		chip->read_buf		= pxa3xx_nand_read_buf;
		chip->write_buf		= pxa3xx_nand_write_buf;
		chip->verify_buf	= pxa3xx_nand_verify_buf;
		chip->scan_ident	= pxa3xx_nand_scan_ident;
		chip->erase_cmd		= pxa3xx_nand_erase_cmd;
#ifdef CONFIG_PXA3XX_BBM
		chip->scan_bbt		= pxa3xx_scan_bbt;
		chip->block_markbad	= pxa3xx_block_markbad;
		chip->block_bad		= pxa3xx_block_bad;
#endif
	}

	nand->dma_buff = dma_alloc_coherent(&pdev->dev, MAX_BUFF_SIZE + DMA_H_SIZE,
		   &nand->dma_buff_phys, GFP_KERNEL);
	if (nand->dma_buff== NULL) {
		dev_err(&pdev->dev, "failed to allocate dma buffer\n");
		ret = -ENOMEM;
		goto fail_free_buf;
	}

	nand->data_dma_ch = pxa_request_dma("nand-data", DMA_PRIO_LOW,
			pxa3xx_nand_data_dma_irq, nand);
	if (nand->data_dma_ch < 0) {
		dev_err(&pdev->dev, "failed to request data dma\n");
		ret = -ENXIO;
		goto fail_free_buf;
	}
	DALGN |= (1 << nand->data_dma_ch);

	spin_lock_init(&nand->controller.lock);
	init_waitqueue_head(&nand->controller.wq);
	return 0;

fail_free_buf:
	for (cs = 0; cs < NUM_CHIP_SELECT; cs ++) {
		info = nand->info[cs];
		free_cs_resource(info, cs);
	}
fail_free_irq:
	free_irq(irq, nand);
	iounmap(nand->mmio_base);
fail_free_res:
	release_mem_region(r->start, resource_size(r));
fail_put_clk:
	clk_disable(nand->clk);
	clk_put(nand->clk);
fail_alloc:
	kfree(nand);
	return ret;
}
static DEVICE_ATTR(flashid, 0644, show_flashid, NULL);
static struct attribute *flashid_attributes[] = {
	&dev_attr_flashid.attr,
	NULL,
};
static struct attribute_group flashid_attr_group = {
	.attrs = flashid_attributes,
};
static int pxa3xx_nand_remove(struct platform_device *pdev)
{
	struct pxa3xx_nand *nand= platform_get_drvdata(pdev);
	struct pxa3xx_nand_info *info;
	struct mtd_info *mtd;
	struct resource *r;
	int irq, cs;
#ifdef CONFIG_PXA3XX_BBM
	struct pxa3xx_bbm *pxa3xx_bbm;
#endif
	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &flashid_attr_group);

	irq = platform_get_irq(pdev, 0);
	if (irq >= 0)
		free_irq(irq, nand);

	iounmap(nand->mmio_base);
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));
	clk_disable(nand->clk);
	clk_put(nand->clk);

	for (cs = 0; cs < NUM_CHIP_SELECT; cs ++) {
		info = nand->info[cs];
		if (!info)
			continue;
		mtd = get_mtd_by_info(info);
#ifdef CONFIG_PXA3XX_BBM
		pxa3xx_bbm = mtd->bbm;
		pxa3xx_bbm->uninit(mtd);
#endif
#ifdef CONFIG_MTD_PARTITIONS
		del_mtd_partitions(mtd);
		del_mtd_device(mtd);
#endif
		free_cs_resource(info, cs);
	}
	if (nand->dma_buff_phys) {
		if (nand->data_dma_ch >= 0)
			pxa_free_dma(nand->data_dma_ch);
		dma_free_coherent(&nand->pdev->dev, MAX_BUFF_SIZE + DMA_H_SIZE,
				  nand->dma_buff, nand->dma_buff_phys);
		nand->dma_buff_phys = 0;
	}
	return 0;
}



static int __devinit pxa3xx_nand_probe(struct platform_device *pdev)
{
	struct pxa3xx_nand_platform_data *pdata;
	struct pxa3xx_nand_info *info;
	struct pxa3xx_nand *nand;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	int ret, cs, probe_success = 0,retval;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partitions = NULL, *parts = NULL;
	int num_part = 0;
#endif

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -ENODEV;
	}

	pxa3xx_nand_mode = pdata->pxa3xx_nand_mode;
	ret = alloc_nand_resource(pdev);
	if (ret)
		return ret;

	nand = platform_get_drvdata(pdev);
	nand->RD_CNT_DEL = pdata->RD_CNT_DEL;
	nand->mode = pxa3xx_nand_mode;
	for (cs = 0; cs < NUM_CHIP_SELECT; cs ++) {
		info = nand->info[cs];
		mtd = get_mtd_by_info(info);
		chip = mtd->priv;
		if (nand_scan(mtd, 1)) {
			dev_err(&pdev->dev, "failed to scan nand on cs %d\n", cs);
			continue;
		}
		chip->oob_poi = (uint8_t *)nand->dma_buff + MAX_DATA_SZ;
		mtd->name = mtd_names[cs];
#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
		num_part = parse_mtd_partitions(mtd, part_probes, &partitions, 0);
#endif
		if (num_part <= 0) {
			num_part = pdata->nr_parts[cs];
			partitions = pdata->parts[cs];
		}

		if (partitions && num_part > 0) {
#ifdef CONFIG_PXA3XX_BBM
			struct pxa3xx_bbm *pxa3xx_bbm = mtd->bbm;
			parts = pxa3xx_bbm->check_partition(mtd, partitions, &num_part);
			if (!parts)
				return -EINVAL;
#else
			parts = partitions;
#endif
			ret = add_mtd_partitions(mtd, parts, num_part);
#ifdef CONFIG_PXA3XX_BBM
			kfree(parts);
#endif
			/* To ensure we don't free the part table that don't come from parser */
			if (partitions != pdata->parts[cs])
				kfree(partitions);
		}
		else
			ret = add_mtd_device(mtd);
#else
		ret = add_mtd_device(mtd);
#endif
		if (!ret)
			probe_success = 1;
	}
	retval = sysfs_create_group(&pdev->dev.kobj, &flashid_attr_group);

	if (!probe_success)
		pxa3xx_nand_remove(pdev);
	

	return ret;
}

#ifdef CONFIG_PM
static int pxa3xx_nand_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct pxa3xx_nand *nand= platform_get_drvdata(pdev);
	struct pxa3xx_nand_info *info;
	struct mtd_info *mtd;
	int ret = 0;
	uint8_t cs;

	if (nand->state & STATE_CMD_PREPARED) {
		dev_err(&pdev->dev, "driver busy, state = %d\n", nand->state);
		return -EAGAIN;
	}

	for (cs = 0; cs < NUM_CHIP_SELECT; cs ++) {
		info = nand->info[cs];
		if (!info)
			continue;
		mtd = get_mtd_by_info(info);
		ret = mtd->suspend(mtd);
	}

	clk_disable(nand->clk);
	return ret;
}

static int pxa3xx_nand_resume(struct platform_device *pdev)
{
	struct pxa3xx_nand *nand= platform_get_drvdata(pdev);
	struct pxa3xx_nand_info *info;
	struct mtd_info *mtd;
	uint8_t cs;

	clk_enable(nand->clk);
	for (cs = 0; cs < NUM_CHIP_SELECT; cs ++) {
		info = nand->info[cs];
		if (!info)
			continue;
		nand_writel(nand, NDTR0CS0, info->ndtr0cs0);
		nand_writel(nand, NDTR1CS0, info->ndtr1cs0);
		nand->chip_select = cs;
		/* Sometimes nand chip would raise a ready interrupt
		 * when resume, reset the by start and stop to prevent
		 * it damage driver's state machine */
		pxa3xx_nand_start(nand);
		pxa3xx_nand_stop(nand);
		mtd = get_mtd_by_info(info);
		mtd->resume(mtd);
	}

	/* set the controller cs to a invalid num to let driver
	 * reconfigure the timing when it call the cmdfunc */
	nand->chip_select = 0xff;
	return 0;
}
#else
#define pxa3xx_nand_suspend	NULL
#define pxa3xx_nand_resume	NULL
#endif

static struct platform_driver pxa3xx_nand_driver = {
	.driver = {
		.name	= "pxa3xx-nand",
	},
	.probe		= pxa3xx_nand_probe,
	.remove		= pxa3xx_nand_remove,
	.suspend	= pxa3xx_nand_suspend,
	.resume		= pxa3xx_nand_resume,
};

static int __init pxa3xx_nand_init(void)
{
#if defined(CONFIG_DVFM)
	dvfm_register("NAND", &dvfm_dev_idx);
#endif
	return platform_driver_register(&pxa3xx_nand_driver);
}
module_init(pxa3xx_nand_init);

static void __exit pxa3xx_nand_exit(void)
{
#if defined(CONFIG_DVFM)
	dvfm_unregister("NAND", &dvfm_dev_idx);
#endif
	platform_driver_unregister(&pxa3xx_nand_driver);
}
module_exit(pxa3xx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PXA3xx NAND controller driver");
