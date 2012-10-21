#ifndef __ASM_MACH_CPUTYPE_H
#define __ASM_MACH_CPUTYPE_H

#include <asm/io.h>
#include <asm/cputype.h>
#include <mach/addr-map.h>

#define CHIP_ID		(AXI_VIRT_BASE + 0x82c00)
#define BLOCK7_RESEVED_2	(AXI_VIRT_BASE + 0x1498)
#define FUSE_ID		BLOCK7_RESEVED_2
/*
 *  CPU   Stepping   OLD_ID       CPU_ID      CHIP_ID
 *
 * PXA168    A0    0x41159263   0x56158400   0x00A0A333
 * PXA910    Y0    0x41159262   0x56158000   0x00F0C910
 * PXA920    Y0                 0x56158400   0x00F2C920
 * PXA920    A0                 0x56158400   0x00A0C920
 */

#ifndef CONFIG_CPU_MOHAWK_OLD_ID

#ifdef CONFIG_CPU_PXA168
#  define __cpu_is_pxa168(id, cid)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; \
	 unsigned int _cid = (cid) & 0xfff; \
	 _id == 0x84 && _cid != 0x910 && _cid != 0x920; })
#else
#  define __cpu_is_pxa168(id, cid)	(0)
#endif

#ifdef CONFIG_CPU_PXA910
#  define __cpu_is_pxa910(id, cid)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; \
	 unsigned int _cid = (cid) & 0xfff; \
	 (_id == 0x84 || _id == 0x80) && (_cid == 0x910 || _cid == 0x920); })
#else
#  define __cpu_is_pxa910(id, cid)	(0)
#endif

#else

#ifdef CONFIG_CPU_PXA168
#  define __cpu_is_pxa168(id, cid)	\
	({ unsigned int _id = (id) & 0xffff; _id == 0x9263; })
#else
#  define __cpu_is_pxa168(id, cid)	(0)
#endif

#ifdef CONFIG_CPU_PXA910
#  define __cpu_is_pxa910(id, cid)	\
	({ unsigned int _id = (id) & 0xffff; _id == 0x9262; })
#else
#  define __cpu_is_pxa910(id, cid)	(0)
#endif

#endif /* CONFIG_CPU_MOHAWK_OLD_ID */

#define cpu_is_pxa168()		({ __cpu_is_pxa168(read_cpuid_id(), __raw_readl(CHIP_ID)); })
#define cpu_is_pxa910()		({ __cpu_is_pxa910(read_cpuid_id(), __raw_readl(CHIP_ID)); })

static inline int cpu_is_pxa910_Ax(void)
{
	unsigned int revision = (__raw_readl(CHIP_ID) >> 16) & 0xff;

	if (cpu_is_pxa910()) {
		if ((revision >= 0xa0) && (revision < 0xb0))
			return 1;
	}
	return 0;
}

static inline int cpu_is_pxa168_S0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa168() && ((chip_id & 0x0000ffff) == 0x0000c910))
		return 1;
	else
	return 0;
}

static inline int cpu_is_pxa168_A0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa168() && ((chip_id & 0x0000ffff) == 0x0000a168))
		return 1;
	else
		return 0;
}

static inline int cpu_is_pxa918(void)
{
	unsigned int fuse_id = __raw_readl(FUSE_ID);

	if (cpu_is_pxa910() && ((fuse_id & 0x3000000) == 0x3000000))
		return 1;
	return 0;
}

static inline int cpu_is_pxa910_c910(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);

	if (cpu_is_pxa910() && ((chip_id & 0xffff) == 0xc910))
		return 1;
		return 0;
}

static inline int cpu_is_pxa910_c920(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);

	if (cpu_is_pxa910() && ((chip_id & 0xffff) == 0xc920))
		return 1;
	return 0;
}
#endif /* __ASM_MACH_CPUTYPE_H */
