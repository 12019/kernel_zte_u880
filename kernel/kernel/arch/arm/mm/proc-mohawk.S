/*
 *  linux/arch/arm/mm/proc-mohawk.S: MMU functions for Marvell PJ1 core
 *
 *  PJ1 (codename Mohawk) is a hybrid of the xscale3 and Marvell's own core.
 *
 *  Heavily based on proc-arm926.S and proc-xsc3.S
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/hwcap.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/ptrace.h>
#include "proc-macros.S"

/*
 * This is the maximum size of an area which will be flushed.  If the
 * area is larger than this, then we flush the whole cache.
 */
#define CACHE_DLIMIT	32768

/*
 * The cache line size of the L1 D cache.
 */
#define CACHE_DLINESIZE	32

	.section ".data"
l2cacheon:
	.long	0

	.text
/*
 * cpu_mohawk_proc_init()
 */
ENTRY(cpu_mohawk_proc_init)
	mov	pc, lr

/*
 * cpu_mohawk_proc_fin()
 */
ENTRY(cpu_mohawk_proc_fin)
	stmfd	sp!, {lr}
	mov	ip, #PSR_F_BIT | PSR_I_BIT | SVC_MODE
	msr	cpsr_c, ip
	bl	mohawk_flush_kern_cache_all
#ifdef CONFIG_CPU_L2_CACHE
	ldr	r0, =l2cacheon
	ldr	r0, [r0]
	cmp	r0, #0x1
	bne	1f
	mov	r0, #0
	mcr	p15, 1, r0, c7, c11, 0		@ clean L2C all
	mcr	p15, 0, r0, c7, c10, 4		@ Drain write buffer
1:
#endif
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1000			@ ...i............
	bic	r0, r0, #0x000e			@ ............wca.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldmfd	sp!, {pc}

/*
 * cpu_mohawk_reset(loc)
 *
 * Perform a soft reset of the system.  Put the CPU into the
 * same state as it would be if it had been reset, and branch
 * to what would be the reset vector.
 *
 * loc: location to jump to for soft reset
 */
	.align	5
ENTRY(cpu_mohawk_reset)
	mov	ip, #0
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate I,D caches
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
#ifdef CONFIG_MMU
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
#endif
	ldr	ip, =0x5087a
	mcr	p15, 0, ip, c1, c0, 0		@ ctrl register
	mrc	p15, 0, ip, c1, c0, 0		@ ctrl register
	mov	pc, r0

/*
 * cpu_mohawk_do_idle()
 *
 * Called with IRQs disabled
 */
	.align	10
ENTRY(cpu_mohawk_do_idle)
	mov	r0, #0
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer
	mcr	p15, 0, r0, c7, c0, 4		@ wait for interrupt
	mov	r0, #25
1:	subs	r0, r0, #1
	bne	1b
	mov	pc, lr

/*
 *	flush_user_cache_all()
 *
 *	Clean and invalidate all cache entries in a particular
 *	address space.
 */
ENTRY(mohawk_flush_user_cache_all)
	/* FALLTHROUGH */

/*
 *	flush_kern_cache_all()
 *
 *	Clean and invalidate the entire cache.
 */
ENTRY(mohawk_flush_kern_cache_all)
	mov	r2, #VM_EXEC
	mov	ip, #0
__flush_whole_cache:
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, ip, c7, c6, 0		@ invalidate D cache
#else
	mcr	p15, 0, ip, c7, c14, 0	 	@ clean,invalidate D cache
#endif
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcrne	p15, 0, ip, c7, c10, 4		@ drain WB
	mcrne   p15, 0, ip, c7, c5, 4		@ Prefetch Flush
	mov	pc, lr

/*
 *	flush_user_cache_range(start, end, flags)
 *
 *	Clean and invalidate a range of cache entries in the
 *	specified address range.
 *
 *	- start	- start address (inclusive)
 *	- end	- end address (exclusive)
 *	- flags	- vm_flags describing address space
 */
ENTRY(mohawk_flush_user_cache_range)
	mov	ip, #0
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #CACHE_DLIMIT
	bgt	__flush_whole_cache
1:	tst	r2, #VM_EXEC
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
#else
	mcr	p15, 0, r0, c7, c14, 1		@ clean and invalidate D entry
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
#endif
	cmp	r0, r1
	blo	1b
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 6		@ Invalidate BTB
	mcrne	p15, 0, ip, c7, c10, 4		@ drain WB
	mcrne	p15, 0, ip, c7, c5, 4		@ Prefetch Flush
	mov	pc, lr

/*
 *	coherent_kern_range(start, end)
 *
 *	Ensure coherency between the Icache and the Dcache in the
 *	region described by start, end.  If you have non-snooping
 *	Harvard caches, you need to implement this function.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 */
ENTRY(mohawk_coherent_kern_range)
	/* FALLTHROUGH */

/*
 *	coherent_user_range(start, end)
 *
 *	Ensure coherency between the Icache and the Dcache in the
 *	region described by start, end.  If you have non-snooping
 *	Harvard caches, you need to implement this function.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 */
ENTRY(mohawk_coherent_user_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 6		@ Invalidate BTB
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mcr	p15, 0, r0, c7, c5, 4		@ Prefetch Flush
	mov	pc, lr

/*
 *	flush_kern_dcache_page(void *page)
 *
 *	Ensure no D cache aliasing occurs, either with itself or
 *	the I cache
 *
 *	- addr	- page aligned address
 */
ENTRY(mohawk_flush_kern_dcache_page)
	add	r1, r0, #PAGE_SZ
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mcr	p15, 0, r0, c7, c5, 4		@ Prefetch Flush
	mov	pc, lr

/*
 *	dma_inv_range(start, end)
 *
 *	Invalidate (discard) the specified virtual address range.
 *	May not write back any entries.  If 'start' or 'end'
 *	are not cache line aligned, those lines must be written
 *	back.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 *
 * (same as v4wb)
 */
ENTRY(mohawk_dma_inv_range)
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
	tst	r0, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r0, c7, c10, 1		@ clean D entry
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
	mcrne	p15, 1, r0, c7, c11, 1		@ clean L2 entry
2:
#endif
	tst	r1, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r1, c7, c10, 1		@ clean D entry
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
	mcrne	p15, 1, r1, c7, c11, 1		@ clean L2 entry
2:
#endif
#endif
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
	mcr	p15, 1, r0, c7, c7, 1		@ invalidate L2 entry
2:
#endif
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	dma_clean_range(start, end)
 *
 *	Clean the specified virtual address range.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 *
 * (same as v4wb)
 */
ENTRY(mohawk_dma_clean_range)
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
	mcr	p15, 1, r0, c7, c11, 1		@ clean L2 entry
2:
#endif
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	dma_flush_range(start, end)
 *
 *	Clean and invalidate the specified virtual address range.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 */
ENTRY(mohawk_dma_flush_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
	mcr	p15, 1, r0, c7, c15, 1		@ clean+invalidate L2 entry
2:
#endif
#else
	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
	mcr	p15, 1, r0, c7, c11, 1		@ clean L2 entry
2:
#endif
#endif
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	flush_l2_range(start, end)
 *
 *	Clean and invalidate the specified L2 physics address range .
 *
 *	- start	- physics start address
 *	- end	- physics end address
 */
ENTRY(mohawk_flush_l2_range)
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:
	mcr	p15, 1, r0, c7, c15, 3		@ clean+invalidate L2 entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
2:
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
#endif
	mov	pc, lr

ENTRY(mohawk_cache_fns)
	.long	mohawk_flush_kern_cache_all
	.long	mohawk_flush_user_cache_all
	.long	mohawk_flush_user_cache_range
	.long	mohawk_coherent_kern_range
	.long	mohawk_coherent_user_range
	.long	mohawk_flush_kern_dcache_page
	.long	mohawk_dma_inv_range
	.long	mohawk_dma_clean_range
	.long	mohawk_dma_flush_range

ENTRY(cpu_mohawk_dcache_clean_area)
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHE_DLINESIZE
	subs	r1, r1, #CACHE_DLINESIZE
	bhi	1b
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

ENTRY(pxa910_cpu_disable_l2)
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
    mcr p15, 1, r0, c7, c11, 0   /* clean L2 cache */
    mrc p15, 0, r0, c1, c0, 0
    bic r0, r0, #(1 << 26)
    mcr p15, 0, r0, c1, c0, 0    /* disable L2 cache */

1:  ldr r0, =0xfe282c08      /* test L2 IDLE bit */
    ldr r0, [r0]
    tst r0, #(1 << 16)
    beq 1b
2:
#endif
    mov pc, lr

ENTRY(pxa910_cpu_enable_l2)
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r3, =l2cacheon
	ldr	r3, [r3]
	cmp	r3, #0x1
	bne	2f
    mcr p15, 1, r0, c7, c7, 0    /* invalidate L2 cache */
    mrc p15, 0, r0, c1, c0, 0
    orr r0, r0, #(1 << 26)
    mcr p15, 0, r0, c1, c0, 0    /* enable L2 cache */

1:  ldr r0, =0xfe282c08      /* test L2 IDLE bit */
    ldr r0, [r0]
    tst r0, #(1 << 16)
    beq 1b
2:
#endif
    mov pc, lr
/* =============================== PageTable ============================== */

/*
 * cpu_mohawk_switch_mm(pgd)
 *
 * Set the translation base pointer to be as described by pgd.
 *
 * pgd: new page tables
 */
	.align	5
ENTRY(cpu_mohawk_switch_mm)
#ifdef CONFIG_MMU
	mov	ip, #0
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, ip, c7, c6, 0		@ invalidate D cache
#else
@ && 'Clean & Invalidate whole DCache'
	mcr	p15, 0, ip, c7, c14, 0	 	@ clean,invalidate D cache
#endif
	mcr	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
	mcr	p15, 0, ip, c7, c5, 4		@ Prefetch Flush

#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
#if defined(CONFIG_CPU_L2_CACHE)
	orr	r0, r0, #0x10			@ L2 - Outer write-through
#endif
#else
#if defined(CONFIG_CPU_L2_CACHE)
	orr	r0, r0, #0x18			@ L2 - Outer write-back
#endif
#endif

	mcr	p15, 0, r0, c2, c0, 0		@ load page table pointer
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
#endif
	mov	pc, lr

/*
 * cpu_mohawk_set_pte_ext(ptep, pte, ext)
 *
 * Set a PTE and flush it out
 */
	.align	5
ENTRY(cpu_mohawk_set_pte_ext)
#ifdef CONFIG_MMU
	str	r1, [r0], #-2048		@ linux version
#if CONFIG_XSM_PAGE
        /* Extended small page for xscale core 4 (PJ1) */

        bic     r2, r1, #0xff0                  @ Keep C, B, coherency bits
        bic     r2, r2, #0x03
        orr     r2, r2, #PTE_TYPE_EXT           @ extended page
        tst     r1, #L_PTE_SHARED               @ shared?
        orrne   r2, r2, #0x200
        eor     r3, r1, #L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_WRITE | L_PTE_DIRTY

        tst     r3, #L_PTE_USER                 @ User?
        orrne   r2, r2, #PTE_EXT_AP_URO_SRW     @ yes -> user r/o, system r/w
        tst     r3, #L_PTE_WRITE | L_PTE_DIRTY  @ Write and Dirty?
        orreq   r2, r2, #PTE_EXT_AP_UNO_SRW     @ yes -> user n/a, system r/w
        @ if C & W is on: set OC to be cacheable if C=1 B=0 W=1
        eor     ip, r1, #L_PTE_CACHEABLE | L_PTE_WRITE
        tst     ip, #L_PTE_CACHEABLE | L_PTE_WRITE
        orreq   r2, r2, #PTE_EXT_TEX(0x5)
        @ if C & B is on:
        eor     ip, r1, #L_PTE_CACHEABLE | L_PTE_BUFFERABLE
        tst     ip, #L_PTE_CACHEABLE | L_PTE_BUFFERABLE
        orreq   r2, r2, #PTE_EXT_TEX(0x5)                       @ IC: WB OC:WBWA
#ifdef CONFIG_MV_L2_CACHE_SUPPORT
        @ If it's cacheable, it needs to be in L2 also.
        eor     ip, r1, #L_PTE_CACHEABLE
        tst     ip, #L_PTE_CACHEABLE
        orreq   r2, r2, #PTE_EXT_TEX(0x5)
#endif
        #ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
        bic     r2, r2, #L_PTE_BUFFERABLE
        #endif
        tst     r3, #L_PTE_PRESENT | L_PTE_YOUNG        	@ Present and Young?
        movne   r2, #0                                  	@ no -> fault
        @ FIXME: Do not masking out when our CPU support S bit
        bic     r2, r2, #0xE00                          @ Mask out SBZ amd S bit
#else /* CONFIG_XSM_PAGE */
	eor	r3, r1, #L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_WRITE | L_PTE_DIRTY

	bic	r2, r1, #PTE_SMALL_AP_MASK	@ keep C, B bits
	bic	r2, r2, #PTE_TYPE_MASK
	orr	r2, r2, #PTE_TYPE_SMALL

	tst	r3, #L_PTE_USER			@ user?
	orrne	r2, r2, #PTE_SMALL_AP_URO_SRW

	tst	r3, #L_PTE_WRITE | L_PTE_DIRTY	@ write and dirty?
	orreq	r2, r2, #PTE_SMALL_AP_UNO_SRW

	tst	r3, #L_PTE_PRESENT | L_PTE_YOUNG	@ present and young?
	movne	r2, #0

#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	eor	r3, r2, #0x0a			@ C & small page?
	tst	r3, #0x0b
	biceq	r2, r2, #L_PTE_BUFFERABLE
#endif
#endif /* CONFIG_XSM_PAGE */
	str	r2, [r0]			@ hardware version
	mov	r0, r0
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
#endif
	mov	pc, lr

	__INIT

	.type	__mohawk_setup, #function
__mohawk_setup:
	ldr	r6, =0xd4282c00
	ldr	r5, [r6]
	mov	r5, r5, LSR #16
	mov	r6, #0xa1			@ A1 processor?
	cmp	r5, r6
	blo	1f
	ldr	r5, =l2cacheon
	bic	r5, #0xC0000000
	mov	r6, #0x1
	str	r6, [r5]
1:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		@ invalidate I,D caches on v4
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r5, =l2cacheon
	bic	r5, #0xC0000000
	ldr	r5, [r5]
	cmp	r5, #0x1
	bne	1f
	mcr	p15, 1, r0, c7, c7, 0		@ invalidate L2 cache
1:
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer on v4
	mcr	p15, 0, r0, c7, c5, 4		@ prefetch flush
#ifdef CONFIG_MMU
	mcr	p15, 0, r0, c8, c7		@ invalidate I,D TLBs on v4
#endif


#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mov	r0, #4				@ disable write-back on caches explicitly
	mcr	p15, 7, r0, c15, c0, 0
#if defined(CONFIG_CPU_L2_CACHE)
	orr	r4, r4, #0x10			@ L2 - Outer write through
#endif
#else
#if defined(CONFIG_CPU_L2_CACHE)
	orr	r4, r4, #0x18			@ L2 - Outer write back
#endif
#endif 

	mrc	p15, 1, r0, c15, c1, 0		@ get extra features register
#ifdef CONFIG_CPU_L2_CACHE_PREFETCH
	bic	r0, r0, #(1 << 24)		@ L2 prefetch enable
#else
	orr	r0, r0, #(1 << 24)		@ L2 prefetch disable
#endif
	mcr	p15, 1, r0, c15, c1, 0		@ set extra features register

	adr	r5, mohawk_crval
	ldmia	r5, {r5, r6}
	mrc	p15, 0, r0, c1, c0		@ get control register v4
	bic	r0, r0, r5
	orr	r0, r0, r6
	orr	r0, r0, #0x800			@ BPU enable
#if defined(CONFIG_CPU_L2_CACHE)
	ldr	r5, =l2cacheon
	bic	r5, #0xC0000000
	ldr	r5, [r5]
	cmp	r5, #0x1
	bne	1f
	orr	r0, r0, #(1 << 26)		@ L2 enable - xscale/PJ41/PJ4
1:
#endif
	mrc	p15, 1, r5, c15, c1, 0		@ get extra features register
	orr	r5, r5, #(1<<29)		@ streaming enable
	@orr	r5, r5, #(1<<19)		@ disable branch return stack
	@orr	r5, r5, #(1<<18)		@ hold conditional indirect branch
	orr	r5, r5, #(3<<9)			@ 8-wait cycles write coalesce
	orr	r5, r5, #(1<<8)			@ enable write buffer coalescing
	@orr	r5, r5, #(1<<7)			@ disable sparse write strobes
	mcr     p15, 1, r5, c15, c1, 0          @ set extra features register

	mov	pc, lr
	.size	__mohawk_setup, . - __mohawk_setup

	/*
	 *  R
	 * .RVI ZFRS BLDP WCAM
	 * .011 0001 ..11 0101
	 *
	 */
	.type	mohawk_crval, #object
mohawk_crval:
	crval	clear=0x00007f3f, mmuset=0x00003135, ucset=0x00001134

	__INITDATA

/*
 * Purpose : Function pointers used to access above functions - all calls
 *	     come through these
 */
	.type	mohawk_processor_functions, #object
mohawk_processor_functions:
	.word	v5t_early_abort
	.word	legacy_pabort
	.word	cpu_mohawk_proc_init
	.word	cpu_mohawk_proc_fin
	.word	cpu_mohawk_reset
	.word	cpu_mohawk_do_idle
	.word	cpu_mohawk_dcache_clean_area
	.word	cpu_mohawk_switch_mm
	.word	cpu_mohawk_set_pte_ext
	.size	mohawk_processor_functions, . - mohawk_processor_functions

	.section ".rodata"

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv5te"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v5"
	.size	cpu_elf_name, . - cpu_elf_name

	.type	cpu_mohawk_name, #object
cpu_mohawk_name:
	.asciz	"Marvell 88SV331x"
	.size	cpu_mohawk_name, . - cpu_mohawk_name

	.align

	.section ".proc.info.init", #alloc, #execinstr

	.type	__88sv331x_proc_info,#object
__88sv331x_proc_info:
	.long	0x56158000			@ Marvell 88SV331x (MOHAWK)
	.long	0xfffff000
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_BIT4 | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_BIT4 | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__mohawk_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP|HWCAP_JAVA
	.long	cpu_mohawk_name
	.long	mohawk_processor_functions
	.long	v4wbi_tlb_fns
	.long	v4wb_user_fns
	.long	mohawk_cache_fns
	.size	__88sv331x_proc_info, . - __88sv331x_proc_info
