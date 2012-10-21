#
# Makefile for Marvell's PXA168 processors line
#

obj-y				+= common.o clock.o mvd_interface.o devices.o time.o onboard/

# SoC support
obj-$(CONFIG_CPU_PXA168)	+= pxa168.o irq-pxa168.o
obj-$(CONFIG_CPU_PXA910)	+= pxa910.o irq-pxa168.o pxa910-squ.o ssp.o
obj-$(CONFIG_CPU_MMP2)		+= mmp2.o irq-mmp2.o

# board support
obj-$(CONFIG_MACH_ASPENITE)	+= aspenite.o
obj-$(CONFIG_MACH_ZYLONITE2)	+= aspenite.o
obj-$(CONFIG_MACH_AVENGERS_LITE)+= avengers_lite.o
obj-$(CONFIG_MACH_TAVOREVB)	+= tavorevb.o
obj-$(CONFIG_MACH_TTC_DKB)	+= ttc_dkb.o
obj-$(CONFIG_MACH_FLINT)	+= flint.o
obj-$(CONFIG_MACH_MARVELL_JASPER) += jasper.o

obj-$(CONFIG_PXA_RAMDUMP)	+= ramdump.o

obj-$(CONFIG_DVFM)		+= dvfm.o
obj-$(CONFIG_PXA910_DVFM_STATS) += dvfm_stats.o
obj-$(CONFIG_DVFM_PXA910)	+= pxa910_dvfm.o pxa910_dvfm_ll.o
obj-$(CONFIG_MSPM_PXA910)	+= pxa910_mspm_idle.o pxa910_mspm_prof.o
obj-$(CONFIG_PM_PXA910)		+= pxa910_pm.o
obj-$(CONFIG_MSPM_PXA910) += cpufreq-pxa910.o

