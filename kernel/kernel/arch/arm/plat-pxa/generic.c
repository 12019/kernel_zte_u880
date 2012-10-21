/*
 *  linux/arch/arm/plat-pxa/generic.c
 *
 *  Code to PXA processor lines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/proc_fs.h>
#include <asm/page.h>
#include <asm/setup.h>

#include <plat/generic.h>

static unsigned _cp_area_addr;
static int __init setup_cpmem(char *p)
{
	unsigned long size, start = 0x7000000;
	size  = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);
	BUG_ON(reserve_bootmem(start, size, BOOTMEM_EXCLUSIVE) != 0);
	printk("Reserved CP memory: %dM at %.8x\n",
			(unsigned)size/0x100000, (unsigned)start);
	_cp_area_addr = (unsigned)start;
	return 1;
}
__setup("cpmem=", setup_cpmem);

unsigned cp_area_addr(void)
{
	return _cp_area_addr;
}
EXPORT_SYMBOL(cp_area_addr);

char default_pxa_cmdline[COMMAND_LINE_SIZE] = "unknow";  
static int __init parse_tag_pxa(const struct tag *tag)
{
	strlcpy(default_pxa_cmdline, tag->u.cmdline.cmdline, COMMAND_LINE_SIZE);
	printk(KERN_INFO "pxa cmdline:%s\n", default_pxa_cmdline);
	return 0;
}
__tagtable('pxa', parse_tag_pxa);

static ssize_t pxa_cmdline_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(default_pxa_cmdline);

	sprintf(page, "%s\n", default_pxa_cmdline);
	return len + 1;
}
static int __init create_pxa_cmdline_proc_file(void)
{
	struct proc_dir_entry *pxa_cmdline_proc_file = 
		create_proc_entry("pxa_cmdline", 0644, NULL);

	if (pxa_cmdline_proc_file) 
		pxa_cmdline_proc_file->read_proc = pxa_cmdline_read_proc;
	else 
		printk(KERN_INFO "pxa_cmdline proc file create failed!\n");

	return 0;
}
module_init(create_pxa_cmdline_proc_file);

static int android_project = 0;
static int __init android_setup(char *__unused)
{
	android_project = 1;
	return 1;
}
__setup("android", android_setup);

int is_android(void)
{
	return android_project;
}
EXPORT_SYMBOL(is_android);

static int lab_kernel = 0;
static int __init lab_setup(char *__unused)
{
	lab_kernel = 1;
	return 1;
}
__setup("lab", lab_setup);

int is_lab(void)
{
	return lab_kernel;
}
EXPORT_SYMBOL(is_lab);
#ifdef CONFIG_ANDROID_PMEM
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
void android_add_pmem(char *name, size_t size, int no_allocator, int cached)
{
	struct platform_device *android_pmem_device;
	struct android_pmem_platform_data *android_pmem_pdata;
	struct page *page;
	unsigned long addr, tmp;
	static int id;
	unsigned long paddr = 0;

	android_pmem_device = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if(android_pmem_device == NULL)
		return ;

	android_pmem_pdata = kzalloc(sizeof(struct android_pmem_platform_data), GFP_KERNEL);
	if(android_pmem_pdata == NULL) {
		kfree(android_pmem_device);
		return ;
	}
	
	page = alloc_pages(GFP_KERNEL, get_order(size));
	if (page == NULL)
		return ;

	addr = (unsigned long)page_address(page);
	paddr = virt_to_phys((void *)addr);
	tmp = size;
	dma_cache_maint((void *)addr, size, DMA_FROM_DEVICE);
	while(tmp > 0) {
		SetPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		tmp -= PAGE_SIZE;
	}
	android_pmem_pdata->name = name;
	android_pmem_pdata->start = paddr;
	android_pmem_pdata->size = size;
	android_pmem_pdata->no_allocator = no_allocator ;
	android_pmem_pdata->cached = cached;

	android_pmem_device->name = "android_pmem";
	android_pmem_device->id = id++;
	android_pmem_device->dev.platform_data = android_pmem_pdata;

	platform_device_register(android_pmem_device);
}
EXPORT_SYMBOL(android_add_pmem);
#endif

