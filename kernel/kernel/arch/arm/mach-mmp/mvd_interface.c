/*
 *  linux/arch/arm/mach-pxa/mvd_interface.c
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <asm/uaccess.h>
//#include <mach/micco.h>
#include <mach/pxa910_pm.h>
#define MVD_KERN_WARN	KERN_WARNING "mvd: "
#define MVD_KERN_INFO	KERN_INFO "mvd: "

//#define ZTE_MVD_MAJOR_NUM		0
//#define ZTE_MVD_DEVICE_NAME 		"mvd"

#define ZTE_MVD_IOCTL_AP_PRINT_SELECT         		1
#define ZTE_MVD_IOCTL_TD_UART_PRINT_SELECT         	2
#define ZTE_MVD_IOCTL_TD_USB_PRINT_SELECT         	3

#define ZTE_MVD_IOCTL_GET_HW_VERSION			4

#define ZTE_MVD_IOCTL_DISABLE_DPM			5
#define ZTE_MVD_IOCTL_ENABLE_DPM			6

#define ZTE_MVD_IOCTL_GET_OS_VERSION			7

char zte_banner;

extern void gpio_dbg_ap_print(void);
extern void gpio_dbg_td_uart_print(void);
extern void gpio_dbg_td_usb_print(void);
extern void set_fixed_624M_freqs(void);
extern  void unset_fixed_624M_freqs(void);

/* adc_in6 
    0-0.85       		hw3
    0.85-1.85  		hw2
    1.85-2.60  		hw1
*/


/* ioctl */
static int mvd_ioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret;
	int var;

	static int iDpmOn = 1; /*default dpm is enabled*/

	printk(MVD_KERN_INFO "mvd_ioctl: (%p,%d)\n", file, cmd);

	switch (cmd) {

	#if 0
		
	case ZTE_MVD_IOCTL_AP_PRINT_SELECT:
		printk(MVD_KERN_INFO "IOCTL_AP_PRINT_SELECT\n");
		gpio_dbg_ap_print();
		ret = 0;
		break;

	case ZTE_MVD_IOCTL_TD_UART_PRINT_SELECT:
		printk(MVD_KERN_INFO "IOCTL_TD_UART_PRINT_SELECT\n");
		gpio_dbg_td_uart_print();
		ret = 0;
		break;

	case ZTE_MVD_IOCTL_TD_USB_PRINT_SELECT:
		printk(MVD_KERN_INFO "IOCTL_TD_USB_PRINT_SELECT\n");
		gpio_dbg_td_usb_print();
		ret = 0;
		break;
	#endif	
	case ZTE_MVD_IOCTL_GET_HW_VERSION:
		printk(MVD_KERN_INFO "ZTE_MVD_IOCTL_GET_HW_VERSION\n");
		var = pm860x_get_boardID();
		ret = copy_to_user((void *) arg, &var,
				    sizeof(var)) ? -EFAULT : 0;
		break;
	#if 0
		
	case ZTE_MVD_IOCTL_DISABLE_DPM:
		printk(MVD_KERN_INFO "ZTE_MVD_IOCTL_DISABLE_DPM\n");
#ifdef CONFIG_PXA3xx_DVFM
		if(0 == iDpmOn)
			printk(MVD_KERN_INFO "dpm is disabled already!\n");
		else {
			iDpmOn = 0;
		set_fixed_624M_freqs();
		}
#endif
		ret = 0;
		break;

	case ZTE_MVD_IOCTL_ENABLE_DPM:
		printk(MVD_KERN_INFO "ZTE_MVD_IOCTL_ENABLE_DPM\n");
#ifdef CONFIG_PXA3xx_DVFM
		if(1 == iDpmOn)
			printk(MVD_KERN_INFO "dpm is enabled already!\n");
		else {
			iDpmOn = 1;
		unset_fixed_624M_freqs();
		}
#endif
		ret = 0;
		break;
	
		case ZTE_MVD_IOCTL_GET_OS_VERSION:
		printk(MVD_KERN_INFO "ZTE_MVD_IOCTL_GET_OS_VERSION\n");
		ret = copy_to_user((void *) arg, zte_banner,
				    strlen(zte_banner)+1) ? -EFAULT : 0;
		ret = 0;
		break;
	#endif
	default:
		printk(MVD_KERN_WARN "mvd_ioctl:unknown cmd\n");
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int mvd_open(struct inode *inode, struct file *file)
{
	printk(MVD_KERN_INFO "mvd is opened now!\n");
	return 0;
}


static int mvd_release(struct inode *inode, struct file *file)
{
	printk(MVD_KERN_INFO "mvd is released now!\n");
	return 0;
}

/*
const struct file_operations mvd_fops = {
	.owner =		THIS_MODULE,
	.ioctl =		mvd_ioctl,
	.open =		mvd_open,
	.release =	mvd_release,
};

static int mvd_probe(struct platform_device *pdev)
{
	int dev_major; 
	dev_major = register_chrdev(0, "mvd", &mvd_fops);
	if (dev_major < 0) 
	{
		printk("Registering the character device failed with %d\n", ZTE_MVD_MAJOR_NUM);
		return -EBUSY;
	}
	else
	{
		printk("mvd register successfully, dev_major is %d\n", dev_major);
	}
	
	return 0;
}

static int mvd_remove(struct platform_device *pdev)
{
	printk("mvd unregister!\n");
	unregister_chrdev(ZTE_MVD_MAJOR_NUM, ZTE_MVD_DEVICE_NAME);
	return 0;
}
*/

const struct file_operations mvd_fops = {
	.owner =		THIS_MODULE,
	.ioctl =		mvd_ioctl,
	.open =		mvd_open,
	.release =	mvd_release,
};

static struct miscdevice mvd_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "mvd",
	.fops	= &mvd_fops,
};

static int mvd_probe(struct platform_device *pdev)
{
	int ret; 
	ret = misc_register(&mvd_miscdev);
	if (ret < 0) {
		printk(MVD_KERN_WARN "unable to register character device /dev/mvd!\n");
	}
	else{
		printk(MVD_KERN_INFO "register character device /dev/mvd sucessfully!\n");
	}
	return ret;
}

static int mvd_remove(struct platform_device *pdev)
{
	printk(MVD_KERN_INFO "unregister character device /dev/mvd!\n");
	misc_deregister(&mvd_miscdev);
	return 0;
}

static struct platform_driver mvd_pdrv = {
	.driver = {
		.name = "zte_mvd",
	},
	.probe = mvd_probe,
	.remove = mvd_remove,
};

int __init mvd_init(void)
{
	int ret;
	
	ret = platform_driver_register(&mvd_pdrv);
	if (ret < 0) {
		printk(MVD_KERN_WARN "Error registering zte_mvd driver\n");
	}

	return ret;
}

/* Cleanup */
static void __exit mvd_exit(void)
{
	platform_driver_unregister(&mvd_pdrv);
}

MODULE_LICENSE("GPL");
module_init(mvd_init);
module_exit(mvd_exit);
