/*
 * Tc358762 MIPI to Parallel Bridge Chip
 *
 *
 * Copyright (C) 2006, Marvell Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <mach/cputype.h>
#include <mach/mfp-mmp2.h>
#include <mach/gpio.h>
#include <mach/regs-mpmu.h>
#include <mach/tc358762.h>

#define TC358762_REG_NUM		(0x510)

/* Unique ID allocation */
static struct i2c_client *g_client;

spinlock_t	lock;

int tc358762_read32(u16 reg, u32 *pval)
{
	int ret;
	int status;
	u8 address[2];
	u8 data[4];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	address[0] = (reg >> 8) & 0xff;
	address[1] = reg & 0xff;

	spin_lock(&lock);
	i2c_master_send(g_client, address, 2);
	ret = i2c_master_recv(g_client, (u8 *)pval, 4);
	if (ret >= 0) {
		status = 0;
	} else
		status = -EIO;
	spin_unlock(&lock);

	return status;
}
EXPORT_SYMBOL(tc358762_read32);

int tc358762_read16(u16 reg, u16 *pval)
{
	int ret;
	int status;
	u8 address[2];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	address[0] = (reg >> 8) & 0xff;
	address[1] = reg & 0xff;

	spin_lock(&lock);
	i2c_master_send(g_client, address, 2);
	ret = i2c_master_recv(g_client, (u8 *)pval, 2);
	if (ret >= 0) {
		status = 0;
	} else
		status = -EIO;
	spin_unlock(&lock);

	return status;
}
EXPORT_SYMBOL(tc358762_read16);

int tc358762_write32(u16 reg, u32 val)
{
	int ret;
	int status;
	u8 data[6];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	data[0] = (reg >> 8) & 0xff;
	data[1] = reg & 0xff;
	data[2]=  val & 0xff;
	data[3] = (val >> 8) & 0xff;
	data[4] = (val >> 16) & 0xff;
	data[5]=  (val >> 24) & 0xff;
	spin_lock(&lock);
	ret = i2c_master_send(g_client, data, 6);
	if (ret >= 0) {
		status = 0;
	} else
		status = -EIO;
	spin_unlock(&lock);

	return status;
}
EXPORT_SYMBOL(tc358762_write32);

int tc358762_write16(u16 reg, u16 val)
{
	int ret;
	int status;
	u8 data[4];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	data[0] = (reg >> 8) & 0xff;
	data[1] = reg & 0xff;
	data[2]=  val & 0xff;
	data[3] = (val >> 8) & 0xff;

	spin_lock(&lock);
	ret = i2c_master_send(g_client, data, 4);
	if (ret >= 0) {
		status = 0;
	} else
		status = -EIO;
	spin_unlock(&lock);

	return status;
}
EXPORT_SYMBOL(tc358762_write16);

#ifdef	CONFIG_PROC_FS
#define	TC358762_PROC_FILE	"driver/tc358762"
static struct proc_dir_entry *tc358762_proc_file;
static int index;

static ssize_t tc358762_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u32 reg_val;

	if ((index < 0) || (index > TC358762_REG_NUM))
		return 0;

	tc358762_read32(index, &reg_val);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t tc358762_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u32 reg_val;
	char messages[256], vol[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		tc358762_write32(index, reg_val);
	}

	return len;
}

static struct file_operations tc358762_proc_ops = {
	.read = tc358762_proc_read,
	.write = tc358762_proc_write,
};

static void create_tc358762_proc_file(void)
{
	tc358762_proc_file = create_proc_entry(TC358762_PROC_FILE, 0644, NULL);
	if (tc358762_proc_file) {
		tc358762_proc_file->proc_fops = &tc358762_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_tc358762_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(TC358762_PROC_FILE, &proc_root);
}

#endif

static int tc358762_poweron(void)
{
	tc358762_write32(0x47c, 0x0);

	mdelay(2);

	return 1;
}


#define TC358762_CHIP_ID	0x4A0
static int __devinit tc358762_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	int status;
	u16 chip_id;
	struct tc358762_platform_data *pdata;

	g_client = client;
	pdata = client->dev.platform_data;
	pdata->platform_init();

	/* init spinlock */
	spin_lock_init(&lock);

	tc358762_poweron();
	status = tc358762_read16(TC358762_CHIP_ID, &chip_id);
	if ((status < 0) || (chip_id != 0x6200)) {
		printk(KERN_WARNING "tc358762 unavailable!\n");
		g_client = NULL;
		return -ENXIO;
	} else {
		printk(KERN_INFO "tc358762(chip id:0x%02x) detected.\n", chip_id);
	}

#ifdef	CONFIG_PROC_FS
	create_tc358762_proc_file();
#endif

	return 0;
}

static int tc358762_remove(struct i2c_client *client)
{
#ifdef	CONFIG_PROC_FS
	remove_tc358762_proc_file();
#endif

	return 0;
}

static const struct i2c_device_id tc358762_id[] = {
	{ "tc358762", 0 },
	{ }
};

static struct i2c_driver tc358762_driver = {
	.driver = {
		.name	= "tc358762",
	},
	.id_table 	= tc358762_id,
	.probe		= tc358762_probe,
	.remove		= tc358762_remove,
};

static int __init tc358762_init(void)
{
	return i2c_add_driver(&tc358762_driver);
}

static void __exit tc358762_exit(void)
{
	i2c_del_driver(&tc358762_driver);
}

subsys_initcall(tc358762_init);
module_exit(tc358762_exit);

MODULE_DESCRIPTION("Tc358762 Driver");
MODULE_LICENSE("GPL");

