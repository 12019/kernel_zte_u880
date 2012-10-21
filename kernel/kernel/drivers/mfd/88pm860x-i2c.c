/*
 * I2C driver for Marvell 88PM860x
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * 	Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/mfd/88pm860x.h>
#include <mach/pxa910_pm.h>
static struct i2c_client *pm8607_i2c_client;
static u8 board_id = 0;
static inline int pm860x_read_device(struct i2c_client *i2c,
				     int reg, int bytes, void *dest)
{
	int ret;

	if (bytes > 1)
		ret = i2c_smbus_read_i2c_block_data(i2c, reg, bytes, dest);
	else {
		ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0)
		return ret;
		*(unsigned char *)dest = (unsigned char)ret;
	}
		return ret;
}

static inline int pm860x_write_device(struct i2c_client *i2c,
				      int reg, int bytes, void *src)
{
	unsigned char buf[bytes + 1];
	int ret;

	buf[0] = (unsigned char)reg;
	memcpy(&buf[1], src, bytes);

	ret = i2c_master_send(i2c, buf, bytes + 1);
	if (ret < 0)
		return ret;
	return 0;
}

int pm860x_reg_read(struct i2c_client *i2c, int reg)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char data = 0;
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pm860x_read_device(i2c, reg, 1, &data);
	mutex_unlock(&chip->io_lock);

	if (ret < 0)
		return ret;
	else
		return (int)data;
}
EXPORT_SYMBOL(pm860x_reg_read);

int pm860x_reg_write(struct i2c_client *i2c, int reg,
		     unsigned char data)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pm860x_write_device(i2c, reg, 1, &data);
	mutex_unlock(&chip->io_lock);

	return ret;
}
EXPORT_SYMBOL(pm860x_reg_write);

int pm860x_codec_reg_read(int reg)
{
	struct pm860x_chip *chip = i2c_get_clientdata(pm8607_i2c_client);
	unsigned char data = 0;
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pm860x_read_device(pm8607_i2c_client, reg, 1, &data);
	mutex_unlock(&chip->io_lock);

	if (ret < 0)
		return ret;
	else
		return (int)data;
}
EXPORT_SYMBOL(pm860x_codec_reg_read);

int pm860x_codec_reg_write(int reg, unsigned char data)
{
	struct pm860x_chip *chip = i2c_get_clientdata(pm8607_i2c_client);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pm860x_write_device(pm8607_i2c_client, reg, 1, &data);
	mutex_unlock(&chip->io_lock);

	return ret;
}
EXPORT_SYMBOL(pm860x_codec_reg_write);

int pm860x_bulk_read(struct i2c_client *i2c, int reg,
		     int count, unsigned char *buf)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pm860x_read_device(i2c, reg, count, buf);
	mutex_unlock(&chip->io_lock);

	return ret;
}
EXPORT_SYMBOL(pm860x_bulk_read);

int pm860x_bulk_write(struct i2c_client *i2c, int reg,
		      int count, unsigned char *buf)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pm860x_write_device(i2c, reg, count, buf);
	mutex_unlock(&chip->io_lock);

	return ret;
}
EXPORT_SYMBOL(pm860x_bulk_write);

int pm860x_set_bits(struct i2c_client *i2c, int reg,
		    unsigned char mask, unsigned char data)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char value;
	int ret;

	mutex_lock(&chip->io_lock);
	ret = pm860x_read_device(i2c, reg, 1, &value);
	if (ret < 0)
		goto out;
	value &= ~mask;
	value |= data;
	ret = pm860x_write_device(i2c, reg, 1, &value);
out:
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_set_bits);

static int read_device(struct i2c_client *i2c, int reg,
		       int bytes, void *dest)
{
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX + 3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX + 2];
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg[2] = {{i2c->addr, i2c->flags, 1, msgbuf0},
				 {i2c->addr, i2c->flags | I2C_M_RD, 0, msgbuf1},
				};
	int num = 1, ret = 0;

	if (dest == NULL)
		return -EINVAL;
	msgbuf0[0] = (unsigned char)reg;	/* command */
	msg[1].len = bytes;

	/* if data needs to read back, num should be 2 */
	if (bytes > 0)
		num = 2;
	ret = adap->algo->master_xfer(adap, msg, num);
	memcpy(dest, msgbuf1, bytes);
	return ret;
}

static int write_device(struct i2c_client *i2c, int reg,
			int bytes, void *src)
{
	unsigned char buf[bytes + 1];
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg;
	int ret;

	buf[0] = (unsigned char)reg;
	memcpy(&buf[1], src, bytes);
	msg.addr = i2c->addr;
	msg.flags = i2c->flags;
	msg.len = bytes + 1;
	msg.buf = buf;

	ret = adap->algo->master_xfer(adap, &msg, 1);
	if (ret < 0)
		return ret;
	return 0;
}

int pm860x_page_reg_read(struct i2c_client *i2c, int reg)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char zero = 0;
	unsigned char data;
	int ret;

	mutex_lock(&chip->io_lock);
	i2c_lock_adapter(i2c->adapter);
	read_device(i2c, 0xFA, 0, &zero);
	read_device(i2c, 0xFB, 0, &zero);
	read_device(i2c, 0xFF, 0, &zero);
	ret = read_device(i2c, reg, 1, &data);
	if (ret >= 0)
		ret = (int)data;
	read_device(i2c, 0xFC, 0, &zero);
	i2c_unlock_adapter(i2c->adapter);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_page_reg_read);

int pm860x_page_reg_write(struct i2c_client *i2c, int reg,
			  unsigned char data)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char zero;
	int ret;

	mutex_lock(&chip->io_lock);
	i2c_lock_adapter(i2c->adapter);
	read_device(i2c, 0xFA, 0, &zero);
	read_device(i2c, 0xFB, 0, &zero);
	read_device(i2c, 0xFF, 0, &zero);
	ret = write_device(i2c, reg, 1, &data);
	read_device(i2c, 0xFC, 0, &zero);
	i2c_unlock_adapter(i2c->adapter);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_page_reg_write);

int pm860x_page_bulk_read(struct i2c_client *i2c, int reg,
			  int count, unsigned char *buf)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char zero = 0;
	int ret;

	mutex_lock(&chip->io_lock);
	i2c_lock_adapter(i2c->adapter);
	read_device(i2c, 0xFA, 0, &zero);
	read_device(i2c, 0xFB, 0, &zero);
	read_device(i2c, 0xFF, 0, &zero);
	ret = read_device(i2c, reg, count, buf);
	read_device(i2c, 0xFC, 0, &zero);
	i2c_unlock_adapter(i2c->adapter);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_page_bulk_read);

int pm860x_page_bulk_write(struct i2c_client *i2c, int reg,
			   int count, unsigned char *buf)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char zero = 0;
	int ret;

	mutex_lock(&chip->io_lock);
	i2c_lock_adapter(i2c->adapter);
	read_device(i2c, 0xFA, 0, &zero);
	read_device(i2c, 0xFB, 0, &zero);
	read_device(i2c, 0xFF, 0, &zero);
	ret = write_device(i2c, reg, count, buf);
	read_device(i2c, 0xFC, 0, &zero);
	i2c_unlock_adapter(i2c->adapter);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_page_bulk_write);

int pm860x_page_set_bits(struct i2c_client *i2c, int reg,
			 unsigned char mask, unsigned char data)
{
	struct pm860x_chip *chip = i2c_get_clientdata(i2c);
	unsigned char zero;
	unsigned char value;
	int ret;

	mutex_lock(&chip->io_lock);
	i2c_lock_adapter(i2c->adapter);
	read_device(i2c, 0xFA, 0, &zero);
	read_device(i2c, 0xFB, 0, &zero);
	read_device(i2c, 0xFF, 0, &zero);
	ret = read_device(i2c, reg, 1, &value);
	if (ret < 0)
		goto out;
	value &= ~mask;
	value |= data;
	ret = write_device(i2c, reg, 1, &value);
out:
	read_device(i2c, 0xFC, 0, &zero);
	i2c_unlock_adapter(i2c->adapter);
	mutex_unlock(&chip->io_lock);
	return ret;
}
EXPORT_SYMBOL(pm860x_page_set_bits);



static const struct i2c_device_id pm860x_id_table[] = {
	{ "88PM860x", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, pm860x_id_table);

static int verify_addr(struct i2c_client *i2c)
{
	unsigned short addr_8607[] = {0x30, 0x34};
	unsigned short addr_8606[] = {0x10, 0x11};
	int size, i;

	if (i2c == NULL)
		return 0;
	size = ARRAY_SIZE(addr_8606);
	for (i = 0; i < size; i++) {
		if (i2c->addr == *(addr_8606 + i))
			return CHIP_PM8606;
	}
	size = ARRAY_SIZE(addr_8607);
	for (i = 0; i < size; i++) {
		if (i2c->addr == *(addr_8607 + i))
			return CHIP_PM8607;
	}
	return 0;
}

void set_ldo13(int on)
{
      int temp=0;

	if(on)
      {
	  	pm860x_reg_write(pm8607_i2c_client, PM8607_VIBRA_SET, 0x0c); 
                temp=pm860x_reg_read(pm8607_i2c_client, PM8607_VIBRA_SET);
                printk("PM8607_VIBRA_SET, shutdown:0x%x\n",temp);
      }
      else
      {
	  	pm860x_reg_write(pm8607_i2c_client, PM8607_VIBRA_SET, 0x0d); 
                temp=pm860x_reg_read(pm8607_i2c_client, PM8607_VIBRA_SET);
                printk("PM8607_VIBRA_SET,open:0x%x\n",temp);
     }
  
      return;

}

void set_ldo8(int enable)
{
  if(enable)
  	{
  	 pm860x_reg_write(pm8607_i2c_client, PM8607_SUPPLIES_EN12, 
		pm860x_reg_read(pm8607_i2c_client, PM8607_SUPPLIES_EN12) | 0x4);
  	 pm860x_reg_write(pm8607_i2c_client, PM8607_LDO8, 0x00);
  	}
  else
  	{
      pm860x_reg_write(pm8607_i2c_client, PM8607_SUPPLIES_EN12, 
		pm860x_reg_read(pm8607_i2c_client, PM8607_SUPPLIES_EN12) & 0xfb);
  	}
  return;
}

void read_boardid(u16 * vbat)
{
	u32 meas_val;
	u8 reg_value[2];
	if (pm860x_bulk_read(pm8607_i2c_client, PM8607_GPADC2_MEAS1, 2, reg_value)
	    >= 0) {
		meas_val = ((reg_value[0] << 4) | (reg_value[1] & 0x0F));
	} else {
		return 0;
	}
	/* voltage in mili volt */
        *vbat = (u16) (((u32) meas_val * 3 * 18 * 1000) >> 12) / 10;
}

int pm860x_get_board(void)
{           
	u16 data=0,i=0,sum=0;
	for(i=0;i<10;i++)
	{
		read_boardid(&data);
		sum+=data;
	}
	sum =sum/10;
	printk("pm860x_get_board sum=%d\n",sum);

	#if  defined CONFIG_PXA_U810 
	
	if(sum<2500)
	return ZTE_HWVERSION1;
	else
	return ZTE_HWVERSION2;
	
	#elif defined  CONFIG_PXA_U802
	
	if(sum<2500)
	return ZTE_HWVERSION2;
	else
	return ZTE_HWVERSION1;
	
	#elif defined  CONFIG_PXA_U880
	
	if(sum<2500)
	return ZTE_HWVERSION1;
	else
	return ZTE_HWVERSION2;

	#elif defined  CONFIG_PXA_U812
	
	return ZTE_HWVERSION1;
	
	#elif defined  CONFIG_PXA_U830
	
	return ZTE_HWVERSION1;

	#endif	  
}

void pm860x_get_boardID_fromboot(void)
{  
	if(strstr(saved_command_line, "boardid=1")) 
	{
		board_id =ZTE_HWVERSION1;
	}
	else if(strstr(saved_command_line, "boardid=2")) 
	{
		board_id =ZTE_HWVERSION2;
	}	
	else if(strstr(saved_command_line, "boardid=3")) 
	{
		board_id =ZTE_HWVERSION3;
	}	
	else if(strstr(saved_command_line, "boardid=4")) 
	{
		board_id =ZTE_HWVERSION4;
	}	printk("pm860x_get_boardID_fromboot board_id=%d\n",board_id);
}

int pm860x_get_boardID(void)
{  
	return board_id;	  
}

static int __devinit pm860x_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct pm860x_platform_data *pdata = client->dev.platform_data;
	struct pm860x_chip *chip;

	if (!pdata) {
		pr_info("No platform data in %s!\n", __func__);
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct pm860x_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->id = verify_addr(client);
	chip->client = client;
	pm8607_i2c_client = client;
	i2c_set_clientdata(client, chip);
	chip->dev = &client->dev;
	mutex_init(&chip->io_lock);
	dev_set_drvdata(chip->dev, chip);

	/*
	 * Both client and companion client shares same platform driver.
	 * Driver distinguishes them by pdata->companion_addr.
	 * pdata->companion_addr is only assigned if companion chip exists.
	 * At the same time, the companion_addr shouldn't equal to client
	 * address.
	 */
	if (pdata->companion_addr && (pdata->companion_addr != client->addr)) {
		chip->companion_addr = pdata->companion_addr;
		chip->companion = i2c_new_dummy(chip->client->adapter,
						chip->companion_addr);
		i2c_set_clientdata(chip->companion, chip);
	}

	if (pdata->fixup)
		pdata->fixup(chip, pdata);
	pm860x_device_init(chip, pdata);
	//board_id = pm860x_get_board();
	printk("board_id=%d\n",board_id);
	return 0;
}

static int __devexit pm860x_remove(struct i2c_client *client)
{
	struct pm860x_chip *chip = i2c_get_clientdata(client);

	pm860x_device_exit(chip);
	i2c_unregister_device(chip->companion);
	i2c_set_clientdata(chip->companion, NULL);
	i2c_set_clientdata(chip->client, NULL);
	kfree(chip);
	return 0;
}

static struct i2c_driver pm860x_driver = {
	.driver	= {
		.name	= "88PM860x",
		.owner	= THIS_MODULE,
	},
	.probe		= pm860x_probe,
	.remove		= __devexit_p(pm860x_remove),
	.id_table	= pm860x_id_table,
};

static int __init pm860x_i2c_init(void)
{
	int ret;
	ret = i2c_add_driver(&pm860x_driver);
	if (ret != 0)
		pr_err("Failed to register 88PM860x I2C driver: %d\n", ret);
	return ret;
}
subsys_initcall(pm860x_i2c_init);

static void __exit pm860x_i2c_exit(void)
{
	i2c_del_driver(&pm860x_driver);
}
module_exit(pm860x_i2c_exit);

MODULE_DESCRIPTION("I2C Driver for Marvell 88PM860x");
MODULE_AUTHOR("Haojian Zhuang <haojian.zhuang@marvell.com>");
MODULE_LICENSE("GPL");
