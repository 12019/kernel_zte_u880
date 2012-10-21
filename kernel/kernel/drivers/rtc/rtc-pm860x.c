/*
 * 88pm860x_rtc.c - Marvell 88PM860x ONKEY driver
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *      Haojian Zhuang <haojian.zhuang@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm860x.h>

#define PM8607_WAKEUP		0x0b

struct pm860x_rtc_info {
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct device		*dev;
	int			irq;
};

static struct i2c_client *i2c = NULL;

int pm860x_rtc_reg_read(int reg)
{
	int ret;
	ret = pm860x_reg_read(i2c, reg);
	return ret;
}
EXPORT_SYMBOL(pm860x_rtc_reg_read);

int pm860x_rtc_reg_write(int reg,unsigned char data)
{
	int ret;
	ret = pm860x_reg_write(i2c, reg, data);
	return ret;
}
EXPORT_SYMBOL(pm860x_rtc_reg_write);


/* 88PM860x gives us an interrupt when ONKEY is held */
static irqreturn_t pm860x_rtc_handler(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int __devinit pm860x_rtc_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_rtc_info *info;
	int irq, ret;
	unsigned char val;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct pm860x_rtc_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->chip = chip;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	i2c = info->i2c;
	info->dev = &pdev->dev;
	info->irq = irq + chip->irq_base;

	ret = request_threaded_irq(info->irq, NULL, pm860x_rtc_handler,
				   IRQF_ONESHOT, "rtc", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, ret);
		goto out_irq;
	}

	platform_set_drvdata(pdev, info);
	/*setting sanremo RTC for adjust accuracy in power off condition*/
	val = pm860x_reg_read(info->i2c,0xa0);
	val |=(1<<7);
	pm860x_reg_write(info->i2c,0xa0,val);

	return 0;

out_irq:
	kfree(info);
	return ret;
}

static int __devexit pm860x_rtc_remove(struct platform_device *pdev)
{
	struct pm860x_rtc_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	kfree(info);
	return 0;
}

static struct platform_driver pm860x_rtc_driver = {
	.driver		= {
		.name	= "88pm860x-rtc",
		.owner	= THIS_MODULE,
	},
	.probe		= pm860x_rtc_probe,
	.remove		= __devexit_p(pm860x_rtc_remove),
};

static int __init pm860x_rtc_init(void)
{
	return platform_driver_register(&pm860x_rtc_driver);
}
module_init(pm860x_rtc_init);

static void __exit pm860x_rtc_exit(void)
{
	platform_driver_unregister(&pm860x_rtc_driver);
}
module_exit(pm860x_rtc_exit);

MODULE_DESCRIPTION("Marvell 88PM860x RTC driver");
MODULE_AUTHOR("Jett Zhou <jett.zhou@marvell.com>");
MODULE_LICENSE("GPL");
