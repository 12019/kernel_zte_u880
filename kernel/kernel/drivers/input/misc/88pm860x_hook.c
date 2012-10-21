/*
 * 88pm860x_hook.c - Marvell 88PM860x HOOK driver
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *      Raul Xiong <xjian@marvell.com>
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

struct pm860x_hook_info {
	struct input_dev	*idev;
	struct pm860x_chip	*chip;
	struct i2c_client	*i2c;
	struct device		*dev;
	int			irq;
};

/* 88PM860x gives us an interrupt when HOOK is held */
static irqreturn_t pm860x_hook_handler(int irq, void *data)
{
	struct pm860x_hook_info *info = data;
	int ret;

	ret = pm860x_reg_read(info->i2c, PM8607_STATUS_1);
	ret &= PM8607_STATUS_HOOK;
	if (ret)
		printk(KERN_INFO "hook button press down\n");
	else
		printk(KERN_INFO "hook button press up\n");
	input_report_key(info->idev, KEY_FN, ret);//Don't know what key Android want, use KEY_FN temporarily
	input_sync(info->idev);
	return IRQ_HANDLED;
}

static int __devinit pm860x_hook_probe(struct platform_device *pdev)
{
	struct pm860x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct pm860x_hook_info *info;
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource!\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct pm860x_hook_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	info->chip = chip;
	info->i2c = (chip->id == CHIP_PM8607) ? chip->client : chip->companion;
	info->dev = &pdev->dev;
	info->irq = irq + chip->irq_base;

	info->idev = input_allocate_device();
	if (!info->idev) {
		dev_err(chip->dev, "Failed to allocate input dev\n");
		ret = -ENOMEM;
		goto out;
	}

	info->idev->name = "88pm860x_hook";
	info->idev->phys = "88pm860x_hook/input0";
	info->idev->id.bustype = BUS_I2C;
	info->idev->dev.parent = &pdev->dev;
	info->idev->evbit[0] = BIT_MASK(EV_KEY);
	info->idev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);

	ret = input_register_device(info->idev);
	if (ret) {
		dev_err(chip->dev, "Can't register input device: %d\n", ret);
		goto out_reg;
	}

	//enable MIC bias to enable hook detection, we must enable mic bias first otherwise we may get false hook detection
	pm860x_reg_write(info->i2c, PM8607_AUDIO_REG_BASE + PM8607_AUDIO_ADC_ANALOG_PROGRAM1, 0x60);
	//set hook detection debounce time to 24ms, it's the best setting we experienced
	ret = pm860x_reg_read(info->i2c, PM8607_HEADSET_DECTION);
	ret |= 0x10;
	pm860x_reg_write(info->i2c, PM8607_HEADSET_DECTION, ret);

	ret = request_threaded_irq(info->irq, NULL, pm860x_hook_handler,
				   IRQF_ONESHOT, "hook", info);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to request IRQ: #%d: %d\n",
			info->irq, ret);
		goto out_irq;
	}

	platform_set_drvdata(pdev, info);
	return 0;

out_irq:
	input_unregister_device(info->idev);
	kfree(info);
	return ret;

out_reg:
	input_free_device(info->idev);
out:
	kfree(info);
	return ret;
}

static int __devexit pm860x_hook_remove(struct platform_device *pdev)
{
	struct pm860x_hook_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	input_unregister_device(info->idev);
	kfree(info);
	return 0;
}

static struct platform_driver pm860x_hook_driver = {
	.driver		= {
		.name	= "88pm860x-hook",
		.owner	= THIS_MODULE,
	},
	.probe		= pm860x_hook_probe,
	.remove		= __devexit_p(pm860x_hook_remove),
};

static int __init pm860x_hook_init(void)
{
	return platform_driver_register(&pm860x_hook_driver);
}
module_init(pm860x_hook_init);

static void __exit pm860x_hook_exit(void)
{
	platform_driver_unregister(&pm860x_hook_driver);
}
module_exit(pm860x_hook_exit);

MODULE_DESCRIPTION("Marvell 88PM860x HOOK driver");
MODULE_AUTHOR("Raul Xiong <xjian@marvell.com>");
MODULE_LICENSE("GPL");
