/*
 *  drivers/sensors/sensors_class.c
 *
 * Copyright (C) 2008 Borqs, Ltd.
 * Author: Ru Yi <yi.ru@borqs.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/sensors.h>

struct class *sensors_class = NULL;
static struct device_attribute sensors_attrs[];

static atomic_t device_count;
static int sensors_major = 0;

static LIST_HEAD(sensors_list);
static DEFINE_MUTEX(sensors_mutex);

int old_sensors_id[SENSORS_MASK] = {
	DEFAULT_ID,
	(1 << 1),
	(1 << 3),
	(1 << 0),
	DEFAULT_ID,
	(1 << 4),
	DEFAULT_ID,
	(1 << 2),
	(1 << 5),
	(1 << 6),
	(1 << 7),
};

static int sensors_open(struct inode * inode, struct file * file)
{
	int minor = iminor(inode);
	struct sensors_dev *sdev;
	int err = -ENODEV;
	const struct file_operations *old_fops, *new_fops = NULL;
        printk(KERN_INFO "sensors_open\n");	

	mutex_lock(&sensors_mutex);
	
	list_for_each_entry(sdev, &sensors_list, list) {
		if (sdev->minor == minor) {
			new_fops = fops_get(sdev->fops);		
			break;
		}
	}
		
	if (!new_fops) {
		mutex_unlock(&sensors_mutex);
		request_module("char-major-%d-%d", sensors_major, minor);
		mutex_lock(&sensors_mutex);

		list_for_each_entry(sdev, &sensors_list, list) {
			if (sdev->minor == minor) {
				new_fops = fops_get(sdev->fops);
				break;
			}
		}
		if (!new_fops)
			goto fail;
	}

	err = 0;
	old_fops = file->f_op;
	file->f_op = new_fops;
	if (file->f_op->open) {
		err=file->f_op->open(inode,file);
		if (err) {
			fops_put(file->f_op);
			file->f_op = fops_get(old_fops);
                        printk(KERN_INFO "sensor fop->open error\n");
		}
	}
	fops_put(old_fops);
fail:
	mutex_unlock(&sensors_mutex);
	return err;
}

static const struct file_operations sensors_fops = {
	.owner		= THIS_MODULE,
	.open		= sensors_open,
};

static int create_sensors_class(void)
{
        printk(KERN_INFO "create_sensors_class\n");
	if (!sensors_major) {
		sensors_major = register_chrdev(0, "sensors", &sensors_fops);
		if (sensors_major < 0) {
			printk(KERN_WARNING
				"Sensors: Could not get major number\n");
			return sensors_major;
		}
	}

	if (!sensors_class) {
		sensors_class = class_create(THIS_MODULE, "sensors");
		if (IS_ERR(sensors_class))
			return PTR_ERR(sensors_class);
		atomic_set(&device_count, 0);
	}

	return 0;
}

#define SENSORS_ATTR_RO(_name)						\
{									\
	.attr = { .name = #_name, .mode = 0444, .owner = THIS_MODULE },	\
	.show = sensors_show_property,					\
	.store = NULL,							\
}

#define SENSORS_ATTR(_name)						\
{									\
	.attr = { .name = #_name, .mode = 0666, .owner = THIS_MODULE },	\
	.show = sensors_show_property,					\
	.store = sensors_store_property,				\
}

static ssize_t sensors_show_property(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	ssize_t ret;
	struct sensors_dev *sdev = dev_get_drvdata(dev);
	const ptrdiff_t off = attr - sensors_attrs;
	union sensors_propval value;

	ret = sdev->get_property(sdev, off, &value);
	if (ret < 0) {
		if (ret != -ENODEV)
			dev_err(dev, "driver failed to report `%s' property\n",
				attr->attr.name);
		return ret;
	}

	if (off <= SENSORS_PROP_CURRENT)
		return sprintf(buf, "%d", value.intval);
	else if (off == SENSORS_PROP_SWITCH)
		return sprintf(buf, "%s", (value.intval ? "on" : "off"));
	else
		return sprintf(buf, "%s", value.strval);
}

static ssize_t sensors_store_property(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	struct sensors_dev *sdev = dev_get_drvdata(dev);
	const ptrdiff_t off = attr - sensors_attrs;
	union sensors_propval value;
	
	if (off <= SENSORS_PROP_CURRENT)
		value.intval = simple_strtol(buf,NULL,10);
	else if (off == SENSORS_PROP_SWITCH) {
		if (0 == strncmp(buf, "off", 3))
			value.intval = 0;
		else if (0 == strncmp(buf, "on", 2)) 
			value.intval = 1;
		else
			return -EINVAL;
	}
	else
		memcpy(value.strval, buf, 
		   sizeof(value.strval) > count ? count : sizeof(value.strval));

	return (ret = sdev->put_property(sdev, off, &value)) 
			? ret : count;
}

static struct device_attribute sensors_attrs[] = {
	SENSORS_ATTR(interval),
	SENSORS_ATTR(threshold),
	SENSORS_ATTR(mode),
	SENSORS_ATTR(teststep),
	SENSORS_ATTR_RO(maxrange),
	SENSORS_ATTR_RO(resolution),
	SENSORS_ATTR_RO(version),
	SENSORS_ATTR_RO(current),

	SENSORS_ATTR(switch),
	SENSORS_ATTR_RO(vendor),
	SENSORS_ATTR_RO(adj), 	/* Adjust register value */
};	

static ssize_t sensors_show_oldid_attrs(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct sensors_dev *sdev = dev_get_drvdata(dev);
        return sprintf(buf,"%x", old_sensors_id[sdev->id]);
}

static ssize_t sensors_show_type_attrs(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct sensors_dev *sdev = dev_get_drvdata(dev);
        return sprintf(buf,"%d", sdev->id);
}

static ssize_t sensors_show_name_attrs(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct sensors_dev *sdev = dev_get_drvdata(dev);
        return sprintf(buf,"%s\n", sdev->name);
}

static struct device_attribute sensors_base_attrs[] = {
	__ATTR(id, 0444, sensors_show_oldid_attrs, NULL),
	__ATTR(type, 0444, sensors_show_type_attrs, NULL),
	__ATTR(name, 0444, sensors_show_name_attrs, NULL),
};

int sensors_dev_register(struct sensors_dev *sdev)
{
	int i, j, ret = 0;
	dev_t dev;
        printk(KERN_INFO "sensors_dev_register\n");        

	if (!sdev->fops) {
		printk(KERN_WARNING "Sensors register: No file operations!\n");
		return -EINVAL;
	}

	if (!sensors_class) {
		ret = create_sensors_class();
		if (ret < 0)
			return ret;
	}

	INIT_LIST_HEAD(&sdev->list);
	mutex_lock(&sensors_mutex);

	sdev->minor = atomic_read(&device_count);
	dev = MKDEV(sensors_major, sdev->minor);
	printk("dev=%d,sensor%d\n",dev,sdev->minor);
	sdev->dev = device_create(sensors_class, NULL, dev, NULL, "sensor%d", sdev->minor);
	if (IS_ERR(sdev->dev)) {
		ret = PTR_ERR(sdev->dev);
		goto dev_create_failed;
	}

	atomic_inc(&device_count);
	dev_set_drvdata(sdev->dev, sdev);
/*
	if (!(sdev->id < SENSORS_MASK))
		goto device_id_invalid; 
*/
	for (i = 0; i < ARRAY_SIZE(sensors_base_attrs); i++) {
		ret = device_create_file(sdev->dev, &sensors_base_attrs[i]);
		if (ret)
			goto base_attrs_create_failed;
	}

	for (j = 0; j < sdev->num_properties; j++) {
		ret = device_create_file(sdev->dev,
				&sensors_attrs[sdev->properties[j]]);
		if (ret)
			goto attrs_create_failed;
	}

	list_add(&sdev->list, &sensors_list);
	mutex_unlock(&sensors_mutex);

	return ret;

attrs_create_failed:
	while (j--)
		device_remove_file(sdev->dev, 
				&sensors_attrs[sdev->properties[j]]);
base_attrs_create_failed:
	while (i--)
		device_remove_file(sdev->dev, &sensors_base_attrs[i]);
device_id_invalid:
	device_destroy(sensors_class, MKDEV(sensors_major, sdev->minor));
dev_create_failed:
	return ret;
}
EXPORT_SYMBOL_GPL(sensors_dev_register);

int sensors_dev_unregister(struct sensors_dev *sdev)
{
	int i, j;
        printk(KERN_INFO "sensors_dev_unregister\n");        

	if (list_empty(&sdev->list))
		return -EINVAL;
        
	mutex_lock(&sensors_mutex);
	list_del(&sdev->list);

	for (i = 0; i < ARRAY_SIZE(sensors_base_attrs); i++)
		device_remove_file(sdev->dev, &sensors_base_attrs[i]);

	for (j = 0; j < sdev->num_properties; j++) {
		device_remove_file(sdev->dev, 
				&sensors_attrs[sdev->properties[j]]);
	}

	dev_set_drvdata(sdev->dev, NULL);
	device_destroy(sensors_class, MKDEV(sensors_major, sdev->minor));

	mutex_unlock(&sensors_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(sensors_dev_unregister);

static int __init sensors_class_init(void)
{

	return create_sensors_class();
}

static void __exit sensors_class_exit(void)
{
	class_destroy(sensors_class);
}

module_init(sensors_class_init);
module_exit(sensors_class_exit);

MODULE_AUTHOR("Ru Yi <yi.ru@borqs.com>");
MODULE_DESCRIPTION("Sensors class driver");
MODULE_LICENSE("GPL");
