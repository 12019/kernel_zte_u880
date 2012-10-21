/*
 *  Sensors class driver
 *
 * Copyright (C) 2008 Borqs, Inc.
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

#ifndef __LINUX_SENSORS_H__
#define __LINUX_SENSORS_H__

/*Ugly Design: New types on Eclair,sdev->id*/
#define SENSORS_ACCELERATION		(1)
#define SENSORS_MAGNETIC_FIELD		(2)
#define SENSORS_ORIENTATION		(3)
#define SENSORS_GYROSCOPE           	(4)
#define SENSORS_LIGHT			(5)
#define SENSOR_TYPE_PRESSURE            (6)
#define SENSORS_TEMPERATURE		(7)
#define SENSORS_PROXIMITY		(8)
#define SENSORS_TRICORDER		(9)
#define SENSORS_HALL			(10)
#define SENSORS_MASK			(11)

#if 0
/*Old id list, translation from type now!!!*/
#define SENSORS_ORIENTATION		(1 << 0)
#define SENSORS_ACCELERATION		(1 << 1)
#define SENSORS_TEMPERATURE		(1 << 2)
#define SENSORS_MAGNETIC_FIELD		(1 << 3)
#define SENSORS_LIGHT			(1 << 4)
#define SENSORS_PROXIMITY		(1 << 5)
#define SENSORS_TRICORDER		(1 << 6)
#define SENSORS_HALL			(1 << 7)
#define SENSORS_MASK			0xFF 
#endif

#define DEFAULT_ID		(0)

enum sensors_property {
	/* int */
	SENSORS_PROP_INTERVAL = 0,
	SENSORS_PROP_THRESHOLD,
	SENSORS_PROP_MODE,
	SENSORS_PROP_TEST_STEP,
	SENSORS_PROP_MAXRANGE,		/* read only */
	SENSORS_PROP_RESOLUTION,	/* read only */
	SENSORS_PROP_VERSION,		/* read only */
	SENSORS_PROP_CURRENT,		/* read only */

	/* char */
	SENSORS_PROP_SWITCH,
	SENSORS_PROP_VENDOR,		/* read only */
	SENSORS_PROP_REGISTER,		/* read only */
};

#define PRO_STR_SIZE		20	
union sensors_propval {
	int intval;
	char strval[PRO_STR_SIZE];
};

struct sensors_dev {
	int		minor;
	unsigned int	id;
	const char	*name;
	size_t num_properties;
	enum sensors_property *properties;
	int (*get_property)(struct sensors_dev *sdev,
			    enum sensors_property property,
			    union sensors_propval *val);
	int (*put_property)(struct sensors_dev *sdev,
			    enum sensors_property property,
			    union sensors_propval *val);

	struct file_operations	*fops;
	struct list_head list;
	struct device	*dev;
};

extern int sensors_dev_register(struct sensors_dev *sdev);
extern int sensors_dev_unregister(struct sensors_dev *sdev);

#endif /* __LINUX_SENSORS_H__ */
