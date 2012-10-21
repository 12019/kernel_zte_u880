#ifndef __ALSPS_H__
#define __ALSPS_H__
#include <linux/types.h>

#define AMBIENT_ENABLE		(1 << 0)
#define PROXIMITY_ENABLE	(1 << 1)
struct tsl2771_alsps_platform_data {
	int irq;
};
#endif
