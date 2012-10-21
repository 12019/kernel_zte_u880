/*
 *
 * Copyright (C) 2006, Marvell Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_TC358762_H
#define __MACH_TC358762_H

struct tc358762_platform_data {
	int		(*platform_init)(void);
};

#endif

