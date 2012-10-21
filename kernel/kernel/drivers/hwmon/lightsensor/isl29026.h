/******************************************************************************
 * isl29028.h - Linux kernel module for Intersil ISL29028 ambient light sensor
 *		and proximity sensor
 *
 * Copyright 2008-2010 Intersil Inc..
 *
 * DESCRIPTION:
 *	- This is the linux driver for ISL29028 and passed the test under the Linux
 *	Kernel version 2.6.30.4
 *
 * modification history
 * --------------------
 * v1.0   2010/04/06, Shouxian Chen(Simon Chen) create this file

 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/


#ifndef __ISL29028_H__
#define __ISL29028_H__


#define ISL29028_ADDR	0x44
#define	ISL29028_MAJOR	250
#define	DEVICE_NAME		"isl29028"
#define	DRIVER_VERSION	"1.0"

/* IOCTL cmd define */
/*
#define WR_CMD1			0x0
#define WR_CMD2			0x1
#define RD_CMD1			0x2
#define RD_CMD2			0x3
#define	RD_DATA			0x4
*/
#define RD_CHIPID		0x0
#define RD_CFG_REG		0x1
#define RD_INT_REG		0x2
#define RD_PROX_TH_L	0x3
#define RD_PROX_TH_H	0x4
#define RD_ALSIR_TH_L	0x5
#define RD_ALSIR_TH_H	0x6
#define RD_PROX_DATA	0x7
#define RD_ALSIR_DATA	0x8
#define WR_CFG_REG		0x9
#define WR_INT_REG		0xa
#define WR_PROX_TH_L	0xb
#define WR_PROX_TH_H	0xc
#define WR_ALSIR_TH_L	0xd
#define WR_ALSIR_TH_H	0xe

#define CHIPID_REG		0x00
#define CFG_REG			0x01
#define PROX_DATA		0x08
#define ALS_DATA_LOW	0x09
#define ALS_DATA_HIGH	0x0A

/* Each client has this additional data */
struct isl29028_data_t 
{
	spinlock_t	lock;
	u8	minor;
	u8	dev_open_cnt;
	struct i2c_client* client;
	u8	pwr_status;
};

#endif
