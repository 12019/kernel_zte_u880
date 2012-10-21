/*
 * A V4L2 driver for OmniVision OV5640 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <linux/i2c.h>
#include <mach/hardware.h>
#include <mach/camera.h>
#include <mach/mfp.h>

#include <linux/clk.h>
#include "pxa910_camera.h"

MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov5640 sensors");
MODULE_LICENSE("GPL");

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define FULL_5M_WIDTH	2592
#define FULL_5M_HEIGHT  1944

#define QXGA_WIDTH	2048
#define QXGA_HEIGHT	1536
#define UXGA_WIDTH	1600
#define UXGA_HEIGHT     1200
#define XGA_WIDTH	1024
#define XGA_HEIGHT      768
#define D1_WIDTH	720
#define D1_HEIGHT      480
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/*for OV5640 porting*/
#define REG_CLKRC	0x3011
#define REG_PIDH        0x300a
#define REG_PIDL        0x300b
#define REG_SYS		0x3008
#define SYS_RESET	0x80
/*
 * Information we maintain about a known sensor.
 */
struct ov5640_format_struct;  /* coming later */
struct ov5640_info {
	struct ov5640_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */

struct regval_list {
	u16 reg_num;
	unsigned char value;
};

static struct regval_list ov5640_common_initial[]=
{
	{0x3008 , 0x82},
	{0x3008 , 0x42},
	{0x3103 , 0x03},
	{0x3017 , 0x00},
	{0x3018 , 0x00},
	{0x3630 , 0x2e},
	{0x3632 , 0xe2},
	{0x3633 , 0x23},
	{0x3634 , 0x44},
	{0x3621 , 0xe0},
	{0x3704 , 0xa0},
	{0x3703 , 0x5a},
	{0x3715 , 0x78},
	{0x3717 , 0x01},
	{0x370b , 0x60},
	{0x3705 , 0x1a},
	{0x3905 , 0x02},
	{0x3906 , 0x10},
	{0x3901 , 0x0a},
	{0x3731 , 0x12},
	{0x3600 , 0x04},
	{0x3601 , 0x22},
	{0x471c , 0x50},
	{0x3a18 , 0x00},
	{0x3a19 , 0xf8},
	{0x3503 , 0x07},
	{0x3500 , 0x00},
	{0x3501 , 0x01},
	{0x3502 , 0x00},
	{0x350a , 0x00},
	{0x350b , 0x3f},
	{0x3002 , 0x00},
	{0x3006 , 0xff},
	{0x300e , 0x45},
	{0x302e , 0x08},
	{0x3612 , 0x4b},
	{0x3618 , 0x04},
	{0x3034 , 0x18},
	{0x3035 , 0x12},
	{0x3036 , 0x54},
	{0x3708 , 0x21},
	{0x3709 , 0x12},
	{0x370c , 0x00},
	{0x3800 , 0x00},
	{0x3801 , 0x00},
	{0x3802 , 0x00},
	{0x3803 , 0x00},
	{0x3804 , 0x0a},
	{0x3805 , 0x3f},
	{0x3806 , 0x07},
	{0x3807 , 0x9f},
	{0x3808 , 0x0a},
	{0x3809 , 0x20},
	{0x380a , 0x07},
	{0x380b , 0x98},
	{0x380c , 0x0b},
	{0x380d , 0x1c},
	{0x380e , 0x07},
	{0x380f , 0xb0},
	{0x3810 , 0x00},
	{0x3811 , 0x10},
	{0x3812 , 0x00},
	{0x3813 , 0x06},
	{0x3814 , 0x11},
	{0x3815 , 0x11},
	{0x3820 , 0x40},
	{0x3821 , 0x26},
	{0x3824 , 0x04},
	{0x3a02 , 0x07},
	{0x3a03 , 0xb0},
	{0x3a08 , 0x01},
	{0x3a09 , 0x27},
	{0x3a0a , 0x00},
	{0x3a0b , 0xf6},
	{0x3a0e , 0x06},
	{0x3a0d , 0x08},
	{0x3a14 , 0x07},
	{0x3a15 , 0xb0},
	{0x4001 , 0x02},
	{0x4004 , 0x06},
	{0x4300 , 0x30},
	{0x460b , 0x35},
	{0x460c , 0x22},
	{0x4713 , 0x02},
	{0x4750 , 0x00},
	{0x4751 , 0x00},
	{0x5000 , 0x07},
	{0x5001 , 0x03},
	{0x501d , 0x00},
	{0x501f , 0x00},
	{0x5684 , 0x10},
	{0x5685 , 0xa0},
	{0x5686 , 0x0c},
	{0x5687 , 0x78},
	{0x5a00 , 0x08},
	{0x5a21 , 0x00},
	{0x5a24 , 0x00},
	{0x5000 , 0x27},
	{0x5001 , 0x83},
	{0x3821 , 0x26},
	{0x5481 , 0x08},
	{0x5482 , 0x14},
	{0x5483 , 0x28},
	{0x5484 , 0x51},
	{0x5485 , 0x65},
	{0x5486 , 0x71},
	{0x5487 , 0x7d},
	{0x5488 , 0x87},
	{0x5489 , 0x91},
	{0x548a , 0x9a},
	{0x548b , 0xaa},
	{0x548c , 0xb8},
	{0x548d , 0xcd},
	{0x548e , 0xdd},
	{0x548f , 0xea},
	{0x5490 , 0x1d},
	{0x5381 , 0x20},
	{0x5382 , 0x64},
	{0x5383 , 0x08},
	{0x5384 , 0x20},
	{0x5385 , 0x80},
	{0x5386 , 0xa0},
	{0x5387 , 0xa2},
	{0x5388 , 0xa0},
	{0x5389 , 0x02},
	{0x538a , 0x01},
	{0x538b , 0x98},
	{0x5300 , 0x08},
	{0x5301 , 0x30},
	{0x5302 , 0x10},
	{0x5303 , 0x00},
	{0x5304 , 0x08},
	{0x5305 , 0x30},
	{0x5306 , 0x08},
	{0x5307 , 0x16},
	{0x5580 , 0x02},
	{0x5583 , 0x40},
	{0x5584 , 0x10},
	{0x5589 , 0x10},
	{0x558a , 0x00},
	{0x558b , 0xf8},
	{0x3a0f , 0x36},
	{0x3a10 , 0x2e},
	{0x3a1b , 0x38},
	{0x3a1e , 0x2c},
	{0x3a11 , 0x70},
	{0x3a1f , 0x18},
	{0x3a18 , 0x00},
	{0x3a19 , 0xf8},
	{0x3003 , 0x03},
	{0x3003 , 0x01},
	{0x3503 , 0x00},
	{0x3035 , 0x12},//;21
	{0x4800 , 0x24}, //clk high
	{0x3a08 , 0x00},
	{0x3a09 , 0xec},
	{0xffff , 0xff},
};

/* need to verify */
static struct regval_list ov5640_fmt_yuv422_qcif[] = {
	{0x3108,0x16},
	{0x3821,0x06},//
	{0x3824,0x01},//
	{0x3808,0x00},//
	{0x3809,0xb0},//
	{0x380a,0x00},//
	{0x380b,0x90},//
	{0x5001,0x23},// ;isp scale down en
	{0x4300,0x32},
	{0xffff,0xff},
};

/* need to verify */
static struct regval_list ov5640_fmt_yuv422_cif[] =
{
	{0x3108,0x16},
	{0x3821,0x06},//
	{0x3824,0x01},//
	{0x3808,0x01},//
	{0x3809,0x60},//
	{0x380a,0x01},//
	{0x380b,0x20},//
	{0x5001,0x23},// ;isp scale down en
	{0x4300,0x32},
	{0xffff,0xff},
};

static struct regval_list ov5640_fmt_yuv422_qvga[] =
{
	{0x3108,0x16},
        {0x3821,0x06},
        {0x3824,0x01},
        {0x3808,0x01},
        {0x3809,0x40},
        {0x380a,0x00},
        {0x380b,0xF0},
        {0x5001,0x23}, //;isp scale down en
        {0x4300,0x32},
        {0xffff,0xff},
};

static struct regval_list ov5640_fmt_yuv422_vga[] =
{
	{0x3108,0x16},
	{0x3821,0x06},//
	{0x3824,0x01},//
	{0x3808,0x02},//
	{0x3809,0x80},//
	{0x380a,0x01},//
	{0x380b,0xe0},//
	{0x5001,0x23},// ;isp scale down en
	{0x4300,0x32},
	{0xffff,0xff},
};

static struct regval_list ov5640_fmt_yuv422_d1[] =
{
	{0x3108,0x15},
	{0x3821,0x06},//
	{0x3824,0x01},//
	{0x3808,0x02},//
	{0x3809,0xd0},//
	{0x380a,0x01},//
	{0x380b,0xe0},//
	{0x5001,0x23},// ;isp scale down en
	{0x4300,0x32},
	{0xffff,0xff},
};
static struct regval_list ov5640_fmt_jpeg_5M[] =
{
	{0x3108,0x16},
	{0x3821,0x26},
	{0x3824,0x04},
	{0x3808,0x0a},
	{0x3809,0x20},
	{0x380a,0x07},
	{0x380b,0x98},
	{0x5001,0x83},
	{0x4300,0x30},
	{0xffff,0xff},
};
/*
 * Low-level register I/O.
 */
/*issue that OV sensor must write then read. 5640 register is 16bit!!!*/
static int ov5640_read(struct i2c_client *c, u16 reg,
		unsigned char *value)
{
	u8 data;
	u8 address[2];
	address[0] = reg>>8;
	address[1] = reg;         
	i2c_smbus_write_byte_data(c,address[0],address[1]);
	data = i2c_smbus_read_byte(c);
	*value = data;
	return 0;
}


static int ov5640_write(struct i2c_client *c, u16 reg,
		unsigned char value)
{
	u8 data[3];
	data[0] = reg>>8;
	data[1] = reg;
	data[2]=  value;
	i2c_master_send(c, data, 3);
	if (reg == REG_SYS && (value & SYS_RESET))
		msleep(2);  /* Wait for reset to run */
	return 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov5640_write_array(struct i2c_client *c, struct regval_list *vals)
{
	int i = 0;
	while (vals->reg_num != 0xffff || vals->value != 0xff) {
		int ret = ov5640_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		if (i == 0)
			mdelay(5);
		i++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */
static void ov5640_reset(struct i2c_client *client)
{
	ov5640_write(client, REG_SYS, SYS_RESET);
	msleep(1);
}

static int ov5640_detect(struct i2c_client *client)
{
	unsigned char v;
	int ret;

	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov5640_read(client, REG_PIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x56){
		return -ENODEV;
	}
	ret = ov5640_read(client, REG_PIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0x40){
		return -ENODEV;
	}
	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct ov5640_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	int bpp;   /* bits per pixel */
} ov5640_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= ov5640_fmt_yuv422_vga,
		.bpp		= 16,
	},
	{
		.desc		= "YUYV422 planar",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.regs 		= ov5640_fmt_yuv422_vga,
		.bpp		= 16,
	},
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.regs           = ov5640_fmt_yuv422_vga,
		.bpp            = 12,
	},
	{
		.desc           = "JFIF JPEG",
		.pixelformat    = V4L2_PIX_FMT_JPEG,
		.regs           = ov5640_fmt_jpeg_5M,
		.bpp            = 16,
	},
};
#define N_OV5640_FMTS ARRAY_SIZE(ov5640_formats)

/*TODO - also can use ccic size register 0x34 to do same thing for cropping...anyway, sensor doing it is better?
  0x3020~0x3027*/
static struct ov5640_win_size {
	int	width;
	int	height;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	/* h/vref stuff */
} ov5640_win_sizes[] = {
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.hstart		= 456,		/* Empirically determined */
		.hstop		=  24,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
		//		.regs 		= ov5640_qcif_regs,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.hstart		= 164,		/* Empirically determined */
		.hstop		=  20,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
	},
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
		.hstart		= 170,		/* Empirically determined */
		.hstop		=  90,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
	},
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.hstart		= 158,		/* These values from */
		.hstop		=  14,		/* Omnivision */
		.vstart		=  10,
		.vstop		= 490,
		.regs 		= NULL,
	},

	/* D1  */
	{
		.width		= D1_WIDTH,
		.height		= D1_HEIGHT,
		.regs 		= NULL,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(ov5640_win_sizes))

static int ov5640_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct ov5640_format_struct *ofmt;

	if (fmt->index >= N_OV5640_FMTS)
		return -EINVAL;

	ofmt = ov5640_formats + fmt->index;
	fmt->flags = 0;
	strncpy(fmt->description, ofmt->desc, 32);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int ov5640_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct ov5640_format_struct **ret_fmt,
		struct ov5640_win_size **ret_wsize)
{
	int index;
	struct ov5640_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	for (index = 0; index < N_OV5640_FMTS; index++)
		if (ov5640_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_OV5640_FMTS){
		printk("%d unsupported format!\n", __LINE__);
		return -EINVAL;
	}
	if (ret_fmt != NULL)
		*ret_fmt = ov5640_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
		return -EINVAL;
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = ov5640_win_sizes; wsize < ov5640_win_sizes + N_WIN_SIZES;
			wsize++)
		if (pix->width <= wsize->width && pix->height <= wsize->height)
			break;
	if (wsize >= ov5640_win_sizes + N_WIN_SIZES){
		printk("size exceed and set as QXGA!\n");
		wsize--;   /* Take the smallest one */
	}
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
#if 0
	pix->width = wsize->width;
	pix->height = wsize->height;
#endif
	pix->bytesperline = pix->width*ov5640_formats[index].bpp/8;
	pix->sizeimage = pix->height*pix->bytesperline;
	printk("ov5640_try_fmt: pix->width is %d, pix->height is %d wsize %d\n", pix->width, pix->height, wsize->width);

	if (ret_fmt == NULL)
		return 0;
	switch (pix->pixelformat)
	{
	case V4L2_PIX_FMT_YUYV: 
	case V4L2_PIX_FMT_YUV422P:
	case V4L2_PIX_FMT_YUV420: 
		switch (pix->width)
		{
		case QCIF_WIDTH:
			(*ret_fmt)->regs = ov5640_fmt_yuv422_qcif;
			break;
		case CIF_WIDTH:
			(*ret_fmt)->regs = ov5640_fmt_yuv422_cif;
			break;
		case QVGA_WIDTH:
			(*ret_fmt)->regs = ov5640_fmt_yuv422_qvga;
			break;
		case VGA_WIDTH:
			(*ret_fmt)->regs = ov5640_fmt_yuv422_vga;
			break;
		case D1_WIDTH:
			(*ret_fmt)->regs = ov5640_fmt_yuv422_d1;
			break;
		default:
			printk("unsupported size!\n");
			return -EINVAL;
		}
		break;
	case V4L2_PIX_FMT_JPEG:
		(*ret_fmt)->regs = ov5640_fmt_jpeg_5M;
		break;
	default:
		(*ret_fmt)->regs = ov5640_fmt_jpeg_5M;
		break;
	}
	return 0;
}

/*
 * Set a format.
 */
static int ov5640_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
	struct ov5640_format_struct *ovfmt;
	struct ov5640_win_size *wsize;
	ret = ov5640_try_fmt(c, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	ov5640_write_array(c, ov5640_common_initial);
	ov5640_write_array(c, ovfmt->regs);
	//	ov5640_set_hw();	//TODO
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov5640_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov5640_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov5640_s_input(struct i2c_client *c, int *id)
{
	return 0;
}

int ccic_sensor_attach(struct i2c_client *client);

/*
 * Basic i2c stuff.
 */
static int __devinit ov5640_probe(struct i2c_client *client, const struct i2c_device_id * i2c_id)
{
	int ret = 0;
	struct ov5640_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;

	client->addr = 0x3C;
	ccic_set_clock_mipi();

	pdata->power_on(1, 1);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct ov5640_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &ov5640_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);
	/*
	 * Make sure it's an ov5640
	 */
	ret = ov5640_detect(client);
	if (ret)
		goto out_free;
	printk(KERN_NOTICE "OmniVision ov5640 sensor detected\n");
	ret = ccic_sensor_attach(client);
	if (ret)
		goto out_free;

out_free:
	pdata->power_on(0, 1);
	ccic_disable_clock();

	if(ret)
		kfree(info);
	return ret;
}


static int ov5640_remove(struct i2c_client *client)
{
	return 0;	//TODO
}


static int ov5640_streamon(struct i2c_client *client)
{
	unsigned char val;
	ov5640_read(client, REG_SYS, &val);
	val &= ~0x40;
	ov5640_write(client, REG_SYS, val);
	return 0;
}

static int ov5640_streamoff(struct i2c_client *client)
{
	unsigned char val;
	ov5640_read(client, REG_SYS, &val);
	val |= 0x40;
	ov5640_write(client, REG_SYS, val);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5640_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov5640_read(client, (u16)reg->reg, (unsigned char *)&(reg->val));
}

static int ov5640_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov5640_write(client, (u16)reg->reg, (unsigned char)reg->val);
}
#endif
static int ov5640_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_OV5640, 0);
		case VIDIOC_INT_RESET:
			ov5640_reset(client);
			return 0;
		case VIDIOC_ENUM_FMT:
			return ov5640_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_TRY_FMT:
			return ov5640_try_fmt(client, (struct v4l2_format *) arg, NULL, NULL);
		case VIDIOC_S_FMT:
			return ov5640_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_S_PARM:
			return ov5640_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return ov5640_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return ov5640_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return ov5640_streamon(client);
		case VIDIOC_STREAMOFF:
			return ov5640_streamoff(client);
#ifdef CONFIG_VIDEO_ADV_DEBUG
		case VIDIOC_DBG_G_REGISTER:
			return ov5640_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return ov5640_s_register(client, (struct v4l2_dbg_register *) arg);
#endif
	}
	return -EINVAL;
}

static struct i2c_device_id ov5640_idtable[] = {
	{ "ov5640", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov5640_idtable);

static struct i2c_driver ov5640_driver = {
	.driver = {
		.name	= "ov5640",
	},
	.id_table       = ov5640_idtable,
	.command	= ov5640_command,
	.probe		= ov5640_probe,
	.remove		= ov5640_remove,
};


/*
 * Module initialization
 */
static int __init ov5640_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision ov5640 sensor driver, at your service\n");
	return i2c_add_driver(&ov5640_driver);
}

static void __exit ov5640_mod_exit(void)
{
	i2c_del_driver(&ov5640_driver);
}

late_initcall(ov5640_mod_init);
module_exit(ov5640_mod_exit);

