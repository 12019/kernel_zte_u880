/*
 * A V4L2 driver for OmniVision OV3640 cameras.
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
#include <linux/clk.h>

#include "pxa910_camera.h"
#include "ov3640_mipi.h"

MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("A low-level driver for OmniVision ov3640 sensors");
MODULE_LICENSE("GPL");

/*
 * Low-level register I/O.
 */
/*issue that OV sensor must write then read. 3640 register is 16bit!!!*/
static int ov3640_read(struct i2c_client *c, u16 reg,
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


static int ov3640_write(struct i2c_client *c, u16 reg,
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
static int ov3640_write_array(struct i2c_client *c, struct regval_list *vals)
{
	int i = 0;
	while (vals->reg_num != 0xffff || vals->value != 0xff) {
		int ret = ov3640_write(c, vals->reg_num, vals->value);
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
static void ov3640_reset(struct i2c_client *client)
{
	ov3640_write(client, REG_SYS, SYS_RESET);
	msleep(1);
}

static int ov3640_detect(struct i2c_client *client)
{
	unsigned char v;
	int ret;

	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov3640_read(client, REG_PIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x36)
		return -ENODEV;
	return 0;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct ov3640_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	int bpp;   /* bits per pixel */
} ov3640_formats[] = {
#if 0
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.bpp		= 16,
	},
	{
		.desc		= "YUYV422 planar",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.bpp		= 16,
	},
#endif
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.bpp            = 12,
	},
	{
		.desc           = "JFIF JPEG",
		.pixelformat    = V4L2_PIX_FMT_JPEG,
		.bpp            = 16,
	},
#if 0
	{
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.bpp		= 16,
	},
	{
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.bpp		= 16,
	},
	{
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.bpp		= 8,
	},
#endif
};
#define N_OV3640_FMTS ARRAY_SIZE(ov3640_formats)

/*TODO - also can use ccic size register 0x34 to do same thing for cropping...anyway, sensor doing it is better?
  0x3020~0x3027*/
static struct ov3640_win_size {
	int	width;
	int	height;
	/* h/vref stuff */
} ov3640_win_sizes[] = {
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
	},
#if 0
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
	},
#endif
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
	},
	/* D1 */
	{
		.width          = D1_WIDTH,
		.height         = D1_HEIGHT,
	},
};

/* capture jpeg size */
static struct ov3640_win_size ov3640_win_sizes_jpeg[] = {
	/* full */
	{
		.width = QXGA_WIDTH,
		.height = QXGA_HEIGHT,
	},
	{
		.width = 640,
		.height = 480,
	},
};

static int ov3640_querycap(struct i2c_client *c, struct v4l2_capability *argp)
{
	if(!argp){
		printk(KERN_ERR" argp is NULL %s %d \n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	strcpy(argp->driver, "ov3640");
	strcpy(argp->card, "TD/TTC");
	return 0;
}

static int ov3640_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct ov3640_format_struct *ofmt;

	if (fmt->index >= ARRAY_SIZE(ov3640_formats))
        {
		printk("NO such fmt->index\n");
		return -EINVAL;
	}
	ofmt = ov3640_formats + fmt->index;
	fmt->flags = 0;
	strncpy(fmt->description, ofmt->desc, 32);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int ov3640_enum_fmsize(struct i2c_client *c, struct v4l2_frmsizeenum *argp)
{
	struct v4l2_frmsizeenum frmsize;

	if (copy_from_user(&frmsize, argp, sizeof(frmsize)))
		   return -EFAULT;

	if (frmsize.pixel_format == V4L2_PIX_FMT_YUV420){
		if (frmsize.index >= (ARRAY_SIZE(ov3640_win_sizes))
){
		    return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = ov3640_win_sizes[frmsize.index].height;
		frmsize.discrete.width = ov3640_win_sizes[frmsize.index].width;
	}else if(frmsize.pixel_format == V4L2_PIX_FMT_JPEG){
		if (frmsize.index >= ARRAY_SIZE(ov3640_win_sizes_jpeg)){
			   return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = ov3640_win_sizes_jpeg[frmsize.index].height;
		frmsize.discrete.width = ov3640_win_sizes_jpeg[frmsize.index].width;

	}else
	   return -EINVAL;

	if (copy_to_user(argp, &frmsize, sizeof(frmsize)))
		   return -EFAULT;
	return 0;
}

static int ov3640_try_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int index = 0;
	int i = 0;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	for (index = 0; index < ARRAY_SIZE(ov3640_formats); index++)
		if (ov3640_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= ARRAY_SIZE(ov3640_formats))
	{
		printk(KERN_ERR"unsupported format!\n");
		return -EINVAL;
	}

	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
	{
		printk(KERN_ERR"pix->filed != V4l2_FIELD_NONE\n");
		return -EINVAL;
	}
	if(pix->pixelformat == V4L2_PIX_FMT_JPEG){
		for (i = 0; i < ARRAY_SIZE(ov3640_win_sizes_jpeg); i++)
			if (pix->width == ov3640_win_sizes_jpeg[i].width && pix->height == ov3640_win_sizes_jpeg[i].height)
				break;

		if (i >= ARRAY_SIZE(ov3640_win_sizes_jpeg)){
				printk(KERN_ERR"invalid size request for jpeg! %d %d  \n",
					pix->width, pix->height);
				return -EINVAL;
			}
		/* for OV5642, HSYNC contains 2048bytes */
		pix->bytesperline = 2048;
		printk(KERN_ERR"change jpeg width as w %d h %d \n",pix->width,pix->height );
	}else{
		for (i = 0; i < ARRAY_SIZE(ov3640_win_sizes);i ++)
			if (pix->width == ov3640_win_sizes[i].width && pix->height == ov3640_win_sizes[i].height)
				break;

		if (i>= ARRAY_SIZE(ov3640_win_sizes)){
				printk(KERN_ERR"invalid size request for preview! %d %d  \n",
					pix->width, pix->height);
				return -EINVAL;
			}
		pix->bytesperline = pix->width*ov3640_formats[index].bpp/8;
		pix->sizeimage = pix->height*pix->bytesperline;
	}
	return 0;
}

/*
 * Set a format.
 */
static int ov3640_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
	struct regval_list    *pregs;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	ret = ov3640_try_fmt(c, fmt);
	if (ret < 0)
	{
		printk("try fmt error\n");
		return ret;
	}

	switch(pix->pixelformat)
	{
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV420:
			switch (pix->width )
			{
				case QCIF_WIDTH:
					pregs = ov3640_fmt_yuv422_qcif;
					printk("choose qcif setting \n");
					break;	
				case QVGA_WIDTH:
					pregs = ov3640_fmt_yuv422_qvga;
					printk("choose qvga setting \n");
					break;
				case VGA_WIDTH:
					pregs = ov3640_fmt_yuv422_vga;
					printk("choose vga setting \n");
					break;
				case 720:
					pregs = ov3640_fmt_yuv422_d1;
					printk("choose D1 setting \n");
					break;
				default:
					printk("unsupported YUV format !\n");
					ret = -EINVAL;
					goto out;
					break;
			}
			break;
		case V4L2_PIX_FMT_JPEG:
			switch (pix->width)
			{
				case VGA_WIDTH:
					pregs = ov3640_fmt_jpeg_vga;
					printk("choose vga jpeg setting \n");
					break;
				case QXGA_WIDTH:
					pregs = ov3640_fmt_jpeg_qxga;
					printk("choose 3M jpeg setting \n");
					break;
				default:
					printk("unsupported JPEG size!\n");
					ret = -EINVAL;
					goto out;
					break;
			}
			break;
		default:
			printk("unsupported format!\n");
			ret = -EINVAL;
			goto out;
			break;
	}	
	
	ov3640_write_array(c, pregs);
out:
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov3640_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	unsigned char clkrc;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	ret = ov3640_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = OV3640_FRAME_RATE;
	if ((clkrc & CLK_EXT) == 0 && (clkrc & CLK_SCALE) > 1)
		cp->timeperframe.denominator /= (clkrc & CLK_SCALE);
	return 0;
}

static int ov3640_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	unsigned char clkrc;
	int ret, div;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;
	/*
	 * CLKRC has a reserved bit, so let's preserve it.
	 */
	ret = ov3640_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator*OV3640_FRAME_RATE)/tpf->denominator;
	if (div == 0)
		div = 1;
	else if (div > CLK_SCALE)
		div = CLK_SCALE;
	clkrc = (clkrc & 0x80) | div;
	tpf->numerator = 1;
	tpf->denominator = OV3640_FRAME_RATE/div;
	return ov3640_write(c, REG_CLKRC, clkrc);
}

static int ov3640_s_input(struct i2c_client *c, int *id)
{
	return 0;
}

/*
 * Code for dealing with controls.
 */


/*TODO - need to port below register codes for 3640...maybe not used*/

#if 0
static int ov3640_store_cmatrix(struct i2c_client *client,
		int matrix[CMATRIX_LEN])
{
	int i, ret;
	unsigned char signbits;

	/*
	 * Weird crap seems to exist in the upper part of
	 * the sign bits register, so let's preserve it.
	 */
	ret = ov3640_read(client, REG_CMATRIX_SIGN, &signbits);
	signbits &= 0xc0;

	for (i = 0; i < CMATRIX_LEN; i++) {
		unsigned char raw;

		if (matrix[i] < 0) {
			signbits |= (1 << i);
			if (matrix[i] < -255)
				raw = 0xff;
			else
				raw = (-1 * matrix[i]) & 0xff;
		}
		else {
			if (matrix[i] > 255)
				raw = 0xff;
			else
				raw = matrix[i] & 0xff;
		}
		ret += ov3640_write(client, REG_CMATRIX_BASE + i, raw);
	}
	ret += ov3640_write(client, REG_CMATRIX_SIGN, signbits);
	return ret;
}
#endif

/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */

int g_brightness;
int g_hue;
int g_saturation;
int g_contrast;
int g_vflip;
int g_hflip;
int g_whitebalance;
int g_exposure;

static int ov3640_t_sat(struct i2c_client *client, int value)
{

	int ret = 0;
	unsigned char regval;

	ov3640_read(client, REG_BRIGHT1, &regval);
	regval |= 0x02;
	switch (value)
	{
		case 0:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x10);
			ret = ov3640_write(client, REG_SAT1, 0x10);
		break;

		case 1:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x30);
			ret = ov3640_write(client, REG_SAT1, 0x30);
		break;

		case 2:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x40);
			ret = ov3640_write(client, REG_SAT1, 0x40);
		break;

		case 3:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x50);
			ret = ov3640_write(client, REG_SAT1, 0x50);
		break;

		case 4:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			ret = ov3640_write(client, REG_SAT0, 0x70);
			ret = ov3640_write(client, REG_SAT1, 0x70);
		break;
	}
	g_saturation = value;
	return ret;
}

static int ov3640_q_sat(struct i2c_client *client, __s32 *value)
{
	*value = g_saturation;
	return 0;
}

static int ov3640_t_hue(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;
	ov3640_read(client, REG_HUE10, &regval);
	switch (value)
	{
		case 1:/*90 degree*/
			ret = ov3640_write(client, REG_HUE0, 0x20);
			ret = ov3640_write(client, REG_HUE1, 0x64);
			ret = ov3640_write(client, REG_HUE2, 0x08);
			ret = ov3640_write(client, REG_HUE3, 0x7f);
			ret = ov3640_write(client, REG_HUE4, 0x78);
			ret = ov3640_write(client, REG_HUE5, 0x06);
			ret = ov3640_write(client, REG_HUE6, 0x3d);
			ret = ov3640_write(client, REG_HUE7, 0xb6);
			ret = ov3640_write(client, REG_HUE8, 0xf3);
			ret = ov3640_write(client, REG_HUE9, 0xc8);
			regval &= ~0x2;
			ret = ov3640_write(client, REG_HUE10, regval);
		break;

		case 2:/*180 degree*/
			ret = ov3640_write(client, REG_HUE0, 0x20);
			ret = ov3640_write(client, REG_HUE1, 0x64);
			ret = ov3640_write(client, REG_HUE2, 0x08);
			ret = ov3640_write(client, REG_HUE3, 0x30);
			ret = ov3640_write(client, REG_HUE4, 0x90);
			ret = ov3640_write(client, REG_HUE5, 0xc0);
			ret = ov3640_write(client, REG_HUE6, 0xa0);
			ret = ov3640_write(client, REG_HUE7, 0x98);
			ret = ov3640_write(client, REG_HUE8, 0x08);
			ret = ov3640_write(client, REG_HUE9, 0x60);
			regval &= ~0x2;
			ret = ov3640_write(client, REG_HUE10,regval);
		break;

		case 3:/*-90 degree*/
			ret = ov3640_write(client, REG_HUE0, 0x20);
			ret = ov3640_write(client, REG_HUE1, 0x64);
			ret = ov3640_write(client, REG_HUE2, 0x08);
			ret = ov3640_write(client, REG_HUE3, 0x7f);
			ret = ov3640_write(client, REG_HUE4, 0x78);
			ret = ov3640_write(client, REG_HUE5, 0x06);
			ret = ov3640_write(client, REG_HUE6, 0x3d);
			ret = ov3640_write(client, REG_HUE7, 0xb6);
			ret = ov3640_write(client, REG_HUE8, 0xf3);
			ret = ov3640_write(client, REG_HUE9, 0x30);
			regval |= 0x2;
			ret = ov3640_write(client, REG_HUE10,regval);
		break;

		case 0:/*0 degree*/
		default:
			ret = ov3640_write(client, REG_HUE0, 0x20);
			ret = ov3640_write(client, REG_HUE1, 0x64);
			ret = ov3640_write(client, REG_HUE2, 0x08);
			ret = ov3640_write(client, REG_HUE3, 0x30);
			ret = ov3640_write(client, REG_HUE4, 0x90);
			ret = ov3640_write(client, REG_HUE5, 0xc0);
			ret = ov3640_write(client, REG_HUE6, 0xa0);
			ret = ov3640_write(client, REG_HUE7, 0x98);
			ret = ov3640_write(client, REG_HUE8, 0x08);
			ret = ov3640_write(client, REG_HUE9, 0x98);
			regval |= 0x2;
			ret = ov3640_write(client, REG_HUE10,regval);
		break;
	}
	g_hue = value;
	return ret;
}


static int ov3640_q_hue(struct i2c_client *client, __s32 *value)
{
	*value = g_hue;
	return 0;
}

static int ov3640_t_brightness(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval, regvalue;

	ov3640_read(client, REG_BRIGHT1, &regval);//55
	regval |= 0x04;
	ov3640_read(client, REG_BRIGHT2, &regvalue);//54
	switch (value)
	{
		case 0:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue |= 0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x20);
		break;

		case 1:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue |= 0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x10);
		break;

		case 3:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x10);
		break;

		case 4:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x20);
		break;

		case 2:
		default:
			ret = ov3640_write(client, REG_BRIGHT0, 0xef);
			ret = ov3640_write(client, REG_BRIGHT1, regval);
			regvalue &= ~0x08;
			ret = ov3640_write(client, REG_BRIGHT2, regvalue);
			ret = ov3640_write(client, REG_BRIGHT3, 0x00);
		break;
	}
	g_brightness = value;
	return ret;
}

static int ov3640_q_brightness(struct i2c_client *client, __s32 *value)
{
	*value = g_brightness;
	return 0;
}

static int ov3640_t_contrast(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval0, regval1;

	ov3640_read(client, REG_BRIGHT1, &regval0);//55
	ov3640_read(client, REG_BRIGHT2, &regval1);//54
	regval1 &= ~0x04;
	regval0 |= 0x04;

	switch (value)
	{
		case 0:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x08);
			ret = ov3640_write(client, REG_CONTRAS1, 0x08);
		break;

		case 1:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x18);
			ret = ov3640_write(client, REG_CONTRAS1, 0x18);
		break;

		case 3:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x40);
			ret = ov3640_write(client, REG_CONTRAS1, 0x40);
		break;

		case 4:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x80);
			ret = ov3640_write(client, REG_CONTRAS1, 0x80);
		break;

		case 2:
		default:
			ret = ov3640_write(client, REG_BRIGHT1, regval0);
			ret = ov3640_write(client, REG_BRIGHT2, regval1);
			ret = ov3640_write(client, REG_CONTRAS0, 0x20);
			ret = ov3640_write(client, REG_CONTRAS1, 0x20);
		break;
	}
	g_brightness = value;
	return ret;
}

static int ov3640_q_contrast(struct i2c_client *client, __s32 *value)
{
	*value = g_contrast;
	return 0;
}

static int ov3640_q_hflip(struct i2c_client *client, __s32 *value)
{
	*value = g_hflip;
	return 0;
}


static int ov3640_t_hflip(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval1, regval2;

	ov3640_read(client, REG_MVFP1, &regval1);
	ov3640_read(client, REG_MVFP2, &regval2);
	if (value){
		regval1 |= 0x02;
		regval2 |= 0x08;
		ret = ov3640_write(client, REG_MVFP1, regval1);
		ret = ov3640_write(client, REG_MVFP2, regval2);
	} else {
		regval1 &= ~0x02;
		regval2 &= ~0x08;
		ret = ov3640_write(client, REG_MVFP1, regval1);
		ret = ov3640_write(client, REG_MVFP2, regval2);
	}
	g_hflip = value;
	return ret;
}



static int ov3640_q_vflip(struct i2c_client *client, __s32 *value)
{
	*value = g_vflip;
	return 0;
}


static int ov3640_t_vflip(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;
	ov3640_read(client, REG_MVFP1, &regval);
	if (value){
		ret = ov3640_write(client, REG_MVFP0, 0x9);
		regval |= 0x1;
		ret = ov3640_write(client, REG_MVFP1, regval);
	} else {
		ret = ov3640_write(client, REG_MVFP0, 0xa);
		regval &= ~0x1;
		ret = ov3640_write(client, REG_MVFP1, regval);
	}
	g_vflip = value;
	return ret;
}

static int ov3640_t_whitebalance(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval;

	ov3640_read(client, REG_WB0, &regval);
	switch (value)
	{
		case 0:/*sunny*/
			regval |= 0x08;
			ret = ov3640_write(client, REG_WB0, regval);
			ret = ov3640_write(client, REG_WB1, 0x5e);
			ret = ov3640_write(client, REG_WB2, 0x40);
			ret = ov3640_write(client, REG_WB3, 0x46);
		break;

		case 1:/*cloudy*/
			regval |= 0x08;
			ret = ov3640_write(client, REG_WB0, regval);
			ret = ov3640_write(client, REG_WB1, 0x68);
			ret = ov3640_write(client, REG_WB2, 0x40);
			ret = ov3640_write(client, REG_WB3, 0x4e);
		break;

		case 3:/*office*/
			regval |= 0x08;
			ret = ov3640_write(client, REG_WB0, regval);
			ret = ov3640_write(client, REG_WB1, 0x52);
			ret = ov3640_write(client, REG_WB2, 0x40);
			ret = ov3640_write(client, REG_WB3, 0x58);
		break;

		case 4:/*home*/
			regval |= 0x08;
			ret = ov3640_write(client, REG_WB0, regval);
			ret = ov3640_write(client, REG_WB1, 0x44);
			ret = ov3640_write(client, REG_WB2, 0x40);
			ret = ov3640_write(client, REG_WB3, 0x70);
		break;

		case 2:
		default:
			regval &= ~0x08;
			ret = ov3640_write(client, REG_WB0, regval);
		break;
	}
	g_whitebalance = value;
	return ret;
}

static int ov3640_q_whitebalance(struct i2c_client *client, __s32 *value)
{
	*value = g_whitebalance;
	return 0;
}

static int ov3640_t_exposure(struct i2c_client *client, int value)
{
	int ret;
	unsigned char regval0;

	ov3640_read(client, REG_EXPOSURE0, &regval0);
	regval0 &= ~0x80;
	ov3640_write(client, REG_EXPOSURE0, regval0);
	switch (value)
	{
		case 0:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x10);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x08);
			ret = ov3640_write(client, REG_EXPOSURE3, 0x21);
		break;

		case 1:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x20);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x18);
			ret = ov3640_write(client, REG_EXPOSURE3, 0x41);
		break;

		case 3:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x48);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x40);
			ret = ov3640_write(client, REG_EXPOSURE3, 0x81);
		break;

		case 4:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x60);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x58);
			ret = ov3640_write(client, REG_EXPOSURE3, 0xa1);
		break;

		case 2:
		default:
			ret = ov3640_write(client, REG_EXPOSURE1, 0x28);
			ret = ov3640_write(client, REG_EXPOSURE2, 0x20);
			ret = ov3640_write(client, REG_EXPOSURE3, 0x51);
		break;
	}
	g_exposure = value;
	return ret;
}

static int ov3640_q_exposure(struct i2c_client *client, __s32 *value)
{
	*value = g_exposure;
	return 0;
}


static struct ov3640_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} ov3640_controls[] =
{
	{
		.qc = {
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_brightness,
		.query = ov3640_q_brightness,
	},
	{
		.qc = {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,   /* XXX ov3640 spec */
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_contrast,
		.query = ov3640_q_contrast,
	},
	{
		.qc = {
			.id = V4L2_CID_SATURATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Saturation",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_sat,
		.query = ov3640_q_sat,
	},
	{
		.qc = {
			.id = V4L2_CID_HUE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "HUE",
			.minimum = 0,
			.maximum = 3,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_hue,
		.query = ov3640_q_hue,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Vertical flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov3640_t_vflip,
		.query = ov3640_q_vflip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Horizontal mirror",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov3640_t_hflip,
		.query = ov3640_q_hflip,
	},
	{
		.qc = {
			.id = V4L2_CID_DO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "white balance",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_whitebalance,
		.query = ov3640_q_whitebalance,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov3640_t_exposure,
		.query = ov3640_q_exposure,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov3640_controls))

static struct ov3640_control *ov3640_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov3640_controls[i].qc.id == id)
			return ov3640_controls + i;
	return NULL;
}


static int ov3640_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct ov3640_control *ctrl = ov3640_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int ov3640_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov3640_control *octrl = ov3640_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int ov3640_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov3640_control *octrl = ov3640_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret =  octrl->tweak(client, ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}




int ccic_sensor_attach(struct i2c_client *client);


/*
 * Basic i2c stuff.
 */
static int __devinit ov3640_probe(struct i2c_client *client, const struct i2c_device_id * i2c_id)
{
	int ret = 0;
	struct ov3640_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;

	ccic_set_clock_mipi();

	pdata->power_on(1, 1);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct ov3640_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &ov3640_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);
	/*
	 * Make sure it's an ov3640
	 */
	ret = ov3640_detect(client);
	if (ret)
		goto out_free;
	printk(KERN_NOTICE "OmniVision ov3640 sensor detected\n");
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


static int ov3640_remove(struct i2c_client *client)
{
	return 0;	//TODO
}


static int ov3640_streamon(struct i2c_client *client)
{
	unsigned char val;
	ov3640_read(client, 0x3086, &val);
	val &= ~0x03;
	ov3640_write(client, 0x3086, val);
	return 0;
}

static int ov3640_streamoff(struct i2c_client *client)
{
	unsigned char val;
	ov3640_read(client, 0x3086, &val);
	val |= 0x03;
	ov3640_write(client, 0x3086, val);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov3640_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov3640_read(client, (u16)reg->reg, (unsigned char *)&(reg->val));
}

static int ov3640_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov3640_write(client, (u16)reg->reg, (unsigned char)reg->val);
}
#endif
static int ov3640_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_OV3640, 0);

		case VIDIOC_INT_RESET:
			ov3640_reset(client);
			return 0;
#if 0
		case VIDIOC_INT_INIT:
			return 0;//ov3640_init(client);		//TODO - should get 3640 default register values
#endif
		case VIDIOC_QUERYCAP:
			return ov3640_querycap(client, (struct v4l2_capability *) arg);
		case VIDIOC_ENUM_FMT:
			return ov3640_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_ENUM_FRAMESIZES:
			return ov3640_enum_fmsize(client, (struct v4l2_frmsizeenum *) arg);
		case VIDIOC_TRY_FMT:
			return ov3640_try_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_S_FMT:
			return ov3640_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_QUERYCTRL:
			return ov3640_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return ov3640_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return ov3640_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return ov3640_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return ov3640_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return ov3640_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return ov3640_streamon(client);
		case VIDIOC_STREAMOFF:
			return ov3640_streamoff(client);
#ifdef CONFIG_VIDEO_ADV_DEBUG
		case VIDIOC_DBG_G_REGISTER:
			return ov3640_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return ov3640_s_register(client, (struct v4l2_dbg_register *) arg);
#endif
	}
	return -EINVAL;
}

static struct i2c_device_id ov3640_idtable[] = {
	{ "ov3640", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov3640_idtable);

static struct i2c_driver ov3640_driver = {
	.driver = {
		.name	= "ov3640",
	},
	.id_table       = ov3640_idtable,
	.command	= ov3640_command,
	.probe		= ov3640_probe,
	.remove		= ov3640_remove,
};


/*
 * Module initialization
 */
static int __init ov3640_mod_init(void)
{
	printk(KERN_NOTICE "OmniVision ov3640 sensor driver, at your service\n");
	return i2c_add_driver(&ov3640_driver);
}

static void __exit ov3640_mod_exit(void)
{
	i2c_del_driver(&ov3640_driver);
}

late_initcall(ov3640_mod_init);
//module_init(ov3640_mod_init);
module_exit(ov3640_mod_exit);

