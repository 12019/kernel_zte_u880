/*
 * A V4L2 driver for SamSung s5k5ca cameras.
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
#include <plat/i2c.h>

#include "pxa910_camera.h"
#include "s5k5ca_dvp.h"

MODULE_AUTHOR("Yang Xiao Jun (yangxj@hojy.com)");
MODULE_DESCRIPTION("A low-level driver for Samsung s5k5ca sensors");
MODULE_LICENSE("GPL");


#define   S5K5CA_ZOOM_BASEVALUE   100

#define V4L2_CID_EXT_FLASH_MODE			(V4L2_CID_PRIVATE_BASE+1)
#define V4L2_CID_EXT_DIGITAL_ZOOM		(V4L2_CID_PRIVATE_BASE+8)
#define V4L2_CID_FOCUS_AUTO             (V4L2_CID_CAMERA_CLASS_BASE+12)
#define V4L2_CID_EXT_EFFECT			    (V4L2_CID_PRIVATE_BASE+15)
#define V4L2_CID_EXT_WB_MODE			(V4L2_CID_PRIVATE_BASE+14)
#define V4L2_CID_CONTRAST               (V4L2_CID_BASE+1)
#define V4L2_CID_EXT_EV_OFFSET			(V4L2_CID_PRIVATE_BASE+6)
#define V4L2_CID_SATURATION 			(V4L2_CID_BASE+2)
#define V4L2_CID_EXT_SHARPNESS			(V4L2_CID_PRIVATE_BASE+4)
#define V4L2_CID_EXT_ISOSPEED           (V4L2_CID_PRIVATE_BASE+7)
#define V4L2_CID_EXT_BANDFILTER         (V4L2_CID_PRIVATE_BASE+9)


static int s5k5ca_zoomvalue; 
extern unsigned int main_camera_id; // add for samsung camera sensor. 

struct i2c_client *g_main_sensor_client = NULL;
extern struct i2c_client *g_ov7690_client;

static int g_dvp_intial = 0;
/*
 * Low-level register I/O.
 */

static int s5k5ca_detect(struct i2c_client *client);

static int s5k5ca_read_word(struct i2c_client *c, u16 reg, u16 * value)
{
	int ret;
     //   int i = 0, ret1, ret2;
	u8 address[2];

	address[0] = reg >> 8;
	address[1] = reg;

	ret = i2c_smbus_write_byte_data(c, address[0], address[1]);
	if(ret < 0){
		dev_err(&c->dev, "%s() ret = %d\n", __func__, ret);
		return ret;
	}

    ret = i2c_smbus_read_word_data(c, address[0]);
    
    /*for(i=0; i<3;i++)
        {
            printk("try to read ############ i=%d\n", i);
	    ret = i2c_smbus_read_word_data(c, address[0]);
	    ret1 = i2c_smbus_read_word_data(c, address[0]);
	    ret2 = i2c_smbus_read_word_data(c, address[0]);
            if((ret == ret1) && (ret1 == ret2))
            {
                break;
            }
        }*/

        if(ret > 0)
                *value = (u16 )ret;

	return ret;
}

static int s5k5ca_write_word(struct i2c_client *c, u16 reg, u16 value)
{
	int ret;
	u8 data[4];

	data[0] = reg >> 8;
	data[1] = reg;
	data[2] = value >> 8;
	data[3] = value;

	ret = i2c_master_send(c, data, 4);
	

	if (4 != ret )	{
		dev_err(&c->dev, "%s() ret = %d\n", __func__, ret);
	}
	if (XFER_NAKED == ret) return 0;
        return (4 == ret) ? 0 :-EIO;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int s5k5ca_write_word_array(struct i2c_client *c, struct regval_list *vals)
{
	int ret = 0;
	while (vals->reg_num != 0xffff || vals->value != 0xffff) {
		ret = s5k5ca_write_word(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}


/*
 * Stuff that knows about the sensor.
 */

static int s5k5ca_init(struct i2c_client *client)
{
	int ret = 0;
	printk("%s in\n", __FUNCTION__);

	ret = s5k5ca_write_word_array(client, s5k5ca_init_table1);
	if (ret < 0) {
		printk(KERN_ERR "%s : init table1 fail!!!\n", __func__);
		goto err;
	}
	else
	{
		;//printk(KERN_ERR "%s : init table1 success!!!\n", __func__);
	}
	msleep(100);
	    
	ret = s5k5ca_write_word_array(client, s5k5ca_init_table3);
	if (ret < 0) {
		printk(KERN_ERR "%s : init table3 fail!!!\n", __func__);
	}
	else
	{
		;//printk(KERN_ERR "%s : init table3 success!!!\n", __func__);
	}
	    msleep(100);


	ret = s5k5ca_write_word_array(client, s5k5ca_init_table2);
	if (ret < 0) {
		printk(KERN_ERR "%s : init table2 fail!!!\n", __func__);
	}
	else
	{
		;//printk(KERN_ERR "%s : init table2 success!!!\n", __func__);
	}
	/*msleep(100);

	ret = s5k5ca_write_word_array(client, s5k5ca_init_table3);
	if (ret < 0) {
		printk(KERN_ERR "%s : init table3 fail!!!\n", __func__);
	}*/
err:
	return ret;
}

static int s5k5ca_reset(struct i2c_client *client)
{
	return 0;//s5k5ca_init(client);
}

static int s5k5ca_detect(struct i2c_client *client)
{
	u16 chip_id;
	int ret;
	printk("%s 0040_2\n", __FUNCTION__);
    //    int i = 0;
	/*
	 * no MID register found. OK, we know we have an OmniVision chip...but which one?
	 */
	//ret = s5k5ca_write_word(client, 0x0028, 0xd000);
	//ret = s5k5ca_write_word(client, 0x002A, 0x1006);
	ret = s5k5ca_write_word(client, 0x002c, 0x0000);
	ret = s5k5ca_write_word(client, 0x002e, 0x0040);
	/*do
	{ 
		ret = s5k5ca_write_word(client, 0x0028, 0xD000);
		i++;
	}while(i<10 && ret<0);*/

	if (ret < 0) {
		printk(KERN_ERR "%s write addr fail !!!", __func__);
		return ret;
	}
	//ret = s5k5ca_read_word(client, 0x1006, &chip_id);
	ret = s5k5ca_read_word(client, 0x0F12, &chip_id);
	if (ret < 0) {
		printk(KERN_ERR "%s read id fail !!!", __func__);
		return ret;
	}
     
	if (chip_id != S5K5CA_ID) {
		dev_err(&client->dev, "Chip id(0x%04X) not correct , should be 0xCA05\n", chip_id);
		return -ENODEV;
	}
	else
	{
		printk(KERN_ERR "%s chip id detect successfully !!!", __func__);
	}
	return 0;
}


static int s5k5ca_preview_config(struct i2c_client *client, unsigned char num)
{
	int ret = 0;
	if(num > PREVIEW_CONFIG4) {
		dev_err(&client->dev, "preview config num(%d) not correct!!\n", num);
		return -ENODEV;
	}

	ret = s5k5ca_write_word(client, 0x0028, 0x7000);
	ret |= s5k5ca_write_word(client, 0x002A, 0x023C); //#REG_TC_GP_ActivePrevConfig 
	ret |= s5k5ca_write_word(client, 0x0F12, num);
	ret |= s5k5ca_write_word(client, 0x002A, 0x0240); //#REG_TC_GP_PrevOpenAfterChange
	ret |= s5k5ca_write_word(client, 0x0F12, 0x0001);
	ret |= s5k5ca_write_word(client, 0x002A, 0x0230); //#REG_TC_GP_NewConfigSync 
	ret |= s5k5ca_write_word(client, 0x0F12, 0x0001);
	ret |= s5k5ca_write_word(client, 0x002A, 0x023E); //#REG_TC_GP_PrevConfigChanged 
	ret |= s5k5ca_write_word(client, 0x0F12, 0x0001); 
	if (ret < 0) {
		printk(KERN_ERR "%s setting fail!!!", __func__);
	}
	else
	{
		printk(KERN_ERR "%s : setting  success!!!\n", __func__);
	}

	//msleep(200);
	return ret;
}

static int s5k5ca_capture_config(struct i2c_client *client, unsigned char num)
{
	int ret = 0;
	if(num > CAPTURE_CONFIG4) {
		dev_err(&client->dev, "capture config num(%d) not correct!!!\n", num);
		return -ENODEV;
	}

	ret = s5k5ca_write_word(client, 0x0028, 0x7000);
	ret |= s5k5ca_write_word(client, 0x002A, 0x0244); //#REG_TC_GP_ActiveCapConfig
	ret |= s5k5ca_write_word(client, 0x0F12, num); 
	ret |= s5k5ca_write_word(client, 0x0F12, 0x0001); //#REG_TC_GP_CapConfigChanged 
	ret |= s5k5ca_write_word(client, 0x002A, 0x0230);
	ret |= s5k5ca_write_word(client, 0x0F12, 0x0001); //#REG_TC_GP_NewConfigSync
	ret |= s5k5ca_write_word(client, 0x002A, 0x0224);
	ret |= s5k5ca_write_word(client, 0x0F12, 0x0001); //#REG_TC_GP_EnableCapture 
	ret |= s5k5ca_write_word(client, 0x0F12, 0x0001); //#REG_TC_GP_EnableCaptureChanged
	if (ret < 0) {
		printk(KERN_ERR "%s setting fail!!!", __func__);
	}
	return ret;
}

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct s5k5ca_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	int bpp;   /* bits per pixel */
} s5k5ca_formats[] = {

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
};
#define N_S5K5CA_FMTS ARRAY_SIZE(s5k5ca_formats)

/*TODO - also can use ccic size register 0x34 to do same thing for cropping...anyway, sensor doing it is better?
  0x3020~0x3027*/
static struct s5k5ca_win_size {
	int	width;
	int	height;
	/* h/vref stuff */
} s5k5ca_win_sizes[] = {
	{
		.width		= 176,
		.height		= 144,

	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
	},
	/* CIF*/
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
	},
	/* HVGA */
	//{
	//	.width		= HVGA_WIDTH,
	//	.height		= HVGA_HEIGHT,
	//},
	/* VGA */
	//{
	//	.width		= VGA_WIDTH,
	//	.height		= VGA_HEIGHT,
	//},
	/* D1 */
	{
		.width     	= D1_WIDTH,
		.height    	= D1_HEIGHT,
	},
	/* SVGA */
	//{
	//	.width          = SVGA_WIDTH,
	//	.height         = SVGA_HEIGHT,
	//},
};

/* capture jpeg size */
static struct s5k5ca_win_size s5k5ca_win_sizes_jpeg[] = {
	/* QXGA */
	{
		.width 		= QXGA_WIDTH,
		.height 	= QXGA_HEIGHT,
	},
	/* UXGA */
	{
		.width 		= UXGA_WIDTH,
		.height 	= UXGA_HEIGHT,
	},
	/* XGA */
	{
		.width 		= 1024,
		.height 	= 768,
	},
	/* SXGA */
	//{
	//	.width 		= SXGA_WIDTH,
	//	.height 	= SXGA_HEIGHT,
	//},
	/* SVGA */
	//{
	//	.width          = SVGA_WIDTH,
	//	.height         = SVGA_HEIGHT,
	//},
	/* D1 */
	//{
	//	.width     	= D1_WIDTH,
	//	.height    	= D1_HEIGHT,
	//},
};

static int s5k5ca_querycap(struct i2c_client *c, struct v4l2_capability *argp)
{
	if(!argp){
		printk(KERN_ERR" argp is NULL %s %d \n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	strcpy(argp->driver, "v4l2MainSensor");
	strcpy(argp->card, "TD/TTC");
	return 0;
}

static int s5k5ca_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct s5k5ca_format_struct *ofmt;

	if (fmt->index >= ARRAY_SIZE(s5k5ca_formats))
        {
		printk(KERN_ERR "NO such fmt->index\n");
		return -EINVAL;
	}
	ofmt = s5k5ca_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int s5k5ca_enum_fmsize(struct i2c_client *c, struct v4l2_frmsizeenum *argp)
{
	struct v4l2_frmsizeenum frmsize;

	if (copy_from_user(&frmsize, argp, sizeof(frmsize)))
		   return -EFAULT;

	if (frmsize.pixel_format == V4L2_PIX_FMT_YUV420){
		if (frmsize.index >= (ARRAY_SIZE(s5k5ca_win_sizes))){
			printk(KERN_ERR" \n max index for preview is %d , index = %d\n", ARRAY_SIZE(s5k5ca_win_sizes), frmsize.index);
		    return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = s5k5ca_win_sizes[frmsize.index].height;
		frmsize.discrete.width = s5k5ca_win_sizes[frmsize.index].width;
	}else if(frmsize.pixel_format == V4L2_PIX_FMT_JPEG){
		if (frmsize.index >= ARRAY_SIZE(s5k5ca_win_sizes_jpeg)){
			   printk(KERN_ERR" \n max index for jpeg  is %d , index = %d\n", ARRAY_SIZE(s5k5ca_win_sizes_jpeg), frmsize.index);
			   return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = s5k5ca_win_sizes_jpeg[frmsize.index].height;
		frmsize.discrete.width = s5k5ca_win_sizes_jpeg[frmsize.index].width;

	}else
	   return -EINVAL;

	if (copy_to_user(argp, &frmsize, sizeof(frmsize)))
		   return -EFAULT;
	return 0;
}

static int s5k5ca_try_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int index = 0;
	int i = 0;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	for (index = 0; index < ARRAY_SIZE(s5k5ca_formats); index++)
		if (s5k5ca_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= ARRAY_SIZE(s5k5ca_formats))
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
		for (i = 0; i < ARRAY_SIZE(s5k5ca_win_sizes_jpeg); i++)
			if (pix->width == s5k5ca_win_sizes_jpeg[i].width && pix->height == s5k5ca_win_sizes_jpeg[i].height)
				break;

		if (i >= ARRAY_SIZE(s5k5ca_win_sizes_jpeg)){
				printk(KERN_ERR"invalid size request for jpeg! %d %d  \n",
					pix->width, pix->height);
				return -EINVAL;
			}
		/* for OV5642, HSYNC contains 2048bytes */
		pix->bytesperline = 2048;
		printk(KERN_ERR"change jpeg width as w %d h %d \n",pix->width,pix->height );
	}else{
		for (i = 0; i < ARRAY_SIZE(s5k5ca_win_sizes);i ++)
			if (pix->width == s5k5ca_win_sizes[i].width && pix->height == s5k5ca_win_sizes[i].height)
				break;

		if (i>= ARRAY_SIZE(s5k5ca_win_sizes)){
				printk(KERN_ERR"invalid size request for preview! %d %d  \n",
					pix->width, pix->height);
				//return 0;//-EINVAL;
			}
		pix->bytesperline = pix->width*s5k5ca_formats[index].bpp/8;
		pix->sizeimage = pix->height*pix->bytesperline;
	}
	return 0;
}

/*
 * Set a format.
 */
static int s5k5ca_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret = 0;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	if(g_dvp_intial == 0)
	{
		s5k5ca_init(c);
		g_dvp_intial = 1;
	}

	ret = s5k5ca_try_fmt(c, fmt);
	if (ret < 0)
	{
		printk(KERN_ERR "try fmt error\n");
		return ret;
	}

	//if ((s5k5ca_zoomvalue > CAM_ZOOM_100)&& (pix->pixelformat == V4L2_PIX_FMT_JPEG))
	//{
	//	if(pix->width > 1024)
	//	{
	//		pix->width = 800;
	//		pix->height = 600;
	//	}
	//}

	switch(pix->pixelformat)
	{
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV420:
			switch (pix->width )
			{
				case QVGA_WIDTH:
					s5k5ca_write_word(c, 0x0028, 0x7000);
					s5k5ca_write_word(c, 0x002A, 0x046C);
					s5k5ca_write_word(c, 0x0F12, 0x0800);
					s5k5ca_write_word(c, 0x0F12, 0x0600);
					s5k5ca_write_word(c, 0x0F12, 0x0000);
					s5k5ca_write_word(c, 0x0F12, 0x0000);
					s5k5ca_write_word(c, 0x002A, 0x0466);
					s5k5ca_write_word(c, 0x0F12, 0x0001);
					ret = s5k5ca_preview_config(c, PREVIEW_CONFIG0);
					printk(KERN_INFO "choose qvga setting \n");
					break;
				case 144:
					s5k5ca_write_word(c, 0x0028, 0x7000);
					s5k5ca_write_word(c, 0x002A, 0x046C);
					s5k5ca_write_word(c, 0x0F12, 0x04E4);
					s5k5ca_write_word(c, 0x0F12, 0x0600);
					s5k5ca_write_word(c, 0x0F12, 0x018E);
					s5k5ca_write_word(c, 0x0F12, 0x0000);
					s5k5ca_write_word(c, 0x002A, 0x0466);
					s5k5ca_write_word(c, 0x0F12, 0x0001);
					ret = s5k5ca_preview_config(c, PREVIEW_CONFIG1);
					printk(KERN_INFO "choose qcif rotate setting \n");
					break;				
				case QCIF_WIDTH:
					s5k5ca_write_word(c, 0x0028, 0x7000);
					s5k5ca_write_word(c, 0x002A, 0x046C);
					s5k5ca_write_word(c, 0x0F12, 0x0800);
					s5k5ca_write_word(c, 0x0F12, 0x0600);
					s5k5ca_write_word(c, 0x0F12, 0x0000);
					s5k5ca_write_word(c, 0x0F12, 0x0000);
					s5k5ca_write_word(c, 0x002A, 0x0466);
					s5k5ca_write_word(c, 0x0F12, 0x0001);
					ret = s5k5ca_preview_config(c, PREVIEW_CONFIG2);
					printk(KERN_INFO "choose qcif setting \n");
					break;
				case CIF_WIDTH:
					s5k5ca_write_word(c, 0x0028, 0x7000);
					s5k5ca_write_word(c, 0x002A, 0x046C);
					s5k5ca_write_word(c, 0x0F12, 0x0800);
					s5k5ca_write_word(c, 0x0F12, 0x0600);
					s5k5ca_write_word(c, 0x0F12, 0x0000);
					s5k5ca_write_word(c, 0x0F12, 0x0000);
					s5k5ca_write_word(c, 0x002A, 0x0466);
					s5k5ca_write_word(c, 0x0F12, 0x0001);
					ret = s5k5ca_preview_config(c, PREVIEW_CONFIG3);
					printk(KERN_INFO "choose cif setting \n");
					break;
				case D1_WIDTH:
					s5k5ca_write_word(c, 0x0028, 0x7000);
					s5k5ca_write_word(c, 0x002A, 0x046C);
					s5k5ca_write_word(c, 0x0F12, 0x0800);
					s5k5ca_write_word(c, 0x0F12, 0x0555);
					s5k5ca_write_word(c, 0x0F12, 0x0000);
					s5k5ca_write_word(c, 0x0F12, 0x0055);
					s5k5ca_write_word(c, 0x002A, 0x0466);
					s5k5ca_write_word(c, 0x0F12, 0x0001);
					ret = s5k5ca_preview_config(c, PREVIEW_CONFIG4);
					printk(KERN_INFO "choose d1 setting \n");
					break;
				default:
					printk(KERN_ERR "unsupported YUV format !\n");
					ret = -EINVAL;
					goto out;
					break;
			}
			break;
		case V4L2_PIX_FMT_JPEG:
			switch (pix->width)
			{
				case QXGA_WIDTH:
					ret = s5k5ca_capture_config(c, CAPTURE_CONFIG0);
					printk(KERN_INFO "choose qxga jpeg setting \n");
					break;
				case UXGA_WIDTH:
					ret = s5k5ca_capture_config(c, CAPTURE_CONFIG1);
					printk(KERN_INFO "choose uxga jpeg setting \n");
					break;
				case 1024:
					ret = s5k5ca_capture_config(c, CAPTURE_CONFIG3);
					printk(KERN_INFO "choose xga jpeg setting \n");
					break;
				//case SVGA_WIDTH:
				//	ret = s5k5ca_capture_config(c, CAPTURE_CONFIG1);
				//	printk(KERN_INFO "choose pv jpeg setting \n");
				//	break;
				//case D1_WIDTH:
				//	ret = s5k5ca_capture_config(c, CAPTURE_CONFIG0);
				//	printk(KERN_INFO "choose d1 jpeg setting \n");
				//	break;
				default:
					printk(KERN_ERR "unsupported JPEG size!\n");
					ret = -EINVAL;
					goto out;
					break;
			}
			break;
		default:
			printk(KERN_ERR "unsupported format!\n");
			ret = -EINVAL;
			goto out;
			break;
	}

out:
	if (ret < 0) {
		printk(KERN_ERR "%s pixelformat fail !!!", __func__);
	}
	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int s5k5ca_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = S5K5CA_FRAME_RATE;

	return 0;
}

static int s5k5ca_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int s5k5ca_s_input(struct i2c_client *c, int *id)
{
	g_dvp_intial = 0;
	return 0;
}

int main_sensor_dvp_initialize(void)
{
	printk("%s\n", __FUNCTION__);
	if(g_main_sensor_client == NULL)
	{
		if(g_ov7690_client != NULL)
		{
			printk("%s g_ov7690_client\n", __FUNCTION__);
			g_ov7690_client->addr = 0x2d;
			s5k5ca_init(g_ov7690_client);
			g_ov7690_client->addr = 0x21;
		}
	}
	else
	{
		printk("%s g_s5k5ca_client\n", __FUNCTION__);
		s5k5ca_init(g_main_sensor_client);
	}
	
	return 0;
}
EXPORT_SYMBOL(main_sensor_dvp_initialize);
/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */

static int g_brightness;
static int g_hue;
static int g_saturation;
static int g_contrast;
static int g_vflip;
static int g_hflip;
static int g_whitebalance;
static int g_exposure;
static int g_evoffset;
static int g_coloreffect;
static int g_sharpness;
static int g_iso;
static int g_bandfilter;

static int s5k5ca_t_sat(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s value = %d\n", __FUNCTION__, value);

	switch (value)
	{
		case CAM_EV_NEG_2:
			ret = s5k5ca_write_word_array(client, s5k5ca_satuation_neg_2);
		break;

		case CAM_EV_NEG_1:
			ret = s5k5ca_write_word_array(client, s5k5ca_satuation_neg_1);
		break;

		case CAM_EV_POS_1:
			ret = s5k5ca_write_word_array(client, s5k5ca_satuation_pos_1);
		break;

		case CAM_EV_POS_2:
			ret = s5k5ca_write_word_array(client, s5k5ca_satuation_pos_2);
		break;

		case CAM_EV_ZERO:
		default:
			ret = s5k5ca_write_word_array(client, s5k5ca_satuation_zero);
		break;
	}
	g_saturation = value;

	return ret;
}

static int s5k5ca_q_sat(struct i2c_client *client, __s32 *value)
{
	*value = g_saturation;
	return 0;
}

static int s5k5ca_t_hue(struct i2c_client *client, int value)
{
	int ret = 0;
	printk("%s value = %d\n", __FUNCTION__, value);

	switch (value)
	{
		case CAM_EV_NEG_2:

		break;

		case CAM_EV_NEG_1:

		break;

		case CAM_EV_POS_1:

		break;

		case CAM_EV_POS_2:

		break;

		case CAM_EV_ZERO:
		default:

		break;
	}
	g_hue = value;

	return ret;
}


static int s5k5ca_q_hue(struct i2c_client *client, __s32 *value)
{
	*value = g_hue;
	return 0;
}

static int s5k5ca_t_brightness(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s value = %d\n", __FUNCTION__, value);

	switch (value)
	{
		case 0:
			ret = s5k5ca_write_word_array(client, s5k5ca_brightness_neg_3);
		break;

		case 1:
			ret = s5k5ca_write_word_array(client, s5k5ca_brightness_neg_2);
		break;

		case 2:
			ret = s5k5ca_write_word_array(client, s5k5ca_brightness_neg_1);
		break;

		case 3:
			ret = s5k5ca_write_word_array(client, s5k5ca_brightness_zero);
		break;

		case 4:
			ret = s5k5ca_write_word_array(client, s5k5ca_brightness_pos_1);
		break;

		case 5:
			ret = s5k5ca_write_word_array(client, s5k5ca_brightness_pos_2);
		break;

		case 6:
			ret = s5k5ca_write_word_array(client, s5k5ca_brightness_pos_3);
		break;
		
		default:
			ret = s5k5ca_write_word_array(client, s5k5ca_brightness_zero);
		break;
	}
	g_brightness = value;

	return 0;
}

static int s5k5ca_q_brightness(struct i2c_client *client, __s32 *value)
{
	*value = g_brightness;
	return 0;
}

static int s5k5ca_t_contrast(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s value = %d\n", __FUNCTION__, value);

	switch (value)
	{
		case CAM_EV_NEG_2:
			ret = s5k5ca_write_word_array(client, s5k5ca_contrast_neg_2);
		break;

		case CAM_EV_NEG_1:
			ret = s5k5ca_write_word_array(client, s5k5ca_contrast_neg_1);
		break;

		case CAM_EV_POS_1:
			ret = s5k5ca_write_word_array(client, s5k5ca_contrast_pos_1);
		break;

		case CAM_EV_POS_2:
			ret = s5k5ca_write_word_array(client, s5k5ca_contrast_pos_2);
		break;

		case CAM_EV_ZERO:
		default:
			ret = s5k5ca_write_word_array(client, s5k5ca_contrast_zero);
		break;
	}
	g_contrast = value;

	return 0;
}

static int s5k5ca_q_contrast(struct i2c_client *client, __s32 *value)
{
	*value = g_contrast;
	return 0;
}

static int s5k5ca_t_hflip(struct i2c_client *client, int value)
{
	int ret = 0;

	if (value) {
		ret = s5k5ca_write_word_array(client, s5k5ca_h_mirror);
	}	
	else {
		ret = s5k5ca_write_word_array(client, s5k5ca_mirror_normal);
	}
	//ret = s5k5ca_write_word_array(client, s5k5ca_mirror_hv_mirror)
	g_hflip = value;

	return ret;
}

static int s5k5ca_q_hflip(struct i2c_client *client, __s32 *value)
{
	*value = g_hflip;
	return 0;
}

static int s5k5ca_t_vflip(struct i2c_client *client, int value)
{
	int ret = 0;

	if  (value) {
		ret = s5k5ca_write_word_array(client, s5k5ca_v_mirror);
	}
	else {
		ret = s5k5ca_write_word_array(client, s5k5ca_mirror_normal);
	}
	//ret = s5k5ca_write_word_array(client, s5k5ca_mirror_hv_mirror)
	g_vflip = value;

	return 0;
}

static int s5k5ca_q_vflip(struct i2c_client *client, __s32 *value)
{
	*value = g_vflip;
	return 0;
}

static int s5k5ca_t_whitebalance(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s value = %d\n", __FUNCTION__, value);

	switch (value)
	{
		case CAM_WHITEBALANCEMODE_CLOUDY:
			ret = s5k5ca_write_word_array(client, s5k5ca_whitebalance_cloud);
		break;

		case CAM_WHITEBALANCEMODE_DAYLIGHT:
			ret = s5k5ca_write_word_array(client, s5k5ca_whitebalance_daylight);
		break;

		case CAM_WHITEBALANCEMODE_INCANDESCENT:
			ret = s5k5ca_write_word_array(client, s5k5ca_whitebalance_incandescence);
		break;

		case CAM_WHITEBALANCEMODE_FLUORESCENT1:
			ret = s5k5ca_write_word_array(client, s5k5ca_whitebalance_fluorescent);
		break;

		case CAM_WHITEBALANCEMODE_SHADOW:
			ret = s5k5ca_write_word_array(client, s5k5ca_whitebalance_tungsten);
		break;

		case CAM_WHITEBALANCEMODE_AUTO:
		default:
			ret = s5k5ca_write_word_array(client, s5k5ca_whitebalance_auto);
		break;
	}
	g_whitebalance = value;

	return ret;
}

static int s5k5ca_q_whitebalance(struct i2c_client *client, __s32 *value)
{
	*value = g_whitebalance;
	return 0;
}

static int s5k5ca_t_exposure(struct i2c_client *client, int value)
{
	int ret = 0;
	printk("%s value = %d\n", __FUNCTION__, value);

	switch (value)
	{
		case CAM_EV_NEG_2:

		break;

		case CAM_EV_NEG_1:

		break;

		case CAM_EV_POS_1:

		break;

		case CAM_EV_POS_2:

		break;

		case CAM_EV_ZERO:
		default:

		break;
	}
	g_exposure = value;

	return ret;
}

static int s5k5ca_q_exposure(struct i2c_client *client, __s32 *value)
{
	*value = g_exposure;
	return 0;
}

static int s5k5ca_t_evoffset(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s value = %d\n", __FUNCTION__, value);
	
	switch (value)
	{
		case -39322:
		case -170://CAM_EV_NEG4_3:
		case -130://CAM_EV_NEG4_3:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_neg4_3);
		break;

		case -100://CAM_EV_NEG3_3:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_neg3_3);
		break;

		case -19661:
		case -70://CAM_EV_NEG2_3:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_neg2_3);
		break;

		case -30://CAM_EV_NEG1_3:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_neg1_3);
		break;

		case 30:// CAM_EV_POS1_3:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_pos1_3);
		break;

		case 19661:
		case 70://CAM_EV_POS2_3:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_pos2_3);
		break;

		case 100://CAM_EV_POS3_3:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_pos3_3);
		break;

		case 39322:
		case 130://CAM_EV_POS4_3:
		case 170:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_pos4_3);
		break;

		case 0://CAM_EV_EXZERO:
		default:
			ret = s5k5ca_write_word_array(client, s5k5ca_evoffset_zero);
		break;
	}
	g_evoffset = value;

	return ret;
}

static int s5k5ca_q_evoffset(struct i2c_client *client, __s32 *value)
{
	*value = CAM_EV_EXMAX;//8;//g_evoffset;
	return 0;
}

static int s5k5ca_t_coloreffect(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s value = %d\n", __FUNCTION__, value);
	
	switch (value)
	{
		case CAM_COLOREFFECT_GRAYSCALE:
			ret = s5k5ca_write_word_array(client, s5k5ca_coloreffect_grayscale);
		break;

		case CAM_COLOREFFECT_SEPIA:
			ret = s5k5ca_write_word_array(client, s5k5ca_coloreffect_sepia);
		break;

		case CAM_COLOREFFECT_NEGATIVE:
			ret = s5k5ca_write_word_array(client, s5k5ca_coloreffect_colorinv);
		break;

		case CAM_COLOREFFECT_VIVID:
			ret = s5k5ca_write_word_array(client, s5k5ca_coloreffect_sepiablue);
		break;

		case CAM_COLOREFFECT_SOLARIZE:
			ret = s5k5ca_write_word_array(client, s5k5ca_coloreffect_sketch);
		break;

		case CAM_COLOREFFECT_OFF:
		default:
			ret = s5k5ca_write_word_array(client, s5k5ca_coloreffect_normal);
		break;
	}
	g_coloreffect = value;

	return ret;
}

static int s5k5ca_q_coloreffect(struct i2c_client *client, __s32 *value)
{
	*value = g_coloreffect;
	return 0;
}

static int s5k5ca_t_sharpness(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s value = %d\n", __FUNCTION__, value);

	switch (value)
	{
		case CAM_EV_NEG_2:
			ret = s5k5ca_write_word_array(client, s5k5ca_sharpness_neg_2);
		break;

		case CAM_EV_NEG_1:
			ret = s5k5ca_write_word_array(client, s5k5ca_sharpness_neg_1);
		break;

		case CAM_EV_POS_1:
			ret = s5k5ca_write_word_array(client, s5k5ca_sharpness_pos_1);
		break;

		case CAM_EV_POS_2:
			ret = s5k5ca_write_word_array(client, s5k5ca_sharpness_pos_2);
		break;

		case CAM_EV_ZERO:
		default:
			ret = s5k5ca_write_word_array(client, s5k5ca_sharpness_zero);
		break;
	}
	g_sharpness = value;

	return ret;
}

static int s5k5ca_q_sharpness(struct i2c_client *client, __s32 *value)
{
	*value = g_sharpness;

	return 0;
}

static int s5k5ca_t_iso(struct i2c_client *c, int value)
{
	printk("%s value = %d\n", __FUNCTION__, value);
	
	switch (value)
	{
		case CAM_ISO_AUTO: // ov3640_effect_none
			s5k5ca_write_word_array(c, s5k5ca_iso_auto);
			break;

		case CAM_ISO_100:  // ov3640_effect_mono
			s5k5ca_write_word_array(c, s5k5ca_iso_100);
			break; 
			
		case CAM_ISO_200:  // ov3640_effect_negative
			s5k5ca_write_word_array(c, s5k5ca_iso_200);
			break;
			
		case CAM_ISO_400: // ov3640_effect_sepia
			s5k5ca_write_word_array(c, s5k5ca_iso_400);
			break;
		case CAM_ISO_800: // ov3640_effect_sepia
			s5k5ca_write_word_array(c, s5k5ca_iso_800);
			break;
			
		default:
			s5k5ca_write_word_array(c, s5k5ca_iso_auto);
			//return -EINVAL;
	}

	g_iso = value;
	return 0;
}
	
static int s5k5ca_q_iso(struct i2c_client *client, __s32 *value)
{
	*value = 4;//g_iso;
	return 0;
}

static int s5k5ca_t_bandfilter(struct i2c_client *c, int value)
{
	//printk("%s value = %d\n", __FUNCTION__, value);
	
	switch (value)
	{
		case CAM_BANDFILTER_50HZ:
			s5k5ca_write_word_array(c, s5k5ca_banding_50hz);
			break;

		case CAM_BANDFILTER_60HZ:
			s5k5ca_write_word_array(c, s5k5ca_banding_60hz);
			break; 

		default:
			s5k5ca_write_word_array(c, s5k5ca_banding_50hz);
	}

	g_bandfilter = value;
	return 0;
}
	
static int s5k5ca_q_bandfilter(struct i2c_client *client, __s32 *value)
{
	*value = g_bandfilter;
	return 0;
}


static int s5k5ca_dvp_t_digitalzoom(struct i2c_client *c, int value)
{
	//save the value
        s5k5ca_zoomvalue = value;
	printk("%s value = %d\n", __FUNCTION__, value);
	
	switch (value)
	{
		case CAM_ZOOM_100:
			s5k5ca_write_word_array(c, s5k5ca_zoom_100); 
			break;

		case CAM_ZOOM_120:
			s5k5ca_write_word_array(c, s5k5ca_zoom_120); 
			break;

		case CAM_ZOOM_140:
			s5k5ca_write_word_array(c, s5k5ca_zoom_140); 
			break;

		case CAM_ZOOM_160:
			s5k5ca_write_word_array(c, s5k5ca_zoom_160); 
			break;
			
		case CAM_ZOOM_180:
			s5k5ca_write_word_array(c, s5k5ca_zoom_180); 
			break;
			
		case CAM_ZOOM_200:
			s5k5ca_write_word_array(c, s5k5ca_zoom_200); 
			break;
		case CAM_ZOOM_220:
			s5k5ca_write_word_array(c, s5k5ca_zoom_220); 
			break;
		case CAM_ZOOM_240:
			s5k5ca_write_word_array(c, s5k5ca_zoom_240); 
			break;
		case CAM_ZOOM_260:
			s5k5ca_write_word_array(c, s5k5ca_zoom_260); 
			break;
		case CAM_ZOOM_280:
			s5k5ca_write_word_array(c, s5k5ca_zoom_280); 
			break;
		case CAM_ZOOM_300:
			s5k5ca_write_word_array(c, s5k5ca_zoom_300); 
			break;
			
		default:
			s5k5ca_write_word_array(c, s5k5ca_zoom_100); 
			break;				
	}
	
	return 0;
}

static int s5k5ca_dvp_q_digitalzoom(struct i2c_client *client, __s32 *value)
{
	*value = 5;//s5k5ca_zoomvalue;;
	
	return 0;
}

static int s5k5ca_t_flash(struct i2c_client *c, int value)
{
    return 0;
}

static int s5k5ca_q_flash(struct i2c_client *client, __s32 *value)
{
	*value = 1;
	printk("s5k5ca_dvp_q_flash \n ");
	
	return 0;
}

static int s5k5ca_t_focus(struct i2c_client *c, int value)
{
	printk("%s value = %d\n", __FUNCTION__, value);
	
    return 0;
}

static int s5k5ca_q_focus(struct i2c_client *client, __s32 *value)
{
	*value = 5;

	return 0;
}

static struct s5k5ca_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} s5k5ca_controls[] =
{
	{
		.qc = {
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = 0,
			.maximum = 6,
			.step = 1,
			.default_value = 3,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_brightness,
		.query = s5k5ca_q_brightness,
	},
	{
		.qc = {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_contrast,
		.query = s5k5ca_q_contrast,
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
		.tweak = s5k5ca_t_sat,
		.query = s5k5ca_q_sat,
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
		.tweak = s5k5ca_t_hue,
		.query = s5k5ca_q_hue,
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
		.tweak = s5k5ca_t_vflip,
		.query = s5k5ca_q_vflip,
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
		.tweak = s5k5ca_t_hflip,
		.query = s5k5ca_q_hflip,
	},
	{
		.qc = {
			.id = V4L2_CID_EXT_WB_MODE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "white balance",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_whitebalance,
		.query = s5k5ca_q_whitebalance,
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
		.tweak = s5k5ca_t_exposure,
		.query = s5k5ca_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_EXT_EV_OFFSET,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "evoffset",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_evoffset,
		.query = s5k5ca_q_evoffset,
	},
	{
		.qc = {
			.id = V4L2_CID_EXT_EFFECT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "coloreffect",
			.minimum = 0,
			.maximum = 5,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_coloreffect,
		.query = s5k5ca_q_coloreffect,
	},
	// add for digital zoom
	{
		.qc = {
			.id = V4L2_CID_EXT_DIGITAL_ZOOM,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "zoom",
			.minimum = 0,
			.maximum = 5,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_dvp_t_digitalzoom,
		.query = s5k5ca_dvp_q_digitalzoom,
	},
	{
		.qc = {
			.id = V4L2_CID_EXT_BANDFILTER,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "banding",
			.minimum = 0,
			.maximum = 3,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_bandfilter,
		.query = s5k5ca_q_bandfilter,
	},
	//add for sharpness 
	{
		.qc = {
			.id = V4L2_CID_SHARPNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "sharpness",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 2,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_sharpness,
		.query = s5k5ca_q_sharpness,
	},
	{
		.qc = {
			.id = V4L2_CID_EXT_ISOSPEED,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "iso",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_iso,
		.query = s5k5ca_q_iso,
	},
        {
		.qc = {
			.id = V4L2_CID_EXT_FLASH_MODE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "flash",
			.minimum = 1,
			.maximum = 1,
			.step = 1,
			.default_value = 1,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_flash,
		.query = s5k5ca_q_flash,
	},
        {
		.qc = {
			.id = V4L2_CID_FOCUS_AUTO,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus",
			.minimum = 5,
			.maximum = 5,
			.step = 1,
			.default_value = 5,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = s5k5ca_t_focus,
		.query = s5k5ca_q_focus,
	},
};
#define N_CONTROLS (ARRAY_SIZE(s5k5ca_controls))

static struct s5k5ca_control *s5k5ca_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (s5k5ca_controls[i].qc.id == id)
			return s5k5ca_controls + i;
	return NULL;
}


static int s5k5ca_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct s5k5ca_control *ctrl = s5k5ca_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int s5k5ca_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct s5k5ca_control *octrl = s5k5ca_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int s5k5ca_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct s5k5ca_control *octrl = s5k5ca_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
        {
             printk("in %s: octrl == null\n", __FUNCTION__);
		return -EINVAL;
        }
	//printk("%s found octrl\n", __FUNCTION__);
	ret =  octrl->tweak(client, ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}


int ccic_sensor_attach(struct i2c_client *client);


/*
 * Basic i2c stuff.
 */
static int __devinit s5k5ca_probe(struct i2c_client *client, const struct i2c_device_id * i2c_id)
{
	int ret = 0;
	struct s5k5ca_info *info;
	struct sensor_platform_data *pdata;
	unsigned int save_camera_id = main_camera_id;

	//MCLK_Value= 24;
	g_main_sensor_client = client;
	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, " missing plftform data\n");
		return -EINVAL;
	}
    
	main_camera_id = S5K5CA_SENSOR_CAMERA;
	//ccic_set_clock_parallel();

	//power on s5k5ca...,refer to bsp code
	if (!pdata->power_on(1, pdata->id)) {
		dev_alert(&client->dev, " power on...\n");
	} else {
		dev_err(&client->dev, " power on failed\n");
		return -EIO;
	}
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct s5k5ca_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &s5k5ca_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);
	/*
	 * Make sure it's an s5k5ca
	 */
	ret = s5k5ca_detect(client);
	if (ret)
		goto out_free;
	printk(KERN_NOTICE "Samsung s5k5ca sensor detected\n");

	ret = ccic_sensor_attach(client);
	if (ret) {
		dev_err(&client->dev, "ccic attach failed\n");
		goto out_free;
	}

out_free:
	pdata->power_on(0, pdata->id);
	ccic_disable_clock();

	if (ret) {
		dev_err(&client->dev, "init failed\n");
		g_main_sensor_client = NULL;
		main_camera_id = save_camera_id;
		kfree(info);
	}

	return ret;
}


static int s5k5ca_remove(struct i2c_client *client)
{
	return 0;	//TODO
}


static int s5k5ca_streamon(struct i2c_client *client)
{
	printk("%s\n", __FUNCTION__);
	return 0;
}

static int s5k5ca_streamoff(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int s5k5ca_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return s5k5ca_read_word(client, (u16)reg->reg, (u16 *)&(reg->val));
}

static int s5k5ca_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return s5k5ca_write_word(client, (u16)reg->reg, (u16)reg->val);
}
#endif

static int s5k5ca_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
        //printk("%s enter, cmd = %d\n", __FUNCTION__, cmd);
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_S5K5CA, 0);//V4L2_IDENT_S5K5CA, 0);

		case VIDIOC_INT_RESET:
			return s5k5ca_reset(client);
#if 0
		case VIDIOC_INT_INIT:
			return s5k5ca_init(client);//	choose qvga setting	//TODO - should get s5k5ca default register values
#endif
		case VIDIOC_QUERYCAP:
			return s5k5ca_querycap(client, (struct v4l2_capability *) arg);
		case VIDIOC_ENUM_FMT:
			return s5k5ca_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_ENUM_FRAMESIZES:
			return s5k5ca_enum_fmsize(client, (struct v4l2_frmsizeenum *) arg);
		case VIDIOC_TRY_FMT:
			return s5k5ca_try_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_S_FMT:
			return s5k5ca_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_QUERYCTRL:
			return s5k5ca_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return s5k5ca_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return s5k5ca_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return s5k5ca_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return s5k5ca_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return s5k5ca_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return s5k5ca_streamon(client);
		case VIDIOC_STREAMOFF:
			return s5k5ca_streamoff(client);
#ifdef CONFIG_VIDEO_ADV_DEBUG
		case VIDIOC_DBG_G_REGISTER:
			return s5k5ca_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return s5k5ca_s_register(client, (struct v4l2_dbg_register *) arg);
#endif
	}
	return -EINVAL;
}

static struct i2c_device_id s5k5ca_idtable[] = {
	{ "s5k5ca", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, s5k5ca_idtable);

static struct i2c_driver s5k5ca_driver = {
	.driver = {
		.name	= "s5k5ca",
	},
	.id_table       = s5k5ca_idtable,
	.command	= s5k5ca_command,
	.probe		= s5k5ca_probe,
	.remove		= s5k5ca_remove,
};


/*
 * Module initialization
 */
static int __init s5k5ca_mod_init(void)
{
	printk(KERN_NOTICE "Samsung s5k5ca sensor driver, at your service\n");
	return i2c_add_driver(&s5k5ca_driver);
}

static void __exit s5k5ca_mod_exit(void)
{
	i2c_del_driver(&s5k5ca_driver);
}

late_initcall(s5k5ca_mod_init);
//module_init(s5k5ca_mod_init);
module_exit(s5k5ca_mod_exit);

