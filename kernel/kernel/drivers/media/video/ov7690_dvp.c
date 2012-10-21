
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

#include "ov7690_dvp.h"

MODULE_AUTHOR("Jacky < Jacky_wang@ovt.com>");
MODULE_DESCRIPTION("OmniVision ov7690_dvp sensors");
MODULE_LICENSE("GPL");

#define V4L2_CID_EXT_FLASH_MODE			(V4L2_CID_PRIVATE_BASE+1)
#define V4L2_CID_EXT_DIGITAL_ZOOM		(V4L2_CID_PRIVATE_BASE+8)
#define V4L2_CID_FOCUS_AUTO             (V4L2_CID_CAMERA_CLASS_BASE+12)
/* camera auto focus control */
#define V4L2CAM_AUTOFOCUS_IDLE		0
#define V4L2CAM_AUTOFOCUS_SINGLE	1
#define V4L2_CID_EXT_EFFECT			    (V4L2_CID_PRIVATE_BASE+15)
#define V4L2_CID_EXT_WB_MODE			(V4L2_CID_PRIVATE_BASE+14)
#define V4L2_CID_CONTRAST               (V4L2_CID_BASE+1)
#define V4L2_CID_EXT_EV_OFFSET			(V4L2_CID_PRIVATE_BASE+6)
#define V4L2_CID_SATURATION 			(V4L2_CID_BASE+2)
#define V4L2_CID_EXT_SHARPNESS			(V4L2_CID_PRIVATE_BASE+4)
#define V4L2_CID_EXT_ISOSPEED           (V4L2_CID_PRIVATE_BASE+7)
#define V4L2_CID_EXT_BANDFILTER         (V4L2_CID_PRIVATE_BASE+9)

struct ov7690_dvp_format_struct;

struct i2c_client *g_ov7690_client = NULL;
#if defined CONFIG_PXA_U812
extern struct i2c_client *g_main_sensor_client;
#else
struct i2c_client *g_main_sensor_client = NULL;
#endif

extern unsigned int main_camera_id; // add for samsung camera sensor. 

struct ov7690_dvp_info
{
	struct ov7690_dvp_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};

static int ov7690_dvp_read(struct i2c_client *c, u16 reg, unsigned char *value)
{
	int ret = 0;
	u8 data;
	u8 address[2];

	address[0] = reg &0xff;
	
	ret = i2c_master_send(c, address, 1);
	if(ret < 0)
		goto out;
	ret = i2c_master_recv(c, &data, 1);
	if(ret < 0)
		goto out;
	*value = data;
out:
	return (ret < 0) ? ret: 0;
}


static int ov7690_dvp_write(struct i2c_client *c, u16 reg, unsigned char value)
{
	u8 data[2];
	int ret = 0;

	data[0] = reg & 0xff;
	data[1] = value;

	ret = i2c_master_send(c, data, 2);
	
	return (ret < 0) ? ret: 0;
}


/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7690_dvp_write_array(struct i2c_client *c, OV7690_WREG    *vals)
{
	int i = 0;
	int ret = 0;
	while (vals->reg_num != OV7690_DVP_END_ADDR|| vals->value != OV7690_DVP_END_VAL)
	{
		ret = ov7690_dvp_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		i++;
	}
	return ret;
}

static void ov7690_dvp_reset(struct i2c_client *client)
{

}

static int ov7690_dvp_set_jpegdata_size(struct i2c_client *c, u16 width,u16 height)
{
	int ret = 0;
	ret = ov7690_dvp_write(c, 0x4602, width>>8);
	if(ret < 0)
		goto out;
	ret= ov7690_dvp_write(c, 0x4603, width&0xff);
	if(ret < 0)
		goto out;
out:
	return ret;
}

static int ov7690_dvp_detect(struct i2c_client *client)
{
	unsigned char val = 0;//v = 0, val =0;
	//int ret = 0;
	int cam_pid = 0, cam_rev = 0;
	
	val = 0x80;
	ov7690_dvp_write(client, 0x12, val);
	mdelay(3);
	/*
	   Before OV7690 power down, 0x49 should be written with different values
	   according to DOVDD voltage. Please see OV5642_OVM7690 Dual Camera Module
	   Application Notes_R1.0.pdf 2.2.1 step 5.
	 */
	ov7690_dvp_read(client, OV7690_REG_PWD, &val);
	val &= 0xf0;
	val |= OV7690_PWD_1_8;
	ov7690_dvp_write(client, OV7690_REG_PWD, val);
#if 0
	{
		int i = 0;
		for(i = 0x10; i < 0x7f; i++)
		{
			client->addr = i;
			ov7690_dvp_read(client, OV7690_PID, &cam_pid);
			printk("--------------ov7690 i2c addr:%x, val:ox%x", i, cam_pid);
		}
		client->addr = 0x21;
	}
#endif
	/* read out version */
	ov7690_dvp_read(client, OV7690_PID, &cam_pid);
	ov7690_dvp_read(client, OV7690_VER, &cam_rev);

	/* Check to make sure we are working with an OV7690 */
	if (cam_pid != PID_OV76XX || cam_rev != PID_7690) {
		printk("VGA camera ov7690 not detected, ID:0x%x%x\n", cam_pid, cam_rev);
		return  -ENODEV;
	}
	printk( "VGA camera ov7690 detected, ID:0x%x%x\n", cam_pid,
	       cam_rev);
	return 0;
}

static struct ov7690_dvp_format_struct
{
	__u8 *desc;
	__u32 pixelformat;
	int bpp;   /* bits per pixel */
} ov7690_dvp_formats[] = {

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
};

static struct ov7690_dvp_win_size {
	int	width;
	int	height;
} ov7690_dvp_win_sizes[] = {
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
	},
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
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
	},
};

/* capture jpeg size */
static struct ov7690_dvp_win_size ov7690_dvp_win_sizes_jpeg[] = {

};


static int ov7690_dvp_querycap(struct i2c_client *c, struct v4l2_capability *argp)
{
	if(!argp){
		printk(KERN_ERR" argp is NULL %s %d \n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	strcpy(argp->driver, "v4l2SmallSensor");
	strcpy(argp->card, "TD/TTC");
	return 0;
}


static int ov7690_dvp_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct ov7690_dvp_format_struct *ofmt;

	if (fmt->index >= ARRAY_SIZE(ov7690_dvp_formats))
	{
		printk(KERN_ERR"NO such fmt->index\n");
		return -EINVAL;
	}
	ofmt = ov7690_dvp_formats + fmt->index;
	fmt->flags = 0;
	strncpy(fmt->description, ofmt->desc, 32);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}

static int ov7690_dvp_enum_fmsize(struct i2c_client *c, struct v4l2_frmsizeenum *argp)
{
	struct v4l2_frmsizeenum frmsize;

	if (copy_from_user(&frmsize, argp, sizeof(frmsize)))
		   return -EFAULT;

	if (frmsize.pixel_format == V4L2_PIX_FMT_YUV420 ||
	frmsize.pixel_format == V4L2_PIX_FMT_YUYV ||
	frmsize.pixel_format == V4L2_PIX_FMT_YUV422P){
		if (frmsize.index >= (ARRAY_SIZE(ov7690_dvp_win_sizes))){
			printk(KERN_ERR" \n max index for preview is %d \n", ARRAY_SIZE(ov7690_dvp_win_sizes));
		    return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = ov7690_dvp_win_sizes[frmsize.index].height;
		frmsize.discrete.width = ov7690_dvp_win_sizes[frmsize.index].width;
	}else if(frmsize.pixel_format == V4L2_PIX_FMT_JPEG){
		if (frmsize.index >= ARRAY_SIZE(ov7690_dvp_win_sizes_jpeg)){
			   printk(KERN_ERR" \n max index for jpeg  is %d \n", ARRAY_SIZE(ov7690_dvp_win_sizes_jpeg));
			   return -EINVAL;
		}
		frmsize.type = V4L2_FRMSIZE_TYPE_DISCRETE;
		frmsize.discrete.height = ov7690_dvp_win_sizes_jpeg[frmsize.index].height;
		frmsize.discrete.width = ov7690_dvp_win_sizes_jpeg[frmsize.index].width;

	}else
	   return -EINVAL;

	if (copy_to_user(argp, &frmsize, sizeof(frmsize)))
		   return -EFAULT;
	return 0;
}


static int ov7690_dvp_try_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int index = 0;
	int i = 0;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	for (index = 0; index < ARRAY_SIZE(ov7690_dvp_formats); index++)
		if (ov7690_dvp_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= ARRAY_SIZE(ov7690_dvp_formats))
	{
		printk(KERN_ERR"%s %d unsupported format!\n", __FUNCTION__, __LINE__);
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
		for (i = 0; i < ARRAY_SIZE(ov7690_dvp_win_sizes_jpeg); i++)
			if (pix->width == ov7690_dvp_win_sizes_jpeg[i].width && pix->height == ov7690_dvp_win_sizes_jpeg[i].height)
				break;

		if (i >= ARRAY_SIZE(ov7690_dvp_win_sizes_jpeg)){
			printk(KERN_ERR"invalid size request for jpeg! %d %d  \n",
				pix->width, pix->height);
				return -EINVAL;
			}
		/* for OV7690, HSYNC contains 2048bytes */
		pix->bytesperline =1024;// 2048
	}else{
		for (i = 0; i < (ARRAY_SIZE(ov7690_dvp_win_sizes));i ++)
		{
			if (pix->width == ov7690_dvp_win_sizes[i].width && pix->height == ov7690_dvp_win_sizes[i].height)
				break;
			else if(pix->width == 144)
			{
				i = 0;
				break;
			}
		}

		if (i>= (ARRAY_SIZE(ov7690_dvp_win_sizes))){
			printk(KERN_ERR"invalid size request for preview! %d %d  \n",
				pix->width, pix->height);
				//return -EINVAL;
			}
		pix->bytesperline = pix->width*ov7690_dvp_formats[index].bpp/8;
		pix->sizeimage = pix->height*pix->bytesperline;
	}
	return 0;
}

/*
 * Set a format.
 */
static int ov7690_dvp_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret = 0;
	OV7690_WREG    *pregs = NULL;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	ret = ov7690_dvp_try_fmt(c, fmt);
	if (ret == -EINVAL)
	{
		printk(KERN_ERR"try fmt error\n");
		return ret;
	}
	switch(pix->pixelformat)
	{
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YUV422P:
		case V4L2_PIX_FMT_YUV420:
			pregs = ov7690_dvp_jpeg_to_yuv;
			ret = ov7690_dvp_write_array(c, pregs);
			if(ret < 0 )
				goto out;
			switch (pix->width )
			{
				case CIF_WIDTH:
					pregs = ov7690_dvp_fmt_yuv422_cif;
					printk(KERN_ERR"choose cif setting \n");
					break;
				case 144:
					pregs = ov7690_dvp_fmt_yuv422_qcif_rotate;
					printk(KERN_ERR"choose qcif rotate setting \n");
					break;
				case QCIF_WIDTH:
					pregs = ov7690_dvp_fmt_yuv422_qcif;
					printk(KERN_ERR"choose qcif setting \n");
					break;
				case QVGA_WIDTH:
					pregs = ov7690_dvp_fmt_yuv422_qvga;
					printk(KERN_ERR"choose qvga setting \n");
					break;
				case D1_WIDTH:
					pregs = ov7690_dvp_fmt_yuv422_d1;
					printk(KERN_ERR"choose d1 setting \n");
					break;
				case VGA_WIDTH:
					pregs = ov7690_dvp_fmt_yuv422_vga;
					printk(KERN_ERR"choose vga setting \n");
					break;
				case 2048:
					pregs = ov7690_dvp_fmt_yuv422_qxga;
					printk(KERN_ERR"choose 3M yuv setting \n");
					break;

				default:
					printk("\n unsupported size for preview! %s %d w=%d h=%d\n", __FUNCTION__, __LINE__, pix->width, pix->height);
					goto out;
					break;
			}
			break;
			case V4L2_PIX_FMT_JPEG:
				switch (pix->width )
				{

					case 2592:
						pregs = ov7690_dvp_fmt_jpeg_5M;
						ret = ov7690_dvp_set_jpegdata_size(c, pix->bytesperline, pix->sizeimage / pix->bytesperline );
						if(ret < 0 )
							goto out;
						printk(KERN_ERR"choose 5M jpeg setting \n");
						printk(KERN_ERR" bytesperline %d height %d\n", pix->bytesperline, pix->sizeimage / pix->bytesperline);
						break;
					case 2048:
						pregs = ov7690_dvp_fmt_jpeg_qxga;
						ret = ov7690_dvp_set_jpegdata_size(c, pix->bytesperline, pix->sizeimage / pix->bytesperline );
						if(ret < 0 )
							goto out;
						printk(KERN_ERR"choose vga jpeg setting \n");
						printk(KERN_ERR" bytesperline %d height %d\n", pix->bytesperline, pix->sizeimage / pix->bytesperline);
						break;

					case 1600:
						pregs = ov7690_dvp_fmt_jpeg_uxga;
						ret = ov7690_dvp_set_jpegdata_size(c, pix->bytesperline, pix->sizeimage / pix->bytesperline );
						if(ret < 0 )
							goto out;
						printk(KERN_ERR"choose qvga jpeg setting \n");
						printk(KERN_ERR" bytesperline %d height %d\n", pix->bytesperline, pix->sizeimage / pix->bytesperline);
						break;

					case 1024:
						pregs = ov7690_dvp_fmt_jpeg_sxga;
						ret = ov7690_dvp_set_jpegdata_size(c, pix->bytesperline, pix->sizeimage / pix->bytesperline );
						if(ret < 0 )
							goto out;
						printk(KERN_ERR"choose qxga jpeg setting \n");
						printk(KERN_ERR" bytesperline %d height %d\n", pix->bytesperline, pix->sizeimage / pix->bytesperline);
						break;

					default:
						printk(KERN_ERR"unsupported JPEG format ! %s %d\n", __FUNCTION__, __LINE__);
						ret = -EINVAL;
						goto out;
						break;
				}
				break;

		default:
			printk("\n unsupported format! %s %d\n", __FUNCTION__, __LINE__);
			break;
	}
	ret = ov7690_dvp_write_array(c, pregs);
out:
	return ret;
}


static int ov7690_dvp_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

//static int ov7690_dvp_firmware_download(struct i2c_client *client,  __s32  value)
//{
//	return 0;
//}

static int ov7690_dvp_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
	//return ov7690_dvp_write(c, REG_CLKRC, clkrc);
}

static int ov7690_dvp_s_input(struct i2c_client *c, int *id)
{
	ov7690_dvp_write(c, 0x12, 0x80);
	msleep(5);
	return ov7690_dvp_write_array(c, ov7690_dvp_fmt_global_init);
}

int ov7690_dvp_initialize(void)
{
	printk("%s\n", __FUNCTION__);
	if(g_ov7690_client == NULL)
	{
		if(g_main_sensor_client != NULL)
		{
			printk("%s g_ov7690_client\n", __FUNCTION__);
			g_main_sensor_client->addr = 0x21;
			ov7690_dvp_s_input(g_main_sensor_client, 0);
			
			if(main_camera_id == S5K5CA_SENSOR_CAMERA)
				g_main_sensor_client->addr = 0x2d;
			else
				g_main_sensor_client->addr = 0x3c;
		}
	}
	else
	{
		printk("%s 7690\n", __FUNCTION__);
		ov7690_dvp_s_input(g_ov7690_client, 0);
	}
	
	return 0;
}
EXPORT_SYMBOL(ov7690_dvp_initialize);
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

static int ov7690_t_sat(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s\n", __FUNCTION__);

	g_saturation = value;

	return ret;
}

static int ov7690_q_sat(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_saturation;
	return 0;
}

static int ov7690_t_hue(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s\n", __FUNCTION__);

	g_hue = value;

	return ret;
}


static int ov7690_q_hue(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_hue;
	return 0;
}

static int ov7690_t_brightness(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s value: %d\n", __FUNCTION__, value);

	//value = value + 4;
	///value:0,1,2,3.default is 3
	switch (value)
	{
		case 0:
			ov7690_dvp_write_array(client, OV7690SET_Brightness_N2);
			break;

		case 1:
			ov7690_dvp_write_array(client, OV7690SET_Brightness_N1);
			break;

		case 2:
			ov7690_dvp_write_array(client, OV7690SET_Brightness_00);
			break;

		case 3:
			ov7690_dvp_write_array(client, OV7690SET_Brightness_P1);
			break;

		case 4:
			ov7690_dvp_write_array(client, OV7690SET_Brightness_P2);
			break;
			

	}
	
	g_brightness = value;

	return ret;
}

static int ov7690_q_brightness(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_brightness;
	return 0;
}

static int ov7690_t_contrast(struct i2c_client *client, int value)
{
	int ret = 0;
	int contrast_val;
	//printk("%s value: %d\n", __FUNCTION__, value);

	//contrast_val = ((value - 100) * 6) / 100;
	switch (value)
	{
		case 0:
			ov7690_dvp_write_array(client, OV7690SET_Contrast_N2);
			break;

		case 1:
			ov7690_dvp_write_array(client, OV7690SET_Contrast_N1);
			break;

		case 2:
			ov7690_dvp_write_array(client, OV7690SET_Contrast_00);
			break;

		case 3:
			ov7690_dvp_write_array(client, OV7690SET_Contrast_P1);
			break;

		case 4:
			ov7690_dvp_write_array(client, OV7690SET_Contrast_P2);
			break;

	}
	
	g_contrast = value;

	return ret;
}

static int ov7690_q_contrast(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_contrast;
	return 0;
}

static int ov7690_t_hflip(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s\n", __FUNCTION__);
	g_hflip = value;

	return ret;
}

static int ov7690_q_hflip(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_hflip;
	return 0;
}

static int ov7690_t_vflip(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s\n", __FUNCTION__);
	g_vflip = value;

	return ret;
}

static int ov7690_q_vflip(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_vflip;
	return 0;
}

static int ov7690_t_whitebalance(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s\n", __FUNCTION__);

	switch (value)
	{
		case CAM_WHITEBALANCEMODE_CLOUDY:
			ret = ov7690_dvp_write_array(client, OV7690SET_WB_CLOUD );
		break;

		case CAM_WHITEBALANCEMODE_DAYLIGHT:
			ret = ov7690_dvp_write_array(client, OV7690SET_WB_DAYLIGHT);
		break;

		case CAM_WHITEBALANCEMODE_INCANDESCENT:
			ret = ov7690_dvp_write_array(client, OV7690SET_WB_INCANDESCENCE);
		break;

		case CAM_WHITEBALANCEMODE_FLUORESCENT1:
			ret = ov7690_dvp_write_array(client, OV7690SET_WB_FLUORESCENT);
		break;
		
		case CAM_WHITEBALANCEMODE_AUTO:
		default:
			ret = ov7690_dvp_write_array(client, OV7690SET_WB_AUTO );
		break;
	}
	g_whitebalance = value;

	return ret;
}

static int ov7690_q_whitebalance(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_whitebalance;
	return 0;
}

static int ov7690_t_exposure(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s\n", __FUNCTION__);
	g_exposure = value;

	return ret;
}

static int ov7690_q_exposure(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_exposure;
	return 0;
}

static int ov7690_t_evoffset(struct i2c_client *client, int value)
{
	int ret = 0;
	//printk("%s\n", __FUNCTION__);

	g_evoffset = value;

	return ret;
}

static int ov7690_q_evoffset(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = 1;//8;//g_evoffset;
	return 0;
}

static int ov7690_t_coloreffect(struct i2c_client *client, int value)
{
	int ret = 0;
	printk("%s value = %d\n", __FUNCTION__, value);
	
	switch (value)
	{
		case CAM_COLOREFFECT_GRAYSCALE:
			ret = ov7690_dvp_write_array(client, OV7690SET_SPE_BlackWhite);
		break;

		case CAM_COLOREFFECT_SEPIA:
			ret = ov7690_dvp_write_array(client, OV7690SET_SPE_Sepia);
		break;

		case CAM_COLOREFFECT_NEGATIVE:
			ret = ov7690_dvp_write_array(client, OV7690SET_SPE_Negative);
		break;

		case CAM_COLOREFFECT_VIVID:
			ret = ov7690_dvp_write_array(client, OV7690SET_SPE_Bluish);
		break;

		case CAM_COLOREFFECT_SOLARIZE:
			ret = ov7690_dvp_write_array(client, OV7690SET_SPE_Reddish);
		break;

		case CAM_COLOREFFECT_OFF:
		default:
			ret = ov7690_dvp_write_array(client, OV7690SET_SPE_Normal);
		break;
	}
	g_coloreffect = value;

	return ret;
}

static int ov7690_q_coloreffect(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_coloreffect;
	return 0;
}

static int ov7690_t_sharpness(struct i2c_client *client, int value)
{
	int ret = 0;
	printk("%s\n", __FUNCTION__);
	g_sharpness = value;

	return ret;
}

static int ov7690_q_sharpness(struct i2c_client *client, __s32 *value)
{
	printk("%s\n", __FUNCTION__);
	*value = g_sharpness;

	return 0;
}

static int ov7690_t_iso(struct i2c_client *c, int value)
{
	//printk("%s\n", __FUNCTION__);

	g_iso = value;
	return 0;
}
	
static int ov7690_q_iso(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = 4;//g_iso;
	return 0;
}

static int ov7690_t_bandfilter(struct i2c_client *c, int value)
{
	//printk("%s\n", __FUNCTION__);

	g_bandfilter = value;
	return 0;
}
	
static int ov7690_q_bandfilter(struct i2c_client *client, __s32 *value)
{
	//printk("%s\n", __FUNCTION__);
	*value = g_bandfilter;
	return 0;
}

static int ov7690_dvp_t_digitalzoom(struct i2c_client *c, int value)
{
	//save the value
	int ov7690_zoomvalue;
	//printk("%s\n", __FUNCTION__);
	ov7690_zoomvalue = value;
	
	
	
	return 0;
}

static int ov7690_dvp_q_digitalzoom(struct i2c_client *client, __s32 *value)
{
	*value = 5;//ov7690_zoomvalue;;
	//printk("%s\n", __FUNCTION__);
	return 0;
}

static int ov7690_t_flash(struct i2c_client *c, int value)
{
	//printk("%s\n", __FUNCTION__);
    return 0;
}

static int ov7690_q_flash(struct i2c_client *client, __s32 *value)
{
	*value = 1;
	//printk("%s\n", __FUNCTION__);
	return 0;
}

static int ov7690_t_focus(struct i2c_client *c, int value)
{
	//printk("%s\n", __FUNCTION__);
    return 0;
}

static int ov7690_q_focus(struct i2c_client *client, __s32 *value)
{
	*value = 1;
	//printk("%s\n", __FUNCTION__);
	return 0;
}

static struct ov7690_dvp_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} ov7690_dvp_controls[] =
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
		.tweak = ov7690_t_brightness,
		.query = ov7690_q_brightness,
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
		.tweak = ov7690_t_contrast,
		.query = ov7690_q_contrast,
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
		.tweak = ov7690_t_sat,
		.query = ov7690_q_sat,
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
		.tweak = ov7690_t_hue,
		.query = ov7690_q_hue,
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
		.tweak = ov7690_t_vflip,
		.query = ov7690_q_vflip,
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
		.tweak = ov7690_t_hflip,
		.query = ov7690_q_hflip,
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
		.tweak = ov7690_t_whitebalance,
		.query = ov7690_q_whitebalance,
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
		.tweak = ov7690_t_exposure,
		.query = ov7690_q_exposure,
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
		.tweak = ov7690_t_evoffset,
		.query = ov7690_q_evoffset,
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
		.tweak = ov7690_t_coloreffect,
		.query = ov7690_q_coloreffect,
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
		.tweak = ov7690_dvp_t_digitalzoom,
		.query = ov7690_dvp_q_digitalzoom,
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
		.tweak = ov7690_t_bandfilter,
		.query = ov7690_q_bandfilter,
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
		.tweak = ov7690_t_sharpness,
		.query = ov7690_q_sharpness,
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
		.tweak = ov7690_t_iso,
		.query = ov7690_q_iso,
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
		.tweak = ov7690_t_flash,
		.query = ov7690_q_flash,
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
		.tweak = ov7690_t_focus,
		.query = ov7690_q_focus,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov7690_dvp_controls))
static struct ov7690_dvp_control *ov7690_dvp_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov7690_dvp_controls[i].qc.id == id)
			return ov7690_dvp_controls + i;
	return NULL;
}


static int ov7690_dvp_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct ov7690_dvp_control *ctrl = ov7690_dvp_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int ov7690_dvp_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov7690_dvp_control *octrl = ov7690_dvp_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int ov7690_dvp_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov7690_dvp_control *octrl = ov7690_dvp_find_control(ctrl->id);
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
static int __devinit ov7690_dvp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct ov7690_dvp_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;
	//ccic_set_clock_parallel();
	client->addr = 0x21;
	g_ov7690_client = client;
	pdata->power_on(1, 0);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct ov7690_dvp_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &ov7690_dvp_formats[1];
	info->sat = 128;
	i2c_set_clientdata(client, info);

	ret = ov7690_dvp_detect(client);
	if (ret)
		goto out_free;
	ret = ccic_sensor_attach(client);
	if (ret)
		goto out_free;

out_free:
	pdata->power_on(0, 0);
	//ccic_disable_clock();

	//if(ret)
	//	kfree(info);
	if (ret) {
		dev_err(&client->dev, "init failed\n");
		g_ov7690_client = NULL;
		kfree(info);
	}
	
	return ret;
}


static int ov7690_dvp_remove(struct i2c_client *client)
{
	printk(KERN_ERR"remove do nothing \n");
	return 0;	//TODO
}


static int ov7690_dvp_streamon(struct i2c_client *client)
{


	unsigned char val;
	ov7690_dvp_read(client, 0x0c, &val);
	val |= 0x06;
	ov7690_dvp_write(client, 0x0c, val);
	
	ov7690_dvp_read(client, 0x0e, &val);
	val &= ~0x01;
	val |= 0x06;
	ov7690_dvp_write(client, 0x0e, val);
	return 0;
#if 0
	int ret = 0;
	ret = ov7690_dvp_write(client, 0x4201, 0x00);
	if(ret < 0)
		goto out;
	ret = ov7690_dvp_write(client, 0x4202, 0x00);
	if(ret < 0)
		goto out;
out:
	return ret;
#endif
}

static int ov7690_dvp_streamoff(struct i2c_client *client)
{

	unsigned char val;
	ov7690_dvp_read(client, 0x0c, &val);
	val &= ~0x06;
	ov7690_dvp_write(client, 0x0c, val);
	
	ov7690_dvp_read(client, 0x0e, &val);
	val |= 0x01;
	val &= ~0x06;
	ov7690_dvp_write(client, 0x0e, val);
	return 0;
#if 0
	int ret = 0;
	ret = ov7690_dvp_write(client, 0x4201, 0x01);
	if(ret < 0)
		goto out;
	ret = ov7690_dvp_write(client, 0x4202, 0x00);
	if(ret < 0)
		goto out;
out:
	return ret;
#endif
}


static int ov7690_dvp_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov7690_dvp_read(client, (u16)reg->reg, (unsigned char *)&(reg->val));
}

static int ov7690_dvp_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return ov7690_dvp_write(client, (u16)reg->reg, (unsigned char)reg->val);
}

static int ov7690_dvp_command(struct i2c_client *client, unsigned int cmd,	void *arg)
{
	switch (cmd)
	{
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_OV7690, 0);
		case VIDIOC_INT_RESET:
			ov7690_dvp_reset(client);
			return 0;
			//		case VIDIOC_INT_INIT:

			//			return 0;//ov7690_dvp_init(client);		//TODO - should get 3640 default register values
		case VIDIOC_QUERYCAP:
			return ov7690_dvp_querycap(client, (struct v4l2_capability *) arg);
		case VIDIOC_ENUM_FMT:
			return ov7690_dvp_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_TRY_FMT:
			return ov7690_dvp_try_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_S_FMT:
			return ov7690_dvp_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_ENUM_FRAMESIZES:
			return ov7690_dvp_enum_fmsize(client, (struct v4l2_frmsizeenum *) arg);
		case VIDIOC_QUERYCTRL:
			return ov7690_dvp_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return ov7690_dvp_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return ov7690_dvp_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return ov7690_dvp_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return ov7690_dvp_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return ov7690_dvp_s_input(client, (int *) arg);
		case VIDIOC_STREAMON:
			return ov7690_dvp_streamon(client);
		case VIDIOC_STREAMOFF:
			return ov7690_dvp_streamoff(client);
		case VIDIOC_DBG_G_REGISTER:
			return ov7690_dvp_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return ov7690_dvp_s_register(client, (struct v4l2_dbg_register *) arg);
		default:
			return -EINVAL;
	}
}

static struct i2c_device_id ov7690_dvp_idtable[] = {
	{ "ov7690", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov7690_dvp_idtable);

static struct i2c_driver ov7690_dvp_driver =
{
	.driver = {
		.name	= "ov7690",
	},
	.id_table         = ov7690_dvp_idtable,
	.command	= ov7690_dvp_command,
	.probe		= ov7690_dvp_probe,
	.remove		= ov7690_dvp_remove,
};


/*
 * Module initialization
 */
static int __init ov7690_dvp_mod_init(void)
{
	printk(KERN_ERR"OmniVision ov7690_dvp sensor driver at your service\n");
	return i2c_add_driver(&ov7690_dvp_driver);
}

static void __exit ov7690_dvp_mod_exit(void)
{
	i2c_del_driver(&ov7690_dvp_driver);
}

late_initcall(ov7690_dvp_mod_init);
module_exit(ov7690_dvp_mod_exit);


