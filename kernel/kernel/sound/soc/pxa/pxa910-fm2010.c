/*
 *  Copyright (C) 2008-2009 Foxconn K.H. Fan
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <asm/io.h>
#include <mach/gpio.h>
#include <mach/pxa910-aec-fm2010.h>
//#include <mach/pxa-regs.h>
//#include <mach/mfp-pxa9xx.h>
//#include <mach/u900_d9035_board.h>
#include <plat/mfp.h>

// major & minor define area
#define AEC_MAJOR       18                   /* dynamic major by default */
#define AEC_MINOR       5

#ifdef AEC_MAJOR
static int aec_major =   AEC_MAJOR;
#else
static int aec_major =   0;
#endif

#ifdef AEC_MINOR
static int aec_minor =   AEC_MINOR;
#else
static int aec_minor =   0;
#endif

static int aec_nr_dev = 1;
static int g_mode = AEC_NORMAL_MODE;
static unsigned int aec_count = 0;


/* Initial values */
static  struct aec_reg  fm2010_param_spk[]=
{
	/*---  5----*/
	{0x1E30,0x0231,},
	{0x1E34,0x0069,},//0x6b-->0x69	
	{0x1E36,0x0013,},	
	{0x1E3D,0x0400,},
	{0x1E3E,0x0100,},	
	  /*-8------*/  
	{0x1E41,0x0101,},
	{0x1E44,0x0081,},
	{0x1E45,0x03CF,}, 
	{0x1E46,0x0010,},//0x11-->0x10, // 0x0011->0x0010:disable noise paste back.
	{0x1E47,0x3000,}, // 0x2000->0x3000
	{0x1E48,0x1000,},
	{0x1E49,0x0880,}, 
	{0x1E4D,0x0280,},//0x02,0xc0,//0x02,0x80, // ------0x0280-->0x0180->0x0080 
	 /*-2---*/
	{0x1E51,0xC000,},
	{0x1E52,0x0013,}, 
	 /*-1---*/
	{0x1E63,0x0003,},
	   /*-1---*/
	{0x1E70,0x05C0,},
	   /*-6---*/
	{0x1E86,0x0009,}, 
	{0x1E87,0x0005,},
	{0x1E88,0x3800,}, 
	{0x1E89,0x0001,},
	{0x1E8B,0x0080,},
	{0x1E8C,0x0010,}, 
	   /*-1---*/
	{0x1E92,0x7800,},
	   /*-3---*/
	{0x1EA0,0x0400,},// 0x1000->0x0400:noise gain 
	{0x1EA1,0x3300,},
	{0x1EA2,0x3200,},
	   /*-3---*/ 
	{0x1EBC,0x6800,},
	{0x1EBD,0x0100,},
	{0x1EBF,0x7000,},
	 	/*-9---*/
	{0x1EC0,0x2680,},
	{0x1EC1,0x1080,},
	{0x1EC5,0x2B06,},
	{0x1EC6,0x0C1F,},
	{0x1EC8,0x2879,},
	{0x1EC9,0x65AB,},
	{0x1ECA,0x4026,},
	{0x1ECB,0x7FFF,},
	{0x1ECC,0x7FFE,},
	   /*-3---*/
	{0x1EF8,0x0400,},
	{0x1EF9,0x0100,},
	{0x1EFF,0x4B00,},
	   /*-4---*/
	{0x1F00,0x7FFF,},
	{0x1F0A,0x0A00,},
	{0x1F0C,0x0100,},
	{0x1F0D,0x7800,},
	   /*-1---*/
	{0x1E3A,0x0000,},
         /*--the end--*/
	{0x1E3A,0x0000,},
};

static  struct aec_reg  fm2010_param_ep[]=
{                                                 
	/*-5------*/      
	{0x1E30,0x0231,}, 
	{0x1E34,0x00AA,},	
	{0x1E36,0x001D,},	
	{0x1E3D,0x0200,},	
	{0x1E3E,0x0100,},	
	/*-8---*/         
	{0x1E41,0x0101,}, 
	{0x1E44,0x0081,}, 
	{0x1E45,0x03CF,}, 
	{0x1E46,0x0011,}, 
	{0x1E47,0x2000,},	
	{0x1E48,0x1000,}, 
	{0x1E49,0x0880,}, 
	{0x1E4D,0x0180,}, 
	/*-2---*/         
	{0x1E51,0xC000,}, 
	{0x1E52,0x0013,}, 
	/*-1---*/     
	{0x1E63,0x0003,}, 
	/*-1---*/    
	{0x1E70,0x05C0,}, 
	/*-6---*/	  
	{0x1E86,0x0006,}, 
	{0x1E87,0x0002,}, 
	{0x1E88,0x3800,}, 
	{0x1E89,0x0001,}, 
	{0x1E8B,0x0080,}, 
	{0x1E8C,0x0010,}, 
	/*-1---*/    
	{0x1E92,0x7800,}, 
	/*-3---*/   
	{0x1EA0,0x1000,}, 
	{0x1EA1,0x3300,}, 
	{0x1EA2,0x3200,}, 
	/*-3---*/    
	{0x1EBC,0x6800,}, 
	{0x1EBD,0x0100,}, 
	{0x1EBF,0x7000,}, 
	/*-9---*/	
	{0x1EC0,0x2680,}, 
	{0x1EC1,0x1080,}, 
	{0x1EC5,0x2B06,}, 
	{0x1EC6,0x0C1F,}, 
	{0x1EC8,0x2879,}, 
	{0x1EC9,0x65AB,}, 
	{0x1ECA,0x4026,}, 
	{0x1ECB,0x7FFF,}, 
	{0x1ECC,0x7FFE,}, 
	/*-3---*/	
	{0x1EF8,0x0400,}, 
	{0x1EF9,0x0100,}, 
	{0x1EFF,0x4B00,}, 
	/*-4---*/     
	{0x1F00,0x7FFF,}, 
	{0x1F0A,0x0A00,}, 
	{0x1F0C,0x0100,}, 
	{0x1F0D,0x7800,}, 
	/*-1---*/	        
	{0x1E3A,0x0000,}, 
        /*- the end- */
	{0x0000,0x0000,}, 
};

static  struct aec_reg  fm2010_param_handset_nss[]=
{
	/*-3------*/                                          
	{0x1E34,0x00DB,},	                                      
	{0x1E36,0x0013,},	                                      
	{0x1E3D,0x0200,},                                       
	/*-6---*/                                             
	{0x1E45,0x000F,},                                        
	{0x1E46,0x0014,},                                        
	{0x1E47,0x1900,},	                                      
	{0x1E48,0x0880,},                                        
	{0x1E49,0x0880,},                                        
	{0x1E4D,0x0001,},                                        
	/*-3---*/                                             
	{0x1E51,0xC000,},                                        
	{0x1E52,0x0013,},                                        
	{0x1E57,0x7FFF,},                                        
	/*-1---*/                                             
	{0x1E70,0x05C0,},                                        
	/*-8---*/                                             
	{0x1EB3,0x1200,},                                        
	{0x1EB4,0x1700,},                                        
	{0x1EB5,0x1100,},                                        
	{0x1EB6,0x0100,},                                        
	{0x1EBC,0x5000,},                                        
	{0x1EBD,0x1800,},                                        
	{0x1EBE,0x2800,},                                        
	{0x1EBF,0x7FFF,},                                        
	/*-4---*/	                                            
	{0x1EC8,0x7FFF,},                                        
	{0x1EC9,0x7FFF,},                                        
	{0x1ECB,0x7FFF,},                                        
	{0x1ECC,0x7FFF,},                                        
	/*-6---*/	                                            
	{0x1ED5,0x4000,},                                        
	{0x1ED6,0x4000,},                                        
	{0x1ED7,0x4000,},                                        
	{0x1ED8,0x3800,},                                        
	{0x1ED9,0x2E00,},                                        
	{0x1EDA,0x5400,},	                                      
	/*-2---*/	                                            
	{0x1EE2,0x000A,},                                        
	{0x1EE3,0x1000,},                                        
	/*-2---*/	                                            
	{0x1EF8,0x0880,},                                        
	{0x1EF9,0x0700,},                                        
	/*-6---*/                                             
	{0x1F00,0x32F4,},                                        
	{0x1F01,0x2B00,},	//2d00---->2b00                       
	{0x1F0A,0x0100,},                                        
	{0x1F0B,0x0300,},//x01,0x00-->	0x03,0x00               
	{0x1F0C,0x1200,},                                        
	{0x1F0D,0x0C00,},                                        
	/*-1---*/	                                            
	{0x1E3A,0x0000,},                                        
       /*--the end--*/
	{0x0000,0x0000,},
};



extern void enable_oscc_pout_aec(void);
extern void disable_oscc_pout_aec(void);

static void ghost_DevRelease(struct device *dev)
{
	
}

#if 0
static struct device ghost_device = {
	.bus_id    = "aec_param",
	.release   = ghost_DevRelease,
};
#endif 

struct i2c_client *gAecClient = NULL;

static int aec_softReset(void)
{
	char buf[10];
	int ret;

	printk("[AEC]aec softReset\n");

	buf[0] = 0xfc;
	buf[1] = 0xf3;
	buf[2] = 0x6a;
	buf[3] = 0x2a;
	buf[4] = 0x00;
	buf[5] = 0x30;

	ret = i2c_master_send(gAecClient, buf, 6);
	if (ret !=6)
	{
		printk("aec send data error\n");
		return -EIO;
	}

	msleep(100);

	return 0;
}


/*
	read one byte from the aec chip's register(0x25 or 0x26), the register is 1byte align;
	return 0, success
*/

static int aec_data_read(u8 reg, u8 *data)
{
	char buf[10];
	int ret;

	if(reg != 0x25 && reg != 0x26){
		printk(KERN_WARNING"error register addr %x\n", reg);	
		return -EIO;
	}
	buf[0] = 0xfc;
	buf[1] = 0xf3;
	buf[2] = 0x60;
	buf[3] = reg;
	ret = i2c_master_send(gAecClient, buf, 4);
	if (ret !=4) {
		printk("aec send data error\n");
		return -EIO;
	}
	ret = i2c_master_recv (gAecClient,  buf, 1);
	if (ret <=0) {
		printk("aec send data error\n");
		return -EIO;
	}
	*data = buf[0];
	return 0;
}

/*
	send two byte to the aec chip's memory, the memory is 2bytes align;
	return 0, success
	-1, fails
*/
int aec_mem_write(unsigned short mem_addr, unsigned short data)
{
	char aec_buf[10];
	int ret;
/*
	if ((mem_addr < 0x1e30) || (mem_addr > 0x1e40))
		return -1;
*/
	//two sync byte
	aec_buf[0] = 0xfc;
	aec_buf[1] = 0xf3; 
	//command byte
	aec_buf[2] = 0x3b;

	//fm2010's memory addr
	aec_buf[3] = (mem_addr>>8) & 0xff;
	aec_buf[4] = mem_addr & 0xff;

	//the data to the memory
	aec_buf[5] = (data>>8) & 0xff;
	aec_buf[6] = data & 0xff;
	
	ret = i2c_master_send(gAecClient, aec_buf, 7);
	if (ret != 7)
	{
        printk("aec aec_mem_write fail, ret=%d\n", ret);
		return -EIO;
    }
	return 0;
}

/*
	read two byte to the aec chip's memory, the memory is 2bytes align;
	return 1, success
	0, fails
*/
int aec_mem_read(unsigned short mem_addr, unsigned short *data)
{
	char aec_buf[10];
	char data_buf[2] = {0};
	int ret;
/*
	if ((mem_addr < 0x1e30) || (mem_addr > 0x1e40))
		return -EIO;
*/
	//two sync byte
	aec_buf[0] = 0xfc;
	aec_buf[1] = 0xf3; 
	//command byte
	aec_buf[2] = 0x37;

	//fm2010's memory addr
	aec_buf[3] = (mem_addr>>8) & 0xff;
	aec_buf[4] = mem_addr & 0xff;
	
	ret = i2c_master_send(gAecClient, aec_buf, 5);
	if (ret != 5) {
		return -EIO;
	}
	
	ret = aec_data_read(0x26, &data_buf[0]);
	if (ret) {
		printk("aec reg %x read error\n", 0x26);
		return -EIO;
	}
	ret = aec_data_read(0x25, &data_buf[1]);
	if (ret) {
		printk("aec reg %x read error\n", 0x25);
		return -EIO;
	}

	*data = data_buf[0];
	*data = (*data << 8) | data_buf[1];
	return  0;
}

static int aec_set_regs_default(int mode)
{
    struct aec_platform_data *pdata;
	int ret = 0;
	aec_reg_t *ptr;
	unsigned int cnt = 0;


	  pdata = gAecClient->dev.platform_data;
	  if(!pdata)
	        return -1;

         ptr = pdata->parmGet(mode);
	  if(!ptr)
	        return -1;
	        
	  while(ptr->reg != 0x0000)
	   {
		ret = aec_mem_write(ptr->reg, ptr->val);
		cnt ++;
		if (ret < 0)
		{
			printk(KERN_ERR"[AEC] cannnot initialize aec chip, cnt=%d\n", cnt);
			return -EIO;
		}
		ptr++;
	   }

	    msleep(200);        	
	return 0;
}

static unsigned char aec_devTest(struct aec_platform_data *pdata)
{
	unsigned short test_data;
	unsigned short org_data;
	unsigned short dsp_test;
	int cnt;
	static unsigned short randData;
	printk("enter aec_devTest\n");
	randData += 3;
	/*I2C check*/
	aec_mem_read(0x1e9f, &org_data);	
	aec_mem_write(0x1e9f, randData);
	aec_mem_read(0x1e9f, &test_data);
	printk("read from memory 0x1e9f, res:%x\n", test_data);
       if(test_data != randData)
        return 0;
    
       aec_mem_write(0x1e9f, org_data);
   	/*Bypass check*/
   	aec_mem_read(0x3fe1, &test_data);
   	printk("read from memory 0x3fe1, res:%x\n", test_data);
   	if(test_data & 0x0020)
   	{
   	    printk("AEC in bypass mode\n");
   	    aec_mem_read(0x3fc0, &test_data);
   	    aec_mem_read(0x1e4a, &dsp_test);
   	    if(test_data != dsp_test)
   	    {
   	        printk("[AEC] bypass mode param not load\n");
   	        return 0;
   	    }
	    if(NULL != pdata->set_mode)
   	    	pdata->set_mode(AEC_NORMAL_MODE);
   	    aec_mem_read(0x3fe1, &test_data);
   	    if(test_data & 0x0020)
   	        return 0;
	    if(NULL != pdata->set_mode)
   	    	pdata->set_mode(AEC_BYPASS_MODE);
   	}
   	else
   	{
   	    printk("AEC in normal mode\n");
   	    aec_mem_read(0x3fc0, &test_data);
   	    aec_mem_read(0x1e34, &dsp_test);
   	    if(test_data != dsp_test)
   	    {
   	        printk("[AEC] normal mode param not load\n");
   	        return 0;
   	    }
   	    /*In/Out test*/
   	    cnt=10;

    	aec_mem_read(0x3fc4, &test_data);
    	printk("read from memory 0x3fc4, res1:%x\n", test_data);
RETRY1:
    	msleep(1);
    	aec_mem_read(0x3fc4, &dsp_test);
    	printk("read from memory 0x3fc4, res2:%x\n", dsp_test);
    	if(test_data == dsp_test)
    	{
    	    if(cnt>0)
    	    {
    	        cnt--;
    	        goto RETRY1;
    	    }
    	    else
    	        return 0;


    	}
    	cnt=10;

    	aec_mem_read(0x3fc9, &test_data);
    	printk("read from memory 0x3fc9, res1:%x\n", test_data);
RETRY2:
    	msleep(1);
    	aec_mem_read(0x3fc9, &dsp_test);
    	printk("read from memory 0x3fc9, res2:%x\n", dsp_test);
    	if(test_data == dsp_test)
    	{
    	    if(cnt>0)
    	    {
    	        cnt--;
    	        goto RETRY2;
    	    }
    	    else
    	        return 0;

    	}
	    if(NULL != pdata->set_mode)
   	    	pdata->set_mode(AEC_BYPASS_MODE);
   	    aec_mem_read(0x3fe1, &test_data);
   	    if(!(test_data & 0x0020))
   	        return 0;
	    if(NULL != pdata->set_mode)
   	    	pdata->set_mode(AEC_NORMAL_MODE);
   	}
   	
   	/*DSP check*/
	printk("Start DSP check\n");
	aec_mem_read(0x1e65, &test_data);
	printk("read from memory 0x1e65, res1:%x\n", test_data);
	msleep(10);
	aec_mem_read(0x1e65, &dsp_test);
	printk("read from memory 0x1e65, res2:%x\n", dsp_test);
	if(test_data == dsp_test)
	    return 0;
	
	return 1;	
}

static int aec_suspend(struct i2c_client * aec, pm_message_t state)
{
#if 0
    struct aec_platform_data *pdata;
printk("enter aec_suspend\n");
  //  enable_oscc_pout_aec();
    	
	pdata = aec->dev.platform_data;

	if(NULL != pdata->sleep)
		pdata->sleep();

 //   disable_oscc_pout_aec();
#endif
	return 0;
}

static int aec_resume(struct i2c_client * aec)
{
#if 0
	struct aec_platform_data *pdata;
     printk("enter aec_resume\n");
     //  enable_oscc_pout_aec();
    	
	pdata = aec->dev.platform_data;

	if(NULL != pdata->wakeup)
		pdata->wakeup();

   //   disable_oscc_pout_aec();
#endif
	return 0;
}


static int aec_devPwrCtrl(int on)
{
    struct aec_platform_data *pdata;
    unsigned char cnt;
    unsigned short val1, val2;
printk("enter aec_devPwrCtrl\n");
    if(!gAecClient)
        return -EPERM;

    pdata = gAecClient->dev.platform_data;

    if(!pdata)
        return -EPERM;

    if(on == 1)
    {   
        cnt = 0;
        /* Perform cold start sequence. */
        do
        {
		if (0 != cnt) {
			if(NULL != pdata->sleep)
				pdata->sleep();
//			disable_oscc_pout_aec();
			msleep(10);
		}
		
	     if(NULL != pdata->wakeup)
            		pdata->wakeup(); 
//            enable_oscc_pout_aec();	
            msleep(10);
	     if(NULL != pdata->reset)
            	pdata->reset();
            msleep(20);
            aec_set_regs_default(g_mode);
	     if(NULL != pdata->set_mode)
            		pdata->set_mode(g_mode);
            /* This is workaround that performs more wakeup process if it fails to wakeup AEC. */
    	    msleep(50);            
    	    aec_mem_read(0x1e65, &val1);
    	    msleep(50);
    	    aec_mem_read(0x1e65, &val2);
    	    cnt++;
            printk("[AEC]Power on. Reset count=%d.\n", cnt);
        }while(val1==val2 && cnt<5);
    }
    else
    {
        if(NULL != pdata->sleep)
        	pdata->sleep();
        printk("[AEC]Power off.\n");
//        disable_oscc_pout_aec();
    }

    return 0;
}

static ssize_t aec_reg_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	int err = -EINVAL;
	int res;
	char cmd;
	u32 reg, val;
	struct aec_platform_data *pdata;
	unsigned char status;
	unsigned short value;
printk("enter aec_reg_store\n");
	if(!gAecClient)
		return -EPERM;

	pdata = gAecClient->dev.platform_data;
	if(!pdata)
		return -EPERM;

	err =0;
	res = sscanf(buf, "%c %x %x", &cmd, &reg, &val);	
	switch(cmd)
	{
	    case 'r':
        	if (res != 2)
        	{
        		err =  -EINVAL;
        		goto out;
        	}
	        aec_mem_read(reg, &value);
	        printk("[AEC]Read reg. :%04x, val :%04x\n", reg, value);
	        break;
	    case 'w':
        	if (res != 3)
        	{
        		err =  -EINVAL;
        		goto out;
        	}
	        aec_mem_write(reg, val);
	        printk("[AEC]Write reg. :%04x, val :%04x\n", reg, val);
	        break;
	    case 'm':  /* AEC mode */
		if (res != 2)
		{
			err =  -EINVAL;
			goto out;
		}

		switch(reg)
		{
			case AEC_NORMAL_MODE:
			    printk(KERN_INFO "[AEC] set normal mode\n");
			break;
			case AEC_BYPASS_MODE:
			    printk(KERN_INFO "[AEC] set bypass mode\n");
			break;
			case AEC_NORMAL_HANDHELD_MODE:
			    printk(KERN_INFO "[AEC] set normal handheld mode\n");
			break;
			default:
			    printk(KERN_INFO "[AEC] Invalid argument\n");
			    reg = AEC_NORMAL_MODE;
			    break;
		}
	    
	     g_mode = reg;
	    
	     aec_softReset();
	     aec_set_regs_default(g_mode);
	     if(NULL != pdata->set_mode)
	         pdata->set_mode(g_mode);
	        break;
	    case 's': /* Sleep & Wake up */
        	if (res != 2)
        	{
        		err =  -EINVAL;
        		goto out;
        	}
        	if(reg)
        	{
        	     if(NULL != pdata->sleep)
	            		pdata->sleep();
	            printk("[AEC]Sleep.\n");
        	}
        	else
        	{
        	    if(NULL != pdata->wakeup)
        	        pdata->wakeup();
        	    printk("[AEC]Wakeup.\n");
	        }
	        break;
	    case 'p': /* Power on & Power off */
        	if (res != 2)
        	{
        		err =  -EINVAL;
        		goto out;
        	}
        	if(reg)
        	{
	            aec_devPwrCtrl(1);
        	}
        	else
        	{
        	    aec_devPwrCtrl(0);
	        }
	        break;
	    case 't': /*AEC test*/
	        status = aec_devTest(pdata);
	        if(status == 0)
	            printk("[AEC]Test fail\n");
	        else
	            printk("[AEC]Test pass\n");
	        break;
	    case 'x':
	        aec_softReset();
	        break;
	    case 'd': /*Debug*/
	        aec_mem_read(0x1e65, &value);
	        printk("DSP count val:%x\n", value);
	        aec_mem_read(0x3fc4, &value);
	        printk("read from memory 0x3fc4, val:%x\n", value);
	        aec_mem_read(0x3fc9, &value);
	        printk("read from memory 0x3fc9, val:%x\n", value);
	        break;
	    default:
	        printk("[AEC]unsupported command.\n");
	        goto out;
	}

out:
	return err ? err : count;
}

//added by job046127 for standby mode
//#define FM2010_PWRDN_EN			    MFP_CFG_X(GPIO4, AF0, DS01X, DRIVE_HIGH)
//#define FM2010_PWRDN_DIS			    MFP_CFG_X(GPIO4, AF0, DS01X, DRIVE_LOW)
void fm2010_Power_control(int bflag)
{

	//mfp_cfg_t u900_fm2010power_high_cfg[]  = {FM2010_PWRDN_EN,};
	//mfp_cfg_t u900_fm2010power_low_cfg[]  = {FM2010_PWRDN_DIS,};
	
	if (gpio_request(MFP_PIN_GPIO4, "ECHO_POWERDOWN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO4);
		return -EIO;
	}
	
	if (bflag)
	{
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 1);
	//	pxa3xx_mfp_config(u900_fm2010power_high_cfg,1);
	}
	else
	{
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 0);
	//	pxa3xx_mfp_config(u900_fm2010power_low_cfg,1);

	}
	gpio_free(MFP_PIN_GPIO4);
}
void fm2010_reset(void)
{
	if (gpio_request(MFP_PIN_GPIO9, "ECHO_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO9);
		return -EIO;
	}
	mdelay(20);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO9), 1);
	mdelay(20);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO9), 0);
	mdelay(200);
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO9), 1);
	gpio_free(MFP_PIN_GPIO9);
	
}
int fm2010_init_client(unsigned int FM2010_MODE)		/* 0-Speaker, 1-Earphone, 2-handset(NSS) */
{
	unsigned short address;
	unsigned short data;
	int i, ret;
	int fm2010_init_mode;
	unsigned short org_data;
	//printk("enter fm2010_init_client FM2010_MODE=%d\n",FM2010_MODE);
	
	fm2010_init_mode = FM2010_MODE;
	if (fm2010_init_mode == 0)
	{
		for(i=0; i < (sizeof(fm2010_param_spk)/sizeof(struct aec_reg)); i++)
		{
			address = fm2010_param_spk[i].reg;
			data = fm2010_param_spk[i].val;
		
			ret = aec_mem_write(address, data);
		}
	}
	else if (fm2010_init_mode == 1)
	{
		for(i=0; i < (sizeof(fm2010_param_ep)/sizeof(struct aec_reg)); i++)
		{
			address = fm2010_param_ep[i].reg;
			data= fm2010_param_ep[i].val;

			ret = aec_mem_write(address, data);
		}
	}
	else if  (fm2010_init_mode == 2)
	{
		for(i=0; i < (sizeof(fm2010_param_handset_nss)/sizeof(struct aec_reg)); i++)
		{
			address = fm2010_param_handset_nss[i].reg;
			data = fm2010_param_handset_nss[i].val;
			
			ret = aec_mem_write(address, data);
		}
	}
	else
	{
		printk("FM2018: init mode is not correct.\n");
	}
	#if 0
	for(i=0; i < (sizeof(fm2010_param_spk)/sizeof(struct aec_reg)); i++)
		{			
			address = fm2010_param_spk[i].reg;
			
			aec_mem_read(address, &org_data);
			printk("i=%d,address=%x,org_data=%x\n",i,address,org_data);
		}	
	#endif
	return ret;
}
/* codec register dump */

static ssize_t aec_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int count = 0;
	unsigned short val;
	unsigned short special_reg;
	aec_reg_t *ptr;
	struct aec_platform_data *pdata;
	printk("enter aec_reg_show\n");
	count += sprintf(buf, "aec registers\n");

    pdata = gAecClient->dev.platform_data;
    if(!pdata)
        return 0;
        	
	ptr = pdata->parmGet(AEC_NORMAL_MODE);
	if(!ptr)
	    return 0;
	    
	while(ptr->reg != 0x0000)
	{
		aec_mem_read(ptr->reg, &val);
		count += sprintf(buf + count, "%4x: %4x\n", ptr->reg, val);
		ptr++;
	}
	
	special_reg = 0x1E65;
	aec_mem_read(special_reg, &val);
	count += sprintf(buf + count, "%4x: %4x\n", special_reg, val);

/* only for test sword*/
	special_reg = 0x3FC4;
	aec_mem_read(special_reg, &val);
	count += sprintf(buf + count, "%4x: %4x\n", special_reg, val);

	special_reg = 0x3FC9;
	aec_mem_read(special_reg, &val);
	count += sprintf(buf + count, "%4x: %4x\n", special_reg, val);
	
	return count;
}
//static DEVICE_ATTR(aec_reg, 0644, aec_reg_show, aec_reg_store);
static struct device_attribute dev_attr_aec_reg = {
	.attr = {	
		.name = "avp_fm2010",
		.owner=THIS_MODULE,
		.mode =S_IRUGO | S_IWUGO,
	},
      .show = aec_reg_show,
      .store = aec_reg_store ,
};
static int aec_probe(struct i2c_client *client)
{
	struct aec_platform_data *pdata;
	int ret;

	printk("AEC: aec_probe\n");

//	enable_oscc_pout_aec();

	pdata = client->dev.platform_data;
	
      if(NULL != pdata->init)
		pdata->init();

	gAecClient = client;
      //printk("aec_probe 00 \n");
	ret = device_create_file(&gAecClient->dev, &dev_attr_aec_reg);
	if (ret < 0)
		printk(KERN_WARNING "AEC: failed to add aec chip sysfs entries\n");
#if 0
	if(device_register(&ghost_device) < 0) {
		printk("[AEC]device_register error\n");	    
	}
#endif

       //printk("aec_probe 111 \n");
       fm2010_init_client(0);
	/* set AEC to sleep mode at probe */
	if(NULL != pdata->sleep)
		pdata->sleep();
      // printk("aec_probe 222 \n");
//	disable_oscc_pout_aec();
	return ret;
}

static int aec_remove(struct i2c_client *client)
{  
	printk("AEC: aec_remove\n");	
//	device_unregister(&ghost_device);
	device_remove_file(&gAecClient->dev, &dev_attr_aec_reg); 	
	gAecClient = NULL;
	return 0;
}
static const struct i2c_device_id fm2010_id[] = {
	{ "avp_fm2010", 0 },
	{ }
};
/* corgi i2c codec control layer */
struct i2c_driver aec_i2c_driver = {
	.driver = {
		.name = "avp_fm2010",		
	},
	.probe      = aec_probe,
	.remove     = aec_remove,
	.id_table	= fm2010_id,
	.suspend    = aec_suspend,
	.resume     = aec_resume,	
};

#if 0
// device structure
struct aec_dev {	
	struct cdev cdev;
};

struct aec_dev *aec_device;	

/*  File ops */
static int aec_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int aec_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int aec_ioctl(struct inode *inode, struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
	int i;
	int ret = 0;
	unsigned char status;	
    struct aec_platform_data *pdata;

	
	printk(KERN_EMERG "aec ioctl cmd %x \n", cmd);	
	printk(KERN_EMERG "aec arg %lx \n", arg);

	if(_IOC_TYPE(cmd) != AEC_IOC_MAGIC)
		return -ENOTTY;	

    if(!gAecClient)
        return -EPERM;

    pdata = gAecClient->dev.platform_data;

    if(!pdata)
        return -EPERM;


	switch (cmd) {
	    case AEC_IOC_SET_REGISTER:
	        {
	            S_AEC_REG_CMD_PARM parm;
	            aec_reg_t *pReg;
	            
    	        if (copy_from_user(&parm, argp, sizeof(parm)))
		            return -EFAULT;
    	        
    	        if(parm.size == 0)
    	            break;
    	            
    	        pReg = kmalloc(parm.size * sizeof(aec_reg_t), GFP_KERNEL);
    	        if (!pReg)
    	            return -ENOMEM;
    	            
    	        if (copy_from_user(pReg, parm.pReg, parm.size * sizeof(aec_reg_t)))
    	        {
    	            kfree(pReg);
		            return -EFAULT;
		        }
	
    	        for (i = 0; i < parm.size; i++){
            		ret = aec_mem_write(pReg[i].reg, pReg[i].val);
            		if (ret < 0){
            			printk(KERN_ERR"cannnot initialize aec chip\n");
            			kfree(pReg);
            			return -EIO;
            		}			

                    printk(KERN_EMERG "aec register addr: 0x%hx, set value: 0x%hx\n", pReg[i].reg, pReg[i].val);
            	}

            	kfree(pReg);
            }
	        break;
	    case AEC_IOC_CMD_TEST:
	        status = aec_devTest(pdata);
	        {
	            unsigned long rtn;
	            rtn=copy_to_user((void *)arg, &status, sizeof(unsigned char));
	        }
	        break;	
	    case AEC_IOC_SET_MODE:
            switch(arg)
            {
                case AEC_NORMAL_MODE:
                    printk(KERN_INFO "[AEC] set normal mode\n");
                    break;
                case AEC_BYPASS_MODE:
                    printk(KERN_INFO "[AEC] set bypass mode\n");
                    break;
                case AEC_NORMAL_HANDHELD_MODE:
                    printk(KERN_INFO "[AEC] set normal handheld mode\n");
                    break;
                default:
                    printk(KERN_INFO "[AEC] Invalid argument\n");
                    arg = AEC_NORMAL_MODE;
                    break;
            }

	        g_mode = arg;
	        
	        aec_set_regs_default(g_mode);
		 if(NULL != pdata->set_mode)
	            pdata->set_mode(g_mode);
	        break;
	    case AEC_IOC_RESET:	        	            	            
            if(gAecClient)
            {
                aec_set_regs_default(g_mode);
            }               	        
	        break;
	    case AEC_IOC_SLEEP:	        
	        if(gAecClient)
            {
                if(arg == 1)
                {
                    if(!aec_count)
                    {
                        printk("WARNING: aec count had been zero\n");
                    }
                    else if(!--aec_count) 
                        aec_devPwrCtrl(0);
                }
                else
                {
                    if(!aec_count++)
                        aec_devPwrCtrl(1);
                }                                               
            }               	        
	        break;
		default:
			return -ENOTTY;
	}    
	return ret;
}

static struct file_operations aec_fops = {
	.owner =    THIS_MODULE,
	.ioctl =    aec_ioctl,
	.open =     aec_open,
	.release =  aec_release,	
};

static int setupCdev(struct aec_dev *dev)
{
	int err = 0;
	int devno = MKDEV(aec_major, aec_minor);
    
	cdev_init(&dev->cdev, &aec_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &aec_fops;
	err = cdev_add (&dev->cdev, devno, 1);
	printk(KERN_EMERG "setupCdev result %d \n", err);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_EMERG "setupCdev Error %d", err);
		
	return err;
}

#endif

static int __init aec_init(void)
{
	int result = -1;
	dev_t dev = 0;
	
	printk(KERN_EMERG "AEC: aec_init start ...\n");
	
	result = i2c_add_driver(&aec_i2c_driver);
	if (result) {
		printk(KERN_ERR "AEC: can't add i2c driver");							
	}
	
	/*
     * Get a range of minor numbers to work with, asking for a dynamic
     * major unless directed otherwise at load time.
     */    
     #if 0
    
    // prepare device id
	if (aec_major) {
		dev = MKDEV(aec_major, aec_minor);
		result = register_chrdev_region(dev, aec_nr_dev, AEC_DEV_FILE);
	} else {
		result = alloc_chrdev_region(&dev, aec_minor, aec_nr_dev,
				AEC_DEV_FILE);
		aec_major = MAJOR(dev);
	}
	if (result < 0) {
		printk(KERN_WARNING "scull: can't get major %d\n", aec_major);
		return result;
	}
	printk(KERN_EMERG "[AEC] :major %d minor %d\n", aec_major, aec_minor);

    aec_device = kmalloc(sizeof(struct aec_dev), GFP_KERNEL);
	if (!aec_device) {
		result = -ENOMEM;
		goto fail_malloc;
	}
	memset(aec_device, 0, sizeof(struct aec_dev));
	
	// setup cdev
	result = setupCdev(aec_device);
	if (result < 0) {
		result = -ENOMEM;
		goto fail_setup;
	}	

	printk(KERN_EMERG "[AEC] aec_init complete >> \n");
	
	return (0);

fail_setup:
	kfree(aec_device);
	
fail_malloc:
	unregister_chrdev_region(dev, aec_nr_dev);
	#endif
	return result;		
}

static void __exit aec_exit(void)
{	
#if 0
	dev_t devno = MKDEV(aec_major, aec_minor);
	cdev_del(&aec_device->cdev);
	kfree(aec_device);

	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, aec_nr_dev);
#endif		
	i2c_del_driver(&aec_i2c_driver);
}

module_init(aec_init);
module_exit(aec_exit);

MODULE_AUTHOR("K.H. Fan <kh.fan@foxconn.com>");
MODULE_DESCRIPTION("Foxconn manafacture AEC");
MODULE_LICENSE("GPL");

