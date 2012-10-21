

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/mfd/88pm860x.h>
#include <linux/regulator/machine.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/onenand.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c/pca9575.h>
#include <linux/i2c/pca953x.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa910.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/mmc.h>
#include <mach/camera.h>
#include <mach/wlan.h> 

#include <linux/spi/spi.h>
#include <linux/spi/cmmb.h>

#include <plat/generic.h>
#include <plat/mfp.h>
#include <plat/i2c.h>
#include <plat/vbus.h>
#include <plat/pxa_u2o.h>
#include <linux/usb/android_composite.h>
#include <mach/pxa910-aec-fm2010.h>  
#include <mach/pxa910-tpa2018.h>	
#if defined(CONFIG_PXA_U812) || defined (CONFIG_PXA_U830)
#include <linux/atmel_xmt140_ts.h> 
#endif
#include <mach/gsensor.h>

#if (defined CONFIG_PXA_U810 || defined CONFIG_PXA_U802)
#include <linux/akm8973.h>
#endif
#ifdef CONFIG_PXA_U880
#include <linux/akm8962.h>
#include <linux/cyttsp.h> 
#endif

#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif

#include "common.h"
#include "onboard.h"
#include <mach/alsps.h>
#ifdef CONFIG_PM
#include <mach/pxa910_pm.h>
#endif

unsigned int main_camera_id = 0;
EXPORT_SYMBOL_GPL(main_camera_id);
 

#define MMP_VENDOR_ID				0x19D2
#define MMP_ALL_PRODUCT_ID			0x4E20
#define MMP_MODEM_DIAG_UMS_PRODUCT_ID		0x4E21

#define MMP_MODEM_DIAG_UMS_ADB_PRODUCT_ID	0x1384
#define MMP_RNDIS_MODEM_DIAG_PRODUCT_ID		0x4E23
#define MMP_RNDIS_MODEM_DIAG_ADB_PRODUCT_ID	0x4E24

#define MMP_RNDIS_PRODUCT_ID			0x1385

#define MMP_UMS_PRODUCT_ID			0x0250

#define MMP_MODEM_DIAG_PRODUCT_ID		0x1383
#define MMP_UMS_ADB_PRODUCT_ID			0x4E28
#define MMP_RNDIS_ADB_PRODUCT_ID		0x4E29

#define MMP_MODEM_UMS_ADB_PRODUCT_ID		0x1382
#define MMP_MODEM_UMS_PRODUCT_ID		0x4E2B
#define MMP_DIAG_PRODUCT_ID			0x4E2C


#ifdef CONFIG_PXA_U802
static struct delayed_work u802_register_g_sensor_work;
#endif


void set_ldo13(int on);
void set_ldo8(int enable);
static unsigned long emmc_pin_config[] __initdata = {
	MFP_CFG(DF_IO0, AF1),
	MFP_CFG(DF_IO1, AF1),
	MFP_CFG(DF_IO2, AF1),
	MFP_CFG(DF_IO3, AF1),
	MFP_CFG(DF_IO4, AF1),
	MFP_CFG(DF_IO5, AF1),
	MFP_CFG(DF_IO6, AF1),
	MFP_CFG(DF_IO7, AF1),
	MFP_CFG(DF_CLE_SM_OEn, AF1),
	MFP_CFG(SM_SCLK, AF1),
};

static unsigned long ttc_dkb_pin_config[] __initdata = {
	
	GPIO43_UART1_RXD,
	GPIO44_UART1_TXD,


	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,

	
	GPIO29_UART3_CTS,
	GPIO30_UART3_RTS,
	GPIO31_UART3_TXD,
	GPIO32_UART3_RXD,

	
	DF_IO0_ND_IO0,
	DF_IO1_ND_IO1,
	DF_IO2_ND_IO2,
	DF_IO3_ND_IO3,
	DF_IO4_ND_IO4,
	DF_IO5_ND_IO5,
	DF_IO6_ND_IO6,
	DF_IO7_ND_IO7,
	DF_IO8_ND_IO8,
	DF_IO9_ND_IO9,
	DF_IO10_ND_IO10,
	DF_IO11_ND_IO11,
	DF_IO12_ND_IO12,
	DF_IO13_ND_IO13,

     

	DF_nCS0_SM_nCS2_nCS0,
	DF_ALE_SM_WEn_ND_ALE,
	DF_CLE_SM_OEn_ND_CLE,
	DF_WEn_DF_WEn,
	DF_REn_DF_REn,
	DF_RDY0_DF_RDY0,


	GPIO53_CI2C_SCL,
	GPIO54_CI2C_SDA,


 	GPIO24_SSP1_SDATA_IN,
 	GPIO21_SSP1_BITCLK,
 	GPIO22_SSP1_SYNC,
 	GPIO23_SSP1_DATA_OUT,


#if (defined CONFIG_PXA_U802 || defined CONFIG_PXA_U880)
	GPIO00_KP_MKIN0,
	GPIO01_KP_MKOUT0,
	GPIO02_KP_MKIN1,
	GPIO03_KP_MKOUT1,
	GPIO04_KP_MKIN2,

#elif (defined CONFIG_PXA_U810 || defined CONFIG_PXA_U830 || defined CONFIG_PXA_U812)
	GPIO00_KP_MKIN0,
	GPIO01_KP_MKOUT0,
	GPIO02_KP_MKIN1,
	GPIO03_KP_MKOUT1,

#endif


	GPIO67_CCIC_IN7,
	GPIO68_CCIC_IN6,
	GPIO69_CCIC_IN5,
	GPIO70_CCIC_IN4,
	GPIO71_CCIC_IN3,
	GPIO72_CCIC_IN2,
	GPIO73_CCIC_IN1,
	GPIO74_CCIC_IN0,
	GPIO75_CAM_HSYNC,
	GPIO76_CAM_VSYNC,
	GPIO77_CAM_MCLK,
	GPIO78_CAM_PCLK,
	
	GPIO16_CAM_PWR_MAIN,
#if defined(CONFIG_PXA_U810) || defined(CONFIG_PXA_U802)
	GPIO50_CAM_RESET_SUB,	
	#endif
	#if defined CONFIG_PXA_U880
		GPIO5_GPIO5,
	#endif
#if defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830)
	GPIO50_CAM_RESET_SUB,
	GPIO5_GPIO5,
#endif

	GPIO17_CAM_RESET_MAIN,
#if defined (CONFIG_PXA_U830)
	GPIO51_CAM_CIF_FLASH,
	GPIO109_CAM_FLASH_EN,
	GPIO29_CAM_AAM_PWREN,
#endif


	GPIO15_CMMB_PWR,
      GPIO14_CMMB_IRQ,
#if defined(CONFIG_PXA_U830)
	GPIO4_CMMB_PD,
#endif

	#if (defined( CONFIG_PXA_U810 ) || defined(CONFIG_PXA_U802 ))
	GPIO51_CHG_STATE,  
	#endif

	#if !(defined( CONFIG_PXA_U830 ) || defined(CONFIG_PXA_U812 ))
	GPIO52_GPS_WAKEUP,
      GPIO49_CHGIN_DET,
	GPIO109_CHG_WAKEUP,
	#endif

      GPIO33_SPI_CLK,	
      GPIO34_SPI_CS,		
      GPIO35_SPI_DOUT,
      GPIO36_SPI_DIN,
      GPIO25_PCM_CLK,		
      GPIO26_PCM_SYN,	
      GPIO27_PCM_TXD,		
      GPIO28_PCM_RXD,	
      GPIO20_USBUART_SW,
      GPIO18_I2C_SCL,
	GPIO19_I2C_SDA,
#ifdef CONFIG_PXA_U810
	GPIO04_ECHO_PER,
#endif
#if defined(CONFIG_PXA_U880) || defined (CONFIG_PXA_U830) || defined (CONFIG_PXA_U812) 
        GPIO07_WLAN_PD,
#else
	GPIO07_EGPIO_RST,
#endif

      GPIO08_AXIS_INT,
      GPIO09_ECHO_RST,
 #if defined(CONFIG_PXA_U880) || defined (CONFIG_PXA_U810) || defined (CONFIG_PXA_U802)
      GPIO05_BtCodec_SW,
      GPIO06_AUDIO_PA_EN,
#endif


#if defined(CONFIG_MMC_PXA_SDH)

	MMC1_DAT7_MMC1_DAT7,
	MMC1_DAT6_MMC1_DAT6,
	MMC1_DAT5_MMC1_DAT5,
	MMC1_DAT4_MMC1_DAT4,
	MMC1_DAT3_MMC1_DAT3,
	MMC1_DAT2_MMC1_DAT2,
	MMC1_DAT1_MMC1_DAT1,
	MMC1_DAT0_MMC1_DAT0,
	MMC1_CMD_MMC1_CMD,
	MMC1_CLK_MMC1_CLK,
	MMC1_CD_MMC1_CD,
	MMC1_WP_MMC1_WP,

	MMC2_DAT3_GPIO_37,
	MMC2_DAT2_GPIO_38,
	MMC2_DAT1_GPIO_39,
	MMC2_DAT0_GPIO_40,
	MMC2_CMD_GPIO_41,
	MMC2_CLK_GPIO_42,


      GPIO13_WB_WAKEUP, 
      GPIO12_WB_HPWR,
      #if ! (defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830))
	GPIO79_BT_CLK,
	#endif
	GPIO80_HEADSET_DET,	

	GPIO45_BT_WAKEUP,
	GPIO46_FM_WAKEUP,
	
#endif
    
	GPIO10_G_INT1,
#if defined(CONFIG_PXA_U880)
        GPIO11_WLAN_RESET,
#elif defined(CONFIG_PXA_U830)
        GPIO79_WLAN_RESET,
#elif defined(CONFIG_PXA_U812)
        GPIO79_WLAN_RESET,
#else
	GPIO11_G_INT2, 
#endif

#if defined (CONFIG_PXA_U830)
	GPIO80_G_INT2,
	GPIO11_ALS_INT,


	GPIO9_GYRO_INT,
	GPIO20_GYRO_READY,


	GPIO30_GPS_ON_OFF,
	GPIO31_GPS_RESET,
	GPIO32_GPS_PWREN,
#endif

#ifdef CONFIG_PXA_U880 
        GPIO124_GPIO124,
#endif
};

static unsigned long lcd_tpo_pin_config[] __initdata = {
#ifdef CONFIG_PXA_U802
	GPIO81_LCD_RDB,
	GPIO82_LCD_A0,
	GPIO83_LCD_WRB,
	
	GPIO85_LCD_DD0,
	GPIO86_LCD_DD1,
	GPIO87_LCD_DD2,
	GPIO88_LCD_DD3,
	GPIO89_LCD_DD4,
	GPIO90_LCD_DD5,
	GPIO91_LCD_DD6,
	GPIO92_LCD_DD7,
	GPIO93_LCD_DD8,
	GPIO94_LCD_DD9,
	GPIO95_LCD_DD10,
	GPIO96_LCD_DD11,
	GPIO97_LCD_DD12,
	GPIO98_LCD_DD13,
	GPIO100_LCD_DD14,
	GPIO101_LCD_DD15,
	GPIO102_LCD_DD16,
	GPIO103_LCD_DD17,
	
	GPIO104_LCD_CS,  
	GPIO106_LCD_RST,  
#elif (defined( CONFIG_PXA_U810 ) || defined(CONFIG_PXA_U880 )|| defined(CONFIG_PXA_U812 )|| defined(CONFIG_PXA_U830 ))
	GPIO81_LCD_FCLK,
	GPIO82_LCD_LCLK,
	GPIO83_LCD_PCLK,
	GPIO84_LCD_DENA,
	GPIO85_LCD_DD0,
	GPIO86_LCD_DD1,
	GPIO87_LCD_DD2,
	GPIO88_LCD_DD3,
	GPIO89_LCD_DD4,
	GPIO90_LCD_DD5,
	GPIO91_LCD_DD6,
	GPIO92_LCD_DD7,
	GPIO93_LCD_DD8,
	GPIO94_LCD_DD9,
	GPIO95_LCD_DD10,
	GPIO96_LCD_DD11,
	GPIO97_LCD_DD12,
	GPIO98_LCD_DD13,
	GPIO100_LCD_DD14,
	GPIO101_LCD_DD15,
	GPIO102_LCD_DD16,
	GPIO103_LCD_DD17,
#if 0
	GPIO104_LCD_SPIDOUT,
	GPIO105_LCD_SPIDIN,
	GPIO106_GPIO106,	
	GPIO107_LCD_CS1,
	GPIO108_LCD_DCLK,
#else
	GPIO104_GPIO104,  

	GPIO106_GPIO106,  
	GPIO107_GPIO107,  
	GPIO108_GPIO108,  
#endif
#endif
};

static unsigned long tds_pin_config[] __initdata = {
	GPIO55_TDS_LNACTRL,
	GPIO57_TDS_TRXSW,
	GPIO58_TDS_RXREV,
	GPIO59_TDS_TXREV,
	GPIO60_GPIO60 | MFP_PULL_HIGH,
};




static int is_td_dkb = 0;
static int __init td_dkb_setup(char *__unused)
{
	        is_td_dkb = 1;
		        return 1;
}
__setup("td_dkb", td_dkb_setup);

static int emmc_boot = 0;
static int __init emmc_setup(char *__unused)
{
#if defined(CONFIG_MMC_PXA_SDH)
	emmc_boot = 1;
#endif
	return 1;
}
__setup("emmc_boot", emmc_setup);

static int ttc_dkb_board_init(void)
{
	return 0;
}


static int wvga_lcd = 0;
static int __init wvga_lcd_setup(char *__unused)
{
        wvga_lcd = 1;
        return 1;
}
__setup("wvga_lcd", wvga_lcd_setup);

static int is_wvga_lcd(void)
{
        return wvga_lcd;
}

static int ttc_dkb_pm860x_fixup(struct pm860x_chip * chip,
			struct pm860x_platform_data *pdata)
{
	int data;

	data = pm860x_page_reg_read(chip->client, 0xD7);
	data &= 0x3;
	if(data ==0x0 || data == 0x3)
	{
		data = pm860x_page_reg_read(chip->client, 0xE1);
		data &= 0x3F;
		if(data < 0x3F)
			data+=1;
		pm860x_page_reg_write(chip->client, 0xE1, data);
		data = pm860x_page_reg_read(chip->client, 0xE1);
		dev_dbg(chip->dev, "detect 0xD7 broken counter: %d", data);
	}


	pm860x_reg_write(chip->client, PM8607_VIBRA_SET, 0x0c); 
	

	pm860x_reg_write(chip->client, PM8607_INT_MASK_1, 0x00); 
	pm860x_reg_write(chip->client, PM8607_INT_MASK_2, 0x00); 
	pm860x_reg_write(chip->client, PM8607_INT_MASK_3, 0x00); 

	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0x3f); 
	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0xff); 
	pm860x_reg_write(chip->client, PM8607_INT_STATUS1, 0xff); 


	pm860x_reg_write(chip->client, PM8607_MISC2, 
		pm860x_reg_read(chip->client, PM8607_MISC2)|0x80); 
	
	pm860x_reg_write(chip->client, PM8607_SUPPLIES_EN11, 
		pm860x_reg_read(chip->client, PM8607_SUPPLIES_EN11)|0x80); 


	pm860x_reg_write(chip->client, PM8607_GPADC_MISC1, 0x0b); 
	
	pm860x_reg_write(chip->client, PM8607_MEAS_EN1, 
	pm860x_reg_read(chip->client, PM8607_MEAS_EN1)|0x40); 
		

	pm860x_reg_write(chip->client, PM8607_LDO6, 0x00);
	pm860x_reg_write(chip->client, PM8607_LDO8, 0x00);
	
	pm860x_reg_write(chip->client, PM8607_SUPPLIES_EN12, 
	pm860x_reg_read(chip->client, PM8607_SUPPLIES_EN12) & 0x3A); 
	pm860x_reg_write(chip->client, PM8607_SLEEP_MODE1, 0xaa); 
	pm860x_reg_write(chip->client, PM8607_SLEEP_MODE2, 0xba); 

   #ifdef CONFIG_PXA_U880
	 if((pm860x_get_boardID() == 2) || (pm860x_get_boardID() == 4))
    {
       pm860x_reg_write(chip->client, PM8607_SLEEP_MODE3, 0xb0);
    }
	 else
       pm860x_reg_write(chip->client, PM8607_SLEEP_MODE3, 0x80);
   #else
       pm860x_reg_write(chip->client, PM8607_SLEEP_MODE3, 0x80);
   #endif
	if(is_td_dkb)
		
		pm860x_reg_write(chip->client, PM8607_SLEEP_MODE4, 0x00);
	else
		pm860x_reg_write(chip->client, PM8607_SLEEP_MODE4, 0x2a); 
	

	pm860x_reg_write(chip->client, PM8607_SLEEP_BUCK1, 0x24); 
	pm860x_reg_write(chip->client, PM8607_SLEEP_BUCK2, 0x24); 
        
        pm860x_reg_write(chip->client, PM8607_RTC_MISC2, 0x02); 



	pm860x_reg_write(chip->client, PM8607_VIBRA_SET, 0x0d); 

      
        pm860x_reg_write(chip->client, PM8607_LP_CONFIG1, 0x40); 
       
        pm860x_reg_write(chip->client, PM8607_LP_CONFIG3, 0x80);
        pm860x_reg_write(chip->client, PM8607_B0_MISC1, 0x80);
        pm860x_reg_write(chip->client, PM8607_MEAS_OFF_TIME1, 0x2);
      
        pm860x_reg_write(chip->client, PM8607_BUCK_CONTROLS, 0x2b); 
        pm860x_reg_write(chip->client, PM8607_LP_CONFIG2, 0x98);
	  pm860x_reg_write(chip->client, PM8607_SUPPLIES_EN12, 
		pm860x_reg_read(chip->client, PM8607_SUPPLIES_EN12) & 0xfb);
        return 0;
}

static struct mtd_partition ttc_dkb_partitions[] = {
	{
		.name		= "init",
		.offset		= 0xc0000,
		.size		= 0x40000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "NVM",
		.offset		= 0x100000,
		.size		= 0x020000,
	}, {
		.name		= "Arbel and Greyback Image",
		.offset		= 0x120000,
		.size		= 0x800000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "Kernel",
		.offset		= 0x920000,
		.size		= 0x300000,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "system",
		.offset		= 0x0c20000,
		.size		= 0x7000000,
	}, {
		.name		= "userdata",
		.offset		= 0x7c20000,
		.size		= 0x7000000,
	}, {
		.name		= "filesystem",
		.offset		= 0xec20000,
		.size		= 0xEE0000,
	}, {
		.name		= "resevred_for_bbm",
		.offset		= 0xfb00000,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= MTD_WRITEABLE,
	}
};

static struct pxa3xx_nand_platform_data ttc_dkb_nand_info = {
	.pxa3xx_nand_mode = POLLING_S | DMA_SUPPORT | ARBITER_ENABLE | NAKEDCMD_S,
	.parts[0] = ttc_dkb_partitions,
	.nr_parts[0] = ARRAY_SIZE(ttc_dkb_partitions),
};

static void __init ttc_dkb_init_flash(void)
{
	pxa910_add_nand(&ttc_dkb_nand_info);
}

static struct onenand_platform_data ttc_dkb_onenand_info = {
	.parts		= ttc_dkb_partitions,
	.nr_parts	= ARRAY_SIZE(ttc_dkb_partitions),
};

static struct resource ttc_dkb_resource_onenand[] = {
	[0] = {
		.start	= SMC_CS0_PHYS_BASE,
		.end	= SMC_CS0_PHYS_BASE + SZ_1M,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ttc_dkb_device_onenand = {
	.name		= "onenand-flash",
	.id		= -1,
	.resource	= ttc_dkb_resource_onenand,
	.num_resources	= ARRAY_SIZE(ttc_dkb_resource_onenand),
	.dev		= {
		.platform_data	= &ttc_dkb_onenand_info,
	},
};

static struct platform_device *ttc_dkb_devices[] = {
	&ttc_dkb_device_onenand,
};


#if defined(CONFIG_I2C_PXA) || defined(CONFIG_I2C_PXA_MODULE)
static struct regulator_consumer_supply regulator_supply[] = {
	[PM8607_ID_BUCK1]	= REGULATOR_SUPPLY("v_buck1", NULL),
	[PM8607_ID_BUCK3]	= REGULATOR_SUPPLY("v_buck3", NULL),
	[PM8607_ID_LDO1]	= REGULATOR_SUPPLY("v_ldo1", NULL),
	[PM8607_ID_LDO2]	= REGULATOR_SUPPLY("v_ldo2", NULL),
	[PM8607_ID_LDO3]	= REGULATOR_SUPPLY("v_ldo3", NULL),
	[PM8607_ID_LDO4]	= REGULATOR_SUPPLY("v_ldo4", NULL),
	[PM8607_ID_LDO5]	= REGULATOR_SUPPLY("v_ldo5", NULL),
	[PM8607_ID_LDO6]	= REGULATOR_SUPPLY("v_ldo6", NULL),
	[PM8607_ID_LDO7]	= REGULATOR_SUPPLY("v_ldo7", NULL),
	[PM8607_ID_LDO8]	= REGULATOR_SUPPLY("v_ldo8", NULL),
	[PM8607_ID_LDO9]	= REGULATOR_SUPPLY("v_ldo9", NULL),
	[PM8607_ID_LDO10]	= REGULATOR_SUPPLY("v_ldo10", NULL),
	[PM8607_ID_LDO12]	= REGULATOR_SUPPLY("v_ldo12", NULL),
	[PM8607_ID_LDO14]	= REGULATOR_SUPPLY("v_ldo14", NULL),
};

#define REG_INIT(_name, _min, _max, _always, _boot)			\
{									\
	.constraints = {						\
		.name		= __stringify(_name),			\
		.min_uV		= _min,					\
		.max_uV		= _max,					\
		.always_on	= _always,				\
		.boot_on	= _boot,				\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,		\
	},								\
	.num_consumer_supplies	= 1,					\
	.consumer_supplies	= &regulator_supply[PM8607_ID_##_name],	\
}

static struct regulator_init_data regulator_data[] = {
	[PM8607_ID_BUCK1] = REG_INIT(BUCK1, 0, 1500000, 1, 1),
	[PM8607_ID_BUCK3] = REG_INIT(BUCK3, 0, 3000000, 1, 1),
	[PM8607_ID_LDO1] = REG_INIT(LDO1, 1200000, 2800000, 1, 1),
	[PM8607_ID_LDO2] = REG_INIT(LDO2, 1800000, 3300000, 1, 1),
	[PM8607_ID_LDO3] = REG_INIT(LDO3, 1800000, 3300000, 1, 1),
	[PM8607_ID_LDO4] = REG_INIT(LDO4, 1800000, 3300000, 0, 0),
	[PM8607_ID_LDO5] = REG_INIT(LDO5, 2900000, 3300000, 1, 1),
	[PM8607_ID_LDO6] = REG_INIT(LDO6, 1800000, 3300000, 1, 1),
	[PM8607_ID_LDO7] = REG_INIT(LDO7, 1800000, 2900000, 1, 1),
	[PM8607_ID_LDO8] = REG_INIT(LDO8, 1800000, 2900000, 1, 1),
        [PM8607_ID_LDO9] = REG_INIT(LDO9, 1800000, 3300000, 1, 1),
        [PM8607_ID_LDO10] = REG_INIT(LDO10, 1200000, 3300000, 1, 1),
        [PM8607_ID_LDO12] = REG_INIT(LDO12, 1200000, 3300000, 0, 1),
        [PM8607_ID_LDO14] = REG_INIT(LDO14, 1800000, 3300000, 0, 1),
};

static struct pm860x_touch_pdata ttc_dkb_touch = {
	.gpadc_prebias	= 1,
	.slot_cycle	= 1,
	.tsi_prebias	= 4,
	.pen_prebias	= 8,
	.pen_prechg	= 2,
	.res_x		= 300,
};

static struct pm860x_backlight_pdata ttc_dkb_backlight[] = {
	{
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(12),
		.flags	= PM8606_BACKLIGHT1,
	},
	{},
};

static struct pm860x_led_pdata ttc_dkb_led[] = {
	{
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(8),
		.flags	= PM8606_LED1_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(8),
		.flags	= PM8606_LED1_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(8),
		.flags	= PM8606_LED1_BLUE,
	}, {
		.id	= PM8606_ID_LED,
		.flags	= PM8607_LED_VIBRATOR,
	},
	{},
};

static struct pm860x_power_pdata ttc_power_data = {
	.fast_charge		= 0,
};

struct pm860x_vbus_pdata ttc_dkb_vbus = {
	.supply		= PM860X_GPIO2_SUPPLY_VBUS,
	.idpin		= PM860X_IDPIN_USE_GPADC2,
	.reg_base	= PXA168_U2O_REGBASE,
	.reg_end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
};

static struct pm860x_platform_data ttc_dkb_pm8607_info = {
	.companion_addr			= 0x11,
	.irq_mode			= 0,
	.irq_base			= IRQ_BOARD_START,

	.i2c_port			= GI2C_PORT,
	.headset_flag	= 0x1,
	.regulator[PM8607_ID_BUCK1]	= &regulator_data[PM8607_ID_BUCK1],
	.regulator[PM8607_ID_BUCK3]	= &regulator_data[PM8607_ID_BUCK3],
	.regulator[PM8607_ID_LDO1]	= &regulator_data[PM8607_ID_LDO1],
	.regulator[PM8607_ID_LDO2]	= &regulator_data[PM8607_ID_LDO2],
	.regulator[PM8607_ID_LDO3]	= &regulator_data[PM8607_ID_LDO3],
	.regulator[PM8607_ID_LDO4]	= &regulator_data[PM8607_ID_LDO4],
	.regulator[PM8607_ID_LDO5]	= &regulator_data[PM8607_ID_LDO5],
	.regulator[PM8607_ID_LDO6]	= &regulator_data[PM8607_ID_LDO6],
	.regulator[PM8607_ID_LDO7]	= &regulator_data[PM8607_ID_LDO7],
	.regulator[PM8607_ID_LDO8]	= &regulator_data[PM8607_ID_LDO8],
	.regulator[PM8607_ID_LDO9]	= &regulator_data[PM8607_ID_LDO9],
	.regulator[PM8607_ID_LDO10]	= &regulator_data[PM8607_ID_LDO10],
	.regulator[PM8607_ID_LDO12]	= &regulator_data[PM8607_ID_LDO12],
	.regulator[PM8607_ID_LDO14]	= &regulator_data[PM8607_ID_LDO14],

	.touch			= &ttc_dkb_touch,
	.backlight		= ttc_dkb_backlight,
	.led			= ttc_dkb_led,
	.power			= &ttc_power_data,
	.fixup			= ttc_dkb_pm860x_fixup,
	.vbus			= &ttc_dkb_vbus,
};

#if 0
//added by huangxin20110210 for U802 lightsensor
#ifdef CONFIG_PXA_U802
static struct tsl2771_alsps_platform_data tsl2771_u802_data = {
        // .irq            = IRQ_GPIO(mfp_to_gpio(GPIO80_HEADSET_DET)),//maybe modified
           .irq            = IRQ_GPIO(80),
 };
#endif

//added by qiumingming in 20110322 for U880 lightsensor
#ifdef CONFIG_PXA_U880
static struct tsl2771_alsps_platform_data tsl2771_u880_data = 
{
};
#endif

//added by huangXin20110210 for U810 lightsensor
#if (defined CONFIG_PXA_U810 || defined CONFIG_PXA_U880)
static struct tsl2771_alsps_platform_data tsl2771_u810_data = {
        #ifdef CONFIG_PXA_U810 
        .irq            = IRQ_GPIO(mfp_to_gpio(GPIO11_G_INT2)),//maybe modified
        #elif defined CONFIG_PXA_U880
        .irq            = IRQ_GPIO(mfp_to_gpio(GPIO80_ALS2AP_INT)),
        #endif
         //  .irq            = IRQ_GPIO(11),
 };
#endif
#endif


static struct tsl2771_alsps_platform_data tsl2771_data = 
{
	.irq = NULL,
};

static struct platform_device tsl2771_platform_device = 
{
        .name = "alsps",
        .id   = -1,
    .dev  = 
	{
        .platform_data = &tsl2771_data,
        },
};

static void __init U810_alsps_init(void)
{
        platform_device_register(&tsl2771_platform_device);
}


static struct platform_device mvd_device  = {
	.name		= "zte_mvd",
	.id		= -1,
};

static void __init u810_init_mvd(void)
{
	platform_device_register(&mvd_device);
}


#if defined(CONFIG_GPIO_PCA9575)
static struct pca9575_platform_data pca9575_data[] = {
        [0] = {
                .gpio_base      = GPIO_EXT1(0),
        },
};
#endif

#ifdef CONFIG_PXA_U880
#define CY_USE_MT
static int cyttsp_i2c_init(int on)
{
	return 0;
}

static int cyttsp_i2c_wakeup(void)
{
	int cyttsp_irq_gpio, ret;
	cyttsp_irq_gpio = mfp_to_gpio(MFP_PIN_GPIO9);
	
	ret = gpio_request(cyttsp_irq_gpio, "CYTTSP I2C IRQ GPIO");
	if (ret) {
		printk(KERN_ERR "%s: Failed to request GPIO %d\n",
			      __func__, cyttsp_irq_gpio);
		return ret;
	}
	gpio_direction_output(cyttsp_irq_gpio, 0);
	msleep(5);
	gpio_direction_output(cyttsp_irq_gpio, 1);
	msleep(5);
	gpio_direction_output(cyttsp_irq_gpio, 0);
	msleep(5);
	gpio_direction_output(cyttsp_irq_gpio, 1);
	
	gpio_direction_input(cyttsp_irq_gpio);

	gpio_free(cyttsp_irq_gpio);

	return 0;
}

static struct cyttsp_platform_data cypress_i2c_ttsp_platform_data = {
	.wakeup = cyttsp_i2c_wakeup,
	.init = cyttsp_i2c_init,
#ifdef CY_USE_MT
	.mt_sync = input_mt_sync,
#endif
	.maxx = 480,
	.maxy = 800,
	.flags = 0,
	.gen = CY_GEN3,
	.use_st = 0,
	.use_mt = 1,
	.use_trk_id = 0,
	.use_hndshk = 1,
	.use_timer = 0,
	.use_sleep = 1,
	.use_gestures = 0,
	.use_load_file = 0,
	.use_force_fw_update = 0,
	.use_virtual_keys = 1,
	
	.gest_set = CY_GEST_GRP_NONE | CY_ACT_DIST,
	
	.scn_typ = 0xA5, 

	
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.name = CY_I2C_NAME,
	.irq_gpio = 9,
};

#endif

#if defined(CONFIG_PXA910_CAMERA)
static int cam_ldo12_1p2v_enable(int on)
{
	static struct regulator *r_vcam = NULL;
	static int f_enabled = 0;
	if (on && (!f_enabled)) {
		r_vcam = regulator_get(NULL, "v_ldo12");
		if (IS_ERR(r_vcam)) {
			r_vcam = NULL;
			return EIO;
		} else {
			regulator_set_voltage(r_vcam, 1200000, 1200000);
			regulator_enable(r_vcam);
			f_enabled = 1;
		}
	}

	if (f_enabled && (!on)) {
		if (r_vcam) {
			regulator_disable(r_vcam);
			regulator_put(r_vcam);
			f_enabled = 0;
			r_vcam = NULL;
		}
	}
	return 0;
}

#ifdef CONFIG_PXA_U802  
#define ZTE_HWVERSION1 1
#define ZTE_HWVERSION2  2
#endif

#if defined(CONFIG_PXA_U830)
static int cam_sensor_power_on(int on, int sensor)
{
	return 0;
}
#endif

#if defined(CONFIG_PXA_U812)
extern int ov7690_dvp_initialize(void);
extern int main_sensor_dvp_initialize(void);
extern void ccic_set_clock_parallel(void);
extern void ccic_disable_clock(void);

static int cam_sensor_power_on(int on, int sensor)
{
	unsigned int cam_pwr, cam_pwr_front;
	unsigned int cam_reset;
	unsigned int cam_avdd_en;
	static int dkb_init = 1;

    cam_pwr = mfp_to_gpio(MFP_PIN_GPIO16);
    cam_pwr_front = mfp_to_gpio(MFP_PIN_GPIO50);
    cam_reset = mfp_to_gpio(MFP_PIN_GPIO17);

	cam_avdd_en = mfp_to_gpio(MFP_PIN_GPIO5);
	
	if (gpio_request(cam_avdd_en, "cam_avdd_en")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_avdd_en);
		return -EIO;
	}
	if (gpio_request(cam_pwr, "CAM_PWR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_pwr);
		return -EIO;
	}
	if (gpio_request(cam_pwr_front, "CAM_PWR_FRONT")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_pwr_front);
		return -EIO;
	}
	if (gpio_request(cam_reset, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_reset);
		return -EIO;
	}

	if(dkb_init == 1)
	{
		gpio_direction_output(cam_avdd_en, 1);
		msleep(1);
		ccic_set_clock_parallel();
		msleep(1);
		gpio_direction_output(cam_reset, 0);
		msleep(1);
		gpio_direction_output(cam_pwr, 1);
		msleep(3);
		gpio_direction_output(cam_pwr_front, 0);
		msleep(1);
		ov7690_dvp_initialize();
		msleep(1);
		gpio_direction_output(cam_pwr_front, 1);
		msleep(100);
		gpio_direction_output(cam_pwr, 0);
		msleep(10);
		gpio_direction_output(cam_reset, 1);
		msleep(10);
		main_sensor_dvp_initialize();
		msleep(1);
		gpio_direction_output(cam_pwr, 1);
		dkb_init = 0;
	}

	if(on){
		if(sensor == 1)
		{

#if 0
			printk("s5k5ca poweron\n");
			gpio_direction_output(cam_pwr, 0);
			gpio_direction_output(cam_reset, 0);
			gpio_direction_output(cam_avdd_en, 1);
			msleep(1);
			ccic_set_clock_parallel();	
			msleep(10);
			gpio_direction_output(cam_reset, 1);
			msleep(10);
#else
			printk("s5k5ca poweron\n");
			ccic_set_clock_parallel();
			msleep(1);
			gpio_direction_output(cam_pwr_front, 0);
			msleep(10);	
			ov7690_dvp_initialize();
			msleep(1);
			gpio_direction_output(cam_pwr_front, 1);
			msleep(100);
			gpio_direction_output(cam_pwr, 0);
			msleep(10);
#endif
		}
		else
		{
			printk("ov7690 poweron\n");
			gpio_direction_output(cam_pwr_front, 0);
		}
		
	
		cam_ldo12_1p2v_enable(1);
		msleep(1);
	}else{
		gpio_direction_output(cam_pwr, 1);
		gpio_direction_output(cam_pwr_front, 1);
		msleep(1);
		cam_ldo12_1p2v_enable(0);
	}

	gpio_free(cam_pwr);
	gpio_free(cam_pwr_front);
	gpio_free(cam_avdd_en);
	gpio_free(cam_reset);

	return 0;
}
#endif


static int sensor_power_onoff(int on, int sensor)
{
	unsigned int cam_pwr;
	unsigned int cam_reset;

        unsigned int cam_dvdd_en;
        unsigned int cam_avdd_en;


	#if  defined(CONFIG_PXA_U810) 
    cam_pwr = sensor ? mfp_to_gpio(MFP_PIN_GPIO16):0; 
    cam_reset = sensor ? mfp_to_gpio(MFP_PIN_GPIO17):mfp_to_gpio(MFP_PIN_GPIO50);

	cam_dvdd_en = GPIO_EXT1(6);
	cam_avdd_en = GPIO_EXT1(7);
	
	if (gpio_request(cam_dvdd_en, "cam_dvdd_en")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_dvdd_en);
		return -EIO;
	}

	if (gpio_request(cam_avdd_en, "cam_avdd_en")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_avdd_en);
		return -EIO;
	}

	if(cam_pwr)
	if (gpio_request(cam_pwr, "CAM_PWR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_pwr);
		return -EIO;
	}


	if (gpio_request(cam_reset, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_reset);
		return -EIO;
	}


	if(on){
		
		if(cam_pwr)
			gpio_direction_output(cam_pwr, 0);
		
		gpio_direction_output(cam_avdd_en, 1);
    gpio_direction_output(cam_dvdd_en, 1);
		
		msleep(1);    
		gpio_direction_output(cam_reset, 0);
		msleep(1);
		gpio_direction_output(cam_reset, 1);
		msleep(1);
	
		cam_ldo12_1p2v_enable(1);
		
		msleep(1);
	}else{
		if(cam_pwr)
			gpio_direction_output(cam_pwr, 1);
		
		gpio_direction_output(cam_avdd_en, 0);
        gpio_direction_output(cam_dvdd_en, 0);
		gpio_direction_output(cam_reset, 0);
		
		cam_ldo12_1p2v_enable(0);
	}

	if(cam_pwr)
		gpio_free(cam_pwr);
	gpio_free(cam_dvdd_en);
	gpio_free(cam_avdd_en);
	gpio_free(cam_reset);

#endif

#if defined(CONFIG_PXA_U802)
	 cam_pwr = sensor ? mfp_to_gpio(MFP_PIN_GPIO16):0; 
    	cam_reset = sensor ? mfp_to_gpio(MFP_PIN_GPIO17):mfp_to_gpio(MFP_PIN_GPIO50);

	
	if(ZTE_HWVERSION1== pm860x_get_boardID())
	{
		cam_dvdd_en = GPIO_EXT1(6);
		cam_avdd_en = GPIO_EXT1(7);
	
	}
	else
	{
		cam_avdd_en = mfp_to_gpio(MFP_PIN_GPIO19);
	}
	
	if(ZTE_HWVERSION1== pm860x_get_boardID())
	{
		if (gpio_request(cam_dvdd_en, "cam_dvdd_en")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_dvdd_en);
		return -EIO;
		}
	}
	if (gpio_request(cam_avdd_en, "cam_avdd_en")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_avdd_en);
		return -EIO;
	}

	if(cam_pwr)
	if (gpio_request(cam_pwr, "CAM_PWR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_pwr);
		return -EIO;
	}

	if (gpio_request(cam_reset, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_reset);
		return -EIO;
	}
  

	if(on){
	
		if(cam_pwr)
			gpio_direction_output(cam_pwr, 0);
		
		if(ZTE_HWVERSION1== pm860x_get_boardID())
		{
			gpio_direction_output(cam_dvdd_en, 1);
		}
	
		gpio_direction_output(cam_avdd_en, 1);
 		
		msleep(1);    
		gpio_direction_output(cam_reset, 0);
		msleep(1);
		gpio_direction_output(cam_reset, 1);
		msleep(1);
		
		cam_ldo12_1p2v_enable(1);
		
		msleep(1);
	}else{
		if(cam_pwr)
			gpio_direction_output(cam_pwr, 1);
		
		if(ZTE_HWVERSION1== pm860x_get_boardID())
		{
			gpio_direction_output(cam_dvdd_en, 0);
		}
		gpio_direction_output(cam_avdd_en, 0);
       		gpio_direction_output(cam_reset, 0);
		
		cam_ldo12_1p2v_enable(0);
	}

	if(cam_pwr)
		gpio_free(cam_pwr);
	
	if(ZTE_HWVERSION1== pm860x_get_boardID())
	{
		gpio_free(cam_dvdd_en);
	}
	gpio_free(cam_avdd_en);
	gpio_free(cam_reset);
	
	#endif
	
 	#if  defined(CONFIG_PXA_U880) 

		cam_pwr = mfp_to_gpio(MFP_PIN_GPIO16);
	cam_reset = mfp_to_gpio(MFP_PIN_GPIO17);
	
	cam_avdd_en =mfp_to_gpio(MFP_PIN_GPIO5);
	
	if (gpio_request(cam_avdd_en, "cam_avdd_en")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_avdd_en);
		return -EIO;
	}

	if(cam_pwr)
	if (gpio_request(cam_pwr, "CAM_PWR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_pwr);
		return -EIO;
	}

	if (gpio_request(cam_reset, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_reset);
		return -EIO;
	}

	if(on){

		
		gpio_direction_output(cam_avdd_en, 1);
		msleep(1);    
		gpio_direction_output(cam_reset, 0);
		msleep(1);
		gpio_direction_output(cam_reset, 1);
		msleep(1);
		if(cam_pwr)
			gpio_direction_output(cam_pwr, 0);
	
		cam_ldo12_1p2v_enable(1);
		
		msleep(1);
	}else{
		
		gpio_direction_output(cam_avdd_en, 0);
		gpio_direction_output(cam_reset, 0);
		if(cam_pwr)
			gpio_direction_output(cam_pwr, 1);
		cam_ldo12_1p2v_enable(0);
	}

	if(cam_pwr)
	gpio_free(cam_pwr);
	gpio_free(cam_avdd_en);
	gpio_free(cam_reset);
	
	#endif
	return 0;
}
#if  defined(CONFIG_PXA_U810) ||defined(CONFIG_PXA_U802)

static struct sensor_platform_data mt9v113_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

static struct sensor_platform_data mt9t111_sensor_data = {
        .id = SENSOR_HIGH,
        .power_on = sensor_power_onoff,
};
#endif

#if defined(CONFIG_PXA_U812)
static struct sensor_platform_data ov7690_sensor_data = {
		.id = SENSOR_LOW,
		.power_on = cam_sensor_power_on,
};
static struct sensor_platform_data s5k5ca_sensor_data = {
		.id = SENSOR_HIGH,
		.power_on = cam_sensor_power_on,
 };
#endif

#if defined(CONFIG_PXA_U830)
static struct sensor_platform_data ov7690_sensor_data = {
		.id = SENSOR_LOW,
		.power_on = cam_sensor_power_on,
};
static struct sensor_platform_data ov5640_sensor_data = {
		.id = SENSOR_HIGH,
		.power_on = cam_sensor_power_on,
};
#endif

#if  defined(CONFIG_PXA_U880) 
static struct sensor_platform_data ov5642_sensor_data = {  
        .id = SENSOR_HIGH,
        .power_on = sensor_power_onoff,
};
static struct sensor_platform_data ov5640_sensor_data = {  
        .id = SENSOR_HIGH,
        .power_on = sensor_power_onoff,
};
#endif



#endif

#if 0
/* tpa2018 init start */
int tpa2018_init(void)
{
	if (gpio_request(MFP_PIN_GPIO6, "AUDIO_PA_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO6);
		return -EIO;
	}
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 1);
	mdelay(20);
	gpio_free(MFP_PIN_GPIO6);
	return 0;
}
int tpa2018_sleep(void)
{
	if (gpio_request(MFP_PIN_GPIO6, "AUDIO_PA_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO6);
		return -EIO;
	}
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 0);
	mdelay(20);
	gpio_free(MFP_PIN_GPIO6);
	return 0;
}
int tpa2018_wakeup(void) 
{
	return 0;
}
int tpa2018_reset(void)
{	
	return 0;	
}
int tpa2018_set_mode(int mode)
{
	return 0;;
}
#endif

static struct aec_platform_data pxa_tpa2018_platform_data = {
	.init = tpa2018_init ,
	.sleep = tpa2018_sleep ,
	.wakeup = tpa2018_wakeup ,		
	.reset = tpa2018_reset,  
	.set_mode = tpa2018_set_mode ,
};




#ifdef CONFIG_PXA_U802
static mfp_cfg_t fm2010_mfp_cfg[] = {
	GPIO09_ECHO_RST  ,   
};
#elif defined CONFIG_PXA_U880
static mfp_cfg_t fm2010_mfp_cfg[] = {  
};
#elif defined CONFIG_PXA_U810
static mfp_cfg_t fm2010_mfp_cfg[] = {
	GPIO04_ECHO_PER ,       
	GPIO09_ECHO_RST  ,   
};
#elif defined CONFIG_PXA_U812
static mfp_cfg_t fm2010_mfp_cfg[] = {  
};
#elif defined CONFIG_PXA_U830
static mfp_cfg_t fm2010_mfp_cfg[] = {  
};
#endif


static int g_sensor_device_init(void)
{
	return 0;
}
static struct gsensor_platform_data g_sensor_platform_data = 
{

	.irq        = IRQ_GPIO(10),

	.init		= g_sensor_device_init, 

};
static struct platform_device g_sensor_device = 
{
	.name = "accel_vir_i2c",
	.id = -1,
	.dev = 
	{
		.platform_data = &g_sensor_platform_data,
	},
};


#if (defined CONFIG_PXA_U810 || defined CONFIG_PXA_U802)
static int ak8973_device_init(void)
{
	return 0;
}
static struct akm8973_platform_data compass_platform_data = 
{



	.gpio_RST		= GPIO_EXT1(13),

};
static struct platform_device akm8973_device = 
{
	.name = AKM8973_I2C_NAME,
	.id = -1,
	.dev = 
	{
		.platform_data = &compass_platform_data,
	},
};
#endif

#ifdef CONFIG_PXA_U880
static int ak8962_device_init(void)
{
	return 0;
}
static struct akm8962_platform_data compass_platform_data = 
{
};
static struct platform_device akm8962_device = 
{
	.name = AKM8962_I2C_NAME,
	.id = -1,
	.dev = 
	{
		.platform_data = &compass_platform_data,
	},
};
#endif



static struct aec_reg fm2010_param_normal[] = {
	{0x0,0x0},
};

static struct aec_reg fm2010_param_bypass[] = {

	{0x0,0x0},
};
static struct aec_reg fm2010_param_handheld[] = {
	{0x0,0x0},
};
int aec_fm2010_init(void)
{

	
	mfp_config(ARRAY_AND_SIZE(fm2010_mfp_cfg));

	mdelay(10);

	if (gpio_request(MFP_PIN_GPIO4, "ECHO_POWERDOWN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO4);
		return -EIO;
	}
	
	if (gpio_request(MFP_PIN_GPIO9, "ECHO_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO9);
		return -EIO;
	}

	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 1);
	mdelay(10);



	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO9), 1);
	mdelay(5);

	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO9), 0);
	mdelay(5);
	
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO9), 1);
	mdelay(10);	


	gpio_free(MFP_PIN_GPIO4);	
	gpio_free(MFP_PIN_GPIO9);
	return 0;
}
int aec_fm2010_sleep(void)
{
	if (gpio_request(MFP_PIN_GPIO4, "ECHO_POWERDOWN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO4);
		return -EIO;
	}
	
	gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 0);
	mdelay(10);
	
	
	gpio_free(MFP_PIN_GPIO4);	
	return 0;

}
int aec_fm2010_wakeup(void) 
{
    aec_fm2010_init( );
    return 0;
}
int aec_fm2010_reset(void)
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
	return 0;
}
int aec_fm2010_set_mode(int mode)
{
	return 0;
}

const aec_reg_t * aec_fm2010_parmGet(int mode)
{
      aec_reg_t* fm_para = NULL;
	switch(mode)
	{
		case AEC_NORMAL_MODE:
		    printk(KERN_INFO "[AEC] set normal mode\n");
		    fm_para =  fm2010_param_normal;
		break;
		case AEC_BYPASS_MODE:
		    printk(KERN_INFO "[AEC] set bypass mode\n");
		    fm_para =  fm2010_param_bypass;
		break;
		case AEC_NORMAL_HANDHELD_MODE:
		    printk(KERN_INFO "[AEC] set normal handheld mode\n");
		    fm_para = fm2010_param_handheld;
		break;
		default:
		    printk(KERN_INFO "[AEC] Invalid argument\n");
		    break;
	}
	return fm_para;
}


static struct aec_platform_data pxa_fm2010_platform_data = {
	.init = aec_fm2010_init ,
	.sleep = aec_fm2010_sleep ,
	.wakeup = aec_fm2010_wakeup ,		
	.reset = aec_fm2010_reset,  
	.set_mode = aec_fm2010_set_mode ,
	.parmGet  = aec_fm2010_parmGet,
};




static struct pca953x_platform_data max7312_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT0(0),
	},
};

#if defined(CONFIG_SENSORS_LIS3LV02D_I2C)
static int lis3lv02d_direction_idx = 1;
#endif


static void i2c_pxa_bus_reset(void)
{
	unsigned long i2c_mfps[] = {
		GPIO53_GPIO53,		
		GPIO54_GPIO54,		
	};
	unsigned long mfp_pin[ARRAY_SIZE(i2c_mfps)];
	int ccnt;

	if (gpio_request(MFP_PIN_GPIO53, "SCL")) {
		pr_err("Failed to request GPIO for SCL pin!\n");
		goto out;
	}
	if (gpio_request(MFP_PIN_GPIO54, "SDA")) {
		pr_err("Failed to request GPIO for SDA pin!\n");
		goto out_sda;
	}
	pr_info("\t<<<i2c bus reseting>>>\n");

	mfp_pin[0] = mfp_read(MFP_PIN_GPIO53);
	mfp_pin[1] = mfp_read(MFP_PIN_GPIO54);
	mfp_config(ARRAY_AND_SIZE(i2c_mfps));

	gpio_direction_input(MFP_PIN_GPIO54);
	for (ccnt = 20; ccnt; ccnt--) {
		gpio_direction_output(MFP_PIN_GPIO53, ccnt & 0x01);
		udelay(4);
	}
	gpio_direction_output(MFP_PIN_GPIO53, 0);
	udelay(4);
	gpio_direction_output(MFP_PIN_GPIO54, 0);
	udelay(4);

	gpio_direction_output(MFP_PIN_GPIO53, 1);
	udelay(4);
	gpio_direction_output(MFP_PIN_GPIO54, 1);
	udelay(4);

	mfp_write(MFP_PIN_GPIO53, mfp_pin[0]);
	mfp_write(MFP_PIN_GPIO54, mfp_pin[1]);
	gpio_free(MFP_PIN_GPIO54);
out_sda:
	gpio_free(MFP_PIN_GPIO53);
out:
	return;
}

#if defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830)
static struct atmel_xmt140_platform_data atmel_platform_data = {
	.x_line = 14,
	.y_line = 9,
	.x_size = 517,
	.y_size = 320,
	.blen = 16,
	.threshold = 80,
	.voltage = 2800000,
	.orient = 1,
};
#endif

static struct i2c_pxa_platform_data i2c_info __initdata = {
	.use_pio		= 0,
	.fast_mode		= 1,
	.hardware_lock = get_RIPC,
	.hardware_unlock =  release_RIPC,
	.i2c_bus_reset = i2c_pxa_bus_reset,
};

static struct i2c_board_info ttc_dkb_i2c_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &ttc_dkb_pm8607_info,
		.irq		= IRQ_PXA910_PMIC_INT,
	},
	

#if (defined CONFIG_PXA_U810 ||defined CONFIG_PXA_U802)
#if defined(CONFIG_GPIO_PCA9575)
	{
		.type           = "pca9575",
		.addr           = 0x20,
		.irq            = IRQ_GPIO(19),
		.platform_data  = &pca9575_data,
	},
#endif
#endif

#if 0 
	{
		.type           = "max7312",
		.addr           = 0x23,
		.irq            = IRQ_GPIO(80),
		.platform_data  = &max7312_data,
	},

#if defined(CONFIG_TOUCHSCREEN_TPO)
	{
		.type		= "tpo_touch",
		.addr		=  0x18,
		.irq		= gpio_to_irq(45),
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_ELAN)
	{
		.type		= "elan_touch",
		.addr		=  0x8,
		.irq		= gpio_to_irq(45),
	},
#endif
#endif




#ifdef CONFIG_PXA_U880
        {
                .type                   = "alsps",
                .addr                   =  0x39, 
                .platform_data  		= &tsl2771_data,
        },
        
        {
                .type                   = "isl29026",
                .addr                   = 0x45, 
        },
#endif

#ifdef CONFIG_PXA_U802
        {
                .type                   = "st_accel_i2c",
                .addr                   = 0x1C, 
                .platform_data  		= &g_sensor_platform_data,
        },

		{
                .type                   = "adi_accel_i2c",
                .addr                   = 0x53, 
                .platform_data  		= &g_sensor_platform_data,
        },
#endif


#if defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830)
	{
		.type		= "atmel_xmt140_ts",
		.addr		=  0x4a,
		.irq		= gpio_to_irq(49),
		.platform_data = &atmel_platform_data,
	},
	{
		.type		= "synaptics-rmi-ts",
		.addr		=  0x22,
		.irq		= gpio_to_irq(49),
	},
#endif
#if defined(CONFIG_PXA_U880)
	{
		.type		= "synaptics-rmi-ts",
		.addr		=  0x22,
		.irq		= gpio_to_irq(9),
	},
	{    
		.type		= CY_I2C_NAME,
		.addr		= 0xa,
		.irq		= gpio_to_irq(9),
		.platform_data = &cypress_i2c_ttsp_platform_data,
	},
#endif

#if defined(CONFIG_PXA910_CAMERA)
#if  defined(CONFIG_PXA_U810) ||defined(CONFIG_PXA_U802)

        {
                .type           = "mt9v113",
                .addr           = 0x7A>>1,
                .platform_data  = &mt9v113_sensor_data,
        },
        {
                .type		= "mt9t111",
                .addr           = 0x78>>1,
                .platform_data  = &mt9t111_sensor_data,
        },
#endif
#if defined(CONFIG_PXA_U812)
		{
			.type           = "ov7690",
			.addr           = 0x21,
			.platform_data  = &ov7690_sensor_data,
		},
		{
			.type			= "s5k5ca",
			.addr			= 0x2d,
			.platform_data = &s5k5ca_sensor_data,
		},
#endif
#if defined(CONFIG_PXA_U830)
		{
			.type           = "ov7690",
			.addr           = 0x21,
			.platform_data  = &ov7690_sensor_data,
		},
		{
			.type			= "ov5640",
			.addr           = 0x3C,
			.platform_data  = &ov5640_sensor_data,
		},
#endif
#if  defined(CONFIG_PXA_U880)    
        {
                .type           = "ov5642",
                .addr           = 0x78>>1,
               .platform_data  = &ov5642_sensor_data,
        },
        {
                .type           = "ov5640",
                .addr           = 0x7A>>1,
               .platform_data  = &ov5640_sensor_data,
        },
  #endif
#endif


#ifdef CONFIG_PXA_U810
        {
		.type	= "tpa2018",
		.addr		= 0x58,
		.platform_data	= &pxa_tpa2018_platform_data,
        },

	{
		.type	= "avp_fm2010",
		.addr		= 0x60,
		.platform_data	= &pxa_fm2010_platform_data,
	},
#endif




};
#endif

static struct platform_device sensor_input_device = {
	.name = "sensor_input",
	.id   = -1,
};

#ifdef KEYPAD_WAKEUP

static void dkb_keypad_wakeup_clear(void)
{
	u32 mfp = 0;
	u32 n = 0;

	
	for(n = 0; n <= 9; n ++) {
		mfp = mfp_read(n);
		mfp_write(n, (1 << 6) | mfp);
		mfp_write(n, mfp);
	}

	n = 12;
	mfp = mfp_read(n);
	mfp_write(n, (1 << 6) | mfp);
	mfp_write(n, mfp);
}
#endif

#ifdef CONFIG_PXA_U802
static unsigned int ttc_dkb_matrix_key_map[] = {
        KEY(0, 0, KEY_VOLUMEUP),
        KEY(0, 1, KEY_VOLUMEDOWN),
        KEY(1, 1, KEY_CAMERA),
        KEY(2, 0, KEY_END),
        KEY(2, 1, KEY_SEND),
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info = {
        .matrix_key_rows        = 3,//7, 
        .matrix_key_cols        = 2,//3
        .matrix_key_map         = ttc_dkb_matrix_key_map,
        .matrix_key_map_size    = ARRAY_SIZE(ttc_dkb_matrix_key_map),
        .debounce_interval      = 30,
#ifdef KEYPAD_WAKEUP
        .wakeup_clear		= dkb_keypad_wakeup_clear,
#endif
};

#elif defined CONFIG_PXA_U880
static unsigned int ttc_dkb_matrix_key_map[] = {
        KEY(0, 0, KEY_VOLUMEUP),
        KEY(0, 1, KEY_VOLUMEDOWN),
        KEY(1, 1, KEY_BACK),
        KEY(2, 0, KEY_HOME),
        KEY(2, 1, KEY_MENU),
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info = {
        .matrix_key_rows        = 3,//7,
        .matrix_key_cols        = 2,//3
        .matrix_key_map         = ttc_dkb_matrix_key_map,
        .matrix_key_map_size    = ARRAY_SIZE(ttc_dkb_matrix_key_map),
        .debounce_interval      = 30,
#ifdef KEYPAD_WAKEUP
        .wakeup_clear		= dkb_keypad_wakeup_clear,
#endif
};
#elif defined CONFIG_PXA_U810
static unsigned int ttc_dkb_matrix_key_map[] = {
        KEY(0, 0, KEY_VOLUMEUP),
        KEY(0, 1, KEY_VOLUMEDOWN),
        KEY(1, 1, KEY_CAMERA),
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info = {
        .matrix_key_rows        = 2,
        .matrix_key_cols        = 2,
        .matrix_key_map         = ttc_dkb_matrix_key_map,
        .matrix_key_map_size    = ARRAY_SIZE(ttc_dkb_matrix_key_map),
        .debounce_interval      = 30,
#ifdef KEYPAD_WAKEUP
        .wakeup_clear		= dkb_keypad_wakeup_clear,
#endif
};
#elif defined CONFIG_PXA_U812
static unsigned int ttc_dkb_matrix_key_map[] = {
        KEY(0, 0, KEY_VOLUMEUP),
        KEY(0, 1, KEY_VOLUMEDOWN),
        KEY(1, 0, KEY_SEARCH),
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info = {
        .matrix_key_rows        = 2,
        .matrix_key_cols        = 2,
        .matrix_key_map         = ttc_dkb_matrix_key_map,
        .matrix_key_map_size    = ARRAY_SIZE(ttc_dkb_matrix_key_map),
        .debounce_interval      = 30,
};
#elif defined CONFIG_PXA_U830
static unsigned int ttc_dkb_matrix_key_map[] = {
        KEY(0, 0, KEY_VOLUMEUP),
        KEY(0, 1, KEY_VOLUMEDOWN),
        KEY(1, 0, KEY_SEARCH),
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info = {
        .matrix_key_rows        = 2,
        .matrix_key_cols        = 2,
        .matrix_key_map         = ttc_dkb_matrix_key_map,
        .matrix_key_map_size    = ARRAY_SIZE(ttc_dkb_matrix_key_map),
        .debounce_interval      = 30,
};
#endif

#if defined(CONFIG_MMC_PXA_SDH)
static int sd_pwr_en = 0;

#define HOST_SLEEP_EN 1

#if !(HOST_SLEEP_EN)
void mmc2_gpio_switch(unsigned int on, int with_card) {}
#else
#include <linux/wakelock.h>
static struct wake_lock gpio_wakeup;

static irqreturn_t dat1_gpio_irq(int irq, void *data)
{
	unsigned int sec = 10;

	printk(KERN_INFO "%s: set wakelock, timout after %d seconds\n",
		__FUNCTION__, sec);

	wake_lock_timeout(&gpio_wakeup, HZ * sec);

	return IRQ_HANDLED;
}

static int gpio_wakeup_setup(u32 w_gpio)
{
	int ret;

	if (gpio_request(w_gpio, "SDIO dat1 GPIO Wakeup")) {
		printk(KERN_ERR "Failed to request GPIO %d "
				"for SDIO DATA1 GPIO Wakeup\n", w_gpio);
		return -EIO;
	}
	gpio_direction_input(w_gpio);

	ret = request_irq(gpio_to_irq(w_gpio), dat1_gpio_irq,
		IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TIMER,
		"SDIO data1 irq", NULL);
	if (ret) {
		printk(KERN_ERR "Request SDIO data1 GPIO irq failed %d\n", ret);
		return -EIO;
	}
	
	ret = gpio_get_value(w_gpio);
	gpio_free(w_gpio);

	if(!ret)
		return -EIO;

	return 0;
}

static int mmc2_gpio_switch(unsigned int on, int with_card)
{
	int ret;

	mfp_cfg_t mfp_cfg_dat1 = MMC2_DAT1_GPIO_39 | MFP_LPM_EDGE_NONE;
	mfp_cfg_t mfp_cfg_gpio = MMC2_DAT1_IRQ_GPIO_39 | MFP_LPM_EDGE_BOTH;

        printk("in %s: on is %d, with card 0x%08x\n", __FUNCTION__, on, with_card);

	if (!with_card)
		return 0;

	if (on) {
                mfp_config(&mfp_cfg_gpio, 1);
		ret = gpio_wakeup_setup(mfp_to_gpio(mfp_cfg_gpio));
		if(ret){
			return ret;
		}
        } else {
                free_irq(gpio_to_irq(mfp_to_gpio(mfp_cfg_gpio)), NULL);
                mfp_config(&mfp_cfg_dat1, 1);
        }
	return 0;
}
#endif 

static int mmc1_lp_switch(unsigned int on, int with_card)
{
	int ret = 0;
	static struct regulator *regulator_sd = NULL;

#if 0


	if (!regulator_sd) {
		/* LDO14, power supply to MMC1 pins */
		regulator_sd = regulator_get(NULL, "v_ldo14");
		if (IS_ERR(regulator_sd)) {
			regulator_sd = NULL;
		} else {
			/* Fix-me, a fake enable call, to find
			 * a right place in the future
			 */
			ret = regulator_enable(regulator_sd);
			if (ret < 0) {
				printk(KERN_ERR "Failed to enalbe LDO14, "
					"SD may not work, ret=%d\n", ret);
				regulator_sd = NULL;
			}
		}
	}

	//get power control pin of SD slot on DKB3.1
	if(is_td_dkb) {
		sd_pwr_en = mfp_to_gpio(GPIO15_GPIO15);
		if (gpio_request(sd_pwr_en, "SD Power Ctrl")) {
			printk(KERN_ERR "Failed to request SD_PWR_EN(gpio %d), "
					"SD card might not work\n", sd_pwr_en);
			sd_pwr_en = 0;
		}
	} else {
		sd_pwr_en = GPIO_EXT1(5);
		if (gpio_request(sd_pwr_en, "SD Power Ctrl")) {
			printk(KERN_ERR "Failed to request SD_PWR_EN(gpio %d), "
					"SD card might not work\n", sd_pwr_en);
			sd_pwr_en = 0;
		}
	}

	if (!regulator_sd || !sd_pwr_en) {
		printk(KERN_ERR "Failed to get power control of SD slot\n");
		return -EIO;
	}

	if (on) {
		gpio_direction_output(sd_pwr_en, 0);
		gpio_free(sd_pwr_en);

		ret = regulator_disable(regulator_sd);
		if (ret < 0)
			printk(KERN_INFO "Failed to turn off LDO14 "
				"for SD slot, ret=%d\n", ret);
	} else {
		gpio_direction_output(sd_pwr_en, 1);
		gpio_free(sd_pwr_en);

		ret = regulator_enable(regulator_sd);
		if (ret < 0)
			printk(KERN_ERR "Failed to enalbe LDO14, "
				"SD may not work, ret=%d\n", ret);
        }
#endif
         set_ldo13(on);  
	return 0;
}

int mmc1_get_ro(struct device *pdev)
{
       
        return 0;
}

static struct pxasdh_platform_data ttc_dkb_sdh_platform_data_0 = {
        .detect_delay   = 20,
        .ocr_mask       = MMC_VDD_27_28|MMC_VDD_28_29,
        .get_ro         = mmc1_get_ro,
        .lp_switch      = mmc1_lp_switch,
	.sd_clock	= 1,
	.sdclk_sel	= 1,
	.sdclk_delay	= 0x2,
};

static struct pxasdh_platform_data ttc_dkb_sdh_platform_data_1 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_165_195,
	.lp_switch	= mmc2_gpio_switch,
};

static struct pxasdh_platform_data ttc_dkb_sdh_platform_data_2 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_27_28|MMC_VDD_28_29,
};

static void ttc_dkb_wifi_set_power(unsigned int on)
{
	static int WIB_EN = 0;
	static int WLAN_LHC = 0;
        #if   0
	if (!WIB_EN || !WLAN_LHC) {
		if (is_td_dkb) {
			WIB_EN = GPIO_EXT1(14);
			WLAN_LHC = GPIO_EXT1(2);
		} else {
			WIB_EN = mfp_to_gpio(WIB_EN_GPIO_33);
			WLAN_LHC = mfp_to_gpio(WLAN_LHC_GPIO_36);
		}

		if (gpio_request(WIB_EN, "WIB_EN")) {
			printk(KERN_INFO "gpio %d request failed\n", WIB_EN);
			WIB_EN = WLAN_LHC = 0;
			return;
		}
		if (gpio_request(WLAN_LHC, "WLAN_LHC")) {
			printk(KERN_INFO "gpio %d request failed\n", WLAN_LHC);
			gpio_free(WIB_EN);
			WIB_EN = WLAN_LHC = 0;
			return;
		}

#ifdef CONFIG_GPIO_SYSFS
		gpio_export(WIB_EN, false);
		gpio_export(WLAN_LHC, false);
#endif
	}
	BUG_ON(!WIB_EN || !WLAN_LHC);

	pr_debug("%s: on=%d\n", __FUNCTION__, on);
	if (on) {
		if (WIB_EN) gpio_direction_output(WIB_EN, 1);
		if (WLAN_LHC) gpio_direction_output(WLAN_LHC, 1);
	} else {
		if (WIB_EN) gpio_direction_output(WIB_EN, 0);
		if (WLAN_LHC) gpio_direction_output(WLAN_LHC, 0);
	}
	#endif
}

static void __init ttc_dkb_init_mmc(void)
{
#if 0
#ifdef CONFIG_SD8XXX_RFKILL
	int WIB_PDn;
	int WIB_RESETn;

	if(is_td_dkb) {
		WIB_PDn = GPIO_EXT1(0);
		WIB_RESETn = GPIO_EXT1(1);
	} else {
		WIB_PDn = mfp_to_gpio(WLAN_PD_GPIO_14);
		WIB_RESETn = mfp_to_gpio(WLAN_RESET_GPIO_20);
	}

	add_sd8x_rfkill_device(WIB_PDn, WIB_RESETn,
		&ttc_dkb_sdh_platform_data_1.pmmc, ttc_dkb_wifi_set_power);
#endif
#endif


#if 0
	if(is_td_dkb) {
		//enable power for SD slot on DKB3.1 for TD
		unsigned long sd_pwr_cfg = GPIO15_GPIO15;

		mfp_config(&sd_pwr_cfg, 1);
		sd_pwr_en = mfp_to_gpio(GPIO15_GPIO15);
		if (gpio_request(sd_pwr_en, "SD Power Ctrl")) {
			printk(KERN_ERR "Failed to request SD_PWR_EN(gpio %d), "
					"SD card might not work\n", sd_pwr_en);
			sd_pwr_en = 0;
		} else {
			gpio_direction_output(sd_pwr_en, 1);
			gpio_free(sd_pwr_en);
		}
	}
#endif


	pxa910_add_sdh(0, &ttc_dkb_sdh_platform_data_0);
	pxa910_add_sdh(1, &ttc_dkb_sdh_platform_data_1);
#if (HOST_SLEEP_EN)
	wake_lock_init(&gpio_wakeup, WAKE_LOCK_SUSPEND, "hs_wakeups");
#endif
	if (emmc_boot) {
		mfp_config(ARRAY_AND_SIZE(emmc_pin_config));
		pxa910_add_sdh(2, &ttc_dkb_sdh_platform_data_2);
	}
}
#endif

#ifdef CONFIG_PXA_U880
extern int  lcd_type;
#endif
#ifdef CONFIG_PM
unsigned long   GPIO[110];
static int ttc_pin_lpm_config(void)
{

	int i2c_trst = MFP_PIN_GPIO86;
	if (gpio_request(i2c_trst, "i2c_trst")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", i2c_trst);
	}
        gpio_direction_output(i2c_trst, 0);     
        gpio_free(i2c_trst);

       	unsigned int pin_index=0,i=0;
       	for(pin_index=MFP_PIN_GPIO0;pin_index<=MFP_PIN_GPIO109;pin_index++)
                 GPIO[i++]=mfp_read(pin_index);

        mfp_write(MFP_PIN_GPIO1,0xd081); 
        mfp_write(MFP_PIN_GPIO3,0xd081);
      
          mfp_write(MFP_PIN_GPIO5,0xd080);

        mfp_write(MFP_PIN_GPIO9,0xd080);
	mfp_write(MFP_PIN_GPIO10,0xb080);
	

       mfp_write(MFP_PIN_GPIO33,0xb080);
	mfp_write(MFP_PIN_GPIO34,0xb080);
	mfp_write(MFP_PIN_GPIO35,0xb080);
	mfp_write(MFP_PIN_GPIO36,0xb080);


	mfp_write(MFP_PIN_GPIO46,0xb0c0);
	mfp_write(MFP_PIN_GPIO49,0xb080);
	#if !defined (CONFIG_PXA_U880	)
	mfp_write(MFP_PIN_GPIO51,0xb080);
	#endif
	mfp_write(MFP_PIN_GPIO52,0xb0c0);

	
	mfp_write(MFP_PIN_GPIO81,0xb081);
	mfp_write(MFP_PIN_GPIO82,0xb081);
	mfp_write(MFP_PIN_GPIO83,0xb081);
	mfp_write(MFP_PIN_GPIO84,0xb081);
	mfp_write(MFP_PIN_GPIO85,0xb081);
	mfp_write(MFP_PIN_GPIO86,0xb081);
	mfp_write(MFP_PIN_GPIO87,0xb081);
	mfp_write(MFP_PIN_GPIO88,0xb081);
	mfp_write(MFP_PIN_GPIO89,0xb081);
	mfp_write(MFP_PIN_GPIO90,0xb081);
	mfp_write(MFP_PIN_GPIO91,0xb081);
	mfp_write(MFP_PIN_GPIO92,0xb081);
	mfp_write(MFP_PIN_GPIO93,0xb081);
	mfp_write(MFP_PIN_GPIO94,0xb081);

	mfp_write(MFP_PIN_GPIO95,0xb081);
	mfp_write(MFP_PIN_GPIO96,0xb081);
	mfp_write(MFP_PIN_GPIO97,0xb081);
	mfp_write(MFP_PIN_GPIO98,0xb081);
	mfp_write(MFP_PIN_GPIO100,0xb081);
	mfp_write(MFP_PIN_GPIO101,0xb081);
	mfp_write(MFP_PIN_GPIO102,0xb081);
	mfp_write(MFP_PIN_GPIO103,0xb081);
	mfp_write(MFP_PIN_GPIO104,0xb083);
	mfp_write(MFP_PIN_GPIO105,0xb083);
	#ifdef CONFIG_PXA_U880
	if(lcd_type!=1)
	     mfp_write(MFP_PIN_GPIO106,0xb080);
        #else
	mfp_write(MFP_PIN_GPIO106,0xb080);
        #endif
     
	mfp_write(MFP_PIN_GPIO107,0xb083);
	mfp_write(MFP_PIN_GPIO108,0xb083);


	mfp_gpio3_power_down(); 

	return 0;
}

static int ttc_pin_restore(void)
{

	int i2c_trst = MFP_PIN_GPIO86;
	if (gpio_request(i2c_trst, "i2c_trst")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", i2c_trst);
	}
        gpio_direction_output(i2c_trst, 1);
        gpio_free(i2c_trst);

       unsigned int pin_index=0,i=0;
        for(pin_index=MFP_PIN_GPIO0;pin_index<=MFP_PIN_GPIO109;pin_index++)
              	mfp_write(pin_index,GPIO[i++]);
#ifdef CONFIG_PXA_U810		
     
      __raw_writel(0x1081,0xfe01e0e0);
        __raw_writel(0x1081,0xfe01e0e8);
      
          __raw_writel(0x1080,0xfe01e0f0);
      
	  __raw_writel(0x1080,0xfe01e100);
	__raw_writel(0x1080,0xfe01e104);
	

      __raw_writel(0x1082,0xfe01e160);
      __raw_writel(0x1082,0xfe01e164);/  
      __raw_writel(0x1082,0xfe01e168);
      __raw_writel(0x1082,0xfe01e16C);

	__raw_writel(0xd0c0,0xfe01e194);
	__raw_writel(0x1080,0xfe01e1a0);
#if (!defined (CONFIG_PXA_U880	))
	__raw_writel(0x1080,0xfe01e1a8);
#endif
	__raw_writel(0xd0c0,0xfe01e1ac);

	__raw_writel(0x1081, 0xfe01e1b8); 
	__raw_writel(0x1081, 0xfe01e1bc);
	__raw_writel(0x1081, 0xfe01e1c0);
	__raw_writel(0x1081, 0xfe01e1c4);
	__raw_writel(0x1081, 0xfe01e1c8);
	__raw_writel(0x1081, 0xfe01e1cc);
	__raw_writel(0x1081, 0xfe01e1d0);
	__raw_writel(0x1081, 0xfe01e1d4);
	__raw_writel(0x1081, 0xfe01e1d8);
	__raw_writel(0x1081, 0xfe01e1dc);
	__raw_writel(0x1081, 0xfe01e1e0);
	__raw_writel(0x1081, 0xfe01e1e4);
	__raw_writel(0xa0c0, 0xfe01e1e8);
	__raw_writel(0xd0c0, 0xfe01e1ec);

       __raw_writel(0x1081, 0xfe01e1f0); 
       __raw_writel(0x1081, 0xfe01e1f4);
       __raw_writel(0x1081, 0xfe01e1f8);
       __raw_writel(0x1081, 0xfe01e1fc);
       __raw_writel(0x1081, 0xfe01e200);
       __raw_writel(0x1081, 0xfe01e204);
       __raw_writel(0x1081, 0xfe01e208);
       __raw_writel(0x1081, 0xfe01e20c);
       __raw_writel(0x1081, 0xfe01e210);
       __raw_writel(0x1081, 0xfe01e214);
       __raw_writel(0x1081, 0xfe01e218);
       __raw_writel(0x1081, 0xfe01e21c);
       __raw_writel(0x1081, 0xfe01e220);
       __raw_writel(0x1081, 0xfe01e224);
       __raw_writel(0x1081, 0xfe01e228);
       __raw_writel(0x1081, 0xfe01e22c);
       __raw_writel(0x1081, 0xfe01e230);
       __raw_writel(0x1081, 0xfe01e234);
       __raw_writel(0x1081, 0xfe01e238);
       __raw_writel(0x1081, 0xfe01e23c);
       __raw_writel(0x1081, 0xfe01e240);
       __raw_writel(0x1081, 0xfe01e244);
       __raw_writel(0x1083, 0xfe01e248);
       __raw_writel(0x1083, 0xfe01e24c);
	#ifdef CONFIG_PXA_U880
	if(lcd_type!=1)
	    __raw_writel(0x1080, 0xfe01e250);
        #else
       __raw_writel(0x1080, 0xfe01e250);
        #endif
       __raw_writel(0x1083, 0xfe01e254);
       __raw_writel(0x1083, 0xfe01e258);

#endif
      
        mfp_gpio3_power_up();
        return 0;
}

static struct pxa910_peripheral_config_ops config_ops = {
	.pin_lpm_config	= ttc_pin_lpm_config,
	.pin_restore	= ttc_pin_restore,
};
#endif

static void gps_power_on(void)
{
	int gps_ldo, gps_rst_n;
	if(is_td_dkb)
                #if (defined( CONFIG_PXA_U880 ) )
		gps_ldo = mfp_to_gpio(MFP_PIN_GPIO50);
                #elif CONFIG_PXA_U812
		gps_ldo = mfp_to_gpio(MFP_PIN_GPIO32);
                #else
                gps_ldo = GPIO_EXT1(12);
                #endif
	else
		gps_ldo = GPIO_EXT1(12);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_ldo);
	}

      gpio_direction_output(gps_ldo, 1);
#if 0   
	if(is_td_dkb)
		gps_rst_n = GPIO_EXT1(11);
	else
		gps_rst_n = mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_rst_n);
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	mdelay(1);
	//gpio_direction_output(gps_ldo, 1);
  gpio_direction_output(gps_ldo, 0);
#endif  
	gpio_free(gps_ldo);
  

	printk(KERN_INFO "sirf gps chip powered on\n");
}

static void gps_power_off(void)
{
	int gps_ldo, gps_rst_n, gps_on;
	if(is_td_dkb)
                #if (defined( CONFIG_PXA_U880 ) )
		gps_ldo =  mfp_to_gpio(MFP_PIN_GPIO50);
 		#elif CONFIG_PXA_U812
		gps_ldo = mfp_to_gpio(MFP_PIN_GPIO32);                
                #else
	        gps_ldo = GPIO_EXT1(12); 
                #endif
	else
		gps_ldo = GPIO_EXT1(7);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_ldo);
	}

	if(is_td_dkb)
                 #if (defined( CONFIG_PXA_U880 ) )
	       	gps_on = mfp_to_gpio(MFP_PIN_GPIO49);
 		#elif CONFIG_PXA_U812
		gps_on = mfp_to_gpio(MFP_PIN_GPIO30);                 
                 #else
                gps_on = GPIO_EXT1(15);
                 #endif
	else
		gps_on = GPIO_EXT1(1);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_on);
	}

	if(is_td_dkb)
                 #if (defined( CONFIG_PXA_U880 ) )
		 gps_rst_n = mfp_to_gpio(MFP_PIN_GPIO109);
 		 #elif CONFIG_PXA_U812
		 gps_rst_n = mfp_to_gpio(MFP_PIN_GPIO31);                 
                 #else
                 gps_rst_n = GPIO_EXT1(11);
                 #endif
	else
		gps_rst_n = mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_rst_n);
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	gpio_direction_output(gps_on, 0);

	gpio_free(gps_ldo);
	gpio_free(gps_on);
	gpio_free(gps_rst_n);
	printk(KERN_INFO "sirf gps chip powered off\n");
}

static void gps_reset(int flag)
{
	int gps_rst_n;

	if(is_td_dkb)
                 #if (defined( CONFIG_PXA_U880 ) )
		gps_rst_n = mfp_to_gpio(MFP_PIN_GPIO109);
 		#elif CONFIG_PXA_U812
		gps_rst_n = mfp_to_gpio(MFP_PIN_GPIO31);                
                 #else
                gps_rst_n = GPIO_EXT1(11);
                #endif
	else
		gps_rst_n = mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_rst_n);
	}
	gpio_direction_output(gps_rst_n, flag);

	gpio_free(gps_rst_n);
	printk(KERN_INFO "sirf gps chip reset\n");
}

static void gps_on_off(int flag)
{
	int gps_on;

	if(is_td_dkb)
                    #if (defined( CONFIG_PXA_U880 ) )
		   gps_on = mfp_to_gpio(MFP_PIN_GPIO49);
		#elif CONFIG_PXA_U812
		gps_on = mfp_to_gpio(MFP_PIN_GPIO30);                     
                   #else
	           gps_on = GPIO_EXT1(15);
                   #endif
	else
		gps_on = GPIO_EXT1(15);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_on);
	}
	gpio_direction_output(gps_on, flag);

	gpio_free(gps_on);
	printk(KERN_INFO "sirf gps chip offon\n");
}

#ifdef	CONFIG_PROC_FS
#define PROC_PRINT(fmt, args...) 	do {len += sprintf(page + len, fmt, ##args); } while(0)

#define SIRF_STATUS_LEN	16
static char sirf_status[SIRF_STATUS_LEN] = "off";

static void gps_get_kind(void)
{
        int gps_kind=1;
	int boardid;

	#ifdef CONFIG_PXA_U810
	gps_kind=0;
 	#endif 

	#ifdef CONFIG_PXA_U812
	gps_kind=1;
 	#endif 

        #ifdef CONFIG_PXA_U880
	boardid = pm860x_get_boardID();
	printk("boardid=%d\n",boardid);
	if((pm860x_get_boardID() == 3)||(pm860x_get_boardID() == 4))
	{
		gps_kind=1;		
	}
	else
	{
		gps_kind=0;
	}
 	#endif
	printk("gps_kind=%d\n",gps_kind); 
	if(1==gps_kind)
	{		
		strcpy(sirf_status, "4t");
	}
	else if(0==gps_kind)
	{
		strcpy(sirf_status, "3tw");
	}
	else
        {
		printk("unknow kind gps kind.\n");
	}
}

static ssize_t sirf_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(sirf_status);

	sprintf(page, "%s\n", sirf_status);
	return len + 1;
}

static ssize_t sirf_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];

	if (len > 255)
		len = 255;

	memset(messages, 0, sizeof(messages));

	if (!buff || copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strlen(messages) > (SIRF_STATUS_LEN - 1)) {
		pr_warning("[ERROR] messages too long! (%d) %s\n", strlen(messages), messages);
		return -EFAULT;
	}

	if (strncmp(messages, "off", 3) == 0) {
		strcpy(sirf_status, "off");
		gps_power_off();
	} else if (strncmp(messages, "on", 2) == 0) {
		strcpy(sirf_status, "on");
		gps_power_on();
	} else if (strncmp(messages, "kind", 4) == 0) {
		gps_get_kind();
	} else if (strncmp(messages, "reset", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_reset(flag);
	} else if (strncmp(messages, "sirfon", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_on_off(flag);
	} else {
		printk("usage: echo {on/off} > /proc/driver/sirf\n");
	}

	return len;
}

static void create_sirf_proc_file(void)
{
	struct proc_dir_entry *sirf_proc_file =
		create_proc_entry("driver/sirf", 0644, NULL);

	if (sirf_proc_file) {
		sirf_proc_file->read_proc = sirf_read_proc;
		sirf_proc_file->write_proc = (write_proc_t  *)sirf_write_proc;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}
#endif

static void __init tds_init(void)
{
	if (is_td_dkb) {
		mfp_config(ARRAY_AND_SIZE(tds_pin_config));
		mfp_write(MFP_PIN_GPIO55, 0x00c0);
		mfp_write(MFP_PIN_GPIO57, 0x00c0);
		mfp_write(MFP_PIN_GPIO58, 0x00c0);
		mfp_write(MFP_PIN_GPIO59, 0x00c0);
	}
}



static struct vibrator_data vibrator_device_data = {
		.max_v = 5,
		.min_v = 0,
		.set_vibrator = pm860x_set_vibrator,
	};
	static struct platform_device vibrator_device = {
		.name            = "vibrator",
		.id              = -1,
		.dev             = {
			.platform_data = &vibrator_device_data,
		},
	};

#if 0
#if (defined CONFIG_CMMB)

static unsigned long cmmb_pin_config[] = {
	GPIO33_SSP0_CLK,
	GPIO35_SSP0_RXD, 
	GPIO36_SSP0_TXD,
};

static struct pxa2xx_spi_master pxa_ssp_master_info = {
        .num_chipselect = 1,
        .enable_dma = 1,
};
/*
 ** spi_finish used by spi_read_bytes to control
 ** GPIO_CMMB_CS to be pull down always
 ** when read not finished.
 **/
int spi_finish = 1;
EXPORT_SYMBOL(spi_finish);

static int cmmb_power_on(void)
{
        int cmmb_en, cmmb_rst;

        cmmb_en = GPIO_EXT1(6);
        if (gpio_request(cmmb_en, "cmmb power")) {
                pr_warning("[ERROR] failed to request GPIO for CMMB POWER\n"); 
                return -EIO;
        }
        gpio_direction_output(cmmb_en, 0);
        msleep(100);  

        gpio_direction_output(cmmb_en, 1);
        gpio_free(cmmb_en);

        msleep(100);

        cmmb_rst = GPIO_EXT1(7);  

        if (gpio_request(cmmb_rst, "cmmb rst")) {
		pr_warning("failed to request GPIO for CMMB RST\n");

                return -EIO;
        }

        /* reset cmmb, keep low for about 1ms */
        gpio_direction_output(cmmb_rst, 0);
        msleep(100);
         
        /* get cmmb go out of reset state */
        gpio_direction_output(cmmb_rst, 1);
        gpio_free(cmmb_rst);

        return 0;
}                                                                                                                                                               
static int cmmb_power_off(void)
{
        int cmmb_en;

        cmmb_en = GPIO_EXT1(6); 

        if (gpio_request(cmmb_en, "cmmb power")) {
                pr_warning("failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}

        gpio_direction_output(cmmb_en, 0);
        gpio_free(cmmb_en);
        msleep(100);

        return 0;
}                                                                                                                                                               
static struct cmmb_platform_data cmmb_info = { 
        .power_on = cmmb_power_on, 
        .power_off = cmmb_power_off,
};                                                                                                                                                              
static void cmmb_if101_cs(u32 cmd) 
{
        int cs;
        if (!spi_finish && cmd == PXA2XX_CS_DEASSERT) 
		return;  
        cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	gpio_direction_output(cs, !(cmd == PXA2XX_CS_ASSERT));
}                                                                                                                                                               
static struct pxa2xx_spi_chip cmmb_if101_chip = { 
        .rx_threshold   = 1, 
        .tx_threshold   = 1, 
        .cs_control     = cmmb_if101_cs,
};                                                                                                                                                              
/* bus_num must match id in pxa2xx_set_spi_info() call */
static struct spi_board_info spi_board_info[] __initdata = {
        {
                .modalias       = "cmmb_if", 
                .platform_data  = &cmmb_info, 
                .controller_data        = &cmmb_if101_chip, 
                .irq            = gpio_to_irq(mfp_to_gpio(GPIO14)), 
                .max_speed_hz   = 8000000,
                .bus_num        = 1,
                .chip_select    = 0,
                .mode           = SPI_MODE_0, 
        },
}; 

static void __init ttc_dkb_init_spi(void)
{
	int err;
	int cmmb_int, cmmb_cs;

	mfp_config(ARRAY_AND_SIZE(cmmb_pin_config));
	cmmb_cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	err = gpio_request(cmmb_cs, "cmmb cs");
	if (err) { 
		pr_warning("[ERROR] failed to request GPIO for CMMB CS\n");
		return;
	}
	gpio_direction_output(cmmb_cs, 1);

	cmmb_int = mfp_to_gpio(GPIO14);

	err = gpio_request(cmmb_int, "cmmb irq");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB IRQ\n");
		return;
	}
	gpio_direction_input(cmmb_int); 
	
	pxa910_add_ssp(0);
	pxa910_add_spi(1, &pxa_ssp_master_info); 
	if(spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info))) {
		pr_warning("[ERROR] failed to register spi device.\n");
		return;
	}
}                                                                                                                                                               
#endif     
#endif

#ifdef CONFIG_USB_ANDROID
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
static struct usb_mass_storage_platform_data mass_storage_pdata = {
       .nluns          = 2,
       .vendor         = "ZTE",
       .product        = "Android",
       .release        = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
       .name   = "usb_mass_storage",
       .id     = -1,
       .dev    = {
		.platform_data  = &mass_storage_pdata,
       },
};
#endif

#if defined(CONFIG_USB_ANDROID_RNDIS)
static struct usb_ether_platform_data usb_rndis_pdata = {
       .ethaddr        = {11, 22, 33, 44, 55, 66},
       .vendorID       = MMP_VENDOR_ID,
       .vendorDescr    = "ZTE Rndis function"
};

static struct platform_device usb_rndis_device = {
       .name   = "rndis",
       .id     = -1,
       .dev    = {
		.platform_data  = &usb_rndis_pdata,
       },
};
#endif

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_DIAG
	"diag",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};


static char *usb_functions_modem_diag_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_MARVELL_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_DIAG
	"diag",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_modem_diag_ums[] = {
#ifdef CONFIG_USB_ANDROID_MARVELL_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_DIAG
	"diag",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
};

static char *usb_functions_rndis_modem_diag[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_DIAG
	"diag",
#endif
};

static char *usb_functions_rndis_modem_diag_adb[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_rndis[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
};

static char *usb_functions_ums[] = {
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
};

static char *usb_functions_modem_diag[] = {
#ifdef CONFIG_USB_ANDROID_MARVELL_MODEM
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_MARVELL_DIAG
	"diag",
#endif
};

static char *usb_functions_ums_adb[] = {
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_rndis_adb[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_modem_ums_adb[] = {
#ifdef CONFIG_USB_ANDROID_MARVELL_MODEM
	"acm",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
#ifdef CONFIG_USB_ANDROID_ADB
	"adb",
#endif
};

static char *usb_functions_modem_ums[] = {
#ifdef CONFIG_USB_ANDROID_MARVELL_MODEM
	"acm",
#endif
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	"usb_mass_storage",
#endif
};

static char *usb_functions_diag[] = {
#ifdef CONFIG_USB_ANDROID_MARVELL_DIAG
	"diag",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id = MMP_ALL_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_all),
		.functions = usb_functions_all,
	},
	{
		.product_id = MMP_MODEM_DIAG_UMS_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_diag_ums),
		.functions = usb_functions_modem_diag_ums,
	},
	{
		.product_id = MMP_MODEM_DIAG_UMS_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_diag_ums_adb),
		.functions = usb_functions_modem_diag_ums_adb,
	},
	{
		.product_id = MMP_RNDIS_MODEM_DIAG_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_rndis_modem_diag),
		.functions = usb_functions_rndis_modem_diag,
	},
	{
		.product_id = MMP_RNDIS_MODEM_DIAG_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_rndis_modem_diag_adb),
		.functions = usb_functions_rndis_modem_diag_adb,
	},
	{
		.product_id = MMP_RNDIS_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_rndis),
		.functions = usb_functions_rndis,
	},
	{
		.product_id = MMP_UMS_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_ums),
		.functions = usb_functions_ums,
	},
	{
		.product_id = MMP_MODEM_DIAG_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_diag),
		.functions = usb_functions_modem_diag,
	},
	{
		.product_id = MMP_UMS_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_ums_adb),
		.functions = usb_functions_ums_adb,
	},
	{
		.product_id = MMP_RNDIS_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions = usb_functions_rndis_adb,
	},
	{
		.product_id = MMP_MODEM_UMS_ADB_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_ums_adb),
		.functions = usb_functions_modem_ums_adb,
	},
	{
		.product_id = MMP_MODEM_UMS_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_modem_ums),
		.functions = usb_functions_modem_ums,
	},
	{
		.product_id = MMP_DIAG_PRODUCT_ID,
		.num_functions = ARRAY_SIZE(usb_functions_diag),
		.functions = usb_functions_diag,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id              = MMP_VENDOR_ID,
	.product_id             = MMP_MODEM_UMS_ADB_PRODUCT_ID,
	.version                = 0x0100,
	.product_name           = "Android",
	.manufacturer_name      = "ZTE",
	.num_functions          = ARRAY_SIZE(usb_functions_all),
	.functions              = usb_functions_all,
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
};

static struct platform_device android_device_usb = {
       .name   = "android_usb",
       .id     = -1,
       .dev    = {
		.platform_data  = &android_usb_pdata,
       },
};


void __init android_add_usb_devices(void)
{
#if defined(CONFIG_USB_ANDROID_MASS_STORAGE) || defined(CONFIG_USB_FILE_STORAGE)
	platform_device_register(&usb_mass_storage_device);
#endif
#if defined(CONFIG_USB_ANDROID_RNDIS)
	platform_device_register(&usb_rndis_device);
#endif
	platform_device_register(&android_device_usb);
}

#endif
   


#ifdef CONFIG_PXA_U802
#define GPIO08_WLAN_RESET              MFP_CFG(GPIO8, AF0)
#define GPIO09_WLAN_PD                MFP_CFG(GPIO9, AF0)
#define U802_YIJIEBAN_ID 2
static unsigned long U802_yijieban_pin_config[] = {
	GPIO08_WLAN_RESET,
	GPIO09_WLAN_PD,
};
#endif

static void u810_wlan_gpio_setpower(int enable)
{
    int err = 0;
    int en = enable?1:0;
    
    #if  1
    printk("in %s, enable is %d\r\n", __func__, en);

  
    err = gpio_request(mfp_to_gpio(GPIO12_WB_HPWR),"8787 3.3V pwr");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO12_WB_HPWR));
    }
    #if ! (defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830))
    err = gpio_request(mfp_to_gpio(GPIO79_BT_CLK),"bt clk");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO79_BT_CLK));
    }
    #endif   
    
    #ifdef CONFIG_PXA_U880
    printk("U880\n");
    err = gpio_request(mfp_to_gpio(GPIO11_WLAN_RESET), "8787 reset");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO11_WLAN_RESET));
    }
    
    err = gpio_request(mfp_to_gpio(GPIO07_WLAN_PD), "8787 power down");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO07_WLAN_PD));
    }
    #elif defined(CONFIG_PXA_U830) ||defined(CONFIG_PXA_U812)
    printk("U830/U812\n");
    err = gpio_request(mfp_to_gpio(GPIO79_WLAN_RESET), "8787 reset");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO79_WLAN_RESET));
    }
    err = gpio_request(mfp_to_gpio(GPIO07_WLAN_PD), "8787 power down");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO07_WLAN_PD));
    }
    #else
    printk("U810/U802\n");

    #ifdef CONFIG_PXA_U802
    printk("in %s, pm860x_get_boardID() is %d\r\n", __func__, pm860x_get_boardID());
    if(pm860x_get_boardID() == U802_YIJIEBAN_ID)
    {
        err = gpio_request(mfp_to_gpio(GPIO08_WLAN_RESET), "8787 reset");
        if(err)
        {
            printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO08_WLAN_RESET));
        }
        
        err = gpio_request(mfp_to_gpio(GPIO09_WLAN_PD), "8787 power down");
        if(err)
        {
            printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO09_WLAN_PD));
        }
    }
    else
    #endif
    {
    err = gpio_request(mfp_to_gpio(GPIO_EXT1(4)),"8787 reset");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO_EXT1(4)));
    }
    
    err = gpio_request(mfp_to_gpio(GPIO_EXT1(5)),"8787 power down");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO_EXT1(5)));
    }

    err = gpio_request(mfp_to_gpio(GPIO_EXT1(2)),"8787 fm rx/tx");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO_EXT1(2)));
    }
    }
    #endif
    
    #if ! (defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830))
  
    gpio_direction_output(mfp_to_gpio(GPIO79_BT_CLK),en);
    msleep(10);
    #endif
    
    
    gpio_direction_output(mfp_to_gpio(GPIO12_WB_HPWR),en);
    msleep(10);
    
    #ifdef CONFIG_PXA_U880 
    if((pm860x_get_boardID() == 2) || (pm860x_get_boardID() == 4))
    {
        set_ldo8(en);
    }

  
    gpio_direction_output(mfp_to_gpio(GPIO07_WLAN_PD),en);
    msleep(10);
    
  
    gpio_direction_output(mfp_to_gpio(GPIO11_WLAN_RESET),(en+1)%2);
    msleep(50);
    gpio_direction_output(mfp_to_gpio(GPIO11_WLAN_RESET),en);
    msleep(50);
    #elif defined(CONFIG_PXA_U830) || defined(CONFIG_PXA_U812)
  
    gpio_direction_output(mfp_to_gpio(GPIO07_WLAN_PD),en);
    msleep(10);
    
 
    gpio_direction_output(mfp_to_gpio(GPIO79_WLAN_RESET),(en+1)%2);
    msleep(50);
    gpio_direction_output(mfp_to_gpio(GPIO79_WLAN_RESET),en);
    msleep(50);
    #else

    #ifdef CONFIG_PXA_U802
    if(pm860x_get_boardID() == U802_YIJIEBAN_ID)
    {
       
        gpio_direction_output(mfp_to_gpio(GPIO09_WLAN_PD),en);
        msleep(10);
        
      
        gpio_direction_output(mfp_to_gpio(GPIO08_WLAN_RESET),(en+1)%2);
        msleep(50);
        gpio_direction_output(mfp_to_gpio(GPIO08_WLAN_RESET),en);
        msleep(50);
    }
    else
    #endif
    {
   
    gpio_direction_output(mfp_to_gpio(GPIO_EXT1(5)),en);
    msleep(10);
    
   
    gpio_direction_output(mfp_to_gpio(GPIO_EXT1(4)),en);
    msleep(10);

  
    gpio_direction_output(mfp_to_gpio(GPIO_EXT1(2)),0);
    msleep(10);
    }
    #endif


    gpio_free(mfp_to_gpio(GPIO12_WB_HPWR));
    #if ! (defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830))
    gpio_free(mfp_to_gpio(GPIO79_BT_CLK));
    #endif

    #ifdef CONFIG_PXA_U880
    gpio_free(mfp_to_gpio(GPIO07_WLAN_PD));

    gpio_free(mfp_to_gpio(GPIO11_WLAN_RESET));
    #elif defined(CONFIG_PXA_U830) || defined(CONFIG_PXA_U812)
    gpio_free(mfp_to_gpio(GPIO07_WLAN_PD));

    gpio_free(mfp_to_gpio(GPIO79_WLAN_RESET));

    #else
    #ifdef CONFIG_PXA_U802
    if(pm860x_get_boardID() == U802_YIJIEBAN_ID)
    {
        gpio_free(mfp_to_gpio(GPIO09_WLAN_PD));

        gpio_free(mfp_to_gpio(GPIO08_WLAN_RESET));
    }
    else
    #endif
    {
    gpio_free(mfp_to_gpio(GPIO_EXT1(5)));

    gpio_free(mfp_to_gpio(GPIO_EXT1(4)));

    gpio_free(mfp_to_gpio(GPIO_EXT1(2)));
    }
    #endif
    
    #endif
}

struct wlan_platform_data u810_wlan_pdata = 
{
    .wlan_state = WB_STATE_STOPPED,
    .bt_state = WB_STATE_STOPPED,
    .chip_gpio_setpower = u810_wlan_gpio_setpower,
};
EXPORT_SYMBOL(u810_wlan_pdata);


static struct platform_device bt_device = {
	.name		= "mrvl8787bt",
	.id		= -1,
};

static struct platform_device wlan_device = {
	.name		= "mrvl8787wlan",
	.id		= -1,
};


static void __init u810_wlan_init(void)
{
    int err = 0;
    
    printk("in %s\r\n", __func__);

    #ifdef CONFIG_PXA_U802
    if(pm860x_get_boardID() == U802_YIJIEBAN_ID)
    {
        mfp_config(ARRAY_AND_SIZE(U802_yijieban_pin_config));
        err = gpio_request(mfp_to_gpio(GPIO08_WLAN_RESET), "8787 reset");
        if(err)
        {
            printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO08_WLAN_RESET));
        }
        
        err = gpio_request(mfp_to_gpio(GPIO09_WLAN_PD), "8787 power down");
        if(err)
        {
            printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO09_WLAN_PD));
        }

     
        gpio_direction_output(mfp_to_gpio(GPIO09_WLAN_PD),0);
        
      
        gpio_direction_output(mfp_to_gpio(GPIO08_WLAN_RESET),0);

        gpio_free(mfp_to_gpio(GPIO09_WLAN_PD));

        gpio_free(mfp_to_gpio(GPIO08_WLAN_RESET));
    }
    #endif
    
  
    #ifdef CONFIG_PXA_U880
    err = gpio_request(mfp_to_gpio(GPIO11_WLAN_RESET), "8787 reset");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO11_WLAN_RESET));
    }
    
    err = gpio_request(mfp_to_gpio(GPIO07_WLAN_PD), "8787 power down");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO07_WLAN_PD));
    }
    #endif
    #if defined(CONFIG_PXA_U830) || defined(CONFIG_PXA_U812)
    err = gpio_request(mfp_to_gpio(GPIO79_WLAN_RESET), "8787 reset");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO79_WLAN_RESET));
    }
    
    err = gpio_request(mfp_to_gpio(GPIO07_WLAN_PD), "8787 power down");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO07_WLAN_PD));
    }
    #endif   
    err = gpio_request(mfp_to_gpio(GPIO12_WB_HPWR),"8787 3.3V pwr");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO12_WB_HPWR));
    }

    err = gpio_request(mfp_to_gpio(GPIO13_WB_WAKEUP),"WB wakeup");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO13_WB_WAKEUP));
    }

    err = gpio_request(mfp_to_gpio(GPIO45_BT_WAKEUP),"bt wakeup");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO45_BT_WAKEUP));
    }

    err = gpio_request(mfp_to_gpio(GPIO46_FM_WAKEUP),"fm wakeup");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO46_FM_WAKEUP));
    }
    #if ! (defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830))
    err = gpio_request(mfp_to_gpio(GPIO79_BT_CLK),"bt clk");
    if(err)
    {
        printk("8787 gpio %d request error\r\n", mfp_to_gpio(GPIO79_BT_CLK));
    }
    #endif

    #ifdef CONFIG_PXA_U880 
   
    gpio_direction_output(mfp_to_gpio(GPIO07_WLAN_PD),0);
    
    
    gpio_direction_output(mfp_to_gpio(GPIO11_WLAN_RESET),0);
    #endif
    #if defined(CONFIG_PXA_U830) || defined(CONFIG_PXA_U812)
    
    gpio_direction_output(mfp_to_gpio(GPIO07_WLAN_PD),0);
    
   
    gpio_direction_output(mfp_to_gpio(GPIO79_WLAN_RESET),0);
    #endif 
  
    gpio_direction_output(mfp_to_gpio(GPIO12_WB_HPWR),0);
    
    #if ! (defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830))
  
    gpio_direction_output(mfp_to_gpio(GPIO79_BT_CLK),0);
    #endif
    

    gpio_direction_input(mfp_to_gpio(GPIO13_WB_WAKEUP));


    gpio_direction_input(mfp_to_gpio(GPIO45_BT_WAKEUP));

 
    gpio_direction_input(mfp_to_gpio(GPIO46_FM_WAKEUP));

    #ifdef CONFIG_PXA_U880 
    gpio_free(mfp_to_gpio(GPIO07_WLAN_PD));

    gpio_free(mfp_to_gpio(GPIO11_WLAN_RESET));
    #endif
    #if defined(CONFIG_PXA_U830) || defined(CONFIG_PXA_U812)
    gpio_free(mfp_to_gpio(GPIO07_WLAN_PD));
    
    gpio_free(mfp_to_gpio(GPIO79_WLAN_RESET));
    #endif    
    gpio_free(mfp_to_gpio(GPIO12_WB_HPWR));

    gpio_free(mfp_to_gpio(GPIO13_WB_WAKEUP));

    gpio_free(mfp_to_gpio(GPIO45_BT_WAKEUP));

    gpio_free(mfp_to_gpio(GPIO46_FM_WAKEUP));

    #if ! (defined(CONFIG_PXA_U812) || defined(CONFIG_PXA_U830))
    gpio_free(mfp_to_gpio(GPIO79_BT_CLK));
    #endif
    
    platform_device_register(&bt_device);
    platform_device_register(&wlan_device);
    
    printk("end %s\r\n", __func__);
    
    return;
}


extern unsigned char zte_prodtestmod;
extern unsigned char zte_fixturemod;

void usb_switch_uart(void)
{
     int err = 0;
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO20), "USB-UART");
        if (err) {
                printk("failed to request GPIO20 for USB-UART\n");
                return;
        }

	if(zte_prodtestmod)
	{
		if(!zte_fixturemod)
		{
			printk("bsp&(no fixture) mode, switch to usb\n");
			gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO20), 0);
			gpio_free(mfp_to_gpio(MFP_PIN_GPIO20));
             		return;
		}	
	}
		
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO20), 1);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO20));
}

void uart_switch_usb(void)
{
      int err = 0;
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO20), "USB-UART");
        if (err) {
                printk("failed to request GPIO20 for USB-UART\n");
                return;
        }
        
        if(zte_fixturemod)
	      {
	            printk("fixture mode, switch to uart\n");
	            gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO20), 1);
        			gpio_free(mfp_to_gpio(MFP_PIN_GPIO20));
	            return;
	      }
      
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO20), 0);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO20));
}

extern void __init u810_add_lcd_nt35580(void);


static void register_g_sensor(struct work_struct *work)
{
	int boardid = pm860x_get_boardID();
	printk("----------------- boardid = %d -----------------\n", boardid);
	if(ZTE_HWVERSION1 == boardid)
	{
		platform_device_register(&g_sensor_device);	
	}
	if(ZTE_HWVERSION2 == boardid)
	{
		printk("********************\n");
	}
}


static void __init ttc_dkb_init(void)
{
	pm860x_get_boardID_fromboot();
	

	platform_device_register(&sensor_input_device);

        mfp_config(ARRAY_AND_SIZE(ttc_dkb_pin_config));

        int gps_ldo;
        if(is_td_dkb)
                gps_ldo = GPIO_EXT1(11);

        if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
                printk(KERN_ERR "Request GPIO failed,"
                                "gpio: %d \n", gps_ldo);
        }

        gpio_direction_output(gps_ldo, 1);
        gpio_free(gps_ldo);


        int err = 0;
#if (defined CONFIG_PXA_U880)
         err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO15), "gps_on");
        if (err) {
                printk("failed to request GPIO15 for USB-UART\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO15), 1);
	  gpio_free(mfp_to_gpio(MFP_PIN_GPIO15));
	  
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO49), "gps_on");
        if (err) {
                printk("failed to request GPIO20 for USB-UART\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO49), 0);
    
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO49));
	      err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO50), "gps_on");
        if (err) {
                printk("failed to request GPIO20 for USB-UART\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO50), 0);
     
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO50));
	      err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO109), "gps_on");
        if (err) {
                printk("failed to request GPIO20 for USB-UART\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO109), 0);
     
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO109));
#endif
	uart_switch_usb();

#if (defined CONFIG_PXA_U810 ||defined CONFIG_PXA_U802)
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO5), "Bt-Codec");
        if (err) {
                printk("failed to request GPIO5 for Bt-Codec\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO5), 0);


        gpio_free(mfp_to_gpio(MFP_PIN_GPIO5));
#endif


 
        #if 0
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO15), "CMMB");
        if (err) {
                printk("failed to request GPIO15 for CMMB\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO15), 0);
        msleep(100);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO15));

        #endif

        #if 0
       err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO12), "Bt-wifi");
        if (err) {
                printk("failed to request GPIO12 for Bt-wifi\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO12), 0);
        msleep(100);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO12));
     
     err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO45), "Bt-Codec");
        if (err) {
                printk("failed to request GPIO45 for Bt-Codec\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO45), 0);
        msleep(100);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO45));
        #endif

#if (defined CONFIG_PXA_U810 ||defined CONFIG_PXA_U802)
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO4), "ECHO_POWERDOWN");
        if (err) {
                printk("failed to request GPIO4 for ECHO_POWERDOWN\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO4), 0);
    
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO4));
        
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO9), "ECHO_RESET");
        if (err) {
                printk("failed to request GPIO9 for ECHO_RESET\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO9), 1);
 
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO9));
#endif   




#ifdef CONFIG_PXA_U880
		err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO6), "AUDIO_PA_EN");
        if (err) {
                printk("failed to request GPIO6 for AUDIO_PA_EN\n");
                return;
        }
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 0);
		gpio_free(mfp_to_gpio(MFP_PIN_GPIO6));
#endif

   
         err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO84), "Bt-Codec");
        if (err) {
                printk("failed to request GPIO9 for Bt-Codec\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO84), 0);
   
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO84));

        #if 0
        err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO79), "Bt-Codec");
        if (err) {
                printk("failed to request GPIO9 for Bt-Codec\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO79), 0);
        msleep(100);
        gpio_free(mfp_to_gpio(MFP_PIN_GPIO79));
        #endif


        #ifdef CONFIG_PXA_U880
        err = gpio_request(mfp_to_gpio(GPIO124_GPIO124), "cmmb pd");
        if (err) {
                printk("failed to request GPIO124 for cmmb pd\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(GPIO124_GPIO124), 0);
        msleep(100);
        gpio_free(mfp_to_gpio(GPIO124_GPIO124));
        #endif
        

        pxa910_add_uart(1);
        pxa910_add_uart(2);
	pxa910_add_uart(3);

	pxa910_add_ire();
	pxa910_add_twsi(0, &i2c_info, ARRAY_AND_SIZE(ttc_dkb_i2c_info));

   
	pxa910_add_ssp(0);
      

	pxa910_add_ssp(1);
	pxa910_add_keypad(&ttc_dkb_keypad_info);

#if defined(CONFIG_MMC_PXA_SDH)
	ttc_dkb_init_mmc();
#endif

#ifdef CONFIG_USB_GADGET
	platform_device_register(&pxa910_device_u2o);
#endif

#ifdef CONFIG_USB_OTG
	platform_device_register(&pxa910_device_u2ootg);
	platform_device_register(&pxa910_device_u2oehci);
#endif

#ifdef CONFIG_USB_ANDROID
	android_add_usb_devices();
#endif

#if defined(CONFIG_PXA910_CAMERA)
	pxa910_add_cam();
#endif
	pxa910_add_acipc();


	ttc_dkb_init_flash();

		pxa910_add_cnm();
	platform_add_devices(ARRAY_AND_SIZE(ttc_dkb_devices));




#if 0
        if(is_wvga_lcd()){
                dkb_add_lcd_sharp();
                printk("LCD: sharp WVGA panel selected.\n");
        }else{
                dkb_add_lcd_tpo();
        }
#endif
#ifdef CONFIG_PXA_U802
	u810_add_lcd_ili9325();
#elif defined CONFIG_PXA_U810
        u810_add_lcd_lead();
#elif defined CONFIG_PXA_U880
        u880_add_lcd();
#elif defined CONFIG_PXA_U812
        u810_add_lcd_lead();
#elif defined CONFIG_PXA_U830
        u810_add_lcd_lead();
#endif

        pxa910_add_freq();
        pxa910_add_rtc();
        tds_init();
	
	u810_init_mvd();
    u810_wlan_init();

#if (defined CONFIG_PXA_U810 || defined CONFIG_PXA_U880 || defined CONFIG_PXA_U812)
		platform_device_register(&g_sensor_device);	
#endif

#ifdef CONFIG_PXA_U802

	if(ZTE_HWVERSION1 == pm860x_get_boardID())
	{
		platform_device_register(&g_sensor_device);	
	}
#endif

#if (defined CONFIG_PXA_U810 || defined CONFIG_PXA_U802)
		platform_device_register(&akm8973_device);	
#endif

#ifdef CONFIG_PXA_U880
		platform_device_register(&akm8962_device);
              err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO15), "gps_on");
        if (err) {
                printk("failed to request GPIO15 for USB-UART\n");
                return;
        }
        gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO15),0);
	  gpio_free(mfp_to_gpio(MFP_PIN_GPIO15));
#endif


#ifdef	CONFIG_PROC_FS
      
        create_sirf_proc_file();
#endif
platform_device_register(&vibrator_device);
#ifdef CONFIG_PM
        pxa910_power_config_register(&config_ops);
#endif


#if (defined CONFIG_PXA_U810 || defined CONFIG_PXA_U812)
  U810_alsps_init();
#endif


#if 0
#if (defined CONFIG_CMMB)
        /* spi device */
        if(is_td_dkb)
                ttc_dkb_init_spi();
#endif
#endif


}

subsys_initcall_sync(ttc_dkb_board_init);

MACHINE_START(TTC_DKB, "PXA910-based TTC_DKB Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa910_init_irq,
	.timer          = &pxa910_timer,
	.init_machine   = ttc_dkb_init,
MACHINE_END
