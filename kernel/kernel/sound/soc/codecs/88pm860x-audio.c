/*
 * linux/sound/soc/codecs/88pm860x_audio.c
 * Base on linux/sound/soc/codecs/wm8753.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * Author: Bin Yang <bin.yang@marvell.com> 
 * 			 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/mfd/88pm860x.h>
#include <mach/regs-mpmu.h>
#include <asm/io.h>
#include <linux/gpio.h>         	
#include <plat/mfp.h>	

#include "88pm860x-audio.h"
#include <mach/pxa910-aec-fm2010.h>  		

#define SANREMO_SOC_PROC
#define AUDIO_NAME "pm860x audio codec"
#define PM860X_AUDIO_VERSION "0.4"

#define ARRAY_AND_SIZE(x)	x, ARRAY_SIZE(x)

//extern int tpa2018_init_client(void);
#ifdef CONFIG_PXA_U810
extern int tpa2018_wakeup(void);
extern int tpa2018_sleep(void);
#endif

/* codec private data */
struct pm860x_audio_priv {
        unsigned int sysclk;
        unsigned int pcmclk;
	unsigned int samplerate;
};

/*here is the pm860x audio default value, audio server will reset the value to it*/
static const u8 pm860x_audio_regs[] = {
	0x00, 0x00, 0x00, 0x00,	/*0x00 ~ 0x03*/
	0x00, 0x00, 0x00, 0x00, /*0x04 ~ 0x07*/
	0x00, 0x00, 0x00, 0x00, /*0x08 ~ 0x0b*/
	0x08, 0x00, 0x40, 0x00, /*0x0c ~ 0x0f*/
	0x00, 0x00, 0x00, 0x00,	/*0x10 ~ 0x13*/
	0x00, 0x3f, 0x3f, 0x3f,	/*0x14 ~ 0x17*/
	0x3f, 0x3f, 0x3f, 0x00,	/*0x18 ~ 0x1b*/
	0x00, 0x00, 0x00, 0x00,	/*0x1c ~ 0x1f*/
	0x00, 0x00, 0x00, 0x00,	/*0x20 ~ 0x23*/
	0x00, 0x00, 0x00, 0x00,	/*0x24 ~ 0x27*/
	0x00, 0x00, 0x00, 0x00,	/*0x28 ~ 0x2b*/
	0x00, 0x00, 0x00, 0x00,	/*0x2c ~ 0x2f*/
	0x00, 0x00, 0x00, 0x00,	/*0x30 ~ 0x33*/
	0x00, 0x00, 0x00, 0x00,	/*0x34 ~ 0x37*/
	0x00, 0x00, 0x00, 0x00,	/*0x38 ~ 0x3b*/
	0x00, 0x00, 0x00, 0x00, 0x00,		  /*0x42 ~ 0x46*/
};

static const struct snd_kcontrol_new levante_direct_access[] = {
	/* Audio Register */
	SOC_SINGLE("LEVANTE_PCM_INTERFACE1", PCM_INTERFACE1, 0, 0xff, 0),              /* 0xB0 */
	SOC_SINGLE("LEVANTE_PCM_INTERFACE2", PCM_INTERFACE2, 0, 0xff, 0),              /* 0xB1 */
	SOC_SINGLE("LEVANTE_PCM_INTERFACE3", PCM_INTERFACE3, 0, 0xff, 0),              /* 0xB2 */
	SOC_SINGLE("LEVANTE_PCM_RATE", ADC_PCM, 0, 0xff, 0),                           /* 0xB3 */
	SOC_SINGLE("LEVANTE_ECHO_CANCEL_PATH", ECHO_CANCEL_PATH, 0, 0xff, 0),          /* 0xB4 */
	SOC_SINGLE("LEVANTE_SIDETONE_GAIN1", SIDETONE_GAIN1, 0, 0xff, 0),              /* 0xB5 */
	SOC_SINGLE("LEVANTE_SIDETONE_GAIN2", SIDETONE_GAIN2, 0, 0xff, 0),              /* 0xB6 */
	SOC_SINGLE("LEVANTE_SIDETONE_GAIN3", SIDETONE_GAIN3, 0, 0xff, 0),              /* 0xB7 */
	SOC_SINGLE("LEVANTE_ADC_OFFSET1", ADC_OFFSET1, 0, 0xff, 0),                    /* 0xB8 */
	SOC_SINGLE("LEVANTE_ADC_OFFSET2", ADC_OFFSET2, 0, 0xff, 0),                    /* 0xB9 */
	SOC_SINGLE("LEVANTE_DMIC_DELAY", DMIC_DELAY, 0, 0xff, 0),                      /* 0xBA */
	SOC_SINGLE("LEVANTE_I2S_INTERFACE1", I2S_INTERFACE_1, 0, 0xff, 0),             /* 0xBB */
	SOC_SINGLE("LEVANTE_I2S_INTERFACE2", I2S_INTERFACE_2, 0, 0xff, 0),             /* 0xBC */
	SOC_SINGLE("LEVANTE_I2S_INTERFACE3", I2S_INTERFACE_3, 0, 0xff, 0),             /* 0xBD */
	SOC_SINGLE("LEVANTE_I2S_INTERFACE4", I2S_INTERFACE_4, 0, 0xff, 0),             /* 0xBE */
	SOC_SINGLE("LEVANTE_EQUALIZER_N0_1", EQUALIZER_N0_1, 0, 0xff, 0),              /* 0xBF */
	SOC_SINGLE("LEVANTE_EQUALIZER_N0_2", EQUALIZER_N0_2, 0, 0xff, 0),              /* 0xC0 */
	SOC_SINGLE("LEVANTE_EQUALIZER_N1_1", EQUALIZER_N1_1, 0, 0xff, 0),              /* 0xC1 */
	SOC_SINGLE("LEVANTE_EQUALIZER_N1_2", EQUALIZER_N1_2, 0, 0xff, 0),              /* 0xC2 */
	SOC_SINGLE("LEVANTE_EQUALIZER_D1_1", EQUALIZER_D1_1, 0, 0xff, 0),              /* 0xC3 */
	SOC_SINGLE("LEVANTE_EQUALIZER_D1_2", EQUALIZER_D1_2, 0, 0xff, 0),              /* 0xC4 */
	SOC_SINGLE("LEVANTE_SIDE_TONE1", SIDE_TONE_1, 0, 0xff, 0),                     /* 0xC5 */
	SOC_SINGLE("LEVANTE_SIDE_TONE2", SIDE_TONE_2, 0, 0xff, 0),                     /* 0xC6 */
	SOC_SINGLE("LEVANTE_LEFT_GAIN1", LEFT_GAIN1, 0, 0xff, 0),                      /* 0xC7 */
	SOC_SINGLE("LEVANTE_LEFT_GAIN2", LEFT_GAIN2, 0, 0xff, 0),                      /* 0xC8 */
	SOC_SINGLE("LEVANTE_RIGHT_GAIN1", RIGHT_GAIN1, 0, 0xff, 0),                    /* 0xC9 */
	SOC_SINGLE("LEVANTE_RIGHT_GAIN2", RIGHT_GAIN2, 0, 0xff, 0),                    /* 0xCA */
	SOC_SINGLE("LEVANTE_DAC_OFFSET", DAC_OFFSET, 0, 0xff, 0),                      /* 0xCB */
	SOC_SINGLE("LEVANTE_OFFSET_LEFT1", OFFSET_LEFT1, 0, 0xff, 0),                  /* 0xCC */
	SOC_SINGLE("LEVANTE_OFFSET_LEFT2", OFFSET_LEFT2, 0, 0xff, 0),                  /* 0xCD */
	SOC_SINGLE("LEVANTE_OFFSET_RIGHT1", OFFSET_RIGHT1, 0, 0xff, 0),                /* 0xCE */
	SOC_SINGLE("LEVANTE_OFFSET_RIGHT2", OFFSET_RIGHT2, 0, 0xff, 0),                /* 0xCF */
	SOC_SINGLE("LEVANTE_ADC_ANALOG_PROGRAM1", ADC_ANALOG_PROGRAM1, 0, 0xff, 0),    /* 0xD0 */
	SOC_SINGLE("LEVANTE_ADC_ANALOG_PROGRAM2", ADC_ANALOG_PROGRAM2, 0, 0xff, 0),    /* 0xD1 */
	SOC_SINGLE("LEVANTE_ADC_ANALOG_PROGRAM3", ADC_ANALOG_PROGRAM3, 0, 0xff, 0),    /* 0xD2 */
	SOC_SINGLE("LEVANTE_ADC_ANALOG_PROGRAM4", ADC_ANALOG_PROGRAM4, 0, 0xff, 0),    /* 0xD3 */
	SOC_SINGLE("LEVANTE_ANALOG_TO_ANALOG", ANALOG_TO_ANALOG, 0, 0xff, 0),          /* 0xD4 */
	SOC_SINGLE("LEVANTE_HS1_CTRL", HS1_CTRL, 0, 0xff, 0),                          /* 0xD5 */
	SOC_SINGLE("LEVANTE_HS2_CTRL", HS2_CTRL, 0, 0xff, 0),                          /* 0xD6 */
	SOC_SINGLE("LEVANTE_LO1_CTRL", LO1_CTRL, 0, 0xff, 0),                          /* 0xD7 */
	SOC_SINGLE("LEVANTE_LO2_CTRL", LO2_CTRL, 0, 0xff, 0),                          /* 0xD8 */
	SOC_SINGLE("LEVANTE_EAR_SPKR_CTRL1", EAR_SPKR_CTRL1, 0, 0xff, 0),              /* 0xD9 */
	SOC_SINGLE("LEVANTE_EAR_SPKR_CTRL2", EAR_SPKR_CTRL2, 0, 0xff, 0),              /* 0xDA */
	SOC_SINGLE("LEVANTE_AUDIO_SUPPLIES1", AUDIO_SUPPLIES1, 0, 0xff, 0),            /* 0xDB */
	SOC_SINGLE("LEVANTE_AUDIO_SUPPLIES2", AUDIO_SUPPLIES2, 0, 0xff, 0),            /* 0xDC */
	SOC_SINGLE("LEVANTE_ADC_ENABLES1", ADC_ENABLES1, 0, 0xff, 0),                  /* 0xDD */
	SOC_SINGLE("LEVANTE_ADC_ENABLES2", ADC_ENABLES2, 0, 0xff, 0),                  /* 0xDE */
	SOC_SINGLE("LEVANTE_DAC_ENABLES1", DAC_ENABLES1, 0, 0xff, 0),                  /* 0xDF */
	SOC_SINGLE("LEVANTE_DAC_DUMMY", DUMMY, 0, 0xff, 0),                            /* 0xE0 */
	SOC_SINGLE("LEVANTE_DAC_ENABLES2", DAC_ENABLES2, 0, 0xff, 0),                  /* 0xE1 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL1", AUDIO_CAL1, 0, 0xff, 0),                      /* 0xE2 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL2", AUDIO_CAL2, 0, 0xff, 0),                      /* 0xE3 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL3", AUDIO_CAL3, 0, 0xff, 0),                      /* 0xE4 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL4", AUDIO_CAL4, 0, 0xff, 0),                      /* 0xE5 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL5", AUDIO_CAL5, 0, 0xff, 0),                      /* 0xE6 */
	SOC_SINGLE("LEVANTE_ANALOG_INPUT_SEL1", ANALOG_INPUT_SEL1, 0, 0xff, 0),        /* 0xE7 */
	SOC_SINGLE("LEVANTE_ANALOG_INPUT_SEL2", ANALOG_INPUT_SEL2, 0, 0xff, 0),        /* 0xE8 */
	SOC_SINGLE("LEVANTE_MIC_BUTTON_DETECTION", PCM_INTERFACE4, 0, 0xff, 0),        /* 0xE9 */
	SOC_SINGLE("LEVANTE_HEADSET_DETECTION", I2S_INTERFACE_5, 0, 0xff, 0),          /* 0xEA */
	SOC_SINGLE("LEVANTE_SHORTS", SHORTS, 0, 0xff, 0),                              /* 0xEB */
	/* MIsc Register for audio clock configuration */
	SOC_SINGLE("LEVANTE_MISC2", MISC2, 0, 0xff, 0),                                /* 0x42 */
	SOC_SINGLE("LEVANTE_PLL_CTRL1", PLL_CTRL1, 0, 0xff, 0),                        /* 0x43 */
	SOC_SINGLE("LEVANTE_PLL_FRAC1", PLL_FRAC1, 0, 0xff, 0),                        /* 0x44 */
	SOC_SINGLE("LEVANTE_PLL_FRAC2", PLL_FRAC2, 0, 0xff, 0),                        /* 0x45 */
	SOC_SINGLE("LEVANTE_PLL_FRAC3", PLL_FRAC3, 0, 0xff, 0),                        /* 0x46 */
    /* support dynamically switch sample rate between 44.1k and 8k */
	SOC_SINGLE("LEVANTE_SAMPLE_RATE_SWITCH", SAMPLE_RATE_SWITCH, 0, 0xff, 0),                        /* 0x47 */

	SOC_SINGLE("FM2018_EN", MICCO_FM2018_EN, 0, 0xff, 0),
	SOC_SINGLE("TPA2018_EN", MICCO_TPA2018_EN, 0, 0xff, 0),
	SOC_SINGLE("FM2018_MODE", MICCO_FM2018_MODE, 0, 0xff, 0),
	
	SOC_SINGLE("Bt-Codec-switch", Bt_Codec_switch, 0, 0xff, 0),		
};

#define LEVANTE_ID_V  (67)


void audio_pa_en_control(int bflag)
{
	

	printk("enter audio_pa_en_control bflag=%d\n",bflag);

	if (gpio_request(MFP_PIN_GPIO6, "AUDIO_PA_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", MFP_PIN_GPIO6);
		return -EIO;
	}
	if (bflag)
	{
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 1);
                mdelay(1);

	}
	else
	{
		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO6), 0);
	
	}
	gpio_free(MFP_PIN_GPIO6);

}

static int levante_id_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	unsigned char value;

	value = pm860x_codec_reg_read(0x00);

	ucontrol->value.integer.value[0] = value;
	return 0;
}


static const struct snd_kcontrol_new levante_id_read =
	SOC_SINGLE_EXT("LEVANTE_ID", LEVANTE_ID_V, 0, 0xff, 0, levante_id_get, NULL);


static int levante_add_direct_access(struct snd_soc_codec *codec)
{
	int err, i;
	for (i = 0; i < ARRAY_SIZE(levante_direct_access); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&levante_direct_access[i], codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static int levante_id_access(struct snd_soc_codec *codec)
{
	int err;
	err = snd_ctl_add(codec->card, snd_soc_cnew(&levante_id_read, codec, NULL));
	if (err < 0)
		return err;

	return 0;
}


/*
 * read pm860x audio register cache
 */
static unsigned int pm860x_audio_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	struct pm860x_audio_priv *pm860x_audio = codec->private_data;

	if (reg > (ARRAY_SIZE(pm860x_audio_regs)))
		return -EIO;
	else if (reg == (ARRAY_SIZE(pm860x_audio_regs)))
		return pm860x_audio->samplerate == SNDRV_PCM_RATE_44100 ? 0 : 1;

	return cache[reg];
}


/*
 * write to the pm860x audio register space
 */
static int pm860x_audio_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	int offset;
	u8 *cache = codec->reg_cache;
	struct pm860x_audio_priv *pm860x_audio = codec->private_data;
	 int err = 0;	 
	if (reg <= PM860X_AUDIO_END)
		offset = PM860X_AUDIO_OFFSET;
	else if (reg < ARRAY_SIZE(pm860x_audio_regs))
		offset = PM860X_AUDIO_MISC_OFFSET - PM860X_AUDIO_END - 1;
	/* dynamically sample rate switch, between 44.1k and 8k */
	else if (reg == (ARRAY_SIZE(pm860x_audio_regs))) {
		/* 0 means default 44.1k, 1 means 8k for VT call */
		if (value == 0)
			pm860x_audio->samplerate = SNDRV_PCM_RATE_44100;
		else
			pm860x_audio->samplerate = SNDRV_PCM_RATE_8000;

		return 0;
	}

	else if(reg==Bt_Codec_switch)
	{
		printk("choice BT and CODEC .\n");
		printk("value is %u .\n",value);
		if((value&0x00ff)==1)			//choice BT
	        {
			printk("choice BT .\n");
			
	#if (defined CONFIG_PXA_U810 ||defined CONFIG_PXA_U802)
			err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO5), "Bt-Codec");
        		if (err) {
                		printk("failed to request GPIO5 for Bt-Codec\n");
               		 return;
        		}
        		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO5), 0);
        		
        		gpio_set_value(mfp_to_gpio(MFP_PIN_GPIO5), 1);
        		
        		gpio_free(mfp_to_gpio(MFP_PIN_GPIO5));	
	#endif
		}
		else							//choice codec
	{
			printk("choice codec .\n");
			
	#if (defined CONFIG_PXA_U810 ||defined CONFIG_PXA_U802)
	
			err = gpio_request(mfp_to_gpio(MFP_PIN_GPIO5), "Bt-Codec");
        		if (err) {
                		printk("failed to request GPIO5 for Bt-Codec\n");
               		 return;
        		}
        		gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO5), 0);
        		
        		gpio_set_value(mfp_to_gpio(MFP_PIN_GPIO5), 0);
        		
        		gpio_free(mfp_to_gpio(MFP_PIN_GPIO5));		
	#endif
		}
		return 0;

		
	}
	else if ( (reg>=MICCO_FM2018_EN) && (reg <= MICCO_FM2018_MODE) ){

	   #if 0
		  if(reg == MICCO_FM2018_EN)
	     {
	      		if((value&0x00ff)==1)
	      		{
	      			   fm2010_Power_control(1);	
			      }
			      else
			      {
				         fm2010_Power_control(0);
			      }
			      return 0;
	     }
	   #endif
	
       if(reg == MICCO_TPA2018_EN)
       {
	      		if((value&0x00ff)==1)
      		  {
      			     audio_pa_en_control(1);	
					 
                   	#ifdef CONFIG_PXA_U810					 
      			     tpa2018_wakeup() ;
			        //   tpa2018_init_client() ;		
	           	#endif
				
	        	}
		        else
		        {
			#ifdef CONFIG_PXA_U810
			     tpa2018_sleep();				       
			#endif	
			     audio_pa_en_control(0);
			         
		        }
		        return 0;
       }
	  
	  #if 0

		 if(reg == MICCO_FM2018_MODE)
		   {
			      value=value&0x00ff;
	      		fm2010_init_client(value);
			      return 0;
		   }	
	#endif

		return 0;
	}
	else
		return -EIO;

	//FIXME: if headset is plug-in, then hook detection is enabled, this case we should keep hook relative settings
	if (reg + offset == PM860X_AUDIO_OFFSET + ADC_ANALOG_PROGRAM1) {
		//EN_MICBIAS2_PAD to 0b11 always, for headset without mic, mic and hook switch detection
		value |= 0x60;
	}
	pm860x_codec_reg_write(reg + offset, value);

	cache[reg] = value;
			
	return 0;
}


/*
 * pm860x audio codec reset
 */
static int pm860x_audio_reset(struct snd_soc_codec *codec)
{
	unsigned int reg;

	for (reg = 0; reg < ARRAY_SIZE(pm860x_audio_regs); reg++) {
		if (reg != 0x2c)
			pm860x_audio_write(codec, reg, pm860x_audio_regs[reg]);
	}

	pm860x_codec_reg_write(0xda, 0x05);

	return 0;
}


/* 
 * add non dapm controls 
 */
static int pm860x_audio_add_controls(struct snd_soc_codec *codec)
{
	levante_add_direct_access(codec);
	levante_id_access(codec);
	return 0;
}


/* 
 * add dapm controls
 */
static int pm860x_audio_add_widgets(struct snd_soc_codec *codec)
{
	return 0;
}


/* set HIFI DAI configuration */
static int pm860x_aduio_hifi_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int pm860x_audio_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai __maybe_unused)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	int rate, value;
	rate =  params_rate(params);
	value = pm860x_audio_read(codec, 0x0e);
	value &= 0xf0; 
	switch(rate){
		case 48000:
			value |= 0x08;
			break;
		case 44100:
			value |= 0x07;
			break;
		case 32000:
			value |= 0x06;
			break;
		case 24000:
			value |= 0x05;
			break;
		case 22050:
			value |= 0x04;
			break;
		case 16000:
			value |= 0x03;
			break;
		case 12000:
			value |= 0x02;
			break;
		case 11025:
			value |= 0x01;
			break;
		case 8000:
			value |= 0x00;
			break;
		default:
			printk(KERN_ERR "unsupported rate\n");
			return -EINVAL;
			break;
	}
	/*pm860x_audio_write(codec, 0x0e, value); pm860x has sample rate switching issue*/

	return 0;
}

static int pm860x_audio_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int pm860x_audio_set_bias_level(struct snd_soc_codec *codec,  enum snd_soc_bias_level level)
{
	switch (level) {
	        case SND_SOC_BIAS_ON: /* full On */
			break;

		case SND_SOC_BIAS_PREPARE: /* partial On */
			break;

		case SND_SOC_BIAS_STANDBY: /* partial On */
			break;

		case SND_SOC_BIAS_OFF: /* Off, without power */
			break;
	}

	codec->bias_level = level;

	return 0;
}

#define PM860X_AUDIO_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		                SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define PM860X_AUDIO_HIFI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

/* initiate playback and capture sample rate to pm860x audio sample_rate */
static int pm860x_audio_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct pm860x_audio_priv *pm860x_audio = codec->private_data;

	dai->playback.rates = dai->capture.rates = pm860x_audio->samplerate;

	return 0;
}

static struct snd_soc_dai_ops pm860x_dai_ops = {
	.startup = pm860x_audio_startup,
	.hw_params = pm860x_audio_hifi_hw_params,
	.digital_mute = pm860x_audio_mute,
	.set_fmt = pm860x_aduio_hifi_set_dai_fmt,
	.set_clkdiv = NULL,
	.set_pll = NULL,
	.set_sysclk = NULL,
};

/*
 * HIFI DAI
 */
struct snd_soc_dai pm860x_audio_dai[]={
/* DAI HIFI mode*/
	{
		.name = "pm860x audio HiFi",
		.id = 1,
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = PM860X_AUDIO_HIFI_RATES,
			.formats = PM860X_AUDIO_HIFI_FORMATS,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = PM860X_AUDIO_HIFI_RATES,
			.formats = PM860X_AUDIO_HIFI_FORMATS,
		},
		.ops = &pm860x_dai_ops,
	},

	{
		.name = "pm860x audio pcm",
		.id = 1,
		.playback = {
			.stream_name = "Pcm Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = PM860X_AUDIO_HIFI_RATES,
			.formats = PM860X_AUDIO_HIFI_FORMATS,
		},
		.capture = {
			.stream_name = "Pcm Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = PM860X_AUDIO_HIFI_RATES,
			.formats = PM860X_AUDIO_HIFI_FORMATS,
		},
		.ops = &pm860x_dai_ops,
	},
};

static void pm860x_audio_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
			container_of(work, struct snd_soc_codec, delayed_work.work);
	pm860x_audio_set_bias_level(codec, codec->bias_level);
}


static int pm860x_audio_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 *cache = codec->reg_cache;

	//make sure that we are not using PCM interface, consider the case if we suspend during a voice call
	if ((cache[ADC_ENABLES2] & (1 << 0)) == 0) {
		//write these two registers to reset all the audio registers, need to observe whether will get pop and click
		pm860x_codec_reg_write(0x42, 0x10);
		pm860x_codec_reg_write(0x42, 0x00);
		pm860x_codec_reg_write(0xdc, 0x00);
	}

	return 0;
}


static int pm860x_audio_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 *cache = codec->reg_cache;
	int i;

	//if no audio path need to be enabled or we did not really suspend since we are in a voice call
	if ((cache[MISC2] & (1 << 5)) && (cache[MISC2] & (1 << 3)) && ((cache[ADC_ENABLES2] & (1 << 0)) == 0)) {
		//set some registers first, to avoid pop and click
		pm860x_codec_reg_write(PM860X_AUDIO_MISC_OFFSET, cache[MISC2]);
		//The audio I2C registers (register addresses 0x0 to 0xEE) are accessible for read or write activity 160 usec after AUDIO_PU set to '1'.
		msleep(1);
		//Turn On regulator 2.7 V, Turn On charge pump, Audio is Active
		pm860x_codec_reg_write(AUDIO_SUPPLIES2 + PM860X_AUDIO_OFFSET, cache[AUDIO_SUPPLIES2]);
		pm860x_codec_reg_write(PM860X_AUDIO_MISC_OFFSET + 1, cache[PLL_CTRL1]);

		if (cache[DAC_ENABLES2] & (1 << 3)) {
			//Enable DAC always for avoid noise
			pm860x_codec_reg_write(DAC_ENABLES2 + PM860X_AUDIO_OFFSET, 0x38);
		}
		pm860x_codec_reg_write(EAR_SPKR_CTRL2 + PM860X_AUDIO_OFFSET, cache[EAR_SPKR_CTRL2] & (~(1 << 2)));
		pm860x_codec_reg_write(EAR_SPKR_CTRL2 + PM860X_AUDIO_OFFSET, cache[EAR_SPKR_CTRL2] | (1 << 2));

		//restore the registers
		for (i = 0; i <= PM860X_AUDIO_END; i++) {
			if (i != AUDIO_SUPPLIES2)
				pm860x_codec_reg_write(i + PM860X_AUDIO_OFFSET, cache[i]);
		}
		//apply changes
		pm860x_codec_reg_write(EAR_SPKR_CTRL2 + PM860X_AUDIO_OFFSET, cache[EAR_SPKR_CTRL2] & (~(1 << 2)));
		pm860x_codec_reg_write(EAR_SPKR_CTRL2 + PM860X_AUDIO_OFFSET, cache[EAR_SPKR_CTRL2] | (1 << 2));
	}

	return 0;
}


static int pm860x_audio_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->card->codec;
	int ret = 0;

	codec->name = "pm860x audio";
	codec->owner = THIS_MODULE;
	codec->read = pm860x_audio_read;
	codec->write = pm860x_audio_write;
	codec->set_bias_level = pm860x_audio_set_bias_level;
	codec->dai = pm860x_audio_dai;
	codec->num_dai = ARRAY_SIZE(pm860x_audio_dai);
	codec->reg_cache_size = sizeof(pm860x_audio_regs);
	codec->reg_cache = kmemdup(pm860x_audio_regs, sizeof(pm860x_audio_regs), GFP_KERNEL);

	if(codec->reg_cache == NULL)
		return -ENOMEM;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "pm860x audio: failed to create pcms\n");
		goto pcm_err;
	}

	codec->bias_level = SND_SOC_BIAS_STANDBY;
	schedule_delayed_work(&codec->delayed_work,msecs_to_jiffies(2));

	pm860x_audio_reset(codec);

	pm860x_audio_add_controls(codec);
	pm860x_audio_add_widgets(codec);
	
	ret = snd_soc_init_card(socdev);
	if (ret < 0)
	{
		printk(KERN_ERR "pm860x audio: failed to register card\n");
		goto card_err;
	}

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}


static ssize_t Levante_proc_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	char			*buf = page;
	char			*next = buf;
	unsigned		size = count;
	int 			t;
	int 			i;
	int 			reg;
	struct snd_soc_codec 	*codec = data;

	t = scnprintf(next, size, "Micco regs: \n");
	size -= t;
	next += t;

	for (i = 0; i < ARRAY_SIZE(pm860x_audio_regs); i++) {
		reg = pm860x_audio_read(codec, i);
		t = scnprintf(next, size, "[0x%02x]=0x%02x  \n", i, reg);
		size -= t;
		next += t;
	}

	*eof = 1;
	return count - size;
}

static int Levante_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[4096];
	char *buf = kbuf;
	struct snd_soc_codec 	*codec = data;
	unsigned int	i, reg, reg2;
	char cmd;

	if (count >= 4096)
		return -EINVAL;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;
	sscanf(buf, "%c 0x%x 0x%x", &cmd, &i, &reg);

	if ('r' == cmd) {
		if (i > 66 || (i & 1)) {
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		reg = pm860x_audio_read(codec, i);
		printk(KERN_INFO "0x[%2x]=0x%2x\n", i, reg);
	} else if ('w' == cmd) {
		if (i > 0x16) {
			printk(KERN_ERR "invalid index!\n");
			goto error;
		}
		if (reg > 0xff) {
			printk(KERN_ERR "invalid value!\n");
			goto error;
		}
		pm860x_audio_write(codec, i, reg);
		reg2 = pm860x_audio_read(codec, i);
		printk(KERN_INFO
			"write 0x%2x to 0x[%2x], read back 0x%2x\n",
			reg, i, reg2);
	} else {
		printk(KERN_ERR "unknow opt!\n");
		goto error;
	}

	return count;
error:
	printk(KERN_INFO "r/w index(0x%%2x) value(0x%%2x)\n");
	return count;
}


static int pm860x_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct _setup_data *setup;
	struct snd_soc_codec *codec;
	struct pm860x_audio_priv *pm860x_audio;

	struct proc_dir_entry *Levante_proc_entry; 
	
	int ret = 0;

	printk(KERN_INFO "pm860x Audio Codec %s\n", PM860X_AUDIO_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	pm860x_audio = kzalloc(sizeof(struct pm860x_audio_priv), GFP_KERNEL);
	if (pm860x_audio == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
	/* initiate pm860x audio sample rate to default sample rate */
	pm860x_audio->samplerate = SNDRV_PCM_RATE_44100;

	codec->private_data = pm860x_audio;
	socdev->card->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	INIT_DELAYED_WORK(&codec->delayed_work, pm860x_audio_work);

	pm860x_audio_init(socdev);

	Levante_proc_entry = create_proc_entry("driver/codec", 0, NULL);
	if (Levante_proc_entry) {
		Levante_proc_entry->data = codec;
		Levante_proc_entry->read_proc = Levante_proc_read;
		Levante_proc_entry->write_proc = Levante_proc_write;
	}
	return ret;
}


/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int pm860x_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		pm860x_audio_set_bias_level(codec, SND_SOC_BIAS_OFF);

	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	
	kfree(codec->private_data);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_pm860x_audio = {
	.probe = 	pm860x_audio_probe,
	.remove = 	pm860x_audio_remove,
	.suspend = 	pm860x_audio_suspend,
	.resume =	pm860x_audio_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_pm860x_audio);

static int __init pm860x_audio_modinit(void)
{
	return snd_soc_register_dais(ARRAY_AND_SIZE(pm860x_audio_dai));
}
module_init(pm860x_audio_modinit);

static void __exit pm860x_audio_exit(void)
{
	snd_soc_unregister_dais(ARRAY_AND_SIZE(pm860x_audio_dai));
}
module_exit(pm860x_audio_exit);

MODULE_DESCRIPTION("ASoC pm860x audio driver");
MODULE_AUTHOR("xjian@marvell.com");
MODULE_LICENSE("GPL");

