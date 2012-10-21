/*
 * linux/sound/soc/pxa/pxa2xx-ac97.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PXA910_TPA2018_H
#define _PXA910_TPA2018_H

int tpa2018_init(void);

int tpa2018_sleep(void);//disabel the SPK_EN

int tpa2018_wakeup(void) ;//enable the SPK_EN

int tpa2018_reset(void);

int tpa2018_set_mode(int mode);

static int tpa_suspend(struct i2c_client * tpa, pm_message_t state);

static int tpa_resume(struct i2c_client * tpa);


#endif

