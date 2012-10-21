/*
 * PXA910 Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __PXA910_PM_H__
#define __PXA910_PM_H__

#define ZTE_HWVERSION1  1
#define ZTE_HWVERSION2  2
#define ZTE_HWVERSION3  3
#define ZTE_HWVERSION4  4

extern void pxa910_pm_enter_lowpower_mode(int state);

struct pxa910_peripheral_config_ops{
	int	(*pin_lpm_config)(void);
	int	(*pin_restore)(void);
};

extern int pxa910_power_config_register(struct pxa910_peripheral_config_ops *ops);

extern void pxa910_suspend_exit_set(void);
extern void pxa910_suspend_exit_clr(void);
extern int pxa910_suspend_exit_status(void);
extern int pm860x_get_boardID(void);
extern void pm860x_get_boardID_fromboot(void);

#endif
