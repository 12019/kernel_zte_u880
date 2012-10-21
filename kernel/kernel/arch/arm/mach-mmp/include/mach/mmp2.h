#ifndef __ASM_MACH_MMP2_H
#define __ASM_MACH_MMP2_H

#include <linux/i2c.h>
#include <mach/devices.h>
#include <plat/i2c.h>
#include <plat/pxa3xx_nand.h>
#include <mach/pxa168fb.h>
#include <plat/pxa27x_keypad.h>

extern struct pxa_device_desc mmp2_device_uart1;
extern struct pxa_device_desc mmp2_device_uart2;
extern struct pxa_device_desc mmp2_device_uart3;
extern struct pxa_device_desc mmp2_device_uart4;
extern struct pxa_device_desc mmp2_device_twsi1;
extern struct pxa_device_desc mmp2_device_twsi2;
extern struct pxa_device_desc mmp2_device_twsi3;
extern struct pxa_device_desc mmp2_device_twsi4;
extern struct pxa_device_desc mmp2_device_twsi5;
extern struct pxa_device_desc mmp2_device_twsi6;
extern struct pxa_device_desc mmp2_device_nand;
extern struct pxa_device_desc mmp2_device_fb;
extern struct pxa_device_desc mmp2_device_fb_tv;
extern struct pxa_device_desc mmp2_device_fb_ovly;
extern struct pxa_device_desc mmp2_device_fb_tv_ovly;
extern struct platform_device mmp2_device_imm;
extern struct pxa_device_desc mmp2_device_hdmi;
extern struct pxa_device_desc mmp2_device_keypad;
extern struct platform_device mmp2_device_rtc;

static inline int mmp2_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &mmp2_device_uart1; break;
	case 2: d = &mmp2_device_uart2; break;
	case 3: d = &mmp2_device_uart3; break;
	case 4: d = &mmp2_device_uart4; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int mmp2_add_twsi(int id, struct i2c_pxa_platform_data *data,
				  struct i2c_board_info *info, unsigned size)
{
	struct pxa_device_desc *d = NULL;
	int ret;

	switch (id) {
	case 0: d = &mmp2_device_twsi1; break;
	case 1: d = &mmp2_device_twsi2; break;
	case 2: d = &mmp2_device_twsi3; break;
	case 3: d = &mmp2_device_twsi4; break;
	case 4: d = &mmp2_device_twsi5; break;
	case 5: d = &mmp2_device_twsi6; break;
	default:
		return -EINVAL;
	}

	ret = i2c_register_board_info(id, info, size);
	if (ret)
		return ret;

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int mmp2_add_nand(struct pxa3xx_nand_platform_data *data)
{
	return pxa_register_device(&mmp2_device_nand, data, sizeof(*data));
}

static inline int mmp2_add_fb(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp2_device_fb, mi, sizeof(*mi));
}

static inline int mmp2_add_fb_tv(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp2_device_fb_tv, mi, sizeof(*mi));
}

static inline int mmp2_add_fb_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp2_device_fb_ovly, mi, sizeof(*mi));
}

static inline int mmp2_add_fb_tv_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&mmp2_device_fb_tv_ovly, mi, sizeof(*mi));
}

static inline void mmp2_add_imm(void)
{
	int ret;
	ret = platform_device_register(&mmp2_device_imm);
	if (ret)
		dev_err(&mmp2_device_imm.dev,
				"unable to register device: %d\n", ret);
}

static inline int mmp2_add_hdmi(void)
{
        return pxa_register_device(&mmp2_device_hdmi, NULL, 0);
}

static inline int mmp2_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	return pxa_register_device(&mmp2_device_keypad, data, sizeof(*data));
}

static inline void mmp2_add_rtc(void)
{
	int ret;
	ret = platform_device_register(&mmp2_device_rtc);
	if (ret)
		dev_err(&mmp2_device_rtc.dev,
			"unable to register device: %d\n", ret);
}
#endif /* __ASM_MACH_MMP2_H */

