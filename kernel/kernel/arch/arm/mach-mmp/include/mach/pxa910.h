#ifndef __ASM_MACH_PXA910_H
#define __ASM_MACH_PXA910_H

#include <linux/i2c.h>
#include <mach/devices.h>
#include <mach/pxa910fb.h>
#include <mach/mmc.h>
#include <plat/i2c.h>
#include <plat/pxa3xx_nand.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa27x_keypad.h>
#include <mach/pxa2xx_spi.h>

extern struct pxa_device_desc pxa910_device_uart1;
extern struct pxa_device_desc pxa910_device_uart2;
extern struct pxa_device_desc pxa910_device_uart3;
extern struct pxa_device_desc pxa910_device_twsi0;
extern struct pxa_device_desc pxa910_device_twsi1;
extern struct pxa_device_desc pxa910_device_pwm1;
extern struct pxa_device_desc pxa910_device_pwm2;
extern struct pxa_device_desc pxa910_device_pwm3;
extern struct pxa_device_desc pxa910_device_pwm4;
extern struct pxa_device_desc pxa910_device_nand;
extern struct platform_device pxa910_device_acipc;
extern struct pxa_device_desc pxa910_device_ire;
extern struct pxa_device_desc pxa910_device_ssp0;
extern struct pxa_device_desc pxa910_device_ssp1;
extern struct pxa_device_desc pxa910_device_ssp2;
extern struct platform_device pxa910_device_imm;
extern struct platform_device pxa910_device_rtc;
extern struct platform_device pxa910_device_1wire;
extern struct pxa_device_desc pxa910_device_fb;
extern struct pxa_device_desc pxa910_device_fb_ovly;
extern struct pxa_device_desc pxa910_device_keypad;
extern struct pxa_device_desc pxa910_device_cnm;
extern struct pxa_device_desc pxa910_device_camera;
extern struct platform_device pxa910_device_freq;
extern struct platform_device pxa910_device_u2o;
extern struct platform_device pxa910_device_u2h;
extern struct platform_device pxa910_device_u2ootg;
extern struct platform_device pxa910_device_u2oehci;

int pxa910_init_usb_phy(unsigned int base);
int pxa910_deinit_usb_phy(unsigned int base);
extern u32 get_pll2_freq(void);
extern void gc_aclk_fc(void);


static inline int pxa910_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &pxa910_device_uart1; break;
	case 2: d = &pxa910_device_uart2; break;
	case 3: d = &pxa910_device_uart3; break;
	}

	if (d == NULL)
		return -EINVAL;

	return pxa_register_device(d, NULL, 0);
}

static inline void pxa910_add_acipc(void)
{
	int ret;
	ret = platform_device_register(&pxa910_device_acipc);
	if (ret)
		dev_err(&pxa910_device_acipc.dev,
			"unable to register device: %d\n", ret);
}

static inline int pxa910_add_ire(void)
{
	return pxa_register_device(&pxa910_device_ire, NULL, 0);
}

static inline int pxa910_add_ssp(int id)
{
        struct pxa_device_desc *d = NULL;

        switch (id) {
	        case 0: d = &pxa910_device_ssp0; break;
	        case 1: d = &pxa910_device_ssp1; break;
	        case 2: d = &pxa910_device_ssp2; break;
	        default:
	                return -EINVAL;
	        }

        return pxa_register_device(d, NULL, 0);
}

static inline void pxa910_add_imm(void)
{
        int ret;
        ret = platform_device_register(&pxa910_device_imm);
	if (ret)
		dev_err(&pxa910_device_imm.dev,
			"unable to register device: %d\n", ret);
}

static inline void pxa910_add_rtc(void)
{
	int ret;
	ret = platform_device_register(&pxa910_device_rtc);
	if (ret)
		dev_err(&pxa910_device_rtc.dev,
			"unable to register device: %d\n", ret);
}

static inline void pxa910_add_1wire(void)
{
	int ret;
	ret = platform_device_register(&pxa910_device_1wire);
	if (ret)
		dev_err(&pxa910_device_1wire.dev,
			"unable to register device: %d\n", ret);
}

static inline int pxa910_add_twsi(int id, struct i2c_pxa_platform_data *data,
				  struct i2c_board_info *info, unsigned size)
{
	struct pxa_device_desc *d = NULL;
	int ret;

	switch (id) {
	case 0: d = &pxa910_device_twsi0; break;
	case 1: d = &pxa910_device_twsi1; break;
	default:
		return -EINVAL;
	}

	ret = i2c_register_board_info(id, info, size);
	if (ret)
		return ret;

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int pxa910_add_pwm(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &pxa910_device_pwm1; break;
	case 2: d = &pxa910_device_pwm2; break;
	case 3: d = &pxa910_device_pwm3; break;
	case 4: d = &pxa910_device_pwm4; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int pxa910_add_nand(struct pxa3xx_nand_platform_data *info)
{
	return pxa_register_device(&pxa910_device_nand, info, sizeof(*info));
}

static inline int pxa910_add_fb(struct pxa910fb_mach_info *mi)
{
	return pxa_register_device(&pxa910_device_fb, mi, sizeof(*mi));
}

static inline int pxa910_add_fb_ovly(struct pxa910fb_mach_info *mi)
{
	return pxa_register_device(&pxa910_device_fb_ovly, mi, sizeof(*mi));
}

static inline int pxa910_add_u2o(void *data)
{
	pxa910_device_u2o.dev.platform_data = data;
	return platform_device_register(&pxa910_device_u2o);
}

static inline int pxa910_add_u2h(struct pxa_usb_plat_info *info)
{
	pxa910_device_u2h.dev.platform_data = info;
	return platform_device_register(&pxa910_device_u2h);
}

static inline int pxa910_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	return pxa_register_device(&pxa910_device_keypad, data, sizeof(*data));
}

static inline int pxa910_add_cnm(void)
{
	return pxa_register_device(&pxa910_device_cnm, NULL, 0);
}

static inline int pxa910_add_cam(void)
{
	return pxa_register_device(&pxa910_device_camera, NULL, 0);
}

static inline int pxa910_add_sdh(int id, struct pxasdh_platform_data *data)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0: d = &pxa910_device_sdh0; break;
	case 1: d = &pxa910_device_sdh1; break;
	case 2: d = &pxa910_device_sdh2; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, data, sizeof(*data));
}

static inline void pxa910_add_freq(void)
{
        int ret;
        ret = platform_device_register(&pxa910_device_freq);
	if (ret)
		dev_err(&pxa910_device_freq.dev,
			"unable to register device: %d\n", ret);
}
static inline int pxa910_add_u2ootg(struct pxa_usb_plat_info *info)
{
	pxa910_device_u2ootg.dev.platform_data = info;
	return platform_device_register(&pxa910_device_u2ootg);
}

static inline int pxa910_add_u2oehci(struct pxa_usb_plat_info *info)
{
	pxa910_device_u2oehci.dev.platform_data = info;
	return platform_device_register(&pxa910_device_u2oehci);
}

static inline int pxa910_add_spi(int id, struct pxa2xx_spi_master *pdata)
{
	struct platform_device *pd;

	pd = platform_device_alloc("pxa2xx-spi", id);
	if (pd == NULL) {
		pr_err("pxa2xx-spi: failed to allocate device (id=%d)\n", id);
		return -ENOMEM;
	}

	platform_device_add_data(pd, pdata, sizeof(*pdata));

	return platform_device_add(pd);
}

#endif /* __ASM_MACH_PXA910_H */
