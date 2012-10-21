#ifndef ONBOARD_H_
#define ONBOARD_H_


void __init dkb_add_lcd_tpo(void);
void __init dkb_add_lcd_sharp(void);
#ifdef CONFIG_PXA_U802
void __init u810_add_lcd_ili9325(void);
#elif defined CONFIG_PXA_U810
void __init u810_add_lcd_lead(void);
#elif defined CONFIG_PXA_U880
void __init u880_add_lcd(void);
#elif defined CONFIG_PXA_U812
void __init u810_add_lcd_lead(void);
#elif defined CONFIG_PXA_U830
void __init u810_add_lcd_lead(void);
#endif
void __init pm860x_set_vibrator(unsigned char value);
#endif /* ONBOARD_H_ */
