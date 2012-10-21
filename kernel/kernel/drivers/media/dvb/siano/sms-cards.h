

#ifndef __SMS_CARDS_H__
#define __SMS_CARDS_H__

#include <linux/usb.h>
#include "smscoreapi.h"

#define SMS_BOARD_UNKNOWN 0
#define SMS1XXX_BOARD_SIANO_STELLAR 1
#define SMS1XXX_BOARD_SIANO_NOVA_A  2
#define SMS1XXX_BOARD_SIANO_NOVA_B  3
#define SMS1XXX_BOARD_SIANO_VEGA    4
#define SMS1XXX_BOARD_HAUPPAUGE_CATAMOUNT 5
#define SMS1XXX_BOARD_HAUPPAUGE_OKEMO_A 6
#define SMS1XXX_BOARD_HAUPPAUGE_OKEMO_B 7
#define SMS1XXX_BOARD_HAUPPAUGE_WINDHAM 8
#define SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD 9
#define SMS1XXX_BOARD_HAUPPAUGE_TIGER_MINICARD_R2 10
#define SMS1XXX_BOARD_SIANO_NICE	11
#define SMS1XXX_BOARD_SIANO_VENICE	12

struct sms_board_gpio_cfg {
	int foreign_lna0_ctrl;
	int foreign_lna1_ctrl;
	int foreign_lna2_ctrl;
	int lna_vhf_exist;
	int lna_vhf_ctrl;
	int lna_uhf_exist;
	int lna_uhf_ctrl;
	int lna_uhf_d_ctrl;
	int lna_sband_exist;
	int lna_sband_ctrl;
	int lna_sband_d_ctrl;
	int leds_power;
	int led0;
	int led1;
	int led2;
	int led3;
	int led4;
	int ir;
	int eeprom_wp;
	int mrc_sense;
	int mrc_pdn_resetn;
	int mrc_gp0; 
	int mrc_gp1;
	int mrc_gp2;
	int mrc_gp3;
	int mrc_gp4;
	int host_spi_gsp_ts_int;
};

struct sms_board {
	enum sms_device_type_st type;
	char *name, *fw[DEVICE_MODE_MAX];
	struct sms_board_gpio_cfg board_cfg;
};

struct sms_board *sms_get_board(int id);

extern struct usb_device_id smsusb_id_table[];
extern struct smscore_device_t *coredev;

enum SMS_BOARD_EVENTS {
	BOARD_EVENT_POWER_INIT,
	BOARD_EVENT_POWER_SUSPEND,
	BOARD_EVENT_POWER_RESUME,
	BOARD_EVENT_BIND,
	BOARD_EVENT_SCAN_PROG,
	BOARD_EVENT_SCAN_COMP,
	BOARD_EVENT_EMERGENCY_WARNING_SIGNAL,
	BOARD_EVENT_FE_LOCK,
	BOARD_EVENT_FE_UNLOCK,
	BOARD_EVENT_DEMOD_LOCK,
	BOARD_EVENT_DEMOD_UNLOCK,
	BOARD_EVENT_RECEPTION_MAX_4,
	BOARD_EVENT_RECEPTION_3,
	BOARD_EVENT_RECEPTION_2,
	BOARD_EVENT_RECEPTION_1,
	BOARD_EVENT_RECEPTION_LOST_0,
	BOARD_EVENT_MULTIPLEX_OK,
	BOARD_EVENT_MULTIPLEX_ERRORS
};

int sms_board_event(struct smscore_device_t *coredev,
		enum SMS_BOARD_EVENTS gevent);

#endif 
