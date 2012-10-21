#include <linux/types.h>

#include <linux/device.h>	

/***** read only registers *****/
#define	ST_REG		0xC0
#define TMPS_REG	0xC1
#define H1X_REG		0xC2
#define H1Y_REG		0xC3
#define H1Z_REG		0xC4

/***** read and write registers *****/
#define MS1_REG		0xE0
#define HXDA_REG	0xE1
#define HYDA_REG	0xE2
#define HZDA_REG	0xE3
#define HXGA_REG	0xE4
#define HYGA_REG	0xE5
#define HZGA_REG	0xE6
#define TS1_REG		0x5D

/***** default value of registers *****/
//bit1 & bit0 of MS1_REG
#define SENSOR_MEASUREMENT_MODE		0x00
#define SETTING_PROHIBITED			0x01
#define EEPROM_ACCESS_MODE			0x02
#define POWER_DOWN_MODE				0x03
#define EEPROM_WRITE_MODE			0xA8

#define SWITCH_ON	1
#define SWITCH_OFF	0

#define INT_HIGH	1
#define INT_LOW		0


struct ak8973_struct
{
	int					(*init_irq)(void);
	int					(*ack_irq)(void);
	void				(*platform_init)(void);
	int					(*get_gpio_value)(void);
	struct power_chip	*power_chips;
	int					xyz_dir;
	int					irq;
};

typedef struct
{
	atomic_t g_rdy;
	struct list_head  list;
}
ak8973_status_t;

typedef struct 
{
	char 					*name;
//	struct i2c_client		*lis302_client;
	ak8973_status_t  		*status_p;
	struct sensors_dev 		*device;
	struct i2c_driver		*driver;
	atomic_t   				rf_count;
	struct list_head  		open_list;
	wait_queue_head_t		ak8973_wait;
	struct work_struct		ak8973_work;
	unsigned int			switch_on;
	struct semaphore		mutex;
	spinlock_t				list_lock;
	struct timer_list		timer;
	void 					*private;
}
ak8973_device_t;

struct compass_platform_data 
{
	int irq;
//	int adc_event;
	int (*init)(void);
//	int (*get_gpio_value)(void);
//	void (*setpower)(int enable);
//	int (*chip_verify)(int *, int);
};

