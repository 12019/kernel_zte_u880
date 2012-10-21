#include <linux/types.h>

#include <linux/device.h>

#define WHO_AM_I    0x0F
#define Ctrl_Reg1   0x20
#define Ctrl_Reg2   0x21
#define Ctrl_Reg3   0x22
#define Status_Reg  0x27
#define OutX    0x29
#define OutY    0x2B
#define OutZ    0x2D
#define FF_WU_CFG   0x30
#define FF_WU_SRC   0x31
#define FF_WU_THS   0x32
#define FF_WU_DURATION  0x33

// ioctl command
#define IO_NOT_READY	0
#define IO_SIG_HANDLE	0x6E
#define IO_PARAM_RESET	0x6F

// write
#define W_Ctrl_Reg1	0x70
#define W_Ctrl_Reg2	0x71
#define W_Ctrl_Reg3	0x72

#define W_FF_WU_CFG   0x73
#define W_FF_WU_THS   0x74
#define W_FF_WU_DURATION  0x75

//click
#define CLICK_CFG 		0x38
#define CLICK_SRC		0x39
#define CLICK_THS_Y_X 	0x3b
#define CLICK_THS_Z		0x3c
#define CLICK_TIMELIMIT 0x3d
#define CLICK_LANTENCY 0x3e
#define CLICK_WINDOW   0x3f

//register configuration value
#define XYZ_HIE			(42)
#define XYZ_LIE			(21)
#define THRESH_OLD		(10)
#define THRESH_DCRM		(0x80)
#define DURATION		(16)
#define THRESH_OLD_G		(25)
#define DATE_RATE_400HZ	        (1<<7)
#define LIS302DL_MAX_INTERVAL		2000
#define LIS302DL_MIN_INTERVAL  	 	100	

#define LIS302DL_SINGLE_CLICK		(0x15)
#define LIS302DL_DBOUBLE_CLICK		(0x2a)
#define LIS302DL_CLICK_THS_YX 		(0x11) //x= y = 0.5g
#define LIS302DL_CLICK_THS_Z		(0x1)//z=0.5g

#define LIS302DL_FF_WU_1		1
#define LIS302DL_FF_WU_2 		2	
#define LIS302DL_FF_WU_1_2		3
#define LIS302DL_DATA_READY		4
#define LIS302DL_CLICK_INTERRUPT	7

#define INTERRUPT_TYPE_MASK 		0x7
#define INTERRUPT_FF_WU			1
#define INTERRUPT_DATA_READY		4
#define INTERRUPT_CLICK			7

//#define LIS302_POWER_UP			(1 << 6)
#define LIS302_POWER_UP			0x47
#define LIS302_POWER_OFF		(~(1<<6))
#define LIS302_LIS302DL			0x3B
#define HP_FF_WU1_ENABLE		(1<<2)

#define MG_PER_SAMPLE			18
#define SWITCH_ON			1
#define SWITCH_OFF			0
#define LIS302_DEBUG_FLOOD		1985

#define X_POS				(1<<0)
#define Y_POS				(1<<1)
#define Z_POS				(1<<2)
#define SWITCH_XY			(1<<3)
#define XYZ_DIR_MASK 			0xF

/*LIS302 multi bytes read enable bit*/
#define MSB_ENABLE			(1 << 7)	
struct lis3lv02d_struct
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
lis302_status_t;

typedef struct 
{
	char 					*name;
	struct i2c_client		*lis302_client;
	lis302_status_t  		*status_p;
	struct sensors_dev 		*device;
	struct i2c_driver		*driver;
	atomic_t   				rf_count;
	struct  list_head  		open_list;
	wait_queue_head_t		lis302_wait;
	struct work_struct		lis302_work;
	unsigned int			switch_on;
	struct semaphore		mutex;
	spinlock_t				list_lock;

	struct timer_list		timer;
	void 					*private;
}
lis302dl_device_t;

struct gsensor_platform_data 
{
	int irq;
//	int adc_event;
	int (*init)(void);
//	int (*get_gpio_value)(void);
//	void (*setpower)(int enable);
//	int (*chip_verify)(int *, int);
};


