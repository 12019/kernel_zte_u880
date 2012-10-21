#ifndef _PXA910_AEC_FM2010_H
#define _PXA910_AEC_FM2010_H

#define AEC_DEV_FILE "aec"

enum aec_mode{
	AEC_NORMAL_MODE,
	AEC_BYPASS_MODE,
	AEC_NORMAL_HANDHELD_MODE,
	AEC_MODE_MAX
};

typedef struct aec_reg{
	unsigned short reg;
	unsigned short val;
}aec_reg_t;

typedef struct tpa_reg{
	char reg;
	char val;
}tpa_reg_t;

struct aec_platform_data{
	int (*init)(void);
	int (*sleep)(void);
	int (*wakeup)(void);		
	int (*reset)(void);   
	int (*set_mode)(int mode);
	const aec_reg_t * (*parmGet)(int mode);
};

extern void pxa3xx_set_aec_mode(int mode);



typedef struct
{
    unsigned short size;
    aec_reg_t *pReg;
}S_AEC_REG_CMD_PARM;

#define AEC_IOC_MAGIC  0xC1
#define AEC_IOC_SET_REGISTER			_IOWR(AEC_IOC_MAGIC, 1, int)
#define AEC_IOC_CMD_TEST			    _IOWR(AEC_IOC_MAGIC, 2, int)
#define AEC_IOC_SET_MODE			    _IOWR(AEC_IOC_MAGIC, 3, int)
#define AEC_IOC_RESET			        _IOWR(AEC_IOC_MAGIC, 4, int)
#define AEC_IOC_SLEEP			        _IOWR(AEC_IOC_MAGIC, 5, int)

int aec_mem_write(unsigned short mem_addr, unsigned short data);
int aec_mem_read(unsigned short mem_addr, unsigned short *data);
int fm2010_init_client(unsigned int FM2010_MODE);	
void fm2010_Power_control(int bflag);
#endif

