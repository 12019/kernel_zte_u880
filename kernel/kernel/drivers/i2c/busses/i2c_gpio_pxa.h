#ifndef _I2C_GPIO_PXA_H
#define _I2C_GPIO_PXA_H

#define ack 0
#define no_ack 1

#define I2C_WR	0x0
#define I2C_RD    0x1

#define COMMON_I2C_OK   0
#define COMMON_I2C_ERROR  0xff

#define HIGH 1
#define LOW	0

#define SDA_GPIO_NUM 19
#define SCL_GPIO_NUM 18

void COMMON_I2C_Reset(void);
void COMMON_I2C_Start(void);
void COMMON_I2C_Restart(void);
void COMMON_I2C_Stop(void);
char COMMON_I2C_Send_Byte(char cByteToSend);
char COMMON_I2C_Receive_Byte(char ackn);
void COMMON_I2C_mutx_down(void);
void COMMON_I2C_mutx_up(void);
void COMMON_I2C_Init(void);
void COMMON_I2C_Exit(void);


#endif /* _Z_UTIL_H */
