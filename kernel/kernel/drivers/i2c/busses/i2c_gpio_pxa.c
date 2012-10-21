/*******************************************************************************
*
*
********************************************************************************
*     
*   gpio for virtual i2c bus 
*
*   used by rtc and tos tsl 2771
*             
*
*******************************************************************************/
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>


#include <asm/uaccess.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <mach/gpio.h>

#include <asm/signal.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/completion.h>
//#include <linux/sensors.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/input.h>

#include <linux/platform_device.h>
#include <linux/list.h>
#include <asm/atomic.h>

#include "./i2c_gpio_pxa.h"

#define SDA_H {__gpio_set_value(SDA_GPIO_NUM, HIGH);}
#define SDA_L {__gpio_set_value(SDA_GPIO_NUM, LOW);}

#define SCL_H {__gpio_set_value(SCL_GPIO_NUM, HIGH);}
#define SCL_L {__gpio_set_value(SCL_GPIO_NUM, LOW);}

#define SDA_OUTPUT_MODE	 {gpio_direction_output(SDA_GPIO_NUM,HIGH);}
#define SCL_OUTPUT_MODE	 {gpio_direction_output(SCL_GPIO_NUM,HIGH);}
#define SDA_INPUT_MODE	 {gpio_direction_input(SDA_GPIO_NUM);}
#define SCL_INPUT_MODE	 {gpio_direction_input(SCL_GPIO_NUM);}

#define SDA_INPUT_DATA(SDA_DATA)   {SDA_DATA =__gpio_get_value(SDA_GPIO_NUM);}

#define  I2C_DELAY(DURATION)     \
{ \
    volatile unsigned short i;      \
    for(i = 0; i <= DURATION; i++)  \
    {}                              \
}

static struct semaphore i2c_gpio_pxa_list;



/*****************************************************************************
*   ������      �� void COMMON_I2C_Reset(void)
*   ����        �� I2C��λ�������ߺ�ʱ��������
*   �������    �� void
*   �������    �� none
*   ����ֵ˵��  �� void
*****************************************************************************/
void COMMON_I2C_Reset(void)
{
    SDA_OUTPUT_MODE;
    SCL_OUTPUT_MODE;
   
    I2C_DELAY(200);
    SCL_H;
    I2C_DELAY(200);
    SDA_H;
    I2C_DELAY(1000);

}

/*****************************************************************************
*   ������      �� void COMMON_I2C_Start(void)
*   ����        ��    ��ʼλ��־
*   �������    �� void
*   �������    �� none
*   ����ֵ˵��  �� void
*****************************************************************************/
void COMMON_I2C_Start(void)
{
    SDA_OUTPUT_MODE;
    SCL_OUTPUT_MODE;

    SDA_H;
    I2C_DELAY(100);
    SCL_H;
    I2C_DELAY(100);
    SDA_L;
    I2C_DELAY(100);
    SCL_L;
    I2C_DELAY(100);
}

/*****************************************************************************
 *	������		��COMMON_I2C_Restart
 *	����		       ������I2C���߿�ʼ�ڶ������ݴ���
 *	�������	����
 *	�������	����
 *	����ֵ˵��	����
 *	����˵��	����
 *****************************************************************************/
void COMMON_I2C_Restart(void)
{
    SDA_OUTPUT_MODE;
    SCL_L;
    I2C_DELAY(100);

    SDA_H;
    I2C_DELAY(100);
    SCL_H;
    I2C_DELAY(100);
    SDA_L;
    I2C_DELAY(100);
    SCL_L;
}

/*****************************************************************************
*   ������      �� void COMMON_I2C_Stop(void)
*   ����        �� ֹͣλ��־
*   �������    �� void
*   �������    �� none
*   ����ֵ˵��  �� void
*****************************************************************************/
void COMMON_I2C_Stop(void)
{   
    SDA_OUTPUT_MODE;
    SDA_L;
    I2C_DELAY(100);
    SCL_H;
    I2C_DELAY(100);
    SDA_H;              /* SDA goes from low to high when SCL is already high */
    I2C_DELAY(100);
}

/*****************************************************************************
*   ������      �� char COMMON_I2C_Send_Byte(char cByteToSend)
*   ����        ��  I2C����һ���ֽڳ�ȥ
*   �������    �� cByteToSend: Ҫ���͵�byte
*   �������    �� none
*   ����ֵ˵��  �� COMMON_I2C_OK:    ok
                   DD_TS_ERROR: error
*****************************************************************************/
char COMMON_I2C_Send_Byte(char cByteToSend)
{
	char cLoop=0;
	char ack_singal=0;
	
    SCL_L;                  //Reset SCL
    SDA_OUTPUT_MODE;        //Enable SDA output
    for (cLoop=8; cLoop>0; cLoop--)
    {
        if (0x80 == (cByteToSend & 0x80))
        {
            SDA_H;          //Send one to SDA pin
        }
        else
        {                   
            SDA_L;          //Send zero to SDA pin
        }
        I2C_DELAY(100);
        SCL_H;              //Set SCL
        I2C_DELAY(200);
        SCL_L;              //Reset SCL
        I2C_DELAY(100);
        cByteToSend <<= 1;   //Rotate data
    }

    SDA_INPUT_MODE;         //SDA becomes an input for the ACKN
    I2C_DELAY(100);
    SCL_H;                  //Set SCL
    I2C_DELAY(100);
    SDA_INPUT_DATA(ack_singal);  //Check SDA for ACKN

    SCL_L;
    I2C_DELAY(100);


    if (0 == ack_singal)
    {
        return COMMON_I2C_OK;
    }
    else
    {
        return COMMON_I2C_ERROR;
    }
}

/*****************************************************************************
*   ������      �� char COMMON_I2C_Receive_Byte(char ackn)
*   ����        ��  I2C�������϶�ȡһ���ֽ�
*   �������    �� void
*   �������    �� none
*   ����ֵ˵��  �� ������8λ����
*****************************************************************************/
char COMMON_I2C_Receive_Byte(char ackn)
{
    char cLoop = 0 ;
    char cReceivedByte = 0;
    bool bTmp=0;

    SDA_INPUT_MODE;             /* Make SDA an input */
    SCL_L;                      /* Reset SCL */
    I2C_DELAY(100);
    for (cLoop=8; cLoop>0; cLoop--)
    {
        SCL_H;                  /* Set SCL */
        I2C_DELAY(100);
        cReceivedByte <<= 1;    /* Rotate data */
        SDA_INPUT_DATA(bTmp);   /* Read SDA -> data */
        if(1 == bTmp )
        {
            cReceivedByte |= 1;  
        }
        I2C_DELAY(100);
        SCL_L;
        I2C_DELAY(200);
    }

    SDA_OUTPUT_MODE;            /* SDA is turned in an output to write the ACK on the data line */
    I2C_DELAY(100);
    if (0 == ackn)
    {
        SDA_L;                  /* SDA = ACK bit */
    }
    else
    {
        SDA_H;                  /* SDA = ACK bit */
    }
    I2C_DELAY(100);
    SCL_H;                      /* Set SCL */
    I2C_DELAY(100);
    SCL_L;                    //Reset SCL
    I2C_DELAY(100);

    return(cReceivedByte);
}

static int gpio_virtual_i2c=0;
void COMMON_I2C_Init()
{
	if(!gpio_virtual_i2c)
		init_MUTEX(&i2c_gpio_pxa_list);
	gpio_virtual_i2c=1;
}

void COMMON_I2C_Exit()
{
	gpio_virtual_i2c=0;
}

void COMMON_I2C_mutx_down()
{
//	down(&i2c_gpio_pxa_list);
}

void COMMON_I2C_mutx_up()
{
//	up(&i2c_gpio_pxa_list);
}

//module_init(COMMON_I2C_Init);
//module_exit(COMMON_I2C_Exit);
