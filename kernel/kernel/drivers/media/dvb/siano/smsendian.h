

#ifndef __SMS_ENDIAN_H__
#define __SMS_ENDIAN_H__

#include <asm/byteorder.h>

void smsendian_handle_tx_message(void *buffer);
void smsendian_handle_rx_message(void *buffer);
void smsendian_handle_message_header(void *msg);

#endif 

