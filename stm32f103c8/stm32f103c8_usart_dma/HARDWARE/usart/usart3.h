#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f10x.h"

void uart3_init(u32 bound);
void usart3_dma_init(u32 baudrate);
void usart3_send_datas(uint8_t *datas,uint16_t len);
void usart3_dma_send(uint8_t *buf,uint16_t dataLen,uint16_t waiteBusy_ms);
#endif


