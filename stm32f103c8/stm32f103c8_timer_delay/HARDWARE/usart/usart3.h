#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f10x.h"

void uart3_init(u32 bound);
void usart3_send_datas(uint8_t *datas,uint16_t len);
#endif


