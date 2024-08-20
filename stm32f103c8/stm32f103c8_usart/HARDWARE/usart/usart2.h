#ifndef __USART2_H
#define __USART2_H
#include "stm32f10x.h"

void uart2_init(u32 bound);
void usart2_send_datas(uint8_t *datas,uint16_t len);
#endif


