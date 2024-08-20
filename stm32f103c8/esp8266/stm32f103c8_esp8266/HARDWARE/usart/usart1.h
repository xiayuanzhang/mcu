#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"
#include "systick.h"
#include <string.h>

#define DMA_BUFFER_LEN 1024

void uart1_init(u32 bound);
void usart1_send_datas(uint8_t *datas, uint16_t len);

#endif
