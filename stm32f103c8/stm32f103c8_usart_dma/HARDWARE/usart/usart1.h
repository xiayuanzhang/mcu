#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"
#include "systick.h"
#include <string.h>

#define DMA_BUFFER_LEN 1024

void usart1_dma_init(uint32_t baudrate);
void usart1_send_datas(uint8_t *datas, uint16_t len);
void usart1_dma_send(uint8_t *buf, uint16_t dataLen, uint16_t waiteBusy_ms);
#endif
