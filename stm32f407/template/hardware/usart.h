#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx.h"
#include "sys.h" 

#define PRINTF_USART USART1

void usart1_init(u32 bound);
void usart1_write_bytes(uint8_t* bytes, uint16_t len);

void usart2_init(u32 bound);
void usart2_write_bytes(uint8_t* bytes, uint16_t len);
#endif


