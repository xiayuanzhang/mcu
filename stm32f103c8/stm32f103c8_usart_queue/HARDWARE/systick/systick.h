#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f10x.h"

void delay_init(void);
void delay_ms(uint32_t count);
uint32_t ostime_get_ms(void);
#endif





























