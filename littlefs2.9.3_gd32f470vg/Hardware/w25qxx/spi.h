#ifndef __SPI_H__
#define __SPI_H__
#include "gd32f4xx.h"


void Spi0_Init(void);
uint8_t Spi0_SendByte(uint8_t data);
uint16_t Spi0_SendHalfWord(uint16_t data);

#endif
