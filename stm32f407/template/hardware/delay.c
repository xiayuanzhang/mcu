/**
 * \copyright Copyright (c) 2024  xxxcompany
 * \file delay.c
 * \date 2024-06-14
 * \author fyuan (xxx@email.com)
 * 
 * \brief 
 * 该文件使用 DWT 计时器来完成us,ms级延时. 并且提供函数运行时间测量函数.
 * 
 * DWT计时器和systick一样,是cortex-m内核的寄存器,所以不受限于芯片
 *
 * DWT计时器的时钟频率为系统时钟频率
 */
#include "delay.h"

static float US_TICK = 0;

//不能使用0xffffffff计算最大值, 要留有足够的时间余量, 才能够使while判断正确
//如果出现程序一直在delay函数中的delay中等到, 应该将该值继续减小.
const uint32_t MAX_TICK = 0xF0000000;
static uint32_t DWT_MAXTIME_US = 0;

void dwt_init(void)
{
    /* 使能DWT外设 */
    DEM_CR |= (uint32_t)DEM_CR_TRCENA;                

    /* DWT CYCCNT寄存器计数清0 */
    DWT_CYCCNT = (uint32_t)0u;

    /* 使能Cortex-M DWT CYCCNT寄存器 */
    DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
	
    US_TICK = DWT_TICK / 1000000;
	
	 DWT_MAXTIME_US = MAX_TICK / US_TICK;
}



void delay_us(uint32_t us)
{
    uint32_t count = 0,start = 0,ticks = 0;
    if(us > DWT_MAXTIME_US){
        count = us / DWT_MAXTIME_US;
        us = us % DWT_MAXTIME_US;
    }
    for(uint32_t i = 0; i < count; i++){
        start = DWT_CYCCNT;
        while(DWT_CYCCNT - start < MAX_TICK);
    }
    start = DWT_CYCCNT;
    ticks = us * US_TICK;
    while(DWT_CYCCNT - start < ticks);
}


void delay_ms(uint32_t ms)
{
	delay_us(ms*1000);
}


void delay(float s)
{
    delay_us(s * 1000000);
}


void time_test_run(void)
{
    DWT_CYCCNT = 0;
}


float time_test_get(void)
{
    return (float)DWT_CYCCNT / DWT_TICK;
}



