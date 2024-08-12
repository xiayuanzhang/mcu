#ifndef _DELAY_H_
#define _DELAY_H_
#include <stdint.h>
#include "stm32f4xx.h"


#define  DWT_CR      *(__IO uint32_t *)0xE0001000  //DWT使能运行寄存器 (DWT)
#define  DWT_CYCCNT  *(__IO uint32_t *)0xE0001004  //DWT计数寄存器 (DWT)
#define  DEM_CR      *(__IO uint32_t *)0xE000EDFC  //使能DWT外设 (CoreDebug)

#define  DEM_CR_TRCENA                   (1 << 24) //使能DWT外设
#define  DWT_CR_CYCCNTENA                (1 <<  0)  //使能DWT计数器


#define DWT_TICK SystemCoreClock

void dwt_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

/**
 * \brief 
 * \param s 延时时间, 单位为秒, 范围为 0.000001 ~ flaot(max)
 */
void delay(float s);




/**
 * \brief 函数运行时间测量
 */
void time_test_run(void);


/**
 * \brief 函数测量的最大时间为 1/DWT_TICK * 0xffffffff, 如果超过该时间或者接近该时间可能是错误的结果
 * 
 * - 480MHz 主频, DWT_MAXTIME_S = 8s
 * - 240MHz 主频, DWT_MAXTIME_S = 16s
 * - 72MHz 主频, DWT_MAXTIME_S = 53s
 * 
 * \return float 函数运行时间测试结果, 单位 s
 */
float time_test_get(void);

#endif // _DELAY_H_
