/**
 * @copyright Copyright (c) 2023 *wait for added* All rights reserved
 * @file bsp_delay.h
 * @version 0.1
 * @date 2023-09-25
 * 
 * @author fyuan (208793439@qq.com)
 * 
 * @brief 延时函数
 */
#ifndef _BSP_DELAY_H
#define _BSP_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f10x.h"

//使用定时器5完成定时
//DelayUs 使用硬件延时,只能在1~1000us延时,使用该延时方法,FreeRTOS不会进行任务调度
//OsDelayMs 使用FreeRTOS提供的延时方法,会进行任务调度


#define DELAY_TIMER_RCU_FUNC            RCC_APB1PeriphClockCmd
#define DELAY_TIMER_RCU  				RCC_APB1Periph_TIM4
#define DELAY_TIMER      				TIM4
#define DELAY_TIMER_IRQHANDLER          TIM4_IRQHandler


//#define BSP_TIMER_RCU  				RCU_TIMER2
//#define BSP_TIMER      				TIMER2
//#define BSP_TIMER_IRQ  			  TIMER2_IRQn
//#define BSP_TIMER_IRQHANDLER  TIMER2_IRQHandler

/// @cond
void delay_timer_config(void);
/// @endcond
//非精准延时,使用该延时会进行任务调度.允许延时很久

/**
* @brief 精准的us级延时函数 (建议使用SleepUs和实时模式组合,而非直接使用DelayUs,为了程序兼容性保留该函数入口)
 * - 误差 <= 0.5us. 
 * 
 * @param us 输入范围 0~(2^32-1) / 10.  (0~858993459us)
 */
void delay_us(uint32_t us);


/**
 * @brief 精准的ms级延时函数. (建议使用SleepUs和实时模式组合,而非直接使用DelayMs,为了程序兼容性保留该函数入口)
 * - 误差 <= 0.5us. 
 * @param ms 输入范围 0~ (2^32-1) / 10000. (0~858993ms)
 */
void delay_ms(uint32_t ms);

/**
* @brief 开始统计程序运行耗时, 单位us. 最长时间为50ms. 不能在 FuncTimeTestStartUs 和  FuncTimeTestEndUs之间使用延时函数, 否则测量结果无效.
 */
void functime_test_start(void);

/// @endcond
/**
 * @brief 结束统计程序运行耗时, 单位us, 最长时间为50ms
 * @return uint32_t 
 */
float functime_test_end_us(void);

#ifdef __cplusplus
}
#endif

#endif
