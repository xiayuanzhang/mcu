
//定义一个定时器使用中断产生时钟信号来检测

#include "bsp_delay.h"


// APB1 = 60Hz 
//TCLK = APB1 * RCU_TIMER_PSC_MUL4  = 240Hz
void delay_timer_config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	DELAY_TIMER_RCU_FUNC(DELAY_TIMER_RCU, ENABLE); 
	
	TIM_DeInit(DELAY_TIMER); //配置定时器基础结构体
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;// 72M, 时间为1us
	TIM_TimeBaseStructure.TIM_Period=0xffff;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数
	
	TIM_TimeBaseInit(DELAY_TIMER, &TIM_TimeBaseStructure);    

	
	TIM_ClearITPendingBit(DELAY_TIMER, TIM_FLAG_Update); //清除更新中断，免得一打开中断立即产生中断
    TIM_ITConfig(DELAY_TIMER, TIM_IT_Update, DISABLE);    //关闭定时器中断
	
    TIM_Cmd(DELAY_TIMER, DISABLE);
}



void delay_us(uint32_t us)
{
	uint32_t count = us >> 16;
	uint32_t uss = us & 0xffff;
	
	TIM_Cmd(DELAY_TIMER, ENABLE);
	if(count > 0)
	{
		for(int i = 0; i < count; i++)
		{
			DELAY_TIMER->CNT = 0;
			while(DELAY_TIMER->CNT < 0xffff);
		}
	}
	DELAY_TIMER->CNT = 0;
	while(DELAY_TIMER->CNT < uss);
	TIM_Cmd(DELAY_TIMER, DISABLE);
}


void delay_ms(uint32_t ms)
{
	delay_us(ms * 1000);
}


void functime_test_start(void)
{
	DELAY_TIMER->CNT = 0;
	TIM_Cmd(DELAY_TIMER, ENABLE);
}


//最大时间为655635us
float functime_test_end_us(void)
{
	TIM_Cmd(DELAY_TIMER, DISABLE);
	uint32_t time =  DELAY_TIMER->CNT;
	return (float)time;
}




