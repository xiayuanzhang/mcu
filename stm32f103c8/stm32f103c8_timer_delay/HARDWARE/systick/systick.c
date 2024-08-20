#include "systick.h"


static volatile uint32_t g_localtime;
		 
void systick_init()
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	SysTick_Config(SystemCoreClock/1000);
}								    

void SysTick_Handler(void)
{
	g_localtime++;
}

void ostime_delay_ms(uint32_t count)
{
	uint32_t delay = g_localtime; 
    while(g_localtime!=delay+count){
    }
}

//1ms
uint32_t ostime_get_ms(void)
{
	return g_localtime;
}







































