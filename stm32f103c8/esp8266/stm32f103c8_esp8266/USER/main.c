//=============================================================================
//文件名称：main.h
//功能概要：STM32F103C8核心检测
//版权所有：源地工作室www.vcc-gnd.com
//版权更新：2013-02-20
//调试方式：J-Link OB ARM SW方式 5MHz
//=============================================================================

//头文件
#include "stm32f10x.h"
#include "GPIOLIKE51.h"
#include "systick.h"
#include "usart3.h"
#include "bsp_delay.h"
#include "communication.h"
#include "ESP8266_USART3.h"
#include "ESP8266_Connect.h"
#include "usart1.h"


//=============================================================================
//文件名称：GPIO_Configuration
//功能概要：GPIO初始化
//参数说明：无
//函数返回：无
//=============================================================================
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE); 						 
//=============================================================================
//LED -> PC13
//=============================================================================			 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

int main(void)
{
	GPIO_Configuration();
	uart1_init(115200);
	uart3_init(115200);
	printf("start run\n");
	systick_init();
	delay_timer_config();
	esp8266_init();
	while(1)
	{        
		
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		for(int i = 2000000;i>0;i--);
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		for(int i = 2000000;i>0;i--);
		
		//解析指令任务.
		communicationTask();
		communicationUpdate();
	}
}





