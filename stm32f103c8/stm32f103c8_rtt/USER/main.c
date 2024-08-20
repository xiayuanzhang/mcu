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
#include "SEGGER_RTT.h"

uint8_t abDataOut[BUFFER_SIZE_UP];
uint8_t downBuffer[1024*4];
int a = 0;


float ramval[12] __attribute__((at(0x20000000))) = {1, 2,3,4,
                  5, 6, 7, 8,	
                  9, 10, 11,	12
                 };
 
float fffff[12] __attribute__((at(0x20000100))) = {1, 2,3,4,
                  5, 6, 7, 8,	
                  9, 10, 11.32,	12.23
                 };
uint16_t iiiii[12] __attribute__((at(0x20000200))) = {1, 2,3,4,
                  5, 6, 7, 8,	
                  9, 10, 11,	12
                 };


void send_plot(void)
{
	static float x = 0;
	//正弦
//	x+=0.01f;
//	float plot_sin = 10 * sin(x);
	//float i = plot_sin;  
	
	//三角
	x > 1000?x = 0:(x+=10);
	float i = x;
	//SEGGER_RTT_printf(0,"%d\r\n",i);
	float buf[6];
	buf[0] = i+4000;
	buf[1] = i+1000;
	buf[2] = i+2000;
	buf[3] = i+6000;
	buf[4] = i+10000;
	ramval[0] = buf[0];
	ramval[1] = buf[1];
	ramval[2] = buf[2];
	ramval[3] = buf[3];
	
	int16_t buff2[5];
	buff2[0] =	(int16_t)(buf[0]);
	buff2[1] = (int16_t)(buf[1]);
	buff2[2] = (int16_t)(buf[2]);
	buff2[3] = (int16_t)(buf[3]);
	buff2[4] = (int16_t)(buf[4]);
//	buff2[3]  = (int16_t)(buf[0]);
//	buff2[3]  = (int16_t)(buf[2]);
	
	SEGGER_RTT_WriteNoLock(2,(char*)buff2,2*5);
//	SEGGER_RTT_WriteNoLock(2,(char*)buf,4*4);
}
int main(void)
{
    SEGGER_RTT_Init();
	SEGGER_RTT_ConfigUpBuffer(2, "JScope_i2i2i2i2i2", abDataOut, 1024*4,
                           SEGGER_RTT_MODE_NO_BLOCK_SKIP);
//SEGGER_RTT_ConfigUpBuffer(2, "JScope_f4f4f4f4", abDataOut, 1024*2,
//					   SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	SEGGER_RTT_ConfigUpBuffer(1, "cmd", abDataOut, BUFFER_SIZE_UP,
                           SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
	SEGGER_RTT_ConfigDownBuffer(1,"read",downBuffer,1024,SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);                                                                        
//
	int wave = 0;
	int terminal = 0;
	char rxbuffer[100];
	while(1)
	{        
		if(wave ++ > 200)
		{
			wave = 0;
			send_plot();
		}
		
		
//		if(terminal ++ > 1000000)
//		{
//			terminal = 0;
//			//SEGGER_RTT_WriteString(1, "Hello World from SEGGER!\r\n");
//			int rx = SEGGER_RTT_Read(1,rxbuffer,100);
//			if(rx>0)
//			{
//				SEGGER_RTT_Write(1,rxbuffer,rx);
//			}
//		}
//		
	}
}

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



