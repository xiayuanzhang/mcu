#include "ESP8266_USART3.h"

#include "ESP8266_Connect.h"
// RX = PB11
// TX = PB10



void uart3_init(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // 使能UART3所在GPIOB的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 使能USART3，GPIOA时钟

	// TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// USART3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  // 根据指定的参数初始化VIC寄存器

	// USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;										// 串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// 字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							// 一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								// 无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// 收发模式

	USART_Init(USART3, &USART_InitStructure); // 初始化串口1

	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); // 开启串口空闲中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启串口接受中断

	USART_Cmd(USART3, ENABLE); // 使能串口1
}


void usart3_send_datas(uint8_t *datas, uint16_t len)
{
	while (len--) // 4 //3 // 2// 1 //0
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
			; // 等待发送完成
		USART_SendData(USART3, *datas);
		datas++;
	}
}

uint8_t usart3_buffer[256] = {0};
void USART3_IRQHandler(void) // 串口1中断服务程序
{
	static uint8_t count = 0;
	if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // 接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		//数据超出一帧
		if(count >= 256){
			count = 0;
		}
		usart3_buffer[count++] = USART_ReceiveData(USART3);
		
	}
	else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) // 空闲中断
	{
		USART_ClearITPendingBit(USART3, USART_IT_IDLE); // 清除中断标志
		USART_ReceiveData(USART3);
		esp8266_rx_task(usart3_buffer,count-1);
		count = 0;
	}
}
