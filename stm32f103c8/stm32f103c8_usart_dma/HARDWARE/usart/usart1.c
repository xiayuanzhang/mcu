#include "usart1.h"	  


uint8_t dma_rx1[DMA_BUFFER_LEN];
uint8_t dma_rx2[DMA_BUFFER_LEN];
uint8_t dma_send[DMA_BUFFER_LEN];
uint8_t dma_send_busy_flag = 0;




void usart1_dma(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//使能DMA传输
    
    //Tx
    DMA_DeInit(DMA1_Channel4);//将DMA通道4重设为缺省值

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMA外设串口基地址 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据宽度为8位 

	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dma_send;//DMA内存基地址
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器递增 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存数据宽度为8位 
	
	DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_LEN;//缓存区大小 
	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//DMA传输方向，从内存读取到外设
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//正常缓存模式  
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//DMA通道拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道没有设置为内存到内存模式 
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);  
    DMA_Cmd(DMA1_Channel4,DISABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//使能串口DMA发送
   

	//Rx
    DMA_DeInit(DMA1_Channel5);//将DMA通道5重设为缺省值

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMA外设串口基地址
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据宽度为8位 

	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)dma_rx1;//DMA内存基地址
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器递增 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存数据宽度为8位 

	DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_LEN;//缓存区大小

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//DMA传输方向，从外设读取到内存  
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//正常缓存模式  
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//DMA通道拥有超高优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道没有设置为内存到内存模式

	DMA_Init(DMA1_Channel5, &DMA_InitStructure);  
    DMA_Cmd(DMA1_Channel5,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//使能串口DMA接收
}

void uart1_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//使能USART1，GPIOA时钟

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//配置为上拉输入,尝试解决485一直读取的问题   //GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	  USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启串口空闲中断
	  USART_ITConfig(USART1, USART_IT_TC, ENABLE);//开启串口发送完成中断
	  
	  USART_ClearITPendingBit(USART3, USART_IT_IDLE); //清除中断标志
	  USART_ClearITPendingBit(USART3, USART_IT_TC); //清除中断标志
	  
	  
	  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}


void usart1_dma_init(u32 baudrate)
{
	uart1_init(baudrate);
	usart1_dma();
}


void usart1_send_datas(uint8_t *datas,uint16_t len)
{
	while(len--) // 4 //3 // 2// 1 //0
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET)
			;//等待发送完成
		USART_SendData(USART1,*datas);
		datas++;
	}
}

void usart1_dma_send(uint8_t *buf,uint16_t dataLen,uint16_t waiteBusy_ms)
{
	while(waiteBusy_ms != 0){
		if(dma_send_busy_flag == 0){
			break;
		}else{
			waiteBusy_ms--;
			delay_ms(1);
		}
	}
	
	memcpy(dma_send,buf,dataLen);
	DMA_Cmd(DMA1_Channel4,DISABLE);
	DMA1_Channel4->CMAR = (uint32_t)dma_send;
	DMA_ClearFlag(DMA1_FLAG_GL4|DMA1_FLAG_TC4|DMA1_FLAG_HT4|DMA1_FLAG_TE4);
	DMA_SetCurrDataCounter(DMA1_Channel4,dataLen);//重置接收数据长度
	DMA_Cmd(DMA1_Channel4,ENABLE);
	dma_send_busy_flag = 1;
}
//从串口接收中断开始收数据,知道空闲中断再处理数据即可

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		USART_ClearITPendingBit(USART1, USART_IT_IDLE); //清除中断标志
		USART_ReceiveData(USART1);
		
		static uint8_t bufferFlag = 0; 
		uint32_t reclen =  DMA_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Channel5);
		//关闭DMA通道
		DMA_Cmd(DMA1_Channel5,DISABLE);
		//切换接收缓存区,没有使用硬件的缓存区切换函数,拷贝数据
		if(bufferFlag == 0)
		{
			//rs485_receive(dma_rx1,reclen);
			DMA1_Channel5->CMAR = (uint32_t)dma_rx2; //切换缓存区
			bufferFlag = 1;
		}
		else
		{
			//rs485_receive(dma_rx2,reclen);
			DMA1_Channel5->CMAR = (uint32_t)dma_rx1;
			bufferFlag = 0;
		}
		//重启DMA传输
		DMA_ClearFlag(DMA1_FLAG_GL5|DMA1_FLAG_TC5|DMA1_FLAG_HT5|DMA1_FLAG_TE5);
		DMA_SetCurrDataCounter(DMA1_Channel5,DMA_BUFFER_LEN);//重置接收数据长度
		DMA_Cmd(DMA1_Channel5,ENABLE);

    }
	else if(USART_GetITStatus(USART1, USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		DMA_Cmd(DMA1_Channel4,DISABLE);
		dma_send_busy_flag = 0;
	}
} 






