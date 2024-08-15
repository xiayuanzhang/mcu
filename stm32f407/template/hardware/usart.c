#include "sys.h"
#include "usart.h"
#include "yplot.h"

// 重定义fputc函数
int fputc(int ch, FILE *f)
{
    while ((PRINTF_USART->SR & 0X40) == 0)
        ; // 循环发送,直到发送完毕
    PRINTF_USART->DR = (u8)ch;
    return ch;
}

// 初始化IO 串口1
// bound:波特率
void usart1_init(u32 bound)
{
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // 使能GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 使能USART1时钟

    // 串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // GPIOA9复用为USART1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // GPIOA10复用为USART1

    // USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // GPIOA9与GPIOA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // 速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  // 初始化PA9，PA10

    // USART1 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                                     // 波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // 收发模式
    USART_Init(USART1, &USART_InitStructure);                                       // 初始化串口1

    USART_Cmd(USART1, ENABLE); // 使能串口1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启相关中断

    // Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;         // 串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // 子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                           // 根据指定的参数初始化VIC寄存器、
}

void usart1_write_bytes(uint8_t *bytes, uint16_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
            ;
        USART_SendData(USART1, bytes[i]);
    }
}



void usart2_init(uint32_t bound)
{
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  // 使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 使能USART2时钟

    // 串口2对应引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  // GPIOA2复用为USART2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // GPIOA3复用为USART2

    // USART2端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; // GPIOA2与GPIOA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // 速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;            // 上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  // 初始化PA2，PA3

    // USART2 初始化设置
    USART_InitStructure.USART_BaudRate = bound;                                     // 波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // 无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // 收发模式
    USART_Init(USART2, &USART_InitStructure);                                       // 初始化串口2

    USART_Cmd(USART2, ENABLE); // 使能串口2

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 开启相关中断
		
		// Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;         // 串口1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // 子优先级3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                           // 根据指定的参数初始化VIC寄存器、
}


void usart2_write_bytes(uint8_t *bytes, uint16_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
            ;
        USART_SendData(USART2, bytes[i]);
    }
}

/************************************************************************************************************** */
void USART1_IRQHandler(void) // 串口1中断服务程序
{
    u8 Res;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // 接收中断(接收到的数据必须是0x0d 0x0a结尾)
    {
        Res = USART_ReceiveData(USART1); //(USART1->DR);	//读取接收到的数据
    }
}

void USART2_IRQHandler(void) // 串口2中断服务程序
{
    u8 Res;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) // 接收中断(接收到的数据必须是0x0d 0x0a结尾)
    {
        Res = USART_ReceiveData(USART2); //(USART1->DR);	//读取接收到的数据
				yplot_readdata(Res);
    }
}
