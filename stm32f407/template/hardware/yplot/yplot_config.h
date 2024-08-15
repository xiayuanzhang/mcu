#ifndef _YPLOT_CONFIG_H_
#define _YPLOT_CONFIG_H_


#include <stdint.h>
#include <string.h>


#include "usart.h"

//接收数据的环形队列长度, 建议设置为 大于 3 * 上位机会下发的指令的最大长度. 否则可能导致数据丢失
//会占用 YPLOT_RXQUEUE_LEN (byte) 的 RAM
#define YPLOT_RXQUEUE_LEN 1024

//解析出的一帧数据会暂存在一个缓存数组中, 通过该宏定义缓存数组的长度. 
//建议设置为 大于 上位机会下发的指令的最大长度. 否则可能导致解析的一帧数据不完整
//不会占用RAM空间, 会占用 stack 空间
#define YPLOT_FRAME_TEMP_LEN 128


//发送数据的缓冲区长度, 建议设置为10 + 可能发送的最大的数据长度.否则会发送失败
//不会占用RAM空间, 会占用 stack 空间. 需要在 startup_xxxx.s(启动文件) 中保证 Stack_Size(Stack_Size单位为word) 足够
#define YPLOT_TXBUFF_LEN 128


//发送函数
///函数格式: void func(uint8_t *bytes, uint16_t len)
#define YPLOT_WRITE_FUNC usart2_write_bytes



#endif // _YPLOT_CONFIG_H_
