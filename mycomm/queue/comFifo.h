
#ifndef __COMFIFO_H
#define __COMFIFO_H
#include "myqueue.h"



#define FIFO_FRAME_HEAD         {0XAA,0X55}            // 帧头内容

#define FRAME_NODATA_LEN      10  //一帧数据最小长度,即不包含数据的长度


//根据调试信息看到, 升级的时候一帧数据最大是 0x7DE, 2014
 //最大的数据长度是波形上报 512*4+9=2057,设置为3000稳当!

#define FRAME_BUFFER_MAXLEN  1024  //收发数据是一帧数据时最大的数组长度
#define FRAME_DATA_MAXLEN     (FRAME_BUFFER_MAXLEN - FRAME_NODATA_LEN)




#define TYPEDEF_TCP_FIFO MyQueue


typedef struct
{
	uint8_t  header1;   //自动填写 0xAA
	uint8_t  header2;   //自动填写 0x55
	uint16_t  addr;
	uint8_t  type;
	uint8_t  cmd;
	uint16_t dataLen;
	uint8_t  data[FRAME_DATA_MAXLEN];
	uint16_t crc16;     //自动填写
}TYPEDEF_DATA_STRUCT;


#define FIFO_Init(queue) myQueueInit(queue)
#define FIFO_DataEnqueued(fifo,data,length) myQueueEnMemberMult(fifo,data,length)

uint8_t FIFO_BufferSeek(TYPEDEF_DATA_STRUCT *Cmd, TYPEDEF_TCP_FIFO *queue);

int16_t FIFO_SendDataPackage(TYPEDEF_DATA_STRUCT *Cmd, uint8_t *const PackagedData);

uint16_t Modbus_CRC16(volatile uint8_t *ptr, uint16_t len);


#endif // !_

