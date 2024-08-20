
#ifndef __COMFIFO_H
#define __COMFIFO_H
#include "myqueue.h"



#define TcpQueue_t MyQueue

#define FRAME_DATA_MAX_LEN 128

typedef struct
{
	uint8_t  id;
	uint8_t  len;
	uint8_t  data[FRAME_DATA_MAX_LEN];
}Frame_t;


#define FIFO_Init(queue) myQueueInit(queue)

#define FIFO_DataEnqueued(fifo,data,length) myQueueEnMemberMult(fifo,data,length)

uint8_t FIFO_BufferSeek(Frame_t *Cmd, TcpQueue_t *queue);

int16_t FIFO_Package(Frame_t *Cmd, uint8_t*  dataStrem);

uint16_t Modbus_CRC16(volatile uint8_t *ptr, uint16_t len);


#endif // !_

