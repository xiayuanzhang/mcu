#include "comFifo.h"




/*****************************************************************
名  称：void FIFO_DataEnqueued(Typedef_FIFOBuffer* const FIFOx,uint8_t const* data,uint8_t length)
功  能：接收的数据存入缓冲队列
参  数：Typedef_FIFOBuffer* const FIFOx : 指向缓冲区队列的指针
        uint8_t const* data : 数据
        uint8_t length : 数据长度
返回值：无
*****************************************************************/
//#define FIFO_DataEnqueued(fifo,data,length) myQueueEnMemberMult(fifo,data,length)



/*****************************************************************
名  称：uint8_t FIFO_BufferSeek(uint8_t* Cmd,uint8_t* const Dt,Typedef_FIFOBuffer* queue)
功  能：从接收缓冲区取出数据
参  数：uint8_t const* Cmd : 存放接收到的命令
        uint8_t* const Dt :  存放接收到的数据
        Typedef_FIFOBuffer* queue : 指定接收缓冲区
返回值：uint8_t类型:返回 0 表示没有收到数据,返回非0,表示收到数据
*****************************************************************/
uint8_t FIFO_BufferSeek(TYPEDEF_DATA_STRUCT* Cmd,TYPEDEF_TCP_FIFO* queue)
{
    int head_index = 0; //帧头索引
    uint16_t deta_len; //要接受的数据长度
    uint16_t crc16; //crc16校验
    uint8_t head[2] = FIFO_FRAME_HEAD; //帧头

    //如果队列中数据少于要接受的数据,则不解析
    if(myQueueNum(queue) < FRAME_NODATA_LEN)
    {
       return 0;
    }
    
    //查找帧头
    head_index = myQueueDeFindMemberMult(queue,head,sizeof(head));
    if(head_index < 0)
    {
        return 0;
    }
    //查询数据长度信息
    myQueueMemberIndex(queue,(uint8_t*)&deta_len,head_index+6);
    myQueueMemberIndex(queue,(uint8_t*)&deta_len+1,head_index+7);

	//数据长度错误,丢弃掉该帧头
	if(deta_len+FRAME_NODATA_LEN > FRAME_BUFFER_MAXLEN)
	{
		myQueueDeMember(queue,&Cmd->header1);
		myQueueDeMember(queue,&Cmd->header2);
		return 0;
	}
	
    //表示数据长度不够
    if(deta_len+FRAME_NODATA_LEN > myQueueNum(queue))
    {
        return 0;
    }
	//数据长度超过了最大一帧的长度,掀桌子
	
    //有完整的一帧数据了,可以开始接收数据了
    myQueueDeMember(queue,&Cmd->header1);
    myQueueDeMember(queue,&Cmd->header2);
    myQueueDeMember(queue,(uint8_t*)&Cmd->addr); //addr低位
    myQueueDeMember(queue,(uint8_t*)&Cmd->addr+1); //addr高位
    myQueueDeMember(queue,&Cmd->type);
    myQueueDeMember(queue,&Cmd->cmd);
    myQueueDeMember(queue,(uint8_t*)&Cmd->dataLen); //datalen低位
    myQueueDeMember(queue,(uint8_t*)&Cmd->dataLen+1); //datalen高位
    myQueueDeMemberMult(queue,Cmd->data,deta_len);
    myQueueDeMember(queue,(uint8_t*)&Cmd->crc16); //crc16低位
    myQueueDeMember(queue,(uint8_t*)&Cmd->crc16+1); //crc16高位

    //crc16校验, type到数据最后一byte,不包括crc16
    crc16 = Modbus_CRC16(&Cmd->type,Cmd->dataLen + 4);

    if(crc16 == Cmd->crc16)
    {
        return 1;
    }
    else
    {
        return 0;
    }

}

/*****************************************************************
名  称：void FIFO_SendDataPackage(uint8_t const* Cmd,uint8_t const* SendData,uint8_t *const PackagedData)
功  能：数据打包,将制定的数据打包成一个帧
参  数：uint8_t const* Cmd : 需要打包的命令
        uint8_t const* SendData : 需要打包的数据
        uint8_t *const PackagedData : 存放以打包数据的数组
返回值：int16_t 
              0:无意义
             -1:数据长度超限
             >0:打包后的数据帧总长度
*****************************************************************/
int16_t FIFO_SendDataPackage(TYPEDEF_DATA_STRUCT *Cmd,uint8_t *const PackagedData)
{
    uint16_t crc16; //crc16校验
    uint8_t head[2] = FIFO_FRAME_HEAD; //帧头

    //数据长度超限
    if(Cmd->dataLen > FRAME_DATA_MAXLEN) 
    {
        return -1;
    }

    //帧头
    PackagedData[0] = head[0];
    PackagedData[1] = head[1];
    //地址
    PackagedData[2] = (uint8_t)Cmd->addr;
    PackagedData[3] = (uint8_t)(Cmd->addr>>8);
    //类型
    PackagedData[4] = Cmd->type;
    //命令
    PackagedData[5] = Cmd->cmd;
    //数据长度
    PackagedData[6] =  (uint8_t)Cmd->dataLen;
    PackagedData[7] = (uint8_t)(Cmd->dataLen >> 8);
    //数据
    memcpy(&PackagedData[8],Cmd->data,Cmd->dataLen);
    //crc16校验, 从类型字段到数据最后一byte
    crc16 = Modbus_CRC16(&PackagedData[4],Cmd->dataLen + 4);
    PackagedData[8+Cmd->dataLen] = (uint8_t)crc16;
    PackagedData[8+Cmd->dataLen+1] = (uint8_t)(crc16 >> 8);

    return Cmd->dataLen + FRAME_NODATA_LEN;

}




/**
 * @brief CRC16校验
 * 
 * @param ptr  
 * @param len 
 * @return uint16_t 
 */
uint16_t Modbus_CRC16(volatile uint8_t *ptr,uint16_t len)
{ 
    unsigned char i; 
    unsigned short crc = 0xFFFF; 
    if(len==0) 
    {
        len = 1;
    } 
    while(len--)  
    {   
        crc ^= *ptr; 
        for(i=0; i<8; i++)  
    	{ 
            if(crc&1) 
        	{ 
                crc >>= 1;  
                crc ^= 0xA001; 
        	}  
        	else 
    		{
                crc >>= 1;
        	} 
        }      
        ptr++; 
    } 
    return(crc); 
} 



