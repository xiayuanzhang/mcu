#include "comFifo.h"

//通信协议
// 0xAA ID(uint8_t) LEN(uint8_t) DATA(uint8_t*) 0x55


/*****************************************************************
名  称：uint8_t FIFO_BufferSeek(uint8_t* Cmd,uint8_t* const Dt,Typedef_FIFOBuffer* queue)
功  能：从接收缓冲区取出数据
参  数：uint8_t const* Cmd : 存放接收到的命令
        uint8_t* const Dt :  存放接收到的数据
        Typedef_FIFOBuffer* queue : 指定接收缓冲区
返回值：uint8_t类型:返回 0 表示没有收到数据,返回非0,表示收到数据
*****************************************************************/
uint8_t FIFO_BufferSeek(Frame_t *Cmd, TcpQueue_t *queue)
{
    int head_index = 0; //帧头索引
    uint8_t deta_len; //要接受的数据长度
    //如果队列中数据少于要接受的数据,则不解析
    if(myQueueNum(queue) < 4)
    {
       return 0;
    }
    
    //查找帧头
    head_index = myQueueDeFindMember(queue,0xAA);
    if(head_index < 0)
    {
        return 0;
    }
    //查询数据长度信息
    myQueueMemberIndex(queue,(uint8_t*)&deta_len,head_index+2);

    //表示数据长度不够
    if(myQueueNum(queue) < deta_len + 4)
    {
        return 0;
    }
    //有完整的一帧数据了,可以开始接收数据了
    uint8_t head,end;
    myQueueDeMember(queue,&head);
    myQueueDeMember(queue,&Cmd->id);
    myQueueDeMember(queue,&Cmd->len);
    myQueueDeMemberMult(queue,Cmd->data,deta_len);
    myQueueDeMember(queue,&end);
    if(end != 0x55)
    {
        //数据帧错误,返回失败
        return 0;
    }
    return 1;
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
int16_t FIFO_Package(Frame_t *Cmd, uint8_t*  dataStrem)
{
    dataStrem[0] = 0xAA;
    //帧头
    dataStrem[1] = Cmd->id;
    dataStrem[2] = Cmd->len;
    //数据
    memcpy(&dataStrem[3],Cmd->data,Cmd->len);
    dataStrem[3+Cmd->len] = 0x55;

    return 4+Cmd->len;
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



