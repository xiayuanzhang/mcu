#include "communication.h"
#include "comFifo.h"
#include "myqueue.h"
#include "typecover.h"
#include "ESP8266_USART3.h"
#include "ESP8266_Connect.h"
#include "systick.h"

int  test = 0;
void communicationTask(void)
{
    Frame_t frame;
    if(FIFO_BufferSeek(&frame,&g_tcp_rxqueue))
    {
        switch(frame.id)
        {
            //前进
            case 0x01:
				test++;
                //do something
                break;
            //后退
            case 0x02:
                //do something
                break;
            //左转
            case 0x03:
                //do something
                break;
            //右转
            case 0x04:
                //do something
                break;
            //停止
            case 0x05:
                //do something
                break;
            //电机速度
            case 0x06:
                //do something
                break;
                //继电器
            case 0x07:
                //do something
                break;
            default:
                break;
        }
    }
}

void communicationUpdate(void)
{
	static uint32_t time = 0;
	//1000ms上传一次数据
	if(ostime_get_ms() - time < 100){
		return;
	}
	time = ostime_get_ms();
    //温度
    float temp = (float)time;
    //湿度
    float humi = 0;
    //可燃气体浓度
    float gas = 0.3;
    //烟雾气体浓度
    float smoke = test;

    //打包
    Frame_t frame;
    frame.id = 0x10;
    float_to_uint8(temp,frame.data);
    float_to_uint8(humi,frame.data+4);
    float_to_uint8(gas,frame.data+8);
    float_to_uint8(smoke,frame.data+12);
    frame.len = 16;

    uint8_t s_buf[50];
    int len = FIFO_Package(&frame,s_buf);
    //发送
    esp8266_send_data(s_buf,len);
}
