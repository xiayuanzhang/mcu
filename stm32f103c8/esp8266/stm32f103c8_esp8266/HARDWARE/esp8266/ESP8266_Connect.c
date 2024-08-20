#include "ESP8266_Connect.h"
#include "bsp_delay.h"
#include "myprintf.h"
#include "ESP8266_USART3.h"
typedef struct
{
	uint8_t rxFlag;
	struct{
		uint8_t len;
		uint8_t ptr[256];
	}data;
}CMDBuffer_t;

CMDBuffer_t cmd_buffer = {0,{0,{0}}};
TcpQueue_t g_tcp_rxqueue = {{0},0,0};
//配置ESP8266的模式
/**
 * @brief 配置ESP8266的模式
 * AP+SATA模式
 * 开启TCP服务器 192.168.5.1 8080
 * 
 * @return uint8_t  1 成功 0 失败
 
 */
uint8_t esp8266_init(void)
{
    uint8_t res = 0;
	for(int i = 0;i<5;i++){
		if(esp8266_send_cmd("ATE0\r\n","OK") == 1){  //使用关闭回显功能,替代AT,检测是否存在模块
			break;
		}
		delay_ms(1000);
		if(i == 5-1){
			printf("not find ESP8266 Module\r\n");
			return 0;
		}
	}
	
    //配置为AP+STA模式
    //AT+CWMODE=3
    res += esp8266_send_cmd("AT+CWMODE_CUR=2\r\n","OK");
	delay_ms(100);
    //配置wifi连接的路由器的SSID和密码, 该指令用时较长,所以wifi配置相关内容,使用串口助手配置.这里只配置tcp服务器
    //AT+CWWAP="SSID","PASSWORD"
    res += esp8266_send_cmd("AT+CWSAP_CUR=\"CAR\",\"12345678\",5,3\r\n","OK");
	delay_ms(2000);
    //配置为多连接模式
    //AT+CIPMUX=1
    res += esp8266_send_cmd("AT+CIPMUX=1\r\n","OK");
	delay_ms(100);
    //开启TCP服务器
    //配置服务器的IP
    //AT+CIPAP_CUR="192.168.5.1","192.168.5.1","255.255.255.0"
    res += esp8266_send_cmd("AT+CIPAP_CUR=\"192.168.5.1\",\"192.168.5.1\",\"255.255.255.0\"\r\n","OK");
	delay_ms(100);
    //AT+CIPSERVER=1,8080
    res += esp8266_send_cmd("AT+CIPSERVER=1,8080\r\n","OK");
    delay_ms(100);
    //关闭接收的 [,<remote IP>,<remote	port>] 信息
    //AT+CIPDINFO=0
    res += esp8266_send_cmd("AT+CIPDINFO=0\r\n","OK");
	delay_ms(100);
    if(res == 6){
        printf("ESP8266 init success\r\n");
        return 1;
    }else{
        printf("ESP8266 init failed,res = %d\r\n",res);
        return 0;
    }
}

/**
 * @brief 等待接收到ready_str字符串,超时时间为timeout_ms.
 * 如果 ready_str == NULL. 表示只要接收到数据即可
 * @return 1 成功 0 失败
 */
uint8_t esp8266_wait_cmd_ack(const char *ready_str, int timeout_ms)
{
    //等待ESP8266准备就绪
    //AT
    while(timeout_ms-- > 0){
        if(cmd_buffer.rxFlag == 1)
        {
			cmd_buffer.rxFlag = 0;
            //如果ready_str为空,表示只关心是否收到数据,不关心具体值,直接返回1
            if(ready_str == NULL)
            {
                return 1;
            }
            if(strstr((char*)cmd_buffer.data.ptr,ready_str) != NULL)
            {
                return 1;
            }
            
        }
        delay_ms(1);
    }
	return 0;
}

void esp8266_rx_task(uint8_t *data,int len)
{
    //接收数据
    //+IPD,<link ID>,<len>[,<remote IP>,<remote	port>]:<data>
    //+IPD,<link ID>,<len>:<data>
    //+IPD,0,10:HELLO WORLD
	//收到的数据起始为  /r/n+
    if(data[2] == '+'){
        //使用逗号分割字符串
        char *p = (char*)data;
        char *p1 = strchr(p,',');
        char *p2 = strchr(p1+1,',');
        int len = atoi(p2+1);
        char *p3 = strchr(p2+1,':');
        FIFO_DataEnqueued(&g_tcp_rxqueue,(uint8_t*)p3+1,len);
    }else{
        if(cmd_buffer.rxFlag == 0){
            memcpy(cmd_buffer.data.ptr,data,len);
            cmd_buffer.data.len = len;
            cmd_buffer.rxFlag = 1;
        }
        
    }
}

/**
 * @brief 
 * @return 1 成功 0 失败
 */
uint8_t esp8266_send_cmd(const char *cmd,const char *ack)
{
    usart3_send_datas((uint8_t *)cmd,strlen(cmd));
    return esp8266_wait_cmd_ack(ack,1000);
}

uint8_t esp8266_send_data(uint8_t *data,int len)
{
    char tenp[20] = {0};
    sprintf(tenp,"AT+CIPSEND=0,%d\r\n",len);
    if(esp8266_send_cmd(tenp,"OK") != 0)
    {
        //usart3_send_datas((const uint8_t *)">",1);
        usart3_send_datas(data,len);
        if(esp8266_wait_cmd_ack("SEND OK",100) != 0)
        {
            return 1;
        }
    }
    printf("send data failed\r\n");
    return 0;
}







