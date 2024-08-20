#ifndef _ESP8266_CONNECT_H_
#define _ESP8266_CONNECT_H_

#include "comFifo.h"
#include <stdlib.h>

extern TcpQueue_t g_tcp_rxqueue;

uint8_t esp8266_init(void);

uint8_t esp8266_wait_cmd_ack(const char *ready_str, int timeout_ms);

void esp8266_rx_task(uint8_t *data,int len);

uint8_t esp8266_send_cmd(const char *cmd, const char *ack);

uint8_t esp8266_send_data(uint8_t *data, int len);


#endif


