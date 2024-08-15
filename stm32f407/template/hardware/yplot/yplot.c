/**
 * **********************************************************************
 * \file yplot.c
 * \date 2024-08-14
 * \version 1.0
 * \author fyuan (208793439@qq.com)
 * \copyright Copyright (c) 2024  xxxcompany
 * 
 * \brief 
 * 
 * **********************************************************************
 */
#include "yplot.h"
#include <stdio.h>
#include <stdarg.h> 
//内部函数



yplot_config_t m_config;
yplot_rxqueue_t m_rxqueue;
uint8_t init_success = 0;

uint8_t yplot_init(yplot_config_t *config)
{
    m_config = *config;
    if(m_config.write != NULL 
    && m_config.rxbuffer != NULL
    && m_config.rxbuffer_len != 0
    && m_config.txbuffer != NULL
    && m_config.txbuffer_len != 0)
    {
        m_rxqueue.head = 0;
        m_rxqueue.tail = 0;
        m_rxqueue.rxbuffer_len = m_config.rxbuffer_len;
        m_rxqueue.rxbuffer = m_config.rxbuffer;

        m_rxqueue.last_analysis_len = 0;

        init_success = 1;
        return YPLOT_OK;
    }
    else
    {
        init_success = 0;
        return YPLOT_ERR;
    }
}

uint8_t yplot_plotname(char *name)
{
    uint16_t len = strlen(name);
    if(!init_success){
        return YPLOT_ERR;
    }
    if(len + 4 > m_config.txbuffer_len){
        return YPLOT_ERR;
    }
    m_config.txbuffer[0] = 0xAA;
    m_config.txbuffer[1] = YPLOT_ID_PLOTNAME;
    m_config.txbuffer[2] = len & 0xFF;
    m_config.txbuffer[3] = (len >> 8) & 0xFF;
    memcpy(m_config.txbuffer + 4, name, len);
    m_config.write(m_config.txbuffer, len + 4);

    return YPLOT_OK;
}


uint8_t yplot_plot(float *data, uint8_t len)
{
    uint16_t datalen = len * 4;
    if(!init_success){
        return YPLOT_ERR;
    }
    if(datalen + 4 > m_config.txbuffer_len){
        return YPLOT_ERR;
    }
    m_config.txbuffer[0] = 0xAA;
    m_config.txbuffer[1] = YPLOT_ID_PLOT;
    m_config.txbuffer[2] = datalen & 0xFF;
    m_config.txbuffer[3] = (datalen >> 8) & 0xFF;
    memcpy(m_config.txbuffer + 4, data, datalen);
    m_config.write(m_config.txbuffer, datalen + 4);

    return YPLOT_OK;
}



uint8_t yplot_info(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    uint16_t len = vsnprintf((char*)(m_config.txbuffer + 4), m_config.txbuffer_len - 4, fmt, args);
    va_end(args);
    if(!init_success){
        return YPLOT_ERR;
    }
    if(len + 4 > m_config.txbuffer_len){
        return YPLOT_ERR;
    }
    m_config.txbuffer[0] = 0xAA;
    m_config.txbuffer[1] = YPLOT_ID_INFO;
    m_config.txbuffer[2] = len & 0xFF;
    m_config.txbuffer[3] = (len >> 8) & 0xFF;
    m_config.write(m_config.txbuffer, len + 4);

    return YPLOT_OK;
}

uint8_t yplot_debug(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    uint16_t len = vsnprintf((char*)(m_config.txbuffer + 4), m_config.txbuffer_len - 4, fmt, args);
    va_end(args);
    if(!init_success){
        return YPLOT_ERR;
    }
    if(len + 4 > m_config.txbuffer_len){
        return YPLOT_ERR;
    }
    m_config.txbuffer[0] = 0xAA;
    m_config.txbuffer[1] = YPLOT_ID_DEBUG;
    m_config.txbuffer[2] = len & 0xFF;
    m_config.txbuffer[3] = (len >> 8) & 0xFF;
    m_config.write(m_config.txbuffer, len + 4);

    return YPLOT_OK;
}

uint8_t yplot_waring(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    uint16_t len = vsnprintf((char*)(m_config.txbuffer + 4), m_config.txbuffer_len - 4, fmt, args);
    va_end(args);
    if(!init_success){
        return YPLOT_ERR;
    }
    if(len + 4 > m_config.txbuffer_len){
        return YPLOT_ERR;
    }
    m_config.txbuffer[0] = 0xAA;
    m_config.txbuffer[1] = YPLOT_ID_WARNING;
    m_config.txbuffer[2] = len & 0xFF;
    m_config.txbuffer[3] = (len >> 8) & 0xFF;
    m_config.write(m_config.txbuffer, len + 4);

    return YPLOT_OK;
}



uint8_t yplot_readdata(uint8_t data)
{
    if(!init_success){
        return YPLOT_ERR;
    }
    uint16_t tail = (m_rxqueue.tail+ 1) % m_rxqueue.rxbuffer_len;
		
    if(tail == m_rxqueue.head ){
        return YPLOT_FULL;
    }
    m_rxqueue.rxbuffer[m_rxqueue.tail] = data;
    m_rxqueue.tail = tail;
    return YPLOT_OK;
}

uint8_t yplot_readdatas(uint8_t *data, uint16_t len)
{
    if(!init_success){
        return YPLOT_ERR;
    }
    for(uint16_t i = 0; i < len; i++){
        if(m_rxqueue.tail == (m_rxqueue.head + 1) % m_rxqueue.rxbuffer_len){
            return YPLOT_FULL;
        }
        m_rxqueue.rxbuffer[m_rxqueue.tail] = data[i];
        m_rxqueue.tail = (m_rxqueue.tail + 1) % m_rxqueue.rxbuffer_len;
    }
    return YPLOT_OK;
}


uint8_t yplot_analyse(yplot_rxframe_t *rxframe)
{
    if(!init_success){
        return YPLOT_ERR;
    }

    //再一次调用该解析函数时, 认为上一帧数据以及处理完成, 因此丢弃上一帧数据
    m_rxqueue.head = (m_rxqueue.head + m_rxqueue.last_analysis_len) % m_rxqueue.rxbuffer_len;
	m_rxqueue.last_analysis_len = 0;//清除
    
    //计算当前缓存区中的数据长度
    int buffer_len = m_rxqueue.tail - m_rxqueue.head;
    if(buffer_len < 4){
        return YPLOT_NONE;
    }

    //每一次解析,要么处理所有缓存区中的数据,要么解析出一帧数据
    for(int i = 0;i < buffer_len;++i){
        if(m_rxqueue.rxbuffer[m_rxqueue.head] != 0xAA){
            m_rxqueue.head = (m_rxqueue.head + 1) % m_rxqueue.rxbuffer_len; //丢弃一个字节
        }else{
            rxframe->id = m_rxqueue.rxbuffer[m_rxqueue.head + 1];
            rxframe->len = m_rxqueue.rxbuffer[m_rxqueue.head + 2] + (m_rxqueue.rxbuffer[m_rxqueue.head + 3] << 8);
            if(rxframe->len + 4 > buffer_len){
                return YPLOT_NONE;
            }
            rxframe->data = m_rxqueue.rxbuffer + m_rxqueue.head + 4;

            m_rxqueue.last_analysis_len = rxframe->len + 4; //记录上一次解析成功的数据的长度(data_len+4)
            return YPLOT_OK;
        }
    }
    return YPLOT_NONE;
}

