/**
 * **********************************************************************
 * \file yplot.h
 * \date 2024-08-14
 * \version 1.0
 * \author fyuan (208793439@qq.com)
 * \copyright Copyright (c) 2024  xxxcompany
 * 
 * \brief 
 * 
 * **********************************************************************
 */
#ifndef _YPLOT_H_
#define _YPLOT_H_

#include "yplot_config.h"



typedef struct{
    //head = 0xAA
    uint16_t head;
    uint16_t tail;
    uint8_t rxbuffer[YPLOT_RXQUEUE_LEN];
}yplot_rxqueue_t;

typedef struct 
{
    uint8_t id;
    uint16_t len;
    uint8_t data[YPLOT_FRAME_TEMP_LEN];
}yplot_rxframe_t;


enum{
    YPLOT_ID_PLOTNAME = 0x01, ///< 向上位机发送plot的name
    YPLOT_ID_PLOT = 0x02, ///< 向上位机发送plot数据
    YPLOT_ID_INFO = 0x03, ///< 向上位机发送info信息
    YPLOT_ID_DEBUG = 0x04, ///< 向上位机发送debug信息
    YPLOT_ID_WARNING = 0x05, ///< 向上位机发送warning信息

    YPLOT_ID_SENDCMD = 0x10, /// < 接收上位机下发的命令
};

enum {
    YPLOT_OK = 0,
    YPLOT_ERR = 1,
    YPLOT_NONE = 2,
    YPLOT_FULL = 3,
};


/**
 * \brief 发送plot的name, 一般使用多少个通道发送多少个name, 数量不一致也不会出错.
 * \param name 格式为 name1,name2,name3,...,namen\0 , 注意这是一个字符串, 以\0结尾
 * \return uint8_t   YPLOT_OK: 成功, YPLOT_ERR: 失败
 */
uint8_t yplot_plotname(char *name);

/**
 * \brief 发送plot数据
 * \param data 一个float数组
 * \param len 数组长度
 * \return uint8_t   YPLOT_OK: 成功, YPLOT_ERR: 失败 
 */
uint8_t yplot_plot(float *data,uint8_t len);

/**
 * \brief 发送info信息
 * \param fmt 格式化字符串 和 printf 是一样的使用方式
 * \return uint8_t   YPLOT_OK: 成功, YPLOT_ERR: 失败
 */
uint8_t yplot_info(const char *fmt, ...);

/**
 * \brief 发送debug信息
 * \param fmt 格式化字符串.和 printf 是一样的使用方式
 * \return uint8_t   YPLOT_OK: 成功, YPLOT_ERR: 失败
 */
uint8_t yplot_debug(const char *fmt, ...);

/**
 * \brief 发送warning信息
 * \param fmt 格式化字符串 和 printf 是一样的使用方式
 * \return uint8_t   YPLOT_OK: 成功, YPLOT_ERR: 失败
 */
uint8_t yplot_waring(const char *fmt, ...);



/**
 * \brief 向 yplot 的接收缓存区中写入一个字节, 可以在串口接收中断中调用. 
 * 缓存区满时会覆盖最早的数据,因此需要及时调用yplot_analyse函数解析数据
 * \param data  串口接收到的数据
 */
void yplot_readdata(uint8_t data);

/**
 * \brief 从 yplot 的接收缓存区中读取数据, 可以在DMA接收中断中调用. 
 * 缓存区满时会覆盖最早的数据,因此需要及时调用yplot_analyse函数解析数据
 * \param data  串口接收到的数据
 * \param len   数据长度
 */
void yplot_readdatas(uint8_t *data, uint16_t len);

/**
 * \brief 解析 yplot 的接收缓存区中的数据, 可在main函数中轮询调用
 * \param id  解析到的数据的ID
 * \param data_len  解析到的数据的长度
 * \param data_ptr  解析到的数据的指针,该指针在下一次调用yplot_analyse函数时有效 (该指针指向 yplot_config_t.rxbuffer 中的数据, 不需要用户申请内存和释放内存)
 * \return uint8_t  YPLOT_OK: 成功, YPLOT_ERR: 失败, YPLOT_NONE: 没有解析到数据
 */
uint8_t yplot_analyse(yplot_rxframe_t *rxframe);
#endif // _YPLOT_H_
