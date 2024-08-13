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

#include <stdint.h>
#include <string.h>



typedef struct{
    void (*write)(uint8_t *data, uint16_t len); ///< yplot调用的发送函数的指针

    uint8_t *rxbuffer; ///< yplot的接收缓冲区, 建议大小至少为, (3*上位机会下发的指令的最大长度)byte. 设置太小会导致接收数据丢失.
    uint16_t rxbuffer_len; ///< yplot的接收缓冲区大小, 即rxbuffer数组的长度.

    uint8_t *txbuffer; ///< yplot的发送缓冲区, yplot发送数据时会将用户输入数据使用通信协议打包数据后存储在发送缓冲区中. 建议大小至少为 (10 + 发送的最大的数据长度)byte
    uint16_t txbuffer_len; ///< yplot的发送缓冲区大小, 即txbuffer数组的长度
}yplot_config_t;



typedef struct{
    //head = 0xAA
    uint16_t head;
    uint16_t tail;
    uint16_t rxbuffer_len;
    uint8_t *rxbuffer;

    uint16_t last_analysis_len; ///< 上一次解析成功的数据的长度(data_len+4)
}yplot_rxqueue_t;


enum{
    YPLOT_ID_PLOTNAME = 0x01,
    YPLOT_ID_PLOT = 0x02,
    YPLOT_ID_INFO = 0x03,
    YPLOT_ID_DEBUG = 0x04,
    YPLOT_ID_WARNING = 0x05,

    YPLOT_ID_SENDCMD = 0x10,
};

enum {
    YPLOT_OK = 0,
    YPLOT_ERR = 1,
    YPLOT_NONE = 2,
    YPLOT_FULL = 3,
};



uint8_t yplot_init(yplot_config_t *config);

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
 * \brief 向 yplot 的接收缓存区中写入一个字节, 可以在串口接收中断中调用. 缓存区满时不会写入数据,因此需要及时调用yplot_analyse函数解析数据
 * \param data  串口接收到的数据
 * \return uint8_t  YPLOT_OK: 成功, YPLOT_ERR: 失败, YPLOT_FULL: 缓冲区满
 */
uint8_t yplot_readdata(uint8_t data);

/**
 * \brief 从 yplot 的接收缓存区中读取数据, 可以在DMA接收中断中调用. 缓存区满时不会写入数据,因此需要及时调用yplot_analyse函数解析数据
 * \param data  串口接收到的数据
 * \param len   数据长度
 * \return uint8_t  YPLOT_OK: 成功, YPLOT_ERR: 失败, YPLOT_FULL: 缓冲区满
 */
uint8_t yplot_readdatas(uint8_t *data, uint16_t len);

/**
 * \brief 解析 yplot 的接收缓存区中的数据, 可在main函数中轮询调用
 * \param id  解析到的数据的ID
 * \param data_len  解析到的数据的长度
 * \param data_ptr  解析到的数据的指针,该指针在下一次调用yplot_analyse函数时有效 (该指针指向 yplot_config_t.rxbuffer 中的数据, 不需要用户申请内存和释放内存)
 * \return uint8_t  YPLOT_OK: 成功, YPLOT_ERR: 失败, YPLOT_NONE: 没有解析到数据
 */
uint8_t yplot_analyse(uint8_t *id,uint16_t *data_len, uint8_t **data_ptr);
#endif // _YPLOT_H_
