#ifndef _TYPECOVER_H
#define _TYPECOVER_H

#include <stdint.h>
#include <string.h>
//需要是小端机模式

/**
 * @brief 将 data 中的数据转换为 float类型. data的长度必须为4.否则转换结果可能异常
 * @return float 
 */
float uint8_to_float(const uint8_t *data);

/**
 * @brief 将 data 中的数据转换为 int类型. data的长度必须为4.否则转换结果可能异常
 * @return int 
 */
int uint8_to_int(const uint8_t *data);

/**
 * @brief 将 data 中的数据转换为 uint8_t数组. buf的空间至少为4.否则导致程序崩溃
 * @return uint16_t 
 */
void float_to_uint8(float data, uint8_t *buf);

/**
 * @brief 将 data 中的数据转换为 uint8_t数组. buf的空间至少为4.否则导致程序崩溃
 * @return uint16_t 
 */
void int_to_uint8(int data, uint8_t *buf);


//...
#endif
