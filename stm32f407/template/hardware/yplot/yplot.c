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
#include <stdarg.h>
#include <stdio.h>
// 内部函数

yplot_rxqueue_t m_rxqueue;
uint8_t init_success = 0;

uint8_t yplot_plotname(char *name) {
  uint16_t len = strlen(name);
  uint8_t txbuffer[YPLOT_TXBUFF_LEN];

  if (len + 4 > YPLOT_TXBUFF_LEN) {
    return YPLOT_ERR;
  }
  txbuffer[0] = 0xAA;
  txbuffer[1] = YPLOT_ID_PLOTNAME;
  txbuffer[2] = len & 0xFF;
  txbuffer[3] = (len >> 8) & 0xFF;
  memcpy(txbuffer + 4, name, len);
  YPLOT_WRITE_FUNC(txbuffer, len + 4);

  return YPLOT_OK;
}

uint8_t yplot_plot(float *data, uint8_t len) {
  uint16_t datalen = len * 4;
  uint8_t txbuffer[YPLOT_TXBUFF_LEN];

  if (datalen + 4 > YPLOT_TXBUFF_LEN) {
    return YPLOT_ERR;
  }
  txbuffer[0] = 0xAA;
  txbuffer[1] = YPLOT_ID_PLOT;
  txbuffer[2] = datalen & 0xFF;
  txbuffer[3] = (datalen >> 8) & 0xFF;
  memcpy(txbuffer + 4, data, datalen);
  YPLOT_WRITE_FUNC(txbuffer, datalen + 4);

  return YPLOT_OK;
}

uint8_t yplot_info(const char *fmt, ...) {
  uint8_t txbuffer[YPLOT_TXBUFF_LEN];

  va_list args;
  va_start(args, fmt);
  uint16_t len =
      vsnprintf((char *)(txbuffer + 4), YPLOT_TXBUFF_LEN - 4, fmt, args);
  va_end(args);

  if (len + 4 > YPLOT_TXBUFF_LEN) {
    return YPLOT_ERR;
  }
  txbuffer[0] = 0xAA;
  txbuffer[1] = YPLOT_ID_INFO;
  txbuffer[2] = len & 0xFF;
  txbuffer[3] = (len >> 8) & 0xFF;
  YPLOT_WRITE_FUNC(txbuffer, len + 4);

  return YPLOT_OK;
}

uint8_t yplot_debug(const char *fmt, ...) {
  uint8_t txbuffer[YPLOT_TXBUFF_LEN];

  va_list args;
  va_start(args, fmt);
  uint16_t len =
      vsnprintf((char *)(txbuffer + 4), YPLOT_TXBUFF_LEN - 4, fmt, args);
  va_end(args);

  if (len + 4 > YPLOT_TXBUFF_LEN) {
    return YPLOT_ERR;
  }
  txbuffer[0] = 0xAA;
  txbuffer[1] = YPLOT_ID_DEBUG;
  txbuffer[2] = len & 0xFF;
  txbuffer[3] = (len >> 8) & 0xFF;
  YPLOT_WRITE_FUNC(txbuffer, len + 4);

  return YPLOT_OK;
}

uint8_t yplot_waring(const char *fmt, ...) {
  uint8_t txbuffer[YPLOT_TXBUFF_LEN];

  va_list args;
  va_start(args, fmt);
  uint16_t len =
      vsnprintf((char *)(txbuffer + 4), YPLOT_TXBUFF_LEN - 4, fmt, args);
  va_end(args);

  if (len + 4 > YPLOT_TXBUFF_LEN) {
    return YPLOT_ERR;
  }
  txbuffer[0] = 0xAA;
  txbuffer[1] = YPLOT_ID_WARNING;
  txbuffer[2] = len & 0xFF;
  txbuffer[3] = (len >> 8) & 0xFF;
  YPLOT_WRITE_FUNC(txbuffer, len + 4);

  return YPLOT_OK;
}

void yplot_readdata(uint8_t data) {
  m_rxqueue.rxbuffer[m_rxqueue.tail] = data;
  m_rxqueue.tail = (m_rxqueue.tail + 1) % YPLOT_RXQUEUE_LEN;
}

void yplot_readdatas(uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    m_rxqueue.rxbuffer[m_rxqueue.tail] = data[i];
    m_rxqueue.tail = (m_rxqueue.tail + 1) % YPLOT_RXQUEUE_LEN;
  }
}

static uint8_t dequeue(uint8_t *data) {
  if (m_rxqueue.head == m_rxqueue.tail) {
    return YPLOT_ERR;
  }
  if (data != NULL) {
    *data = m_rxqueue.rxbuffer[m_rxqueue.head];
  }
  m_rxqueue.head = (m_rxqueue.head + 1) % YPLOT_RXQUEUE_LEN;
  return YPLOT_OK;
}

static uint8_t readqueue(uint16_t index, uint8_t *data) {
  uint16_t pos = (m_rxqueue.head + index) % YPLOT_RXQUEUE_LEN;
  if (pos == m_rxqueue.tail) {
    return YPLOT_ERR;
  }
  if (data != NULL) {
    *data = m_rxqueue.rxbuffer[pos];
  }
  return YPLOT_OK;
}

uint8_t yplot_analyse(yplot_rxframe_t *rxframe) {
  uint8_t temp = 0;

  // 缓存区中的数据长度
  int buffer_len = (m_rxqueue.tail - m_rxqueue.head +YPLOT_RXQUEUE_LEN) %
                  YPLOT_RXQUEUE_LEN;

  // 每一次解析,要么处理所有缓存区中的数据,要么解析出一帧数据
  for (int i = 0; i < buffer_len; ++i) {
    if (readqueue(0, &temp) == YPLOT_OK) {
      if (temp != 0xAA) {
        dequeue(NULL);
        continue;
      }
    }

    // 当前数据长度 < 4, 不解析长度
    if (buffer_len - i < 4) {
      return YPLOT_NONE;
    }
    readqueue(2, &temp);
    uint16_t data_len = temp;
    readqueue(3, &temp);
    data_len += temp << 8;
    // 当前剩余长度 < 数据长度 + 4 ,不解析数据
    if (buffer_len - i < data_len + 4) {
      return YPLOT_NONE;
    }

    // 开始解析
    dequeue(NULL); // 出队头部的0xAA
    dequeue(&rxframe->id);
    dequeue((uint8_t *)&rxframe->len);
    dequeue((uint8_t *)&rxframe->len + 1);
    for (int j = 0; j < rxframe->len; ++j) {
      dequeue(rxframe->data + j);
    }
    return YPLOT_OK;
  }
  return YPLOT_NONE;
}
