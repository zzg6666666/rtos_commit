#ifndef UART_PRINTF_H
#define UART_PRINTF_H

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
//typedef unsigned int uint32_t;

#define MIN(a, b) ((a < b) ? a : b)

// 使用中断进行输出
#define uart_IT 1

// 使用RTOS
#define use_RTOS 1

#if use_RTOS
#include "FreeRTOS.h"
#endif

#if uart_IT
#include "stm32f103xb.h"
#endif



#define FIFO_LOG_LEN 1024

typedef enum
{
    UART_TX_FAIL,
    UART_TX_READY,
    UART_TX_BUSY,
    UART_TX_DR_EMPTY,
    UART_TX_NONE
} UART_ERROR_CODE;

typedef enum
{
    FIFO_IN_OR_OUT_COMPLETE = 0, // 写或读fifo成功
    FIFO_OUT_FAIL,               // 读fifo失败，因为in = out, fifo为空
    FIFO_IN_FAIL                 // 写fifo失败，因为fifo写满了
} FIFO_ERROR_CODE;

typedef struct
{
    // FIFO buff
    uint8_t buff[FIFO_LOG_LEN];
    // 下次写fifo index
    uint16_t in;
    // 下次读fifo index
    uint16_t out;
} ST_FIFO_LOG;

typedef struct
{
    uint16_t fifo_len;
    volatile UART_ERROR_CODE uart_error_code;
    // 使用中断时只用一个fifo
    ST_FIFO_LOG fifo_log;

} HANDLE_LOG_FIFO;

/******************Macro definition function************/

#define UART_ASSERT(x) \
    if ((x) == 0)      \
    {                  \
        for (;;)       \
            ;          \
    }

/***********************function***********************/

// 发动到DR寄存器
void uart_out();

// 使用串口打印
uint8_t uart_printf(const void *strData, const uint8_t *data, const uint8_t len);
/******************************************************/
#endif
