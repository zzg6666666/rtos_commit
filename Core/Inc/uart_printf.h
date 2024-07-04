#ifndef UART_PRINTF_H
#define UART_PRINTF_H

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

#define MIN(a, b) ((a < b) ? a : b)

#define BIT2 1U << 2
#define BIT6 1U << 6
#define BIT7 1U << 7
#define BIT14 1U << 14

// 使用中断进行输出
#define uart_IT 1

// 使用RTOS
#define use_RTOS 1

#define uart_test 0
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

// 打开UART1 RCC时钟
#define UART_RCC_INIT()         \
    do                          \
    {                           \
        RCC->APB2ENR |= (BIT14) \
    } while (0)

// 设置GPIOA启用
#define UART_GPIO_RCC_INIT()   \
    do                         \
    {                          \
        RCC->APB2ENR |= (BIT2) \
    } while (0);

// 打开移位寄存器非空中断
#define ENABLE_UART_TX_DR_IT()            \
    do                                    \
    {                                     \
        USART1->CR1 |= ((BIT7) & 0xFFFF); \
    } while (0)
// 禁用移位寄存器非空中断
#define DISABLE_UART_TX_DR_IT()            \
    do                                     \
    {                                      \
        USART1->CR1 &= ~((BIT7) & 0xFFFF); \
    } while (0)

// 设置传输完成中断
#define ENABLE_UART_TX_TC_IT()            \
    do                                    \
    {                                     \
        USART1->CR1 |= ((BIT6) & 0xFFFF); \
    } while (0)

// 禁用传输完成中断
#define DISABLE_UART_TX_TC_IT()            \
    do                                     \
    {                                      \
        USART1->CR1 &= ~((BIT6) & 0xFFFF); \
    } while (0)

// 发送数据到UART移位寄存器
#define UART_SEND_DATA_DR(data)     \
    do                              \
    {                               \
        USART1->DR = (data & 0xFF); \
    } while (0)
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
