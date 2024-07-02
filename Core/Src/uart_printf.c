#include "uart_printf.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"
#include "stdlib.h"

// fifio是否初始化
static uint8_t initUartPrintf = 0;
// fifo的句柄
static HANDLE_LOG_FIFO log_fifo_handle;

// 写数据到fifo
static uint8_t uart_fifo_put(const void *data, const uint16_t dataLen);
// 初始化FIFO
static void uart_printf_init();
// 从fifo出数据
static uint8_t uart_fifo_get(uint8_t *data, const uint16_t dataLen);
// 格式转换
static void HEX_TO_STR(uint8_t data, char *str);
// 初始化fifo
static void uart_printf_init()
{
    log_fifo_handle.fifo_len = FIFO_LOG_LEN;
    log_fifo_handle.uart_error_code = UART_TX_READY;

    log_fifo_handle.fifo_log.in = 0;
    log_fifo_handle.fifo_log.out = 0;
}

static uint8_t uart_fifo_put(const void *data, const uint16_t dataLen)
{
    // 此次写到fifo的长度
    uint16_t maxLength = 0;
    // fifo的剩余空间
    uint16_t buffSize = 0;
    // 从in到fifo_len的长度
    uint16_t forwardLength = 0;
    uint8_t *buff = NULL;
    // 下次写的位置
    uint16_t *in = NULL;
    // 下次读的位置
    uint16_t *out = NULL;

    // 初始化指针
    in = &(log_fifo_handle.fifo_log.in);
    out = &(log_fifo_handle.fifo_log.out);
    buff = &(log_fifo_handle.fifo_log.buff[0]);

    // 检查buff是否已经满  w + 1 = r 时， buff就是满的状态
    // 检查缓冲区是否已满

    if ((*in == (log_fifo_handle.fifo_len - 1) && *out == 0) || (*in + 1) == *out)
    {
        return FIFO_IN_FAIL;
    }

    // 计算出buff的容量

    // *in > *out时,能写的位置是 buff[in] 到 buff[len -1],  buff[0] 到 buff[out -1]
    if ((*in) > (*out))
    {
        buffSize = (log_fifo_handle.fifo_len - (*in)) + (*out);
    }
    // buff是空的，这个时候，能从buff[0] 写到 buff[len -1]
    else if ((*out) == (*in))
    {
        buffSize = log_fifo_handle.fifo_len;
    }
    // *in < *out,此时能从buff[in] 写到 buff[out -1]
    else
    {
        buffSize = *out - *in;
    }

    /*计算出，此次写buff的数据长度
      当 *in > *out时,能从buff[*in],写到buff[bufflen -1],从buff[0]写到buff[buffsize - forwardLength -1]
      假如buff长度= 20,*in = 15，*out = 8, buffsize = 20-15+8=13,forwardLength=5,
      能从buff[15]写到buff[19],从buff[0]写到buff[7]，更新*in为8。但是*out=8,不符合 *in +1 = *w的规则。
      设置maxLength为buffSize - 1 和 dataLen的最小值，maxLength = 12在写满buff的情况下,能从buff[0]-buff[6],
      更新*in = maxLength - forwardLength = 7，符合判定满的规则

      假如buff长度= 20,*in = 8，*out = 15,buffSize =15-8=7,在写满的情况下，设置maxLength = 7时，从buff[8]到
      buff[14],更新*in为15,和*out =15冲突不满足判定满的情况，设置maxLength为buffSize - 1 和 dataLen的最小值，
      maxLength = 6,从buff[8]到buff[13],更新*in为14,符合判定满的规则
    */

    maxLength = MIN(buffSize - 1, dataLen);

    // 当 *in > *out 时，计算出从fifo[in]到fifo[fifo_len - 1]的长度
    forwardLength = log_fifo_handle.fifo_len - *in;

    // maxLength > forwardLength ,一定是*in > *out的情况,需要写 buff[*in] 到 buff[len -1],buff[0]到buff[maxLength - forwardLength -1]
    if (maxLength > forwardLength)
    {

        memcpy(&buff[*in], data, forwardLength);                                        // 复制到 buff[*in] 到 buff[len -1]
        memcpy(&buff[0], &((uint8_t *)data)[forwardLength], maxLength - forwardLength); // 复制到 buff[0] 到 buff[ maxLength - forwardLength]

        // 更新 *in
        *in = maxLength - forwardLength; // 下次写入的位置是buff[maxLength - forwardLength]
    }
    //*in <= *out , maxLength =forwardLength  写入 buff[in] 到 buff[in-1] , 或是 *out < *in 且 写入长度小于forwardLength,写入的位置是buff[*in] 到 buff[*in + maxlen ]
    else
    {
        memcpy(&buff[*in], data, maxLength);
        *in = *in + maxLength;

        // 防止*in溢出
        if (*in == log_fifo_handle.fifo_len)
        {
            *in = 0;
        }
    }

    return FIFO_IN_OR_OUT_COMPLETE;
}

uint8_t uart_fifo_get(uint8_t *data, const uint16_t dataLen)
{
    // 此次进fifo的最长长度
    uint16_t maxLength = 0;
    // 从in到fifo_len的长度
    uint16_t forwardLength = 0;
    uint8_t *buff;
    uint16_t *in = 0;
    uint16_t *out = 0;

    in = &(log_fifo_handle.fifo_log.in);
    out = &(log_fifo_handle.fifo_log.out);
    buff = &(log_fifo_handle.fifo_log.buff[0]);

    // buf内是否有数据
    if (*in == *out)
    {

        return FIFO_OUT_FAIL;
    }

    // 计算已经入buff数据长度
    if (*in < *out)
    {
        maxLength = log_fifo_handle.fifo_len - *out + *in;
    }
    //*in > *out
    else
    {
        maxLength = *in - *out;
    }

    // 计算出从fifo[out]到fifo[fifo_len - 1]的长度
    forwardLength = log_fifo_handle.fifo_len - *out;

    /* 此次出buff的长度,不能超过buff的长度*/
    maxLength = MIN(maxLength, dataLen);
    if (maxLength > forwardLength)
    {
        memcpy(data, &buff[*out], forwardLength);
        memcpy(&data[forwardLength], buff, maxLength - forwardLength);
        *out = maxLength - forwardLength;
    }
    else
    {
        memcpy(data, &buff[*out], maxLength);
        *out = *out + maxLength;
        if (*out == log_fifo_handle.fifo_len)
        {
            *out = 0;
        }
    }

    return FIFO_IN_OR_OUT_COMPLETE;
    // 开启os调度
}

uint8_t uart_printf(const void *strData, const uint8_t *data, const uint8_t len)
{
#if use_RTOS
    // 关闭中断
    portDISABLE_INTERRUPTS();

#endif
    // 初始化
    if (initUartPrintf == 0)
    {
        initUartPrintf = 1;
        uart_printf_init();
    }

    // 将数据放入到FIFO中

    // strData 不能为NUll
    UART_ASSERT(strData != NULL);

    if (data != NULL)
    {
        uart_fifo_put(strData, strlen((char *)strData));

        // 储存HEX_TO_STR()的str,"-"，"ASCLl"，"ASCLl"
        char tempData[3] = "";
        for (uint8_t i = 0; i < len; i++)
        {
            HEX_TO_STR(*data, &tempData[0]);
            data++;

            // i == 1 删除 "-"
            if (i == 0)
            {
                // 复制"ascll"、"ascll"
                uart_fifo_put(&tempData[1], 2);
            }
            else
            {
                // 复制"-"","ascll"、"ascll"
                uart_fifo_put(&tempData[0], 3);
            }
        }
        uart_fifo_put("\r\n", 4);
    }
    else
    {
        uart_fifo_put(strData, strlen((char *)strData));
    }

    // 使能串口中断
    if (log_fifo_handle.uart_error_code == UART_TX_READY)
    {
        // 打开串口中断
        ENABLE_UART_TX_DR_IT();
        log_fifo_handle.uart_error_code = UART_TX_BUSY;
    }

#if use_RTOS
    // 打开中断
    portENABLE_INTERRUPTS();

#endif
    return 0;
}

// 仅操作DR寄存器
void uart_out()
{
    uint8_t data = 0;
    if (uart_fifo_get(&data, 1) == FIFO_OUT_FAIL)
    {
        UART_SEND_DATA_DR(data);
        DISABLE_UART_TX_DR_IT();
        log_fifo_handle.uart_error_code = UART_TX_READY;
    }
    else
    {
        UART_SEND_DATA_DR(data);
    }
}

// 输入data 转换成asill
static void HEX_TO_STR(uint8_t data, char *str)
{
    uint8_t temp = 0;
    memcpy(str, "-", 1);

    for (uint8_t i = 0; i < 2; i++)
    {
        // 高4位
        if (i == 0)
        {
            temp = (data & 0xf0) >> 4;
        }
        // 低四位
        else
        {
            temp = data & 0x0f;
        }
        temp = temp < 0x0a ? (temp + '0') : (temp - 0x0a + 'A');

        memcpy(&str[i + 1], &temp, 1);
    }
}

// 单元测试
