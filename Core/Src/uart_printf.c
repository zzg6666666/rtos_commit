#include "uart_printf.h"
#include "string.h"
// #include "usart.h"
#include "stdio.h"
#include "stdlib.h"

// fifio是否初始化
static uint8_t initUartPrintf = 0;
// fifo的句柄
static HANDLE_LOG_FIFO log_fifo_handle;

// RCC_BASS
const uint32_t RCC_BASE1 = 0x40021000;
// RCC_APB2ENR 时钟
static volatile uint32_t *RCC_APB2ENR = (uint32_t *)(RCC_BASE1 + 0x18);
// RCC 时钟配置寄存器
static volatile uint32_t *RCC_CFGR = (uint32_t *)(RCC_BASE1 + 0x04);

// GPIOA 端口配置寄存器
static volatile uint32_t *GPIOA_CRH = (uint32_t *)(0x40010800 + 0x04);

// USART1 寄存器基地址
const uint32_t USART1_BASS = 0x40013800;

#if !uart_IT
// USART1_SR寄存器
static volatile uint32_t *USART1_SR = (uint32_t *)0x40013800;
#endif

// USART1_DR寄存器
static volatile uint32_t *USART1_DR = (uint32_t *)(USART1_BASS + 0x04);
// USART1 波特比率寄存器
static volatile uint32_t *USART1_BRR = (uint32_t *)(USART1_BASS + 0x08);
// USART1 CR1寄存器
static volatile uint32_t *USART1_CR1 = (uint32_t *)(USART1_BASS + 0x0C);
// USART1 CR2寄存器
static volatile uint32_t *USART1_CR2 = (uint32_t *)(USART1_BASS + 0x10);
// USART1 CR3寄存器
static volatile uint32_t *USART1_CR3 = (uint32_t *)(USART1_BASS + 0x14);


// 将data写到DR寄存器
#define UART_SEND_DATA_DR(data) *USART1_DR = (data & 0xFF)
// 禁用DR寄存器空中断
#define DISABLE_UART_TX_DR_IT() *USART1_CR1 &= ~(1U << 7)
// 启用DR寄存器空中断
#define ENABLE_UART_TX_DR_IT() *USART1_CR1 |= (1U << 7)

// 写数据到fifo
static uint8_t uart_fifo_put(const void *data, const uint16_t dataLen);
// 初始化FIFO
static void uart_printf_init();
// 初始化uart硬件
static void uart_hardware_init();
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

        uart_hardware_init();
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
#if uart_IT
    // 使能串口中断
    if (log_fifo_handle.uart_error_code == UART_TX_READY)
    {
        // 打开串口中断
        ENABLE_UART_TX_DR_IT();
        log_fifo_handle.uart_error_code = UART_TX_BUSY;
    }
#else

    uint8_t temp = 0;
    while (uart_fifo_get(&temp, 1) != FIFO_OUT_FAIL)
    {
        while ((*USART1_SR & (1U << 7)) == 0)
        {
        }
        *USART1_DR = temp & 0xFF;
    }

#endif

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

// 串口硬件初始化
static void uart_hardware_init()
{
    // 系统时钟HCLK
    extern uint32_t SystemCoreClock;

    // PCK2 倍频倍数
    uint8_t APBPrescTable[8U] = {0, 0, 0, 0, 1, 2, 3, 4};

    uint32_t temp = 0;
    // USARTDIV的整数部分
    uint32_t DIV_Mantissa = 0;
    // USARTDIV的小数部分
    uint32_t DIV_Fraction = 0;

    /* 启用时钟 */

    // 启用USART1时钟
    *RCC_APB2ENR |= (1U << 14);

    // 启用GPIO A时钟
    *RCC_APB2ENR |= (1U << 2);

    /* 配置gpio pin (确保GPIOA_CRH = 0x4bX)*/

    // 清除GPIO A9的配置位 bit4:7
    *GPIOA_CRH &= 0xFFFFFF0F;

    // 配置GPIO A9 输出模式为复用推挽输出(bit6:7)和输出速度为高(bit4:5)
    *GPIOA_CRH |= (3U << 4) | (2U << 6);

    // 清除GPIO A10的配置位 bit8:11
    *GPIOA_CRH &= 0xFFFFF0FF;

    // 配置GPIO A10 输出模式为浮空输入(bit10:11)和输入模式(bit8:9)
    *GPIOA_CRH |= (1U << 10);

    /*配置串口参数*/

    // 配置字长bit12、校验使能bit9、奇偶校验bit8、接受使能bit3、发送使能bit2
    // 清除位
    *USART1_CR1 &= (~((1U << 12) | (1U << 9) | (1 << 8) | (1 << 3) | (1 << 2)));

    // 配置位 bit2、3为1，其他位为0
    *USART1_CR1 |= ((1U << 2) | (1U << 3));

    // 配置字长(bit12:13) 一个停止位、禁用LIN模式bit14、禁用CLKEN bit11
    // 清除位
    *USART1_CR2 &= (~(3U << 12) | (1U << 11) | (1U << 14));

    // 配置禁用红外模式、半双工模式、智能卡模式
    // 清除位
    *USART1_CR3 &= (~(3U << 1) | (1U << 3) | (1U << 5));

    // 配置 USART_BRR寄存器 USART1时钟源PCLK2,先获取到PCLK2的分频率数
    temp = (*RCC_CFGR & (7U << 11)) >> 11;         // 分频倍数
    temp = SystemCoreClock >> APBPrescTable[temp]; // PCLK2 频率

    // 波特率计算公式 ：baud = fck/(16 * USARTDIV )，USARTDIV =fck / ( 16 * baud )

    // 计算整数部分， 将PCLK2 放大100倍数(HAL库)，用于提高精度
    DIV_Mantissa = (((temp * 25U) / (4U * 115200U)) / 100U);
    // 小数部分 *16是为了将小数转换成二进制数 加50是补偿(手动四舍五入)
    DIV_Fraction = ((((temp * 25U) / (4U * 115200U)) - DIV_Mantissa * 100U) * 16U + 50U) / 100U;
    *USART1_BRR = (DIV_Mantissa << 4) + DIV_Fraction;

#if uart_IT

    // 设置USART1 中断优先级 nvic 偏移值0x0000_00D4
    NVIC_SetPriority(37, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));

    // 启用中断
    NVIC_EnableIRQ(37);
#endif
    // 使能USART1
    *USART1_CR1 |= (1U << 13);
}
