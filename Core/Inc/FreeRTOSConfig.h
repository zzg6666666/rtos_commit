/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
  void xPortSysTickHandler(void);
#endif
#define configUSE_PREEMPTION                     1 //使用抢占式调度
#define configSUPPORT_STATIC_ALLOCATION          1 //静态分配内存
#define configSUPPORT_DYNAMIC_ALLOCATION         1 //动态分配内存
#define configUSE_IDLE_HOOK                      0 //使用空闲钩子(空闲任务会执行)
#define configUSE_TICK_HOOK                      0 //使用定时器钩子(xTaskIncrementTick会执行)
#define configCPU_CLOCK_HZ                       ( SystemCoreClock ) //CPU运行频率
#define configTICK_RATE_HZ                       ((TickType_t)1000)   //tick节拍，一秒的节拍次数
#define configMAX_PRIORITIES                     ( 56 ) //task的最大优先级
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128) //task的最小栈深度
#define configTOTAL_HEAP_SIZE                    ((size_t)3072*3)   //堆允许使用的最大容量
#define configMAX_TASK_NAME_LEN                  ( 16 ) //任务最长的名字
#define configUSE_TRACE_FACILITY                 1  //用于实现可视化追踪
#define configUSE_16_BIT_TICKS                   0  //使用16bit来计数tick的计数次数(false为32bit)
#define configUSE_MUTEXES                        1  //使用互斥量
#define configQUEUE_REGISTRY_SIZE                8  //可以注册和使用的队列和信号量最大数量
#define configUSE_RECURSIVE_MUTEXES              1  //使能递归互斥信号量
#define configUSE_COUNTING_SEMAPHORES            1  //使能计数信号量
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0  //内核移植相关

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0  //携程功能 croutine.c
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 ) //携程优先级

/* Software timer definitions. */
#define configUSE_TIMERS                         1  //软件定时器
#define configTIMER_TASK_PRIORITY                ( 2 ) //软件定时器优先级
#define configTIMER_QUEUE_LENGTH                 10 //软件定时器命令队列的长度
#define configTIMER_TASK_STACK_DEPTH             256 ///软件定时器栈深度

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            1 //重设任务优先级
#define INCLUDE_uxTaskPriorityGet           1 //获取任务优先级
#define INCLUDE_vTaskDelete                 1 //将任务从就绪列表中删除
#define INCLUDE_vTaskCleanUpResources       0 //未找到相关实现
#define INCLUDE_vTaskSuspend                1  //允许暂停任务
#define INCLUDE_vTaskDelayUntil             1  //精确延时任务,到时间立即唤醒
#define INCLUDE_vTaskDelay                  1  //延时任务
#define INCLUDE_xTaskGetSchedulerState      1  //允许获取调度器情况
#define INCLUDE_xTimerPendFunctionCall      1  //挂起任务到RTOS的守护进程?
#define INCLUDE_xQueueGetMutexHolder        1  //允许获取到互斥量的任务是谁
#define INCLUDE_uxTaskGetStackHighWaterMark 1  //检测任务堆栈使用情况
#define INCLUDE_eTaskGetState               1   //获取任务状态

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below
 */
#define USE_FreeRTOS_HEAP_4

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */

/* #define xPortSysTickHandler SysTick_Handler */

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
