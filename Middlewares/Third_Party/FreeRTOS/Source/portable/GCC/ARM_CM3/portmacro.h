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

#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C"
{
#endif

#define assembly_debug

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#define portCHAR char
#define portFLOAT float
#define portDOUBLE double
#define portLONG long
#define portSHORT short
#define portSTACK_TYPE uint32_t
#define portBASE_TYPE long

	typedef portSTACK_TYPE StackType_t;
	typedef long BaseType_t;
	typedef unsigned long UBaseType_t;

#if (configUSE_16_BIT_TICKS == 1)
	typedef uint16_t TickType_t;
#define portMAX_DELAY (TickType_t)0xffff
#else
typedef uint32_t TickType_t;
#define portMAX_DELAY (TickType_t)0xffffffffUL

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
not need to be guarded with a critical section. */
#define portTICK_TYPE_IS_ATOMIC 1
#endif
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH (-1)
#define portTICK_PERIOD_MS ((TickType_t)1000 / configTICK_RATE_HZ)
#define portBYTE_ALIGNMENT 8
/*-----------------------------------------------------------*/

/* Scheduler utilities. */
#define portYIELD()                                                                \
	{                                                                              \
		/* Set a PendSV to request a context switch. */                            \
		portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;                            \
                                                                                   \
		/* Barriers are normally not required but do ensure the code is completely \
		within the specified behaviour for the architecture. */                    \
		__asm volatile("dsb" ::: "memory");                                        \
		__asm volatile("isb");                                                     \
	}

#define portNVIC_INT_CTRL_REG (*((volatile uint32_t *)0xe000ed04))
#define portNVIC_PENDSVSET_BIT (1UL << 28UL)
#define portEND_SWITCHING_ISR(xSwitchRequired) \
	if (xSwitchRequired != pdFALSE)            \
	portYIELD()
#define portYIELD_FROM_ISR(x) portEND_SWITCHING_ISR(x)
	/*-----------------------------------------------------------*/

	/* Critical section management. */
	extern void vPortEnterCritical(void);
	extern void vPortExitCritical(void);
#define portSET_INTERRUPT_MASK_FROM_ISR() ulPortRaiseBASEPRI()
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x) vPortSetBASEPRI(x)
#define portDISABLE_INTERRUPTS() vPortRaiseBASEPRI()
#define portENABLE_INTERRUPTS() vPortSetBASEPRI(0)
#define portENTER_CRITICAL() vPortEnterCritical()
#define portEXIT_CRITICAL() vPortExitCritical()

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
not necessary for to use this port.  They are defined so the common demo files
(which build with all the ports) will build. */
#define portTASK_FUNCTION_PROTO(vFunction, pvParameters) void vFunction(void *pvParameters)
#define portTASK_FUNCTION(vFunction, pvParameters) void vFunction(void *pvParameters)
/*-----------------------------------------------------------*/

/* Tickless idle/low power functionality. */
#ifndef portSUPPRESS_TICKS_AND_SLEEP
	extern void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime);
#define portSUPPRESS_TICKS_AND_SLEEP(xExpectedIdleTime) vPortSuppressTicksAndSleep(xExpectedIdleTime)
#endif
/*-----------------------------------------------------------*/

/* Architecture specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

	/* Generic helper function. */
	__attribute__((always_inline)) static inline uint8_t ucPortCountLeadingZeros(uint32_t ulBitmap)
	{
		uint8_t ucReturn;

		__asm volatile("clz %0, %1" : "=r"(ucReturn) : "r"(ulBitmap) : "memory");
		return ucReturn;
	}

/* Check the configuration. */
#if (configMAX_PRIORITIES > 32)
#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
#endif

/* Store/clear the ready priorities in a bit map. */
#define portRECORD_READY_PRIORITY(uxPriority, uxReadyPriorities) (uxReadyPriorities) |= (1UL << (uxPriority))
#define portRESET_READY_PRIORITY(uxPriority, uxReadyPriorities) (uxReadyPriorities) &= ~(1UL << (uxPriority))

	/*-----------------------------------------------------------*/

#define portGET_HIGHEST_PRIORITY(uxTopPriority, uxReadyPriorities) uxTopPriority = (31UL - (uint32_t)ucPortCountLeadingZeros((uxReadyPriorities)))

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

	/*-----------------------------------------------------------*/

#ifdef configASSERT
	void vPortValidateInterruptPriority(void);
#define portASSERT_IF_INTERRUPT_PRIORITY_INVALID() vPortValidateInterruptPriority()
#endif

/* portNOP() is not required by this port. */
#define portNOP()

#define portINLINE __inline

#ifndef portFORCE_INLINE
#define portFORCE_INLINE inline __attribute__((always_inline))
#endif

	// 检查当前中断是不是内部中断
	portFORCE_INLINE static BaseType_t xPortIsInsideInterrupt(void)
	{
		uint32_t ulCurrentInterrupt;
		BaseType_t xReturn;

		// 将ipsr寄存器的值移动到ulCurrentInterrupt中
		/* Obtain the number of the currently executing interrupt. */
		__asm volatile("mrs %0, ipsr" : "=r"(ulCurrentInterrupt)::"memory"); //:"memory"相当于volatile

		if (ulCurrentInterrupt == 0)
		{
			xReturn = pdFALSE;
		}
		else
		{
			xReturn = pdTRUE;
		}

		return xReturn;
	}

	/*-----------------------------------------------------------*/

#ifdef assembly_debug

	// only debug ,将汇编指令拆开可能会影响原子性

	// 设置basepri值
	portFORCE_INLINE static void vPortRaiseBASEPRI(void)
	{
		// 申请一个寄存器，用于储存configMAX_SYSCALL_INTERRUPT_PRIORITY
		uint32_t ulNewBASEPRI;

		// 将configMAX_SYSCALL_INTERRUPT_PRIORITY写入到申请的寄存器里面 "=r" ：将结果储存到通用寄存器中
		__asm volatile("mov %0, %1" : "=r"(ulNewBASEPRI) : "i"(configMAX_SYSCALL_INTERRUPT_PRIORITY) : "memory");

		// 将申请的寄存器的值复制到basepri中 "r" : 表示通用寄存器
		__asm volatile("msr basepri, %0 " : : "r"(ulNewBASEPRI) : "memory");

		// 假设申请的是r3，configMAX_SYSCALL_INTERRUPT_PRIORITY = 0x20 , 以上指令变成汇编：
		// mov r3 0x20;msr basepri r3
		// 等待操作完成
		__asm volatile("isb\n dsb\n");
	}
#else
portFORCE_INLINE static void vPortRaiseBASEPRI(void)
{
	uint32_t ulNewBASEPRI;

	__asm volatile(
		"	mov %0, %1												\n"
		"	msr basepri, %0											\n"
		"	isb														\n"
		"	dsb														\n"
		: "=r"(ulNewBASEPRI) : "i"(configMAX_SYSCALL_INTERRUPT_PRIORITY) : "memory");
}
#endif

	/*-----------------------------------------------------------*/

	// 将值basepri赋值出，给basepri设置(最大优先级)
	portFORCE_INLINE static uint32_t ulPortRaiseBASEPRI(void)
	{
		uint32_t ulOriginalBASEPRI, ulNewBASEPRI;

		__asm volatile(
			"	mrs %0, basepri											\n" // ulOriginalBASEPRI = basepri
			"	mov %1, %2												\n" // ulNewBASEPRI =  	configMAX_SYSCALL_INTERRUPT_PRIORITY
			"	msr basepri, %1											\n" // basepri = ulNewBASEPRI
			"	isb														\n"
			"	dsb														\n"
			: "=r"(ulOriginalBASEPRI), "=r"(ulNewBASEPRI) : "i"(configMAX_SYSCALL_INTERRUPT_PRIORITY) : "memory");

		// mrs r0 basepri
		// mov r4 0x20
		// msr basepri r4

		//返回值无用，仅仅是为了防止编译器警告
		/* This return will not be reached but is necessary to prevent compiler
		warnings. */
		return ulOriginalBASEPRI;
	}
	/*-----------------------------------------------------------*/

//设定basepri 给指定 的值
	portFORCE_INLINE static void vPortSetBASEPRI(uint32_t ulNewMaskValue)
	{	
		//msr basepri r0
		__asm volatile(
			"	msr basepri, %0	" ::"r"(ulNewMaskValue) : "memory");
	}
	/*-----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */
