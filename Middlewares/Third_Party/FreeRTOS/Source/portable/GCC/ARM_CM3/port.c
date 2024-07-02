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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM3 port.
 *----------------------------------------------------------*/

#define assembly_debug

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* For backward compatibility, ensure configKERNEL_INTERRUPT_PRIORITY is
defined.  The value should also ensure backward compatibility.
FreeRTOS.org versions prior to V4.4.0 did not include this definition. */
#ifndef configKERNEL_INTERRUPT_PRIORITY
#define configKERNEL_INTERRUPT_PRIORITY 255
#endif

#ifndef configSYSTICK_CLOCK_HZ
#define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
/* Ensure the SysTick is clocked at the same frequency as the core. */
#define portNVIC_SYSTICK_CLK_BIT (1UL << 2UL)
#else
/* The way the SysTick is clocked is not modified in case it is not the same
as the core. */
#define portNVIC_SYSTICK_CLK_BIT (0)
#endif

/* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG (*((volatile uint32_t *)0xe000e010))
#define portNVIC_SYSTICK_LOAD_REG (*((volatile uint32_t *)0xe000e014))
#define portNVIC_SYSTICK_CURRENT_VALUE_REG (*((volatile uint32_t *)0xe000e018))
#define portNVIC_SYSPRI2_REG (*((volatile uint32_t *)0xe000ed20))
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_INT_BIT (1UL << 1UL)
#define portNVIC_SYSTICK_ENABLE_BIT (1UL << 0UL)
#define portNVIC_SYSTICK_COUNT_FLAG_BIT (1UL << 16UL)
#define portNVIC_PENDSVCLEAR_BIT (1UL << 27UL)
#define portNVIC_PEND_SYSTICK_CLEAR_BIT (1UL << 25UL)

#define portNVIC_PENDSV_PRI (((uint32_t)configKERNEL_INTERRUPT_PRIORITY) << 16UL)
#define portNVIC_SYSTICK_PRI (((uint32_t)configKERNEL_INTERRUPT_PRIORITY) << 24UL)

/* Constants required to check the validity of an interrupt priority. */
#define portFIRST_USER_INTERRUPT_NUMBER (16) // 用户定义的第一个中断号(外设 非cortex内核)
#define portNVIC_IP_REGISTERS_OFFSET_16 (0xE000E3F0)
#define portAIRCR_REG (*((volatile uint32_t *)0xE000ED0C))
#define portMAX_8_BIT_VALUE ((uint8_t)0xff)
#define portTOP_BIT_OF_BYTE ((uint8_t)0x80)
#define portMAX_PRIGROUP_BITS ((uint8_t)7)
#define portPRIORITY_GROUP_MASK (0x07UL << 8UL)
#define portPRIGROUP_SHIFT (8UL)

/* Masks off all bits but the VECTACTIVE bits in the ICSR register. */
#define portVECTACTIVE_MASK (0xFFUL)

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR (0x01000000UL)

/* The systick is a 24-bit counter. */
#define portMAX_24_BIT_NUMBER (0xffffffUL)

/* A fiddle factor to estimate the number of SysTick counts that would have
occurred while the SysTick counter is stopped during tickless idle
calculations. */
#define portMISSED_COUNTS_FACTOR (45UL)

/* For strict compliance with the Cortex-M spec the task start address should
have bit-0 clear, as it is loaded into the PC on exit from an ISR. */
#define portSTART_ADDRESS_MASK ((StackType_t)0xfffffffeUL)

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#ifdef configTASK_RETURN_ADDRESS
#define portTASK_RETURN_ADDRESS configTASK_RETURN_ADDRESS
#else
#define portTASK_RETURN_ADDRESS prvTaskExitError
#endif

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt(void);

/*
 * Exception handlers.
 */
void xPortPendSVHandler(void) __attribute__((naked));
void xPortSysTickHandler(void);
void vPortSVCHandler(void) __attribute__((naked));

/*
 * Start first task is a separate function so it can be tested in isolation.
 开始的第一项任务是一个单独的功能，因此可以单独测试。
 __attribute__ (( naked )) 是gcc的扩展语法，用于告诉编译器生成的函数不要包含默认
 的函数入口和退出代码(保存和恢复寄存器、调用)
 */
static void prvPortStartFirstTask(void) __attribute__((naked));

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError(void);

/*-----------------------------------------------------------*/

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
 * The number of SysTick increments that make up one tick period.
 */
#if (configUSE_TICKLESS_IDLE == 1)
static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if (configUSE_TICKLESS_IDLE == 1)
static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if (configUSE_TICKLESS_IDLE == 1)
static uint32_t ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Used by the portASSERT_IF_INTERRUPT_PRIORITY_INVALID() macro to ensure
 * FreeRTOS API functions are not called from interrupts that have been assigned
 * a priority above configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#if (configASSERT_DEFINED == 1)
static uint8_t ucMaxSysCallPriority = 0;
static uint32_t ulMaxPRIGROUPValue = 0;
static const volatile uint8_t *const pcInterruptPriorityRegisters = (const volatile uint8_t *const)portNVIC_IP_REGISTERS_OFFSET_16;
#endif /* configASSERT_DEFINED */

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
// 初始化程序堆栈，同时伪造寄存器
StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters)
{
	// 在进入中断的时候，m3内核按照以下的顺序保存 xpsr、PC(R15)、lR(R14)、r12、r3、r2、r0,剩下的寄存器需要手动保存(除了SP寄存器(R13))
	/* Simulate the stack frame as it would be created by a context switch
	interrupt.
	模拟上下文切换中断创建的堆栈帧。保存所有必要的寄存器
	*/
	pxTopOfStack--;					  /* Offset added to account for the way the MCU uses the stack on entry/exit of interrupts.增加偏移量，以考虑 MCU 在进入/退出中断时使用堆栈的方式 在进行字节对齐有损失,跳过不满32字节的位置? */
	*pxTopOfStack = portINITIAL_XPSR; /* xPSR */
	pxTopOfStack--;
	*pxTopOfStack = ((StackType_t)pxCode) & portSTART_ADDRESS_MASK; /* PC (R15)程序计数寄存器，中断执行任务切换后，执行的第一条指令 */
	pxTopOfStack--;
	*pxTopOfStack = (StackType_t)portTASK_RETURN_ADDRESS; /* LR (R14)伪造的返回地址， 当程序意外退出后执行的错误处理*/
	pxTopOfStack -= 5;									  /* R12, R3, R2 and R1. */
	*pxTopOfStack = (StackType_t)pvParameters;			  /* R0 程序的参数，ARM规定传参只能用R0 - R3，如果超出4个寄存器，将会用堆栈传递参数*/
	pxTopOfStack -= 8;									  /*手动保存的寄存器 R11, R10, R9, R8, R7, R6, R5 and R4. */

	// 更新栈指针，在svc或者pendsv函数进行函数切换时，从这里开始恢复寄存器(低地址向高地址出栈)
	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

// 用于伪造任务的返回地址，
static void prvTaskExitError(void)
{
	volatile uint32_t ulDummy = 0UL;

	/* A function that implements a task must not exit or attempt to return to
	its caller as there is nothing to return to.  If a task wants to exit it
	should instead call vTaskDelete( NULL ).
	实现任务对的函数必须不能返回或者企图返回到他的调用者，因为这里没有什么是可以返回的，
	如果一个任务想要退出，应该使用vTaskDelete替代

	Artificially force an assert() to be triggered if configASSERT() is
	defined, then stop here so application writers can catch the error.

	手动强制触发断言，如果定义了configASSERT()。断言错误，停在这，以便开发者检查错误
	 */
	configASSERT(uxCriticalNesting == ~0UL);
	// 关闭中断
	portDISABLE_INTERRUPTS();

	while (ulDummy == 0)
	{
		/* This file calls prvTaskExitError() after the scheduler has been
		started to remove a compiler warning about the function being defined
		but never called.  ulDummy is used purely to quieten other warnings
		about code appearing after this function is called - making ulDummy
		volatile makes the compiler think the function could return and
		therefore not output an 'unreachable code' warning for code that appears
		after it.
		用于避开编译器警告
		 */
	}
}
/*-----------------------------------------------------------*/

#ifdef assembly_debug
// svc中断函数，用于开启第一个任务切换(一条指令一行方便debug)
void vPortSVCHandler(void)
{
	// 将标签处pxCurrentTCBConst2，储存的pxCurrentTCB的地址加载到r3寄存器
	__asm volatile("ldr	r3, pxCurrentTCBConst2");
#if 0
	//和这个一样
	__asm volatile("ldr	r3, =pxCurrentTCB");
#endif
	// 将pxCurrentTCB的地址加载到r1寄存器
	__asm volatile("ldr r1, [r3]");
	// 将任务代码栈的堆栈地址加载到r0寄存器
	__asm volatile("ldr r0, [r1]");
	// 从R0的地址指向的值，依次递增4个字节，加载到r4-r11(手动保存的寄存器)中
	__asm volatile("ldmia r0!, {r4-r11}");
	// 将r0的地址(任务栈代码栈出栈相关寄存器的内存地址)加载到psp(进程栈指针 R13)寄存器(恢复进程后，会自动出栈xpsr、PC(R15)、lR(R14)、r12、r3、r2、r0,))
	__asm volatile("msr psp, r0");
	// 刷新流水线，等待前面的指令执行完成
	__asm volatile("isb	");
	// 给r0寄存器赋值为0
	__asm volatile("mov r0, #0 	");
	// 加载0到basepri寄存器中，打开所有中断
	__asm volatile("msr	basepri, r0	");
	// 设置lr(r14)寄存器值，R14  = R14 | 1101b ,R14(LR)= 0xfffffffd,从0xFFFFFFE9(返回线程模式并在返回后使用主栈)变成0xFFFFFFED(返回处理模式并在返回后使用进程栈)
	__asm volatile("orr r14, #0xd	");
	// 中断返回EXC_RETURN，从PSP恢复堆栈
	__asm volatile("bx r14	");
	// 采用4字节对齐
	__asm volatile(".align 4	");
	// 定义pxCurrentTCBConst2标签，用于储存pxCurrentTCB的位置
	__asm volatile("pxCurrentTCBConst2: .word pxCurrentTCB	");

#else
__asm volatile("	ldr	r3, pxCurrentTCBConst2		\n" /* Restore the context. */
			   "	ldr r1, [r3]					\n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
			   "	ldr r0, [r1]					\n" /* The first item in pxCurrentTCB is the task top of stack. */
			   "	ldmia r0!, {r4-r11}				\n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
			   "	msr psp, r0						\n" /* Restore the task stack pointer. */
			   "	isb								\n"
			   "	mov r0, #0 						\n"
			   "	msr	basepri, r0					\n"
			   "	orr r14, #0xd					\n"
			   "	bx r14							\n"
			   "									\n"
			   "	.align 4						\n"
			   "pxCurrentTCBConst2: .word pxCurrentTCB				\n");
#endif
}
/*-----------------------------------------------------------*/

// 执行第一个任务
static void prvPortStartFirstTask(void)
{
	__asm volatile(
		// r0 = 0xE000ED08
		" ldr r0, =0xE000ED08 	\n" /* Use the NVIC offset register to locate the stack.使用向量表偏移量寄存器去定位向量表，每个中断向量占比为4字节 */
		// r0 = 0x00
		" ldr r0, [r0] 			\n" /* 将中断向量表中，第一个向量MSP的地址加到R0寄存器(中断向量表中，储存的是异常/向量的地址)*/
		// r0 = 0x20005000
		" ldr r0, [r0] 			\n" /* 将MSP的初始值加载到R0寄存器 */
		// msp = 0x20005000
		" msr msp, r0			\n"	  /* Set the msp back to the start of the stack. 将主堆栈指针（MSP）的初始值加载到msp寄存器中 */
		" cpsie i				\n"	  /* Globally enable interrupts. 打开isr中断(basepri 寄存器) */
		" cpsie f				\n"	  /* 打开fault异常(PRIMASK和FAULTMASK 寄存器) */
		" dsb					\n"	  /* 数据同步隔离 */
		" isb					\n"	  /* 指令同步隔离 */
		" svc 0					\n"	  /* System call to start first task. 调用svc异常开始第一个任务*/
		" nop					\n"); // 不执行任何东西，消耗一个指令周期
}
/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
// 启动调度器
BaseType_t xPortStartScheduler(void)
{
	/* configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.
	See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html */
	configASSERT(configMAX_SYSCALL_INTERRUPT_PRIORITY);
	// 是否开启了断言 (用于debug)
#if (configASSERT_DEFINED == 1)
	{
		// 当前原始优先级
		volatile uint32_t ulOriginalPriority;
		// 中断优先级寄存器的地址
		volatile uint8_t *const pucFirstUserPriorityRegister = (uint8_t *)(portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER);
		// 最大优先级
		volatile uint8_t ucMaxPriorityValue;

		/*用于确定中断安全API能够使用的最高的ISR优先级，在这个ISR或者更低优先级的函数能够安全的调用FromISR结尾的函数功能*/
		/* Determine the maximum priority from which ISR safe FreeRTOS API
		functions can be called.  ISR safe functions are those that end in
		"FromISR".  FreeRTOS maintains separate thread and ISR API functions to
		ensure interrupt entry is as fast and simple as possible.

		//保存中断优先级
		Save the interrupt priority value that is about to be clobbered. */
		ulOriginalPriority = *pucFirstUserPriorityRegister;

		// 确定有效的中断优先级个数，往中断寄存器写值，未使用的寄存器位，回读为0
		/* Determine the number of priority bits available.  First write to all
		possible bits. */
		*pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

		// 回读寄存器，确定最高优先级
		/* Read the value back to see how many bits stuck. */
		ucMaxPriorityValue = *pucFirstUserPriorityRegister;

		// 对最大系统调用优先级使用相同的掩码
		/* Use the same mask on the maximum system call priority. */
		ucMaxSysCallPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY & ucMaxPriorityValue;

		/* Calculate the maximum acceptable priority group value for the number
		of bits read back. */
		// 计算回读位数的可接受的最大优先级组值
		ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;
		while ((ucMaxPriorityValue & portTOP_BIT_OF_BYTE) == portTOP_BIT_OF_BYTE)
		{
			ulMaxPRIGROUPValue--;
			ucMaxPriorityValue <<= (uint8_t)0x01;
		}

#ifdef __NVIC_PRIO_BITS
		{
			/* Check the CMSIS configuration that defines the number of
			priority bits matches the number of priority bits actually queried
			from the hardware. */
			configASSERT((portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue) == __NVIC_PRIO_BITS);
		}
#endif

#ifdef configPRIO_BITS
		{
			// 检查FreeRTOS配置，该配置定义的优先级比特数与实际从硬件查询的优先级比特数匹配。
			/* Check the FreeRTOS configuration that defines the number of
			priority bits matches the number of priority bits actually queried
			from the hardware. */
			configASSERT((portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue) == configPRIO_BITS);
		}
#endif

		/* Shift the priority group value back to its position within the AIRCR
		register. */
		ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
		ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;

		/* Restore the clobbered interrupt priority register to its original
		value. */
		// 恢复实际优先级
		*pucFirstUserPriorityRegister = ulOriginalPriority;
	}
#endif /* conifgASSERT_DEFINED */

	/* Make PendSV and SysTick the lowest priority interrupts. */
	// 设置成最低优先级
	portNVIC_SYSPRI2_REG |= portNVIC_PENDSV_PRI;
	portNVIC_SYSPRI2_REG |= portNVIC_SYSTICK_PRI;

	/* Start the timer that generates the tick ISR.  Interrupts are disabled
	here already. */
	// 开始计时器，用于生成tick中断，中断早已被禁用
	vPortSetupTimerInterrupt();

	/* Initialise the critical nesting count ready for the first task. */
	// 初始化临界区计数,用于第一个任务
	uxCriticalNesting = 0;

	/* Start the first task. */
	// 开启第一个任务
	prvPortStartFirstTask();

	/* 应该永远不会到达这里因为任务正在执行! 调用任务退出错误函数，预防编译器警告
	静态函数没有被调用,当configTASK_RETURN_ADDRESS被定义的情况下。调用
	vTaskSwitchContext(),以便链接优化时不会移除该符号
	Should never get here as the tasks will now be executing!  Call the task
	exit error function to prevent compiler warnings about a static function
	not being called in the case that the application writer overrides this
	functionality by defining configTASK_RETURN_ADDRESS.  Call
	vTaskSwitchContext() so link time optimisation does not remove the
	symbol. */
	vTaskSwitchContext();
	prvTaskExitError();

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

// 结束调度器
void vPortEndScheduler(void)
{
	/* Not implemented in ports where there is nothing to return to.
	Artificially force an assert. */
	configASSERT(uxCriticalNesting == 1000UL);
}
/*-----------------------------------------------------------*/

// 进入临界区，不让rtos打断任务
void vPortEnterCritical(void)
{
	portDISABLE_INTERRUPTS();
	uxCriticalNesting++;

	/* This is not the interrupt safe version of the enter critical function so
	assert() if it is being called from an interrupt context.  Only API
	functions that end in "FromISR" can be used in an interrupt.  Only assert if
	the critical nesting count is 1 to protect against recursive calls if the
	assert function also uses a critical section. */
	if (uxCriticalNesting == 1)
	{
		configASSERT((portNVIC_INT_CTRL_REG & portVECTACTIVE_MASK) == 0);
	}
}
/*-----------------------------------------------------------*/

// 退出临界区
void vPortExitCritical(void)
{
	configASSERT(uxCriticalNesting);
	uxCriticalNesting--;
	if (uxCriticalNesting == 0)
	{
		portENABLE_INTERRUPTS();
	}
}
/*-----------------------------------------------------------*/

#ifdef assembly_debug
// 保存现场，恢复现场 方便debug
void xPortPendSVHandler(void)
{
	/* This is a naked function. */
	// 触发了pendsv中断后，sp寄存器切换到msp
	// 自动存储了，xpsr、PC(R15)、lR(R14)、r12、r3、r2、r0,))到代码栈

	// 将PSP储存到r0寄存器
	__asm volatile(" mrs r0, psp                 ");
	// 等待流水线刷新
	__asm volatile(" isb                         ");
	// 将储存了pxCurrentTCB的地址的地址加载到r3
	__asm volatile(" ldr	r3, pxCurrentTCBConst");
	// 将pxCurrentTCB的地址的地址加载到r2
	__asm volatile(" ldr	r2, [r3]             ");
	// 手动保存r4-r11寄存器到psp代码栈
	__asm volatile(" stmdb r0!, {r4-r11}		 ");
	// 存储新的栈顶(PSP)到TCB的栈指针，栈顶新增的项为手动保存的寄存器
	__asm volatile(" str r0, [r2]				 ");
	/*保存r3(储存了pxCurrentTCB的地址的地址) 和 r14(lr)到 msp栈指针，
	在执行vTaskSwitchContext时，可能会使用到r3寄存器，以及会使用到lr寄
	存器返回到该函数，（但不会进行入栈和出栈），(lr寄存器保存的是结束中断
	的0xfffffffd)，在跳转回该函	数后需要恢复r3 和 R14(lr)，用于更新上
	下文和结束中断，切换成线程模式*/
	__asm volatile(" stmdb sp!, {r3, r14}		 ");

	// 屏蔽中断
	__asm volatile(" mov r0, %0				 " ::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY)); //	mov r0, #configMAX_SYSCALL_INTERRUPT_PRIORITY
	__asm volatile(" msr basepri, r0			 ");

	// 执行vTaskSwitchContext,更新pxCurrentTCB
	__asm volatile(" bl vTaskSwitchContext		 ");

	// 恢复中断优先级
	__asm volatile(" mov r0, #0					 ");
	__asm volatile(" msr basepri, r0			 ");

	// 恢复r3(储存了pxCurrentTCB的地址的地址) 和 r14(lr)
	__asm volatile(" ldmia sp!, {r3, r14}	     ");

	// 恢复上下文 加载pxCurrentTCB的地址到r1寄存器
	__asm volatile(" ldr r1, [r3] 				 ");
	// 从pxCurrentTCB取出用户代码栈地址(PSP)到r0
	__asm volatile(" ldr r0, [r1]				 ");
	// 恢复r4 - r11
	__asm volatile(" ldmia r0!, {r4-r11}		 ");
	// 将用户代码栈地址更新到PSP
	__asm volatile(" msr psp, r0				 ");
	// 等待刷新流水线
	__asm volatile(" isb						 ");
	// 返回到线程模式，自动恢复xpsr、PC(R15)、lR(R14)、r12、r3、r2、r0,))
	__asm volatile(" bx r14						 ");
	// 4字节对齐
	__asm volatile(" .align 4					 ");
	// pxCurrentTCBConst链接到pxCurrentTCB
	__asm volatile("pxCurrentTCBConst: .word pxCurrentTCB");
}
/*-----------------------------------------------------------*/

#else
// 保存现场，恢复现场
void xPortPendSVHandler(void)
{
	/* This is a naked function. */

	__asm volatile(
		"	mrs r0, psp							\n"
		"	isb									\n"
		"										\n"
		"	ldr	r3, pxCurrentTCBConst			\n" /* Get the location of the current TCB. */
		"	ldr	r2, [r3]						\n"
		"										\n"
		"	stmdb r0!, {r4-r11}					\n" /* Save the remaining registers. */
		"	str r0, [r2]						\n" /* Save the new top of stack into the first member of the TCB. */
		"										\n"
		"	stmdb sp!, {r3, r14}				\n"
		"	mov r0, %0							\n"
		"	msr basepri, r0						\n"
		"	bl vTaskSwitchContext				\n"
		"	mov r0, #0							\n"
		"	msr basepri, r0						\n"
		"	ldmia sp!, {r3, r14}				\n"
		"										\n" /* Restore the context, including the critical nesting count. */
		"	ldr r1, [r3]						\n"
		"	ldr r0, [r1]						\n" /* The first item in pxCurrentTCB is the task top of stack. */
		"	ldmia r0!, {r4-r11}					\n" /* Pop the registers. */
		"	msr psp, r0							\n"
		"	isb									\n"
		"	bx r14								\n"
		"										\n"
		"	.align 4							\n"
		"pxCurrentTCBConst: .word pxCurrentTCB	\n" ::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY));
}
/*-----------------------------------------------------------*/

#endif
// 每一节拍进入一次 Systick 中断，进行任务切换
void xPortSysTickHandler(void)
{
	/* The SysTick runs at the lowest interrupt priority, so when this interrupt
	executes all interrupts must be unmasked.  There is therefore no need to
	save and then restore the interrupt mask value as its value is already
	known.
	 SysTick运行在最低优先级的中断，因此当systick中断执行的时候，所有的中断都必须被屏蔽
	*/

	// 关闭中断
	portDISABLE_INTERRUPTS();
	{
		// 增加RTOS tick计数
		/* Increment the RTOS tick. */
		if (xTaskIncrementTick() != pdFALSE)
		{
			/* A context switch is required.  Context switching is performed in
			the PendSV interrupt.  Pend the PendSV interrupt. */
			// nvic寄存器启动上下文切换，在pendsv中进行(中断控制及状态寄存器ICSR bit28)
			portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
		}
	}
	// 重启中断
	portENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/

// 启用低功耗
#if (configUSE_TICKLESS_IDLE == 1)

// 进入低功耗模式
__attribute__((weak)) void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
	uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
	TickType_t xModifiableIdleTime;

	/* Make sure the SysTick reload value does not overflow the counter. */
	// 确保滴答计时器不会溢出，期待进入低功耗的时间值，不会超过最大允许的计数值

	if (xExpectedIdleTime > xMaximumPossibleSuppressedTicks)
	{
		xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
	}

	/* Stop the SysTick momentarily.  The time the SysTick is stopped for
	is accounted for as best it can be, but using the tickless mode will
	inevitably result in some tiny drift of the time maintained by the
	kernel with respect to calendar time. */
	// 暂时停止滴答定时器，使用低功耗模式不可避免的会导致内核时间相较于日历时间出现一些轻微的偏移

	portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

	/* Calculate the reload value required to wait xExpectedIdleTime
	tick periods.  -1 is used because this code will execute part way
	through one of the tick periods. */
	//根据xExpectedIdleTime的值来计算重装值,

	//从当前计数寄存器读出值 + 单周期的节拍数 * 期望休眠的时间长度
	ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + (ulTimerCountsForOneTick * (xExpectedIdleTime - 1UL));

	//补偿时间
	if (ulReloadValue > ulStoppedTimerCompensation)
	{
		ulReloadValue -= ulStoppedTimerCompensation;
	}

	/* Enter a critical section but don't use the taskENTER_CRITICAL()
	method as that will mask interrupts that should exit sleep mode. */
	//关闭中断
	__asm volatile("cpsid i" ::: "memory");
	__asm volatile("dsb");
	__asm volatile("isb");

	/* If a context switch is pending or a task is waiting for the scheduler
	to be unsuspended then abandon the low power entry. */
	//如果任务切换被推迟或者是任务在等待调度器回复，将不会进如低功耗模式
	if (eTaskConfirmSleepModeStatus() == eAbortSleep)
	{
		/* Restart from whatever is left in the count register to complete
		this tick period. */
		//设置load寄存器，完成该周期剩下的计时
		portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

		/* Restart SysTick. */
		//启用systick
		portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

		/* Reset the reload register to the value required for normal tick
		periods. */
		//重设load寄存器，回归正常的计时
		portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

		/* Re-enable interrupts - see comments above the cpsid instruction()
		above. */
		//打开中断
		__asm volatile("cpsie i" ::: "memory");
	}
	//可以进入休眠模式
	else
	{
		/* Set the new reload value. */
		//设置load寄存器为休眠的计数tick数据
		portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

		/* Clear the SysTick count flag and set the count value back to
		zero. */
		//清除当前计数值
		portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

		/* Restart SysTick. */
		//启用tick
		portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

		/* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
		set its parameter to 0 to indicate that its implementation contains
		its own wait for interrupt or wait for event instruction, and so wfi
		should not be executed again.  However, the original expected idle
		time variable must remain unmodified, so a copy is taken. 
		休眠直到某些事情的出现，configPRE_SLEEP_PROCESSING()用于配置参数为0,表示
		参数包含它自己等待中断或者事件指令，然后wfi将不会再次执行。然而，原始预期休眠
		事件不能被修改，因此需要一个副本*/
		
		xModifiableIdleTime = xExpectedIdleTime;
		//配置休眠模式
		configPRE_SLEEP_PROCESSING(&xModifiableIdleTime);
		if (xModifiableIdleTime > 0)
		{
			__asm volatile("dsb" ::: "memory");
			//进入待机状态
			__asm volatile("wfi");
			__asm volatile("isb");
		}

		//执行到这里已经退出休眠模式，进行恢复休眠模式相关
		configPOST_SLEEP_PROCESSING(&xExpectedIdleTime);

		/* Re-enable interrupts to allow the interrupt that brought the MCU
		out of sleep mode to execute immediately.  see comments above
		__disable_interrupt() call above. */
		//重新打开中断，让mcu在退出睡眠模式后，可以执行
		__asm volatile("cpsie i" ::: "memory");
		__asm volatile("dsb");
		__asm volatile("isb");

		/* Disable interrupts again because the clock is about to be stopped
		and interrupts that execute while the clock is stopped will increase
		any slippage between the time maintained by the RTOS and calendar
		time. */
		//关闭中断，因为时钟将要停止,中断将会导致计数值和真实值出现偏差
		__asm volatile("cpsid i" ::: "memory");
		__asm volatile("dsb");
		__asm volatile("isb");

		/* Disable the SysTick clock without reading the
		portNVIC_SYSTICK_CTRL_REG register to ensure the
		portNVIC_SYSTICK_COUNT_FLAG_BIT is not cleared if it is set.  Again,
		the time the SysTick is stopped for is accounted for as best it can
		be, but using the tickless mode will inevitably result in some tiny
		drift of the time maintained by the kernel with respect to calendar
		time*/
		//停止计时,
		portNVIC_SYSTICK_CTRL_REG = (portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT);

		/* Determine if the SysTick clock has already counted to zero and
		been set back to the current reload value (the reload back being
		correct for the entire expected idle time) or if the SysTick is yet
		to count to zero (in which case an interrupt other than the SysTick
		must have brought the system out of sleep mode). */
		//确定退出休眠模式的原因,是tick计数到0还是外部中断
		if ((portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT) != 0)
		{
			uint32_t ulCalculatedLoadValue;

			/* The tick interrupt is already pending, and the SysTick count
			reloaded with ulReloadValue.  Reset the
			portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
			period. */
			ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL) - (ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG);

			/* Don't allow a tiny value, or values that have somehow
			underflowed because the post sleep hook did something
			that took too long. */
			if ((ulCalculatedLoadValue < ulStoppedTimerCompensation) || (ulCalculatedLoadValue > ulTimerCountsForOneTick))
			{
				ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL);
			}

			portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

			/* As the pending tick will be processed as soon as this
			function exits, the tick value maintained by the tick is stepped
			forward by one less than the time spent waiting. */
			ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
		}
		else
		{
			/* Something other than the tick interrupt ended the sleep.
			Work out how long the sleep lasted rounded to complete tick
			periods (not the ulReload value which accounted for part
			ticks). */
			ulCompletedSysTickDecrements = (xExpectedIdleTime * ulTimerCountsForOneTick) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

			/* How many complete tick periods passed while the processor
			was waiting? */
			ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

			/* The reload value is set to whatever fraction of a single tick
			period remains. */
			portNVIC_SYSTICK_LOAD_REG = ((ulCompleteTickPeriods + 1UL) * ulTimerCountsForOneTick) - ulCompletedSysTickDecrements;
		}

		/* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
		again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
		value. */
		portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
		portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
		vTaskStepTick(ulCompleteTickPeriods);
		portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

		/* Exit with interrpts enabled. */
		__asm volatile("cpsie i" ::: "memory");
	}
}

#endif /* configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/

/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
// 配置操作系统的心跳(SysTick 相关寄存器)
__attribute__((weak)) void vPortSetupTimerInterrupt(void)
{
/* Calculate the constants required to configure the tick interrupt. */
#if (configUSE_TICKLESS_IDLE == 1)
	{
		ulTimerCountsForOneTick = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ);
		xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
		ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);
	}
#endif /* configUSE_TICKLESS_IDLE */

	/* Stop and clear the SysTick. */
	// 停止SysTick计时器
	portNVIC_SYSTICK_CTRL_REG = 0UL;
	// 将计数器清0
	portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

	/* Configure SysTick to interrupt at the requested rate. */
	// SysTick重装载数值寄存器
	portNVIC_SYSTICK_LOAD_REG = (configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ) - 1UL;
	// SysTick控制及状态寄存器,设置计数时间
	portNVIC_SYSTICK_CTRL_REG = (portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT | portNVIC_SYSTICK_ENABLE_BIT);
}
/*-----------------------------------------------------------*/

#if (configASSERT_DEFINED == 1)

// 验证当前的中断等级是否合法，预防中断嵌套 没弄懂，以后再看
void vPortValidateInterruptPriority(void)
{
	uint32_t ulCurrentInterrupt;
	uint8_t ucCurrentPriority;

	/* Obtain the number of the currently executing interrupt. */
	// 读取当前的中断号到ulCurrentInterrupt中
	__asm volatile("mrs %0, ipsr" : "=r"(ulCurrentInterrupt)::"memory");

	/* Is the interrupt number a user defined interrupt? */
	// 中断是用户定义(外设)的中断?
	if (ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER)
	{
		// 获取当前中断的优先级
		/* Look up the interrupt's priority. */
		ucCurrentPriority = pcInterruptPriorityRegisters[ulCurrentInterrupt];

		/* The following assertion will fail if a service routine (ISR) for
		an interrupt that has been assigned a priority above
		configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
		function.  ISR safe FreeRTOS API functions must *only* be called
		from interrupts that have been assigned a priority at or below
		configMAX_SYSCALL_INTERRUPT_PRIORITY.

		Numerically low interrupt priority numbers represent logically high
		interrupt priorities, therefore the priority of the interrupt must
		be set to a value equal to or numerically *higher* than
		configMAX_SYSCALL_INTERRUPT_PRIORITY.

		Interrupts that	use the FreeRTOS API must not be left at their
		default priority of	zero as that is the highest possible priority,
		which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
		and	therefore also guaranteed to be invalid.

		FreeRTOS maintains separate thread and ISR API functions to ensure
		interrupt entry is as fast and simple as possible.

		The following links provide detailed information:
		http://www.freertos.org/RTOS-Cortex-M3-M4.html
		http://www.freertos.org/FAQHelp.html */
		// 如果当前的中断的优先级大于配置的优先级，将会出错
		configASSERT(ucCurrentPriority >= ucMaxSysCallPriority);
	}

	/* Priority grouping:  The interrupt controller (NVIC) allows the bits
	that define each interrupt's priority to be split between bits that
	define the interrupt's pre-emption priority bits and bits that define
	the interrupt's sub-priority.  For simplicity all bits must be defined
	to be pre-emption priority bits.  The following assertion will fail if
	this is not the case (if some bits represent a sub-priority).

	If the application only uses CMSIS libraries for interrupt
	configuration then the correct setting can be achieved on all Cortex-M
	devices by calling NVIC_SetPriorityGrouping( 0 ); before starting the
	scheduler.  Note however that some vendor specific peripheral libraries
	assume a non-zero priority group setting, in which cases using a value
	of zero will result in unpredictable behaviour. */
	configASSERT((portAIRCR_REG & portPRIORITY_GROUP_MASK) <= ulMaxPRIGROUPValue);
}

#endif /* configASSERT_DEFINED */
