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

#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#if (configUSE_CO_ROUTINES == 1)
#include "croutine.h"
#endif

/* Lint e961 and e750 are suppressed as a MISRA exception justified because the
MPU ports require MPU_WRAPPERS_INCLUDED_FROM_API_FILE to be defined for the
header files above, but not in this file, in order to generate the correct
privileged Vs unprivileged linkage and placement. */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e961 !e750. */

/* Constants used with the cRxLock and cTxLock structure members. */
// 队列上锁和解锁的常数
#define queueUNLOCKED ((int8_t) - 1)
#define queueLOCKED_UNMODIFIED ((int8_t)0)

/* When the Queue_t structure is used to represent a base queue its pcHead and
pcTail members are used as pointers into the queue storage area.  When the
Queue_t structure is used to represent a mutex pcHead and pcTail pointers are
not necessary, and the pcHead pointer is set to NULL to indicate that the
pcTail pointer actually points to the mutex holder (if any).  Map alternative
names to the pcHead and pcTail structure members to ensure the readability of
the code is maintained despite this dual use of two structure members.  An
alternative implementation would be to use a union, but use of a union is
against the coding standard (although an exception to the standard has been
permitted where the dual use also significantly changes the type of the
structure member). */
// 队列被用于互斥量
#define pxMutexHolder pcTail
#define uxQueueType pcHead

#define queueQUEUE_IS_MUTEX NULL
// x信号量不存储或者复制数据
/* Semaphores do not actually store or copy data, so have an item size of
zero. */
#define queueSEMAPHORE_QUEUE_ITEM_LENGTH ((UBaseType_t)0)
#define queueMUTEX_GIVE_BLOCK_TIME ((TickType_t)0U)

#if (configUSE_PREEMPTION == 0)
/* If the cooperative scheduler is being used then a yield should not be
performed just because a higher priority task has been woken. */
#define queueYIELD_IF_USING_PREEMPTION()
#else
#define queueYIELD_IF_USING_PREEMPTION() portYIELD_WITHIN_API()
#endif

/*
 * Definition of the queue used by the scheduler.
 * Items are queued by copy, not reference.  See the following link for the
 * rationale: http://www.freertos.org/Embedded-RTOS-Queues.html
 */
typedef struct QueueDefinition
{
	/*queue储存区域的起始地址  互斥量时 为null*/
	int8_t *pcHead; /*< Points to the beginning of the queue storage area. */
	/*queue储存区域的结束地址*/
	int8_t *pcTail; /*< Points to the byte at the end of the queue storage area.  Once more byte is allocated than necessary to store the queue items, this is used as a marker. */
	// 下一个写入的位置在储存空间
	int8_t *pcWriteTo; /*< Points to the free next place in the storage area. */

	// union 是标准编码中的一个例外，确保两个互斥结构体成员不会同时出现
	union /* Use of a union is an exception to the coding standard to ensure two mutually exclusive structure members don't appear simultaneously (wasting RAM). */
	{
		// 队列时，被读取后的最后一个位置
		int8_t *pcReadFrom; /*< Points to the last place that a queued item was read from when the structure is used as a queue. */
		// 递归互斥量时，互斥锁taken的次数
		UBaseType_t uxRecursiveCallCount; /*< Maintains a count of the number of times a recursive mutex has been recursively 'taken' when the structure is used as a mutex. */
	} u;
	// 等待写入的task,优先级排序
	List_t xTasksWaitingToSend; /*< List of tasks that are blocked waiting to post onto this queue.  Stored in priority order. */
	// 等待读出的task,优先级排序
	List_t xTasksWaitingToReceive; /*< List of tasks that are blocked waiting to read from this queue.  Stored in priority  order. */

	/*当前存在queue里面的数据长度*/
	volatile UBaseType_t uxMessagesWaiting; /*< The number of items currently in the queue. */
	/*最大能存在队列的信息长度*/
	UBaseType_t uxLength; /*< The length of the queue defined as the number of items it will hold, not the number of bytes. */
	/*单条信息的长度*/
	UBaseType_t uxItemSize; /*< The size of each items that the queue will hold. */

	// 存储队列锁定时，从队列接收（从队列中删除）的出队项目数。 如果队列没有上锁，设置为queueUNLOCKED
	volatile int8_t cRxLock; /*< Stores the number of items received from the queue (removed from the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */
	// 存储队列锁定时，传输到队列（添加到队列）的入队项目数。 如果队列没有上锁，设置为queueUNLOCKED
	volatile int8_t cTxLock; /*< Stores the number of items transmitted to the queue (added to the queue) while the queue was locked.  Set to queueUNLOCKED when the queue is not locked. */

/*queue使用的内存是否是动态分配的，如果是动态分配的，那么ucStaticallyAllocated = false，确保静态分配的queue不会被 free掉*/
#if ((configSUPPORT_STATIC_ALLOCATION == 1) && (configSUPPORT_DYNAMIC_ALLOCATION == 1))
	uint8_t ucStaticallyAllocated; /*< Set to pdTRUE if the memory used by the queue was statically allocated to ensure no attempt is made to free the memory. */
#endif

// 消息集
#if (configUSE_QUEUE_SETS == 1)
	struct QueueDefinition *pxQueueSetContainer;
#endif

// 性能追踪
#if (configUSE_TRACE_FACILITY == 1)
	UBaseType_t uxQueueNumber;
	uint8_t ucQueueType;
#endif

} xQUEUE;

/* The old xQUEUE name is maintained above then typedefed to the new Queue_t
name below to enable the use of older kernel aware debuggers. */
typedef xQUEUE Queue_t;

/*-----------------------------------------------------------*/

/*
 * The queue registry is just a means for kernel aware debuggers to locate
 * queue structures.  It has no other purpose so is an optional component.
 */
#if (configQUEUE_REGISTRY_SIZE > 0)

/* The type stored within the queue registry array.  This allows a name
to be assigned to each queue making kernel aware debugging a little
more user friendly. */
// 用于储存队列登记数组
typedef struct QUEUE_REGISTRY_ITEM
{
	const char *pcQueueName; /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
	QueueHandle_t xHandle;
} xQueueRegistryItem;

/* The old xQueueRegistryItem name is maintained above then typedefed to the
new xQueueRegistryItem name below to enable the use of older kernel aware
debuggers. */
// 队列注册表子项
typedef xQueueRegistryItem QueueRegistryItem_t;

/* The queue registry is simply an array of QueueRegistryItem_t structures.
The pcQueueName member of a structure being NULL is indicative of the
array position being vacant. */
// 队列注册表
PRIVILEGED_DATA QueueRegistryItem_t xQueueRegistry[configQUEUE_REGISTRY_SIZE];

#endif /* configQUEUE_REGISTRY_SIZE */

/*
 * Unlocks a queue locked by a call to prvLockQueue.  Locking a queue does not
 * prevent an ISR from adding or removing items to the queue, but does prevent
 * an ISR from removing tasks from the queue event lists.  If an ISR finds a
 * queue is locked it will instead increment the appropriate queue lock count
 * to indicate that a task may require unblocking.  When the queue in unlocked
 * these lock counts are inspected, and the appropriate action taken.
 */
static void prvUnlockQueue(Queue_t *const pxQueue) PRIVILEGED_FUNCTION;

/*
 * Uses a critical section to determine if there is any data in a queue.
 *
 * @return pdTRUE if the queue contains no items, otherwise pdFALSE.
 */
static BaseType_t prvIsQueueEmpty(const Queue_t *pxQueue) PRIVILEGED_FUNCTION;

/*
 * Uses a critical section to determine if there is any space in a queue.
 *
 * @return pdTRUE if there is no space, otherwise pdFALSE;
 */
static BaseType_t prvIsQueueFull(const Queue_t *pxQueue) PRIVILEGED_FUNCTION;

/*
 * Copies an item into the queue, either at the front of the queue or the
 * back of the queue.
 */
static BaseType_t prvCopyDataToQueue(Queue_t *const pxQueue, const void *pvItemToQueue, const BaseType_t xPosition) PRIVILEGED_FUNCTION;

/*
 * Copies an item out of a queue.
 */
static void prvCopyDataFromQueue(Queue_t *const pxQueue, void *const pvBuffer) PRIVILEGED_FUNCTION;

#if (configUSE_QUEUE_SETS == 1)
/*
 * Checks to see if a queue is a member of a queue set, and if so, notifies
 * the queue set that the queue contains data.
 */
static BaseType_t prvNotifyQueueSetContainer(const Queue_t *const pxQueue, const BaseType_t xCopyPosition) PRIVILEGED_FUNCTION;
#endif

/*
 * Called after a Queue_t structure has been allocated either statically or
 * dynamically to fill in the structure's members.
 * 在队列结构体被创建后，被调用来给队列结构体填充值
 */
static void prvInitialiseNewQueue(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, uint8_t *pucQueueStorage, const uint8_t ucQueueType, Queue_t *pxNewQueue) PRIVILEGED_FUNCTION;

/*
 * Mutexes are a special type of queue.  When a mutex is created, first the
 * queue is created, then prvInitialiseMutex() is called to configure the queue
 * as a mutex.
 */
#if (configUSE_MUTEXES == 1)
static void prvInitialiseMutex(Queue_t *pxNewQueue) PRIVILEGED_FUNCTION;
#endif

#if (configUSE_MUTEXES == 1)
/*
 * If a task waiting for a mutex causes the mutex holder to inherit a
 * priority, but the waiting task times out, then the holder should
 * disinherit the priority - but only down to the highest priority of any
 * other tasks that are waiting for the same mutex.  This function returns
 * that priority.
 */
static UBaseType_t prvGetDisinheritPriorityAfterTimeout(const Queue_t *const pxQueue) PRIVILEGED_FUNCTION;
#endif
/*-----------------------------------------------------------*/

/*
 * Macro to mark a queue as locked.  Locking a queue prevents an ISR from
 * accessing the queue event lists.
 */

// 上锁队列
#define prvLockQueue(pxQueue)                            \
	taskENTER_CRITICAL();                                \
	{                                                    \
		if ((pxQueue)->cRxLock == queueUNLOCKED)         \
		{                                                \
			(pxQueue)->cRxLock = queueLOCKED_UNMODIFIED; \
		}                                                \
		if ((pxQueue)->cTxLock == queueUNLOCKED)         \
		{                                                \
			(pxQueue)->cTxLock = queueLOCKED_UNMODIFIED; \
		}                                                \
	}                                                    \
	taskEXIT_CRITICAL()
/*-----------------------------------------------------------*/

BaseType_t xQueueGenericReset(QueueHandle_t xQueue, BaseType_t xNewQueue)
{
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	configASSERT(pxQueue);

	// 关闭临界区
	taskENTER_CRITICAL();
	{
		// 初始化queue.tail的地址(尾)，计数型信号量、二进制信号量、 互斥量的tail 和 head 都是队列本身
		pxQueue->pcTail = pxQueue->pcHead + (pxQueue->uxLength * pxQueue->uxItemSize);
		// 当前在queue里面的数据的长度。
		pxQueue->uxMessagesWaiting = (UBaseType_t)0U;
		// 将pcWriteTo的地址，指向储存内存开始的地址
		pxQueue->pcWriteTo = pxQueue->pcHead;
		//	将pcReadFrom的地址指向储存空间的最后一项(读的时候会自动加一)
		pxQueue->u.pcReadFrom = pxQueue->pcHead + ((pxQueue->uxLength - (UBaseType_t)1U) * pxQueue->uxItemSize);
		// 读写锁
		pxQueue->cRxLock = queueUNLOCKED;
		pxQueue->cTxLock = queueUNLOCKED;

		// 不是新创建的队列
		if (xNewQueue == pdFALSE)
		{
			/*如果阻塞的task在等待读取该queue，那么阻塞task将会保持堵塞，因为在该函数结束后，queue仍然是空的，
			如果有阻塞的task在等待写该queue，那么一个task应该不再阻塞，因为在该函数结束之后，task将有可能写入queue*/
			/* If there are tasks blocked waiting to read from the queue, then
			the tasks will remain blocked as after this function exits the queue
			will still be empty.  If there are tasks blocked waiting to write to
			the queue, then one should be unblocked as after this function exits
			it will be possible to write to it. */

			// 检查是等待往队列写数据的列表
			if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToSend)) == pdFALSE)
			{
				// 将等待写队列的任务列表释放(现在队列为空 可以写)
				if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToSend)) != pdFALSE)
				{
					// 进行任务切换
					queueYIELD_IF_USING_PREEMPTION();
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		// 新queue
		else
		{
			/* Ensure the event queues start in the correct state. */
			// 确保事件队列从正确的位置开始(初始化等待发送和等待接受的队列)
			vListInitialise(&(pxQueue->xTasksWaitingToSend));
			vListInitialise(&(pxQueue->xTasksWaitingToReceive));
		}
	}
	// 打开临界区
	taskEXIT_CRITICAL();

	/* A value is returned for calling semantic consistency with previous
	versions. */
	return pdPASS;
}
/*-----------------------------------------------------------*/

#if (configSUPPORT_STATIC_ALLOCATION == 1)

/*
静态的创造queue,相比动态的创造queue,静态创造多了两个参数 pucQueueStorage pxStaticQueue,
其中StaticQueue_t隐藏了变量类型(为什么这样做，查看函数定义)，StaticQueue_t在内存空间和结构上，和QueueDefinition一样的，
动态创建的queue是不能被用户代码所访问，从而提高了代码的安全性，而静态创造queue的时候，用户可能会尝试直接去访问queue结构体
而不是使用函数去访问，为了让用户认识到，直接访问StaticQueue_t是不明智的行为，freeRTOS.h故意让其定义变得模糊

pucQueueStorage 储存区域
pxStaticQueue queue 结构体

*/
QueueHandle_t xQueueGenericCreateStatic(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, uint8_t *pucQueueStorage, StaticQueue_t *pxStaticQueue, const uint8_t ucQueueType)
{

	// 创建队列指针
	Queue_t *pxNewQueue;

	configASSERT(uxQueueLength > (UBaseType_t)0);

	/* The StaticQueue_t structure and the queue storage area must be
	supplied. */
	configASSERT(pxStaticQueue != NULL);

	/* A queue storage area should be provided if the item size is not 0, and
	should not be provided if the item size is 0. */
	configASSERT(!((pucQueueStorage != NULL) && (uxItemSize == 0)));
	configASSERT(!((pucQueueStorage == NULL) && (uxItemSize != 0)));

#if (configASSERT_DEFINED == 1)
	{
		/* Sanity check that the size of the structure used to declare a
		variable of type StaticQueue_t or StaticSemaphore_t equals the size of
		the real queue and semaphore structures. */
		// 检查 StaticQueue_t的大小和Queue_t是否是一致
		volatile size_t xSize = sizeof(StaticQueue_t);
		configASSERT(xSize == sizeof(Queue_t));
	}
#endif /* configASSERT_DEFINED */

	/* The address of a statically allocated queue was passed in, use it.
	The address of a statically allocated storage area was also passed in
	but is already set. */

	// 将pxNewQueue的地址指向pxStaticQueue
	pxNewQueue = (Queue_t *)pxStaticQueue; /*lint !e740 Unusual cast is ok as the structures are designed to have the same alignment, and the size is checked by an assert. */

	if (pxNewQueue != NULL)
	{
#if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
		{
			/* Queues can be allocated wither statically or dynamically, so
			note this queue was allocated statically in case the queue is
			later deleted. */
			pxNewQueue->ucStaticallyAllocated = pdTRUE;
		}
#endif /* configSUPPORT_DYNAMIC_ALLOCATION */

		// 初始化队列
		prvInitialiseNewQueue(uxQueueLength, uxItemSize, pucQueueStorage, ucQueueType, pxNewQueue);
	}
	else
	{
		traceQUEUE_CREATE_FAILED(ucQueueType);
	}

	return pxNewQueue;
}

#endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

#if (configSUPPORT_DYNAMIC_ALLOCATION == 1)

// 通用创造队列，给其他队列函数提供接口
/*
	信号类型		uxQueueLength	uxItemSize	ucQueueType
	互斥量 			1  				0 			queueQUEUE_TYPE_MUTEX(1)
	二进制信号量	1				 0			 queueQUEUE_TYPE_BINARY_SEMAPHORE(3)
	计数型信号量	uxMaxCount		 0			 queueQUEUE_TYPE_COUNTING_SEMAPHORE(2)
	队列		    itemLength		itemSize	queueQUEUE_TYPE_BASE
	递归互斥量 		1  				0 			queueQUEUE_TYPE_RECURSIVE_MUTEX(4)
 */
QueueHandle_t xQueueGenericCreate(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, const uint8_t ucQueueType)
{
	// 新队列
	Queue_t *pxNewQueue;
	// 进入队列的项的字节大小
	size_t xQueueSizeInBytes;
	// 队列的储存区域
	uint8_t *pucQueueStorage;

	// 检查需要分配的长度合理性
	configASSERT(uxQueueLength > (UBaseType_t)0);

	// 检查队列的每个数据的大小 计数型信号量、二进制信号量、互斥量 为0
	if (uxItemSize == (UBaseType_t)0)
	{
		// 将不会创建队列存储区域
		/* There is not going to be a queue storage area. */
		xQueueSizeInBytes = (size_t)0;
	}
	else
	{
		/* Allocate enough space to hold the maximum number of items that
		can be in the queue at any time. */
		/*计算存储queue的内存长度 队列*/
		xQueueSizeInBytes = (size_t)(uxQueueLength * uxItemSize); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
	}
	/*分配queue的内存地址,包括queue的结构体长度和数据储存长度(队列)*/
	pxNewQueue = (Queue_t *)pvPortMalloc(sizeof(Queue_t) + xQueueSizeInBytes);

	if (pxNewQueue != NULL)
	{
		/* Jump past the queue structure to find the location of the queue
		storage area. */
		/*储存数据的地址，跳过队列结构体地址*/
		pucQueueStorage = ((uint8_t *)pxNewQueue) + sizeof(Queue_t);

#if (configSUPPORT_STATIC_ALLOCATION == 1)
		{
			// 队列能静态创建或者动态创建，标记该队列是动态创建的，以便在未来可以释放
			/* Queues can be created either statically or dynamically, so
			note this task was created dynamically in case it is later
			deleted. */
			pxNewQueue->ucStaticallyAllocated = pdFALSE;
		}
#endif /* configSUPPORT_STATIC_ALLOCATION */
		// 初始换新队列
		prvInitialiseNewQueue(uxQueueLength, uxItemSize, pucQueueStorage, ucQueueType, pxNewQueue);
	}
	else
	{
		traceQUEUE_CREATE_FAILED(ucQueueType);
	}

	return pxNewQueue;
}

#endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

// 初始化新queue
/*
	信号类型		uxQueueLength	uxItemSize	pucQueueStorage		ucQueueType
	互斥量 			1  				0 			NULL				queueQUEUE_TYPE_MUTEX(1)
	二进制信号量	1				 0			 NULL				 queueQUEUE_TYPE_BINARY_SEMAPHORE(3)
	计数型信号量  	maxCount		0			NULL				queueQUEUE_TYPE_COUNTING_SEMAPHORE(2)
	队列		    itemLength		itemSize	queueStoreAddree	queueQUEUE_TYPE_BASE
 */
static void prvInitialiseNewQueue(const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, uint8_t *pucQueueStorage, const uint8_t ucQueueType, Queue_t *pxNewQueue)
{
	/* Remove compiler warnings about unused parameters should
	configUSE_TRACE_FACILITY not be set to 1. */
	// 初始化队列用不到队列类型
	(void)ucQueueType;

	// 互斥量、二进制信号量、计数型信号量
	if (uxItemSize == (UBaseType_t)0)
	{ // uxItemSize = 0 的时候，没有分配queue的存储区，但是pcHead指向NULL是互斥量，因此这里将pcHead指向queue自己。
		/* No RAM was allocated for the queue storage area, but PC head cannot
		be set to NULL because NULL is used as a key to say the queue is used as
		a mutex.  Therefore just set pcHead to point to the queue as a benign
		value that is known to be within the memory map. */
		pxNewQueue->pcHead = (int8_t *)pxNewQueue;
	}
	else
	{
		// 指向queue数据储存区域
		/* Set the head to the start of the queue storage area. */
		pxNewQueue->pcHead = (int8_t *)pucQueueStorage;
	}

	/* Initialise the queue members as described where the queue type is
	defined. */
	// 初始化队列成员
	pxNewQueue->uxLength = uxQueueLength; // 存储长度
	pxNewQueue->uxItemSize = uxItemSize;  // 单个队列项的大小
	// 复位queue
	(void)xQueueGenericReset(pxNewQueue, pdTRUE);

// 性能追踪
#if (configUSE_TRACE_FACILITY == 1)
	{
		pxNewQueue->ucQueueType = ucQueueType;
	}
#endif /* configUSE_TRACE_FACILITY */

// 队列集
#if (configUSE_QUEUE_SETS == 1)
	{
		pxNewQueue->pxQueueSetContainer = NULL;
	}
#endif /* configUSE_QUEUE_SETS */

	traceQUEUE_CREATE(pxNewQueue);
}
/*-----------------------------------------------------------*/

#if (configUSE_MUTEXES == 1)

// 初始化互斥量
static void prvInitialiseMutex(Queue_t *pxNewQueue)
{
	if (pxNewQueue != NULL)
	{
		/* The queue create function will set all the queue structure members
		correctly for a generic queue, but this function is creating a
		mutex.  Overwrite those members that need to be set differently -
		in particular the information required for priority inheritance. */
		/*互斥量是一种特殊的queue*/
		// 互斥量拥有者
		pxNewQueue->pxMutexHolder = NULL; //(pxMutexHolder = pcTail)
		// pxNewQueue.pcHead = NUll 用于 标识 queue是互斥量
		pxNewQueue->uxQueueType = queueQUEUE_IS_MUTEX; //(uxQueueType = pcHead)

		/* In case this is a recursive mutex. */
		// 设置互斥量递归被使用次数
		pxNewQueue->u.uxRecursiveCallCount = 0;

		traceCREATE_MUTEX(pxNewQueue);

		/* Start with the semaphore in the expected state. */
		// 设置互斥量初始数据
		(void)xQueueGenericSend(pxNewQueue, NULL, (TickType_t)0U, queueSEND_TO_BACK);
	}
	else
	{
		traceCREATE_MUTEX_FAILED();
	}
}

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if ((configUSE_MUTEXES == 1) && (configSUPPORT_DYNAMIC_ALLOCATION == 1))
// 创建互斥量
QueueHandle_t xQueueCreateMutex(const uint8_t ucQueueType)
{
	Queue_t *pxNewQueue;
	// 互斥量数据长度为1 互斥量中的item长度为0
	const UBaseType_t uxMutexLength = (UBaseType_t)1, uxMutexSize = (UBaseType_t)0;

	// 创建互斥量
	pxNewQueue = (Queue_t *)xQueueGenericCreate(uxMutexLength, uxMutexSize, ucQueueType);
	// 初始化互斥量
	prvInitialiseMutex(pxNewQueue);

	return pxNewQueue;
}

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if ((configUSE_MUTEXES == 1) && (configSUPPORT_STATIC_ALLOCATION == 1))
// 静态创建互斥量
QueueHandle_t xQueueCreateMutexStatic(const uint8_t ucQueueType, StaticQueue_t *pxStaticQueue)
{
	Queue_t *pxNewQueue;
	const UBaseType_t uxMutexLength = (UBaseType_t)1, uxMutexSize = (UBaseType_t)0;

	/* Prevent compiler warnings about unused parameters if
	configUSE_TRACE_FACILITY does not equal 1. */
	(void)ucQueueType;

	pxNewQueue = (Queue_t *)xQueueGenericCreateStatic(uxMutexLength, uxMutexSize, NULL, pxStaticQueue, ucQueueType);
	prvInitialiseMutex(pxNewQueue);

	return pxNewQueue;
}

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

#if ((configUSE_MUTEXES == 1) && (INCLUDE_xSemaphoreGetMutexHolder == 1))

// 获取互斥量拥有者
void *xQueueGetMutexHolder(QueueHandle_t xSemaphore)
{
	void *pxReturn;

	/* This function is called by xSemaphoreGetMutexHolder(), and should not
	be called directly.  Note:  This is a good way of determining if the
	calling task is the mutex holder, but not a good way of determining the
	identity of the mutex holder, as the holder may change between the
	following critical section exiting and the function returning. */
	taskENTER_CRITICAL();
	{
		if (((Queue_t *)xSemaphore)->uxQueueType == queueQUEUE_IS_MUTEX)
		{
			pxReturn = (void *)((Queue_t *)xSemaphore)->pxMutexHolder;
		}
		else
		{
			pxReturn = NULL;
		}
	}
	taskEXIT_CRITICAL();

	return pxReturn;
} /*lint !e818 xSemaphore cannot be a pointer to const because it is a typedef. */

#endif
/*-----------------------------------------------------------*/

#if ((configUSE_MUTEXES == 1) && (INCLUDE_xSemaphoreGetMutexHolder == 1))
// 获取互斥量拥有者在中断中
void *xQueueGetMutexHolderFromISR(QueueHandle_t xSemaphore)
{
	void *pxReturn;

	configASSERT(xSemaphore);

	/* Mutexes cannot be used in interrupt service routines, so the mutex
	holder should not change in an ISR, and therefore a critical section is
	not required here. */
	if (((Queue_t *)xSemaphore)->uxQueueType == queueQUEUE_IS_MUTEX)
	{
		pxReturn = (void *)((Queue_t *)xSemaphore)->pxMutexHolder;
	}
	else
	{
		pxReturn = NULL;
	}

	return pxReturn;
} /*lint !e818 xSemaphore cannot be a pointer to const because it is a typedef. */

#endif
/*-----------------------------------------------------------*/

#if (configUSE_RECURSIVE_MUTEXES == 1)

// 释放递归互斥量
BaseType_t xQueueGiveMutexRecursive(QueueHandle_t xMutex)
{
	BaseType_t xReturn;
	Queue_t *const pxMutex = (Queue_t *)xMutex;

	configASSERT(pxMutex);

	/* If this is the task that holds the mutex then pxMutexHolder will not
	change outside of this task.  If this task does not hold the mutex then
	pxMutexHolder can never coincidentally equal the tasks handle, and as
	this is the only condition we are interested in it does not matter if
	pxMutexHolder is accessed simultaneously by another task.  Therefore no
	mutual exclusion is required to test the pxMutexHolder variable. */
	//想要释放互斥量的holder是当前的tcb
	if (pxMutex->pxMutexHolder == (void *)xTaskGetCurrentTaskHandle()) /*lint !e961 Not a redundant cast as TaskHandle_t is a typedef. */
	{
		traceGIVE_MUTEX_RECURSIVE(pxMutex);

		/* uxRecursiveCallCount cannot be zero if pxMutexHolder is equal to
		the task handle, therefore no underflow check is required.  Also,
		uxRecursiveCallCount is only modified by the mutex holder, and as
		there can only be one, no mutual exclusion is required to modify the
		uxRecursiveCallCount member. */
		//互斥量被占用的次数
		(pxMutex->u.uxRecursiveCallCount)--;

		/* Has the recursive call count unwound to 0? */
		if (pxMutex->u.uxRecursiveCallCount == (UBaseType_t)0)
		{
			/* Return the mutex.  This will automatically unblock any other
			task that might be waiting to access the mutex. */
			//释放互斥量 ,将会自动解锁试图take()互斥量的任务
			(void)xQueueGenericSend(pxMutex, NULL, queueMUTEX_GIVE_BLOCK_TIME, queueSEND_TO_BACK);
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		xReturn = pdPASS;
	}
	////想要释放互斥量的holder不是当前的tcb
	else
	{
		/* The mutex cannot be given because the calling task is not the
		holder. */
		xReturn = pdFAIL;

		traceGIVE_MUTEX_RECURSIVE_FAILED(pxMutex);
	}

	return xReturn;
}

#endif /* configUSE_RECURSIVE_MUTEXES */
/*-----------------------------------------------------------*/

#if (configUSE_RECURSIVE_MUTEXES == 1)
// 获取递归互斥信号量
BaseType_t xQueueTakeMutexRecursive(QueueHandle_t xMutex, TickType_t xTicksToWait)
{
	BaseType_t xReturn;
	Queue_t *const pxMutex = (Queue_t *)xMutex;

	configASSERT(pxMutex);

	/* Comments regarding mutual exclusion as per those within
	xQueueGiveMutexRecursive(). */

	traceTAKE_MUTEX_RECURSIVE(pxMutex);

	// 想要获取递归互斥量的tcb是互斥量的Holder
	if (pxMutex->pxMutexHolder == (void *)xTaskGetCurrentTaskHandle()) /*lint !e961 Cast is not redundant as TaskHandle_t is a typedef. */
	{
		//互斥量被占用的次数
		(pxMutex->u.uxRecursiveCallCount)++;
		//递归互斥量take() 成功
		xReturn = pdPASS;
	}
	//想要获取互斥量的tcb不是互斥量的Holder
	else
	{
		//调度普通互斥量调用函数
		xReturn = xQueueSemaphoreTake(pxMutex, xTicksToWait);

		/* pdPASS will only be returned if the mutex was successfully
		obtained.  The calling task may have entered the Blocked state
		before reaching here. */
		//pdPASS只会在成功获取到了互斥量之后返回，调用者可能会被阻塞
		if (xReturn != pdFAIL)
		{
			(pxMutex->u.uxRecursiveCallCount)++;
		}
		else
		{
			traceTAKE_MUTEX_RECURSIVE_FAILED(pxMutex);
		}
	}

	return xReturn;
}

#endif /* configUSE_RECURSIVE_MUTEXES */
/*-----------------------------------------------------------*/

#if ((configUSE_COUNTING_SEMAPHORES == 1) && (configSUPPORT_STATIC_ALLOCATION == 1))

// 静态创建计数型信号量
QueueHandle_t xQueueCreateCountingSemaphoreStatic(const UBaseType_t uxMaxCount, const UBaseType_t uxInitialCount, StaticQueue_t *pxStaticQueue)
{
	QueueHandle_t xHandle;

	configASSERT(uxMaxCount != 0);
	configASSERT(uxInitialCount <= uxMaxCount);

	xHandle = xQueueGenericCreateStatic(uxMaxCount, queueSEMAPHORE_QUEUE_ITEM_LENGTH, NULL, pxStaticQueue, queueQUEUE_TYPE_COUNTING_SEMAPHORE);

	if (xHandle != NULL)
	{
		((Queue_t *)xHandle)->uxMessagesWaiting = uxInitialCount;

		traceCREATE_COUNTING_SEMAPHORE();
	}
	else
	{
		traceCREATE_COUNTING_SEMAPHORE_FAILED();
	}

	return xHandle;
}

#endif /* ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

#if ((configUSE_COUNTING_SEMAPHORES == 1) && (configSUPPORT_DYNAMIC_ALLOCATION == 1))
// 创建计数型信号量
QueueHandle_t xQueueCreateCountingSemaphore(const UBaseType_t uxMaxCount, const UBaseType_t uxInitialCount)
{
	QueueHandle_t xHandle;

	configASSERT(uxMaxCount != 0);
	configASSERT(uxInitialCount <= uxMaxCount);

	xHandle = xQueueGenericCreate(uxMaxCount, queueSEMAPHORE_QUEUE_ITEM_LENGTH, queueQUEUE_TYPE_COUNTING_SEMAPHORE);

	if (xHandle != NULL)
	{
		// 设置计数型信号量里面的消息数
		((Queue_t *)xHandle)->uxMessagesWaiting = uxInitialCount;

		traceCREATE_COUNTING_SEMAPHORE();
	}
	else
	{
		traceCREATE_COUNTING_SEMAPHORE_FAILED();
	}

	return xHandle;
}

#endif /* ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

// pvItemToQueue的值和指针地址都不能被修改
// 往队列里面写数据
/*
	信号类型		pvItemToQueue	xTicksToWait	xCopyPosition
	互斥量 			NULL  			0 				queueSEND_TO_BACK(0)
	二进制信号量	 NULL			 0			 	queueSEND_TO_BACK(0)
	计数型信号量	 NULL			 0			 	queueSEND_TO_BACK(0)
	队列		    data			waitTime		queueSEND_TO_BACK(0)
*/
BaseType_t xQueueGenericSend(QueueHandle_t xQueue, const void *const pvItemToQueue, TickType_t xTicksToWait, const BaseType_t xCopyPosition)
{
	BaseType_t xEntryTimeSet = pdFALSE, xYieldRequired;
	TimeOut_t xTimeOut;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	// 检查队列不为空
	configASSERT(pxQueue);
	// pvItemToQueue 为 null 时 uxItemSize 必须 0
	configASSERT(!((pvItemToQueue == NULL) && (pxQueue->uxItemSize != (UBaseType_t)0U)));
	// 覆写 且 数据长度为1
	configASSERT(!((xCopyPosition == queueOVERWRITE) && (pxQueue->uxLength != 1)));
#if ((INCLUDE_xTaskGetSchedulerState == 1) || (configUSE_TIMERS == 1))
	{
		// 调度器不运行时，xTicksToWait 必须为0
		configASSERT(!((xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) && (xTicksToWait != 0)));
	}
#endif

	/* This function relaxes the coding standard somewhat to allow return
	statements within the function itself.  This is done in the interest
	of execution time efficiency. */
	for (;;)
	{
		// 进入临界区01
		taskENTER_CRITICAL();
		{
			/* Is there room on the queue now?  The running task must be the
			highest priority task wanting to access the queue.  If the head item
			in the queue is to be overwritten then it does not matter if the
			queue is full. */
			// 数据未满或者是为覆写 give()、send()
			if ((pxQueue->uxMessagesWaiting < pxQueue->uxLength) || (xCopyPosition == queueOVERWRITE))
			{
				traceQUEUE_SEND(pxQueue);
				// 复制数据到队列
				xYieldRequired = prvCopyDataToQueue(pxQueue, pvItemToQueue, xCopyPosition);

#if (configUSE_QUEUE_SETS == 1) // 消息集
				{
					if (pxQueue->pxQueueSetContainer != NULL)
					{
						if (prvNotifyQueueSetContainer(pxQueue, xCopyPosition) != pdFALSE)
						{
							/* The queue is a member of a queue set, and posting
							to the queue set caused a higher priority task to
							unblock. A context switch is required. */
							queueYIELD_IF_USING_PREEMPTION();
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else
					{
						/* If there was a task waiting for data to arrive on the
						queue then unblock it now. */
						if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
						{
							if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
							{
								/* The unblocked task has a priority higher than
								our own so yield immediately.  Yes it is ok to
								do this from within the critical section - the
								kernel takes care of that. */
								queueYIELD_IF_USING_PREEMPTION();
							}
							else
							{
								mtCOVERAGE_TEST_MARKER();
							}
						}
						else if (xYieldRequired != pdFALSE)
						{
							/* This path is a special case that will only get
							executed if the task was holding multiple mutexes
							and the mutexes were given back in an order that is
							different to that in which they were taken. */
							queueYIELD_IF_USING_PREEMPTION();
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
				}
#else  /* configUSE_QUEUE_SETS */
				{
					/* If there was a task waiting for data to arrive on the
					queue then unblock it now. */
					// 有任务在等待队列有数据
					if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
					{
						// 唤醒任务
						if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
						{
							/* The unblocked task has a priority higher than
							our own so yield immediately.  Yes it is ok to do
							this from within the critical section - the kernel
							takes care of that. */
							queueYIELD_IF_USING_PREEMPTION();
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else if (xYieldRequired != pdFALSE)
					{
						/* This path is a special case that will only get
						executed if the task was holding multiple mutexes and
						the mutexes were given back in an order that is
						different to that in which they were taken. */
						// 切换任务 往互斥量写数据后 可能会切换任务
						queueYIELD_IF_USING_PREEMPTION();
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
#endif /* configUSE_QUEUE_SETS */

				// 退出临界区
				taskEXIT_CRITICAL();
				// 返回写入数据成功
				return pdPASS;
			}
			// 队列已经满了
			else
			{
				// 未设置休眠时间
				if (xTicksToWait == (TickType_t)0)
				{
					/* The queue was full and no block time is specified (or
					the block time has expired) so leave now. */
					taskEXIT_CRITICAL();

					/* Return to the original privilege level before exiting
					the function. */
					traceQUEUE_SEND_FAILED(pxQueue);
					// 返回错误，队列已近满了
					return errQUEUE_FULL;
				}
				// 未设置入口时间
				else if (xEntryTimeSet == pdFALSE)
				{
					/* The queue was full and a block time was specified so
					configure the timeout structure. */
					// 配置超时结构体
					vTaskInternalSetTimeOutState(&xTimeOut);
					// 已设置超时时间
					xEntryTimeSet = pdTRUE;
				}
				else
				{
					/* Entry time was already set. */
					mtCOVERAGE_TEST_MARKER();
				}
			}
		}
		// 退出临界区 队列已满且设置了超时时间
		taskEXIT_CRITICAL();

		/* Interrupts and other tasks can send to and receive from the queue
		now the critical section has been exited. */

		// 队列已满且设置了超时时间 会执行以下代码
		vTaskSuspendAll();
		// 锁定队列
		prvLockQueue(pxQueue);

		/* Update the timeout state to see if it has expired yet. */
		// 更新超时结构体，检查是否发生了超时
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE)
		{
			// 未发生超时 但是队列满了
			if (prvIsQueueFull(pxQueue) != pdFALSE)
			{
				traceBLOCKING_ON_QUEUE_SEND(pxQueue);
				// 将任务加入到事件列表
				vTaskPlaceOnEventList(&(pxQueue->xTasksWaitingToSend), xTicksToWait);

				/* Unlocking the queue means queue events can effect the
				event list.  It is possible that interrupts occurring now
				remove this task from the event list again - but as the
				scheduler is suspended the task will go onto the pending
				ready last instead of the actual ready list. */
				prvUnlockQueue(pxQueue);

				/* Resuming the scheduler will move tasks from the pending
				ready list into the ready list - so it is feasible that this
				task is already in a ready list before it yields - in which
				case the yield will not cause a context switch unless there
				is also a higher priority task in the pending ready list. */
				if (xTaskResumeAll() == pdFALSE)
				{
					portYIELD_WITHIN_API();
				}
			}
			// 未发生超时 且队列未满
			else
			{
				/* Try again. */
				prvUnlockQueue(pxQueue);
				(void)xTaskResumeAll();
			}
		}
		// 发生超时 返回队列满
		else
		{
			/* The timeout has expired. */
			prvUnlockQueue(pxQueue);
			(void)xTaskResumeAll();

			traceQUEUE_SEND_FAILED(pxQueue);
			return errQUEUE_FULL;
		}
	}
}
/*-----------------------------------------------------------*/
// 中断发送数据到队列
BaseType_t xQueueGenericSendFromISR(QueueHandle_t xQueue, const void *const pvItemToQueue, BaseType_t *const pxHigherPriorityTaskWoken, const BaseType_t xCopyPosition)
{
	BaseType_t xReturn;
	UBaseType_t uxSavedInterruptStatus;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	// 检查队列是否是空的
	configASSERT(pxQueue);
	// pvItemToQueue为null的时候，uxItemSize必须为0
	configASSERT(!((pvItemToQueue == NULL) && (pxQueue->uxItemSize != (UBaseType_t)0U)));
	// 写入方式为覆盖写时，uxLength必须为1
	configASSERT(!((xCopyPosition == queueOVERWRITE) && (pxQueue->uxLength != 1)));

	/* RTOS ports that support interrupt nesting have the concept of a maximum
	system call (or maximum API call) interrupt priority.  Interrupts that are
	above the maximum system call priority are kept permanently enabled, even
	when the RTOS kernel is in a critical section, but cannot make any calls to
	FreeRTOS API functions.  If configASSERT() is defined in FreeRTOSConfig.h
	then portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
	failure if a FreeRTOS API function is called from an interrupt that has been
	assigned a priority above the configured maximum system call priority.
	Only FreeRTOS functions that end in FromISR can be called from interrupts
	that have been assigned a priority at or (logically) below the maximum
	system call	interrupt priority.  FreeRTOS maintains a separate interrupt
	safe API to ensure interrupt entry is as fast and as simple as possible.
	More information (albeit Cortex-M specific) is provided on the following
	link: http://www.freertos.org/RTOS-Cortex-M3-M4.html */

	// 关闭中断
	portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

	/* Similar to xQueueGenericSend, except without blocking if there is no room
	in the queue.  Also don't directly wake a task that was blocked on a queue
	read, instead return a flag to say whether a context switch is required or
	not (i.e. has a task with a higher priority than us been woken by this
	post). */
	uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		if ((pxQueue->uxMessagesWaiting < pxQueue->uxLength) || (xCopyPosition == queueOVERWRITE))
		{
			const int8_t cTxLock = pxQueue->cTxLock;

			traceQUEUE_SEND_FROM_ISR(pxQueue);

			/* Semaphores use xQueueGiveFromISR(), so pxQueue will not be a
			semaphore or mutex.  That means prvCopyDataToQueue() cannot result
			in a task disinheriting a priority and prvCopyDataToQueue() can be
			called here even though the disinherit function does not check if
			the scheduler is suspended before accessing the ready lists. */
			(void)prvCopyDataToQueue(pxQueue, pvItemToQueue, xCopyPosition);

			/* The event list is not altered if the queue is locked.  This will
			be done when the queue is unlocked later. */
			if (cTxLock == queueUNLOCKED)
			{
#if (configUSE_QUEUE_SETS == 1) // 消息集
				{
					if (pxQueue->pxQueueSetContainer != NULL)
					{
						if (prvNotifyQueueSetContainer(pxQueue, xCopyPosition) != pdFALSE)
						{
							/* The queue is a member of a queue set, and posting
							to the queue set caused a higher priority task to
							unblock.  A context switch is required. */
							if (pxHigherPriorityTaskWoken != NULL)
							{
								*pxHigherPriorityTaskWoken = pdTRUE;
							}
							else
							{
								mtCOVERAGE_TEST_MARKER();
							}
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else
					{
						if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
						{
							if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
							{
								/* The task waiting has a higher priority so
								record that a context switch is required. */
								if (pxHigherPriorityTaskWoken != NULL)
								{
									*pxHigherPriorityTaskWoken = pdTRUE;
								}
								else
								{
									mtCOVERAGE_TEST_MARKER();
								}
							}
							else
							{
								mtCOVERAGE_TEST_MARKER();
							}
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
				}
#else  /* configUSE_QUEUE_SETS */
				{
					if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
					{
						if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
						{
							/* The task waiting has a higher priority so record that a
							context	switch is required. */
							if (pxHigherPriorityTaskWoken != NULL)
							{
								*pxHigherPriorityTaskWoken = pdTRUE;
							}
							else
							{
								mtCOVERAGE_TEST_MARKER();
							}
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
#endif /* configUSE_QUEUE_SETS */
			}
			else
			{
				/* Increment the lock count so the task that unlocks the queue
				knows that data was posted while it was locked. */
				pxQueue->cTxLock = (int8_t)(cTxLock + 1);
			}

			xReturn = pdPASS;
		}
		else
		{
			traceQUEUE_SEND_FROM_ISR_FAILED(pxQueue);
			xReturn = errQUEUE_FULL;
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueGiveFromISR(QueueHandle_t xQueue, BaseType_t *const pxHigherPriorityTaskWoken)
{
	BaseType_t xReturn;
	UBaseType_t uxSavedInterruptStatus;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	/* Similar to xQueueGenericSendFromISR() but used with semaphores where the
	item size is 0.  Don't directly wake a task that was blocked on a queue
	read, instead return a flag to say whether a context switch is required or
	not (i.e. has a task with a higher priority than us been woken by this
	post). */

	configASSERT(pxQueue);

	/* xQueueGenericSendFromISR() should be used instead of xQueueGiveFromISR()
	if the item size is not 0. */
	configASSERT(pxQueue->uxItemSize == 0);

	/* Normally a mutex would not be given from an interrupt, especially if
	there is a mutex holder, as priority inheritance makes no sense for an
	interrupts, only tasks. */
	configASSERT(!((pxQueue->uxQueueType == queueQUEUE_IS_MUTEX) && (pxQueue->pxMutexHolder != NULL)));

	/* RTOS ports that support interrupt nesting have the concept of a maximum
	system call (or maximum API call) interrupt priority.  Interrupts that are
	above the maximum system call priority are kept permanently enabled, even
	when the RTOS kernel is in a critical section, but cannot make any calls to
	FreeRTOS API functions.  If configASSERT() is defined in FreeRTOSConfig.h
	then portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
	failure if a FreeRTOS API function is called from an interrupt that has been
	assigned a priority above the configured maximum system call priority.
	Only FreeRTOS functions that end in FromISR can be called from interrupts
	that have been assigned a priority at or (logically) below the maximum
	system call	interrupt priority.  FreeRTOS maintains a separate interrupt
	safe API to ensure interrupt entry is as fast and as simple as possible.
	More information (albeit Cortex-M specific) is provided on the following
	link: http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

	uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;

		/* When the queue is used to implement a semaphore no data is ever
		moved through the queue but it is still valid to see if the queue 'has
		space'. */
		if (uxMessagesWaiting < pxQueue->uxLength)
		{
			const int8_t cTxLock = pxQueue->cTxLock;

			traceQUEUE_SEND_FROM_ISR(pxQueue);

			/* A task can only have an inherited priority if it is a mutex
			holder - and if there is a mutex holder then the mutex cannot be
			given from an ISR.  As this is the ISR version of the function it
			can be assumed there is no mutex holder and no need to determine if
			priority disinheritance is needed.  Simply increase the count of
			messages (semaphores) available. */
			pxQueue->uxMessagesWaiting = uxMessagesWaiting + (UBaseType_t)1;

			/* The event list is not altered if the queue is locked.  This will
			be done when the queue is unlocked later. */
			if (cTxLock == queueUNLOCKED)
			{
#if (configUSE_QUEUE_SETS == 1)
				{
					if (pxQueue->pxQueueSetContainer != NULL)
					{
						if (prvNotifyQueueSetContainer(pxQueue, queueSEND_TO_BACK) != pdFALSE)
						{
							/* The semaphore is a member of a queue set, and
							posting	to the queue set caused a higher priority
							task to	unblock.  A context switch is required. */
							if (pxHigherPriorityTaskWoken != NULL)
							{
								*pxHigherPriorityTaskWoken = pdTRUE;
							}
							else
							{
								mtCOVERAGE_TEST_MARKER();
							}
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else
					{
						if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
						{
							if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
							{
								/* The task waiting has a higher priority so
								record that a context switch is required. */
								if (pxHigherPriorityTaskWoken != NULL)
								{
									*pxHigherPriorityTaskWoken = pdTRUE;
								}
								else
								{
									mtCOVERAGE_TEST_MARKER();
								}
							}
							else
							{
								mtCOVERAGE_TEST_MARKER();
							}
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
				}
#else  /* configUSE_QUEUE_SETS */
				{
					if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
					{
						if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
						{
							/* The task waiting has a higher priority so record that a
							context	switch is required. */
							if (pxHigherPriorityTaskWoken != NULL)
							{
								*pxHigherPriorityTaskWoken = pdTRUE;
							}
							else
							{
								mtCOVERAGE_TEST_MARKER();
							}
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
#endif /* configUSE_QUEUE_SETS */
			}
			else
			{
				/* Increment the lock count so the task that unlocks the queue
				knows that data was posted while it was locked. */
				pxQueue->cTxLock = (int8_t)(cTxLock + 1);
			}

			xReturn = pdPASS;
		}
		else
		{
			traceQUEUE_SEND_FROM_ISR_FAILED(pxQueue);
			xReturn = errQUEUE_FULL;
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);

	return xReturn;
}
/*-----------------------------------------------------------*/
// 从队列读数据
BaseType_t xQueueReceive(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait)
{
	// 入口时间设置否
	BaseType_t xEntryTimeSet = pdFALSE;
	TimeOut_t xTimeOut;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	/* Check the pointer is not NULL. */
	configASSERT((pxQueue));

	/* The buffer into which data is received can only be NULL if the data size
	is zero (so no data is copied into the buffer. */
	// 只有uxItemSize为0时，pvBuffer才能为NULL
	configASSERT(!(((pvBuffer) == NULL) && ((pxQueue)->uxItemSize != (UBaseType_t)0U)));

/* Cannot block if the scheduler is suspended. */
#if ((INCLUDE_xTaskGetSchedulerState == 1) || (configUSE_TIMERS == 1))
	{
		configASSERT(!((xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) && (xTicksToWait != 0)));
	}
#endif

	/* This function relaxes the coding standard somewhat to allow return
	statements within the function itself.  This is done in the interest
	of execution time efficiency. */

	for (;;)
	{
		// 进入临界区
		taskENTER_CRITICAL();
		{
			// queue中的信息条数
			const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;

			/* Is there data in the queue now?  To be running the calling task
			must be the highest priority task wanting to access the queue. */
			// 有数据
			if (uxMessagesWaiting > (UBaseType_t)0)
			{
				/* Data available, remove one item. */
				// 读取一个数据，并且将其移除
				prvCopyDataFromQueue(pxQueue, pvBuffer);
				traceQUEUE_RECEIVE(pxQueue);
				// 信息条数减少
				pxQueue->uxMessagesWaiting = uxMessagesWaiting - (UBaseType_t)1;

				/* There is now space in the queue, were any tasks waiting to
				post to the queue?  If so, unblock the highest priority waiting
				task. */
				// 从等待写入的事件组移除
				if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToSend)) == pdFALSE)
				{
					if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToSend)) != pdFALSE)
					{
						queueYIELD_IF_USING_PREEMPTION();
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}

				// 退出临界区
				taskEXIT_CRITICAL();
				// 读取成功
				return pdPASS;
			}
			// 无数据
			else
			{
				// 超时时间为0
				if (xTicksToWait == (TickType_t)0)
				{
					/* The queue was empty and no block time is specified (or
					the block time has expired) so leave now. */
					// 退出临界区
					taskEXIT_CRITICAL();
					traceQUEUE_RECEIVE_FAILED(pxQueue);
					// 返回数据空
					return errQUEUE_EMPTY;
				}
				// 未设置入口时间
				else if (xEntryTimeSet == pdFALSE)
				{
					/* The queue was empty and a block time was specified so
					configure the timeout structure. */
					// 设置超时时间结构体
					vTaskInternalSetTimeOutState(&xTimeOut);
					xEntryTimeSet = pdTRUE;
				}
				else
				{
					/* Entry time was already set. */
					mtCOVERAGE_TEST_MARKER();
				}
			}
		}
		taskEXIT_CRITICAL();

		/* Interrupts and other tasks can send to and receive from the queue
		now the critical section has been exited. */

		// queue无数据，且未超时，将会执行
		//  暂停调度
		vTaskSuspendAll();
		prvLockQueue(pxQueue);

		/* Update the timeout state to see if it has expired yet. */
		// 检查超时
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE)
		{
			/* The timeout has not expired.  If the queue is still empty place
			the task on the list of tasks waiting to receive from the queue. */
			// queue里无数据
			if (prvIsQueueEmpty(pxQueue) != pdFALSE)
			{
				traceBLOCKING_ON_QUEUE_RECEIVE(pxQueue);
				// 添加到等待读数据列表
				vTaskPlaceOnEventList(&(pxQueue->xTasksWaitingToReceive), xTicksToWait);
				prvUnlockQueue(pxQueue);
				// 恢复调度
				if (xTaskResumeAll() == pdFALSE)
				{
					portYIELD_WITHIN_API();
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			// 有数据，retry
			else
			{
				/* The queue contains data again.  Loop back to try and read the
				data. */
				prvUnlockQueue(pxQueue);
				(void)xTaskResumeAll();
			}
		}
		// 超时
		else
		{
			/* Timed out.  If there is no data in the queue exit, otherwise loop
			back and attempt to read the data. */
			prvUnlockQueue(pxQueue);
			(void)xTaskResumeAll();

			// 队列不空了 再次尝试读取
			if (prvIsQueueEmpty(pxQueue) != pdFALSE)
			{
				traceQUEUE_RECEIVE_FAILED(pxQueue);
				// 返回队列空
				return errQUEUE_EMPTY;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
	}
}
/*-----------------------------------------------------------*/

// 信号量、互斥量、取数据
BaseType_t xQueueSemaphoreTake(QueueHandle_t xQueue, TickType_t xTicksToWait)
{
	// 记录使用take的事件
	BaseType_t xEntryTimeSet = pdFALSE;
	TimeOut_t xTimeOut;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

#if (configUSE_MUTEXES == 1)
	BaseType_t xInheritanceOccurred = pdFALSE;
#endif

	/* Check the queue pointer is not NULL. */
	configASSERT((pxQueue));

	/* Check this really is a semaphore, in which case the item size will be
	0. */
	configASSERT(pxQueue->uxItemSize == 0);
	// 调度器不能是暂停状态
/* Cannot block if the scheduler is suspended. */
#if ((INCLUDE_xTaskGetSchedulerState == 1) || (configUSE_TIMERS == 1))
	{
		configASSERT(!((xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) && (xTicksToWait != 0)));
	}
#endif

	/* This function relaxes the coding standard somewhat to allow return
	statements within the function itself.  This is done in the interest
	of execution time efficiency.
	该函数在一定程度上放宽了编码标准，允许在函数本身中使用返回语句。 这样做是为了提高执行效率
	*/

	for (;;)
	{
		// 进入临界区
		taskENTER_CRITICAL();
		{
			// 信号量是item size为0的信号量，uxMessagesWaiting的数据量就是信号量的计数值
			/* Semaphores are queues with an item size of 0, and where the
			number of messages in the queue is the semaphore's count value. */
			const UBaseType_t uxSemaphoreCount = pxQueue->uxMessagesWaiting;

			/* Is there data in the queue now?  To be running the calling task
			must be the highest priority task wanting to access the queue. */
			// 队列里面是否有数据，访问队列的任务的优先级必须是最高优先级的任务
			if (uxSemaphoreCount > (UBaseType_t)0)
			{
				traceQUEUE_RECEIVE(pxQueue);

				/* Semaphores are queues with a data size of zero and where the
				messages waiting is the semaphore's count.  Reduce the count. */
				// 队列里面的数据减少
				pxQueue->uxMessagesWaiting = uxSemaphoreCount - (UBaseType_t)1;

// 互斥量
#if (configUSE_MUTEXES == 1)
				{
					if (pxQueue->uxQueueType == queueQUEUE_IS_MUTEX)
					{
						/* Record the information required to implement
						priority inheritance should it become necessary. */
						// 将互斥量的pxMutexHolder设置为调用take()的任务，并且将任务获取到的互斥量增加
						pxQueue->pxMutexHolder = (int8_t *)pvTaskIncrementMutexHeldCount(); /*lint !e961 Cast is not redundant as TaskHandle_t is a typedef. */
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
#endif /* configUSE_MUTEXES */

				/* Check to see if other tasks are blocked waiting to give the
				semaphore, and if so, unblock the highest priority such task. */
				// 检查有没有任务想要释放信号(写数据)到信号量
				if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToSend)) == pdFALSE)
				{
					// 解锁运行权限最高的任务到就序列表，将该任务从事件列表移除
					if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToSend)) != pdFALSE)
					{
						// 切换任务
						queueYIELD_IF_USING_PREEMPTION();
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
				// 退出临界区
				taskEXIT_CRITICAL();
				// 返回到called function 已经获取到了信号量
				return pdPASS;
			}
			// 队列里面没有数据
			else
			{
				// 不等待
				if (xTicksToWait == (TickType_t)0)
				{
/* For inheritance to have occurred there must have been an
initial timeout, and an adjusted timeout cannot become 0, as
if it were 0 the function would have exited. */
#if (configUSE_MUTEXES == 1)
					{
						// 优先级继承未发生
						configASSERT(xInheritanceOccurred == pdFALSE);
					}
#endif /* configUSE_MUTEXES */

					/* The semaphore count was 0 and no block time is specified
					(or the block time has expired) so exit now. */
					// 退出临界区
					taskEXIT_CRITICAL();
					traceQUEUE_RECEIVE_FAILED(pxQueue);
					// 返回called function，未获取到信号量
					return errQUEUE_EMPTY;
				}
				// 队列没有数据，等待超时时间
				else if (xEntryTimeSet == pdFALSE)
				{
					// 信号量计数为0，阻塞时间已经指定，配置超时结构体
					/* The semaphore count was 0 and a block time was specified
					so configure the timeout structure ready to block. */
					// 记录当前任务的时刻
					vTaskInternalSetTimeOutState(&xTimeOut);
					// 已经记录等待时刻
					xEntryTimeSet = pdTRUE;
				}
				else
				{
					/* Entry time was already set. */
					mtCOVERAGE_TEST_MARKER();
				}
			}
		}
		// 退出临界区
		taskEXIT_CRITICAL();

		/* Interrupts and other tasks can give to and take from the semaphore
		now the critical section has been exited. */

		// 暂缓调度
		vTaskSuspendAll();
		// 锁定队列
		prvLockQueue(pxQueue);

		/* Update the timeout state to see if it has expired yet. */
		// 检查超时是否发生

		// 无数据且任务等待未发生超时
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE)
		{
			/* A block time is specified and not expired.  If the semaphore
			count is 0 then enter the Blocked state to wait for a semaphore to
			become available.  As semaphores are implemented with queues the
			queue being empty is equivalent to the semaphore count being 0. */
			// 队列不为空? 检查uxMessagesWaiting长度
			if (prvIsQueueEmpty(pxQueue) != pdFALSE)
			{
				traceBLOCKING_ON_QUEUE_RECEIVE(pxQueue);

// 互斥量
#if (configUSE_MUTEXES == 1)
				{
					if (pxQueue->uxQueueType == queueQUEUE_IS_MUTEX)
					{
						taskENTER_CRITICAL();
						{
							// 让当前的任务的优先级等于pxMutexHolder的优先级。并添加到延时列表
							xInheritanceOccurred = xTaskPriorityInherit((void *)pxQueue->pxMutexHolder);
						}
						taskEXIT_CRITICAL();
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
#endif

				// 将任务放置在等待事件的任务列表中
				vTaskPlaceOnEventList(&(pxQueue->xTasksWaitingToReceive), xTicksToWait);
				// 解锁队列
				prvUnlockQueue(pxQueue);
				// 恢复任务调度
				if (xTaskResumeAll() == pdFALSE)
				{
					portYIELD_WITHIN_API();
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			// 未发生超时，且队列里面有了数据，尝试再次获取数据继续for(;;)
			else
			{
				/* There was no timeout and the semaphore count was not 0, so
				attempt to take the semaphore again. */
				// 队列不为空 且未发生超时尝试获取数据
				prvUnlockQueue(pxQueue);
				// 恢复任务调度
				(void)xTaskResumeAll();
			}
		}
		// 无数据且，任务等待已经超时
		else
		{
			/* Timed out. */
			prvUnlockQueue(pxQueue);
			// 回复调度
			(void)xTaskResumeAll();

			/* If the semaphore count is 0 exit now as the timeout has
			expired.  Otherwise return to attempt to take the semaphore that is
			known to be available.  As semaphores are implemented by queues the
			queue being empty is equivalent to the semaphore count being 0. */
			// 任务已经超时但是 队列不为空
			if (prvIsQueueEmpty(pxQueue) != pdFALSE)
			{
				// 互斥量
#if (configUSE_MUTEXES == 1)
				{
					/* xInheritanceOccurred could only have be set if
					pxQueue->uxQueueType == queueQUEUE_IS_MUTEX so no need to
					test the mutex type again to check it is actually a mutex. */
					// 优先级继承未发生
					if (xInheritanceOccurred != pdFALSE)
					{
						taskENTER_CRITICAL();
						{
							UBaseType_t uxHighestWaitingPriority;

							/* This task blocking on the mutex caused another
							task to inherit this task's priority.  Now this task
							has timed out the priority should be disinherited
							again, but only as low as the next highest priority
							task that is waiting for the same mutex. */

							// 计算出优先级
							uxHighestWaitingPriority = prvGetDisinheritPriorityAfterTimeout(pxQueue);
							// 剥夺优先级继承权
							vTaskPriorityDisinheritAfterTimeout((void *)pxQueue->pxMutexHolder, uxHighestWaitingPriority);
						}
						taskEXIT_CRITICAL();
					}
				}
#endif /* configUSE_MUTEXES */

				traceQUEUE_RECEIVE_FAILED(pxQueue);
				return errQUEUE_EMPTY;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
	}
}
/*-----------------------------------------------------------*/

BaseType_t xQueuePeek(QueueHandle_t xQueue, void *const pvBuffer, TickType_t xTicksToWait)
{
	BaseType_t xEntryTimeSet = pdFALSE;
	TimeOut_t xTimeOut;
	int8_t *pcOriginalReadPosition;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	/* Check the pointer is not NULL. */
	configASSERT((pxQueue));

	/* The buffer into which data is received can only be NULL if the data size
	is zero (so no data is copied into the buffer. */
	configASSERT(!(((pvBuffer) == NULL) && ((pxQueue)->uxItemSize != (UBaseType_t)0U)));

/* Cannot block if the scheduler is suspended. */
#if ((INCLUDE_xTaskGetSchedulerState == 1) || (configUSE_TIMERS == 1))
	{
		configASSERT(!((xTaskGetSchedulerState() == taskSCHEDULER_SUSPENDED) && (xTicksToWait != 0)));
	}
#endif

	/* This function relaxes the coding standard somewhat to allow return
	statements within the function itself.  This is done in the interest
	of execution time efficiency. */

	for (;;)
	{
		taskENTER_CRITICAL();
		{
			const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;

			/* Is there data in the queue now?  To be running the calling task
			must be the highest priority task wanting to access the queue. */
			if (uxMessagesWaiting > (UBaseType_t)0)
			{
				/* Remember the read position so it can be reset after the data
				is read from the queue as this function is only peeking the
				data, not removing it. */
				pcOriginalReadPosition = pxQueue->u.pcReadFrom;

				prvCopyDataFromQueue(pxQueue, pvBuffer);
				traceQUEUE_PEEK(pxQueue);

				/* The data is not being removed, so reset the read pointer. */
				pxQueue->u.pcReadFrom = pcOriginalReadPosition;

				/* The data is being left in the queue, so see if there are
				any other tasks waiting for the data. */
				if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
				{
					if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
					{
						/* The task waiting has a higher priority than this task. */
						queueYIELD_IF_USING_PREEMPTION();
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}

				taskEXIT_CRITICAL();
				return pdPASS;
			}
			else
			{
				if (xTicksToWait == (TickType_t)0)
				{
					/* The queue was empty and no block time is specified (or
					the block time has expired) so leave now. */
					taskEXIT_CRITICAL();
					traceQUEUE_PEEK_FAILED(pxQueue);
					return errQUEUE_EMPTY;
				}
				else if (xEntryTimeSet == pdFALSE)
				{
					/* The queue was empty and a block time was specified so
					configure the timeout structure ready to enter the blocked
					state. */
					vTaskInternalSetTimeOutState(&xTimeOut);
					xEntryTimeSet = pdTRUE;
				}
				else
				{
					/* Entry time was already set. */
					mtCOVERAGE_TEST_MARKER();
				}
			}
		}
		taskEXIT_CRITICAL();

		/* Interrupts and other tasks can send to and receive from the queue
		now the critical section has been exited. */

		vTaskSuspendAll();
		prvLockQueue(pxQueue);

		/* Update the timeout state to see if it has expired yet. */
		if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) == pdFALSE)
		{
			/* Timeout has not expired yet, check to see if there is data in the
			queue now, and if not enter the Blocked state to wait for data. */
			if (prvIsQueueEmpty(pxQueue) != pdFALSE)
			{
				traceBLOCKING_ON_QUEUE_PEEK(pxQueue);
				vTaskPlaceOnEventList(&(pxQueue->xTasksWaitingToReceive), xTicksToWait);
				prvUnlockQueue(pxQueue);
				if (xTaskResumeAll() == pdFALSE)
				{
					portYIELD_WITHIN_API();
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				/* There is data in the queue now, so don't enter the blocked
				state, instead return to try and obtain the data. */
				prvUnlockQueue(pxQueue);
				(void)xTaskResumeAll();
			}
		}
		else
		{
			/* The timeout has expired.  If there is still no data in the queue
			exit, otherwise go back and try to read the data again. */
			prvUnlockQueue(pxQueue);
			(void)xTaskResumeAll();

			if (prvIsQueueEmpty(pxQueue) != pdFALSE)
			{
				traceQUEUE_PEEK_FAILED(pxQueue);
				return errQUEUE_EMPTY;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
	}
}
/*-----------------------------------------------------------*/

BaseType_t xQueueReceiveFromISR(QueueHandle_t xQueue, void *const pvBuffer, BaseType_t *const pxHigherPriorityTaskWoken)
{
	BaseType_t xReturn;
	UBaseType_t uxSavedInterruptStatus;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	configASSERT(pxQueue);
	configASSERT(!((pvBuffer == NULL) && (pxQueue->uxItemSize != (UBaseType_t)0U)));

	/* RTOS ports that support interrupt nesting have the concept of a maximum
	system call (or maximum API call) interrupt priority.  Interrupts that are
	above the maximum system call priority are kept permanently enabled, even
	when the RTOS kernel is in a critical section, but cannot make any calls to
	FreeRTOS API functions.  If configASSERT() is defined in FreeRTOSConfig.h
	then portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
	failure if a FreeRTOS API function is called from an interrupt that has been
	assigned a priority above the configured maximum system call priority.
	Only FreeRTOS functions that end in FromISR can be called from interrupts
	that have been assigned a priority at or (logically) below the maximum
	system call	interrupt priority.  FreeRTOS maintains a separate interrupt
	safe API to ensure interrupt entry is as fast and as simple as possible.
	More information (albeit Cortex-M specific) is provided on the following
	link: http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

	uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;

		/* Cannot block in an ISR, so check there is data available. */
		if (uxMessagesWaiting > (UBaseType_t)0)
		{
			const int8_t cRxLock = pxQueue->cRxLock;

			traceQUEUE_RECEIVE_FROM_ISR(pxQueue);

			prvCopyDataFromQueue(pxQueue, pvBuffer);
			pxQueue->uxMessagesWaiting = uxMessagesWaiting - (UBaseType_t)1;

			/* If the queue is locked the event list will not be modified.
			Instead update the lock count so the task that unlocks the queue
			will know that an ISR has removed data while the queue was
			locked. */
			if (cRxLock == queueUNLOCKED)
			{
				if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToSend)) == pdFALSE)
				{
					if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToSend)) != pdFALSE)
					{
						/* The task waiting has a higher priority than us so
						force a context switch. */
						if (pxHigherPriorityTaskWoken != NULL)
						{
							*pxHigherPriorityTaskWoken = pdTRUE;
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				/* Increment the lock count so the task that unlocks the queue
				knows that data was removed while it was locked. */
				pxQueue->cRxLock = (int8_t)(cRxLock + 1);
			}

			xReturn = pdPASS;
		}
		else
		{
			xReturn = pdFAIL;
			traceQUEUE_RECEIVE_FROM_ISR_FAILED(pxQueue);
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueuePeekFromISR(QueueHandle_t xQueue, void *const pvBuffer)
{
	BaseType_t xReturn;
	UBaseType_t uxSavedInterruptStatus;
	int8_t *pcOriginalReadPosition;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	configASSERT(pxQueue);
	configASSERT(!((pvBuffer == NULL) && (pxQueue->uxItemSize != (UBaseType_t)0U)));
	configASSERT(pxQueue->uxItemSize != 0); /* Can't peek a semaphore. */

	/* RTOS ports that support interrupt nesting have the concept of a maximum
	system call (or maximum API call) interrupt priority.  Interrupts that are
	above the maximum system call priority are kept permanently enabled, even
	when the RTOS kernel is in a critical section, but cannot make any calls to
	FreeRTOS API functions.  If configASSERT() is defined in FreeRTOSConfig.h
	then portASSERT_IF_INTERRUPT_PRIORITY_INVALID() will result in an assertion
	failure if a FreeRTOS API function is called from an interrupt that has been
	assigned a priority above the configured maximum system call priority.
	Only FreeRTOS functions that end in FromISR can be called from interrupts
	that have been assigned a priority at or (logically) below the maximum
	system call	interrupt priority.  FreeRTOS maintains a separate interrupt
	safe API to ensure interrupt entry is as fast and as simple as possible.
	More information (albeit Cortex-M specific) is provided on the following
	link: http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	portASSERT_IF_INTERRUPT_PRIORITY_INVALID();

	uxSavedInterruptStatus = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		/* Cannot block in an ISR, so check there is data available. */
		if (pxQueue->uxMessagesWaiting > (UBaseType_t)0)
		{
			traceQUEUE_PEEK_FROM_ISR(pxQueue);

			/* Remember the read position so it can be reset as nothing is
			actually being removed from the queue. */
			pcOriginalReadPosition = pxQueue->u.pcReadFrom;
			prvCopyDataFromQueue(pxQueue, pvBuffer);
			pxQueue->u.pcReadFrom = pcOriginalReadPosition;

			xReturn = pdPASS;
		}
		else
		{
			xReturn = pdFAIL;
			traceQUEUE_PEEK_FROM_ISR_FAILED(pxQueue);
		}
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR(uxSavedInterruptStatus);

	return xReturn;
}
/*-----------------------------------------------------------*/

UBaseType_t uxQueueMessagesWaiting(const QueueHandle_t xQueue)
{
	UBaseType_t uxReturn;

	configASSERT(xQueue);

	taskENTER_CRITICAL();
	{
		uxReturn = ((Queue_t *)xQueue)->uxMessagesWaiting;
	}
	taskEXIT_CRITICAL();

	return uxReturn;
} /*lint !e818 Pointer cannot be declared const as xQueue is a typedef not pointer. */
/*-----------------------------------------------------------*/

UBaseType_t uxQueueSpacesAvailable(const QueueHandle_t xQueue)
{
	UBaseType_t uxReturn;
	Queue_t *pxQueue;

	pxQueue = (Queue_t *)xQueue;
	configASSERT(pxQueue);

	taskENTER_CRITICAL();
	{
		uxReturn = pxQueue->uxLength - pxQueue->uxMessagesWaiting;
	}
	taskEXIT_CRITICAL();

	return uxReturn;
} /*lint !e818 Pointer cannot be declared const as xQueue is a typedef not pointer. */
/*-----------------------------------------------------------*/

UBaseType_t uxQueueMessagesWaitingFromISR(const QueueHandle_t xQueue)
{
	UBaseType_t uxReturn;

	configASSERT(xQueue);

	uxReturn = ((Queue_t *)xQueue)->uxMessagesWaiting;

	return uxReturn;
} /*lint !e818 Pointer cannot be declared const as xQueue is a typedef not pointer. */
/*-----------------------------------------------------------*/

// 删除队列 释放分配内存
void vQueueDelete(QueueHandle_t xQueue)
{
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	configASSERT(pxQueue);
	traceQUEUE_DELETE(pxQueue);

#if (configQUEUE_REGISTRY_SIZE > 0)
	{
		// 从队列注册表中删除该队列
		vQueueUnregisterQueue(pxQueue);
	}
#endif

#if ((configSUPPORT_DYNAMIC_ALLOCATION == 1) && (configSUPPORT_STATIC_ALLOCATION == 0))
	{
		/* The queue can only have been allocated dynamically - free it
		again. */
		vPortFree(pxQueue);
	}
#elif ((configSUPPORT_DYNAMIC_ALLOCATION == 1) && (configSUPPORT_STATIC_ALLOCATION == 1))
	{
		/* The queue could have been allocated statically or dynamically, so
		check before attempting to free the memory. */
		if (pxQueue->ucStaticallyAllocated == (uint8_t)pdFALSE)
		{
			// 释放队列的内存
			vPortFree(pxQueue);
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
#else
	{
		/* The queue must have been statically allocated, so is not going to be
		deleted.  Avoid compiler warnings about the unused parameter. */
		(void)pxQueue;
	}
#endif /* configSUPPORT_DYNAMIC_ALLOCATION */
}
/*-----------------------------------------------------------*/

#if (configUSE_TRACE_FACILITY == 1)

UBaseType_t uxQueueGetQueueNumber(QueueHandle_t xQueue)
{
	return ((Queue_t *)xQueue)->uxQueueNumber;
}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/

#if (configUSE_TRACE_FACILITY == 1)

void vQueueSetQueueNumber(QueueHandle_t xQueue, UBaseType_t uxQueueNumber)
{
	((Queue_t *)xQueue)->uxQueueNumber = uxQueueNumber;
}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/

#if (configUSE_TRACE_FACILITY == 1)

uint8_t ucQueueGetQueueType(QueueHandle_t xQueue)
{
	return ((Queue_t *)xQueue)->ucQueueType;
}

#endif /* configUSE_TRACE_FACILITY */
/*-----------------------------------------------------------*/

#if (configUSE_MUTEXES == 1)

// 在超时后 取消任务的优先级继承权
static UBaseType_t prvGetDisinheritPriorityAfterTimeout(const Queue_t *const pxQueue)
{
	UBaseType_t uxHighestPriorityOfWaitingTasks;

	/* If a task waiting for a mutex causes the mutex holder to inherit a
	priority, but the waiting task times out, then the holder should
	disinherit the priority - but only down to the highest priority of any
	other tasks that are waiting for the same mutex.  For this purpose,
	return the priority of the highest priority task that is waiting for the
	mutex. */
	// 有任务在等待读取
	if (listCURRENT_LIST_LENGTH(&(pxQueue->xTasksWaitingToReceive)) > 0)
	{
		uxHighestPriorityOfWaitingTasks = configMAX_PRIORITIES - listGET_ITEM_VALUE_OF_HEAD_ENTRY(&(pxQueue->xTasksWaitingToReceive));
	}
	else
	{
		uxHighestPriorityOfWaitingTasks = tskIDLE_PRIORITY;
	}

	return uxHighestPriorityOfWaitingTasks;
}

#endif /* configUSE_MUTEXES */
/*-----------------------------------------------------------*/

// 复制数据到队列 改变queue.pcWriteTo
static BaseType_t prvCopyDataToQueue(Queue_t *const pxQueue, const void *pvItemToQueue, const BaseType_t xPosition)
{
	BaseType_t xReturn = pdFALSE;
	UBaseType_t uxMessagesWaiting;

	/* This function is called from a critical section. */

	uxMessagesWaiting = pxQueue->uxMessagesWaiting;

	// 单条信息的长度为0 是互斥量和信号量?
	if (pxQueue->uxItemSize == (UBaseType_t)0)
	{
#if (configUSE_MUTEXES == 1)
		{
			// 互斥量释放资源 释放互斥量往互斥量写数据，表示互斥量释放
			if (pxQueue->uxQueueType == queueQUEUE_IS_MUTEX)
			{
				/* The mutex is no longer being held. */
				// 剥夺之前任务的优先级的持有权 新创建的互斥量不存在继承优先级的问题
				xReturn = xTaskPriorityDisinherit((void *)pxQueue->pxMutexHolder);
				pxQueue->pxMutexHolder = NULL;
			}
			// 信号量
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
#endif /* configUSE_MUTEXES */
	}
	// 先进先出
	else if (xPosition == queueSEND_TO_BACK)
	{
		// 复制到队列有数据的位置的下一个
		(void)memcpy((void *)pxQueue->pcWriteTo, pvItemToQueue, (size_t)pxQueue->uxItemSize); /*lint !e961 !e418 MISRA exception as the casts are only redundant for some ports, plus previous logic ensures a null pointer can only be passed to memcpy() if the copy size is 0. */
		// 增加位置
		pxQueue->pcWriteTo += pxQueue->uxItemSize;
		// 写到了末尾了 换位子
		if (pxQueue->pcWriteTo >= pxQueue->pcTail) /*lint !e946 MISRA exception justified as comparison of pointers is the cleanest solution. */
		{
			// 换到队列头
			pxQueue->pcWriteTo = pxQueue->pcHead;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	// xPosition != queueSEND_TO_BACK and pxQueue->uxItemSize != 0(xPosition = queueOVERWRITE or queueSEND_TO_FRONT 后进先出 和 覆写)
	else
	{
		// 写到下次读的位置
		(void)memcpy((void *)pxQueue->u.pcReadFrom, pvItemToQueue, (size_t)pxQueue->uxItemSize); /*lint !e961 MISRA exception as the casts are only redundant for some ports. */
		// 回退reda
		pxQueue->u.pcReadFrom -= pxQueue->uxItemSize;
		// 超过fifo的地址，到最末尾
		if (pxQueue->u.pcReadFrom < pxQueue->pcHead) /*lint !e946 MISRA exception justified as comparison of pointers is the cleanest solution. */
		{
			// 队列末尾
			pxQueue->u.pcReadFrom = (pxQueue->pcTail - pxQueue->uxItemSize);
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		// 覆写
		if (xPosition == queueOVERWRITE)
		{
			if (uxMessagesWaiting > (UBaseType_t)0)
			{
				/* An item is not being added but overwritten, so subtract
				one from the recorded number of items in the queue so when
				one is added again below the number of recorded items remains
				correct. */
				// 减去消息量，确保queue只有一条数据
				--uxMessagesWaiting;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}

	// 增加数据条目
	pxQueue->uxMessagesWaiting = uxMessagesWaiting + (UBaseType_t)1;

	return xReturn;
}
/*-----------------------------------------------------------*/

// 从队列复制数据到buff 改变 pcReadFrom
static void prvCopyDataFromQueue(Queue_t *const pxQueue, void *const pvBuffer)
{
	// pcReadFrom 会始终比 pcWriteTo 落后，哪怕是在初始化时，也要落后一个位置

	// 不为互斥量
	if (pxQueue->uxItemSize != (UBaseType_t)0)
	{
		// pcReadFrom向前进位
		pxQueue->u.pcReadFrom += pxQueue->uxItemSize;
		// 到了末尾，更新index
		if (pxQueue->u.pcReadFrom >= pxQueue->pcTail) /*lint !e946 MISRA exception justified as use of the relational operator is the cleanest solutions. */
		{
			pxQueue->u.pcReadFrom = pxQueue->pcHead;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
		// 复制数据到buff
		(void)memcpy((void *)pvBuffer, (void *)pxQueue->u.pcReadFrom, (size_t)pxQueue->uxItemSize); /*lint !e961 !e418 MISRA exception as the casts are only redundant for some ports.  Also previous logic ensures a null pointer can only be passed to memcpy() when the count is 0. */
	}
}
/*-----------------------------------------------------------*/

// 解锁队列
static void prvUnlockQueue(Queue_t *const pxQueue)
{
	// 该函数必须由SCHEDULER SUSPENDED调用
	/* THIS FUNCTION MUST BE CALLED WITH THE SCHEDULER SUSPENDED. */

	/* The lock counts contains the number of extra data items placed or
	removed from the queue while the queue was locked.  When a queue is
	locked items can be added or removed, but the event lists cannot be
	updated. */
	// 两个锁定计数，包含了在锁定队列期间，进队列和出队列的数据长度，当队列锁定时，
	// 队列项是可以加入或者移除队列的，但是对应的列表不会更新。
	taskENTER_CRITICAL();
	{
		int8_t cTxLock = pxQueue->cTxLock;

		/* See if data was added to the queue while it was locked. */
		// 检查在队列上锁期间，是否有数据添加到了队列
		while (cTxLock > queueLOCKED_UNMODIFIED)
		{
/* Data was posted while the queue was locked.  Are any tasks
blocked waiting for data to become available? */
// 开启队列集功能
#if (configUSE_QUEUE_SETS == 1)
			{
				if (pxQueue->pxQueueSetContainer != NULL)
				{
					if (prvNotifyQueueSetContainer(pxQueue, queueSEND_TO_BACK) != pdFALSE)
					{
						/* The queue is a member of a queue set, and posting to
						the queue set caused a higher priority task to unblock.
						A context switch is required. */
						vTaskMissedYield();
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
				else
				{
					/* Tasks that are removed from the event list will get
					added to the pending ready list as the scheduler is still
					suspended. */
					if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
					{
						if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
						{
							/* The task waiting has a higher priority so record that a
							context	switch is required. */
							vTaskMissedYield();
						}
						else
						{
							mtCOVERAGE_TEST_MARKER();
						}
					}
					else
					{
						break;
					}
				}
			}
#else  /* configUSE_QUEUE_SETS */
			{
				/* Tasks that are removed from the event list will get added to
				the pending ready list as the scheduler is still suspended. */
				// 当调度器还是处于suspended，任务将会从事件列表移除，并添加到就绪列表
				if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
				{
					// 等待接收
					if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
					{
						/* The task waiting has a higher priority so record that
						a context switch is required. */
						vTaskMissedYield();
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}
				}
				else
				{
					break;
				}
			}
#endif /* configUSE_QUEUE_SETS */

			--cTxLock;
		}

		pxQueue->cTxLock = queueUNLOCKED;
	}
	taskEXIT_CRITICAL();

	/* Do the same for the Rx lock. */
	taskENTER_CRITICAL();
	{
		int8_t cRxLock = pxQueue->cRxLock;

		while (cRxLock > queueLOCKED_UNMODIFIED)
		{
			if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToSend)) == pdFALSE)
			{
				if (xTaskRemoveFromEventList(&(pxQueue->xTasksWaitingToSend)) != pdFALSE)
				{
					vTaskMissedYield();
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}

				--cRxLock;
			}
			else
			{
				break;
			}
		}

		pxQueue->cRxLock = queueUNLOCKED;
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/
// 检查队列是否为空
static BaseType_t prvIsQueueEmpty(const Queue_t *pxQueue)
{
	BaseType_t xReturn;

	taskENTER_CRITICAL();
	{
		// 在队列里面的数据长度为0
		if (pxQueue->uxMessagesWaiting == (UBaseType_t)0)
		{
			// 队列是空的
			xReturn = pdTRUE;
		}
		else
		{
			xReturn = pdFALSE;
		}
	}
	taskEXIT_CRITICAL();

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueIsQueueEmptyFromISR(const QueueHandle_t xQueue)
{
	BaseType_t xReturn;

	configASSERT(xQueue);
	if (((Queue_t *)xQueue)->uxMessagesWaiting == (UBaseType_t)0)
	{
		xReturn = pdTRUE;
	}
	else
	{
		xReturn = pdFALSE;
	}

	return xReturn;
} /*lint !e818 xQueue could not be pointer to const because it is a typedef. */
/*-----------------------------------------------------------*/

static BaseType_t prvIsQueueFull(const Queue_t *pxQueue)
{
	BaseType_t xReturn;

	taskENTER_CRITICAL();
	{
		if (pxQueue->uxMessagesWaiting == pxQueue->uxLength)
		{
			xReturn = pdTRUE;
		}
		else
		{
			xReturn = pdFALSE;
		}
	}
	taskEXIT_CRITICAL();

	return xReturn;
}
/*-----------------------------------------------------------*/

BaseType_t xQueueIsQueueFullFromISR(const QueueHandle_t xQueue)
{
	BaseType_t xReturn;

	configASSERT(xQueue);
	if (((Queue_t *)xQueue)->uxMessagesWaiting == ((Queue_t *)xQueue)->uxLength)
	{
		xReturn = pdTRUE;
	}
	else
	{
		xReturn = pdFALSE;
	}

	return xReturn;
} /*lint !e818 xQueue could not be pointer to const because it is a typedef. */
/*-----------------------------------------------------------*/

#if (configUSE_CO_ROUTINES == 1)

BaseType_t xQueueCRSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait)
{
	BaseType_t xReturn;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	/* If the queue is already full we may have to block.  A critical section
	is required to prevent an interrupt removing something from the queue
	between the check to see if the queue is full and blocking on the queue. */
	portDISABLE_INTERRUPTS();
	{
		if (prvIsQueueFull(pxQueue) != pdFALSE)
		{
			/* The queue is full - do we want to block or just leave without
			posting? */
			if (xTicksToWait > (TickType_t)0)
			{
				/* As this is called from a coroutine we cannot block directly, but
				return indicating that we need to block. */
				vCoRoutineAddToDelayedList(xTicksToWait, &(pxQueue->xTasksWaitingToSend));
				portENABLE_INTERRUPTS();
				return errQUEUE_BLOCKED;
			}
			else
			{
				portENABLE_INTERRUPTS();
				return errQUEUE_FULL;
			}
		}
	}
	portENABLE_INTERRUPTS();

	portDISABLE_INTERRUPTS();
	{
		if (pxQueue->uxMessagesWaiting < pxQueue->uxLength)
		{
			/* There is room in the queue, copy the data into the queue. */
			prvCopyDataToQueue(pxQueue, pvItemToQueue, queueSEND_TO_BACK);
			xReturn = pdPASS;

			/* Were any co-routines waiting for data to become available? */
			if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
			{
				/* In this instance the co-routine could be placed directly
				into the ready list as we are within a critical section.
				Instead the same pending ready list mechanism is used as if
				the event were caused from within an interrupt. */
				if (xCoRoutineRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
				{
					/* The co-routine waiting has a higher priority so record
					that a yield might be appropriate. */
					xReturn = errQUEUE_YIELD;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			xReturn = errQUEUE_FULL;
		}
	}
	portENABLE_INTERRUPTS();

	return xReturn;
}

#endif /* configUSE_CO_ROUTINES */
/*-----------------------------------------------------------*/

#if (configUSE_CO_ROUTINES == 1)

BaseType_t xQueueCRReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait)
{
	BaseType_t xReturn;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	/* If the queue is already empty we may have to block.  A critical section
	is required to prevent an interrupt adding something to the queue
	between the check to see if the queue is empty and blocking on the queue. */
	portDISABLE_INTERRUPTS();
	{
		if (pxQueue->uxMessagesWaiting == (UBaseType_t)0)
		{
			/* There are no messages in the queue, do we want to block or just
			leave with nothing? */
			if (xTicksToWait > (TickType_t)0)
			{
				/* As this is a co-routine we cannot block directly, but return
				indicating that we need to block. */
				vCoRoutineAddToDelayedList(xTicksToWait, &(pxQueue->xTasksWaitingToReceive));
				portENABLE_INTERRUPTS();
				return errQUEUE_BLOCKED;
			}
			else
			{
				portENABLE_INTERRUPTS();
				return errQUEUE_FULL;
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	portENABLE_INTERRUPTS();

	portDISABLE_INTERRUPTS();
	{
		if (pxQueue->uxMessagesWaiting > (UBaseType_t)0)
		{
			/* Data is available from the queue. */
			pxQueue->u.pcReadFrom += pxQueue->uxItemSize;
			if (pxQueue->u.pcReadFrom >= pxQueue->pcTail)
			{
				pxQueue->u.pcReadFrom = pxQueue->pcHead;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
			--(pxQueue->uxMessagesWaiting);
			(void)memcpy((void *)pvBuffer, (void *)pxQueue->u.pcReadFrom, (unsigned)pxQueue->uxItemSize);

			xReturn = pdPASS;

			/* Were any co-routines waiting for space to become available? */
			if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToSend)) == pdFALSE)
			{
				/* In this instance the co-routine could be placed directly
				into the ready list as we are within a critical section.
				Instead the same pending ready list mechanism is used as if
				the event were caused from within an interrupt. */
				if (xCoRoutineRemoveFromEventList(&(pxQueue->xTasksWaitingToSend)) != pdFALSE)
				{
					xReturn = errQUEUE_YIELD;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			xReturn = pdFAIL;
		}
	}
	portENABLE_INTERRUPTS();

	return xReturn;
}

#endif /* configUSE_CO_ROUTINES */
/*-----------------------------------------------------------*/

#if (configUSE_CO_ROUTINES == 1)

BaseType_t xQueueCRSendFromISR(QueueHandle_t xQueue, const void *pvItemToQueue, BaseType_t xCoRoutinePreviouslyWoken)
{
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	/* Cannot block within an ISR so if there is no space on the queue then
	exit without doing anything. */
	if (pxQueue->uxMessagesWaiting < pxQueue->uxLength)
	{
		prvCopyDataToQueue(pxQueue, pvItemToQueue, queueSEND_TO_BACK);

		/* We only want to wake one co-routine per ISR, so check that a
		co-routine has not already been woken. */
		if (xCoRoutinePreviouslyWoken == pdFALSE)
		{
			if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToReceive)) == pdFALSE)
			{
				if (xCoRoutineRemoveFromEventList(&(pxQueue->xTasksWaitingToReceive)) != pdFALSE)
				{
					return pdTRUE;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	return xCoRoutinePreviouslyWoken;
}

#endif /* configUSE_CO_ROUTINES */
/*-----------------------------------------------------------*/

#if (configUSE_CO_ROUTINES == 1)

BaseType_t xQueueCRReceiveFromISR(QueueHandle_t xQueue, void *pvBuffer, BaseType_t *pxCoRoutineWoken)
{
	BaseType_t xReturn;
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	/* We cannot block from an ISR, so check there is data available. If
	not then just leave without doing anything. */
	if (pxQueue->uxMessagesWaiting > (UBaseType_t)0)
	{
		/* Copy the data from the queue. */
		pxQueue->u.pcReadFrom += pxQueue->uxItemSize;
		if (pxQueue->u.pcReadFrom >= pxQueue->pcTail)
		{
			pxQueue->u.pcReadFrom = pxQueue->pcHead;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
		--(pxQueue->uxMessagesWaiting);
		(void)memcpy((void *)pvBuffer, (void *)pxQueue->u.pcReadFrom, (unsigned)pxQueue->uxItemSize);

		if ((*pxCoRoutineWoken) == pdFALSE)
		{
			if (listLIST_IS_EMPTY(&(pxQueue->xTasksWaitingToSend)) == pdFALSE)
			{
				if (xCoRoutineRemoveFromEventList(&(pxQueue->xTasksWaitingToSend)) != pdFALSE)
				{
					*pxCoRoutineWoken = pdTRUE;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}

#endif /* configUSE_CO_ROUTINES */
/*-----------------------------------------------------------*/

#if (configQUEUE_REGISTRY_SIZE > 0)

// 注册队列到队列注册表中去
void vQueueAddToRegistry(QueueHandle_t xQueue, const char *pcQueueName) /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
{
	UBaseType_t ux;

	/* See if there is an empty space in the registry.  A NULL name denotes
	a free slot. */
	for (ux = (UBaseType_t)0U; ux < (UBaseType_t)configQUEUE_REGISTRY_SIZE; ux++)
	{
		// pcQueueName为null 证明该插槽为空
		if (xQueueRegistry[ux].pcQueueName == NULL)
		{
			/* Store the information on this queue. */
			// 储存队列信息到队列注册区域中
			// 队列名
			xQueueRegistry[ux].pcQueueName = pcQueueName;
			// 队列句柄
			xQueueRegistry[ux].xHandle = xQueue;

			traceQUEUE_REGISTRY_ADD(xQueue, pcQueueName);
			break;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
}

#endif /* configQUEUE_REGISTRY_SIZE */
/*-----------------------------------------------------------*/

#if (configQUEUE_REGISTRY_SIZE > 0)

// 获取队列的名字
const char *pcQueueGetName(QueueHandle_t xQueue) /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
{
	UBaseType_t ux;
	// 默认没有找到数据
	const char *pcReturn = NULL; /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

	/* Note there is nothing here to protect against another task adding or
	removing entries from the registry while it is being searched. */
	for (ux = (UBaseType_t)0U; ux < (UBaseType_t)configQUEUE_REGISTRY_SIZE; ux++)
	{
		if (xQueueRegistry[ux].xHandle == xQueue)
		{
			// 找到了数据
			pcReturn = xQueueRegistry[ux].pcQueueName;
			break;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}

	return pcReturn;
} /*lint !e818 xQueue cannot be a pointer to const because it is a typedef. */

#endif /* configQUEUE_REGISTRY_SIZE */
/*-----------------------------------------------------------*/

#if (configQUEUE_REGISTRY_SIZE > 0)

// 从队列注册表中删除队列
void vQueueUnregisterQueue(QueueHandle_t xQueue)
{
	UBaseType_t ux;

	/* See if the handle of the queue being unregistered in actually in the
	registry. */
	// 迭代队列注册表，找到需要删除的队列
	for (ux = (UBaseType_t)0U; ux < (UBaseType_t)configQUEUE_REGISTRY_SIZE; ux++)
	{
		if (xQueueRegistry[ux].xHandle == xQueue)
		{
			// 设置名字为NULL，表示该插槽为空
			/* Set the name to NULL to show that this slot if free again. */
			xQueueRegistry[ux].pcQueueName = NULL;

			/* Set the handle to NULL to ensure the same queue handle cannot
			appear in the registry twice if it is added, removed, then
			added again. */
			xQueueRegistry[ux].xHandle = (QueueHandle_t)0;
			break;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}

} /*lint !e818 xQueue could not be pointer to const because it is a typedef. */

#endif /* configQUEUE_REGISTRY_SIZE */
/*-----------------------------------------------------------*/

#if (configUSE_TIMERS == 1)

void vQueueWaitForMessageRestricted(QueueHandle_t xQueue, TickType_t xTicksToWait, const BaseType_t xWaitIndefinitely)
{
	Queue_t *const pxQueue = (Queue_t *)xQueue;

	/* This function should not be called by application code hence the
	'Restricted' in its name.  It is not part of the public API.  It is
	designed for use by kernel code, and has special calling requirements.
	It can result in vListInsert() being called on a list that can only
	possibly ever have one item in it, so the list will be fast, but even
	so it should be called with the scheduler locked and not from a critical
	section. */

	/* Only do anything if there are no messages in the queue.  This function
	will not actually cause the task to block, just place it on a blocked
	list.  It will not block until the scheduler is unlocked - at which
	time a yield will be performed.  If an item is added to the queue while
	the queue is locked, and the calling task blocks on the queue, then the
	calling task will be immediately unblocked when the queue is unlocked. */
	prvLockQueue(pxQueue);
	if (pxQueue->uxMessagesWaiting == (UBaseType_t)0U)
	{
		/* There is nothing in the queue, block for the specified period. */
		vTaskPlaceOnEventListRestricted(&(pxQueue->xTasksWaitingToReceive), xTicksToWait, xWaitIndefinitely);
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
	prvUnlockQueue(pxQueue);
}

#endif /* configUSE_TIMERS */
/*-----------------------------------------------------------*/

#if ((configUSE_QUEUE_SETS == 1) && (configSUPPORT_DYNAMIC_ALLOCATION == 1))

QueueSetHandle_t xQueueCreateSet(const UBaseType_t uxEventQueueLength)
{
	QueueSetHandle_t pxQueue;

	pxQueue = xQueueGenericCreate(uxEventQueueLength, (UBaseType_t)sizeof(Queue_t *), queueQUEUE_TYPE_SET);

	return pxQueue;
}

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if (configUSE_QUEUE_SETS == 1)

BaseType_t xQueueAddToSet(QueueSetMemberHandle_t xQueueOrSemaphore, QueueSetHandle_t xQueueSet)
{
	BaseType_t xReturn;

	taskENTER_CRITICAL();
	{
		if (((Queue_t *)xQueueOrSemaphore)->pxQueueSetContainer != NULL)
		{
			/* Cannot add a queue/semaphore to more than one queue set. */
			xReturn = pdFAIL;
		}
		else if (((Queue_t *)xQueueOrSemaphore)->uxMessagesWaiting != (UBaseType_t)0)
		{
			/* Cannot add a queue/semaphore to a queue set if there are already
			items in the queue/semaphore. */
			xReturn = pdFAIL;
		}
		else
		{
			((Queue_t *)xQueueOrSemaphore)->pxQueueSetContainer = xQueueSet;
			xReturn = pdPASS;
		}
	}
	taskEXIT_CRITICAL();

	return xReturn;
}

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if (configUSE_QUEUE_SETS == 1)

BaseType_t xQueueRemoveFromSet(QueueSetMemberHandle_t xQueueOrSemaphore, QueueSetHandle_t xQueueSet)
{
	BaseType_t xReturn;
	Queue_t *const pxQueueOrSemaphore = (Queue_t *)xQueueOrSemaphore;

	if (pxQueueOrSemaphore->pxQueueSetContainer != xQueueSet)
	{
		/* The queue was not a member of the set. */
		xReturn = pdFAIL;
	}
	else if (pxQueueOrSemaphore->uxMessagesWaiting != (UBaseType_t)0)
	{
		/* It is dangerous to remove a queue from a set when the queue is
		not empty because the queue set will still hold pending events for
		the queue. */
		xReturn = pdFAIL;
	}
	else
	{
		taskENTER_CRITICAL();
		{
			/* The queue is no longer contained in the set. */
			pxQueueOrSemaphore->pxQueueSetContainer = NULL;
		}
		taskEXIT_CRITICAL();
		xReturn = pdPASS;
	}

	return xReturn;
} /*lint !e818 xQueueSet could not be declared as pointing to const as it is a typedef. */

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if (configUSE_QUEUE_SETS == 1)

QueueSetMemberHandle_t xQueueSelectFromSet(QueueSetHandle_t xQueueSet, TickType_t const xTicksToWait)
{
	QueueSetMemberHandle_t xReturn = NULL;

	(void)xQueueReceive((QueueHandle_t)xQueueSet, &xReturn, xTicksToWait); /*lint !e961 Casting from one typedef to another is not redundant. */
	return xReturn;
}

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if (configUSE_QUEUE_SETS == 1)

QueueSetMemberHandle_t xQueueSelectFromSetFromISR(QueueSetHandle_t xQueueSet)
{
	QueueSetMemberHandle_t xReturn = NULL;

	(void)xQueueReceiveFromISR((QueueHandle_t)xQueueSet, &xReturn, NULL); /*lint !e961 Casting from one typedef to another is not redundant. */
	return xReturn;
}

#endif /* configUSE_QUEUE_SETS */
/*-----------------------------------------------------------*/

#if (configUSE_QUEUE_SETS == 1)

static BaseType_t prvNotifyQueueSetContainer(const Queue_t *const pxQueue, const BaseType_t xCopyPosition)
{
	Queue_t *pxQueueSetContainer = pxQueue->pxQueueSetContainer;
	BaseType_t xReturn = pdFALSE;

	/* This function must be called form a critical section. */

	configASSERT(pxQueueSetContainer);
	configASSERT(pxQueueSetContainer->uxMessagesWaiting < pxQueueSetContainer->uxLength);

	if (pxQueueSetContainer->uxMessagesWaiting < pxQueueSetContainer->uxLength)
	{
		const int8_t cTxLock = pxQueueSetContainer->cTxLock;

		traceQUEUE_SEND(pxQueueSetContainer);

		/* The data copied is the handle of the queue that contains data. */
		xReturn = prvCopyDataToQueue(pxQueueSetContainer, &pxQueue, xCopyPosition);

		if (cTxLock == queueUNLOCKED)
		{
			if (listLIST_IS_EMPTY(&(pxQueueSetContainer->xTasksWaitingToReceive)) == pdFALSE)
			{
				if (xTaskRemoveFromEventList(&(pxQueueSetContainer->xTasksWaitingToReceive)) != pdFALSE)
				{
					/* The task waiting has a higher priority. */
					xReturn = pdTRUE;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			pxQueueSetContainer->cTxLock = (int8_t)(cTxLock + 1);
		}
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	return xReturn;
}

#endif /* configUSE_QUEUE_SETS */
