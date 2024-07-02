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
#include "FreeRTOS.h"
#include "list.h"

/*-----------------------------------------------------------
 * PUBLIC LIST API documented in list.h
 *----------------------------------------------------------*/
//初始化链表
void vListInitialise( List_t * const pxList )
{
	/* The list structure contains a list item which is used to mark the
	end of the list.  To initialise the list the list end is inserted
	as the only list entry. */
	/*初始化时，pxList中只有一个xListEnd，因此将其指向xListEnd*/
	pxList->pxIndex = ( ListItem_t * ) &( pxList->xListEnd );			/*lint !e826 !e740 The mini list structure is used as the list end to save RAM.  This is checked and valid. */

	/* The list end value is the highest possible value in the list to
	ensure it remains at the end of the list. */
	/*在排序时确保xListEnd在最后*/
	pxList->xListEnd.xItemValue = portMAX_DELAY;

	/* The list end next and previous pointers point to itself so we know
	when the list is empty. */
	//初始化时,列表中只有一个xListEnd，因此，xListEnd的之前一个和后面一个都是自己
	pxList->xListEnd.pxNext = ( ListItem_t * ) &( pxList->xListEnd );	/*lint !e826 !e740 The mini list structure is used as the list end to save RAM.  This is checked and valid. */
	pxList->xListEnd.pxPrevious = ( ListItem_t * ) &( pxList->xListEnd );/*lint !e826 !e740 The mini list structure is used as the list end to save RAM.  This is checked and valid. */

	//目前没有一个列表项
	pxList->uxNumberOfItems = ( UBaseType_t ) 0U;

	/* Write known values into the list if
	configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES is set to 1. */
	listSET_LIST_INTEGRITY_CHECK_1_VALUE( pxList );
	listSET_LIST_INTEGRITY_CHECK_2_VALUE( pxList );
}
/*-----------------------------------------------------------*/
//列表节点初始化
void vListInitialiseItem( ListItem_t * const pxItem )
{
	/* Make sure the list item is not recorded as being on a list. */
	//初始化时 设置listItem所在的list为空
	pxItem->pvContainer = NULL;

	/* Write known values into the list item if
	configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES is set to 1. */
	//用于检测完整性
	listSET_FIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem );
	listSET_SECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE( pxItem );
}
/*-----------------------------------------------------------*/

//将列表节点插入到列表的尾部(其实是插入到pxIndex前面)
void vListInsertEnd( List_t * const pxList, ListItem_t * const pxNewListItem )
{

ListItem_t * const pxIndex = pxList->pxIndex;

	/* Only effective when configASSERT() is also defined, these tests may catch
	the list data structures being overwritten in memory.  They will not catch
	data errors caused by incorrect configuration or use of FreeRTOS. */
	//列表完整性检验
	listTEST_LIST_INTEGRITY( pxList );
	//列表项完整性检查
	listTEST_LIST_ITEM_INTEGRITY( pxNewListItem );

	/* Insert a new list item into pxList, but rather than sort the list,
	makes the new list item the last item to be removed by a call to
	listGET_OWNER_OF_NEXT_ENTRY(). */
	//当pxNewListItem是插入的第一个的时候，pxNext指向列表头的xListEnd(xListEnd)
	pxNewListItem->pxNext = pxIndex;
	//当pxNewListItem是插入的第一个的时候，pxPrevious指向列表头的xListEnd(xListEnd)
	pxNewListItem->pxPrevious = pxIndex->pxPrevious;

	/* Only used during decision coverage testing. */
	mtCOVERAGE_TEST_DELAY();
	
	//当pxNewListItem是插入的第一个的时候，pxNext指向列表头的pxNewListItem
	pxIndex->pxPrevious->pxNext = pxNewListItem;
	//当pxNewListItem是插入的第一个的时候，pxPrevious指向列表头的pxNewListItem
	pxIndex->pxPrevious = pxNewListItem;

	/* Remember which list the item is in. */
	//将列表项的pvContainer指向插入的列表头
	pxNewListItem->pvContainer = ( void * ) pxList;

	//列表中对的链表项数量
	( pxList->uxNumberOfItems )++;
}
/*-----------------------------------------------------------*/
//列表节点按照升序排列插入到列表中
void vListInsert( List_t * const pxList, ListItem_t * const pxNewListItem )
{

ListItem_t *pxIterator;
const TickType_t xValueOfInsertion = pxNewListItem->xItemValue;

	/* Only effective when configASSERT() is also defined, these tests may catch
	the list data structures being overwritten in memory.  They will not catch
	data errors caused by incorrect configuration or use of FreeRTOS. */
	listTEST_LIST_INTEGRITY( pxList );
	listTEST_LIST_ITEM_INTEGRITY( pxNewListItem );

	/* Insert the new list item into the list, sorted in xItemValue order.

	如果有两个相同的xItemValue，将该节点插入到原有节点的后面，这是确保如果TCB有相同的
	xItemValue，TCB可以共享CPU
	If the list already contains a list item with the same item value then the
	new list item should be placed after it.  This ensures that TCB's which are
	stored in ready lists (all of which have the same xItemValue value) get a
	share of the CPU.  However, if the xItemValue is the same as the back marker
	the iteration loop below will not end.  Therefore the value is checked
	first, and the algorithm slightly modified if necessary. */
	if( xValueOfInsertion == portMAX_DELAY )
	{	
		//将新列表表插入到xListEnd的前面一个列表节点中去
		pxIterator = pxList->xListEnd.pxPrevious;
	}
	else
	{
		/* *** NOTE ***********************************************************
		If you find your application is crashing here then likely causes are
		listed below.  In addition see http://www.freertos.org/FAQHelp.html for
		more tips, and ensure configASSERT() is defined!
		http://www.freertos.org/a00110.html#configASSERT

			1) Stack overflow -
			   see http://www.freertos.org/Stacks-and-stack-overflow-checking.html
			2) Incorrect interrupt priority assignment, especially on Cortex-M
			   parts where numerically high priority values denote low actual
			   interrupt priorities, which can seem counter intuitive.  See
			   http://www.freertos.org/RTOS-Cortex-M3-M4.html and the definition
			   of configMAX_SYSCALL_INTERRUPT_PRIORITY on
			   http://www.freertos.org/a00110.html
			3) Calling an API function from within a critical section or when
			   the scheduler is suspended, or calling an API function that does
			   not end in "FromISR" from an interrupt.
			4) Using a queue or semaphore before it has been initialised or
			   before the scheduler has been started (are interrupts firing
			   before vTaskStartScheduler() has been called?).
		**********************************************************************/

		
		//从前往后迭代，找到第一个xItemValue比自己大的列表节点
		for( pxIterator = ( ListItem_t * ) &( pxList->xListEnd ); pxIterator->pxNext->xItemValue <= xValueOfInsertion; pxIterator = pxIterator->pxNext ) /*lint !e826 !e740 The mini list structure is used as the list end to save RAM.  This is checked and valid. */
		{
			/* There is nothing to do here, just iterating to the wanted
			insertion position. */
		}
	}

	//将pxNewListItem->pxNext指向第一个比pxNewListItem大的列表节点
	pxNewListItem->pxNext = pxIterator->pxNext;
	//将第一个比pxNewListItem大的列表节点->pxPrevious指向pxNewListItem
	pxNewListItem->pxNext->pxPrevious = pxNewListItem;
	//将pxNewListItem->pxPrevious指向最后一个比pxNewListItem小的节点
	pxNewListItem->pxPrevious = pxIterator;
	//将最后一个比pxNewListItem小的节点->pxNext指向pxNewListItem
	pxIterator->pxNext = pxNewListItem;

	/* Remember which list the item is in.  This allows fast removal of the
	item later. */
	//将pxNewListItem的pvContainer指向列表头
	pxNewListItem->pvContainer = ( void * ) pxList;

	( pxList->uxNumberOfItems )++;
}
/*-----------------------------------------------------------*/

//将节点从链表删除
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove )
{
/* The list item knows which list it is in.  Obtain the list from the list
item. */
//找到pxItemToRemove所在的列表头
List_t * const pxList = ( List_t * ) pxItemToRemove->pvContainer;

	//将pxItemToRemove从列表中删除
	pxItemToRemove->pxNext->pxPrevious = pxItemToRemove->pxPrevious;
	pxItemToRemove->pxPrevious->pxNext = pxItemToRemove->pxNext;

	/* Only used during decision coverage testing. */
	mtCOVERAGE_TEST_DELAY();

	/* Make sure the index is left pointing to a valid item. */
	//改变pxIndex指向,如果pxIndex指向的listItem被删除
	if( pxList->pxIndex == pxItemToRemove )
	{
		pxList->pxIndex = pxItemToRemove->pxPrevious;
	}
	else
	{	
		mtCOVERAGE_TEST_MARKER();
	}

	pxItemToRemove->pvContainer = NULL;
	//减少列表节点数
	( pxList->uxNumberOfItems )--;

	return pxList->uxNumberOfItems;
}
/*-----------------------------------------------------------*/

