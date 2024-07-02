/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "uart_printf.h"
#include "semphr.h"
#include "stdlib.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
SemaphoreHandle_t mutexTest;                   // 互斥量
SemaphoreHandle_t recursiveMutexTest;          // 递归互斥量
SemaphoreHandle_t binarySemaphoreTest;         // 二进制信号量
SemaphoreHandle_t binarySemaphoreCountingTest; // 计数型信号量
QueueHandle_t queueTest;                       // 队列
TaskHandle_t taskHandle0, taskHandle1, taskHandle2, taskHandle3, taskHandle4, taskHandle5, taskHandle14, taskHandle15;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void task0(void *argument); // 互斥量0
void task1(void *argument); // 互斥量1
void task2(void *argument); // 互斥量2

void task3(void *argument); // 二进制信号量1
void task4(void *argument); // 二进制信号量2
void task5(void *argument); // 二进制信号量3
void task6(void *argument); // 计数信号量1
void task7(void *argument); // 计数信号量2
void task8(void *argument); // 计数信号量3

void task9(void *argument);  // 队列1
void task10(void *argument); // 队列2

void task11(void *argument); // 递归互斥量1
void task12(void *argument); // 递归互斥量2
void task13(void *argument); // 递归互斥量3

void task15(void *argument); // 任务通知1
void task14(void *argument); // 任务通知2
// void taskGetRAMUse(void *argument); // 获取内存占用

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  // 创建互斥量 设置uxMessagesWaiting = 1
  mutexTest = xSemaphoreCreateMutex();
  // 创建二进制信号量
  binarySemaphoreTest = xSemaphoreCreateBinary();

  // 创建二进制信号量后，需要手动释放信号量,还有就是 少了 优先级继承
  if (binarySemaphoreTest != NULL)
  {
    // 设置uxMessagesWaiting = 1
    xSemaphoreGive(binarySemaphoreTest);
  }
  /* USER CODE END Init */
  binarySemaphoreCountingTest = xSemaphoreCreateCounting(3, 2);

  // 创建队列
  queueTest = xQueueCreate(10, 1);
  if (queueTest == NULL)
  {
    for (;;)
      ;
  }
  // 创建递归互斥量
  recursiveMutexTest = xSemaphoreCreateRecursiveMutex();
  if (recursiveMutexTest == NULL)
  {
    for (;;)
      ;
  }
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  // taskHandle0 = taskHandle1 = taskHandle2 = taskHandle3 = taskHandle4 = taskHandle5 = NULL;
  // xTaskCreate(task0, "task0", 100, NULL, osPriorityNormal, &taskHandle0);
  // xTaskCreate(task1, "task1", 60, NULL, osPriorityNormal, &taskHandle1);
  // xTaskCreate(task2, "task2", 60, NULL, osPriorityNormal, &taskHandle2);
  // xTaskCreate(task3, "task3", 60, NULL, osPriorityNormal, &taskHandle3);
  // xTaskCreate(task4, "task4", 60, NULL, osPriorityNormal, &taskHandle4);
  // xTaskCreate(task5, "task5", 60, NULL, osPriorityNormal, &taskHandle5);
  // xTaskCreate(task6, "task6", 60, NULL, osPriorityNormal, NULL);
  // xTaskCreate(task7, "task7", 60, NULL, osPriorityNormal, NULL);
  // xTaskCreate(task8, "task8", 60, NULL, osPriorityNormal, NULL);
  // xTaskCreate(task9, "task7", 60, NULL, osPriorityNormal, NULL);
  // xTaskCreate(task10, "task8", 60, NULL, osPriorityNormal, NULL);
  // xTaskCreate(task11, "task7", 60, NULL, osPriorityNormal, NULL);
  // xTaskCreate(task12, "task8", 60, NULL, osPriorityNormal, NULL);
  // xTaskCreate(task13, "task7", 60, NULL, osPriorityNormal, NULL);

  taskHandle14 = taskHandle15 = NULL;
  xTaskCreate(task14, "task14", 70, NULL, osPriorityNormal, &taskHandle14);
  xTaskCreate(task15, "task15", 70, NULL, osPriorityNormal, &taskHandle15);

  // xTaskCreate(taskGetRAMUse, "taskGetRAMUse", 30, NULL, osPriorityAboveNormal1, taskHandle5);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1000);
    // uart_printf("StartDefaultTask\r\n", NULL);
  }
  /* USER CODE END StartDefaultTask */
}
void task0(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    //  互斥量获取(上锁)
    if (xSemaphoreTake(mutexTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task0 ram mark:", &RAMUse, 1);
      //  uart_printf("task 1 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      // uart_printf("task 1 get mutex after :", &tickCount, 1);

      // 互斥量释放(落锁)
      xSemaphoreGive(mutexTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task1 get mutexTest fail after :", &tickCount, 1);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task1(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    //  互斥量获取(上锁)
    if (xSemaphoreTake(mutexTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task1 ram mark:", &RAMUse, 1);
      //  uart_printf("task 0 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      //    uart_printf("task 0 get mutex after :", &tickCount, 1);

      // 互斥量释放(落锁)
      xSemaphoreGive(mutexTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task1 get mutexTest fail after :", &tickCount, 1);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task2(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    //  互斥量获取(上锁)
    if (xSemaphoreTake(mutexTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task2 ram mark:", &RAMUse, 1);
      // uart_printf("task 2 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      // uart_printf("task 2 get mutex after :", &tickCount, 1);

      // 互斥量释放(落锁)
      xSemaphoreGive(mutexTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task2 get mutexTest fail after :", &tickCount, 1);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task3(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    // 二进制信号量获取
    if (xSemaphoreTake(binarySemaphoreTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task3 ram mark:", &RAMUse, 1);
      //  uart_printf("task 3 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      // uart_printf("task 3 get mutex after :", &tickCount, 1);

      // 释放
      xSemaphoreGive(binarySemaphoreTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task2 get mutexTest fail after :", &tickCount, 1);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task4(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    //  二进制信号量获取(上锁)
    if (xSemaphoreTake(binarySemaphoreTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task4 ram mark:", &RAMUse, 1);
      //  uart_printf("task 4 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      // uart_printf("task 4 get mutex after :", &tickCount, 1);

      // 释放
      xSemaphoreGive(binarySemaphoreTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task2 get mutexTest fail after :", &tickCount, 1);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task5(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    //  二进制信号量获取(上锁)
    if (xSemaphoreTake(binarySemaphoreTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task5 ram mark:", &RAMUse, 1);
      // uart_printf("task 5 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      // uart_printf("task 5 get mutex after :", &tickCount, 1);

      // 释放
      xSemaphoreGive(binarySemaphoreTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task5 get mutexTest fail\r\n",NULL,0);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}

void task6(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    //  二进制信号量获取(上锁)
    if (xSemaphoreTake(binarySemaphoreCountingTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task6 ram mark:", &RAMUse, 1);
      // uart_printf("task 5 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      // uart_printf("task 5 get mutex after :", &tickCount, 1);

      // 释放
      xSemaphoreGive(binarySemaphoreCountingTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task5 get mutexTest fail\r\n",NULL,0);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task7(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    //  二进制信号量获取(上锁)
    if (xSemaphoreTake(binarySemaphoreCountingTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task7 ram mark:", &RAMUse, 1);
      // uart_printf("task 5 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      // uart_printf("task 5 get mutex after :", &tickCount, 1);

      // 释放
      xSemaphoreGive(binarySemaphoreCountingTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task5 get mutexTest fail\r\n",NULL,0);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task8(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t RAMUse = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  uint8_t sleeptime = 0;
  /* Infinite loop */
  for (;;)
  {
    tickCount = (uint8_t)xTaskGetTickCount();
    //  二进制信号量获取(上锁)
    if (xSemaphoreTake(binarySemaphoreCountingTest, 30) == pdTRUE)
    {
      tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      sleeptime = (uint8_t)(rand() % 255);
      RAMUse = uxTaskGetStackHighWaterMark(NULL);
      uart_printf("task8 ram mark:", &RAMUse, 1);
      // uart_printf("task 5 sleep time :", &sleeptime, 1);
      osDelay(sleeptime);
      // 传入数字 会 触发硬件falult
      // uart_printf("task 5 get mutex after :", &tickCount, 1);

      // 释放
      xSemaphoreGive(binarySemaphoreCountingTest);
    }
    else
    {
      // tickCount = xTaskGetTickCount() - (uint8_t)tickCount;
      // uart_printf("task5 get mutexTest fail\r\n",NULL,0);
    }
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task9(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t randomNum = 0;
  uint8_t tickCount = 0;
  srand(tickCount);
  /* Infinite loop */
  for (;;)
  {
    randomNum = rand() % 10;
    // 随机往队列写数据
    if (randomNum > 4)
    {
      // 将数据发送到队列头部
      xQueueSendToFront(queueTest, &randomNum, 100);

      // 将数据发送到队末尾 和xQueueSend()一样
      xQueueSendToBack(queueTest, &randomNum, 100);
      xQueueSend(queueTest, &randomNum, 100);
    }
    uart_printf("queue random write :", &randomNum, 1);
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}
void task10(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  uint8_t randomNum = 0;
  uint8_t tickCount = 0;
  uint8_t getNum = 0;
  srand(tickCount);
  /* Infinite loop */
  for (;;)
  {
    getNum = 0;

    randomNum = rand() % 10;
    // 读数据
    if (randomNum > 1)
    {
      xQueueReceive(queueTest, &randomNum, 100);
    }
    uart_printf("queue read :", &getNum, 1);
    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

  /* USER CODE END StartDefaultTask */
}

void task11(void *argument)
{
  srand(10);
  for (;;)
  {
    // randomNum = rand() % 20;

    // 获取信号量
    if (xSemaphoreTakeRecursive(recursiveMutexTest, 100) == pdPASS)
    {
      if (xSemaphoreTakeRecursive(recursiveMutexTest, 100) == pdPASS)
      {
        // uart_printf("recursiveMutexTest take 01 by task11", NULL, 0);
        // 释放信号量
        xSemaphoreGiveRecursive(recursiveMutexTest);
        uart_printf("recursiveMutexTest give() times 02 by task11\r\n", NULL, 0);
      }
    }
    else
    {
      uart_printf("recursiveMutexTest give() times 02 fail task11\r\n", NULL, 0);
    }
    xSemaphoreGiveRecursive(recursiveMutexTest);
    uart_printf("recursiveMutexTest give() times 01 by task11\r\n", NULL, 0);

    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}

void task12(void *argument)
{
  srand(10);
  for (;;)
  {
    // randomNum = rand() % 20;

    // 获取信号量
    if (xSemaphoreTakeRecursive(recursiveMutexTest, 100) == pdPASS)
    {
      if (xSemaphoreTakeRecursive(recursiveMutexTest, 100) == pdPASS)
      {
        // uart_printf("recursiveMutexTest take 01 by task11", NULL, 0);
        // 释放信号量
        xSemaphoreGiveRecursive(recursiveMutexTest);
        uart_printf("recursiveMutexTest give() times 02  by task12\r\n", NULL, 0);
      }
    }
    else
    {
      uart_printf("recursiveMutexTest give() times 02  fail task12\r\n", NULL, 0);
    }
    xSemaphoreGiveRecursive(recursiveMutexTest);
    uart_printf("recursiveMutexTest give() times 01 by task13\r\n", NULL, 0);

    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}
void task13(void *argument)
{
  srand(10);
  for (;;)
  {
    // randomNum = rand() % 20;

    // 获取信号量
    if (xSemaphoreTakeRecursive(recursiveMutexTest, 100) == pdPASS)
    {
      if (xSemaphoreTakeRecursive(recursiveMutexTest, 100) == pdPASS)
      {
        // uart_printf("recursiveMutexTest take 01 by task11", NULL, 0);
        // 释放信号量
        xSemaphoreGiveRecursive(recursiveMutexTest);
        uart_printf("recursiveMutexTest give() times 02  by task13\r\n", NULL, 0);
      }
    }
    else
    {
      uart_printf("recursiveMutexTest give() times 02  fail task13\r\n", NULL, 0);
    }
    xSemaphoreGiveRecursive(recursiveMutexTest);
    uart_printf("recursiveMutexTest give() times 01 by task13\r\n", NULL, 0);

    osDelay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}

void task14(void *argument)
{
  uint32_t ulNotificationValue;

  for (;;)
  {
    // 接受通知 且在收到通知后，清除通知 简易版
    // ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 复杂版本
    xTaskNotifyWait(0x01, 0x00, &ulNotificationValue, portMAX_DELAY);
    if (ulNotificationValue > 0)
    {
      vTaskDelay(100);
      uart_printf("task14 recive the notify\r\n", NULL, 0);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
  }
}

void task15(void *argument)
{
  for (;;)
  {
    // 通知任务14 简易版
    // xTaskNotifyGive(taskHandle14);
    vTaskDelay(1000);
    // 复杂版 设置通知值 和 通知方式
    xTaskNotify(taskHandle14, 12, eSetBits);
    vTaskDelay(1000);
    xTaskNotifyAndQuery(taskHandle14, 12, eSetBits, NULL);
    vTaskDelay(1000);
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
}
