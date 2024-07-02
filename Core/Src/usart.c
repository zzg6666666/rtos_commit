/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  //使能USART1
  huart1.Instance = USART1;
  //波特率
  huart1.Init.BaudRate = 115200;
  //8字节的字长
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  //一位的停止位
  huart1.Init.StopBits = UART_STOPBITS_1;
  //无奇偶校验
  huart1.Init.Parity = UART_PARITY_NONE;
  //使能TX 、RX
  huart1.Init.Mode = UART_MODE_TX_RX;
  //不使用硬件控制流
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  //过采样，仅仅在F100xx生效
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

//初始化串口硬件
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    //使能USART1，RCC->APB2ENR bit14
    __HAL_RCC_USART1_CLK_ENABLE();

    //使能GPIOA，未映射
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
   //配置 PA9
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    //复用推挽输出
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //高速度
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    //UART 发送硬件初始化
    //设置DMA1通道4
    hdma_usart1_tx.Instance = DMA1_Channel4;
    //方向是从内存到外设
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    //不自增外设地址
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    //自增内存地址
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    //设置外设数据宽度为字节
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    //设置内存数据宽度为字节
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    //设置DMA模式为正常模式
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    //DMA的优先级为低
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    // if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    // {
    //   Error_Handler();
    // }

    //将UART和DMA链接在一起，需要串口、DMA初始化句柄、DMA的句柄
    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1_RX Init */
    //设置DMA1通道5
    hdma_usart1_rx.Instance = DMA1_Channel5;
    //DMA方向是外设到内存
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    //禁止外设地址自增
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    //开启内存地址自增
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    //外设的数据格式是字节
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    //内存的数据格式是字节
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    //DMA发送的模式正常
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    //DMA优先级低等级
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    // if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    // {
    //   Error_Handler();
    // }

    //将UART和DMA链接在一起，需要串口、DMA的句柄，
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    //设置USART串口中断优先级
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    //使能串口中断
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
