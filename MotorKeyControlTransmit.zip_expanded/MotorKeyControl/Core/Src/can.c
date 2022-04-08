/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "stdio.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_key.h"
#include "bsp_led.h"
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxMessage;
uint8_t aData[8]={0};
uint32_t DesiredVelocity=100;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PI9     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PI9     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOI, GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t CanFilterConfig(CAN_HandleTypeDef* hcan)
{
	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank = 0;
	HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
//	printf("CAN sFilterConfiged!\n");
	return 1;
}

uint8_t Can_Send_sdo_Msg(uint8_t* msg)
{
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.StdId = 0x0605;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 8;
	TxMessage.ExtId = 0x00;
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, msg, (uint32_t*)CAN_TX_MAILBOX0);

	while( HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0) != 1U);
	return 1;
}

uint8_t MotorDisable(void)
{
	printf("Enter MotroDisable!\n");
	  aData[0] = 0X2B;
	  aData[1] = 0X40;
	  aData[2] = 0X60;
	  aData[3] = 0X00;
	  aData[4] = 0X06;
	  aData[5] = 0X00;
	  aData[6] = 0X00;
	  aData[7] = 0X00;
	  Can_Send_sdo_Msg(aData);
	  osDelay(10);
	  return 0;
}

uint8_t MotorEnable(void)
{
	printf("Enter MotroEnable!\n");
	  aData[0] = 0X2B;
	  aData[1] = 0X40;
	  aData[2] = 0X60;
	  aData[3] = 0X00;
	  aData[4] = 0X0F;
	  aData[5] = 0X00;
	  aData[6] = 0X00;
	  aData[7] = 0X00;
	  Can_Send_sdo_Msg(aData);
	  osDelay(10);
	  return 0;
}


uint8_t MotorErrorClear(void)
{
	printf("Enter MotorErrorClear!\n");
	  aData[0] = 0X2B;
	  aData[1] = 0X40;
	  aData[2] = 0X60;
	  aData[3] = 0X00;
	  aData[4] = 0X86;
	  aData[5] = 0X00;
	  aData[6] = 0X00;
	  aData[7] = 0X00;
	  Can_Send_sdo_Msg(aData);
	  return 0;
}

uint8_t setAsVelocityMode(void)
{
	printf("Enter setModeVelocity!\n");
	  aData[0] = 0X2F;
	  aData[1] = 0X60;
	  aData[2] = 0X60;
	  aData[3] = 0X00;
	  aData[4] = 0X03;
	  aData[5] = 0X00;
	  aData[6] = 0X00;
	  aData[7] = 0X00;
	  Can_Send_sdo_Msg(aData);
	  osDelay(10);
	  return 0;
}

uint8_t ApplyZeroVelocity(void)
{
	printf("Enter ApplyZeroVelocity!\n");
	  aData[0] = 0X23;
	  aData[1] = 0XFF;
	  aData[2] = 0X60;
	  aData[3] = 0X00;
	  aData[4] = 0X00;
	  aData[5] = 0X00;
	  aData[6] = 0X00;
	  aData[7] = 0X00;
	  Can_Send_sdo_Msg(aData);
	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
	  MotorEnable();
	  return 1;
}

uint8_t ApplyDesiredVelocity(uint32_t DesiredVelocity)
{
	printf("Enter ApplyDesiredVelocity=%d",(int)DesiredVelocity);
	  aData[0] = 0X23;
	  aData[1] = 0XFF;
	  aData[2] = 0X60;
	  aData[3] = 0X00;
	  aData[4] = DesiredVelocity;
	  aData[5] = DesiredVelocity>>8;
	  aData[6] = DesiredVelocity>>16;
	  aData[7] = DesiredVelocity>>24;
	  Can_Send_sdo_Msg(aData);
//	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
	  MotorEnable();
	  return 1;
}
uint8_t motorInit(void)
{
	printf("Enter motorInit!\n");
	MotorDisable();
	MotorEnable();
	MotorErrorClear();
	setAsVelocityMode();
	ApplyZeroVelocity();
	osDelay(10);
	return 1;
}

uint8_t motorTask(void)
{
	printf("Enter motortask!\n");
	motorInit();
	DesiredVelocity=100;
	ApplyDesiredVelocity(DesiredVelocity);
	osDelay(5000);
	DesiredVelocity=DesiredVelocity*2;
	ApplyDesiredVelocity(DesiredVelocity);
	osDelay(5000);
	DesiredVelocity=DesiredVelocity*3;
	ApplyDesiredVelocity(DesiredVelocity);
	osDelay(5000);
	DesiredVelocity=DesiredVelocity*0;
	ApplyDesiredVelocity(DesiredVelocity);
	osDelay(5000);
	MotorDisable();
	return 1;
}
uint8_t keycontrolmotor(void)
{
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)==GPIO_PIN_RESET)
	{
		osDelay(20);
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)==GPIO_PIN_RESET)
		{
			uint8_t i = 0;
			while (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)==GPIO_PIN_RESET)
			{
				printf("Key1 down!\n");
				DesiredVelocity=100*(1+i/10);
				printf("Velocity = %d\n",(int)DesiredVelocity*100);
				ApplyDesiredVelocity(DesiredVelocity);
				HAL_GPIO_WritePin(GPIOH, GPIO_PIN_9, GPIO_PIN_RESET);
				osDelay(100);
				i = i+1;
			}
		}

	}
	if(KEY2_StateRead()==KEY_DOWN)
	{
		printf("Key2 down!\n");
		ApplyDesiredVelocity(0);
		printf("Fixed!\n");
	}

	if(KEY3_StateRead()==KEY_DOWN)
	{
		printf("Key3 down!\n");
		MotorDisable();
		printf("MotorDisable!\n");
	}

	if(KEY4_StateRead()==KEY_DOWN)
	{
		printf("Key2 down!\n");
		motorInit();
		printf("MotorInit!\n");
	}


	return 1;

}

/* USER CODE END 1 */
