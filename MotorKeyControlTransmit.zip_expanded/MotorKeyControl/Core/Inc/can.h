/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "stdio.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void motortask(void);
uint8_t Can_Send_sdo_Msg(uint8_t* msg);
uint8_t CanFilterConfig(CAN_HandleTypeDef* hcan);
uint8_t MotorDisable(void);
uint8_t MotorEnable(void);
uint8_t MotorErrorClear(void);
uint8_t setAsVelocityMode(void);
uint8_t ApplyZeroVelocity(void);
uint8_t ApplyDesiredVelocity(uint32_t data);
uint8_t motorInit(void);
uint8_t motorTask(void);
uint8_t keycontrolmotor(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

