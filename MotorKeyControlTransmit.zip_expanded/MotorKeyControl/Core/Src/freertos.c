/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "gpio.h"
#include "stdio.h"
#include "usart.h"
#include "bsp_beep.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include"bsp_spi.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 #define RXBUFFERSIZE  256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char RxBuffer[RXBUFFERSIZE];
uint8_t tx_buffer=0x03;
uint8_t rx_buffer=7;
__IO uint8_t SI4432_RxBUFF[100];
__IO uint8_t data;
__IO uint8_t res,result1,result2,result=0,SI4432_RxLenth,SI4432_RxCount;
//uint32_t Velocity=100;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for statelight */
osThreadId_t statelightHandle;
const osThreadAttr_t statelight_attributes = {
  .name = "statelight",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for beepTask */
osThreadId_t beepTaskHandle;
const osThreadAttr_t beepTask_attributes = {
  .name = "beepTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void flashlight(void *argument);
void candrive_task(void *argument);
void beep(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of statelight */
  statelightHandle = osThreadNew(flashlight, NULL, &statelight_attributes);

  /* creation of CANTask */
  CANTaskHandle = osThreadNew(candrive_task, NULL, &CANTask_attributes);

  /* creation of beepTask */
  beepTaskHandle = osThreadNew(beep, NULL, &beepTask_attributes);

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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_flashlight */
/**
* @brief Function implementing the statelight thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_flashlight */
void flashlight(void *argument)
{
  /* USER CODE BEGIN flashlight */
	Si4432reset_IRQ_Reg();
	Si4432config();
	data=0;
//	CanFilterConfig(&hcan1);
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL);
//	HAL_CAN_Start(&hcan1);
//	motorInit();
	/* Infinite loop */
  for(;;)
  {
	Si4432startTransmit(data);
	HAL_Delay(1000);
	data++;
   }
}

/* USER CODE BEGIN Header_candrive_task */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_candrive_task */
void candrive_task(void *argument)
{
  /* USER CODE BEGIN candrive_task */

//	  CanFilterConfig(&hcan1);
//	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_FULL);
//	  HAL_CAN_Start(&hcan1);
//	  printf("CAN Started!\n");
  /* Infinite loop */
  for(;;)
  {
//	  keycontrolmotor();
	  osDelay(50);
  }
  /* USER CODE END candrive_task */
}

/* USER CODE BEGIN Header_beep */
/**
* @brief Function implementing the beepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_beep */
void beep(void *argument)
{
  /* USER CODE BEGIN beep */
//	BEEP_GPIO_Init();
//	osDelay(10);
	BEEP_GPIO_Init();
	didi();
  /* Infinite loop */
  for(;;)
  {
//	  keyResponse();

	  osDelay(10);
	  	  if(KEY1_StateRead()==KEY_DOWN)
	  	      {
//	  	        di();
	  	        LED1_ON;
	  	        osDelay(500);
	  	        LED1_OFF;
	  	      }
	  	      if(KEY2_StateRead()==KEY_DOWN)
	  	      {
//	  	        di();
	  	        LED2_ON;
	  	        osDelay(500);
	  	        LED2_OFF;
	  	      }
	  	      if(KEY3_StateRead()==KEY_DOWN)
	  	      {
//	  	        di();
	  	        LED3_ON;
	  	        osDelay(500);
	  	        LED3_OFF;
	  	      }
	  	      if(KEY4_StateRead()==KEY_DOWN)
	  	      {
//	  				di();
	  				LED1_ON;LED2_ON;LED3_ON;
	  				osDelay(500);
	  				LED1_OFF;LED2_OFF;LED3_OFF;
	  	      }
	  	      if(KEY5_StateRead()==KEY_DOWN)
	  	      {
//	  	    	 di();
	  			LED1_ON;LED2_ON;LED3_ON;
	  			osDelay(500);
	  			LED1_OFF;LED2_OFF;LED3_OFF;
	  	      }
	  	    //	  vTaskSuspend(beepTaskHandle);
  }
  /* USER CODE END beep */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

