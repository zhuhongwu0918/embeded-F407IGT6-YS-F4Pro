/*
 * bsp_key.h
 *
 *  Created on: Mar 31, 2022
 *      Author: AIRS
 */

#ifndef INC_BSP_KEY_H_
#define INC_BSP_KEY_H_
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* 类型定义 --------------------------------------------------------------*/
typedef enum
{
	  KEY_UP   = 0,
	  KEY_DOWN = 1,
}KEYState_TypeDef;
/* 宏定义 --------------------------------------------------------------------*/
#define KEY1_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY1_GPIO_PIN                 GPIO_PIN_0
#define KEY1_GPIO                     GPIOE
#define KEY1_DOWN_LEVEL               0
/* 根据原理图设计，KEY1按下时引脚为低电平，所以这里设置为0 */
#define KEY2_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY2_GPIO_PIN                 GPIO_PIN_1
#define KEY2_GPIO                     GPIOE
#define KEY2_DOWN_LEVEL               0
/* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */
#define KEY3_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY3_GPIO_PIN                 GPIO_PIN_2
#define KEY3_GPIO                     GPIOE
#define KEY3_DOWN_LEVEL               0
/* 根据原理图设计，KEY2按下时引脚为低电平，所以这里设置为0 */

#define KEY4_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY4_GPIO_PIN                 GPIO_PIN_3
#define KEY4_GPIO                     GPIOE
#define KEY4_DOWN_LEVEL               0
/* 根据原理图设计，KEY4按下时引脚为低电平，所以这里设置为0 */

#define KEY5_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEY5_GPIO_PIN                 GPIO_PIN_4
#define KEY5_GPIO                     GPIOE
#define KEY5_DOWN_LEVEL               0
/* 根据原理图设计，KEY5按下时引脚为低电平，所以这里设置为0 */
/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/
void KEY_GPIO_Init(void);
KEYState_TypeDef KEY1_StateRead(void);
KEYState_TypeDef KEY2_StateRead(void);
KEYState_TypeDef KEY3_StateRead(void);
KEYState_TypeDef KEY4_StateRead(void);
KEYState_TypeDef KEY5_StateRead(void);
uint8_t keyResponse(void);
#endif /* INC_BSP_KEY_H_ */
