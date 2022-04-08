/*
 * bsp_beep.c
 *
 *  Created on: Mar 31, 2022
 *      Author: AIRS
 */

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_beep.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 初始化
  * 输入参数：state:设置蜂鸣器的状态。
  *             可选值：BEEPState_OFF：蜂鸣器不响；
  *             可选值：BEEPState_ON： 蜂鸣器响。
  * 返 回 值: 无
  * 说    明：
  */
void BEEP_GPIO_Init(void)
{
	BEEP_OFF;
}


/**
  * 函数功能: 设置板载蜂鸣器的状态
  * 输入参数：state:设置蜂鸣器的状态。
  *             可选值：BEEPState_OFF：蜂鸣器不响；
  *             可选值：BEEPState_ON： 蜂鸣器响。
  * 返 回 值: 无
  * 说    明：该函数使用类似HALA库函数的编程方法，方便理解HAL库函数编程思想。
  */
void BEEP_StateSet(BEEPState_TypeDef state)
{
  /* 检查输入参数是否合法 */
  assert_param(BEEPState_TypeDef(state));

  /* 判断设置的蜂鸣器状态，如果设置为蜂鸣器响 */
  if(state==BEEPState_ON)
  {
	  HAL_GPIO_WritePin(BEEP_GPIO,BEEP_GPIO_PIN,GPIO_PIN_SET);
//    BEEP_ON;/* 蜂鸣器响 */
  }
  else /* state=BEEPState_OFF：设置蜂鸣器不响 */
  {
	HAL_GPIO_WritePin(BEEP_GPIO,BEEP_GPIO_PIN,GPIO_PIN_RESET);  // 输出低电平
//    BEEP_OFF;/* 蜂鸣器不响 */
  }
}
void di(void)
{
	  BEEP_StateSet(BEEPState_ON);
	  osDelay(100);
	  BEEP_StateSet(BEEPState_OFF);
}
void didi(void)
{
	  BEEP_StateSet(BEEPState_ON);
	  osDelay(50);
	  BEEP_StateSet(BEEPState_OFF);
	  osDelay(50);
	  BEEP_StateSet(BEEPState_ON);
	  osDelay(50);
	  BEEP_StateSet(BEEPState_OFF);
}
