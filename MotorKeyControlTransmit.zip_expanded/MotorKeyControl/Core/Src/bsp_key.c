/*
 * bsp_key.c
 *
 *  Created on: Mar 31, 2022
 *      Author: AIRS
 */

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_key.h"
#include "bsp_beep.h"
#include "bsp_led.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/**
  * 函数功能: 读取按键KEY1的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */

KEYState_TypeDef KEY1_StateRead(void)
{
	if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
	  {
	    /* 延时一小段时间，消除抖动 */
	    HAL_Delay(20);
	    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
	    if(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL)
	    {
	      /* 等待按键弹开才退出按键扫描函数 */
	      while(HAL_GPIO_ReadPin(KEY1_GPIO,KEY1_GPIO_PIN)==KEY1_DOWN_LEVEL);
	       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
	      return KEY_DOWN;
	    }
	  }
    return KEY_UP;
}
/**
  * 函数功能: 读取按键KEY2的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */

KEYState_TypeDef KEY2_StateRead(void)
{
	if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
	  {
	    /* 延时一小段时间，消除抖动 */
	    HAL_Delay(20);
	    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
	    if(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL)
	    {
	      /* 等待按键弹开才退出按键扫描函数 */
	      while(HAL_GPIO_ReadPin(KEY2_GPIO,KEY2_GPIO_PIN)==KEY2_DOWN_LEVEL);
	       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
	      return KEY_DOWN;
	    }
	  }
    return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY3的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY3_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(10);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
      while(HAL_GPIO_ReadPin(KEY3_GPIO,KEY3_GPIO_PIN)==KEY3_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY4的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY4_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(10);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
      while(HAL_GPIO_ReadPin(KEY4_GPIO,KEY4_GPIO_PIN)==KEY4_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}

/**
  * 函数功能: 读取按键KEY5的状态
  * 输入参数：无
  * 返 回 值: KEY_DOWN：按键被按下；
  *           KEY_UP  ：按键没被按下
  * 说    明：无。
  */
KEYState_TypeDef KEY5_StateRead(void)
{
  /* 读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内 */
  if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL)
  {
    /* 延时一小段时间，消除抖动 */
    HAL_Delay(10);
    /* 延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下 */
    if(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL)
    {
      /* 等待按键弹开才退出按键扫描函数 */
      while(HAL_GPIO_ReadPin(KEY5_GPIO,KEY5_GPIO_PIN)==KEY5_DOWN_LEVEL);
       /* 按键扫描完毕，确定按键被按下，返回按键被按下状态 */
      return KEY_DOWN;
    }
  }
  /* 按键没被按下，返回没被按下状态 */
  return KEY_UP;
}
uint8_t keyResponse(void)
{
	  if(KEY1_StateRead()==KEY_DOWN)
	      {
	        didi();
	        LED1_ON;
	        osDelay(500);
	        LED1_OFF;

	      }
	      if(KEY2_StateRead()==KEY_DOWN)
	      {
	        didi();
	        LED2_ON;
	        osDelay(500);
	        LED2_OFF;
	      }
	      if(KEY3_StateRead()==KEY_DOWN)
	      {
	        didi();
	        LED3_ON;
	        osDelay(500);
	        LED3_OFF;
	      }
	      if(KEY4_StateRead()==KEY_DOWN)
	      {
	    	  didi();
				LED1_ON;LED2_ON;LED3_ON;
				osDelay(500);
				LED1_OFF;LED2_OFF;LED3_OFF;
	      }
	      if(KEY5_StateRead()==KEY_DOWN)
	      {
	    	  di();
			LED1_ON;LED2_ON;LED3_ON;
			osDelay(500);
			LED1_OFF;LED2_OFF;LED3_OFF;
	      }
	      return 1;
}
