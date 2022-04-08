#ifndef __BSP_SPIx_H__
#define __BSP_SPIx_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include"stm32f4xx_hal_spi.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/

#define SPIx                                  SPI1
#define SPIx_RCC_CLK_ENABLE()                 __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_RCC_CLK_DISABLE()                __HAL_RCC_SPI1_CLK_DISABLE()
 
#define SPIx_SCK_ClK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_SCK_PORT                         GPIOA
#define SPIx_SCK_PIN                          GPIO_PIN_5

#define SPIx_MSS_ClK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MSS_PORT                         GPIOB
#define SPIx_MISO_PIN                         GPIO_PIN_5
#define SPIx_MOSI_PIN                         GPIO_PIN_4

#define SPIx_CS_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()    
#define SPIx_CS_PORT                          GPIOC
#define SPIx_CS_PIN                           GPIO_PIN_10
#define SPIx_CS_ENABLE()                      HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_RESET)
#define SPIx_CS_DISABLE()                     HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_SET)

#define SPIx_CE_CLK_ENABLE()                  __HAL_RCC_GPIOC_CLK_ENABLE()    
#define SPIx_CE_PORT                          GPIOC
#define SPIx_CE_PIN                           GPIO_PIN_11
#define SPIx_CE_LOW()                         HAL_GPIO_WritePin(SPIx_CE_PORT, SPIx_CE_PIN, GPIO_PIN_RESET)
#define SPIx_CE_HIGH()                        HAL_GPIO_WritePin(SPIx_CE_PORT, SPIx_CE_PIN, GPIO_PIN_SET)

#define SPIx_IRQ_CLK_ENABLE()                 __HAL_RCC_GPIOC_CLK_ENABLE()    
#define SPIx_IRQ_PORT                         GPIOG
#define SPIx_IRQ_PIN                          GPIO_PIN_15
#define SPIx_IRQ_PIN_READ()                   HAL_GPIO_ReadPin(SPIx_IRQ_PORT,SPIx_IRQ_PIN)
                 
/* ��չ���� ------------------------------------------------------------------*/
void bsp_SystemClock_Config(void);
extern SPI_HandleTypeDef hspi_SPI;

/* �������� ------------------------------------------------------------------*/

void bsp_SPIx_Init(void);
void SPIx_RX_Mode(void);					//����Ϊ����ģʽ
void SPIx_TX_Mode(void);					//����Ϊ����ģʽ
uint8_t SPIx_Read_Reg(uint8_t reg);					//���Ĵ���
uint8_t SPIx_Write_Reg(uint8_t reg, uint8_t value);		//д�Ĵ���
uint8_t Recieve(void);
void Si4432config(void);
uint8_t Si4432statecheck(void);
void Si4432reset_IRQ_Reg(void);
void Si4432startRecieve(void);
void Si4432startTransmit(uint8_t data);
#endif  

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
