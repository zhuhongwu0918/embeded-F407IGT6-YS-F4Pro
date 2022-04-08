/**
  ******************************************************************************
  * 文件名程: bsp_spiflash.c
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: 板载串行Flash底层驱动实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */

/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_spi.h"
#include "bsp_led.h"
#include"stm32f4xx_hal_spi.h"
#include"main.h"
#include"stdio.h"
//#include"stm32f4xx_hal_spi.h"
//#include "usart/bsp_debug_usart.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/

/* 私有变量 ------------------------------------------------------------------*/
SPI_HandleTypeDef hspi_SPI;
__IO uint8_t res,result,Count,result1,result2,SI4432_RxLenth,SI4432_RxCount;
__IO uint8_t SI4432_RxBUFF[100];
//__IO uint8_t Count=0;
//__IO uint8_t res,result1,result2,result=0,SI4432_RxLenth,SI4432_RxCount;
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/

/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void bsp_SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用

 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * 函数功能: 串行FLASH初始化
  * 输入参数: huart：串口句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
*/
void bsp_SPIx_Init(void)
{
  hspi_SPI.Instance = SPIx;
  hspi_SPI.Init.Mode = SPI_MODE_MASTER;
  hspi_SPI.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_SPI.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_SPI.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_SPI.Init.NSS = SPI_NSS_SOFT;
  hspi_SPI.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi_SPI.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_SPI.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_SPI.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_SPI.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi_SPI);
  __HAL_SPI_ENABLE(&hspi_SPI);
}

/**
  * 函数功能: SPI外设系统级初始化
  * 输入参数: hspi：SPI句柄类型指针
  * 返 回 值: 无
  * 说    明: 该函数被HAL库内部调用
  */
void bsp_HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hspi->Instance==SPIx)
  {
    SPIx_RCC_CLK_ENABLE();
    SPIx_SCK_ClK_ENABLE();
    SPIx_MSS_ClK_ENABLE();
    SPIx_CS_CLK_ENABLE();
    SPIx_CE_CLK_ENABLE();
    SPIx_IRQ_CLK_ENABLE();    
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = SPIx_SCK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(SPIx_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPIx_MISO_PIN|SPIx_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(SPIx_MSS_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(SPIx_CS_PORT, SPIx_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = SPIx_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SPIx_CS_PORT, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(SPIx_CE_PORT, SPIx_CE_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = SPIx_CE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(SPIx_CE_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = SPIx_IRQ_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SPIx_IRQ_PORT, &GPIO_InitStruct);
    
    SPIx_CE_LOW();
  }
}

/**
  * 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
  * 输入参数: byte：待发送数据
  * 返 回 值: uint8_t：接收到的数据
  * 说    明：无
  */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef* hspi,uint8_t byte)
{
  uint8_t d_read,d_send=byte;
  if(HAL_SPI_TransmitReceive(hspi,&d_send,&d_read,1,0xFF)!=HAL_OK)
  {   
    d_read=0xFF;
  }
  //printf("d_read=%x\n",d_read);
  return d_read; 
}


/**
  * 函数功能: SPI写寄存器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:指定寄存器地址
  *           
  */ 
uint8_t SPIx_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
  SPIx_CS_ENABLE();                 //使能SPI传输
  status =SPIx_ReadWriteByte(&hspi_SPI,reg|0x80);//发送寄存器号
  SPIx_ReadWriteByte(&hspi_SPI,value);      //写入寄存器的值
  SPIx_CS_DISABLE();                 //禁止SPI传输
  return status;       			//返回状态值
}

/**
  * 函数功能: 读取SPI寄存器值
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：reg:要读的寄存器
  *           
  */ 
uint8_t SPIx_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;	    
 	SPIx_CS_ENABLE();          //使能SPI传输
  SPIx_ReadWriteByte(&hspi_SPI,reg);   //发送寄存器号
  reg_val=SPIx_ReadWriteByte(&hspi_SPI,0XFF);//读取寄存器内容
  SPIx_CS_DISABLE();          //禁止SPI传输
  return reg_val;           //返回状态值
}	

/**
  * 函数功能: 在指定位置读出指定长度的数据
  * 输入参数: 无
  * 返 回 值: 此次读到的状态寄存器值
  * 说    明：无
  *           
  */ 
uint8_t SPIx_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	   
  
  SPIx_CS_ENABLE();           //使能SPI传输
  status=SPIx_ReadWriteByte(&hspi_SPI,reg);//发送寄存器值(位置),并读取状态值
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
  {
    pBuf[uint8_t_ctr]=SPIx_ReadWriteByte(&hspi_SPI,0XFF);//读出数据
  }
  SPIx_CS_DISABLE();       //关闭SPI传输
  return status;        //返回读到的状态值
}

void Si4432reset_IRQ_Reg(void)
{
	SPIx_CS_ENABLE();
	HAL_Delay(60);
	res=SPIx_Write_Reg(0x07, 0x80); //向0X07地址  写入0X80  软件复位
	HAL_Delay(20);
	res=SPIx_Read_Reg(0x07); //读取0X07地址内容
	HAL_Delay(20);
	printf("0x07res =%x \n",(unsigned short)res );
	printf("SPIx_IRQ_PIN_READ before!!!\n");
	while ( SPIx_IRQ_PIN_READ()== 1) HAL_Delay(10);//0表示等待接收中断
	res = SPIx_IRQ_PIN_READ();
	printf("SPIx_IRQ_PIN_READ=%d\n",res);

}

void Si4432config(void)
{
		SPIx_CS_ENABLE();
		HAL_Delay(60);
	/////////////////////////开始设置
		// 频率设置 434
		SPIx_Write_Reg(0x75, 0x53);
		SPIx_Write_Reg(0x76, 0x64);
		SPIx_Write_Reg(0x77, 0x00);
		// 1.2K bps 发射速率
		SPIx_Write_Reg(0x2a, 0x14);
		SPIx_Write_Reg(0x6e, 0x09);
		SPIx_Write_Reg(0x6f, 0xd5);
		SPIx_Write_Reg(0x70, 0x2c);

		res=SPIx_Read_Reg(0x70);
		HAL_Delay(20);
		printf("0x70res =%x \n",(unsigned short)res );
		res=SPIx_Read_Reg(0x6f); //�??0X07地址  写入0X80  软件复位
		HAL_Delay(20);
		printf("0x06res =%x \n",(unsigned short)res );
//		SPIx_Write_Reg(0x72, 0x48);	  //(9.6kbps)
		SPIx_Write_Reg(0x72, 0x38);	//频率偏差(1.2kbps)
		// 下面的设置根据Silabs 的Excel	(9.6 kbps, deviation: 45 kHz, channel filter BW: 102.2 kHz
		SPIx_Write_Reg(0x1C, 0x1b);															//write 0x1E to the IF Filter Bandwidth register
		SPIx_Write_Reg(0x20, 0x83);															//write 0xD0 to the Clock Recovery Oversampling Ratio register
		SPIx_Write_Reg(0x21, 0xc0);															//write 0x00 to the Clock Recovery Offset 2 register
		SPIx_Write_Reg(0x22, 0x13);															//write 0x9D to the Clock Recovery Offset 1 register
		SPIx_Write_Reg(0x23, 0xa9);															//write 0x49 to the Clock Recovery Offset 0 register
		SPIx_Write_Reg(0x24, 0x00);															//write 0x00 to the Clock Recovery Timing Loop Gain 1 register
		SPIx_Write_Reg(0x25, 0x03);															//write 0x24 to the Clock Recovery Timing Loop Gain 0 register
		SPIx_Write_Reg(0x1D, 0x40);															//write 0x40 to the AFC Loop Gearshift Override register
		SPIx_Write_Reg(0x1E, 0x0A);															//write 0x0A to the AFC Timing Control register
		SPIx_Write_Reg(0x2A, 0x14);															//write 0x20 to the AFC Limiter register

		//前导码 同步字
		SPIx_Write_Reg(0x34, 0X0A);    // 发射5字节的Preamble
		SPIx_Write_Reg(0x35, 0x2A);    // 需要检测Preamble
		SPIx_Write_Reg(0x33, 0x02);    // 同步字3,2 是同步字
		SPIx_Write_Reg(0x36, 0x2d);    // 同步字为 0x2dd4
		SPIx_Write_Reg(0x37, 0xd4);
		SPIx_Write_Reg(0x30, 0x8D);    // 使能PH+ FIFO模式，高位在前面，使能CRC校验	CCITTT
		SPIx_Write_Reg(0x32, 0x00 ); //	禁止帧头
		SPIx_Write_Reg(0x71, 0x63);    // 发射不需要 CLK，FiFo ， FSK模式

		res=SPIx_Read_Reg(0x36); //�??0X07地址  写入0X80  软件复位
		HAL_Delay(20);
		printf("0x36res =%x \n",(unsigned short)res );
		res=SPIx_Read_Reg(0x37); //向0X07地址写入0X80  软件复位
		HAL_Delay(20);
		printf("0x37res =%x \n",(unsigned short)res );

		//GPIO
		SPIx_Write_Reg(0x0b, 0x12);
		SPIx_Write_Reg(0x0c, 0x15);

		//其他设置
		SPIx_Write_Reg(0x09, 0xD7);    //负载电容
		SPIx_Write_Reg(0x69, 0x60);    //AGC过载
		//发射功率
		SPIx_Write_Reg(0x6d, 0x1e);
		//手动打开接收
		SPIx_Write_Reg(0x07, 0x05);
		//打开 接收中断
		SPIx_Write_Reg(0x05, 0x03);
		SPIx_Write_Reg(0x06, 0x00);

		//清中断
		result1 = SPIx_Read_Reg(0x03);	  //read the Interrupt Status1 register
		result2 = SPIx_Read_Reg(0x04);	  //read the Interrupt Status2 register

		//接收设置
		SPIx_Write_Reg(0x08, 0x02);
		SPIx_Write_Reg(0x08, 0x00);
		SPIx_Write_Reg(0x07, 0x05);    	//手动打开接收
		printf("Configrued! ItStatus1=%x ItStatus2=%x\n",(unsigned short)result1,(unsigned short)result2 );

}

void Si4432startRecieve(void)
{
	for(;;)
	{
//		printf("Enter Recieve loop!\n");
		result=SPIx_IRQ_PIN_READ();//0表示等待接收中断，等待上一个中断结束
		printf("SPIx_IRQ_PIN_READ=%x\n",(unsigned short)result);
		HAL_Delay(50);
			if ( result == 0)
			{
				printf("IRQ ready! result == 0!\n");
				HAL_Delay(50);
				result1 = SPIx_Read_Reg(0x03);		//read the Interrupt result1 register
				result2 = SPIx_Read_Reg(0x04);		//read the Interrupt result2 register
				if((result1&0x02)==0x02)
				{
					SI4432_RxLenth=SPIx_Read_Reg(0x4B);
					for(SI4432_RxCount=0;SI4432_RxCount<SI4432_RxLenth;SI4432_RxCount++)
					{
						SI4432_RxBUFF[SI4432_RxCount] = SPIx_Read_Reg(0x7F);
//						motorInit();
						printf("SI4432_RxBUFF=%x\n",SI4432_RxBUFF[SI4432_RxCount]);
	//					ApplyDesiredVelocity(SI4432_RxBUFF[SI4432_RxCount]);
					}
					SI4432_RxCount=0;
					SPIx_Write_Reg(0x08, 0x02);
					SPIx_Write_Reg(0x08, 0x00);
					SPIx_Write_Reg(0x07, 0x05); //手动打开接收
				}
			}
	}
}

void Si4432startTransmit(uint8_t data)
{
//	for(;;)
//	{
	printf("Enter Transmit loop!\n");
	    LED1_ON;
		res = SPIx_Read_Reg(0x02);
		HAL_Delay(20);
		printf("0x02Res =%02x \n",(unsigned short)res );
		SPIx_Write_Reg(0x07, 0x01);	// rf 模块进入Ready 模式
		HAL_Delay(20);		// 延时 20ms, 让系统稳定

		SPIx_Write_Reg(0x3e, 0x01);  // 每次发射1个字节的数据
		res = SPIx_Read_Reg(0x3e);
		printf("set length =%02x !\n",(unsigned short)res );
		SPIx_Write_Reg(0x7F, data);
		printf("Writ to TX FIFO=%02x !\n",(unsigned short)data);
//		Count++;                      //Count计数自加
//		res = SPIx_Read_Reg(0x7F);
//		printf("Read TX FIFO =%02x !\n",(unsigned short)res);
		SPIx_Write_Reg(0x05, 0x04);	// 整包数据发射完后，产生中断
//		printf("Enable External Interrrupt!\n");

		SPIx_Write_Reg(0x06, 0x00);

		result1 = SPIx_Read_Reg(0x03);//清除中断
		result1 = SPIx_Read_Reg(0x04);

		SPIx_Write_Reg(0x07, 0x09);//打开发射
		printf("Sent and clear the FIFO!\n");

		HAL_Delay(20);
	    while ( SPIx_IRQ_PIN_READ()== 1) LED1_OFF;

		result1 = SPIx_Read_Reg(0x03);
//		printf("0x03result1 =%02x \n",(unsigned short)result1 );
		result2 = SPIx_Read_Reg(0x04);
//		printf("0x04result2 =%02x \n",(unsigned short)result2 );
		HAL_Delay(1000);
//	}
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/

