/**
  ******************************************************************************
  * @file    bsp_spi.h
  * @author  phoenix
  * @version V1.0.0
  * @date    20-October-2017
  * @brief   This file provides set of firmware functions to manage Leds ,
  *          push-button and spi available on STM32F4-Discovery Kit from STMicroelectronics.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

#include "./spi/bsp_spi.h"

SPI_HandleTypeDef SpiHandle;

/**
  * @brief  GPIO_Config function
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
    //开启GPIO外设时钟
    LED_GPIO_CLK_ENABLE();
    CC1101_IRQ_GPIO_CLK_ENABLE();
    CC1101_GDO2_GPIO_CLK_ENABLE();
	
    //配置LED的GPIO引脚
    GPIO_InitStructure.Pin = LED_GREEN_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
    
    //配置IRQ的GPIO引脚
    GPIO_InitStructure.Pin = CC1101_IRQ_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(CC1101_IRQ_GPIO_PORT, &GPIO_InitStructure);
    
		/* Configure GDO2 pin as input floating */
//		GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
//		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Pin = CC1101_GDO2_PIN;
//		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(CC1101_GDO2_GPIO_PORT, &GPIO_InitStructure);

		/* Enable and set EXTI4_15 Interrupt to the lowest priority */
//		HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
//		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief  SPI_Config function
  * @param  None
  * @retval None
  */
void SPI_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* Enable SCK, MOSI and MISO GPIO clocks */
    CC1101_SPI_SCK_GPIO_CLK_ENABLE();
		CC1101_SPI_MISO_GPIO_CLK_ENABLE();
		CC1101_SPI_MOSI_GPIO_CLK_ENABLE();
		CC1101_SPI_CSN_GPIO_CLK_ENABLE();

    /* Set SPI_SCK Pin */
    GPIO_InitStructure.Pin = CC1101_SPI_SCK_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		GPIO_InitStructure.Alternate = CC1101_SPI_SCK_AF;
    HAL_GPIO_Init(CC1101_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
    
    /* Set SPI_MISO Pin */
    GPIO_InitStructure.Pin = CC1101_SPI_MISO_PIN;
		GPIO_InitStructure.Alternate = CC1101_SPI_MISO_AF;
    HAL_GPIO_Init(CC1101_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
    
    /* Set SPI_MOSI Pin */
    GPIO_InitStructure.Pin = CC1101_SPI_MOSI_PIN;
		GPIO_InitStructure.Alternate = CC1101_SPI_MOSI_AF;
    HAL_GPIO_Init(CC1101_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
    
    /* Set SPI_CSN Pin */
    GPIO_InitStructure.Pin = CC1101_SPI_CSN_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(CC1101_SPI_CSN_GPIO_PORT, &GPIO_InitStructure);
    
		/* Enable the SPI periph */
    CC1101_SPI_CLK_ENABLE();
		
    /* Set SPI_CSN Pin High */
    CC1101_CSN_HIGH();
    
    /* SPI configuration -------------------------------------------------------*/
    SpiHandle.Instance								= CC1101_SPI;
    SpiHandle.Init.Direction					= SPI_DIRECTION_2LINES;
    SpiHandle.Init.DataSize						= SPI_DATASIZE_8BIT;
    SpiHandle.Init.CLKPolarity				= SPI_POLARITY_LOW;
    SpiHandle.Init.CLKPhase 				  = SPI_PHASE_1EDGE;
    SpiHandle.Init.NSS 								= SPI_NSS_SOFT;
    SpiHandle.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_8;//2mHz/8 = 250kHz
    SpiHandle.Init.FirstBit						= SPI_FIRSTBIT_MSB;
		SpiHandle.Init.CRCCalculation    	= SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial		  = 7;
		SpiHandle.Init.TIMode            	= SPI_TIMODE_DISABLE;
    SpiHandle.Init.Mode								= SPI_MODE_MASTER;
		if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
		{
			/* Initialization Error */
			Error_Handler();
		}
		/* Enable SPI peripheral */
    __HAL_SPI_ENABLE(&SpiHandle);

}

/*===========================================================================
* 函数 ：SPI_ExchangeByte() => 通过SPI进行数据交换                          * 
* 输入 ：需要写入SPI的值                                                    * 
* 输出 ：通过SPI读出的值                                                    * 
============================================================================*/
uint8_t SPI_ExchangeByte(uint8_t input)
{
	SPI_SendData(&SpiHandle,input);

//	printf("spi send data:%x\n",input);
//	while (!(__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_TXE)));   // 等待数据传输完成	
//	while (!(__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_RXNE))){}; // 等待数据接收完成
	while (RESET == SPI_GetFlagStatus(&SpiHandle,SPI_FLAG_TXE));   // 等待数据传输完成	
	while (RESET == SPI_GetFlagStatus(&SpiHandle,SPI_FLAG_RXNE)){}; // 等待数据接收完成	

//	printf("spi receive data:%x\n",SPI_ReceiveData(&SpiHandle));
	return (SPI_ReceiveData(&SpiHandle));
}

/**
  * @brief  Transmits a Data through the SPIx peripheral.
  * @param  hspi: To select the SPIx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode.   
  * @param  Data: Data to be transmitted.
  * @retval None
  */
void SPI_SendData(SPI_HandleTypeDef *hspi, uint16_t Data)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_INSTANCE(hspi->Instance));
  
  /* Write in the DR register the data to be sent */
  hspi->Instance->DR = Data;
}

/**
  * @brief  Returns the most recent received data by the SPIx peripheral. 
  * @param  SPIx: To select the SPIx/I2Sx peripheral, where x can be: 1, 2 or 3 
  *         in SPI mode. 
  * @retval The value of the received data.
  */
uint16_t SPI_ReceiveData(SPI_HandleTypeDef *hspi)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_INSTANCE(hspi->Instance));
  
  /* Return the data in the DR register */
  return hspi->Instance->DR;
}

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  hspi specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  SPI_FLAG specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  * @retval The new state of SPI_FLAG (SET or RESET).
  */
FlagStatus SPI_GetFlagStatus(SPI_HandleTypeDef *hspi, uint16_t SPI_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_SPI_ALL_INSTANCE(hspi->Instance));
  
  /* Check the status of the specified SPI flag */
  if ((hspi->Instance->SR & SPI_FLAG) != (uint16_t)RESET)
  {
    /* SPI_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* SPI_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the SPI_FLAG status */
  return  bitstatus;
}

/******************* END OF FILE ******************/
