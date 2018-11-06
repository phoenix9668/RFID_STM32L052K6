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
extern void Delay(__IO uint32_t nCount);
extern void Error_Handler(void);

void CC1101_CSN_LOW(void)
{
	Delay(2);
	HAL_GPIO_WritePin(ADXL362_SPI_CSN_GPIO_PORT, ADXL362_SPI_CSN_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CC1101_SPI_CSN_GPIO_PORT, CC1101_SPI_CSN_PIN, GPIO_PIN_RESET);
	Delay(2);
}

void CC1101_CSN_HIGH(void)
{
//	Delay(1);
	HAL_GPIO_WritePin(CC1101_SPI_CSN_GPIO_PORT, CC1101_SPI_CSN_PIN, GPIO_PIN_SET);
//	Delay(2);
}

void ADXL362_CSN_LOW(void)
{
	Delay(2);
	HAL_GPIO_WritePin(CC1101_SPI_CSN_GPIO_PORT, CC1101_SPI_CSN_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ADXL362_SPI_CSN_GPIO_PORT, ADXL362_SPI_CSN_PIN, GPIO_PIN_RESET);
	Delay(2);
}

void ADXL362_CSN_HIGH(void)
{
	Delay(2);
	HAL_GPIO_WritePin(ADXL362_SPI_CSN_GPIO_PORT, ADXL362_SPI_CSN_PIN, GPIO_PIN_SET);
	Delay(2);
}

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
		ADC_IN1_CLK_ENABLE();
    CC1101_IRQ_GPIO_CLK_ENABLE();
//    CC1101_GDO2_GPIO_CLK_ENABLE();
		ADXL362_INT1_GPIO_CLK_ENABLE();
		ADXL362_INT2_GPIO_CLK_ENABLE();

    //配置LED的GPIO引脚
    GPIO_InitStructure.Pin = LED_GREEN_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);

		LED_GREEN_ON();

		//配置ADC_IN1的GPIO引脚
    GPIO_InitStructure.Pin = ADC_IN1_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull  = GPIO_PULLDOWN;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ADC_IN1_PORT, &GPIO_InitStructure);
		
		/* Enable GPIOs clock */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
	
		/* Configure GPIO port pins in Analog Input mode (floating input trigger OFF) */
		GPIO_InitStructure.Pin = PA0_PIN;
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = PA4_PIN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = PA8_PIN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = PA9_PIN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = PA10_PIN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = PA11_PIN;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.Pin = PB0_PIN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = PB1_PIN;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.Pin = PC14_PIN;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = PC15_PIN;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		/* Disable GPIOs clock */
		__HAL_RCC_GPIOC_CLK_DISABLE();
		
}

/**
  * @brief  INT_GPIO_Config function
  * @param  None
  * @retval None
  */
void INT_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	
    GPIO_InitStructure.Pin = CC1101_IRQ_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CC1101_IRQ_GPIO_PORT, &GPIO_InitStructure);

		/* Configure GDO2 pin as input floating */
		GPIO_InitStructure.Pin = CC1101_GDO2_PIN;
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(CC1101_GDO2_GPIO_PORT, &GPIO_InitStructure);
		
		/* Configure INT2 pin as input floating */
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Pin = ADXL362_INT2_PIN;
		HAL_GPIO_Init(ADXL362_INT2_GPIO_PORT, &GPIO_InitStructure);

		/* Configure INT1 pin as input floating */
		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Pin = ADXL362_INT1_PIN;
		HAL_GPIO_Init(ADXL362_INT1_GPIO_PORT, &GPIO_InitStructure);

		/* Enable and set EXTI4_15 Interrupt to the lowest priority */
		HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
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
		ADXL362_SPI_CSN_GPIO_CLK_ENABLE();

    /* Set SPI_CSN Pin */
    GPIO_InitStructure.Pin = CC1101_SPI_CSN_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CC1101_SPI_CSN_GPIO_PORT, &GPIO_InitStructure);
		
		/* Set SPI_CSN2 Pin */
    GPIO_InitStructure.Pin = ADXL362_SPI_CSN_PIN;
    HAL_GPIO_Init(ADXL362_SPI_CSN_GPIO_PORT, &GPIO_InitStructure);
    
    /* Set SPI_CSN Pin High */
    CC1101_CSN_HIGH();
		ADXL362_CSN_HIGH();	
	
    /* Set SPI_SCK Pin */
    GPIO_InitStructure.Pin = CC1101_SPI_SCK_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStructure.Alternate = CC1101_SPI_SCK_AF;
    HAL_GPIO_Init(CC1101_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
    
    /* Set SPI_MOSI Pin */
    GPIO_InitStructure.Pin = CC1101_SPI_MOSI_PIN;
		GPIO_InitStructure.Alternate = CC1101_SPI_MOSI_AF;
    HAL_GPIO_Init(CC1101_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /* Set SPI_MISO Pin */
    GPIO_InitStructure.Pin = CC1101_SPI_MISO_PIN;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Alternate = CC1101_SPI_MISO_AF;
    HAL_GPIO_Init(CC1101_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
    
		/* Enable the SPI periph */
    CC1101_SPI_CLK_ENABLE();
    
    /* SPI configuration -------------------------------------------------------*/
    SpiHandle.Instance								= CC1101_SPI;
    SpiHandle.Init.Direction					= SPI_DIRECTION_2LINES;
    SpiHandle.Init.DataSize						= SPI_DATASIZE_8BIT;
    SpiHandle.Init.CLKPolarity				= SPI_POLARITY_LOW;
    SpiHandle.Init.CLKPhase 				  = SPI_PHASE_1EDGE;
    SpiHandle.Init.NSS 								= SPI_NSS_SOFT;
    SpiHandle.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_2;//65.536 KHz/2 = 32.768kHz
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

// Function for sending and receiving data through SPI
void SpiFunction(unsigned char OutputBuff[],unsigned char InputBuff[], unsigned int OutNoOfBytes, unsigned int InNoOfBytes)
{
	int i;

	for(i=0;i<OutNoOfBytes;i++)
	{
    SPI_ExchangeByte(OutputBuff[i]);					// Send data
	}
   
	for(i=0;i<InNoOfBytes;i++)
	{
		InputBuff[i] = SPI_ExchangeByte(0xFF);		// Receive data
	}
	/**/
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
