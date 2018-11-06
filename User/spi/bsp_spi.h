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

#ifndef _BSP_SPI_H_
#define _BSP_SPI_H_

#include "stm32l0xx.h"
#include "./cc1101/cc1101.h"
#include "./usart/bsp_debug_usart.h"
/**
  * @brief  CC1101 SPI Interface pins
  */
#define CC1101_SPI                      SPI1
#define CC1101_SPI_CLK_ENABLE()  				__HAL_RCC_SPI1_CLK_ENABLE()

#define CC1101_SPI_SCK_PIN              GPIO_PIN_3                 	/* PB.3 */
#define CC1101_SPI_SCK_GPIO_PORT        GPIOB                       /* GPIOB */
#define CC1101_SPI_SCK_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()
#define CC1101_SPI_SCK_AF               GPIO_AF0_SPI1

#define CC1101_SPI_MISO_PIN             GPIO_PIN_4                 	/* PB.4 */
#define CC1101_SPI_MISO_GPIO_PORT       GPIOB                       /* GPIOB */
#define CC1101_SPI_MISO_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOB_CLK_ENABLE()
#define CC1101_SPI_MISO_AF              GPIO_AF0_SPI1

#define CC1101_SPI_MOSI_PIN             GPIO_PIN_5                 	/* PB.5 */
#define CC1101_SPI_MOSI_GPIO_PORT       GPIOB                       /* GPIOB */
#define CC1101_SPI_MOSI_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOB_CLK_ENABLE()
#define CC1101_SPI_MOSI_AF              GPIO_AF0_SPI1

/*===========================================================================
------------------------------Internal IMPORT functions----------------------
you must offer the following functions for this module
1. uint8_t SPI_ExchangeByte(uint8_t input); // SPI Send and Receive function
2. CC1101_CSN_LOW();                        // Pull down the CSN line
3. CC1101_CSN_HIGH();                       // Pull up the CSN Line
===========================================================================*/
// CC1101相关控制引脚定义， CSN(PA15), IRQ(PB6), GDO2(PA12)
#define CC1101_SPI_CSN_PIN              GPIO_PIN_15                	/* PA.15 */
#define CC1101_SPI_CSN_GPIO_PORT        GPIOA                       /* GPIOA */
#define CC1101_SPI_CSN_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()

#define CC1101_IRQ_PIN                  GPIO_PIN_6                  /* PB.06 */
#define CC1101_IRQ_GPIO_PORT            GPIOB                       /* GPIOB */
#define CC1101_IRQ_GPIO_CLK_ENABLE() 		__HAL_RCC_GPIOB_CLK_ENABLE()

#define CC1101_GDO2_PIN                 GPIO_PIN_12                	/* PA.12 */
#define CC1101_GDO2_GPIO_PORT           GPIOA                       /* GPIOA */
#define CC1101_GDO2_GPIO_CLK_ENABLE()  	__HAL_RCC_GPIOA_CLK_ENABLE()

//#define CC1101_CSN_LOW()                HAL_GPIO_WritePin(CC1101_SPI_CSN_GPIO_PORT, CC1101_SPI_CSN_PIN, GPIO_PIN_RESET)

//#define CC1101_CSN_HIGH()               HAL_GPIO_WritePin(CC1101_SPI_CSN_GPIO_PORT, CC1101_SPI_CSN_PIN, GPIO_PIN_SET)

#define CC1101_IRQ_READ()               HAL_GPIO_ReadPin(CC1101_IRQ_GPIO_PORT, CC1101_IRQ_PIN)

#define CC1101_GDO2_READ()             	HAL_GPIO_ReadPin(CC1101_GDO2_GPIO_PORT, CC1101_GDO2_PIN)

/*===========================================================================
4. ADXL362_CSN_LOW();                 			// Pull down the CSN line
5. ADXL362_CSN_HIGH();                			// Pull up the CSN Line
===========================================================================*/
// ADXL362相关控制引脚定义， CSN(PA5), INT1(PA7), INT2(PA6)
#define ADXL362_SPI_CSN_PIN         		GPIO_PIN_5                	/* PA.5 */
#define ADXL362_SPI_CSN_GPIO_PORT    		GPIOA                       /* GPIOA */
#define ADXL362_SPI_CSN_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()

#define ADXL362_INT1_PIN              	GPIO_PIN_7                  /* PA.07 */
#define ADXL362_INT1_GPIO_PORT        	GPIOA                     	/* GPIOA */
#define ADXL362_INT1_GPIO_CLK_ENABLE() 	__HAL_RCC_GPIOA_CLK_ENABLE()

#define ADXL362_INT2_PIN              	GPIO_PIN_6                	/* PA.06 */
#define ADXL362_INT2_GPIO_PORT         	GPIOA                       /* GPIOA */
#define ADXL362_INT2_GPIO_CLK_ENABLE() 	__HAL_RCC_GPIOA_CLK_ENABLE()

//#define ADXL362_CSN_LOW()             	HAL_GPIO_WritePin(ADXL362_SPI_CSN_GPIO_PORT, ADXL362_SPI_CSN_PIN, GPIO_PIN_RESET)

//#define ADXL362_CSN_HIGH()             	HAL_GPIO_WritePin(ADXL362_SPI_CSN_GPIO_PORT, ADXL362_SPI_CSN_PIN, GPIO_PIN_SET)

#define ADXL362_INT1_READ()          		HAL_GPIO_ReadPin(ADXL362_INT1_GPIO_PORT, ADXL362_INT1_PIN)

#define ADXL362_INT2_READ()           	HAL_GPIO_ReadPin(ADXL362_INT2_GPIO_PORT, ADXL362_INT2_PIN)

/**
  * @brief  LED Interface pins
  */
#define LED_GREEN_PIN        						GPIO_PIN_7
#define	LED_GPIO_PORT      	 						GPIOB
#define	LED_GPIO_CLK_ENABLE()        		__HAL_RCC_GPIOB_CLK_ENABLE()

/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)									{p->BSRR=i;}			  						//设置为高电平
#define digitalLo(p,i)									{p->BSRR=(uint32_t)i << 16;}		//输出低电平
#define digitalToggle(p,i)							{p->ODR ^=i;}										//输出反转状态

// LED操作函数，(ON)打开, (OFF)关闭，(TOG)翻转
#define LED_GREEN_OFF()      						HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GREEN_PIN, GPIO_PIN_RESET)        
#define LED_GREEN_ON()       						HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GREEN_PIN, GPIO_PIN_SET)
#define LED_GREEN_TOG()      						digitalToggle(LED_GPIO_PORT,LED_GREEN_PIN)

/**
  * @brief  ADC_IN1 Interface pins
  */
#define ADC_IN1_PIN        							GPIO_PIN_1
#define	ADC_IN1_PORT      	 						GPIOA
#define	ADC_IN1_CLK_ENABLE()        		__HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC_IN1_READ()               		HAL_GPIO_ReadPin(ADC_IN1_PORT, ADC_IN1_PIN)

/**
  * @brief  No Used Interface pins
  */
#define PA0_PIN        							GPIO_PIN_0
#define PA4_PIN        							GPIO_PIN_4
#define PA8_PIN        							GPIO_PIN_8
#define PA9_PIN        							GPIO_PIN_9
#define PA10_PIN        						GPIO_PIN_10
#define PA11_PIN        						GPIO_PIN_11

#define PB0_PIN        							GPIO_PIN_0
#define PB1_PIN        							GPIO_PIN_1

#define PC14_PIN        						GPIO_PIN_14
#define PC15_PIN        						GPIO_PIN_15


void CC1101_CSN_LOW(void);
void CC1101_CSN_HIGH(void);
void ADXL362_CSN_LOW(void);
void ADXL362_CSN_HIGH(void);
void GPIO_Config(void);                // 初始化通用IO端口
void INT_GPIO_Config(void);
void SPI_Config(void);                 // 初始化SPI

uint8_t SPI_ExchangeByte(uint8_t input);
void SpiFunction(unsigned char OutputBuff[],unsigned char InputBuff[], unsigned int OutNoOfBytes, unsigned int InNoOfBytes);
void SPI_SendData(SPI_HandleTypeDef *hspi, uint16_t Data);
uint16_t SPI_ReceiveData(SPI_HandleTypeDef *hspi);
FlagStatus SPI_GetFlagStatus(SPI_HandleTypeDef *hspi, uint16_t SPI_FLAG);

#endif //_BSP_SPI_H_

/******************* END OF FILE ******************/
