#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32l0xx.h"
extern void Error_Handler(void);

#define MMA7361L_NOFCHANEL	3

/*=====================sleep IO======================*/
// sleep IO宏定义
#define MMA7361L_SL_GPIO_PORT				GPIOA
#define MMA7361L_SL_GPIO_PIN				GPIO_PIN_8
#define MMA7361L_SL_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()

#define MMA7361L_SL_ON()       			HAL_GPIO_WritePin(MMA7361L_SL_GPIO_PORT, MMA7361L_SL_GPIO_PIN, GPIO_PIN_RESET)
#define MMA7361L_SL_OFF()        		HAL_GPIO_WritePin(MMA7361L_SL_GPIO_PORT, MMA7361L_SL_GPIO_PIN, GPIO_PIN_SET)

/*=====================g-select IO======================*/
// g-select IO宏定义
#define MMA7361L_GS_GPIO_PORT				GPIOB
#define MMA7361L_GS_GPIO_PIN				GPIO_PIN_0
#define MMA7361L_GS_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()

#define MMA7361L_GS_1G5()       		HAL_GPIO_WritePin(MMA7361L_GS_GPIO_PORT, MMA7361L_GS_GPIO_PIN, GPIO_PIN_RESET)
#define MMA7361L_GS_6G()        		HAL_GPIO_WritePin(MMA7361L_GS_GPIO_PORT, MMA7361L_GS_GPIO_PIN, GPIO_PIN_SET)

/*=====================通道1 IO======================*/
// ADC IO宏定义
#define MMA7361L_ADC1_GPIO_PORT			GPIOA
#define MMA7361L_ADC1_GPIO_PIN			GPIO_PIN_5
#define MMA7361L_ADC1_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define MMA7361L_ADC1_CHANNEL     	ADC_CHANNEL_5
/*=====================通道2 IO ======================*/
// ADC IO宏定义
#define MMA7361L_ADC2_GPIO_PORT    	GPIOA
#define MMA7361L_ADC2_GPIO_PIN     	GPIO_PIN_6
#define MMA7361L_ADC2_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define MMA7361L_ADC2_CHANNEL     	ADC_CHANNEL_6
/*=====================通道3 IO ======================*/
// ADC IO宏定义
#define MMA7361L_ADC3_GPIO_PORT    	GPIOA
#define MMA7361L_ADC3_GPIO_PIN     	GPIO_PIN_7
#define MMA7361L_ADC3_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOA_CLK_ENABLE()
#define MMA7361L_ADC3_CHANNEL		   	ADC_CHANNEL_7

// ADC 序号宏定义
#define MMA7361L_ADC            		ADC1
#define MMA7361L_ADC_CLK_ENABLE()	__HAL_RCC_ADC1_CLK_ENABLE()

// ADC DR寄存器宏定义，ADC转换后的数字值则存放在这里
#define MMA7361L_ADC_DR_ADDR    		((uint32_t)ADC1+0x40)

// ADC DMA 通道宏定义，这里我们使用DMA传输
#define MMA7361L_ADC_DMA_CLK_ENABLE()		__HAL_RCC_DMA1_CLK_ENABLE()
#define MMA7361L_ADC_DMA_CHANNEL  	DMA1_Channel1


void MMA7361L_Config(void);

#endif /* __BSP_ADC_H */



