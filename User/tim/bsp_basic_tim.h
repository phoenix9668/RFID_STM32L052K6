#ifndef __BASIC_TIM_H
#define	__BASIC_TIM_H

#include "stm32l0xx.h"
extern void Error_Handler(void);

#define BASIC_TIM           			TIM6
#define BASIC_TIM_CLK_ENABLE()		__HAL_RCC_TIM6_CLK_ENABLE()

#define BASIC_TIM_IRQn		        TIM6_DAC_IRQn
#define BASIC_TIM_IRQHandler   		TIM6_DAC_IRQHandler

void TIM_Config(void);

#endif /* __BASIC_TIM_H */

