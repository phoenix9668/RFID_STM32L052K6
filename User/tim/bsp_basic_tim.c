/**
  ******************************************************************************
  * @file    bsp_basic_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2017-12-5
  * @brief   基本定时器定时范例
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "./tim/bsp_basic_tim.h"

TIM_HandleTypeDef TimHandle;
uint32_t uwPrescalerValue = 0;
/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
void TIM_Config(void)
{
  /* Compute the prescaler value to have TIM6 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) (SystemCoreClock / 10000) - 1;

	TimHandle.Instance = BASIC_TIM;
	/* 累计 TIM_Period个后产生一个更新或者中断*/
	//当定时器从0计数到9999，即为10000次，为一个定时周期
	TimHandle.Init.Period = 10000-1;
	
	//定时器时钟源TIMxCLK = 2 * PCLK1
	//				PCLK1 = HCLK / 4 
	//				=> TIMxCLK=HCLK/2=SystemCoreClock/2=84MHz
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
	TimHandle.Init.Prescaler = uwPrescalerValue;
	TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;  
	
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }	
}

/*********************************************END OF FILE**********************/
