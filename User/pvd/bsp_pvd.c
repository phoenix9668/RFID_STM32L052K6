/********************************************************************************
 Author : PH

 Date :   Feb, 2019

 File name :  bsp_pvd.c 

 Description :	 STM32L052 PVD driver                

 Hardware plateform : 	STM32L052
 
********************************************************************************/

#include "./pvd/bsp_pvd.h"

extern void Error_Handler(void);
extern void Delay(__IO uint32_t nCount);
PWR_PVDTypeDef sConfigPVD;

/**
  * @brief  Configures the PVD resources.
  * @param  None
  * @retval None
  */
void PVD_Config(void)
{
  /*##-1- Enable Power Clock #################################################*/
  __HAL_RCC_PWR_CLK_ENABLE();

  /*##-2- Configure the NVIC for PVD #########################################*/
  HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(PVD_IRQn);

  /* Configure the PVD Level to 5 and generate an interrupt on rising and falling
     edges(PVD detection level set to 2.8V, refer to the electrical characteristics
     of you device datasheet for more details) */
  sConfigPVD.PVDLevel = PWR_PVDLEVEL_5;
  sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
  HAL_PWR_ConfigPVD(&sConfigPVD);

  /* Enable the PVD Output */
  HAL_PWR_EnablePVD();
}

