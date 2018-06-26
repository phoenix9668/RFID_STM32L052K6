/********************************************************************************
 Author : PH

 Date :   June, 2018

 File name :  bsp_wwdg.c 

 Description :	 STM32L052 WWDG driver                

 Hardware plateform : 	STM32L052
 
********************************************************************************/

#include "./wwdg/bsp_wwdg.h"

extern void Error_Handler(void);
extern void Delay(__IO uint32_t nCount);
/* WWDG handler declaration */
WWDG_HandleTypeDef WwdgHandle;

/*******************************************************************
  @brief  WWDG_Config function
  @param  None
  @retval None
*******************************************************************/
void WWDG_Config(void)
{
  /*## Init & Start WWDG peripheral ######################################*/
  /* WWDG clock counter = (PCLK1 (0.065536MHz)/4096)/8) = 2Hz (500 ms) 
     WWDG Window value = 80 means that the WWDG counter should be refreshed only 
     when the counter is below 80 (and greater than 64) otherwise a reset will 
     be generated. 
     WWDG Counter value = 127, WWDG timeout = 500 ms * 64 = 32 s
     In this case the refresh window is comprised between : 500 * (127-80) = 23.5s and 500 * 64 = 32 s
   */
  WwdgHandle.Instance = WWDG;
  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;
  WwdgHandle.Init.Window    = 0x50;
  WwdgHandle.Init.Counter   = 0x7F;
  WwdgHandle.Init.EWIMode   = WWDG_EWI_ENABLE;

  if (HAL_WWDG_Init(&WwdgHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
	
	HAL_WWDG_MspInit(&WwdgHandle);
}

/*******************************************************************
  @brief  WWDG_Refresh function
  @param  None
  @retval None
*******************************************************************/
void WWDG_Refresh(void)
{
	if (HAL_WWDG_Refresh(&WwdgHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/*******************************************************************
  @brief  WWDGRST_Clear function
  @param  None
  @retval None
*******************************************************************/
void WWDGRST_Clear(void)
{
  /*## Check if the system has resumed from WWDG reset ####################*/
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    /* WWDGRST flag set: Turn LED on */
    LED_GREEN_OFF();

    /* Insert 4s delay */
    Delay(8000);

    /* Prior to clear WWDGRST flag: Turn LED off */
    LED_GREEN_ON();
  }

  /* Clear reset flags in any case */
  __HAL_RCC_CLEAR_RESET_FLAGS();	
}
