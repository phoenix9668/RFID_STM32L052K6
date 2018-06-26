/********************************************************************************
 Author : PH

 Date :   June, 2018

 File name :  bsp_wwdg.h 

 Description :	 STM32L052 WWDG driver                

 Hardware plateform : 	STM32L052
********************************************************************************/
#ifndef _WWDG_H_
#define _WWDG_H_

#include "stm32l0xx_hal.h"
#include "./spi/bsp_spi.h"
#include "./usart/bsp_debug_usart.h"

void WWDG_Config(void);
void WWDG_Refresh(void);
void WWDGRST_Clear(void);

#endif
