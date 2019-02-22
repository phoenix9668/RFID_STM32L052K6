/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32L0xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
unsigned char Temp;
uint8_t rx_index = 0x0;
__IO uint8_t step_stage;
__IO uint16_t timedelay;
__IO uint8_t wwdg_time;
__IO uint8_t wwdg_flag;
__IO uint32_t step1;
__IO uint32_t step2;
__IO uint32_t step3;
__IO uint32_t step4;
__IO uint32_t step5;
__IO uint32_t step6;
__IO uint32_t step7;
__IO uint32_t step8;
__IO uint32_t step9;
__IO uint32_t step10;
__IO uint32_t step11;
__IO uint32_t step12;
__IO uint8_t battery_low;
uint32_t sysclockfreq;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern uint8_t aRxBuffer[RXBUFFERSIZE];								// Buffer used for reception
extern uint8_t addr_eeprom;
extern uint16_t sync_eeprom;
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void Error_Handler(void);
static void Show_Message(void);
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

/*===========================================================================
* 函数 : main() => 主函数，程序入口                                       	*
* 说明 : 接收一包数据，并发送应答数据																				*
============================================================================*/
int main(void)
{
	Delay(2000);
	/* STM32L0xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user 
             can eventually implement his proper time base source (a general purpose 
             timer for example or other time source), keeping in mind that Time base 
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
             handled in milliseconds basis.
       - Low Level Initialization
	*/
	HAL_Init();

	/* Disable Prefetch Buffer */
  __HAL_FLASH_PREFETCH_BUFFER_DISABLE();

  /* Configure the system clock @ 64 KHz */
  SystemClock_Config();
	
	System_Initial();
	
	/* Check if the system has resumed from WWDG reset */
	WWDGRST_Clear();

	Show_Message();
	
	/* Enter LP RUN mode */
	HAL_PWREx_EnableLowPowerRunMode();

	/* Wait until the system enters LP RUN and the Regulator is in LP mode */
	while(__HAL_PWR_GET_FLAG(PWR_FLAG_REGLP) == RESET){}	

//	/* Exit LP RUN mode */
//	HAL_PWREx_DisableLowPowerRunMode();

//	/* Wait until the system exits LP RUN and the Regulator is in main mode */
//	while(__HAL_PWR_GET_FLAG(PWR_FLAG_REGLP) != RESET){}

	sysclockfreq = HAL_RCC_GetSysClockFreq();
//	printf("sysclockfreq = %d\n",sysclockfreq);
	/*Configure the SysTick to have interrupt in 10s time basis*/
	HAL_SYSTICK_Config(sysclockfreq*10);
	/*Configure the SysTick IRQ priority */
	HAL_NVIC_SetPriority(SysTick_IRQn, TICK_INT_PRIORITY ,0U);

	/* Init & Start WWDG peripheral */
	WWDG_Config();

	wwdg_time = 0x3;
	timedelay = Time_Delay;
	step1 = DATAEEPROM_Read(EEPROM_START_ADDR+8);
	DATAEEPROM_Program(EEPROM_START_ADDR+8, 0x0);
	step2 = DATAEEPROM_Read(EEPROM_START_ADDR+16);
	DATAEEPROM_Program(EEPROM_START_ADDR+16, 0x0);
	step3 = DATAEEPROM_Read(EEPROM_START_ADDR+24);
	DATAEEPROM_Program(EEPROM_START_ADDR+24, 0x0);
	step4 = DATAEEPROM_Read(EEPROM_START_ADDR+32);
	DATAEEPROM_Program(EEPROM_START_ADDR+32, 0x0);	
	step5 = DATAEEPROM_Read(EEPROM_START_ADDR+40);
	DATAEEPROM_Program(EEPROM_START_ADDR+40, 0x0);
	step6 = DATAEEPROM_Read(EEPROM_START_ADDR+48);
	DATAEEPROM_Program(EEPROM_START_ADDR+48, 0x0);
	step7 = DATAEEPROM_Read(EEPROM_START_ADDR+56);
	DATAEEPROM_Program(EEPROM_START_ADDR+56, 0x0);
	step8 = DATAEEPROM_Read(EEPROM_START_ADDR+64);
	DATAEEPROM_Program(EEPROM_START_ADDR+64, 0x0);
	step9 = DATAEEPROM_Read(EEPROM_START_ADDR+72);
	DATAEEPROM_Program(EEPROM_START_ADDR+72, 0x0);
	step10 = DATAEEPROM_Read(EEPROM_START_ADDR+80);
	DATAEEPROM_Program(EEPROM_START_ADDR+80, 0x0);
	step11 = DATAEEPROM_Read(EEPROM_START_ADDR+88);
	DATAEEPROM_Program(EEPROM_START_ADDR+88, 0x0);
	step12 = DATAEEPROM_Read(EEPROM_START_ADDR+96);
	DATAEEPROM_Program(EEPROM_START_ADDR+96, 0x0);
	step_stage = (uint8_t)(0x000000FF & DATAEEPROM_Read(EEPROM_START_ADDR+104));
	DATAEEPROM_Program(EEPROM_START_ADDR+104, 0x0);
	battery_low = (uint8_t)(0x000000FF & DATAEEPROM_Read(EEPROM_START_ADDR+112));
	DATAEEPROM_Program(EEPROM_START_ADDR+112, 0x0);

	while(1)
		{
			rx_index = RF_RecvHandler();
			if(rx_index != 0x0)   // 无线数据接收处理
			{
				RF_SendPacket(rx_index);
			}
			if(wwdg_time == 0x0)
			{
				#ifdef DEBUG
				if(ADC_IN1_READ() == 1)
				{
					printf("timedelay_1 = %d\n",timedelay);
					printf("wwdg_time_1 = %d\n",wwdg_time);
				}
				#endif
				WWDG_Refresh();
				wwdg_time = 0x3;
			}
			if(timedelay == 0x0)
			{
				#ifdef DEBUG
				if(ADC_IN1_READ() == 1)
				{
					printf("timedelay_2 = %d\n",timedelay);
					printf("wwdg_time_2 = %d\n",wwdg_time);
				}
				#endif
				timedelay = Time_Delay;
				step_stage++;
				if(step_stage%3 == 0)
				{
					RF_Initial(addr_eeprom, sync_eeprom, IDLE);
				}
			}
			wwdg_flag = 0x1;
		}
}

#ifdef UART_PROG
/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
//  printf("trasfer complete\n");
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
		uint32_t data;
		uint8_t uart_addr_eeprom;// 从eeprom中读出的数
		uint16_t uart_sync_eeprom;
		uint32_t uart_rfid_eeprom;
		#ifdef DEBUG
		if(ADC_IN1_READ() == 1)
		{
			printf("data = %x %x %x %x %x %x %x %x %x %x %x %x \n",aRxBuffer[0],aRxBuffer[1],aRxBuffer[2],aRxBuffer[3],aRxBuffer[4],aRxBuffer[5],aRxBuffer[6],aRxBuffer[7],aRxBuffer[8],aRxBuffer[9],aRxBuffer[10],aRxBuffer[11]);
		}
		#endif
		/*##-1- Check UART receive data whether is ‘ABCD’ begin or not ###########################*/
		if(aRxBuffer[0] == 0x41 && aRxBuffer[1] == 0x42 && aRxBuffer[2] == 0x43 && aRxBuffer[3] == 0x44)//输入‘ABCD’
			{
				data = ((uint32_t)(0xFF000000 & aRxBuffer[4]<<24)+(uint32_t)(0x00FF0000 & aRxBuffer[5]<<16)+(uint32_t)(0x0000FF00 & aRxBuffer[6]<<8)+(uint32_t)(0x000000FF & aRxBuffer[7]));
				DATAEEPROM_Program(EEPROM_START_ADDR, data);
				data = ((uint32_t)(0xFF000000 & aRxBuffer[8]<<24)+(uint32_t)(0x00FF0000 & aRxBuffer[9]<<16)+(uint32_t)(0x0000FF00 & aRxBuffer[10]<<8)+(uint32_t)(0x000000FF & aRxBuffer[11]));
				DATAEEPROM_Program(EEPROM_START_ADDR+4, data);
				uart_addr_eeprom = (uint8_t)(0xff & DATAEEPROM_Read(EEPROM_START_ADDR)>>16);
				uart_sync_eeprom = (uint16_t)(0xffff & DATAEEPROM_Read(EEPROM_START_ADDR));
				uart_rfid_eeprom	= DATAEEPROM_Read(EEPROM_START_ADDR+4);
				#ifdef DEBUG
				if(ADC_IN1_READ() == 1)
				{
					printf("eeprom program end\n");
					printf("addr_eeprom = %x\n",uart_addr_eeprom);
					printf("sync_eeprom = %x\n",uart_sync_eeprom);
					printf("rfid_eeprom = %x\n",uart_rfid_eeprom);
				}
				#endif
			}
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
		while(1)
    {
			LED_GREEN_OFF();
    }
}
#endif

/**
  * @brief EXTI line detection callback.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == ADXL362_INT1_PIN)
  {
		switch(step_stage)
		{
			case 0x0: step1++;break;
			case 0x1: step2++;break;
			case 0x2: step3++;break;
			case 0x3: step4++;break;
			case 0x4: step5++;break;
			case 0x5: step6++;break;
			case 0x6: step7++;break;
			case 0x7: step8++;break;
			case 0x8: step9++;break;
			case 0x9: step10++;break;
			case 0x10: step11++;break;
			default: step12++;
		}
//		if(CC1101_GDO2_READ() == 0)//WOR状态被中断打断
//		{
//			while (CC1101_GDO2_READ() == 0);
//			LED5_Red_TOG();
//		}

		#ifdef DEBUG
		if(ADC_IN1_READ() == 1)
		{
			Temp = ADXL362RegisterRead(XL362_STATUS);
			printf("Status is %x\n",Temp);
			printf("Step1 is %d\n",step1);
			printf("Step2 is %d\n",step2);
			printf("Step3 is %d\n",step3);
			printf("Step4 is %d\n",step4);
			printf("Step5 is %d\n",step5);
			printf("Step6 is %d\n",step6);
			printf("Step7 is %d\n",step7);
			printf("Step8 is %d\n",step8);
			printf("Step9 is %d\n",step9);
			printf("Step10 is %d\n",step10);
			printf("Step11 is %d\n",step11);
			printf("Step12 is %d\n",step12);
//			Temp = CC1101ReadStatus(CC1101_MARCSTATE);
//			printf("CC1101_MARCSTATE is %d\n",Temp);
		}
		#endif
  }
}

/**
  * @brief Decrement detection callback.
  * @param None
  * @retval None
  */
void HAL_SysTick_Decrement(void)
{
	if(timedelay != 0x0)
	{
		timedelay--;
	}
	if(wwdg_time != 0x0)
	{
		wwdg_time--;
	}
}

/**
  * @brief  PWR PVD interrupt callback
  * @param  none 
  * @retval none
  */
void HAL_PWR_PVDCallback(void)
{
	battery_low = 0x01;
	DATAEEPROM_Program(EEPROM_START_ADDR+112, battery_low);
}

/**
  * @brief  WWDG Early Wakeup callback.
  * @param  hwwdg  pointer to a WWDG_HandleTypeDef structure that contains
  *                the configuration information for the specified WWDG module.
  * @retval None
  */
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg)
{
	if(wwdg_flag == 0x1)
	{
		WWDG_Refresh();
		#ifdef DEBUG
		if(ADC_IN1_READ() == 1)
		{
			printf("EarlyWakeupCallback\n");
			printf("wwdg_time = %d\n",wwdg_time);
			printf("wwdg_flag = %d\n",wwdg_flag);
		}
		#endif
		wwdg_time = 0x3;
		wwdg_flag = 0x0;
	}
	DATAEEPROM_Program(EEPROM_START_ADDR+8, step1);
	DATAEEPROM_Program(EEPROM_START_ADDR+16, step2);
	DATAEEPROM_Program(EEPROM_START_ADDR+24, step3);
	DATAEEPROM_Program(EEPROM_START_ADDR+32, step4);
	DATAEEPROM_Program(EEPROM_START_ADDR+40, step5);
	DATAEEPROM_Program(EEPROM_START_ADDR+48, step6);
	DATAEEPROM_Program(EEPROM_START_ADDR+56, step7);
	DATAEEPROM_Program(EEPROM_START_ADDR+64, step8);
	DATAEEPROM_Program(EEPROM_START_ADDR+72, step9);
	DATAEEPROM_Program(EEPROM_START_ADDR+80, step10);
	DATAEEPROM_Program(EEPROM_START_ADDR+88, step11);
	DATAEEPROM_Program(EEPROM_START_ADDR+96, step12);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = (MSI)
  *            MSI Range                      = 0
  *            SYSCLK(Hz)                     = 65536
  *            HCLK(Hz)                       = 65536
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            Main regulator output voltage  = Scale2 mode
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.MSICalibrationValue = 0x00;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
	uint32_t i;
	uint16_t m = 500;

	for(i = 0;i<=nCount; i++)
	{
		while(m)
		{
			m--;
		}
	}
}

/**
  * @brief  打印指令输入提示信息
  * @param  none
  * @retval none
  */
static void Show_Message(void)
{
	unsigned int  ReadValueTemp;
	#ifdef DEBUG
	if(ADC_IN1_READ() == 1)
	{
		printf("\r\n CC1101 chip transfer program \n");
		printf(" using USART2,configuration:%d 8-N-1 \n",DEBUG_USART_BAUDRATE);
		printf(" when in transfer mode,the data must not exceed 60 bytes!!\r\n");  
	}
	#endif
	ReadValueTemp = ADXL362RegisterRead(XL362_DEVID_AD);     	//Analog Devices device ID, 0xAD
	if(ReadValueTemp == 0xAD)
	{
		LED_GREEN_OFF();
		Delay(500);
	}
	#ifdef DEBUG
	if(ADC_IN1_READ() == 1)
	{
		printf("Analog Devices device ID: %x\n",ReadValueTemp);	 	//send via UART
	}
	#endif
	ReadValueTemp = ADXL362RegisterRead(XL362_DEVID_MST);    	//Analog Devices MEMS device ID, 0x1D
	if(ReadValueTemp == 0x1D)
	{
		LED_GREEN_ON();
		Delay(500);
	}
	#ifdef DEBUG
	if(ADC_IN1_READ() == 1)
	{
		printf("Analog Devices MEMS device ID: %x\n",ReadValueTemp);	//send via UART
	}
	#endif
	ReadValueTemp = ADXL362RegisterRead(XL362_PARTID);       	//part ID, 0xF2
	if(ReadValueTemp == 0xF2)
	{
		LED_GREEN_OFF();
		Delay(500);
	}
	#ifdef DEBUG
	if(ADC_IN1_READ() == 1)
	{
		printf("Part ID: %x\n",ReadValueTemp);										//send via UART
	}
	#endif
	ReadValueTemp = ADXL362RegisterRead(XL362_REVID);       	//version ID, 0x02
	if(ReadValueTemp == 0x02)
	{
		LED_GREEN_ON();
	}
	#ifdef DEBUG
	if(ADC_IN1_READ() == 1)
	{
		printf("Version ID: %x\n",ReadValueTemp);									//send via UART
	}
	#endif
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  LED_GREEN_OFF();
	#ifdef DEBUG
	printf("error\n");
	#endif
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
