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
//#define MMA7361L_DEBUG
extern uint8_t aRxBuffer[RXBUFFERSIZE];								// Buffer used for reception
/** @addtogroup STM32L0xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*===========================================================================
* 函数 : main() => 主函数，程序入口                                         																																*
* 说明 : 接收一包数据，并发送应答数据																																																*
============================================================================*/
int main(void)
{
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

	/* Configure the System clock to have a frequency of 32 MHz */
	SystemClock_Config();
	System_Initial();
	#ifdef DEBUG
	Show_Message();
	#endif

	while(1)
		{
			index = RF_RecvHandler();
			if(index != 0)   // 无线数据接收处理
			{
				RF_SendPacket(index);
			}
			#ifdef MMA7361L_DEBUG
			MMA7361L_display();
			#endif
		}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(RecvWaitTime != 0 && RecvWaitTime != 1)	// 数据接收计时
		{	RecvWaitTime--;}
	else if(RecvWaitTime == 1)
		{	RecvFlag=1;}
	else
		{	RecvFlag=0;}
        
	if(SendTime != 0)                           // 0.1s时间到，置位SendFlag标志，主函数查询发送数据  
		{ 
			if(--SendTime == 0)    
				{	SendTime=SEND_GAP; 
						SendFlag=1;
						LED_GREEN_TOG();
						MMA7361L_ReadHandler();
				}
		}
}
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
		uint8_t addr_eeprom;// 从eeprom中读出的数
		uint16_t sync_eeprom;
		uint32_t rfid_eeprom;
		#ifdef DEBUG
		printf("data = %x %x %x %x %x %x %x %x %x %x %x %x \n",aRxBuffer[0],aRxBuffer[1],aRxBuffer[2],aRxBuffer[3],aRxBuffer[4],aRxBuffer[5],aRxBuffer[6],aRxBuffer[7],aRxBuffer[8],aRxBuffer[9],aRxBuffer[10],aRxBuffer[11]);
		#endif
		/*##-1- Check UART receive data whether is ‘ABCD’ begin or not ###########################*/
		if(aRxBuffer[0] == 0x41 && aRxBuffer[1] == 0x42 && aRxBuffer[2] == 0x43 && aRxBuffer[3] == 0x44)//输入‘ABCD’
			{
				data = ((uint32_t)(0xFF000000 & aRxBuffer[4]<<24)+(uint32_t)(0x00FF0000 & aRxBuffer[5]<<16)+(uint32_t)(0x0000FF00 & aRxBuffer[6]<<8)+(uint32_t)(0x000000FF & aRxBuffer[7]));
				DATAEEPROM_Program(EEPROM_START_ADDR, data);
				data = ((uint32_t)(0xFF000000 & aRxBuffer[8]<<24)+(uint32_t)(0x00FF0000 & aRxBuffer[9]<<16)+(uint32_t)(0x0000FF00 & aRxBuffer[10]<<8)+(uint32_t)(0x000000FF & aRxBuffer[11]));
				DATAEEPROM_Program(EEPROM_START_ADDR+4, data);
				addr_eeprom = (uint8_t)(0xff & DATAEEPROM_Read(EEPROM_START_ADDR)>>16); 
				sync_eeprom = (uint16_t)(0xffff & DATAEEPROM_Read(EEPROM_START_ADDR));
				rfid_eeprom	= DATAEEPROM_Read(EEPROM_START_ADDR+4);
				#ifdef DEBUG
				printf("eeprom program end\n");
				printf("addr_eeprom = %x\n",addr_eeprom);
				printf("sync_eeprom = %x\n",sync_eeprom);
				printf("rfid_eeprom = %x\n",rfid_eeprom);
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
			LED_GREEN_ON();
    }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_MUL                        = 4
  *            PLL_DIV                        = 2
  *            Flash Latency(WS)              = 1
  *            Main regulator output voltage  = Scale1 mode
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

/**
  * @brief  打印指令输入提示信息
  * @param  none
  * @retval none
  */
#ifdef DEBUG
static void Show_Message(void)
{   
	printf("\r\n CC1101 chip transfer performance test program \n");
	printf(" using USART3,configuration:%d 8-N-1 \n",DEBUG_USART_BAUDRATE);
	printf(" you need press USER button when you want transfer data\r\n");
	printf(" if choose transfer,the data must not exceed 60 bytes!!\r\n");
	printf(" PS: green led light when system in transfer mode\r\n");    
	printf("     orange led light when system in receive mode\r\n");
}
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  LED_GREEN_ON();
	printf("error\n");
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
