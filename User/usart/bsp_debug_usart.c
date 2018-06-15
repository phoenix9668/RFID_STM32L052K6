 /**
  ******************************************************************************
  * @file    bsp_debug_usart.c
  * @author  phoenix
  * @version V1.0.0
  * @date    23-October-2017
  * @brief   重定向c库printf函数到usart端口，中断接收模式
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 
  
#include "./usart/bsp_debug_usart.h"

UART_HandleTypeDef UartHandle;

 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
  * @param  无
  * @retval 无
  */
void Debug_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
		
	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	DEBUG_USART_RX_GPIO_CLK_ENABLE();
	DEBUG_USART_TX_GPIO_CLK_ENABLE();
	
	/* Enable USART2 clock */
	DEBUG_USART_CLK_ENABLE();
  
	/*##-2- Configure peripheral GPIO ##########################################*/  
	/* UART TX GPIO pin configuration  */
	GPIO_InitStructure.Pull = GPIO_NOPULL;  
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pin = DEBUG_USART_TX_PIN;
	GPIO_InitStructure.Alternate = DEBUG_USART_TX_AF;  
	HAL_GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStructure.Pin = DEBUG_USART_RX_PIN;
	GPIO_InitStructure.Alternate = DEBUG_USART_RX_AF;
	HAL_GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/*##-3- Configure the UART peripheral ######################################*/
	/* Put the USART peripheral in the Asynchronous mode (UART Mode) */
	/* UART1 configured as follow:
		- Word Length = 8 Bits
		- Stop Bit = One Stop bit
		- Parity = None
		- BaudRate = 115200 baud
		- Hardware flow control disabled (RTS and CTS signals) */
	UartHandle.Instance = DEBUG_USART;
	UartHandle.Init.BaudRate = DEBUG_USART_BAUDRATE;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
		
	if(HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}

	/*##-4- Configure the NVIC for UART ########################################*/
	/* NVIC for USART2 */
	HAL_NVIC_SetPriority(DEBUG_USART_IRQ, 0, 1);
	HAL_NVIC_EnableIRQ(DEBUG_USART_IRQ);
	/*配置串口接收中断 */
//    __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);
}

/*****************  发送一个字符 **********************/
void Usart_SendByte(UART_HandleTypeDef *huart, uint8_t ch)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(huart, (uint8_t *)&ch, 1, 1000);
}

/*****************  发送字符串 **********************/
void Usart_SendString(UART_HandleTypeDef *huart, uint8_t *str)
{
	unsigned int k=0;
	do 
	{
		HAL_UART_Transmit(huart, (uint8_t *)(str + k), 1, 1000);
		k++;
	} while(*(str+k) != '\0');
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord(UART_HandleTypeDef *huart, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	/* 发送高八位 */
	HAL_UART_Transmit(huart, (uint8_t *)&temp_h, 1, 1000);
	/* 发送低八位 */
	HAL_UART_Transmit(huart, (uint8_t *)&temp_l, 1, 1000);
}

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口DEBUG_USART */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);
	return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
	int ch;
	/* 等待串口输入数据 */
	while(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) == RESET);
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, 1000);	
	return (ch);
}

/*********************************************END OF FILE**********************/
