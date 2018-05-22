#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32l0xx.h"
#include <stdio.h>
//#define DEBUG
//#define UART_PROG
extern void Error_Handler(void);
extern UART_HandleTypeDef UartHandle;

//Òý½Å¶¨Òå
/*******************************************************/
#define DEBUG_USART                             USART2
#define DEBUG_USART_CLK_ENABLE()       	        __USART2_CLK_ENABLE();
#define DEBUG_USART_BAUDRATE                    2400

#define DEBUG_USART_FORCE_RESET()            		__HAL_RCC_USART2_FORCE_RESET()
#define DEBUG_USART_RELEASE_RESET()           	__HAL_RCC_USART2_RELEASE_RESET()

#define RCC_PERIPHCLK_UARTx               			RCC_PERIPHCLK_USART2
#define RCC_UARTxCLKSOURCE_SYSCLK         			RCC_USART2CLKSOURCE_SYSCLK

#define DEBUG_USART_RX_GPIO_PORT                GPIOA
#define DEBUG_USART_RX_GPIO_CLK_ENABLE()				__HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_RX_PIN                      GPIO_PIN_3
#define DEBUG_USART_RX_AF                       GPIO_AF4_USART2

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()    		__HAL_RCC_GPIOA_CLK_ENABLE()
#define DEBUG_USART_TX_PIN                      GPIO_PIN_2
#define DEBUG_USART_TX_AF                       GPIO_AF4_USART2

#define DEBUG_USART_IRQHandler                  USART2_IRQHandler
#define DEBUG_USART_IRQ                 	    	USART2_IRQn
/************************************************************/

void Debug_USART_Config(void);
void Usart_SendByte( UART_HandleTypeDef *huart, uint8_t ch);
void Usart_SendString( UART_HandleTypeDef *huart, uint8_t *str);
void Usart_SendHalfWord( UART_HandleTypeDef *huart, uint16_t ch);
void Scanf_Function(void);
#endif /* __DEBUG_USART_H */
