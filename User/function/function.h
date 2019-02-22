/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FUNCTION_H
#define __FUNCTION_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "./usart/bsp_debug_usart.h"
#include "./spi/bsp_spi.h"
#include "./adxl362/adxl362.h"
#include "./wwdg/bsp_wwdg.h"
#include "./pvd/bsp_pvd.h"

/* Exported types ------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RXBUFFERSIZE  		12					// Size of Reception buffer
#define EEPROM_START_ADDR   0x08080000   /* Start @ of user eeprom area */
#define TX              	0       	// cc1101发送模式
#define RX              	1       	// cc1101接收模式
#define IDLE          		2       	// cc1101空闲模式
#define SEND_SLENGTH     	19				// 发送数据包长度
#define SEND_LLENGTH     	61				// 发送数据包长度
#define SEND_PACKAGE_NUM	3					// 发生数据包数
#define RECV_LENGTH   		18				// 接收数据包长度
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void MCU_Initial(void);
void RF_Initial(uint8_t addr, uint16_t sync, uint8_t mode);
void System_Initial(void);
void RF_SendPacket(uint8_t index);
uint8_t RF_RecvHandler(void);
void DATAEEPROM_Program(uint32_t Address, uint32_t Data);
uint32_t DATAEEPROM_Read(uint32_t Address);
void HAL_SysTick_Decrement(void);

#endif /* __FUNCTION_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
