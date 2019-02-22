/* Includes ------------------------------------------------------------------*/
#include "./function/function.h"

/* Private typedef -----------------------------------------------------------*/
/* UART handler declared in "bsp_debug_usart.c" file */
extern UART_HandleTypeDef UartHandle;
extern __IO uint8_t step_stage;
extern __IO uint32_t step1;
extern __IO uint32_t step2;
extern __IO uint32_t step3;
extern __IO uint32_t step4;
extern __IO uint32_t step5;
extern __IO uint32_t step6;
extern __IO uint32_t step7;
extern __IO uint32_t step8;
extern __IO uint32_t step9;
extern __IO uint32_t step10;
extern __IO uint32_t step11;
extern __IO uint32_t step12;
extern __IO uint8_t battery_low;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t	Chip_Addr	= 0;					// cc1101��ַ
uint8_t	RSSI = 0;								// RSSIֵ
uint8_t addr_eeprom;
uint16_t sync_eeprom;
uint32_t rfid_eeprom;
uint8_t SendBuffer[SEND_LLENGTH] = {0};// �������ݰ�
uint8_t RecvBuffer[RECV_LENGTH] = {0};// �������ݰ�
uint8_t aRxBuffer[RXBUFFERSIZE];			// Buffer used for reception
/* Private function prototypes -----------------------------------------------*/
extern void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

/*===========================================================================
* ���� :MCU_Initial() => ��ʼ��CPU����Ӳ��																	*
* ˵�� ��������Ӳ���ĳ�ʼ���������Ѿ�������C��,��bsp.c�ļ�									*
============================================================================*/
void MCU_Initial(void)
{
    GPIO_Config();
		#ifdef DEBUG
		if(ADC_IN1_READ() == 1)
		{
			Debug_USART_Config();
		}
		#endif
		Delay(0x100);
		SPI_Config();
		Delay(0x100);
		ADXL362_Init();
		Delay(0x100);
		INT_GPIO_Config();
		Delay(0x100);
		PVD_Config();
}

/*===========================================================================
* ���� :RF_Initial() => ��ʼ��RFоƬ																					*
* ���� :mode, =0,����ģʽ, else,����ģʽ																			*
* ˵�� :CC1101�Ĳ���,�Ѿ�������C��,��CC1101.c�ļ�,�ṩSPI��CSN����						*
				���ɵ������ڲ����к����û������ٹ���CC1101�ļĴ�����������						*
============================================================================*/
void RF_Initial(uint8_t addr, uint16_t sync, uint8_t mode)
{
	CC1101Init(addr, sync);                       			// ��ʼ��CC1101�Ĵ���
	if(mode == RX)				{CC1101SetTRMode(RX_MODE);}		// ����ģʽ
	else if(mode == TX)		{CC1101SetTRMode(TX_MODE);}   // ����ģʽ
	else
	{
		CC1101SetIdle();																	// ����ģʽ����ת��sleep״̬
		CC1101WORInit();																	// ��ʼ����Ų������
		CC1101SetWORMode();
	}
}

/*===========================================================================
* ����: System_Initial() => ��ʼ��ϵͳ��������                             	*
============================================================================*/
void System_Initial(void)
{
		/*##-1- initial all peripheral ###########################*/
    MCU_Initial(); 
		/*##-2- initial CC1101 peripheral,configure it's address and sync code ###########################*/
		addr_eeprom = (uint8_t)(0xff & DATAEEPROM_Read(EEPROM_START_ADDR)>>16); 
		sync_eeprom = (uint16_t)(0xffff & DATAEEPROM_Read(EEPROM_START_ADDR)); 
		rfid_eeprom	= DATAEEPROM_Read(EEPROM_START_ADDR+4);
		#ifdef DEBUG
		if(ADC_IN1_READ() == 1)
		{
			printf("addr_eeprom = %x\n",addr_eeprom);
			printf("sync_eeprom = %x\n",sync_eeprom);
			printf("rfid_eeprom = %x\n",rfid_eeprom);
		}
		#endif
    RF_Initial(addr_eeprom, sync_eeprom, IDLE);     // ��ʼ������оƬ������ģʽ
		#ifdef UART_PROG
		/*##-3- Put UART peripheral in reception process ###########################*/
		if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
		{
			Error_Handler();
		}
		#endif
}

/*===========================================================================
* ���� :RF_RecvHandler() => �������ݽ��մ���                               	* 
============================================================================*/
uint8_t RF_RecvHandler(void)
{
	uint8_t i=0, length=0;
	int16_t rssi_dBm;
	uint32_t timeout;
	
	if(CC1101_IRQ_READ() == 0)         // �������ģ���Ƿ���������ж� 
		{
			HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
//			__disable_irq();
			#ifdef DEBUG
			if(ADC_IN1_READ() == 1)
			{
				printf("interrupt occur\n");
			}
			#endif
			timeout = Delay_TimeOut;
			while (CC1101_IRQ_READ() == 0 && timeout != 0)
			{
				timeout--;
			}
			for (i=0; i<RECV_LENGTH; i++)   { RecvBuffer[i] = 0; } // clear array
			length = CC1101RecPacket(RecvBuffer, &Chip_Addr, &RSSI);	// ��ȡ���յ������ݳ��Ⱥ���������
			
			#ifdef DEBUG
			if(ADC_IN1_READ() == 1)
			{
				rssi_dBm = CC1101CalcRSSI_dBm(RSSI);
				printf("RSSI = %ddBm, length = %d, address = %d\n",rssi_dBm,length,Chip_Addr);
				for(i=0; i<RECV_LENGTH; i++)
				{	
					printf("%x ",RecvBuffer[i]);
				}
			}
			#endif
			if(length == 0)
				{
					#ifdef DEBUG
					if(ADC_IN1_READ() == 1)
					{
						printf("receive error or Address Filtering fail\n");
					}
					#endif
					return 0x01;
				}
			else
				{
					if(RecvBuffer[0] == 0xAB && RecvBuffer[1] == 0xCD)
					{
						if(RecvBuffer[4] == 0xC0 && RecvBuffer[5] == 0xC0)
						{return 0x04;}
						else if(RecvBuffer[4] == 0xC2 && RecvBuffer[5] == 0xC2)
						{return 0x05;}
						else if(RecvBuffer[4] == 0xC3 && RecvBuffer[5] == 0xC3)
						{return 0x06;}
						else if(RecvBuffer[4] == 0xC4 && RecvBuffer[5] == 0xC4)
						{return 0x07;}
						else if(RecvBuffer[4] == 0xC5 && RecvBuffer[5] == 0xC5)
						{return 0x08;}
						else if(RecvBuffer[4] == 0xC6 && RecvBuffer[5] == 0xC6)
						{return 0x09;}
						else
						{	
							#ifdef DEBUG
							if(ADC_IN1_READ() == 1)
							{
								printf("receive function order error\r\n");
							}
							#endif
							return 0x03;}
					}
					else
					{
						#ifdef DEBUG
						if(ADC_IN1_READ() == 1)
						{
							printf("receive package beginning error\r\n");
						}
						#endif
						return 0x02;}
				}
		}
	else	{return 0x00;}
}

/*===========================================================================
* ���� : RF_SendPacket() => ���߷������ݺ���                            			*
* ���� :Sendbufferָ������͵����ݣ�length���������ֽ���                     		*
* ��� :0,����ʧ��;1,���ͳɹ�                                       					*
============================================================================*/
void RF_SendPacket(uint8_t index)
{
	uint8_t i=0;
	uint32_t data;
	uint32_t dataeeprom;// ��eeprom�ж�������
	
	LED_GREEN_OFF();
	
	switch(index)
	{
		case 0x01://receive error or Address Filtering fail
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];
			SendBuffer[4] = 0x01;
			SendBuffer[5] = 0x01;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			SendBuffer[10] = 0xE0;
			SendBuffer[18] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_SLENGTH, ADDRESS_CHECK);
			}
			break;
		case 0x02://receive package beginning error
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];		
			SendBuffer[4] = 0x02;
			SendBuffer[5] = 0x02;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			SendBuffer[10] = 0xE1;
			SendBuffer[18] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_SLENGTH, ADDRESS_CHECK);
			}
			break;
		case 0x03://receive function order error
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];		
			SendBuffer[4] = 0x03;
			SendBuffer[5] = 0x03;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			SendBuffer[10] = 0xE2;
			SendBuffer[18] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_SLENGTH, ADDRESS_CHECK);
			}
			break;
		case 0x04:
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];
			SendBuffer[4] = 0xD0;
			SendBuffer[5] = 0x01;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			SendBuffer[10] = (uint8_t)(0x000000FF & step1>>24);
			SendBuffer[11] = (uint8_t)(0x000000FF & step1>>16);
			SendBuffer[12] = (uint8_t)(0x000000FF & step1>>8);
			SendBuffer[13] = (uint8_t)(0x000000FF & step1);
			SendBuffer[14] = (uint8_t)(0x000000FF & step2>>24);
			SendBuffer[15] = (uint8_t)(0x000000FF & step2>>16);
			SendBuffer[16] = (uint8_t)(0x000000FF & step2>>8);
			SendBuffer[17] = (uint8_t)(0x000000FF & step2);
			SendBuffer[18] = (uint8_t)(0x000000FF & step3>>24);
			SendBuffer[19] = (uint8_t)(0x000000FF & step3>>16);
			SendBuffer[20] = (uint8_t)(0x000000FF & step3>>8);
			SendBuffer[21] = (uint8_t)(0x000000FF & step3);
			SendBuffer[22] = (uint8_t)(0x000000FF & step4>>24);
			SendBuffer[23] = (uint8_t)(0x000000FF & step4>>16);
			SendBuffer[24] = (uint8_t)(0x000000FF & step4>>8);
			SendBuffer[25] = (uint8_t)(0x000000FF & step4);
			SendBuffer[26] = (uint8_t)(0x000000FF & step5>>24);
			SendBuffer[27] = (uint8_t)(0x000000FF & step5>>16);
			SendBuffer[28] = (uint8_t)(0x000000FF & step5>>8);
			SendBuffer[29] = (uint8_t)(0x000000FF & step5);
			SendBuffer[30] = (uint8_t)(0x000000FF & step6>>24);
			SendBuffer[31] = (uint8_t)(0x000000FF & step6>>16);
			SendBuffer[32] = (uint8_t)(0x000000FF & step6>>8);
			SendBuffer[33] = (uint8_t)(0x000000FF & step6);
			SendBuffer[34] = (uint8_t)(0x000000FF & step7>>24);
			SendBuffer[35] = (uint8_t)(0x000000FF & step7>>16);
			SendBuffer[36] = (uint8_t)(0x000000FF & step7>>8);
			SendBuffer[37] = (uint8_t)(0x000000FF & step7);
			SendBuffer[38] = (uint8_t)(0x000000FF & step8>>24);
			SendBuffer[39] = (uint8_t)(0x000000FF & step8>>16);
			SendBuffer[40] = (uint8_t)(0x000000FF & step8>>8);
			SendBuffer[41] = (uint8_t)(0x000000FF & step8);
			SendBuffer[42] = (uint8_t)(0x000000FF & step9>>24);
			SendBuffer[43] = (uint8_t)(0x000000FF & step9>>16);
			SendBuffer[44] = (uint8_t)(0x000000FF & step9>>8);
			SendBuffer[45] = (uint8_t)(0x000000FF & step9);
			SendBuffer[46] = (uint8_t)(0x000000FF & step10>>24);
			SendBuffer[47] = (uint8_t)(0x000000FF & step10>>16);
			SendBuffer[48] = (uint8_t)(0x000000FF & step10>>8);
			SendBuffer[49] = (uint8_t)(0x000000FF & step10);
			SendBuffer[50] = (uint8_t)(0x000000FF & step11>>24);
			SendBuffer[51] = (uint8_t)(0x000000FF & step11>>16);
			SendBuffer[52] = (uint8_t)(0x000000FF & step11>>8);
			SendBuffer[53] = (uint8_t)(0x000000FF & step11);
			SendBuffer[54] = (uint8_t)(0x000000FF & step12>>24);
			SendBuffer[55] = (uint8_t)(0x000000FF & step12>>16);
			SendBuffer[56] = (uint8_t)(0x000000FF & step12>>8);
			SendBuffer[57] = (uint8_t)(0x000000FF & step12);
			SendBuffer[58] = step_stage;
			SendBuffer[59] = battery_low;
//		for(i=10; i<SEND_LENGTH; i++)
//		{
//			SendBuffer[i] = i-10;
//		}
//		printf("\r\n");
//		for(i=0; i<SEND_LENGTH; i++)
//		{
//			printf("%x ",SendBuffer[i]);
//		}
			SendBuffer[60] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_LLENGTH, ADDRESS_CHECK);
			}
			break;
		case 0x05:
			battery_low = 0x00;
			DATAEEPROM_Program(EEPROM_START_ADDR+112, battery_low);
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];		
			SendBuffer[4] = 0xD2;
			SendBuffer[5] = 0x01;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			battery_low = (uint8_t)(0x000000FF & DATAEEPROM_Read(EEPROM_START_ADDR+112));
			SendBuffer[10] = battery_low;
			SendBuffer[18] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_SLENGTH, ADDRESS_CHECK);
			}
			break;	
		case 0x06:
			ADXL362_ReInit(RecvBuffer[10], RecvBuffer[11], RecvBuffer[13], RecvBuffer[14], RecvBuffer[15], RecvBuffer[16], RecvBuffer[17]);
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];		
			SendBuffer[4] = 0xD3;
			SendBuffer[5] = 0x01;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			SendBuffer[10] = ADXL362RegisterRead(XL362_THRESH_ACT_H);
			SendBuffer[11] = ADXL362RegisterRead(XL362_THRESH_ACT_L);
			SendBuffer[12] = ADXL362RegisterRead(XL362_TIME_ACT);
			SendBuffer[13] = ADXL362RegisterRead(XL362_THRESH_INACT_H);
			SendBuffer[14] = ADXL362RegisterRead(XL362_THRESH_INACT_L);
			SendBuffer[15] = ADXL362RegisterRead(XL362_TIME_INACT_H);
			SendBuffer[16] = ADXL362RegisterRead(XL362_TIME_INACT_L);
			SendBuffer[17] = ADXL362RegisterRead(XL362_FILTER_CTL);
			SendBuffer[18] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_SLENGTH, ADDRESS_CHECK);
			}
			break;

		case 0x07:
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];		
			SendBuffer[4] = 0xD4;
			SendBuffer[5] = 0x01;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			SendBuffer[10] = (uint8_t)(0x000000FF & step1>>24);
			SendBuffer[11] = (uint8_t)(0x000000FF & step1>>16);
			SendBuffer[12] = (uint8_t)(0x000000FF & step1>>8);
			SendBuffer[13] = (uint8_t)(0x000000FF & step1);
			SendBuffer[14] = (uint8_t)(0x000000FF & step2>>24);
			SendBuffer[15] = (uint8_t)(0x000000FF & step2>>16);
			SendBuffer[16] = (uint8_t)(0x000000FF & step2>>8);
			SendBuffer[17] = (uint8_t)(0x000000FF & step2);
			SendBuffer[18] = (uint8_t)(0x000000FF & step3>>24);
			SendBuffer[19] = (uint8_t)(0x000000FF & step3>>16);
			SendBuffer[20] = (uint8_t)(0x000000FF & step3>>8);
			SendBuffer[21] = (uint8_t)(0x000000FF & step3);
			SendBuffer[22] = (uint8_t)(0x000000FF & step4>>24);
			SendBuffer[23] = (uint8_t)(0x000000FF & step4>>16);
			SendBuffer[24] = (uint8_t)(0x000000FF & step4>>8);
			SendBuffer[25] = (uint8_t)(0x000000FF & step4);
			SendBuffer[26] = (uint8_t)(0x000000FF & step5>>24);
			SendBuffer[27] = (uint8_t)(0x000000FF & step5>>16);
			SendBuffer[28] = (uint8_t)(0x000000FF & step5>>8);
			SendBuffer[29] = (uint8_t)(0x000000FF & step5);
			SendBuffer[30] = (uint8_t)(0x000000FF & step6>>24);
			SendBuffer[31] = (uint8_t)(0x000000FF & step6>>16);
			SendBuffer[32] = (uint8_t)(0x000000FF & step6>>8);
			SendBuffer[33] = (uint8_t)(0x000000FF & step6);
			SendBuffer[34] = (uint8_t)(0x000000FF & step7>>24);
			SendBuffer[35] = (uint8_t)(0x000000FF & step7>>16);
			SendBuffer[36] = (uint8_t)(0x000000FF & step7>>8);
			SendBuffer[37] = (uint8_t)(0x000000FF & step7);
			SendBuffer[38] = (uint8_t)(0x000000FF & step8>>24);
			SendBuffer[39] = (uint8_t)(0x000000FF & step8>>16);
			SendBuffer[40] = (uint8_t)(0x000000FF & step8>>8);
			SendBuffer[41] = (uint8_t)(0x000000FF & step8);
			SendBuffer[42] = (uint8_t)(0x000000FF & step9>>24);
			SendBuffer[43] = (uint8_t)(0x000000FF & step9>>16);
			SendBuffer[44] = (uint8_t)(0x000000FF & step9>>8);
			SendBuffer[45] = (uint8_t)(0x000000FF & step9);
			SendBuffer[46] = (uint8_t)(0x000000FF & step10>>24);
			SendBuffer[47] = (uint8_t)(0x000000FF & step10>>16);
			SendBuffer[48] = (uint8_t)(0x000000FF & step10>>8);
			SendBuffer[49] = (uint8_t)(0x000000FF & step10);
			SendBuffer[50] = (uint8_t)(0x000000FF & step11>>24);
			SendBuffer[51] = (uint8_t)(0x000000FF & step11>>16);
			SendBuffer[52] = (uint8_t)(0x000000FF & step11>>8);
			SendBuffer[53] = (uint8_t)(0x000000FF & step11);
			SendBuffer[54] = (uint8_t)(0x000000FF & step12>>24);
			SendBuffer[55] = (uint8_t)(0x000000FF & step12>>16);
			SendBuffer[56] = (uint8_t)(0x000000FF & step12>>8);
			SendBuffer[57] = (uint8_t)(0x000000FF & step12);
			SendBuffer[58] = step_stage;
			SendBuffer[59] = battery_low;
			SendBuffer[60] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_LLENGTH, ADDRESS_CHECK);
			}
			break;
		case 0x08:
			data = ((uint32_t)(0xFF000000 & RecvBuffer[10]<<24)+(uint32_t)(0x00FF0000 & RecvBuffer[11]<<16)+(uint32_t)(0x0000FF00 & RecvBuffer[12]<<8)+(uint32_t)(0x000000FF & RecvBuffer[13]));
			DATAEEPROM_Program(EEPROM_START_ADDR, data);
			data = ((uint32_t)(0xFF000000 & RecvBuffer[14]<<24)+(uint32_t)(0x00FF0000 & RecvBuffer[15]<<16)+(uint32_t)(0x0000FF00 & RecvBuffer[16]<<8)+(uint32_t)(0x000000FF & RecvBuffer[17]));
			DATAEEPROM_Program(EEPROM_START_ADDR+4, data);
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];		
			SendBuffer[4] = 0xD5;
			SendBuffer[5] = 0x01;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			dataeeprom = DATAEEPROM_Read(EEPROM_START_ADDR);
			SendBuffer[10] = (uint8_t)(0x000000FF & dataeeprom>>24);
			SendBuffer[11] = (uint8_t)(0x000000FF & dataeeprom>>16);
			SendBuffer[12] = (uint8_t)(0x000000FF & dataeeprom>>8);
			SendBuffer[13] = (uint8_t)(0x000000FF & dataeeprom);
			dataeeprom = DATAEEPROM_Read(EEPROM_START_ADDR+4);
			SendBuffer[14] = (uint8_t)(0x000000FF & dataeeprom>>24);
			SendBuffer[15] = (uint8_t)(0x000000FF & dataeeprom>>16);
			SendBuffer[16] = (uint8_t)(0x000000FF & dataeeprom>>8);
			SendBuffer[17] = (uint8_t)(0x000000FF & dataeeprom);	
			SendBuffer[18] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_SLENGTH, ADDRESS_CHECK);
			}
			break;
		case 0x09:
			step1 = 0;
			step2 = 0;
			step3 = 0;
			step4 = 0;
			step5 = 0;
			step6 = 0;
			step7 = 0;
			step8 = 0;
			step9 = 0;
			step10 = 0;
			step11 = 0;
			step12 = 0;
			step_stage = 0;
			SendBuffer[0] = 0xAB;
			SendBuffer[1] = 0xCD;
			SendBuffer[2] = RecvBuffer[2];
			SendBuffer[3] = RecvBuffer[3];		
			SendBuffer[4] = 0xD6;
			SendBuffer[5] = 0x01;
			SendBuffer[6] = RecvBuffer[6];
			SendBuffer[7] = RecvBuffer[7];
			SendBuffer[8] = RecvBuffer[8];
			SendBuffer[9] = RecvBuffer[9];
			SendBuffer[18] = RSSI;
			for(i=0; i<SEND_PACKAGE_NUM; i++)
			{
				Delay(0x500);
				CC1101SendPacket(SendBuffer, SEND_SLENGTH, ADDRESS_CHECK);
			}
			break;
		default : break;
	}

	for(i=0; i<SEND_LLENGTH; i++) // clear array
	{SendBuffer[i] = 0;}

	CC1101SetIdle();
	CC1101WORInit();
	CC1101SetWORMode();
//	__enable_irq();
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	LED_GREEN_ON();
}

/*===========================================================================
* ���� :DATAEEPROM_Program() => ��eeprom�������                         			* 
============================================================================*/
void DATAEEPROM_Program(uint32_t Address, uint32_t Data)
{
	/* Unlocks the data memory and FLASH_PECR register access *************/ 
	if(HAL_FLASHEx_DATAEEPROM_Unlock() != HAL_OK)
	{
    Error_Handler();
	}
	/* Clear FLASH error pending bits */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR | FLASH_FLAG_SIZERR |
																											FLASH_FLAG_OPTVERR | FLASH_FLAG_RDERR | 
																												FLASH_FLAG_FWWERR | FLASH_FLAG_NOTZEROERR);
	/*Erase a word in data memory *************/
	if (HAL_FLASHEx_DATAEEPROM_Erase(Address) != HAL_OK)
	{
		Error_Handler();
	}
	/*Enable DATA EEPROM fixed Time programming (2*Tprog) *************/
	HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram();
  /* Program word at a specified address *************/
	if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, Address, Data) != HAL_OK)
	{
		Error_Handler();
	}
	/*Disables DATA EEPROM fixed Time programming (2*Tprog) *************/
	HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram();

  /* Locks the Data memory and FLASH_PECR register access. (recommended
     to protect the DATA_EEPROM against possible unwanted operation) *********/
  HAL_FLASHEx_DATAEEPROM_Lock(); 

}

/*===========================================================================
* ���� :DATAEEPROM_Read() => ��eeprom��ȡ����                         				* 
============================================================================*/
uint32_t DATAEEPROM_Read(uint32_t Address)
{
	return *(__IO uint32_t*)Address;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
