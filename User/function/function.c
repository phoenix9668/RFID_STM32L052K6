/* Includes ------------------------------------------------------------------*/
#include "./function/function.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t CollectCnt;      				// 接收MMA7361L次数
uint8_t	Chip_Addr	= 0;					// cc1101地址
uint8_t	RSSI = 0;								// RSSI值
uint8_t SendBuffer[SEND_LENGTH] = {0};// 发送数据包
uint8_t RecvBuffer[RECV_LENGTH] = {0};// 接收数据包
float ADC_ConvertedValueLocal[MMA7361L_NOFCHANEL];// 用于保存转化计算后的电压值
/* Private function prototypes -----------------------------------------------*/
extern void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

/*===========================================================================
* 函数 :MCU_Initial() => 初始化CPU所有硬件																	*
* 说明 关于所有硬件的初始化操作，已经被建成C库,见bsp.c文件									*
============================================================================*/
void MCU_Initial(void)
{ 
    Debug_USART_Config();
    GPIO_Config();
    TIM_Config();
    SPI_Config();
		MMA7361L_Config();
}

/*===========================================================================
* 函数 :RF_Initial() => 初始化RF芯片																				*
* 输入 :mode, =0,接收模式, else,发送模式																		*
* 说明 :CC1101的操作,已经被建成C库,见CC1101.c文件,提供SPI和CSN操作					*
				即可调用其内部所有函数用户无需再关心CC1101的寄存器操作问题					*
============================================================================*/
void RF_Initial(uint8_t addr, uint16_t sync, uint8_t mode)
{
	CC1101Init(addr, sync);                       			// 初始化CC1101寄存器
	if(mode == RX)				{CC1101SetTRMode(RX_MODE);}		// 接收模式
	else if(mode == TX)		{CC1101SetTRMode(TX_MODE);}   // 发送模式
	else
	{
		CC1101SetIdle();																	// 空闲模式，以转到sleep状态
		CC1101WORInit();																	// 初始化电磁波激活功能
		CC1101SetWORMode();
	}
}

/*===========================================================================
* 函数: System_Initial() => 初始化系统所有外设                              *
============================================================================*/
void System_Initial(void)
{
    MCU_Initial();      // 初始化CPU所有硬件
    RF_Initial(0x5, 0xD391, IDLE);     // 初始化无线芯片，空闲模式       
}

/*===========================================================================
* 函数 :RF_RecvHandler() => 无线数据接收处理                               * 
============================================================================*/
uint8_t RF_RecvHandler(void)
{
	uint8_t i=0, length=0;
	int16_t rssi_dBm;
	
	if(CC1101_IRQ_READ() == 0)         // 检测无线模块是否产生接收中断 
		{
			printf("interrupt occur\n");
			while (CC1101_IRQ_READ() == 0);
			for (i=0; i<RECV_LENGTH; i++)   { RecvBuffer[i] = 0; } // clear array
			
			// 读取接收到的数据长度和数据内容
			length = CC1101RecPacket(RecvBuffer, &Chip_Addr, &RSSI);
			// 打印数据
			rssi_dBm = CC1101CalcRSSI_dBm(RSSI);
			printf("RSSI = %ddBm, length = %d, address = %d\n",rssi_dBm,length,Chip_Addr);
			for(i=0; i<RECV_LENGTH; i++)
			{	
				printf("%x ",RecvBuffer[i]);
			}
			if(length == 0)
				{
					printf("receive error or Address Filtering fail\n");
					return 1;
				}
			else
				{
					if(RecvBuffer[0] == 0xAB && RecvBuffer[1] == 0xCD)
					{
						if(RecvBuffer[4] == 0xB0 && RecvBuffer[5] == 0xB0)
						{return 4;}
						else if(RecvBuffer[4] == 0xB3 && RecvBuffer[5] == 0xB3)
						{return 5;}
						else if(RecvBuffer[4] == 0xB4 && RecvBuffer[5] == 0xB4)
						{return 6;}
						else
						{	printf("receive function order error\r\n");
							return 3;}
					}
					else
					{	printf("receive package beginning error\r\n");
						return 2;}
				}
		}
	else	{return 0;}
}

/*===========================================================================
* 函数 : RF_SendPacket() => 无线发送数据函数                            *
* 输入 :Sendbuffer指向待发送的数据，length发送数据字节数                     *
* 输出 :0,发送失败;1,发送成功                                       *
============================================================================*/
void RF_SendPacket(uint8_t index)
{
	uint8_t i=0;	
		
	if(index == 4)
	{
		SendBuffer[0] = 0xAB;
		SendBuffer[1] = 0xCD;
		SendBuffer[2] = 0xB0;
		SendBuffer[3] = 0xB0;
		SendBuffer[4] = RecvBuffer[6];
		SendBuffer[5] = RecvBuffer[7];
		SendBuffer[6] = RecvBuffer[8];
		SendBuffer[7] = RecvBuffer[9];
		SendBuffer[14] = RSSI;
		for(i=0; i<SEND_PACKAGE_NUM; i++)
		{
			CC1101SendPacket(SendBuffer, SEND_LENGTH, ADDRESS_CHECK);    // 发送数据
			Delay(0xFFFF);									// 计算得到平均27ms发送一次数据
//		Delay(0xFFFFF);									// 计算得到平均130ms发送一次数据
		}
//		printf("send over\r\n");
	}
	else if(index == 5)
	{
		SendBuffer[0] = 0xAB;
		SendBuffer[1] = 0xCD;
		SendBuffer[2] = 0xC3;
		SendBuffer[3] = 0x01;
		SendBuffer[4] = RecvBuffer[6];
		SendBuffer[5] = RecvBuffer[7];
		SendBuffer[6] = RecvBuffer[8];
		SendBuffer[7] = RecvBuffer[9];
		SendBuffer[14] = RSSI;
		for(i=0; i<SEND_PACKAGE_NUM; i++)
		{
			CC1101SendPacket(SendBuffer, SEND_LENGTH, ADDRESS_CHECK);    // 发送数据
			Delay(0xFFFF);									// 计算得到平均27ms发送一次数据
//		Delay(0xFFFFF);									// 计算得到平均130ms发送一次数据
		}
	}
	else if(index == 6)
	{
		SendBuffer[0] = 0xAB;
		SendBuffer[1] = 0xCD;
		SendBuffer[2] = 0xB4;
		SendBuffer[3] = 0xB4;
		SendBuffer[4] = RecvBuffer[6];
		SendBuffer[5] = RecvBuffer[7];
		SendBuffer[6] = RecvBuffer[8];
		SendBuffer[7] = RecvBuffer[9];
		SendBuffer[14] = RSSI;
		for(i=0; i<SEND_PACKAGE_NUM; i++)
		{
			CC1101SendPacket(SendBuffer, SEND_LENGTH, ADDRESS_CHECK);    // 发送数据
			Delay(0xFFFF);									// 计算得到平均27ms发送一次数据
//		Delay(0xFFFFF);									// 计算得到平均130ms发送一次数据
		}
	}	
	else if(index == 1)
	{
		SendBuffer[0] = 0xAB;
		SendBuffer[1] = 0xCD;
		SendBuffer[2] = 0x01;
		SendBuffer[3] = 0x01;
		SendBuffer[4] = RecvBuffer[6];
		SendBuffer[5] = RecvBuffer[7];
		SendBuffer[6] = RecvBuffer[8];
		SendBuffer[7] = RecvBuffer[9];
		SendBuffer[14] = 0x01;
		for(i=0; i<SEND_PACKAGE_NUM; i++)
		{
			CC1101SendPacket(SendBuffer, SEND_LENGTH, ADDRESS_CHECK);    // 发送数据
			Delay(0xFFFF);									// 计算得到平均27ms发送一次数据
//		Delay(0xFFFFF);									// 计算得到平均130ms发送一次数据
		}
	}
	else if(index == 2)
	{
		SendBuffer[0] = 0xAB;
		SendBuffer[1] = 0xCD;
		SendBuffer[2] = 0x02;
		SendBuffer[3] = 0x02;
		SendBuffer[4] = RecvBuffer[6];
		SendBuffer[5] = RecvBuffer[7];
		SendBuffer[6] = RecvBuffer[8];
		SendBuffer[7] = RecvBuffer[9];
		SendBuffer[14] = 0x02;
		for(i=0; i<SEND_PACKAGE_NUM; i++)
		{
			CC1101SendPacket(SendBuffer, SEND_LENGTH, ADDRESS_CHECK);    // 发送数据
			Delay(0xFFFF);									// 计算得到平均27ms发送一次数据
//		Delay(0xFFFFF);									// 计算得到平均130ms发送一次数据
		}
	}
	else if(index == 3)
	{
		SendBuffer[0] = 0xAB;
		SendBuffer[1] = 0xCD;
		SendBuffer[2] = 0x03;
		SendBuffer[3] = 0x03;
		SendBuffer[4] = RecvBuffer[6];
		SendBuffer[5] = RecvBuffer[7];
		SendBuffer[6] = RecvBuffer[8];
		SendBuffer[7] = RecvBuffer[9];
		SendBuffer[14] = 0x03;
		for(i=0; i<SEND_PACKAGE_NUM; i++)
		{
			CC1101SendPacket(SendBuffer, SEND_LENGTH, ADDRESS_CHECK);    // 发送数据
			Delay(0xFFFF);									// 计算得到平均27ms发送一次数据
//		Delay(0xFFFFF);									// 计算得到平均130ms发送一次数据
		}
	}

//	Usart_SendString(&UartHandle, (uint8_t *)"Transmit OK\r\n");
	CC1101SetIdle();																	// 空闲模式，以转到sleep状态
	CC1101WORInit();																	// 初始化电磁波激活功能
	CC1101SetWORMode();
}

/*===========================================================================
* 函数 :MMA7361L_ReadHandler() => 读MMA731L中断函数                         * 
============================================================================*/
void MMA7361L_ReadHandler(void)
{
	MMA7361L_GS_1G5();
	MMA7361L_SL_OFF();
	Delay(0x3FFF);
	SendBuffer[8] = (uint8_t)(0xFF & ADC_ConvertedValue[0]);
	SendBuffer[9] = (uint8_t)(0x0F & (ADC_ConvertedValue[0]>>8));
	SendBuffer[10] = (uint8_t)(0xFF & ADC_ConvertedValue[1]);
	SendBuffer[11] = (uint8_t)(0xFF & (ADC_ConvertedValue[1]>>8));
	SendBuffer[12] = (uint8_t)(0xFF & ADC_ConvertedValue[2]);
	SendBuffer[13] = (uint8_t)(0xFF & (ADC_ConvertedValue[2]>>8));
  MMA7361L_SL_ON();
}

/*===========================================================================
* 函数 :MMA7361L_display() => 打印MMA7361L的数据                              * 
============================================================================*/
//void MMA7361L_display(void)
//{
//	Delay(0xffffee);
//	if(temp2 < 10)
//	{
//		MMA7361L_GS_1G5();
//		MMA7361L_SL_OFF();
//		Delay(0xFF);
//		ADC_ConvertedValueLocal[0] =(float)((uint16_t)ADC_ConvertedValue[0]*3.3/4096); 
//		ADC_ConvertedValueLocal[1] =(float)((uint16_t)ADC_ConvertedValue[1]*3.3/4096);
//		ADC_ConvertedValueLocal[2] =(float)((uint16_t)ADC_ConvertedValue[2]*3.3/4096);  
//    
//		printf("The current AD1 value = 0x%08X \r\n", ADC_ConvertedValue[0]); 
//		printf("The current AD2 value = 0x%08X \r\n", ADC_ConvertedValue[1]);
//		printf("The current AD3 value = 0x%08X \r\n", ADC_ConvertedValue[2]);   
//    
//		printf("The current ADC1 value = %f V \r\n", ADC_ConvertedValueLocal[0]); 
//		printf("The current ADC2 value = %f V \r\n", ADC_ConvertedValueLocal[1]);
//		printf("The current ADC2 value = %f V \r\n", ADC_ConvertedValueLocal[2]);
//		temp2++;
//	}
//	else if(temp2 < 20)
//	{
//		MMA7361L_GS_6G();
//		MMA7361L_SL_OFF();
//		Delay(0xFF);
//		ADC_ConvertedValueLocal[0] =(float)((uint16_t)ADC_ConvertedValue[0]*3.3/4096); 
//		ADC_ConvertedValueLocal[1] =(float)((uint16_t)ADC_ConvertedValue[1]*3.3/4096);
//		ADC_ConvertedValueLocal[2] =(float)((uint16_t)ADC_ConvertedValue[2]*3.3/4096);  
//    
//		printf("The current AD1 value = 0x%08X \r\n", ADC_ConvertedValue[0]); 
//		printf("The current AD2 value = 0x%08X \r\n", ADC_ConvertedValue[1]);
//		printf("The current AD3 value = 0x%08X \r\n", ADC_ConvertedValue[2]);   
//    
//		printf("The current ADC1 value = %f V \r\n", ADC_ConvertedValueLocal[0]); 
//		printf("The current ADC2 value = %f V \r\n", ADC_ConvertedValueLocal[1]);
//		printf("The current ADC2 value = %f V \r\n", ADC_ConvertedValueLocal[2]);
//		temp2++;
//	}
//	else if(temp2 < 30)
//	{
//		MMA7361L_SL_ON();
//		Delay(0xFF);
//		ADC_ConvertedValueLocal[0] =(float)((uint16_t)ADC_ConvertedValue[0]*3.3/4096); 
//		ADC_ConvertedValueLocal[1] =(float)((uint16_t)ADC_ConvertedValue[1]*3.3/4096);
//		ADC_ConvertedValueLocal[2] =(float)((uint16_t)ADC_ConvertedValue[2]*3.3/4096);  
//    
//		printf("The current AD1 value = 0x%08X \r\n", ADC_ConvertedValue[0]); 
//		printf("The current AD2 value = 0x%08X \r\n", ADC_ConvertedValue[1]);
//		printf("The current AD3 value = 0x%08X \r\n", ADC_ConvertedValue[2]);   
//    
//		printf("The current ADC1 value = %f V \r\n", ADC_ConvertedValueLocal[0]); 
//		printf("The current ADC2 value = %f V \r\n", ADC_ConvertedValueLocal[1]);
//		printf("The current ADC2 value = %f V \r\n", ADC_ConvertedValueLocal[2]);
//		temp2++;
//	}
//	else
//	{
//		temp2 = 0;
//	}
//}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
