#include "sbus_proc.h"


extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart3;
extern u8 SbusRxBuf[40];
u16 SbusChannels[16];
osThreadId SbusUart6TaskHandle;

SBUS_CH_CAL_Struct CAL_SBUS_CH;
SBUS_CH_Struct SBUS_CH;



void Sbus_Uart6_Task_Proc(void const * argument)
{
	for(;;)
	{
//	TIM1->CCR1 = Sbus_To_Range(SBUS_CH.CH3, 1000, 2000);
//	TIM1->CCR2 = Sbus_To_Range(SBUS_CH.CH3, 1000, 2000);
//	TIM1->CCR3 = Sbus_To_Range(SBUS_CH.CH3, 1000, 2000);
//	TIM1->CCR4 = Sbus_To_Range(SBUS_CH.CH3, 1000, 2000);
		CAL_SBUS_CH.CAL_CH1 = Sbus_To_Range(SBUS_CH.CH1, 1000, 2000);
		CAL_SBUS_CH.CAL_CH2 = Sbus_To_Range(SBUS_CH.CH2, 1000, 2000);
		CAL_SBUS_CH.CAL_CH3 = Sbus_To_Range(SBUS_CH.CH3, 1000, 2000);
		CAL_SBUS_CH.CAL_CH4 = Sbus_To_Range(SBUS_CH.CH4, 1000, 2000);	
		
		CAL_SBUS_CH.CAL_CH5 = Sbus_To_Range(SBUS_CH.CH5, 1000, 2000);
		CAL_SBUS_CH.CAL_CH6 = Sbus_To_Range(SBUS_CH.CH6, 1000, 2000);
		CAL_SBUS_CH.CAL_CH7 = Sbus_To_Range(SBUS_CH.CH7, 1000, 2000);
		CAL_SBUS_CH.CAL_CH8 = Sbus_To_Range(SBUS_CH.CH8, 1000, 2000);	
		CAL_SBUS_CH.Connect_State = SBUS_CH.Connect_State;
		osDelay(200);
//  TIM1->CCR1 = 1500;	
//	TIM1->CCR1 = 1100;	
//	TIM1->CCR3 = 1200;
//	TIM1->CCR4 = 1100;	
//	  printf("channel_1: %f \r\n", Sbus_To_Range(SBUS_CH.CH1, 1000, 2000));
//		printf("channel_2: %f \r\n", Sbus_To_Range(SBUS_CH.CH2, 1000, 2000));
//		printf("channel_3: %f \r\n", Sbus_To_Range(SBUS_CH.CH3, 1000, 2000));
//		printf("channel_4: %f \r\n", Sbus_To_Range(SBUS_CH.CH4, 1000, 2000));
//	printf("channel_2: %d \r\n", SBUS_CH.CH2);
//	printf("channel_3: %d \r\n", SBUS_CH.CH3);
//	printf("channel_4: %d \r\n", SBUS_CH.CH4);
//	printf("channel_5:%d\r\n", SBUS_CH.CH5);
//	printf("channel_6:%d\r\n", SBUS_CH.CH6);		
//	printf("channel_7:%d\r\n", SBUS_CH.CH7);
//	printf("channel_8:%d\r\n", SBUS_CH.CH8);
//	printf("channel_9:%d\r\n", SBUS_CH.CH9);
//	printf("channel_10:%d\r\n", SBUS_CH.CH10);		
//	printf("channel_11:%d\r\n", SBUS_CH.CH11);
//	printf("channel_12:%d\r\n", SBUS_CH.CH12);
//	printf("channel_13:%d\r\n", SBUS_CH.CH13);
//	printf("channel_14:%d\r\n", SBUS_CH.CH14);		
//	printf("channel_15:%d\r\n", SBUS_CH.CH15);
//	printf("channel_16:%d\r\n", SBUS_CH.CH16);		
//	printf("ConnectState: %d\r\n", SBUS_CH.Connect_State);
	 
	}
}


void Sbus_Uart6_IDLE_Proc(uint16_t Size)
{
	
	if(Size == 25 )          //效验
	{
		
		Sbus_Channels_Proc();
		if(SbusRxBuf[23] == 0)
		{
				SBUS_CH.Connect_State = 1; //遥控器打开
		}
		else
		{
				SBUS_CH.Connect_State = 0; //遥控器关闭
		}

	}

}

void Sbus_Channels_Proc(void)          //解析sbus函数
{
    SbusChannels[0]  = ((SbusRxBuf[1]|SbusRxBuf[2]<<8)           & 0x07FF);
    SbusChannels[1]  = ((SbusRxBuf[2]>>3 |SbusRxBuf[3]<<5)                 & 0x07FF);
    SbusChannels[2]  = ((SbusRxBuf[3]>>6 |SbusRxBuf[4]<<2 |SbusRxBuf[5]<<10)  & 0x07FF);
    SbusChannels[3]  = ((SbusRxBuf[5]>>1 |SbusRxBuf[6]<<7)                 & 0x07FF);
    SbusChannels[4]  = ((SbusRxBuf[6]>>4 |SbusRxBuf[7]<<4)                 & 0x07FF);
    SbusChannels[5]  = ((SbusRxBuf[7]>>7 |SbusRxBuf[8]<<1 |SbusRxBuf[9]<<9)   & 0x07FF);
    SbusChannels[6]  = ((SbusRxBuf[9]>>2 |SbusRxBuf[10]<<6)                & 0x07FF);
    SbusChannels[7]  = ((SbusRxBuf[10]>>5|SbusRxBuf[11]<<3)                & 0x07FF);
    SbusChannels[8]  = ((SbusRxBuf[12]   |SbusRxBuf[13]<<8)                & 0x07FF);
    SbusChannels[9]  = ((SbusRxBuf[13]>>3|SbusRxBuf[14]<<5)                & 0x07FF);
    SbusChannels[10] = ((SbusRxBuf[14]>>6|SbusRxBuf[15]<<2|SbusRxBuf[16]<<10) & 0x07FF);
    SbusChannels[11] = ((SbusRxBuf[16]>>1|SbusRxBuf[17]<<7)                & 0x07FF);
    SbusChannels[12] = ((SbusRxBuf[17]>>4|SbusRxBuf[18]<<4)                & 0x07FF);
    SbusChannels[13] = ((SbusRxBuf[18]>>7|SbusRxBuf[19]<<1|SbusRxBuf[20]<<9)  & 0x07FF);
    SbusChannels[14] = ((SbusRxBuf[20]>>2|SbusRxBuf[21]<<6)                & 0x07FF);
    SbusChannels[15] = ((SbusRxBuf[21]>>5|SbusRxBuf[22]<<3)                & 0x07FF);
	
		SBUS_CH.CH1 = SbusChannels[0];
		SBUS_CH.CH2 = SbusChannels[1];
		SBUS_CH.CH3 = SbusChannels[2];
		SBUS_CH.CH4 = SbusChannels[3];
		SBUS_CH.CH5 = SbusChannels[4];
		SBUS_CH.CH6 = SbusChannels[5];
		SBUS_CH.CH7 = SbusChannels[6];
		SBUS_CH.CH8 = SbusChannels[7];
		SBUS_CH.CH9 = SbusChannels[8];
		SBUS_CH.CH10 = SbusChannels[9];
		SBUS_CH.CH11 = SbusChannels[10];
		SBUS_CH.CH12 = SbusChannels[11];
		SBUS_CH.CH13 = SbusChannels[12];
		SBUS_CH.CH14 = SbusChannels[13];
		SBUS_CH.CH15 = SbusChannels[14];
		SBUS_CH.CH16 = SbusChannels[15];
}

 
float Sbus_To_Range(u16 sbus_value, float p_min, float p_max)
{
    float p;
    p = p_min + (float)(sbus_value - SBUS_RANGE_MIN) * (p_max-p_min)/(float)(SBUS_RANGE_MAX - SBUS_RANGE_MIN);  
    if (p > p_max) p = p_max;
    if (p < p_min) p = p_min;
    return p;
}