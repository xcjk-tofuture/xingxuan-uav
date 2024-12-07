#include "pc_proc.h"


osThreadId PCTaskHandle;
char InfoBuffer[1000];

u8 DataSend [100];


extern u8 Bmi088Init_Flag;    //bmi088初始化
extern u8 AK8975Flag;    //AK8975初始化
extern u8 SPL06Flag;    //AK8975初始化
extern _imuData_all imudata_all;

extern _ahrs_data attitude_t;
void PC_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN PC_Task_Proc */
  /* Infinite loop */
	osDelay(3000);
  for(;;)
  {
		//printf("%0.1f  %0.1f  %0.1f \r\n", imudata_all.mag.x * 0.3 * 10 , imudata_all.mag.y * 0.3 * 10, imudata_all.mag.z * 0.3 * 10 );
//	  ANODT_SendState(attitude_t.roll * 100, attitude_t.pitch * 100, attitude_t.yaw * 100, 1);
//		ANODT_SendRawAccGyro(imudata_all.acc.x * 100, imudata_all.acc.y * 100, imudata_all.acc.z * 100, imudata_all.gyro.roll * 100, imudata_all.gyro.pitch * 100, imudata_all.gyro.yaw * 100, !Bmi088Init_Flag);
//		ANODT_SendRawMagTempAlt(imudata_all.mag.x, imudata_all.mag.y, imudata_all.mag.z, imudata_all.Pressure , imudata_all.f_temperature * 100, !SPL06Flag, !AK8975Flag);
		// ANODT_SendQuaternion()
//		memset(InfoBuffer,0,1000);	 
//		vTaskList(InfoBuffer);
//		printf("\r\n任务名称 运行状态 优先级 剩余堆栈 任务序号\r\n");
//		printf("%s\r\n", InfoBuffer);
//		printf("B : 阻塞, R : 就绪, D : 删除, S : 暂停\r\n"); 
 
    osDelay(5);
  }
  /* USER CODE END PC_Task_Proc */
}


void ANODT_SendState(s16 roll, s16 pitch, s16 yaw, s8 FUSION_STA)      //向匿名上位机发送姿态数据
{
 
  u8 cnt = 0;
	DataSend[cnt++] = 0xAA;
	DataSend[cnt++] = 0xFF;
	DataSend[cnt++] = 0x03;
	DataSend[cnt++] = 7;
	
  DataSend[cnt++] = BYTE0(roll);
	DataSend[cnt++] = BYTE1(roll);
  
	DataSend[cnt++] = BYTE0(pitch);
  DataSend[cnt++] = BYTE1(pitch);
  
	DataSend[cnt++] = BYTE0(yaw);
  DataSend[cnt++] = BYTE1(yaw);
	
	DataSend[cnt++] = FUSION_STA;
	
	u8 sc = 0;
	u8 ac = 0;
	
	for (u8 i = 0; i < DataSend[3] + 4; i++)
	{
	     sc += DataSend[i];
		   ac += sc;
	}
	
	DataSend[cnt++] = sc;
	DataSend[cnt++] = ac;

	HAL_UART_Transmit(&huart1,(u8 *)DataSend, cnt ,30);
	
}


void ANODT_SendRawAccGyro(s16 acc_x, s16 acc_y, s16 acc_z, s16 gyo_x, s16 gyo_y, s16 gyo_z, s8 state)      //向匿名上位机发送传感器原始数据 加速度（100） 陀螺仪（100）
{
 
  u8 cnt = 0;
	DataSend[cnt++] = 0xAA;
	DataSend[cnt++] = 0xFF;
	DataSend[cnt++] = 0x01;
	DataSend[cnt++] = 13;
	
  DataSend[cnt++] = BYTE0(acc_x);
	DataSend[cnt++] = BYTE1(acc_x);
  
	DataSend[cnt++] = BYTE0(acc_y);
  DataSend[cnt++] = BYTE1(acc_y);
  
	DataSend[cnt++] = BYTE0(acc_z);
  DataSend[cnt++] = BYTE1(acc_z);
	
	DataSend[cnt++] = BYTE0(gyo_x);
  DataSend[cnt++] = BYTE1(gyo_x);
	
	DataSend[cnt++] = BYTE0(gyo_y);
  DataSend[cnt++] = BYTE1(gyo_y);
	
	
	DataSend[cnt++] = BYTE0(gyo_z);
  DataSend[cnt++] = BYTE1(gyo_z);
	
	DataSend[cnt++] = state;
	
	
	u8 sc = 0;
	u8 ac = 0;
	
	for (u8 i = 0; i < DataSend[3] + 4; i++)
	{
	     sc += DataSend[i];
		   ac += sc;
	}
	
	DataSend[cnt++] = sc;
	DataSend[cnt++] = ac;

	HAL_UART_Transmit(&huart1,(u8 *)DataSend, cnt ,30);
	
}




void ANODT_SendRawMagTempAlt(s16 mag_x, s16 mag_y, s16 mag_z, s32 alt_bar, s16 tmp, s8 alt_state, s8 mag_state)      //向匿名上位机发送传感器原始数据 磁力计（1） 气压计（原始） 温度（10）
{
 
  u8 cnt = 0;
	DataSend[cnt++] = 0xAA;
	DataSend[cnt++] = 0xFF;
	DataSend[cnt++] = 0x02;
	DataSend[cnt++] = 14;
	
  DataSend[cnt++] = BYTE0(mag_x);
	DataSend[cnt++] = BYTE1(mag_x);
  
	DataSend[cnt++] = BYTE0(mag_y);
  DataSend[cnt++] = BYTE1(mag_y);
  
	DataSend[cnt++] = BYTE0(mag_z);
  DataSend[cnt++] = BYTE1(mag_z);
	
	DataSend[cnt++] = BYTE0(alt_bar);
  DataSend[cnt++] = BYTE1(alt_bar);
	DataSend[cnt++] = BYTE2(alt_bar);
  DataSend[cnt++] = BYTE3(alt_bar);
	
	DataSend[cnt++] = BYTE0(tmp);
  DataSend[cnt++] = BYTE1(tmp);
	
	
	DataSend[cnt++] = (alt_state);
  DataSend[cnt++] = (mag_state);
	
	
	
	u8 sc = 0;
	u8 ac = 0;
	
	for (u8 i = 0; i < DataSend[3] + 4; i++)
	{
	     sc += DataSend[i];
		   ac += sc;
	}
	
	DataSend[cnt++] = sc;
	DataSend[cnt++] = ac;

	HAL_UART_Transmit(&huart1,(u8 *)DataSend, cnt ,30);
	
}


void ANODT_SendQuaternion(s16 v0, s16 v1, s16 v2, s16 v3,  s8 state)      //向匿名上位机发送四元数 放大10000倍
{
 
  u8 cnt = 0;
	DataSend[cnt++] = 0xAA;
	DataSend[cnt++] = 0xFF;
	DataSend[cnt++] = 0x03;
	DataSend[cnt++] = 9;
	
  DataSend[cnt++] = BYTE0(v0);
	DataSend[cnt++] = BYTE1(v0);
  
	DataSend[cnt++] = BYTE0(v1);
  DataSend[cnt++] = BYTE1(v1);
  
	DataSend[cnt++] = BYTE0(v2);
  DataSend[cnt++] = BYTE1(v2);
	
	DataSend[cnt++] = BYTE0(v3);
  DataSend[cnt++] = BYTE1(v3);


  DataSend[cnt++] = (state);
	
	
	
	u8 sc = 0;
	u8 ac = 0;
	
	for (u8 i = 0; i < DataSend[3] + 4; i++)
	{
	     sc += DataSend[i];
		   ac += sc;
	}
	
	DataSend[cnt++] = sc;
	DataSend[cnt++] = ac;

	HAL_UART_Transmit(&huart1,(u8 *)DataSend, cnt ,30);
	
}