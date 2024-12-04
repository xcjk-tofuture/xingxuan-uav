#include "led_proc.h"




#include "motor_proc.h"

osThreadId RGBTaskHandle;

u8 ucLed;

extern u8 uavSafeFlag;
extern u8 AccCalFlag;
extern u8 GyroCalFlag;
extern u8 MagCalFlag;






void RGB_Task_Proc(void const * argument)           //RGB进程主程序
{
  /* USER CODE BEGIN RGB_Task_Proc */
  /* Infinite loop */
	ucLed = 0x00;
  for(;;)
  {
		
	 if(!(AccCalFlag || GyroCalFlag))
		RGB_Calibration(ucLed);
	 else
	 {
		if(uavSafeFlag == LOCKED)
			RGB_Normal(ucLed);
		else if (uavSafeFlag == UNLOCKED)
			RGB_Unlocked(ucLed);
		else if (uavSafeFlag == EMERGENCY)
			RGB_Emergency(ucLed);
		else if (uavSafeFlag == FLYING)
			RGB_Flying(ucLed);
	 }
 
		osDelay(100);
  }
  /* USER CODE END RGB_Task_Proc */
}

void RGB_Calibration(u8 ucled)
{
	static u8 ledTemp;
	ledTemp ^= 0x01;
	ucLed = ledTemp;
	RGB_Show_Proc(ucLed);
}

void RGB_Normal(u8 ucled)
{
	//if(ucLed > 0x04) ucLed = 0x01;

	static u8 ledTemp;
	ledTemp ^= 0x07;
	ucLed = ledTemp;
	osDelay(400);
	RGB_Show_Proc(ucLed);
}

void RGB_Emergency(u8 ucled)
{
	//if(ucLed > 0x04) ucLed = 0x01;
	static u8 ledTemp;
	ledTemp ^= 0x02;
	ucLed = ledTemp;
	RGB_Show_Proc(ucLed);
}

void RGB_Unlocked(u8 ucled)
{
	//if(ucLed > 0x04) ucLed = 0x01;
	static u8 ledTemp;
	ledTemp ^= 0x06;
	ucLed = ledTemp;
	osDelay(200);
	RGB_Show_Proc(ucLed);
}

void RGB_Flying(u8 ucled)
{
	//if(ucLed > 0x04) ucLed = 0x01;
	static u8 ledTemp;
	ledTemp ^= 0x04;
	ucLed = ledTemp;
	osDelay(400);
	RGB_Show_Proc(ucLed);
}


void RGB_Show_Proc(u8 ucled)                      //0b000 后三位赋1 灯亮
{
	
	HAL_GPIO_WritePin(GPIOE, RGB_R_Pin, (ucled & 0x04) == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, RGB_G_Pin, (ucled & 0x02) == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, RGB_B_Pin, (ucled & 0x01) == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

};