#include "key_proc.h"



u8 keyUp, keyDown, keyOld, keyValue;

osThreadId KeyTaskHandle;

extern void OLED_Clear();

extern u8 remoteCaliFlag;
extern u8 remoteCaliSaveFlashFlag;
extern u8 displayPage;
void Key_Task_Proc(void const * argument)
{
	
	u32 keyCount;
	u8 keyLongFlag = 0;
  /* USER CODE BEGIN Key_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
		
	keyValue = Key_Scan();
	keyDown  =  keyValue &  (keyOld ^ keyValue);
	keyUp =   ~keyValue &  (keyOld ^ keyValue);
	keyOld = keyValue;
		
		
			if(keyDown == 2)
				keyCount = 0;
			if(keyValue == 2)
				keyCount++;
			if(keyUp == 2 && keyCount >= 100)
			{ //长按逻辑处理
				if(remoteCaliFlag)
				{
					displayPage = 1;
					remoteCaliSaveFlashFlag = 1;
				}
					
				if(remoteCaliFlag == 0)
				{
					displayPage = 19;
					remoteCaliFlag = 1;  //校准遥控器
				}
					
				keyCount = 0;
			}
			
			if(keyDown == 1)
			{
				if(++displayPage > 3)
				{
					
					displayPage = 1;
				}
					
			}
    osDelay(5);
  }
  /* USER CODE END Key_Task_Proc */
}


u8 Key_Scan()
{
 if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0) == GPIO_PIN_RESET)
	 return 1;
 else if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1) == GPIO_PIN_RESET)
	 return 2;
 else
   return 0;	 

}