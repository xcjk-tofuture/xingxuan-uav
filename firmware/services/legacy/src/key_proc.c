#include "sensor_port.h"
#include "display_service.h"
#include "key_proc.h"

#include "sbus_proc.h"



u8 keyUp, keyDown, keyOld, keyValue;

osThreadId KeyTaskHandle;






void Key_Task_Proc(void const * argument)
{
	
	u32 keyCount=0;
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
				if(sbus_calibration_active())
				{
					uav_display_request_page(1);
					sbus_request_calibration(1);
				}
					
				if(!sbus_calibration_active())
				{
					uav_display_request_page(19);
					sbus_request_calibration(0);  //校准遥控器
				}
					
				keyCount = 0;
			}
			
			if(keyDown == 1)
			{
				uav_display_next_page();
					
			}
    osDelay(5);
  }
  /* USER CODE END Key_Task_Proc */
}


u8 Key_Scan(void) {uint8_t key;uav_device_key_scan(&key);return key;}
