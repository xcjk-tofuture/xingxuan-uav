#include "key_proc.h"



u8 keyUp, keyDown, keyOld, keyValue;

osThreadId KeyTaskHandle;
void Key_Task_Proc(void const * argument)
{
	
  keyValue = Key_Scan();
	keyDown  =  keyValue &  (keyOld ^ keyValue);
	keyUp =   ~keyValue &  (keyOld ^ keyValue);
	keyOld = keyValue;
  /* USER CODE BEGIN Key_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Key_Task_Proc */
}


u8 Key_Scan()
{
	if


}