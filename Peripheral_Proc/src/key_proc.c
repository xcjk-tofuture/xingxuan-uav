#include "key_proc.h"

osThreadId KeyTaskHandle;
void Key_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Key_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Key_Task_Proc */
}