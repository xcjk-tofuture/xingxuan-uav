#include "flow_proc.h"







osThreadId FlowTaskHandle;

void Flow_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Flow_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Flow_Task_Proc */
}