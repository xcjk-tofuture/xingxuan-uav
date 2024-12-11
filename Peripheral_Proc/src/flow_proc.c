#include "flow_proc.h"


extern u8 uart2RX[200];
extern u8 uart2TX[200];


extern _uav_control_data uav_control_data;
_flow_data upixels_flow_T2_data;
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



void Flow_Data_Proc(u8 size)
{
	if(size == 14 && uart2RX[0] == 0XFE && uart2RX[1] == 0X0A)
	{
		upixels_flow_T2_data.zNowHeight = (uart2RX[8] | uart2RX[9] << 8);
		upixels_flow_T2_data.xNowOffset = (uart2RX[2] | uart2RX[3] << 8) * upixels_flow_T2_data.zNowHeight /10000.f;
		upixels_flow_T2_data.yNowOffset = (uart2RX[4] | uart2RX[5] << 8) * upixels_flow_T2_data.zNowHeight /10000.f;
		upixels_flow_T2_data.delatTime  = (uart2RX[6] | uart2RX[7] << 8);
		upixels_flow_T2_data.flowFlag = uart2RX[10];
		upixels_flow_T2_data.flowConf = uart2RX[11];
	
		upixels_flow_T2_data.xFlowVel = (upixels_flow_T2_data.xNowOffset - upixels_flow_T2_data.xLastOffset) / (float)(upixels_flow_T2_data.delatTime / 1000000.f);
		upixels_flow_T2_data.yFlowVel = (upixels_flow_T2_data.yNowOffset - upixels_flow_T2_data.yLastOffset) / (float)(upixels_flow_T2_data.delatTime / 1000000.f);
		upixels_flow_T2_data.zFlowVel = (upixels_flow_T2_data.zNowHeight - upixels_flow_T2_data.zLastHeight) / (float)(upixels_flow_T2_data.delatTime / 1000000.f);
		//uav_control_data.height = upixels_flow_T2_data.zNowHeight;
		
		upixels_flow_T2_data.xLastOffset = upixels_flow_T2_data.xNowOffset;
		upixels_flow_T2_data.yLastOffset = upixels_flow_T2_data.yNowOffset;
		upixels_flow_T2_data.zLastHeight = upixels_flow_T2_data.zNowHeight;
	}
}
	

