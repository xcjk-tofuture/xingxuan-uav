#ifndef __FLOW_PROC_H__
#define __FLOW_PROC_H__


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "motor_proc.h"


typedef struct
{
	s16 xNowOffset;
	s16 xLastOffset;

	s16 yNowOffset;
	s16 yLastOffset;


	s16 zNowHeight;
	s16 zLastHeight;
	
	s16 delatTime;
	
	float xFlowVel;
	float yFlowVel;
	float zFlowVel;
	
	s8 flowFlag;
	s8 flowConf;
	
} _flow_data;


void Flow_Task_Proc(void const * argument);
void Flow_Data_Proc(u8 size);




#endif