#include "motor_proc.h"


extern SBUS_CH_CAL_Struct CAL_SBUS_CH;
extern _ahrs_data ahrs_data;

osThreadId MotorTaskHandle;

_uav_control_data uav_control_data;



u8 uavSafeFlag = LOCKED;
u8 uavAutoTakeOffFlag = 0;
u32 unlockCount = 0;

void Motor_Task_Proc(void const * argument)
{
	TIM1->CCR1 = 1000;
	TIM1->CCR2 = 1000;
	TIM1->CCR3 = 1000;
	TIM1->CCR4 = 1000;
	UAV_Control_Init(&uav_control_data);
	osDelay(5000);
  for(;;)
  {
		if(CAL_SBUS_CH.Connect_State == 1)
    {
			switch (uavSafeFlag){
				case LOCKED:
					TIM1->CCR1 = 1000;
					TIM1->CCR2 = 1000;
					TIM1->CCR3 = 1000;
					TIM1->CCR4 = 1000;	
					break;
				case UNLOCKED:
					TIM1->CCR1 = 1050;
					TIM1->CCR2 = 1050;
					TIM1->CCR3 = 1050;
					TIM1->CCR4 = 1050;
					break;
				case FLYING:

					uav_control_data.rollSpeedOut = (s16)PID_Control(&uav_control_data.rollPid, &uav_control_data.rollData, 5, 0, (CAL_SBUS_CH.CAL_CH1 - 1500)  / 12.0f, ahrs_data.roll, 1000);
					uav_control_data.pitchSpeedOut = (s16)PID_Control(&uav_control_data.pitchPid, &uav_control_data.pitchData, 5, 0, (CAL_SBUS_CH.CAL_CH2 - 1500)  / 12.0f, ahrs_data.pitch, 1000);
					
				
					uav_control_data.rollOut = (s16)PID_Control(&uav_control_data.rollSpeedPid, &uav_control_data.rollSpeedData, 5, 0, uav_control_data.rollSpeedOut , ahrs_data.rollSpeed, 1000);
					uav_control_data.pitchOut = (s16)PID_Control(&uav_control_data.pitchSpeedPid, &uav_control_data.pitchSpeedData, 5, 0, uav_control_data.pitchSpeedOut , ahrs_data.pitchSpeed, 1000);
					
					uav_control_data.yawOut = (s16)PID_Control(&uav_control_data.yawPid, &uav_control_data.yawData, 5, 0, ((CAL_SBUS_CH.CAL_CH4 - 1500)  / 12.0f) * 0.01, ahrs_data.yawSpeed, 1000);
				
					if(CAL_SBUS_CH.CAL_CH3 > 1100)
					{
					TIM1->CCR1 = CAL_SBUS_CH.CAL_CH3 - uav_control_data.rollOut - uav_control_data.pitchOut;//+ uav_control_data.yawOut;
					TIM1->CCR2 = CAL_SBUS_CH.CAL_CH3 + uav_control_data.rollOut + uav_control_data.pitchOut;// + uav_control_data.yawOut;
					TIM1->CCR3 = CAL_SBUS_CH.CAL_CH3 + uav_control_data.rollOut - uav_control_data.pitchOut;// - uav_control_data.yawOut;
					TIM1->CCR4 = CAL_SBUS_CH.CAL_CH3 - uav_control_data.rollOut + uav_control_data.pitchOut;//- uav_control_data.yawOut;
					}
					break;
				case EMERGENCY:
					TIM1->CCR1 = 1000;
					TIM1->CCR2 = 1000;
					TIM1->CCR3 = 1000;
					TIM1->CCR4 = 1000;
					break;
			}
			aux_channel_proc();
			if(lock_unlock_proc())
			{
				if(uavSafeFlag == LOCKED && unlockCount >= 200)
				{
					uavSafeFlag = UNLOCKED;
					unlockCount = 0;
				}
				else if(uavSafeFlag == UNLOCKED && unlockCount >= 200)
				{
					uavSafeFlag = LOCKED;
					unlockCount = 0;
				}
			}
			
		}
		else
		{
			uavSafeFlag = EMERGENCY;
			TIM1->CCR1 = 1000;
			TIM1->CCR2 = 1000;
			TIM1->CCR3 = 1000;
			TIM1->CCR4 = 1000;
		}
		

		osDelay(5);
  }	
	
}


void 	UAV_Control_Init(_uav_control_data* uav_data)
{
	uav_data->rollData.Kp = 1; 
	uav_data->rollData.Ki = 0; 
	uav_data->rollData.Kd = 0;
	uav_data->rollData.ErrorMax = 70;
	uav_data->rollData.DifferentialMax = 200;
	uav_data->rollData.IntegrateMax = 1000;
	
	uav_data->pitchData.Kp = 1; 
	uav_data->pitchData.Ki = 0; 
	uav_data->pitchData.Kd = 0;
	uav_data->pitchData.ErrorMax = 70;
	uav_data->pitchData.DifferentialMax = 200;
	uav_data->pitchData.IntegrateMax = 1000;
	
  uav_data->yawData.Kp = 1; 
	uav_data->yawData.Ki = 0; 
	uav_data->yawData.Kd = 0; 
	uav_data->yawData.ErrorMax = 70;
	uav_data->yawData.DifferentialMax = 200;
	uav_data->yawData.IntegrateMax = 1000;

  uav_data->rollSpeedData.Kp = 1;
	uav_data->rollSpeedData.Ki = 0;
	uav_data->rollSpeedData.Kd = 0;
	uav_data->rollSpeedData.ErrorMax = 100;
	uav_data->rollSpeedData.DifferentialMax = 200;
	uav_data->rollSpeedData.IntegrateMax = 1000;
	
	uav_data->pitchSpeedData.Kp = 1;
	uav_data->pitchSpeedData.Ki = 0;
	uav_data->pitchSpeedData.Kd = 100;
	uav_data->pitchSpeedData.ErrorMax = 100;
	uav_data->pitchSpeedData.DifferentialMax = 200;
	uav_data->pitchSpeedData.IntegrateMax = 1000;
}	

void aux_channel_proc()      //辅助通道处理
{
	if(CAL_SBUS_CH.CAL_CH5 < 2100 && CAL_SBUS_CH.CAL_CH5 > 1400 && uavSafeFlag == UNLOCKED)
	{
			uavSafeFlag = FLYING;
	}
	else if(CAL_SBUS_CH.CAL_CH5 < 1100 && CAL_SBUS_CH.CAL_CH5 > 900 && uavSafeFlag == FLYING)
	{
			uavSafeFlag = UNLOCKED;
	}
	
	if(CAL_SBUS_CH.CAL_CH8 < 2100 && CAL_SBUS_CH.CAL_CH8 > 1400)  //急停
	{
			uavSafeFlag = LOCKED;
	}
}


u8 lock_unlock_proc()				//上锁解锁处理
{
	if(CAL_SBUS_CH.CAL_CH1 < 1050 && CAL_SBUS_CH.CAL_CH1 > 990 &&
		CAL_SBUS_CH.CAL_CH2 < 1050 && CAL_SBUS_CH.CAL_CH2 > 990 &&
		CAL_SBUS_CH.CAL_CH3 < 1050 && CAL_SBUS_CH.CAL_CH3 > 990 &&
		CAL_SBUS_CH.CAL_CH4 < 2010 && CAL_SBUS_CH.CAL_CH4 > 1950)
	{
		printf("unlockCount :%d\r\n", unlockCount);
		unlockCount++;
		return 1;
	}
	else
	{
		unlockCount = 0;
		return 0;
	}
}

