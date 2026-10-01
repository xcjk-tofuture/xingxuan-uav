#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "sbus_proc.h"
#include "AHRS.h"
#include "display_service.h"
#include "motor_proc.h"
#include "uav_actuator.h"
#include "flight_snapshot.h"
#include "platform_time.h"


static _sbus_ch_cal_struct control_channels;
static _ahrs_data control_attitude;

static _uav_control_data uav_control_data;

osThreadId MotorTaskHandle;


   //传感器校准标准位
//5通道 1000是锁定  内8打杆后1500是飞行（自稳）2000是飞行（定高）  8通道 1000是 正常  1500是紧急停机

static u8 uavSafeFlag = LOCKED;
u8 uavAutoTakeOffFlag = 0;
u32 unlockCount = 0;

void Motor_Task_Proc(void const * argument)
{
	uav_actuator_write(0, 1000);
	uav_actuator_write(1, 1000);
	uav_actuator_write(2, 1000);
	uav_actuator_write(3, 1000);
	UAV_Control_Init(&uav_control_data);
    TickType_t wake;
	osDelay(5000);
    wake=xTaskGetTickCount();
  for(;;)
  {
        flight_snapshot_t sensed;sbus_snapshot(&control_channels);flight_snapshot_read(&sensed);
        control_attitude.roll=sensed.roll_rad*57.29577951f;control_attitude.pitch=sensed.pitch_rad*57.29577951f;
        control_attitude.yaw=sensed.yaw_rad*57.29577951f;
        control_attitude.rollSpeed=sensed.roll_rate_radps*57.29577951f;
        control_attitude.pitchSpeed=sensed.pitch_rate_radps*57.29577951f;
        control_attitude.yawSpeed=sensed.yaw_rate_radps*57.29577951f;
        if(uavSafeFlag!=LOCKED && (!sensed.valid || (uint32_t)(platform_millis()-sensed.attitude_ms)>100)) uavSafeFlag=EMERGENCY;
        if(control_channels.Connect_State == 1)
    {
			//printf("State :%d\r\n", uavSafeFlag);
			switch (uavSafeFlag){
				case LOCKED:
					uav_actuator_write(0, 1000);
					uav_actuator_write(1, 1000);
					uav_actuator_write(2, 1000);
					uav_actuator_write(3, 1000);	
//				 uav_actuator_write(0, control_channels.CAL_CH3);
//				 uav_actuator_write(1, control_channels.CAL_CH3);
//				 uav_actuator_write(2, control_channels.CAL_CH3);
//				 uav_actuator_write(3, control_channels.CAL_CH3);
					break;
				case UNLOCKED:
					uav_actuator_write(0, 1050);
					uav_actuator_write(1, 1050);
					uav_actuator_write(2, 1050);
					uav_actuator_write(3, 1050);
					break;
				case FLYING:

					uav_control_data.rollSpeedOut = (s16)PID_Control(&uav_control_data.rollPid, &uav_control_data.rollData, 0.005f, 0, (control_channels.CAL_CH1 - 1500)  / 12.0f, control_attitude.roll, 1000);
					uav_control_data.pitchSpeedOut = (s16)PID_Control(&uav_control_data.pitchPid, &uav_control_data.pitchData, 0.005f, 0, (control_channels.CAL_CH2 - 1500)  / 12.0f, control_attitude.pitch, 1000);
					
				
					uav_control_data.rollOut = (s16)PID_Control(&uav_control_data.rollSpeedPid, &uav_control_data.rollSpeedData, 0.005f, 0, uav_control_data.rollSpeedOut , control_attitude.rollSpeed, 1000);
					uav_control_data.pitchOut = (s16)PID_Control(&uav_control_data.pitchSpeedPid, &uav_control_data.pitchSpeedData, 0.005f, 0, uav_control_data.pitchSpeedOut , control_attitude.pitchSpeed, 1000);
					
					uav_control_data.yawOut = (s16)PID_Control(&uav_control_data.yawPid, &uav_control_data.yawData, 0.005f, 0, ((control_channels.CAL_CH4 - 1500)  / 12.0f) * 0.01, control_attitude.yawSpeed, 1000);
				
					if(control_channels.CAL_CH3 > 1100)
					{
					uav_actuator_write(0, control_channels.CAL_CH3 - uav_control_data.rollOut - uav_control_data.pitchOut);//+ uav_control_data.yawOut;
					uav_actuator_write(1, control_channels.CAL_CH3 + uav_control_data.rollOut + uav_control_data.pitchOut);// + uav_control_data.yawOut;
					uav_actuator_write(2, control_channels.CAL_CH3 + uav_control_data.rollOut - uav_control_data.pitchOut);// - uav_control_data.yawOut;
					uav_actuator_write(3, control_channels.CAL_CH3 - uav_control_data.rollOut + uav_control_data.pitchOut);//- uav_control_data.yawOut;
					}
                    else { uav_actuator_stop(); }
					break;
				case EMERGENCY:
					uav_actuator_write(0, 1000);
					uav_actuator_write(1, 1000);
					uav_actuator_write(2, 1000);
					uav_actuator_write(3, 1000);
					break;
			}
			
			if(!sbus_calibration_active() && sensed.valid)
			{
				mag_cail_proc();
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
			

			
		}
		else
		{
			uavSafeFlag = EMERGENCY;
			uav_actuator_write(0, 1000);
			uav_actuator_write(1, 1000);
			uav_actuator_write(2, 1000);
			uav_actuator_write(3, 1000);
		}
		

		flight_state_publish(uavSafeFlag);
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(5));
  }	
	
}


void 	UAV_Control_Init(_uav_control_data* uav_data)
{
	uav_data->rollData.Kp = 0.1; 
	uav_data->rollData.Ki = 0; 
	uav_data->rollData.Kd = 0;
	uav_data->rollData.ErrorMax = 70;
	uav_data->rollData.DifferentialMax = 200;
	uav_data->rollData.IntegrateMax = 1000;
	
	uav_data->pitchData.Kp = 0.1; 
	uav_data->pitchData.Ki = 0; 
	uav_data->pitchData.Kd = 0;
	uav_data->pitchData.ErrorMax = 70;
	uav_data->pitchData.DifferentialMax = 200;
	uav_data->pitchData.IntegrateMax = 1000;
	
  uav_data->yawData.Kp = 0.1; 
	uav_data->yawData.Ki = 0; 
	uav_data->yawData.Kd = 0; 
	uav_data->yawData.ErrorMax = 70;
	uav_data->yawData.DifferentialMax = 200;
	uav_data->yawData.IntegrateMax = 1000;

  uav_data->rollSpeedData.Kp = 0.1;
	uav_data->rollSpeedData.Ki = 0;
	uav_data->rollSpeedData.Kd = 0;
	uav_data->rollSpeedData.ErrorMax = 100;
	uav_data->rollSpeedData.DifferentialMax = 200;
	uav_data->rollSpeedData.IntegrateMax = 1000;
	
	uav_data->pitchSpeedData.Kp = 0.1;
	uav_data->pitchSpeedData.Ki = 0;
	uav_data->pitchSpeedData.Kd = 100;
	uav_data->pitchSpeedData.ErrorMax = 100;
	uav_data->pitchSpeedData.DifferentialMax = 200;
	uav_data->pitchSpeedData.IntegrateMax = 1000;
}	

void aux_channel_proc()      //辅助通道处理
{
	if(control_channels.CAL_CH5 < 2100 && control_channels.CAL_CH5 > 1400 && uavSafeFlag == UNLOCKED)
	{
			uavSafeFlag = FLYING;
	}
	else if(control_channels.CAL_CH5 < 1100 && control_channels.CAL_CH5 > 900 && uavSafeFlag == FLYING)
	{
			uavSafeFlag = UNLOCKED;
	}
	
	if(control_channels.CAL_CH8 < 2100 && control_channels.CAL_CH8 > 1400)  //急停
	{
			uavSafeFlag = LOCKED;
	}
}


u8 lock_unlock_proc()				//上锁解锁处理
{
	if(control_channels.CAL_CH1 < 1050 && control_channels.CAL_CH1 > 990 &&
		control_channels.CAL_CH2 < 1050 && control_channels.CAL_CH2 > 990 &&
		control_channels.CAL_CH3 < 1050 && control_channels.CAL_CH3 > 990 &&
		control_channels.CAL_CH4 < 2010 && control_channels.CAL_CH4 > 1950)
	{
		
		unlockCount++;
		return 1;
	}
	else
	{
		unlockCount = 0;
		return 0;
	}
}


void mag_cail_proc()				//上锁解锁处理
{
	
	static u32 magCount;
	if(!sensors_mag_calibration_active())
	{
			if(control_channels.CAL_CH1 < 2050 && control_channels.CAL_CH1 > 1950 &&
		control_channels.CAL_CH2 < 2050 && control_channels.CAL_CH2 > 1950 &&
		control_channels.CAL_CH3 < 2050 && control_channels.CAL_CH3 > 1950 &&
		control_channels.CAL_CH4 < 1050 && control_channels.CAL_CH4 > 950)
	{
		
		magCount++;
		if(magCount > 200)
		{
			  
			  uav_display_request_page(20);
				magCount = 0;
			  sensors_request_mag_calibration();
		}
	}
	else
	{
		magCount = 0;
	}
	
	}

}

