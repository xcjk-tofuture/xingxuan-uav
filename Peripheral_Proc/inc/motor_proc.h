#ifndef __MOTOR_PROC_H__
#define __MOTOR_PROC_H__


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "timers.h"


#include "sbus_proc.h"
#include "PID.h"
#include "AHRS.h"
typedef struct _UAV_CONTROL_DATA
{
	PID	rollPid;
	PID_DATA rollData;
	
	PID	pitchPid;
	PID_DATA 	pitchData;
	
	PID	yawPid;
	PID_DATA yawData;
	
	PID	rollSpeedPid;
	PID_DATA rollSpeedData;
	
	PID	pitchSpeedPid;
	PID_DATA pitchSpeedData;
	
	s16 rollSpeedOut;
	s16 pitchSpeedOut;
	
	s16 rollOut;
	s16 pitchOut;
	s16 yawOut;
	
} _uav_control_data;


typedef enum{
	LOCKED = 0,
	UNLOCKED,
	FLYING,
	EMERGENCY
} UAV_STA;


void Motor_Task_Proc(void const * argument);
void 	UAV_Control_Init(_uav_control_data* uav_data);
u8 lock_unlock_proc(void);
void aux_channel_proc();
//void pwm_timer_callback(TimerHandle_t xTimer);
//void pwm_init(uint16_t frequency, uint8_t duty_cycle);
//void pwm_set(uint16_t frequency, uint8_t duty_cycle);




#endif