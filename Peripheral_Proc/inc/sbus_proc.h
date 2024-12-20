#ifndef __SBUS_PROC_H__
#define __SBUS_PROC_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "timers.h"
#include "parameter.h"


#include <stdio.h>
typedef struct
{
		uint16_t CH1;//通道1数值
		uint16_t CH2;//通道2数值
		uint16_t CH3;//通道3数值
		uint16_t CH4;//通道4数值
		uint16_t CH5;//通道5数值
		uint16_t CH6;//通道6数值
    uint16_t CH7;//通道7数值
    uint16_t CH8;//通道8数值
    uint16_t CH9;//通道9数值
    uint16_t CH10;//通道10数值
    uint16_t CH11;//通道11数值
    uint16_t CH12;//通道12数值
    uint16_t CH13;//通道13数值
    uint16_t CH14;//通道14数值
    uint16_t CH15;//通道15数值
    uint16_t CH16;//通道16数值
		uint8_t Connect_State;//遥控器与接收器连接状态 0=未连接，1=正常连接
	
		uint16_t CH1_MAX;
		uint16_t CH1_MIN;
	
		uint16_t CH2_MAX;
		uint16_t CH2_MIN;
		
		uint16_t CH3_MAX;
		uint16_t CH3_MIN;
		
		uint16_t CH4_MAX;
		uint16_t CH4_MIN;
		
		uint16_t CH5_MAX;
		uint16_t CH5_MIN;
	
		uint16_t CH6_MAX;
		uint16_t CH6_MIN;
		
		uint16_t CH7_MAX;
		uint16_t CH7_MIN;
		
		uint16_t CH8_MAX;
		uint16_t CH8_MIN;
}_sbus_ch_struct;


typedef struct
{
		uint16_t CAL_CH1;//通道1数值
		uint16_t CAL_CH2;//通道2数值
		uint16_t CAL_CH3;//通道3数值
		uint16_t CAL_CH4;//通道4数值
		uint16_t CAL_CH5;//通道5数值
		uint16_t CAL_CH6;//通道6数值
		uint16_t CAL_CH7;//通道7数值
		uint16_t CAL_CH8;//通道8数值
		uint8_t Connect_State;//遥控器与接收器连接状态 0=未连接，1=正常连接
}_sbus_ch_cal_struct;

void Sbus_Uart6_Task_Proc(void const * argument);
void Sbus_Uart6_IDLE_Proc(uint16_t Size);

void Sbus_Channels_Proc(void);
u16 Sbus_To_Pwm(u16 sbus_value);
float Sbus_To_Range(u16 sbus_value, float p_min, float p_max, u16 ch_min, u16 ch_max);

void Channel_Param_Init();
void Remote_Channel_Calibration();

#endif