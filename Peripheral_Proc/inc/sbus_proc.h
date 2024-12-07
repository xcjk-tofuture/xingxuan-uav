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
		uint16_t CH1;//ͨ��1��ֵ
		uint16_t CH2;//ͨ��2��ֵ
		uint16_t CH3;//ͨ��3��ֵ
		uint16_t CH4;//ͨ��4��ֵ
		uint16_t CH5;//ͨ��5��ֵ
		uint16_t CH6;//ͨ��6��ֵ
    uint16_t CH7;//ͨ��7��ֵ
    uint16_t CH8;//ͨ��8��ֵ
    uint16_t CH9;//ͨ��9��ֵ
    uint16_t CH10;//ͨ��10��ֵ
    uint16_t CH11;//ͨ��11��ֵ
    uint16_t CH12;//ͨ��12��ֵ
    uint16_t CH13;//ͨ��13��ֵ
    uint16_t CH14;//ͨ��14��ֵ
    uint16_t CH15;//ͨ��15��ֵ
    uint16_t CH16;//ͨ��16��ֵ
		uint8_t Connect_State;//ң���������������״̬ 0=δ���ӣ�1=��������
	
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
		float CAL_CH1;//ͨ��1��ֵ
		float CAL_CH2;//ͨ��2��ֵ
		float CAL_CH3;//ͨ��3��ֵ
		float CAL_CH4;//ͨ��4��ֵ
		float CAL_CH5;//ͨ��5��ֵ
		float CAL_CH6;//ͨ��6��ֵ
		float CAL_CH7;//ͨ��7��ֵ
		float CAL_CH8;//ͨ��8��ֵ
		uint8_t Connect_State;//ң���������������״̬ 0=δ���ӣ�1=��������
}_sbus_ch_cal_struct;

void Sbus_Uart6_Task_Proc(void const * argument);
void Sbus_Uart6_IDLE_Proc(uint16_t Size);

void Sbus_Channels_Proc(void);
u16 Sbus_To_Pwm(u16 sbus_value);
float Sbus_To_Range(u16 sbus_value, float p_min, float p_max);




#endif