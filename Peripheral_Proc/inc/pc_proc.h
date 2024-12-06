#ifndef __PC_PROC_H__
#define __PC_PROC_H__


#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "AK8975.h"
#include "BMI088_1.h"
#include "usart.h"
#include "vector3.h"
#include "AHRS.h"
#include "stdio.h"

#define BYTE0(dwTemp)     (*(char *)(&dwTemp))
#define BYTE1(dwTemp)     (*((char *)(&dwTemp) +1))
#define BYTE2(dwTemp)     (*((char *)(&dwTemp) +2))
#define BYTE3(dwTemp)     (*((char *)(&dwTemp) +3))


void ANODT_SendState(s16 pitch, s16 roll, s16 yaw, s8  FUSION_STA);  
void ANODT_SendQuaternion(s16 v0, s16 v1, s16 v2, s16 v3,  s8 state);      //向匿名上位机发送四元数 放大10000倍
void ANODT_SendRawMagTempAlt(s16 mag_x, s16 mag_y, s16 mag_z, s32 alt_bar, s16 tmp, s8 alt_state, s8 mag_state);      //向匿名上位机发送传感器原始数据 磁力计（100） 气压计（原始） 温度（10）
void ANODT_SendRawAccGyro(s16 acc_x, s16 acc_y, s16 acc_z, s16 gyo_x, s16 gyo_y, s16 gyo_z, s8 state);      //向匿名上位机发送传感器原始数据 加速度（100） 磁力计（100）
void PC_Task_Proc(void const * argument);

#endif

