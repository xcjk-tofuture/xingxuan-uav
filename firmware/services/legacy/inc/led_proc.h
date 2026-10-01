#ifndef __LED_PROC_H__

#define __LED_PROC_H__



#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"






void RGB_Task_Proc(void const * argument);		//RGB进程主程序
void RGB_Show_Proc(u8 ucled);                 //RGB处理函数
void RGB_Normal(u8 ucled);

void RGB_Calibration(u8 ucled);
void RGB_Flying(u8 ucled);
void RGB_Unlocked(u8 ucled);
void RGB_Emergency(u8 ucled);
#endif