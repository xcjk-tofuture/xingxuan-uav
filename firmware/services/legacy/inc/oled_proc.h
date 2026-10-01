#ifndef UAV_OLED_SERVICE_H
#define UAV_OLED_SERVICE_H
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "flow_proc.h"
#include "AHRS.h"
#include "sbus_proc.h"
#include "oled_device.h"
#include "display_service.h"
void Show_Data(uint8_t page);
void OLED_Auto_Clear(void);
#endif
