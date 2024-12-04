/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define portGET_RUN_TIME_COUNTER_VALUE()	FreeRTOSRunTimeTicks
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define XC_FREERTOS_MAKE_FLAG //编译标志位 彻底解决cube改代码需要删除东西
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern osThreadId RGBTaskHandle;
extern osThreadId BleUart3TaskHandle;
extern osThreadId BuzzerTaskHandle;
extern osThreadId MotorTaskHandle;
extern osThreadId SbusUart6TaskHandle;
extern osThreadId FlashTaskHandle;
extern osThreadId SensorDataTaskHandle;
extern osThreadId OLEDTaskHandle;
extern osThreadId PCTaskHandle;
extern osThreadId FlowTaskHandle;
extern osThreadId KeyTaskHandle;



extern void RGB_Task_Proc(void const * argument);
extern void Buzzer_Task_Proc(void const * argument);
extern void Flash_Task_Proc(void const * argument);
extern void Key_Task_Proc(void const * argument);
extern void PC_Task_Proc(void const * argument);
extern void Sbus_Uart6_Task_Proc(void const * argument);
extern void Motor_Task_Proc(void const * argument);
extern void Ble_Uart3_Task_Proc(void const * argument);
extern void Sensor_Data_Task_Proc(void const * argument);
extern void Flow_Task_Proc(void const * argument);
extern void OLED_Task_Proc(void const * argument);
#ifndef XC_FREERTOS_MAKE_FLAG
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId RGBTaskHandle;
osThreadId KeyTaskHandle;
osThreadId BuzzerTaskHandle;
osThreadId BleUart3TaskHandle;
osThreadId SbusUart6TaskHandle;
osThreadId OLEDTaskHandle;
osThreadId SensorDataTaskHandle;
osThreadId FlashTaskHandle;
osThreadId FlowTaskHandle;
osThreadId PCTaskHandle;
osThreadId MotorTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#endif
/* USER CODE END FunctionPrototypes */

void RGB_Task_Proc(void const * argument);
void Key_Task_Proc(void const * argument);
void Buzzer_Task_Proc(void const * argument);
void Ble_Uart3_Task_Proc(void const * argument);
void Sbus_Uart6_Task_Proc(void const * argument);
void OLED_Task_Proc(void const * argument);
void Sensor_Data_Task_Proc(void const * argument);
void Flash_Task_Proc(void const * argument);
void Flow_Task_Proc(void const * argument);
void PC_Task_Proc(void const * argument);
void Motor_Task_Proc(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of RGBTask */
  osThreadDef(RGBTask, RGB_Task_Proc, osPriorityIdle, 0, 128);
  RGBTaskHandle = osThreadCreate(osThread(RGBTask), NULL);

  /* definition and creation of KeyTask */
  osThreadDef(KeyTask, Key_Task_Proc, osPriorityIdle, 0, 128);
  KeyTaskHandle = osThreadCreate(osThread(KeyTask), NULL);

  /* definition and creation of BuzzerTask */
  osThreadDef(BuzzerTask, Buzzer_Task_Proc, osPriorityIdle, 0, 128);
  BuzzerTaskHandle = osThreadCreate(osThread(BuzzerTask), NULL);

  /* definition and creation of BleUart3Task */
  osThreadDef(BleUart3Task, Ble_Uart3_Task_Proc, osPriorityIdle, 0, 128);
  BleUart3TaskHandle = osThreadCreate(osThread(BleUart3Task), NULL);

  /* definition and creation of SbusUart6Task */
  osThreadDef(SbusUart6Task, Sbus_Uart6_Task_Proc, osPriorityIdle, 0, 128);
  SbusUart6TaskHandle = osThreadCreate(osThread(SbusUart6Task), NULL);

  /* definition and creation of OLEDTask */
  osThreadDef(OLEDTask, OLED_Task_Proc, osPriorityIdle, 0, 256);
  OLEDTaskHandle = osThreadCreate(osThread(OLEDTask), NULL);

  /* definition and creation of SensorDataTask */
  osThreadDef(SensorDataTask, Sensor_Data_Task_Proc, osPriorityNormal, 0, 512);
  SensorDataTaskHandle = osThreadCreate(osThread(SensorDataTask), NULL);

  /* definition and creation of FlashTask */
  osThreadDef(FlashTask, Flash_Task_Proc, osPriorityIdle, 0, 128);
  FlashTaskHandle = osThreadCreate(osThread(FlashTask), NULL);

  /* definition and creation of FlowTask */
  osThreadDef(FlowTask, Flow_Task_Proc, osPriorityIdle, 0, 128);
  FlowTaskHandle = osThreadCreate(osThread(FlowTask), NULL);

  /* definition and creation of PCTask */
  osThreadDef(PCTask, PC_Task_Proc, osPriorityIdle, 0, 256);
  PCTaskHandle = osThreadCreate(osThread(PCTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, Motor_Task_Proc, osPriorityIdle, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_RGB_Task_Proc */
#ifndef XC_FREERTOS_MAKE_FLAG
/**
  * @brief  Function implementing the RGBTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RGB_Task_Proc */
void RGB_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN RGB_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RGB_Task_Proc */
}

/* USER CODE BEGIN Header_Key_Task_Proc */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Key_Task_Proc */
void Key_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Key_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Key_Task_Proc */
}

/* USER CODE BEGIN Header_Buzzer_Task_Proc */
/**
* @brief Function implementing the BuzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzer_Task_Proc */
void Buzzer_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Buzzer_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Buzzer_Task_Proc */
}

/* USER CODE BEGIN Header_Ble_Uart3_Task_Proc */
/**
* @brief Function implementing the BleUart3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ble_Uart3_Task_Proc */
void Ble_Uart3_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Ble_Uart3_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ble_Uart3_Task_Proc */
}

/* USER CODE BEGIN Header_Sbus_Uart6_Task_Proc */
/**
* @brief Function implementing the SbusUart6TaskHa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sbus_Uart6_Task_Proc */
void Sbus_Uart6_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Sbus_Uart6_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Sbus_Uart6_Task_Proc */
}

/* USER CODE BEGIN Header_OLED_Task_Proc */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_Task_Proc */
void OLED_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN OLED_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OLED_Task_Proc */
}

/* USER CODE BEGIN Header_Sensor_Data_Task_Proc */
/**
* @brief Function implementing the SensorDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sensor_Data_Task_Proc */
void Sensor_Data_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Sensor_Data_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Sensor_Data_Task_Proc */
}

/* USER CODE BEGIN Header_Flash_Task_Proc */
/**
* @brief Function implementing the FlashTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Flash_Task_Proc */
void Flash_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Flash_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Flash_Task_Proc */
}

/* USER CODE BEGIN Header_Flow_Task_Proc */
/**
* @brief Function implementing the FlowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Flow_Task_Proc */
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

/* USER CODE BEGIN Header_PC_Task_Proc */
/**
* @brief Function implementing the PCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PC_Task_Proc */
void PC_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN PC_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PC_Task_Proc */
}

/* USER CODE BEGIN Header_Motor_Task_Proc */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task_Proc */
void Motor_Task_Proc(void const * argument)
{
  /* USER CODE BEGIN Motor_Task_Proc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Task_Proc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
#endif
/* USER CODE END Application */
