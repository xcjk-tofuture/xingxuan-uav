/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "dma.h"
#include "sbus_proc.h"
#include "led_proc.h"
#include "oled_proc.h"
#include "parameter.h"
#include "AHRS.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
u8 SbusRxBuf[100];
u8 BleLogRxBuf[100];
u8 uart1RX[100];
u8 uart1TX[100];
u8 uart2RX[200];
u8 uart2TX[200];
u8 uart4RX[200];
u8 uart4TX[200];
u8 uart5RX[200];
u8 uart5TX[200];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, SbusRxBuf, 100);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1RX, 100);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5RX, 200);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4RX, 200);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2RX, 200);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, BleLogRxBuf, 200);
	//HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4RX, 100)
	//spi cs�ų�ʼ��
  OLED_Init();                           //OLED��ʼ
  OLED_Clear();                         //����


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  

		if(huart->Instance == USART1)
	{
		//HAL_UART_Transmit_DMA(&huart1, uart1RX, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1RX, 100);
	}	
	else if(huart->Instance == USART2)
	{
		HAL_UART_Transmit(&huart2, uart2RX, Size, 50);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2RX, 200);		
	}
	else if(huart->Instance == USART3)
	{
		HAL_UART_Transmit(&huart3, BleLogRxBuf, Size, 50);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, BleLogRxBuf, 100);		
	}
	else if(huart->Instance == UART4)
	{
		//printf("1111111111");
		AHRS_Uart4_IDLE_Proc(Size);
//	  HAL_UART_Transmit(&huart4, uart4RX, Size, 50);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4RX, 200);		
	}
	else if(huart->Instance == UART5)
	{
//		printf("1111111111");
//		AHRS_Uart4_IDLE_Proc(Size);
		//HAL_UART_Transmit(&huart5, uart5RX, Size, 50);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5RX, 200);		
	}
		else if(huart->Instance == USART6)
	{
		//printf("1111111111");
		//Sbus_Uart6_IDLE_Proc(Size);
		//printf("length : %d\r\n",Size);
		//sprintf((char *)TxBufTemp, "hello world");
		//HAL_UART_Transmit(&huart3, SbusRxBuf,Size, 50);
		Sbus_Uart6_IDLE_Proc(Size);
    HAL_UART_Transmit_DMA(&huart1, SbusRxBuf, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, SbusRxBuf, 100);
		
	}


}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6)
	{
//ִ��HAL_UART_ErrorCallbackʱ��������lock������unlock��
//��ΪHAL_UART_Receive_ITִ��ʱ���ж������lock��ֱ�ӷ���BUSY
		__HAL_UNLOCK(huart);		
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, SbusRxBuf, 100);	//SBUS���� ����ͨ�ų�ʼ��
	}


}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{



//}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

