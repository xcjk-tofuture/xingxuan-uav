#include "oled_proc.h"
#include "flight_snapshot.h"
#include "oledfont.h"
#include <stdio.h>

/********************************************************/
/*   2022/9/30                                     
*   Unicorn_Yihao                                       
*   STM32 7针硬件SPI 0.96 OLED HAL库显示驱动   
*********************************************************
*   引脚定义：                                         
*   OLED_CS OLED_RES OLED_DC OUTPUT    Output push pull
*********************************************************
*   SPI定义：                                          
*   Mode:Transmit only Master                          
*   Hardware Nss Signal:Disable                        
*   Data Size : 8Bits                                  
*   First Bit : MSB First                              
*   CPOL : Low                                         
*   CPHA : 1 Edge	 
*********************************************************
*   接线：                                             
*   GND ---> GND                                       
*   VCC ---> 3.3V                                      
*   DO  ---> SPI_SCK       PA5                            
*   D1  ---> SPI_MOSI      PA7                            
*   RES ---> OLED_RES      PB1                            
*   DC  ---> OLED_DC       PC5                           
*   CS  ---> OLED_CS       PC4                            
*                                                      
*/                                                     
/********************************************************/

 osThreadId OLEDTaskHandle;

static _imuData_all screen_sensors;
static _ahrs_data screen_attitude;
static _sbus_ch_cal_struct screen_channels;
static _sbus_ch_struct screen_raw;
static _flow_data screen_flow;
static u8 displayPage = 1;
static uint8_t requested_page,page_pending;
void uav_display_request_page(uint8_t page) {taskENTER_CRITICAL();requested_page=page;page_pending=1;taskEXIT_CRITICAL();}
void uav_display_next_page(void) {taskENTER_CRITICAL();requested_page=displayPage>=4?1:(uint8_t)(displayPage+1);page_pending=1;taskEXIT_CRITICAL();}



static uint8_t screen_calibration_step;
 void OLED_Task_Proc(void const * argument)           //OLED进程主程序
{
  /* USER CODE BEGIN RGB_Task_Proc */
  /* Infinite loop */
	OLED_Init();
	OLED_Clear();
  for(;;)
  {
		OLED_Auto_Clear();
		taskENTER_CRITICAL();if(page_pending){displayPage=requested_page;page_pending=0;}taskEXIT_CRITICAL();
        flight_snapshot_t flight;flight_snapshot_read(&flight);
        screen_attitude.roll=flight.roll_rad*57.29577951f;screen_attitude.pitch=flight.pitch_rad*57.29577951f;screen_attitude.yaw=flight.yaw_rad*57.29577951f;
        sensor_snapshot_read(&screen_sensors);sbus_snapshot(&screen_channels);sbus_raw_snapshot(&screen_raw);flow_snapshot(&screen_flow);screen_calibration_step=sensor_calibration_step();
        Show_Data(displayPage);
		//HAL_UART_Transmit(&huart1, (u8 *)"2233", 6, 50);
		//上面的初始化以及清屏的代码在一开始处一定要写
		//OLED_ShowString(0,0,"xcjk",16, 0);    //反相显示8X16字符串


		osDelay(100);

  }
  /* USER CODE END RGB_Task_Proc */
}
 

void OLED_Auto_Clear()
{
	static u8 lastPage;
	
	if(displayPage != lastPage)
	{
			OLED_Clear();
	}
	lastPage =displayPage;
}
 
void Show_Data(u8 page)
{
	  u8 oledDisp[120];
	if(page == 1)
	{
	  sprintf((char *)oledDisp,"PAGE 1  IMU DATA "); 
		OLED_ShowString(0, 0,(char *)oledDisp,12, 0); 
		sprintf((char *)oledDisp,"R:%+0.1f P:%+0.1f Y:%0.1f     ",screen_attitude.roll, screen_attitude.pitch, screen_attitude.yaw); 
		OLED_ShowString(0, 1,(char *)oledDisp,12, 0);  
	  sprintf((char *)oledDisp, "TEMP%0.1f BA:%0.1f  ",  screen_sensors.f_temperature, screen_sensors.Pressure);
		OLED_ShowString(0, 2 ,(char *)oledDisp ,12, 0);
		sprintf((char *)oledDisp, "GX %+.2f AX %+.2f     " , screen_sensors.gyro.roll, screen_sensors.acc.x);
		OLED_ShowString(0, 3 ,(char *)oledDisp , 12, 0);
		sprintf((char *)oledDisp, "GY %+.2f AY %+.2f     " , screen_sensors.gyro.pitch, screen_sensors.acc.y);
		OLED_ShowString(0, 4 ,(char *)oledDisp , 12, 0);
		sprintf((char *)oledDisp, "GZ %+.2f AZ %+.2f     ",  screen_sensors.gyro.yaw, screen_sensors.acc.z);
		OLED_ShowString(0, 5 ,(char *)oledDisp    , 12, 0);
		sprintf((char *)oledDisp, "MX%+6.1f MY%+6.1f    MZ%+6.1f  ", screen_sensors.mag.x, screen_sensors.mag.y, screen_sensors.mag.z);
		OLED_ShowString(0, 6 ,(char *)oledDisp    , 12, 0);
	}
	else if(page == 2)
	{
//		sprintf((char *)oledDisp,"PAGE 2  REMOTE DATA "); 
//		OLED_ShowString(0, 0,(char *)oledDisp,12, 0);
		
		sprintf((char *)oledDisp,"Ch1 %04d %04d       ", screen_channels.CAL_CH1, screen_raw.CH1); 
		OLED_ShowString(0, 0,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Ch2 %04d %04d       ", screen_channels.CAL_CH2, screen_raw.CH2); 
		OLED_ShowString(0, 1,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Ch3 %04d %04d       ", screen_channels.CAL_CH3, screen_raw.CH3); 
		OLED_ShowString(0, 2,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Ch4 %04d %04d       ", screen_channels.CAL_CH4, screen_raw.CH4); 
		OLED_ShowString(0, 3,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Ch5 %04d %04d       ", screen_channels.CAL_CH5, screen_raw.CH5); 
		OLED_ShowString(0, 4,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Ch6 %04d %04d       ", screen_channels.CAL_CH6, screen_raw.CH6); 
		OLED_ShowString(0, 5,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Ch7 %04d %04d       ", screen_channels.CAL_CH7, screen_raw.CH7); 
		OLED_ShowString(0, 6,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Ch8 %04d %04d       ", screen_channels.CAL_CH8, screen_raw.CH8); 
		OLED_ShowString(0, 7,(char *)oledDisp,12, 0);
	}
	else if(page == 3)
	{
		sprintf((char *)oledDisp,"PAGE  3   FLOW DATA   "); 
		OLED_ShowString(0, 0,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"TYPE Upixels          "); 
		OLED_ShowString(0, 1,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Vx:%+6.2fmm/s   ", screen_flow.xFlowVel); 
		OLED_ShowString(0, 2,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Vy:%+6.2fmm/s   ", screen_flow.yFlowVel);  
		OLED_ShowString(0, 3,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"Vz:%+6.2fmm/s   ", screen_flow.zFlowVel); 
		OLED_ShowString(0, 4,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"H:%05d mm       ", screen_flow.zNowHeight); 
		OLED_ShowString(0, 5,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"conf:%02d%%       ", screen_flow.flowConf); 
		OLED_ShowString(0, 6,(char *)oledDisp,12, 0);
	}
		else if(page == 4)
	{
		sprintf((char *)oledDisp,"PAGE  4   CALI DATA   "); 
		OLED_ShowString(0, 0,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"         "); 
		OLED_ShowString(0, 1,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"AxO:%+4.1f %+4.3f  ", screen_sensors.accoffsetbias.x, screen_sensors.gyrooffsetbias.x); 
		OLED_ShowString(0, 2,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"AyO:%+4.1f %+4.3f  ", screen_sensors.accoffsetbias.x ,screen_sensors.gyrooffsetbias.x);  
		OLED_ShowString(0, 3,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"AzO:%+4.1f %+4.3f  ", screen_sensors.accoffsetbias.x ,screen_sensors.gyrooffsetbias.x); 
		OLED_ShowString(0, 4,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"MxO:%4.1f       ", screen_sensors.magoffsetbias.x); 
		OLED_ShowString(0, 5,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"MyO:%4.1f       ", screen_sensors.magoffsetbias.y); 
		OLED_ShowString(0, 6,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"MzO:%4.1f       ", screen_sensors.magoffsetbias.z); 
		OLED_ShowString(0, 7,(char *)oledDisp,12, 0);
	}

	
		else if(page == 19)
	{
		sprintf((char *)oledDisp,"CAL_Ch1 %04d %04d    ",screen_raw.CH1_MAX, screen_raw.CH1_MIN); 
		OLED_ShowString(0, 0,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"CAL_Ch2 %04d %04d    ",screen_raw.CH2_MAX, screen_raw.CH2_MIN); 
		OLED_ShowString(0, 1,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"CAL_Ch3 %04d %04d    ",screen_raw.CH3_MAX, screen_raw.CH3_MIN); 
		OLED_ShowString(0, 2,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"CAL_Ch4 %04d %04d    ",screen_raw.CH4_MAX, screen_raw.CH4_MIN); 
		OLED_ShowString(0, 3,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"CAL_Ch5 %04d %04d    ",screen_raw.CH5_MAX, screen_raw.CH5_MIN); 
		OLED_ShowString(0, 4,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"CAL_Ch6 %04d %04d    ",screen_raw.CH6_MAX, screen_raw.CH6_MIN); 
		OLED_ShowString(0, 5,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"CAL_Ch7 %04d %04d    ",screen_raw.CH7_MAX, screen_raw.CH7_MIN); 
		OLED_ShowString(0, 6,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"CAL_Ch8 %04d %04d    ",screen_raw.CH8_MAX, screen_raw.CH8_MIN); 
		OLED_ShowString(0, 7,(char *)oledDisp,12, 0);
	}
	else if(page == 20)
	{
		sprintf((char *)oledDisp,"    MAG CAIL "); 
		OLED_ShowString(0, 0,(char *)oledDisp,12, 0);
		
		sprintf((char *)oledDisp,"MX %+4.1f ", screen_sensors.magoffsetbias.x); 
		OLED_ShowString(0, 5,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"MY %+4.1f ", screen_sensors.magoffsetbias.y); 
		OLED_ShowString(0, 6,(char *)oledDisp,12, 0);
		sprintf((char *)oledDisp,"MZ %+4.1f ", screen_sensors.magoffsetbias.z); 
		OLED_ShowString(0, 7,(char *)oledDisp,12, 0);
		if(screen_calibration_step == 0)
		{
			sprintf((char *)oledDisp,"Rotate along the X"); 
			OLED_ShowString(0, 1,(char *)oledDisp,12, 0);
		}
	  else if(screen_calibration_step == 1)
		{
			sprintf((char *)oledDisp,"Rotate along the Y"); 
			OLED_ShowString(0, 1,(char *)oledDisp,12, 0);
		}
		else if(screen_calibration_step == 2)
		{
			sprintf((char *)oledDisp,"Rotate along the Z"); 
			OLED_ShowString(0, 1,(char *)oledDisp,12, 0);
		}
			else if(screen_calibration_step == 3)
		{
			sprintf((char *)oledDisp,"Cail mag OK Saving"); 
			OLED_ShowString(0, 1,(char *)oledDisp,12, 0);
		}	
		else if(screen_calibration_step == 4)
		{
			displayPage = 1;
		}	
	}

}






 
/**********************************************************
 * 初始化命令,根据芯片手册书写
 ***********************************************************/
