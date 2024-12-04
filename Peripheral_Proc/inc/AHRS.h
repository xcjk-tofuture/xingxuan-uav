#ifndef __AHRS_H__
#define __AHRS_H__

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "timers.h"
#include "parameter.h"
#include "SPL06.h"
#include "AK8975.h"
#include "BMI088_1.h"
#include "flash_proc.h"
//==����



//==����



typedef struct
{
	//
	u8 data_sta;  //0������  1����
	float IEM[3][3];
	//
	vec3_f f_gyrRaw;
	vec3_f f_accRaw;
	//
	vec3_f f_gyr_dps;
	vec3_f f_acc_cmpss;	
	//
	vec3_f f_gyr_dps_nb;
	vec3_f f_gyr_radps_nb;
	vec3_f f_acc_cmpss_nb;		
	//
	float f_temperature;
	vec3_f gyrSensitivity;
	vec3_f accSensitivity;
	//
	
}_imuData_st;


typedef struct
{
	acc_raw_data_t acc;
	Vector3f_t accoffsetbias;  //���ٶȼ���ƫ���
	Vector3f_t accscalebias;   //���ٶȼƿ̶����
	
	
	gyro_raw_data_t gyro;
	Vector3f_t gyrobias;  		 //��������ƫ���
	
	
	mag_raw_data_t mag;
	Vector3f_t magoffsetbias;  //��������ƫ���
	Vector3f_t magscalebias;   //�����ƿ̶����
	
	
	float f_temperature;
  float Pressure;
	
} _imuData_all;

typedef struct
{
	float roll;
	float pitch;
	float yaw;
	
} uav_attitude;


typedef struct _AHRS_DATA
{
	float roll;
	float pitch;
	float yaw;
	
	float rollSpeed;
	float pitchSpeed;
	float yawSpeed;
	
	float q0;
	float q1;
	float q2;
	float q3;
	
} _ahrs_data;




typedef struct
{
	u8 test_u8[4];
	float test[3];
}__attribute__ ((__packed__)) _test_st;
extern _test_st test_st;


//==��������
extern _imuData_st st_imuData;	

//==��������

//static


//public

void AHRS_Uart4_IDLE_Proc(u8 size);

void Sensors_Update();
void Sensors_Init();
/*IMU�����������ȳ�ʼ��*/
void ImuSensitivityInit(u8 ins_calibrated,vec3_f accRefValue);
/*IMU���������ݻ�ȡ*/
void ImuDataGet(vec3_s16 gyrRaw,vec3_s16 accRaw);
/*IMU�¶Ȼ�ȡ*/
void ImuTemperatureGet(float f_temperature);
/*IMU���ݼ��㴦��*/
void ImuDataCalcu(u8 ins_calibrated,vec3_f gyrOffset,vec3_f accOffset,float IEM[3][3]);

void IMU_Update(acc_raw_data_t acc, gyro_raw_data_t gyro, mag_raw_data_t mag, _imuData_all* imu);

void AHRS_Mahony_Update(_imuData_all imu, uav_attitude *attitude);
void AHRS_Kalman_Update(_imuData_all imu, uav_attitude *attitude);


void Sensor_Calibration(_imuData_all* imu);

void Simple_Zero_Offset_Calibration(_imuData_all* imu, Vector3f_t * offset);  //����ƫУ׼

void SensorData_Task_Proc(void const * argument);


void LMS_Fitting(float raw[6][3], Vector3f_t * offset, Vector3f_t * scale);  //�����󵼵�������Ϻ���
 
void Acc_LMS_Calibration(_imuData_all* imu, Vector3f_t * offset, Vector3f_t * scale);   //���ٶȼ��������

void IMU_Temperature_Control_Init();  //IMU���¿��Ƴ�ʼ��

void IMU_Temperature_Control(float target);  //IMU���¿���  �����¶�

float invSqrt(float x);
float DATA_Trans(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4);

void Cold_Start_ARHS(_imuData_all imu, uav_attitude *attitude);

#endif