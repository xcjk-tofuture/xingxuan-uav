#include "AHRS.h"
#include "oled_proc.h"
#include "lowPassFilter.h"
#include "matrix6.h"
#include "pid.h"
#include "tim.h"
#include "stdio.h"

#define RAD_PER_DEG     0.017453293f
#define DEG_PER_RAD     57.29577951f


#define EXTERN_IMU 0

#define SENSORS_ENABLE_SPL06 1;
#define UPDATE_TIME 5


#define CALIBRATION_COUNT 500 //决定用多少个值去做校准



extern u8 uart4RX[200];

float gyroCalibration[CALIBRATION_COUNT];



osThreadId SensorDataTaskHandle;

//u16 FlashTest;
acc_raw_data_t test_acc;
gyro_raw_data_t test_gyro;
mag_raw_data_t test_mag;


_imuData_all imudata_all;
_ahrs_data attitude_t;



PID_DATA imu_temperature_control_pid_data;
PID imu_temperature_control_pid;


u8 SensorError = 0;
u8 AccCalFlag = 1;   //传感器校准标准位
u8 GyroCalFlag = 1;   //传感器校准标准位
u8 MagCalFlag = 1;   //传感器校准标准位
u8 Bmi088Init_Flag = 1;
u8 AK8975Flag = 1;
u8 SPL06Flag = 1;
u8 IMUTemperatureFlag = 1;

u32 sensorTimeCount = 0;
void Sensor_Data_Task_Proc(void const * argument)
{
	osDelay(2000);
	#if !EXTERN_IMU
	Sensors_Init();  //传感器初始化
	#else
	#ifdef SENSORS_ENABLE_SPL06
		SPL06Flag				=	Drv_Spl0601_Init();
	#endif
	#endif

	
	static TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

 for(;;)
	{
		
	vTaskDelayUntil(&xLastWakeTime, 1); //绝对延时
	sensorTimeCount ++;
	#if !EXTERN_IMU
	
	if(sensorTimeCount % 3 == 0)
	{	
		IMU_Temperature_Control(40);
		ReadAccTemperature(&imudata_all.f_temperature);
		ReadAccData(&test_acc);
		ReadGyroData(&test_gyro);

	}
	#endif	
	if(sensorTimeCount % UPDATE_TIME == 0)
	{
		
		imudata_all.Pressure = Drv_SPl0601_Read();
		//Spl0601Get(&imudata_all.Hight);
	#if !EXTERN_IMU
		ReadMagData(&test_mag);
		IMU_Update(test_acc, test_gyro, test_mag, &imudata_all);
	 if(!(AccCalFlag || GyroCalFlag))	
	 {
		//AHRS_Kalman_Update(imudata_all, &attitude_t);  
		AHRS_Mahony_Update(imudata_all, &attitude_t);
	 }

	 else
	 {	
		//printf("CALLING... \r\n");
		Sensor_Calibration(&imudata_all);
		if(sensorTimeCount == CALIBRATION_COUNT * UPDATE_TIME)
			Cold_Start_ARHS(imudata_all, &attitude_t);
	 }
	#endif
	}

	
		 //传感器
		//FlashTest = W25QXX_ReadID();
	
	
	
	}

}


void AHRS_Uart4_IDLE_Proc(u8 size)
{
	if(size == 56 && uart4RX[0] == 0xFC && uart4RX[1] == 0x41)
	{
		#if EXTERN_IMU
		attitude_t.rollSpeed=DATA_Trans(uart4RX[7],uart4RX[8],uart4RX[9],uart4RX[10]) * DEG_PER_RAD;       //横滚角速度
		attitude_t.pitchSpeed=DATA_Trans(uart4RX[11],uart4RX[12],uart4RX[13],uart4RX[14]) * DEG_PER_RAD;   //俯仰角速度
		attitude_t.yawSpeed=DATA_Trans(uart4RX[15],uart4RX[16],uart4RX[17],uart4RX[18]) * DEG_PER_RAD; //偏航角速度
			
    attitude_t.roll=DATA_Trans(uart4RX[19],uart4RX[20],uart4RX[21],uart4RX[22]) * DEG_PER_RAD;      //横滚角
		attitude_t.pitch=DATA_Trans(uart4RX[23],uart4RX[24],uart4RX[25],uart4RX[26]) * DEG_PER_RAD;     //俯仰角
		attitude_t.yaw=DATA_Trans(uart4RX[27],uart4RX[28],uart4RX[29],uart4RX[30]) * DEG_PER_RAD;	 //偏航角
			
		attitude_t.q0=DATA_Trans(uart4RX[31],uart4RX[32],uart4RX[33],uart4RX[34]);  //四元数
		attitude_t.q1=DATA_Trans(uart4RX[35],uart4RX[36],uart4RX[37],uart4RX[38]);
		attitude_t.q2=DATA_Trans(uart4RX[39],uart4RX[40],uart4RX[41],uart4RX[42]);
		attitude_t.q3=DATA_Trans(uart4RX[43],uart4RX[44],uart4RX[45],uart4RX[46]);
		#endif
	}

}


float DATA_Trans(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4)
{
  u32 transition_32;
	float tmp=0;
	int sign=0;
	int exponent=0;
	float mantissa=0;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
  sign = (transition_32 & 0x80000000) ? -1 : 1;//符号位
	//先右移操作，再按位与计算，出来结果是30到23位对应的e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	//将22~0转化为10进制，得到对应的x系数 
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp=sign * mantissa * pow(2, exponent);
	return tmp;
}

void Sensors_Init()  //传感器初始化
{
	
	AK8975Flag      = DrvAK8975Check();
	Bmi088Init_Flag = BMI088_INIT();
	
	#ifdef SENSORS_ENABLE_SPL06
		SPL06Flag				=	Drv_Spl0601_Init();
	#endif

 if(Bmi088Init_Flag || AK8975Flag)
 {
	#ifdef SENSORS_ENABLE_SPL06
		if(SPL06Flag)
				SensorError = 1;
	#endif
				SensorError = 1;
 }
 
 IMU_Temperature_Control_Init();

 
  
 
}


void Sensor_Calibration(_imuData_all* imu)  //传感器校准
{
	if(GyroCalFlag)
		Simple_Zero_Offset_Calibration(imu, &(imu->gyrooffsetbias));  //简单零偏误差校准
	if(AccCalFlag)
		Acc_LMS_Calibration(imu, &(imu->accoffsetbias), &(imu->accscalebias));
}


void Simple_Zero_Offset_Calibration(_imuData_all* imu, Vector3f_t *offset)//陀螺仪零偏校准
{
 static float gyroBias[3] = {0.0f};
 static int i = 0;

 gyroBias[0] += imu->gyro.roll;
 gyroBias[1] += imu->gyro.pitch;
 gyroBias[2] += imu->gyro.yaw;

 i++;
 if(i >= CALIBRATION_COUNT)
 {
	 
	 if(gyroBias[0] >= 500 || gyroBias[1] >= 500 || gyroBias[2] >= 500) //陀螺仪存在运动状态
	 {
	    i = 0;
		  gyroBias[0] = 0;
		  gyroBias[1] = 0;
			gyroBias[2] = 0;
	 }
	 else
	{
			gyroBias[0] /= i;
			gyroBias[1] /= i;
			gyroBias[2] /= i;	 
			 
			offset->x = gyroBias[0];
			offset->y = gyroBias[1];
			offset->z = gyroBias[2];
			//printf("%d\r\n",i);
			GyroCalFlag = 0; 
	}

 }

}	

float raw[6][3];
void Acc_LMS_Calibration(_imuData_all* imu, Vector3f_t * offset, Vector3f_t * scale) //传感器广义椭球校准
{
	osDelay(200);
	static int i = 0;
	raw[i][0] = imu->acc.x;
	raw[i][1] = imu->acc.y;
	raw[i][2] = imu->acc.z;
	i++;
	if(i == 6)
	{
		//LMS_Fitting(raw, offset, scale);
		AccCalFlag = 0;
	}
	

}


void LMS_Fitting(float raw[6][3], Vector3f_t * offset, Vector3f_t * scale)   //用于椭球拟合加速度拟合
{
    float x[6], y[6], z[6];
    float m[6][6];
    float m_t[6][6];
    float m_txm_inv[6][6];
    float m_txm_invxm_t[6][6];
    float m_txm[6][6];
    float p[6];
    double v[7];
    float x0, y0, z0, A, B, C;

    
	for(int i = 0; i <= 5; i++)
	{
		x[i] = raw[i][0];
		y[i] = raw[i][1];
		z[i] = raw[i][2];
	}		
	for(int i = 0; i < 6 ; i++) 
		p[i] = - (x[i] * x[i]);        //p矩阵
	for(int i = 0; i <= 5; i++)
	{
		m[i][0] = y[i] * y[i];
		m[i][1] = z[i] * z[i];
		m[i][2] = x[i];
		m[i][3] = y[i];
		m[i][4] = z[i];
		m[i][5] = 1.0f;                  //m矩阵
	}	
	
	Matrix6_Tran(m, m_t);  //求m的转置
	Matrix6_Mul(m_t, m ,m_txm);
	Matrix6_Det(m_txm,m_txm_inv);
	Matrix6_Mul(m_txm_inv, m_t ,m_txm_invxm_t);
	
	for(int i = 0; i < 6; i++)
	{
		v[0] += (m_txm_invxm_t[0][i] * p[i]);
		v[1] += (m_txm_invxm_t[1][i] * p[i]);
		v[2] += (m_txm_invxm_t[2][i] * p[i]);
		v[3] += (m_txm_invxm_t[3][i] * p[i]);
		v[4] += (m_txm_invxm_t[4][i] * p[i]);
		v[5] += (m_txm_invxm_t[5][i] * p[i]);
		
		//printf("%f\n",m_txm_invxm_t[5][i] * p[i]);
	}  
	x0 = -v[2]/ 2;
	y0 = -v[3] / (2 * v[0]);
	z0 = -v[4] / (2 * v[1]);
	A = sqrt(x0*x0 + v[1] * y0 * y0 + v[1] * z0 * z0 - v[5]);
	B = A * invSqrt(v[0]);
	C = A * invSqrt(v[1]);
	
	offset->x = x0;
	offset->y = y0;
	offset->z = z0;
	
	scale->x = A;
	scale->y = B;
	scale->z = C;

}

void IMU_Temperature_Control_Init()  //IMU恒温控制初始化
{
	HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);  
	TIM10->ARR  = 999;
	TIM10->CCR1 = 0;
	
	imu_temperature_control_pid_data.ErrorMax = 20;
	imu_temperature_control_pid_data.DifferentialMax = 70;
	imu_temperature_control_pid_data.IntegrateMax = 90;
	
	imu_temperature_control_pid_data.Kf = 0; //前馈控制
	
	imu_temperature_control_pid_data.Kp = 0.01; 
	imu_temperature_control_pid_data.Ki = 0; 
	imu_temperature_control_pid_data.Kd = 0; 
}


void IMU_Temperature_Control(float target)  //IMU恒温控制  输入温度
{
	s16 out;
	out = (u16)PID_Control(&imu_temperature_control_pid, &imu_temperature_control_pid_data, 5, 0, target, imudata_all.f_temperature,1000);
	out =  out > 999 ? 999 : out;
	out =  out < 0 ? 0 : out;
	TIM10->CCR1 = out;
	// printf("out:%d\r\n", out);
}	

/****************************************************************************************************
* 函  数：static float invSqrt(float x) 
* 功　能: 快速计算 1/Sqrt(x) 	
* 参  数：要计算的值
* 返回值：计算的结果
* 备  注：比普通Sqrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*****************************************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

#define Kp 6.f                         // proportional gain governs rate of convergence to accelerometer/magnetometer
                                         //比例增益控制加速度计，磁力计的收敛速率
#define Ki 0.008f                        // integral gain governs rate of convergence of gyroscope biases  
                                         //积分增益控制陀螺偏差的收敛速度
#define halfT UPDATE_TIME / 2000.f                     // half the sample period 采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

void IMU_Update(acc_raw_data_t acc, gyro_raw_data_t gyro, mag_raw_data_t mag, _imuData_all* imu)
{
//	imu->acc.x =imu->acc.x * (1 - 0.9) + acc.x * 0.9;
//	imu->acc.y =imu->acc.y * (1 - 0.9) + acc.y * 0.9;
//	imu->acc.z =imu->acc.z * (1 - 0.9) + acc.z * 0.9; //一阶低通滤波33	
	
//	gyro.roll = gyro.roll - imu->gyrooffsetbias.x;
//	gyro.pitch = gyro.pitch - imu->gyrooffsetbias.y;
//	gyro.yaw = gyro.yaw - imu->gyrooffsetbias.z;
	
//	imu->gyro.pitch =imu->gyro.pitch * (1 - 0.3) +  gyro.pitch * 0.3;
//	imu-> gyro.roll =imu-> gyro.roll * (1 - 0.3) +  gyro.roll * 0.3;
//	imu-> gyro.yaw =imu-> gyro.yaw * (1 - 0.3) +  gyro.yaw * 0.3; //一阶低通滤波33	
	
	
	
	imu->acc.x =acc.x ;
	imu->acc.y =acc.y;
	imu->acc.z =acc.z ;

	LPF2ndData_t LPF2_GYRO;
	Vector3f_t LPF2_GYRO_Data;
	
	LPF2ndData_t LPF2_ACC;
	Vector3f_t LPF2_ACC_Data;
	
	LPF2ndData_t LPF2_MAG;
	Vector3f_t LPF2_MAG_Data;
	
	LPF2_GYRO_Data.x = gyro.roll - imu->gyrooffsetbias.x;
	LPF2_GYRO_Data.y = gyro.pitch - imu->gyrooffsetbias.y;
	LPF2_GYRO_Data.z = gyro.yaw - imu->gyrooffsetbias.z;
	
	LPF2_ACC_Data.x = acc.x;
	LPF2_ACC_Data.y = acc.y;
	LPF2_ACC_Data.z = acc.z;
	

	
	LowPassFilter2ndFactorCal(UPDATE_TIME, 30, &LPF2_GYRO);
	LPF2_GYRO_Data = LowPassFilter2nd(&LPF2_GYRO, LPF2_GYRO_Data);     //陀螺仪二阶低通滤波 截止频率50HZ
	
	LowPassFilter2ndFactorCal(UPDATE_TIME, 50, &LPF2_ACC);
	LPF2_ACC_Data = LowPassFilter2nd(&LPF2_ACC, LPF2_ACC_Data);     //陀螺仪二阶低通滤波 截止频率50HZ
	
	imu->gyro.pitch = LPF2_GYRO_Data.y;
	imu->gyro.roll =  LPF2_GYRO_Data.x;
	imu->gyro.yaw =  LPF2_GYRO_Data.z;   //减去零偏误差
	
  imu->acc.x = LPF2_ACC_Data.x;
	imu->acc.y = LPF2_ACC_Data.y;
	imu->acc.z = LPF2_ACC_Data.z;   //减去零偏误差

	imu->mag.x = mag.x;
	imu->mag.y = mag.y;
	imu->mag.z = mag.z;
}

void AHRS_Mahony_Update(_imuData_all imu, _ahrs_data *attitude)
{	
	u8 i;
	float matrix[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f};
	float ax = imu.acc.x, ay = imu.acc.y, az = imu.acc.z;
	float mx = imu.mag.x, my = imu.mag.y, mz = imu.mag.z; //实际重力加速度分量
	float gx = imu.gyro.roll, gy = imu.gyro.pitch, gz = imu.gyro.yaw; //实际重力加速度分量

	static float R11,R21;		/*矩阵(1,1),(2,1)项*/
  static float vecxZ, vecyZ, veczZ;	/*机体坐标系下的Z方向向量*/
	float vx, vy, vz;  //理论重力加速度分量
	float ex, ey, ez;  //理论重力加速度与实际加速度误差
	float hx, hy;
	float bx, bz;
	float wx, wy, wz;
	float norm;			//求模变量		

	q0 = attitude->q0;
	q1 = attitude->q1;
	q2 = attitude->q2;
	q3 = attitude->q3;
	
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	if(ax*ay*az == 0)
		return;

	norm = invSqrt(ax * ax + ay * ay + az * az);	
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm; //加速度单位化

	
	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= norm;
	my *= norm;
	mz *= norm;    //磁力计单位化
	
	hx = (q0q0 + q1q1 - q2q2 - q3q3) * mx + (2.f * (q1q2 - q0q3)) * my  + (2.f * (q1q3 + q0q2)) * mz;
	hy = (2.f * (q1q2 + q0q3)) * mx + (q0q0 - q1q1 + q2q2 - q3q3) * my + (2.f * (q2q3 - q0q1)) * mz;

	
//	float emx,emy,emz;
//	float em_bx,em_by,em_bz;
//	em_bx = 0;
//	em_by = 0;
//	em_bz = -(hy * 1);
//	
//	emx = em_bz * (2.f * (q1q3 - q0q2));
//	emy = em_bz * (2.f * (q2q3 + q0q1));
//	emz = em_bz * (q0q0 - q1q1 - q2q2 + q3q3);
//	
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3; //理论重力加速度分量


	bz = (2.f * (q1q3 - q0q2)) * mx + (2.f * (q2q3 + q0q1)) * my + (q0q0 - q1q1 - q2q2 + q3q3) * mz;
	bx = sqrt(hx * hx + hy * hy);
	wx = (q0q0 + q1q1 - q2q2 - q3q3) * bx + (2.f * (q1q3 - q0q2)) * bz;
	wy = (2.f * (q1q2 - q0q3)) * bx + (2.f * (q2q3 + q0q1)) * bz;
	wz = (2.f * (q1q3 + q0q2)) * bx + (q0q0 - q1q1 - q2q2 + q3q3) * bz;
	
	
	ex = (ay * vz - az * vy); //(my * wz - mz * wy);
	ey = (az * vx - ax * vz); //(mz * wx - mx * wz);
	ez = (ax * vy - ay * vx); //(mx * wy - my * wx);  //叉乘求误差
	
	
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;   //对误差进行积分

	gx = gx + Kp * ex + exInt;
	gy = gy + Kp * ey + eyInt;
	gz = gz + Kp * ez + ezInt;  //对误差进行补偿

	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;  //四元数更新
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);  //四元数单位化
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;

	matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;  //四元数转矩阵  这个旋转矩阵是导航系到机体系
	matrix[1] = 2.f * (q1q2 + q0q3);
	matrix[2] = 2.f * (q1q3 - q0q2);
	matrix[3] = 2.f * (q1q2 - q0q3);
	matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;
	matrix[5] = 2.f * (q2q3 + q0q1);
	matrix[6] = 2.f * (q1q3 + q0q2);
	matrix[7] = 2.f * (q2q3 - q0q1);
	matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;
	
	
//	vecxZ = 2 * (q1 * q3 - q0 * q2);/*矩阵(1,3)项*/
//	vecyZ = 2 * (q0 * q1 + q2 * q3);/*矩阵(2,3)项*/
//	veczZ = q0q0 - q1q1 - q2q2 + q3q3;	/*矩阵(3,3)项*/

    //z->y->x 姿态角更新
	//attitude->yaw += imu.gyro.yaw * 0.01f * SEC2DEG;
	attitude->yaw = -atan2f(matrix[1], matrix[0]) * SEC2DEG; // T12/T11
	attitude->pitch = asinf(matrix[2]) *SEC2DEG;   //T13
	attitude->roll = atan2f(matrix[5], matrix[8]) * SEC2DEG;  // T23/T33
	attitude->q0 = q0 ;
	attitude->q1 = q1 ;
	attitude->q2 = q2 ;
	attitude->q3 = q3 ;

}



#define allT UPDATE_TIME / 1000.f                     // half the sample period 采样周期的一半

void AHRS_Kalman_Update(_imuData_all imu, _ahrs_data *attitude)
{
	 
  float ax = imu.acc.x;
	float ay = imu.acc.y;
	float az = imu.acc.z;
	
	float gx = imu.gyro.roll;
	float gy = imu.gyro.pitch;
	float gz = imu.gyro.yaw;
	
	float v_roll, v_pitch, v_yaw = 0;
	
	float mbx = imu.mag.x;
	float mby = imu.mag.y;
	float mbz = imu.mag.z;
	
	float mZx, mZy, mZz = 0;

	
	float roll_z,pitch_z,yaw_z = 0;
	
	static float roll_k,pitch_k,yaw_k = 0;
	static float roll_k_,pitch_k_,yaw_k_ = 0;
	static float roll_k_1,pitch_k_1,yaw_k_1 = 0;
	
	static float p_k_[9] = {0};
	static float p_k_1[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f};
	static float p_k[9] = {1.f, 0.0f, 0.0f, 0.0f, 1.f, 0.0f, 0.0f, 0.0f, 1.f};
	
  float Kk[9] = {0};
	float Q[9] = {0.0025, 0.0f, 0.0f,
								0.0f, 0.0025, 0.0f, 
								0.0f, 0.0f, 0.0025};
	float R[9] = {0.3,  0.0f, 0.0f, 
								0.0f, 0.3, 0.0f, 
								0.0f, 0.0f,0.3}; //观测噪声协方差矩阵
	//step1 - system input
  mZx = cos(pitch_k) * mbx + sin(pitch_k) * sin(roll_k) * mby + 
								sin(pitch_k) * cos(roll_k) * mbz;   //先绕roll 再绕pitch
	mZy = cos(roll_k) * mby - sin(roll_k) * mbz;
//  mZz = -sin(pitch_k) * mbx + cos(pitch_k) * sin(roll_k) * mby + 
//								cos(pitch_k) * cos(roll_k) * mbz;
								
								
  v_roll = gx - ((sin(pitch_k) * sin(roll_k)) / cos(pitch_k) ) * gy 
								+ ((cos(roll_k) * sin(pitch_k)) / cos(pitch_k) ) * gz;
	v_pitch = gy * cos(roll_k) - gz * sin(roll_k); 
	v_yaw = gy * sin(roll_k)/cos(pitch_k) + gz * cos(roll_k)/cos(pitch_k); 
						
	roll_k_ = roll_k_1 + allT * v_roll;
	pitch_k_ = pitch_k_1 + allT * v_pitch;
	yaw_k_ = yaw_k_1 + allT * v_yaw;

//		if( yaw_k_ > 3.141f ) //if it is greater than PI
//	{
//		yaw_k_ -= - 3.141f * 2.f;
//	}		
//	else if(yaw_k_ < -3.141f ) //if it is less than -PI
//	{
//		yaw_k_ += 3.141f * 2.f;
//	}

	//step2 - Prior estimation
	p_k_[0] = p_k_1[0] + Q[0];
	p_k_[4] = p_k_1[4] + Q[4];
	p_k_[8] = p_k_1[8] + Q[8];

	//step3 - Prior estimation error covariance
	//step4 - kalman gain
	Kk[0] = p_k_[0] / (p_k_[0] + R[0]);
	Kk[4] = p_k_[4] / (p_k_[4] + R[4]);
	Kk[8] = p_k_[8] / (p_k_[8] + R[8]);
	
	roll_z = atan((ay) / (az));
  pitch_z = -1 * atan((ax) / sqrt(ay * ay  + az * az));
	yaw_z = atan(mZy / mZx);
	
	roll_k = roll_k_ + Kk[0] * (roll_z - roll_k_);
	pitch_k = pitch_k_ + Kk[4] * (pitch_z - pitch_k_);
	yaw_k = yaw_k_ + Kk[8] * (yaw_z - yaw_k_);
	//step5 - measure data
	//printf("%f % f %f \r\n", Zk[0] ,Zk[1],  Zk[2] );
	p_k[0] = (1 - Kk[0]) * p_k_[0];
	p_k[4] = (1 - Kk[4]) * p_k_[4];
	p_k[8] = (1 - Kk[8]) * p_k_[8];
	
	p_k_1[0] = p_k[0];
	p_k_1[4] = p_k[4];
	p_k_1[8] = p_k[8];
	
	roll_k_1 = roll_k;
	pitch_k_1 = pitch_k;
	yaw_k_1 = yaw_k;
	
	//step6 - Posterior estimation
	//step7 - Posteriori estimation error covariance
	
  attitude->yaw = yaw_k * SEC2DEG;
	attitude->pitch = pitch_k * SEC2DEG;   //T13
	attitude->roll = roll_k * SEC2DEG;  // T23/T33

	//calculate the angle,unit: degree


}	


void Cold_Start_ARHS(_imuData_all imu, _ahrs_data *attitude)
{
  float roll,pitch,yaw = 0;


	float ax = imu.acc.x;
	float ay = imu.acc.y;
	float az = imu.acc.z;
	
	float gx = imu.gyro.roll;
	float gy = imu.gyro.pitch;
	float gz = imu.gyro.yaw;
	
	float mbx = imu.mag.x;
	float mby = imu.mag.y;
	float mbz = imu.mag.z;
	
	float mZx, mZy, mZz = 0;
	
	roll = atan((ay) / (az));
  pitch = -1 * atan((ax) / sqrt(ay * ay  + az * az));
	
	mZx = cos(pitch) * mbx + sin(pitch) * sin(roll) * mby + 
								sin(pitch) * cos(roll) * mbz;   //先绕roll 再绕pitch
	mZy = cos(roll) * mby - sin(roll) * mbz;
	

	yaw = atan(mZy / mZx);
	
	
	
	
	attitude->yaw = 0;
	attitude->pitch = pitch * SEC2DEG;   //T13
	attitude->roll = roll * SEC2DEG;  // T23/T33
	
	// 将角度转换为弧度
	double half_roll = roll * 0.5;
	double half_pitch = pitch * 0.5;
	double half_yaw = yaw * 0.5;


	// 计算三角函数值
	double sin_r = sin(half_roll);
	double cos_r = cos(half_roll);
	double sin_p = sin(half_pitch);
	double cos_p = cos(half_pitch);
	double sin_y = sin(half_yaw);
	double cos_y = cos(half_yaw);

	// 计算四元数
	attitude->q0 = cos_r * cos_p * cos_y - sin_r * sin_p * sin_y;
	attitude->q1 = sin_r * cos_p * cos_y + cos_r * sin_p * sin_y;
	attitude->q2 = cos_r * sin_p * cos_y - sin_r * cos_p * sin_y;
	attitude->q3 = cos_r * cos_p * sin_y + sin_r * sin_p * cos_y;

//	// 计算四元数
//	attitude->q0 = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y;
//	attitude->q1  = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y;
//	attitude->q2  = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y;
//	attitude->q3  = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y;

}

