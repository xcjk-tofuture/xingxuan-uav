#include "AK8975.h"


#define GPIO_ResetBits() HAL_GPIO_WritePin(SPI2_CS3_GPIO_Port, SPI2_CS3_Pin, GPIO_PIN_RESET)
#define GPIO_SetBits()   HAL_GPIO_WritePin(SPI2_CS3_GPIO_Port, SPI2_CS3_Pin, GPIO_PIN_SET)

#define MAG_ROTATION ROTATION_NONE

s16 ADCMAG[3];
static uint8_t ak8975_buf[6];


u8 Drv_SPI2_RW(u8 dat)
{
 return SPI_SingleWirteAndRead(2,dat);
}	


static void ak8975_enable(u8 ena)
{
	if(ena)
		GPIO_ResetBits();
	else
		GPIO_SetBits();
}

u8 DrvAK8975Check(void)
{
	u8 _tmp;
	ak8975_enable(1);
	Drv_SPI2_RW(AK8975_WIA_REG|0x80);
	_tmp = Drv_SPI2_RW(0xFF);
	ak8975_enable(0);
	if(_tmp == 0x48)
		return 0;
	else
		return 1;
}

static void ak8975_Trig(void)
{
	ak8975_enable(1);
	Drv_SPI2_RW(AK8975_CNTL_REG);
	Drv_SPI2_RW(0x01);
	ak8975_enable(0);
}

void DrvAk8975Read(void)
{	
	
	ak8975_enable(1);
	Drv_SPI2_RW(AK8975_HXL_REG|0x80);
	for(u8 i=0; i<6; i++)
		ak8975_buf[i] = Drv_SPI2_RW(0xff);
	ak8975_enable(0);
	
	ak8975_Trig();
	
}

//拓空者PRO-088
void ReadMagData(mag_raw_data_t* Mag)
{
	
	DrvAk8975Read();
	s16 tmp[3];
	
	tmp[0] = ((((int16_t)ak8975_buf[1]) << 8) | ak8975_buf[0]) ;
	tmp[1] = ((((int16_t)ak8975_buf[3]) << 8) | ak8975_buf[2]) ;
	tmp[2] = ((((int16_t)ak8975_buf[5]) << 8) | ak8975_buf[4]) ;
	
	/*转换坐标*/
	Mag->x = +tmp[0] * 0.3;  //单位uT
	Mag->y = -tmp[1] * 0.3;
	Mag->z = -tmp[2] * 0.3;


	
}

