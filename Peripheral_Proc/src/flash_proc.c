#include "flash_proc.h"


osThreadId FlashTaskHandle;

#define EXTERN_FLASH 1

#define FLASH_IMU_ADDR 0X000
#define FLASH_REMOTE_ADDR 0X200
#define FLASH_MOTOR_ADDR 0X400


uint16_t W25QXX_TYPE = 0;
uint32_t W25QXX_SIZE = 0;
uint8_t  W25QXX_UID[8];




//extern _uav_control_data uav_control_data;
//_uav_control_data uav_control_test_data;

void Flash_Task_Proc(void const * argument)
{    
//    // 定义要写入的float数组及其对应的地址
//    float testDataFloat[4] = {1.23f, 2.34f, 3.45f, 4.56f};
//    uint8_t writeBuffer[sizeof(float) * 4]; // 用于写入的缓冲区
//    uint8_t readBuffer[sizeof(float) * 4];  // 用于读取的缓冲区
//    float readDataFloat[4];
//    uint32_t writeAddress = 0x0000; // 写入起始地址

//    // 将float数据转换为字节数组以便写入
//    memcpy(writeBuffer, testDataFloat, sizeof(testDataFloat));
    for(;;)
    {
			
			
//        printf("flash_size:%d\r\n", W25QXX_SIZE);
//        printf("flash_id:%d\r\n", W25QXX_TYPE);

//        // 测试写入float数据到指定地址
//        W25QXX_Write(writeBuffer, writeAddress, sizeof(writeBuffer));
//        printf("Float data written to address: 0x%X\r\n", writeAddress);

//        // 从指定地址读取数据并转换回float类型
//        W25QXX_Read(readBuffer, writeAddress, sizeof(readBuffer));
//        memcpy(readDataFloat, readBuffer, sizeof(readDataFloat));

//        printf("Float data read from address: 0x%X - ", writeAddress);
//        for (int i = 0; i < 4; ++i) {
//            printf("%f ", readDataFloat[i]);
//        }
//        printf("\r\n");

//        // 检查写入和读取的数据是否一致
//        if (memcmp(writeBuffer, readBuffer, sizeof(writeBuffer)) == 0) {
//            printf("Test passed.\r\n");
//        } else {
//            printf("Test failed.\r\n");
//        }
//				UAV_Write_Param_Motor(uav_control_data);
//				UAV_Read_Param_Motor(&uav_control_test_data);


        osDelay(2000); // 等待2秒再进行下一次循环
    }
}

static void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
    {
        ;
    }
}

	
//读飞控IMU数据 主要占据0x000 - 0x199
void UAV_Read_Param_IMU(_imuData_all* imu_data)
{
	u8 temp_imu_read[18 * 4] = {0};
	float temp_imu_read_value[18] = {0};
	W25QXX_Read(temp_imu_read, FLASH_IMU_ADDR, sizeof(temp_imu_read));
	memcpy(temp_imu_read_value, temp_imu_read, sizeof(temp_imu_read));
	
//	imu_data->accoffsetbias.x = temp_imu_read_value[0];
//	imu_data->accoffsetbias.y = temp_imu_read_value[1];
//	imu_data->accoffsetbias.z = temp_imu_read_value[2];
//	imu_data->accscalebias.x = temp_imu_read_value[3];
//	imu_data->accscalebias.y = temp_imu_read_value[4];
//	imu_data->accscalebias.z = temp_imu_read_value[5];
//	
//	
//  imu_data->gyrooffsetbias.x = temp_imu_read_value[6];
//	imu_data->gyrooffsetbias.y = temp_imu_read_value[7];
//	imu_data->gyrooffsetbias.z = temp_imu_read_value[8];
//	imu_data->gyroscalebias.x = temp_imu_read_value[9];
//	imu_data->gyroscalebias.y = temp_imu_read_value[10];
//	imu_data->gyroscalebias.z = temp_imu_read_value[11];
	
	imu_data->magoffsetbias.x = temp_imu_read_value[12];
	imu_data->magoffsetbias.y = temp_imu_read_value[13];
	imu_data->magoffsetbias.z = temp_imu_read_value[14];
	imu_data->magscalebias.x = temp_imu_read_value[15];
	imu_data->magscalebias.y = temp_imu_read_value[16];
	imu_data->magscalebias.z = temp_imu_read_value[17];

}

//读飞控遥控器数据 主要占据0x200 - 0x399
void UAV_Read_Param_Remote(_sbus_ch_struct* channel_data)
{
	u8 temp_remote_read[8 * 2 * 2] = {0};
	u16 temp_remote_read_value[8 * 2] = {0};
	W25QXX_Read(temp_remote_read, FLASH_REMOTE_ADDR, sizeof(temp_remote_read));
	memcpy(temp_remote_read_value, temp_remote_read, sizeof(temp_remote_read));

	channel_data->CH1_MAX = temp_remote_read_value[0];
	channel_data->CH1_MIN = temp_remote_read_value[1];
	
	channel_data->CH2_MAX = temp_remote_read_value[2];
	channel_data->CH2_MIN = temp_remote_read_value[3];
	
	channel_data->CH3_MAX = temp_remote_read_value[4];
	channel_data->CH3_MIN = temp_remote_read_value[5];
	
	channel_data->CH4_MAX = temp_remote_read_value[6];
	channel_data->CH4_MIN = temp_remote_read_value[7];
	
	channel_data->CH5_MAX = temp_remote_read_value[8];
	channel_data->CH5_MIN = temp_remote_read_value[9];
	
	channel_data->CH6_MAX = temp_remote_read_value[10];
	channel_data->CH6_MIN = temp_remote_read_value[11];
	
	channel_data->CH7_MAX = temp_remote_read_value[12];
	channel_data->CH7_MIN = temp_remote_read_value[13];
	
	channel_data->CH8_MAX = temp_remote_read_value[14];
	channel_data->CH8_MIN = temp_remote_read_value[15];
}	




//读飞控电机数据  主要占据0x400 - 0x599
void UAV_Read_Param_Motor(_uav_control_data* motor_data)
{

 u8 temp_motor_read[5 * 3 * 4] = {0};
 float temp_motor_read_value[15] = {0};
 W25QXX_Read(temp_motor_read, FLASH_MOTOR_ADDR, sizeof(temp_motor_read));
 memcpy(temp_motor_read_value, temp_motor_read, sizeof(temp_motor_read));
 
 motor_data->rollData.Kp = temp_motor_read_value[0];
 motor_data->rollData.Ki = temp_motor_read_value[1];
 motor_data->rollData.Kd = temp_motor_read_value[2];
 
 motor_data->pitchData.Kp = temp_motor_read_value[3];
 motor_data->pitchData.Ki = temp_motor_read_value[4];
 motor_data->pitchData.Kd = temp_motor_read_value[5];
 
 motor_data->yawData.Kp = temp_motor_read_value[6];
 motor_data->yawData.Ki = temp_motor_read_value[7];
 motor_data->yawData.Kd = temp_motor_read_value[8];
 
 motor_data->rollSpeedData.Kp = temp_motor_read_value[9];
 motor_data->rollSpeedData.Ki = temp_motor_read_value[10];
 motor_data->rollSpeedData.Kd = temp_motor_read_value[11];
 
 motor_data->pitchSpeedData.Kp = temp_motor_read_value[12];
 motor_data->pitchSpeedData.Ki = temp_motor_read_value[13];
 motor_data->pitchSpeedData.Kd = temp_motor_read_value[14];
 
 
}


//写飞控IMU数据 主要占据0x000 - 0x199
void UAV_Write_Param_IMU(_imuData_all imu_data)
{
 float temp_imu_write[18] = {0};
 temp_imu_write[0] = imu_data.accoffsetbias.x;
 temp_imu_write[1] = imu_data.accoffsetbias.y;
 temp_imu_write[2] = imu_data.accoffsetbias.z;
 temp_imu_write[3] = imu_data.accscalebias.x;
 temp_imu_write[4] = imu_data.accscalebias.y;
 temp_imu_write[5] = imu_data.accscalebias.z;
 
 temp_imu_write[6] = imu_data.gyrooffsetbias.x;
 temp_imu_write[7] = imu_data.gyrooffsetbias.y;
 temp_imu_write[8] = imu_data.gyrooffsetbias.z;
 temp_imu_write[9] = imu_data.gyroscalebias.x;
 temp_imu_write[10] = imu_data.gyroscalebias.y;
 temp_imu_write[11] = imu_data.gyroscalebias.z;
 
 temp_imu_write[12] = imu_data.magoffsetbias.x;
 temp_imu_write[13] = imu_data.magoffsetbias.y;
 temp_imu_write[14] = imu_data.magoffsetbias.z;
 temp_imu_write[15] = imu_data.magscalebias.x;
 temp_imu_write[16] = imu_data.magscalebias.y;
 temp_imu_write[17] = imu_data.magscalebias.z;
 
 W25QXX_Write((u8 *)temp_imu_write, FLASH_IMU_ADDR, sizeof(temp_imu_write));
}	


//写飞控遥控器数据 主要占据0x200 - 0x399
void UAV_Write_Param_Remote(_sbus_ch_struct channe_data)
{
 uint16_t temp_remote_write[16] = {0};
 temp_remote_write[0] =  channe_data.CH1_MAX;
 temp_remote_write[1] =  channe_data.CH1_MIN;
 
 temp_remote_write[2] =  channe_data.CH2_MAX;
 temp_remote_write[3] =  channe_data.CH2_MIN;
 
 temp_remote_write[4] =  channe_data.CH3_MAX;
 temp_remote_write[5] =  channe_data.CH3_MIN;
 
 temp_remote_write[6] =  channe_data.CH4_MAX;
 temp_remote_write[7] =  channe_data.CH4_MIN;
 
 temp_remote_write[8] =  channe_data.CH5_MAX;
 temp_remote_write[9] =  channe_data.CH5_MIN;
 
 temp_remote_write[10] =  channe_data.CH6_MAX;
 temp_remote_write[11] =  channe_data.CH6_MIN;
 
 temp_remote_write[12] =  channe_data.CH7_MAX;
 temp_remote_write[13] =  channe_data.CH7_MIN;
 
 temp_remote_write[14] =  channe_data.CH8_MAX;
 temp_remote_write[15] =  channe_data.CH8_MIN;

 W25QXX_Write((u8 *)temp_remote_write, FLASH_REMOTE_ADDR, sizeof(temp_remote_write));
}	


//写飞控电机数据  主要占据0x400 - 0x599
void UAV_Write_Param_Motor(_uav_control_data motor_data)
{
	float temp_motor_write[15] = {0};

	temp_motor_write[0] = motor_data.rollData.Kp;
	temp_motor_write[1] = motor_data.rollData.Ki;
	temp_motor_write[2] = motor_data.rollData.Kd;
	
	temp_motor_write[3] = motor_data.pitchData.Kp;
	temp_motor_write[4] = motor_data.pitchData.Ki;
	temp_motor_write[5] = motor_data.pitchData.Kd;
	
	temp_motor_write[6] = motor_data.yawData.Kp;
	temp_motor_write[7] = motor_data.yawData.Ki;
	temp_motor_write[8] = motor_data.yawData.Kd;
	
	temp_motor_write[9] = motor_data.rollSpeedData.Kp;
	temp_motor_write[10] = motor_data.rollSpeedData.Ki;
	temp_motor_write[11] = motor_data.rollSpeedData.Kd;
	
	temp_motor_write[12] = motor_data.pitchSpeedData.Kp;
	temp_motor_write[13] = motor_data.pitchSpeedData.Ki;
	temp_motor_write[14] = motor_data.pitchSpeedData.Kd;
	
	W25QXX_Write((u8 *)temp_motor_write, FLASH_MOTOR_ADDR, sizeof(temp_motor_write));
}

//SPI读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
static uint8_t W25QXX_SPI_ReadWriteByte(uint8_t TxData)
{
    uint8_t RxData = 0X00;
    if(HAL_SPI_TransmitReceive(&hspi1, &TxData, &RxData, 1, 10) != HAL_OK)
    {
        RxData = 0XFF;
    }
    return RxData;
}

//4Kbytes为一个Sector
//16个扇区为1个Block
//W25Q128
//容量为16M字节,共有128个Block,4096个Sector

//初始化SPI FLASH的IO口
int W25QXX_Init(void)
{
    MX_SPI1_Init();
    W25QXX_CS_L(); /* 拉低选中 */
    W25QXX_SPI_ReadWriteByte(0XFF);
    W25QXX_CS_H(); /* 拉高取消 */
    W25QXX_TYPE = W25QXX_ReadID();          // 读取FLASH ID.
    W25QXX_SIZE = W25QXX_ReadCapacity();    // 读取容量
    W25QXX_ReadUniqueID(W25QXX_UID);        // 读取唯一ID
    if((W25QXX_TYPE & 0XEF00) != 0XEF00)
    {
        return -1;
    }
    return 0;
}

//读取W25QXX的状态寄存器
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
uint8_t W25QXX_ReadSR(void)
{
    uint8_t byte = 0;
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_ReadStatusReg); //发送读取状态寄存器命令
    byte = W25QXX_SPI_ReadWriteByte(0Xff);          //读取一个字节
    W25QXX_CS_H();  //取消片选
    return byte;
}
//写W25QXX状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void W25QXX_Write_SR(uint8_t sr)
{
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_WriteStatusReg);                 //发送写取状态寄存器命令
    W25QXX_SPI_ReadWriteByte(sr);                   //写入一个字节
    W25QXX_CS_H();  //取消片选
}
//W25QXX写使能
//将WEL置位
void W25QXX_Write_Enable(void)
{
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_WriteEnable);     //发送写使能
    W25QXX_CS_H();  //取消片选
}
//W25QXX写禁止
//将WEL清零
void W25QXX_Write_Disable(void)
{
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_WriteDisable);  //发送写禁止指令
    W25QXX_CS_H();  //取消片选
}
//读取芯片ID
//返回值如下:
//0XEF13,表示芯片型号为W25Q80
//0XEF14,表示芯片型号为W25Q16
//0XEF15,表示芯片型号为W25Q32
//0XEF16,表示芯片型号为W25Q64
//0XEF17,表示芯片型号为W25Q128
uint16_t W25QXX_ReadID(void)
{
    uint16_t Temp = 0;
    W25QXX_CS_L();
    W25QXX_SPI_ReadWriteByte(0x90);                            //发送读取ID命令
    W25QXX_SPI_ReadWriteByte(0x00);
    W25QXX_SPI_ReadWriteByte(0x00);
    W25QXX_SPI_ReadWriteByte(0x00);
    Temp |= W25QXX_SPI_ReadWriteByte(0xFF) << 8;
    Temp |= W25QXX_SPI_ReadWriteByte(0xFF);
    W25QXX_CS_H();
    return Temp;
}

uint32_t W25QXX_ReadCapacity(void)
{
    int i = 0;
    uint8_t arr[4] = {0,0,0,0};
    W25QXX_CS_L();
    W25QXX_SPI_ReadWriteByte(0x5A);
    W25QXX_SPI_ReadWriteByte(0x00);
    W25QXX_SPI_ReadWriteByte(0x00);
    W25QXX_SPI_ReadWriteByte(0x84);
    W25QXX_SPI_ReadWriteByte(0x00);
    for(i = 0; i < sizeof(arr); i++)
    {
        arr[i] = W25QXX_SPI_ReadWriteByte(0xFF);
    }
    W25QXX_CS_H();
    return ((((*(uint32_t *)arr)) + 1) >> 3);
}

void W25QXX_ReadUniqueID(uint8_t UID[8])
{
    int i = 0;
    W25QXX_CS_L();
    W25QXX_SPI_ReadWriteByte(0x4B);
    W25QXX_SPI_ReadWriteByte(0x00);
    W25QXX_SPI_ReadWriteByte(0x00);
    W25QXX_SPI_ReadWriteByte(0x00);
    W25QXX_SPI_ReadWriteByte(0x00);
    for(i = 0; i < 8; i++)
    {
        UID[i] = W25QXX_SPI_ReadWriteByte(0xFF);
    }
    W25QXX_CS_H();
}

//读取SPI FLASH
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void W25QXX_Read(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    uint16_t i;
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_ReadData);            //发送读取命令
    W25QXX_SPI_ReadWriteByte((uint8_t)((ReadAddr) >> 16));    //发送24bit地址
    W25QXX_SPI_ReadWriteByte((uint8_t)((ReadAddr) >> 8));
    W25QXX_SPI_ReadWriteByte((uint8_t)ReadAddr);
    for (i = 0; i < NumByteToRead; i++)
    {
        pBuffer[i] = W25QXX_SPI_ReadWriteByte(0XFF);    //循环读数
    }
    W25QXX_CS_H();
}
//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
void W25QXX_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t i;
    W25QXX_Write_Enable();                      //SET WEL
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_PageProgram);         //发送写页命令
    W25QXX_SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 16));   //发送24bit地址
    W25QXX_SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
    W25QXX_SPI_ReadWriteByte((uint8_t)WriteAddr);
    for (i = 0; i < NumByteToWrite; i++)
        W25QXX_SPI_ReadWriteByte(pBuffer[i]); //循环写数
    W25QXX_CS_H();  //取消片选
    W25QXX_Wait_Busy();                         //等待写入结束
}
//无检验写SPI FLASH
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;
    pageremain = 256 - WriteAddr % 256; //单页剩余的字节数
    if (NumByteToWrite <= pageremain)
        pageremain = NumByteToWrite; //不大于256个字节
    while (1)
    {
        W25QXX_Write_Page(pBuffer, WriteAddr, pageremain);
        if (NumByteToWrite == pageremain)
            break; //写入结束了
        else //NumByteToWrite>pageremain
        {
            pBuffer += pageremain;
            WriteAddr += pageremain;

            NumByteToWrite -= pageremain;             //减去已经写入了的字节数
            if (NumByteToWrite > 256)
                pageremain = 256; //一次可以写入256个字节
            else
                pageremain = NumByteToWrite;      //不够256个字节了
        }
    };
}
//写SPI FLASH
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
uint8_t W25QXX_BUFFER[4096];
void W25QXX_Write(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t *W25QXX_BUF;
    W25QXX_BUF = W25QXX_BUFFER;
    secpos = WriteAddr / 4096;    //扇区地址
    secoff = WriteAddr % 4096;    //在扇区内的偏移
    secremain = 4096 - secoff;    //扇区剩余空间大小
    if (NumByteToWrite <= secremain)
        secremain = NumByteToWrite;       //不大于4096个字节
    while (1)
    {
        W25QXX_Read(W25QXX_BUF, secpos * 4096, 4096);     //读出整个扇区的内容
        for (i = 0; i < secremain; i++) //校验数据
        {
            if (W25QXX_BUF[secoff + i] != 0XFF)
                break; //需要擦除
        }
        if (i < secremain) //需要擦除
        {
            W25QXX_Erase_Sector(secpos);        //擦除这个扇区
            for (i = 0; i < secremain; i++)          //复制
            {
                W25QXX_BUF[i + secoff] = pBuffer[i];
            }
            W25QXX_Write_NoCheck(W25QXX_BUF, secpos * 4096, 4096);      //写入整个扇区

        }
        else
            W25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain); //写已经擦除了的,直接写入扇区剩余区间.
        if (NumByteToWrite == secremain)
            break; //写入结束了
        else //写入未结束
        {
            secpos++; //扇区地址增1
            secoff = 0; //偏移位置为0

            pBuffer += secremain;               //指针偏移
            WriteAddr += secremain;             //写地址偏移
            NumByteToWrite -= secremain;            //字节数递减
            if (NumByteToWrite > 4096)
                secremain = 4096;           //下一个扇区还是写不完
            else
                secremain = NumByteToWrite;     //下一个扇区可以写完了
        }
    };
}

//擦除整个芯片
//等待时间超长...
void W25QXX_Erase_Chip(void)
{
    W25QXX_Write_Enable();                      //SET WEL
    W25QXX_Wait_Busy();
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_ChipErase);           //发送片擦除命令
    W25QXX_CS_H();  //取消片选
    W25QXX_Wait_Busy();                         //等待芯片擦除结束
}
//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个山区的最少时间:150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)
{
    //监视falsh擦除情况,测试用
    Dst_Addr *= 4096;
    W25QXX_Write_Enable();                      //SET WEL
    W25QXX_Wait_Busy();
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_SectorErase);         //发送扇区擦除指令
    W25QXX_SPI_ReadWriteByte((uint8_t)((Dst_Addr) >> 16));    //发送24bit地址
    W25QXX_SPI_ReadWriteByte((uint8_t)((Dst_Addr) >> 8));
    W25QXX_SPI_ReadWriteByte((uint8_t)Dst_Addr);
    W25QXX_CS_H();  //取消片选
    W25QXX_Wait_Busy();                         //等待擦除完成
}
//等待空闲
void W25QXX_Wait_Busy(void)
{
    while ((W25QXX_ReadSR() & 0x01) == 0x01);       // 等待BUSY位清空
}
//进入掉电模式
void W25QXX_PowerDown(void)
{
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_PowerDown);        //发送掉电命令
    W25QXX_CS_H();  //取消片选
    delay_us(3);                               //等待TPD
}
//唤醒
void W25QXX_WAKEUP(void)
{
    W25QXX_CS_L(); //使能器件
    W25QXX_SPI_ReadWriteByte(W25X_ReleasePowerDown); //  send W25X_PowerDown command 0xAB
    W25QXX_CS_H();  //取消片选
    delay_us(3);  







}


