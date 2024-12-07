#include "flash_proc.h"


osThreadId FlashTaskHandle;

#define EXTERN_FLASH 1

#define W25QXX_CS_L()  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET)
#define W25QXX_CS_H()  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET)



uint16_t W25QXX_TYPE = 0;
uint32_t W25QXX_SIZE = 0;
uint8_t  W25QXX_UID[8];

extern 

void Flash_Task_Proc(void const * argument)
{    
//    // ����Ҫд���float���鼰���Ӧ�ĵ�ַ
//    float testDataFloat[4] = {1.23f, 2.34f, 3.45f, 4.56f};
//    uint8_t writeBuffer[sizeof(float) * 4]; // ����д��Ļ�����
//    uint8_t readBuffer[sizeof(float) * 4];  // ���ڶ�ȡ�Ļ�����
//    float readDataFloat[4];
//    uint32_t writeAddress = 0x0000; // д����ʼ��ַ

//    // ��float����ת��Ϊ�ֽ������Ա�д��
//    memcpy(writeBuffer, testDataFloat, sizeof(testDataFloat));

    for(;;)
    {
//        printf("flash_size:%d\r\n", W25QXX_SIZE);
//        printf("flash_id:%d\r\n", W25QXX_TYPE);

//        // ����д��float���ݵ�ָ����ַ
//        W25QXX_Write(writeBuffer, writeAddress, sizeof(writeBuffer));
//        printf("Float data written to address: 0x%X\r\n", writeAddress);

//        // ��ָ����ַ��ȡ���ݲ�ת����float����
//        W25QXX_Read(readBuffer, writeAddress, sizeof(readBuffer));
//        memcpy(readDataFloat, readBuffer, sizeof(readDataFloat));

//        printf("Float data read from address: 0x%X - ", writeAddress);
//        for (int i = 0; i < 4; ++i) {
//            printf("%f ", readDataFloat[i]);
//        }
//        printf("\r\n");

//        // ���д��Ͷ�ȡ�������Ƿ�һ��
//        if (memcmp(writeBuffer, readBuffer, sizeof(writeBuffer)) == 0) {
//            printf("Test passed.\r\n");
//        } else {
//            printf("Test failed.\r\n");
//        }
        osDelay(2000); // �ȴ�2���ٽ�����һ��ѭ��
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

	
//���ɿ�IMU���� ��Ҫռ��0x000 - 0x199
void UAV_Read_Param_IMU(_imuData_all imu_data)
{




}

//���ɿ�ң�������� ��Ҫռ��0x200 - 0x399
void UAV_Read_Param_Remote(_sbus_ch_struct channel_data)
{




}	


//���ɿص������  ��Ҫռ��0x400 - 0x599
void UAV_Read_Param_Motor(_uav_control_data motor_data)
{




}


//д�ɿ�IMU���� ��Ҫռ��0x000 - 0x199
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
 
 W25QXX_Write((u8 *)temp_imu_write, 0x000, sizeof(temp_imu_write));
}	


//д�ɿ�ң�������� ��Ҫռ��0x200 - 0x399
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

 W25QXX_Write((u8 *)temp_remote_write, 0x200, sizeof(temp_remote_write));
}	


//д�ɿص������  ��Ҫռ��0x400 - 0x599
void UAV_Write_Param_Motor(_uav_control_data motor_data)
{
	float temp_motor_write[9] = {0};

	temp_motor_write[0] = motor_data.rollData.Kp;
	temp_motor_write[1] = motor_data.rollData.Ki;
	temp_motor_write[2] = motor_data.rollData.Kd;
	
	temp_motor_write[3] = motor_data.rollData.Kp;
	temp_motor_write[4] = motor_data.rollData.Ki;
	temp_motor_write[5] = motor_data.rollData.Kd;
	
	temp_motor_write[6] = motor_data.rollData.Kp;
	temp_motor_write[7] = motor_data.rollData.Ki;
	temp_motor_write[8] = motor_data.rollData.Kd;
	
	W25QXX_Write((u8 *)temp_motor_write, 0x00, sizeof(temp_motor_write));
}

//SPI��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
static uint8_t W25QXX_SPI_ReadWriteByte(uint8_t TxData)
{
    uint8_t RxData = 0X00;
    if(HAL_SPI_TransmitReceive(&hspi1, &TxData, &RxData, 1, 10) != HAL_OK)
    {
        RxData = 0XFF;
    }
    return RxData;
}

//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q128
//����Ϊ16M�ֽ�,����128��Block,4096��Sector

//��ʼ��SPI FLASH��IO��
int W25QXX_Init(void)
{
    MX_SPI1_Init();
    W25QXX_CS_L(); /* ����ѡ�� */
    W25QXX_SPI_ReadWriteByte(0XFF);
    W25QXX_CS_H(); /* ����ȡ�� */
    W25QXX_TYPE = W25QXX_ReadID();          // ��ȡFLASH ID.
    W25QXX_SIZE = W25QXX_ReadCapacity();    // ��ȡ����
    W25QXX_ReadUniqueID(W25QXX_UID);        // ��ȡΨһID
    if((W25QXX_TYPE & 0XEF00) != 0XEF00)
    {
        return -1;
    }
    return 0;
}

//��ȡW25QXX��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
uint8_t W25QXX_ReadSR(void)
{
    uint8_t byte = 0;
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_ReadStatusReg); //���Ͷ�ȡ״̬�Ĵ�������
    byte = W25QXX_SPI_ReadWriteByte(0Xff);          //��ȡһ���ֽ�
    W25QXX_CS_H();  //ȡ��Ƭѡ
    return byte;
}
//дW25QXX״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void W25QXX_Write_SR(uint8_t sr)
{
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_WriteStatusReg);                 //����дȡ״̬�Ĵ�������
    W25QXX_SPI_ReadWriteByte(sr);                   //д��һ���ֽ�
    W25QXX_CS_H();  //ȡ��Ƭѡ
}
//W25QXXдʹ��
//��WEL��λ
void W25QXX_Write_Enable(void)
{
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_WriteEnable);     //����дʹ��
    W25QXX_CS_H();  //ȡ��Ƭѡ
}
//W25QXXд��ֹ
//��WEL����
void W25QXX_Write_Disable(void)
{
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_WriteDisable);  //����д��ָֹ��
    W25QXX_CS_H();  //ȡ��Ƭѡ
}
//��ȡоƬID
//����ֵ����:
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128
uint16_t W25QXX_ReadID(void)
{
    uint16_t Temp = 0;
    W25QXX_CS_L();
    W25QXX_SPI_ReadWriteByte(0x90);                            //���Ͷ�ȡID����
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

//��ȡSPI FLASH
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    uint16_t i;
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_ReadData);            //���Ͷ�ȡ����
    W25QXX_SPI_ReadWriteByte((uint8_t)((ReadAddr) >> 16));    //����24bit��ַ
    W25QXX_SPI_ReadWriteByte((uint8_t)((ReadAddr) >> 8));
    W25QXX_SPI_ReadWriteByte((uint8_t)ReadAddr);
    for (i = 0; i < NumByteToRead; i++)
    {
        pBuffer[i] = W25QXX_SPI_ReadWriteByte(0XFF);    //ѭ������
    }
    W25QXX_CS_H();
}
//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!
void W25QXX_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t i;
    W25QXX_Write_Enable();                      //SET WEL
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_PageProgram);         //����дҳ����
    W25QXX_SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 16));   //����24bit��ַ
    W25QXX_SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
    W25QXX_SPI_ReadWriteByte((uint8_t)WriteAddr);
    for (i = 0; i < NumByteToWrite; i++)
        W25QXX_SPI_ReadWriteByte(pBuffer[i]); //ѭ��д��
    W25QXX_CS_H();  //ȡ��Ƭѡ
    W25QXX_Wait_Busy();                         //�ȴ�д�����
}
//�޼���дSPI FLASH
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ����
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint16_t pageremain;
    pageremain = 256 - WriteAddr % 256; //��ҳʣ����ֽ���
    if (NumByteToWrite <= pageremain)
        pageremain = NumByteToWrite; //������256���ֽ�
    while (1)
    {
        W25QXX_Write_Page(pBuffer, WriteAddr, pageremain);
        if (NumByteToWrite == pageremain)
            break; //д�������
        else //NumByteToWrite>pageremain
        {
            pBuffer += pageremain;
            WriteAddr += pageremain;

            NumByteToWrite -= pageremain;             //��ȥ�Ѿ�д���˵��ֽ���
            if (NumByteToWrite > 256)
                pageremain = 256; //һ�ο���д��256���ֽ�
            else
                pageremain = NumByteToWrite;      //����256���ֽ���
        }
    };
}
//дSPI FLASH
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
uint8_t W25QXX_BUFFER[4096];
void W25QXX_Write(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t *W25QXX_BUF;
    W25QXX_BUF = W25QXX_BUFFER;
    secpos = WriteAddr / 4096;    //������ַ
    secoff = WriteAddr % 4096;    //�������ڵ�ƫ��
    secremain = 4096 - secoff;    //����ʣ��ռ��С
    if (NumByteToWrite <= secremain)
        secremain = NumByteToWrite;       //������4096���ֽ�
    while (1)
    {
        W25QXX_Read(W25QXX_BUF, secpos * 4096, 4096);     //������������������
        for (i = 0; i < secremain; i++) //У������
        {
            if (W25QXX_BUF[secoff + i] != 0XFF)
                break; //��Ҫ����
        }
        if (i < secremain) //��Ҫ����
        {
            W25QXX_Erase_Sector(secpos);        //�����������
            for (i = 0; i < secremain; i++)          //����
            {
                W25QXX_BUF[i + secoff] = pBuffer[i];
            }
            W25QXX_Write_NoCheck(W25QXX_BUF, secpos * 4096, 4096);      //д����������

        }
        else
            W25QXX_Write_NoCheck(pBuffer, WriteAddr, secremain); //д�Ѿ������˵�,ֱ��д������ʣ������.
        if (NumByteToWrite == secremain)
            break; //д�������
        else //д��δ����
        {
            secpos++; //������ַ��1
            secoff = 0; //ƫ��λ��Ϊ0

            pBuffer += secremain;               //ָ��ƫ��
            WriteAddr += secremain;             //д��ַƫ��
            NumByteToWrite -= secremain;            //�ֽ����ݼ�
            if (NumByteToWrite > 4096)
                secremain = 4096;           //��һ����������д����
            else
                secremain = NumByteToWrite;     //��һ����������д����
        }
    };
}

//��������оƬ
//�ȴ�ʱ�䳬��...
void W25QXX_Erase_Chip(void)
{
    W25QXX_Write_Enable();                      //SET WEL
    W25QXX_Wait_Busy();
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_ChipErase);           //����Ƭ��������
    W25QXX_CS_H();  //ȡ��Ƭѡ
    W25QXX_Wait_Busy();                         //�ȴ�оƬ��������
}
//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void W25QXX_Erase_Sector(uint32_t Dst_Addr)
{
    //����falsh�������,������
    Dst_Addr *= 4096;
    W25QXX_Write_Enable();                      //SET WEL
    W25QXX_Wait_Busy();
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_SectorErase);         //������������ָ��
    W25QXX_SPI_ReadWriteByte((uint8_t)((Dst_Addr) >> 16));    //����24bit��ַ
    W25QXX_SPI_ReadWriteByte((uint8_t)((Dst_Addr) >> 8));
    W25QXX_SPI_ReadWriteByte((uint8_t)Dst_Addr);
    W25QXX_CS_H();  //ȡ��Ƭѡ
    W25QXX_Wait_Busy();                         //�ȴ��������
}
//�ȴ�����
void W25QXX_Wait_Busy(void)
{
    while ((W25QXX_ReadSR() & 0x01) == 0x01);       // �ȴ�BUSYλ���
}
//�������ģʽ
void W25QXX_PowerDown(void)
{
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_PowerDown);        //���͵�������
    W25QXX_CS_H();  //ȡ��Ƭѡ
    delay_us(3);                               //�ȴ�TPD
}
//����
void W25QXX_WAKEUP(void)
{
    W25QXX_CS_L(); //ʹ������
    W25QXX_SPI_ReadWriteByte(W25X_ReleasePowerDown); //  send W25X_PowerDown command 0xAB
    W25QXX_CS_H();  //ȡ��Ƭѡ
    delay_us(3);  







}


