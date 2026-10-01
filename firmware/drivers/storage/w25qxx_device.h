#ifndef UAV_W25QXX_DEVICE_H
#define UAV_W25QXX_DEVICE_H
#include <stdint.h>
#include "shared_spi.h"


//W25XÏµÁÐ/QÏµÁÐÐ¾Æ¬ÁÐ±í
//W25Q80  ID  0XEF13
//W25Q16  ID  0XEF14
//W25Q32  ID  0XEF15
//W25Q64  ID  0XEF16
//W25Q128 ID  0XEF17
#define W25Q80  0XEF13
#define W25Q16  0XEF14
#define W25Q32  0XEF15
#define W25Q64  0XEF16
#define W25Q128 0XEF17



#define W25QXX_CS_L() uav_flash_select(1)
#define W25QXX_CS_H() uav_flash_select(0)





//
//Ö¸Áî±í
#define W25X_WriteEnable    0x06
#define W25X_WriteDisable   0x04
#define W25X_ReadStatusReg    0x05
#define W25X_WriteStatusReg   0x01
#define W25X_ReadData     0x03
#define W25X_FastReadData   0x0B
#define W25X_FastReadDual   0x3B
#define W25X_PageProgram    0x02
#define W25X_BlockErase     0xD8
#define W25X_SectorErase    0x20
#define W25X_ChipErase      0xC7
#define W25X_PowerDown      0xB9
#define W25X_ReleasePowerDown 0xAB
#define W25X_DeviceID     0xAB
#define W25X_ManufactDeviceID 0x90
#define W25X_JedecDeviceID    0x9F

int W25QXX_Init(void);
void W25QXX_ReadUniqueID(uint8_t UID[8]);
uint16_t  W25QXX_ReadID(void);            //¶ÁÈ¡FLASH ID
uint8_t  W25QXX_ReadSR(void);           //¶ÁÈ¡×´Ì¬¼Ä´æÆ÷
void W25QXX_Write_SR(uint8_t sr);       //Ð´×´Ì¬¼Ä´æÆ÷
void W25QXX_Write_Enable(void);     //Ð´Ê¹ÄÜ
void W25QXX_Write_Disable(void);    //Ð´±£»¤
void W25QXX_Write_NoCheck(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);
void W25QXX_Read(uint8_t* pBuffer,uint32_t ReadAddr,uint16_t NumByteToRead);   //¶ÁÈ¡flash
void W25QXX_Write(uint8_t* pBuffer,uint32_t WriteAddr,uint16_t NumByteToWrite);//Ð´Èëflash
void W25QXX_Erase_Chip(void);         //ÕûÆ¬²Á³ý
void W25QXX_Erase_Sector(uint32_t Dst_Addr);  //ÉÈÇø²Á³ý
void W25QXX_Wait_Busy(void);            //µÈ´ý¿ÕÏÐ
void W25QXX_PowerDown(void);          //½øÈëµôµçÄ£Ê½
void W25QXX_WAKEUP(void);       //»½ÐÑ
uint32_t W25QXX_ReadCapacity(void);





#endif
