#include "shared_spi.h"
#include "main.h"
#include "spi.h"
void uav_flash_select(int selected) {
    if(selected)uav_spi1_lock();
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port,FLASH_CS_Pin,selected?GPIO_PIN_RESET:GPIO_PIN_SET);
    if(!selected)uav_spi1_unlock();
}
uint8_t uav_flash_byte(uint8_t byte) {
    uint8_t response;
    if(HAL_SPI_TransmitReceive(&hspi1,&byte,&response,1,10)!=HAL_OK)Error_Handler();
    return response;
}
