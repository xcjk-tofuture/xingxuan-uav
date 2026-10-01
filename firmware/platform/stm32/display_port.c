#include "display_port.h"
#include "shared_spi.h"
#include "main.h"
#include "spi.h"
#include "cmsis_os.h"
void uav_display_port_pin(unsigned pin,int high) {
    if(pin==2 && !high)uav_spi1_lock();
    GPIO_TypeDef *port=pin==0?OLED_RES_GPIO_Port:pin==1?OLED_DC_GPIO_Port:OLED_CS_GPIO_Port;
    uint16_t bit=pin==0?OLED_RES_Pin:pin==1?OLED_DC_Pin:OLED_CS_Pin;
    HAL_GPIO_WritePin(port,bit,high?GPIO_PIN_SET:GPIO_PIN_RESET);
    if(pin==2 && high)uav_spi1_unlock();
}
void uav_display_port_byte(uint8_t byte) {if(HAL_SPI_Transmit(&hspi1,&byte,1,10)!=HAL_OK)Error_Handler();}
void uav_display_port_delay(uint32_t ms) {osDelay(ms);}
