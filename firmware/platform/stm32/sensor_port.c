#include "sensor_port.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "uav_board.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
void uav_sensor_select(unsigned device,int selected) {
    GPIO_TypeDef *port=device==0?BMI088_ACC_GPIOx:device==1?BMI088_GYRO_GPIOx:device==2?SPI2_CS3_GPIO_Port:SPI2_CS2_GPIO_Port;
    uint16_t pin=device==0?BMI088_ACC_GPIOp:device==1?BMI088_GYRO_GPIOp:device==2?SPI2_CS3_Pin:SPI2_CS2_Pin;
    HAL_GPIO_WritePin(port,pin,selected?GPIO_PIN_RESET:GPIO_PIN_SET);
}
void uav_sensor_tx(const uint8_t *bytes,uint16_t size) {
    if(HAL_SPI_Transmit(&hspi2,(uint8_t *)bytes,size,2)!=HAL_OK)Error_Handler();
}
void uav_sensor_rx(uint8_t *bytes,uint16_t size) {
    if(HAL_SPI_Receive(&hspi2,bytes,size,2)!=HAL_OK)Error_Handler();
}
uint8_t uav_sensor_byte(uint8_t byte) {
    uint8_t response;
    if(HAL_SPI_TransmitReceive(&hspi2,&byte,&response,1,2)!=HAL_OK)Error_Handler();
    return response;
}
void uav_sensor_delay_ms(uint32_t ms) {
    if(xTaskGetSchedulerState()==taskSCHEDULER_NOT_STARTED)HAL_Delay(ms);else osDelay(ms);
}
void uav_device_key_scan(uint8_t *key) {
    *key=HAL_GPIO_ReadPin(UAV_KEY_PORT,UAV_KEY1_PIN)==GPIO_PIN_RESET?1:HAL_GPIO_ReadPin(UAV_KEY_PORT,UAV_KEY2_PIN)==GPIO_PIN_RESET?2:0;
}
void uav_device_led_write(uint8_t bits) {
    HAL_GPIO_WritePin(UAV_LED_PORT,RGB_R_Pin,(bits&4)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(UAV_LED_PORT,RGB_G_Pin,(bits&2)?GPIO_PIN_RESET:GPIO_PIN_SET);
    HAL_GPIO_WritePin(UAV_LED_PORT,RGB_B_Pin,(bits&1)?GPIO_PIN_RESET:GPIO_PIN_SET);
}
void uav_device_heater_init(void) {
    if(HAL_TIM_PWM_Start(&UAV_HEATER_TIMER,TIM_CHANNEL_1)!=HAL_OK)Error_Handler();
    __HAL_TIM_SET_AUTORELOAD(&UAV_HEATER_TIMER,999);__HAL_TIM_SET_COMPARE(&UAV_HEATER_TIMER,TIM_CHANNEL_1,0);
}
void uav_device_heater_write(uint16_t value) {__HAL_TIM_SET_COMPARE(&UAV_HEATER_TIMER,TIM_CHANNEL_1,value>999?999:value);}
void uav_device_delay_us(uint32_t us) {
    volatile uint32_t count=(HAL_RCC_GetHCLKFreq()/4000000)*us;
    while(count--){} /* Same short device wake delays as the original driver. */
}
