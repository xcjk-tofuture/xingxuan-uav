#ifndef UAV_DISPLAY_PORT_H
#define UAV_DISPLAY_PORT_H
#include <stdint.h>
void uav_display_port_pin(unsigned pin,int high);
void uav_display_port_byte(uint8_t byte);
void uav_display_port_delay(uint32_t ms);
#define OLED_RES_Clr() uav_display_port_pin(0,0)
#define OLED_RES_Set() uav_display_port_pin(0,1)
#define OLED_DC_Clr() uav_display_port_pin(1,0)
#define OLED_DC_Set() uav_display_port_pin(1,1)
#define OLED_CS_Clr() uav_display_port_pin(2,0)
#define OLED_CS_Set() uav_display_port_pin(2,1)
#endif
