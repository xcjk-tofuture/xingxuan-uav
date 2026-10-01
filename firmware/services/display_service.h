#ifndef UAV_DISPLAY_SERVICE_H
#define UAV_DISPLAY_SERVICE_H
#include <stdint.h>
/* Task context; requests coalesce, only the OLED task writes page state. */
void uav_display_request_page(uint8_t page);
void uav_display_next_page(void);
#endif
