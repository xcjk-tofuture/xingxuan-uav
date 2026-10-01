#ifndef UAV_SHARED_SPI_H
#define UAV_SHARED_SPI_H
#include <stdint.h>
/* Task context; recursive lock has priority inheritance, 1000 ms wait bound.
 * Pre-scheduler boot calls need no lock. Any timeout invokes safe fatal handling. */
int uav_spi1_init(void);
void uav_spi1_lock(void);
void uav_spi1_unlock(void);
void uav_flash_select(int selected);
uint8_t uav_flash_byte(uint8_t byte);
void uav_flash_wait_pause(void);
#endif
