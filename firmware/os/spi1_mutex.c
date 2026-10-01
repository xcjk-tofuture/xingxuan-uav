#include "shared_spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
extern void Error_Handler(void);
static SemaphoreHandle_t mutex;
int uav_spi1_init(void) {
    mutex = xSemaphoreCreateRecursiveMutex();
    return mutex ? 0 : -1;
}
void uav_spi1_lock(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
        return;
    if (!mutex || xSemaphoreTakeRecursive(mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
        Error_Handler();
}
void uav_spi1_unlock(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        xSemaphoreGiveRecursive(mutex);
}
void uav_flash_wait_pause(void) {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        vTaskDelay(pdMS_TO_TICKS(1));
}
