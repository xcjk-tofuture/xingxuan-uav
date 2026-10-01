#ifndef UAV_LOG_SERVICE_H
#define UAV_LOG_SERVICE_H
#include <stdint.h>
#include <stddef.h>
/* Task-context API. Init before task creation. Emit never blocks and drops on full.
 * One low-priority consumer owns UART3; startup logs before init are discarded. */
int uav_log_init(void);
void uav_log_byte(uint8_t byte);
size_t uav_log_receive(uint8_t *bytes,size_t capacity,uint32_t timeout_ms);
#endif
