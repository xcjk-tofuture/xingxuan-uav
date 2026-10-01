#ifndef UAV_LOG_PORT_H
#define UAV_LOG_PORT_H
#include <stdint.h>
#include <stddef.h>
int uav_log_port_send(const uint8_t *bytes, size_t length);
#endif
