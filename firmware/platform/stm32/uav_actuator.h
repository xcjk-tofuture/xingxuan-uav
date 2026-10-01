#ifndef UAV_ACTUATOR_H
#define UAV_ACTUATOR_H
void uav_actuator_write(unsigned motor, int pulse_us);
void uav_actuator_stop(void);
#endif
