#include "uav_actuator.h"
void uav_pwm_hal_write(unsigned motor, int pulse_us);
void uav_pwm_hal_stop(void);
void uav_actuator_write(unsigned motor, int pulse_us) { uav_pwm_hal_write(motor, pulse_us); }
void uav_actuator_stop(void) { uav_pwm_hal_stop(); }
