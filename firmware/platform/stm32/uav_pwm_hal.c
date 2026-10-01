#include "uav_actuator.h"
#include "tim.h"
#include "uav_board.h"
static const uint32_t channels[4] = UAV_PWM_CHANNELS;
void uav_pwm_hal_write(unsigned motor, int pulse) {
    if (motor >= 4)
        return;
    if (pulse < UAV_PWM_STOP_US)
        pulse = UAV_PWM_STOP_US;
    if (pulse > UAV_PWM_MAX_US)
        pulse = UAV_PWM_MAX_US;
    __HAL_TIM_SET_COMPARE(&UAV_PWM_TIMER, channels[motor], (uint32_t)pulse);
}
void uav_pwm_hal_stop(void) {
    for (unsigned i = 0; i < 4; i++)
        uav_pwm_hal_write(i, UAV_PWM_STOP_US);
}
