#include "main.h"
void platform_emergency_stop(void) {
    __disable_irq();
    if (RCC->APB2ENR & RCC_APB2ENR_TIM10EN)
        TIM10->CCR1 = 0;
    if (RCC->APB2ENR & RCC_APB2ENR_TIM1EN) {
        TIM1->CCR1 = 1000;
        TIM1->CCR2 = 1000;
        TIM1->CCR3 = 1000;
        TIM1->CCR4 = 1000;
    }
}
