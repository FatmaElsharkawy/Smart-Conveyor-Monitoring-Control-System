//
// Created by somai on 2025-06-09.
//

#include "PWM.h"
#include "stm32f4xx.h"

void PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;    // 1. Enable Timer 2 clock

    TIM2->PSC = 83;                        // 2. Prescaler: 84MHz / (83 + 1) = 1MHz timer clock
    TIM2->ARR = PWM_PERIOD;               // 3. Auto-reload: defines PWM period (e.g. 999 -> 1 kHz)

    TIM2->CCR3 = 0;                        // 4. Start with 0% duty cycle. CCR3 define the duty cycle

    TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // 5. Set PWM Mode 1 OC3M[2:0] = 110
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;       // 6. Preload enable --> any changes you make to CCR3 will be buffered and only take effect on the next update event.

    TIM2->CCER |= TIM_CCER_CC3E;          // 7. Enable output for channel 3

    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; // 8. Enable auto-reload and counter
}

void PWM_SetDutyCycle(uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;
    // Add +50 for rounding instead of truncation
    TIM2->CCR3 = ((uint32_t)PWM_PERIOD * duty_percent + 50) / 100;
}