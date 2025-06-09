//
// Created by Yasmine on 04-Jun-25.
//

#include "Speed_calc.h"
#include "stm32f4xx.h"
#include "GPIO.h"
#include "Std_Types.h"
#include <stdio.h>

static volatile uint32 time_capture_1 = 0;
static volatile uint32 time_capture_2 = 0;

void Capture_Edge(void){
    uint32_t timeout;

    // First edge
    TIM3->SR &= ~TIM_SR_CC1IF;
    TIM3->CNT = 0; // Optional
    timeout = 1000000;
    while (!(TIM3->SR & TIM_SR_CC1IF) && timeout--);
    if (timeout == 0) return;
    time_capture_1 = TIM3->CCR1;
    TIM3->SR &= ~TIM_SR_CC1IF;

    // Second edge
    timeout = 1000000;
    while (!(TIM3->SR & TIM_SR_CC1IF) && timeout--);
    if (timeout == 0) return;
    time_capture_2 = TIM3->CCR1;
    TIM3->SR &= ~TIM_SR_CC1IF;
}


void Time_Capture_Init(void) {

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // pin a6 as alternate function TIM3_CH1
    GPIO_Init(GPIO_A, 6, GPIO_AF, GPIO_PUSH_PULL);
    GPIOA->AFR[0] &= ~(0xF << (6 * 4)); // Clear alternate function bits for PA6
    GPIOA->AFR[0] |= (2 << (6 * 4)); // Set AF2 (TIM3) for PA6

    // Configure TIM3 for input capture
    TIM3->PSC = 83; // Prescaler to get 1 MHz clock
    TIM3->ARR = 0xFFFF; // Max value for 16-bit timer
    TIM3->CCMR1 &= ~TIM_CCMR1_CC1S; // Clear CC1S bits
    TIM3->CCMR1 |= TIM_CCMR1_CC1S_0; // Set CC1S to 01 (input capture on TI1)
    TIM3->CCMR1 |= TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1; // Set input filter (fdiv = 16)
    TIM3->CCER |= TIM_CCER_CC1E; // Enable capture on channel 1
    TIM3->CCER &= ~TIM_CCER_CC1P; // Capture on rising edge
    TIM3->CR1 |= TIM_CR1_CEN; // Enable the timer

}


float Get_Belt_Speed(void) {
    uint32 dt = 0;
    Capture_Edge();

//    dt = time_capture_2; // dt in timer ticks

    dt = (time_capture_2 >= time_capture_1) ? (time_capture_2 - time_capture_1) : ((0xFFFF - time_capture_1) + time_capture_2 );
    if (dt == 0) return 0.0f;   // prevent div/0

    return 1000000.0f / dt;     // freq = 1 / period (in seconds)
}


