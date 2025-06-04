//
// Created by Yasmine on 04-Jun-25.
//

#include "Speed_calc.h"
#include "stm32f4xx.h"
#include "GPIO.h"
#include <stdio.h>
#include "Std_Types.h"
#include <string.h>

static volatile uint32 time_capture_1 = 0;
static volatile uint32 time_capture_2 = 0;

void Capture_Edge(void){
    while(!(TIM2->SR & TIM_SR_CC1IF)); // Wait for capture event
    time_capture_1 = TIM2->CCR1; // Read the capture value
    TIM2->SR &= ~TIM_SR_CC1IF; // Clear the capture flag
    while(!(TIM2->SR & TIM_SR_CC1IF)); // Wait for the next capture event
    time_capture_2 = TIM2->CCR1; // Read the next capture value
    TIM2->SR &= ~TIM_SR_CC1IF; // Clear the capture flag again
}

void Time_Capture_Init(void) {

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // pin a5 as alternate function TIM2_CH1
    GPIO_Init(GPIO_A, 5, GPIO_AF, GPIO_PUSH_PULL);
    // Set alternate function AF1 (TIM2) for PA5
    GPIOA->AFR[0] &= ~(0xF << (5 * 4)); // Clear bits for PA5
    GPIOA->AFR[0] |=  (1 << (5 * 4));   // Set AF1 for PA5
    // Configure PA5 as input with pull-up
    GPIOA->MODER &= ~(0x3 << (5 * 2)); // Clear mode bits for PA5
    GPIOA->MODER |=  (0x2 << (5 * 2)); // Set PA5 to alternate function mode
    GPIOA->PUPDR &= ~(0x3 << (5 * 2)); // Clear pull-up/pull-down bits for PA5
    GPIOA->PUPDR |=  (0x1 << (5 * 2)); // Set PA5 to pull-up


    // configure TIM2
    TIM2->PSC = 83;
    TIM2->ARR = 0xFFFF;
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= 0x01;   // CC1 mapped to TI1
    TIM2->CCER &= ~TIM_CCER_CC1P;
    TIM2->CCER |=  TIM_CCER_CC1E;
    TIM2->CR1  |=  TIM_CR1_CEN;
}

float Get_Belt_Speed(void) {
    uint32 dt = 0;
    // Capture the time difference between two edges
    Capture_Edge();
    time_capture_1 = time_capture_1;
    time_capture_2 = time_capture_2;

    dt = (time_capture_2 >= time_capture_1) ? (time_capture_2 - time_capture_1) : ((0xFFFF - time_capture_1) + time_capture_2 );
    if (dt == 0) return 0.0f;   // prevent div/0

    return 1000000.0f / dt;     // freq = 1 / period (in seconds)
}


