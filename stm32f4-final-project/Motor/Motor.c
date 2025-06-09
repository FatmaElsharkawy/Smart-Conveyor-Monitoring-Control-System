//
// Created by somai on 2025-06-09.
//

#include "Motor.h"
#include "PWM.h"

// Global variables
volatile uint8_t motor_speed_percent = 0;

void Motor_Init(void)
{
    PWM_Init();
    motor_speed_percent = 0;
}

void Motor_SetSpeed(uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;
    motor_speed_percent = speed_percent;
    PWM_SetDutyCycle(speed_percent);
}

void Motor_Stop(void)
{
    motor_speed_percent = 0;
    PWM_SetDutyCycle(0); // 0% duty = no power to motor
}

uint8_t Motor_GetSpeed(void)
{
    return motor_speed_percent;
}