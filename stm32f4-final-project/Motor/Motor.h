//
// Created by somai on 2025-06-09.
//

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// Global variables (extern declarations)
extern volatile uint8_t motor_speed_percent;

// Function prototypes
void Motor_Init(void);
void Motor_SetSpeed(uint8_t speed_percent);
void Motor_Stop(void);
uint8_t Motor_GetSpeed(void);

#endif // MOTOR_H