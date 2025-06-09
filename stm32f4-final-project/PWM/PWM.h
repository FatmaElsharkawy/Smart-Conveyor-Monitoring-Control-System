//
// Created by somai on 2025-06-09.
//

#ifndef PWM_H
#define PWM_H

#include <stdint.h>

// Constants
#define PWM_PERIOD        999

// Function prototypes
void PWM_Init(void);
void PWM_SetDutyCycle(uint8_t duty_percent);

#endif // PWM_H