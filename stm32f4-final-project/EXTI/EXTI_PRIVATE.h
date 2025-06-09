#ifndef EXTI_PRIVATE_H
#define EXTI_PRIVATE_H

#include "Std_Types.h"

#define EXTI_BASE_ADDR     0x40013C00UL

#define EXTI_IMR           (*(volatile uint32*)(EXTI_BASE_ADDR + 0x00))
#define EXTI_EMR           (*(volatile uint32*)(EXTI_BASE_ADDR + 0x04))
#define EXTI_RTSR          (*(volatile uint32*)(EXTI_BASE_ADDR + 0x08))
#define EXTI_FTSR          (*(volatile uint32*)(EXTI_BASE_ADDR + 0x0C))
#define EXTI_SWIER         (*(volatile uint32*)(EXTI_BASE_ADDR + 0x10))
#define EXTI_PR            (*(volatile uint32*)(EXTI_BASE_ADDR + 0x14))

#endif // EXTI_PRIVATE_H
