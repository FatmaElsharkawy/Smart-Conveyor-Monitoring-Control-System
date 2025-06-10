#include "Std_Types.h"

#ifndef GPIO_PRIVATE_H
#define GPIO_PRIVATE_H

#define GPIOA_BASE_ADDR 0x40020000
#define GPIOB_BASE_ADDR 0x40020400
#define GPIOC_BASE_ADDR 0x40020800
#define GPIOD_BASE_ADDR 0x40020C00
typedef struct {
    volatile uint32 GPIO_MODER;
    volatile uint32 GPIO_OTYPER;
    volatile uint32 GPIO_OSPEEDR;
    volatile uint32 GPIO_PUPDR;
    volatile uint32 GPIO_IDR;
    volatile uint32 GPIO_ODR;
    volatile uint32 GPIO_BSRR;
    volatile uint32 GPIO_LCKR;
    volatile uint32 GPIO_AFRL;
    volatile uint32 GPIO_AFRH;
} GpioType;
#endif //GPIO_PRIVATE_H