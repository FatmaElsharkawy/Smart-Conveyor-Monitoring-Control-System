//
// Created by somai on 2025-06-10.
//

// stm32f4xx_custom.h
#ifndef PWM_CONFIG_H
#define PWM_CONFIG_H

#include <stdint.h>

// Base addresses
#define PERIPH_BASE     ((uint32_t)0x40000000)
#define APB1PERIPH_BASE PERIPH_BASE
#define AHB1PERIPH_BASE (PERIPH_BASE + 0x00020000)

#define RCC_BASE        (AHB1PERIPH_BASE + 0x3800)
#define TIM2_BASE       (APB1PERIPH_BASE + 0x0000)

// RCC register map
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    // ... (Other registers not used here)
} RCC_TypeDef;

// TIM register map (only what you need)
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} TIM_TypeDef;

// Pointers to peripheral base addresses
#define RCC     ((RCC_TypeDef *) RCC_BASE)
#define TIM2    ((TIM_TypeDef *) TIM2_BASE)

// Bit definitions for used fields
#define RCC_APB1ENR_TIM2EN     (1 << 0)

#define TIM_CCMR2_OC3M_1       (1 << 6)
#define TIM_CCMR2_OC3M_2       (1 << 5)
#define TIM_CCMR2_OC3PE        (1 << 3)

#define TIM_CCER_CC3E          (1 << 8)

#define TIM_CR1_ARPE           (1 << 7)
#define TIM_CR1_CEN            (1 << 0)

#endif // PWM_CONFIG_H
