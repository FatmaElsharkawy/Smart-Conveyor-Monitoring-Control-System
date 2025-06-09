//
// Created by somai on 2025-06-10.
//

// ADC_config.h
#ifndef ADC_CONFIG_H
#define ADC_CONFIG_H

#include <stdint.h>

// Base addresses
#define PERIPH_BASE        ((uint32_t)0x40000000)
#define AHB1PERIPH_BASE    (PERIPH_BASE + 0x00020000)
#define APB2PERIPH_BASE    (PERIPH_BASE + 0x00010000)

#define RCC_BASE           (AHB1PERIPH_BASE + 0x3800)
#define ADC1_BASE          (APB2PERIPH_BASE + 0x2000)

// RCC register map (partial)
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
    // ... other unused registers
} RCC_TypeDef;

// ADC register map (partial)
typedef struct {
    volatile uint32_t SR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMPR1;
    volatile uint32_t SMPR2;
    volatile uint32_t JOFR1;
    volatile uint32_t JOFR2;
    volatile uint32_t JOFR3;
    volatile uint32_t JOFR4;
    volatile uint32_t HTR;
    volatile uint32_t LTR;
    volatile uint32_t SQR1;
    volatile uint32_t SQR2;
    volatile uint32_t SQR3;
    volatile uint32_t JSQR;
    volatile uint32_t JDR1;
    volatile uint32_t JDR2;
    volatile uint32_t JDR3;
    volatile uint32_t JDR4;
    volatile uint32_t DR;
} ADC_TypeDef;

// Peripheral instances
#define RCC     ((RCC_TypeDef *) RCC_BASE)
#define ADC1    ((ADC_TypeDef *) ADC1_BASE)

// RCC enable bit
#define RCC_APB2ENR_ADC1EN    (1 << 8)

// ADC control bits
#define ADC_CR2_ADON          (1 << 0)
#define ADC_CR2_SWSTART       (1 << 30)

#define ADC_SR_EOC            (1 << 1)

#define ADC_SMPR2_SMP0_0      (1 << 0)
#define ADC_SMPR2_SMP0_1      (1 << 1)

#endif // ADC_CONFIG_H
