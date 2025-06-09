//
// Created by somai on 2025-06-09.
//

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

// Constants
#define ADC_MAX_VALUE     4095
#define FILTER_ALPHA      8

// Function prototypes
void ADC_Init(void);
uint16_t ADC_Read(void);
uint16_t ADC_Filter(uint16_t new_value);
uint8_t ADC_GetSpeedPercent(uint16_t adc_value);

// Global variables (extern declarations)
extern volatile uint16_t adc_value;
extern volatile uint16_t adc_filtered;

#endif // ADC_H