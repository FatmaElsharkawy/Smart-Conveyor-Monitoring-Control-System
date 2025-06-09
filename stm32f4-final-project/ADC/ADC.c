//
// Created by somai on 2025-06-09.
//
#include "ADC.h"
#include "ADC_Config.h"

// Global variables
volatile uint16_t adc_value = 0;
volatile uint16_t adc_filtered = 0;

void ADC_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // 1. Enable ADC1 clock
    ADC1->CR2 = 0;                      // 2. Reset ADC1 control register
    ADC1->SQR3 = 0;                     // 3. Select channel 0 (PA0) for the first conversion
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0; // 4. Set sampling time for channel 0 (PA0) to 56 cycles
    ADC1->CR2 |= ADC_CR2_ADON;         // 5. Enable ADC1
}

uint16_t ADC_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;           // 1. Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));       // 2. Wait until conversion is complete (End Of Conversion flag is set)
    return ADC1->DR;                        // 3. Return result from Data Register
}

uint16_t ADC_Filter(uint16_t new_value)
{

    if (adc_filtered == 0) {
        adc_filtered = new_value; // Initialize on first call
        return adc_filtered;
    }
    return (adc_filtered * (FILTER_ALPHA - 1) + new_value) / FILTER_ALPHA;
}

uint8_t ADC_GetSpeedPercent(uint16_t adc_value)
{
    // Use 32-bit arithmetic to prevent overflow, then cast back
    uint32_t temp = ((uint32_t)adc_value * 100UL) / ADC_MAX_VALUE;
    if (temp > 100) temp = 100; // Safety check
    return (uint8_t)temp;
}
