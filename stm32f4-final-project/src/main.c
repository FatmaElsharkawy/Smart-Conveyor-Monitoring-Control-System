#include "stm32f4xx.h"
#include "LCD.h"
#include "Gpio.h"
#include <stdint.h>
#include "Speed_calc.h"
#include "Rcc.h"
#include "EXTI.h"

// Aliases for clarity
#ifndef uint32
#define uint32 uint32_t
#endif
#ifndef uint8
#define uint8 uint8_t
#endif
#ifndef uint16
#define uint16 uint16_t
#endif

// Global variables
volatile uint16_t adc_value = 0;
volatile uint8_t motor_speed_percent = 0;
volatile uint16_t adc_filtered = 0;
uint8_t prevButtonState = 1;
uint32_t fallingEdgeCount = 0;
static float conveyor_speed = 0.0f;

// Constants
#define ADC_MAX_VALUE     4095
#define FILTER_ALPHA      8
#define PWM_PERIOD        999

typedef struct {
    uint32 NVIC_ISER[8]; //Enable
    uint32 NVIC_ICER[8];  //Disable
}NVICType;

#define NVIC        ((NVICType*) 0xE000E100)
#define Emergency_Button              12

// Function prototypes
void SystemClock_Config(void);
void GPIO_Init_All(void);
void ADC_Init(void);
void PWM_Init(void);
uint16_t ADC_Read(void);
uint16_t ADC_Filter(uint16_t new_value);
void PWM_SetDutyCycle(uint8_t duty_percent);
void LCD_DisplayMotorSpeed(uint8_t speed_percent);
void Delay_ms(uint32_t ms);
void Motor_Stop(void);

// Main function
int main(void) {
    SystemClock_Config();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    GPIO_Init_All();
    ADC_Init();
    PWM_Init();
    LCD_Init();
    Time_Capture_Init();
    conveyor_speed = Get_Belt_Speed();
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("B.SP. M.SP. Cnt");

    LCD_SetCursor(1, 0);
    LCD_PrintNumber((uint32_t) (conveyor_speed));


    adc_filtered = ADC_Read();

    while (1) {
        // --- Motor speed control ---
        adc_value = ADC_Read();
        // Optionally apply filtering:
        // adc_filtered = ADC_Filter(adc_value);
        // motor_speed_percent = (adc_filtered * 100) / ADC_MAX_VALUE;
        motor_speed_percent = (adc_value * 100) / ADC_MAX_VALUE;
        EXTI_Init(GPIO_B, Emergency_Button, RISING_AND_FALLING);
        EXTI_Enable(Emergency_Button); // Enable pin E12 (EXTI->IMR |= (0x1 << 12))
        NVIC->NVIC_ISER[1] |= (0x1 << 8); // 40 - 32 = 8 i.e. second register position 8 for both Button_LED


        while (1) {
            // --- Motor speed control ---
            adc_value = ADC_Read();
            // Optionally apply filtering:
            // adc_filtered = ADC_Filter(adc_value);
            // motor_speed_percent = (adc_filtered * 100) / ADC_MAX_VALUE;
            motor_speed_percent = (adc_value * 100) / ADC_MAX_VALUE;

            PWM_SetDutyCycle(motor_speed_percent);

            // Efficient LCD update (only when speed changes)
            static uint8_t prev_speed = 255;
            if (motor_speed_percent != prev_speed) {
//                LCD_SetCursor(1, 0);
//                LCD_Print("Speed: ");
                LCD_SetCursor(1, 6); // after "Speed: "
                LCD_Print("   ");    // clear old value
                LCD_SetCursor(1, 6);
                LCD_PrintNumber(motor_speed_percent);
                LCD_Print("%");
                prev_speed = motor_speed_percent;
            }

            // --- Object counter ---
            uint8_t currButtonState = GPIO_ReadPin(GPIO_A, 1);
            if (prevButtonState == 1 && currButtonState == 0) {
                Delay_ms(50); // debounce
                currButtonState = GPIO_ReadPin(GPIO_A, 1);
                if (currButtonState == 0) {
                    fallingEdgeCount++;
//                    LCD_SetCursor(0, 0);
//                    LCD_Print("Object Count:");
                    LCD_SetCursor(1, 14); // or adjust as needed
                    LCD_Print("     ");    // clear old number
                    LCD_SetCursor(1, 14);
                    LCD_PrintNumber(fallingEdgeCount);

                    while (GPIO_ReadPin(GPIO_A, 1) == 0) {
                        Delay_ms(10); // wait for release
                    }
                    Delay_ms(50); // debounce after release
                }
            }
            prevButtonState = currButtonState;
            // main loop delay
            Delay_ms(10);
        }
    }
}
// ----------------------------- Initialization -----------------------------

void GPIO_Init_All(void)
{
    Rcc_Init();
    Rcc_Enable(RCC_GPIOA);
    Rcc_Enable(RCC_GPIOB);
    Rcc_Enable(RCC_SYSCFG);
    // PA0 -> Analog input for ADC
    GPIO_Init(GPIO_A, 0, GPIO_ANALOG, GPIO_NO_PULL_DOWN);

    // PA1 -> Button input
    GPIO_Init(GPIO_A, 1, GPIO_INPUT, GPIO_PULL_UP);

    // PB10 -> PWM output (TIM2 CH3)
    GPIO_Init(GPIO_B, 10, GPIO_AF, GPIO_PUSH_PULL);

    // Set alternate function AF1 (TIM2) for PB10
    GPIOB->AFR[1] &= ~(0xF << ((10 - 8) * 4));
    GPIOB->AFR[1] |=  (1 << ((10 - 8) * 4));

    //init for emergency button
    GPIO_Init(GPIO_B, Emergency_Button, GPIO_INPUT, GPIO_PULL_UP);
}

void ADC_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = 0;
    ADC1->SQR3 = 0;            // Channel 0 (PA0)
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0; // sampling time
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC_Read(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

uint16_t ADC_Filter(uint16_t new_value)
{
    return (adc_filtered * (FILTER_ALPHA - 1) + new_value) / FILTER_ALPHA;
}

void PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 83;         // 84 MHz / (83+1) = 1 MHz
    TIM2->ARR = PWM_PERIOD; // 1 kHz PWM
    TIM2->CCR3 = 0;         // Start with 0% duty cycle
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM2->CCER |= TIM_CCER_CC3E;
    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

void PWM_SetDutyCycle(uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100;
    TIM2->CCR3 = (PWM_PERIOD * duty_percent) / 100;
}

void Motor_Stop(void) {
    PWM_SetDutyCycle(0); // 0% duty = no power to motor
}

void LCD_DisplayMotorSpeed(uint8_t speed_percent)
{
    LCD_SetCursor(1, 6);
    LCD_Print("   ");
    LCD_SetCursor(1, 6);
    LCD_PrintNumber(speed_percent);
    LCD_Print("%");
}

// ----------------------------- Utility Functions -----------------------------

void Delay_ms(uint32_t ms)
{
    SysTick->LOAD = 84000 - 1;  // 1ms at 84MHz
    SysTick->VAL = 0;
    SysTick->CTRL = 5;          // Enable SysTick, no interrupt

    for (uint32_t i = 0; i < ms; i++)
    {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }

    SysTick->CTRL = 0;
}

void SystemClock_Config(void)
{
    // Assume HSI (16 MHz) -> PLL -> SYSCLK = 84 MHz
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) |
                   (336 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) |
                   (RCC_PLLCFGR_PLLSRC_HSI) |
                   (7 << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL));
}

//Define Function Called From vector lookup Table
// Emergency Stop (Overwrite EXTI15_10_IRQHandler)

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << Emergency_Button)) { // Check if EXTI12 triggered
        EXTI->PR |= (1 << Emergency_Button); // Clear pending bit by writing 1

        //stop the motor and show “EMERGENCY STOP” message on LCD
        Motor_Stop();            // Stop the motor
        LCD_Clear();             // Optional
        LCD_SetCursor(1, 0);
        LCD_Print("Emergency Stop");

    }
}

