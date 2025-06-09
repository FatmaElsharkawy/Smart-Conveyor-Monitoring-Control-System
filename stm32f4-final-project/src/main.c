// #include "stm32f4xx.h"
#include "LCD.h"
#include "GPIO.h"
#include <stdint.h>
#include "Speed_calc.h"
#include "Rcc.h"
#include "EXTI.h"
#include "ADC.h"
#include "Motor.h"

// Global variables
uint8_t prevButtonState = 1; // Object detection button
uint32_t fallingEdgeCount = 0;
static float conveyor_speed = 0.0f;

typedef struct {
    uint32 NVIC_ISER[8]; //Enable
    uint32 NVIC_ICER[8];  //Disable
}NVICType;

#define NVIC        ((NVICType*) 0xE000E100)
#define Emergency_Button              12

// Function prototypes
void SystemClock_Config(void);
void GPIO_Init_All(void);
void LCD_DisplayMotorSpeed(uint8_t speed_percent);
void Delay_ms(uint32_t ms);

// Main function
int main(void) {
    SystemClock_Config();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    GPIO_Init_All();
    ADC_Init();
    Motor_Init();
    LCD_Init();
    Time_Capture_Init();
    // Emergency Interrupt //
    EXTI_Init(GPIO_B, Emergency_Button, RISING_AND_FALLING);
    EXTI_Enable(Emergency_Button); // Enable pin E12 (EXTI->IMR |= (0x1 << 12))
    NVIC->NVIC_ISER[1] |= (0x1 << 8); // 40 - 32 = 8 i.e. second register position 8 for both Button_LED

//    conveyor_speed = Get_Belt_Speed();

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("B.SP. M.SP. Cnt");

//    LCD_SetCursor(1, 0);
//    LCD_PrintNumber((uint32_t) (conveyor_speed));

    adc_filtered = ADC_Read();

    while (1) {
        // --- Belt speed calculation --- //
        //making sure to only change lcd reading when speed changes
        conveyor_speed = Get_Belt_Speed();
        static float old_speed = -1.0f; // Initialize to an invalid value
        if (conveyor_speed != old_speed) {
            LCD_SetCursor(1, 0);
            LCD_Print("   ");    // clear old value
            LCD_SetCursor(1, 0);
            LCD_PrintNumber((uint32_t) (conveyor_speed));
            LCD_Print("Hz");
            old_speed = conveyor_speed;
        }
        // --- Motor speed control --- //
        adc_value = ADC_Read();
        adc_filtered = ADC_Filter(adc_value);
//        adc_filtered = 4095;
        uint8_t speed_percent = ADC_GetSpeedPercent(adc_filtered);
        Motor_SetSpeed((100 - speed_percent));

        // LCD update (only when speed changes)
        static uint8_t prev_speed = 255;
        if (Motor_GetSpeed() != prev_speed) {
            LCD_SetCursor(1, 6); // after "Speed: "
            LCD_Print("   ");    // clear old value
            LCD_SetCursor(1, 6);
            LCD_PrintNumber(Motor_GetSpeed());
            LCD_Print("%");
            prev_speed = Motor_GetSpeed();
        }

        uint8_t currButtonState = GPIO_ReadPin(GPIO_A, 1);
        if (prevButtonState == 1 && currButtonState == 0) {
            Delay_ms(20); // Debounce delay (20 ms)
            // Increment count and update LCD
            fallingEdgeCount++;
            LCD_SetCursor(1, 14);
            LCD_Print("     ");    // clear old number
            LCD_SetCursor(1, 14);
            LCD_PrintNumber(fallingEdgeCount);
            // Wait for button release to prevent multiple counts
            while (GPIO_ReadPin(GPIO_A, 1) == 0) {
                Delay_ms(10); // Check every 10 ms
            }
        }
        prevButtonState = currButtonState;
    }

}

// ----------------------------- Initialization ----------------------------- //

void GPIO_Init_All(void)
{
    Rcc_Init();
    Rcc_Enable(RCC_GPIOA);
    Rcc_Enable(RCC_GPIOB);
    Rcc_Enable(RCC_SYSCFG);
    // PA0 -> Analog input for ADC
    GPIO_Init(GPIO_A, 0, GPIO_ANALOG, GPIO_NO_PULL_DOWN);

    // PA1 -> Object Detection Simulation Button
    GPIO_Init(GPIO_A, 1, GPIO_INPUT, GPIO_PULL_UP);

    // PB10 -> PWM output (TIM2 CH3)
    GPIO_Init(GPIO_B, 10, GPIO_AF, GPIO_PUSH_PULL);

    // Set alternate function AF1 (TIM2) for PB10
    GPIOB->AFR[1] &= ~(0xF << ((10 - 8) * 4));
    GPIOB->AFR[1] |=  (1 << ((10 - 8) * 4));

    //init for emergency button
    GPIO_Init(GPIO_B, Emergency_Button, GPIO_INPUT, GPIO_PULL_UP);
}

void LCD_DisplayMotorSpeed(uint8_t speed_percent)
{
    LCD_SetCursor(1, 6);
    LCD_Print("   ");
    LCD_SetCursor(1, 6);
    LCD_PrintNumber(speed_percent);
    LCD_Print("%");
}

// ----------------------------- Utility Functions ----------------------------- //
void Delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 8400; i++);
}

void SystemClock_Config(void)
{
    // Assume HSI (16 MHz) -> PLL -> SYSCLK = 84 MHz
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) |
                   (168 << RCC_PLLCFGR_PLLN_Pos) |
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

        //stop the motor and show "EMERGENCY STOP" message on LCD
        Motor_Stop();            // Stop the motor
        LCD_Clear();             // Optional
        LCD_SetCursor(1, 0);
        LCD_Print("Emergency Stop");

    }
}