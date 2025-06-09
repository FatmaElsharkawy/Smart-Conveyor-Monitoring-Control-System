#include "Speed_calc.h"
#include "GPIO_PRIVATE.h"
#include "Rcc_Private.h"
#include "Std_Types.h"

// Register Access Macros
#define TIM3_BASE_ADDR       0x40000400

#define TIM3_CR1             (*(volatile uint32*)(TIM3_BASE_ADDR + 0x00))
#define TIM3_SR              (*(volatile uint32*)(TIM3_BASE_ADDR + 0x10))
#define TIM3_CCMR1           (*(volatile uint32*)(TIM3_BASE_ADDR + 0x18))
#define TIM3_CCER            (*(volatile uint32*)(TIM3_BASE_ADDR + 0x20))
#define TIM3_PSC             (*(volatile uint32*)(TIM3_BASE_ADDR + 0x28))
#define TIM3_ARR             (*(volatile uint32*)(TIM3_BASE_ADDR + 0x2C))
#define TIM3_CCR1            (*(volatile uint32*)(TIM3_BASE_ADDR + 0x34))
#define TIM3_CNT             (*(volatile uint32*)(TIM3_BASE_ADDR + 0x24))

// Bit Masks
#define RCC_TIM3_EN          (1 << 1)   // TIM3 enable bit in RCC_APB1ENR
#define TIM_SR_CC1IF         (1 << 1)
#define TIM_CCER_CC1E        (1 << 0)
#define TIM_CCER_CC1P        (1 << 1)
#define TIM_CCMR1_CC1S_0     (1 << 0)
#define TIM_CCMR1_IC1F_0     (1 << 4)
#define TIM_CCMR1_IC1F_1     (1 << 5)

// Capture Logic
static volatile uint32 time_capture_1 = 0;
static volatile uint32 time_capture_2 = 0;

void Time_Capture_Init(void)
{
    // Enable GPIOA and TIM3 clocks
    RCC_AHB1ENR |= (1 << 0);      // GPIOAEN
    RCC_APB1ENR |= RCC_TIM3_EN;   // TIM3EN

    // Configure PA6 as alternate function (AF2) for TIM3_CH1
    GpioType* gpioa = (GpioType*) GPIOA_BASE_ADDR;
    gpioa->GPIO_MODER &= ~(0x3 << (6 * 2));
    gpioa->GPIO_MODER |=  (0x2 << (6 * 2));          // Alternate function mode
    gpioa->GPIO_AFRL  &= ~(0xF << (6 * 4));
    gpioa->GPIO_AFRL  |=  (0x2 << (6 * 4));          // AF2 for TIM3_CH1

    // Timer configuration
    TIM3_PSC = 83;           // 84 MHz / (83+1) = 1 MHz → 1 tick = 1 µs
    TIM3_ARR = 0xFFFF;       // Max auto-reload
    TIM3_CCMR1 &= ~0x3;      // Clear CC1S bits
    TIM3_CCMR1 |= TIM_CCMR1_CC1S_0; // CC1 mapped to TI1
    TIM3_CCMR1 |= TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1; // Filter = 1111b: max digital filter
    TIM3_CCER &= ~TIM_CCER_CC1P;   // Rising edge
    TIM3_CCER |= TIM_CCER_CC1E;    // Enable capture
    TIM3_CR1 |= 1;                 // Enable timer
}

static void Capture_Edge(void)
{
    uint32 timeout = 1000000;

    // Wait for first edge
    TIM3_SR &= ~TIM_SR_CC1IF;
    TIM3_CNT = 0;
    while (!(TIM3_SR & TIM_SR_CC1IF) && timeout--);
    if (timeout == 0) return;
    time_capture_1 = TIM3_CCR1;
    TIM3_SR &= ~TIM_SR_CC1IF;

    // Wait for second edge
    timeout = 1000000;
    while (!(TIM3_SR & TIM_SR_CC1IF) && timeout--);
    if (timeout == 0) return;
    time_capture_2 = TIM3_CCR1;
    TIM3_SR &= ~TIM_SR_CC1IF;
}

float Get_Belt_Speed(void)
{
    Capture_Edge();

    uint32 dt = (time_capture_2 >= time_capture_1)
                ? (time_capture_2 - time_capture_1)
                : ((0xFFFF - time_capture_1) + time_capture_2);

    if (dt == 0) return 0.0f;

    return 1000000.0f / dt; // Return frequency in Hz
}
