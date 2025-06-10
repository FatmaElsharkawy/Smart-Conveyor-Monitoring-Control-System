#ifndef LCD_H
#define LCD_H

// #include "stm32f4xx.h"
#include <stdint.h>
#include "GPIO.h"

// LCD Pin Connections
#define LCD_D4_PORT     GPIO_B
#define LCD_D4_PIN      4
#define LCD_D5_PORT     GPIO_B
#define LCD_D5_PIN      5
#define LCD_D6_PORT     GPIO_B
#define LCD_D6_PIN      6
#define LCD_D7_PORT     GPIO_B
#define LCD_D7_PIN      7

#define LCD_RS_PORT     GPIO_B
#define LCD_RS_PIN      0
#define LCD_RW_PORT     GPIO_B
#define LCD_RW_PIN      1
#define LCD_E_PORT      GPIO_B
#define LCD_E_PIN       2

// LCD Commands
#define LCD_CLEAR           0x01
#define LCD_FUNCTION_SET    0x20
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_ENTRY_MODE      0x04
#define LCD_SET_DDRAM       0x80

// LCD Control bits
#define LCD_4BIT_MODE       0x00
#define LCD_2_LINE          0x08
#define LCD_5x8DOTS         0x00
#define LCD_DISPLAY_ON      0x04
#define LCD_CURSOR_OFF      0x00
#define LCD_BLINK_OFF       0x00
#define LCD_ENTRY_LEFT      0x02
#define LCD_ENTRY_SHIFT_DECREMENT   0x00

// Function prototypes
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char* str);
void LCD_PrintNumber(uint32_t number);
void LCD_Command(uint8_t cmd);
void LCD_Data(uint8_t data);

static void LCD_GPIO_Init(void);
static void LCD_Write4Bits(uint8_t nibble);
static void LCD_PulseEnable(void);
static void LCD_SetPin(uint8_t port, uint8_t pin, uint8_t state);
static void Delay_us(uint32_t us);
static void Delay_ms(uint32_t ms);

#endif // LCD_H