#ifndef LCD_H
#define LCD_H

#include "stm32f4xx.h"
#include <stdint.h>

// LCD Pin Connections (you can modify these as needed)
// Data pins D4-D7 connected to GPIOB pins 4-7
#define LCD_D4_PORT     GPIOB
#define LCD_D4_PIN      4
#define LCD_D5_PORT     GPIOB
#define LCD_D5_PIN      5
#define LCD_D6_PORT     GPIOB
#define LCD_D6_PIN      6
#define LCD_D7_PORT     GPIOB
#define LCD_D7_PIN      7

// Control pins
#define LCD_RS_PORT     GPIOB
#define LCD_RS_PIN      0
#define LCD_RW_PORT     GPIOB
#define LCD_RW_PIN      1
#define LCD_E_PORT      GPIOB
#define LCD_E_PIN       2

// LCD Commands
#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_ENTRY_MODE      0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT    0x10
#define LCD_FUNCTION_SET    0x20
#define LCD_SET_CGRAM       0x40
#define LCD_SET_DDRAM       0x80

// LCD Control bits
#define LCD_ENTRY_RIGHT     0x00
#define LCD_ENTRY_LEFT      0x02
#define LCD_ENTRY_SHIFT_INCREMENT   0x01
#define LCD_ENTRY_SHIFT_DECREMENT   0x00

#define LCD_DISPLAY_ON      0x04
#define LCD_DISPLAY_OFF     0x00
#define LCD_CURSOR_ON       0x02
#define LCD_CURSOR_OFF      0x00
#define LCD_BLINK_ON        0x01
#define LCD_BLINK_OFF       0x00

#define LCD_DISPLAY_MOVE    0x08
#define LCD_CURSOR_MOVE     0x00
#define LCD_MOVE_RIGHT      0x04
#define LCD_MOVE_LEFT       0x00

#define LCD_8BIT_MODE       0x10
#define LCD_4BIT_MODE       0x00
#define LCD_2_LINE          0x08
#define LCD_1_LINE          0x00
#define LCD_5x10DOTS        0x04
#define LCD_5x8DOTS         0x00

// Function prototypes
void LCD_Init(void);
void LCD_Clear(void);
void LCD_Home(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char* str);
void LCD_PrintNumber(uint32_t number);
void LCD_Command(uint8_t cmd);
void LCD_Data(uint8_t data);

// Private functions
static void LCD_GPIO_Init(void);
static void LCD_Write4Bits(uint8_t nibble);
static void LCD_PulseEnable(void);
static void LCD_SetPin(GPIO_TypeDef* port, uint8_t pin, uint8_t state);
static void Delay_us(uint32_t us);
static void Delay_ms(uint32_t ms);

#endif // LCD_PARALLEL_H