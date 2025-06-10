#include "LCD.h"
#include "GPIO.h"
#include "Rcc.h"

void LCD_Init(void) {
    LCD_GPIO_Init(); // Initialize GPIO pins
    Delay_ms(50); // Wait for LCD to power up
    LCD_SetPin(LCD_RW_PORT, LCD_RW_PIN, 0); // Set RW to write mode


    // Set to 4-bit mode
    LCD_Write4Bits(0x02);
    Delay_us(150);

    // Function set: 4-bit, 2 line, 5x8 dots
    LCD_Command(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2_LINE | LCD_5x8DOTS);

    // Display control: display on, cursor off, blink off
    LCD_Command(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);

    // Clear display
    LCD_Clear();

    // Entry mode: increment cursor, no shift
    LCD_Command(LCD_ENTRY_MODE | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT); // move right after each character
}

void LCD_Clear(void) {
    LCD_Command(LCD_CLEAR);
    Delay_ms(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row < 4) {
        LCD_Command(LCD_SET_DDRAM | (col + row_offsets[row])); // calculates offset of cursor
    }
}

void LCD_Print(const char* str) {
    // Prints a string to the LCD
    while (*str) {
        LCD_Data(*str++);
    }
}

void LCD_PrintNumber(uint32_t number) {
    if (number == 0) {
        LCD_Data('0');
        return;
    }

    int i = 0;
    char temp[12];
    while (number > 0) { // break number into digits
        temp[i++] = (number % 10) + '0';
        number /= 10;
    }

    // Print digits in reverse order (correct order)
    while (i > 0) {
        LCD_Data(temp[--i]);
    }
}

void LCD_Command(uint8_t cmd) {
    uint8_t higher_bits = (cmd & 0xF0) >> 4;
    uint8_t lower_bits = cmd & 0x0F;

    LCD_SetPin(LCD_RS_PORT, LCD_RS_PIN, 0); // RS = 0 for command mode

    LCD_Write4Bits(higher_bits);
    LCD_Write4Bits(lower_bits);

    if (cmd == LCD_CLEAR) {
        Delay_ms(2);
    } else {
        Delay_us(50);
    }
}

void LCD_Data(uint8_t data) {
    // write byte to LCD
    uint8_t higher_bits = (data & 0xF0) >> 4;
    uint8_t lower_bits = data & 0x0F;

    LCD_SetPin(LCD_RS_PORT, LCD_RS_PIN, 1); // RS = 1 for data mode

    LCD_Write4Bits(higher_bits);
    LCD_Write4Bits(lower_bits);

    Delay_us(50);
}

static void LCD_GPIO_Init(void) {
    // Enable GPIOB clock
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure data pins D4–D7 (PB4–PB7) as output, push-pull, no pull-up/down
    GPIO_Init(GPIO_B, LCD_D4_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL | (GPIO_NO_PULL_DOWN << 1));
    GPIO_Init(GPIO_B, LCD_D5_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL | (GPIO_NO_PULL_DOWN << 1));
    GPIO_Init(GPIO_B, LCD_D6_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL | (GPIO_NO_PULL_DOWN << 1));
    GPIO_Init(GPIO_B, LCD_D7_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL | (GPIO_NO_PULL_DOWN << 1));

    // Configure control pins RS, RW, E (PB0, PB1, PB2) similarly
    GPIO_Init(GPIO_B, LCD_RS_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL | (GPIO_NO_PULL_DOWN << 1));
    GPIO_Init(GPIO_B, LCD_RW_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL | (GPIO_NO_PULL_DOWN << 1));
    GPIO_Init(GPIO_B, LCD_E_PIN,  GPIO_OUTPUT, GPIO_PUSH_PULL | (GPIO_NO_PULL_DOWN << 1));

    // Initialize all used pins to LOW
    GPIO_WritePin(GPIO_B, LCD_D4_PIN, LOW);
    GPIO_WritePin(GPIO_B, LCD_D5_PIN, LOW);
    GPIO_WritePin(GPIO_B, LCD_D6_PIN, LOW);
    GPIO_WritePin(GPIO_B, LCD_D7_PIN, LOW);

    GPIO_WritePin(GPIO_B, LCD_RS_PIN, LOW);
    GPIO_WritePin(GPIO_B, LCD_RW_PIN, LOW);
    GPIO_WritePin(GPIO_B, LCD_E_PIN,  LOW);
}

static void LCD_Write4Bits(uint8_t half_byte) {
    // Set data pins
    LCD_SetPin(LCD_D4_PORT, LCD_D4_PIN, (half_byte >> 0) & 1);
    LCD_SetPin(LCD_D5_PORT, LCD_D5_PIN, (half_byte >> 1) & 1);
    LCD_SetPin(LCD_D6_PORT, LCD_D6_PIN, (half_byte >> 2) & 1);
    LCD_SetPin(LCD_D7_PORT, LCD_D7_PIN, (half_byte >> 3) & 1);

    // Pulse enable pin
    LCD_PulseEnable();
}

static void LCD_PulseEnable(void) {
    // generates a high-to-low pulse on the Enable (E) pin
    // to make it accept 4 bits from the microcontroller for writing
    LCD_SetPin(LCD_E_PORT, LCD_E_PIN, 1);
    Delay_us(1);
    LCD_SetPin(LCD_E_PORT, LCD_E_PIN, 0);
    Delay_us(50);
}

static void LCD_SetPin(uint8_t port, uint8_t pin, uint8_t state) {
    // Sets a GPIO pin on a given port to high or low
    GPIO_WritePin(port, pin, state);
}

static void Delay_us(uint32_t us) {
    // Assuming 84 MHz system clock
    // Each loop iteration takes approximately 4 clock cycles
    uint32_t cycles = (84 * us) / 4;
    for (volatile uint32_t i = 0; i < cycles; i++);
}

static void Delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        Delay_us(1000);
    }
}