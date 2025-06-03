#include <Std_Types.h>
#include "GPIO.h"
#include "GPIO_PRIVATE.h"
#define GPIO_REG(PORT_ID, REG_ID) ((volatile uint32 *) ((PORT_ID) + (REG_ID)))

uint32 addressMap[4] = {GPIOA_BASE_ADDR, GPIOB_BASE_ADDR, GPIOC_BASE_ADDR, GPIOD_BASE_ADDR};

void GPIO_Init(uint8 PortName, uint8 PinNumber, uint8 PinMode, uint8 DefaultState) {
    uint8 addressIndex = PortName - GPIO_A;
    GpioType* gpioDevice = (GpioType*) addressMap[addressIndex];
    gpioDevice->GPIO_MODER &= ~(0x03 << (PinNumber * 2));
    gpioDevice->GPIO_MODER |= (PinMode << (PinNumber * 2));
    if (PinMode == GPIO_INPUT) {
        gpioDevice->GPIO_PUPDR &= ~(0x03 << (PinNumber * 2));
        gpioDevice->GPIO_PUPDR |= (DefaultState << (PinNumber * 2));
    } else {
        gpioDevice->GPIO_OTYPER &= ~(0x1 << PinNumber);
        gpioDevice->GPIO_OTYPER |= (DefaultState << (PinNumber));
    }
}

uint8 GPIO_WritePin(uint8 PortName, uint8 PinNumber, uint8 Data) {
    uint8 status = NOK;
    uint8 addressIndex = PortName - GPIO_A;
    GpioType* gpioDevice = (GpioType*) addressMap[addressIndex];
    if (((gpioDevice->GPIO_MODER & (0x03 << (PinNumber * 2))) >> (PinNumber * 2)) != GPIO_INPUT) {
        gpioDevice->GPIO_ODR &= ~(0x1 << PinNumber);
        gpioDevice->GPIO_ODR |= (Data << PinNumber);
        status = OK;
    } return status;
}

uint8 GPIO_ReadPin(uint8 PortName, uint8 PinNum) {
    uint8 data = 0; uint8 addressIndex = PortName - GPIO_A;
    GpioType* gpioDevice = (GpioType*) addressMap[addressIndex];
    data = (gpioDevice->GPIO_IDR & (0x1 << PinNum)) >> PinNum;
    return data;
}