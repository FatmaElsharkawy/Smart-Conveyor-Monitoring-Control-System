#ifndef GPIO_H
#define GPIO_H
#include "Std_Types.h"
#include "GPIO_PRIVATE.h"
/*PortName*/
#define GPIO_A 'A'
#define GPIO_B 'B'
#define GPIO_C 'C'
#define GPIO_D 'D'

/*PinMode*/
#define GPIO_INPUT 0x00
#define GPIO_OUTPUT 0x01
#define GPIO_AF 0x02
#define GPIO_ANALOG 0x03

/*DefaultState*/
#define GPIO_PUSH_PULL 0x00
#define GPIO_OPEN_DRAIN 0x01
#define GPIO_NO_PULL_DOWN 0x00
#define GPIO_PULL_UP 0x01
#define GPIO_PULL_DOWN 0x02

/*Data*/
#define LOW 0x0
#define HIGH 0x1
#define OK 0x0
#define NOK 0x1

void GPIO_Init(uint8 PortName, uint8 PinNumber, uint8 PinMode, uint8 DefaultState);
uint8 GPIO_WritePin(uint8 PortName, uint8 PinNumber, uint8 Data);
uint8 GPIO_ReadPin(uint8 PortName, uint8 PinNumber);

#endif //GPIO_H