//
// Created by nadak on 12/05/2025.
//

#ifndef EXTI_H
#define EXTI_H
#include "Std_Types.h"
#include "EXTI_PRIVATE.h"

// Define the EXTI registers
typedef struct{
    uint32 IMR; //interrupt mask register 
    uint32 EMR;
    uint32 RTSR; //Rising trigger selection register
    uint32 FTSR;  //Falling trigger selection register
    uint32 SWIER;
    uint32 PR;  //Pending Register: Check trigger occured or not 
}ExtiType;

// Define the SYSCFG registers
typedef struct{
    uint32 MEMR;
    uint32 PMC;
    uint32 EXTICR[4];
    uint32 CMPCR;

}SYSCFG_t;

// Define the base addresses of the EXTI and SYSCFG registers
// extern ExtiType *EXTI;
// extern SYSCFG *sys_conf;


#define Enable 1
#define Disable 0

#define GPIO_A 'A'
#define GPIO_B 'B'
#define GPIO_C 'C'
#define GPIO_D 'D'

#define RISING_EDGE 1
#define FALLING_EDGE 2
#define RISING_AND_FALLING 3

void EXTI_Init(uint8 PortName,  uint8 PinNum, uint8 triggerMode);
void EXTI_Enable(uint8 PinNum);
void EXTI_Disable(uint8 PinNum);

#endif

