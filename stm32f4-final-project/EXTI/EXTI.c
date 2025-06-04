//
// Created by nadak on 12/05/2025.
//
#include "EXTI.h"

// Define the EXTI registers
ExtiType *EXTI = (ExtiType *)0x40013C00;
SYSCFG_t *sys_conf = (SYSCFG_t *)0x40013800;

void EXTI_Init(uint8 portName,  uint8 pinNum, uint8 triggerMode){

    uint8 EXTICR_ID = pinNum / 4 ; // get EXTICR register
    uint8 EXTICR_Shift = (pinNum % 4) * 4; // get position in EXTICR
    uint8 Port_ID = portName - GPIO_A; // port number


    sys_conf->EXTICR[EXTICR_ID] &= ~(0xF << EXTICR_Shift); // clear the port in the EXTICR register
    sys_conf->EXTICR[EXTICR_ID] |= (Port_ID << EXTICR_Shift); // Set the port in the EXTICR register

    if (triggerMode == RISING_EDGE) {
        EXTI->RTSR |= (1 << pinNum);
        EXTI->FTSR &= ~(1 << pinNum);
    }
    else if (triggerMode == FALLING_EDGE) {
        EXTI->FTSR |= (1 << pinNum);
        EXTI->RTSR &= ~(1 << pinNum);
    }
    else if (triggerMode == RISING_AND_FALLING) {
        EXTI->FTSR |= (1 << pinNum);
        EXTI->RTSR |= (1 << pinNum);
    }
}

void EXTI_Enable(uint8 pinNum){
    EXTI->IMR |= (1 << pinNum);
}

void EXTI_Disable(uint8 pinNum){
    // CLEAR_BIT(EXTI->IMR, pinNum);
    EXTI->IMR &= ~(1 << pinNum);
}
