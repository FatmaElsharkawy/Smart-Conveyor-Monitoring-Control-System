/**
 * Rcc_Private.h
 *
 *  Created on: Sun Mar 26 2023
 *  Author    : Abdullah Darwish
 */

#ifndef RCC_PRIVATE_H
#define RCC_PRIVATE_H
#include "Std_Types.h"
#include "Utils.h"

#define RCC_BASE_ADDR       0x40023800
#define RCC_CR              REG32(RCC_BASE_ADDR + 0x00UL)
#define RCC_PLLCFGR         REG32(RCC_BASE_ADDR + 0x04UL)
#define RCC_CFGR            REG32(RCC_BASE_ADDR + 0x08UL)
#define RCC_CIR             REG32(RCC_BASE_ADDR + 0x0CUL)
#define RCC_AHB1RSTR        REG32(RCC_BASE_ADDR + 0x10UL)
#define RCC_AHB2RSTR        REG32(RCC_BASE_ADDR + 0x14UL)
#define RCC_AHB3RSTR        REG32(RCC_BASE_ADDR + 0x18UL)
#define RCC_APB1RSTR        REG32(RCC_BASE_ADDR + 0x20UL)
#define RCC_APB2RSTR        REG32(RCC_BASE_ADDR + 0x24UL)
#define RCC_AHB1ENR         REG32(RCC_BASE_ADDR + 0x30UL)
#define RCC_AHB2ENR         REG32(RCC_BASE_ADDR + 0x34UL)
#define RCC_AHB3ENR         REG32(RCC_BASE_ADDR + 0x38UL)
#define RCC_APB1ENR         REG32(RCC_BASE_ADDR + 0x40UL)
#define RCC_APB2ENR         REG32(RCC_BASE_ADDR + 0x44UL)
#define RCC_AHB1LPENR       REG32(RCC_BASE_ADDR + 0x50UL)
#define RCC_AHB2LPENR       REG32(RCC_BASE_ADDR + 0x54UL)
#define RCC_AHB3LPENR       REG32(RCC_BASE_ADDR + 0x58UL)
#define RCC_APB1LPENR       REG32(RCC_BASE_ADDR + 0x60UL)
#define RCC_APB2LPENR       REG32(RCC_BASE_ADDR + 0x64UL)
#define RCC_BDCR            REG32(RCC_BASE_ADDR + 0x70UL)
#define RCC_CSR             REG32(RCC_BASE_ADDR + 0x74UL)
#define RCC_SSCGR           REG32(RCC_BASE_ADDR + 0x80UL)
#define RCC_PLLI2SCFGR      REG32(RCC_BASE_ADDR + 0x84UL)


// Added
// RCC bit masks
#define RCC_CR_HSION        (1U << 0)
#define RCC_CR_HSIRDY       (1U << 1)
#define RCC_CR_PLLON        (1U << 24)
#define RCC_CR_PLLRDY       (1U << 25)

#define RCC_PLLCFGR_PLLSRC_HSI   (0U << 22)
#define RCC_PLLCFGR_PLLM_Pos     0
#define RCC_PLLCFGR_PLLN_Pos     6
#define RCC_PLLCFGR_PLLP_Pos     16
#define RCC_PLLCFGR_PLLQ_Pos     24

#define RCC_CFGR_SW_PLL      (0b10U)
#define RCC_CFGR_SWS_PLL     (0b10U << 2)
#define RCC_CFGR_HPRE_DIV1   (0U << 4)
#define RCC_CFGR_PPRE1_DIV2  (0b100U << 10)
#define RCC_CFGR_PPRE2_DIV1  (0b000U << 13)

#define RCC_AHB1ENR_GPIOAEN  (1U << 0)
#define RCC_AHB1ENR_GPIOBEN  (1U << 1)
#define RCC_AHB1ENR_GPIOCEN  (1U << 2)

#define FLASH_BASE           0x40023C00
#define FLASH_ACR            REG32(FLASH_BASE + 0x00U)
#define FLASH_ACR_LATENCY_2WS  (2U << 0)


#endif /* RCC_PRIVATE_H */
