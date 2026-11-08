#include "board.h"
void SystemInit(void)
{
    SCB->CPACR |= (0xFU << 20);                       /* FPU full access */
}
/* Hardware: HSE in BYPASS mode - the 25 MHz TCXO drives OSC_IN through the 74LVC2G34 buffer.
 * Simulation builds (SIM_F401 / SIM_F407): PLL fed from the internal 16 MHz HSI, so the Proteus
 * model needs no 25 MHz clock generator on OSC_IN (that would make the simulation very slow).
 * The resulting SYSCLK / APB / timer clocks are identical to the hardware build. */
#ifdef SIM_NOETH
#define PLL_M   16U
#define PLL_SRC 0U
#else
#define PLL_M   25U
#define PLL_SRC RCC_PLLCFGR_PLLSRC_HSE
#endif
void clock_init(void)
{
#ifndef SIM_NOETH
    RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)) { }
#endif
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
#ifdef SIM_F401
    /* 16(25) MHz / M * 336 / 4 = 84 MHz ; APB1 = 42 MHz (timers x2 = 84 MHz) ; APB2 = 84 MHz */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;
    RCC->PLLCFGR = PLL_M | (336 << 6) | (1 << 16) | PLL_SRC | (7 << 24);
    RCC->CFGR = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
#else
    /* 16(25) MHz / M * 336 / 2 = 168 MHz ; APB1 = 42 MHz (timers 84 MHz) ; APB2 = 84 MHz (timers 168 MHz) */
    PWR->CR |= PWR_CR_VOS;
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
    RCC->PLLCFGR = PLL_M | (336 << 6) | (0 << 16) | PLL_SRC | (7 << 24);
    RCC->CFGR = RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
#endif
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }
#ifndef SIM_NOETH
    /* MCO1 = HSE on PA8 (test point on J5-3: lets you verify the TCXO with a scope) */
    RCC->CFGR |= RCC_CFGR_MCO1_1;
#endif
}
