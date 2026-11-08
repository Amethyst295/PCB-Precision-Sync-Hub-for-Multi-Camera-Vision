/* Precision Sync Hub - board definitions (AI-SED 2026, team Borrenautas)
 * Target: STM32F407VET6.
 * Simulation builds for Proteus VSM (no Ethernet/PTP - Proteus does not simulate them; TIM2 free-runs):
 *   -DSIM_F407 : STM32F407VE model (same chip as the board)   <- recommended
 *   -DSIM_F401 : STM32F401VE model (fallback)
 * (STM32F401VE model: same Cortex-M4 core and same TIM2/TIM3/TIM4/TIM1/TIM9 pins, no Ethernet). */
#ifndef BOARD_H
#define BOARD_H
/* Proteus VSM Studio passes -DSTM32F40_41xxx (and no SIM_ flag): select the F407 simulation build automatically */
#if defined(STM32F40_41xxx) && !defined(SIM_F401) && !defined(SIM_F407)
#define SIM_F407
#endif
#if defined(SIM_F401) || defined(SIM_F407)
#define SIM_NOETH
#endif
/* VSM Studio also passes -D__STATIC_INLINE=__INLINE, which turns CMSIS 'static inline' into plain 'inline'
 * and causes 'undefined reference to __NVIC_EnableIRQ' at -O0. Restore the CMSIS default. */
#undef __STATIC_INLINE
#ifdef SIM_F401
#include "stm32f401xe.h"
#define SYSCLK_HZ   84000000UL
#else
#include "stm32f407xx.h"
#define SYSCLK_HZ  168000000UL
#endif
#define TIMCLK_APB1_HZ 84000000UL      /* TIM2/3/4/12 kernel clock (both builds) */
#ifdef SIM_F401
#define APB2_CAP_PSC 0                 /* TIM1/TIM9 run at 84 MHz in the F401 build */
#else
#define APB2_CAP_PSC 1                 /* 168 MHz / 2 = 84 MHz -> same tick as TIM12 */
#endif
#define N_CAM 8
/* Pin map (LQFP-100) - must match the schematic netlist:
 *  TRIG1..4 : PC6..PC9   TIM3_CH1..4  (AF2)      TRIG5..8 : PD12..PD15 TIM4_CH1..4 (AF2)
 *  STB1..4  : PE9,PE11,PE13,PE14 TIM1_CH1..4 (AF1)
 *  STB5..6  : PE5,PE6 TIM9_CH1..2 (AF3)       STB7..8 : PB14,PB15 TIM12_CH1..2 (AF9, F407 only)
 *  PPS loop-back capture : PA3 TIM2_CH4 (AF1)   ETH_PPS_OUT : PB5 (AF11)
 *  RMII : PA1 REF_CLK, PA2 MDIO, PA7 CRS_DV, PC1 MDC, PC4/PC5 RXD0/1, PB11 TX_EN, PB12/PB13 TXD0/1 (AF11)
 *  PHY nRST : PE3      LEDs : PD0 status, PD1 link, PD2 fault      USART1 : PA9 TX, PA10 RX (AF7) */
#endif
