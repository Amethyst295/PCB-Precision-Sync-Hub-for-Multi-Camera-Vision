/* STM32F407 Ethernet MAC IEEE-1588 hardware clock (RM0090 sec. 33.5.9).
 * The PTP protocol stack (Sync/Delay_Req messages) runs on lwIP + an IEEE 1588 stack (e.g. the PTPd
 * port used in ST's STM32F4x7 PTP application note); it hands every exchange (t1..t4) to servo_update().
 * This file contains only the hardware-clock part that the Sync Hub needs. */
#include "board.h"
#include "ptp_hw.h"
#ifndef SIM_NOETH
#define SSINC_NS 20U                                  /* sub-second increment: 50 MHz equivalent */
static uint32_t base_addend;
void ptp_hw_init(void)
{
    ETH->MACIMR |= ETH_MACIMR_TSTIM;                  /* mask time-stamp trigger interrupt (we use TIM2 ITR1) */
    ETH->PTPTSCR |= ETH_PTPTSCR_TSE;                  /* enable time stamping */
    ETH->PTPSSIR = SSINC_NS;
    /* addend = 2^32 * (1e9/SSINC) / HCLK -> 2^32 * 50e6 / 168e6 */
    base_addend = (uint32_t)(4294967296.0 * (1e9 / SSINC_NS) / (double)SYSCLK_HZ);
    ETH->PTPTSAR = base_addend;
    ETH->PTPTSCR |= ETH_PTPTSCR_TSARU; while (ETH->PTPTSCR & ETH_PTPTSCR_TSARU) { }
    ETH->PTPTSCR |= ETH_PTPTSCR_TSFCU;                /* fine correction method */
    ETH->PTPTSCR |= ETH_PTPTSCR_TSSARFE;              /* time-stamp all received frames (PTP over UDP) */
    ETH->PTPTSHUR = 0; ETH->PTPTSLUR = 0;
    ETH->PTPTSCR |= ETH_PTPTSCR_TSSTI; while (ETH->PTPTSCR & ETH_PTPTSCR_TSSTI) { }
}
void ptp_hw_adjust_ppb(double ppb)
{
    if (ppb >  500000) ppb =  500000;                 /* +-500 ppm slew limit */
    if (ppb < -500000) ppb = -500000;
    ETH->PTPTSAR = (uint32_t)((double)base_addend * (1.0 + ppb * 1e-9));
    ETH->PTPTSCR |= ETH_PTPTSCR_TSARU;
}
void ptp_hw_step(int64_t off)
{
    int neg = off < 0; uint64_t a = neg ? (uint64_t)(-off) : (uint64_t)off;
    ETH->PTPTSHUR = (uint32_t)(a / 1000000000ULL);
    ETH->PTPTSLUR = (uint32_t)(a % 1000000000ULL) | (neg ? 0x80000000U : 0);   /* ADDSUB bit */
    ETH->PTPTSCR |= ETH_PTPTSCR_TSSTU; while (ETH->PTPTSCR & ETH_PTPTSCR_TSSTU) { }
}
void ptp_hw_set_target(uint32_t sec)
{
    ETH->PTPTTHR = sec; ETH->PTPTTLR = 0;             /* target time -> PTP trigger -> TIM2 ITR1 */
    ETH->PTPTSCR |= ETH_PTPTSCR_TSITE;
}
uint64_t ptp_hw_now_ns(void)
{
    uint32_t s = ETH->PTPTSHR, ns = ETH->PTPTSLR & 0x7FFFFFFF;
    return (uint64_t)s * 1000000000ULL + ns;
}
#else
void ptp_hw_init(void) {}  void ptp_hw_adjust_ppb(double p) { (void)p; }  void ptp_hw_step(int64_t o) { (void)o; }
void ptp_hw_set_target(uint32_t s) { (void)s; }  uint64_t ptp_hw_now_ns(void) { return 0; }
#endif
