#ifndef PTP_HW_H
#define PTP_HW_H
#include <stdint.h>
void ptp_hw_init(void);                         /* MAC IEEE-1588 clock: fine update, 20 ns sub-second increment */
void ptp_hw_adjust_ppb(double ppb);             /* frequency correction from the servo */
void ptp_hw_step(int64_t offset_ns);            /* coarse phase step (first lock only) */
void ptp_hw_set_target(uint32_t sec);           /* fire the TIM2 start trigger at sec.000000000 */
uint64_t ptp_hw_now_ns(void);
#endif
