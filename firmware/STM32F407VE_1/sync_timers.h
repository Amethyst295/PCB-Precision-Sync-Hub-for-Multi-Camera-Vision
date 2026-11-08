#ifndef SYNC_TIMERS_H
#define SYNC_TIMERS_H
#include <stdint.h>
#include "board.h"
typedef struct {
    uint32_t frame_period_ticks;          /* TIM2 period @84 MHz (e.g. 30 fps -> 2 800 000)        */
    uint16_t delay_ticks[N_CAM];          /* per-camera rising-edge delay @84 MHz (11.9 ns step)   */
    uint16_t pulse_end_ticks;             /* common falling edge (one-pulse ARR)                   */
} sync_cfg_t;
void sync_timers_init(const sync_cfg_t *cfg);
void sync_set_delay(int cam, uint16_t ticks);           /* AI/latency compensation hook */
void sync_trim_period(int32_t ppb);                     /* frequency steering from PTP servo */
void sync_arm_on_ptp_target(void);                      /* F407: TIM2 starts on PTP target time */
/* strobe (exposure-start) latency of the last frame, ticks @84 MHz, 0xFFFF = no strobe */
extern volatile uint16_t g_strobe_lat[N_CAM];
extern volatile uint8_t  g_strobe_cnt[N_CAM];
extern volatile uint32_t g_pps_capture, g_pps_flag, g_frame_count;
#endif
