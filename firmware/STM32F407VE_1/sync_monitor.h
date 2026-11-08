#ifndef SYNC_MONITOR_H
#define SYNC_MONITOR_H
#include <stdint.h>
#include "board.h"
#include "anomaly_tree.h"
typedef struct { float ptp_offset_ns, path_delay_std_us, freq_corr_rate_ppb_s, pps_phase_err_ns; } hub_health_t;
void monitor_on_frame(void);                                  /* call once per frame (after TIM2 update) */
void monitor_on_second(const hub_health_t *h);                /* features -> tree -> persistence filter */
sync_fault_t monitor_state(int cam);
float monitor_latency_us(int cam);
#endif
