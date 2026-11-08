/* Per-camera synchronisation monitor:
 *  - measures trigger->exposure latency of every camera from the strobe captures,
 *  - learns each camera's baseline during the first 60 s,
 *  - every second builds the 9-feature vector used in training (07_IA/A4) and runs the
 *    exported decision tree, then a 3-of-3 persistence filter to suppress false alarms. */
#include "mathlite.h"
#include <string.h>
#include "sync_monitor.h"
#include "sync_timers.h"
#define TICK_US (1e6f / 84e6f)
#define K_PERSIST 3
static float sum[N_CAM], sum2[N_CAM]; static int nrx[N_CAM], nframes, extra[N_CAM];
static uint8_t last_cnt[N_CAM];
static float base[N_CAM]; static int base_n; static float hist_ld[N_CAM][30]; static int hist_i;
static sync_fault_t state[N_CAM], cand[N_CAM]; static int run_[N_CAM]; static float lat_mean[N_CAM];

void monitor_on_frame(void)
{
    nframes++;
    for (int c = 0; c < N_CAM; c++) {
        uint8_t n = (uint8_t)(g_strobe_cnt[c] - last_cnt[c]); last_cnt[c] = g_strobe_cnt[c];
        if (n >= 1) { float l = g_strobe_lat[c] * TICK_US; sum[c] += l; sum2[c] += l * l; nrx[c]++; }
        if (n > 1) extra[c] += n - 1;
    }
}
void monitor_on_second(const hub_health_t *h)
{
    for (int c = 0; c < N_CAM; c++) {
        float m = nrx[c] ? sum[c] / nrx[c] : 0, sd = nrx[c] > 1 ? ml_sqrtf(ml_fmaxf(sum2[c] / nrx[c] - m * m, 0)) : 0;
        lat_mean[c] = m;
        if (base_n < 60) { base[c] += m / 60.0f; }                 /* calibration: first minute */
        float ld = (nrx[c] ? m - base[c] : 0);
        float slope = (ld - hist_ld[c][hist_i]) * 2.0f;          /* us/min over the last 30 s */
        hist_ld[c][hist_i] = ld;
        float x[ANOM_NFEAT] = { ld, sd, nframes ? 1.0f - (float)nrx[c] / nframes : 0, (float)extra[c], slope,
                                h->ptp_offset_ns, h->path_delay_std_us, h->freq_corr_rate_ppb_s, h->pps_phase_err_ns };
        sync_fault_t p = (base_n < 60) ? FAULT_NORMAL : anomaly_classify(x);
        if (p != FAULT_NORMAL && p == cand[c]) run_[c]++; else { cand[c] = p; run_[c] = (p != FAULT_NORMAL); }
        state[c] = (run_[c] >= K_PERSIST) ? cand[c] : FAULT_NORMAL;
        sum[c] = sum2[c] = 0; nrx[c] = 0; extra[c] = 0;
    }
    hist_i = (hist_i + 1) % 30; nframes = 0; if (base_n < 60) base_n++;
}
sync_fault_t monitor_state(int cam) { return state[cam]; }
float monitor_latency_us(int cam) { return lat_mean[cam]; }
