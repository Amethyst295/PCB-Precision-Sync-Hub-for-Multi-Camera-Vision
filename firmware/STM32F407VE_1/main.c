/* Precision Sync Hub - main loop.  AI-SED 2026, team Borrenautas.
 * Builds: make            -> STM32F407VET6 hardware image (PTP + 8 triggers + monitor)
 *         make sim407     -> STM32F407VE image for Proteus VSM (no Ethernet, free-running frame clock)
 *         make sim        -> STM32F401VE image for Proteus VSM (fallback) */
#include <stdint.h>
#include "board.h"
#include "gpio.h"
#include "sync_timers.h"
#include "sync_monitor.h"
#include "ptp_servo.h"
#include "ptp_hw.h"
void clock_init(void);
void uart_init(void);
void uart_printf(const char *fmt, ...);

#define FPS 30
static const char *FAULT_NAME[] = { "OK", "DISCONNECTED", "INTERMITTENT", "LAT_DRIFT", "DOUBLE_TRIG", "PTP_LOSS", "CONGESTION", "OSC_FAULT" };

int main(void)
{
    clock_init();
    uart_init();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    gpio_out(GPIOD, 0); gpio_out(GPIOD, 1); gpio_out(GPIOD, 2);          /* LEDs */
    gpio_out(GPIOE, 3); gpio_set(GPIOE, 3, 1);                          /* release PHY reset */
    uart_printf("\nSync Hub fw - %s build\n",
#ifdef SIM_NOETH
                "SIM (Proteus VSM, no Ethernet)"
#else
                "HW (F407)"
#endif
    );
    sync_cfg_t cfg = { .frame_period_ticks = TIMCLK_APB1_HZ / FPS, .pulse_end_ticks = 8400 /* 100 us */ };
    for (int i = 0; i < N_CAM; i++) cfg.delay_ticks[i] = 840;           /* 10 us default pre-delay */
#ifndef SIM_NOETH
    ptp_hw_init();
    servo_cfg_t sc = { .kp = 0.01, .ki = 0.0001, .W = 64, .gate_us = 0.5, .clip_us = 5.0, .ema = 1.0 / 16, .ood_z = 3.0f, .ai_enable = 1 };
    servo_init(&sc);
    ptp_hw_set_target(2);                                               /* frames start at PTP second 2 */
#endif
    sync_timers_init(&cfg);

    uint32_t last_frame = 0, sec = 0; int32_t pps_I = 0;
    hub_health_t hh = { 0 };
    for (;;) {
        if (g_frame_count != last_frame) {                              /* ---- every frame ---- */
            last_frame = g_frame_count;
            monitor_on_frame();
            if (last_frame % FPS == 0) {                                /* ---- every second ---- */
                sec++;
                monitor_on_second(&hh);
                int fault = 0;
                for (int c = 0; c < N_CAM; c++) if (monitor_state(c) != FAULT_NORMAL) fault = 1;
                gpio_set(GPIOD, 2, fault); gpio_set(GPIOD, 0, sec & 1);
                uart_printf("t=%lus", (unsigned long)sec);
                for (int c = 0; c < N_CAM; c++) {                       /* integer print: no float printf needed */
                    unsigned long cus = (unsigned long)(monitor_latency_us(c) * 100.0f + 0.5f);
                    uart_printf(" C%d:%lu.%02luus/%s", c + 1, cus / 100, cus % 100, FAULT_NAME[monitor_state(c)]);
                }
                uart_printf("\n");
                /* exposure alignment: every 10 s equalise camera latencies (pre-delay faster cameras) */
                if (sec > 60 && sec % 10 == 0) {
                    float mx = 0; for (int c = 0; c < N_CAM; c++) if (monitor_latency_us(c) > mx) mx = monitor_latency_us(c);
                    for (int c = 0; c < N_CAM; c++) if (monitor_latency_us(c) > 0)
                        sync_set_delay(c, (uint16_t)(840 + (mx - monitor_latency_us(c)) * 84.0f));
                }
            }
        }
        if (g_pps_flag) {                                               /* ---- PPS: discipline the frame clock ---- */
            g_pps_flag = 0;
            int32_t per = (int32_t)(TIMCLK_APB1_HZ / FPS);
            int32_t ph = (int32_t)g_pps_capture; if (ph > per / 2) ph -= per;   /* ticks early/late */
            hh.pps_phase_err_ns = (float)ph * 11.905f; if (hh.pps_phase_err_ns < 0) hh.pps_phase_err_ns = -hh.pps_phase_err_ns;
            pps_I += ph;
            sync_trim_period(-(ph * 2000 + pps_I * 200));               /* PI in ppb (small, slow loop) */
        }
#ifndef SIM_NOETH
        /* PTP stack integration point: for each completed Sync/Delay_Req exchange call
         *   double off, bias; double ppb = servo_update(&ex, &off, &bias); ptp_hw_adjust_ppb(-ppb);  /* negative feedback: offset > 0 -> slow the clock */
         *   hh.ptp_offset_ns = |off|; hh.path_delay_std_us = ...; hh.freq_corr_rate_ppb_s = ...;      */
#endif
    }
}
