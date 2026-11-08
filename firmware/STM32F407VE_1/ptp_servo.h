#ifndef PTP_SERVO_H
#define PTP_SERVO_H
#include <stdint.h>
/* One PTP exchange (all in ns): t1 master send, t2 slave rx, t3 slave send, t4 master rx */
typedef struct { int64_t t1, t2, t3, t4; } ptp_exchange_t;
typedef struct {
    double kp, ki;            /* PI gains (tuned in simulation, see 07_IA/A3)              */
    int    W;                 /* lucky-packet window (samples @ 8 Hz)                      */
    double gate_us, clip_us;  /* AI bias-correction gate and clip                          */
    double ema;               /* low-pass of the AI bias estimate                          */
    float  ood_z;             /* out-of-distribution guard (|z-score| limit, 3.0)          */
    int    ai_enable;
} servo_cfg_t;
void   servo_init(const servo_cfg_t *c);
/* returns the frequency correction to apply (ppb); *offset_ns = filtered offset estimate */
double servo_update(const ptp_exchange_t *e, double *offset_ns, double *ai_bias_ns);
#endif
