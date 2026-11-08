#include <stdint.h>
#include "ptp_servo.h"
#include "anomaly_tree.h"
volatile double g_out[3]; volatile int g_cls;
__attribute__((noinline)) void bench_servo(int64_t k){ ptp_exchange_t e={k*125000000LL, k*125000000LL+48000+(k*37%900), k*125000000LL+60000, k*125000000LL+108500+(k*53%1200)}; double o,b; g_out[0]=servo_update(&e,&o,&b); g_out[1]=o; }
__attribute__((noinline)) void bench_tree(float a){ float x[9]={0.2f*a,0.1f,0.0f,0,0.1f,25.0f,1.2f,0.3f,40.0f}; g_cls=anomaly_classify(x); }
__attribute__((noinline)) void bench_init(void){ servo_cfg_t sc={.kp=0.01,.ki=0.0001,.W=64,.gate_us=0.5,.clip_us=5.0,.ema=1.0/16,.ood_z=3.0f,.ai_enable=1}; servo_init(&sc);}
int main(void){ bench_init(); for(int64_t k=0;;k++){bench_servo(k);bench_tree((float)k);} }
