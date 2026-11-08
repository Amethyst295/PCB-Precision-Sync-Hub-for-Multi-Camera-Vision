/* Trigger generation and strobe capture - the hardware-timed core of the Sync Hub.
 *
 *  TIM2 (32-bit, 84 MHz)  : frame master. Update event -> TRGO. On the F407 its counter is started by the
 *                           Ethernet PTP "target time" trigger (ITR1 remapped to PTP), so every frame boundary
 *                           is phase-locked to the PTP second. CH4 captures the PPS loop-back (PA3).
 *  TIM3, TIM4 (84 MHz)    : slave, trigger mode on ITR1 (= TIM2 TRGO), one-pulse mode, PWM2 on 4 channels each.
 *                           Rising edge of camera i at CCRi (programmable delay), falling edge at ARR (common).
 *                           Both start on the same TRGO edge -> 0 cycles skew between the 8 outputs.
 *  TIM1, TIM9 (APB2) and TIM12 : slave RESET mode on the frame trigger, input-capture of the camera strobe
 *                           (falling edge, open-collector) -> trigger-to-exposure latency per camera.        */
#include "sync_timers.h"
#include "gpio.h"
volatile uint16_t g_strobe_lat[N_CAM];
volatile uint8_t  g_strobe_cnt[N_CAM];
volatile uint32_t g_pps_capture, g_pps_flag, g_frame_count;
static uint32_t s_nominal_arr;

static void oc_pwm2(TIM_TypeDef *t)
{
    t->CCMR1 = (7U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE | (7U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    t->CCMR2 = (7U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE | (7U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;
    t->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;      /* active high */
}
static void slave_one_pulse(TIM_TypeDef *t, uint16_t arr)
{
    t->PSC = 0; t->ARR = arr;
    t->CR1 = TIM_CR1_OPM | TIM_CR1_ARPE;
    t->SMCR = (1U << TIM_SMCR_TS_Pos) | (6U << TIM_SMCR_SMS_Pos);   /* TS=ITR1 (TIM2 TRGO), SMS=trigger mode */
    oc_pwm2(t);
    t->EGR = TIM_EGR_UG;
}
static void capture_falling(TIM_TypeDef *t, int nch, uint32_t ts, uint16_t psc)
{
    t->PSC = psc; t->ARR = 0xFFFF;
    t->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos) | (3U << TIM_CCMR1_IC1F_Pos) | (1U << TIM_CCMR1_CC2S_Pos) | (3U << TIM_CCMR1_IC2F_Pos);
    if (nch > 2) t->CCMR2 = (1U << TIM_CCMR2_CC3S_Pos) | (3U << TIM_CCMR2_IC3F_Pos) | (1U << TIM_CCMR2_CC4S_Pos) | (3U << TIM_CCMR2_IC4F_Pos);
    t->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P;
    if (nch > 2) t->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC4E | TIM_CCER_CC4P;
    t->SMCR = (ts << TIM_SMCR_TS_Pos) | (4U << TIM_SMCR_SMS_Pos);   /* reset mode: CNT=0 at frame trigger */
    t->DIER = TIM_DIER_CC1IE | TIM_DIER_CC2IE | (nch > 2 ? (TIM_DIER_CC3IE | TIM_DIER_CC4IE) : 0);
    t->EGR = TIM_EGR_UG; t->CR1 = TIM_CR1_CEN;
}

void sync_timers_init(const sync_cfg_t *cfg)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN;
    for (unsigned p = 6; p <= 9; p++)   gpio_af(GPIOC, p, 2);       /* TRIG1..4 */
    for (unsigned p = 12; p <= 15; p++) gpio_af(GPIOD, p, 2);       /* TRIG5..8 */
    gpio_af(GPIOE, 9, 1); gpio_af(GPIOE, 11, 1); gpio_af(GPIOE, 13, 1); gpio_af(GPIOE, 14, 1);   /* STB1..4 */
    gpio_af(GPIOE, 5, 3); gpio_af(GPIOE, 6, 3);                                                 /* STB5..6 */
    gpio_af(GPIOA, 3, 1);                                                                        /* PPS loop-back */

    /* ---- slaves first, so they are waiting when the master starts ---- */
    slave_one_pulse(TIM3, cfg->pulse_end_ticks);
    slave_one_pulse(TIM4, cfg->pulse_end_ticks);
    for (int i = 0; i < N_CAM; i++) sync_set_delay(i, cfg->delay_ticks[i]);
    TIM3->EGR = TIM_EGR_UG; TIM4->EGR = TIM_EGR_UG;
    TIM4->CR2 = (1U << TIM_CR2_MMS_Pos);          /* TIM4 TRGO = enable -> resets TIM12 capture */

    /* ---- strobe capture: TIM1/TIM9 @84 MHz (168 MHz /2), ITR = TIM2 ---- */
    capture_falling(TIM1, 4, 1, APB2_CAP_PSC);    /* TIM1 ITR1 = TIM2 */
    capture_falling(TIM9, 2, 0, APB2_CAP_PSC);    /* TIM9 ITR0 = TIM2 */
#ifndef SIM_F401
    RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
    gpio_af(GPIOB, 14, 9); gpio_af(GPIOB, 15, 9);                                                /* STB7..8 */
    capture_falling(TIM12, 2, 0, 0);              /* TIM12 ITR0 = TIM4 (starts at the frame trigger) */
    NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
#endif
    NVIC_EnableIRQ(TIM1_CC_IRQn); NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

    /* ---- TIM2 master ---- */
    s_nominal_arr = cfg->frame_period_ticks - 1;
    TIM2->PSC = 0; TIM2->ARR = s_nominal_arr;
    TIM2->CR2 = (2U << TIM_CR2_MMS_Pos);          /* TRGO on update */
    TIM2->CCMR2 = (1U << TIM_CCMR2_CC4S_Pos);     /* CH4 = input capture of PPS (rising) */
    TIM2->CCER = TIM_CCER_CC4E;
    TIM2->DIER = TIM_DIER_CC4IE | TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    NVIC_EnableIRQ(TIM2_IRQn);
#ifdef SIM_NOETH
    TIM2->CR1 |= TIM_CR1_CEN;                     /* simulation: free-running frame clock */
#else
    sync_arm_on_ptp_target();                     /* hardware: wait for the PTP second */
#endif
}

void sync_set_delay(int cam, uint16_t ticks)
{
    TIM_TypeDef *t = (cam < 4) ? TIM3 : TIM4;
    volatile uint32_t *ccr[4] = { &t->CCR1, &t->CCR2, &t->CCR3, &t->CCR4 };
    if (ticks == 0) ticks = 1;
    *ccr[cam & 3] = ticks;
}

/* Frequency steering of the frame clock (software PLL): ppb > 0 makes frames shorter */
void sync_trim_period(int32_t ppb)
{
    int64_t d = ((int64_t)(s_nominal_arr + 1) * ppb) / 1000000000LL;
    TIM2->ARR = (uint32_t)((int64_t)s_nominal_arr - d);
}

void sync_arm_on_ptp_target(void)
{
#ifndef SIM_NOETH
    TIM2->OR = TIM_OR_ITR1_RMP_0;                 /* ITR1 <- Ethernet PTP trigger output */
    TIM2->SMCR = (1U << TIM_SMCR_TS_Pos) | (6U << TIM_SMCR_SMS_Pos);   /* start TIM2 on PTP target time */
#endif
}

void TIM2_IRQHandler(void)
{
    uint32_t sr = TIM2->SR;
    if (sr & TIM_SR_CC4IF) { g_pps_capture = TIM2->CCR4; g_pps_flag = 1; }
    if (sr & TIM_SR_UIF)   { g_frame_count++; }
    TIM2->SR = ~sr;
}
static void cap(TIM_TypeDef *t, int base, int n)
{
    uint32_t sr = t->SR;
    if (sr & TIM_SR_CC1IF) { g_strobe_lat[base+0] = t->CCR1; g_strobe_cnt[base+0]++; }
    if (sr & TIM_SR_CC2IF) { g_strobe_lat[base+1] = t->CCR2; g_strobe_cnt[base+1]++; }
    if (n > 2 && (sr & TIM_SR_CC3IF)) { g_strobe_lat[base+2] = t->CCR3; g_strobe_cnt[base+2]++; }
    if (n > 2 && (sr & TIM_SR_CC4IF)) { g_strobe_lat[base+3] = t->CCR4; g_strobe_cnt[base+3]++; }
    t->SR = ~sr;
}
void TIM1_CC_IRQHandler(void)        { cap(TIM1, 0, 4); }
void TIM1_BRK_TIM9_IRQHandler(void)  { cap(TIM9, 4, 2); }
#ifndef SIM_F401
void TIM8_BRK_TIM12_IRQHandler(void) { cap(TIM12, 6, 2); }
#endif
