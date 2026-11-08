#ifndef GPIO_H
#define GPIO_H
#include "board.h"
static inline void gpio_af(GPIO_TypeDef *g, unsigned pin, unsigned af)
{
    g->MODER   = (g->MODER & ~(3U << (2*pin))) | (2U << (2*pin));
    g->OSPEEDR |= (3U << (2*pin));                              /* very-high speed (fast edges for triggers) */
    if (pin < 8) g->AFR[0] = (g->AFR[0] & ~(0xFU << (4*pin))) | (af << (4*pin));
    else         g->AFR[1] = (g->AFR[1] & ~(0xFU << (4*(pin-8)))) | (af << (4*(pin-8)));
}
static inline void gpio_out(GPIO_TypeDef *g, unsigned pin)
{
    g->MODER = (g->MODER & ~(3U << (2*pin))) | (1U << (2*pin));
}
static inline void gpio_set(GPIO_TypeDef *g, unsigned pin, int on) { g->BSRR = on ? (1U << pin) : (1U << (pin + 16)); }
#endif
