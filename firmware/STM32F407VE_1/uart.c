#include <stdarg.h>
#include <stdio.h>
#include "board.h"
#include "gpio.h"
void uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    gpio_af(GPIOA, 9, 7); gpio_af(GPIOA, 10, 7);
    USART1->BRR = 729;                                /* 84 MHz / 115200 */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}
static void putc_(char c) { while (!(USART1->SR & USART_SR_TXE)) { } USART1->DR = (uint8_t)c; }
void uart_printf(const char *fmt, ...)
{
    char b[160]; va_list a; va_start(a, fmt); int n = vsnprintf(b, sizeof b, fmt, a); va_end(a);
    for (int i = 0; i < n && i < (int)sizeof b; i++) { if (b[i] == '\n') putc_('\r'); putc_(b[i]); }
}
