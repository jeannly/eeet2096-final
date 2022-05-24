#include "peripherals.h"

#include <stdbool.h>
#include "stm32f439xx.h"
#include "core_cm4.h"

void initUART(UARTInterface *uart_interface)
{
    // 16x over sampling
    uart_interface->uart->CR1 &= ~(USART_CR1_OVER8);

    // 57.6k baudrate (assuming a clock speed of 42MHz)
    uart_interface->uart->BRR &= 0xFFFF0000;
    uart_interface->uart->BRR |= (45 << USART_BRR_DIV_Mantissa_Pos) | (9 << USART_BRR_DIV_Fraction_Pos);

    // 8 bits per transfer
    uart_interface->uart->CR1 &= ~(USART_CR1_M);

    // Number of stop bits = 1
    uart_interface->uart->CR2 &= ~(USART_CR2_STOP_Msk);
    uart_interface->uart->CR2 |= (0 << USART_CR2_STOP_Pos);

    // No parity
    uart_interface->uart->CR1 &= ~(USART_CR1_PCE);

    // Async and no clock
    uart_interface->uart->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA);

    // Disable hardware flow control
    uart_interface->uart->CR2 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

    // Enable interrupts for RXNE, data received (**warning: and consequently overrun error***)
    uart_interface->uart->CR1 |= (USART_CR1_RXNEIE);

    // Now that the configuration is complete, enable the USART, TX, RX sections.
    uart_interface->uart->CR1 |= (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE);
    return;
}
void initTimer(Timer *timer)
{
    // Set prescaler for a clock speed of 5000Hz.
    //  We choose a prescaled clock speed of 5000Hz because it's easy to calculate with,
    //  and the prescaler value for all clock speeds fits in 16 bits.
    timer->timer_component->PSC = (uint16_t)((timer->clock_speed / 5000) - 1);
    timer->timer_component->DIER |= TIM_DIER_UIE;        // Enable interrupts
    timer->timer_component->CR1 |= TIM_CR1_ARPE;         // Enable ARR preload
    timer->timer_component->ARR = 5 * timer->time_in_ms; // Set the value to count up to

    // This should check if the timer is a general timer,
    //  but since TIM10 is the only general timer we're using, we're just simplifying the logic for time sake.
    if (timer->timer_component == TIM10)
    {
        // Disable the fancy capture/compare output stuff
        timer->timer_component->CCMR1 &= ~(TIM_CCMR1_OC1M);
        timer->timer_component->CCER &= ~(TIM_CCER_CC1E);
    }
    if (timer->in_one_pulse_mode == true)
    {
        timer->timer_component->CR1 |= TIM_CR1_OPM; // Enable one pulse mode (TIM_CR1_CEN will be cleared after counting)
    }
    if (timer->is_running == true)
    {
        timer->timer_component->CR1 |= TIM_CR1_CEN; // Enable the timer
    }
    return;
}