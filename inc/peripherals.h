#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <stdbool.h>
#include <stdint.h>
#include "gpioControl.h"
#include "stm32f439xx.h"

// Clock speed is unfortunately not determined by the TIM_TypeDef
#define AHB1_ClkSpeed 168000000
#define APB2_ClkSpeed 84000000
#define APB1_ClkSpeed 42000000

/* See gpioControl.h for GPIO interface */
typedef struct UARTInterface {
    uint8_t last_byte_received;
    uint32_t transmit_data;
    uint32_t receive_data;
    bool is_controlling_ACSystem;
    USART_TypeDef * uart; // The uart component to be used
    GPIO_Config* tx_config;
    GPIO_Config* rx_config;
} UARTInterface;

typedef struct Timer {
    bool is_running;
    uint16_t time_in_ms;
    bool in_one_pulse_mode;
    uint32_t clock_speed;
    TIM_TypeDef * timer_component; // The specific hardware timer to be used
} Timer;


/****************** FUNCTIONS *****************/
/* initUART:
    Configure UART to 57,600bps, 8 data-bits, No Parity and 1 Stop Bit
*/
void initUART(UARTInterface*);
/* initTimer:
    Configure a timer's control register, enable its interrupts, and enable it if ready.
*/
void initTimer(Timer*);

#endif //PERIPHERALS_H