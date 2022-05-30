#include "peripherals.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stm32f439xx.h"
#include "core_cm4.h"

#include "homeManagement.h"

void initUART(UARTInterface *uart_interface)
{
    // Initialise the UART receive data pointer
    if (uart_interface->receive_data_ptr == 0) {
        uart_interface->receive_data_ptr = &uart_interface->receive_data[0];
    }
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
    // Set prescaler for a clock speed of 4000Hz.
    //  We choose a prescaled clock speed of 4000Hz because it's easy to calculate with,
    //  and the prescaler value for all clock speeds fits in 16 bits.
    timer->timer_component->PSC = (uint16_t)((timer->clock_speed / 4000) - 1);
    timer->timer_component->DIER |= TIM_DIER_UIE;        // Enable interrupts
    timer->timer_component->CR1 |= TIM_CR1_ARPE;         // Enable ARR preload
    timer->timer_component->ARR = 4 * timer->time_in_ms; // Set the value to count up to

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

void initADC3(void)

{
    ADC123_COMMON->CCR &= ~(ADC_CCR_VBATE);                                 // disable vbate
    ADC123_COMMON->CCR |= (ADC_CCR_TSVREFE) | (0x03 << ADC_CCR_ADCPRE_Pos); // enable temperature sensor
    ADC3->CR1 &= ~((ADC_CR1_SCAN) | (0x00 << ADC_CR1_RES_Pos));             // disable scan mode and resolution of 12bits (15 ADCCLK cycles)
    ADC3->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_SWSTART);         // right aligment, single conversion mode, reset state
    ADC3->SQR3 &= ~(ADC_SQR3_SQ1_Msk);                                      // clear first conversion
    ADC3->SQR3 |= 0x08;                                                     // conversion at 8th channel
    ADC3->SQR1 &= ~(ADC_SQR1_L_Msk);                                        // 1 conversion
    ADC3->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk);                                   // clear sample time (cycles)
    ADC3->SMPR2 |= 0x03 << ADC_SMPR2_SMP0_Pos;                              // set sample time of SMP0 to 56 cycles
    ADC3->CR2 |= ADC_CR2_ADON;
    return;
}

void UARTPrintLn(const char* str) {
    size_t length = strlen(str);
    for (size_t i = 0; i < length; i++) {
        if (str[i] == '\0') {
            transmitChar(CARRIAGE_RETURN);
            transmitChar(LINE_FEED);
            return;
        }
        transmitChar((ASCII)str[i]);
    }
    transmitChar(CARRIAGE_RETURN);
    transmitChar(LINE_FEED);
    return;
}
void transmitChar(ASCII val)
{
    while ((USART3->SR & USART_SR_TXE) == 0x00);
    USART3->DR = val;
    while ((USART3->SR & USART_SR_TC) == 0x00);
    return;
}
void receiveChar(UARTInterface *uart)
{
    // The first time we've received user input, let the user know we're pausing monitoring mode
    if (uart->in_monitoring_mode == true) {
        UARTPrintLn("User input detected. Pausing monitoring...");
    }
    uart->in_monitoring_mode = false;
    
    // EOL detected?
    if (uart->received_char == CARRIAGE_RETURN)
    {
        // Reset the pointer, we can now process the given command.
        uart->receive_data_ptr = &uart->receive_data[0];
        uart->command_received = true;
        transmitChar(CARRIAGE_RETURN);
        transmitChar(LINE_FEED);
        return;
    }
    // Undo input
    if (uart->received_char == BACKSPACE) {
        // If the array is empty, do nothing
        if (uart->receive_data_ptr == &uart->receive_data[0]) {
            return;
        }
        uart->receive_data_ptr--;
        transmitChar(BACKSPACE);
        return;
    }
    // User has already typed in 2 characters, and array end has been reached.
    if (uart->receive_data_ptr == &uart->receive_data[MAX_RECEIVED_CHARS])
    {
        // Ignore everything until an EOL is sent.
        return;
    }
    // Store the received character and increment the pointer.
    *uart->receive_data_ptr = uart->received_char;
    uart->receive_data_ptr++;

    transmitChar(uart->received_char); // feedback to the terminal
    return;
}
void transmitTemperature(float temp) {
    char temperature_str[4];
    int temperature_plus_dec = (int)(temp * 10); // Convert float to temp, but keep the first decimal number

    sprintf(temperature_str, "%d", temperature_plus_dec); //Convert to temperature to string
    
    // Insert the decimal character and print, depending on how many digits the temperature has
    if (temp < 10) {
        temperature_str[2] = temperature_str[1];
        temperature_str[1] = '.';
        for (int i = 0; i < 3; i++) {
            transmitChar((ASCII)temperature_str[i]);
        }
        return;
    } else if (temp < 100) {
        temperature_str[3] = temperature_str[2];
        temperature_str[2] = '.';
        for (int i = 0; i < 4; i++) {
            transmitChar((ASCII)temperature_str[i]);
        }
        return;
    } 
    // Don't worry about temp > 100, because if that was the case we'd all be dead
    return;
}
void sendHMSStatus(Thermometer *thermometer, Component *light, Component *fan, Component *heater, Component *cooler)
{
        ASCII status = HMS_TO_PC_STATUS;

        // 2. Determine status
        if (light->is_on == true) {
            status |= LIGHT_CTRL_Pos;
        } else { 
            status &= ~(LIGHT_CTRL_Pos);
        }
        if (fan->is_on == true) {
            status |= FAN_CTRL_Pos;
        } else {
            status &= ~(FAN_CTRL_Pos);
        }
        if (heater->is_on == true) {
            status |= HEATER_CTRL_Pos;
        } else {
            status &= ~(HEATER_CTRL_Pos);
        }
        if (cooler->is_on == true) {
            status |= COOLER_CTRL_Pos;
        } else {
            status &= ~(COOLER_CTRL_Pos);
        }
        transmitChar(HEADER);
        transmitTemperature(thermometer->celcius);
        transmitChar(status);
        transmitChar(CARRIAGE_RETURN);
        transmitChar(LINE_FEED);
    return;
}

void runCommand(uint32_t command, Component* light, Component* fan, Component* heater, Component* cooler) {
    uint32_t light_on = (command & LIGHT_CTRL_Pos);
    uint32_t fan_on = (command & FAN_CTRL_Pos);
    uint32_t heater_on = (command & HEATER_CTRL_Pos);
    uint32_t cooler_on = (command & COOLER_CTRL_Pos);
    if (light_on) {
        UARTPrintLn("Light: ON");
        turnOn(light);
    } else {
        UARTPrintLn("Light: OFF");
        turnOff(light);
    }
    if (fan_on) {
        UARTPrintLn("Fan: ON");
        turnOn(fan);
    } else {
        UARTPrintLn("Fan: OFF");
        turnOff(fan);
    }
    if (heater_on) {
        UARTPrintLn("Heater: ON");
        turnOn(heater);
    } else {
        UARTPrintLn("Heater: OFF");
        turnOff(heater);
    }
    if (cooler_on) {
        if (heater_on) {
            UARTPrintLn("Heater is on, unable to turn on cooler.");
        } else {
            UARTPrintLn("Cooler: ON");
            turnOn(cooler);
        }
    } else {
        UARTPrintLn("Cooler: OFF");
        turnOff(cooler);
    }
    return;
}
void processCommand(UARTInterface* uart, Component* light, Component* fan, Component* heater, Component* cooler) {
    uart->command_received = false;
    // Command must start with !
    if (uart->receive_data[0] != '!') {
        UARTPrintLn("Error: Command must start with !");
        transmitChar(CARRIAGE_RETURN);
        transmitChar(LINE_FEED);
        // Reset received data
        uart->receive_data[0] = 0;
        uart->receive_data[1] = 0;
        // Go back to monitoring mode
        UARTPrintLn("Resuming monitoring..");
        uart->in_monitoring_mode = true;
        return;
    }
    // And be a value between COMMANDS_START and COMMANDS_END
    if (uart->receive_data[1] < COMMANDS_START || uart->receive_data[1] > COMMANDS_END) {
        UARTPrintLn("Error: valid commands are !@ and !(A-O)");
        UARTPrintLn("For example, !H is valid (only have light turned on)");
        // Reset received data
        uart->receive_data[0] = 0;
        uart->receive_data[1] = 0;
        // Go back to monitoring mode
        UARTPrintLn("Resuming monitoring..");
        uart->in_monitoring_mode = true;
        return;
    }
    UARTPrintLn("Command received!");
    uart->is_controlling_HMS = true;
    runCommand(uart->receive_data[1], light, fan, heater, cooler);

    // Reset received data
    uart->receive_data[0] = 0;
    uart->receive_data[1] = 0;
    // Go back to monitoring mode
    UARTPrintLn("Resuming monitoring..");
    uart->in_monitoring_mode = true;
    return;
}