#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "gpioControl.h"
#include "stm32f439xx.h"
#include "homeManagement.h"

// Clock speed is unfortunately not determined by the TIM_TypeDef
#define AHB1_ClkSpeed 168000000
#define APB2_ClkSpeed 84000000
#define APB1_ClkSpeed 42000000

// UART Receive constants
#define MAX_RECEIVED_CHARS 2 // Only 2 characters can be typed into the terminal
#define BACKSPACE 0x7F
#define CARRIAGE_RETURN '\r'
#define LINE_FEED '\n'
#define HEADER '!'
#define COMMANDS_START 0x40
#define COMMANDS_END 0x4F
#define HMS_TO_PC_STATUS 0x30
#define LIGHT_CTRL_Pos 0x08
#define FAN_CTRL_Pos 0x04
#define HEATER_CTRL_Pos 0x02
#define COOLER_CTRL_Pos 0x01

typedef uint32_t ASCII;
/* See gpioControl.h for GPIO interface */
typedef struct UARTInterface {
    uint32_t * receive_data_ptr; // location of current character in receive_data
    uint32_t received_char;
    uint32_t receive_data[3]; // 2 chars max + 1 for end of array
    bool command_received;
    bool is_controlling_HMS;
    bool in_monitoring_mode;
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
void initADC3(void);

/*** UART Communication stuff **/

// Be extremely careful with UARTprint. 
// Make sure strlen is correct to avoid overflow, and/or str is null terminated.
void UARTPrintLn(const char* str);
void transmitChar(ASCII);
// Print human-readable statuses,
// then send the status according to the communication protocol
void sendHMSStatus(Thermometer* thermometer, 
                   Component* light, 
                   Component* fan, 
                   Component* heater,
                   Component* cooler);
void receiveChar(UARTInterface*);
void processCommand(UARTInterface* uart,
                   Component* light, 
                   Component* fan, 
                   Component* heater,
                   Component* cooler);

#endif //PERIPHERALS_H