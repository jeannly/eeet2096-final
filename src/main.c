/********************************************
 * A project designed to imitate a home management system.
 * RMIT University - EEET2096 Final Project
 *	Authors: Dr. Glenn Matthews, Jean Yap, Xavier Um													*
 ********************************************/

#include <stdbool.h>
#include <stdint.h>
#include "stm32f439xx.h"
#include "boardSupport.h"
#include "core_cm4.h"
#include "gpioControl.h"

#include "acSystem.h"
#include "lightSystem.h"
#include "peripherals.h"

#pragma diag_warning 1 // Disable warning:  #1-D: last line of file ends without a newline

/***************************************************/
/*              INITIALISATION START               */
/***************************************************/
/****************** GLOBAL VARIABLES AND CONSTANTS ******************/
#define RISING_EDGE_TIMER_PERIOD 50 //ms

/****************** GPIO CONFIGS *****************/
// UART
GPIO_Config tx_uart_config = {
    .port = GPIOB,
    .pin = Pin10,
    .mode = GPIO_AlternateFunction,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull, // Not relevant, but we'll define it just incase
    .speed = GPIO_2MHz};
GPIO_Config rx_uart_config = {
    .port = GPIOB,
    .pin = Pin11,
    .mode = GPIO_AlternateFunction,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull, // Not relevant
    .speed = GPIO_2MHz};

// Inputs
GPIO_Config light_switch_config = {
    .port = GPIOA,
    .pin = Pin9,
    .mode = GPIO_Input,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull, // Not relevant
    .speed = GPIO_2MHz};
GPIO_Config light_sensor_config = {
    .port = GPIOA,
    .pin = Pin8,
    .mode = GPIO_Input,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull, // Not relevant
    .speed = GPIO_2MHz};
GPIO_Config fan_switch_config = {
    .port = GPIOA,
    .pin = Pin3,
    .mode = GPIO_Input,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull, // Not relevant
    .speed = GPIO_2MHz};
GPIO_Config thermometer_config = { // TODO: ADC. Config is dummy values
    .port = GPIOF,
    .pin = Pin10,
    .mode = GPIO_AlternateFunction,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull, // Not relevant
    .speed = GPIO_2MHz};

// Outputs
GPIO_Config heater_config = {
    .port = GPIOF,
    .pin = Pin8,
    .mode = GPIO_Output,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull,
    .speed = GPIO_2MHz};
GPIO_Config cooler_config = {
    .port = GPIOB,
    .pin = Pin8,
    .mode = GPIO_Output,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull,
    .speed = GPIO_2MHz};
GPIO_Config fan_config = {
    .port = GPIOA,
    .pin = Pin10,
    .mode = GPIO_Output,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull,
    .speed = GPIO_2MHz};
GPIO_Config light_config = {
    .port = GPIOB,
    .pin = Pin0,
    .mode = GPIO_Output,
    .pullUpDown = GPIO_No_Pull,
    .outputType = GPIO_Output_PushPull,
    .speed = GPIO_2MHz};

/****************** PERIPHERALS ******************/
UARTInterface uart = {
    .last_byte_received = 0,
    .transmit_data = 0,
    .receive_data = 0,
    .is_controlling_ACSystem = false,
    .tx_config = &tx_uart_config,
    .rx_config = &rx_uart_config,
    .uart = USART3};
Timer monitoring_timer = {
    .is_running = true,
    .time_in_ms = 1000,
    .in_one_pulse_mode = false,
    .clock_speed = AHB1_ClkSpeed,
    .timer_component = TIM6};
Timer rising_edge_timer = {
    .is_running = true,
    .time_in_ms = RISING_EDGE_TIMER_PERIOD,
    .in_one_pulse_mode = false,
    .clock_speed = AHB1_ClkSpeed,
    .timer_component = TIM7};
Timer uart_priority_timer = {
    .is_running = false,
    .time_in_ms = 1000,
    .in_one_pulse_mode = true,
    .clock_speed = APB2_ClkSpeed,
    .timer_component = TIM10};

/****************** HOME MANAGEMENT SYSTEM COMPONENTS ******************/
// Unfortunately we don't have constructors for these.
// Leave uninitialised until we've determined member data.
Light light = {
    
    .gpio_config = &light_config};
LightSwitch light_switch = {
    .was_toggled = false,
    .was_pressed = false,
    .is_pressed = false,
    .has_been_held_for = 0,
    .gpio_config = &light_switch_config};
LightSensor light_sensor = {

    .gpio_config = &light_sensor_config};

Heater heater = {

    .gpio_config = &heater_config};
Cooler cooler = {

    .gpio_config = &cooler_config};
Fan fan = {

    .gpio_config = &fan_config};
FanSwitch fan_switch = {
    .was_toggled = false,
    .was_pressed = false,
    .is_pressed = false,
    .has_been_held_for = 0,
    .gpio_config = &fan_switch_config};
Thermometer thermometer = {

    .gpio_config = &thermometer_config};
int main(void)
{
  /****************** ENABLE CLOCKS FOR GPIO ******************/
  // Needs to be changed by hand if any of the peripherals change 
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOFEN);  // GPIO A, B, F
  RCC->APB1ENR |= (RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_USART3EN);   // Timer 6, Timer 7, USART 3
  RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; // Timer 10
  // Reset, then clear the resets
  RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOFRST);
  RCC->APB1RSTR |= (RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST);
  RCC->APB2RSTR |= RCC_APB2RSTR_TIM10RST;
  __asm("nop");
  __asm("nop");
  RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOFRST);
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST);
  RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM10RST;
  __asm("nop");
  __asm("nop");

  /****************** ENABLE AND CONFIGURE GPIO ******************/
  // Bring up the GPIO for the power regulators.
  boardSupport_init();
  // Bring up GPIO for all our HMS inputs/outputs
  gpio_configureGPIO(uart.tx_config);
  gpio_configureGPIO(uart.rx_config);
  gpio_configureGPIO(light.gpio_config);
  gpio_configureGPIO(light_switch.gpio_config);
  gpio_configureGPIO(light_sensor.gpio_config);
  gpio_configureGPIO(heater.gpio_config);
  gpio_configureGPIO(cooler.gpio_config);
  gpio_configureGPIO(fan.gpio_config);
  gpio_configureGPIO(fan_switch.gpio_config);
  gpio_configureGPIO(thermometer.gpio_config);

  /****************** SETUP ******************/
  //Interrupts
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  NVIC_EnableIRQ(USART3_IRQn);
	
	initUART(&uart);
  initTimer(&monitoring_timer);
  initTimer(&rising_edge_timer);
  initTimer(&uart_priority_timer);
  // ADC?
	
	
  /***************************************************/
  /*              INITIALISATION END                 */
  /***************************************************/

  /***************************************************/
  /*                    PROGRAM START                */
  /***************************************************/
  while (1)
  {
    updateLightSwitch(&light_switch);
    if (light_switch.was_toggled) {
      // For now, just toggle the light on and off.
      if (light.is_on) {
        turnOffLight(&light);
      } else {
        turnOnLight(&light);
      }
    }
    // Same as above, except it's a fan.
    updateFanSwitch(&fan_switch);
    if (fan_switch.was_toggled) {
      if (fan.is_on) {
        turnOffFan(&fan);
      } else {
        turnOnFan(&fan);
      }
    }
  }
}

/****************** INTERRUPTS *****************/
/* TIM6_DAC_IRQHandler: TIM6 Global Interrupt
    Occurs every 1s. Handles:
    - monitoring (sending info to the UART)
    - 15s timer for the fan switch
    - 15s timer for the AC system
*/
void TIM6_DAC_IRQHandler(void) {
  // Clear update interrupt
  TIM6->SR &= ~(TIM_SR_UIF);

  // Flush pipeline (allow last instruction to go through, if it was interrupted midway)
  __asm("isb");
}

/* TIM7_IRQHandler: TIM7 Global Interrupt
    Occurs regularly, every (RISING_EDGE_TIMER_PERIOD)ms. Handles:
    - updating how long a switch has been held for, if it is pressed
*/
void TIM7_IRQHandler(void) {
  // Clear update interrupt
  TIM7->SR &= ~(TIM_SR_UIF);

  //TODO: Verify the logic for this. Is 'held for' was_pressed && is_pressed, or just is_pressed?
  if (fan_switch.is_pressed) {
    fan_switch.has_been_held_for += RISING_EDGE_TIMER_PERIOD;
  }
  if (light_switch.is_pressed) {
    light_switch.has_been_held_for += RISING_EDGE_TIMER_PERIOD;
  }

  // Flush pipeline (allow last instruction to go through, if it was interrupted midway)
  __asm("isb");
}

/* TIM1_UP_TIM10_IRQHandler: TIM10 Global Interrupt
    Triggered by UART. Handles:
    - UART commands getting priority over the light switch
    - Disabling timer after 1s
*/
void TIM1_UP_TIM10_IRQHandler(void) {
  // Clear update interrupt
  TIM10->SR &= ~(TIM_SR_UIF);

  // Flush pipeline (allow last instruction to go through, if it was interrupted midway)
  __asm("isb");
}

/* USART3_IRQHandler: UART3 Global Interrupt
    Triggered when UART receives data. Handles:
    - data processing and validation of incoming bytes
    - if a command is received, setting HMS controls
*/
void USART3_IRQHandler(void) {

  // Flush pipeline (allow last instruction to go through, if it was interrupted midway)
    __asm("isb");
}

