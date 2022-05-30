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

#include "homeManagement.h"
#include "peripherals.h"

#pragma diag_warning 1 // Disable warning:  #1-D: last line of file ends without a newline

/***************************************************/
/*              INITIALISATION START               */
/***************************************************/
/****************** GLOBAL VARIABLES AND CONSTANTS ******************/
#define COUNTER_PERIOD 50 //ms

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
    .mode = GPIO_Analog,
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
    .receive_data_ptr = 0,
    .received_char = 0,
    .receive_data = {0, 0, 0},
    .command_received = false,
    .is_controlling_HMS = false,
    .in_monitoring_mode = true,
    .tx_config = &tx_uart_config,
    .rx_config = &rx_uart_config,
    .uart = USART3};
Timer monitoring_timer = {
    .is_running = true,
    .time_in_ms = 1000,
    .in_one_pulse_mode = false,
    .clock_speed = APB1_ClkSpeed*2, // The timers operate at 2x bus speed for some reason...
    .timer_component = TIM6};
Timer counter_timer = {
    .is_running = true,
    .time_in_ms = COUNTER_PERIOD,
    .in_one_pulse_mode = false,
    .clock_speed = APB1_ClkSpeed*2,
    .timer_component = TIM7};
Timer uart_priority_timer = {
    .is_running = false,
    .time_in_ms = 15000,
    .in_one_pulse_mode = true,
    .clock_speed = APB2_ClkSpeed*2,
    .timer_component = TIM10};

/****************** HOME MANAGEMENT SYSTEM COMPONENTS ******************/
// Unfortunately we don't have constructors for these, so here we go.
Component light = {
    .is_on = false,
    .gpio_config = &light_config};
LightSwitch light_switch = {
    .is_disabled = false,
    .was_toggled = false, // was_toggled MUST BE HANDLED BY MAIN.C: i.e. set it to false after detecting it
    .was_pressed = false,
    .is_pressed = false,
    .has_been_held_for = 0,
    .gpio_config = &light_switch_config};
LightSensor light_sensor = {
    .is_active = false,
    .gpio_config = &light_sensor_config};

Component heater = {
    .is_on = false,
    .gpio_config = &heater_config};
Component cooler = {
    .is_on = false,
    .gpio_config = &cooler_config};
Component fan = {
    .is_on = false,
    .gpio_config = &fan_config};
FanSwitch fan_switch = {
    .was_toggled = false, // was_toggled MUST BE HANDLED BY MAIN.C: i.e. set it to false after detecting it
    .was_pressed = false,
    .is_pressed = false,
    .override_active = false,
    .override_time = 0,
    .has_been_held_for = 0,
    .gpio_config = &fan_switch_config};
Thermometer thermometer = {
    .adc_val = 0,
    .celcius = 0.0,
    .gpio_config = &thermometer_config};

int main(void)
{
  /****************** ENABLE CLOCKS FOR GPIO ******************/
  // Needs to be changed by hand if any of the peripherals change 
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOFEN);  // GPIO A, B, F
  RCC->APB1ENR |= (RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_USART3EN);   // Timer 6, Timer 7, USART 3
  RCC->APB2ENR |= (RCC_APB2ENR_ADC3EN | RCC_APB2ENR_TIM10EN); // Timer 10
  // Reset, then clear the resets
  RCC->AHB1RSTR |= (RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOFRST);
  RCC->APB1RSTR |= (RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST);
  RCC->APB2RSTR |= (RCC_APB2RSTR_ADCRST | RCC_APB2RSTR_TIM10RST);
  __asm("nop");
  __asm("nop");
  RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | RCC_AHB1RSTR_GPIOFRST);
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_TIM7RST | RCC_APB1RSTR_USART3RST);
  RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST | RCC_APB2RSTR_TIM10RST);
  __asm("nop");
  __asm("nop");

  /****************** ENABLE AND CONFIGURE GPIO ******************/
  // Bring up the GPIO for the power regulators.
  boardSupport_init();
  // Bring up GPIO for all our HMS inputs/outputs
  gpio_configureGPIO(light.gpio_config);
  gpio_configureGPIO(light_switch.gpio_config);
  gpio_configureGPIO(light_sensor.gpio_config);
  gpio_configureGPIO(heater.gpio_config);
  gpio_configureGPIO(cooler.gpio_config);
  gpio_configureGPIO(fan.gpio_config);
  gpio_configureGPIO(fan_switch.gpio_config);
  gpio_configureGPIO(thermometer.gpio_config);
  gpio_configureGPIO(uart.tx_config);
  gpio_configureGPIO(uart.rx_config);
  //Need to set UART3 alternate function configurations manually, because gpioControl doesn't handle it
  GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL11_Msk);
  GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL10_Pos) | (0x07 << GPIO_AFRH_AFSEL11_Pos);

  /****************** SETUP ******************/
  //Interrupts
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  NVIC_EnableIRQ(USART3_IRQn);
	
	initUART(&uart);
  initTimer(&monitoring_timer);
  initTimer(&counter_timer);
  initTimer(&uart_priority_timer);
  initADC3();
	
  /***************************************************/
  /*              INITIALISATION END                 */
  /***************************************************/

  /***************************************************/
  /*                    PROGRAM START                */
  /***************************************************/
  while (1)
  {
    if (uart.command_received) {
      /* Don't let any commands be sent while we process the previous command */
      NVIC_DisableIRQ(USART3_IRQn);
      processCommand(&uart, &light, &fan, &heater, &cooler);
      if (uart.is_controlling_HMS) {
        if (thermometer.celcius < 18 || thermometer.celcius > 26) {
          UARTPrintLn("Command failed: temperature is out of range");
          uart.is_controlling_HMS = false;
        } else {
          uart_priority_timer.is_running = true;
          initTimer(&uart_priority_timer);
        }
      }
      NVIC_EnableIRQ(USART3_IRQn);
    }
    /***************************************************/
    /*                    AC SYSTEM                    */
    /***************************************************/
    updateFanSwitch(&fan_switch);
    updateThermometer(&thermometer);
    if (!uart.is_controlling_HMS) {
      if (thermometer.celcius < 22) {
        turnOn(&heater);
        if (!fan_switch.override_active) { turnOn(&fan); }
        turnOff(&cooler);
      } else if (thermometer.celcius > 24) {
        turnOn(&cooler);
        if (!fan_switch.override_active) { turnOn(&fan); }
        turnOff(&heater);
      } else { // Perfect temperature, do nothing
        turnOff(&heater);
        if (!fan_switch.override_active) { turnOff(&fan); }
        turnOff(&cooler);
      }

      /* When the fan switch is toggled:
        if the AC is being controlled by the UART, do nothing
        if the AC is on, we want to override the AC system logic by turning the fan OFF for 15s 
        if the AC is off, we want to just toggle the fan (and keep that state for 15s)
      */
      if (fan_switch.was_toggled) {
        if (thermometer.celcius < 22 || thermometer.celcius > 24) {
          turnOff(&fan);
        } else {
          toggle(&fan);
        }
        activateOverride(&fan_switch); // Start (or reset) the override timer (controlled by TIM6)
      }
    }
    // UART still has priority over the fan switch, so this will have no effect if UART is controlling,
    //but deactivate the override logic regardless.
    if (fan_switch.override_active && fan_switch.override_time >= 15000) {
        deactivateOverride(&fan_switch);
    }

  /***************************************************/
  /*                    LIGHT SYSTEM                 */
  /***************************************************/
    updateLightSensor(&light_sensor);
    updateLightSwitch(&light_switch);
    if (!uart.is_controlling_HMS) {
      if (light_sensor.is_active) {
        light_switch.is_disabled = true;
      } else {
        light_switch.is_disabled = false;
      }
      if (light_switch.was_toggled) {
        toggle(&light);
      }
    }

  }
}

/****************** INTERRUPTS *****************/
/* TIM6_DAC_IRQHandler: TIM6 Global Interrupt
    Occurs every 1s. Handles:
    - monitoring (sending info to the UART)
*/
void TIM6_DAC_IRQHandler(void) {
  // Clear update interrupt
  TIM6->SR &= ~(TIM_SR_UIF);
  if (uart.in_monitoring_mode) {
    sendHMSStatus(&thermometer, &light, &fan, &heater, &cooler);
  }
  // Flush pipeline (allow last instruction to go through, if it was interrupted midway)
  __asm("isb");
}

/* TIM7_IRQHandler: TIM7 Global Interrupt
    Occurs regularly, every (COUNTER_PERIOD)ms. Handles:
    - updating how long a switch has been held for, if it is pressed
    - 15s for the fan switch override
*/
void TIM7_IRQHandler(void) {
  // Clear update interrupt
  TIM7->SR &= ~(TIM_SR_UIF);

  if (fan_switch.is_pressed) {
    fan_switch.has_been_held_for += COUNTER_PERIOD;
  }
  if (light_switch.is_pressed) {
    light_switch.has_been_held_for += COUNTER_PERIOD;
  }
  if (fan_switch.override_active) {
    fan_switch.override_time += COUNTER_PERIOD;
  }
  // Flush pipeline (allow last instruction to go through, if it was interrupted midway)
  __asm("isb");
}

/* TIM1_UP_TIM10_IRQHandler: TIM10 Global Interrupt
    Triggered by UART. Handles:
    - Letting UART command being in force for 15s
*/
void TIM1_UP_TIM10_IRQHandler(void) {
  // Clear update interrupt
  TIM10->SR &= ~(TIM_SR_UIF);
  uart.is_controlling_HMS = false;
  uart_priority_timer.is_running = false;
  initTimer(&uart_priority_timer);
  // Flush pipeline (allow last instruction to go through, if it was interrupted midway)
  __asm("isb");
}

/* USART3_IRQHandler: UART3 Global Interrupt
    Triggered when UART receives data. Handles:
    -
*/
void USART3_IRQHandler(void) {
  // Reading the status register, then the data registers, causes the ORE interrupt to be cleared
  // Reading the data register causes the RXNE interrupt to be cleared
  uart.received_char = USART3->SR; // we don't need the SR data, this is just to clear the interrupt
  uart.received_char = USART3->DR;
  
  receiveChar(&uart);
  // Flush pipeline (allow last instruction to go through, if it was interrupted midway)
    __asm("isb");
}

