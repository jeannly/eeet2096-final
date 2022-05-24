#include "homeManagement.h"

#include <stdbool.h>
#include <stdint.h>
#include "gpioControl.h"

/* 
Updates light switch status, based on rising edge logic.
First, reads the switch's current status (pressed or not pressed).
Then, uses its variables to determine if it's been toggled on the rising edge.
*/
void updateLightSwitch(LightSwitch* light_switch) {
// Reset the toggle logic
    if (light_switch->was_toggled) {
        light_switch->was_toggled = false;
    }
//  Get current switch status
    if (light_switch->is_disabled) {
        return;
    }
    uint8_t gpio_val;
    gpio_val = gpio_getPinValue(light_switch->gpio_config->port, light_switch->gpio_config->pin);
    if (gpio_val == 0) { //  if switch is ON (because active low)
        light_switch->is_pressed = true;
    } else {
        light_switch->is_pressed = false;
    }

// Button lifted? 
    if (light_switch->was_pressed && !light_switch->is_pressed) {
        // Is our button press valid?
        if (light_switch->has_been_held_for >= 500) {
            light_switch->was_toggled = true;
        }
        light_switch->has_been_held_for = 0;
    }
// Save the reading as the last state
    light_switch->was_pressed = light_switch->is_pressed;
    return;
}



/* 
Read the light sensor, update its state.
*/
void updateLightSensor(LightSensor* sensor) {
    uint8_t gpio_val;
    gpio_val = gpio_getPinValue(sensor->gpio_config->port, sensor->gpio_config->pin);
    if (gpio_val == 0) { //  if switch is ON (because active low)
        sensor->is_active = true;
    } else {
        sensor->is_active = false;
    }
    return;
}

/* 
Updates fan switch status, based on rising edge logic.
*/
void updateFanSwitch(FanSwitch* fan_switch) {
// If the switch has been pressed and the override is active, 
//    disable the override it once it's been active for 15 seconds.
    if (fan_switch->override_active) {
        if (fan_switch->override_time >= 15000) {
            fan_switch->override_active = false;
            fan_switch->override_time = 0;
        }
    }
// Reset the toggle logic
    if (fan_switch->was_toggled) {
        fan_switch->was_toggled = false;
    }

// Read current switch status
    uint8_t gpio_val;
    gpio_val = gpio_getPinValue(fan_switch->gpio_config->port, fan_switch->gpio_config->pin);
    if (gpio_val == 0) { //  if switch is ON (because active low)
        fan_switch->is_pressed = true;
    } else {
        fan_switch->is_pressed = false;
    }

// Button lifted? 
    if (fan_switch->was_pressed && !fan_switch->is_pressed) {
        // Is our button press valid?
        if (fan_switch->has_been_held_for >= 500) {
            fan_switch->was_toggled = true;
        }
        fan_switch->has_been_held_for = 0;
    }
// Save the reading as the last state
    fan_switch->was_pressed = fan_switch->is_pressed;
    return;
}

void setOverride(FanSwitch* fan_switch) {
    fan_switch->override_active = true;
    fan_switch->override_time = 0;
    return;
}

/* Updates thermometer readings from ADC3.
    Reads from ADC3, and stores in in both adc and celcius.
*/
void updateThermometer(Thermometer* thermometer) {
    ADC3->CR2 |= ADC_CR2_SWSTART;
	while((ADC3->SR & ADC_SR_EOC) == 0x00); //check if regular channel is complete
	thermometer->adc_val = (ADC3->DR & 0x0000FFFF);
    // Convert ADC into celcius.
    //  When the ADC is 0, temperature = 8C. When ADC is 4095, temperature = 45C
	thermometer->celcius = ((((float)thermometer->adc_val / 4095) * 37) + 8);
    return;
}
void toggle(Component* component) {
    if (component->is_on) {
        turnOff(component);
        component->is_on = false;
    } else {
        turnOn(component);
        component->is_on = true;
    }
}
void turnOn(Component* component) {
    // LEDs are active LOW, so clear the component's bit
    gpio_resetGPIO(component->gpio_config->port, component->gpio_config->pin);
    component->is_on = true;
    return;
}
void turnOff(Component* component) {
    // LEDs are active LOW, so set the fan's bit
    gpio_setGPIO(component->gpio_config->port, component->gpio_config->pin);
    component->is_on = false;
    return;
}