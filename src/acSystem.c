#include "acSystem.h"

#include <stdbool.h>
#include <stdint.h>
#include "gpioControl.h"

/* Updates fan switch status.
First, reads the switch's current status (pressed or not pressed).
Then, uses its variables to determine if it's been toggled on the rising edge.
*/
void updateFanSwitch(FanSwitch* fan_switch) {
// 1. Get current switch status
    uint8_t gpio_val;
    gpio_val = gpio_getPinValue(fan_switch->gpio_config->port, fan_switch->gpio_config->pin);
    if (gpio_val == 0) { //  if switch is ON (because active low)
        fan_switch->is_pressed = true;
    } else {
        fan_switch->is_pressed = false;
    }

// 2. Have we detected a rising edge?
    if (fan_switch->was_pressed && !fan_switch->is_pressed 
        && fan_switch->has_been_held_for >= 500) {
        fan_switch->was_toggled = true;
        fan_switch->has_been_held_for = 0;
    } else {
        fan_switch->was_toggled = false;
    }
// 3. Update the last state
    fan_switch->was_pressed = fan_switch->is_pressed;
    return;
}

void turnOnFan(Fan* fan) {
    // LEDs are active LOW, so clear the fan's bit
    gpio_resetGPIO(fan->gpio_config->port, fan->gpio_config->pin);
    fan->is_on = true;
    return;
}
void turnOffFan(Fan* fan) {
    // LEDs are active LOW, so set the fan's bit
    gpio_setGPIO(fan->gpio_config->port, fan->gpio_config->pin);
    fan->is_on = false;
    return;
}


