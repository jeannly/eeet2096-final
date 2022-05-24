#include "lightSystem.h"

#include <stdbool.h>
#include <stdint.h>
#include "gpioControl.h"

/* Updates light switch status.
First, reads the switch's current status (pressed or not pressed).
Then, uses its variables to determine if it's been toggled on the rising edge.
*/
void updateLightSwitch(LightSwitch* light_switch) {
// 1. Get current switch status
    uint8_t gpio_val;
    gpio_val = gpio_getPinValue(light_switch->gpio_config->port, light_switch->gpio_config->pin);
    if (gpio_val == 0) { //  if switch is ON (because active low)
        light_switch->is_pressed = true;
    } else {
        light_switch->is_pressed = false;
    }

// 2. Have we detected a rising edge?
    if (light_switch->was_pressed && !light_switch->is_pressed 
        && light_switch->has_been_held_for >= 500) {
        light_switch->was_toggled = true;
        light_switch->has_been_held_for = 0;
    } else {
        light_switch->was_toggled = false;
    }
// 3. Update the last state
    light_switch->was_pressed = light_switch->is_pressed;
    return;
}

void turnOnLight(Light* light) {
    // LEDs are active LOW, so clear the light's bit
    gpio_resetGPIO(light->gpio_config->port, light->gpio_config->pin);
    light->is_on = true;
    return;
}
void turnOffLight(Light* light) {
    // LEDs are active LOW, so set the light's bit
    gpio_setGPIO(light->gpio_config->port, light->gpio_config->pin);
    light->is_on = false;
    return;
}