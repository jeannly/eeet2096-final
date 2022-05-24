#ifndef HOMEMANAGEMENT_H
#define HOMEMANAGEMENT_H

#include <stdint.h>
#include <stdbool.h>
#include "gpioControl.h"

/* Component is the generic term for any HMS output. Lights, fans, etc. */
typedef struct Component {
    bool is_on;
    GPIO_Config* gpio_config;
} Component;

typedef struct LightSwitch {
    bool is_disabled;
    bool was_toggled; //if was_pressed and has_been_held_for > 500ms, then LightSwitch was_toggled.
    bool was_pressed;
    bool is_pressed;
    volatile uint16_t has_been_held_for;
    GPIO_Config* gpio_config;
} LightSwitch;

typedef struct LightSensor {
    bool is_active;
    GPIO_Config* gpio_config;
} LightSensor;

typedef struct FanSwitch {
    bool was_toggled; //if was_pressed and has_been_held_for >= 500ms, then FanSwitch was_toggled.
    bool was_pressed;
    bool is_pressed;
    bool override_active;
    volatile uint16_t override_time; // in milliseconds
    volatile uint16_t has_been_held_for;
    GPIO_Config* gpio_config;
} FanSwitch;

typedef struct Thermometer {
    volatile uint16_t adc_val;
    volatile float celcius;
    GPIO_Config* gpio_config;
} Thermometer;

void updateFanSwitch(FanSwitch*);
void setOverride(FanSwitch*);
void updateThermometer(Thermometer*);
void updateLightSwitch(LightSwitch*);
void updateLightSensor(LightSensor*);
void toggle(Component*);
void turnOn(Component*);
void turnOff(Component*);

#endif //HOMEMANAGEMENT_H