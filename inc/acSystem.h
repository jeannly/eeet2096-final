#ifndef ACSYSTEM_H
#define ACSYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include "gpioControl.h"


typedef struct Heater {
    bool is_on;
    GPIO_Config* gpio_config;
} Heater;

typedef struct Cooler {
    bool is_on;
    GPIO_Config* gpio_config;
} Cooler;

typedef struct Fan {
    bool is_on;
    GPIO_Config* gpio_config;
} Fan;

typedef struct FanSwitch {
    bool was_toggled; //if was_pressed and has_been_held_for >= 500ms, then FanSwitch was_toggled.
    bool was_pressed;
    bool is_pressed;
    uint16_t has_been_held_for;
    GPIO_Config* gpio_config;
} FanSwitch;

typedef struct Thermometer {
    uint16_t adc_val;
    uint16_t celcius;
    GPIO_Config* gpio_config;
} Thermometer;

void updateFanSwitch(FanSwitch*);
void turnOnFan(Fan*);
void turnOffFan(Fan*);

#endif //ACSYSTEM_H

