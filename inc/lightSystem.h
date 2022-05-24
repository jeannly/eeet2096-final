#ifndef LIGHTSYSTEM_H
#define LIGHTSYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include "gpioControl.h"

typedef struct Light {
    bool is_on;
    GPIO_Config* gpio_config;
} Light;

typedef struct LightSwitch {
    bool was_toggled; //if was_pressed and has_been_held_for > 500ms, then LightSwitch was_toggled.
    bool was_pressed;
    bool is_pressed;
    uint16_t has_been_held_for;
    GPIO_Config* gpio_config;
} LightSwitch;

typedef struct LightSensor {
    bool is_active;
    GPIO_Config* gpio_config;
} LightSensor;

void updateLightSwitch(LightSwitch*);
void turnOnLight(Light*);
void turnOffLight(Light*);
#endif