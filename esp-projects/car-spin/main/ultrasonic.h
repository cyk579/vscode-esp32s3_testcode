#pragma once

#include <stdint.h>
#include "driver/gpio.h"

typedef enum {
    ULTRASONIC_OK,
    ULTRASONIC_NO_ECHO,
    ULTRASONIC_ECHO_STUCK_HIGH,
    ULTRASONIC_OUT_OF_RANGE,
} ultrasonic_status_t;

void ultrasonic_init(gpio_num_t trig_gpio, gpio_num_t echo_gpio);
float ultrasonic_read_cm(ultrasonic_status_t *status);
const char *ultrasonic_status_name(ultrasonic_status_t status);
