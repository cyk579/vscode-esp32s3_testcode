#pragma once

#include <stdint.h>
#include "driver/gpio.h"

void ultrasonic_init(gpio_num_t trig_gpio, gpio_num_t echo_gpio);
float ultrasonic_read_cm(void);
