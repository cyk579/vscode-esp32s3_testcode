#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void ultrasonic_init(gpio_num_t trig_gpio, gpio_num_t echo_gpio);
float ultrasonic_read_cm(void);

#ifdef __cplusplus
}
#endif
