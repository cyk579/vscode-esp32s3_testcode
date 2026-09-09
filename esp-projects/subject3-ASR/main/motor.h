#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t motor_init(void);
/* A,B,D signed PWM percentage before physical polarity. */
esp_err_t motor_apply(const float pwm[3], bool enabled);
void motor_stop(void);
