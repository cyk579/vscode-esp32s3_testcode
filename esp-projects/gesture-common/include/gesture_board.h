#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#define PIN_UNASSIGNED (-1)
typedef struct { const char *name; int gpio; bool output; } gesture_pin_t;
/* No GPIO is touched by validation. UART0 remains reserved for the console. */
esp_err_t gesture_board_validate(const gesture_pin_t *pins, size_t count, bool allow_strapping);
