#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float distance_cm;
    uint8_t ir_mask;
    int error;
    int turn;
    int motor_a;
    int motor_b;
    int motor_d;
    const char *mode;
} tft_status_t;

bool tft_st7735_init(void);
void tft_st7735_show(const tft_status_t *status);
