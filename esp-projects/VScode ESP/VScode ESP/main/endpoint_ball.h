#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef void (*endpoint_ball_drive_fn_t)(float a, float b, float d);
typedef void (*endpoint_ball_stop_fn_t)(void);
typedef void (*endpoint_ball_control_fn_t)(void);

typedef struct {
    bool route_completed;
    bool paused;
    bool active;
    const char *phase;
    bool target_valid;
    bool target_is_red;
    int target_x;
    int target_y;
    int target_width;
    int target_height;
    int target_frame_width;
    int target_frame_height;
    int motor_a;
    int motor_b;
    int motor_d;
} endpoint_ball_status_t;

esp_err_t endpoint_ball_init(endpoint_ball_drive_fn_t drive,
                             endpoint_ball_stop_fn_t stop,
                             endpoint_ball_control_fn_t pause);

void endpoint_ball_process_frame(const uint8_t *frame,
                                 uint16_t width,
                                 uint16_t height);

void endpoint_ball_start(void);

bool endpoint_ball_active(void);
void endpoint_ball_get_status(endpoint_ball_status_t *status);
