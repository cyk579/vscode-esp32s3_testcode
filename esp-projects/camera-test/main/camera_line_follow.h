#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise the TB6612 outputs and leave the vehicle in standby. */
esp_err_t camera_line_follow_start(void);

/* Stop all motors and put the TB6612 into standby. */
void camera_line_follow_stop(void);

/* Called by the JPEG decoder task with one decoded RGB565 frame. */
void camera_line_follow_frame_callback(uint8_t *rgb565_big_endian,
                                       uint16_t width,
                                       uint16_t height,
                                       uint8_t source_threshold,
                                       bool draw_overlay,
                                       void *user_ctx);

/* Draw the latest control result on the independently decoded TFT preview. */
void camera_line_follow_preview_callback(uint8_t *rgb565_big_endian,
                                         uint16_t width,
                                         uint16_t height,
                                         uint8_t source_threshold,
                                         uint32_t sequence,
                                         int64_t capture_us,
                                         void *user_ctx);

typedef struct {
    const char *state;
    bool armed;
    bool stby;
    bool candidate;
    const char *ball_phase;
    int motor_a;
    int motor_b;
    int motor_d;
    int ultrasonic_distance_x10;
    int threshold;
    int seed_x;
    uint8_t valid_rows;
    uint8_t confidence;
} camera_line_follow_debug_snapshot_t;

/* Read the last control values for low-rate diagnostics (no motor changes). */
void camera_line_follow_get_debug_snapshot(camera_line_follow_debug_snapshot_t *snapshot);

/* Called after a preview image is sent, so the status text is not overwritten. */
void camera_line_follow_tft_status_callback(void *user_ctx);

#ifdef __cplusplus
}
#endif
