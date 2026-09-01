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

#ifdef __cplusplus
}
#endif
