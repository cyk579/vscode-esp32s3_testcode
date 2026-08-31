#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*camera_display_frame_callback_t)(const uint8_t *rgb565_big_endian,
                                                 uint16_t width,
                                                 uint16_t height,
                                                 void *user_ctx);

/** Initialise the asynchronous JPEG decoder and the optional ST7735 preview. */
esp_err_t camera_display_start(void);

/**
 * Register a callback for decoded frames.  Set this before starting the
 * decoder.  The callback runs in the decoder task and must not block.
 */
void camera_display_set_frame_callback(camera_display_frame_callback_t callback,
                                       void *user_ctx);

/**
 * Give the preview task one complete MJPEG frame.
 *
 * The function is non-blocking and intentionally drops stale frames.  The
 * caller retains ownership of @p jpeg as soon as this function returns.
 */
bool camera_display_submit(const uint8_t *jpeg, size_t jpeg_len);

#ifdef __cplusplus
}
#endif
