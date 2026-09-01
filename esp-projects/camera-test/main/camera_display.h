#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*camera_display_frame_callback_t)(uint8_t *rgb565_big_endian,
                                                uint16_t width,
                                                uint16_t height,
                                                uint8_t source_threshold,
                                                bool draw_overlay,
                                                void *user_ctx);

/** Initialise the asynchronous JPEG decoder and the optional ST7735 preview. */
esp_err_t camera_display_start(void);

/**
 * Register a callback for decoded frames. Set this before starting the
 * decoder. The callback receives the writable decoded RGB565 frame without
 * any display-only conversion. source_threshold is the adaptive grayscale
 * threshold, or zero when the source frame did not have enough contrast.
 * draw_overlay is true only for a frame that will be shown on the throttled
 * TFT preview. The callback runs in the decoder task and must not block.
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

/* Read cumulative UVC, decoded, and dropped-frame counters. */
void camera_display_get_counters(uint32_t *camera_frames,
                                 uint32_t *processed_frames,
                                 uint32_t *dropped_frames);

#ifdef __cplusplus
}
#endif
