#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialise the ST7735 preview and its asynchronous JPEG decoder task. */
esp_err_t camera_display_start(void);

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
