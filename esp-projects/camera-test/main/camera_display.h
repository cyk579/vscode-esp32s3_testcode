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

typedef enum {
    CAMERA_DISPLAY_STATUS_NORMAL = 0,
    CAMERA_DISPLAY_STATUS_CORNER,
    CAMERA_DISPLAY_STATUS_LOST,
    CAMERA_DISPLAY_STATUS_ALIGN,
} camera_display_status_state_t;

typedef struct {
    camera_display_status_state_t state;
    bool armed;
    bool stby;
    int motor_a;
    int motor_b;
    int motor_d;
    int lateral_error;
    int heading_error;
    int turn_command;
    int ultrasonic_cm;
} camera_display_status_t;

typedef bool (*camera_display_status_callback_t)(camera_display_status_t *status,
                                                 void *user_ctx);

/* Called by the low-priority preview task with an independent RGB565 frame. */
typedef void (*camera_display_preview_callback_t)(uint8_t *rgb565_big_endian,
                                                   uint16_t width,
                                                   uint16_t height,
                                                   uint8_t source_threshold,
                                                   uint32_t sequence,
                                                   int64_t capture_us,
                                                   void *user_ctx);

typedef struct {
    uint32_t camera_frames;
    uint32_t processed_frames;
    uint32_t control_frames;
    uint32_t preview_frames;
    uint32_t frames_dropped;
    uint32_t control_dropped_frames;
    uint32_t preview_dropped_frames;
    uint32_t control_decode_us;
    uint32_t preview_decode_us;
    uint32_t threshold_us;
    uint32_t tft_us;
    uint32_t last_control_age_us;
    uint32_t last_control_sequence;
} camera_display_pipeline_stats_t;

/** Initialise the asynchronous JPEG decoder and the optional TFT status page. */
esp_err_t camera_display_start(void);

/**
 * Register a callback for decoded frames. Set this before starting the
 * decoder. The callback receives the writable decoded RGB565 frame without
 * any display-only conversion. source_threshold is the adaptive grayscale
 * threshold, or zero when the source frame did not have enough contrast.
 * draw_overlay is retained for source compatibility and is always false for
 * the control-only callback. The callback runs in the control task and must
 * not block for long.
 */
void camera_display_set_frame_callback(camera_display_frame_callback_t callback,
                                       void *user_ctx);

void camera_display_set_status_callback(camera_display_status_callback_t callback,
                                         void *user_ctx);

/* Kept for source compatibility; the image-preview callback is no longer run. */
void camera_display_set_preview_callback(camera_display_preview_callback_t callback,
                                         void *user_ctx);

/** Give the control decoder one complete MJPEG frame. */
bool camera_display_submit(const uint8_t *jpeg, size_t jpeg_len);

/* Read cumulative UVC, decoded, and dropped-frame counters. */
void camera_display_get_counters(uint32_t *camera_frames,
                                 uint32_t *processed_frames,
                                 uint32_t *dropped_frames);

/* 最近一次解码、阈值计算和 TFT 传输耗时（微秒），用于低频诊断。 */
void camera_display_get_timing(uint32_t *decode_us,
                               uint32_t *threshold_us,
                               uint32_t *tft_us);

/* Read the split control/preview pipeline counters and timings. */
void camera_display_get_pipeline_stats(camera_display_pipeline_stats_t *stats);

#ifdef __cplusplus
}
#endif
