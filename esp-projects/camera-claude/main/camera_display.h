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

/* Called at the TFT refresh rate with the decoded control RGB565 frame.
 * The callback may draw into the frame; whatever it leaves there is what the
 * panel shows. */
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

/** Initialise the asynchronous JPEG decoder and optional direct TFT preview. */
esp_err_t camera_display_start(void);

/**
 * Register a callback for decoded frames. Set this before starting the
 * decoder. The callback receives the writable decoded RGB565 frame without
 * any display-only conversion. source_threshold is the adaptive grayscale
 * threshold, or zero when the source frame did not have enough contrast.
 * draw_overlay is true when the low-frequency TFT preview is enabled, allowing
 * the callback to update a small overlay snapshot on the writable frame.
 * The callback runs in the control task and must not block for long.
 */
void camera_display_set_frame_callback(camera_display_frame_callback_t callback,
                                       void *user_ctx);

/* Register the low-frequency preview callback. It receives the decoded
 * control frame while that buffer is still owned by the control task. */
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

/*
 * 最近一帧 ROI 亮度直方图的两个原始百分位：dark = 第 2、light = 第 90。
 * 阈值就是 dark + (light-dark)*45%，所以校准时看这两个数比看阈值有用：
 *   dark 高（>80）   胶带没进 ROI，或者胶带反光/曝光过度
 *   light 低（<120） 整体太暗，白板没到该有的亮度
 *   两者差 <32       对比度守卫会触发，阈值返回 0
 */
void camera_display_get_levels(uint8_t *dark, uint8_t *light);

/* Read the split control/preview pipeline counters and timings. */
void camera_display_get_pipeline_stats(camera_display_pipeline_stats_t *stats);

#ifdef __cplusplus
}
#endif
