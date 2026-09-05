#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化电机、超声波，创建看门狗和测距任务。电机保持停止，直到连续
 * 若干帧看到黑线才上电（arm），方便摆车。 */
esp_err_t line_follow_start(void);
void line_follow_stop(void);

/* 注册给 camera_display 的回调。 */
void line_follow_frame_callback(uint8_t *rgb565_big_endian,
                                uint16_t width, uint16_t height,
                                uint8_t threshold, bool draw_overlay,
                                void *user_ctx);
void line_follow_preview_callback(uint8_t *rgb565_big_endian,
                                  uint16_t width, uint16_t height,
                                  uint8_t threshold, uint32_t sequence,
                                  int64_t capture_us, void *user_ctx);

#ifdef __cplusplus
}
#endif
