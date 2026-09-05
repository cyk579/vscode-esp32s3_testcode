#ifndef _CAMERA_TRACK_H_
#define _CAMERA_TRACK_H_

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// 🌟 新增：视觉模式枚举
typedef enum {
    VISION_MODE_LINE = 0,    // 默认：循迹模式
    VISION_MODE_GREEN_BALL,  // 找球：识别绿色小球
    VISION_MODE_RED_BALL     // 找球：识别红色小球
} vision_mode_t;

typedef struct {
    uint8_t *data;
    size_t len;
} frame_item_t;

// 基础初始化与控制
void camera_track_init(void);
void camera_track_start(void);
void camera_set_pan_tilt(float yaw_angle, float pitch_angle);
QueueHandle_t camera_track_get_frame_queue(void);

// 循迹专用接口
float get_camera_track_error(void);
int camera_track_is_line_lost(void);

// 🌟 新增：找球专用接口
void camera_track_set_mode(vision_mode_t mode); // 切换视觉模式
float get_ball_track_error(void);               // 获取小球的水平偏差 (-3.0 左 ~ 3.0 右)
int is_ball_detected(void);                     // 判断视野中是否看到了小球 (1=看到, 0=没看到)

#endif