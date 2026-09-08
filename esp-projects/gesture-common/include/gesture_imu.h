#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef struct {
    int sda, scl;
    int axis[3]; /* body X forward, Y left, Z up; permutation of sensor 0,1,2 */
    int sign[3];
} gesture_imu_config_t;
typedef struct {
    float pitch, roll; /* canonical: forward tilt positive, left tilt positive */
    float body_roll, body_pitch;
    bool valid;
    uint32_t sampled_ms;
} gesture_imu_sample_t;
esp_err_t gesture_imu_start(const gesture_imu_config_t *config);
/* Thread-safe copy. False while calibrating, disconnected or data is stale. */
bool gesture_imu_latest(gesture_imu_sample_t *sample);
