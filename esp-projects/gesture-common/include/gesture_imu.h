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
typedef enum { GESTURE_IMU_OK=0, GESTURE_IMU_I2C_INIT=1, GESTURE_IMU_NOT_FOUND=2, GESTURE_IMU_WHO_AM_I=3, GESTURE_IMU_READ=4, GESTURE_IMU_CALIBRATING=5, GESTURE_IMU_FUSION=6 } gesture_imu_health_t;
esp_err_t gesture_imu_start(const gesture_imu_config_t *config);
gesture_imu_health_t gesture_imu_health(void);
/* Thread-safe copy. False while calibrating, disconnected or data is stale. */
bool gesture_imu_latest(gesture_imu_sample_t *sample);
