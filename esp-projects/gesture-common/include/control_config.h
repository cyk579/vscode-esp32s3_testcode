#pragma once
#define CONTROL_TIMEOUT_MS 250u
#define CONTROL_NEUTRAL_MS 300u
#define CONTROL_REVERSE_MS 20u
#define CONTROL_DEADZONE_DEG 5.0f
#define CONTROL_FULL_TILT_DEG 25.0f
#define CONTROL_MAX_TILT_DEG 60.0f
#define CONTROL_MAX_PWM 30.0f
#define CONTROL_SLEW_PWM_PER_SECOND 60.0f
/* Initial estimates from existing car documentation; measure again with payload. */
#define CONTROL_FLOOR_AD 11.0f
#define CONTROL_FLOOR_B 13.0f
#define IMU_STALE_MS 100u
#define CAR_TILT_LIMIT_DEG 30.0f
#define CAR_TILT_HOLD_MS 200u
