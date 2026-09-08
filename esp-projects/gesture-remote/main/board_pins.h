#pragma once
#include "gesture_board.h"
#define PIN_IMU_SDA PIN_UNASSIGNED
#define PIN_IMU_SCL PIN_UNASSIGNED
#define PIN_ENABLE_BUTTON PIN_UNASSIGNED
/* Normally-open button to GND. Internal pull-up; released or broken wire = disabled. */
#define ALLOW_STRAPPING_PINS 0
#define IMU_AXIS_X 0
#define IMU_AXIS_Y 1
#define IMU_AXIS_Z 2
#define IMU_SIGN_X 1
#define IMU_SIGN_Y 1
#define IMU_SIGN_Z 1
#define REMOTE_PITCH_SIGN 1
#define REMOTE_ROLL_SIGN 1
