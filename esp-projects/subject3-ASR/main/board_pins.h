#pragma once
#include "gesture_board.h"
/* Matches 引脚对应表-创新实验.xlsx. GPIO17/3 are released from encoder
 * inputs and are routed to the MPU6500 I2C bus. GPIO1/2 remain servos. */
#define PIN_PWMA 9
#define PIN_AIN1 12
#define PIN_AIN2 10
#define PIN_PWMB 4
#define PIN_BIN1 6
#define PIN_BIN2 5
#define PIN_PWMD 16
#define PIN_DIN1 7
#define PIN_DIN2 15
#define PIN_MOTOR_STBY 8
#define PIN_IMU_SDA 17
#define PIN_IMU_SCL 3
#define PIN_TFT_SCLK PIN_UNASSIGNED
#define PIN_TFT_MOSI PIN_UNASSIGNED
#define PIN_TFT_DC PIN_UNASSIGNED
#define PIN_TFT_CS PIN_UNASSIGNED
#define PIN_TFT_RST PIN_UNASSIGNED
/* The single-S3 design keeps the MPU6500 on the vehicle. */
#ifndef ENABLE_CAR_IMU
#define ENABLE_CAR_IMU 1
#endif
#ifndef ENABLE_TFT
#define ENABLE_TFT 0
#endif
#define ALLOW_STRAPPING_PINS 1 /* GPIO3 is used for MPU6500 SCL; verify reset wiring. */
/* Measured in existing car project; verify with wheels lifted before floor testing. */
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN (-1)
#define IMU_AXIS_X 0
#define IMU_AXIS_Y 1
#define IMU_AXIS_Z 2
#define IMU_SIGN_X 1
#define IMU_SIGN_Y 1
#define IMU_SIGN_Z 1
