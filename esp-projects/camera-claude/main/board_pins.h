#ifndef _BOARD_PINS_H_
#define _BOARD_PINS_H_

/* ============================================================
 * 本车（ESP32-S3-WROOM-2-N32R16V）的引脚定义
 *
 * 原代码来自另一组的小车，底盘同型号但接线不同。这里把散在
 * motor.c / encoder.c / ultrasonic.c / tft.c / camera_track.c / track.c
 * 里的引脚全部集中过来，改线只改这一个文件。
 *
 * 右边注释的"他们"是原仓库 esp-projects/camera-other_group 的值，
 * 留着是为了对照，出问题时能一眼看出改了哪几个脚。
 * 本车的值来自 car-spin/README.md 和 camera-test 的实车接线。
 * ============================================================ */

/* ===== TB6612 电机驱动 ===== */
/* A、D 是两个斜置驱动轮，B 是第三个轮（横移轮）。
 * 这个几何角色和原代码一致，所以是按角色映射，不是按脚号映射：
 * 他们的 A 脚号(4/6/5)恰好等于本车的 B 脚号，照抄脚号会把轮子接错。 */
#define PIN_PWMA 9  /* 他们 4  */
#define PIN_AIN1 12 /* 他们 6  */
#define PIN_AIN2 10 /* 他们 5  */

#define PIN_PWMB 4  /* 他们 9  */
#define PIN_BIN1 6  /* 他们 11 */
#define PIN_BIN2 5  /* 他们 10 */

#define PIN_PWMD 16 /* 他们 40 */
#define PIN_DIN1 7  /* 他们 42 */
#define PIN_DIN2 15 /* 他们 41 */

/* 本车 TB6612 的 STBY 接在 GPIO8，必须拉高驱动板才工作。
 * 他们的板子把 STBY 直接跳到 VCC，所以原代码里没有这一脚 ——
 * 少了它的话三个轮子全都不转，而且日志一切正常，很难查。 */
#define PIN_MOTOR_STBY 8 /* 他们无此脚 */

/* ===== HC-SR04 超声波 ===== */
#define PIN_ULTRASONIC_TRIG 18 /* 他们 3  */
#define PIN_ULTRASONIC_ECHO 11 /* 他们 14 */

/* ===== 摄像头云台舵机（MG90S ×2） ===== */
/* 注意本车这两脚和他们的编码器脚、红外脚都撞，见下面的说明。 */
#define PIN_SERVO_YAW 1   /* 水平，他们 8  */
#define PIN_SERVO_PITCH 2 /* 俯仰，他们 18 */

/* ===== ST7735 TFT（128x160，SPI2_HOST） ===== */
#define PIN_TFT_MOSI 14 /* 他们 21 */
#define PIN_TFT_CLK 13  /* 他们 47 */
#define PIN_TFT_CS 47   /* 他们 48 */
#define PIN_TFT_DC 21   /* 他们 17 */
#define PIN_TFT_RST 38  /* 他们 16 */
#define PIN_TFT_MISO (-1)

/* ===== 电机编码器 ===== */
/* 本车没有接编码器。他们的编码器占用 7/15/12/13/2/1，而本车这几脚是：
 *   7  = DIN1（D 轮方向）
 *   15 = DIN2（D 轮方向）
 *   12 = AIN1（A 轮方向）
 *   13 = TFT 时钟
 *   2  = 俯仰舵机
 *   1  = 水平舵机
 * PCNT 一旦把这些脚配成输入，会和电机方向输出、屏幕时钟直接顶牛。
 * 所以默认关闭编码器，速度闭环退化成开环 —— 这也是本车原来固件的做法。
 *
 * 如果后面真的接了编码器：把 USE_ENCODER 打开，把下面六个脚改成实际接线
 * （必须是空闲脚，不能用上面列出的任何一个），然后删掉那句 #error。
 * 下面填的还是他们的原值，纯粹是留个记录，直接拿来用一定会撞脚。 */
/* #define USE_ENCODER 1 */

#define PIN_ENC_A1 7  /* 他们 7 —— 本车是 DIN1 */
#define PIN_ENC_B1 15 /* 他们 15 —— 本车是 DIN2 */
#define PIN_ENC_A2 12 /* 他们 12 —— 本车是 AIN1 */
#define PIN_ENC_B2 13 /* 他们 13 —— 本车是 TFT 时钟 */
#define PIN_ENC_A4 2  /* 他们 2 —— 本车是俯仰舵机 */
#define PIN_ENC_B4 1  /* 他们 1 —— 本车是水平舵机 */

#ifdef USE_ENCODER
#error "编码器脚还是另一组的原值，会和本车的电机方向脚/TFT时钟/舵机撞。请先在 board_pins.h 里改成实际接线，再删掉这句 #error。"
#endif

/* ===== 四路红外循迹（摄像头模式下不用） ===== */
/* track_config.h 里定义了 USE_CAMERA_TRACK，所以 track_init() 不会被调用，
 * 这四脚实际不会被配置。这里填的是 car-spin 的实车红外接线。
 * 警告：41/42 和上面编码器的占位脚重复，2/1 和舵机重复。要切回红外模式的话
 * 必须先确认这些脚没被别的外设占着。 */
#define PIN_IR_OUT1 41 /* 他们 16 */
#define PIN_IR_OUT2 42 /* 他们 17 */
#define PIN_IR_OUT3 2  /* 他们 18 */
#define PIN_IR_OUT4 1  /* 他们 8  */

/* ===== 固定占用，不可改 ===== */
/* GPIO19/20 = USB OTG D-/D+，摄像头走这里，硬件固定。
 * GPIO26/30 + 33~37 = 八线 PSRAM 和 OPI Flash，模组内部占用。 */

#endif
