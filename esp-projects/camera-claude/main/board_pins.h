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
 * 本车的值来自根目录 引脚对应表2.xlsx；LCD CS 已确认从 GPIO47 改接 GPIO0。
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
/* 实物 CS 线从47移到0。GPIO0兼作BOOT，复位期间LCD端不能拉低它。
 * GPIO47/48是1.8V域；屏幕曾能显示不等于电平符合规格。
 * 如改为单屏CS接GND，PIN_TFT_CS应设为(-1)，不要同时接GPIO0。 */
#define PIN_TFT_MOSI 14 /* 他们 21 */
#define PIN_TFT_CLK 13  /* 他们 47 */
#define PIN_TFT_CS 0    /* 他们 48；本车旧表为47，现已确认改接0 */
#define PIN_TFT_DC 21   /* 他们 17 */
#define PIN_TFT_RST 38  /* 他们 16 */
#define PIN_TFT_MISO (-1)

/* ===== 电机编码器 ===== */
/* 表中 E1/E2/E4 分别对应 A/B/D 电机，全部启用。
 * 模组只引出 GPIO0~21、38~48；GPIO22~37 不能在本 WROOM-2 上分配。
 * GPIO3 是 JTAG 选择 strapping 脚，出厂默认 eFuse 组合忽略其电平；
 * 修改过 JTAG eFuse 时须按数据手册表4-5核对，不能一概声称没有限制。
 * GPIO39~42 可用作 PCNT 输入，与外部四线 JTAG 不能同时使用。
 * GPIO47/48 为 1.8V 域（中文数据手册第11页脚注2），不能接3.3V编码器。
 * GPIO0 给 LCD CS：上电保持高电平，不能外接下拉；GPIO43/44 留给 UART0。
 * GPIO45/46 是 strapping 脚，复位采样后可作普通GPIO；是否可接某外设
 * 要检查复位电平及eFuse，不能把“默认弱下拉”写成“上电必须为低”。
 */
#define USE_ENCODER 1

#define PIN_ENC_A1 39 /* A 轮，他们 7  */
#define PIN_ENC_B1 40 /* A 轮，他们 15 */
#define PIN_ENC_A2 17 /* B 轮，他们 12 */
#define PIN_ENC_B2 3  /* B 轮，他们 13 */
#define PIN_ENC_A4 41 /* D 轮，他们 2 —— 原红外 OUT1，红外已拆 */
#define PIN_ENC_B4 42 /* D 轮，他们 1 —— 原红外 OUT2，红外已拆 */

/* ===== 四路红外循迹（本车已拆除，仅保留宏让 track.c 能编译） ===== */
/* 红外模块已经从车上拆掉，这四个脚现在归别人：
 *   41/42 → D 轮编码器（PIN_ENC_A4/B4）
 *   2/1   → 俯仰舵机 / 水平舵机
 * 下面的值是 car-spin 的旧红外接线，留着纯粹是让 track.c 有东西可编。
 * track.c 顶上加了 #error 拦住"切回红外模式"这个操作 —— 真切回去会把这四脚
 * 重新配成输入，D 轮编码器和两个舵机同时失效。 */
#define PIN_IR_OUT1 41 /* 已让给 D 轮编码器 A 相 */
#define PIN_IR_OUT2 42 /* 已让给 D 轮编码器 B 相 */
#define PIN_IR_OUT3 2  /* 已让给俯仰舵机 */
#define PIN_IR_OUT4 1  /* 已让给水平舵机 */

/* ===== 固定占用，不可改 ===== */
/* GPIO19/20 = USB OTG D-/D+，摄像头走这里，硬件固定。
 * GPIO26/30 + 33~37 = 八线 PSRAM 和 OPI Flash，模组内部占用。 */

#endif
