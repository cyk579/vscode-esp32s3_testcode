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
/* 他们的编码器原本占用 7/15/12/13/2/1，这六脚在本车上分别是 DIN1、DIN2、
 * AIN1、TFT 时钟、俯仰舵机、水平舵机，全都已被占用，所以脚号必须重新分配。
 *
 * 下面这组是按 ESP32-S3-WROOM-2 数据手册表 3-1（管脚定义）和表 4-1
 * （strapping 管脚默认配置）逐条筛出来的。
 *
 * 模组实际引出 33 个 GPIO：0~21、38~48。没有 26，也没有 33~37 ——
 * 那几个在模组内部给 OPI Flash 和 OCT PSRAM 用了，压根没引出来。
 *
 * 本车已占 21 个：电机 9/12/10、4/6/5、16/7/15、STBY 8；超声波 18/11；
 * 舵机 1/2；TFT 14/13/47/21/38；USB 摄像头 19/20（硬件固定）。
 *
 * 剩下 12 个里能用的只有 6 个，正好够，没有余量：
 *   可用：3、17、39、40、41、42
 *   43/44 = UART0 串口，占了就没法 monitor 和烧录
 *   45/46 = 表 4-1 写明弱下拉、上电必须为低。编码器空闲时输出高电平会拉高
 *           它们，而 45 决定 VDD_SPI 电压，被拉高可能起不来
 *   0     = 表 4-1 弱上拉，BOOT 脚。上电瞬间被编码器拉低就进下载模式
 *   48    = 1.8V 域，见下面的警告
 *
 * 关于 39~42：这四脚默认功能是 MTCK/MTDO/MTDI/MTMS，但外部 JTAG 需要
 * eFuse STRAP_JTAG_SEL 开启才生效，出厂默认走 USB-Serial-JTAG，所以是
 * 普通 GPIO。GPIO3 同理——它管的 JTAG 源选择受同一颗 eFuse 控制，且表 4-1
 * 写明「浮空」无内部上下拉，做输入是安全的。
 *
 * 【不能用 GPIO47/48】数据手册表 3-1 脚注 2：ESP32-S3R8V 和 ESP32-S3R16V
 * 的 VDD_SPI 已设为 1.8V，所以 GPIO47/48 的工作电压也是 1.8V。本车模组是
 * N32R16V，末尾的 V 就是这个意思。编码器由 3.3V 供电、输出 3.3V 逻辑，
 * 灌进 1.8V 域的脚超规格，有损坏风险。
 *
 * 编号 1/2/4 对应电机 A/B/D（见 encoder.c 里 pcnt_units[0..2] 的顺序）。
 * A、D 是斜置驱动轮，闭环里最关键的一对，放在物理相邻的 39~42 上便于走线；
 * B 是横移轮，只在避障动作里用到，给了分散的 17/3。 */
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
