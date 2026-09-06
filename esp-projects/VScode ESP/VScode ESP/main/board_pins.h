#pragma once

#include "driver/gpio.h"

/* 接线依据：仓库根目录 引脚对应表2.xlsx / Sheet1，LCD CS 恢复 GPIO0，注意启动绑带限制。
 * 按 car-spin/README.md 的车体位置映射：M1=左前D，M2=后B，M3=右前A。
 * 这里的数字都是 GPIO 号，不是 WROOM-2 模组焊盘序号。
 */

/* ST7735，SPI2 通过 GPIO matrix 路由到 13/14。 */
/* 注意：WROOM-2 的 GPIO47/48 是 1.8V 域，不能默认直连 3.3V 屏幕。
 * 本轮将 LCD CS 从 GPIO1 改回 GPIO0，避免与水平舵机信号共线。
 * 必须断电后换线：屏幕 CS 只接 GPIO0，不再连接 GPIO1 或 GND。
 * GPIO0 是 BOOT 启动绑带脚，正常上电/复位时不能被屏幕电路拉低。
 * 运行中不要按 BOOT；本工程不驱动舵机，不使用时断开舵机供电并固定摄像头。
 * 详见仓库根目录 引脚核查与复现说明.md。
 */
#define LCD_CS_GPIO    GPIO_NUM_0
#define LCD_SCK_GPIO   GPIO_NUM_13
#define LCD_MOSI_GPIO  GPIO_NUM_14
#define LCD_DC_GPIO    GPIO_NUM_21
#define LCD_RST_GPIO   GPIO_NUM_38

#define TRIG_GPIO      GPIO_NUM_18
#define ECHO_GPIO      GPIO_NUM_11
#define MOTOR_STBY_GPIO GPIO_NUM_8

/* M1：左前轮 -> 本车 D，编码器 E4A/E4B。 */
#define M1_ENC_A       GPIO_NUM_41
#define M1_ENC_B       GPIO_NUM_42
#define M1_INA         GPIO_NUM_7
#define M1_INB         GPIO_NUM_15
#define M1_PWM         GPIO_NUM_16

/* M2：后轮 -> 本车 B，编码器 E2A/E2B。 */
#define M2_ENC_A       GPIO_NUM_17
#define M2_ENC_B       GPIO_NUM_3
#define M2_INA         GPIO_NUM_6
#define M2_INB         GPIO_NUM_5
#define M2_PWM         GPIO_NUM_4

/* M3：右前轮 -> 本车 A，编码器 E1A/E1B。 */
#define M3_ENC_A       GPIO_NUM_39
#define M3_ENC_B       GPIO_NUM_40
#define M3_INA         GPIO_NUM_12
#define M3_INB         GPIO_NUM_10
#define M3_PWM         GPIO_NUM_9

/* USB 摄像头固定 D-=GPIO19、D+=GPIO20，由 usb_stream 驱动配置。
 * 本组没有舵机控制：GPIO1/2 均不配置，LCD CS 使用 GPIO0。
 * 编码器仍只配置输入，原程序没有 PCNT 计数或速度闭环。
 */
