#pragma once                                              // 防止头文件被重复包含。

#include <stdbool.h>                                      // 使用 bool 类型。
#include <stdint.h>                                       // 使用 uint8_t 类型。

typedef struct {                                          // 定义 TFT 需要显示的一帧状态。
    float distance_cm;                                    // 超声波距离；小于 0 表示无有效测距。
    uint8_t ir_mask;                                      // 四路红外 ACTIVE 掩码。
    int error;                                            // 当前巡线误差。
    int turn;                                             // 当前 yaw/转向命令。
    int motor_a;                                          // Motor A 逻辑输出。
    int motor_b;                                          // Motor B 逻辑输出。
    int motor_d;                                          // Motor D 逻辑输出。
    const char *mode;                                     // 当前 TEST/LINE/AV-L/AV-F/AV-R/DIST/FAIL/END 状态。
} tft_status_t;                                           // TFT 状态结构结束。

bool tft_st7735_init(void);                               // 初始化 ST7735。
void tft_st7735_show(const tft_status_t *status);         // 显示一帧状态。
