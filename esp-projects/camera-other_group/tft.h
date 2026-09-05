#ifndef _TFT_H_
#define _TFT_H_

#include <stdint.h>

// 初始化 TFT 显示屏
void tft_init(void);

// 在指定位置显示字符串
void tft_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color);

// 清屏
void tft_clear(uint16_t color);

// 定义常用颜色
#define TFT_BLACK   0x0000
#define TFT_WHITE   0xFFFF
#define TFT_RED     0xF800
#define TFT_GREEN   0x07E0
#define TFT_BLUE    0x001F
#define TFT_YELLOW  0xFFE0

#endif
