#pragma once

#include <stdbool.h>
#include <stdint.h>

/* The 128x160 panel is used in landscape so a 320x240 UVC frame scales to
 * 160x120 and leaves four-pixel black bars above and below the image. */
#define TFT_ST7735_WIDTH  160
#define TFT_ST7735_HEIGHT 128

bool tft_st7735_init(void);
bool tft_st7735_fill(uint16_t color);
bool tft_st7735_draw_rgb565(const uint8_t *rgb565_big_endian,
                             uint16_t width,
                             uint16_t height);
