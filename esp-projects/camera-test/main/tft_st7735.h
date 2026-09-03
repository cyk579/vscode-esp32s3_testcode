#pragma once

#include <stdbool.h>
#include <stddef.h>
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

/* Draw a source up to 320x240 by sampling every second row and column. */
bool tft_st7735_draw_rgb565_2x(const uint8_t *rgb565_big_endian,
                              uint16_t source_width,
                              uint16_t source_height);

/* Same 2:1 sampling, but keep the source coordinates and paint pixels outside
 * the crop rectangle with blank_color. No full-size intermediate framebuffer. */
bool tft_st7735_draw_rgb565_2x_crop(const uint8_t *rgb565_big_endian,
                                    uint16_t source_width,
                                    uint16_t source_height,
                                    uint16_t crop_left,
                                    uint16_t crop_top,
                                    uint16_t crop_right,
                                    uint16_t crop_bottom,
                                    uint16_t blank_color);

/* Stream a small fixed-width text page without allocating a framebuffer. */
bool tft_st7735_draw_text_lines(const char *const lines[],
                                size_t line_count,
                                uint16_t foreground,
                                uint16_t background);
