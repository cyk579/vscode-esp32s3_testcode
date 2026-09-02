#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
typedef enum {
    JPEG_IMAGE_FORMAT_RGB565 = 0,
} esp_jpeg_image_format_t;
typedef enum {
    JPEG_IMAGE_SCALE_0 = 0,
    JPEG_IMAGE_SCALE_1_2,
    JPEG_IMAGE_SCALE_1_4,
    JPEG_IMAGE_SCALE_1_8,
} esp_jpeg_image_scale_t;
typedef struct {
    const uint8_t *indata;
    size_t indata_size;
    uint8_t *outbuf;
    size_t outbuf_size;
    esp_jpeg_image_format_t out_format;
    esp_jpeg_image_scale_t out_scale;
    struct { int swap_color_bytes; } flags;
    struct { uint8_t *working_buffer; size_t working_buffer_size; } advanced;
} esp_jpeg_image_cfg_t;
typedef struct {
    uint16_t width;
    uint16_t height;
    size_t output_len;
} esp_jpeg_image_output_t;
esp_err_t esp_jpeg_get_image_info(const esp_jpeg_image_cfg_t *cfg,
                                  esp_jpeg_image_output_t *output);
esp_err_t esp_jpeg_decode(const esp_jpeg_image_cfg_t *cfg,
                          esp_jpeg_image_output_t *output);
