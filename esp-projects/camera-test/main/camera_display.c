#include "camera_display.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "tft_st7735.h"

#define CAMERA_DISPLAY_SLOT_COUNT 2
#define CAMERA_DISPLAY_MAX_JPEG_BYTES (160 * 1024)
#define CAMERA_DISPLAY_RGB565_BYTES \
    (TFT_ST7735_WIDTH * (TFT_ST7735_HEIGHT - 8) * 2)

typedef struct {
    uint8_t *jpeg_slots[CAMERA_DISPLAY_SLOT_COUNT];
    size_t jpeg_sizes[CAMERA_DISPLAY_SLOT_COUNT];
    uint8_t *rgb565;
    QueueHandle_t free_slots;
    QueueHandle_t ready_slots;
    bool started;
    uint16_t previous_width;
    uint16_t previous_height;
    uint32_t frames_drawn;
    uint32_t frames_dropped;
} camera_display_state_t;

static const char *TAG = "camera_display";
static camera_display_state_t s_display;

static bool choose_scale(uint16_t source_width,
                         uint16_t source_height,
                         esp_jpeg_image_scale_t *scale)
{
    static const struct {
        uint8_t divider;
        esp_jpeg_image_scale_t scale;
    } choices[] = {
        {1, JPEG_IMAGE_SCALE_0},
        {2, JPEG_IMAGE_SCALE_1_2},
        {4, JPEG_IMAGE_SCALE_1_4},
        {8, JPEG_IMAGE_SCALE_1_8},
    };

    for (size_t i = 0; i < sizeof(choices) / sizeof(choices[0]); ++i) {
        if ((source_width / choices[i].divider) <= TFT_ST7735_WIDTH &&
            (source_height / choices[i].divider) <= TFT_ST7735_HEIGHT - 8) {
            *scale = choices[i].scale;
            return true;
        }
    }
    return false;
}

static bool decode_and_draw(uint8_t *jpeg, size_t jpeg_len)
{
    esp_jpeg_image_cfg_t info_config = {
        .indata = jpeg,
        .indata_size = jpeg_len,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
    };
    esp_jpeg_image_output_t source_info = {0};
    esp_err_t err = esp_jpeg_get_image_info(&info_config, &source_info);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MJPEG header is not a baseline JPEG: %s", esp_err_to_name(err));
        return false;
    }

    esp_jpeg_image_scale_t scale;
    if (!choose_scale(source_info.width, source_info.height, &scale)) {
        ESP_LOGW(TAG, "Frame %ux%u is too large for the TFT preview",
                 (unsigned)source_info.width, (unsigned)source_info.height);
        return false;
    }

    esp_jpeg_image_cfg_t decode_config = {
        .indata = jpeg,
        .indata_size = jpeg_len,
        .outbuf = s_display.rgb565,
        .outbuf_size = CAMERA_DISPLAY_RGB565_BYTES,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = scale,
        .flags = {
            // ST7735 expects the high byte of each RGB565 pixel first.
            .swap_color_bytes = true,
        },
    };
    esp_jpeg_image_output_t output = {0};
    err = esp_jpeg_decode(&decode_config, &output);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MJPEG decode failed: %s", esp_err_to_name(err));
        return false;
    }
    if (output.width == 0 || output.height == 0 || output.width > TFT_ST7735_WIDTH ||
        output.height > TFT_ST7735_HEIGHT - 8 || output.output_len > CAMERA_DISPLAY_RGB565_BYTES) {
        ESP_LOGW(TAG, "Unexpected decoded size %ux%u (%u bytes)",
                 (unsigned)output.width, (unsigned)output.height, (unsigned)output.output_len);
        return false;
    }

    if (output.width != s_display.previous_width || output.height != s_display.previous_height) {
        tft_st7735_fill(0x0000);
        s_display.previous_width = output.width;
        s_display.previous_height = output.height;
    }
    if (!tft_st7735_draw_rgb565(s_display.rgb565, output.width, output.height)) {
        ESP_LOGW(TAG, "TFT draw failed");
        return false;
    }
    return true;
}

static void camera_display_task(void *arg)
{
    (void)arg;
    uint8_t slot;
    int64_t next_report_us = 0;

    while (true) {
        if (xQueueReceive(s_display.ready_slots, &slot, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (decode_and_draw(s_display.jpeg_slots[slot], s_display.jpeg_sizes[slot])) {
            s_display.frames_drawn++;
        }
        if (xQueueSend(s_display.free_slots, &slot, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "Display frame pool corruption");
        }

        const int64_t now = esp_timer_get_time();
        if (now >= next_report_us) {
            ESP_LOGI(TAG, "TFT frames=%u, skipped=%u", (unsigned)s_display.frames_drawn,
                     (unsigned)s_display.frames_dropped);
            next_report_us = now + 5000000;
        }
    }
}

static void release_allocations(void)
{
    for (size_t i = 0; i < CAMERA_DISPLAY_SLOT_COUNT; ++i) {
        heap_caps_free(s_display.jpeg_slots[i]);
        s_display.jpeg_slots[i] = NULL;
    }
    heap_caps_free(s_display.rgb565);
    s_display.rgb565 = NULL;
    if (s_display.free_slots != NULL) {
        vQueueDelete(s_display.free_slots);
        s_display.free_slots = NULL;
    }
    if (s_display.ready_slots != NULL) {
        vQueueDelete(s_display.ready_slots);
        s_display.ready_slots = NULL;
    }
}

esp_err_t camera_display_start(void)
{
    if (s_display.started) {
        return ESP_OK;
    }
    if (!tft_st7735_init()) {
        return ESP_FAIL;
    }

    s_display.free_slots = xQueueCreate(CAMERA_DISPLAY_SLOT_COUNT, sizeof(uint8_t));
    s_display.ready_slots = xQueueCreate(1, sizeof(uint8_t));
    if (s_display.free_slots == NULL || s_display.ready_slots == NULL) {
        ESP_LOGE(TAG, "Could not allocate display queues");
        release_allocations();
        return ESP_ERR_NO_MEM;
    }

    for (uint8_t slot = 0; slot < CAMERA_DISPLAY_SLOT_COUNT; ++slot) {
        s_display.jpeg_slots[slot] = heap_caps_malloc(CAMERA_DISPLAY_MAX_JPEG_BYTES,
                                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_display.jpeg_slots[slot] == NULL ||
            xQueueSend(s_display.free_slots, &slot, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Could not allocate JPEG frame buffer %u", slot);
            release_allocations();
            return ESP_ERR_NO_MEM;
        }
    }

    s_display.rgb565 = heap_caps_malloc(CAMERA_DISPLAY_RGB565_BYTES,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_display.rgb565 == NULL) {
        ESP_LOGE(TAG, "Could not allocate RGB565 frame buffer");
        release_allocations();
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(camera_display_task, "camera_tft", 6144, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not create TFT preview task");
        release_allocations();
        return ESP_ERR_NO_MEM;
    }

    s_display.started = true;
    ESP_LOGI(TAG, "TFT preview started; JPEG frames larger than %u bytes are skipped",
             CAMERA_DISPLAY_MAX_JPEG_BYTES);
    return ESP_OK;
}

bool camera_display_submit(const uint8_t *jpeg, size_t jpeg_len)
{
    if (!s_display.started || jpeg == NULL || jpeg_len == 0) {
        return false;
    }
    if (jpeg_len > CAMERA_DISPLAY_MAX_JPEG_BYTES) {
        if ((++s_display.frames_dropped % 100U) == 1U) {
            ESP_LOGW(TAG, "Dropping %u-byte JPEG; preview limit is %u bytes",
                     (unsigned)jpeg_len, CAMERA_DISPLAY_MAX_JPEG_BYTES);
        }
        return false;
    }

    uint8_t slot;
    if (xQueueReceive(s_display.free_slots, &slot, 0) != pdTRUE) {
        // A queued frame is already stale. Reuse its slot rather than making
        // the USB callback wait for the decoder and TFT transfer.
        if (xQueueReceive(s_display.ready_slots, &slot, 0) != pdTRUE) {
            s_display.frames_dropped++;
            return false;
        }
        s_display.frames_dropped++;
    }

    memcpy(s_display.jpeg_slots[slot], jpeg, jpeg_len);
    s_display.jpeg_sizes[slot] = jpeg_len;
    if (xQueueSend(s_display.ready_slots, &slot, 0) == pdTRUE) {
        return true;
    }

    // A new frame arrived while a previous one was queued. Keep only this
    // newer frame and return the replaced slot to the free pool.
    uint8_t stale_slot;
    if (xQueueReceive(s_display.ready_slots, &stale_slot, 0) == pdTRUE) {
        xQueueSend(s_display.free_slots, &stale_slot, 0);
        s_display.frames_dropped++;
    }
    if (xQueueSend(s_display.ready_slots, &slot, 0) == pdTRUE) {
        return true;
    }

    xQueueSend(s_display.free_slots, &slot, 0);
    s_display.frames_dropped++;
    return false;
}
