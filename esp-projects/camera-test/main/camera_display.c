#include "camera_display.h"

#include <stdio.h>
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
#define CAMERA_DISPLAY_MAX_JPEG_BYTES (256 * 1024)
#define CAMERA_DISPLAY_RGB565_BYTES \
    (TFT_ST7735_WIDTH * (TFT_ST7735_HEIGHT - 8) * 2)
#define CAMERA_DISPLAY_JPEG_WORK_BYTES (16 * 1024)
#define CAMERA_BINARY_ROI_TOP_PERCENT 30
#define CAMERA_BINARY_ROI_BOTTOM_PERCENT 95
#define CAMERA_BINARY_ROW_STEP 3
#define CAMERA_BINARY_DARK_PERCENTILE 2
#define CAMERA_BINARY_LIGHT_PERCENTILE 90
#define CAMERA_BINARY_MIN_CONTRAST 32
#define CAMERA_BINARY_BLACK_FRACTION_PERCENT 20
#define CAMERA_BINARY_THRESHOLD_MIN 25
#define CAMERA_BINARY_THRESHOLD_MAX 120
/* Set a fixed grayscale threshold for track calibration; 0 keeps adaptive mode. */
#define CAMERA_BINARY_FIXED_THRESHOLD 70

typedef struct {
    uint8_t *jpeg_slots[CAMERA_DISPLAY_SLOT_COUNT];
    size_t jpeg_sizes[CAMERA_DISPLAY_SLOT_COUNT];
    uint8_t *rgb565;
    uint8_t *jpeg_work;
    QueueHandle_t free_slots;
    QueueHandle_t ready_slots;
    bool started;
    bool tft_ready;
    camera_display_frame_callback_t frame_callback;
    void *frame_callback_ctx;
    uint16_t previous_width;
    uint16_t previous_height;
    uint8_t binary_threshold;
    uint32_t frames_drawn;
    uint32_t frames_dropped;
} camera_display_state_t;

static const char *TAG = "camera_display";
static camera_display_state_t s_display;

static uint8_t rgb565_luma(const uint8_t *pixel)
{
    const uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    const uint8_t red = (uint8_t)(((value >> 11) & 0x1f) * 255U / 31U);
    const uint8_t green = (uint8_t)(((value >> 5) & 0x3f) * 255U / 63U);
    const uint8_t blue = (uint8_t)((value & 0x1f) * 255U / 31U);
    return (uint8_t)((77U * red + 150U * green + 29U * blue) >> 8);
}

#if CAMERA_BINARY_FIXED_THRESHOLD == 0
static uint8_t histogram_percentile(const uint16_t histogram[256],
                                    uint32_t sample_count,
                                    uint32_t percentile)
{
    const uint32_t target = (sample_count * percentile + 99U) / 100U;
    uint32_t cumulative = 0;
    for (uint32_t value = 0; value < 256U; ++value) {
        cumulative += histogram[value];
        if (cumulative >= target) {
            return (uint8_t)value;
        }
    }
    return 255;
}
#endif

static uint8_t calculate_binary_threshold(const uint8_t *frame,
                                          uint16_t width,
                                          uint16_t height)
{
#if CAMERA_BINARY_FIXED_THRESHOLD > 0
    (void)frame;
    (void)width;
    (void)height;
    return CAMERA_BINARY_FIXED_THRESHOLD;
#else
    uint16_t histogram[256] = {0};
    const int x_step = width >= 96 ? 2 : 1;
    int top = (int)height * CAMERA_BINARY_ROI_TOP_PERCENT / 100;
    int bottom = (int)height * CAMERA_BINARY_ROI_BOTTOM_PERCENT / 100;
    if (bottom >= (int)height) {
        bottom = (int)height - 1;
    }

    uint32_t sample_count = 0;
    for (int y = top; y <= bottom; y += CAMERA_BINARY_ROW_STEP) {
        for (int x = 0; x < (int)width; x += x_step) {
            ++histogram[rgb565_luma(frame + (((size_t)y * width + (size_t)x) * 2))];
            ++sample_count;
        }
    }
    if (sample_count == 0) {
        return 0;
    }

    const int dark = histogram_percentile(histogram, sample_count,
                                          CAMERA_BINARY_DARK_PERCENTILE);
    const int light = histogram_percentile(histogram, sample_count,
                                           CAMERA_BINARY_LIGHT_PERCENTILE);
    if (light - dark < CAMERA_BINARY_MIN_CONTRAST) {
        return 0;
    }

    int threshold = dark +
                    (light - dark) * CAMERA_BINARY_BLACK_FRACTION_PERCENT / 100;
    if (threshold < CAMERA_BINARY_THRESHOLD_MIN) {
        threshold = CAMERA_BINARY_THRESHOLD_MIN;
    }
    if (threshold > CAMERA_BINARY_THRESHOLD_MAX) {
        threshold = CAMERA_BINARY_THRESHOLD_MAX;
    }
    return (uint8_t)threshold;
#endif
}

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
    static bool logged_format_diagnostic;
    /* Recover the newest complete JPEG when a malformed bulk UVC device
     * concatenates an incomplete frame with the following frame. */
    size_t candidate_soi = jpeg_len;
    size_t selected_soi = jpeg_len;
    size_t selected_end = 0;
    for (size_t i = 0; i + 1 < jpeg_len; ++i) {
        if (jpeg[i] == 0xff && jpeg[i + 1] == 0xd8) {
            candidate_soi = i;
        } else if (jpeg[i] == 0xff && jpeg[i + 1] == 0xd9 && candidate_soi < i) {
            selected_soi = candidate_soi;
            selected_end = i + 2;
            candidate_soi = jpeg_len;
        }
    }
    if (selected_soi == jpeg_len) {
        ESP_LOGW(TAG, "MJPEG frame has no complete JPEG SOI/EOI pair (len=%u)",
                 (unsigned)jpeg_len);
        return false;
    }
    jpeg += selected_soi;
    jpeg_len = selected_end - selected_soi;

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
        if (!logged_format_diagnostic) {
            logged_format_diagnostic = true;
            size_t sof0 = 0, sof2 = 0, dht = 0, dqt = 0;
            for (size_t i = 0; i + 1 < jpeg_len; ++i) {
                if (jpeg[i] != 0xff) {
                    continue;
                }
                switch (jpeg[i + 1]) {
                case 0xc0: sof0++; break;
                case 0xc2: sof2++; break;
                case 0xc4: dht++; break;
                case 0xdb: dqt++; break;
                default: break;
                }
            }
            char head[3 * 32 + 1] = {0};
            size_t head_len = jpeg_len < 32 ? jpeg_len : 32;
            for (size_t i = 0; i < head_len; ++i) {
                snprintf(head + i * 3, 4, "%02x ", jpeg[i]);
            }
            char markers[160] = {0};
            size_t marker_pos = 0;
            for (size_t i = 0; i + 1 < jpeg_len && marker_pos + 8 < sizeof(markers); ++i) {
                if (jpeg[i] == 0xff && jpeg[i + 1] != 0x00 && jpeg[i + 1] != 0xff) {
                    int written = snprintf(markers + marker_pos, sizeof(markers) - marker_pos,
                                            "%02x@%u ", jpeg[i + 1], (unsigned)i);
                    if (written > 0) marker_pos += (size_t)written;
                }
            }
            ESP_LOGW(TAG, "JPEG diagnostic len=%u SOF0=%u SOF2=%u DQT=%u DHT=%u head=%s",
                     (unsigned)jpeg_len, (unsigned)sof0, (unsigned)sof2,
                     (unsigned)dqt, (unsigned)dht, head);
            ESP_LOGW(TAG, "JPEG markers: %s", markers);
        }
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
            /* Decoder RGB565 words are little-endian in memory; ST7735
             * expects the high byte first on the wire. */
            .swap_color_bytes = true,
        },
        .advanced = {
            .working_buffer = s_display.jpeg_work,
            .working_buffer_size = CAMERA_DISPLAY_JPEG_WORK_BYTES,
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

    const uint8_t source_threshold =
        calculate_binary_threshold(s_display.rgb565, output.width, output.height);
    for (size_t i = 0; i < output.output_len; i += 2) {
        const uint8_t luma = rgb565_luma(s_display.rgb565 + i);
        const uint16_t bw = source_threshold != 0 && luma <= source_threshold ?
                            0x0000 : 0xffff;
        s_display.rgb565[i] = (uint8_t)(bw >> 8);
        s_display.rgb565[i + 1] = (uint8_t)bw;
    }
    s_display.binary_threshold = source_threshold;

    if (s_display.frame_callback != NULL) {
        s_display.frame_callback(s_display.rgb565, output.width, output.height,
                                 source_threshold, s_display.frame_callback_ctx);
    }

    if (!s_display.tft_ready) {
        return true;
    }

    if (output.width != s_display.previous_width || output.height != s_display.previous_height) {
        tft_st7735_fill(0xffff);
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
            ESP_LOGI(TAG, "Decoded frames=%u, skipped=%u, binary threshold=%u",
                     (unsigned)s_display.frames_drawn,
                     (unsigned)s_display.frames_dropped,
                     (unsigned)s_display.binary_threshold);
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
    heap_caps_free(s_display.jpeg_work);
    s_display.jpeg_work = NULL;
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
    s_display.tft_ready = tft_st7735_init();
    if (!s_display.tft_ready) {
        ESP_LOGW(TAG, "ST7735 unavailable; decoded-frame callbacks remain active");
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

    s_display.jpeg_work = heap_caps_malloc(CAMERA_DISPLAY_JPEG_WORK_BYTES,
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_display.jpeg_work == NULL) {
        ESP_LOGE(TAG, "Could not allocate JPEG working buffer");
        release_allocations();
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(camera_display_task, "camera_tft", 6144, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not create TFT preview task");
        release_allocations();
        return ESP_ERR_NO_MEM;
    }

    s_display.started = true;
    ESP_LOGI(TAG, "Camera decoder started; fixed binary threshold=%u, TFT preview=%s; JPEG frames larger than %u bytes are skipped",
             (unsigned)CAMERA_BINARY_FIXED_THRESHOLD,
             s_display.tft_ready ? "ready" : "disabled",
             CAMERA_DISPLAY_MAX_JPEG_BYTES);
    return ESP_OK;
}

void camera_display_set_frame_callback(camera_display_frame_callback_t callback,
                                       void *user_ctx)
{
    s_display.frame_callback = callback;
    s_display.frame_callback_ctx = user_ctx;
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
