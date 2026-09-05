#include "camera_display.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#if CONFIG_EXAMPLE_ENABLE_TFT_PREVIEW
#include "tft_st7735.h"
#endif

/* Three JPEG slots let USB reception and control decode overlap. */
#define CAMERA_DISPLAY_SLOT_COUNT 3
#define CAMERA_DISPLAY_MAX_JPEG_BYTES (256 * 1024)
#define CAMERA_CONTROL_BUFFER_COUNT 2
#define CAMERA_CONTROL_MAX_WIDTH 160
#define CAMERA_CONTROL_MAX_HEIGHT 120
#define CAMERA_CONTROL_RGB565_BYTES \
    ((size_t)CAMERA_CONTROL_MAX_WIDTH * CAMERA_CONTROL_MAX_HEIGHT * 2)
#define CAMERA_DISPLAY_JPEG_WORK_BYTES (16 * 1024)
#define CAMERA_DECODE_IDLE_YIELD_FRAMES 4U

#define CAMERA_BINARY_ROI_TOP_PERCENT 15
#define CAMERA_BINARY_ROI_BOTTOM_PERCENT 100
#define CAMERA_BINARY_ROI_LEFT_PERCENT 10
#define CAMERA_BINARY_ROI_RIGHT_PERCENT 90
#define CAMERA_BINARY_ROW_STEP 2
#define CAMERA_BINARY_DARK_PERCENTILE 2
#define CAMERA_BINARY_LIGHT_PERCENTILE 90
#define CAMERA_BINARY_MIN_CONTRAST 32
#define CAMERA_BINARY_BLACK_FRACTION_PERCENT 35
#define CAMERA_BINARY_THRESHOLD_MIN 25
#define CAMERA_BINARY_THRESHOLD_MAX 120
#define CAMERA_BINARY_THRESHOLD_SLEW 4
#define CAMERA_BINARY_THRESHOLD_FILTER_OLD 3
#define CAMERA_BINARY_THRESHOLD_FILTER_NEW 1

#define CAMERA_TFT_STATUS_REFRESH_MS 500U

typedef struct {
    uint8_t slot;
    size_t jpeg_len;
    uint32_t sequence;
    int64_t capture_us;
} jpeg_frame_ref_t;

typedef struct {
    uint8_t buffer;
    uint16_t width;
    uint16_t height;
    uint8_t threshold;
    uint32_t sequence;
    int64_t capture_us;
} control_frame_ref_t;

typedef struct {
    uint8_t *jpeg_slots[CAMERA_DISPLAY_SLOT_COUNT];
    uint8_t *control_rgb565[CAMERA_CONTROL_BUFFER_COUNT];
    uint8_t *control_jpeg_work;
    QueueHandle_t free_slots;
    QueueHandle_t ready_slots;
    QueueHandle_t free_control_buffers;
    QueueHandle_t ready_control_frames;
    bool started;
    bool tft_ready;
    camera_display_frame_callback_t frame_callback;
    void *frame_callback_ctx;
    camera_display_status_callback_t status_callback;
    void *status_callback_ctx;
    camera_display_preview_callback_t preview_callback;
    void *preview_callback_ctx;
    uint8_t binary_threshold;
    bool threshold_initialized;
    uint8_t threshold_filtered;
    volatile uint32_t camera_frames;
    volatile uint32_t processed_frames;
    volatile uint32_t control_frames;
    volatile uint32_t preview_frames;
    volatile uint32_t frames_dropped;
    volatile uint32_t control_dropped_frames;
    volatile uint32_t preview_dropped_frames;
    volatile uint32_t last_control_decode_us;
    volatile uint32_t last_preview_decode_us;
    volatile uint32_t last_threshold_us;
    volatile uint32_t last_tft_us;
    volatile uint32_t last_control_age_us;
    volatile uint32_t last_control_sequence;
    uint32_t next_sequence;
} camera_display_state_t;

static const char *TAG = "camera_display";
static camera_display_state_t s_display;
static uint32_t s_mjpeg_drop_log_count;

static void log_mjpeg_drop(const char *reason, size_t jpeg_len)
{
    const uint32_t count = ++s_mjpeg_drop_log_count;
    /* A damaged stream can deliver hundreds of bad frames in a row.  Do not
     * let warning output become another source of control latency. */
    if (count == 1 || (count % 32U) == 0U) {
        ESP_LOGW(TAG, "%s (len=%u dropped=%u)", reason,
                 (unsigned)jpeg_len, (unsigned)count);
    }
}

static uint8_t rgb565_luma(const uint8_t *pixel)
{
    const uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    const uint8_t red = (uint8_t)(((value >> 11) & 0x1f) * 255U / 31U);
    const uint8_t green = (uint8_t)(((value >> 5) & 0x3f) * 255U / 63U);
    const uint8_t blue = (uint8_t)((value & 0x1f) * 255U / 31U);
    return (uint8_t)((77U * red + 150U * green + 29U * blue) >> 8);
}

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

static uint8_t calculate_binary_threshold(const uint8_t *frame,
                                          uint16_t width,
                                          uint16_t height)
{
    uint16_t histogram[256] = {0};
    const int x_step = width >= 96 ? 2 : 1;
    int top = (int)height * CAMERA_BINARY_ROI_TOP_PERCENT / 100;
    int bottom = (int)height * CAMERA_BINARY_ROI_BOTTOM_PERCENT / 100;
    if (bottom >= (int)height) {
        bottom = (int)height - 1;
    }
    const int left = (int)width * CAMERA_BINARY_ROI_LEFT_PERCENT / 100;
    const int right = ((int)width * CAMERA_BINARY_ROI_RIGHT_PERCENT / 100) - 1;

    uint32_t sample_count = 0;
    for (int y = top; y <= bottom; y += CAMERA_BINARY_ROW_STEP) {
        for (int x = left; x <= right; x += x_step) {
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
}

static uint8_t filter_binary_threshold(uint8_t candidate)
{
    if (candidate == 0) {
        return 0;
    }
    if (!s_display.threshold_initialized) {
        s_display.threshold_filtered = candidate;
        s_display.threshold_initialized = true;
        return candidate;
    }

    int limited = candidate;
    const int delta = limited - s_display.threshold_filtered;
    if (delta > CAMERA_BINARY_THRESHOLD_SLEW) {
        limited = s_display.threshold_filtered + CAMERA_BINARY_THRESHOLD_SLEW;
    } else if (delta < -CAMERA_BINARY_THRESHOLD_SLEW) {
        limited = s_display.threshold_filtered - CAMERA_BINARY_THRESHOLD_SLEW;
    }
    s_display.threshold_filtered = (uint8_t)((s_display.threshold_filtered *
                                                CAMERA_BINARY_THRESHOLD_FILTER_OLD +
                                                limited * CAMERA_BINARY_THRESHOLD_FILTER_NEW) /
                                               (CAMERA_BINARY_THRESHOLD_FILTER_OLD +
                                                CAMERA_BINARY_THRESHOLD_FILTER_NEW));
    return s_display.threshold_filtered;
}

static bool find_complete_jpeg(const uint8_t *data, size_t length,
                               size_t *offset, size_t *jpeg_length)
{
    size_t candidate_soi = SIZE_MAX;
    size_t selected_soi = SIZE_MAX;
    size_t selected_end = 0;
    for (size_t i = 0; i + 1 < length; ++i) {
        if (data[i] == 0xff && data[i + 1] == 0xd8) {
            candidate_soi = i;
        } else if (data[i] == 0xff && data[i + 1] == 0xd9 &&
                   candidate_soi != SIZE_MAX) {
            selected_soi = candidate_soi;
            selected_end = i + 2;
            candidate_soi = SIZE_MAX;
        }
    }
    if (selected_soi == SIZE_MAX || selected_end <= selected_soi) {
        return false;
    }
    *offset = selected_soi;
    *jpeg_length = selected_end - selected_soi;
    return true;
}

static void log_jpeg_diagnostic(const uint8_t *jpeg, size_t jpeg_len);

/* esp_jpeg_get_image_info() expects a complete baseline header and walks
 * marker lengths without a separate bounds contract.  Validate the header
 * here so a truncated UVC payload is discarded before that parser runs. */
static bool jpeg_has_baseline_header(const uint8_t *jpeg, size_t jpeg_len)
{
    if (jpeg == NULL || jpeg_len < 12 || jpeg[0] != 0xff || jpeg[1] != 0xd8 ||
        jpeg[jpeg_len - 2] != 0xff || jpeg[jpeg_len - 1] != 0xd9) {
        return false;
    }

    bool saw_sof0 = false;
    size_t pos = 2;
    while (pos + 1 < jpeg_len) {
        if (jpeg[pos++] != 0xff) {
            return false;
        }
        while (pos < jpeg_len && jpeg[pos] == 0xff) {
            ++pos;
        }
        if (pos >= jpeg_len) {
            return false;
        }
        const uint8_t marker = jpeg[pos++];
        if (marker == 0xd9) {
            return false;
        }
        if (marker == 0xd8 || (marker >= 0xd0 && marker <= 0xd7) ||
            marker == 0x01) {
            continue;
        }
        if (pos + 2 > jpeg_len) {
            return false;
        }
        const size_t segment_len = ((size_t)jpeg[pos] << 8) | jpeg[pos + 1];
        if (segment_len < 2 || pos + segment_len > jpeg_len) {
            return false;
        }
        if (marker == 0xda) { /* SOS: entropy data follows the header. */
            return saw_sof0;
        }
        if (marker == 0xc0) {
            saw_sof0 = true;
        } else if (marker == 0xc2) {
            /* The bundled decoder intentionally supports baseline JPEG only. */
            return false;
        }
        pos += segment_len;
    }
    return false;
}

static void log_jpeg_diagnostic(const uint8_t *jpeg, size_t jpeg_len)
{
    static bool logged;
    if (logged) {
        return;
    }
    logged = true;
    size_t sof0 = 0;
    size_t sof2 = 0;
    size_t dht = 0;
    size_t dqt = 0;
    for (size_t i = 0; i + 1 < jpeg_len; ++i) {
        if (jpeg[i] != 0xff) {
            continue;
        }
        switch (jpeg[i + 1]) {
        case 0xc0: ++sof0; break;
        case 0xc2: ++sof2; break;
        case 0xc4: ++dht; break;
        case 0xdb: ++dqt; break;
        default: break;
        }
    }
    char head[3 * 32 + 1] = {0};
    const size_t head_len = jpeg_len < 32 ? jpeg_len : 32;
    for (size_t i = 0; i < head_len; ++i) {
        snprintf(head + i * 3, 4, "%02x ", jpeg[i]);
    }
    ESP_LOGW(TAG, "JPEG diagnostic len=%u SOF0=%u SOF2=%u DQT=%u DHT=%u head=%s",
             (unsigned)jpeg_len, (unsigned)sof0, (unsigned)sof2,
             (unsigned)dqt, (unsigned)dht, head);
}

static bool choose_scale(uint16_t source_width, uint16_t source_height,
                         uint16_t max_width, uint16_t max_height,
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
        const uint16_t scaled_width = (source_width + choices[i].divider - 1) /
                                      choices[i].divider;
        const uint16_t scaled_height = (source_height + choices[i].divider - 1) /
                                       choices[i].divider;
        if (scaled_width <= max_width && scaled_height <= max_height) {
            *scale = choices[i].scale;
            return true;
        }
    }
    return false;
}

static bool decode_jpeg_to_buffer(const uint8_t *input, size_t input_len,
                                  uint8_t *output_buffer, size_t output_size,
                                  uint16_t max_width, uint16_t max_height,
                                  uint8_t *working_buffer,
                                  uint16_t *output_width, uint16_t *output_height,
                                  uint32_t *decode_us)
{
    const int64_t start_us = esp_timer_get_time();
    size_t offset = 0;
    size_t jpeg_len = 0;
    if (!find_complete_jpeg(input, input_len, &offset, &jpeg_len)) {
        log_mjpeg_drop("MJPEG frame has no complete JPEG SOI/EOI pair", input_len);
        return false;
    }
    const uint8_t *jpeg = input + offset;

    if (!jpeg_has_baseline_header(jpeg, jpeg_len)) {
        log_mjpeg_drop("Dropping malformed/non-baseline MJPEG frame", jpeg_len);
        log_jpeg_diagnostic(jpeg, jpeg_len);
        return false;
    }

    esp_jpeg_image_cfg_t info_config = {
        .indata = jpeg,
        .indata_size = jpeg_len,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = JPEG_IMAGE_SCALE_0,
    };
    esp_jpeg_image_output_t source_info = {0};
    esp_err_t err = esp_jpeg_get_image_info(&info_config, &source_info);
    if (err != ESP_OK) {
        log_mjpeg_drop("MJPEG header is not a baseline JPEG", jpeg_len);
        log_jpeg_diagnostic(jpeg, jpeg_len);
        return false;
    }

    esp_jpeg_image_scale_t scale;
    if (!choose_scale(source_info.width, source_info.height,
                      max_width, max_height, &scale)) {
        ESP_LOGW(TAG, "Frame %ux%u is too large for the decoded frame buffer",
                 (unsigned)source_info.width, (unsigned)source_info.height);
        return false;
    }

    esp_jpeg_image_cfg_t decode_config = {
        .indata = jpeg,
        .indata_size = jpeg_len,
        .outbuf = output_buffer,
        .outbuf_size = output_size,
        .out_format = JPEG_IMAGE_FORMAT_RGB565,
        .out_scale = scale,
        .flags = {
            .swap_color_bytes = true,
        },
        .advanced = {
            .working_buffer = working_buffer,
            .working_buffer_size = CAMERA_DISPLAY_JPEG_WORK_BYTES,
        },
    };
    esp_jpeg_image_output_t output = {0};
    err = esp_jpeg_decode(&decode_config, &output);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MJPEG decode failed: %s", esp_err_to_name(err));
        return false;
    }
    if (output.width == 0 || output.height == 0 ||
        output.width > max_width || output.height > max_height ||
        output.output_len > output_size) {
        ESP_LOGW(TAG, "Unexpected decoded size %ux%u (%u bytes)",
                 (unsigned)output.width, (unsigned)output.height,
                 (unsigned)output.output_len);
        return false;
    }
    *output_width = output.width;
    *output_height = output.height;
    if (decode_us != NULL) {
        *decode_us = (uint32_t)(esp_timer_get_time() - start_us);
    }
    return true;
}

static void release_jpeg_slot(uint8_t slot)
{
    if (xQueueSend(s_display.free_slots, &slot, 0) != pdTRUE) {
        ESP_LOGE(TAG, "JPEG frame pool corruption (slot=%u)", (unsigned)slot);
    }
}

static bool acquire_control_buffer(uint8_t *buffer)
{
    if (xQueueReceive(s_display.free_control_buffers, buffer, 0) == pdTRUE) {
        return true;
    }

    /* The control queue is latest-only too. Recycle a frame not yet consumed. */
    control_frame_ref_t stale;
    if (xQueueReceive(s_display.ready_control_frames, &stale, 0) == pdTRUE) {
        *buffer = stale.buffer;
        ++s_display.control_dropped_frames;
        ++s_display.frames_dropped;
        return true;
    }
    return false;
}

static bool enqueue_latest_control(const control_frame_ref_t *frame)
{
    if (xQueueSend(s_display.ready_control_frames, frame, 0) == pdTRUE) {
        return true;
    }

    control_frame_ref_t stale;
    if (xQueueReceive(s_display.ready_control_frames, &stale, 0) == pdTRUE) {
        if (xQueueSend(s_display.free_control_buffers, &stale.buffer, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Control frame pool corruption");
        }
        ++s_display.control_dropped_frames;
        ++s_display.frames_dropped;
    }
    if (xQueueSend(s_display.ready_control_frames, frame, 0) == pdTRUE) {
        return true;
    }
    if (xQueueSend(s_display.free_control_buffers, &frame->buffer, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Control frame pool corruption");
    }
    ++s_display.control_dropped_frames;
    ++s_display.frames_dropped;
    return false;
}

#if 0 /* Image preview is intentionally disabled; TFT is a status page. */
static bool acquire_preview_buffer(uint8_t *buffer)
{
    if (xQueueReceive(s_display.free_preview_buffers, buffer, 0) == pdTRUE) {
        return true;
    }

    /* If the low-priority TFT task has not consumed the previous frame, reuse
     * that stale buffer for the newest preview. */
    preview_frame_ref_t stale;
    if (xQueueReceive(s_display.ready_preview_frames, &stale, 0) == pdTRUE) {
        *buffer = stale.buffer;
        ++s_display.preview_dropped_frames;
        ++s_display.frames_dropped;
        return true;
    }
    return false;
}

static bool enqueue_latest_preview(const preview_frame_ref_t *frame)
{
    if (xQueueSend(s_display.ready_preview_frames, frame, 0) == pdTRUE) {
        return true;
    }
    if (xQueueSend(s_display.free_preview_buffers, &frame->buffer, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Preview buffer pool corruption");
    }
    ++s_display.preview_dropped_frames;
    ++s_display.frames_dropped;
    return false;
}
#endif

static void camera_control_task(void *arg)
{
    (void)arg;
    control_frame_ref_t frame;
    while (true) {
        if (xQueueReceive(s_display.ready_control_frames, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        const int64_t now = esp_timer_get_time();
        const int64_t age = now >= frame.capture_us ? now - frame.capture_us : 0;
        s_display.last_control_age_us = age > UINT32_MAX ? UINT32_MAX : (uint32_t)age;
        s_display.last_control_sequence = frame.sequence;
        if (s_display.frame_callback != NULL) {
            ++s_display.control_frames;
            s_display.frame_callback(s_display.control_rgb565[frame.buffer],
                                     frame.width, frame.height, frame.threshold,
                                     false,
                                     s_display.frame_callback_ctx);
        }
#if 0 /* Image preview is intentionally disabled; TFT is a status page. */
        /* Copy only at the low preview cadence. SPI and preview callbacks run
         * in a separate low-priority task, so the control task never blocks
         * on TFT transfer. */
        const int64_t preview_now = esp_timer_get_time();
        const bool preview_due = s_display.tft_ready &&
                                 (s_display.last_preview_us == 0 ||
                                  preview_now - s_display.last_preview_us >=
                                      CAMERA_TFT_REFRESH_US);
        if (preview_due) {
            /* Reserve this cadence even if both preview buffers are busy; a
             * slow SPI transfer must not make every control frame retry. */
            s_display.last_preview_us = preview_now;
            uint8_t preview_buffer = 0;
            if (acquire_preview_buffer(&preview_buffer)) {
                const size_t bytes = (size_t)frame.width * frame.height * 2U;
                memcpy(s_display.preview_rgb565[preview_buffer],
                       s_display.control_rgb565[frame.buffer], bytes);
                const preview_frame_ref_t preview = {
                    .buffer = preview_buffer,
                    .width = frame.width,
                    .height = frame.height,
                    .threshold = frame.threshold,
                    .sequence = frame.sequence,
                    .capture_us = frame.capture_us,
                };
                (void)enqueue_latest_preview(&preview);
            } else {
                ++s_display.preview_dropped_frames;
                ++s_display.frames_dropped;
            }
        }
#endif
        if (xQueueSend(s_display.free_control_buffers, &frame.buffer, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Control frame pool corruption after callback");
        }
    }
}

#if 0 /* Image preview is intentionally disabled; TFT is a status page. */
static void camera_preview_task(void *arg)
{
    (void)arg;
    preview_frame_ref_t frame;
    while (true) {
        if (xQueueReceive(s_display.ready_preview_frames, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        ++s_display.preview_frames;
        if (s_display.preview_callback != NULL) {
            s_display.preview_callback(s_display.preview_rgb565[frame.buffer],
                                        frame.width, frame.height, frame.threshold,
                                        frame.sequence, frame.capture_us,
                                        s_display.preview_callback_ctx);
        }
        const int64_t tft_start_us = esp_timer_get_time();
        if (!tft_st7735_draw_rgb565(s_display.preview_rgb565[frame.buffer],
                                    frame.width, frame.height)) {
            ESP_LOGW(TAG, "TFT control-frame draw failed");
        } else {
            s_display.last_tft_us =
                (uint32_t)(esp_timer_get_time() - tft_start_us);
        }
        if (xQueueSend(s_display.free_preview_buffers, &frame.buffer, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Preview buffer pool corruption after draw");
        }
    }
}
#endif

#if CONFIG_EXAMPLE_ENABLE_TFT_PREVIEW
static const char *status_state_name(camera_display_status_state_t state)
{
    switch (state) {
    case CAMERA_DISPLAY_STATUS_CORNER:
        return "CORNER";
    case CAMERA_DISPLAY_STATUS_LOST:
        return "LOST";
    case CAMERA_DISPLAY_STATUS_ALIGN:
        return "ALIGN";
    case CAMERA_DISPLAY_STATUS_BRAKE:
        return "BRAKE";
    case CAMERA_DISPLAY_STATUS_AVOID_LEFT:
        return "AVOID_L";
    case CAMERA_DISPLAY_STATUS_AVOID_FORWARD:
        return "AVOID_F";
    case CAMERA_DISPLAY_STATUS_AVOID_RIGHT:
        return "AVOID_R";
    case CAMERA_DISPLAY_STATUS_NORMAL:
    default:
        return "NORMAL";
    }
}

static void camera_tft_status_task(void *arg)
{
    (void)arg;
    camera_display_status_t cached = {
        .state = CAMERA_DISPLAY_STATUS_LOST,
        .ultrasonic_cm = -1,
    };
    camera_display_pipeline_stats_t previous = {0};
    int64_t previous_us = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(CAMERA_TFT_STATUS_REFRESH_MS));
        if (!s_display.started || !s_display.tft_ready) {
            continue;
        }

        camera_display_status_t sampled = cached;
        if (s_display.status_callback != NULL &&
            s_display.status_callback(&sampled, s_display.status_callback_ctx)) {
            cached = sampled;
        }

        camera_display_pipeline_stats_t stats = {0};
        camera_display_get_pipeline_stats(&stats);
        const int64_t now = esp_timer_get_time();
        uint32_t camera_fps = 0;
        uint32_t control_fps = 0;
        if (previous_us != 0 && now > previous_us) {
            const uint64_t elapsed = (uint64_t)(now - previous_us);
            camera_fps = (uint32_t)(((uint64_t)(stats.camera_frames -
                                                previous.camera_frames) * 1000000U) /
                                   elapsed);
            control_fps = (uint32_t)(((uint64_t)(stats.control_frames -
                                                previous.control_frames) * 1000000U) /
                                   elapsed);
        }
        previous = stats;
        previous_us = now;

        char lines[8][32];
        const char *line_ptrs[8] = {
            lines[0], lines[1], lines[2], lines[3],
            lines[4], lines[5], lines[6], lines[7],
        };
        (void)snprintf(lines[0], sizeof(lines[0]), "STATE:%s",
                       status_state_name(cached.state));
        (void)snprintf(lines[1], sizeof(lines[1]), "ARM:%d STBY:%d",
                       cached.armed ? 1 : 0, cached.stby ? 1 : 0);
        (void)snprintf(lines[2], sizeof(lines[2]), "M A:%d B:%d D:%d",
                       cached.motor_a, cached.motor_b, cached.motor_d);
        (void)snprintf(lines[3], sizeof(lines[3]), "LAT:%d HEAD:%d",
                       cached.lateral_error, cached.heading_error);
        (void)snprintf(lines[4], sizeof(lines[4]), "TURN:%d",
                       cached.turn_command);
        (void)snprintf(lines[5], sizeof(lines[5]), "CAM:%u CTRL:%u",
                       (unsigned)camera_fps, (unsigned)control_fps);
        (void)snprintf(lines[6], sizeof(lines[6]), "DROP:%u CD:%u",
                       (unsigned)stats.frames_dropped,
                       (unsigned)stats.control_dropped_frames);
        if (cached.ultrasonic_cm < 0) {
            (void)snprintf(lines[7], sizeof(lines[7]), "US:--");
        } else {
            (void)snprintf(lines[7], sizeof(lines[7]), "US:%d CM",
                           cached.ultrasonic_cm);
        }

        const int64_t draw_start_us = esp_timer_get_time();
        if (!tft_st7735_draw_text_lines(line_ptrs, 8, 0xffff, 0x0000)) {
            ESP_LOGW(TAG, "TFT status draw failed");
        } else {
            s_display.last_tft_us =
                (uint32_t)(esp_timer_get_time() - draw_start_us);
        }
    }
}
#endif

static void camera_decode_task(void *arg)
{
    (void)arg;
    jpeg_frame_ref_t frame;
    uint8_t frames_since_idle_yield = 0;
    while (true) {
        if (xQueueReceive(s_display.ready_slots, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint8_t control_buffer = 0;
        uint16_t width = 0;
        uint16_t height = 0;
        uint32_t decode_us = 0;
        bool control_buffer_acquired = false;
        bool control_decoded = false;
        if (acquire_control_buffer(&control_buffer)) {
            control_buffer_acquired = true;
            control_decoded = decode_jpeg_to_buffer(
                s_display.jpeg_slots[frame.slot], frame.jpeg_len,
                s_display.control_rgb565[control_buffer], CAMERA_CONTROL_RGB565_BYTES,
                CAMERA_CONTROL_MAX_WIDTH, CAMERA_CONTROL_MAX_HEIGHT,
                s_display.control_jpeg_work, &width, &height, &decode_us);
            s_display.last_control_decode_us = decode_us;
        } else {
            ++s_display.control_dropped_frames;
            ++s_display.frames_dropped;
        }

        if (control_decoded) {
            const int64_t threshold_start_us = esp_timer_get_time();
            const uint8_t threshold_candidate =
                calculate_binary_threshold(s_display.control_rgb565[control_buffer],
                                           width, height);
            const uint8_t source_threshold = filter_binary_threshold(threshold_candidate);
            s_display.last_threshold_us =
                (uint32_t)(esp_timer_get_time() - threshold_start_us);
            s_display.binary_threshold = source_threshold;
            control_frame_ref_t control = {
                .buffer = control_buffer,
                .width = width,
                .height = height,
                .threshold = source_threshold,
                .sequence = frame.sequence,
                .capture_us = frame.capture_us,
            };
            if (enqueue_latest_control(&control)) {
                ++s_display.processed_frames;
            }
        } else if (control_buffer_acquired) {
            if (xQueueSend(s_display.free_control_buffers, &control_buffer, 0) != pdTRUE) {
                ESP_LOGE(TAG, "Control frame pool corruption after decode failure");
            }
        }

        release_jpeg_slot(frame.slot);

        /* One tick is 10 ms in this build. Yield periodically so CPU0 idle can
         * feed the watchdog without adding that delay to every decoded frame. */
        if (++frames_since_idle_yield >= CAMERA_DECODE_IDLE_YIELD_FRAMES) {
            frames_since_idle_yield = 0;
            vTaskDelay(1);
        }
    }
}

static void release_allocations(void)
{
    for (size_t i = 0; i < CAMERA_DISPLAY_SLOT_COUNT; ++i) {
        heap_caps_free(s_display.jpeg_slots[i]);
        s_display.jpeg_slots[i] = NULL;
    }
    for (size_t i = 0; i < CAMERA_CONTROL_BUFFER_COUNT; ++i) {
        heap_caps_free(s_display.control_rgb565[i]);
        s_display.control_rgb565[i] = NULL;
    }
#if 0 /* Image preview buffers are disabled; no display framebuffer is needed. */
    for (size_t i = 0; i < CAMERA_PREVIEW_BUFFER_COUNT; ++i) {
        heap_caps_free(s_display.preview_rgb565[i]);
        s_display.preview_rgb565[i] = NULL;
    }
#endif
    heap_caps_free(s_display.control_jpeg_work);
    s_display.control_jpeg_work = NULL;
    if (s_display.free_slots != NULL) {
        vQueueDelete(s_display.free_slots);
        s_display.free_slots = NULL;
    }
    if (s_display.ready_slots != NULL) {
        vQueueDelete(s_display.ready_slots);
        s_display.ready_slots = NULL;
    }
    if (s_display.free_control_buffers != NULL) {
        vQueueDelete(s_display.free_control_buffers);
        s_display.free_control_buffers = NULL;
    }
    if (s_display.ready_control_frames != NULL) {
        vQueueDelete(s_display.ready_control_frames);
        s_display.ready_control_frames = NULL;
    }
#if 0 /* Image preview queues are disabled; TFT uses the status task only. */
    if (s_display.free_preview_buffers != NULL) {
        vQueueDelete(s_display.free_preview_buffers);
        s_display.free_preview_buffers = NULL;
    }
    if (s_display.ready_preview_frames != NULL) {
        vQueueDelete(s_display.ready_preview_frames);
        s_display.ready_preview_frames = NULL;
    }
#endif
}

esp_err_t camera_display_start(void)
{
    if (s_display.started) {
        return ESP_OK;
    }
    s_display.camera_frames = 0;
    s_display.processed_frames = 0;
    s_display.control_frames = 0;
    s_display.preview_frames = 0;
    s_display.frames_dropped = 0;
    s_display.control_dropped_frames = 0;
    s_display.preview_dropped_frames = 0;
    s_display.binary_threshold = 0;
    s_display.threshold_initialized = false;
    s_display.threshold_filtered = 0;
    s_display.last_control_decode_us = 0;
    s_display.last_preview_decode_us = 0;
    s_display.last_threshold_us = 0;
    s_display.last_tft_us = 0;
    s_display.last_control_age_us = 0;
    s_display.last_control_sequence = 0;
    s_display.next_sequence = 0;
    s_mjpeg_drop_log_count = 0;
#if CONFIG_EXAMPLE_ENABLE_TFT_PREVIEW
    s_display.tft_ready = tft_st7735_init();
    if (!s_display.tft_ready) {
        ESP_LOGW(TAG, "ST7735 unavailable; control pipeline remains active");
    }
#else
    s_display.tft_ready = false;
#endif

    s_display.free_slots = xQueueCreate(CAMERA_DISPLAY_SLOT_COUNT, sizeof(uint8_t));
    s_display.ready_slots = xQueueCreate(1, sizeof(jpeg_frame_ref_t));
    s_display.free_control_buffers =
        xQueueCreate(CAMERA_CONTROL_BUFFER_COUNT, sizeof(uint8_t));
    s_display.ready_control_frames = xQueueCreate(1, sizeof(control_frame_ref_t));
#if 0 /* Image preview queues are disabled; TFT uses the status task only. */
    s_display.free_preview_buffers =
        xQueueCreate(CAMERA_PREVIEW_BUFFER_COUNT, sizeof(uint8_t));
    s_display.ready_preview_frames = xQueueCreate(1, sizeof(preview_frame_ref_t));
#endif
    if (s_display.free_slots == NULL || s_display.ready_slots == NULL ||
        s_display.free_control_buffers == NULL ||
        s_display.ready_control_frames == NULL
#if 0 /* Image preview queues are disabled; TFT uses the status task only. */
        || s_display.free_preview_buffers == NULL ||
        s_display.ready_preview_frames == NULL
#endif
        ) {
        ESP_LOGE(TAG, "Could not allocate camera pipeline queues");
        release_allocations();
        return ESP_ERR_NO_MEM;
    }

    for (uint8_t slot = 0; slot < CAMERA_DISPLAY_SLOT_COUNT; ++slot) {
        s_display.jpeg_slots[slot] = heap_caps_malloc(
            CAMERA_DISPLAY_MAX_JPEG_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_display.jpeg_slots[slot] == NULL ||
            xQueueSend(s_display.free_slots, &slot, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Could not allocate JPEG frame buffer %u", slot);
            release_allocations();
            return ESP_ERR_NO_MEM;
        }
    }
    for (uint8_t buffer = 0; buffer < CAMERA_CONTROL_BUFFER_COUNT; ++buffer) {
        /* The detector reads this buffer immediately after decode.  Keeping
         * it in internal RAM avoids a PSRAM/cache round trip on every pixel;
         * retain a PSRAM fallback for boards with a smaller DRAM heap. */
        s_display.control_rgb565[buffer] = heap_caps_malloc(
            CAMERA_CONTROL_RGB565_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (s_display.control_rgb565[buffer] == NULL) {
            s_display.control_rgb565[buffer] = heap_caps_malloc(
                CAMERA_CONTROL_RGB565_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (s_display.control_rgb565[buffer] == NULL ||
            xQueueSend(s_display.free_control_buffers, &buffer, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Could not allocate control RGB565 buffer %u", buffer);
            release_allocations();
            return ESP_ERR_NO_MEM;
        }
    }
#if 0 /* Image preview buffers are disabled; TFT uses the status task only. */
    for (uint8_t buffer = 0; buffer < CAMERA_PREVIEW_BUFFER_COUNT; ++buffer) {
        s_display.preview_rgb565[buffer] = heap_caps_malloc(
            CAMERA_PREVIEW_RGB565_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (s_display.preview_rgb565[buffer] == NULL) {
            s_display.preview_rgb565[buffer] = heap_caps_malloc(
                CAMERA_PREVIEW_RGB565_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        }
        if (s_display.preview_rgb565[buffer] == NULL ||
            xQueueSend(s_display.free_preview_buffers, &buffer, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Could not allocate preview RGB565 buffer %u", buffer);
            release_allocations();
            return ESP_ERR_NO_MEM;
        }
    }
#endif
    s_display.control_jpeg_work = heap_caps_malloc(
        CAMERA_DISPLAY_JPEG_WORK_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_display.control_jpeg_work == NULL) {
        ESP_LOGE(TAG, "Could not allocate camera decode buffers");
        release_allocations();
        return ESP_ERR_NO_MEM;
    }
    /* USB reception runs on core 0. Keep the expensive JPEG decode on core 1
     * and below the short control callback so a stale frame cannot block the
     * latest control decision. */
    if (xTaskCreatePinnedToCore(camera_decode_task, "camera_decode", 6144,
                                NULL, 4, NULL, 1) != pdPASS ||
        xTaskCreatePinnedToCore(camera_control_task, "camera_control", 4096,
                                NULL, 6, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "Could not create camera control tasks");
        return ESP_ERR_NO_MEM;
    }
#if 0 /* Image preview task is disabled; TFT uses the status task only. */
    if (s_display.tft_ready &&
        xTaskCreatePinnedToCore(camera_preview_task, "tft_preview", 3072,
                                NULL, 1, NULL, 1) != pdPASS) {
        ESP_LOGW(TAG, "Could not create TFT preview task; control remains active");
        s_display.tft_ready = false;
    }
#endif
    s_display.started = true;
#if CONFIG_EXAMPLE_ENABLE_TFT_PREVIEW
    if (s_display.tft_ready &&
        xTaskCreatePinnedToCore(camera_tft_status_task, "tft_status", 3072,
                                NULL, 1, NULL, 1) != pdPASS) {
        ESP_LOGW(TAG, "Could not create TFT status task; control remains active");
        s_display.tft_ready = false;
    }
#endif
    ESP_LOGI(TAG, "Camera pipeline started: control <=%ux%u, TFT=%s",
             CAMERA_CONTROL_MAX_WIDTH, CAMERA_CONTROL_MAX_HEIGHT,
#if CONFIG_EXAMPLE_ENABLE_TFT_PREVIEW
             s_display.tft_ready ? "low-priority status task (2Hz)" : "unavailable"
#else
             "disabled"
#endif
             );
    return ESP_OK;
}

void camera_display_set_frame_callback(camera_display_frame_callback_t callback,
                                       void *user_ctx)
{
    s_display.frame_callback = callback;
    s_display.frame_callback_ctx = user_ctx;
}

void camera_display_set_status_callback(camera_display_status_callback_t callback,
                                         void *user_ctx)
{
    s_display.status_callback = callback;
    s_display.status_callback_ctx = user_ctx;
}

void camera_display_set_preview_callback(camera_display_preview_callback_t callback,
                                         void *user_ctx)
{
    s_display.preview_callback = callback;
    s_display.preview_callback_ctx = user_ctx;
}

bool camera_display_submit(const uint8_t *jpeg, size_t jpeg_len)
{
    if (!s_display.started || jpeg == NULL || jpeg_len == 0) {
        return false;
    }
    ++s_display.camera_frames;
    if (jpeg_len > CAMERA_DISPLAY_MAX_JPEG_BYTES) {
        ++s_display.frames_dropped;
        if (s_display.frames_dropped % 100U == 1U) {
            ESP_LOGW(TAG, "Dropping %u-byte JPEG; input limit is %u bytes",
                     (unsigned)jpeg_len, CAMERA_DISPLAY_MAX_JPEG_BYTES);
        }
        return false;
    }

    uint8_t slot;
    if (xQueueReceive(s_display.free_slots, &slot, 0) != pdTRUE) {
        jpeg_frame_ref_t stale;
        if (xQueueReceive(s_display.ready_slots, &stale, 0) != pdTRUE) {
            ++s_display.frames_dropped;
            return false;
        }
        slot = stale.slot;
        ++s_display.frames_dropped;
    }

    const int64_t capture_us = esp_timer_get_time();
    memcpy(s_display.jpeg_slots[slot], jpeg, jpeg_len);
    jpeg_frame_ref_t frame = {
        .slot = slot,
        .jpeg_len = jpeg_len,
        .sequence = ++s_display.next_sequence,
        .capture_us = capture_us,
    };
    if (xQueueSend(s_display.ready_slots, &frame, 0) == pdTRUE) {
        return true;
    }

    jpeg_frame_ref_t stale;
    if (xQueueReceive(s_display.ready_slots, &stale, 0) == pdTRUE) {
        release_jpeg_slot(stale.slot);
        ++s_display.frames_dropped;
    }
    if (xQueueSend(s_display.ready_slots, &frame, 0) == pdTRUE) {
        return true;
    }
    release_jpeg_slot(slot);
    ++s_display.frames_dropped;
    return false;
}

void camera_display_get_pipeline_stats(camera_display_pipeline_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    stats->camera_frames = s_display.camera_frames;
    stats->processed_frames = s_display.processed_frames;
    stats->control_frames = s_display.control_frames;
    stats->preview_frames = s_display.preview_frames;
    stats->frames_dropped = s_display.frames_dropped;
    stats->control_dropped_frames = s_display.control_dropped_frames;
    stats->preview_dropped_frames = s_display.preview_dropped_frames;
    stats->control_decode_us = s_display.last_control_decode_us;
    stats->preview_decode_us = s_display.last_preview_decode_us;
    stats->threshold_us = s_display.last_threshold_us;
    stats->tft_us = s_display.last_tft_us;
    stats->last_control_age_us = s_display.last_control_age_us;
    stats->last_control_sequence = s_display.last_control_sequence;
}

void camera_display_get_counters(uint32_t *camera_frames,
                                 uint32_t *processed_frames,
                                 uint32_t *dropped_frames)
{
    camera_display_pipeline_stats_t stats;
    camera_display_get_pipeline_stats(&stats);
    if (camera_frames != NULL) {
        *camera_frames = stats.camera_frames;
    }
    if (processed_frames != NULL) {
        *processed_frames = stats.processed_frames;
    }
    if (dropped_frames != NULL) {
        *dropped_frames = stats.frames_dropped;
    }
}

void camera_display_get_timing(uint32_t *decode_us,
                               uint32_t *threshold_us,
                               uint32_t *tft_us)
{
    camera_display_pipeline_stats_t stats;
    camera_display_get_pipeline_stats(&stats);
    if (decode_us != NULL) {
        *decode_us = stats.control_decode_us;
    }
    if (threshold_us != NULL) {
        *threshold_us = stats.threshold_us;
    }
    if (tft_us != NULL) {
        *tft_us = stats.tft_us;
    }
}
