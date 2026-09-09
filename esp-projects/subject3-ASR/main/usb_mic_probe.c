#include "sdkconfig.h"
#include "board_pins.h"
#include "motor.h"
#include "pcm_level.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_stream.h"
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if CONFIG_SUBJECT3_USB_MIC_PROBE
static const char *TAG = "mic_probe";
static portMUX_TYPE stats_lock = portMUX_INITIALIZER_UNLOCKED;
static struct {
    pcm_level_t level;
    uint32_t bytes, frames, rate;
    uint16_t bits;
} stats;

/* usb_stream owns frame->data. Consume it here and never retain the pointer.
 * Do not log, block, or run a speech model in this callback. */
static void mic_frame(mic_frame_t *frame, void *arg)
{
    (void)arg;
    if (!frame || !frame->data || !frame->data_bytes) return;
    pcm_level_t level = {0};
    if (frame->bit_resolution == 16) {
        level = pcm16le_measure(frame->data, frame->data_bytes);
    }
    portENTER_CRITICAL(&stats_lock);
    stats.bytes += frame->data_bytes;
    ++stats.frames;
    stats.rate = frame->samples_frequence;
    stats.bits = frame->bit_resolution;
    stats.level.samples += level.samples;
    stats.level.sum_squares += level.sum_squares;
    if (level.peak > stats.level.peak) stats.level.peak = level.peak;
    portEXIT_CRITICAL(&stats_lock);
}

static void stream_state(usb_stream_state_t state, void *arg)
{
    (void)arg;
    portENTER_CRITICAL(&stats_lock);
    memset(&stats, 0, sizeof(stats));
    portEXIT_CRITICAL(&stats_lock);
    if (state != STREAM_CONNECTED) {
        ESP_LOGW(TAG, "USB disconnected; motors remain disabled");
        return;
    }
    size_t count = 0, current = 0;
    esp_err_t err = uac_frame_size_list_get(STREAM_UAC_MIC, NULL, &count, &current);
    if (err != ESP_OK || count == 0 || count > 255) {
        ESP_LOGW(TAG, "USB connected but no matching UAC mic format (%s). Check descriptors and requested format; video alone does not prove audio support.", esp_err_to_name(err));
        return;
    }
    /* Called from the driver's state callback, as in subject2-camera. */
    uac_frame_size_t *formats = calloc(count, sizeof(*formats));
    if (!formats) {
        ESP_LOGE(TAG, "Cannot allocate mic format list");
        return;
    }
    err = uac_frame_size_list_get(STREAM_UAC_MIC, formats, &count, &current);
    if (err == ESP_OK) {
        for (size_t i = 0; i < count; ++i) {
            ESP_LOGI(TAG, "format[%u]%s: channels=%u bits=%u rate=%" PRIu32 " range=%" PRIu32 "..%" PRIu32,
                     (unsigned)i, i == current ? " SELECTED" : "",
                     formats[i].ch_num, formats[i].bit_resolution,
                     formats[i].samples_frequence,
                     formats[i].samples_frequence_min, formats[i].samples_frequence_max);
        }
    }
    free(formats);
}

void subject3_usb_mic_probe_main(void)
{
    const gesture_pin_t pins[] = {
        {"PWMA", PIN_PWMA, true}, {"AIN1", PIN_AIN1, true}, {"AIN2", PIN_AIN2, true},
        {"PWMB", PIN_PWMB, true}, {"BIN1", PIN_BIN1, true}, {"BIN2", PIN_BIN2, true},
        {"PWMD", PIN_PWMD, true}, {"DIN1", PIN_DIN1, true}, {"DIN2", PIN_DIN2, true},
        {"STBY", PIN_MOTOR_STBY, true}, {"USB D-", 19, false}, {"USB D+", 20, false},
    };
    ESP_ERROR_CHECK(gesture_board_validate(pins, sizeof(pins) / sizeof(pins[0]), ALLOW_STRAPPING_PINS));
    ESP_ERROR_CHECK(motor_init());
    motor_stop();
    /* No BLE, IMU, LCD, video, or speaker task is started in this mode. */
    if ((CONFIG_SUBJECT3_MIC_RATE != 0 && CONFIG_SUBJECT3_MIC_RATE < 1000) ||
        (CONFIG_SUBJECT3_MIC_BITS != 0 && CONFIG_SUBJECT3_MIC_BITS < 8)) {
        ESP_LOGE(TAG, "Invalid format: rate must be 0 or 1000..48000; bits must be 0 or 8..24");
        return;
    }
    const uac_config_t cfg = {
        .mic_ch_num = CONFIG_SUBJECT3_MIC_CHANNELS,
        .mic_bit_resolution = CONFIG_SUBJECT3_MIC_BITS ? CONFIG_SUBJECT3_MIC_BITS : UAC_BITS_ANY,
        .mic_samples_frequence = CONFIG_SUBJECT3_MIC_RATE ? CONFIG_SUBJECT3_MIC_RATE : UAC_FREQUENCY_ANY,
        .mic_buf_size = 0,
        .mic_cb = mic_frame,
    };
    ESP_ERROR_CHECK(uac_streaming_config(&cfg));
    ESP_ERROR_CHECK(usb_streaming_state_register(stream_state, NULL));
    ESP_ERROR_CHECK(usb_streaming_start());
    ESP_LOGI(TAG, "Waiting for USB microphone: D-=GPIO19 D+=GPIO20, external 5V, UART0 log. STBY stays LOW.");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        portENTER_CRITICAL(&stats_lock);
        const pcm_level_t level = stats.level;
        const uint32_t bytes = stats.bytes, frames = stats.frames, rate = stats.rate;
        const unsigned bits = stats.bits;
        memset(&stats, 0, sizeof(stats));
        portEXIT_CRITICAL(&stats_lock);
        if (!frames) {
            ESP_LOGW(TAG, "No mic frames in last interval; check 5V/D-/D+, UAC1 support and format selection");
        } else if (bits == 16 && level.samples) {
            double rms = sqrt((double)level.sum_squares / (double)level.samples);
            ESP_LOGI(TAG, "frames=%" PRIu32 " bytes=%" PRIu32 " rate=%" PRIu32 " bits=%u peak=%" PRIu32 " rms=%.1f (PCM units; not recognition)",
                     frames, bytes, rate, bits, level.peak, rms);
        } else {
            ESP_LOGI(TAG, "frames=%" PRIu32 " bytes=%" PRIu32 " rate=%" PRIu32 " bits=%u; level meter requires 16-bit PCM",
                     frames, bytes, rate, bits);
        }
    }
}
#endif
