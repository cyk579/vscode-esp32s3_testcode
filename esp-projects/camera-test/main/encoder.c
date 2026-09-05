#include "encoder.h"

#include <limits.h>
#include <stddef.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* The six pins are the wiring confirmed on the ESP32-S3 board.  Each wheel
 * uses A as the edge input and B as the level input for signed direction. */
#define ENCODER_A_PHASE_A GPIO_NUM_39
#define ENCODER_A_PHASE_B GPIO_NUM_40
#define ENCODER_B_PHASE_A GPIO_NUM_17
#define ENCODER_B_PHASE_B GPIO_NUM_3
#define ENCODER_D_PHASE_A GPIO_NUM_41
#define ENCODER_D_PHASE_B GPIO_NUM_42

#define ENCODER_PCNT_HIGH_LIMIT 32767
#define ENCODER_PCNT_LOW_LIMIT (-32768)
#define ENCODER_GLITCH_FILTER_NS 1000U
#define ENCODER_FILTER_OLD 3
#define ENCODER_FILTER_NEW 1

#ifndef CONFIG_EXAMPLE_ENCODER_SAMPLE_PERIOD_MS
#define CONFIG_EXAMPLE_ENCODER_SAMPLE_PERIOD_MS 10
#endif
#ifndef CONFIG_EXAMPLE_ENCODER_COUNTS_PER_REV
#define CONFIG_EXAMPLE_ENCODER_COUNTS_PER_REV 1
#endif
#ifndef CONFIG_EXAMPLE_ENCODER_A_INVERT
#define CONFIG_EXAMPLE_ENCODER_A_INVERT 0
#endif
#ifndef CONFIG_EXAMPLE_ENCODER_B_INVERT
#define CONFIG_EXAMPLE_ENCODER_B_INVERT 0
#endif
#ifndef CONFIG_EXAMPLE_ENCODER_D_INVERT
#define CONFIG_EXAMPLE_ENCODER_D_INVERT 0
#endif

#if CONFIG_EXAMPLE_ENCODER_SAMPLE_PERIOD_MS < 5
#define ENCODER_SAMPLE_PERIOD_MS 5U
#else
#define ENCODER_SAMPLE_PERIOD_MS ((uint32_t)CONFIG_EXAMPLE_ENCODER_SAMPLE_PERIOD_MS)
#endif

static const char *TAG = "encoder";
static const gpio_num_t s_phase_a[ENCODER_WHEEL_COUNT] = {
    ENCODER_A_PHASE_A, ENCODER_B_PHASE_A, ENCODER_D_PHASE_A,
};
static const gpio_num_t s_phase_b[ENCODER_WHEEL_COUNT] = {
    ENCODER_A_PHASE_B, ENCODER_B_PHASE_B, ENCODER_D_PHASE_B,
};
static const int s_direction_sign[ENCODER_WHEEL_COUNT] = {
    CONFIG_EXAMPLE_ENCODER_A_INVERT ? -1 : 1,
    CONFIG_EXAMPLE_ENCODER_B_INVERT ? -1 : 1,
    CONFIG_EXAMPLE_ENCODER_D_INVERT ? -1 : 1,
};

static pcnt_unit_handle_t s_units[ENCODER_WHEEL_COUNT];
static pcnt_channel_handle_t s_channels[ENCODER_WHEEL_COUNT];
static portMUX_TYPE s_snapshot_lock = portMUX_INITIALIZER_UNLOCKED;
static encoder_snapshot_t s_snapshot;
static bool s_initialized;
static bool s_task_created;
static bool s_have_sample;

static int32_t saturate_i64(int64_t value)
{
    if (value > INT32_MAX) {
        return INT32_MAX;
    }
    if (value < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static esp_err_t configure_encoder_gpio(gpio_num_t phase_a, gpio_num_t phase_b)
{
    const uint64_t mask = (UINT64_C(1) << (uint32_t)phase_a) |
                          (UINT64_C(1) << (uint32_t)phase_b);
    const gpio_config_t config = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

static void delete_units(void)
{
    for (int i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
        if (s_channels[i] != NULL) {
            (void)pcnt_del_channel(s_channels[i]);
            s_channels[i] = NULL;
        }
        if (s_units[i] != NULL) {
            (void)pcnt_unit_disable(s_units[i]);
            (void)pcnt_del_unit(s_units[i]);
            s_units[i] = NULL;
        }
    }
}

static void encoder_sample_task(void *arg)
{
    (void)arg;
    TickType_t wake_time = xTaskGetTickCount();
    int64_t previous_us = esp_timer_get_time();

    while (true) {
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(ENCODER_SAMPLE_PERIOD_MS));

        int32_t delta[ENCODER_WHEEL_COUNT] = {0};
        for (int i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
            int count = 0;
            if (pcnt_unit_get_count(s_units[i], &count) != ESP_OK) {
                count = 0;
            }
            (void)pcnt_unit_clear_count(s_units[i]);
            delta[i] = (int32_t)count * s_direction_sign[i];
        }

        const int64_t now_us = esp_timer_get_time();
        int64_t elapsed_us = now_us - previous_us;
        previous_us = now_us;
        if (elapsed_us < 1000) {
            elapsed_us = (int64_t)ENCODER_SAMPLE_PERIOD_MS * 1000;
        }

        int32_t rate[ENCODER_WHEEL_COUNT] = {0};
        for (int i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
            rate[i] = saturate_i64((int64_t)delta[i] * 1000000LL / elapsed_us);
        }

        portENTER_CRITICAL(&s_snapshot_lock);
        for (int i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
            const int32_t filtered = s_snapshot.rate_cps[i];
            s_snapshot.delta[i] = delta[i];
            s_snapshot.rate_cps[i] = s_have_sample ?
                (int32_t)(((int64_t)filtered * ENCODER_FILTER_OLD +
                           (int64_t)rate[i] * ENCODER_FILTER_NEW) /
                          (ENCODER_FILTER_OLD + ENCODER_FILTER_NEW)) :
                rate[i];
            s_snapshot.total[i] = saturate_i64(
                (int64_t)s_snapshot.total[i] + delta[i]);
        }
        s_snapshot.ready = true;
        portEXIT_CRITICAL(&s_snapshot_lock);
        s_have_sample = true;
    }
}

esp_err_t encoder_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    const pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
    };
    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = ENCODER_GLITCH_FILTER_NS,
    };

    for (int i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
        esp_err_t err = configure_encoder_gpio(s_phase_a[i], s_phase_b[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "encoder %d GPIO config failed: %s", i,
                     esp_err_to_name(err));
            delete_units();
            return err;
        }
        err = pcnt_new_unit(&unit_config, &s_units[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "encoder %d PCNT unit failed: %s", i,
                     esp_err_to_name(err));
            delete_units();
            return err;
        }
        const pcnt_chan_config_t channel_config = {
            .edge_gpio_num = s_phase_a[i],
            .level_gpio_num = s_phase_b[i],
        };
        err = pcnt_new_channel(s_units[i], &channel_config, &s_channels[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "encoder %d PCNT channel failed: %s", i,
                     esp_err_to_name(err));
            delete_units();
            return err;
        }
        err = pcnt_channel_set_edge_action(
            s_channels[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        if (err == ESP_OK) {
            err = pcnt_channel_set_level_action(
                s_channels[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        }
        if (err == ESP_OK) {
            err = pcnt_unit_set_glitch_filter(s_units[i], &filter_config);
        }
        if (err == ESP_OK) {
            err = pcnt_unit_enable(s_units[i]);
        }
        if (err == ESP_OK) {
            err = pcnt_unit_clear_count(s_units[i]);
        }
        if (err == ESP_OK) {
            err = pcnt_unit_start(s_units[i]);
        }
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "encoder %d PCNT start failed: %s", i,
                     esp_err_to_name(err));
            delete_units();
            return err;
        }
    }

    portENTER_CRITICAL(&s_snapshot_lock);
    s_snapshot = (encoder_snapshot_t){
        .ready = true,
        .sample_period_ms = ENCODER_SAMPLE_PERIOD_MS,
        .counts_per_revolution = CONFIG_EXAMPLE_ENCODER_COUNTS_PER_REV > 0 ?
                                  (uint32_t)CONFIG_EXAMPLE_ENCODER_COUNTS_PER_REV : 1U,
    };
    portEXIT_CRITICAL(&s_snapshot_lock);
    s_have_sample = false;
    s_initialized = true;

    if (!s_task_created) {
        if (xTaskCreate(encoder_sample_task, "encoder_sample", 2048,
                        NULL, 3, NULL) != pdPASS) {
            s_initialized = false;
            delete_units();
            ESP_LOGE(TAG, "encoder sampling task creation failed");
            return ESP_ERR_NO_MEM;
        }
        s_task_created = true;
    }

    ESP_LOGI(TAG,
             "encoders ready A[%d,%d] B[%d,%d] D[%d,%d] sample=%ums cpr=%u invert=%d/%d/%d",
             (int)s_phase_a[ENCODER_WHEEL_A], (int)s_phase_b[ENCODER_WHEEL_A],
             (int)s_phase_a[ENCODER_WHEEL_B], (int)s_phase_b[ENCODER_WHEEL_B],
             (int)s_phase_a[ENCODER_WHEEL_D], (int)s_phase_b[ENCODER_WHEEL_D],
             (unsigned)ENCODER_SAMPLE_PERIOD_MS,
             (unsigned)s_snapshot.counts_per_revolution,
             CONFIG_EXAMPLE_ENCODER_A_INVERT ? 1 : 0,
             CONFIG_EXAMPLE_ENCODER_B_INVERT ? 1 : 0,
             CONFIG_EXAMPLE_ENCODER_D_INVERT ? 1 : 0);
    return ESP_OK;
}

bool encoder_get_snapshot(encoder_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }
    portENTER_CRITICAL(&s_snapshot_lock);
    *snapshot = s_snapshot;
    const bool ready = s_initialized && s_snapshot.ready;
    portEXIT_CRITICAL(&s_snapshot_lock);
    return ready;
}

void encoder_reset_counts(void)
{
    if (!s_initialized) {
        return;
    }
    for (int i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
        (void)pcnt_unit_clear_count(s_units[i]);
    }
    portENTER_CRITICAL(&s_snapshot_lock);
    for (int i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
        s_snapshot.delta[i] = 0;
        s_snapshot.rate_cps[i] = 0;
        s_snapshot.total[i] = 0;
    }
    portEXIT_CRITICAL(&s_snapshot_lock);
    s_have_sample = false;
}
