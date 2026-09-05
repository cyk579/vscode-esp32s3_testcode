#include "encoder.h"
#include "board_pins.h"
#include "esp_log.h"

/* ============================================================
 * 本车没有接编码器，默认整个文件退化成空实现。
 *
 * 他们的编码器占用 7/15/12/13/2/1，本车这六个脚分别是 D 轮方向 ×2、
 * A 轮方向、TFT 时钟、俯仰舵机、水平舵机。PCNT 把这些脚配成输入后会
 * 和电机方向输出、屏幕时钟直接冲突。
 *
 * 没有编码器时速度闭环必须整段跳过，而不是让它读到 0：读 0 的话闭环
 * 认为每个轮子都堵转，会按各自目标值往上补功率，直道上两轮对称还好，
 * 转弯时 A/D 目标不同会落进不同增益档（0.3/0.6/1.2），把调好的差速
 * 非线性放大。所以 pid.c 里也做了对应的 #ifdef。
 *
 * 要启用：在 board_pins.h 里打开 USE_ENCODER 并填实际接线。
 * ============================================================ */

#ifdef USE_ENCODER

#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCNT_HIGH_LIMIT 10000
#define PCNT_LOW_LIMIT  -10000

// 引脚定义见 board_pins.h
#define E1A_PIN PIN_ENC_A1
#define E1B_PIN PIN_ENC_B1
#define E2A_PIN PIN_ENC_A2
#define E2B_PIN PIN_ENC_B2
#define E4A_PIN PIN_ENC_A4
#define E4B_PIN PIN_ENC_B4

// 后台采样周期（毫秒）
#define ENCODER_TASK_PERIOD_MS 10

static pcnt_unit_handle_t pcnt_units[3];
static pcnt_channel_handle_t pcnt_chans[3];

// 全局速度数据（由后台任务写入，其他任务只读）
static volatile float g_latest_speeds[3] = {0.0f, 0.0f, 0.0f};
static volatile float g_raw_speeds[3] = {0.0f, 0.0f, 0.0f};

// 速度平滑系数
#define SPEED_SMOOTH_ALPHA 0.7f

// 后台采样任务
static void encoder_sample_task(void *arg) {
    float raw[3];
    float smoothed[3] = {0.0f, 0.0f, 0.0f};

    while (1) {
        // 读取并清零计数器
        for (int i = 0; i < 3; i++) {
            int cur_count;
            pcnt_unit_get_count(pcnt_units[i], &cur_count);
            pcnt_unit_clear_count(pcnt_units[i]);
            raw[i] = (float)cur_count;
            g_raw_speeds[i] = (float)cur_count;
        }

        // 低通滤波平滑（每10ms采样一次）
        for (int i = 0; i < 3; i++) {
            smoothed[i] = SPEED_SMOOTH_ALPHA * smoothed[i] +
                         (1.0f - SPEED_SMOOTH_ALPHA) * raw[i];
            g_latest_speeds[i] = smoothed[i];
        }

        vTaskDelay(pdMS_TO_TICKS(ENCODER_TASK_PERIOD_MS));
    }
}

void encoder_init(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };

    int a_pins[] = {E1A_PIN, E2A_PIN, E4A_PIN};
    int b_pins[] = {E1B_PIN, E2B_PIN, E4B_PIN};

    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_units[i]));

        pcnt_chan_config_t chan_config = {
            .edge_gpio_num = a_pins[i],
            .level_gpio_num = b_pins[i],
        };
        ESP_ERROR_CHECK(pcnt_new_channel(pcnt_units[i], &chan_config, &pcnt_chans[i]));

        // 设置边缘和电平动作实现四倍频/双向计数
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chans[i], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chans[i], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        // 过滤抖动
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 1000,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_units[i], &filter_config));

        ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(pcnt_units[i]));
    }

    // 创建后台采样任务（优先级比PID高）
    xTaskCreate(encoder_sample_task, "enc_sample", 1024, NULL, 10, NULL);
}

// 获取最新平滑速度（只读，不影响后台采样）
void encoder_get_speeds(float *speed_a, float *speed_b, float *speed_d) {
    if (speed_a) *speed_a = g_latest_speeds[0];
    if (speed_b) *speed_b = g_latest_speeds[1];
    if (speed_d) *speed_d = g_latest_speeds[2];
}

// 获取原始速度（未平滑）
void encoder_get_raw_speeds(float *speed_a, float *speed_b, float *speed_d) {
    if (speed_a) *speed_a = g_raw_speeds[0];
    if (speed_b) *speed_b = g_raw_speeds[1];
    if (speed_d) *speed_d = g_raw_speeds[2];
}

#else /* 没有编码器：空实现，屏幕上三个速度会一直是 0 */

void encoder_init(void) {
    ESP_LOGW("ENCODER", "本车未接编码器，速度闭环已关闭（开环运行）");
}

void encoder_get_speeds(float *speed_a, float *speed_b, float *speed_d) {
    if (speed_a) *speed_a = 0.0f;
    if (speed_b) *speed_b = 0.0f;
    if (speed_d) *speed_d = 0.0f;
}

void encoder_get_raw_speeds(float *speed_a, float *speed_b, float *speed_d) {
    if (speed_a) *speed_a = 0.0f;
    if (speed_b) *speed_b = 0.0f;
    if (speed_d) *speed_d = 0.0f;
}

#endif /* USE_ENCODER */