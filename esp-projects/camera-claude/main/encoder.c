#include "encoder.h"
#include "board_pins.h"
#include "esp_log.h"

/* ============================================================
 * 三个电机编码器，PCNT 硬件计数。引脚见 board_pins.h。
 *
 * 脚号和原代码不同：他们用 7/15/12/13/2/1，这六脚在本车上已经是电机方向、
 * TFT 时钟和两个舵机，PCNT 配成输入会和这些输出直接顶牛，所以重新分配到
 * A 轮 39/40、B 轮 17/3、D 轮 41/42（见 board_pins.h，那里写了为什么只剩这六个脚
 * 可用，以及为什么 47/48 不能用 —— 它们是 1.8V 域）。
 *
 * 用 USE_ENCODER 开关控制。关掉时整个文件退化成返回 0 的空实现，同时
 * pid.c 的速度闭环也会整段跳过 —— 必须成对，不能只关一个：闭环读到恒 0
 * 会认为三个轮子全堵转，按各自目标值往上补功率，转弯时 A/D 目标不同会
 * 落进不同增益档（0.3/0.6/1.2），把调好的差速非线性放大。
 *
 * 没接编码器却打开这个开关，后果比关掉更糟：PCNT 读不到脉冲恒为 0，
 * 闭环每周期补 +500 一路顶到 PID_MAX_SPEED，车会以 44% 占空比冲出去。
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
    // 栈从原来的 1024 提到 2048：任务里有浮点运算和 PCNT 调用，1024 太紧。
    xTaskCreate(encoder_sample_task, "enc_sample", 2048, NULL, 10, NULL);
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