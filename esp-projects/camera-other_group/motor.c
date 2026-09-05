#include "motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// ================= 最新引脚定义 =================
// A轮
#define PWMA_PIN 4  //[cite: 3]
#define AIN1_PIN 6  //[cite: 3]
#define AIN2_PIN 5  //[cite: 3]

// B轮
#define PWMB_PIN 9  //[cite: 3]
#define BIN1_PIN 11 //[cite: 3]
#define BIN2_PIN 10 //[cite: 3]

// D轮
#define PWMD_PIN 40 //[cite: 3]
#define DIN1_PIN 42 //[cite: 3]
#define DIN2_PIN 41 //[cite: 3]

// ================= 函数实现 =================
void motor_init(void) {
    // 1. 初始化方向控制引脚为输出模式
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<AIN1_PIN) | (1ULL<<AIN2_PIN) |
                        (1ULL<<BIN1_PIN) | (1ULL<<BIN2_PIN) |
                        (1ULL<<DIN1_PIN) | (1ULL<<DIN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    // 2. 初始化 PWM 定时器 (频率 5kHz, 13位分辨率)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // 3. 配置 A 轮 PWM 通道 (Channel 0)
    ledc_channel_config_t ch_conf_A = {
        .gpio_num = PWMA_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_conf_A);

    // 4. 配置 B 轮 PWM 通道 (Channel 1)
    ledc_channel_config_t ch_conf_B = {
        .gpio_num = PWMB_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_conf_B);

    // 5. 配置 D 轮 PWM 通道 (Channel 2)
    ledc_channel_config_t ch_conf_D = {
        .gpio_num = PWMD_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_conf_D);
}

// A 轮控制
void set_motor_A(int dir, uint32_t speed) {
    if (dir == 1) {
        gpio_set_level(AIN1_PIN, 1);
        gpio_set_level(AIN2_PIN, 0);
    } else if (dir == -1) {
        gpio_set_level(AIN1_PIN, 0);
        gpio_set_level(AIN2_PIN, 1);
    } else {
        gpio_set_level(AIN1_PIN, 0);
        gpio_set_level(AIN2_PIN, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// B 轮控制
void set_motor_B(int dir, uint32_t speed) {
    if (dir == 1) {
        gpio_set_level(BIN1_PIN, 1);
        gpio_set_level(BIN2_PIN, 0);
    } else if (dir == -1) {
        gpio_set_level(BIN1_PIN, 0);
        gpio_set_level(BIN2_PIN, 1);
    } else {
        gpio_set_level(BIN1_PIN, 0);
        gpio_set_level(BIN2_PIN, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// D 轮控制
void set_motor_D(int dir, uint32_t speed) {
    if (dir == 1) {
        gpio_set_level(DIN1_PIN, 1);
        gpio_set_level(DIN2_PIN, 0);
    } else if (dir == -1) {
        gpio_set_level(DIN1_PIN, 0);
        gpio_set_level(DIN2_PIN, 1);
    } else {
        gpio_set_level(DIN1_PIN, 0);
        gpio_set_level(DIN2_PIN, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}