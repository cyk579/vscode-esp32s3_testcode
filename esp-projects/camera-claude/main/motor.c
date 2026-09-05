#include "motor.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

/* 引脚全部搬到 board_pins.h，改线只改那一个文件。
 * 按轮子的几何角色映射，不是按脚号：A/D 是两个斜置驱动轮，B 是横移轮。 */
#define PWMA_PIN PIN_PWMA
#define AIN1_PIN PIN_AIN1
#define AIN2_PIN PIN_AIN2

#define PWMB_PIN PIN_PWMB
#define BIN1_PIN PIN_BIN1
#define BIN2_PIN PIN_BIN2

#define PWMD_PIN PIN_PWMD
#define DIN1_PIN PIN_DIN1
#define DIN2_PIN PIN_DIN2

// ================= 函数实现 =================
void motor_init(void) {
    // 1. 初始化方向控制引脚为输出模式
    //    本车多一个 STBY：TB6612 的总使能，不拉高三个轮子全都不转。
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<AIN1_PIN) | (1ULL<<AIN2_PIN) |
                        (1ULL<<BIN1_PIN) | (1ULL<<BIN2_PIN) |
                        (1ULL<<DIN1_PIN) | (1ULL<<DIN2_PIN) |
                        (1ULL<<PIN_MOTOR_STBY),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_MOTOR_STBY, 1);

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