#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 四路红外，按车体从左到右排列：OUT4、OUT3、OUT2、OUT1。
   若左右方向装反，把这一行的四个引脚顺序颠倒即可。 */
static const gpio_num_t sensors[4] = {
    GPIO_NUM_1,  /* OUT4，车体最左 */
    GPIO_NUM_2,  /* OUT3 */
    GPIO_NUM_42, /* OUT2 */
    GPIO_NUM_41  /* OUT1，车体最右 */
};
static const int weights[4] = {-3, -1, 1, 3};
#define IR_ACTIVE_LEVEL 0 /* LQ_R4CHVB：黑线低电平 */

#define A_PWM GPIO_NUM_9
#define A_IN1 GPIO_NUM_12
#define A_IN2 GPIO_NUM_10
#define B_PWM GPIO_NUM_4
#define B_IN1 GPIO_NUM_6
#define B_IN2 GPIO_NUM_5
#define D_PWM GPIO_NUM_16
#define D_IN1 GPIO_NUM_7
#define D_IN2 GPIO_NUM_15
#define STBY_GPIO GPIO_NUM_8

/* OUT2/OUT3 on black: equal A/D magnitude, B stopped. */
#define STRAIGHT_A_SPEED 18
#define STRAIGHT_D_SPEED 18
#define CURVE_A_SPEED 16
#define CURVE_D_SPEED 16
#define TURN_MAX 12
#define MAX_OUTPUT 28
#define FILTER_SAMPLES 5U
#define FILTER_STABLE_CYCLES 2U
#define TURN_DELAY_CYCLES 3U
#define PID_KP 4
#define PID_KI 1
#define PID_KD 2
#define PID_SCALE 10
#define PID_DEADBAND 3
#define PID_INTEGRAL_LIMIT 30
#define LOOP_MS 10U
#define LOG_MS 100U
#define START_DELAY_MS 2000U
#define PWM_MAX 1023U
#define START_KICK_OUTPUT 30
#define START_KICK_CYCLES 8U
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN -1

typedef struct {
    gpio_num_t pwm, in1, in2;
    ledc_channel_t channel;
    int sign;
} motor_t;

static const char *TAG = "line_follow";
static const gpio_num_t sensors[4] = {OUT1_GPIO, OUT2_GPIO, OUT3_GPIO, OUT4_GPIO};
static const int weights[4] = {-3, -1, 1, 3};
static const motor_t motor_a = {A_IN1, A_IN2, LEDC_CHANNEL_0, MOTOR_A_SIGN};
static const motor_t motor_b = {B_IN1, B_IN2, LEDC_CHANNEL_1, MOTOR_B_SIGN};
static const motor_t motor_d = {D_IN1, D_IN2, LEDC_CHANNEL_2, MOTOR_D_SIGN};

static int clamp(int value, int limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static void motor_set(motor_t *motor, int speed)
{
    speed = clamp(speed, MAX_OUTPUT);
    int direction = (speed > 0) - (speed < 0);
    if (direction == 0) kick_cycles[index] = 0;
    else if (direction != last_direction[index]) kick_cycles[index] = START_KICK_CYCLES;
    last_direction[index] = direction;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
    gpio_set_level(motor->in1, speed > 0);
    gpio_set_level(motor->in2, speed < 0);
    int output = abs(speed);
    if (kick_cycles[index] && output < START_KICK_OUTPUT) output = START_KICK_OUTPUT;
    if (kick_cycles[index]) --kick_cycles[index];
    uint32_t duty = PWM_MAX * (uint32_t)output / 100U;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

/* A 镜像安装所以取负；B 只提供旋转分量；正 turn 为左转。 */
static void drive(int forward, int turn, int out[3])
{
    out[0] = clamp(-forward - turn, MAX_OUTPUT);
    out[1] = clamp(turn, MAX_OUTPUT);
    out[2] = clamp(forward - turn, MAX_OUTPUT);
    for (int i = 0; i < 3; ++i) motor_set(&motors[i], out[i]);
}

static uint8_t read_sensors(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < 4; ++i)
        if (gpio_get_level(sensors[i]) == IR_ACTIVE_LEVEL) mask |= 1U << i;
    return mask;
}

/* 负值表示线在左侧，正值表示线在右侧，范围约 ±30。 */
static int line_error(uint8_t mask)
{
    int sum = 0, count = 0;
    for (int i = 0; i < 4; ++i)
        if (mask & (1U << i)) { sum += weights[i]; ++count; }
    return count ? sum * 10 / count : 0;
}

static void hardware_init(void)
{
    for (int i = 0; i < 4; ++i) {
        gpio_reset_pin(sensors[i]);
        gpio_set_direction(sensors[i], GPIO_MODE_INPUT);
    }
    gpio_reset_pin(STBY_GPIO);
    gpio_set_direction(STBY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(STBY_GPIO, 0);

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 2000, .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < 3; ++i) {
        gpio_reset_pin(motors[i].in1);
        gpio_set_direction(motors[i].in1, GPIO_MODE_OUTPUT);
        gpio_set_level(motors[i].in1, 0);
        gpio_reset_pin(motors[i].in2);
        gpio_set_direction(motors[i].in2, GPIO_MODE_OUTPUT);
        gpio_set_level(motors[i].in2, 0);
        motors[i].last_direction = 0;
        const ledc_channel_config_t channel = {
            .gpio_num = motors[i].pwm, .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i].channel, .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0, .duty = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
    gpio_set_level(STBY_GPIO, 1);
}

static uint8_t sample_active_mask(void)
{
    uint8_t votes[4] = {0};
    for (size_t sample = 0; sample < FILTER_SAMPLES; ++sample) {
        for (size_t connector = 0; connector < 4; ++connector) {
            size_t position = REVERSE_SENSOR_ORDER ? 3U - connector : connector;
            if (gpio_get_level(sensors[connector]) == IR_ACTIVE_LEVEL) ++votes[position];
        }
        if (sample + 1U < FILTER_SAMPLES) vTaskDelay(pdMS_TO_TICKS(1));
    }
    uint8_t mask = 0;
    for (size_t i = 0; i < 4; ++i) if (votes[i] >= 3U) mask |= 1U << i;
    return mask;
}

static int line_error(uint8_t mask)
{
    int sum = 0, count = 0;
    for (int i = 0; i < 4; ++i) if (mask & (1U << i)) { sum += weights[i]; ++count; }
    return count ? sum * 10 / count : 0;
}

static uint8_t filtered_mask(void)
{
    static uint8_t stable = 0, pending = 0, count = 0;
    uint8_t mask = sample_active_mask();
    if (mask && line_error(mask) == 0) { count = 0; return stable = mask; }
    if (line_error(mask) * line_error(stable) < 0) {
        if (mask != pending) { pending = mask; count = 1; }
        else if (++count >= FILTER_STABLE_CYCLES) { stable = pending; count = 0; return stable; }
        return CENTER_MASK;
    }
    if (mask == stable) { count = 0; return stable; }
    if (mask != pending) { pending = mask; count = 1; }
    else if (++count >= FILTER_STABLE_CYCLES) { stable = pending; count = 0; }
    return stable;
}

static uint8_t control_mask(void)
{
    static uint8_t history[TURN_DELAY_CYCLES] = {0};
    static size_t index = 0;
    uint8_t mask = filtered_mask();
    int current_error = line_error(mask);
    if (mask && current_error == 0) {
        for (size_t i = 0; i < TURN_DELAY_CYCLES; ++i) history[i] = mask;
        index = 0;
        return mask;
    }
    uint8_t delayed = history[index];
    history[index] = mask;
    index = (index + 1U) % TURN_DELAY_CYCLES;
    if (current_error * line_error(delayed) < 0) return CENTER_MASK;
    return delayed;
}

static int pid_steering(int error)
{
    static int integral = 0, previous = 0, derivative_filtered = 0;
    if (abs(error) <= PID_DEADBAND) {
        integral = previous = derivative_filtered = 0;
        return 0;
    }
    integral = clamp(integral + error, PID_INTEGRAL_LIMIT);
    int derivative = error - previous;
    derivative_filtered = (derivative_filtered + derivative) / 2;
    previous = error;
    int output = -(PID_KP * error + PID_KI * integral + PID_KD * derivative_filtered) / PID_SCALE;
    return clamp(output, TURN_MAX);
}

static uint8_t raw_levels(void)
{
    uint8_t raw = 0;
    for (int i = 0; i < 4; ++i) if (gpio_get_level(sensors[i])) raw |= 1U << i;
    return raw;
}

void app_main(void)
{
    hardware_init();
    int out[3] = {0};
    int search_turn = TURN_MAX; /* 丢线沿最后一次偏差方向搜索；无偏差时默认左转 */
    bool line_seen = false;
    uint32_t log_elapsed = LOG_MS;

    drive(0, 0, out);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    while (true) {
        uint8_t mask = read_sensors();
        int error = line_error(mask);
        int forward, turn;

        if (mask == 0) {
            forward = 0;
            turn = line_seen ? search_turn : 0; /* 上电首次见线前不动 */
        } else {
            line_seen = true;
            forward = error ? CURVE_SPEED : STRAIGHT_SPEED;
            turn = clamp(-error * TURN_KP / 10, TURN_MAX);
            if (error) search_turn = error < 0 ? TURN_MAX : -TURN_MAX;
        }
        drive(forward, turn, out);

        if (log_elapsed >= LOG_MS) {
            ESP_LOGI(TAG, "ACTIVE=%u%u%u%u err=%d turn=%d motor[A,B,D]=[%d,%d,%d]",
                     (unsigned)(mask & 1U), (unsigned)((mask >> 1) & 1U),
                     (unsigned)((mask >> 2) & 1U), (unsigned)((mask >> 3) & 1U),
                     error, turn, out[0], out[1], out[2]);
            log_elapsed = 0;
        }
        log_elapsed += LOOP_MS;
        /* 不依赖额外的 FreeRTOS DelayUntil 配置；控制周期约为 10 ms。 */
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
