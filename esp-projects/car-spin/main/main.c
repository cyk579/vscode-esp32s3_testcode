#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define OUT1_GPIO GPIO_NUM_41
#define OUT2_GPIO GPIO_NUM_42
#define OUT3_GPIO GPIO_NUM_2
#define OUT4_GPIO GPIO_NUM_1
#define IR_ACTIVE_LEVEL 0
#define REVERSE_SENSOR_ORDER 1
#define CENTER_MASK 0x06U

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
#define STRAIGHT_A_SPEED 30
#define STRAIGHT_D_SPEED 30
#define CURVE_A_SPEED 20
#define CURVE_D_SPEED 20
#define TURN_MAX 12
#define MAX_OUTPUT 30
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
    gpio_num_t in1, in2;
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

static void motor_set(const motor_t *motor, int speed)
{
    static int last_direction[3] = {0};
    static uint8_t kick_cycles[3] = {0};
    speed = clamp(speed * motor->sign, MAX_OUTPUT);
    int index = (int)motor->channel;
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

static void drive(int forward_a, int forward_d, int turn, int *a, int *b, int *d)
{
    *a = clamp(-forward_a - turn, MAX_OUTPUT);
    *b = clamp(turn, MAX_OUTPUT);
    *d = clamp(forward_d - turn, MAX_OUTPUT);
    motor_set(&motor_a, *a);
    motor_set(&motor_b, *b);
    motor_set(&motor_d, *d);
}

static void hardware_init(void)
{
    const gpio_num_t outputs[] = {A_IN1, A_IN2, B_IN1, B_IN2, D_IN1, D_IN2, STBY_GPIO};
    for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); ++i) {
        gpio_reset_pin(outputs[i]);
        gpio_set_direction(outputs[i], GPIO_MODE_OUTPUT);
    }
    for (size_t i = 0; i < 4; ++i) {
        gpio_reset_pin(sensors[i]);
        gpio_set_direction(sensors[i], GPIO_MODE_INPUT);
    }
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 2000, .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    const gpio_num_t pwm[3] = {A_PWM, B_PWM, D_PWM};
    for (int i = 0; i < 3; ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = pwm[i], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = i,
            .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0
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
    /* 强偏差通常是急弯，直接采用当前稳定读数，避免前探队列把弯道拖过。 */
    if (abs(current_error) >= 20) return mask;
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
    int a = 0, b = 0, d = 0, turn = 0, last_turn = 1;
    uint32_t log_elapsed = LOG_MS;
    bool line_seen = false;
    drive(0, 0, 0, &a, &b, &d);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));
    while (true) {
        uint8_t mask = control_mask();
        int error = line_error(mask);
        if (mask) line_seen = true;
        if (!line_seen) {
            turn = pid_steering(0);
            drive(0, 0, 0, &a, &b, &d);
        } else if (mask == CENTER_MASK || mask == 0x0FU || (mask && error == 0)) {
            turn = pid_steering(0);
            drive(STRAIGHT_A_SPEED, STRAIGHT_D_SPEED, 0, &a, &b, &d);
        } else if (mask == 0) {
            pid_steering(0);
            turn = last_turn * TURN_MAX;
            drive(0, 0, turn, &a, &b, &d);
        }
        else {
            last_turn = error < 0 ? 1 : -1;
            turn = pid_steering(error);
            drive(CURVE_A_SPEED, CURVE_D_SPEED, turn, &a, &b, &d);
        }
        if (log_elapsed >= LOG_MS) {
            uint8_t raw = raw_levels();
            ESP_LOGI(TAG, "RAW=%u%u%u%u ACTIVE=%u%u%u%u err=%d turn=%d motor[A,B,D]=[%d,%d,%d]",
                     raw & 1, raw >> 1 & 1, raw >> 2 & 1, raw >> 3 & 1,
                     mask & 1, mask >> 1 & 1, mask >> 2 & 1, mask >> 3 & 1,
                     error, turn, a, b, d);
            log_elapsed = 0;
        }
        log_elapsed += LOOP_MS;
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
