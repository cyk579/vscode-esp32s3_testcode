#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Four IR sensors, ordered from the car's left to right. */
#define SENSOR_OUT1_GPIO       GPIO_NUM_41
#define SENSOR_OUT2_GPIO       GPIO_NUM_42
#define SENSOR_OUT3_GPIO       GPIO_NUM_2
#define SENSOR_OUT4_GPIO       GPIO_NUM_1

/* Measured on this module: white=high, black=low. */
#define IR_ACTIVE_LEVEL        0
#define SENSOR_REVERSE_ORDER   0

/* TB6612 motor driver pins. */
#define MOTOR_A_PWM_GPIO       GPIO_NUM_9
#define MOTOR_A_IN1_GPIO       GPIO_NUM_12
#define MOTOR_A_IN2_GPIO       GPIO_NUM_10
#define MOTOR_B_PWM_GPIO       GPIO_NUM_4
#define MOTOR_B_IN1_GPIO       GPIO_NUM_6
#define MOTOR_B_IN2_GPIO       GPIO_NUM_5
#define MOTOR_D_PWM_GPIO       GPIO_NUM_16
#define MOTOR_D_IN1_GPIO       GPIO_NUM_7
#define MOTOR_D_IN2_GPIO       GPIO_NUM_15
#define MOTOR_STBY_GPIO        GPIO_NUM_8

/* Main tuning area. */
#define CONTROL_PERIOD_MS       10U
#define START_DELAY_MS          2000U
#define DEBUG_PRINT_PERIOD_MS   100U
#define BASE_SPEED_PERCENT      28
#define MAX_SPEED_PERCENT       55
#define MIN_CORNER_SPEED        18
#define KP                      25
#define KD                      9
#define CORNER_HOLD_MS          180U
#define LOST_LINE_TIMEOUT_MS    1200U

/* Change an individual sign to -1 if that motor runs backwards. */
#define MOTOR_A_SIGN            1
#define MOTOR_B_SIGN            1
#define MOTOR_D_SIGN            1

/* Set to 1 to test only the IR sensors without moving the car. */
#define SENSOR_ONLY_DEBUG       0

#define MOTOR_PWM_FREQUENCY_HZ  10000
#define MOTOR_PWM_RESOLUTION    LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX_DUTY      ((1U << 10) - 1U)

typedef struct {
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    ledc_channel_t pwm_channel;
    int sign;
} motor_t;

static const char *TAG = "line_follow";
static const gpio_num_t sensor_gpio[4] = {
    SENSOR_OUT1_GPIO, SENSOR_OUT2_GPIO, SENSOR_OUT3_GPIO, SENSOR_OUT4_GPIO
};
static const int sensor_weight[4] = {-3, -1, 1, 3};
static const motor_t motor_a = {
    MOTOR_A_IN1_GPIO, MOTOR_A_IN2_GPIO, LEDC_CHANNEL_0, MOTOR_A_SIGN
};
static const motor_t motor_b = {
    MOTOR_B_IN1_GPIO, MOTOR_B_IN2_GPIO, LEDC_CHANNEL_1, MOTOR_B_SIGN
};
static const motor_t motor_d = {
    MOTOR_D_IN1_GPIO, MOTOR_D_IN2_GPIO, LEDC_CHANNEL_2, MOTOR_D_SIGN
};

static uint32_t percent_to_duty(int speed)
{
    if (speed < 0) speed = -speed;
    if (speed > 100) speed = 100;
    return (MOTOR_PWM_MAX_DUTY * (uint32_t)speed) / 100U;
}

static void set_pwm_duty(ledc_channel_t channel, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static int clamp_speed(int speed)
{
    if (speed > MAX_SPEED_PERCENT) return MAX_SPEED_PERCENT;
    if (speed < -MAX_SPEED_PERCENT) return -MAX_SPEED_PERCENT;
    return speed;
}

static void motor_set(const motor_t *motor, int speed)
{
    speed = clamp_speed(speed * motor->sign);
    set_pwm_duty(motor->pwm_channel, 0);
    ESP_ERROR_CHECK(gpio_set_level(motor->in1_gpio, speed > 0));
    ESP_ERROR_CHECK(gpio_set_level(motor->in2_gpio, speed < 0));
    set_pwm_duty(motor->pwm_channel, percent_to_duty(speed));
}

/*
 * 120-degree three-wheel chassis. The sensor/front is between A and D,
 * with B opposite it. Forward therefore uses A/D oppositely and B is idle.
 * A common signed rotation component is superimposed for steering.
 */
static void car_set_wheels(int a, int b, int d)
{
    motor_set(&motor_a, a);
    motor_set(&motor_b, b);
    motor_set(&motor_d, d);
}

static void car_stop(void)
{
    car_set_wheels(0, 0, 0);
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY_GPIO, 0));
}

static void driver_enable(void)
{
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY_GPIO, 1));
    vTaskDelay(pdMS_TO_TICKS(2));
}

static void motor_driver_init(void)
{
    const uint64_t output_mask =
        (1ULL << MOTOR_A_IN1_GPIO) | (1ULL << MOTOR_A_IN2_GPIO) |
        (1ULL << MOTOR_B_IN1_GPIO) | (1ULL << MOTOR_B_IN2_GPIO) |
        (1ULL << MOTOR_D_IN1_GPIO) | (1ULL << MOTOR_D_IN2_GPIO) |
        (1ULL << MOTOR_STBY_GPIO);
    const gpio_config_t output_config = {
        .pin_bit_mask = output_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    const ledc_channel_config_t channels[] = {
        { .gpio_num = MOTOR_A_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE,
          .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR_B_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_1, .intr_type = LEDC_INTR_DISABLE,
          .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR_D_PWM_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_2, .intr_type = LEDC_INTR_DISABLE,
          .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
    };
    for (size_t i = 0; i < sizeof(channels) / sizeof(channels[0]); ++i) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }
    car_stop();
}

static void sensor_init(void)
{
    uint64_t mask = 0;
    for (size_t i = 0; i < 4; ++i) mask |= 1ULL << sensor_gpio[i];
    const gpio_config_t config = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

/* bit0=OUT1, bit3=OUT4; a set bit means line detected. */
static uint8_t read_sensor_mask(void)
{
    uint8_t mask = 0;
    for (size_t i = 0; i < 4; ++i) {
        bool active = gpio_get_level(sensor_gpio[i]) == IR_ACTIVE_LEVEL;
        size_t bit = SENSOR_REVERSE_ORDER ? 3U - i : i;
        if (active) mask |= (uint8_t)(1U << bit);
    }
    return mask;
}

static bool mask_has(uint8_t mask, size_t bit)
{
    return (mask & (1U << bit)) != 0;
}

static void print_debug(uint8_t mask, int error, int correction,
                        int a, int b, int d)
{
    ESP_LOGI(TAG, "IR(L->R)=%u%u%u%u mask=0x%02X err=%d corr=%d motor[A,B,D]=[%d,%d,%d]",
             mask_has(mask, 0), mask_has(mask, 1),
             mask_has(mask, 2), mask_has(mask, 3),
             mask, error, correction, a, b, d);
}

static void line_follow_loop(void)
{
    int last_error = 0;
    int previous_error = 0;
    uint32_t lost_ms = 0;
    uint32_t corner_ms = 0;
    uint32_t debug_ms = DEBUG_PRINT_PERIOD_MS;
    bool line_seen = false;

    driver_enable();
    ESP_LOGI(TAG, "Line following started: base=%d%% kp=%d kd=%d", BASE_SPEED_PERCENT, KP, KD);

    while (true) {
        uint8_t mask = read_sensor_mask();
        int sum = 0;
        int count = 0;
        for (size_t i = 0; i < 4; ++i) {
            if (mask_has(mask, i)) {
                sum += sensor_weight[i];
                ++count;
            }
        }

        int error = last_error;
        if (count > 0) {
            error = (sum * 10) / count;
            last_error = error;
            lost_ms = 0;
            line_seen = true;
        } else {
            lost_ms += CONTROL_PERIOD_MS;
        }

        /* Do not move until at least one sensor has actually found the line. */
        if (!line_seen) {
            car_set_wheels(0, 0, 0);
            if (debug_ms >= DEBUG_PRINT_PERIOD_MS) {
                print_debug(mask, 0, 0, 0, 0, 0);
                debug_ms = 0;
            }
            debug_ms += CONTROL_PERIOD_MS;
            vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
            continue;
        }

        bool left_corner = mask == 0x01U;
        bool right_corner = mask == 0x08U;
        if (left_corner || right_corner) corner_ms = CORNER_HOLD_MS;
        else if (corner_ms > CONTROL_PERIOD_MS) corner_ms -= CONTROL_PERIOD_MS;
        else corner_ms = 0;

        /* Left error needs CCW rotation; right error needs CW rotation. */
        int correction = -(KP * error + KD * (error - previous_error)) / 10;
        previous_error = error;
        if (correction > 100) correction = 100;
        if (correction < -100) correction = -100;

        int base = (corner_ms > 0 || abs(error) >= 20) ? MIN_CORNER_SPEED : BASE_SPEED_PERCENT;
        int a, b, d;
        if (lost_ms > LOST_LINE_TIMEOUT_MS) {
            int search = last_error >= 0 ? -35 : 35;
            a = search;
            b = search;
            d = search;
        } else {
            a = -base + correction;
            b = correction;
            d = base + correction;
            if (left_corner)  { a =  40; b =  40; d =  40; }
            if (right_corner) { a = -40; b = -40; d = -40; }
        }
        car_set_wheels(a, b, d);

        if (debug_ms >= DEBUG_PRINT_PERIOD_MS) {
            print_debug(mask, error, correction, a, b, d);
            debug_ms = 0;
        }
        debug_ms += CONTROL_PERIOD_MS;
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void app_main(void)
{
    motor_driver_init();
    sensor_init();
    ESP_LOGI(TAG, "IR pins OUT1..4=GPIO%d,%d,%d,%d; active=%d; reverse=%d",
             SENSOR_OUT1_GPIO, SENSOR_OUT2_GPIO, SENSOR_OUT3_GPIO,
             SENSOR_OUT4_GPIO, IR_ACTIVE_LEVEL, SENSOR_REVERSE_ORDER);

    if (SENSOR_ONLY_DEBUG) {
        ESP_LOGW(TAG, "SENSOR_ONLY_DEBUG=1: motors are disabled. Move a black line under each sensor.");
        while (true) {
            print_debug(read_sensor_mask(), 0, 0, 0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(DEBUG_PRINT_PERIOD_MS));
        }
    }

    ESP_LOGW(TAG, "Keep wheels lifted: line following starts in %u ms", START_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));
    line_follow_loop();
}
