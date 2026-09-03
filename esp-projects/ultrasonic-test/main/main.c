#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ultrasonic.h"

/* Motor pins and polarity are copied from the latest local camera project. */
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

#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN (-1)

#define ULTRASONIC_TRIG GPIO_NUM_18
#define ULTRASONIC_ECHO GPIO_NUM_11
#define ULTRASONIC_MIN_CM 2.0f
#define ULTRASONIC_MAX_CM 400.0f
#define OBSTACLE_DETECT_CM 10.0f
#define OBSTACLE_CONFIRM_SAMPLES 2U
#define ULTRASONIC_PERIOD_MS 60U

/*
 * These are raw mixer commands, before MOTOR_*_SIGN is applied.
 * Forward is deliberately A/D only: [-25, 0, +20].
 * Change only these values while tuning the physical route.
 */
#define FORWARD_A_CMD (-20)
#define FORWARD_B_CMD 0
#define FORWARD_D_CMD 25

#define LEFT_A_CMD (-24)
#define LEFT_B_CMD (-30)
#define LEFT_D_CMD (-18)

#define RETURN_FORWARD_A_CMD FORWARD_A_CMD
#define RETURN_FORWARD_B_CMD FORWARD_B_CMD
#define RETURN_FORWARD_D_CMD FORWARD_D_CMD

#define RIGHT_A_CMD 18
#define RIGHT_B_CMD 30
#define RIGHT_D_CMD 24

#define STOP_WAIT_MS 500U
#define LEFT_SHIFT_MS 1500U
#define FORWARD_AFTER_LEFT_MS 1000U
#define RIGHT_SHIFT_MS 1500U

#define MOTOR_PWM_LIMIT 40
#define PWM_MAX 1023U
#define MOTOR_PWM_FREQUENCY_HZ 2000
#define CONTROL_PERIOD_MS 10U
#define STATUS_LOG_MS 250U

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    gpio_num_t pwm_gpio;
    ledc_channel_t channel;
    int sign;
} motor_t;

typedef enum {
    MOTION_FORWARD = 0,
    MOTION_STOP_WAIT,
    MOTION_SHIFT_LEFT,
    MOTION_FORWARD_AFTER_LEFT,
    MOTION_SHIFT_RIGHT,
    MOTION_DONE,
} motion_state_t;

static const char *TAG = "ultrasonic_test";
static const motor_t s_motor_a = {
    A_IN1, A_IN2, A_PWM, LEDC_CHANNEL_0, MOTOR_A_SIGN
};
static const motor_t s_motor_b = {
    B_IN1, B_IN2, B_PWM, LEDC_CHANNEL_1, MOTOR_B_SIGN
};
static const motor_t s_motor_d = {
    D_IN1, D_IN2, D_PWM, LEDC_CHANNEL_2, MOTOR_D_SIGN
};

static volatile float s_distance_cm = -1.0f;
static volatile bool s_distance_valid;
static volatile uint32_t s_distance_sequence;

static motion_state_t s_motion_state = MOTION_FORWARD;
static int64_t s_phase_started_us;
static int s_command_a;
static int s_command_b;
static int s_command_d;
static int s_effective_a;
static int s_effective_b;
static int s_effective_d;
static int s_last_direction[3];

static int clamp_command(int value)
{
    if (value > MOTOR_PWM_LIMIT) {
        return MOTOR_PWM_LIMIT;
    }
    if (value < -MOTOR_PWM_LIMIT) {
        return -MOTOR_PWM_LIMIT;
    }
    return value;
}

static void set_motor_duty(const motor_t *motor, int effective_command)
{
    const uint32_t duty = PWM_MAX * (uint32_t)abs(effective_command) / 100U;
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty);
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

static int motor_write(const motor_t *motor, int command)
{
    const int effective = clamp_command(command * motor->sign);
    const int index = (int)motor->channel;
    const int direction = (effective > 0) - (effective < 0);

    if (direction != s_last_direction[index]) {
        set_motor_duty(motor, 0);
        gpio_set_level(motor->in1, direction > 0 ? 1 : 0);
        gpio_set_level(motor->in2, direction < 0 ? 1 : 0);
        s_last_direction[index] = direction;
    }
    if (direction == 0) {
        gpio_set_level(motor->in1, 0);
        gpio_set_level(motor->in2, 0);
    }
    set_motor_duty(motor, effective);
    return effective;
}

static void set_drive(int a, int b, int d)
{
    s_command_a = clamp_command(a);
    s_command_b = clamp_command(b);
    s_command_d = clamp_command(d);
    s_effective_a = motor_write(&s_motor_a, s_command_a);
    s_effective_b = motor_write(&s_motor_b, s_command_b);
    s_effective_d = motor_write(&s_motor_d, s_command_d);
}

static void stop_drive(void)
{
    set_drive(0, 0, 0);
}

static const char *motion_state_name(motion_state_t state)
{
    switch (state) {
    case MOTION_FORWARD:
        return "FORWARD";
    case MOTION_STOP_WAIT:
        return "STOP_WAIT";
    case MOTION_SHIFT_LEFT:
        return "SHIFT_LEFT";
    case MOTION_FORWARD_AFTER_LEFT:
        return "FORWARD_2";
    case MOTION_SHIFT_RIGHT:
        return "SHIFT_RIGHT";
    case MOTION_DONE:
    default:
        return "DONE";
    }
}

static uint32_t phase_elapsed_ms(int64_t now)
{
    if (s_phase_started_us == 0 || now <= s_phase_started_us) {
        return 0;
    }
    const int64_t elapsed = (now - s_phase_started_us) / 1000;
    return elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
}

static void enter_state(motion_state_t next, int64_t now)
{
    s_motion_state = next;
    s_phase_started_us = now;
    if (next == MOTION_STOP_WAIT || next == MOTION_DONE) {
        stop_drive();
    }
    ESP_LOGI(TAG, "state=%s", motion_state_name(next));
}

static void hardware_init(void)
{
    const gpio_num_t direction_pins[] = {
        A_IN1, A_IN2, B_IN1, B_IN2, D_IN1, D_IN2, STBY_GPIO
    };
    for (size_t i = 0; i < sizeof(direction_pins) / sizeof(direction_pins[0]); ++i) {
        ESP_ERROR_CHECK(gpio_reset_pin(direction_pins[i]));
        ESP_ERROR_CHECK(gpio_set_direction(direction_pins[i], GPIO_MODE_OUTPUT));
    }
    gpio_set_level(STBY_GPIO, 0);

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    const motor_t *motors[] = {&s_motor_a, &s_motor_b, &s_motor_d};
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = motors[i]->pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i]->channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }

    ultrasonic_init(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    s_last_direction[0] = 0;
    s_last_direction[1] = 0;
    s_last_direction[2] = 0;
}

static void ultrasonic_task(void *arg)
{
    (void)arg;
    while (true) {
        float distance = ultrasonic_read_cm();
        const bool valid = distance >= ULTRASONIC_MIN_CM &&
                           distance <= ULTRASONIC_MAX_CM;
        if (!valid) {
            distance = -1.0f;
        }
        s_distance_cm = distance;
        s_distance_valid = valid;
        ++s_distance_sequence;
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
    }
}

static void read_sensor_snapshot(float *distance, bool *valid, uint32_t *sequence)
{
    uint32_t before;
    uint32_t after;
    do {
        before = s_distance_sequence;
        *distance = s_distance_cm;
        *valid = s_distance_valid;
        after = s_distance_sequence;
    } while (before != after);
    *sequence = after;
}

static void log_status(int64_t now, float distance, bool valid,
                       uint32_t close_samples)
{
    ESP_LOGI(TAG,
             "state=%s phase_ms=%lu distance_cm=%.1f valid=%d close=%lu "
             "cmd[A,B,D]=[%d,%d,%d] effective[A,B,D]=[%d,%d,%d] STBY=%d",
             motion_state_name(s_motion_state),
             (unsigned long)phase_elapsed_ms(now), (double)distance, valid,
             (unsigned long)close_samples,
             s_command_a, s_command_b, s_command_d,
             s_effective_a, s_effective_b, s_effective_d,
             s_motion_state == MOTION_DONE ? 0 : 1);
}

void app_main(void)
{
    hardware_init();
    if (xTaskCreate(ultrasonic_task, "ultrasonic", 2048, NULL, 1, NULL) != pdPASS) {
        ESP_LOGE(TAG, "cannot create ultrasonic task; motors remain disabled");
        return;
    }

    /* Keep the control loop responsive while the sensor task performs its echo wait. */
    vTaskPrioritySet(NULL, 3);
    gpio_set_level(STBY_GPIO, 1);
    s_phase_started_us = esp_timer_get_time();
    set_drive(FORWARD_A_CMD, FORWARD_B_CMD, FORWARD_D_CMD);
    ESP_LOGI(TAG, "start: direct A/D forward cmd=[%d,%d,%d]",
             FORWARD_A_CMD, FORWARD_B_CMD, FORWARD_D_CMD);

    uint32_t last_sensor_sequence = 0;
    uint32_t close_samples = 0;
    int64_t last_log_us = 0;

    while (true) {
        const int64_t now = esp_timer_get_time();
        float distance;
        bool valid;
        uint32_t sensor_sequence;
        read_sensor_snapshot(&distance, &valid, &sensor_sequence);
        if (sensor_sequence != last_sensor_sequence) {
            last_sensor_sequence = sensor_sequence;
            if (valid && distance <= OBSTACLE_DETECT_CM) {
                if (close_samples < UINT32_MAX) {
                    ++close_samples;
                }
            } else {
                close_samples = 0;
            }
        }

        if (s_motion_state == MOTION_FORWARD &&
            close_samples >= OBSTACLE_CONFIRM_SAMPLES) {
            close_samples = 0;
            ESP_LOGW(TAG, "obstacle confirmed at %.1f cm", (double)distance);
            enter_state(MOTION_STOP_WAIT, now);
        }

        switch (s_motion_state) {
        case MOTION_FORWARD:
            set_drive(FORWARD_A_CMD, FORWARD_B_CMD, FORWARD_D_CMD);
            break;
        case MOTION_STOP_WAIT:
            stop_drive();
            if (phase_elapsed_ms(now) >= STOP_WAIT_MS) {
                enter_state(MOTION_SHIFT_LEFT, now);
            }
            break;
        case MOTION_SHIFT_LEFT:
            set_drive(LEFT_A_CMD, LEFT_B_CMD, LEFT_D_CMD);
            if (phase_elapsed_ms(now) >= LEFT_SHIFT_MS) {
                enter_state(MOTION_FORWARD_AFTER_LEFT, now);
            }
            break;
        case MOTION_FORWARD_AFTER_LEFT:
            set_drive(RETURN_FORWARD_A_CMD, RETURN_FORWARD_B_CMD,
                      RETURN_FORWARD_D_CMD);
            if (phase_elapsed_ms(now) >= FORWARD_AFTER_LEFT_MS) {
                enter_state(MOTION_SHIFT_RIGHT, now);
            }
            break;
        case MOTION_SHIFT_RIGHT:
            set_drive(RIGHT_A_CMD, RIGHT_B_CMD, RIGHT_D_CMD);
            if (phase_elapsed_ms(now) >= RIGHT_SHIFT_MS) {
                enter_state(MOTION_DONE, now);
            }
            break;
        case MOTION_DONE:
        default:
            stop_drive();
            gpio_set_level(STBY_GPIO, 0);
            break;
        }

        if (last_log_us == 0 || now - last_log_us >= (int64_t)STATUS_LOG_MS * 1000) {
            log_status(now, distance, valid, close_samples);
            last_log_us = now;
        }
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
