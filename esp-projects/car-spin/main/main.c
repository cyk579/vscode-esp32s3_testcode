#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ultrasonic.h"
#include "tft_st7735.h"
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
#define STRAIGHT_A_SPEED 24
#define STRAIGHT_D_SPEED 24
#define CURVE_A_SPEED 14
#define CURVE_D_SPEED 14
#define LOST_LINE_FORWARD_MS 100U
#define LOST_LINE_A_SPEED 20
#define LOST_LINE_D_SPEED 20
#define SEARCH_A_SPEED 16
#define SEARCH_D_SPEED 16
#define TURN_MAX 9
#define TURN_B_GAIN 2
#define MAX_OUTPUT 26
#define TURN_SIGN 1
#define FILTER_SAMPLES 5U
#define FILTER_STABLE_CYCLES 2U
#define PID_KP 4
#define PID_KI 1
#define PID_KD 2
#define PID_SCALE 10
#define PID_DEADBAND 3
#define PID_INTEGRAL_LIMIT 30
#define ERROR_FILTER_DIVISOR 2
#define TURN_SLEW_STEP 2
#define LOOP_MS 10U
#define LOG_MS 100U
#define START_DELAY_MS 2000U
#define PWM_MAX 1023U
#define START_KICK_OUTPUT 30
#define START_KICK_CYCLES 2U
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN -1

#define ULTRASONIC_TRIG GPIO_NUM_18
#define ULTRASONIC_ECHO GPIO_NUM_11
#define OBSTACLE_DETECT_CM 7.0f
#define OBSTACLE_CLEAR_CM 15.0f
#define AVOID_SHIFT_B_SPEED 18
#define AVOID_SHIFT_AD_SPEED 8
#define AVOID_FORWARD_SPEED 16
#define AVOID_FORWARD_MS 1000U
#define AVOID_SHIFT_TIMEOUT_MS 3500U
#define ULTRASONIC_PERIOD_MS 100U
#define DISPLAY_PERIOD_MS 100U
#define ULTRASONIC_WARN_SAMPLES 10U
#define OBSTACLE_CONFIRM_SAMPLES 1U
#define END_ARM_MS 80U
#define END_TURN_MAX 8
#define END_BRAKE_DELAY_MS 20U
#define END_CONFIRM_MS 80U

typedef struct {
    gpio_num_t in1, in2;
    ledc_channel_t channel;
    int sign;
    bool use_start_kick;
} motor_t;

static const char *TAG = "line_follow";
static const gpio_num_t sensors[4] = {OUT1_GPIO, OUT2_GPIO, OUT3_GPIO, OUT4_GPIO};
static const int weights[4] = {-3, -1, 1, 3};
static const motor_t motor_a = {A_IN1, A_IN2, LEDC_CHANNEL_0, MOTOR_A_SIGN, true};
static const motor_t motor_b = {B_IN1, B_IN2, LEDC_CHANNEL_1, MOTOR_B_SIGN, false};
static const motor_t motor_d = {D_IN1, D_IN2, LEDC_CHANNEL_2, MOTOR_D_SIGN, true};

typedef enum { AVOID_LINE, AVOID_LEFT, AVOID_FORWARD, AVOID_RIGHT } avoid_state_t;
static volatile tft_status_t display_status = { -1.0f, 0, 0, 0, 0, 0, 0, "LINE" };
static volatile avoid_state_t avoid_state = AVOID_LINE;
static volatile ultrasonic_status_t ultrasonic_status = ULTRASONIC_NO_ECHO;
static volatile uint32_t ultrasonic_sequence = 0;

static int clamp(int value, int limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static int move_towards(int value, int target, int step)
{
    if (value < target) return value + clamp(target - value, step);
    if (value > target) return value - clamp(value - target, step);
    return value;
}

static void motor_set(const motor_t *motor, int speed)
{
    static int last_direction[3] = {0};
    static uint8_t kick_cycles[3] = {0};
    speed = clamp(speed * motor->sign, MAX_OUTPUT);
    int index = (int)motor->channel;
    int direction = (speed > 0) - (speed < 0);
    if (direction != last_direction[index]) {
        /* 仅在真正换向时撤掉 PWM，避免每 10 ms 产生一次制动脉冲。 */
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
        gpio_set_level(motor->in1, speed > 0);
        gpio_set_level(motor->in2, speed < 0);
        kick_cycles[index] = motor->use_start_kick && direction != 0 ? START_KICK_CYCLES : 0;
        last_direction[index] = direction;
    }
    int output = abs(speed);
    if (motor->use_start_kick && kick_cycles[index] && output < START_KICK_OUTPUT)
        output = START_KICK_OUTPUT;
    if (kick_cycles[index]) --kick_cycles[index];
    uint32_t duty = PWM_MAX * (uint32_t)output / 100U;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

static void drive(int forward_a, int forward_d, int turn, int *a, int *b, int *d)
{
    /* 若所有左右纠偏都与实车相反，仅把 TURN_SIGN 改为 -1。 */
    turn *= TURN_SIGN;
    *a = clamp(-forward_a - turn, MAX_OUTPUT);
    *b = clamp(turn * TURN_B_GAIN, MAX_OUTPUT);
    *d = clamp(forward_d - turn, MAX_OUTPUT);
    motor_set(&motor_a, *a);
    motor_set(&motor_b, *b);
    motor_set(&motor_d, *d);
}

/* 横移沿用原有正负方向，但由后轮 B 主导，A/D 仅作辅助。 */
static void drive_lateral(int direction, int *a, int *b, int *d)
{
    *a = direction * AVOID_SHIFT_AD_SPEED;
    *b = -direction * AVOID_SHIFT_B_SPEED;
    *d = direction * AVOID_SHIFT_AD_SPEED;
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
    ultrasonic_init(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
}

static const char *avoid_mode_name(void)
{
    switch (avoid_state) {
    case AVOID_LEFT: return "AVOID-L";
    case AVOID_FORWARD: return "AVOID-F";
    case AVOID_RIGHT: return "AVOID-R";
    default: return "LINE";
    }
}

static void display_task(void *arg)
{
    (void)arg;
    if (!tft_st7735_init()) vTaskDelete(NULL);
    while (true) {
        tft_status_t snapshot = {
            display_status.distance_cm, display_status.ir_mask, display_status.error,
            display_status.turn, display_status.motor_a, display_status.motor_b,
            display_status.motor_d, display_status.mode
        };
        tft_st7735_show(&snapshot);
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

static void ultrasonic_task(void *arg)
{
    (void)arg;
    uint8_t failed_samples = 0;
    ultrasonic_status_t last_status = ULTRASONIC_OK;
    bool status_reported = false;
    while (true) {
        ultrasonic_status_t status;
        float distance = ultrasonic_read_cm(&status);
        ultrasonic_status = status;
        display_status.distance_cm = distance;
        ++ultrasonic_sequence;
        if (!status_reported || status != last_status) {
            ESP_LOGI(TAG, "US %s trig=GPIO%d echo=GPIO%d dist=%.1fcm",
                     ultrasonic_status_name(status), (int)ULTRASONIC_TRIG,
                     (int)ULTRASONIC_ECHO, (double)distance);
            last_status = status;
            status_reported = true;
        }
        if (distance < 0.0f) {
            if (failed_samples < UINT8_MAX) ++failed_samples;
            if (failed_samples == ULTRASONIC_WARN_SAMPLES) {
                ESP_LOGW(TAG, "US %s: NO_ECHO=power/TRIG/ECHO/pin mapping; ECHO_HIGH=level wiring; OUT_OF_RANGE=echo received but invalid",
                         ultrasonic_status_name(status));
            }
        } else {
            failed_samples = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
    }
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
    /* filtered_mask 已做多数投票和方向确认，不再回放旧方向的历史输入。 */
    return filtered_mask();
}

static int pid_steering(int error)
{
    static int integral = 0, filtered_error = 0;
    static int previous = 0, derivative_filtered = 0;
    if (abs(error) <= PID_DEADBAND) {
        integral = filtered_error = previous = derivative_filtered = 0;
        return 0;
    }

    /* 四路数字传感器的误差只有少数几个离散值，先低通再送入 PID。 */
    filtered_error += (error - filtered_error) / ERROR_FILTER_DIVISOR;
    integral = clamp(integral + filtered_error, PID_INTEGRAL_LIMIT);
    int derivative = filtered_error - previous;
    derivative_filtered = (derivative_filtered + derivative) / 2;
    previous = filtered_error;
    int output = -(PID_KP * filtered_error + PID_KI * integral + PID_KD * derivative_filtered) / PID_SCALE;
    return clamp(output, TURN_MAX);
}

static uint8_t raw_levels(void)
{
    uint8_t raw = 0;
    for (int i = 0; i < 4; ++i) if (gpio_get_level(sensors[i])) raw |= 1U << i;
    return raw;
}

/* IR_ACTIVE_LEVEL 为 0 时，此条件正是接口原始电平 RAW=0000。 */
static bool raw_all_sensors_on_black(void)
{
    for (size_t i = 0; i < 4; ++i)
        if (gpio_get_level(sensors[i]) != IR_ACTIVE_LEVEL) return false;
    return true;
}

static int follow_speed(int straight_speed, int curve_speed, int turn)
{
    int speed = straight_speed - abs(turn);
    return speed < curve_speed ? curve_speed : speed;
}

void app_main(void)
{
    hardware_init();
    xTaskCreate(ultrasonic_task, "ultrasonic", 2048, NULL, 1, NULL);
    xTaskCreate(display_task, "tft", 4096, NULL, 1, NULL);
    int a = 0, b = 0, d = 0, turn = 0, last_turn = 1;
    uint32_t log_elapsed = LOG_MS;
    uint32_t avoid_elapsed = 0;
    uint32_t left_shift_ms = 0;
    uint32_t line_lost_ms = 0;
    uint32_t end_active_ms = 0;
    uint32_t end_straight_ms = 0;
    uint32_t last_ultrasonic_sequence = 0;
    uint8_t obstacle_close_samples = 0;
    bool line_seen = false;
    bool finished = false;
    drive(0, 0, 0, &a, &b, &d);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        uint8_t mask = control_mask();
        int error = line_error(mask);
        float distance = display_status.distance_cm;
        bool raw_end_line = raw_all_sensors_on_black();
        bool ultrasonic_valid = ultrasonic_status == ULTRASONIC_OK;
        bool ultrasonic_updated = ultrasonic_sequence != last_ultrasonic_sequence;
        bool end_braking = false;

        if (ultrasonic_updated) last_ultrasonic_sequence = ultrasonic_sequence;

        if (avoid_state == AVOID_LINE) {
            if (ultrasonic_updated) {
                if (ultrasonic_valid && distance > 0.0f && distance <= OBSTACLE_DETECT_CM) {
                    if (obstacle_close_samples < UINT8_MAX) ++obstacle_close_samples;
                } else {
                    obstacle_close_samples = 0;
                }
            }
            if (obstacle_close_samples >= OBSTACLE_CONFIRM_SAMPLES) {
                avoid_state = AVOID_LEFT;
                avoid_elapsed = 0;
                left_shift_ms = 0;
                obstacle_close_samples = 0;
                ESP_LOGI(TAG, "Obstacle %.1fcm: stop line following and shift left", (double)distance);
            }
        } else if (avoid_state == AVOID_LEFT) {
            bool obstacle_cleared = ultrasonic_updated && ultrasonic_valid &&
                                    distance >= OBSTACLE_CLEAR_CM;
            bool shift_timed_out = avoid_elapsed >= AVOID_SHIFT_TIMEOUT_MS;
            if (obstacle_cleared || shift_timed_out) {
                left_shift_ms = avoid_elapsed;
                avoid_state = AVOID_FORWARD;
                avoid_elapsed = 0;
                if (shift_timed_out && !obstacle_cleared) {
                    ESP_LOGW(TAG, "Left shift timed out after %lums; using that duration for right shift",
                             (unsigned long)left_shift_ms);
                } else {
                    ESP_LOGI(TAG, "Obstacle cleared at %.1fcm; recorded left shift=%lums",
                             (double)distance, (unsigned long)left_shift_ms);
                }
            }
        } else if (avoid_state == AVOID_FORWARD && avoid_elapsed >= AVOID_FORWARD_MS) {
            avoid_state = AVOID_RIGHT;
            avoid_elapsed = 0;
            ESP_LOGI(TAG, "Forward complete; shift right for %lums", (unsigned long)left_shift_ms);
        } else if (avoid_state == AVOID_RIGHT) {
            bool line_centered = mask == CENTER_MASK;
            bool matched_left_time = avoid_elapsed >= left_shift_ms;
            if (line_centered || matched_left_time) {
                avoid_state = AVOID_LINE;
                ESP_LOGI(TAG, "Right shift complete after %lums (%s); resume line following",
                         (unsigned long)avoid_elapsed,
                         line_centered ? "line centered" : "matched left time");
                avoid_elapsed = 0;
            }
        }

        /*
         * 终点只看未经控制滤波的四路读数：IR_ACTIVE_LEVEL=0 时即 RAW=0000。
         * 先要求小车已稳定直行，转弯或丢线状态会取消预备；随后再连续确认，
         * 因而转弯时短暂扫过横线不会触发停车。
         */
        if (finished || avoid_state != AVOID_LINE || !line_seen) {
            end_active_ms = 0;
            end_straight_ms = 0;
        } else if (!raw_end_line) {
            bool straight = mask != 0 && mask != 0x0FU &&
                            abs(error) <= 10 && abs(turn) <= END_TURN_MAX;
            end_straight_ms = straight ? end_straight_ms + LOOP_MS : 0;
            end_active_ms = 0;
        } else if (abs(turn) > END_TURN_MAX || end_straight_ms < END_ARM_MS) {
            if (abs(turn) > END_TURN_MAX) end_straight_ms = 0;
            end_active_ms = 0;
        } else {
            end_active_ms += LOOP_MS;
            end_braking = end_active_ms >= END_BRAKE_DELAY_MS;
            if (end_active_ms >= END_CONFIRM_MS) finished = true;
        }

        if (finished || end_braking) {
            avoid_state = AVOID_LINE;
            drive(0, 0, 0, &a, &b, &d);
            turn = 0;
        } else if (avoid_state == AVOID_LEFT) {
            drive_lateral(1, &a, &b, &d);
        } else if (avoid_state == AVOID_FORWARD) {
            drive(AVOID_FORWARD_SPEED, AVOID_FORWARD_SPEED, 0, &a, &b, &d);
        } else if (avoid_state == AVOID_RIGHT) {
            drive_lateral(-1, &a, &b, &d);
        } else if (!line_seen) {
            turn = pid_steering(0);
            drive(0, 0, 0, &a, &b, &d);
        } else if (mask == 0) {
            pid_steering(0);
            line_lost_ms += LOOP_MS;
            if (line_lost_ms <= LOST_LINE_FORWARD_MS) {
                turn = move_towards(turn, last_turn * (TURN_MAX / 2), TURN_SLEW_STEP);
                drive(LOST_LINE_A_SPEED, LOST_LINE_D_SPEED, turn, &a, &b, &d);
            } else {
                turn = move_towards(turn, last_turn * TURN_MAX, TURN_SLEW_STEP);
                drive(SEARCH_A_SPEED, SEARCH_D_SPEED, turn, &a, &b, &d);
            }
        } else {
            line_lost_ms = 0;
            if (error != 0) last_turn = error < 0 ? 1 : -1;
            turn = move_towards(turn, pid_steering(error), TURN_SLEW_STEP);
            drive(follow_speed(STRAIGHT_A_SPEED, CURVE_A_SPEED, turn),
                  follow_speed(STRAIGHT_D_SPEED, CURVE_D_SPEED, turn), turn, &a, &b, &d);
        }
        if (mask) line_seen = true;
        display_status.ir_mask = mask;
        display_status.error = error;
        display_status.turn = turn;
        display_status.motor_a = a;
        display_status.motor_b = b;
        display_status.motor_d = d;
        const char *mode = (finished || end_braking) ? "END" : avoid_mode_name();
        display_status.mode = mode;
        if (log_elapsed >= LOG_MS) {
            uint8_t raw = raw_levels();
            ESP_LOGI(TAG, "RAW=%u%u%u%u ACTIVE=%u%u%u%u dist=%.1fcm mode=%s err=%d turn=%d end=%lums motor[A,B,D]=[%d,%d,%d]",
                     raw & 1, raw >> 1 & 1, raw >> 2 & 1, raw >> 3 & 1,
                     mask & 1, mask >> 1 & 1, mask >> 2 & 1, mask >> 3 & 1,
                     (double)distance, mode, error, turn, (unsigned long)end_active_ms, a, b, d);
            log_elapsed = 0;
        }
        log_elapsed += LOOP_MS;
        if (avoid_state != AVOID_LINE) avoid_elapsed += LOOP_MS;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_MS));
    }
}
