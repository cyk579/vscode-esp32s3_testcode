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

#define FULL_RUN_ENABLE 1  /* 0: 10 cm 处停车测试；1: 执行完整避障并跑到 END。 */

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
#define CURVE_A_SPEED 22
#define CURVE_D_SPEED 22
#define TURN_MAX 19
#define LOST_REVERSE_SPEED 17
#define LOST_TURN 17
#define LOST_SPIN_B_SPEED 16
#define LOST_SPIN_MS 320U
#define LINE_MAX_OUTPUT 33
#define MAX_OUTPUT 34
#define FILTER_SAMPLES 5U
#define FILTER_STABLE_CYCLES 2U
#define TURN_DELAY_CYCLES 3U
#define PID_KP 4
#define PID_KI 1
#define PID_KD 2
#define PID_SCALE 10
#define PID_DEADBAND 3
#define PID_INTEGRAL_LIMIT 30
#define PID_SMOOTH_NEW_WEIGHT 5  /* PID 输出保留 50% 旧值、50% 新值。 */
#define PID_SMOOTH_WEIGHT_SUM 10
#define PID_SMOOTH_BYPASS_ERROR 20
#define TURN_MEMORY_STRONG_ERROR 20
#define TURN_MEMORY_CONFIRM_CYCLES 2U
#define LOOP_MS 10U
#define LOG_MS 100U
#define START_DELAY_MS 2000U
#define PWM_MAX 1023U
#define MOTOR_MIN_RUN_OUTPUT 11
#define MOTOR_B_MIN_RUN_OUTPUT 13
#define START_KICK_OUTPUT 32
#define START_KICK_CYCLES 8U
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN -1

#define ULTRASONIC_TRIG GPIO_NUM_18
#define ULTRASONIC_ECHO GPIO_NUM_11
#define ULTRASONIC_MIN_CM 2.0f
#define ULTRASONIC_MAX_CM 400.0f
#define OBSTACLE_DETECT_CM 15.0f
#define OBSTACLE_CLEAR_CM 80.0f
#define AVOID_BRAKE_MS 500U
#define AVOID_ALIGN_SPEED 13
#define AVOID_ALIGN_TIMEOUT_MS 2000U
#define AVOID_LEFT_SIDE_SPEED 18
#define AVOID_LEFT_B_SPEED 25
#define AVOID_RIGHT_A_SPEED 18
#define AVOID_RIGHT_B_SPEED 25
#define AVOID_RIGHT_D_SPEED 18
#define AVOID_LEFT_MIN_MS 250U
#define AVOID_LEFT_TIMEOUT_MS 6000U
#define AVOID_FORWARD_SPEED 25
#define AVOID_FORWARD_MS 2500U
#define AVOID_RIGHT_MIN_MS 200U
#define AVOID_RIGHT_CENTER_CONFIRM_CYCLES 3U
#define AVOID_RIGHT_TIMEOUT_MS 6000U
#define ULTRASONIC_PERIOD_MS 60U
#define DISPLAY_PERIOD_MS 100U
#define END_CONFIRM_MS 30U

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

typedef enum { US_OK, US_NO_ECHO, US_ECHO_HIGH, US_OUT_OF_RANGE } ultrasonic_status_t;
typedef enum {
    AVOID_LINE,
    AVOID_BRAKE,
    AVOID_ALIGN,
    AVOID_LEFT,
    AVOID_FORWARD,
    AVOID_RIGHT,
    AVOID_DISTANCE_STOP,
    AVOID_FAIL_STOP,
    AVOID_END
} avoid_state_t;
static volatile tft_status_t display_status = { -1.0f, 0, 0, 0, 0, 0, 0, "TEST" };
static volatile float ultrasonic_distance_cm = -1.0f;
static volatile ultrasonic_status_t ultrasonic_status = US_NO_ECHO;
static volatile uint32_t ultrasonic_sequence = 0;
static volatile avoid_state_t avoid_state = AVOID_LINE;
static uint8_t latest_sampled_mask = 0;

static int clamp(int value, int limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static void motor_set_impl(const motor_t *motor, int speed, bool enable_kick)
{
    static int last_direction[3] = {0};
    static uint8_t kick_cycles[3] = {0};
    speed = clamp(speed * motor->sign, MAX_OUTPUT);
    int index = (int)motor->channel;
    int direction = (speed > 0) - (speed < 0);
    bool direction_changed = direction != last_direction[index];
    if (!enable_kick || direction == 0) kick_cycles[index] = 0;
    else if (direction_changed) kick_cycles[index] = START_KICK_CYCLES;
    if (direction_changed) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
        gpio_set_level(motor->in1, speed > 0);
        gpio_set_level(motor->in2, speed < 0);
        last_direction[index] = direction;
    }
    int output = abs(speed);
    int minimum_output = motor->channel == LEDC_CHANNEL_1 ?
                         MOTOR_B_MIN_RUN_OUTPUT : MOTOR_MIN_RUN_OUTPUT;
    if (output > 0 && output < minimum_output) output = minimum_output;
    if (enable_kick && kick_cycles[index] &&
        (motor->channel != LEDC_CHANNEL_1 || output > TURN_MAX) &&
        output < START_KICK_OUTPUT) {
        output = START_KICK_OUTPUT;
    }
    if (kick_cycles[index]) --kick_cycles[index];
    uint32_t duty = PWM_MAX * (uint32_t)output / 100U;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

static void motor_set(const motor_t *motor, int speed)
{
    motor_set_impl(motor, speed, true);
}

static void motor_set_direct(const motor_t *motor, int speed)
{
    motor_set_impl(motor, speed, false);
}

static void drive(int forward_a, int forward_d, int turn, int *a, int *b, int *d)
{
    *a = clamp(-forward_a - turn, LINE_MAX_OUTPUT);
    *b = clamp(turn, LINE_MAX_OUTPUT);
    *d = clamp(forward_d - turn, LINE_MAX_OUTPUT);
    motor_set(&motor_a, *a);
    motor_set(&motor_b, *b);
    motor_set(&motor_d, *d);
}

/* 三轮全向底盘横移混控：A/D 为半幅，B 为全幅。 */
static void drive_vector(int forward, int lateral, int turn, int *a, int *b, int *d)
{
    int half_lateral = lateral / 2;
    *a = clamp(-forward + half_lateral + turn, MAX_OUTPUT);
    *b = clamp(-lateral + turn, MAX_OUTPUT);
    *d = clamp(forward + half_lateral + turn, MAX_OUTPUT);
    motor_set(&motor_a, *a);
    motor_set(&motor_b, *b);
    motor_set(&motor_d, *d);
}

static void drive_spin(int direction, int *a, int *b, int *d)
{
    *a = clamp(-direction * LOST_TURN, LINE_MAX_OUTPUT);
    *b = clamp(direction * LOST_SPIN_B_SPEED, LINE_MAX_OUTPUT);
    *d = clamp(-direction * LOST_TURN, LINE_MAX_OUTPUT);
    motor_set_direct(&motor_a, *a);
    motor_set_direct(&motor_b, *b);
    motor_set_direct(&motor_d, *d);
}

static void drive_align(int direction, int *a, int *b, int *d)
{
    *a = -direction * AVOID_ALIGN_SPEED;
    *b = direction * AVOID_ALIGN_SPEED;
    *d = -direction * AVOID_ALIGN_SPEED;
    motor_set_direct(&motor_a, *a);
    motor_set_direct(&motor_b, *b);
    motor_set_direct(&motor_d, *d);
}

static void drive_lateral(bool left, int *a, int *b, int *d)
{
    if (left) {
        *a = -AVOID_LEFT_SIDE_SPEED;
        *b = -AVOID_LEFT_B_SPEED;
        *d = -AVOID_LEFT_SIDE_SPEED;
    } else {
        *a = AVOID_RIGHT_A_SPEED;
        *b = AVOID_RIGHT_B_SPEED;
        *d = AVOID_RIGHT_D_SPEED;
    }
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
    case AVOID_BRAKE: return "WAIT";
    case AVOID_ALIGN: return "ALIGN";
    case AVOID_LEFT: return "AVOID-L";
    case AVOID_FORWARD: return "AVOID-F";
    case AVOID_RIGHT: return "AVOID-R";
    case AVOID_DISTANCE_STOP: return "DIST";
    case AVOID_FAIL_STOP: return "FAIL";
    case AVOID_END: return "END";
    default: return FULL_RUN_ENABLE ? "LINE" : "TEST";
    }
}

static void ultrasonic_task(void *arg)
{
    (void)arg;
    while (true) {
        float distance = ultrasonic_read_cm();
        ultrasonic_status_t status = US_OK;
        if (distance < 0.0f) {
            status = gpio_get_level(ULTRASONIC_ECHO) ? US_ECHO_HIGH : US_NO_ECHO;
        } else if (distance < ULTRASONIC_MIN_CM || distance > ULTRASONIC_MAX_CM) {
            distance = -1.0f;
            status = US_OUT_OF_RANGE;
        }
        ultrasonic_distance_cm = distance;
        ultrasonic_status = status;
        display_status.distance_cm = distance;
        ++ultrasonic_sequence;
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
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

static void remember_turn(int error, int *last_turn, int *candidate,
                          uint8_t *candidate_cycles)
{
    if (error == 0) {
        *candidate = 0;
        *candidate_cycles = 0;
        return;
    }
    int direction = error < 0 ? 1 : -1;
    if (abs(error) >= TURN_MEMORY_STRONG_ERROR) {
        *last_turn = direction;
        *candidate = 0;
        *candidate_cycles = 0;
        return;
    }
    if (*candidate != direction) {
        *candidate = direction;
        *candidate_cycles = 1;
        return;
    }
    if (*candidate_cycles < TURN_MEMORY_CONFIRM_CYCLES) ++*candidate_cycles;
    if (*candidate_cycles >= TURN_MEMORY_CONFIRM_CYCLES) {
        *last_turn = direction;
        *candidate = 0;
        *candidate_cycles = 0;
    }
}

static uint8_t filtered_mask(void)
{
    static uint8_t stable = 0, pending = 0, count = 0;
    uint8_t mask = sample_active_mask();
    latest_sampled_mask = mask;
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
    static int integral = 0, previous = 0, derivative_filtered = 0, output_filtered = 0;
    if (abs(error) <= PID_DEADBAND) {
        integral = previous = derivative_filtered = output_filtered = 0;
        return 0;
    }
    integral = clamp(integral + error, PID_INTEGRAL_LIMIT);
    int derivative = error - previous;
    derivative_filtered = (derivative_filtered + derivative) / 2;
    previous = error;
    int output = -(PID_KP * error + PID_KI * integral + PID_KD * derivative_filtered) / PID_SCALE;
    output = clamp(output, TURN_MAX);
    if (abs(error) >= PID_SMOOTH_BYPASS_ERROR) {
        output_filtered = output;
        return output;
    }
    output_filtered = ((PID_SMOOTH_WEIGHT_SUM - PID_SMOOTH_NEW_WEIGHT) * output_filtered +
                       PID_SMOOTH_NEW_WEIGHT * output) / PID_SMOOTH_WEIGHT_SUM;
    return output_filtered;
}

static uint8_t raw_levels(void)
{
    uint8_t raw = 0;
    for (int i = 0; i < 4; ++i) if (gpio_get_level(sensors[i])) raw |= 1U << i;
    return raw;
}

static bool raw_all_black(void)
{
    for (size_t i = 0; i < 4; ++i) {
        if (gpio_get_level(sensors[i]) != IR_ACTIVE_LEVEL) return false;
    }
    return true;
}

void app_main(void)
{
    hardware_init();
    xTaskCreate(ultrasonic_task, "ultrasonic", 2048, NULL, 1, NULL);
    xTaskCreate(display_task, "tft", 4096, NULL, 1, NULL);
    vTaskPrioritySet(NULL, 3);
    int a = 0, b = 0, d = 0, turn = 0, last_turn = -1;
    int turn_candidate = 0;
    uint8_t turn_candidate_cycles = 0;
    uint8_t right_center_cycles = 0;
    uint32_t log_elapsed = LOG_MS;
    uint32_t avoid_elapsed = 0;
    uint32_t left_shift_ms = 0;
    uint32_t end_active_ms = 0;
    uint32_t lost_elapsed_ms = 0;
    uint32_t last_ultrasonic_sequence = 0;
    bool line_seen = false;
    bool end_armed = false;
    drive(0, 0, 0, &a, &b, &d);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));
    while (true) {
        uint8_t mask = control_mask();
        int error = line_error(mask);
        int immediate_error = line_error(latest_sampled_mask);
        int immediate_turn = abs(immediate_error) >= TURN_MEMORY_STRONG_ERROR ?
                             (immediate_error < 0 ? 1 : -1) : 0;
        if (avoid_state == AVOID_LINE) {
            remember_turn(immediate_error, &last_turn, &turn_candidate,
                          &turn_candidate_cycles);
        } else {
            turn_candidate = 0;
            turn_candidate_cycles = 0;
        }
        bool end_raw = raw_all_black();
        float distance = ultrasonic_distance_cm;
        ultrasonic_status_t current_ultrasonic_status = ultrasonic_status;
        uint32_t current_ultrasonic_sequence = ultrasonic_sequence;
        bool ultrasonic_new = current_ultrasonic_sequence != last_ultrasonic_sequence;
        bool ultrasonic_valid = current_ultrasonic_status == US_OK && distance > 0.0f;
        if (ultrasonic_new) last_ultrasonic_sequence = current_ultrasonic_sequence;
        if (mask) line_seen = true;

        if (avoid_state == AVOID_LINE && line_seen && ultrasonic_new && ultrasonic_valid &&
            distance <= OBSTACLE_DETECT_CM) {
            drive(0, 0, 0, &a, &b, &d);
            turn = pid_steering(0);
            avoid_elapsed = 0;
            lost_elapsed_ms = 0;
            avoid_state = FULL_RUN_ENABLE ? AVOID_BRAKE : AVOID_DISTANCE_STOP;
            ESP_LOGW(TAG, "BREAKPOINT 10CM dist=%.1fcm FULL_RUN=%d",
                     (double)distance, FULL_RUN_ENABLE);
        }

        if (avoid_state == AVOID_END || avoid_state == AVOID_DISTANCE_STOP ||
            avoid_state == AVOID_FAIL_STOP) {
            drive(0, 0, 0, &a, &b, &d);
            turn = pid_steering(0);
        } else if (avoid_state == AVOID_BRAKE) {
            drive(0, 0, 0, &a, &b, &d);
            turn = pid_steering(0);
            if (avoid_elapsed >= AVOID_BRAKE_MS) {
                avoid_state = AVOID_ALIGN;
                avoid_elapsed = 0;
                ESP_LOGI(TAG, "WAIT done; align ACTIVE to 0110");
            }
        } else if (avoid_state == AVOID_ALIGN) {
            uint8_t align_mask = latest_sampled_mask;
            if (align_mask == CENTER_MASK) {
                drive(0, 0, 0, &a, &b, &d);
                turn = pid_steering(0);
                avoid_state = AVOID_LEFT;
                avoid_elapsed = 0;
                ESP_LOGI(TAG, "ALIGN 0110; AVOID LEFT start");
            } else if (avoid_elapsed >= AVOID_ALIGN_TIMEOUT_MS) {
                drive(0, 0, 0, &a, &b, &d);
                turn = pid_steering(0);
                avoid_state = AVOID_FAIL_STOP;
                ESP_LOGE(TAG, "ALIGN timeout -> FAIL STOP");
            } else {
                int align_error = line_error(align_mask);
                int align_direction = align_error < 0 ? 1 :
                                      (align_error > 0 ? -1 : last_turn);
                turn = align_direction * AVOID_ALIGN_SPEED;
                pid_steering(0);
                drive_align(align_direction, &a, &b, &d);
            }
        } else if (avoid_state == AVOID_LEFT) {
            drive_lateral(true, &a, &b, &d);
            turn = pid_steering(0);
            bool far_clear = ultrasonic_new && ultrasonic_valid && distance > OBSTACLE_CLEAR_CM;
            if (avoid_elapsed >= AVOID_LEFT_MIN_MS && far_clear) {
                left_shift_ms = avoid_elapsed;
                avoid_state = AVOID_FORWARD;
                avoid_elapsed = 0;
                ESP_LOGI(TAG, "LEFT done time=%lums clear=%d",
                         (unsigned long)left_shift_ms, far_clear);
            } else if (avoid_elapsed >= AVOID_LEFT_TIMEOUT_MS) {
                avoid_state = AVOID_FAIL_STOP;
                ESP_LOGE(TAG, "LEFT timeout -> FAIL STOP");
            }
        } else if (avoid_state == AVOID_FORWARD) {
            drive_vector(AVOID_FORWARD_SPEED, 0, 0, &a, &b, &d);
            turn = pid_steering(0);
            if (avoid_elapsed >= AVOID_FORWARD_MS) {
                avoid_state = AVOID_RIGHT;
                avoid_elapsed = 0;
                right_center_cycles = 0;
                ESP_LOGI(TAG, "AVOID RIGHT start left=%lums", (unsigned long)left_shift_ms);
            }
        } else if (avoid_state == AVOID_RIGHT) {
            drive_lateral(false, &a, &b, &d);
            turn = pid_steering(0);
            bool centered = latest_sampled_mask == CENTER_MASK;
            if (avoid_elapsed < AVOID_RIGHT_MIN_MS || !centered) {
                right_center_cycles = 0;
            } else if (right_center_cycles < AVOID_RIGHT_CENTER_CONFIRM_CYCLES) {
                ++right_center_cycles;
            }
            if (right_center_cycles >= AVOID_RIGHT_CENTER_CONFIRM_CYCLES) {
                avoid_state = AVOID_LINE;
                end_armed = true;
                avoid_elapsed = 0;
                end_active_ms = 0;
                lost_elapsed_ms = 0;
                right_center_cycles = 0;
                turn = pid_steering(0);
                ESP_LOGI(TAG, "AVOID done; center confirmed; END armed");
            } else if (avoid_elapsed >= AVOID_RIGHT_TIMEOUT_MS) {
                avoid_state = AVOID_FAIL_STOP;
                ESP_LOGE(TAG, "RIGHT timeout -> FAIL STOP");
            }
        } else if (!line_seen) {
            lost_elapsed_ms = 0;
            turn = pid_steering(0);
            drive(0, 0, 0, &a, &b, &d);
        } else if (end_armed && end_raw) {
            lost_elapsed_ms = 0;
            end_active_ms += LOOP_MS;
            turn = pid_steering(0);
            drive(0, 0, 0, &a, &b, &d);
            if (end_active_ms >= END_CONFIRM_MS) {
                avoid_state = AVOID_END;
                ESP_LOGW(TAG, "END confirmed");
            }
        } else if (immediate_turn != 0) {
            lost_elapsed_ms = 0;
            end_active_ms = 0;
            turn = immediate_turn * LOST_TURN;
            pid_steering(0);
            drive_spin(immediate_turn, &a, &b, &d);
        } else if (mask == CENTER_MASK || mask == 0x0FU || (mask && error == 0)) {
            lost_elapsed_ms = 0;
            end_active_ms = 0;
            turn = pid_steering(0);
            drive(STRAIGHT_A_SPEED, STRAIGHT_D_SPEED, 0, &a, &b, &d);
        } else if (mask == 0) {
            end_active_ms = 0;
            pid_steering(0);
            lost_elapsed_ms += LOOP_MS;
            turn = last_turn * LOST_TURN;
            if (lost_elapsed_ms <= LOST_SPIN_MS) {
                drive_spin(last_turn, &a, &b, &d);
            } else {
                drive(-LOST_REVERSE_SPEED, -LOST_REVERSE_SPEED, turn, &a, &b, &d);
            }
        } else {
            lost_elapsed_ms = 0;
            end_active_ms = 0;
            turn = pid_steering(error);
            drive(CURVE_A_SPEED, CURVE_D_SPEED, turn, &a, &b, &d);
        }
        display_status.ir_mask = mask;
        display_status.error = error;
        display_status.turn = turn;
        display_status.motor_a = a;
        display_status.motor_b = b;
        display_status.motor_d = d;
        display_status.mode = avoid_mode_name();
        if (log_elapsed >= LOG_MS) {
            uint8_t raw = raw_levels();
            ESP_LOGI(TAG, "RAW=%u%u%u%u ACTIVE=%u%u%u%u dist=%.1fcm mode=%s err=%d turn=%d motor[A,B,D]=[%d,%d,%d]",
                     raw & 1, raw >> 1 & 1, raw >> 2 & 1, raw >> 3 & 1,
                     mask & 1, mask >> 1 & 1, mask >> 2 & 1, mask >> 3 & 1,
                     (double)distance, avoid_mode_name(), error, turn, a, b, d);
            log_elapsed = 0;
        }
        log_elapsed += LOOP_MS;
        if (avoid_state == AVOID_BRAKE || avoid_state == AVOID_ALIGN ||
            avoid_state == AVOID_LEFT ||
            avoid_state == AVOID_FORWARD || avoid_state == AVOID_RIGHT) {
            avoid_elapsed += LOOP_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
