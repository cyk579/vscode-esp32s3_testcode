#include "camera_line_follow.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* The pin map is shared with the tested car-spin application. */
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

/* Camera output is delivered by esp_jpeg with swap_color_bytes enabled. */
#define CAMERA_LINE_MIRROR_X 0

/* Image processing parameters.  The camera should point down at the track. */
#define LINE_ROI_TOP_PERCENT 45
#define LINE_ROI_BOTTOM_PERCENT 95
#define LINE_ROW_STEP 3
#define LINE_MIN_CONTRAST 28
#define LINE_BLACK_FRACTION_PERCENT 42
#define LINE_BLACK_THRESHOLD_MIN 45
#define LINE_BLACK_THRESHOLD_MAX 145
#define LINE_MIN_RUN_SAMPLES 2
#define LINE_MAX_RUN_PERCENT 70
#define LINE_MIN_VALID_ROWS 2
#define LINE_FINISH_ROW_COVERAGE_PERCENT 58
#define LINE_FINISH_MIN_ROWS 3

/* Motion and temporal safety parameters. */
#define LINE_START_DELAY_MS 1200U
#define LINE_ARM_CONFIRM_FRAMES 4U
#define LINE_LOST_HOLD_MS 160U
#define LINE_LOST_SEARCH_MS 520U
#define LINE_LOST_STOP_MS 900U
#define LINE_FRAME_TIMEOUT_MS 450U
#define LINE_FINISH_CONFIRM_FRAMES 6U

#define LINE_FORWARD_FAST 28
#define LINE_FORWARD_MEDIUM 22
#define LINE_FORWARD_SLOW 16
#define LINE_FORWARD_CRAWL 11
#define LINE_TURN_MAX 30
#define LINE_PID_KP 30
#define LINE_PID_KI 1
#define LINE_PID_KD 10
#define LINE_PID_SCALE 100
#define LINE_PID_DEADBAND 3
#define LINE_PID_INTEGRAL_LIMIT 220

#define PWM_MAX 1023U
#define MOTOR_MIN_RUN_OUTPUT 10
#define MOTOR_B_MIN_RUN_OUTPUT 12
#define START_KICK_OUTPUT 27
#define START_KICK_CYCLES 5U
#define MOTOR_B_KICK_MIN_OUTPUT 18
#define MAX_OUTPUT 34

/* These signs match the direction calibration documented by car-spin. */
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN -1

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t channel;
    int sign;
} motor_t;

typedef struct {
    bool valid;
    bool finish_candidate;
    int error;
    int threshold;
    uint8_t confidence;
} line_observation_t;

static const char *TAG = "camera_line";
static const motor_t motor_a = {A_IN1, A_IN2, LEDC_CHANNEL_0, MOTOR_A_SIGN};
static const motor_t motor_b = {B_IN1, B_IN2, LEDC_CHANNEL_1, MOTOR_B_SIGN};
static const motor_t motor_d = {D_IN1, D_IN2, LEDC_CHANNEL_2, MOTOR_D_SIGN};

static volatile bool s_started;
static bool s_armed;
static bool s_finished;
static uint8_t s_arm_frames;
static uint8_t s_finish_frames;
static int64_t s_first_frame_us;
static int64_t s_last_line_us;
static int64_t s_last_log_us;
static int s_last_error;
static int s_pid_integral;
static int s_pid_previous;
static int s_pid_derivative;
static int s_pid_output;
static int s_command_a;
static int s_command_b;
static int s_command_d;
static int s_last_threshold;
static uint8_t s_last_confidence;
static int s_last_direction[3];
static uint8_t s_kick_cycles[3];
static int64_t s_last_frame_us;
static SemaphoreHandle_t s_control_mutex;
static bool s_watchdog_created;

static void camera_line_follow_watchdog_task(void *arg);

static int clamp_int(int value, int limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static uint8_t rgb565_luma(const uint8_t *pixel)
{
    const uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    const int red = ((value >> 11) & 0x1F) * 255 / 31;
    const int green = ((value >> 5) & 0x3F) * 255 / 63;
    const int blue = (value & 0x1F) * 255 / 31;
    return (uint8_t)((77 * red + 150 * green + 29 * blue) >> 8);
}

static void consider_run(int start_x,
                         int sample_count,
                         int sample_step,
                         int width,
                         int expected_x,
                         int *best_score,
                         int *best_center,
                         int *best_width)
{
    if (sample_count < LINE_MIN_RUN_SAMPLES) {
        return;
    }

    const int run_width = sample_count * sample_step;
    if (run_width > width * LINE_MAX_RUN_PERCENT / 100) {
        return;
    }

    const int end_x = start_x + run_width - sample_step;
    const int center_x = (start_x + end_x) / 2;
    const int score = sample_count * 100 - abs(center_x - expected_x) * 2;
    if (score > *best_score) {
        *best_score = score;
        *best_center = center_x;
        *best_width = run_width;
    }
}

static bool observe_line(const uint8_t *frame,
                         uint16_t width,
                         uint16_t height,
                         line_observation_t *observation)
{
    if (frame == NULL || observation == NULL || width < 16 || height < 16) {
        return false;
    }

    int top = (int)height * LINE_ROI_TOP_PERCENT / 100;
    int bottom = (int)height * LINE_ROI_BOTTOM_PERCENT / 100;
    if (top < 0) {
        top = 0;
    }
    if (bottom >= (int)height) {
        bottom = (int)height - 1;
    }
    if (bottom <= top) {
        return false;
    }

    const int x_step = width >= 96 ? 2 : 1;
    const int y_step = LINE_ROW_STEP;
    int minimum = 255;
    int maximum = 0;
    int sample_total = 0;
    for (int y = top; y <= bottom; y += y_step) {
        for (int x = 0; x < (int)width; x += x_step) {
            const uint8_t value = rgb565_luma(frame +
                                              (((size_t)y * (size_t)width + (size_t)x) * 2));
            if (value < minimum) {
                minimum = value;
            }
            if (value > maximum) {
                maximum = value;
            }
            ++sample_total;
        }
    }
    if (sample_total == 0 || maximum - minimum < LINE_MIN_CONTRAST) {
        return false;
    }

    int threshold = minimum + (maximum - minimum) * LINE_BLACK_FRACTION_PERCENT / 100;
    if (threshold < LINE_BLACK_THRESHOLD_MIN) {
        threshold = LINE_BLACK_THRESHOLD_MIN;
    }
    if (threshold > LINE_BLACK_THRESHOLD_MAX) {
        threshold = LINE_BLACK_THRESHOLD_MAX;
    }

    int expected_x = (int)width / 2 + s_last_error * (int)width / 240;
    if (expected_x < 0) {
        expected_x = 0;
    }
    if (expected_x >= (int)width) {
        expected_x = (int)width - 1;
    }

    int64_t weighted_center = 0;
    int weight_sum = 0;
    int valid_rows = 0;
    int wide_rows = 0;
    int wide_center_sum = 0;
    for (int y = top; y <= bottom; y += y_step) {
        int best_score = -1;
        int best_center = 0;
        int best_width = 0;
        int run_start = 0;
        int run_samples = 0;
        int dark_samples = 0;
        int dark_sum = 0;
        const int total_row_samples = ((int)width + x_step - 1) / x_step;

        for (int x = 0; x < (int)width; x += x_step) {
            const uint8_t value = rgb565_luma(frame +
                                              (((size_t)y * (size_t)width + (size_t)x) * 2));
            if (value <= threshold) {
                if (run_samples == 0) {
                    run_start = x;
                }
                ++run_samples;
                ++dark_samples;
                dark_sum += x;
            } else if (run_samples > 0) {
                consider_run(run_start, run_samples, x_step, width, expected_x,
                             &best_score, &best_center, &best_width);
                run_samples = 0;
            }
        }
        if (run_samples > 0) {
            consider_run(run_start, run_samples, x_step, width, expected_x,
                         &best_score, &best_center, &best_width);
        }

        if (dark_samples * 100 >= total_row_samples * LINE_FINISH_ROW_COVERAGE_PERCENT) {
            ++wide_rows;
            if (dark_samples > 0) {
                wide_center_sum += dark_sum / dark_samples;
            }
        }
        if (best_score >= 0 && best_width > 0) {
            const int weight = y - top + 1;
            weighted_center += (int64_t)best_center * weight;
            weight_sum += weight;
            ++valid_rows;
        }
    }

    observation->threshold = threshold;
    observation->finish_candidate = wide_rows >= LINE_FINISH_MIN_ROWS &&
                                     (wide_center_sum / (wide_rows ? wide_rows : 1)) >=
                                         (int)width / 5 &&
                                     (wide_center_sum / (wide_rows ? wide_rows : 1)) <=
                                         (int)width * 4 / 5;
    observation->confidence = (uint8_t)((valid_rows * 100) /
                                        ((bottom - top) / y_step + 1));
    if (observation->confidence > 100) {
        observation->confidence = 100;
    }
    if (valid_rows < LINE_MIN_VALID_ROWS || weight_sum == 0) {
        observation->valid = false;
        return false;
    }

    const int center_x = (int)(weighted_center / weight_sum);
    int error = (center_x - (int)width / 2) * 100 / ((int)width / 2);
    error = clamp_int(error, 100);
#if CAMERA_LINE_MIRROR_X
    error = -error;
#endif
    observation->error = error;
    observation->valid = true;
    return true;
}

static void set_motor_duty(const motor_t *motor, int output)
{
    if (output < 0) {
        output = 0;
    }
    if (output > MAX_OUTPUT) {
        output = MAX_OUTPUT;
    }
    const uint32_t duty = PWM_MAX * (uint32_t)output / 100U;
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty);
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

static void motor_set(const motor_t *motor, int command)
{
    const int index = (int)motor->channel;
    int speed = clamp_int(command * motor->sign, MAX_OUTPUT);
    const int direction = (speed > 0) - (speed < 0);
    const bool direction_changed = direction != s_last_direction[index];

    if (direction_changed) {
        set_motor_duty(motor, 0);
        gpio_set_level(motor->in1, speed > 0);
        gpio_set_level(motor->in2, speed < 0);
        s_last_direction[index] = direction;
        s_kick_cycles[index] = direction == 0 ? 0 : START_KICK_CYCLES;
    }

    if (direction == 0) {
        s_kick_cycles[index] = 0;
        gpio_set_level(motor->in1, 0);
        gpio_set_level(motor->in2, 0);
        set_motor_duty(motor, 0);
        return;
    }

    int output = abs(speed);
    const int minimum = motor->channel == LEDC_CHANNEL_1 ?
                        MOTOR_B_MIN_RUN_OUTPUT : MOTOR_MIN_RUN_OUTPUT;
    if (output < minimum) {
        output = minimum;
    }
    if (s_kick_cycles[index] > 0 &&
        (motor->channel != LEDC_CHANNEL_1 || output >= MOTOR_B_KICK_MIN_OUTPUT) &&
        output < START_KICK_OUTPUT) {
        output = START_KICK_OUTPUT;
        --s_kick_cycles[index];
    }
    set_motor_duty(motor, output);
}

static void zero_motor_outputs(void)
{
    motor_set(&motor_a, 0);
    motor_set(&motor_b, 0);
    motor_set(&motor_d, 0);
    s_command_a = 0;
    s_command_b = 0;
    s_command_d = 0;
}

static void stop_motors(void)
{
    zero_motor_outputs();
    gpio_set_level(STBY_GPIO, 0);
}

static void drive(int forward, int turn)
{
    s_command_a = clamp_int(-forward - turn, MAX_OUTPUT);
    s_command_b = clamp_int(turn, MAX_OUTPUT);
    s_command_d = clamp_int(forward - turn, MAX_OUTPUT);
    motor_set(&motor_a, s_command_a);
    motor_set(&motor_b, s_command_b);
    motor_set(&motor_d, s_command_d);
}

static void drive_spin(int direction)
{
    s_command_a = clamp_int(-direction * LINE_TURN_MAX, MAX_OUTPUT);
    s_command_b = clamp_int(direction * (LINE_TURN_MAX - 4), MAX_OUTPUT);
    s_command_d = clamp_int(-direction * LINE_TURN_MAX, MAX_OUTPUT);
    motor_set(&motor_a, s_command_a);
    motor_set(&motor_b, s_command_b);
    motor_set(&motor_d, s_command_d);
}

static void reset_pid(void)
{
    s_pid_integral = 0;
    s_pid_previous = 0;
    s_pid_derivative = 0;
    s_pid_output = 0;
}

static int pid_steering(int error)
{
    if (abs(error) <= LINE_PID_DEADBAND) {
        reset_pid();
        return 0;
    }

    s_pid_integral = clamp_int(s_pid_integral + error, LINE_PID_INTEGRAL_LIMIT);
    const int derivative = error - s_pid_previous;
    s_pid_derivative = (s_pid_derivative + derivative) / 2;
    s_pid_previous = error;

    int output = -(LINE_PID_KP * error +
                   LINE_PID_KI * s_pid_integral +
                   LINE_PID_KD * s_pid_derivative) / LINE_PID_SCALE;
    output = clamp_int(output, LINE_TURN_MAX);
    if (abs(error) >= 70) {
        s_pid_output = output;
        return output;
    }
    s_pid_output = (s_pid_output * 2 + output) / 3;
    return s_pid_output;
}

static int forward_speed(int error, int turn)
{
    const int magnitude = abs(error);
    if (magnitude >= 80 || abs(turn) >= 26) {
        return LINE_FORWARD_CRAWL;
    }
    if (magnitude >= 55 || abs(turn) >= 18) {
        return LINE_FORWARD_SLOW;
    }
    if (magnitude >= 28 || abs(turn) >= 10) {
        return LINE_FORWARD_MEDIUM;
    }
    return LINE_FORWARD_FAST;
}

static void log_state(const char *mode, const line_observation_t *observation)
{
    const int64_t now = esp_timer_get_time();
    if (s_last_log_us != 0 && now - s_last_log_us < 500000) {
        return;
    }
    s_last_log_us = now;
    ESP_LOGI(TAG, "mode=%s line=%d err=%d conf=%u threshold=%d motor[A,B,D]=[%d,%d,%d]",
             mode,
             observation != NULL && observation->valid,
             observation != NULL && observation->valid ? observation->error : s_last_error,
             (unsigned)(observation != NULL ? observation->confidence : s_last_confidence),
             observation != NULL ? observation->threshold : s_last_threshold,
             s_command_a, s_command_b, s_command_d);
}

esp_err_t camera_line_follow_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    if (s_control_mutex == NULL) {
        s_control_mutex = xSemaphoreCreateMutex();
        if (s_control_mutex == NULL) {
            ESP_LOGE(TAG, "Could not allocate control mutex");
            return ESP_ERR_NO_MEM;
        }
    }

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
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer init failed: %s", esp_err_to_name(err));
        return err;
    }

    const motor_t *motors[] = {&motor_a, &motor_b, &motor_d};
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = i == 0 ? A_PWM : (i == 1 ? B_PWM : D_PWM),
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i]->channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        err = ledc_channel_config(&channel);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LEDC channel %u init failed: %s",
                     (unsigned)i, esp_err_to_name(err));
            return err;
        }
    }

    s_started = true;
    s_armed = false;
    s_finished = false;
    s_arm_frames = 0;
    s_finish_frames = 0;
    s_first_frame_us = 0;
    s_last_line_us = 0;
    s_last_frame_us = 0;
    s_last_log_us = 0;
    s_last_error = 0;
    s_last_threshold = 0;
    s_last_confidence = 0;
    s_last_direction[0] = 0;
    s_last_direction[1] = 0;
    s_last_direction[2] = 0;
    s_kick_cycles[0] = 0;
    s_kick_cycles[1] = 0;
    s_kick_cycles[2] = 0;
    reset_pid();
    gpio_set_level(STBY_GPIO, 0);
    stop_motors();

    if (!s_watchdog_created) {
        if (xTaskCreate(camera_line_follow_watchdog_task, "camera_line_wd", 3072,
                        NULL, 2, NULL) != pdPASS) {
            s_started = false;
            ESP_LOGE(TAG, "Could not create camera line watchdog task");
            return ESP_ERR_NO_MEM;
        }
        s_watchdog_created = true;
    }
    ESP_LOGI(TAG, "Camera line follower ready; motors stay stopped until a stable line is seen");
    return ESP_OK;
}

void camera_line_follow_stop(void)
{
    if (!s_started) {
        return;
    }
    if (s_control_mutex != NULL) {
        (void)xSemaphoreTake(s_control_mutex, portMAX_DELAY);
    }
    stop_motors();
    s_started = false;
    s_last_frame_us = 0;
    if (s_control_mutex != NULL) {
        (void)xSemaphoreGive(s_control_mutex);
    }
}

static void camera_line_follow_process_frame(const uint8_t *rgb565_big_endian,
                                             uint16_t width,
                                             uint16_t height)
{
    if (!s_started || s_finished) {
        if (s_started) {
            stop_motors();
        }
        return;
    }

    const int64_t now = esp_timer_get_time();
    if (s_first_frame_us == 0) {
        s_first_frame_us = now;
    }

    line_observation_t observation = {0};
    const bool line_seen = observe_line(rgb565_big_endian, width, height, &observation);
    s_last_threshold = observation.threshold;
    s_last_confidence = observation.confidence;

    if (line_seen) {
        s_last_line_us = now;
        s_last_error = observation.error;
    }

    if (!s_armed) {
        if (line_seen && now - s_first_frame_us >= (int64_t)LINE_START_DELAY_MS * 1000) {
            if (s_arm_frames < LINE_ARM_CONFIRM_FRAMES) {
                ++s_arm_frames;
            }
            if (s_arm_frames >= LINE_ARM_CONFIRM_FRAMES) {
                s_armed = true;
                reset_pid();
                gpio_set_level(STBY_GPIO, 1);
                ESP_LOGI(TAG, "Line confirmed; camera steering enabled");
            }
        } else if (!line_seen) {
            s_arm_frames = 0;
        }
        if (!s_armed) {
            stop_motors();
            log_state("ARM", &observation);
            return;
        }
        zero_motor_outputs();
        log_state("ARMED", &observation);
        return;
    }

    if (observation.finish_candidate) {
        if (s_finish_frames < LINE_FINISH_CONFIRM_FRAMES) {
            ++s_finish_frames;
        }
        if (s_finish_frames >= LINE_FINISH_CONFIRM_FRAMES) {
            s_finished = true;
            stop_motors();
            ESP_LOGW(TAG, "Finish bar confirmed; motors stopped");
            return;
        }
    } else {
        s_finish_frames = 0;
    }

    if (line_seen) {
        const int turn = pid_steering(observation.error);
        drive(forward_speed(observation.error, turn), turn);
        log_state("LINE", &observation);
        return;
    }

    reset_pid();
    const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
    if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000) {
        const int turn = s_last_error < 0 ? LINE_TURN_MAX / 2 : -LINE_TURN_MAX / 2;
        drive(LINE_FORWARD_CRAWL, turn);
        log_state("LOST-HOLD", &observation);
    } else if (lost_us <= (int64_t)LINE_LOST_SEARCH_MS * 1000) {
        drive_spin(s_last_error < 0 ? 1 : -1);
        log_state("SEARCH", &observation);
    } else {
        stop_motors();
        log_state("LOST-STOP", &observation);
        if (lost_us >= (int64_t)LINE_LOST_STOP_MS * 1000) {
            s_armed = false;
            s_arm_frames = 0;
            s_first_frame_us = 0;
            gpio_set_level(STBY_GPIO, 0);
        }
    }
}

static void camera_line_follow_watchdog_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!s_started || s_control_mutex == NULL) {
            continue;
        }
        const int64_t now = esp_timer_get_time();
        if (xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }
        if (s_started && s_last_frame_us != 0 &&
            now - s_last_frame_us > (int64_t)LINE_FRAME_TIMEOUT_MS * 1000) {
            stop_motors();
            s_armed = false;
            s_arm_frames = 0;
            s_first_frame_us = 0;
            s_last_line_us = 0;
            s_last_frame_us = 0;
            reset_pid();
            ESP_LOGW(TAG, "Decoded-frame timeout; motors stopped and line confirmation reset");
        }
        (void)xSemaphoreGive(s_control_mutex);
    }
}

void camera_line_follow_frame_callback(const uint8_t *rgb565_big_endian,
                                       uint16_t width,
                                       uint16_t height,
                                       void *user_ctx)
{
    (void)user_ctx;
    if (!s_started) {
        return;
    }
    if (s_control_mutex != NULL &&
        xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    s_last_frame_us = esp_timer_get_time();
    camera_line_follow_process_frame(rgb565_big_endian, width, height);
    if (s_control_mutex != NULL) {
        (void)xSemaphoreGive(s_control_mutex);
    }
}
