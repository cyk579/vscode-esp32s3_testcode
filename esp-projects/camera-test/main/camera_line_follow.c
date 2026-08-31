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

/* 电机引脚与已经校准过的 car-spin 工程保持一致。 */
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

/* 解码器输出的是大端字节序 RGB565；画面左右方向若相反，把它改为 1。 */
#define CAMERA_LINE_MIRROR_X 0

/*
 * 图像坐标的 y 轴向下，越靠近 LINE_ROI_BOTTOM 越接近小车。
 * 只在下方的“近端带”控制方向；终点横条也要进入中近景才确认。
 * 摄像头最底部可能被车体遮挡，所以先跳过一行，再从下往上找连续黑线。
 */
#define LINE_ROI_TOP_PERCENT 45
#define LINE_ROI_BOTTOM_PERCENT 95
#define LINE_CONTROL_TOP_PERCENT 65
#define LINE_CONTROL_BOTTOM_PERCENT 90
#define LINE_ROW_STEP 4
#define LINE_BOTTOM_SKIP_ROWS 1
#define LINE_NEAR_BAND_ROWS 6
#define LINE_ANCHOR_SEARCH_ROWS 4
#define LINE_NEAR_ROWS 4
#define LINE_MAX_CENTER_STEP_PERCENT 14
#define LINE_MAX_ERROR_STEP 45
#define LINE_MIN_CONTRAST 32
#define LINE_BLACK_FRACTION_PERCENT 30
#define LINE_BLACK_THRESHOLD_MIN 35
#define LINE_BLACK_THRESHOLD_MAX 120
#define LINE_MIN_RUN_SAMPLES 3
#define LINE_MAX_RUN_PERCENT 45
#define LINE_MIN_VALID_ROWS 3
#define LINE_FINISH_ROW_COVERAGE_PERCENT 50
#define LINE_FINISH_MIN_ROWS 3

/* 启动、丢线和终点确认均使用连续帧，避免单帧噪声直接驱动车体。 */
#define LINE_START_DELAY_MS 600U
#define LINE_ARM_CONFIRM_FRAMES 3U
#define LINE_LOST_HOLD_MS 160U
#define LINE_LOST_SEARCH_MS 520U
#define LINE_LOST_STOP_MS 900U
#define LINE_FRAME_TIMEOUT_MS 450U
#define LINE_FINISH_CONFIRM_FRAMES 5U

#define LINE_FORWARD_FAST 22  /* 红外直行 30，摄像头再降 8。 */
#define LINE_FORWARD_MEDIUM 14 /* 红外弯道 22，摄像头再降 8。 */
#define LINE_FORWARD_SLOW 9
#define LINE_FORWARD_CRAWL 7
#define LINE_TURN_MAX 15
#define LINE_ERROR_DEADBAND 18
#define LINE_ERROR_MEDIUM 35
#define LINE_ERROR_LARGE 60
#define LINE_PID_KP 15
#define LINE_PID_KI 1
#define LINE_PID_KD 3
#define LINE_PID_SCALE 100
#define LINE_PID_INTEGRAL_LIMIT 60
#define LINE_PID_SLEW_PER_FRAME 4
#define LINE_PID_OUTPUT_DEADBAND 1
#define LINE_ERROR_FILTER_OLD 2
#define LINE_ERROR_FILTER_NEW 1

#define PWM_MAX 1023U
#define MOTOR_MIN_RUN_OUTPUT 8
#define MOTOR_B_MIN_RUN_OUTPUT 0
#define START_KICK_OUTPUT 18
#define START_KICK_CYCLES 3U
#define MAX_OUTPUT 26

/* 方向符号与 car-spin 的实车校准一致。 */
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
    bool rejected_jump; /* 本帧像素中心跳变，按丢线处理而不是立即转向。 */
    int error;
    int threshold;
    uint8_t confidence;
    uint8_t track_rows; /* 实际参与 PID 的近端行数，串口调试用。 */
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
static int s_tracking_error;
static bool s_line_history_valid;
static bool s_error_filter_initialized;
static int s_pid_integral;
static int s_pid_previous_error;
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

/*
 * 把一段连续黑像素作为候选线段。距离期望位置太远的段不进入当前行，
 * 这样从近端向上跟踪时，远处突然出现的另一条弯线会被断开。
 */
static void consider_run(int start_x,
                         int sample_count,
                         int sample_step,
                         int width,
                         int expected_x,
                         int max_center_delta,
                         int *best_distance,
                         int *best_center,
                         int *best_width)
{
    if (sample_count < LINE_MIN_RUN_SAMPLES) {
        return;
    }

    const int run_width = sample_count * sample_step;
    if (run_width > width * LINE_MAX_RUN_PERCENT / 100) {
        return; /* 宽黑块留给终点判断，不拿来驱动转向。 */
    }

    const int end_x = start_x + run_width - sample_step;
    const int center_x = (start_x + end_x) / 2;
    const int distance = abs(center_x - expected_x);
    if (max_center_delta >= 0 && distance > max_center_delta) {
        return;
    }

    /* 先选离上一行最近的黑段；距离相同才比较宽度，避免噪点抢走主线。 */
    if (distance < *best_distance ||
        (distance == *best_distance && run_width > *best_width)) {
        *best_distance = distance;
        *best_center = center_x;
        *best_width = run_width;
    }
}

static bool scan_row(const uint8_t *frame,
                     uint16_t width,
                     int y,
                     int threshold,
                     int expected_x,
                     int max_center_delta,
                     int *center,
                     int *run_width,
                     int *dark_samples)
{
    const int x_step = width >= 96 ? 2 : 1;
    int best_distance = (int)width + 1;
    int best_center = 0;
    int best_width = 0;
    int run_start = 0;
    int run_count = 0;
    int dark_sum = 0;
    *center = 0;
    *run_width = 0;
    *dark_samples = 0;

    for (int x = 0; x < (int)width; x += x_step) {
        const uint8_t value = rgb565_luma(frame +
                                          (((size_t)y * (size_t)width + (size_t)x) * 2));
        if (value <= threshold) {
            ++*dark_samples;
            dark_sum += x;
            if (run_count == 0) {
                run_start = x;
            }
            ++run_count;
            continue;
        }

        if (run_count > 0) {
            consider_run(run_start, run_count, x_step, width, expected_x,
                         max_center_delta, &best_distance, &best_center, &best_width);
            run_count = 0;
        }
    }
    if (run_count > 0) {
        consider_run(run_start, run_count, x_step, width, expected_x,
                     max_center_delta, &best_distance, &best_center, &best_width);
    }

    /* 即使黑块太宽，也保留其重心给终点检测；控制逻辑会因 best_width=0 忽略它。 */
    if (*dark_samples > 0) {
        *center = dark_sum / *dark_samples;
    }
    if (best_width == 0) {
        return false;
    }
    *center = best_center;
    *run_width = best_width;
    return true;
}

static bool observe_line(const uint8_t *frame,
                         uint16_t width,
                         uint16_t height,
                         line_observation_t *observation)
{
    if (frame == NULL || observation == NULL || width < 16 || height < 16) {
        return false;
    }
    *observation = (line_observation_t){0};

    int roi_top = (int)height * LINE_ROI_TOP_PERCENT / 100;
    int roi_bottom = (int)height * LINE_ROI_BOTTOM_PERCENT / 100;
    if (roi_top < 0) {
        roi_top = 0;
    }
    if (roi_bottom >= (int)height) {
        roi_bottom = (int)height - 1;
    }
    if (roi_bottom <= roi_top) {
        return false;
    }

    int control_top = (int)height * LINE_CONTROL_TOP_PERCENT / 100;
    int control_bottom = (int)height * LINE_CONTROL_BOTTOM_PERCENT / 100;
    if (control_top < roi_top) {
        control_top = roi_top;
    }
    if (control_bottom > roi_bottom) {
        control_bottom = roi_bottom;
    }
    if (control_bottom <= control_top) {
        return false;
    }

    const int x_step = width >= 96 ? 2 : 1;
    const int y_step = LINE_ROW_STEP;

    /* 阈值在裁剪后的 ROI 内估计；远处像素只影响黑白分界，不会进入转向中心。 */
    int minimum = 255;
    int maximum = 0;
    int sample_total = 0;
    for (int y = roi_top; y <= roi_bottom; y += y_step) {
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
    observation->threshold = threshold;

    /* 终点横黑条也只在中近景确认；远处弯道的横向黑段不能提前停车。 */
    int wide_rows = 0;
    int wide_center_sum = 0;
    const int total_row_samples = ((int)width + x_step - 1) / x_step;
    int finish_top = control_bottom -
                     (LINE_NEAR_BAND_ROWS + LINE_BOTTOM_SKIP_ROWS - 1) * y_step;
    if (finish_top < control_top) {
        finish_top = control_top;
    }
    for (int y = control_bottom; y >= finish_top; y -= y_step) {
        int center = 0;
        int run_width = 0;
        int dark_samples = 0;
        (void)scan_row(frame, width, y, threshold, (int)width / 2, -1,
                       &center, &run_width, &dark_samples);
        if (dark_samples * 100 >= total_row_samples * LINE_FINISH_ROW_COVERAGE_PERCENT) {
            ++wide_rows;
            wide_center_sum += center;
        }
    }
    const int finish_center = wide_rows > 0 ? wide_center_sum / wide_rows : (int)width / 2;
    observation->finish_candidate = wide_rows >= LINE_FINISH_MIN_ROWS &&
                                     finish_center >= (int)width / 5 &&
                                     finish_center <= (int)width * 4 / 5;

    /*
     * 近端跟踪：从控制区底部略过遮挡行后，只看固定的 6 行。
     * 第一条有效黑段作为锚点；之后每一行必须与上一行连续，否则立即停止，
     * 因而远处锐角/直角支路不会被“跨空”接入中心计算。
     */
    int start_y = control_bottom;
    /* 控制区很窄时不要为了跳过遮挡而牺牲最少的 3 行有效样本。 */
    if (control_bottom - control_top >=
        (LINE_MIN_VALID_ROWS + LINE_BOTTOM_SKIP_ROWS - 1) * y_step) {
        start_y -= LINE_BOTTOM_SKIP_ROWS * y_step;
    }
    int near_top = start_y - (LINE_NEAR_BAND_ROWS - 1) * y_step;
    if (near_top < control_top) {
        near_top = control_top;
    }
    int band_rows = 0;
    for (int y = start_y; y >= near_top; y -= y_step) {
        ++band_rows;
    }

    int expected_x = (int)width / 2;
    if (s_line_history_valid) {
        int image_error = s_last_error;
#if CAMERA_LINE_MIRROR_X
        image_error = -image_error;
#endif
        expected_x += image_error * (int)width / 200;
    }
    if (expected_x < 0) {
        expected_x = 0;
    }
    if (expected_x >= (int)width) {
        expected_x = (int)width - 1;
    }
    int center_step_limit = (int)width * LINE_MAX_CENTER_STEP_PERCENT / 100;
    if (center_step_limit < 4) {
        center_step_limit = 4;
    }

    int center_sum = 0;
    int center_weight = 0;
    int valid_rows = 0;
    int anchor_attempts = 0;
    int previous_center = expected_x;
    bool anchored = false;
    bool continuity_rejected = false;
    for (int y = start_y; y >= near_top; y -= y_step) {
        int center = 0;
        int run_width = 0;
        int dark_samples = 0;
        /* 有上一帧时锚点也不能瞬移；首次上电则允许从画面中心重新捕获。 */
        const int max_delta = anchored ? center_step_limit :
                              (s_line_history_valid ? center_step_limit * 2 : -1);
        const bool has_run = scan_row(frame, width, y, threshold, previous_center, max_delta,
                                      &center, &run_width, &dark_samples);
        if (!anchored) {
            ++anchor_attempts;
            if (!has_run) {
                continuity_rejected = s_line_history_valid;
                if (anchor_attempts >= LINE_ANCHOR_SEARCH_ROWS) {
                    break;
                }
                continue;
            }
            anchored = true;
        } else if (!has_run) {
            continuity_rejected = s_line_history_valid;
            break;
        }

        if (valid_rows >= LINE_NEAR_ROWS) {
            break;
        }
        const int weight = LINE_NEAR_ROWS - valid_rows;
        center_sum += center * weight;
        center_weight += weight;
        previous_center = center;
        ++valid_rows;
    }

    const int required_rows = LINE_NEAR_ROWS < band_rows ? LINE_NEAR_ROWS : band_rows;
    observation->track_rows = (uint8_t)valid_rows;
    observation->confidence = (uint8_t)(valid_rows * 100 / (required_rows ? required_rows : 1));
    if (observation->confidence > 100) {
        observation->confidence = 100;
    }
    if (valid_rows < LINE_MIN_VALID_ROWS || center_weight == 0) {
        observation->rejected_jump = continuity_rejected;
        return false;
    }

    const int center_x = center_sum / center_weight;
    int error = (center_x - (int)width / 2) * 100 / ((int)width / 2);
    error = clamp_int(error, 100);
#if CAMERA_LINE_MIRROR_X
    error = -error;
#endif

    /* 一帧若突然跳出很大误差，先当作远端支路/噪声，交给丢线保持逻辑。 */
    if (s_line_history_valid && abs(error - s_last_error) > LINE_MAX_ERROR_STEP) {
        observation->rejected_jump = true;
        return false;
    }
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
    /* command 是混控层的有符号 PWM；换向时先清零，避免 TB6612 硬切方向。 */
    const int index = (int)motor->channel;
    int speed = clamp_int(command * motor->sign, MAX_OUTPUT);
    const int direction = (speed > 0) - (speed < 0);
    const bool direction_changed = direction != s_last_direction[index];

    if (direction_changed) {
        set_motor_duty(motor, 0);
        gpio_set_level(motor->in1, speed > 0);
        gpio_set_level(motor->in2, speed < 0);
        s_last_direction[index] = direction;
        s_kick_cycles[index] = (direction == 0 || motor->channel == LEDC_CHANNEL_1) ?
                              0 : START_KICK_CYCLES;
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
    if (s_kick_cycles[index] > 0 && output < START_KICK_OUTPUT) {
        output = START_KICK_OUTPUT;
    }
    if (s_kick_cycles[index] > 0) {
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
    /* 直行向量固定为 A=-forward、D=forward、B=0；只有 turn 非零才动后轮 B。 */
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

static void reset_tracking(void)
{
    s_tracking_error = 0;
    s_line_history_valid = false;
    s_error_filter_initialized = false;
    s_pid_integral = 0;
    s_pid_previous_error = 0;
    s_pid_output = 0;
}

static int pid_steering(int error)
{
    /* 低精度位置式 PID：先做整数低通，再用死区、积分限幅和斜率限幅。 */
    /* error>0 表示黑线在画面右侧，当前电机校准要求输出相反方向的修正。 */
    if (!s_error_filter_initialized) {
        s_tracking_error = error;
        s_pid_previous_error = error;
        s_error_filter_initialized = true;
    } else {
        s_tracking_error = (s_tracking_error * LINE_ERROR_FILTER_OLD +
                            error * LINE_ERROR_FILTER_NEW) /
                           (LINE_ERROR_FILTER_OLD + LINE_ERROR_FILTER_NEW);
    }

    const int magnitude = abs(s_tracking_error);
    if (magnitude <= LINE_ERROR_DEADBAND) {
        s_pid_integral = 0;
        s_pid_previous_error = s_tracking_error;
        s_pid_output = 0;
        return 0;
    }

    if ((s_tracking_error > 0 && s_pid_integral < 0) ||
        (s_tracking_error < 0 && s_pid_integral > 0)) {
        s_pid_integral = 0;
    }
    s_pid_integral = clamp_int(s_pid_integral + s_tracking_error,
                               LINE_PID_INTEGRAL_LIMIT);
    const int derivative = s_tracking_error - s_pid_previous_error;
    s_pid_previous_error = s_tracking_error;

    int target = -(LINE_PID_KP * s_tracking_error +
                   LINE_PID_KI * s_pid_integral +
                   LINE_PID_KD * derivative) /
                 LINE_PID_SCALE;
    target = clamp_int(target, LINE_TURN_MAX);
    const int delta = target - s_pid_output;
    if (delta > LINE_PID_SLEW_PER_FRAME) {
        target = s_pid_output + LINE_PID_SLEW_PER_FRAME;
    } else if (delta < -LINE_PID_SLEW_PER_FRAME) {
        target = s_pid_output - LINE_PID_SLEW_PER_FRAME;
    }
    s_pid_output = target;
    if (abs(s_pid_output) <= LINE_PID_OUTPUT_DEADBAND) {
        s_pid_output = 0;
        return 0;
    }
    return s_pid_output;
}

static int forward_speed(int error)
{
    const int magnitude = abs(error);
    if (magnitude >= LINE_ERROR_LARGE) {
        return LINE_FORWARD_CRAWL;
    }
    if (magnitude >= LINE_ERROR_MEDIUM) {
        return LINE_FORWARD_SLOW;
    }
    if (magnitude > LINE_ERROR_DEADBAND) {
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
    ESP_LOGI(TAG, "mode=%s line=%d jump=%d err=%d filt=%d rows=%u conf=%u threshold=%d motor[A,B,D]=[%d,%d,%d]",
             mode,
             observation != NULL && observation->valid,
             observation != NULL && observation->rejected_jump,
             observation != NULL && observation->valid ? observation->error : s_last_error,
             s_tracking_error,
             (unsigned)(observation != NULL ? observation->track_rows : 0),
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
    reset_tracking();
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
        s_line_history_valid = true;
    }

    /* 先确认画面稳定，再允许 STBY；复位后不会因为一帧误检突然起步。 */
    if (!s_armed) {
        if (line_seen && now - s_first_frame_us >= (int64_t)LINE_START_DELAY_MS * 1000) {
            if (s_arm_frames < LINE_ARM_CONFIRM_FRAMES) {
                ++s_arm_frames;
            }
            if (s_arm_frames >= LINE_ARM_CONFIRM_FRAMES) {
                const int confirmed_error = observation.error;
                s_armed = true;
                reset_tracking();
                /* 重置 PID 后保留刚确认的图像位置，防止起步第一帧跳到远处支路。 */
                s_last_error = confirmed_error;
                s_line_history_valid = true;
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
        drive(forward_speed(s_tracking_error), turn);
        log_state("LINE", &observation);
        return;
    }

    /* 丢线时先保持最后方向低速走，再原地搜索；超过时限直接停车。 */
    const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
    const int search_direction = s_tracking_error < 0 ? 1 : -1;
    if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000) {
        /* 跳变拒绝时保持直行，避免远处支路在第一帧触发旧方向修正。 */
        const int turn = observation.rejected_jump ? 0 :
                         search_direction * (LINE_TURN_MAX / 2);
        drive(LINE_FORWARD_CRAWL, turn);
        log_state(observation.rejected_jump ? "JUMP-HOLD" : "LOST-HOLD", &observation);
    } else if (lost_us <= (int64_t)LINE_LOST_SEARCH_MS * 1000) {
        drive_spin(search_direction);
        log_state("SEARCH", &observation);
    } else {
        stop_motors();
        log_state("LOST-STOP", &observation);
        if (lost_us >= (int64_t)LINE_LOST_STOP_MS * 1000) {
            s_armed = false;
            s_arm_frames = 0;
            s_first_frame_us = 0;
            reset_tracking();
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
            reset_tracking();
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
