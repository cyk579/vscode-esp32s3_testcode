#include "camera_line_follow.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "camera_display.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "line_geometry.h"

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

/* 解码器输出大端字节序 RGB565；画面左右相反时改为 1。 */
#define CAMERA_LINE_MIRROR_X 0

/* ROI、行距、逐行搜索和线段判据全部在 line_geometry.h 里，ESP 端和 host
 * 回归测试共用同一组常量。这里只保留状态机自己的窗口策略和时序。 */
#define LINE_LOST_WINDOW_GROW_PERCENT 4
#define LINE_LOST_WINDOW_MAX_PERCENT 42
#define LINE_CORNER_WINDOW_START_PERCENT 28
#define LINE_CORNER_WINDOW_GROW_PERCENT 4
#define LINE_CORNER_WINDOW_MAX_PERCENT 46
#define LINE_CORNER_PREDICT_STEP_PERCENT 4
#define LINE_BRANCH_CONFIRM_FRAMES 3U
#define LINE_PENDING_MISS_CONFIRM_FRAMES 2U
#define LINE_REACQUIRE_CONFIRM_FRAMES 2U
#define LINE_CORNER_EXIT_CONFIRM_FRAMES 3U
#define LINE_CORNER_MIN_SHIFT_PERCENT 8

/* 彩色干扰守卫：赛道板上的深色红球亮度会落进黑线阈值区间，但通道差很大。
 * 真机采帧确认需要之后再开启，默认关闭以免引入未验证的过滤。 */
#define LINE_SATURATION_GUARD 0
#define LINE_SATURATION_MAX 60
#define LINE_WIDTH_FILTER_OLD 3
#define LINE_WIDTH_FILTER_NEW 1

/* 这些值保留原有的上电、限速、换向保护和超时停车行为。 */
#define LINE_START_DELAY_MS 600U
#define LINE_ARM_CONFIRM_FRAMES 3U
#define LINE_MOTOR_START_RAMP_FRAMES 18U
#define LINE_MOTOR_START_MIN_OUTPUT 12
#define LINE_LOST_HOLD_MS 300U
#define LINE_LOST_STOP_MS 900U
#define LINE_FRAME_TIMEOUT_MS 1200U
#define LINE_CORNER_TIMEOUT_MS 3000U
#define LINE_FINISH_CONFIRM_FRAMES 5U

#define LINE_FORWARD_FAST 30
#define LINE_FORWARD_MEDIUM 22
#define LINE_FORWARD_SLOW 22
#define LINE_FORWARD_CRAWL 17
#define LINE_TURN_MAX 19
#define LINE_PID_TURN_MAX 10
#define LINE_ERROR_DEADBAND 18
#define LINE_ERROR_MEDIUM 35
#define LINE_ERROR_LARGE 60
#define LINE_PID_KP 12
#define LINE_PID_KH 8
#define LINE_PID_KD 4
#define LINE_PID_SCALE 100
#define LINE_PID_SLEW_PER_FRAME 2
#define LINE_PID_OUTPUT_DEADBAND 1
#define LINE_ERROR_FILTER_OLD 3
#define LINE_ERROR_FILTER_NEW 1

/* 起转下限、启动冲量和方向符号全部复用 car-spin 的实车校准值。红外巡线
 * 工程是在同一台三轮全向车上测过的，这里不要再自行下调：B 轮实测起转
 * 需要 13，低于该值它完全不转，而卡死的后轮会把旋转中心从几何中心挪到
 * 后轮接地点，摄像头的横向扫过量随之放大。 */
#define PWM_MAX 1023U
#define MOTOR_MIN_RUN_OUTPUT 11
#define MOTOR_B_MIN_RUN_OUTPUT 13
#define START_KICK_OUTPUT 32
#define START_KICK_CYCLES 8U
#define MAX_OUTPUT 34

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

typedef enum {
    LINE_STATE_NORMAL = 0,
    LINE_STATE_CORNER,
    LINE_STATE_LOST,
} line_state_t;

static const char *TAG = "camera_line";
static const motor_t motor_a = {A_IN1, A_IN2, LEDC_CHANNEL_0, MOTOR_A_SIGN};
static const motor_t motor_b = {B_IN1, B_IN2, LEDC_CHANNEL_1, MOTOR_B_SIGN};
static const motor_t motor_d = {D_IN1, D_IN2, LEDC_CHANNEL_2, MOTOR_D_SIGN};

static volatile bool s_started;
static bool s_armed;
static bool s_finished;
static bool s_stby_enabled;
static uint8_t s_arm_frames;
static uint8_t s_motor_start_ramp_frames;
static uint8_t s_finish_frames;
static int64_t s_first_frame_us;
static int64_t s_last_line_us;
static int64_t s_last_frame_us;

static line_state_t s_state;
static bool s_seed_valid;
static int s_seed_x;
static int s_line_width;
static uint32_t s_lost_frames;
static uint8_t s_reacquire_frames;
static int s_last_lateral_error;
static int s_last_heading_error;
static uint8_t s_last_valid_rows;
static uint8_t s_last_confidence;
static int s_last_threshold;
static bool s_last_candidate;
static int s_last_seed_x;

static bool s_error_filter_initialized;
static int s_tracking_error;
static int s_previous_tracking_error;
static int s_pid_output;

static int s_pending_turn;
static int s_pending_seed_x;
static uint8_t s_pending_miss_frames;
static int s_hint_direction;
static uint8_t s_hint_frames;
static uint8_t s_corner_frames;
static uint8_t s_corner_exit_frames;
static int s_corner_origin_x;
static int s_corner_predicted_x;
static int64_t s_corner_started_us;
static int s_reacquire_x;

static int s_command_a;
static int s_command_b;
static int s_command_d;
static int s_last_direction[3];
static uint8_t s_kick_cycles[3];

static uint32_t s_control_frames;
static uint32_t s_line_us_sum;
static uint32_t s_line_us_max;
static int64_t s_summary_start_us;
static uint32_t s_summary_camera_frames;
static uint32_t s_summary_processed_frames;
static uint32_t s_summary_dropped_frames;
static uint32_t s_callback_dropped_frames;
static uint32_t s_summary_callback_dropped_frames;

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

static int clamp_range(int value, int low, int high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static int direction_of(int value)
{
    return (value > 0) - (value < 0);
}

static int positive_percent(int value, int percent, int minimum)
{
    return line_geometry_positive_percent(value, percent, minimum);
}

static void overlay_pixel(uint8_t *frame,
                          uint16_t width,
                          uint16_t height,
                          int x,
                          int y,
                          uint16_t color)
{
    if (frame == NULL || x < 0 || y < 0 || x >= (int)width || y >= (int)height) {
        return;
    }
    uint8_t *pixel = frame + (((size_t)y * width + (size_t)x) * 2);
    pixel[0] = (uint8_t)(color >> 8);
    pixel[1] = (uint8_t)color;
}

static void overlay_dot(uint8_t *frame,
                        uint16_t width,
                        uint16_t height,
                        int x,
                        int y,
                        uint16_t color)
{
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            overlay_pixel(frame, width, height, x + dx, y + dy, color);
        }
    }
}

static void overlay_cross(uint8_t *frame,
                          uint16_t width,
                          uint16_t height,
                          int x,
                          int y,
                          uint16_t color)
{
    for (int offset = -5; offset <= 5; ++offset) {
        overlay_pixel(frame, width, height, x + offset, y, color);
        overlay_pixel(frame, width, height, x, y + offset, color);
    }
}

static void render_tracking_overlay(uint8_t *frame,
                                    uint16_t width,
                                    uint16_t height,
                                    const line_observation_t *observation)
{
    if (frame == NULL || observation == NULL) {
        return;
    }
    /* Keep the debug overlay sparse so it is effectively free at 2.5 FPS. */
    for (int i = 0; i < observation->point_count; i += 2) {
        overlay_dot(frame, width, height, observation->point_x[i],
                    observation->point_y[i], 0x07e0);
    }
    if (observation->point_count == 0) {
        return;
    }
    /* 红十字画在实际扫描起始行上，方便用地面胶带量出最近扫描行的落地距离。 */
    overlay_cross(frame, width, height, observation->point_x[0],
                  observation->scan_bottom_y, 0xf800);
}

static int state_search_half_percent(void)
{
    if (s_state == LINE_STATE_CORNER) {
        int percent = LINE_CORNER_WINDOW_START_PERCENT +
                      (int)s_corner_frames * LINE_CORNER_WINDOW_GROW_PERCENT;
        return percent > LINE_CORNER_WINDOW_MAX_PERCENT ?
               LINE_CORNER_WINDOW_MAX_PERCENT : percent;
    }
    if (s_state == LINE_STATE_LOST) {
        const int percent = LINE_SEARCH_HALF_PERCENT +
                            (int)(s_lost_frames * LINE_LOST_WINDOW_GROW_PERCENT);
        return percent > LINE_LOST_WINDOW_MAX_PERCENT ?
               LINE_LOST_WINDOW_MAX_PERCENT : percent;
    }
    return LINE_SEARCH_HALF_PERCENT;
}

static bool observe_line(uint8_t *frame,
                         uint16_t width,
                         uint16_t height,
                         uint8_t source_threshold,
                         bool remembered_corridor,
                         bool draw_overlay,
                         line_observation_t *observation)
{
    if (observation == NULL) {
        return false;
    }
    const line_observation_t empty = {0};
    *observation = empty;
    observation->threshold = source_threshold;
    observation->corner_row_y = -1;
    observation->scan_bottom_y = -1;
    if (source_threshold == 0) {
        return false;
    }

    line_scan_cfg_t cfg = {0};
    cfg.width = width;
    cfg.height = height;
    cfg.threshold = source_threshold;
    cfg.use_history = s_seed_valid;
    cfg.seed_x = s_seed_x;
    cfg.search_half_percent = state_search_half_percent();
    cfg.expected_width = s_line_width;
    cfg.mirror_x = CAMERA_LINE_MIRROR_X ? true : false;
    cfg.saturation_guard = LINE_SATURATION_GUARD ? true : false;
    cfg.saturation_max = LINE_SATURATION_MAX;
    cfg.corridor_x = remembered_corridor && s_pending_turn != 0 ?
                     s_pending_seed_x : -1;
    cfg.corridor_half = positive_percent((int)width, LINE_SEARCH_HALF_PERCENT,
                                         LINE_SEARCH_HALF_MIN);

    if (s_state == LINE_STATE_CORNER && s_pending_turn != 0) {
        const int step = positive_percent((int)width,
                                          LINE_CORNER_PREDICT_STEP_PERCENT, 2);
        const int predicted = s_corner_origin_x +
                              s_pending_turn * step * (int)s_corner_frames;
        s_corner_predicted_x = clamp_range(predicted, 0, (int)width - 1);
        cfg.seed_x = s_corner_predicted_x;
        cfg.corridor_x = s_corner_predicted_x;
        cfg.corridor_half = positive_percent((int)width,
                                             LINE_CORNER_WINDOW_START_PERCENT,
                                             LINE_SEARCH_HALF_MIN);
    }

    const bool candidate = line_geometry_track(frame, &cfg, observation);
    observation->threshold = source_threshold;
    if (draw_overlay) {
        render_tracking_overlay(frame, width, height, observation);
    }
    return candidate;
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
    /* command 是混控层有符号 PWM；换向时先清零，避免 TB6612 硬切方向。 */
    const int index = (int)motor->channel;
    int output_limit = MAX_OUTPUT;
    if (s_motor_start_ramp_frames > 0) {
        const uint8_t elapsed = LINE_MOTOR_START_RAMP_FRAMES -
                                s_motor_start_ramp_frames;
        output_limit = LINE_MOTOR_START_MIN_OUTPUT +
                       ((MAX_OUTPUT - LINE_MOTOR_START_MIN_OUTPUT) * elapsed) /
                       LINE_MOTOR_START_RAMP_FRAMES;
    }
    const int speed = clamp_int(command * motor->sign, output_limit);
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
    if (s_kick_cycles[index] > 0) {
        /* car-spin 的实车规则：换向冲量固定为 START_KICK_OUTPUT，B 轮只在
         * 转向量足够大时才参与冲量。冲量绝对不能被启动 ramp 限幅压低，
         * 否则首帧的 kick 会变成 12%，等于把破静摩擦的冲量自己抵消掉。 */
        if ((motor->channel != LEDC_CHANNEL_1 || output > LINE_TURN_MAX) &&
            output < START_KICK_OUTPUT) {
            output = START_KICK_OUTPUT;
        }
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
    s_stby_enabled = false;
}

static void drive(int forward, int turn)
{
    /* 直行向量固定为 A=-forward、D=forward；只有 turn 非零才动后轮 B。 */
    s_command_a = clamp_int(-forward - turn, MAX_OUTPUT);
    s_command_b = clamp_int(turn, MAX_OUTPUT);
    s_command_d = clamp_int(forward - turn, MAX_OUTPUT);
    motor_set(&motor_a, s_command_a);
    motor_set(&motor_b, s_command_b);
    motor_set(&motor_d, s_command_d);
    if (s_motor_start_ramp_frames > 0) {
        --s_motor_start_ramp_frames;
    }
}

static void reset_pd(void)
{
    s_tracking_error = 0;
    s_previous_tracking_error = 0;
    s_error_filter_initialized = false;
    s_pid_output = 0;
}

static void clear_turn_plan(void)
{
    s_pending_turn = 0;
    s_pending_seed_x = 0;
    s_pending_miss_frames = 0;
    s_hint_direction = 0;
    s_hint_frames = 0;
    s_corner_frames = 0;
    s_corner_exit_frames = 0;
    s_corner_origin_x = 0;
    s_corner_predicted_x = 0;
    s_corner_started_us = 0;
}

static void reset_tracking(void)
{
    reset_pd();
    clear_turn_plan();
    s_seed_valid = false;
    s_seed_x = 0;
    s_line_width = 0;
    s_lost_frames = 0;
    s_reacquire_frames = 0;
    s_reacquire_x = 0;
    s_state = LINE_STATE_NORMAL;
}

static int pd_steering(int lateral_error, int heading_error)
{
    if (!s_error_filter_initialized) {
        s_tracking_error = lateral_error;
        s_previous_tracking_error = lateral_error;
        s_error_filter_initialized = true;
    } else {
        s_tracking_error = (s_tracking_error * LINE_ERROR_FILTER_OLD +
                            lateral_error * LINE_ERROR_FILTER_NEW) /
                           (LINE_ERROR_FILTER_OLD + LINE_ERROR_FILTER_NEW);
    }

    const int delta_lateral_error = s_tracking_error - s_previous_tracking_error;
    s_previous_tracking_error = s_tracking_error;
    if (abs(s_tracking_error) <= LINE_ERROR_DEADBAND &&
        abs(heading_error) <= LINE_ERROR_DEADBAND) {
        s_pid_output = 0;
        return 0;
    }

    /* 只保留整数 P、heading 前馈和 D；没有积分状态。 */
    int target = (LINE_PID_KP * s_tracking_error -
                  LINE_PID_KH * heading_error -
                  LINE_PID_KD * delta_lateral_error) / LINE_PID_SCALE;
    target = clamp_int(target, LINE_PID_TURN_MAX);
    const int delta = target - s_pid_output;
    if (delta > LINE_PID_SLEW_PER_FRAME) {
        target = s_pid_output + LINE_PID_SLEW_PER_FRAME;
    } else if (delta < -LINE_PID_SLEW_PER_FRAME) {
        target = s_pid_output - LINE_PID_SLEW_PER_FRAME;
    }
    s_pid_output = target;
    if (abs(s_pid_output) <= LINE_PID_OUTPUT_DEADBAND) {
        s_pid_output = 0;
    }
    return s_pid_output;
}

static int forward_speed(int lateral_error)
{
    const int magnitude = abs(lateral_error);
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

static void update_seed(const line_observation_t *observation)
{
    if (observation == NULL || !observation->candidate) {
        return;
    }
    s_seed_x = observation->seed_x;
    s_seed_valid = true;
    if (observation->near_width > 0) {
        s_line_width = s_line_width == 0 ? observation->near_width :
                       (s_line_width * LINE_WIDTH_FILTER_OLD +
                        observation->near_width * LINE_WIDTH_FILTER_NEW) /
                       (LINE_WIDTH_FILTER_OLD + LINE_WIDTH_FILTER_NEW);
    }
}

static void update_turn_memory(const line_observation_t *observation)
{
    if (observation == NULL || !observation->candidate ||
        observation->corner_direction == 0 || s_pending_turn != 0 ||
        s_state != LINE_STATE_NORMAL) {
        if (s_pending_turn == 0) {
            s_hint_direction = 0;
            s_hint_frames = 0;
        }
        return;
    }

    const int direction = observation->corner_direction;
    if (direction != s_hint_direction) {
        s_hint_direction = direction;
        s_hint_frames = 1;
    } else if (s_hint_frames < LINE_BRANCH_CONFIRM_FRAMES) {
        ++s_hint_frames;
    }

    if (s_hint_frames >= LINE_BRANCH_CONFIRM_FRAMES) {
        s_pending_turn = direction;
        s_pending_seed_x = observation->seed_x;
        s_hint_direction = 0;
        s_hint_frames = 0;
        ESP_LOGI(TAG, "pending_turn=%d; continue on the old line until it disappears",
                 s_pending_turn);
    }
}

static const char *state_name(void)
{
    switch (s_state) {
    case LINE_STATE_CORNER:
        return "CORNER";
    case LINE_STATE_LOST:
        return "LOST";
    case LINE_STATE_NORMAL:
    default:
        return "NORMAL";
    }
}

static void maybe_log_summary(int64_t now)
{
    if (s_summary_start_us == 0) {
        s_summary_start_us = now;
        camera_display_get_counters(&s_summary_camera_frames,
                                    &s_summary_processed_frames,
                                    &s_summary_dropped_frames);
        s_summary_callback_dropped_frames = s_callback_dropped_frames;
        return;
    }
    if (now - s_summary_start_us < 1000000) {
        return;
    }

    uint32_t camera_frames = 0;
    uint32_t processed_frames = 0;
    uint32_t dropped_frames = 0;
    camera_display_get_counters(&camera_frames, &processed_frames, &dropped_frames);
    const uint32_t camera_fps = camera_frames - s_summary_camera_frames;
    const uint32_t processed_fps = processed_frames - s_summary_processed_frames;
    const uint32_t frames_dropped = (dropped_frames - s_summary_dropped_frames) +
                                    (s_callback_dropped_frames -
                                     s_summary_callback_dropped_frames);
    const uint32_t line_us_avg = s_control_frames == 0 ? 0 :
                                 s_line_us_sum / s_control_frames;
    ESP_LOGI(TAG,
             "camera_fps=%u processed_fps=%u control_fps=%u frames_dropped=%u "
             "line_us_avg=%u line_us_max=%u state=%s armed=%d candidate=%d "
             "arm_frames=%u threshold=%d seed_x=%d line_w=%d valid_rows=%u "
             "confidence=%u lateral_error=%d heading_error=%d pending_turn=%d STBY=%d "
             "motor[A,B,D]=[%d,%d,%d]",
             (unsigned)camera_fps, (unsigned)processed_fps,
             (unsigned)s_control_frames, (unsigned)frames_dropped,
             (unsigned)line_us_avg, (unsigned)s_line_us_max, state_name(),
             s_armed, s_last_candidate, (unsigned)s_arm_frames,
             s_last_threshold, s_last_seed_x, s_line_width,
             (unsigned)s_last_valid_rows, (unsigned)s_last_confidence,
             s_last_lateral_error, s_last_heading_error, s_pending_turn,
             s_stby_enabled,
             s_command_a, s_command_b, s_command_d);

    s_summary_camera_frames = camera_frames;
    s_summary_processed_frames = processed_frames;
    s_summary_dropped_frames = dropped_frames;
    s_summary_callback_dropped_frames = s_callback_dropped_frames;
    s_summary_start_us = now;
    s_control_frames = 0;
    s_line_us_sum = 0;
    s_line_us_max = 0;
}

static void disarm_tracking(void)
{
    stop_motors();
    s_armed = false;
    s_arm_frames = 0;
    s_motor_start_ramp_frames = 0;
    s_first_frame_us = 0;
    s_last_line_us = 0;
    reset_tracking();
    s_state = LINE_STATE_LOST;
}

static void camera_line_follow_process_frame(uint8_t *rgb565_big_endian,
                                             uint16_t width,
                                             uint16_t height,
                                             uint8_t source_threshold,
                                             bool draw_overlay)
{
    const int64_t line_start_us = esp_timer_get_time();
    const int64_t now = line_start_us;
    if (!s_started || s_finished) {
        if (s_started) {
            stop_motors();
        }
        goto done;
    }
    if (s_first_frame_us == 0) {
        s_first_frame_us = now;
    }

    line_observation_t observation = {0};
    if (s_state == LINE_STATE_CORNER && s_corner_frames < UINT8_MAX) {
        ++s_corner_frames;
    }
    if (s_state == LINE_STATE_LOST && s_lost_frames < UINT32_MAX) {
        ++s_lost_frames;
    }
    const bool remembered_corridor = s_pending_turn != 0 &&
                                     s_state != LINE_STATE_CORNER;
    const bool candidate = observe_line(rgb565_big_endian, width, height,
                                        source_threshold, remembered_corridor,
                                        draw_overlay,
                                        &observation);
    s_last_threshold = observation.threshold;
    s_last_candidate = candidate;
    s_last_seed_x = observation.valid_rows > 0 ? observation.seed_x : -1;
    s_last_valid_rows = observation.valid_rows;
    s_last_confidence = observation.confidence;
    if (candidate) {
        s_last_lateral_error = observation.lateral_error;
        s_last_heading_error = observation.heading_error;
    }

    if (!s_armed) {
        if (candidate && now - s_first_frame_us >= (int64_t)LINE_START_DELAY_MS * 1000) {
            if (s_arm_frames < LINE_ARM_CONFIRM_FRAMES) {
                ++s_arm_frames;
            }
            if (s_arm_frames >= LINE_ARM_CONFIRM_FRAMES) {
                s_armed = true;
                s_state = LINE_STATE_NORMAL;
                reset_pd();
                update_seed(&observation);
                s_last_line_us = now;
                s_motor_start_ramp_frames = LINE_MOTOR_START_RAMP_FRAMES;
                gpio_set_level(STBY_GPIO, 1);
                s_stby_enabled = true;
                ESP_LOGI(TAG, "line confirmed; camera steering enabled");
            }
        } else if (!candidate) {
            s_arm_frames = 0;
        }
        if (!s_armed) {
            stop_motors();
            goto done;
        }
        zero_motor_outputs();
        goto done;
    }

    if (observation.finish_candidate && candidate) {
        if (s_finish_frames < LINE_FINISH_CONFIRM_FRAMES) {
            ++s_finish_frames;
        }
        if (s_finish_frames >= LINE_FINISH_CONFIRM_FRAMES) {
            s_finished = true;
            stop_motors();
            ESP_LOGW(TAG, "finish bar confirmed; motors stopped");
            goto done;
        }
    } else {
        s_finish_frames = 0;
    }

    if (s_state == LINE_STATE_CORNER) {
        bool new_line_candidate = candidate;
        if (new_line_candidate && s_pending_turn != 0) {
            const int displacement = observation.seed_x - s_corner_origin_x;
            const int minimum_shift = positive_percent((int)width,
                                                       LINE_CORNER_MIN_SHIFT_PERCENT,
                                                       LINE_SEARCH_HALF_MIN);
            const int prediction_window = positive_percent((int)width,
                                                            LINE_CORNER_WINDOW_START_PERCENT,
                                                            LINE_SEARCH_HALF_MIN);
            /* Do not count the old seed as a CORNER exit.  The bottom seed
             * must move in the remembered direction and stay near prediction. */
            if (direction_of(displacement) != s_pending_turn ||
                abs(displacement) < minimum_shift ||
                abs(observation.seed_x - s_corner_predicted_x) > prediction_window) {
                new_line_candidate = false;
            }
        }
        if (new_line_candidate) {
            if (s_corner_exit_frames < LINE_CORNER_EXIT_CONFIRM_FRAMES) {
                ++s_corner_exit_frames;
            }
        } else {
            s_corner_exit_frames = 0;
        }
        if (s_corner_exit_frames >= LINE_CORNER_EXIT_CONFIRM_FRAMES) {
            s_state = LINE_STATE_NORMAL;
            update_seed(&observation);
            clear_turn_plan();
            reset_pd();
            s_last_line_us = now;
            const int turn = pd_steering(observation.lateral_error,
                                         observation.heading_error);
            drive(forward_speed(observation.lateral_error), turn);
            goto done;
        }
        if (s_corner_started_us != 0 &&
            now - s_corner_started_us > (int64_t)LINE_CORNER_TIMEOUT_MS * 1000) {
            disarm_tracking();
            ESP_LOGW(TAG, "corner timed out; motors stopped and line confirmation reset");
            goto done;
        }
        drive(LINE_FORWARD_CRAWL, -s_pending_turn * LINE_TURN_MAX);
        goto done;
    }

    if (s_pending_turn != 0) {
        if (candidate && observation.near_line_visible) {
            s_state = LINE_STATE_NORMAL;
            s_pending_miss_frames = 0;
            s_lost_frames = 0;
            s_reacquire_frames = 0;
            s_last_line_us = now;
            update_seed(&observation);
            const int turn = pd_steering(observation.lateral_error,
                                         observation.heading_error);
            drive(forward_speed(observation.lateral_error), turn);
            goto done;
        }

        /* 旧直线消失需连续确认，避免一帧曝光抖动提前拐。 */
        if (s_pending_miss_frames < LINE_PENDING_MISS_CONFIRM_FRAMES) {
            ++s_pending_miss_frames;
        }
        if (s_pending_miss_frames < LINE_PENDING_MISS_CONFIRM_FRAMES) {
            drive(LINE_FORWARD_CRAWL, 0);
            goto done;
        }

        /* 旧直线已不在保存的局部走廊内，现在才进入统一 CORNER。 */
        s_state = LINE_STATE_CORNER;
        s_corner_started_us = now;
        s_corner_frames = 0;
        s_corner_exit_frames = 0;
        s_corner_origin_x = s_pending_seed_x;
        s_corner_predicted_x = s_pending_seed_x;
        reset_pd();
        drive(LINE_FORWARD_CRAWL, -s_pending_turn * LINE_TURN_MAX);
        goto done;
    }

    if (s_state == LINE_STATE_LOST) {
        const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
        if (candidate) {
            const int confirm_window = positive_percent((int)width,
                                                        LINE_SEARCH_HALF_PERCENT,
                                                        LINE_SEARCH_HALF_MIN);
            /* A LOST candidate must first remain close to the last trusted
             * seed.  The expanding window is for small real motion, not for
             * jumping across the image onto a chair leg or other dark object. */
            const bool near_last_seed =
                abs(observation.seed_x - s_seed_x) <= confirm_window;
            if (!near_last_seed) {
                s_reacquire_frames = 0;
                s_reacquire_x = s_seed_x;
            } else if (s_reacquire_frames == 0 ||
                       abs(observation.seed_x - s_reacquire_x) <= confirm_window) {
                s_reacquire_x = observation.seed_x;
                if (s_reacquire_frames < LINE_REACQUIRE_CONFIRM_FRAMES) {
                    ++s_reacquire_frames;
                }
            } else {
                s_reacquire_x = observation.seed_x;
                s_reacquire_frames = 1;
            }
            if (s_reacquire_frames >= LINE_REACQUIRE_CONFIRM_FRAMES) {
                s_state = LINE_STATE_NORMAL;
                s_lost_frames = 0;
                s_reacquire_frames = 0;
                update_seed(&observation);
                s_last_line_us = now;
                reset_pd();
                update_turn_memory(&observation);
                const int turn = pd_steering(observation.lateral_error,
                                             observation.heading_error);
                drive(forward_speed(observation.lateral_error), turn);
                goto done;
            }
            if (near_last_seed) {
                const int correction = direction_of(observation.seed_x - s_seed_x);
                drive(LINE_FORWARD_CRAWL, -correction * 4);
            } else {
                drive(LINE_FORWARD_CRAWL, 0);
            }
        } else {
            /* 候选线必须在相邻帧连续出现，任何空帧都重新开始确认。 */
            s_reacquire_frames = 0;
            s_reacquire_x = s_seed_x;
            if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000) {
                drive(LINE_FORWARD_CRAWL, 0);
            } else {
                zero_motor_outputs();
            }
        }
        if (lost_us >= (int64_t)LINE_LOST_STOP_MS * 1000) {
            disarm_tracking();
        }
        goto done;
    }

    if (candidate) {
        s_state = LINE_STATE_NORMAL;
        s_lost_frames = 0;
        s_reacquire_frames = 0;
        s_last_line_us = now;
        update_turn_memory(&observation);
        update_seed(&observation);
        const int turn = pd_steering(observation.lateral_error,
                                     observation.heading_error);
        drive(forward_speed(observation.lateral_error), turn);
        goto done;
    }

    s_state = LINE_STATE_LOST;
    s_lost_frames = 1;
    s_reacquire_frames = 0;
    s_reacquire_x = s_seed_x;
    reset_pd();
    const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
    if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000) {
        drive(LINE_FORWARD_CRAWL, 0);
    } else {
        zero_motor_outputs();
        if (lost_us >= (int64_t)LINE_LOST_STOP_MS * 1000) {
            disarm_tracking();
        }
    }

done:
    {
        const uint32_t elapsed = (uint32_t)(esp_timer_get_time() - line_start_us);
        ++s_control_frames;
        s_line_us_sum += elapsed;
        if (elapsed > s_line_us_max) {
            s_line_us_max = elapsed;
        }
        maybe_log_summary(esp_timer_get_time());
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
            disarm_tracking();
            s_last_frame_us = 0;
            ESP_LOGW(TAG, "decoded-frame timeout; motors stopped and line confirmation reset");
        }
        (void)xSemaphoreGive(s_control_mutex);
    }
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
    s_stby_enabled = false;
    s_arm_frames = 0;
    s_motor_start_ramp_frames = 0;
    s_finish_frames = 0;
    s_first_frame_us = 0;
    s_last_line_us = 0;
    s_last_frame_us = 0;
    s_state = LINE_STATE_NORMAL;
    s_last_lateral_error = 0;
    s_last_heading_error = 0;
    s_last_valid_rows = 0;
    s_last_confidence = 0;
    s_last_threshold = 0;
    s_last_candidate = false;
    s_last_seed_x = -1;
    s_last_direction[0] = 0;
    s_last_direction[1] = 0;
    s_last_direction[2] = 0;
    s_kick_cycles[0] = 0;
    s_kick_cycles[1] = 0;
    s_kick_cycles[2] = 0;
    s_summary_start_us = 0;
    s_summary_camera_frames = 0;
    s_summary_processed_frames = 0;
    s_summary_dropped_frames = 0;
    s_callback_dropped_frames = 0;
    s_summary_callback_dropped_frames = 0;
    s_control_frames = 0;
    s_line_us_sum = 0;
    s_line_us_max = 0;
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

void camera_line_follow_frame_callback(uint8_t *rgb565_big_endian,
                                       uint16_t width,
                                       uint16_t height,
                                       uint8_t source_threshold,
                                       bool draw_overlay,
                                       void *user_ctx)
{
    (void)user_ctx;
    if (!s_started) {
        return;
    }
    if (s_control_mutex != NULL &&
        xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(5)) != pdTRUE) {
        ++s_callback_dropped_frames;
        return;
    }
    s_last_frame_us = esp_timer_get_time();
    camera_line_follow_process_frame(rgb565_big_endian, width, height,
                                     source_threshold, draw_overlay);
    if (s_control_mutex != NULL) {
        (void)xSemaphoreGive(s_control_mutex);
    }
}
