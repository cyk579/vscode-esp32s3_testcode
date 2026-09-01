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

/* 只从底部种子向上跟踪，固定窗口和固定数组保证单帧时间有界。 */
#define LINE_ROI_TOP_PERCENT 30
#define LINE_ROI_BOTTOM_PERCENT 95
#define LINE_NEAR_TOP_PERCENT 65
#define LINE_NEAR_BOTTOM_PERCENT 92
#define LINE_ROW_STEP 4
#define LINE_BOTTOM_SKIP_ROWS 1
#define LINE_SCAN_MAX_ROWS 64
#define LINE_SEARCH_HALF_PERCENT 20
#define LINE_SEARCH_HALF_MIN 6
#define LINE_SEARCH_HALF_MAX 80
#define LINE_LOST_WINDOW_GROW_PERCENT 4
#define LINE_LOST_WINDOW_MAX_PERCENT 42
#define LINE_CORNER_WINDOW_START_PERCENT 28
#define LINE_CORNER_WINDOW_GROW_PERCENT 4
#define LINE_CORNER_WINDOW_MAX_PERCENT 46
#define LINE_CORNER_PREDICT_STEP_PERCENT 4
#define LINE_MAX_CENTER_JUMP_PERCENT 18
#define LINE_MIN_SEGMENT_WIDTH 3
#define LINE_MAX_SEGMENT_WIDTH_PERCENT 55
#define LINE_FINISH_WIDTH_PERCENT 62
#define LINE_FINISH_MIN_ROWS 2
#define LINE_MIN_VALID_ROWS 4
#define LINE_BRANCH_ROWS 3
#define LINE_BRANCH_OFFSET_PERCENT 14
#define LINE_BRANCH_WINDOW_PERCENT 12
#define LINE_BRANCH_MIN_OFFSET_PERCENT 8
#define LINE_BRANCH_CONFIRM_FRAMES 3U
#define LINE_PENDING_MISS_CONFIRM_FRAMES 2U
#define LINE_REACQUIRE_CONFIRM_FRAMES 2U
#define LINE_CORNER_EXIT_CONFIRM_FRAMES 3U

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

#define PWM_MAX 1023U
#define MOTOR_MIN_RUN_OUTPUT 8
#define MOTOR_B_MIN_RUN_OUTPUT 0
#define START_KICK_OUTPUT 18
#define START_KICK_CYCLES 3U
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

typedef struct {
    int x;
    int y;
    int width;
} line_point_t;

typedef struct {
    int center;
    int width;
    bool wide;
} line_segment_t;

typedef enum {
    LINE_STATE_NORMAL = 0,
    LINE_STATE_CORNER,
    LINE_STATE_LOST,
} line_state_t;

typedef struct {
    bool candidate;
    bool old_line_visible;
    bool finish_candidate;
    int seed_x;
    int lateral_error;
    int heading_error;
    int branch_direction;
    uint8_t valid_rows;
    uint8_t confidence;
    int threshold;
} line_observation_t;

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
static int s_seed_heading;
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
    int result = value * percent / 100;
    return result < minimum ? minimum : result;
}

static uint8_t rgb565_luma(const uint8_t *pixel)
{
    const uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    const int red = ((value >> 11) & 0x1F) * 255 / 31;
    const int green = ((value >> 5) & 0x3F) * 255 / 63;
    const int blue = (value & 0x1F) * 255 / 31;
    return (uint8_t)((77 * red + 150 * green + 29 * blue) >> 8);
}

/* 正值表示赛道在画面中心左侧，便于直接套用整数 PD 符号。 */
static int line_error(int center, uint16_t width)
{
    int error = ((int)width / 2 - center) * 100 / ((int)width / 2);
    error = clamp_int(error, 100);
#if CAMERA_LINE_MIRROR_X
    error = -error;
#endif
    return error;
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
                                    const line_point_t *points,
                                    int point_count,
                                    int seed_y)
{
    /* Keep the debug overlay sparse so it is effectively free at 2.5 FPS. */
    for (int i = 0; i < point_count; i += 2) {
        overlay_dot(frame, width, height, points[i].x, points[i].y, 0x07e0);
    }
    if (point_count == 0) {
        return;
    }

    overlay_cross(frame, width, height, points[0].x, seed_y, 0xf800);
}

static bool scan_segment(const uint8_t *frame,
                         uint16_t width,
                         int y,
                         int expected_x,
                         int half_window,
                         int threshold,
                         line_segment_t *segment)
{
    if (frame == NULL || segment == NULL || y < 0) {
        return false;
    }

    *segment = (line_segment_t){0};
    int left = expected_x < 0 ? 0 : expected_x - half_window;
    int right = expected_x < 0 ? (int)width - 1 : expected_x + half_window;
    left = left < 0 ? 0 : left;
    right = right >= (int)width ? (int)width - 1 : right;
    if (left > right) {
        return false;
    }

    const int min_width = LINE_MIN_SEGMENT_WIDTH;
    const int max_width = positive_percent((int)width,
                                           LINE_MAX_SEGMENT_WIDTH_PERCENT,
                                           min_width + 2);
    /* Finish width is relative to the local window, not the whole image. */
    const int window_width = right - left + 1;
    const int finish_width = positive_percent(window_width,
                                              LINE_FINISH_WIDTH_PERCENT,
                                              LINE_MIN_SEGMENT_WIDTH + 1);
    int best_distance = INT_MAX;
    line_segment_t best = {0};
    int best_wide_distance = INT_MAX;
    line_segment_t best_wide = {0};
    bool in_run = false;
    int run_start = 0;

    for (int x = left; x <= right + 1; ++x) {
        const bool dark = x <= right &&
                          rgb565_luma(frame + (((size_t)y * width + (size_t)x) * 2)) <= threshold;
        if (dark && !in_run) {
            in_run = true;
            run_start = x;
        }
        if (!dark && in_run) {
            const int run_end = x - 1;
            const int run_width = run_end - run_start + 1;
            const int run_center = (run_start + run_end) / 2;
            if (run_width >= finish_width) {
                const int wide_distance = expected_x < 0 ?
                                           abs(run_center - (int)width / 2) :
                                           abs(run_center - expected_x);
                if (wide_distance < best_wide_distance) {
                    best_wide_distance = wide_distance;
                    best_wide.center = run_center;
                    best_wide.width = run_width;
                    best_wide.wide = true;
                }
            }
            if (run_width >= min_width && run_width <= max_width) {
                const int distance = expected_x < 0 ?
                                     abs(run_center - (int)width / 2) :
                                     abs(run_center - expected_x);
                if (distance < best_distance) {
                    best_distance = distance;
                    best.center = run_center;
                    best.width = run_width;
                    best.wide = run_width >= finish_width;
                }
            }
            in_run = false;
        }
    }

    if (best_distance == INT_MAX) {
        /* A broad local finish bar is still a valid seed for confirmation. */
        if (best_wide_distance == INT_MAX) {
            return false;
        }
        best = best_wide;
    }
    /* Only the selected run may mark a finish candidate.  A separate wide
     * run elsewhere in the search window must not taint the tracked line. */
    *segment = best;
    return true;
}

static int average_points(const line_point_t *points, int first, int count)
{
    if (points == NULL || count <= 0) {
        return 0;
    }
    int total = 0;
    for (int i = 0; i < count; ++i) {
        total += points[first + i].x;
    }
    return total / count;
}

static int branch_hint(const uint8_t *frame,
                       uint16_t width,
                       int threshold,
                       const line_point_t *points,
                       int point_count)
{
    if (point_count < LINE_MIN_VALID_ROWS) {
        return 0;
    }

    const line_point_t *end = &points[point_count - 1];
    const int offset = positive_percent((int)width, LINE_BRANCH_OFFSET_PERCENT,
                                        LINE_SEARCH_HALF_MIN * 2);
    const int half = positive_percent((int)width, LINE_BRANCH_WINDOW_PERCENT,
                                      LINE_SEARCH_HALF_MIN);
    int found_direction = 0;
    for (int direction = -1; direction <= 1; direction += 2) {
        int hits = 0;
        for (int i = 0; i < LINE_BRANCH_ROWS; ++i) {
            int y = end->y + (i - 1) * LINE_ROW_STEP;
            if (y < 0) {
                y = 0;
            }
            line_segment_t branch = {0};
            if (scan_segment(frame, width, y, end->x + direction * offset,
                             half, threshold, &branch) &&
                abs(branch.center - end->x) >= offset * LINE_BRANCH_MIN_OFFSET_PERCENT /
                                                LINE_BRANCH_OFFSET_PERCENT) {
                ++hits;
            }
        }
        if (hits >= 2) {
            if (found_direction != 0 && found_direction != direction) {
                return 0;
            }
            found_direction = direction;
        }
    }

    return found_direction;
}

static bool track_line(uint8_t *frame,
                       uint16_t width,
                       uint16_t height,
                       int threshold,
                       bool use_history,
                       int seed_x,
                       int corridor_x,
                       int corridor_half,
                       bool draw_overlay,
                       line_observation_t *observation)
{
    if (frame == NULL || observation == NULL || width < 16 || height < 16 || threshold <= 0) {
        return false;
    }

    int top = (int)height * LINE_ROI_TOP_PERCENT / 100;
    int roi_bottom = (int)height * LINE_ROI_BOTTOM_PERCENT / 100;
    const int near_top = (int)height * LINE_NEAR_TOP_PERCENT / 100;
    const int near_bottom = (int)height * LINE_NEAR_BOTTOM_PERCENT / 100 -
                            LINE_BOTTOM_SKIP_ROWS * LINE_ROW_STEP;
    top = top < 0 ? 0 : top;
    roi_bottom = roi_bottom >= (int)height ? (int)height - 1 : roi_bottom;
    const int bottom = use_history ? near_bottom :
                       roi_bottom - LINE_BOTTOM_SKIP_ROWS * LINE_ROW_STEP;
    if (bottom <= top) {
        return false;
    }

    int search_percent = LINE_SEARCH_HALF_PERCENT;
    if (s_state == LINE_STATE_CORNER) {
        search_percent = LINE_CORNER_WINDOW_START_PERCENT +
                         (int)s_corner_frames * LINE_CORNER_WINDOW_GROW_PERCENT;
        if (search_percent > LINE_CORNER_WINDOW_MAX_PERCENT) {
            search_percent = LINE_CORNER_WINDOW_MAX_PERCENT;
        }
    } else if (s_state == LINE_STATE_LOST) {
        const int extra = (int)(s_lost_frames * LINE_LOST_WINDOW_GROW_PERCENT);
        search_percent += extra;
        if (search_percent > LINE_LOST_WINDOW_MAX_PERCENT) {
            search_percent = LINE_LOST_WINDOW_MAX_PERCENT;
        }
    }
    int half_window = positive_percent((int)width, search_percent, LINE_SEARCH_HALF_MIN);
    if (half_window > LINE_SEARCH_HALF_MAX) {
        half_window = LINE_SEARCH_HALF_MAX;
    }

    int expected = use_history ? seed_x : (int)width / 2;
    expected = expected < 0 ? 0 : (expected >= (int)width ? (int)width - 1 : expected);
    line_point_t points[LINE_SCAN_MAX_ROWS];
    int point_count = 0;
    int misses = 0;
    int wide_near_rows = 0;
    int y = bottom;

    while (y >= top && point_count < LINE_SCAN_MAX_ROWS) {
        line_segment_t segment = {0};
        const int row_half = point_count == 0 && !use_history ? (int)width : half_window;
        const bool found = scan_segment(frame, width, y,
                                        point_count == 0 && !use_history ? -1 : expected,
                                        row_half, threshold, &segment);
        if (!found) {
            /* Initial acquisition checks a few bottom rows, still using a
             * full-width nearest-to-centre seed search on each row. */
            if (point_count == 0 && !use_history && y > near_bottom) {
                y -= LINE_ROW_STEP;
                continue;
            }
            if (point_count > 0 && misses == 0) {
                ++misses;
                y -= LINE_ROW_STEP;
                continue;
            }
            break;
        }
        misses = 0;

        const int jump_limit = positive_percent((int)width,
                                                LINE_MAX_CENTER_JUMP_PERCENT,
                                                LINE_SEARCH_HALF_MIN * 2);
        if (point_count > 0 && abs(segment.center - expected) > jump_limit) {
            break;
        }
        if (corridor_x >= 0 && abs(segment.center - corridor_x) > corridor_half) {
            break;
        }

        points[point_count++] = (line_point_t){segment.center, y, segment.width};
        expected = segment.center;
        if (segment.wide && y >= near_top && y <= near_bottom) {
            ++wide_near_rows;
        }
        y -= LINE_ROW_STEP;
    }

    observation->valid_rows = (uint8_t)(point_count > UINT8_MAX ? UINT8_MAX : point_count);
    observation->finish_candidate = wide_near_rows >= LINE_FINISH_MIN_ROWS;
    if (point_count > 0) {
        observation->seed_x = points[0].x;
    }
    if (point_count < LINE_MIN_VALID_ROWS) {
        if (draw_overlay) {
            render_tracking_overlay(frame, width, height, points, point_count, bottom);
        }
        return false;
    }

    const int near_count = point_count < 4 ? point_count : 4;
    const int far_count = point_count < 4 ? point_count : 4;
    const int near_center = average_points(points, 0, near_count);
    const int far_center = average_points(points, point_count - far_count, far_count);
    observation->candidate = true;
    observation->old_line_visible = point_count >= LINE_MIN_VALID_ROWS + 1;
    observation->seed_x = points[0].x;
    observation->lateral_error = line_error(near_center, width);
    /* Positive heading means the far path is to the right.  This keeps the
     * requested -Kh*heading term aligned with the calibrated motor sign. */
    observation->heading_error = line_error(near_center, width) -
                                 line_error(far_center, width);
    observation->confidence = (uint8_t)((point_count * 100) /
                                        (LINE_MIN_VALID_ROWS + 6));
    if (observation->confidence > 100) {
        observation->confidence = 100;
    }
    observation->branch_direction = branch_hint(frame, width, threshold,
                                                 points, point_count);
    if (observation->finish_candidate &&
        abs(observation->seed_x - (use_history ? seed_x : (int)width / 2)) > half_window) {
        observation->finish_candidate = false;
    }
    if (draw_overlay) {
        render_tracking_overlay(frame, width, height, points, point_count, bottom);
    }
    return true;
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
    *observation = (line_observation_t){0};
    observation->threshold = source_threshold;
    if (source_threshold == 0) {
        return false;
    }

    int seed = s_seed_x;
    if (s_state == LINE_STATE_CORNER && s_pending_turn != 0) {
        const int step = positive_percent((int)width,
                                          LINE_CORNER_PREDICT_STEP_PERCENT, 2);
        const int predicted = s_corner_origin_x +
                              s_pending_turn * step * (int)s_corner_frames;
        s_corner_predicted_x = clamp_range(predicted, 0, (int)width - 1);
        seed = s_corner_predicted_x;
    } else if (s_state == LINE_STATE_LOST && s_seed_valid) {
        const int predicted_delta = s_seed_heading * (int)s_lost_frames * (int)width / 300;
        seed = clamp_range(s_seed_x + predicted_delta, 0, (int)width - 1);
    }

    int corridor_half = positive_percent((int)width,
                                         LINE_SEARCH_HALF_PERCENT,
                                         LINE_SEARCH_HALF_MIN);
    int corridor_x = remembered_corridor && s_pending_turn != 0 ?
                     s_pending_seed_x : -1;
    if (s_state == LINE_STATE_CORNER && s_pending_turn != 0) {
        corridor_half = positive_percent((int)width,
                                         LINE_CORNER_WINDOW_START_PERCENT,
                                         LINE_SEARCH_HALF_MIN);
        corridor_x = s_corner_predicted_x;
    }
    return track_line(frame, width, height, source_threshold,
                      s_seed_valid, seed, corridor_x,
                      corridor_half, draw_overlay, observation);
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
    if (s_kick_cycles[index] > 0) {
        const int kick_output = s_motor_start_ramp_frames > 0 ?
                                output_limit : START_KICK_OUTPUT;
        if (output < kick_output) {
            output = kick_output;
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
    s_seed_heading = 0;
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
    s_seed_heading = observation->heading_error;
    s_seed_valid = true;
}

static void update_turn_memory(const line_observation_t *observation)
{
    if (observation == NULL || !observation->candidate ||
        observation->branch_direction == 0 || s_pending_turn != 0 ||
        s_state != LINE_STATE_NORMAL) {
        if (s_pending_turn == 0) {
            s_hint_direction = 0;
            s_hint_frames = 0;
        }
        return;
    }

    const int direction = observation->branch_direction;
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
             "arm_frames=%u threshold=%d seed_x=%d valid_rows=%u confidence=%u "
             "lateral_error=%d heading_error=%d pending_turn=%d STBY=%d "
             "motor[A,B,D]=[%d,%d,%d]",
             (unsigned)camera_fps, (unsigned)processed_fps,
             (unsigned)s_control_frames, (unsigned)frames_dropped,
             (unsigned)line_us_avg, (unsigned)s_line_us_max, state_name(),
             s_armed, s_last_candidate, (unsigned)s_arm_frames,
             s_last_threshold, s_last_seed_x,
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
                                                       LINE_BRANCH_MIN_OFFSET_PERCENT,
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
        if (candidate && observation.old_line_visible) {
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
            if (s_reacquire_frames == 0 ||
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
            const int correction = direction_of(observation.seed_x - s_seed_x);
            drive(LINE_FORWARD_CRAWL, -correction * 4);
        } else {
            /* 候选线必须在相邻帧连续出现，任何空帧都重新开始确认。 */
            s_reacquire_frames = 0;
            s_reacquire_x = s_seed_x;
            if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000) {
                drive(LINE_FORWARD_CRAWL, -s_seed_heading * 3);
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
