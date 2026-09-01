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
 * 图像坐标 y 向下增大。画面被明确分成三个职责：
 *   远端区（35%~60%）：只记住下一次向左还是向右，绝不直接控制电机；
 *   近端区（70%~90%）：判断线形和终点；其中最下方区域才给 PID/触发使用。
 * 这样上方已经出现锐角时，小车仍会沿下方尚未走完的直线前进。
 */
#define LINE_ROI_TOP_PERCENT 30
#define LINE_ROI_BOTTOM_PERCENT 95
#define LINE_FAR_TOP_PERCENT 35
#define LINE_FAR_BOTTOM_PERCENT 60
#define LINE_NEAR_TOP_PERCENT 70
#define LINE_NEAR_BOTTOM_PERCENT 90
#define LINE_LOWER_TOP_PERCENT 78
#define LINE_ROW_STEP 4
#define LINE_BOTTOM_SKIP_ROWS 1
#define LINE_STRAIGHT_SPAN_PERCENT 14
#define LINE_MIN_CONTRAST 32
#define LINE_BLACK_FRACTION_PERCENT 30
#define LINE_BLACK_THRESHOLD_MIN 35
#define LINE_BLACK_THRESHOLD_MAX 120
#define LINE_MIN_ROW_SAMPLES 3
#define LINE_MIN_VALID_ROWS 2
#define LINE_STRAIGHT_MIN_ROWS 3
#define LINE_MIN_DARK_ROWS 2
#define LINE_FINISH_ROW_COVERAGE_PERCENT 50
#define LINE_FINISH_MIN_ROWS 3

/* 弯道采用“先记忆、后触发”：所有判断都要求连续帧，单帧噪声不会转动车体。 */
#define LINE_START_DELAY_MS 600U
#define LINE_ARM_CONFIRM_FRAMES 3U
#define LINE_FAR_HINT_ERROR 18
#define LINE_FAR_CONFIRM_FRAMES 3U
#define LINE_STRAIGHT_CORRIDOR_PERCENT 8
#define LINE_TURN_TRIGGER_ERROR 25
#define LINE_TURN_TRIGGER_FRAMES 2U
#define LINE_TURN_EXIT_ERROR 24
#define LINE_TURN_EXIT_FRAMES 3U
#define LINE_CORNER_WAIT_STOP_MS 800U
#define LINE_TURN_TIMEOUT_MS 1600U
#define LINE_LOST_HOLD_MS 180U
#define LINE_LOST_STOP_MS 900U
#define LINE_FRAME_TIMEOUT_MS 450U
#define LINE_FINISH_CONFIRM_FRAMES 5U

#define LINE_FORWARD_FAST 30   /* 与红外直行速度一致。 */
#define LINE_FORWARD_MEDIUM 22 /* 与红外弯道速度一致。 */
#define LINE_FORWARD_SLOW 22
#define LINE_FORWARD_CRAWL 17  /* 与红外丢线恢复速度一致，避免低速失速。 */
#define LINE_TURN_MAX 19       /* 与红外最大转向量一致。 */
#define LINE_PID_TURN_MAX 8
#define LINE_ERROR_DEADBAND 18
#define LINE_ERROR_MEDIUM 35
#define LINE_ERROR_LARGE 60
#define LINE_PID_KP 12
#define LINE_PID_KI 1
#define LINE_PID_KD 4
#define LINE_PID_SCALE 100
#define LINE_PID_INTEGRAL_LIMIT 40
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
    bool valid;
    bool straight_visible;
    bool far_valid;
    bool lower_valid;
    bool finish_candidate;
    int error;       /* 近端黑线相对画面中心的误差，只供直线 PID 使用。 */
    int far_error;   /* 远端黑线相对近端线的方向，只供记忆左/右弯。 */
    int lower_error; /* 画面下方全部黑像素的重心，只供确认转弯时机。 */
    int threshold;
    uint8_t track_rows;
} line_observation_t;

typedef struct {
    int center;
    int span;
    uint8_t rows;
    uint8_t wide_rows;
} band_measurement_t;

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
static bool s_error_filter_initialized;
static int s_pid_integral;
static int s_pid_previous_error;
static int s_pid_output;
static int s_pending_turn;        /* 镜像校正后：-1=左弯，+1=右弯，0=尚未记忆。 */
static int s_hint_direction;
static uint8_t s_hint_frames;
static uint8_t s_trigger_frames;
static uint8_t s_turn_exit_frames;
static bool s_turning;
static int s_turn_reference_error;
static int64_t s_corner_wait_us;
static int64_t s_turn_started_us;
static int s_command_a;
static int s_command_b;
static int s_command_d;
static int s_last_threshold;
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

static int normalized_error(int delta_x, uint16_t width)
{
    int error = delta_x * 100 / ((int)width / 2);
    error = clamp_int(error, 100);
#if CAMERA_LINE_MIRROR_X
    error = -error;
#endif
    return error;
}

/*
 * 三个区域共用这一段统计：每隔 4 行取样，计算黑像素重心、行间跨度和宽行数。
 * 没有浮点、连通域或动态内存，160x120 图像上只需少量整数加法。
 */
static void measure_band(const uint8_t *frame,
                         uint16_t width,
                         int top,
                         int bottom,
                         int threshold,
                         band_measurement_t *band)
{
    const int x_step = width >= 96 ? 2 : 1;
    const int row_samples = ((int)width + x_step - 1) / x_step;
    int total_x = 0;
    int total_dark = 0;
    int minimum_center = (int)width;
    int maximum_center = 0;
    *band = (band_measurement_t){0};

    for (int y = top; y <= bottom; y += LINE_ROW_STEP) {
        int row_x = 0;
        int row_dark = 0;
        for (int x = 0; x < (int)width; x += x_step) {
            if (rgb565_luma(frame + (((size_t)y * width + (size_t)x) * 2)) <= threshold) {
                row_x += x;
                ++row_dark;
            }
        }
        if (row_dark < LINE_MIN_ROW_SAMPLES) {
            continue;
        }

        const int row_center = row_x / row_dark;
        total_x += row_x;
        total_dark += row_dark;
        ++band->rows;
        if (row_center < minimum_center) {
            minimum_center = row_center;
        }
        if (row_center > maximum_center) {
            maximum_center = row_center;
        }
        if (row_dark * 100 >= row_samples * LINE_FINISH_ROW_COVERAGE_PERCENT) {
            ++band->wide_rows;
        }
    }

    if (total_dark > 0) {
        band->center = total_x / total_dark;
        band->span = maximum_center - minimum_center;
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

    int far_top = (int)height * LINE_FAR_TOP_PERCENT / 100;
    int far_bottom = (int)height * LINE_FAR_BOTTOM_PERCENT / 100;
    int near_top = (int)height * LINE_NEAR_TOP_PERCENT / 100;
    int near_bottom = (int)height * LINE_NEAR_BOTTOM_PERCENT / 100;
    int lower_top = (int)height * LINE_LOWER_TOP_PERCENT / 100;
    far_top = far_top < roi_top ? roi_top : far_top;
    far_bottom = far_bottom > roi_bottom ? roi_bottom : far_bottom;
    near_top = near_top < roi_top ? roi_top : near_top;
    near_bottom = near_bottom > roi_bottom ? roi_bottom : near_bottom;
    near_bottom -= LINE_BOTTOM_SKIP_ROWS * LINE_ROW_STEP;
    lower_top = lower_top < near_top ? near_top : lower_top;
    if (far_bottom <= far_top || near_bottom <= near_top) {
        return false;
    }

    const int x_step = width >= 96 ? 2 : 1;
    const int y_step = LINE_ROW_STEP;

    /* 整个 ROI 只计算一次灰度阈值，避免远近区域各自二值化后标准不一致。 */
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

    /*
     * far 只负责记忆；lower 同时给直线 PID 和转弯触发；near 只判断线形与终点。
     * 因为 PID 完全不读取 far/near 的重心，画面上方的弯线不可能提前拉动后轮。
     */
    band_measurement_t far = {0};
    band_measurement_t near = {0};
    band_measurement_t lower = {0};
    measure_band(frame, width, far_top, far_bottom, threshold, &far);
    measure_band(frame, width, near_top, near_bottom, threshold, &near);
    measure_band(frame, width, lower_top, near_bottom, threshold, &lower);

    observation->finish_candidate = near.wide_rows >= LINE_FINISH_MIN_ROWS &&
                                    near.center >= (int)width / 5 &&
                                    near.center <= (int)width * 4 / 5;
    if (lower.rows >= LINE_MIN_DARK_ROWS) {
        observation->lower_error = normalized_error(lower.center - (int)width / 2, width);
        observation->lower_valid = true;
    }
    if (lower.rows < LINE_MIN_VALID_ROWS || lower.wide_rows > 0) {
        return false;
    }

    observation->error = observation->lower_error;
    observation->track_rows = lower.rows;
    observation->straight_visible = near.rows >= LINE_STRAIGHT_MIN_ROWS &&
                                    near.wide_rows == 0 &&
                                    near.span * 100 <=
                                    (int)width * LINE_STRAIGHT_SPAN_PERCENT;
    if (far.rows >= LINE_MIN_DARK_ROWS) {
        /* 与眼前线比较，整车轻微横移不会被误记成弯道。 */
        observation->far_error = normalized_error(far.center - lower.center, width);
        observation->far_valid = true;
    }
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

static void reset_pid(void)
{
    s_tracking_error = 0;
    s_error_filter_initialized = false;
    s_pid_integral = 0;
    s_pid_previous_error = 0;
    s_pid_output = 0;
}

static void clear_turn_plan(void)
{
    s_pending_turn = 0;
    s_hint_direction = 0;
    s_hint_frames = 0;
    s_trigger_frames = 0;
    s_turn_exit_frames = 0;
    s_turning = false;
    s_turn_reference_error = 0;
    s_corner_wait_us = 0;
    s_turn_started_us = 0;
}

static void reset_tracking(void)
{
    reset_pid();
    clear_turn_plan();
}

static int pid_steering(int error)
{
    /*
     * 低精度定点 PID，只负责直线上的小修正：
     * 3:1 低通抑制摄像头延迟造成的帧间抖动，输出最多 8 且每帧只变 2。
     * 大角度转弯由状态机固定输出，绝不把它混进直线 PID。
     */
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

static int error_direction(int error)
{
    return (error > 0) - (error < 0);
}

/* 远端弯向连续出现 3 帧才保存；保存后不再让远端像素参与本次转弯。 */
static void update_turn_memory(const line_observation_t *observation)
{
    if (s_turning || s_pending_turn != 0 || !observation->valid ||
        !observation->far_valid ||
        abs(observation->far_error) < LINE_FAR_HINT_ERROR) {
        if (s_pending_turn == 0) {
            s_hint_direction = 0;
            s_hint_frames = 0;
        }
        return;
    }

    const int direction = error_direction(observation->far_error);
    if (direction != s_hint_direction) {
        s_hint_direction = direction;
        s_hint_frames = 1;
    } else if (s_hint_frames < LINE_FAR_CONFIRM_FRAMES) {
        ++s_hint_frames;
    }

    if (s_hint_frames >= LINE_FAR_CONFIRM_FRAMES) {
        s_pending_turn = direction;
        s_turn_reference_error = observation->error;
        s_trigger_frames = 0;
        s_corner_wait_us = 0;
        ESP_LOGI(TAG, "Remembered upcoming %s turn; near line remains in control",
                 direction < 0 ? "left" : "right");
    }
}

/*
 * 记住弯向时同时冻结“当前直线”的横坐标。之后只在该坐标左右 8% 宽的
 * 窄走廊里找线并计算 PID；走廊外的未来支路即使像素更多也无法拉动后轮。
 * 当走廊连续看不到线，才认为原直线真正消失。
 */
static bool measure_remembered_straight(const uint8_t *frame,
                                        uint16_t width,
                                        uint16_t height,
                                        int threshold,
                                        int *error)
{
    int image_error = s_turn_reference_error;
#if CAMERA_LINE_MIRROR_X
    image_error = -image_error;
#endif
    const int reference_x = (int)width / 2 + image_error * (int)width / 200;
    const int half_width = (int)width * LINE_STRAIGHT_CORRIDOR_PERCENT / 100;
    int left = reference_x - half_width;
    int right = reference_x + half_width;
    const int top = (int)height * LINE_LOWER_TOP_PERCENT / 100;
    const int bottom = (int)height * LINE_NEAR_BOTTOM_PERCENT / 100 -
                       LINE_BOTTOM_SKIP_ROWS * LINE_ROW_STEP;
    const int x_step = width >= 96 ? 2 : 1;
    int total_x = 0;
    int total_dark = 0;
    int dark_rows = 0;

    left = left < 0 ? 0 : left;
    right = right >= (int)width ? (int)width - 1 : right;
    for (int y = top; y <= bottom; y += LINE_ROW_STEP) {
        int row_dark = 0;
        for (int x = left; x <= right; x += x_step) {
            if (rgb565_luma(frame + (((size_t)y * width + (size_t)x) * 2)) <= threshold) {
                total_x += x;
                ++total_dark;
                ++row_dark;
            }
        }
        if (row_dark >= LINE_MIN_ROW_SAMPLES) {
            ++dark_rows;
        }
    }

    if (dark_rows < LINE_MIN_DARK_ROWS || total_dark == 0) {
        return false;
    }
    *error = normalized_error(total_x / total_dark - (int)width / 2, width);
    return true;
}

/* 下方黑重心必须明显偏向已记住的一侧，反向黑块和居中噪声都不能触发。 */
static bool lower_turn_matches(const line_observation_t *observation)
{
    return observation->lower_valid &&
           abs(observation->lower_error) >= LINE_TURN_TRIGGER_ERROR &&
           error_direction(observation->lower_error) == s_pending_turn;
}

static void disarm_tracking(void)
{
    stop_motors();
    s_armed = false;
    s_arm_frames = 0;
    s_first_frame_us = 0;
    s_last_line_us = 0;
    reset_tracking();
}

static void log_state(const char *mode, const line_observation_t *observation)
{
    const int64_t now = esp_timer_get_time();
    if (s_last_log_us != 0 && now - s_last_log_us < 500000) {
        return;
    }
    s_last_log_us = now;
    ESP_LOGI(TAG, "mode=%s near=%d straight=%d err=%d filt=%d far=%d memory=%d lower=%d rows=%u threshold=%d motor[A,B,D]=[%d,%d,%d]",
             mode,
             observation != NULL && observation->valid,
             observation != NULL && observation->straight_visible,
             observation != NULL && observation->valid ? observation->error : s_last_error,
             s_tracking_error,
             observation != NULL && observation->far_valid ? observation->far_error : 0,
             s_pending_turn,
             observation != NULL && observation->lower_valid ? observation->lower_error : 0,
             (unsigned)(observation != NULL ? observation->track_rows : 0),
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

    if (line_seen) {
        s_last_line_us = now;
        s_last_error = observation.error;
    }

    /* 先确认画面稳定，再允许 STBY；复位后不会因为一帧误检突然起步。 */
    if (!s_armed) {
        if (line_seen && now - s_first_frame_us >= (int64_t)LINE_START_DELAY_MS * 1000) {
            if (s_arm_frames < LINE_ARM_CONFIRM_FRAMES) {
                ++s_arm_frames;
            }
            if (s_arm_frames >= LINE_ARM_CONFIRM_FRAMES) {
                s_armed = true;
                reset_tracking();
                s_last_error = observation.error;
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

    update_turn_memory(&observation);

    /*
     * TURN：真正开始转弯后使用固定的小车转向量，不再受远端像素多少影响。
     * 新直线回到近端中央并稳定 3 帧后退出，随后重新交给小幅 PID。
     */
    if (s_turning) {
        if (observation.valid && observation.straight_visible &&
            abs(observation.error) <= LINE_TURN_EXIT_ERROR) {
            if (s_turn_exit_frames < LINE_TURN_EXIT_FRAMES) {
                ++s_turn_exit_frames;
            }
        } else {
            s_turn_exit_frames = 0;
        }

        if (s_turn_exit_frames >= LINE_TURN_EXIT_FRAMES) {
            clear_turn_plan();
            reset_pid();
            const int turn = pid_steering(observation.error);
            drive(forward_speed(s_tracking_error), turn);
            log_state("TURN-EXIT", &observation);
            return;
        }

        if (now - s_turn_started_us > (int64_t)LINE_TURN_TIMEOUT_MS * 1000) {
            disarm_tracking();
            ESP_LOGW(TAG, "Turn timed out; motors stopped and line confirmation reset");
            log_state("TURN-TIMEOUT", &observation);
            return;
        }

        drive(LINE_FORWARD_CRAWL, -s_pending_turn * LINE_TURN_MAX);
        log_state("TURN", &observation);
        return;
    }

    /*
     * MEMORY：弯向虽已记住，只要原来的近端直线还在，就继续按它做 PID。
     * 原直线消失后先保持 B=0 慢慢前探；只有下方黑重心与记忆同向连续
     * 两帧，才进入 TURN。等待过久则原地停车，但保留画面判断，不盲目搜索。
    */
    if (s_pending_turn != 0) {
        int straight_error = 0;
        if (measure_remembered_straight(rgb565_big_endian, width, height,
                                        observation.threshold, &straight_error)) {
            s_trigger_frames = 0;
            s_corner_wait_us = 0;
            const int turn = pid_steering(straight_error);
            drive(forward_speed(s_tracking_error), turn);
            log_state("STRAIGHT-MEM", &observation);
            return;
        }

        reset_pid();
        if (s_corner_wait_us == 0) {
            s_corner_wait_us = now;
        }
        if (lower_turn_matches(&observation)) {
            if (s_trigger_frames < LINE_TURN_TRIGGER_FRAMES) {
                ++s_trigger_frames;
            }
        } else {
            s_trigger_frames = 0;
        }

        if (s_trigger_frames >= LINE_TURN_TRIGGER_FRAMES) {
            s_turning = true;
            s_turn_started_us = now;
            s_turn_exit_frames = 0;
            drive(LINE_FORWARD_CRAWL, -s_pending_turn * LINE_TURN_MAX);
            log_state("TURN-START", &observation);
        } else if (now - s_corner_wait_us <=
                   (int64_t)LINE_CORNER_WAIT_STOP_MS * 1000) {
            drive(LINE_FORWARD_CRAWL, 0);
            log_state("CORNER-WAIT", &observation);
        } else {
            zero_motor_outputs();
            log_state("CORNER-WAIT-STOP", &observation);
        }
        return;
    }

    if (line_seen) {
        const int turn = pid_steering(observation.error);
        drive(forward_speed(s_tracking_error), turn);
        log_state("LINE", &observation);
        return;
    }

    /* 未记住弯向时绝不按旧误差原地搜索：短暂前探，持续丢线就停车重认。 */
    const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
    if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000) {
        reset_pid();
        drive(LINE_FORWARD_CRAWL, 0);
        log_state("LOST-HOLD", &observation);
    } else {
        zero_motor_outputs();
        log_state("LOST-STOP", &observation);
        if (lost_us >= (int64_t)LINE_LOST_STOP_MS * 1000) {
            disarm_tracking();
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
            disarm_tracking();
            s_last_frame_us = 0;
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
