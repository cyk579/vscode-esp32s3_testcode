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
#include "line_control.h"
#include "line_geometry.h"
#include "line_mixer.h"

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
#define CAMERA_LINE_MIRROR_X 1

/* 摄像头相对车体的安装旋转。扫描坐标系永远是车体视角（sy 越大越靠近车），
 * 缓冲区按这个值反查，不做整帧旋转拷贝，所以改它不增加单帧耗时。
 * 判断方法：看 TFT 上的绿点是**沿着**胶带走还是**横切**胶带；或者看日志
 * 的 valid_rows —— 装反了会一直是 0~3 且反复 LOST/finish。 */
#define CAMERA_LINE_ROTATION LINE_ROTATE_0

/* ROI、行距、逐行搜索和线段判据全部在 line_geometry.h 里，ESP 端和 host
 * 回归测试共用同一组常量。这里只保留状态机自己的窗口策略和时序。 */
#define LINE_LOST_WINDOW_GROW_PERCENT 4
#define LINE_LOST_WINDOW_MAX_PERCENT 42
#define LINE_CORNER_WINDOW_START_PERCENT 28
#define LINE_CORNER_WINDOW_GROW_PERCENT 4
#define LINE_CORNER_WINDOW_MAX_PERCENT 46
#define LINE_REACQUIRE_CONFIRM_FRAMES 2U

/* 彩色干扰守卫：赛道板上的深色红球亮度会落进黑线阈值区间，但通道差很大。
 * 真机采帧确认需要之后再开启，默认关闭以免引入未验证的过滤。 */
#define LINE_SATURATION_GUARD 0
#define LINE_SATURATION_MAX 60
#define LINE_WIDTH_FILTER_OLD 3
#define LINE_WIDTH_FILTER_NEW 1
#define LINE_ERROR_FILTER_OLD 4
#define LINE_ERROR_FILTER_NEW 6

/* 这些值保留原有的上电、限速、换向保护和超时停车行为。 */
#define LINE_START_DELAY_MS 600U
#define LINE_ARM_CONFIRM_FRAMES 3U
#define LINE_MOTOR_START_RAMP_MS 700
#define LINE_MOTOR_START_MIN_OUTPUT 14
#define LINE_LOST_HOLD_MS 300U
#define LINE_LOST_STOP_MS 900U
#define LINE_FRAME_TIMEOUT_MS 1200U
#define LINE_FINISH_CONFIRM_FRAMES 5U
/* 终点 T 停车。调巡线时可以临时置 0，避免把"误停"当成"丢线"。 */
#define LINE_FINISH_ENABLE 1

/* 实车调试阶段四档前进量统一压到最低可持续前进值。 */
#define LINE_FORWARD_FAST 15
#define LINE_FORWARD_MEDIUM 15
#define LINE_FORWARD_SLOW 15
#define LINE_FORWARD_CRAWL 15
#define LINE_FORWARD_SLEW 4

/* 误差门限。|error| 被 ROI 夹在 55 以内（center 只能落在 48..191），所以
 * 原来的 LINE_ERROR_LARGE=60 在 NORMAL 里永远不可达，CRAWL 那一档是死的。 */
#define LINE_ERROR_DEADBAND 8
#define LINE_ERROR_MEDIUM 25
#define LINE_ERROR_LARGE 45
#define LINE_HEADING_DEADBAND 3
#define LINE_HEADING_SLOW 12
#define LINE_FAR_SLOW 35

/* 偏航只做粗对正，横移做精修。两个增益都要等实测速度标定。
 * TODO(实测): KP_LAT 按"1 单位 lat 对应多少 cm/s 侧移"标定；
 * TODO(实测): KH 按"1 单位 heading 对应多少度"标定。 */
#define LINE_PID_KP_LAT 45
#define LINE_PID_KH 130
#define LINE_PID_SCALE 100
#define LINE_TURN_MAX 19
#define LINE_YAW_MIN_OUTPUT 13
#define LINE_LAT_MAX 16
#define LINE_LAT_MIN_OUTPUT 11
#define LINE_SEED_SLEW_PX 12

/* 折角：事件行落到画面下方 80% 以内才动手，之前只降速。 */
#define LINE_TURN_TRIGGER_PERCENT 80
#define LINE_TURN_HINT_FRAMES 2U
#define LINE_TURN_EXIT_FRAMES 2U
/* 旋转至少持续这么久才接受退出，否则第一帧还在看入弯前那条线就会假退出，
 * 然后立刻又检测到同一个折角，来回抖。 */
#define LINE_TURN_MIN_MS 250U
#define LINE_TURN_EXIT_ERROR 30
#define LINE_TURN_TIMEOUT_MS 2500U
/* 绕摄像头旋转：lat = turn * a/(2L)。偏航量必须够大，否则 a/d = lat - turn
 * 落在起转值以下会被混控丢掉，只剩后轮在推。
 * TODO(实测): LINE_CAM_PIVOT_PERCENT 用尺子量 a 和 L 后填 a*100/(2L)。 */
#define LINE_PIVOT_TURN 26
/* 实测 a=7~8 cm、L=9~10 cm -> a/(2L) 约 40%。 */
#define LINE_CAM_PIVOT_PERCENT 40
#define LINE_ALERT_MS 900U

/* 校准模式：置 1 后不跑视觉，直接按脚本输出电机命令并打日志。
 * 把车放在赛道板上（不要架空，静摩擦要真实），看串口 + 卷尺就能得到
 * 三个通道的静摩擦死区和 cm/s，不需要任何仪器。测完记得改回 0。 */
#define LINE_CALIB_MODE 0
#define LINE_CALIB_SETTLE_MS 3000U
#define LINE_CALIB_STEP_MS 1500U
#define LINE_CALIB_STEP_FROM 4
#define LINE_CALIB_STEP_TO 18
#define LINE_CALIB_RUN_MS 3000U
#define LINE_CALIB_SPIN_MS 10000U

/* 扫描几何按这个解码尺寸调过；协商到别的分辨率时绝对像素量的含义会变。 */
#define LINE_EXPECTED_WIDTH 240
#define LINE_EXPECTED_HEIGHT 160

/* 起转下限、启动冲量和方向符号全部复用 car-spin 的实车校准值。红外巡线
 * 工程是在同一台三轮全向车上测过的，这里不要再自行下调：B 轮实测起转
 * 需要 13，低于该值它完全不转，而卡死的后轮会把旋转中心从几何中心挪到
 * 后轮接地点，摄像头的横向扫过量随之放大。 */
#define PWM_MAX 1023U
#define MOTOR_MIN_RUN_OUTPUT 11
#define MOTOR_B_MIN_RUN_OUTPUT 13
#define START_KICK_OUTPUT 15
#define START_KICK_CYCLES 8U
/* 调试阶段把所有实际轮端输出限制在 18 以内：前进量为 15，启动冲量也不
 * 超过 15，转弯混控最多只到 18，避免日志里出现 30 级的突然冲刺。 */
#define MOTOR_PWM_CEILING 18
#define LINE_SPEED_CAP 15
#define MOTOR_TRIM_A 100
#define MOTOR_TRIM_D 100

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
    LINE_STATE_TURN,
    LINE_STATE_LOST,
    LINE_STATE_FINISH,
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
static int64_t s_motor_start_us;
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

static int s_lat_accum;
static int s_yaw_accum;
static int s_forward_output;
static int s_turn_output;
static bool s_error_filter_initialized;
static int s_lateral_control_error;
static int s_heading_control_error;

static int s_turn_direction;
static uint8_t s_turn_hint_frames;
static uint8_t s_turn_exit_frames;
static uint32_t s_turn_frames;
static int64_t s_turn_started_us;
static int64_t s_alert_until_us;
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
static bool s_logged_frame_size;
static bool s_suppress_kick;

static void camera_line_follow_watchdog_task(void *arg);
#if LINE_CALIB_MODE
static void camera_line_calibration_task(void *arg);
#endif

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

/* points[] 是扫描坐标，overlay 要映射回缓冲区坐标才能画在原始帧上。 */
static void render_tracking_overlay(uint8_t *frame,
                                    const line_scan_cfg_t *cfg,
                                    const line_observation_t *observation)
{
    if (frame == NULL || cfg == NULL || observation == NULL) {
        return;
    }
    const uint16_t width = cfg->width;
    const uint16_t height = cfg->height;
    int bx = 0;
    int by = 0;
    /* Keep the debug overlay sparse so it is effectively free at 2.5 FPS. */
    for (int i = 0; i < observation->point_count; i += 2) {
        line_geometry_map(cfg, observation->point_x[i], observation->point_y[i],
                          &bx, &by);
        overlay_dot(frame, width, height, bx, by, 0x07e0);
    }
    if (observation->point_count == 0) {
        return;
    }
    /* 红十字画在实际扫描起始行上，方便用地面胶带量出最近扫描行的落地距离。 */
    line_geometry_map(cfg, observation->point_x[0], observation->scan_bottom_y,
                      &bx, &by);
    overlay_cross(frame, width, height, bx, by, 0xf800);
}

static int state_search_half_percent(void)
{
    if (s_state == LINE_STATE_TURN) {
        const int percent = LINE_CORNER_WINDOW_START_PERCENT +
                            (int)s_turn_frames * LINE_CORNER_WINDOW_GROW_PERCENT;
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
    /* 折角旋转时不要用旧种子：新赛道会从侧面转到中间来，应该在整条 ROI 底部
     * 重新找最靠中心的黑线，而不是围着入弯前的位置猜。 */
    cfg.use_history = s_seed_valid && s_state != LINE_STATE_TURN;
    cfg.seed_x = s_state == LINE_STATE_TURN ? (int)width / 2 : s_seed_x;
    cfg.search_half_percent = state_search_half_percent();
    cfg.expected_width = s_line_width;
    cfg.rotation = CAMERA_LINE_ROTATION;
    cfg.mirror_x = CAMERA_LINE_MIRROR_X ? true : false;
    cfg.saturation_guard = LINE_SATURATION_GUARD ? true : false;
    cfg.saturation_max = LINE_SATURATION_MAX;
    cfg.corridor_x = -1;
    cfg.corridor_half = 0;

    const bool candidate = line_geometry_track(frame, &cfg, observation);
    observation->threshold = source_threshold;
    if (draw_overlay) {
        render_tracking_overlay(frame, &cfg, observation);
    }
    return candidate;
}

static void set_motor_duty(const motor_t *motor, int output)
{
    if (output < 0) {
        output = 0;
    }
    if (output > MOTOR_PWM_CEILING) {
        output = MOTOR_PWM_CEILING;
    }
    const uint32_t duty = PWM_MAX * (uint32_t)output / 100U;
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty);
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

static void motor_set(const motor_t *motor, int command)
{
    /* command 是混控层有符号 PWM；换向时先清零，避免 TB6612 硬切方向。 */
    const int index = (int)motor->channel;
    const int speed = clamp_int(command * motor->sign, MOTOR_PWM_CEILING);
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

    /* 起转值筛选和整向量缩放都在混控层完成，这里不再逐轮抬值：抬值会把
     * 指令向量整体转向（turn=1 被抬到 13 就是 13 倍偏航）。 */
    int output = abs(speed);
    if (s_kick_cycles[index] > 0 && !s_suppress_kick) {
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

/* 启动斜坡只压前进量，而且按时间而不是按帧：摄像头协商到别的帧率时，
 * 按帧计数的斜坡长度会整体变样。转向和横移在起步阶段必须立刻满权限，
 * 否则起点正好在弯上时转不过去。 */
static int forward_ramp_cap(int64_t now)
{
    if (s_motor_start_us == 0) {
        return LINE_SPEED_CAP;
    }
    const int64_t span = (int64_t)LINE_MOTOR_START_RAMP_MS * 1000;
    const int64_t elapsed = now - s_motor_start_us;
    if (elapsed >= span || elapsed < 0) {
        s_motor_start_us = 0;
        return LINE_SPEED_CAP;
    }
    return LINE_MOTOR_START_MIN_OUTPUT +
           (int)((LINE_SPEED_CAP - LINE_MOTOR_START_MIN_OUTPUT) * elapsed / span);
}

static void drive(int forward, int turn, int lat)
{
    const line_mixer_cfg_t cfg = {
        MOTOR_PWM_CEILING, MOTOR_MIN_RUN_OUTPUT, MOTOR_B_MIN_RUN_OUTPUT,
        MOTOR_TRIM_A, MOTOR_TRIM_D,
    };
    line_mixer_out_t out = {0, 0, 0, false, false};
    forward = clamp_int(forward, forward_ramp_cap(esp_timer_get_time()));
    line_mixer_solve(forward, turn, lat, &cfg, &out);
    s_command_a = out.a;
    s_command_b = out.b;
    s_command_d = out.d;
    motor_set(&motor_a, s_command_a);
    motor_set(&motor_b, s_command_b);
    motor_set(&motor_d, s_command_d);
}

static void reset_control(void)
{
    s_lat_accum = 0;
    s_yaw_accum = 0;
    s_forward_output = 0;
    s_turn_output = 0;
    s_error_filter_initialized = false;
    s_lateral_control_error = 0;
    s_heading_control_error = 0;
}

static void reset_tracking(void)
{
    reset_control();
    s_seed_valid = false;
    s_seed_x = 0;
    s_line_width = 0;
    s_lost_frames = 0;
    s_reacquire_frames = 0;
    s_reacquire_x = 0;
    s_turn_direction = 0;
    s_turn_hint_frames = 0;
    s_turn_exit_frames = 0;
    s_turn_frames = 0;
    s_turn_started_us = 0;
    s_alert_until_us = 0;
    s_finish_frames = 0;
    s_state = LINE_STATE_NORMAL;
}

static line_control_cfg_t control_cfg(void)
{
    const line_control_cfg_t cfg = {
        LINE_PID_KH, LINE_PID_KP_LAT, LINE_PID_SCALE, LINE_TURN_MAX,
        LINE_YAW_MIN_OUTPUT, LINE_LAT_MAX, LINE_LAT_MIN_OUTPUT,
        LINE_HEADING_DEADBAND, LINE_ERROR_DEADBAND, LINE_ERROR_MEDIUM,
        LINE_ERROR_LARGE, LINE_HEADING_SLOW, LINE_FAR_SLOW,
        LINE_FORWARD_FAST, LINE_FORWARD_MEDIUM, LINE_FORWARD_SLOW,
        LINE_FORWARD_CRAWL, LINE_MIN_VALID_ROWS,
    };
    return cfg;
}

static int filter_control_error(int current, int *filtered)
{
    if (filtered == NULL) {
        return current;
    }
    if (!s_error_filter_initialized) {
        *filtered = current;
    } else {
        *filtered = (LINE_ERROR_FILTER_OLD * *filtered +
                     LINE_ERROR_FILTER_NEW * current) /
                    (LINE_ERROR_FILTER_OLD + LINE_ERROR_FILTER_NEW);
    }
    return *filtered;
}

/* forward 单独限速率，避免速度档位在门限附近来回跳造成推力抖动。 */
static void drive_normal(const line_observation_t *observation, int64_t now)
{
    const line_control_cfg_t cfg = control_cfg();
    const bool alert = s_alert_until_us != 0 && now < s_alert_until_us;
    const int target = line_control_speed(&cfg, observation, alert);
    const int delta = target - s_forward_output;
    if (delta > LINE_FORWARD_SLEW) {
        s_forward_output += LINE_FORWARD_SLEW;
    } else if (delta < -LINE_FORWARD_SLEW) {
        s_forward_output -= LINE_FORWARD_SLEW;
    } else {
        s_forward_output = target;
    }
    const int lateral_error = filter_control_error(observation->lateral_error,
                                                   &s_lateral_control_error);
    const int heading_error = filter_control_error(observation->heading_error,
                                                   &s_heading_control_error);
    s_error_filter_initialized = true;
    s_turn_output = line_control_yaw(&cfg, heading_error,
                                     &s_yaw_accum);
    drive(s_forward_output, s_turn_output,
          line_control_strafe(&cfg, lateral_error, &s_lat_accum));
}

static void update_seed(const line_observation_t *observation)
{
    if (observation == NULL || !observation->candidate) {
        return;
    }
    /* 种子只做速率限制，不做低通：低通会让搜索窗口系统性滞后于真实线。 */
    if (!s_seed_valid) {
        s_seed_x = observation->seed_x;
    } else {
        const int delta = clamp_int(observation->seed_x - s_seed_x,
                                    LINE_SEED_SLEW_PX);
        s_seed_x += delta;
    }
    s_seed_valid = true;
    if (observation->near_width > 0) {
        s_line_width = s_line_width == 0 ? observation->near_width :
                       (s_line_width * LINE_WIDTH_FILTER_OLD +
                        observation->near_width * LINE_WIDTH_FILTER_NEW) /
                       (LINE_WIDTH_FILTER_OLD + LINE_WIDTH_FILTER_NEW);
    }
}

static const char *state_name(void)
{
    switch (s_state) {
    case LINE_STATE_TURN:
        return "TURN";
    case LINE_STATE_LOST:
        return "LOST";
    case LINE_STATE_FINISH:
        return "FINISH";
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
             s_last_lateral_error, s_last_heading_error, s_turn_direction,
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
    s_motor_start_us = 0;
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
#if LINE_CALIB_MODE
    /* 校准模式下电机由校准任务独占，视觉只负责刷 overlay。 */
    (void)source_threshold;
    (void)draw_overlay;
    (void)width;
    (void)height;
    (void)rgb565_big_endian;
    goto done;
#endif
    if (!s_started || s_finished) {
        if (s_started) {
            stop_motors();
        }
        goto done;
    }
    if (s_first_frame_us == 0) {
        s_first_frame_us = now;
    }
    if (!s_logged_frame_size) {
        s_logged_frame_size = true;
        if (width != LINE_EXPECTED_WIDTH || height != LINE_EXPECTED_HEIGHT) {
            ESP_LOGW(TAG,
                     "decoded frame is %ux%u but the scan geometry was tuned for "
                     "%dx%d; row step, minimum segment width and window limits "
                     "are absolute pixels",
                     (unsigned)width, (unsigned)height,
                     LINE_EXPECTED_WIDTH, LINE_EXPECTED_HEIGHT);
        } else {
            ESP_LOGI(TAG, "decoded frame %ux%u matches the tuned scan geometry",
                     (unsigned)width, (unsigned)height);
        }
    }

    line_observation_t observation = {0};
    if (s_state == LINE_STATE_TURN && s_turn_frames < UINT32_MAX) {
        ++s_turn_frames;
    }
    if (s_state == LINE_STATE_LOST && s_lost_frames < UINT32_MAX) {
        ++s_lost_frames;
    }
    const bool candidate = observe_line(rgb565_big_endian, width, height,
                                        source_threshold, draw_overlay,
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
                reset_control();
                update_seed(&observation);
                s_last_line_us = now;
                s_motor_start_us = now;
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

    /* ---- 终点 T：双侧敞开的横杆 + 下方仍有立柱 ---- */
    if (!observation.finish_candidate) {
        s_finish_frames = 0;
    } else if (LINE_FINISH_ENABLE && candidate && s_state == LINE_STATE_NORMAL) {
        if (s_finish_frames < LINE_FINISH_CONFIRM_FRAMES) {
            ++s_finish_frames;
        }
        if (s_finish_frames >= LINE_FINISH_CONFIRM_FRAMES) {
            s_finished = true;
            s_state = LINE_STATE_FINISH;
            stop_motors();
            ESP_LOGW(TAG, "finish T confirmed at row %d; motors stopped",
                     observation.corner_row_y);
            goto done;
        }
        /* 确认期间继续爬行，让车停在横杆上而不是提前刹住。 */
        drive(LINE_FORWARD_CRAWL, 0, 0);
        goto done;
    }

    /* ---- 折角：绕摄像头原地旋转，用近场闭环退出 ---- */
    if (s_state == LINE_STATE_TURN) {
        const bool settled = s_turn_started_us != 0 &&
                             now - s_turn_started_us >=
                                 (int64_t)LINE_TURN_MIN_MS * 1000;
        const bool aligned = settled && candidate && observation.near_line_visible &&
                             observation.corner_direction == 0 &&
                             abs(observation.lateral_error) <= LINE_TURN_EXIT_ERROR;
        if (aligned) {
            if (s_turn_exit_frames < LINE_TURN_EXIT_FRAMES) {
                ++s_turn_exit_frames;
            }
        } else {
            s_turn_exit_frames = 0;
        }
        if (s_turn_exit_frames >= LINE_TURN_EXIT_FRAMES) {
            s_state = LINE_STATE_NORMAL;
            s_turn_direction = 0;
            s_turn_frames = 0;
            s_turn_exit_frames = 0;
            s_turn_started_us = 0;
            /* 出弯后一段时间内限速：赛道右上角两个直角弯之间只有 10 cm。 */
            s_alert_until_us = now + (int64_t)LINE_ALERT_MS * 1000;
            s_lost_frames = 0;
            s_reacquire_frames = 0;
            s_last_line_us = now;
            update_seed(&observation);
            reset_control();
            ESP_LOGI(TAG, "turn complete; back to NORMAL at ey=%d",
                     observation.lateral_error);
            drive_normal(&observation, now);
            goto done;
        }
        if (s_turn_started_us != 0 &&
            now - s_turn_started_us > (int64_t)LINE_TURN_TIMEOUT_MS * 1000) {
            s_state = LINE_STATE_LOST;
            s_lost_frames = 1;
            s_turn_direction = 0;
            s_turn_started_us = 0;
            s_reacquire_frames = 0;
            s_reacquire_x = s_seed_x;
            zero_motor_outputs();
            ESP_LOGW(TAG, "turn timed out without reacquiring the line");
            goto done;
        }
        /* 摄像头在旋转中心前方 a 处，纯原地旋转会让它横扫 a*theta 而丢线。
         * 叠加 lat = turn * a/(2L) 就变成绕镜头旋转，全程保住近场视野。 */
        {
            const int turn = -s_turn_direction * LINE_PIVOT_TURN;
            drive(0, turn, turn * LINE_CAM_PIVOT_PERCENT / 100);
        }
        goto done;
    }

    /* ---- LOST：只在最后可信种子附近重捕获 ---- */
    if (s_state == LINE_STATE_LOST) {
        const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
        if (candidate) {
            const int confirm_window = positive_percent((int)width,
                                                        LINE_SEARCH_HALF_PERCENT,
                                                        LINE_SEARCH_HALF_MIN);
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
                reset_control();
                drive_normal(&observation, now);
                goto done;
            }
            if (near_last_seed) {
                /* 已经看见线但还没确认：用横移把它拉回中间，不要转车身。 */
                /* seed_x 是原始图像坐标，控制量要按镜像设置翻一次。 */
                const int mirror = CAMERA_LINE_MIRROR_X ? -1 : 1;
                const int correction =
                    mirror * direction_of(observation.seed_x - s_seed_x);
                drive(LINE_FORWARD_CRAWL, 0, correction * LINE_LAT_MIN_OUTPUT);
            } else {
                drive(LINE_FORWARD_CRAWL, 0, 0);
            }
        } else {
            s_reacquire_frames = 0;
            s_reacquire_x = s_seed_x;
            if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000) {
                drive(LINE_FORWARD_CRAWL, 0, 0);
            } else {
                zero_motor_outputs();
            }
        }
        if (lost_us >= (int64_t)LINE_LOST_STOP_MS * 1000) {
            disarm_tracking();
        }
        goto done;
    }

    /* ---- NORMAL ---- */
    if (candidate) {
        s_state = LINE_STATE_NORMAL;
        s_lost_frames = 0;
        s_reacquire_frames = 0;
        s_last_line_us = now;
        update_seed(&observation);

        /* 近场折角事件才动手；远处的同一个事件只在 forward_speed 里降速。 */
        const int trigger_y = (int)height * LINE_TURN_TRIGGER_PERCENT / 100;
        if (observation.corner_direction != 0 &&
            observation.corner_row_y >= trigger_y) {
            if (observation.corner_direction == s_turn_direction) {
                if (s_turn_hint_frames < LINE_TURN_HINT_FRAMES) {
                    ++s_turn_hint_frames;
                }
            } else {
                s_turn_direction = observation.corner_direction;
                s_turn_hint_frames = 1;
            }
            if (s_turn_hint_frames >= LINE_TURN_HINT_FRAMES) {
                s_state = LINE_STATE_TURN;
                s_turn_frames = 0;
                s_turn_exit_frames = 0;
                s_turn_hint_frames = 0;
                s_turn_started_us = now;
                reset_control();
                ESP_LOGI(TAG, "corner %s at row %d; pivoting about the camera",
                         s_turn_direction < 0 ? "left" : "right",
                         observation.corner_row_y);
                const int turn = -s_turn_direction * LINE_PIVOT_TURN;
                drive(0, turn, turn * LINE_CAM_PIVOT_PERCENT / 100);
                goto done;
            }
        } else {
            s_turn_hint_frames = 0;
            s_turn_direction = 0;
        }
        drive_normal(&observation, now);
        goto done;
    }

    s_state = LINE_STATE_LOST;
    s_lost_frames = 1;
    s_reacquire_frames = 0;
    s_reacquire_x = s_seed_x;
    reset_control();
    {
        const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
        if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000) {
            drive(LINE_FORWARD_CRAWL, 0, 0);
        } else {
            zero_motor_outputs();
            if (lost_us >= (int64_t)LINE_LOST_STOP_MS * 1000) {
                disarm_tracking();
            }
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

#if LINE_CALIB_MODE
/*
 * 单通道阶梯：每级 1.5 s，眼看第几级车子开始动，那一级就是该通道的静摩擦
 * 死区。之后是定时跑，用卷尺量位移换成 cm/s；旋转固定跑 10 s 数圈数。
 */
/*
 * 阶梯专用输出：绕过混控的起转值过滤，并临时关掉换向冲量。
 *
 * 两件事都必须做，否则测出来的不是真实静摩擦：
 *   - 混控会把低于 11/13 的分量直接置零，阶梯永远"从地板值开始动"；
 *   - 换向冲量按调用次数衰减，而阶梯每级只调用一次，8 级都会被抬到启动上限。
 */
static void calibration_drive_raw(int forward, int turn, int lat)
{
    const line_mixer_cfg_t cfg = {
        MOTOR_PWM_CEILING, 0, 0, MOTOR_TRIM_A, MOTOR_TRIM_D,
    };
    line_mixer_out_t out = {0, 0, 0, false, false};
    line_mixer_solve(forward, turn, lat, &cfg, &out);
    s_command_a = out.a;
    s_command_b = out.b;
    s_command_d = out.d;
    motor_set(&motor_a, out.a);
    motor_set(&motor_b, out.b);
    motor_set(&motor_d, out.d);
}

static void calibration_stair(const char *label, int channel)
{
    ESP_LOGW(TAG, "CALIB stair %s: %d..%d, %u ms per step "
                  "(mixer floors and the kick are bypassed here)",
             label, LINE_CALIB_STEP_FROM, LINE_CALIB_STEP_TO,
             (unsigned)LINE_CALIB_STEP_MS);
    s_suppress_kick = true;
    for (int v = LINE_CALIB_STEP_FROM; v <= LINE_CALIB_STEP_TO; ++v) {
        ESP_LOGW(TAG, "CALIB %s=%d  motor[A,B,D]=[%d,%d,%d]", label, v,
                 s_command_a, s_command_b, s_command_d);
        calibration_drive_raw(channel == 0 ? v : 0, channel == 1 ? v : 0,
                              channel == 2 ? v : 0);
        vTaskDelay(pdMS_TO_TICKS(LINE_CALIB_STEP_MS));
    }
    s_suppress_kick = false;
    zero_motor_outputs();
    vTaskDelay(pdMS_TO_TICKS(LINE_CALIB_STEP_MS));
}

static void calibration_run(const char *label, int forward, int turn, int lat,
                            uint32_t duration_ms)
{
    ESP_LOGW(TAG, "CALIB run %s: forward=%d turn=%d lat=%d for %u ms",
             label, forward, turn, lat, (unsigned)duration_ms);
    drive(forward, turn, lat);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    zero_motor_outputs();
    ESP_LOGW(TAG, "CALIB run %s done; measure the displacement now", label);
    vTaskDelay(pdMS_TO_TICKS(LINE_CALIB_SETTLE_MS));
}

static void camera_line_calibration_task(void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "CALIB mode: put the car on the track surface, not on blocks");
    vTaskDelay(pdMS_TO_TICKS(LINE_CALIB_SETTLE_MS));
    gpio_set_level(STBY_GPIO, 1);
    s_stby_enabled = true;
    s_motor_start_us = 0;

    calibration_stair("forward", 0);
    calibration_stair("turn", 1);
    calibration_stair("lat", 2);

    calibration_run("forward", 20, 0, 0, LINE_CALIB_RUN_MS);
    calibration_run("lat", 0, 0, 14, LINE_CALIB_RUN_MS);
    calibration_run("spin", 0, LINE_TURN_MAX, 0, LINE_CALIB_SPIN_MS);
    calibration_run("pivot", 0, LINE_PIVOT_TURN,
                    LINE_PIVOT_TURN * LINE_CAM_PIVOT_PERCENT / 100,
                    LINE_CALIB_SPIN_MS);

    stop_motors();
    ESP_LOGW(TAG, "CALIB done: set LINE_CALIB_MODE back to 0");
    vTaskDelete(NULL);
}
#endif

static void camera_line_follow_watchdog_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!s_started || s_control_mutex == NULL) {
            continue;
        }
#if LINE_CALIB_MODE
        /* 校准脚本自己管电机；解码帧超时不能在测试中途把 STBY 拉低。 */
        continue;
#endif
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
    s_motor_start_us = 0;
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
#if LINE_CALIB_MODE
    if (xTaskCreate(camera_line_calibration_task, "camera_line_calib", 3072,
                    NULL, 2, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not create the motor calibration task");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGW(TAG, "LINE_CALIB_MODE is on; vision control is disabled");
#endif
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
