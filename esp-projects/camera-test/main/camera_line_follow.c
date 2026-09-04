#include "camera_line_follow.h"

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

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
#include "tcp_server.h"
#include "ultrasonic.h"
#if CONFIG_EXAMPLE_ENABLE_TFT_PREVIEW
#include "tft_st7735.h"
#endif

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
/*
 * The chassis center is about 5 cm left of the tape in the real vehicle.
 * The negative reference offset is intentional for this installation: it
 * moves the accepted track reference left of the optical center, compensating
 * the measured chassis/camera placement without changing the mixer.
 */
#define CAMERA_LINE_CENTER_BIAS_PX (-18)

/* 摄像头相对车体的安装旋转。扫描坐标系永远是车体视角（sy 越大越靠近车），
 * 缓冲区按这个值反查，不做整帧旋转拷贝，所以改它不增加单帧耗时。
 * 判断方法：看 TFT 上的绿点是**沿着**胶带走还是**横切**胶带；或者看日志
 * 的 valid_rows —— 装反了会一直是 0~3 且反复 LOST/finish。 */
#define CAMERA_LINE_ROTATION LINE_ROTATE_0

/* ROI、行距、逐行搜索和线段判据全部在 line_geometry.h 里，ESP 端和 host
 * 回归测试共用同一组常量。这里只保留状态机自己的窗口策略和时序。 */
#define LINE_LOST_WINDOW_GROW_PERCENT 4
#define LINE_LOST_WINDOW_MAX_PERCENT 60
#define LINE_CORNER_WINDOW_START_PERCENT 40
#define LINE_CORNER_WINDOW_GROW_PERCENT 4
#define LINE_CORNER_WINDOW_MAX_PERCENT 60
#define LINE_REACQUIRE_CONFIRM_FRAMES 2U

/* 彩色干扰守卫：赛道板上的深色红球亮度会落进黑线阈值区间，但通道差很大。
 * 真机采帧确认需要之后再开启，默认关闭以免引入未验证的过滤。 */
#define LINE_SATURATION_GUARD 0
#define LINE_SATURATION_MAX 60
#define LINE_WIDTH_FILTER_OLD 3
#define LINE_WIDTH_FILTER_NEW 1
#define LINE_ERROR_FILTER_OLD 2
#define LINE_ERROR_FILTER_NEW 4
#define LINE_PD_KD_LAT 20
#define LINE_PD_KD_HEADING 50

/* 这些值保留原有的上电、限速、换向保护和超时停车行为。 */
#define LINE_START_DELAY_MS 600U
#define LINE_ARM_CONFIRM_FRAMES 3U
#define LINE_ARM_MIN_VALID_ROWS 4U
#define LINE_MOTOR_START_RAMP_MS 700
#define LINE_MOTOR_START_MIN_OUTPUT 14
#define LINE_LOST_HOLD_MS 600U
#define LINE_LOST_STOP_MS 1800U
#define LINE_FRAME_TIMEOUT_MS 1200U
#define LINE_FINISH_CONFIRM_FRAMES 5U
/* At a T finish, keep crawling and use heading only to leave the nose
 * pointing forward before the confirmation stop. */
#define LINE_FINISH_ALIGN_HEADING_ERROR 8
#define LINE_FINISH_ALIGN_TIMEOUT_MS 1500U
/* 终点 T 停车。调巡线时可以临时置 0，避免把"误停"当成"丢线"。 */
#define LINE_FINISH_ENABLE 1

/* 持续巡航速度取 24% 版和 30% 版之间的折中值；起步另有短时增扭脉冲。 */
#define LINE_FORWARD_FAST 27
#define LINE_FORWARD_MEDIUM 25
#define LINE_FORWARD_SLOW 22
#define LINE_FORWARD_CRAWL 18
#define LINE_FORWARD_SLEW 8

/* A two-point partial scan is often a one-frame JPEG/lighting miss, not a
 * genuine loss of the track. Keep the previous steering briefly while the
 * detector catches up, but do not let partial noise extend the line age. */
#define LINE_PARTIAL_MIN_POINTS 2
#define LINE_PARTIAL_TRACK_HOLD_MS 450U

/* 误差门限。|error| 被 ROI 夹在 55 以内（center 只能落在 48..191），所以
 * 原来的 LINE_ERROR_LARGE=60 在 NORMAL 里永远不可达，CRAWL 那一档是死的。 */
#define LINE_ERROR_DEADBAND 14
#define LINE_ERROR_MEDIUM 25
#define LINE_ERROR_LARGE 45
#define LINE_HEADING_DEADBAND 3
#define LINE_HEADING_SLOW 12
#define LINE_FAR_SLOW 35

/* 偏航负责车体对正；横向误差只作为同一 yaw 控制量的辅助项。
 * 两个增益都要等实测速度标定。
 * TODO(实测): KP_LAT 按"1 单位 lat 对应多少 cm/s 侧移"标定；
 * TODO(实测): KH 按"1 单位 heading 对应多少度"标定。 */
#define LINE_PID_KP_LAT 45
#define LINE_PID_KH 130
#define LINE_PID_SCALE 100
#define LINE_TURN_MAX 16
#define LINE_YAW_MIN_OUTPUT 13
#define LINE_LAT_MAX 12
#define LINE_LAT_MIN_OUTPUT 11
#define LINE_SEED_SLEW_PX 12

/* 折角：事件行进入画面下方 88% 才动手；更远处只降速，避免提前转弯。 */
#define LINE_TURN_TRIGGER_PERCENT 80
#define LINE_TURN_HINT_FRAMES 1U
#define LINE_TURN_EXIT_FRAMES 2U
/* 旋转至少持续这么久才接受退出，否则第一帧还在看入弯前那条线就会假退出，
 * 然后立刻又检测到同一个折角，来回抖。 */
#define LINE_TURN_MIN_MS 250U
#define LINE_TURN_EXIT_ERROR 30
#define LINE_TURN_TIMEOUT_MS 2500U
/* 绕摄像头旋转：lat = turn * a/(2L)。偏航量必须够大，否则 a/d = lat - turn
 * 落在起转值以下会被混控丢掉，只剩后轮在推。
 * TODO(实测): LINE_CAM_PIVOT_PERCENT 用尺子量 a 和 L 后填 a*100/(2L)。 */
#define LINE_PIVOT_TURN 17
#define LINE_TURN_A_SPEED 15
#define LINE_TURN_B_SPEED 15
#define LINE_TURN_D_SPEED 16
#define LINE_TURN_PENDING_MS 900U
#define LINE_SOFT_LOST_FRAMES 3U
#define LINE_OVERLAY_HOLD_FRAMES 4U
/* 低速调试时优先保证三轮都超过各自起转阈值；此前叠加 40% 横移后，
 * 在较低总上限下 A/D 被缩放掉，锐角阶段实际只剩 B 轮。 */
#define LINE_CAM_PIVOT_PERCENT 0
#define LINE_ALERT_MS 900U

/* 校准模式：置 1 后不跑视觉，直接按脚本输出电机命令并打日志。
 * 把车放在赛道板上（不要架空，静摩擦要真实），看串口 + 卷尺就能得到
 * 三个通道的静摩擦死区和 cm/s，不需要任何仪器。测完记得改回 0。 */
#define LINE_CALIB_MODE 0
#define LINE_CALIB_SETTLE_MS 3000U
#define LINE_CALIB_STEP_MS 1500U
#define LINE_CALIB_STEP_FROM 4
#define LINE_CALIB_STEP_TO 24
#define LINE_CALIB_RUN_MS 3000U
#define LINE_CALIB_SPIN_MS 10000U

/* 扫描几何按这个解码尺寸调过；协商到别的分辨率时绝对像素量的含义会变。 */
#define LINE_EXPECTED_WIDTH 120
#define LINE_EXPECTED_HEIGHT 80

/* 起转下限、启动冲量和方向符号全部复用 car-spin 的实车校准值。红外巡线
 * 工程是在同一台三轮全向车上测过的，这里不要再自行下调：B 轮实测起转
 * 需要 13，低于该值它完全不转，而卡死的后轮会把旋转中心从几何中心挪到
 * 后轮接地点，摄像头的横向扫过量随之放大。 */
#define PWM_MAX 1023U
#define MOTOR_MIN_RUN_OUTPUT 11
#define MOTOR_B_MIN_RUN_OUTPUT 13
#define START_KICK_OUTPUT 30
#define START_KICK_CYCLES 6U
/* 持续巡航上限为 27%；30% 只作为起步/换向脉冲和混控余量。 */
#define MOTOR_PWM_CEILING 30
#define LINE_SPEED_CAP 27
#define MOTOR_TRIM_A 90
#define MOTOR_TRIM_D 100

/* 方向符号与 car-spin 的实车校准一致。 */
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN -1

/* HC-SR04 is the only obstacle sensor in the camera vehicle. */
#define ULTRASONIC_TRIG GPIO_NUM_18
#define ULTRASONIC_ECHO GPIO_NUM_11
#define ULTRASONIC_MIN_CM 2.0f
#define ULTRASONIC_MAX_CM 400.0f
#define OBSTACLE_DETECT_CM 10.0f
#define OBSTACLE_CLEAR_CM 80.0f
#define OBSTACLE_CLOSE_CONFIRM_SAMPLES 2U
#define ULTRASONIC_PERIOD_MS 60U
#define AVOID_BRAKE_MS 500U
#define AVOID_LEFT_SIDE_SPEED 18
#define AVOID_LEFT_B_SPEED 25
#define AVOID_RIGHT_A_SPEED 15
#define AVOID_RIGHT_B_SPEED 25
#define AVOID_RIGHT_D_SPEED 20
#define AVOID_LEFT_MS 2500U
#define AVOID_FORWARD_A_SPEED 25
#define AVOID_FORWARD_D_SPEED 20
#define AVOID_FORWARD_MS 2000U
/* No IR sensor is available here: finish the return strafe by time. */
#define AVOID_RIGHT_MS 2500U
#define AVOID_REACQUIRE_GRACE_MS 1000U
#define AVOID_COOLDOWN_MS 1200U

/* End-point ball handling is a separate task phase; it never changes the
 * line-follow state enum.  Times are deliberately conservative because the
 * camera is tilted up and the vehicle should move only after a stable visual
 * alignment. */
#define PAN_SERVO_PIN GPIO_NUM_1
#define TILT_SERVO_PIN GPIO_NUM_2
#define SERVO_TIMER LEDC_TIMER_1
#define SERVO_PAN_CHANNEL LEDC_CHANNEL_3
#define SERVO_TILT_CHANNEL LEDC_CHANNEL_4
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY ((1U << 14) - 1U)
#define SERVO_PERIOD_US 20000U
#define BALL_PAN_CENTER_US 1500U
#define BALL_TILT_HIGH_US 1750U
#define BALL_SERVO_SETTLE_MS 700U
#define BALL_SEARCH_TURN_SPEED 13
#define BALL_ALIGN_TURN_SPEED 12
#define BALL_CHARGE_A_SPEED 30
#define BALL_CHARGE_D_SPEED 25
#define BALL_CHARGE_MS 3000U
#define BALL_SEARCH_TIMEOUT_MS 14000U
#define BALL_ALIGN_TIMEOUT_MS 5000U
#define BALL_ALIGN_TOLERANCE_PX 7
#define BALL_ALIGN_CONFIRM_FRAMES 4U
#define BALL_MIN_PIXELS 10
#define BALL_SCAN_TOP_PERCENT 5
#define BALL_SCAN_BOTTOM_PERCENT 90
#define BALL_RED_MIN_R 90
#define BALL_RED_CHANNEL_GAP 35
#define BALL_GREEN_MIN_G 70
#define BALL_GREEN_CHANNEL_GAP 8
#define BALL_MIN_LUMA 35
#define BALL_MIN_FILL_PERCENT 50
#define BALL_MIN_EDGE_PIXELS 6
#define BALL_MIN_BORDER_MARGIN 4
#define BALL_MAX_WIDTH_PERCENT 20
#define BALL_ZONE_MIN_RUN_PIXELS 8
#define BALL_ZONE_MIN_ROWS 3
#define BALL_ZONE_THRESHOLD 55
#define BALL_GRID_STEP 1
#define BALL_GRID_MAX_CELLS 20000

/* subject2-findball is a standalone endpoint test image.  The vehicle is
 * placed at the finish and must enter the ball state machine directly; the
 * line detector, finish detector, and obstacle route are skipped. */
#define BALL_DIRECT_TEST_MODE 1

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

typedef enum {
    OBSTACLE_IDLE = 0,
    OBSTACLE_BRAKE,
    OBSTACLE_LEFT,
    OBSTACLE_FORWARD,
    OBSTACLE_RIGHT,
} obstacle_state_t;

typedef enum {
    BALL_IDLE = 0,
    BALL_SEARCH_RED,
    BALL_ALIGN_RED,
    BALL_FORWARD_RED,
    BALL_BACK_RED,
    BALL_SEARCH_GREEN,
    BALL_ALIGN_GREEN,
    BALL_FORWARD_GREEN,
    BALL_BACK_GREEN,
    BALL_DONE,
} ball_phase_t;

typedef enum {
    BALL_COLOUR_RED = 0,
    BALL_COLOUR_GREEN,
} ball_colour_t;

typedef struct {
    bool valid;
    int x;
    int y;
    int width;
    int height;
    int pixels;
    int edge_score;
} ball_blob_t;

typedef struct {
    bool valid;
    int x;
    int y;
    int width;
    int rows;
    int score;
} ball_zone_t;

static const char *TAG = "camera_line";
static const motor_t motor_a = {A_IN1, A_IN2, LEDC_CHANNEL_0, MOTOR_A_SIGN};
static const motor_t motor_b = {B_IN1, B_IN2, LEDC_CHANNEL_1, MOTOR_B_SIGN};
static const motor_t motor_d = {D_IN1, D_IN2, LEDC_CHANNEL_2, MOTOR_D_SIGN};

static volatile bool s_started;
static bool s_armed;
static bool s_finished;
static bool s_stby_enabled;
static volatile float s_ultrasonic_distance_cm = -1.0f;
static volatile bool s_ultrasonic_valid;
static volatile uint32_t s_ultrasonic_sequence;
static bool s_ultrasonic_task_created;
static obstacle_state_t s_obstacle_state;
static uint32_t s_obstacle_close_samples;
static uint32_t s_obstacle_last_sequence;
static int64_t s_obstacle_phase_start_us;
static int64_t s_obstacle_cooldown_until_us;
static int64_t s_obstacle_reacquire_until_us;
static bool s_obstacle_ready;
static bool s_servo_initialized;
static ball_phase_t s_ball_phase;
static int64_t s_ball_phase_start_us;
static uint8_t s_ball_align_frames;
static int s_ball_x;
static int s_ball_y;
static bool s_ball_target_valid;
static int s_ball_search_direction;
static uint8_t s_ball_target_miss_frames;
static uint8_t s_arm_frames;
static int64_t s_motor_start_us;
static uint8_t s_finish_frames;
static int64_t s_finish_align_start_us;
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
static bool s_last_candidate_held;
static int s_last_seed_x;
static uint16_t s_last_frame_width;
static uint16_t s_last_frame_height;
static int s_last_scan_bottom_y;
static int s_last_point_count;
static int s_last_near_normal_rows;
static int s_last_far_error;
static int s_last_corner_direction;
static int s_last_corner_row_y;
static bool s_last_near_line_visible;

static int s_lat_accum;
static int s_yaw_accum;
static int s_forward_output;
static int s_turn_output;
static bool s_error_filter_initialized;
static int s_lateral_control_error;
static int s_heading_control_error;
static int s_lateral_error_rate;
static int s_heading_error_rate;
static int s_last_lateral_raw;
static int s_last_lateral_filtered;
static int s_last_lateral_delta;
static int s_last_heading_raw;
static int s_last_heading_filtered;
static int s_last_heading_delta;
static int s_last_forward_target;
static int s_last_forward_ramped;
static int s_last_drive_forward;
static int s_last_drive_turn;
static int s_last_drive_lat;
static int s_mix_pre_a;
static int s_mix_pre_b;
static int s_mix_pre_d;
static int s_mix_post_a;
static int s_mix_post_b;
static int s_mix_post_d;
static bool s_mix_scaled;
static bool s_mix_dropped;
static bool s_mix_repaired;

static int s_turn_direction;
static uint8_t s_turn_hint_frames;
static int64_t s_turn_pending_until_us;
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
static line_observation_t s_last_good_observation;
static bool s_have_last_good_observation;
static uint8_t s_candidate_miss_frames;
static uint8_t s_overlay_miss_frames;

static uint32_t s_control_frames;
static uint32_t s_line_us_sum;
static uint32_t s_line_us_max;
static int64_t s_summary_start_us;
static uint32_t s_summary_camera_frames;
static uint32_t s_summary_processed_frames;
static uint32_t s_summary_control_frames;
static uint32_t s_summary_preview_frames;
static uint32_t s_summary_dropped_frames;
static uint32_t s_callback_dropped_frames;
static uint32_t s_summary_callback_dropped_frames;

static SemaphoreHandle_t s_control_mutex;
static bool s_watchdog_created;
static bool s_logged_frame_size;
static bool s_suppress_kick;

#define LINE_OVERLAY_MAX_POINTS (LINE_SCAN_MAX_ROWS / 2 + 1)
typedef struct {
    bool valid;
    uint16_t control_width;
    uint16_t control_height;
    line_rotation_t rotation;
    uint8_t point_count;
    int16_t point_x[LINE_OVERLAY_MAX_POINTS];
    int16_t point_y[LINE_OVERLAY_MAX_POINTS];
    int16_t seed_x;
    int16_t seed_y;
    bool corner_valid;
    int16_t corner_x;
    int16_t corner_y;
    uint32_t sequence;
} line_overlay_snapshot_t;

static line_overlay_snapshot_t s_overlay_snapshot;
static SemaphoreHandle_t s_overlay_mutex;

static void camera_line_follow_watchdog_task(void *arg);
static void camera_ultrasonic_task(void *arg);
static void disarm_tracking(void);
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

static bool turn_pending_active(int64_t now)
{
    return s_turn_direction != 0 &&
           s_turn_hint_frames >= LINE_TURN_HINT_FRAMES &&
           s_turn_pending_until_us > now;
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

/* Keep only the detector's sparse points. Preview never runs a second scan. */
static void save_overlay_snapshot(uint16_t width, uint16_t height,
                                  const line_observation_t *observation,
                                  uint32_t sequence)
{
    if (observation == NULL || s_overlay_mutex == NULL) {
        return;
    }
    if (xSemaphoreTake(s_overlay_mutex, 0) != pdTRUE) {
        return;
    }
    if (observation->point_count <= 0) {
        /* 保留最近一组真实点几个控制周期，避免一次阈值抖动就让 TFT
         * 突然变成没有绿点；超过保持窗口后再明确清空。 */
        if (s_overlay_snapshot.valid) {
            if (s_overlay_miss_frames < UINT8_MAX) {
                ++s_overlay_miss_frames;
            }
            if (s_overlay_miss_frames > LINE_OVERLAY_HOLD_FRAMES) {
                s_overlay_snapshot.valid = false;
            }
        }
        (void)xSemaphoreGive(s_overlay_mutex);
        return;
    }
    s_overlay_miss_frames = 0;
    s_overlay_snapshot.valid = true;
    s_overlay_snapshot.control_width = width;
    s_overlay_snapshot.control_height = height;
    s_overlay_snapshot.rotation = CAMERA_LINE_ROTATION;
    s_overlay_snapshot.point_count = 0;
    s_overlay_snapshot.seed_x = -1;
    s_overlay_snapshot.seed_y = -1;
    s_overlay_snapshot.corner_valid = false;
    s_overlay_snapshot.corner_x = -1;
    s_overlay_snapshot.corner_y = -1;
    s_overlay_snapshot.sequence = sequence;
    for (int i = 0; i < observation->point_count &&
                    s_overlay_snapshot.point_count < LINE_OVERLAY_MAX_POINTS; i += 2) {
        const uint8_t index = s_overlay_snapshot.point_count++;
        s_overlay_snapshot.point_x[index] = observation->point_x[i];
        s_overlay_snapshot.point_y[index] = observation->point_y[i];
    }
    if (observation->point_count > 0) {
        s_overlay_snapshot.seed_x = observation->point_x[0];
        s_overlay_snapshot.seed_y = observation->scan_bottom_y;
    }
    if (observation->corner_direction != 0 && observation->corner_x >= 0 &&
        observation->corner_row_y >= 0) {
        s_overlay_snapshot.corner_valid = true;
        s_overlay_snapshot.corner_x = observation->corner_x;
        s_overlay_snapshot.corner_y = observation->corner_row_y;
    }
    (void)xSemaphoreGive(s_overlay_mutex);
}

static void render_preview_overlay(uint8_t *frame, uint16_t width, uint16_t height)
{
    if (frame == NULL || s_overlay_mutex == NULL) {
        return;
    }
    line_overlay_snapshot_t snapshot;
    if (xSemaphoreTake(s_overlay_mutex, 0) != pdTRUE) {
        return;
    }
    snapshot = s_overlay_snapshot;
    (void)xSemaphoreGive(s_overlay_mutex);
    if (!snapshot.valid || snapshot.control_width == 0 ||
        snapshot.control_height == 0) {
        return;
    }

    line_scan_cfg_t cfg = {0};
    cfg.width = snapshot.control_width;
    cfg.height = snapshot.control_height;
    cfg.rotation = snapshot.rotation;
    int bx = 0;
    int by = 0;
    for (uint8_t i = 0; i < snapshot.point_count; ++i) {
        line_geometry_map(&cfg, snapshot.point_x[i], snapshot.point_y[i], &bx, &by);
        const int px = bx * (int)width / snapshot.control_width;
        const int py = by * (int)height / snapshot.control_height;
        overlay_dot(frame, width, height, px, py, 0x07e0);
    }
    if (snapshot.seed_x >= 0 && snapshot.seed_y >= 0) {
        line_geometry_map(&cfg, snapshot.seed_x, snapshot.seed_y, &bx, &by);
        const int px = bx * (int)width / snapshot.control_width;
        const int py = by * (int)height / snapshot.control_height;
        overlay_cross(frame, width, height, px, py, 0xf800);
    }
    if (snapshot.corner_valid) {
        line_geometry_map(&cfg, snapshot.corner_x, snapshot.corner_y, &bx, &by);
        const int px = bx * (int)width / snapshot.control_width;
        const int py = by * (int)height / snapshot.control_height;
        overlay_dot(frame, width, height, px, py, 0xffe0);
    }
}

static int state_search_half_percent(void)
{
    if (s_state == LINE_STATE_CORNER) {
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
    observation->corner_x = -1;
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
    cfg.use_history = s_seed_valid && s_state != LINE_STATE_CORNER;
    cfg.seed_x = s_state == LINE_STATE_CORNER ? (int)width / 2 : s_seed_x;
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
    if (candidate && CAMERA_LINE_CENTER_BIAS_PX != 0 && width >= 2) {
        /* Shift only the lateral reference. Heading is a difference of two
         * errors, so the same optical offset cancels out there. */
        const int bias = CAMERA_LINE_CENTER_BIAS_PX * 100 / ((int)width / 2);
        const int signed_bias = CAMERA_LINE_MIRROR_X ? -bias : bias;
        observation->lateral_error = clamp_int(observation->lateral_error +
                                               signed_bias, 100);
    }
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

/* Obstacle motion uses the vectors already validated by car-spin.  Keep these
 * commands outside the line mixer so the lateral path is not altered by the
 * camera controller's forward ramp or steering limits. */
static void obstacle_drive_direct(int a, int b, int d, bool suppress_kick)
{
    s_last_forward_target = 0;
    s_last_forward_ramped = 0;
    s_last_drive_forward = 0;
    s_last_drive_turn = 0;
    s_last_drive_lat = 0;
    s_mix_pre_a = a;
    s_mix_pre_b = b;
    s_mix_pre_d = d;
    s_mix_post_a = a;
    s_mix_post_b = b;
    s_mix_post_d = d;
    s_mix_scaled = false;
    s_mix_dropped = false;
    s_mix_repaired = false;
    s_command_a = a;
    s_command_b = b;
    s_command_d = d;

    const bool previous_suppress_kick = s_suppress_kick;
    s_suppress_kick = suppress_kick;
    motor_set(&motor_a, a);
    motor_set(&motor_b, b);
    motor_set(&motor_d, d);
    s_suppress_kick = previous_suppress_kick;
}

static void obstacle_drive_lateral(bool left)
{
    if (left) {
        obstacle_drive_direct(-AVOID_LEFT_SIDE_SPEED, -AVOID_LEFT_B_SPEED,
                               -AVOID_LEFT_SIDE_SPEED, false);
    } else {
        obstacle_drive_direct(AVOID_RIGHT_A_SPEED, AVOID_RIGHT_B_SPEED,
                               AVOID_RIGHT_D_SPEED, false);
    }
}

static void obstacle_drive_forward(void)
{
    obstacle_drive_direct(AVOID_FORWARD_A_SPEED, 0, AVOID_FORWARD_D_SPEED,
                          false);
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

/*
 * The generic mixer deliberately drops sub-floor wheel components so its
 * standalone contract remains exact.  In NORMAL, however, a valid forward
 * command must not collapse to a single wheel just because a steering pulse
 * was scaled.  Restore only components that were requested and then dropped;
 * the repair is bounded by the measured stiction floors and the PWM ceiling.
 */
static bool repair_normal_drive(const line_mixer_cfg_t *cfg,
                                int forward,
                                int raw_a,
                                int raw_b,
                                int raw_d,
                                line_mixer_out_t *out)
{
    if (cfg == NULL || out == NULL || forward < LINE_MOTOR_START_MIN_OUTPUT ||
        !out->dropped) {
        return false;
    }

    bool repaired = false;
    if (out->a == 0) {
        int sign = direction_of(raw_a);
        if (sign == 0) {
            sign = -direction_of(forward);
        }
        if (sign != 0) {
            out->a = sign * cfg->floor_ad;
            repaired = true;
        }
    }
    if (out->d == 0) {
        int sign = direction_of(raw_d);
        if (sign == 0) {
            sign = direction_of(forward);
        }
        if (sign != 0) {
            out->d = sign * cfg->floor_ad;
            repaired = true;
        }
    }
    /* A nonzero yaw request should keep the rear wheel over its own floor. */
    if (out->b == 0 && raw_b != 0) {
        out->b = direction_of(raw_b) * cfg->floor_b;
        repaired = true;
    }
    return repaired;
}

static void drive_internal(int forward, int turn, int lat,
                           bool normal_traction_guard)
{
    const line_mixer_cfg_t cfg = {
        MOTOR_PWM_CEILING, MOTOR_MIN_RUN_OUTPUT, MOTOR_B_MIN_RUN_OUTPUT,
        MOTOR_TRIM_A, MOTOR_TRIM_D,
    };
    line_mixer_out_t out = {0, 0, 0, false, false};
    s_last_drive_forward = forward;
    s_last_drive_turn = turn;
    s_last_drive_lat = lat;
    forward = clamp_int(forward, forward_ramp_cap(esp_timer_get_time()));
    s_last_forward_ramped = forward;
    const int forward_a = forward * MOTOR_TRIM_A / 100;
    const int forward_d = forward * MOTOR_TRIM_D / 100;
    s_mix_pre_a = -forward_a - turn + lat;
    s_mix_pre_b = turn + 2 * lat;
    s_mix_pre_d = forward_d - turn + lat;
    line_mixer_solve(forward, turn, lat, &cfg, &out);
    s_mix_repaired = normal_traction_guard && s_armed &&
                     repair_normal_drive(&cfg, forward, s_mix_pre_a,
                                         s_mix_pre_b, s_mix_pre_d, &out);
    s_command_a = out.a;
    s_command_b = out.b;
    s_command_d = out.d;
    s_mix_post_a = out.a;
    s_mix_post_b = out.b;
    s_mix_post_d = out.d;
    s_mix_scaled = out.scaled;
    s_mix_dropped = out.dropped;
    motor_set(&motor_a, s_command_a);
    motor_set(&motor_b, s_command_b);
    motor_set(&motor_d, s_command_d);
}

static void drive(int forward, int turn, int lat)
{
    drive_internal(forward, turn, lat, false);
}

static void drive_normal_vector(int forward, int turn, int lat)
{
    drive_internal(forward, turn, lat, true);
}

/* Use the low-speed IR spin calibration in TURN.  The direct wheel commands
 * avoid the normal direction-change kick, which otherwise makes the camera
 * sweep past the line before the next frame is processed. */
static void drive_slow_spin(int turn)
{
    const int direction = direction_of(turn);
    if (direction == 0) {
        zero_motor_outputs();
        return;
    }

    const int a = -direction * LINE_TURN_A_SPEED;
    const int b = direction * LINE_TURN_B_SPEED;
    const int d = -direction * LINE_TURN_D_SPEED;
    s_last_forward_target = 0;
    s_last_forward_ramped = 0;
    s_last_drive_forward = 0;
    s_last_drive_turn = turn;
    s_last_drive_lat = 0;
    s_mix_pre_a = a;
    s_mix_pre_b = b;
    s_mix_pre_d = d;
    s_mix_post_a = a;
    s_mix_post_b = b;
    s_mix_post_d = d;
    s_mix_scaled = false;
    s_mix_dropped = false;
    s_mix_repaired = false;
    s_command_a = a;
    s_command_b = b;
    s_command_d = d;

    const bool saved_suppress_kick = s_suppress_kick;
    s_suppress_kick = true;
    motor_set(&motor_a, a);
    motor_set(&motor_b, b);
    motor_set(&motor_d, d);
    s_kick_cycles[0] = 0;
    s_kick_cycles[1] = 0;
    s_kick_cycles[2] = 0;
    s_suppress_kick = saved_suppress_kick;
}

static uint32_t servo_pulse_to_duty(uint32_t pulse_us)
{
    return (SERVO_MAX_DUTY * pulse_us) / SERVO_PERIOD_US;
}

static void servo_set_pulse(ledc_channel_t channel, uint32_t pulse_us)
{
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, channel,
                        servo_pulse_to_duty(pulse_us));
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static esp_err_t ball_servo_init(void)
{
    if (s_servo_initialized) {
        return ESP_OK;
    }
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = SERVO_TIMER,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }

    const ledc_channel_config_t channels[] = {
        {
            .gpio_num = PAN_SERVO_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = SERVO_PAN_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_TIMER,
            .duty = servo_pulse_to_duty(BALL_PAN_CENTER_US),
            .hpoint = 0,
        },
        {
            .gpio_num = TILT_SERVO_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = SERVO_TILT_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_TIMER,
            .duty = servo_pulse_to_duty(BALL_TILT_HIGH_US),
            .hpoint = 0,
        },
    };
    for (size_t i = 0; i < sizeof(channels) / sizeof(channels[0]); ++i) {
        err = ledc_channel_config(&channels[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    servo_set_pulse(SERVO_PAN_CHANNEL, BALL_PAN_CENTER_US);
    servo_set_pulse(SERVO_TILT_CHANNEL, BALL_TILT_HIGH_US);
    s_servo_initialized = true;
    ESP_LOGI(TAG, "ball servos ready: pan=center tilt=high");
    return ESP_OK;
}

static void ball_enable_stby(void)
{
    if (!s_stby_enabled) {
        gpio_set_level(STBY_GPIO, 1);
        s_stby_enabled = true;
    }
}

/* Ball motion deliberately bypasses the line controller's forward ramp.  It
 * still uses the validated wheel signs and the same stiction floors. */
static void ball_drive_direct(int a, int b, int d)
{
    ball_enable_stby();
    obstacle_drive_direct(a, b, d, true);
}

static void ball_drive_spin(int direction, int speed)
{
    const int magnitude = speed < MOTOR_MIN_RUN_OUTPUT ? MOTOR_MIN_RUN_OUTPUT : speed;
    const int rear = magnitude < MOTOR_B_MIN_RUN_OUTPUT ? MOTOR_B_MIN_RUN_OUTPUT : magnitude;
    ball_drive_direct(-direction * magnitude, direction * rear,
                      -direction * magnitude);
}

static void ball_drive_charge(bool reverse)
{
    const int sign = reverse ? -1 : 1;
    /* This chassis moves forward with a negative A command and a positive D
     * command. Reverse both signs for the return leg. */
    ball_drive_direct(-sign * BALL_CHARGE_A_SPEED, 0,
                      sign * BALL_CHARGE_D_SPEED);
}

static void ball_drive_stop(void)
{
    stop_motors();
}

static const char *ball_phase_name(void)
{
    switch (s_ball_phase) {
    case BALL_SEARCH_RED: return "SEARCH_R";
    case BALL_ALIGN_RED: return "ALIGN_R";
    case BALL_FORWARD_RED: return "FWD_R";
    case BALL_BACK_RED: return "BACK_R";
    case BALL_SEARCH_GREEN: return "SEARCH_G";
    case BALL_ALIGN_GREEN: return "ALIGN_G";
    case BALL_FORWARD_GREEN: return "FWD_G";
    case BALL_BACK_GREEN: return "BACK_G";
    case BALL_DONE: return "DONE";
    case BALL_IDLE:
    default: return "IDLE";
    }
}

static uint8_t ball_luma(const uint8_t *pixel, int *red, int *green, int *blue)
{
    const uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    const int r = ((value >> 11) & 0x1f) * 255 / 31;
    const int g = ((value >> 5) & 0x3f) * 255 / 63;
    const int b = (value & 0x1f) * 255 / 31;
    if (red != NULL) *red = r;
    if (green != NULL) *green = g;
    if (blue != NULL) *blue = b;
    return (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
}

static bool ball_pixel_matches(const uint8_t *frame, uint16_t width,
                               uint16_t height, int x, int y,
                               ball_colour_t colour)
{
    if (frame == NULL || x < 0 || y < 0 || x >= (int)width || y >= (int)height) {
        return false;
    }
    const uint8_t *pixel = frame + (((size_t)y * width + (size_t)x) * 2U);
    int red = 0;
    int green = 0;
    int blue = 0;
    const int luma = ball_luma(pixel, &red, &green, &blue);
    if (colour == BALL_COLOUR_RED) {
        return luma >= BALL_MIN_LUMA && red >= BALL_RED_MIN_R &&
               red > green + BALL_RED_CHANNEL_GAP &&
               red > blue + BALL_RED_CHANNEL_GAP;
    }
    return luma >= BALL_MIN_LUMA && green >= BALL_GREEN_MIN_G &&
           green > red + BALL_GREEN_CHANNEL_GAP &&
           green > blue + BALL_GREEN_CHANNEL_GAP;
}

static uint8_t s_ball_visited[BALL_GRID_MAX_CELLS];
static uint16_t s_ball_queue[BALL_GRID_MAX_CELLS];

static bool find_ball_blob(const uint8_t *frame, uint16_t width, uint16_t height,
                           ball_colour_t colour, ball_blob_t *blob)
{
    if (frame == NULL || blob == NULL || width < 16 || height < 16) {
        return false;
    }
    *blob = (ball_blob_t){0};
    const int grid_w = ((int)width + BALL_GRID_STEP - 1) / BALL_GRID_STEP;
    const int grid_h = ((int)height + BALL_GRID_STEP - 1) / BALL_GRID_STEP;
    const size_t cells = (size_t)grid_w * (size_t)grid_h;
    if (cells > BALL_GRID_MAX_CELLS) {
        return false;
    }
    memset(s_ball_visited, 0, cells);

    const int top = (int)height * BALL_SCAN_TOP_PERCENT / 100;
    const int bottom = (int)height * BALL_SCAN_BOTTOM_PERCENT / 100;
    int best_score = 0;
    ball_blob_t best = {0};

    /* Flood-fill the colour mask at half resolution.  A valid ball must be a
     * compact, reasonably round component; isolated highlights and large
     * coloured areas therefore cannot win merely by having one bright pixel. */
    for (int gy = 0; gy < grid_h; ++gy) {
        const int seed_y = gy * BALL_GRID_STEP;
        if (seed_y < top || seed_y >= bottom) continue;
        for (int gx = 0; gx < grid_w; ++gx) {
            const size_t seed = (size_t)gy * grid_w + (size_t)gx;
            const int seed_x = gx * BALL_GRID_STEP;
            if (s_ball_visited[seed] ||
                !ball_pixel_matches(frame, width, height, seed_x, seed_y, colour)) {
                continue;
            }
            s_ball_visited[seed] = 1;
            size_t head = 0;
            size_t tail = 0;
            s_ball_queue[tail++] = (uint16_t)seed;
            int count = 0;
            int sum_x = 0;
            int sum_y = 0;
            int left = width;
            int right = 0;
            int top_y = height;
            int bottom_y = 0;
            while (head < tail) {
                const uint16_t cell = s_ball_queue[head++];
                const int cell_x = (cell % grid_w) * BALL_GRID_STEP;
                const int cell_y = (cell / grid_w) * BALL_GRID_STEP;
                ++count;
                sum_x += cell_x;
                sum_y += cell_y;
                if (cell_x < left) left = cell_x;
                if (cell_x > right) right = cell_x;
                if (cell_y < top_y) top_y = cell_y;
                if (cell_y > bottom_y) bottom_y = cell_y;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int nx = (cell % grid_w) + dx;
                        const int ny = (cell / grid_w) + dy;
                        if (nx < 0 || ny < 0 || nx >= grid_w || ny >= grid_h) continue;
                        const size_t next = (size_t)ny * grid_w + (size_t)nx;
                        if (s_ball_visited[next]) continue;
                        const int pixel_x = nx * BALL_GRID_STEP;
                        const int pixel_y = ny * BALL_GRID_STEP;
                        if (pixel_y < top || pixel_y >= bottom ||
                            !ball_pixel_matches(frame, width, height, pixel_x, pixel_y,
                                                 colour)) {
                            continue;
                        }
                        s_ball_visited[next] = 1;
                        if (tail < BALL_GRID_MAX_CELLS) {
                            s_ball_queue[tail++] = (uint16_t)next;
                        }
                    }
                }
            }

            const int box_w = right - left + BALL_GRID_STEP;
            const int box_h = bottom_y - top_y + BALL_GRID_STEP;
            /* count is the number of colour samples on the half-resolution
             * grid, so compare it with the bounding box in grid cells. Using
             * full-resolution pixel area here underestimates fill by 4x and
             * rejects a valid 5 cm ball at the control resolution. */
            const int sample_box_w = (right - left) / BALL_GRID_STEP + 1;
            const int sample_box_h = (bottom_y - top_y) / BALL_GRID_STEP + 1;
            const int sample_box_area = sample_box_w * sample_box_h;
            const int fill_percent = sample_box_area > 0 ?
                                     count * 100 / sample_box_area : 0;
            /* Components clipped by the image boundary are commonly caused
             * by red/green UI or cable artifacts. A real ball must have a
             * small colour-free border around its blob before it can steer
             * the vehicle. */
            if (count < BALL_MIN_PIXELS || box_w < 4 || box_h < 4 ||
                left <= BALL_MIN_BORDER_MARGIN ||
                top_y <= BALL_MIN_BORDER_MARGIN ||
                right >= (int)width - 1 - BALL_MIN_BORDER_MARGIN ||
                bottom_y >= (int)height - 1 - BALL_MIN_BORDER_MARGIN ||
                fill_percent < BALL_MIN_FILL_PERCENT ||
                box_w > (int)width * BALL_MAX_WIDTH_PERCENT / 100 ||
                box_h > (int)height * BALL_MAX_WIDTH_PERCENT / 100 ||
                box_w > (int)width * 3 / 4 || box_h > (int)height * 3 / 4 ||
                box_w > box_h * 22 / 10 || box_h > box_w * 22 / 10) {
                continue;
            }

            int edge_score = 0;
            for (int y = top_y - 3; y <= bottom_y + 3; y += BALL_GRID_STEP) {
                for (int x = left - 3; x <= right + 3; x += BALL_GRID_STEP) {
                    if (x < 0 || y < 0 || x >= (int)width || y >= (int)height) continue;
                    if ((x >= left && x <= right && y >= top_y && y <= bottom_y) ||
                        ball_pixel_matches(frame, width, height, x, y, colour)) {
                        continue;
                    }
                    ++edge_score;
                }
            }
            if (edge_score < BALL_MIN_EDGE_PIXELS) continue;

            const int score = count * fill_percent + edge_score * 6;
            if (score > best_score) {
                best_score = score;
                best.valid = true;
                best.x = sum_x / count;
                best.y = sum_y / count;
                best.width = box_w;
                best.height = box_h;
                best.pixels = count * BALL_GRID_STEP * BALL_GRID_STEP;
                best.edge_score = edge_score;
            }
        }
    }
    if (best_score <= 0) return false;
    *blob = best;
    return true;
}

static void ball_stream_detect(const char *colour, const ball_blob_t *blob,
                               uint16_t frame_width, uint16_t frame_height)
{
    if (colour == NULL || blob == NULL || !blob->valid) return;
    char line[96];
    const int len = snprintf(line, sizeof(line), "DETECT %s %d %d %d %d %s %u %u\n",
                             colour, blob->x, blob->y, blob->width, blob->height,
                             ball_phase_name(), (unsigned)frame_width,
                             (unsigned)frame_height);
    if (len > 0 && (size_t)len < sizeof(line)) {
        (void)tcp_server_send((uint8_t *)line, (size_t)len);
    }
}

static void ball_stream_clear(void)
{
    static const uint8_t clear[] = "DETECT CLEAR\n";
    (void)tcp_server_send((uint8_t *)clear, sizeof(clear) - 1U);
}

static bool __attribute__((unused)) find_black_zone(const uint8_t *frame, uint16_t width, uint16_t height,
                            ball_zone_t *zone)
{
    if (frame == NULL || zone == NULL || width < 24 || height < 24) {
        return false;
    }
    *zone = (ball_zone_t){0};
    const int window_w = width / 6 < BALL_ZONE_MIN_RUN_PIXELS ?
                         BALL_ZONE_MIN_RUN_PIXELS : width / 6;
    const int window_h = height / 8 < BALL_ZONE_MIN_ROWS ?
                         BALL_ZONE_MIN_ROWS : height / 8;
    int best_score = 0;
    int best_x = -1;
    int best_y = -1;
    for (int y = (int)height / 10; y + window_h < (int)height * 9 / 10; y += 3) {
        for (int x = 0; x + window_w < (int)width; x += 3) {
            int dark = 0;
            for (int row = 0; row < window_h; row += 2) {
                int run = 0;
                for (int column = 0; column < window_w; column += 2) {
                    const uint8_t *pixel = frame +
                        (((size_t)(y + row) * width + (size_t)(x + column)) * 2U);
                    if (ball_luma(pixel, NULL, NULL, NULL) <= BALL_ZONE_THRESHOLD) {
                        ++dark;
                        ++run;
                    } else {
                        run = 0;
                    }
                }
                if (run >= BALL_ZONE_MIN_RUN_PIXELS / 2) {
                    dark += run;
                }
            }
            if (dark > best_score) {
                best_score = dark;
                best_x = x;
                best_y = y;
            }
        }
    }
    const int required = (window_w / 2) * (window_h / 3);
    if (best_x < 0 || best_score < required) {
        return false;
    }
    zone->valid = true;
    zone->x = best_x + window_w / 2;
    zone->y = best_y + window_h / 2;
    zone->width = window_w;
    zone->rows = window_h;
    zone->score = best_score;
    return true;
}

static bool ball_is_search_phase(void)
{
    return s_ball_phase == BALL_SEARCH_RED || s_ball_phase == BALL_SEARCH_GREEN;
}

static ball_colour_t ball_target_colour(void)
{
    return (s_ball_phase == BALL_SEARCH_RED || s_ball_phase == BALL_ALIGN_RED ||
            s_ball_phase == BALL_FORWARD_RED || s_ball_phase == BALL_BACK_RED) ?
           BALL_COLOUR_RED : BALL_COLOUR_GREEN;
}

static const char *ball_colour_name(ball_colour_t colour)
{
    return colour == BALL_COLOUR_RED ? "red" : "green";
}

static void ball_begin(int64_t now)
{
    if (ball_servo_init() != ESP_OK) {
        ESP_LOGE(TAG, "ball task disabled: servo init failed");
        s_ball_phase = BALL_DONE;
        return;
    }
    servo_set_pulse(SERVO_PAN_CHANNEL, BALL_PAN_CENTER_US);
    servo_set_pulse(SERVO_TILT_CHANNEL, BALL_TILT_HIGH_US);
    s_ball_phase = BALL_SEARCH_RED;
    s_ball_phase_start_us = now;
    s_ball_align_frames = 0;
    s_ball_target_valid = false;
    s_ball_search_direction = 1;
    s_ball_target_miss_frames = 0;
    ball_drive_stop();
    ball_stream_clear();
    ESP_LOGW(TAG, "finish task: tilt high, searching RED then GREEN");
}

static void ball_next_search(int64_t now)
{
    ball_stream_clear();
    s_ball_phase = BALL_SEARCH_GREEN;
    s_ball_phase_start_us = now;
    s_ball_align_frames = 0;
    s_ball_target_valid = false;
    s_ball_search_direction = 1;
    s_ball_target_miss_frames = 0;
    ball_drive_stop();
    ESP_LOGI(TAG, "red centered; searching GREEN");
}

static void ball_begin_charge(int64_t now, ball_colour_t colour)
{
    s_ball_phase = colour == BALL_COLOUR_RED ? BALL_FORWARD_RED : BALL_FORWARD_GREEN;
    s_ball_phase_start_us = now;
    s_ball_align_frames = 0;
    s_ball_target_miss_frames = 0;
    ball_drive_charge(false);
    ESP_LOGI(TAG, "%s centered; charging forward for %ums",
             ball_colour_name(colour), (unsigned)BALL_CHARGE_MS);
}

static void ball_finish_charge(int64_t now, ball_colour_t colour)
{
    s_ball_phase = colour == BALL_COLOUR_RED ? BALL_BACK_RED : BALL_BACK_GREEN;
    s_ball_phase_start_us = now;
    ball_drive_charge(true);
    ESP_LOGI(TAG, "%s forward complete; reversing for %ums",
             ball_colour_name(colour), (unsigned)BALL_CHARGE_MS);
}

static void ball_process_frame(uint8_t *frame, uint16_t width, uint16_t height,
                               int64_t now)
{
    if (frame == NULL || !s_finished || s_ball_phase == BALL_IDLE ||
        s_ball_phase == BALL_DONE) {
        return;
    }
    const ball_colour_t colour = ball_target_colour();
    const bool red_target = colour == BALL_COLOUR_RED;

    if (s_ball_phase == BALL_FORWARD_RED || s_ball_phase == BALL_FORWARD_GREEN) {
        ball_drive_charge(false);
        if (now - s_ball_phase_start_us >= (int64_t)BALL_CHARGE_MS * 1000) {
            ball_finish_charge(now, colour);
        }
        return;
    }
    if (s_ball_phase == BALL_BACK_RED || s_ball_phase == BALL_BACK_GREEN) {
        ball_drive_charge(true);
        if (now - s_ball_phase_start_us >= (int64_t)BALL_CHARGE_MS * 1000) {
            ball_drive_stop();
            if (red_target) {
                ball_next_search(now);
            } else {
                s_ball_phase = BALL_DONE;
                ESP_LOGW(TAG, "red and green complete; vehicle stopped");
            }
        }
        return;
    }

    ball_blob_t blob = {0};
    const bool found_ball = find_ball_blob(frame, width, height, colour, &blob);

    if (ball_is_search_phase()) {
        if (found_ball) {
            s_ball_x = blob.x;
            s_ball_y = blob.y;
            s_ball_target_valid = true;
            s_ball_phase = red_target ? BALL_ALIGN_RED : BALL_ALIGN_GREEN;
            s_ball_phase_start_us = now;
            s_ball_align_frames = 0;
            s_ball_target_miss_frames = 0;
            ball_drive_stop();
            ESP_LOGI(TAG, "%s ball found at (%d,%d) pixels=%d; aligning",
                     ball_colour_name(colour), blob.x, blob.y, blob.pixels);
            ball_stream_detect(red_target ? "RED" : "GREEN", &blob,
                               width, height);
            return;
        }
        const int64_t elapsed = now - s_ball_phase_start_us;
        if (elapsed < (int64_t)BALL_SERVO_SETTLE_MS * 1000) {
            ball_drive_stop();
            return;
        }
        if (elapsed > (int64_t)BALL_SEARCH_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "%s ball search timeout; stopping safely",
                     ball_colour_name(colour));
            s_ball_phase = BALL_DONE;
            ball_drive_stop();
        } else {
            if (elapsed > (int64_t)BALL_SEARCH_TIMEOUT_MS * 500 &&
                s_ball_search_direction > 0) {
                s_ball_search_direction = -1;
            }
            ball_drive_spin(s_ball_search_direction, BALL_SEARCH_TURN_SPEED);
        }
        return;
    }

    if (s_ball_phase == BALL_ALIGN_RED || s_ball_phase == BALL_ALIGN_GREEN) {
        if (found_ball) {
            s_ball_x = blob.x;
            s_ball_y = blob.y;
            s_ball_target_valid = true;
            s_ball_target_miss_frames = 0;
            ball_stream_detect(red_target ? "RED" : "GREEN", &blob,
                               width, height);
        } else if (s_ball_target_miss_frames < 3U) {
            ++s_ball_target_miss_frames;
        } else {
            s_ball_phase = red_target ? BALL_SEARCH_RED : BALL_SEARCH_GREEN;
            s_ball_phase_start_us = now;
            s_ball_target_valid = false;
            ESP_LOGW(TAG, "%s ball lost during alignment; resuming search",
                     ball_colour_name(colour));
            return;
        }
        if (!s_ball_target_valid) {
            ball_drive_spin(s_ball_search_direction, BALL_ALIGN_TURN_SPEED);
        } else {
            /* The camera optical axis is the target: keep the ball itself
             * centred instead of using unrelated scene features. */
            const int error = s_ball_x - (int)width / 2;
            if (abs(error) <= BALL_ALIGN_TOLERANCE_PX) {
                if (s_ball_align_frames < BALL_ALIGN_CONFIRM_FRAMES) {
                    ++s_ball_align_frames;
                }
                ball_drive_stop();
            } else {
                s_ball_align_frames = 0;
                const int direction = error > 0 ? -1 : 1;
                ball_drive_spin(direction, BALL_ALIGN_TURN_SPEED);
            }
            if (s_ball_align_frames >= BALL_ALIGN_CONFIRM_FRAMES) {
                ball_drive_stop();
                s_ball_align_frames = 0;
                ESP_LOGI(TAG, "%s centered ball=(%d,%d); preparing charge",
                         ball_colour_name(colour), s_ball_x, s_ball_y);
                ball_begin_charge(now, colour);
                return;
            }
        }
        if (now - s_ball_phase_start_us > (int64_t)BALL_ALIGN_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "%s alignment timeout; returning to search",
                     ball_colour_name(colour));
            s_ball_phase = red_target ? BALL_SEARCH_RED : BALL_SEARCH_GREEN;
            s_ball_phase_start_us = now;
            s_ball_target_valid = false;
        }
        return;
    }
}

static void reset_control(bool reset_forward)
{
    s_lat_accum = 0;
    s_yaw_accum = 0;
    /* State transitions must not make a moving car restart its forward ramp.
     * Only arm/disarm paths request a fresh starting value. */
    if (reset_forward) {
        s_forward_output = LINE_MOTOR_START_MIN_OUTPUT;
    }
    s_turn_output = 0;
    s_error_filter_initialized = false;
    s_lateral_control_error = 0;
    s_heading_control_error = 0;
    s_lateral_error_rate = 0;
    s_heading_error_rate = 0;
}

static void reset_tracking(void)
{
    reset_control(true);
    s_seed_valid = false;
    s_seed_x = 0;
    s_line_width = 0;
    s_lost_frames = 0;
    s_reacquire_frames = 0;
    s_reacquire_x = 0;
    s_turn_direction = 0;
    s_turn_hint_frames = 0;
    s_turn_pending_until_us = 0;
    s_turn_exit_frames = 0;
    s_turn_frames = 0;
    s_turn_started_us = 0;
    s_alert_until_us = 0;
    s_finish_frames = 0;
    s_finish_align_start_us = 0;
    s_have_last_good_observation = false;
    s_candidate_miss_frames = 0;
    s_overlay_miss_frames = 0;
    s_last_candidate_held = false;
    s_state = LINE_STATE_NORMAL;
}

static const char *obstacle_state_name(void)
{
    switch (s_obstacle_state) {
    case OBSTACLE_BRAKE:
        return "BRAKE";
    case OBSTACLE_LEFT:
        return "LEFT";
    case OBSTACLE_FORWARD:
        return "FORWARD";
    case OBSTACLE_RIGHT:
        return "RIGHT";
    case OBSTACLE_IDLE:
    default:
        return "IDLE";
    }
}

static uint32_t obstacle_elapsed_ms(int64_t now)
{
    if (s_obstacle_phase_start_us == 0 || now <= s_obstacle_phase_start_us) {
        return 0;
    }
    const int64_t elapsed_us = now - s_obstacle_phase_start_us;
    if (elapsed_us >= (int64_t)UINT32_MAX * 1000) {
        return UINT32_MAX;
    }
    return (uint32_t)(elapsed_us / 1000);
}

static void obstacle_update_sensor(int64_t now)
{
    const uint32_t sequence = s_ultrasonic_sequence;
    if (sequence == s_obstacle_last_sequence) {
        return;
    }
    s_obstacle_last_sequence = sequence;
    const float distance = s_ultrasonic_distance_cm;
    const bool valid = s_ultrasonic_valid && distance >= ULTRASONIC_MIN_CM &&
                       distance <= ULTRASONIC_MAX_CM;
    if (valid && distance <= OBSTACLE_DETECT_CM) {
        if (s_obstacle_close_samples < UINT32_MAX) {
            ++s_obstacle_close_samples;
        }
    } else {
        s_obstacle_close_samples = 0;
    }
    if (s_obstacle_state == OBSTACLE_IDLE && valid &&
        distance > OBSTACLE_CLEAR_CM && now >= s_obstacle_cooldown_until_us) {
        s_obstacle_ready = true;
    }
}

static void obstacle_begin(int64_t now, float distance)
{
    s_obstacle_state = OBSTACLE_BRAKE;
    s_obstacle_phase_start_us = now;
    s_obstacle_ready = false;
    s_obstacle_reacquire_until_us = 0;
    s_obstacle_cooldown_until_us = 0;
    s_obstacle_close_samples = 0;
    reset_control(false);
    zero_motor_outputs();
    ESP_LOGW(TAG, "obstacle %.1fcm confirmed; brake then camera-independent route",
             (double)distance);
}

static void obstacle_finish(int64_t now, uint16_t width)
{
    s_obstacle_state = OBSTACLE_IDLE;
    s_obstacle_phase_start_us = 0;
    s_obstacle_close_samples = 0;
    s_obstacle_ready = false;
    s_obstacle_cooldown_until_us = now + (int64_t)AVOID_COOLDOWN_MS * 1000;
    s_obstacle_reacquire_until_us =
        now + (int64_t)AVOID_REACQUIRE_GRACE_MS * 1000;

    /* The camera has moved sideways, so discard the old seed and let the next
     * decoded frame perform a fresh bottom-seed search.  Keep the arm/STBY
     * latch; this is a route manoeuvre, not a line-following re-arm. */
    reset_tracking();
    s_last_line_us = now;
    s_reacquire_x = (int)width / 2;
    zero_motor_outputs();
    ESP_LOGI(TAG, "obstacle fixed route complete (right %ums); visual reacquire enabled",
             (unsigned)AVOID_RIGHT_MS);
}

/* Advance one non-blocking avoidance phase.  Return true while the visual
 * controller must be suppressed for this frame. */
static bool obstacle_step(int64_t now, uint16_t width, bool line_available)
{
    obstacle_update_sensor(now);

    if (s_obstacle_state == OBSTACLE_IDLE) {
        if (s_armed && !s_finished && line_available &&
            now >= s_obstacle_cooldown_until_us &&
            s_obstacle_ready &&
            s_obstacle_close_samples >= OBSTACLE_CLOSE_CONFIRM_SAMPLES) {
            obstacle_begin(now, s_ultrasonic_distance_cm);
            return true;
        }
        return false;
    }

    const uint32_t elapsed = obstacle_elapsed_ms(now);
    switch (s_obstacle_state) {
    case OBSTACLE_BRAKE:
        zero_motor_outputs();
        if (elapsed >= AVOID_BRAKE_MS) {
            s_obstacle_state = OBSTACLE_LEFT;
            s_obstacle_phase_start_us = now;
            ESP_LOGI(TAG, "obstacle brake done; left shift start");
        }
        break;

    case OBSTACLE_LEFT: {
        obstacle_drive_lateral(true);
        /* The side sensor is not a reliable stop signal while the chassis is
         * moving alongside an obstacle.  Use the validated fixed route time. */
        if (elapsed >= AVOID_LEFT_MS) {
            s_obstacle_state = OBSTACLE_FORWARD;
            s_obstacle_phase_start_us = now;
            ESP_LOGI(TAG, "left shift complete after fixed %ums; forward start",
                     (unsigned)elapsed);
        }
        break;
    }

    case OBSTACLE_FORWARD:
        obstacle_drive_forward();
        if (elapsed >= AVOID_FORWARD_MS) {
            s_obstacle_state = OBSTACLE_RIGHT;
            s_obstacle_phase_start_us = now;
            ESP_LOGI(TAG, "obstacle forward done; fixed right shift %ums",
                     (unsigned)AVOID_RIGHT_MS);
        }
        break;

    case OBSTACLE_RIGHT:
        obstacle_drive_lateral(false);
        if (elapsed >= AVOID_RIGHT_MS) {
            obstacle_finish(now, width);
        }
        break;

    case OBSTACLE_IDLE:
    default:
        break;
    }
    return true;
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
    s_last_forward_target = target;
    const int delta = target - s_forward_output;
    if (delta > LINE_FORWARD_SLEW) {
        s_forward_output += LINE_FORWARD_SLEW;
    } else if (delta < -LINE_FORWARD_SLEW) {
        s_forward_output -= LINE_FORWARD_SLEW;
    } else {
        s_forward_output = target;
    }
    const int previous_lateral = s_lateral_control_error;
    const int previous_heading = s_heading_control_error;
    const int lateral_error = filter_control_error(observation->lateral_error,
                                                   &s_lateral_control_error);
    const int heading_error = filter_control_error(observation->heading_error,
                                                   &s_heading_control_error);
    const int lateral_delta = s_error_filter_initialized ?
                              lateral_error - previous_lateral : 0;
    const int heading_delta = s_error_filter_initialized ?
                              heading_error - previous_heading : 0;
    s_lateral_error_rate = (s_lateral_error_rate * 2 + lateral_delta) / 3;
    s_heading_error_rate = (s_heading_error_rate * 2 + heading_delta) / 3;
    s_last_lateral_raw = observation->lateral_error;
    s_last_lateral_filtered = lateral_error;
    s_last_lateral_delta = s_lateral_error_rate;
    s_last_heading_raw = observation->heading_error;
    s_last_heading_filtered = heading_error;
    s_last_heading_delta = s_heading_error_rate;
    s_error_filter_initialized = true;
    /* 正常巡线只用车体转向校正，不用横移硬拉回数学中心线。
     * lateral 正值表示线在左侧，因此折算成负 heading，yaw PD 输出左转。
     * 死区内保留有宽度的中心区域，避免小车左右来回校正。 */
    const int lateral_for_steer = abs(lateral_error) <= LINE_ERROR_DEADBAND ?
                                   0 : lateral_error;
    const int steer_error = heading_error -
                            (LINE_PID_KP_LAT * lateral_for_steer) /
                            (LINE_PID_KH > 0 ? LINE_PID_KH : 1);
    const int steer_rate = s_heading_error_rate -
                           (LINE_PID_KP_LAT * s_lateral_error_rate) /
                           (LINE_PID_KH > 0 ? LINE_PID_KH : 1);
    s_turn_output = line_control_yaw_pd(&cfg, steer_error, steer_rate,
                                        LINE_PD_KD_HEADING, &s_yaw_accum);
    s_lat_accum = 0;
    drive_normal_vector(s_forward_output, s_turn_output, 0);
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
    case LINE_STATE_CORNER:
        return "CORNER";
    case LINE_STATE_LOST:
        return "LOST";
    case LINE_STATE_NORMAL:
    default:
        return "NORMAL";
    }
}

void camera_line_follow_get_debug_snapshot(camera_line_follow_debug_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    bool locked = false;
    if (s_control_mutex != NULL) {
        locked = xSemaphoreTake(s_control_mutex, pdMS_TO_TICKS(5)) == pdTRUE;
    }
    snapshot->state = state_name();
    snapshot->armed = s_armed;
    snapshot->stby = s_stby_enabled;
    snapshot->candidate = s_last_candidate;
    snapshot->ball_phase = ball_phase_name();
    snapshot->motor_a = s_command_a;
    snapshot->motor_b = s_command_b;
    snapshot->motor_d = s_command_d;
    snapshot->ultrasonic_distance_x10 = s_ultrasonic_valid ?
                                        (int)(s_ultrasonic_distance_cm * 10.0f + 0.5f) : -1;
    snapshot->threshold = s_last_threshold;
    snapshot->seed_x = s_last_seed_x;
    snapshot->valid_rows = s_last_valid_rows;
    snapshot->confidence = s_last_confidence;
    if (locked) {
        (void)xSemaphoreGive(s_control_mutex);
    }
}

void camera_line_follow_tft_status_callback(void *user_ctx)
{
    (void)user_ctx;
#if CONFIG_EXAMPLE_ENABLE_TFT_PREVIEW
    camera_line_follow_debug_snapshot_t snapshot = {0};
    camera_line_follow_get_debug_snapshot(&snapshot);

    char line[32];
    (void)snprintf(line, sizeof(line), "STATE %s", snapshot.state);
    (void)tft_st7735_draw_text(0, 0, line, 0xffff, 0x0000);
    (void)snprintf(line, sizeof(line), "ARM %d STBY %d",
                   snapshot.armed ? 1 : 0, snapshot.stby ? 1 : 0);
    (void)tft_st7735_draw_text(0, 8, line, 0xffff, 0x0000);
    (void)snprintf(line, sizeof(line), "M A%d B%d D%d",
                   snapshot.motor_a, snapshot.motor_b, snapshot.motor_d);
    (void)tft_st7735_draw_text(0, 16, line, 0xffff, 0x0000);

    const int distance = snapshot.ultrasonic_distance_x10;
    if (distance >= 0) {
        (void)snprintf(line, sizeof(line), "US %d.%d C%d T%d",
                       distance / 10, distance % 10,
                       snapshot.candidate ? 1 : 0, snapshot.threshold);
    } else {
        (void)snprintf(line, sizeof(line), "US -- C%d T%d",
                       snapshot.candidate ? 1 : 0, snapshot.threshold);
    }
    (void)tft_st7735_draw_text(0, 104, line, 0xffff, 0x0000);
    (void)snprintf(line, sizeof(line), "SEED %d V%02d Q%03d",
                   snapshot.seed_x, snapshot.valid_rows, snapshot.confidence);
    (void)tft_st7735_draw_text(0, 112, line, 0xffff, 0x0000);
    (void)snprintf(line, sizeof(line), "BALL %s", snapshot.ball_phase);
    (void)tft_st7735_draw_text(0, 120, line, 0xffff, 0x0000);
#endif
}

static void maybe_log_summary(int64_t now)
{
    camera_display_pipeline_stats_t pipeline = {0};
    camera_display_get_pipeline_stats(&pipeline);
    if (s_summary_start_us == 0) {
        s_summary_start_us = now;
        s_summary_camera_frames = pipeline.camera_frames;
        s_summary_processed_frames = pipeline.processed_frames;
        s_summary_control_frames = pipeline.control_frames;
        s_summary_preview_frames = pipeline.preview_frames;
        s_summary_dropped_frames = pipeline.frames_dropped;
        s_summary_callback_dropped_frames = s_callback_dropped_frames;
        return;
    }
    if (now - s_summary_start_us < 1000000) {
        return;
    }

    const uint32_t camera_fps = pipeline.camera_frames - s_summary_camera_frames;
    const uint32_t processed_fps = pipeline.processed_frames - s_summary_processed_frames;
    const uint32_t control_fps = pipeline.control_frames - s_summary_control_frames;
    const uint32_t preview_fps = pipeline.preview_frames - s_summary_preview_frames;
    const uint32_t frames_dropped = (pipeline.frames_dropped - s_summary_dropped_frames) +
                                    (s_callback_dropped_frames -
                                     s_summary_callback_dropped_frames);
    const uint32_t line_us_avg = s_control_frames == 0 ? 0 :
                                 s_line_us_sum / s_control_frames;
    const int64_t line_age_ms = s_last_line_us == 0 ? -1 :
                                (now - s_last_line_us) / 1000;
    const int64_t frame_age_ms = pipeline.last_control_age_us == 0 ? (int64_t)-1 :
                                 (int64_t)(pipeline.last_control_age_us / 1000U);
    const bool pending = turn_pending_active(now);
    const int64_t pending_ms = pending ?
                               (s_turn_pending_until_us - now) / 1000 : 0;
    const int obstacle_distance_x10 = s_ultrasonic_valid ?
                                      (int)(s_ultrasonic_distance_cm * 10.0f + 0.5f) : -1;
    const uint32_t obstacle_phase_ms = obstacle_elapsed_ms(now);
    ESP_LOGI(TAG,
             "fps camera=%u decoded=%u control=%u preview=%u drop=%u "
             "control_drop=%u preview_drop=%u callback_drop=%u frame=%ux%u "
             "age_ms=%lld timing_us[control_decode,preview_decode,threshold,tft]=[%u,%u,%u,%u] "
             "line_us[avg,max]=[%u,%u]",
             (unsigned)camera_fps, (unsigned)processed_fps,
             (unsigned)control_fps, (unsigned)preview_fps,
             (unsigned)frames_dropped,
             (unsigned)pipeline.control_dropped_frames,
             (unsigned)pipeline.preview_dropped_frames,
             (unsigned)(s_callback_dropped_frames -
                        s_summary_callback_dropped_frames),
             (unsigned)s_last_frame_width, (unsigned)s_last_frame_height,
             (long long)frame_age_ms,
             (unsigned)pipeline.control_decode_us,
             (unsigned)pipeline.preview_decode_us,
             (unsigned)pipeline.threshold_us,
             (unsigned)pipeline.tft_us,
             (unsigned)line_us_avg, (unsigned)s_line_us_max);
    ESP_LOGI(TAG,
             "vision state=%s armed=%d STBY=%d candidate=%d held=%d arm_frames=%u lost=%u "
             "reacq=%u/%u seed_valid=%d threshold=%d seed_x=%d line_w=%d "
             "scan_bottom=%d points=%d valid_rows=%u near_rows=%d confidence=%u "
             "near_line=%d far_error=%d corner=%d@%d pending=%d/%lldms "
             "line_age_ms=%lld avoid=%s dist_x10=%d dist_ok=%d phase_ms=%u ball=%s",
             state_name(), s_armed, s_stby_enabled, s_last_candidate,
             s_last_candidate_held,
             (unsigned)s_arm_frames, (unsigned)s_lost_frames,
             (unsigned)s_reacquire_frames, (unsigned)LINE_REACQUIRE_CONFIRM_FRAMES,
             s_seed_valid, s_last_threshold, s_last_seed_x, s_line_width,
             s_last_scan_bottom_y, s_last_point_count,
             (unsigned)s_last_valid_rows, s_last_near_normal_rows,
             (unsigned)s_last_confidence, s_last_near_line_visible,
             s_last_far_error, s_last_corner_direction, s_last_corner_row_y,
             pending, (long long)pending_ms, (long long)line_age_ms,
             obstacle_state_name(), obstacle_distance_x10, s_ultrasonic_valid,
             (unsigned)obstacle_phase_ms, ball_phase_name());
    ESP_LOGI(TAG,
             "control forward[target,ramped]=[%d,%d] drive[in f,t,l]=[%d,%d,%d] "
             "lat[raw,filtered,delta,cmd]=[%d,%d,%d,%d] "
             "head[raw,filtered,delta,cmd]=[%d,%d,%d,%d] "
             "mix_pre[A,B,D]=[%d,%d,%d] mix_post[A,B,D]=[%d,%d,%d] "
             "scaled=%d dropped=%d repaired=%d kick[A,B,D]=[%u,%u,%u] "
             "dir[A,B,D]=[%d,%d,%d] motor[A,B,D]=[%d,%d,%d]",
             s_last_forward_target, s_last_forward_ramped,
             s_last_drive_forward, s_last_drive_turn, s_last_drive_lat,
             s_last_lateral_raw, s_last_lateral_filtered, s_last_lateral_delta,
             s_last_drive_lat, s_last_heading_raw, s_last_heading_filtered,
             s_last_heading_delta, s_last_drive_turn,
             s_mix_pre_a, s_mix_pre_b, s_mix_pre_d,
             s_mix_post_a, s_mix_post_b, s_mix_post_d,
             s_mix_scaled, s_mix_dropped, s_mix_repaired,
             (unsigned)s_kick_cycles[0], (unsigned)s_kick_cycles[1],
             (unsigned)s_kick_cycles[2], s_last_direction[0],
             s_last_direction[1], s_last_direction[2],
             s_command_a, s_command_b, s_command_d);

    s_summary_camera_frames = pipeline.camera_frames;
    s_summary_processed_frames = pipeline.processed_frames;
    s_summary_control_frames = pipeline.control_frames;
    s_summary_preview_frames = pipeline.preview_frames;
    s_summary_dropped_frames = pipeline.frames_dropped;
    s_summary_callback_dropped_frames = s_callback_dropped_frames;
    s_summary_start_us = now;
    s_control_frames = 0;
    s_line_us_sum = 0;
    s_line_us_max = 0;
}

static void disarm_tracking(void)
{
    stop_motors();
    s_obstacle_state = OBSTACLE_IDLE;
    s_obstacle_close_samples = 0;
    s_obstacle_phase_start_us = 0;
    s_obstacle_cooldown_until_us = 0;
    s_obstacle_reacquire_until_us = 0;
    s_obstacle_ready = true;
    s_ball_phase = BALL_IDLE;
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
    if (!s_started) {
        goto done;
    }
    if (s_finished) {
        /* The line is complete (or deliberately bypassed in the standalone
         * endpoint image), and the latest frame feeds only the ball state
         * machine.  Start the direct test on the first decoded frame so the
         * search timeout does not run while USB Host is still booting. */
#if BALL_DIRECT_TEST_MODE
        if (s_ball_phase == BALL_IDLE) {
            ball_begin(now);
        }
#endif
        s_last_frame_width = width;
        s_last_frame_height = height;
        ball_process_frame(rgb565_big_endian, width, height, now);
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
    s_last_frame_width = width;
    s_last_frame_height = height;
    if (s_state == LINE_STATE_CORNER && s_turn_frames < UINT32_MAX) {
        ++s_turn_frames;
    }
    if (s_state == LINE_STATE_LOST && s_lost_frames < UINT32_MAX) {
        ++s_lost_frames;
    }
    const bool detected_candidate = observe_line(rgb565_big_endian, width, height,
                                                 source_threshold, draw_overlay,
                                                 &observation);
    bool candidate = detected_candidate;
    bool held_candidate = false;
    if (detected_candidate) {
        s_last_good_observation = observation;
        s_have_last_good_observation = true;
        s_candidate_miss_frames = 0;
    } else if (s_armed && s_state == LINE_STATE_NORMAL &&
               !turn_pending_active(now) && s_seed_valid &&
               observation.point_count >= LINE_PARTIAL_MIN_POINTS &&
               observation.near_normal_rows >= LINE_PARTIAL_MIN_POINTS &&
               s_last_line_us != 0 &&
               now - s_last_line_us <= (int64_t)LINE_PARTIAL_TRACK_HOLD_MS * 1000 &&
               s_candidate_miss_frames < LINE_SOFT_LOST_FRAMES) {
        /* A partial scan is usually a one-frame JPEG/lighting miss. Keep the
         * previous control error while showing the real points that were found. */
        ++s_candidate_miss_frames;
        observation.candidate = true;
        observation.seed_x = s_seed_x;
        observation.lateral_error = s_last_lateral_error;
        observation.heading_error = s_last_heading_error;
        observation.far_error = s_last_far_error;
        observation.near_line_visible = true;
        candidate = true;
        held_candidate = true;
    } else if (s_armed && s_state == LINE_STATE_NORMAL &&
               !turn_pending_active(now) && s_have_last_good_observation &&
               s_candidate_miss_frames < LINE_SOFT_LOST_FRAMES) {
        /* A single threshold/aliasing miss must not immediately remove the
         * line from control or the TFT. Reuse only the last real observation
         * for a few frames, with corner events disabled so it cannot retrigger
         * a turn from stale data. */
        ++s_candidate_miss_frames;
        observation = s_last_good_observation;
        observation.corner_direction = 0;
        observation.corner_row_y = -1;
        observation.corner_x = -1;
        observation.finish_candidate = false;
        observation.candidate = true;
        candidate = true;
        held_candidate = true;
    } else if (!detected_candidate && s_candidate_miss_frames < UINT8_MAX) {
        ++s_candidate_miss_frames;
    }
    save_overlay_snapshot(width, height, &observation, 0);
    s_last_scan_bottom_y = observation.scan_bottom_y;
    s_last_point_count = observation.point_count;
    s_last_near_normal_rows = observation.near_normal_rows;
    s_last_far_error = observation.far_error;
    s_last_corner_direction = observation.corner_direction;
    s_last_corner_row_y = observation.corner_row_y;
    s_last_near_line_visible = observation.near_line_visible;
    s_last_threshold = observation.threshold;
    s_last_candidate = detected_candidate;
    s_last_candidate_held = held_candidate;
    s_last_seed_x = observation.valid_rows > 0 ? observation.seed_x : -1;
    s_last_valid_rows = observation.valid_rows;
    s_last_confidence = observation.confidence;
    if (candidate) {
        s_last_lateral_error = observation.lateral_error;
        s_last_heading_error = observation.heading_error;
    }

    if (!s_armed) {
        if (detected_candidate &&
            observation.valid_rows >= LINE_ARM_MIN_VALID_ROWS &&
            now - s_first_frame_us >= (int64_t)LINE_START_DELAY_MS * 1000) {
            if (s_arm_frames < LINE_ARM_CONFIRM_FRAMES) {
                ++s_arm_frames;
            }
            if (s_arm_frames >= LINE_ARM_CONFIRM_FRAMES) {
                s_armed = true;
                s_state = LINE_STATE_NORMAL;
                reset_control(true);
                update_seed(&observation);
                s_last_line_us = now;
                s_motor_start_us = now;
                gpio_set_level(STBY_GPIO, 1);
                s_stby_enabled = true;
                ESP_LOGI(TAG, "line confirmed; camera steering enabled");
            }
        } else if (!detected_candidate ||
                   observation.valid_rows < LINE_ARM_MIN_VALID_ROWS) {
            s_arm_frames = 0;
        }
        if (!s_armed) {
            stop_motors();
            goto done;
        }
        zero_motor_outputs();
        goto done;
    }

    /* Obstacle motion owns the motors until the fixed right shift is complete.
     * The current frame is still scanned above for the TFT overlay, but its
     * candidate cannot alter the line state or interrupt the manoeuvre. */
    if (obstacle_step(now, width, candidate && s_state == LINE_STATE_NORMAL)) {
        goto done;
    }

    /* ---- 终点 T：双侧敞开的横杆 + 下方仍有立柱 ---- */
    if (!observation.finish_candidate) {
        s_finish_frames = 0;
        s_finish_align_start_us = 0;
    } else if (LINE_FINISH_ENABLE && candidate && s_state == LINE_STATE_NORMAL) {
        if (s_finish_align_start_us == 0) {
            s_finish_align_start_us = now;
            /* Do not let the preceding line-follow derivative kick turn the
             * chassis while the T stem is being aligned. */
            reset_control(false);
        }

        const bool heading_aligned = observation.near_line_visible &&
                                     abs(observation.heading_error) <=
                                         LINE_FINISH_ALIGN_HEADING_ERROR;
        if (heading_aligned) {
            if (s_finish_frames < LINE_FINISH_CONFIRM_FRAMES) {
                ++s_finish_frames;
            }
        } else {
            s_finish_frames = 0;
        }

        const int64_t align_us = now - s_finish_align_start_us;
        if (s_finish_frames >= LINE_FINISH_CONFIRM_FRAMES ||
            align_us >= (int64_t)LINE_FINISH_ALIGN_TIMEOUT_MS * 1000) {
            s_finished = true;
            s_state = LINE_STATE_NORMAL;
            stop_motors();
            ball_begin(now);
            ESP_LOGW(TAG, "finish T stopped at row %d; heading=%d aligned=%d",
                     observation.corner_row_y, observation.heading_error,
                     heading_aligned);
            goto done;
        }
        /* Keep the T stem as the forward reference.  Ignore the lateral
         * offset used for normal tracking so the nose, rather than the
         * camera's optical center, is aligned before stopping. */
        line_observation_t finish_control = observation;
        finish_control.lateral_error = 0;
        finish_control.far_error = 0;
        finish_control.corner_direction = 0;
        finish_control.finish_candidate = true;
        drive_normal(&finish_control, now);
        goto done;
    } else {
        s_finish_frames = 0;
        s_finish_align_start_us = 0;
    }

    /* ---- 折角：绕摄像头原地旋转，用近场闭环退出 ---- */
    if (s_state == LINE_STATE_CORNER) {
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
            reset_control(false);
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
            drive_slow_spin(turn);
        }
        goto done;
    }

    /* ---- LOST：只在最后可信种子附近重捕获 ---- */
    if (s_state == LINE_STATE_LOST) {
        const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
        const bool obstacle_grace = s_obstacle_reacquire_until_us != 0 &&
                                    now < s_obstacle_reacquire_until_us;
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
                reset_control(false);
                s_obstacle_reacquire_until_us = 0;
                drive_normal(&observation, now);
                goto done;
            }
            if (near_last_seed) {
                /* 重捕获阶段也沿用红外巡线式转向，避免横移把车推离赛道。 */
                const int mirror = CAMERA_LINE_MIRROR_X ? -1 : 1;
                const int correction =
                    mirror * direction_of(observation.seed_x - s_seed_x);
                drive(LINE_FORWARD_CRAWL,
                      -correction * LINE_YAW_MIN_OUTPUT, 0);
            } else {
                drive(LINE_FORWARD_CRAWL, 0, 0);
            }
        } else {
            s_reacquire_frames = 0;
            s_reacquire_x = s_seed_x;
            if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000 || obstacle_grace) {
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
        s_obstacle_reacquire_until_us = 0;
        s_lost_frames = 0;
        s_reacquire_frames = 0;
        if (!held_candidate) {
            s_last_line_us = now;
            update_seed(&observation);
        }

        /* 近场折角先挂起；只有旧线随后消失才进入 CORNER 原地慢转。 */
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
                s_turn_pending_until_us = 0;
            }
            if (s_turn_hint_frames >= LINE_TURN_HINT_FRAMES) {
                s_turn_pending_until_us = now +
                                           (int64_t)LINE_TURN_PENDING_MS * 1000;
            }
        } else if (!turn_pending_active(now)) {
            s_turn_hint_frames = 0;
            s_turn_direction = 0;
            s_turn_pending_until_us = 0;
        }
        drive_normal(&observation, now);
        goto done;
    }

    if (turn_pending_active(now)) {
        s_state = LINE_STATE_CORNER;
        s_turn_frames = 0;
        s_turn_exit_frames = 0;
        s_turn_started_us = now;
        s_turn_pending_until_us = 0;
        reset_control(false);
        ESP_LOGI(TAG, "corner %s confirmed after old line loss; pivoting",
                 s_turn_direction < 0 ? "left" : "right");
        drive_slow_spin(-s_turn_direction * LINE_PIVOT_TURN);
        goto done;
    }

    s_state = LINE_STATE_LOST;
    s_lost_frames = 1;
    s_reacquire_frames = 0;
    s_reacquire_x = s_seed_x;
    reset_control(false);
    {
        const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
        const bool obstacle_grace = s_obstacle_reacquire_until_us != 0 &&
                                    now < s_obstacle_reacquire_until_us;
        if (lost_us <= (int64_t)LINE_LOST_HOLD_MS * 1000 || obstacle_grace) {
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

static void camera_ultrasonic_task(void *arg)
{
    (void)arg;
    while (true) {
        if (!s_started) {
            vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
            continue;
        }

        const float distance = ultrasonic_read_cm();
        const bool valid = distance >= ULTRASONIC_MIN_CM &&
                           distance <= ULTRASONIC_MAX_CM;
        s_ultrasonic_distance_cm = valid ? distance : -1.0f;
        s_ultrasonic_valid = valid;
        ++s_ultrasonic_sequence;
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
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
    if (s_overlay_mutex == NULL) {
        s_overlay_mutex = xSemaphoreCreateMutex();
        if (s_overlay_mutex == NULL) {
            ESP_LOGE(TAG, "Could not allocate overlay mutex");
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

    ultrasonic_init(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    s_started = true;
    s_armed = false;
    s_finished = false;
    s_stby_enabled = false;
    s_ultrasonic_distance_cm = -1.0f;
    s_ultrasonic_valid = false;
    s_ultrasonic_sequence = 0;
    s_obstacle_state = OBSTACLE_IDLE;
    s_obstacle_close_samples = 0;
    s_obstacle_last_sequence = 0;
    s_obstacle_phase_start_us = 0;
    s_obstacle_cooldown_until_us = 0;
    s_obstacle_reacquire_until_us = 0;
    s_obstacle_ready = true;
    s_ball_phase = BALL_IDLE;
    s_ball_phase_start_us = 0;
    s_ball_align_frames = 0;
    s_ball_x = 0;
    s_ball_y = 0;
    s_ball_target_valid = false;
    s_ball_search_direction = 1;
    s_ball_target_miss_frames = 0;
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
    s_mix_repaired = false;
    s_summary_start_us = 0;
    s_summary_camera_frames = 0;
    s_summary_processed_frames = 0;
    s_summary_control_frames = 0;
    s_summary_preview_frames = 0;
    s_summary_dropped_frames = 0;
    s_callback_dropped_frames = 0;
    s_summary_callback_dropped_frames = 0;
    s_overlay_snapshot = (line_overlay_snapshot_t){0};
    s_control_frames = 0;
    s_line_us_sum = 0;
    s_line_us_max = 0;
    reset_tracking();
#if BALL_DIRECT_TEST_MODE
    /* This branch starts with the chassis already at the endpoint.  Mark the
     * route complete and arm the motor permission latch, but defer ball_begin
     * until the first camera frame arrives (see process_frame above). */
    s_finished = true;
    s_armed = true;
    ESP_LOGW(TAG,
             "BALL_DIRECT_TEST_MODE=1: line following, finish T, and obstacle "
             "avoidance skipped; waiting for first frame at endpoint");
#endif
    gpio_set_level(STBY_GPIO, 0);
    stop_motors();

    if (!s_ultrasonic_task_created) {
        if (xTaskCreate(camera_ultrasonic_task, "camera_ultrasonic", 2048,
                        NULL, 1, NULL) != pdPASS) {
            s_started = false;
            ESP_LOGE(TAG, "Could not create ultrasonic task");
            return ESP_ERR_NO_MEM;
        }
        s_ultrasonic_task_created = true;
    }

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
#if BALL_DIRECT_TEST_MODE
    ESP_LOGI(TAG, "Endpoint ball test ready; place vehicle at finish and keep camera view clear");
#else
    ESP_LOGI(TAG, "Camera line follower ready; motors stay stopped until a stable line is seen");
#endif
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
    s_obstacle_state = OBSTACLE_IDLE;
    s_obstacle_close_samples = 0;
    s_obstacle_phase_start_us = 0;
    s_obstacle_cooldown_until_us = 0;
    s_obstacle_reacquire_until_us = 0;
    s_obstacle_ready = true;
    s_ball_phase = BALL_IDLE;
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

void camera_line_follow_preview_callback(uint8_t *rgb565_big_endian,
                                         uint16_t width,
                                         uint16_t height,
                                         uint8_t source_threshold,
                                         uint32_t sequence,
                                         int64_t capture_us,
                                         void *user_ctx)
{
    (void)source_threshold;
    (void)sequence;
    (void)capture_us;
    (void)user_ctx;
    /* This callback is display-only. It never scans pixels or touches motors. */
    render_preview_overlay(rgb565_big_endian, width, height);
}
