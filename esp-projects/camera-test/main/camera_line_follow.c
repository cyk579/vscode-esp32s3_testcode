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
#include "ultrasonic.h"

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
/* Negative values shift the desired track center left in the camera image. */
#define CAMERA_LINE_CENTER_BIAS_PX (-4)

/* 摄像头相对车体的安装旋转。扫描坐标系永远是车体视角（sy 越大越靠近车），
 * 缓冲区按这个值反查，不做整帧旋转拷贝，所以改它不增加单帧耗时。
 * 判断方法：看 TFT 上的绿点是**沿着**胶带走还是**横切**胶带；或者看日志
 * 的 valid_rows —— 装反了会一直是 0~3 且反复 LOST/finish。 */
#define CAMERA_LINE_ROTATION LINE_ROTATE_0

/* ROI、行距、逐行搜索和线段判据全部在 line_geometry.h 里，ESP 端和 host
 * 回归测试共用同一组常量。这里只保留状态机自己的窗口策略和时序。 */
#define LINE_LOST_WINDOW_GROW_PERCENT 5
#define LINE_LOST_WINDOW_MAX_PERCENT 48
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
#define LINE_ERROR_FILTER_OLD 2
#define LINE_ERROR_FILTER_NEW 4
#define LINE_PD_KD_LAT 20
#define LINE_PD_KD_HEADING 50

/* 这些值保留原有的上电、限速、换向保护和超时停车行为。 */
#define LINE_START_DELAY_MS 600U
#define LINE_ARM_CONFIRM_FRAMES 3U
#define LINE_MOTOR_START_RAMP_MS 700
#define LINE_MOTOR_START_MIN_OUTPUT 14
/* 只在首次 armed 后短暂提高 forward，先克服静摩擦；仍在 mixer 前
 * 处理，因此不会逐轮改变 A/B/D 比例，也不会重复触发 startup kick。 */
#define LINE_START_TRACTION_OUTPUT 34
#define LINE_START_TRACTION_MS 350U
#define LINE_LOST_HOLD_MS 600U
#define LINE_LOST_STOP_MS 1800U
#define LINE_FRAME_TIMEOUT_MS 1200U
#define LINE_FINISH_CONFIRM_FRAMES 3U
/* 终点 T 停车；固定避障路线完成后才会启用。 */
#define LINE_FINISH_ENABLE 1

/* 22% 是 car-spin 已验证的低速可持续区间；斜坡仍按控制周期平滑上升。 */
#define LINE_FORWARD_FAST 22
#define LINE_FORWARD_MEDIUM 20
#define LINE_FORWARD_SLOW 18
#define LINE_FORWARD_CRAWL 14
#define LINE_FORWARD_SLEW 4

/* A two-point partial scan is often a one-frame JPEG/lighting miss, not a
 * genuine loss of the track. Keep the previous steering briefly while the
 * detector catches up, but do not let partial noise extend the line age. */
#define LINE_PARTIAL_MIN_POINTS 2
#define LINE_PARTIAL_TRACK_HOLD_MS 450U

/* 误差门限。|error| 被 ROI 夹在 55 以内（center 只能落在 48..191），所以
 * 原来的 LINE_ERROR_LARGE=60 在 NORMAL 里永远不可达，CRAWL 那一档是死的。 */
#define LINE_ERROR_DEADBAND 12
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
#define LINE_TURN_MAX 13
#define LINE_YAW_MIN_OUTPUT 13
#define LINE_LAT_MAX 16
#define LINE_LAT_MIN_OUTPUT 11
#define LINE_SEED_SLEW_PX 12

/* 折角：事件行进入画面下方 88% 才动手；更远处只降速，避免提前转弯。 */
#define LINE_TURN_TRIGGER_PERCENT 88
#define LINE_TURN_HINT_FRAMES 1U
#define LINE_TURN_EXIT_FRAMES 2U
/* 旋转至少持续这么久才接受退出，否则第一帧还在看入弯前那条线就会假退出，
 * 然后立刻又检测到同一个折角，来回抖。 */
#define LINE_TURN_MIN_MS 250U
#define LINE_TURN_EXIT_ERROR 30
#define LINE_TURN_TIMEOUT_MS 2500U
/* 出弯后先低速回中，连续稳定若干帧再恢复 NORMAL；超时只作为兜底。 */
#define LINE_POST_CORNER_ALIGN_LATERAL_ERROR 18
#define LINE_POST_CORNER_ALIGN_HEADING_ERROR 8
#define LINE_POST_CORNER_ALIGN_FRAMES 4U
#define LINE_POST_CORNER_ALIGN_TIMEOUT_MS 1200U
/* 绕摄像头旋转：lat = turn * a/(2L)。偏航量必须够大，否则 a/d = lat - turn
 * 落在起转值以下会被混控丢掉，只剩后轮在推。
 * TODO(实测): LINE_CAM_PIVOT_PERCENT 用尺子量 a 和 L 后填 a*100/(2L)。 */
#define LINE_PIVOT_TURN 15
#define LINE_TURN_A_SPEED 14
#define LINE_TURN_B_SPEED 13
#define LINE_TURN_D_SPEED 15
#define LINE_TURN_PENDING_MS 500U
/* 摄像头在底盘前方：旧线消失并确认要转弯后，先让底盘向前走一小段。 */
#define LINE_CORNER_CENTER_DELAY_MS 200U
/* 普通丢线时按最近转向方向低速扫线；每隔一小段时间换向，避免卡在
 * 台阶弯的内侧。避障完成后的 grace 期间仍使用直行重捕获。 */
#define LINE_LOST_SEARCH_SWITCH_MS 450U
/* 低速调试时优先保证三轮都超过各自起转阈值；此前叠加 40% 横移后，
 * 在较低总上限下 A/D 被缩放掉，锐角阶段实际只剩 B 轮。 */
#define LINE_CAM_PIVOT_PERCENT 0
#define LINE_ALERT_MS 900U

/* NORMAL 低速用时间脉冲而不是把 PWM 压到静摩擦死区。MIN 只抬 mixer
 * 输入的 forward，绝不逐轮改 A/B/D；esp_timer 只负责切换 duty。 */
#define LINE_BURST_ENABLE 1
#define LINE_BURST_PERIOD_MS 600U
#define LINE_BURST_ON_MS 580U
#define LINE_BURST_MIN_OUTPUT 27
#define LINE_BURST_TICK_MS 5U

/* 直线回中只补一个很小的 yaw，8 个误差单位以内保持死区，避免来回抖动。 */
#define LINE_CENTER_YAW_DEADBAND 8
#define LINE_CENTER_YAW_GAIN 18
#define LINE_CENTER_YAW_MAX 5

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
#define START_KICK_OUTPUT 27
#define START_KICK_CYCLES 3U
/* 正常巡航保持 22%，burst 前进下限为 27%，给转向混控留出三轮起转余量。
 * 对 f=27、turn=±13 的 NORMAL 向量，44 比最大实际轮值 40 多留 4 个单位。 */
#define MOTOR_PWM_CEILING 44
#define LINE_SPEED_CAP 30
#define MOTOR_TRIM_A 90
#define MOTOR_TRIM_D 100

/* 方向符号与 car-spin 的实车校准一致。 */
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN -1

/* HC-SR04 obstacle route. Motion timing is independent of camera FPS. */
#define ULTRASONIC_TRIG GPIO_NUM_18
#define ULTRASONIC_ECHO GPIO_NUM_11
#define ULTRASONIC_MIN_CM 2.0f
#define ULTRASONIC_MAX_CM 400.0f
#define OBSTACLE_DETECT_CM 20.0f
#define OBSTACLE_CLOSE_CONFIRM_SAMPLES 2U
#define ULTRASONIC_PERIOD_MS 60U
#define AVOID_BRAKE_MS 500U
#define AVOID_LEFT_MS 1500U
#define AVOID_FORWARD_MS 1500U
#define AVOID_RIGHT_MS 1300U
#define AVOID_REACQUIRE_GRACE_MS 1000U
#define AVOID_LEFT_A_SPEED 24
#define AVOID_LEFT_B_SPEED 33
#define AVOID_LEFT_D_SPEED 18
#define AVOID_RIGHT_A_SPEED 18
#define AVOID_RIGHT_B_SPEED 30
#define AVOID_RIGHT_D_SPEED 24
#define AVOID_FORWARD_A_SPEED 22
#define AVOID_FORWARD_D_SPEED 25

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
static int64_t s_obstacle_reacquire_until_us;
static bool s_obstacle_completed;
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

static int s_turn_direction;
static int s_lost_search_direction;
static uint8_t s_turn_hint_frames;
static int64_t s_turn_pending_until_us;
static int64_t s_corner_center_delay_until_us;
static bool s_post_corner_align;
static uint8_t s_post_corner_align_frames;
static int64_t s_post_corner_align_started_us;
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

#if LINE_BURST_ENABLE
static esp_timer_handle_t s_burst_timer;
static volatile bool s_burst_active;
static volatile bool s_burst_on;
static volatile bool s_burst_timer_running;
static volatile bool s_burst_context;
static volatile int s_burst_output[3];
static int64_t s_burst_epoch_us;
#endif

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

/* 在进入 LOST 前记住应该先扫向哪一侧。corner detector 已给出方向时优先
 * 使用它；否则由上一帧的 yaw 命令反推（turn 命令与 corner 方向相反），
 * 再没有信息时从左侧开始。 */
static void remember_lost_search_direction(void)
{
    int direction = s_turn_direction;
    if (direction == 0) {
        direction = -direction_of(s_last_drive_turn);
    }
    if (direction == 0) {
        /* heading_error 的符号与即将进入的弯向一致。 */
        direction = direction_of(s_last_heading_error);
    }
    if (direction == 0 && abs(s_last_lateral_error) > LINE_ERROR_DEADBAND) {
        /* 近场线明显偏向一侧时，优先向线所在侧扫回去；这能避免
         * yaw dither 恰好输出 0 时落入固定的左侧兜底。 */
        direction = -direction_of(s_last_lateral_error);
    }
    if (direction == 0) {
        direction = -1;
    }
    s_lost_search_direction = direction;
}

static int lost_search_turn_command(int64_t lost_us)
{
    int direction = s_lost_search_direction;
    if (direction == 0) {
        remember_lost_search_direction();
        direction = s_lost_search_direction;
    }

    const uint64_t switch_us = (uint64_t)(LINE_LOST_SEARCH_SWITCH_MS == 0 ?
                                          1U : LINE_LOST_SEARCH_SWITCH_MS) * 1000U;
    const uint64_t hold_us = (uint64_t)LINE_LOST_HOLD_MS * 1000U;
    const uint64_t search_age_us = lost_us > (int64_t)hold_us ?
                                   (uint64_t)lost_us - hold_us : 0U;
    if (lost_us >= 0 && (search_age_us / switch_us) & 1U) {
        direction = -direction;
    }
    /* drive_slow_spin 使用已校准的低速三轮输出，这里只需要传递方向。 */
    return -direction * LINE_PIVOT_TURN;
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
    /* Keep the last usable overlay through a detector miss. Clearing it here
     * made the TFT appear to stop updating exactly when the car lost the line,
     * which hid the evidence needed to tune the controller. */
    if (observation->point_count == 0) {
        (void)xSemaphoreGive(s_overlay_mutex);
        return;
    }
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
    if (s_state == LINE_STATE_CORNER && !s_post_corner_align) {
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
     * 重新找最靠中心的黑线，而不是围着入弯前的位置猜。出弯回中阶段已经停止
     * 旋转，重新沿用 seed 附近的历史跟踪，避免把回中误判成一次新折角。 */
    const bool corner_spin = s_state == LINE_STATE_CORNER && !s_post_corner_align;
    cfg.use_history = s_seed_valid && !corner_spin;
    cfg.seed_x = corner_spin ? (int)width / 2 : s_seed_x;
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
        observation->far_error = clamp_int(observation->far_error +
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

#if LINE_BURST_ENABLE
static uint64_t burst_period_us(void)
{
    const uint32_t period_ms = LINE_BURST_PERIOD_MS == 0 ? 1U :
                               LINE_BURST_PERIOD_MS;
    return (uint64_t)period_ms * 1000U;
}

static uint64_t burst_on_us(void)
{
    const uint64_t period = burst_period_us();
    uint32_t on_ms = LINE_BURST_ON_MS;
    if (on_ms == 0) {
        on_ms = 1U;
    }
    uint64_t on = (uint64_t)on_ms * 1000U;
    /* 至少留出一个 tick 的 OFF 窗口，避免参数误设成连续输出。 */
    const uint64_t tick = (uint64_t)(LINE_BURST_TICK_MS == 0 ? 1U :
                                     LINE_BURST_TICK_MS) * 1000U;
    if (on >= period) {
        on = period > tick ? period - tick : period;
    }
    return on;
}

static void burst_apply_duty(bool on)
{
    const int output_a = on ? s_burst_output[0] : 0;
    const int output_b = on ? s_burst_output[1] : 0;
    const int output_d = on ? s_burst_output[2] : 0;
    set_motor_duty(&motor_a, output_a);
    set_motor_duty(&motor_b, output_b);
    set_motor_duty(&motor_d, output_d);
}

static void normal_burst_timer_callback(void *arg)
{
    (void)arg;
    if (!s_burst_active) {
        return;
    }
    /* 真正的停车路径仍由 process/watchdog 调用 stop_motors；这里仅把 duty
     * 拉低，绝不改 armed、state、seed 或任何巡线历史。 */
    if (!s_started || !s_armed || !s_stby_enabled || s_finished) {
        s_burst_on = false;
        burst_apply_duty(false);
        return;
    }

    const int64_t elapsed = esp_timer_get_time() - s_burst_epoch_us;
    const uint64_t period = burst_period_us();
    const uint64_t on_time = burst_on_us();
    bool on = true;
    if (elapsed >= 0) {
        on = ((uint64_t)elapsed % period) < on_time;
    }
    if (on != s_burst_on) {
        s_burst_on = on;
        burst_apply_duty(on);
    }
}

static bool normal_burst_activate(void)
{
    if (s_burst_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = normal_burst_timer_callback,
            .arg = NULL,
            .name = "line_burst",
        };
        const esp_err_t err = esp_timer_create(&args, &s_burst_timer);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "NORMAL burst timer unavailable (%s); using direct duty",
                     esp_err_to_name(err));
            return false;
        }
    }

    if (!s_burst_active) {
        s_burst_active = true;
        s_burst_on = true;
        s_burst_epoch_us = esp_timer_get_time();
        s_burst_output[0] = 0;
        s_burst_output[1] = 0;
        s_burst_output[2] = 0;
    }
    if (!s_burst_timer_running) {
        const uint64_t tick_us = (uint64_t)(LINE_BURST_TICK_MS == 0 ? 1U :
                                            LINE_BURST_TICK_MS) * 1000U;
        const esp_err_t err = esp_timer_start_periodic(s_burst_timer, tick_us);
        if (err != ESP_OK) {
            s_burst_active = false;
            s_burst_on = false;
            ESP_LOGW(TAG, "NORMAL burst timer start failed (%s); using direct duty",
                     esp_err_to_name(err));
            return false;
        }
        s_burst_timer_running = true;
    }
    return true;
}

static void normal_burst_deactivate(void)
{
    if (!s_burst_active && !s_burst_timer_running) {
        return;
    }
    s_burst_active = false;
    if (s_burst_timer_running && s_burst_timer != NULL) {
        (void)esp_timer_stop(s_burst_timer);
        s_burst_timer_running = false;
    }
    s_burst_on = false;
    s_burst_output[0] = 0;
    s_burst_output[1] = 0;
    s_burst_output[2] = 0;
    burst_apply_duty(false);
}
#else
static bool normal_burst_activate(void)
{
    return false;
}

static void normal_burst_deactivate(void)
{
}
#endif

static void motor_set(const motor_t *motor, int command)
{
    /* command 是混控层有符号 PWM；换向时先清零，避免 TB6612 硬切方向。 */
    const int index = (int)motor->channel;
    const int speed = clamp_int(command * motor->sign, MOTOR_PWM_CEILING);
    const int direction = (speed > 0) - (speed < 0);
    const bool direction_changed = direction != s_last_direction[index];
#if LINE_BURST_ENABLE
    const bool burst_output = s_burst_context && s_burst_active;
#else
    const bool burst_output = false;
#endif

    if (direction_changed) {
        set_motor_duty(motor, 0);
        gpio_set_level(motor->in1, speed > 0);
        gpio_set_level(motor->in2, speed < 0);
        s_last_direction[index] = direction;
        s_kick_cycles[index] = direction == 0 ? 0 : START_KICK_CYCLES;
#if LINE_BURST_ENABLE
        if (burst_output) {
            s_burst_output[index] = 0;
        }
#endif
    }

    if (direction == 0) {
        s_kick_cycles[index] = 0;
        gpio_set_level(motor->in1, 0);
        gpio_set_level(motor->in2, 0);
#if LINE_BURST_ENABLE
        if (burst_output) {
            s_burst_output[index] = 0;
        }
#endif
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
#if LINE_BURST_ENABLE
    if (burst_output) {
        /* mixer 已经确定 A/B/D 的比例；burst 这里只缓存并门控 duty。 */
        s_burst_output[index] = output;
        /* OFF 只关 duty，方向脚和 kick 计数仍保持；下一次 ON 不会重新
         * 经过 motor_set，因此不会再次触发完整 startup kick。 */
        set_motor_duty(motor, s_burst_on ? output : 0);
        return;
    }
#endif
    set_motor_duty(motor, output);
}

static void zero_motor_outputs(void)
{
    /* 进入真正的停车/非 NORMAL 路径时先停掉时间脉冲；这不会清理巡线
     * 历史，只保证 timer 不会在停车后再次拉高 duty。 */
    normal_burst_deactivate();
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

static void drive_internal(int forward, int turn, int lat,
                           bool normal_burst)
{
    const line_mixer_cfg_t cfg = {
        MOTOR_PWM_CEILING, MOTOR_MIN_RUN_OUTPUT, MOTOR_B_MIN_RUN_OUTPUT,
        MOTOR_TRIM_A, MOTOR_TRIM_D,
    };
    line_mixer_out_t out = {0, 0, 0, false, false};
    const int64_t now = esp_timer_get_time();
    s_last_drive_forward = forward;
    s_last_drive_turn = turn;
    s_last_drive_lat = lat;
    forward = clamp_int(forward, forward_ramp_cap(now));
#if LINE_BURST_ENABLE
    if (normal_burst && forward != 0) {
        int minimum = LINE_BURST_MIN_OUTPUT;
        if (s_motor_start_us != 0 && now >= s_motor_start_us &&
            now - s_motor_start_us < (int64_t)LINE_START_TRACTION_MS * 1000 &&
            minimum < LINE_START_TRACTION_OUTPUT) {
            minimum = LINE_START_TRACTION_OUTPUT;
        }
        if (abs(forward) < minimum) {
            forward = direction_of(forward) * minimum;
        }
    }
#endif
    s_last_forward_ramped = forward;
    const int forward_a = forward * MOTOR_TRIM_A / 100;
    const int forward_d = forward * MOTOR_TRIM_D / 100;
    s_mix_pre_a = -forward_a - turn + lat;
    s_mix_pre_b = turn + 2 * lat;
    s_mix_pre_d = forward_d - turn + lat;
    line_mixer_solve(forward, turn, lat, &cfg, &out);
    s_command_a = out.a;
    s_command_b = out.b;
    s_command_d = out.d;
    s_mix_post_a = out.a;
    s_mix_post_b = out.b;
    s_mix_post_d = out.d;
    s_mix_scaled = out.scaled;
    s_mix_dropped = out.dropped;
#if LINE_BURST_ENABLE
    const bool burst_ready = normal_burst && normal_burst_activate();
    s_burst_context = burst_ready;
#else
    const bool burst_ready = false;
#endif
    motor_set(&motor_a, s_command_a);
    motor_set(&motor_b, s_command_b);
    motor_set(&motor_d, s_command_d);
#if LINE_BURST_ENABLE
    s_burst_context = false;
#endif
    (void)burst_ready;
}

static void drive(int forward, int turn, int lat)
{
    normal_burst_deactivate();
    drive_internal(forward, turn, lat, false);
}

static void drive_normal_output(int forward, int turn, int lat)
{
    drive_internal(forward, turn, lat, true);
}

/* Use the low-speed IR spin calibration in TURN.  The direct wheel commands
 * avoid the normal direction-change kick, which otherwise makes the camera
 * sweep past the line before the next frame is processed. */
static void drive_slow_spin(int turn)
{
    normal_burst_deactivate();
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

static void reset_control(void)
{
    s_lat_accum = 0;
    s_yaw_accum = 0;
    s_forward_output = 0;
    s_turn_output = 0;
    s_error_filter_initialized = false;
    s_lateral_control_error = 0;
    s_heading_control_error = 0;
    s_lateral_error_rate = 0;
    s_heading_error_rate = 0;
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
    s_lost_search_direction = 0;
    s_turn_direction = 0;
    s_turn_hint_frames = 0;
    s_turn_pending_until_us = 0;
    s_corner_center_delay_until_us = 0;
    s_post_corner_align = false;
    s_post_corner_align_frames = 0;
    s_post_corner_align_started_us = 0;
    s_turn_exit_frames = 0;
    s_turn_frames = 0;
    s_turn_started_us = 0;
    s_alert_until_us = 0;
    s_finish_frames = 0;
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

static void obstacle_drive_direct(int a, int b, int d)
{
    normal_burst_deactivate();
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
    s_command_a = a;
    s_command_b = b;
    s_command_d = d;
    motor_set(&motor_a, a);
    motor_set(&motor_b, b);
    motor_set(&motor_d, d);
}

static void obstacle_update_sensor(void)
{
    const uint32_t sequence = s_ultrasonic_sequence;
    if (sequence == s_obstacle_last_sequence) {
        return;
    }
    s_obstacle_last_sequence = sequence;
    const float distance = s_ultrasonic_distance_cm;
    const bool close = s_ultrasonic_valid &&
                       distance >= ULTRASONIC_MIN_CM &&
                       distance <= OBSTACLE_DETECT_CM;
    if (close) {
        if (s_obstacle_close_samples < UINT32_MAX) {
            ++s_obstacle_close_samples;
        }
    } else {
        s_obstacle_close_samples = 0;
    }
}

static void obstacle_begin(int64_t now)
{
    s_obstacle_state = OBSTACLE_BRAKE;
    s_obstacle_phase_start_us = now;
    s_obstacle_close_samples = 0;
    s_obstacle_reacquire_until_us = 0;
    reset_control();
    zero_motor_outputs();
    ESP_LOGW(TAG, "obstacle %.1fcm confirmed; fixed avoidance route started",
             (double)s_ultrasonic_distance_cm);
}

static void obstacle_finish(int64_t now, uint16_t width)
{
    s_obstacle_state = OBSTACLE_IDLE;
    s_obstacle_phase_start_us = 0;
    s_obstacle_close_samples = 0;
    s_obstacle_completed = true;
    s_obstacle_reacquire_until_us =
        now + (int64_t)AVOID_REACQUIRE_GRACE_MS * 1000;

    /* The timed lateral route invalidates the previous line seed. */
    reset_tracking();
    s_last_line_us = now;
    s_reacquire_x = (int)width / 2;
    zero_motor_outputs();
    ESP_LOGI(TAG, "avoidance complete; visual line follow and finish-T enabled");
}

/* Return true while the fixed obstacle route owns the motors. */
static bool obstacle_step(int64_t now, uint16_t width)
{
    obstacle_update_sensor();

    if (s_obstacle_state == OBSTACLE_IDLE) {
        if (s_obstacle_close_samples >= OBSTACLE_CLOSE_CONFIRM_SAMPLES) {
            obstacle_begin(now);
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
            ESP_LOGI(TAG, "avoidance: left shift for %ums", (unsigned)AVOID_LEFT_MS);
        }
        break;
    case OBSTACLE_LEFT:
        obstacle_drive_direct(-AVOID_LEFT_A_SPEED, -AVOID_LEFT_B_SPEED,
                              -AVOID_LEFT_D_SPEED);
        if (elapsed >= AVOID_LEFT_MS) {
            s_obstacle_state = OBSTACLE_FORWARD;
            s_obstacle_phase_start_us = now;
            ESP_LOGI(TAG, "avoidance: forward for %ums", (unsigned)AVOID_FORWARD_MS);
        }
        break;
    case OBSTACLE_FORWARD:
        obstacle_drive_direct(-AVOID_FORWARD_A_SPEED, 0, AVOID_FORWARD_D_SPEED);
        if (elapsed >= AVOID_FORWARD_MS) {
            s_obstacle_state = OBSTACLE_RIGHT;
            s_obstacle_phase_start_us = now;
            ESP_LOGI(TAG, "avoidance: right shift for %ums", (unsigned)AVOID_RIGHT_MS);
        }
        break;
    case OBSTACLE_RIGHT:
        obstacle_drive_direct(AVOID_RIGHT_A_SPEED, AVOID_RIGHT_B_SPEED,
                              AVOID_RIGHT_D_SPEED);
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

/* forward 单独限速率，避免速度档位在门限附近来回跳造成推力抖动。
 * forward_cap 只限制本次实际输出，保留原有误差滤波、yaw PD 和 ramp 状态。 */
static void drive_normal_limited(const line_observation_t *observation,
                                 int64_t now,
                                 int forward_cap)
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
    /* 摄像头在底盘前方时，单靠远端 heading 容易留下一个长期横向偏差。
     * 在现有误差上叠加很小的连续 yaw；小误差保持独立 deadband，避免把
     * 摄像头量化噪声变成左右抖动。 */
    const int center_yaw = abs(lateral_error) <= LINE_CENTER_YAW_DEADBAND ?
                           0 :
                           clamp_int((LINE_CENTER_YAW_GAIN * lateral_error) / 100,
                                     LINE_CENTER_YAW_MAX);
    const int steer_error = heading_error -
                            (LINE_PID_KP_LAT * lateral_for_steer) /
                            (LINE_PID_KH > 0 ? LINE_PID_KH : 1) - center_yaw;
    const int steer_rate = s_heading_error_rate -
                           (LINE_PID_KP_LAT * s_lateral_error_rate) /
                           (LINE_PID_KH > 0 ? LINE_PID_KH : 1);
    s_turn_output = line_control_yaw_pd(&cfg, steer_error, steer_rate,
                                        LINE_PD_KD_HEADING, &s_yaw_accum);
    s_lat_accum = 0;
    int forward = s_forward_output;
    if (forward_cap > 0 && forward > forward_cap) {
        forward = forward_cap;
    }
    drive_normal_output(forward, s_turn_output, 0);
}

static void drive_normal(const line_observation_t *observation, int64_t now)
{
    drive_normal_limited(observation, now, 0);
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

bool camera_line_follow_status_callback(camera_display_status_t *status,
                                        void *user_ctx)
{
    (void)user_ctx;
    if (status == NULL || s_control_mutex == NULL ||
        xSemaphoreTake(s_control_mutex, 0) != pdTRUE) {
        return false;
    }

    status->state = s_post_corner_align ? CAMERA_DISPLAY_STATUS_ALIGN :
                    (s_state == LINE_STATE_CORNER ? CAMERA_DISPLAY_STATUS_CORNER :
                     (s_state == LINE_STATE_LOST ? CAMERA_DISPLAY_STATUS_LOST :
                      CAMERA_DISPLAY_STATUS_NORMAL));
    status->armed = s_armed;
    status->stby = s_stby_enabled;
    status->motor_a = s_command_a;
    status->motor_b = s_command_b;
    status->motor_d = s_command_d;
    status->lateral_error = s_last_lateral_error;
    status->heading_error = s_last_heading_error;
    status->turn_command = s_last_drive_turn;
    status->ultrasonic_cm = s_ultrasonic_valid ?
                            (int)(s_ultrasonic_distance_cm + 0.5f) : -1;

    (void)xSemaphoreGive(s_control_mutex);
    return true;
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
             "vision state=%s armed=%d STBY=%d candidate=%d arm_frames=%u lost=%u "
             "reacq=%u/%u seed_valid=%d threshold=%d seed_x=%d line_w=%d "
             "scan_bottom=%d points=%d valid_rows=%u near_rows=%d confidence=%u "
             "near_line=%d far_error=%d corner=%d@%d pending=%d/%lldms "
             "line_age_ms=%lld",
             state_name(), s_armed, s_stby_enabled, s_last_candidate,
             (unsigned)s_arm_frames, (unsigned)s_lost_frames,
             (unsigned)s_reacquire_frames, (unsigned)LINE_REACQUIRE_CONFIRM_FRAMES,
             s_seed_valid, s_last_threshold, s_last_seed_x, s_line_width,
             s_last_scan_bottom_y, s_last_point_count,
             (unsigned)s_last_valid_rows, s_last_near_normal_rows,
             (unsigned)s_last_confidence, s_last_near_line_visible,
             s_last_far_error, s_last_corner_direction, s_last_corner_row_y,
             pending, (long long)pending_ms, (long long)line_age_ms);
    ESP_LOGI(TAG,
             "control forward[target,ramped]=[%d,%d] drive[in f,t,l]=[%d,%d,%d] "
             "lat[raw,filtered,delta,cmd]=[%d,%d,%d,%d] "
             "head[raw,filtered,delta,cmd]=[%d,%d,%d,%d] "
             "mix_pre[A,B,D]=[%d,%d,%d] mix_post[A,B,D]=[%d,%d,%d] "
             "scaled=%d dropped=%d kick[A,B,D]=[%u,%u,%u] "
             "dir[A,B,D]=[%d,%d,%d] motor[A,B,D]=[%d,%d,%d]",
             s_last_forward_target, s_last_forward_ramped,
             s_last_drive_forward, s_last_drive_turn, s_last_drive_lat,
             s_last_lateral_raw, s_last_lateral_filtered, s_last_lateral_delta,
             s_last_drive_lat, s_last_heading_raw, s_last_heading_filtered,
             s_last_heading_delta, s_last_drive_turn,
             s_mix_pre_a, s_mix_pre_b, s_mix_pre_d,
             s_mix_post_a, s_mix_post_b, s_mix_post_d,
             s_mix_scaled, s_mix_dropped,
             (unsigned)s_kick_cycles[0], (unsigned)s_kick_cycles[1],
             (unsigned)s_kick_cycles[2], s_last_direction[0],
             s_last_direction[1], s_last_direction[2],
             s_command_a, s_command_b, s_command_d);
    ESP_LOGI(TAG, "route avoid=%s phase_ms=%u distance_x10=%d valid=%d done=%d",
             obstacle_state_name(), (unsigned)obstacle_elapsed_ms(now),
             s_ultrasonic_valid ?
                 (int)(s_ultrasonic_distance_cm * 10.0f + 0.5f) : -1,
             s_ultrasonic_valid, s_obstacle_completed);

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
    s_obstacle_reacquire_until_us = 0;
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
    s_last_frame_width = width;
    s_last_frame_height = height;
    if (s_state == LINE_STATE_CORNER && !s_post_corner_align &&
        s_turn_frames < UINT32_MAX) {
        ++s_turn_frames;
    }
    if (s_state == LINE_STATE_LOST && s_lost_frames < UINT32_MAX) {
        ++s_lost_frames;
    }
    const bool detector_candidate = observe_line(rgb565_big_endian, width, height,
                                                 source_threshold, draw_overlay,
                                                 &observation);
    bool candidate = detector_candidate;
    bool partial_candidate = false;
    if (!detector_candidate && s_armed && s_state == LINE_STATE_NORMAL &&
        s_seed_valid && observation.point_count >= LINE_PARTIAL_MIN_POINTS &&
        observation.near_normal_rows >= LINE_PARTIAL_MIN_POINTS &&
        s_last_line_us != 0 &&
        now - s_last_line_us <= (int64_t)LINE_PARTIAL_TRACK_HOLD_MS * 1000) {
        /* Geometry remains strict for arming and corner/finish events. Only
         * normal steering gets this one-frame grace period. */
        observation.candidate = true;
        observation.seed_x = s_seed_x;
        observation.lateral_error = s_last_lateral_error;
        observation.heading_error = s_last_heading_error;
        observation.far_error = s_last_far_error;
        observation.near_line_visible = true;
        candidate = true;
        partial_candidate = true;
    }
    /* Image preview is disabled; avoid copying overlay points on every
     * control frame.  Keep the old path available if a future display client
     * explicitly requests it. */
    if (draw_overlay) {
        save_overlay_snapshot(width, height, &observation, 0);
    }
    s_last_scan_bottom_y = observation.scan_bottom_y;
    s_last_point_count = observation.point_count;
    s_last_near_normal_rows = observation.near_normal_rows;
    s_last_far_error = observation.far_error;
    s_last_corner_direction = observation.corner_direction;
    s_last_corner_row_y = observation.corner_row_y;
    s_last_near_line_visible = observation.near_line_visible;
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

    /* Two consecutive close ultrasonic samples are sufficient to take over. */
    if (obstacle_step(now, width)) {
        goto done;
    }

    /* ---- 终点 T：双侧敞开的横杆 + 下方仍有立柱 ---- */
    if (!observation.finish_candidate) {
        s_finish_frames = 0;
    } else if (LINE_FINISH_ENABLE && s_obstacle_completed && candidate &&
               s_state == LINE_STATE_NORMAL) {
        if (s_finish_frames < LINE_FINISH_CONFIRM_FRAMES) {
            ++s_finish_frames;
        }
        if (s_finish_frames >= LINE_FINISH_CONFIRM_FRAMES) {
            s_finished = true;
            s_state = LINE_STATE_NORMAL;
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
    if (s_state == LINE_STATE_CORNER) {
        if (s_post_corner_align) {
            const bool align_sample = candidate && observation.near_line_visible &&
                                      observation.corner_direction == 0;
            const bool in_band = align_sample &&
                                 abs(observation.lateral_error) <=
                                     LINE_POST_CORNER_ALIGN_LATERAL_ERROR &&
                                 abs(observation.heading_error) <=
                                     LINE_POST_CORNER_ALIGN_HEADING_ERROR;
            if (candidate) {
                s_last_line_us = now;
                update_seed(&observation);
            }
            if (in_band) {
                if (s_post_corner_align_frames < LINE_POST_CORNER_ALIGN_FRAMES) {
                    ++s_post_corner_align_frames;
                }
            } else {
                s_post_corner_align_frames = 0;
            }

            const bool timed_out = s_post_corner_align_started_us != 0 &&
                                   now - s_post_corner_align_started_us >=
                                       (int64_t)LINE_POST_CORNER_ALIGN_TIMEOUT_MS *
                                       1000;
            const bool leave_align =
                s_post_corner_align_frames >= LINE_POST_CORNER_ALIGN_FRAMES ||
                timed_out;
            if (leave_align) {
                if (timed_out && !candidate) {
                    /* 超时仍没有新线时交给现有 LOST 重捕获/停车保护，
                     * 不让回中阶段无限期保持电机输出。 */
                    s_post_corner_align = false;
                    s_post_corner_align_frames = 0;
                    s_post_corner_align_started_us = 0;
                    remember_lost_search_direction();
                    s_state = LINE_STATE_LOST;
                    s_lost_frames = 1;
                    s_reacquire_frames = 0;
                    s_reacquire_x = s_seed_x;
                    s_turn_direction = 0;
                    s_turn_frames = 0;
                    s_turn_exit_frames = 0;
                    s_turn_started_us = 0;
                    s_corner_center_delay_until_us = 0;
                    s_turn_pending_until_us = 0;
                    reset_control();
                    zero_motor_outputs();
                    ESP_LOGW(TAG,
                             "post-corner align timeout without line; "
                             "entering LOST");
                    goto done;
                }
                s_post_corner_align = false;
                s_post_corner_align_frames = 0;
                s_post_corner_align_started_us = 0;
                s_state = LINE_STATE_NORMAL;
                s_turn_direction = 0;
                s_turn_frames = 0;
                s_turn_exit_frames = 0;
                s_turn_started_us = 0;
                s_corner_center_delay_until_us = 0;
                s_turn_pending_until_us = 0;
                s_lost_frames = 0;
                s_reacquire_frames = 0;
                s_last_line_us = now;
                reset_control();
                if (timed_out && !in_band) {
                    ESP_LOGW(TAG,
                             "post-corner align timeout; resuming NORMAL "
                             "at lat=%d head=%d",
                             observation.lateral_error,
                             observation.heading_error);
                } else {
                    ESP_LOGI(TAG,
                             "post-corner align complete; resuming NORMAL "
                             "at lat=%d head=%d",
                             observation.lateral_error,
                             observation.heading_error);
                }
                /* 轮向可能刚从旋转切回前进，首个 NORMAL 输出不再触发
                 * startup kick；正常起步的 kick 逻辑保持不变。 */
                const bool saved_suppress_kick = s_suppress_kick;
                s_suppress_kick = true;
                if (candidate) {
                    drive_normal(&observation, now);
                } else {
                    drive(LINE_FORWARD_CRAWL, 0, 0);
                }
                s_kick_cycles[0] = 0;
                s_kick_cycles[1] = 0;
                s_kick_cycles[2] = 0;
                s_suppress_kick = saved_suppress_kick;
                goto done;
            }

            /* 出弯回中期间只保留低速前进和现有 yaw 校正；没有新点时
             * 沿用上一帧的转向，直到重捕获或超时交给 LOST 保护。 */
            if (candidate) {
                drive_normal_limited(&observation, now, LINE_FORWARD_CRAWL);
            } else {
                drive(LINE_FORWARD_CRAWL, s_turn_output, 0);
            }
            goto done;
        }

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
            /* 原地转向完成只代表新线已被看到；底盘中心仍可能落在线外。
             * 保持 CORNER 状态，先进入短暂的低速回中阶段。 */
            s_post_corner_align = true;
            s_post_corner_align_frames = 0;
            s_post_corner_align_started_us = now;
            s_turn_direction = 0;
            s_turn_frames = 0;
            s_turn_exit_frames = 0;
            s_turn_started_us = 0;
            s_corner_center_delay_until_us = 0;
            s_turn_pending_until_us = 0;
            /* 出弯后一段时间内限速：赛道右上角两个直角弯之间只有 10 cm。 */
            s_alert_until_us = now + (int64_t)LINE_ALERT_MS * 1000;
            s_lost_frames = 0;
            s_reacquire_frames = 0;
            s_last_line_us = now;
            update_seed(&observation);
            reset_control();
            ESP_LOGI(TAG, "turn complete; post-corner align start at ey=%d",
                     observation.lateral_error);
            /* CORNER 的轮向与回中前进不同。只屏蔽这一次切换产生的 kick，
             * 并清掉刚装载的计数，避免下一帧补发。 */
            const bool saved_suppress_kick = s_suppress_kick;
            s_suppress_kick = true;
            drive_normal_limited(&observation, now, LINE_FORWARD_CRAWL);
            s_kick_cycles[0] = 0;
            s_kick_cycles[1] = 0;
            s_kick_cycles[2] = 0;
            s_suppress_kick = saved_suppress_kick;
            goto done;
        }
        if (s_turn_started_us != 0 &&
            now - s_turn_started_us > (int64_t)LINE_TURN_TIMEOUT_MS * 1000) {
            remember_lost_search_direction();
            s_state = LINE_STATE_LOST;
            s_lost_frames = 1;
            s_turn_direction = 0;
            s_turn_started_us = 0;
            s_corner_center_delay_until_us = 0;
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
                s_obstacle_reacquire_until_us = 0;
                reset_control();
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
            } else if (obstacle_grace) {
                drive(LINE_FORWARD_CRAWL, 0, 0);
            } else {
                drive_slow_spin(lost_search_turn_command(lost_us));
            }
        } else {
            s_reacquire_frames = 0;
            s_reacquire_x = s_seed_x;
            if (obstacle_grace) {
                drive(LINE_FORWARD_CRAWL, 0, 0);
            } else if (lost_us < (int64_t)LINE_LOST_STOP_MS * 1000) {
                /* 弯中丢线不能继续盲目前进；用已校准的低速原地扫线，
                 * 直到重捕获或原有停车超时。 */
                drive_slow_spin(lost_search_turn_command(lost_us));
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
    if (s_turn_direction != 0 && s_corner_center_delay_until_us != 0) {
        if (now < s_corner_center_delay_until_us) {
            /* 已确认旧线消失：只前进补偿摄像头到车体中心的距离。 */
            drive_normal_output(LINE_FORWARD_CRAWL, 0, 0);
            goto done;
        }
        s_state = LINE_STATE_CORNER;
        s_turn_frames = 0;
        s_turn_exit_frames = 0;
        s_turn_started_us = now;
        s_turn_pending_until_us = 0;
        s_corner_center_delay_until_us = 0;
        reset_control();
        ESP_LOGI(TAG, "corner %s center delay complete; pivoting",
                 s_turn_direction < 0 ? "left" : "right");
        drive_slow_spin(-s_turn_direction * LINE_PIVOT_TURN);
        goto done;
    }

    if (candidate) {
        s_state = LINE_STATE_NORMAL;
        s_obstacle_reacquire_until_us = 0;
        s_lost_frames = 0;
        s_reacquire_frames = 0;
        if (!partial_candidate) {
            s_last_line_us = now;
            update_seed(&observation);
        }

        /* 近场折角这里只记录 pending；旧线消失后才开始车体中心补偿。 */
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
                s_corner_center_delay_until_us = 0;
            }
            if (s_turn_hint_frames >= LINE_TURN_HINT_FRAMES) {
                s_turn_pending_until_us = now +
                                           (int64_t)LINE_TURN_PENDING_MS * 1000;
            }
        } else if (!turn_pending_active(now)) {
            s_turn_hint_frames = 0;
            s_turn_direction = 0;
            s_turn_pending_until_us = 0;
            s_corner_center_delay_until_us = 0;
        }
        drive_normal(&observation, now);
        goto done;
    }

    if (turn_pending_active(now)) {
        /* pending + 旧线消失才锁定本次转弯，并从此刻开始完整的中心补偿。 */
        s_corner_center_delay_until_us = now +
                                         (int64_t)LINE_CORNER_CENTER_DELAY_MS * 1000;
        s_turn_pending_until_us = 0;
        s_state = LINE_STATE_NORMAL;
        drive_normal_output(LINE_FORWARD_CRAWL, 0, 0);
        goto done;
    }

    remember_lost_search_direction();
    s_state = LINE_STATE_LOST;
    s_lost_frames = 1;
    s_reacquire_frames = 0;
    s_reacquire_x = s_seed_x;
    reset_control();
    {
        const int64_t lost_us = s_last_line_us == 0 ? INT64_MAX : now - s_last_line_us;
        const bool obstacle_grace = s_obstacle_reacquire_until_us != 0 &&
                                    now < s_obstacle_reacquire_until_us;
        if (obstacle_grace) {
            drive(LINE_FORWARD_CRAWL, 0, 0);
        } else if (lost_us < (int64_t)LINE_LOST_STOP_MS * 1000) {
            drive_slow_spin(lost_search_turn_command(lost_us));
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
    normal_burst_deactivate();
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
    s_obstacle_reacquire_until_us = 0;
    s_obstacle_completed = false;
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
    s_obstacle_state = OBSTACLE_IDLE;
    s_obstacle_close_samples = 0;
    s_obstacle_phase_start_us = 0;
    s_obstacle_reacquire_until_us = 0;
    s_obstacle_completed = false;
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
