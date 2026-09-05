#include "line_follow.h"

#include <stdio.h>
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
#include "line_detect.h"
#include "line_mixer.h"
#include "tft_st7735.h"
#include "ultrasonic.h"

/* ===== 硬件：与 car-spin 实车校准一致，不要改 ===== */
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
#define ULTRASONIC_TRIG GPIO_NUM_18
#define ULTRASONIC_ECHO GPIO_NUM_11

#define PWM_MAX 1023U
#define MOTOR_A_SIGN 1
#define MOTOR_B_SIGN 1
#define MOTOR_D_SIGN (-1)
#define MOTOR_MIN_RUN_OUTPUT 11 /* A/D 实测起转值 */
#define MOTOR_B_MIN_RUN_OUTPUT 13 /* B 实测起转值 */
#define MOTOR_PWM_CEILING 44
#define MOTOR_TRIM_A 90
#define MOTOR_TRIM_D 100
#define START_KICK_OUTPUT 27 /* 换向瞬间的破静摩擦冲量 */
#define START_KICK_CYCLES 3U

/* 速度、增益、死区、输出上限都在 line_control.h 里，那些是赛道上要调的量。
 * 控制律本身在 line_control.c，由 test/harness.c 钉住符号。 */

/* 丢线：按最近的转向方向原地旋转找线。弯道处丢线是正常现象（顶点走到
 * 车底下时线确实不在画面里），所以这里不是急停，而是巡线的一部分。 */
#define SEARCH_STOP_MS 5000U
/* 起步：连续看到线才上电，避免摆车时车突然跑掉。 */
#define ARM_CONFIRM_FRAMES 3U
#define START_DELAY_MS 600U
/* 视频流断了就停车。 */
#define FRAME_TIMEOUT_MS 1200U

/* ===== 超声波避障（硬编码，不看摄像头） ===== */
#define OBSTACLE_DETECT_CM 10.0f
#define OBSTACLE_CONFIRM_SAMPLES 2U
#define ULTRASONIC_PERIOD_MS 70U
#define AVOID_BRAKE_MS 400U
#define AVOID_LEFT_MS 1600U
#define AVOID_FORWARD_MS 1800U
#define AVOID_RIGHT_MS 1600U
#define AVOID_STRAFE_OUTPUT 16
#define AVOID_FORWARD_OUTPUT 20
/* 避障走完后先直行重捕获这么久，不让刚回到线上的第一帧触发丢线旋转。 */
#define AVOID_REACQUIRE_MS 900U

typedef enum {
    ST_WAIT = 0,   /* 还没确认黑线，电机断电 */
    ST_FOLLOW,     /* 正常巡线 */
    ST_SEARCH,     /* 丢线，原地旋转找线 */
    ST_AVOID_BRAKE,
    ST_AVOID_LEFT,
    ST_AVOID_FORWARD,
    ST_AVOID_RIGHT,
    ST_STOPPED,    /* 长时间找不到线，停车 */
} follow_state_t;

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t channel;
    int sign;
} motor_t;

static const char *TAG = "line_follow";
static const motor_t motor_a = {A_IN1, A_IN2, LEDC_CHANNEL_0, MOTOR_A_SIGN};
static const motor_t motor_b = {B_IN1, B_IN2, LEDC_CHANNEL_1, MOTOR_B_SIGN};
static const motor_t motor_d = {D_IN1, D_IN2, LEDC_CHANNEL_2, MOTOR_D_SIGN};

static bool s_started;
static bool s_armed;
static bool s_stby_enabled;
static follow_state_t s_state;
static uint8_t s_arm_frames;
static int64_t s_first_frame_us;
static int64_t s_last_frame_us;
static int64_t s_lost_since_us;
static int64_t s_phase_start_us;
static int64_t s_reacquire_until_us;
static bool s_avoid_done;
/* 负 = 右转。起点直行 50cm 后第一个弯是向右 120°，所以默认往右找。 */
static int s_search_turn = -LINE_TURN_MAX;

static int s_command_a;
static int s_command_b;
static int s_command_d;
static int s_last_direction[3];
static uint8_t s_kick_cycles[3];
static bool s_suppress_kick;

static line_obs_t s_obs;
static int s_out_forward;
static int s_out_turn;
static int s_out_lat;
static uint8_t s_threshold;

static volatile float s_distance_cm = -1.0f;
static volatile uint32_t s_close_samples;
static bool s_tasks_created;
static SemaphoreHandle_t s_lock;

/* ===== 电机 ===== */

static int clamp_int(int value, int limit)
{
    if (value > limit) {
        return limit;
    }
    return value < -limit ? -limit : value;
}

static void set_duty(const motor_t *motor, int output)
{
    if (output < 0) {
        output = 0;
    }
    if (output > MOTOR_PWM_CEILING) {
        output = MOTOR_PWM_CEILING;
    }
    (void)ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel,
                        PWM_MAX * (uint32_t)output / 100U);
    (void)ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

static void motor_set(const motor_t *motor, int command)
{
    const int index = (int)motor->channel;
    const int speed = clamp_int(command * motor->sign, MOTOR_PWM_CEILING);
    const int direction = (speed > 0) - (speed < 0);

    if (direction != s_last_direction[index]) {
        /* 换向前先断 duty，不让 TB6612 硬切方向。 */
        set_duty(motor, 0);
        gpio_set_level(motor->in1, speed > 0);
        gpio_set_level(motor->in2, speed < 0);
        s_last_direction[index] = direction;
        s_kick_cycles[index] = direction == 0 ? 0 : START_KICK_CYCLES;
    }

    if (direction == 0) {
        s_kick_cycles[index] = 0;
        gpio_set_level(motor->in1, 0);
        gpio_set_level(motor->in2, 0);
        set_duty(motor, 0);
        return;
    }

    int output = abs(speed);
    if (s_kick_cycles[index] > 0 && !s_suppress_kick) {
        if (output < START_KICK_OUTPUT) {
            output = START_KICK_OUTPUT;
        }
        --s_kick_cycles[index];
    }
    set_duty(motor, output);
}

/* (forward, turn, lat) -> 三轮。混控层负责整向量缩放和起转值筛选。 */
static void drive(int forward, int turn, int lat)
{
    const line_mixer_cfg_t cfg = {
        MOTOR_PWM_CEILING, MOTOR_MIN_RUN_OUTPUT, MOTOR_B_MIN_RUN_OUTPUT,
        MOTOR_TRIM_A, MOTOR_TRIM_D,
    };
    line_mixer_out_t out = {0, 0, 0, false, false};
    line_mixer_solve(forward, turn, lat, &cfg, &out);
    s_out_forward = forward;
    s_out_turn = turn;
    s_out_lat = lat;
    s_command_a = out.a;
    s_command_b = out.b;
    s_command_d = out.d;
    motor_set(&motor_a, out.a);
    motor_set(&motor_b, out.b);
    motor_set(&motor_d, out.d);
}

static void motors_off(void)
{
    drive(0, 0, 0);
}

static void enable_stby(void)
{
    if (!s_stby_enabled) {
        gpio_set_level(STBY_GPIO, 1);
        s_stby_enabled = true;
    }
}

static void disarm(const char *reason)
{
    motors_off();
    gpio_set_level(STBY_GPIO, 0);
    s_stby_enabled = false;
    s_armed = false;
    s_arm_frames = 0;
    s_state = ST_WAIT;
    line_detect_reset();
    ESP_LOGW(TAG, "disarmed: %s", reason);
}

/* ===== 超声波 ===== */

static void ultrasonic_task(void *arg)
{
    (void)arg;
    while (true) {
        const float cm = ultrasonic_read_cm();
        s_distance_cm = cm;
        /* 只有上电巡线之后才累计。摆车时手从传感器前面过不能启动避障。 */
        const bool close = s_armed && cm > 1.0f && cm < OBSTACLE_DETECT_CM;
        if (close) {
            ++s_close_samples;
        } else {
            s_close_samples = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
    }
}

/* ===== 避障路线：刹车 -> 左移 -> 直行 -> 右移，全程不看摄像头 ===== */

static void avoid_advance(int64_t now)
{
    const int64_t elapsed_ms = (now - s_phase_start_us) / 1000;
    switch (s_state) {
    case ST_AVOID_BRAKE:
        motors_off();
        if (elapsed_ms >= (int64_t)AVOID_BRAKE_MS) {
            s_state = ST_AVOID_LEFT;
            s_phase_start_us = now;
            ESP_LOGI(TAG, "avoid: strafe left");
        }
        break;
    case ST_AVOID_LEFT:
        drive(0, 0, -AVOID_STRAFE_OUTPUT);
        if (elapsed_ms >= (int64_t)AVOID_LEFT_MS) {
            s_state = ST_AVOID_FORWARD;
            s_phase_start_us = now;
            ESP_LOGI(TAG, "avoid: forward");
        }
        break;
    case ST_AVOID_FORWARD:
        drive(AVOID_FORWARD_OUTPUT, 0, 0);
        if (elapsed_ms >= (int64_t)AVOID_FORWARD_MS) {
            s_state = ST_AVOID_RIGHT;
            s_phase_start_us = now;
            ESP_LOGI(TAG, "avoid: strafe right");
        }
        break;
    case ST_AVOID_RIGHT:
        drive(0, 0, AVOID_STRAFE_OUTPUT);
        if (elapsed_ms >= (int64_t)AVOID_RIGHT_MS) {
            motors_off();
            s_avoid_done = true;
            s_close_samples = 0;
            s_state = ST_FOLLOW;
            s_lost_since_us = 0;
            s_reacquire_until_us = now + (int64_t)AVOID_REACQUIRE_MS * 1000;
            line_detect_reset();
            ESP_LOGI(TAG, "avoid: done, back to line following");
        }
        break;
    default:
        break;
    }
}

/* ===== 巡线控制 ===== */

/*
 * 转向和平移分工：
 *   heading（线在画面里的倾斜）-> turn，负责把车头拧到线的方向上
 *   lateral（线相对画面中心的偏移）-> lat，负责把车横着挪到线上
 * 全向底盘才能这么分。转弯时不给平移，否则两者互相抵消。
 * 弯道不需要专门识别：左转弯时远处那段横线把 far_x 拉到左边，heading 自然
 * 变成大负数，turn 就是左转。
 */
static void follow_line(const line_obs_t *obs)
{
    line_cmd_t cmd;
    line_control_follow(obs->lateral, obs->heading, &cmd);
    drive(cmd.forward, cmd.turn, cmd.lat);

    /* 记住往哪边转，丢线时按这个方向找。 */
    const int search = line_control_search_turn(obs->lateral, obs->heading);
    if (search != 0) {
        s_search_turn = search;
    }
}

static void process_frame(uint8_t *frame, uint16_t width, uint16_t height,
                          uint8_t threshold)
{
    const int64_t now = esp_timer_get_time();
    s_last_frame_us = now;
    s_threshold = threshold;
    if (s_first_frame_us == 0) {
        s_first_frame_us = now;
    }

    const bool found = line_detect_run(frame, width, height, threshold, &s_obs);

    /* 避障路线一旦开始就走完，不看摄像头。 */
    if (s_state >= ST_AVOID_BRAKE && s_state <= ST_AVOID_RIGHT) {
        avoid_advance(now);
        return;
    }

    if (s_state == ST_STOPPED) {
        motors_off();
        return;
    }

    /* 上电确认：连续几帧看到线，且过了上电延时，才允许电机动。 */
    if (!s_armed) {
        motors_off();
        if (!found) {
            s_arm_frames = 0;
            return;
        }
        if (now - s_first_frame_us < (int64_t)START_DELAY_MS * 1000) {
            return;
        }
        if (++s_arm_frames < ARM_CONFIRM_FRAMES) {
            return;
        }
        s_armed = true;
        s_state = ST_FOLLOW;
        s_lost_since_us = 0;
        enable_stby();
        ESP_LOGI(TAG, "armed: line confirmed, motors live");
    }

    /* 障碍物：连续两次小于 10cm 就立刻停车进硬编码路线。 */
    if (!s_avoid_done && s_close_samples >= OBSTACLE_CONFIRM_SAMPLES) {
        s_state = ST_AVOID_BRAKE;
        s_phase_start_us = now;
        ESP_LOGI(TAG, "obstacle at %.1f cm: brake then fixed route",
                 (double)s_distance_cm);
        motors_off();
        return;
    }

    if (found) {
        s_lost_since_us = 0;
        s_state = ST_FOLLOW;
        follow_line(&s_obs);
        return;
    }

    /* 丢线。避障刚结束的一小段时间内直行重捕获，不旋转。 */
    if (s_lost_since_us == 0) {
        s_lost_since_us = now;
    }
    if (now < s_reacquire_until_us) {
        drive(LINE_FORWARD_CORNER, 0, 0);
        return;
    }
    if ((now - s_lost_since_us) / 1000 >= (int64_t)SEARCH_STOP_MS) {
        s_state = ST_STOPPED;
        disarm("line lost too long");
        return;
    }
    s_state = ST_SEARCH;
    drive(0, s_search_turn, 0);
}

/* ===== TFT 叠加层：把检测器看到的东西画在画面上 ===== */

static const char *state_name(void)
{
    switch (s_state) {
    case ST_FOLLOW: return "FOLLOW";
    case ST_SEARCH: return "SEARCH";
    case ST_AVOID_BRAKE: return "BRAKE";
    case ST_AVOID_LEFT: return "AVOID-L";
    case ST_AVOID_FORWARD: return "AVOID-F";
    case ST_AVOID_RIGHT: return "AVOID-R";
    case ST_STOPPED: return "STOPPED";
    case ST_WAIT:
    default: return "WAIT";
    }
}

static void put_pixel(uint8_t *frame, int width, int height, int x, int y,
                      uint16_t colour)
{
    if (x < 0 || y < 0 || x >= width || y >= height) {
        return;
    }
    uint8_t *pixel = frame + ((size_t)y * (size_t)width + (size_t)x) * 2U;
    pixel[0] = (uint8_t)(colour >> 8);
    pixel[1] = (uint8_t)(colour & 0xff);
}

static void draw_column(uint8_t *frame, int width, int height, int x,
                        int top, int bottom, uint16_t colour)
{
    for (int y = top; y <= bottom; ++y) {
        put_pixel(frame, width, height, x - 1, y, colour);
        put_pixel(frame, width, height, x, y, colour);
        put_pixel(frame, width, height, x + 1, y, colour);
    }
}

static void draw_row(uint8_t *frame, int width, int height, int y,
                     int lo, int hi, uint16_t colour)
{
    for (int x = lo; x <= hi; ++x) {
        put_pixel(frame, width, height, x, y, colour);
    }
}

void line_follow_preview_callback(uint8_t *frame, uint16_t width,
                                  uint16_t height, uint8_t threshold,
                                  uint32_t sequence, int64_t capture_us,
                                  void *user_ctx)
{
    (void)threshold;
    (void)sequence;
    (void)capture_us;
    (void)user_ctx;
    if (frame == NULL || width == 0 || height == 0) {
        return;
    }
    const int w = (int)width;
    const int h = (int)height;
    const uint16_t cyan = 0x07ff;
    const uint16_t green = 0x07e0;
    const uint16_t red = 0xf800;
    const uint16_t yellow = 0xffe0;

    const int near_top = h * LINE_NEAR_TOP_PERCENT / 100;
    const int near_bottom = h * LINE_NEAR_BOTTOM_PERCENT / 100 - 1;
    const int far_top = h * LINE_FAR_TOP_PERCENT / 100;
    const int far_bottom = h * LINE_FAR_BOTTOM_PERCENT / 100 - 1;

    /* 条带边界（青）：绿柱应该落在下面那条带里，红柱在上面那条。 */
    draw_row(frame, w, h, near_top, 0, w - 1, cyan);
    draw_row(frame, w, h, near_bottom, 0, w - 1, cyan);
    draw_row(frame, w, h, far_top, 0, w - 1, cyan);
    draw_row(frame, w, h, far_bottom, 0, w - 1, cyan);
    /* 搜索窗口（黄）：窗口外的黑色一律不看。 */
    draw_column(frame, w, h, s_obs.window_lo, far_top, near_bottom, yellow);
    draw_column(frame, w, h, s_obs.window_hi, far_top, near_bottom, yellow);

    if (s_obs.found) {
        /* 绿 = 近场线心（决定平移），红 = 远场线心（决定转向）。 */
        draw_column(frame, w, h, s_obs.near_x, near_top, near_bottom, green);
        if (s_obs.far_valid) {
            draw_column(frame, w, h, s_obs.far_x, far_top, far_bottom, red);
        }
    }

    char line0[24];
    char line1[24];
    char line2[24];
    snprintf(line0, sizeof(line0), "%s %s", state_name(),
             s_armed ? "ARM" : "off");
    snprintf(line1, sizeof(line1), "lat%+d hd%+d t%u", s_obs.lateral,
             s_obs.heading, (unsigned)s_threshold);
    snprintf(line2, sizeof(line2), "f%d t%+d l%+d %.0fcm", s_out_forward,
             s_out_turn, s_out_lat, (double)s_distance_cm);
    const char *const lines[] = {line0, line1, line2};
    (void)tft_st7735_overlay_text_rgb565(frame, width, height, lines, 3, 0x0000);
}

void line_follow_frame_callback(uint8_t *frame, uint16_t width,
                                uint16_t height, uint8_t threshold,
                                bool draw_overlay, void *user_ctx)
{
    (void)draw_overlay;
    (void)user_ctx;
    if (!s_started || frame == NULL) {
        return;
    }
    if (s_lock != NULL && xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }
    process_frame(frame, width, height, threshold);
    if (s_lock != NULL) {
        (void)xSemaphoreGive(s_lock);
    }
}

/* ===== 看门狗 + 低频日志 ===== */

static void watchdog_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(300));
        if (s_lock == NULL || xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
            continue;
        }
        const int64_t now = esp_timer_get_time();
        /* 视频流断了：电机必须停，否则车会带着最后一条指令跑掉。 */
        if (s_armed && s_last_frame_us != 0 &&
            (now - s_last_frame_us) / 1000 >= (int64_t)FRAME_TIMEOUT_MS) {
            disarm("no camera frame");
        }
        uint8_t dark = 0;
        uint8_t light = 0;
        camera_display_get_levels(&dark, &light);
        ESP_LOGI(TAG,
                 "%s armed=%d lat=%+d hd=%+d fill=%d/%d thr=%u lum=%u/%u "
                 "f=%d t=%+d l=%+d abd=%d/%d/%d dist=%.1f",
                 state_name(), s_armed ? 1 : 0, s_obs.lateral, s_obs.heading,
                 s_obs.near_fill, s_obs.far_fill, (unsigned)s_threshold,
                 (unsigned)dark, (unsigned)light,
                 s_out_forward, s_out_turn, s_out_lat,
                 s_command_a, s_command_b, s_command_d,
                 (double)s_distance_cm);
        (void)xSemaphoreGive(s_lock);
    }
}

/* ===== 启动 / 停止 ===== */

esp_err_t line_follow_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    const gpio_num_t pins[] = {A_IN1, A_IN2, B_IN1, B_IN2, D_IN1, D_IN2, STBY_GPIO};
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        ESP_ERROR_CHECK(gpio_reset_pin(pins[i]));
        ESP_ERROR_CHECK(gpio_set_direction(pins[i], GPIO_MODE_OUTPUT));
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
    const gpio_num_t pwm_pins[] = {A_PWM, B_PWM, D_PWM};
    for (size_t i = 0; i < 3; ++i) {
        const ledc_channel_config_t channel = {
            .gpio_num = pwm_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i]->channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        err = ledc_channel_config(&channel);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LEDC channel %u init failed: %s", (unsigned)i,
                     esp_err_to_name(err));
            return err;
        }
    }

    ultrasonic_init(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
    line_detect_reset();
    s_started = true;
    s_armed = false;
    s_stby_enabled = false;
    s_state = ST_WAIT;
    s_arm_frames = 0;
    s_first_frame_us = 0;
    s_last_frame_us = 0;
    s_lost_since_us = 0;
    s_phase_start_us = 0;
    s_reacquire_until_us = 0;
    s_avoid_done = false;
    s_search_turn = -LINE_TURN_MAX;
    s_close_samples = 0;
    s_distance_cm = -1.0f;
    s_last_direction[0] = 0;
    s_last_direction[1] = 0;
    s_last_direction[2] = 0;
    s_kick_cycles[0] = 0;
    s_kick_cycles[1] = 0;
    s_kick_cycles[2] = 0;
    s_obs = (line_obs_t){0};
    motors_off();
    gpio_set_level(STBY_GPIO, 0);

    if (!s_tasks_created) {
        if (xTaskCreate(ultrasonic_task, "ultrasonic", 3072, NULL, 5, NULL) != pdPASS ||
            xTaskCreate(watchdog_task, "line_wd", 3072, NULL, 2, NULL) != pdPASS) {
            s_started = false;
            ESP_LOGE(TAG, "could not create tasks");
            return ESP_ERR_NO_MEM;
        }
        s_tasks_created = true;
    }
    ESP_LOGI(TAG, "ready: motors stay off until a line is confirmed");
    return ESP_OK;
}

void line_follow_stop(void)
{
    if (!s_started) {
        return;
    }
    if (s_lock != NULL) {
        (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    motors_off();
    gpio_set_level(STBY_GPIO, 0);
    s_stby_enabled = false;
    s_armed = false;
    s_started = false;
    if (s_lock != NULL) {
        (void)xSemaphoreGive(s_lock);
    }
}
