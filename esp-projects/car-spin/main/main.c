#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 四路红外，按车体从左到右排列：OUT4、OUT3、OUT2、OUT1。
   若左右方向装反，把这一行的四个引脚顺序颠倒即可。 */
static const gpio_num_t sensors[4] = {
    GPIO_NUM_1,  /* OUT4，车体最左 */
    GPIO_NUM_2,  /* OUT3 */
    GPIO_NUM_42, /* OUT2 */
    GPIO_NUM_41  /* OUT1，车体最右 */
};
static const int weights[4] = {-3, -1, 1, 3};
#define IR_ACTIVE_LEVEL 0 /* LQ_R4CHVB：黑线低电平 */

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

#define STRAIGHT_SPEED 18 /* 零误差时的前进幅值 */
#define CURVE_SPEED 16    /* 有误差时的前进幅值 */
#define TURN_KP 3         /* turn = -error * TURN_KP / 10 */
#define TURN_MAX 12       /* 普通比例转向上限 */
#define RECOVERY_TURN 18  /* 锐角锁向和持续丢线搜索幅值 */
#define TURN_SLEW_STEP 3  /* 每 10 ms 最多改变的转向量 */
#define MAX_OUTPUT 28     /* 任一电机的输出上限 */

#define LOST_CONFIRM_CYCLES 3U
#define CORNER_CONFIRM_CYCLES 3U
#define CORNER_ADVANCE_CYCLES 6U
#define CORNER_MIN_TURN_CYCLES 8U
#define CENTER_CONFIRM_CYCLES 3U
#define FINISH_MARK_CYCLES 5U
#define FINISH_BLANK_CYCLES 12U
#define FINISH_WINDOW_CYCLES 30U

#define CENTER_MASK 0x06U      /* ACTIVE=0110 */
#define LEFT_CORNER_MASK 0x07U /* ACTIVE=1110 */
#define RIGHT_CORNER_MASK 0x0EU /* ACTIVE=0111 */
#define FULL_LINE_MASK 0x0FU   /* ACTIVE=1111 */

#define LOOP_MS 10U
#define LOG_MS 100U
#define START_DELAY_MS 2000U
#define PWM_MAX 1023U

typedef struct {
    gpio_num_t pwm, in1, in2;
    ledc_channel_t channel;
    int last_direction;
} motor_t;

typedef enum {
    FOLLOW_LINE,
    CORNER_ADVANCE,
    CORNER_TURN,
    STOPPED
} control_state_t;

typedef struct {
    control_state_t state;
    int turn;
    int direction;
    uint8_t state_cycles;
    uint8_t finish_cycles;
    uint8_t last_mask;
    uint8_t same_mask_cycles;
    bool line_seen;
    bool finish_armed;
} controller_t;

static const char *const state_names[] = {
    "FOLLOW", "ADVANCE", "CORNER", "STOPPED"
};

/* 顺序固定为 A、B、D，与 drive() 的混控一致。
   IN1/IN2 宏始终按驱动板接线表，电机极性只在 drive() 中校准。 */
static motor_t motors[3] = {
    {A_PWM, A_IN1, A_IN2, LEDC_CHANNEL_0, 0}, /* 右前 */
    {B_PWM, B_IN1, B_IN2, LEDC_CHANNEL_1, 0}, /* 后轮 */
    {D_PWM, D_IN1, D_IN2, LEDC_CHANNEL_2, 0}, /* 左前 */
};

static const char *TAG = "line_follow";

static int clamp(int value, int limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static void motor_set(motor_t *motor, int speed)
{
    speed = clamp(speed, MAX_OUTPUT);
    int direction = (speed > 0) - (speed < 0);
    if (direction != motor->last_direction) {
        /* TB6612 换向前先撤掉 PWM，避免带载时直接翻转方向。 */
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
        gpio_set_level(motor->in1, speed > 0);
        gpio_set_level(motor->in2, speed < 0);
        motor->last_direction = direction;
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, PWM_MAX * (uint32_t)abs(speed) / 100U);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel);
}

/* 按实车电机极性混控；B 只提供旋转分量；正 turn 为左转。 */
static void drive(int forward, int turn, int out[3])
{
    out[0] = clamp(-forward - turn, MAX_OUTPUT);
    out[1] = clamp(turn, MAX_OUTPUT);
    out[2] = clamp(-forward + turn, MAX_OUTPUT);
    for (int i = 0; i < 3; ++i) motor_set(&motors[i], out[i]);
}

static uint8_t read_sensors(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < 4; ++i)
        if (gpio_get_level(sensors[i]) == IR_ACTIVE_LEVEL) mask |= 1U << i;
    return mask;
}

/* 负值表示线在左侧，正值表示线在右侧，范围约 ±30。 */
static int line_error(uint8_t mask)
{
    int sum = 0, count = 0;
    for (int i = 0; i < 4; ++i)
        if (mask & (1U << i)) { sum += weights[i]; ++count; }
    return count ? sum * 10 / count : 0;
}

static int move_towards(int value, int target, int step)
{
    if (value < target) return value + clamp(target - value, step);
    if (value > target) return value - clamp(value - target, step);
    return value;
}

static void enter_state(controller_t *control, control_state_t state)
{
    control->state = state;
    control->state_cycles = 0;
}

static void observe_mask(controller_t *control, uint8_t mask)
{
    if (mask != control->last_mask) {
        control->last_mask = mask;
        control->same_mask_cycles = 1;
    } else if (control->same_mask_cycles < UINT8_MAX) {
        ++control->same_mask_cycles;
    }
}

static void follow_targets(controller_t *control, uint8_t mask,
                           int *forward, int *target_turn)
{
    int error = line_error(mask);
    if (mask == 0) {
        if (!control->line_seen) {
            *forward = 0;
            *target_turn = 0;
        } else if (control->same_mask_cycles < LOST_CONFIRM_CYCLES) {
            /* 短暂漏读保持当前动作，不因一个采样点突然刹停或反转。 */
            *forward = CURVE_SPEED;
            *target_turn = control->turn;
        } else {
            *forward = 0;
            *target_turn = control->direction;
        }
        return;
    }

    control->line_seen = true;
    *forward = error ? CURVE_SPEED : STRAIGHT_SPEED;
    *target_turn = clamp(-error * TURN_KP / 10, TURN_MAX);
    if (error) control->direction = error < 0 ? RECOVERY_TURN : -RECOVERY_TURN;
}

static void control_step(controller_t *control, uint8_t mask,
                         int *forward, int *turn)
{
    int target_turn = 0;
    if (control->state == STOPPED) {
        control->turn = 0;
        *forward = 0;
        *turn = 0;
        return;
    }

    observe_mask(control, mask);
    if (mask == FULL_LINE_MASK &&
        control->same_mask_cycles >= FINISH_MARK_CYCLES) {
        control->finish_armed = true;
        control->finish_cycles = 0;
        enter_state(control, FOLLOW_LINE);
    } else if (control->finish_armed && control->finish_cycles < UINT8_MAX) {
        ++control->finish_cycles;
    }

    if (control->finish_armed && mask == 0) {
        /* 只有先见到 T 型横杠，持续全白才是终点。 */
        if (control->same_mask_cycles >= FINISH_BLANK_CYCLES) {
            enter_state(control, STOPPED);
            control->turn = 0;
            *forward = 0;
            *turn = 0;
            return;
        }
        if (control->finish_cycles < FINISH_WINDOW_CYCLES) {
            *forward = STRAIGHT_SPEED;
            control->turn = move_towards(control->turn, 0, TURN_SLEW_STEP);
            *turn = control->turn;
            return;
        }
    }
    if (control->finish_armed && control->finish_cycles >= FINISH_WINDOW_CYCLES)
        control->finish_armed = false;

    switch (control->state) {
    case FOLLOW_LINE:
        follow_targets(control, mask, forward, &target_turn);
        if (!control->finish_armed &&
            (mask == LEFT_CORNER_MASK || mask == RIGHT_CORNER_MASK) &&
            control->same_mask_cycles >= CORNER_CONFIRM_CYCLES) {
            control->direction = mask == LEFT_CORNER_MASK
                               ? RECOVERY_TURN : -RECOVERY_TURN;
            enter_state(control, CORNER_ADVANCE);
            *forward = STRAIGHT_SPEED;
            target_turn = 0;
        }
        break;

    case CORNER_ADVANCE:
        *forward = STRAIGHT_SPEED;
        target_turn = 0;
        if (mask != FULL_LINE_MASK &&
            ++control->state_cycles >= CORNER_ADVANCE_CYCLES) {
            enter_state(control, CORNER_TURN);
        }
        break;

    case CORNER_TURN:
        /* 保持前进基量，用锁存转向量形成平滑锐角弧线。 */
        *forward = STRAIGHT_SPEED;
        if (mask == FULL_LINE_MASK) {
            target_turn = 0;
        } else {
            target_turn = control->direction;
            if (control->state_cycles < UINT8_MAX) ++control->state_cycles;
            if (control->state_cycles >= CORNER_MIN_TURN_CYCLES &&
                mask == CENTER_MASK &&
                control->same_mask_cycles >= CENTER_CONFIRM_CYCLES) {
                enter_state(control, FOLLOW_LINE);
                follow_targets(control, mask, forward, &target_turn);
            }
        }
        break;

    case STOPPED:
        break; /* 已在函数开头处理。 */
    }

    control->turn = move_towards(control->turn, target_turn, TURN_SLEW_STEP);
    *turn = control->turn;
}

static void hardware_init(void)
{
    for (int i = 0; i < 4; ++i) {
        gpio_reset_pin(sensors[i]);
        gpio_set_direction(sensors[i], GPIO_MODE_INPUT);
    }
    gpio_reset_pin(STBY_GPIO);
    gpio_set_direction(STBY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(STBY_GPIO, 0);

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 10000, .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < 3; ++i) {
        gpio_reset_pin(motors[i].in1);
        gpio_set_direction(motors[i].in1, GPIO_MODE_OUTPUT);
        gpio_set_level(motors[i].in1, 0);
        gpio_reset_pin(motors[i].in2);
        gpio_set_direction(motors[i].in2, GPIO_MODE_OUTPUT);
        gpio_set_level(motors[i].in2, 0);
        motors[i].last_direction = 0;
        const ledc_channel_config_t channel = {
            .gpio_num = motors[i].pwm, .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motors[i].channel, .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0, .duty = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
    gpio_set_level(STBY_GPIO, 1);
}

void app_main(void)
{
    hardware_init();
    int out[3] = {0};
    controller_t control = {
        .state = FOLLOW_LINE,
        .direction = RECOVERY_TURN /* 无历史偏差时默认向左搜索 */
    };
    uint32_t log_elapsed = LOG_MS;

    drive(0, 0, out);
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));

    while (true) {
        uint8_t mask = read_sensors();
        int error = line_error(mask);
        int forward = 0, turn = 0;
        control_step(&control, mask, &forward, &turn);
        drive(forward, turn, out);

        if (log_elapsed >= LOG_MS) {
            ESP_LOGI(TAG, "ACTIVE=%u%u%u%u state=%s finish=%u err=%d turn=%d motor[A,B,D]=[%d,%d,%d]",
                     (unsigned)(mask & 1U), (unsigned)((mask >> 1) & 1U),
                     (unsigned)((mask >> 2) & 1U), (unsigned)((mask >> 3) & 1U),
                     state_names[control.state], (unsigned)control.finish_armed,
                     error, turn, out[0], out[1], out[2]);
            log_elapsed = 0;
        }
        log_elapsed += LOOP_MS;
        /* 不依赖额外的 FreeRTOS DelayUntil 配置；控制周期约为 10 ms。 */
        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
