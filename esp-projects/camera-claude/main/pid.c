#include "pid.h"
#include "motor.h"
#include "track.h"
#include "encoder.h"
#include "camera_track.h"
#include "track_config.h"
#include <math.h>
#include <stddef.h>

// ============ 基础速度 ============
#define PID_CRUISE_SPEED 1600       // 直行巡航速度（乘以 FRONT_FORWARD_RATIO 才是实际输出）
#define PID_MIN_FORWARD_SPEED 100   // 误差最大时的最低前进速度下限
#define PID_SLOWDOWN_PER_ERROR 0    // 每 1.0 误差降多少速度（0 = 不关，弯道保持全速）
/* 原值 8191（13 位满量程 = 100% 占空比）。本车 car-spin 实车标定的安全上限是
 * 44% 占空比，再高会打滑失控，所以这里压到 44% × 8191 = 3604。
 * 他们现有的调参值最大是 STRAFE_POWER 2200（27%），全都在这条线以下，
 * 所以这个限幅不改变当前任何动作，只是防止把参数调大后车冲出去。 */
#define PID_MAX_SPEED 3604          // 电机功率上限（本车标定 44% 占空比）
#define PID_HW_FULL_SCALE 8191      // 13 位 PWM 满量程，仅作参考

/* 本车实测起转值：A/D 约 11% 占空比（901 counts）、B 约 13%（1065 counts）。
 * 低于这个值电机只嗡不转。原代码没有起转值保护（SPEED_MIN_POWER 100 只有
 * 1.2%，远低于实际起转），急弯时内侧轮可能算出 785 counts（9.6%）而不转，
 * 车会变成原地转而不是画弧。出现这个症状调 FRONT_LATERAL_RATIO 或
 * PID_CRUISE_SPEED，让内侧轮结果保持在 901 以上。 */
#define MOTOR_AD_STICTION_COUNTS 901
#define MOTOR_B_STICTION_COUNTS 1065

// ============ 差速换算 ============
#define FRONT_FORWARD_RATIO 866     // 前进投影系数（斜向轮布局，cos30°≈0.866）
#define FRONT_LATERAL_RATIO 400     // 差速放大倍数（correction × R/1000 = 两轮差速）

// ============ PID 三项 ============
#define PID_KP 300                  // 比例系数：error → correction 的增益（大=灵敏，容易飘）
#define PID_KI 0                     // 积分系数：消除稳态误差（0 = 不用，因为循迹要快反应）
#define PID_KD 500                   // 微分系数：抑制过冲振荡（大=稳，但对噪声敏感）
#define PID_INTEGRAL_LIMIT 100       // 积分项累计限幅
#define PID_CORRECTION_LIMIT 1500    // correction 总输出限幅
#define PID_DERIVATIVE_LIMIT 4       // 微分项限幅（限制 derivative = Δerror/Δt）
#define PID_CORRECTION_RATE_MAX 250  // correction 每 PID 周期最大变化量（小=变化慢，稳）

// ============ 丢线原地旋转 ============
#define PID_LOST_TURN_SPEED 1400     // 丢线原地旋转的电机功率（A 反转 + D 正转）

// ============ 电机速度闭环 ============
#define SPEED_LOOP_DEADZONE 5        // 电机转速死区（低于此误差不修正，防抖动）
#define SPEED_LOOP_MAX_CORRECTION 500  // 速度闭环单次最大修正量
#define SPEED_MIN_POWER 100          // 电机最小启动功率（低于此值不做闭环）
#define TARGET_SPEED_SMOOTH_ALPHA 0.8f  // 目标速度低通滤波（0.8=变化快，0.5=更平滑）

/* 功率 1000 对应每 10ms 多少个编码器计数。取决于编码器线数 × 减速比 ×
 * PCNT 倍频方式，换电机就必须重测 —— 下面 12.0 是他们车上的值。
 *
 * 用 TEST_MODE 7 测（main.c 末尾），落地带负载测，不能垫起来空转。
 * 错在哪个方向都会坏事：
 *   偏大 → 目标转速永远追不上，闭环每周期补 +500 顶到 PID_MAX_SPEED，
 *          车以 44% 占空比冲出去，而不是巡航的 22%
 *   偏小 → 目标低于实际，speed_diff 变负，闭环反过来减功率，车比设定的还慢 */
#define POWER_TO_SPEED_A 12.0f       // A 轮：待实测
#define POWER_TO_SPEED_B 12.0f       // B 轮：待实测（辅助轮）
#define POWER_TO_SPEED_D 12.0f       // D 轮：待实测

#define SPEED_KP_SMALL 0.3f          // 电机小偏差增益（< 8 计数/周期）
#define SPEED_KP_MEDIUM 0.6f         // 电机中偏差增益（8 ~ 20）
#define SPEED_KP_LARGE 1.2f          // 电机大偏差增益（> 20）

static float integral;
static float previous_error;
static float previous_correction;
static int last_motor_A_power;
static int last_motor_B_power;
static int last_motor_D_power;
static float last_turn_error;
static float turn_direction_score;
static float frozen_direction;
static int recent_error_signs[6];
static int sign_idx;

static float filtered_target_speed[3] = {0.0f, 0.0f, 0.0f};

static void speed_loop_control(int *motor_A_power, int *motor_B_power, int *motor_D_power);

static int clamp(int value, int minimum, int maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float clamp_float(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void set_motor_power(void (*set_motor)(int, uint32_t), int power) {
    power = -power;
    int direction = power < 0 ? -1 : power > 0 ? 1 : 0;
    uint32_t speed = (uint32_t)(power < 0 ? -power : power);
    set_motor(direction, speed);
}

void pid_init(void) {
    integral = 0.0f;
    previous_error = 0.0f;
    previous_correction = 0.0f;
    last_motor_A_power = 0;
    last_motor_B_power = 0;
    last_motor_D_power = 0;
    last_turn_error = 0.0f;
    turn_direction_score = 0.0f;
    frozen_direction = 0.0f;
    sign_idx = 0;
    for (int i = 0; i < 6; i++) recent_error_signs[i] = 0;
    filtered_target_speed[0] = 0.0f;
    filtered_target_speed[1] = 0.0f;
    filtered_target_speed[2] = 0.0f;
    pid_stop();
}

static void pid_follow_line(float error) {
    float derivative;
    float correction;
    float absolute_error = fabsf(error);
    int front_forward;
    int motor_A_power;
    int motor_B_power = 0;
    int motor_D_power;

    if (error > 1.5f || error < -1.5f) {
        last_turn_error = error;
    }

    frozen_direction = error;

    recent_error_signs[sign_idx % 6] = (error > 0.05f) ? 1 : (error < -0.05f) ? -1 : 0;
    sign_idx++;

    turn_direction_score *= 0.95f;
    if (absolute_error > 0.05f) {
        turn_direction_score += error;
        turn_direction_score = clamp_float(turn_direction_score, -3.0f, 3.0f);
    }

    if (absolute_error < 0.05f) {
        integral = 0.0f;
    } else {
        integral = clamp_float(integral + error,
                               -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    }

    derivative = clamp_float(error - previous_error,
                             -PID_DERIVATIVE_LIMIT, PID_DERIVATIVE_LIMIT);
    correction = PID_KP * error + PID_KI * integral + PID_KD * derivative;
    correction = clamp_float(correction, -PID_CORRECTION_LIMIT,
                             PID_CORRECTION_LIMIT);

    if (correction > previous_correction + PID_CORRECTION_RATE_MAX)
        correction = previous_correction + PID_CORRECTION_RATE_MAX;
    else if (correction < previous_correction - PID_CORRECTION_RATE_MAX)
        correction = previous_correction - PID_CORRECTION_RATE_MAX;
    previous_correction = correction;

    previous_error = error;

    front_forward = PID_CRUISE_SPEED - (int)(absolute_error * PID_SLOWDOWN_PER_ERROR);
    front_forward = clamp(front_forward, PID_MIN_FORWARD_SPEED, PID_CRUISE_SPEED);
    front_forward = front_forward * FRONT_FORWARD_RATIO / 1000;

    motor_A_power = front_forward -
                    (int)(correction * FRONT_LATERAL_RATIO / 1000.0f);
    motor_D_power = front_forward +
                    (int)(correction * FRONT_LATERAL_RATIO / 1000.0f);

    speed_loop_control(&motor_A_power, &motor_B_power, &motor_D_power);

    last_motor_A_power = -motor_A_power;
    last_motor_B_power = -motor_B_power;
    last_motor_D_power = -motor_D_power;
    set_motor_power(set_motor_A, motor_A_power);
    set_motor_power(set_motor_B, motor_B_power);
    set_motor_power(set_motor_D, motor_D_power);
}

static void pid_turn_to_find_line(void) {
    previous_correction = 0.0f;

    int pos_count = 0, neg_count = 0;
    for (int i = 0; i < 6; i++) {
        if (recent_error_signs[i] > 0) pos_count++;
        else if (recent_error_signs[i] < 0) neg_count++;
    }

    int motor_A_power = 0;
    int motor_B_power = 0;
    int motor_D_power = 0;

    if (fabsf(frozen_direction) > 0.3f) {
        if (frozen_direction > 0.0f) {
            motor_A_power = -PID_LOST_TURN_SPEED;
            motor_D_power = PID_LOST_TURN_SPEED;
        } else {
            motor_A_power = PID_LOST_TURN_SPEED;
            motor_D_power = -PID_LOST_TURN_SPEED;
        }
    } else if (pos_count > neg_count) {
        motor_A_power = -PID_LOST_TURN_SPEED;
        motor_D_power = PID_LOST_TURN_SPEED;
    } else if (neg_count > pos_count) {
        motor_A_power = PID_LOST_TURN_SPEED;
        motor_D_power = -PID_LOST_TURN_SPEED;
    } else if (fabsf(turn_direction_score) > 0.1f) {
        if (turn_direction_score > 0.0f) {
            motor_A_power = -PID_LOST_TURN_SPEED;
            motor_D_power = PID_LOST_TURN_SPEED;
        } else {
            motor_A_power = PID_LOST_TURN_SPEED;
            motor_D_power = -PID_LOST_TURN_SPEED;
        }
    } else if (fabsf(last_turn_error) > 0.5f) {
        if (last_turn_error > 0.0f) {
            motor_A_power = -PID_LOST_TURN_SPEED;
            motor_D_power = PID_LOST_TURN_SPEED;
        } else {
            motor_A_power = PID_LOST_TURN_SPEED;
            motor_D_power = -PID_LOST_TURN_SPEED;
        }
    }

    speed_loop_control(&motor_A_power, &motor_B_power, &motor_D_power);

    last_motor_A_power = -motor_A_power;
    last_motor_B_power = 0;
    last_motor_D_power = -motor_D_power;
    set_motor_power(set_motor_A, motor_A_power);
    set_motor_power(set_motor_B, 0);
    set_motor_power(set_motor_D, motor_D_power);
}

void pid_update(float error) {
    error = clamp_float(error, -3.0f, 3.0f);

#ifdef USE_CAMERA_TRACK
    int line_lost = camera_track_is_line_lost();
#else
    int line_lost = track_is_line_lost();
#endif

    if (line_lost) {
        pid_turn_to_find_line();
    } else {
        pid_follow_line(error);
    }
}

void pid_get_motor_states(pid_motor_state_t *motor_A,
                          pid_motor_state_t *motor_B,
                          pid_motor_state_t *motor_D) {
    if (motor_A != 0) {
        motor_A->direction = last_motor_A_power < 0 ? -1 :
                             last_motor_A_power > 0 ? 1 : 0;
        motor_A->speed = (uint32_t)(last_motor_A_power < 0 ?
                                    -last_motor_A_power : last_motor_A_power);
    }
    if (motor_B != 0) {
        motor_B->direction = last_motor_B_power < 0 ? -1 :
                             last_motor_B_power > 0 ? 1 : 0;
        motor_B->speed = (uint32_t)(last_motor_B_power < 0 ?
                                    -last_motor_B_power : last_motor_B_power);
    }
    if (motor_D != 0) {
        motor_D->direction = last_motor_D_power < 0 ? -1 :
                             last_motor_D_power > 0 ? 1 : 0;
        motor_D->speed = (uint32_t)(last_motor_D_power < 0 ?
                                    -last_motor_D_power : last_motor_D_power);
    }
}

void pid_stop(void) {
    previous_correction = 0.0f;
    last_motor_A_power = 0;
    last_motor_B_power = 0;
    last_motor_D_power = 0;
    set_motor_A(0, 0);
    set_motor_B(0, 0);
    set_motor_D(0, 0);
}

static void speed_loop_control(int *motor_A_power, int *motor_B_power, int *motor_D_power) {
#ifndef USE_ENCODER
    /* 没有编码器就别做闭环。读到的速度恒为 0，闭环会当成三个轮子全堵转，
     * 按各自目标值往上补 —— 转弯时 A/D 目标不同，会落进不同的增益档，
     * 把原本线性的差速非线性放大。这里只保留限幅。 */
    *motor_A_power = clamp(*motor_A_power, -PID_MAX_SPEED, PID_MAX_SPEED);
    *motor_B_power = clamp(*motor_B_power, -PID_MAX_SPEED, PID_MAX_SPEED);
    *motor_D_power = clamp(*motor_D_power, -PID_MAX_SPEED, PID_MAX_SPEED);
    return;
#else
    float real_speeds[3];
    encoder_get_speeds(&real_speeds[0], &real_speeds[1], &real_speeds[2]);

    int *motors[3] = {motor_A_power, motor_B_power, motor_D_power};
    const float power_to_speed[3] = {POWER_TO_SPEED_A, POWER_TO_SPEED_B, POWER_TO_SPEED_D};

    for (int i = 0; i < 3; i++) {
        int power = *motors[i];
        int abs_power = (power < 0) ? -power : power;

        if (abs_power < SPEED_MIN_POWER) {
            if (abs_power == 0) {
                filtered_target_speed[i] = 0.0f;
            }
            continue;
        }

        float raw_target = (float)abs_power / 1000.0f * power_to_speed[i];

        filtered_target_speed[i] = TARGET_SPEED_SMOOTH_ALPHA * filtered_target_speed[i] +
                                   (1.0f - TARGET_SPEED_SMOOTH_ALPHA) * raw_target;

        float target = filtered_target_speed[i];
        float measured = (real_speeds[i] < 0.0f) ? -real_speeds[i] : real_speeds[i];
        float speed_diff = target - measured;

        if (fabsf(speed_diff) < (float)SPEED_LOOP_DEADZONE) continue;

        float abs_diff = fabsf(speed_diff);
        float kp;
        if (abs_diff < 8.0f) {
            kp = SPEED_KP_SMALL;
        } else if (abs_diff < 20.0f) {
            kp = SPEED_KP_MEDIUM;
        } else {
            kp = SPEED_KP_LARGE;
        }

        float correction_f = kp * speed_diff;
        int correction = (int)correction_f;
        correction = clamp(correction, -SPEED_LOOP_MAX_CORRECTION, SPEED_LOOP_MAX_CORRECTION);

        if (power > 0) {
            *motors[i] += correction;
        } else {
            *motors[i] -= correction;
        }
    }

    *motor_A_power = clamp(*motor_A_power, -PID_MAX_SPEED, PID_MAX_SPEED);
    *motor_B_power = clamp(*motor_B_power, -PID_MAX_SPEED, PID_MAX_SPEED);
    *motor_D_power = clamp(*motor_D_power, -PID_MAX_SPEED, PID_MAX_SPEED);
#endif /* USE_ENCODER */
}

void pid_manual_control(int power_A, int power_B, int power_D) {
    power_A = clamp(power_A, -PID_MAX_SPEED, PID_MAX_SPEED);
    power_B = clamp(power_B, -PID_MAX_SPEED, PID_MAX_SPEED);
    power_D = clamp(power_D, -PID_MAX_SPEED, PID_MAX_SPEED);

    speed_loop_control(&power_A, &power_B, &power_D);

    last_motor_A_power = -power_A;
    last_motor_B_power = -power_B;
    last_motor_D_power = -power_D;

    set_motor_power(set_motor_A, power_A);
    set_motor_power(set_motor_B, power_B);
    set_motor_power(set_motor_D, power_D);
}