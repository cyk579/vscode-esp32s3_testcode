#pragma once

/*
 * 120° 三轮全向轮（kiwi drive）的混控层，不依赖 ESP-IDF，可以在 host 上测试。
 *
 * 车体坐标：+y 车头，+x 车体右侧，ω 逆时针为正。轮位角（自 +x 轴逆时针）
 * A(右前)=30°、D(左前)=150°、B(后)=270°，驱动方向为切向 (-sinφ, cosφ)：
 *
 *     v_A = -0.5*vx + 0.866*vy + L*w
 *     v_D = -0.5*vx - 0.866*vy + L*w
 *     v_B = +1.0*vx +   0      + L*w
 *
 * 配合 car-spin 实车校准的极性（MOTOR_A_SIGN=1, B=1, D=-1，A 相当于反装），
 * 这组方程化成有符号 PWM 命令就是：
 *
 *     a = -forward - turn +   lat
 *     d = +forward - turn +   lat
 *     b =            turn + 2*lat
 *
 * 其中 forward 对应 0.866*vy，turn 对应 L*w，lat 对应 0.5*vx，三者同量纲。
 * 纯直行时 b == 0 —— 这是全向轮的正确行为，不是"后轮没工作"。
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int ceiling;   /* 单轮 PWM 绝对上限（%），混控余量由它决定 */
    int floor_ad;  /* A/D 实测起转值，低于它电机不动 */
    int floor_b;   /* B 实测起转值 */
    int trim_a;    /* 直行配平，百分比，100 表示不修正 */
    int trim_d;
} line_mixer_cfg_t;

typedef struct {
    int a;
    int b;
    int d;
    bool scaled;   /* 触发了整向量等比缩放 */
    bool dropped;  /* 有分量低于起转值被置零 */
} line_mixer_out_t;

/*
 * 把 (forward, turn, lat) 解成三个轮子的有符号 PWM。
 *
 * 超出 ceiling 时整向量等比缩放，绝不单边削顶 —— 在全向底盘上单独裁剪一个
 * 分量会把指令向量整体转向，凭空产生没人要求的旋转或侧移。
 *
 * 低于起转值的分量置零而不是抬到起转值：抬值同样会旋转指令向量，而且幅度
 * 更大（例如 turn=1 被抬到 13 就是 13 倍的偏航）。
 */
void line_mixer_solve(int forward, int turn, int lat,
                      const line_mixer_cfg_t *cfg,
                      line_mixer_out_t *out);

/* 反解：由三个轮子命令还原车体意图，单位与输入相同。测试和诊断用。 */
void line_mixer_body(int a, int b, int d, int *forward, int *turn, int *lat);

#ifdef __cplusplus
}
#endif
