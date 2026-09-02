#pragma once

/*
 * 巡线的控制律，不依赖 ESP-IDF，可以在 host 上测符号。
 *
 * 符号约定（全部已由 car-spin 的实车校准间接确认）：
 *   lateral_error > 0   赛道在画面中心**左侧**，即车在线的右边
 *   heading_error < 0   线的远端偏**左**（远端 x 更小）
 *   turn > 0            车体逆时针旋转 = 左转
 *   lat  > 0            车体向**右**平移（+x）
 *
 * 于是：
 *   线在左  -> 要向左平移 -> lat < 0   -> lat  = -K * lateral_error
 *   远端偏左 -> 要左转     -> turn > 0  -> turn = -K * heading_error
 *
 * 两个负号都容易写错，而且写错了在合成图上看不出来（误差为 0 时输出也是 0），
 * 所以专门抽出来做单元测试。
 */

#include <stdbool.h>

#include "line_geometry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int kh;              /* 偏航增益 / scale */
    int kp_lat;          /* 横移增益 / scale */
    int scale;
    int turn_max;
    int yaw_min;         /* B 轮起转值：偏航要么 0 要么 >= 这个值 */
    int lat_max;
    int lat_min;         /* A/D 起转值：横移要么 0 要么 >= 这个值 */
    int heading_deadband;
    int error_deadband;
    int error_medium;
    int error_large;
    int heading_slow;
    int far_slow;
    int speed_fast;
    int speed_medium;
    int speed_slow;
    int speed_crawl;
    int min_valid_rows;
} line_control_cfg_t;

/*
 * 偏航：低于 yaw_min 的指令后轮推不动，所以和横移一样做时间抖动 —— 攒够了
 * 发一个整脉冲。否则死区要一直开到 |heading| >= yaw_min*scale/kh，在这台车
 * 上相当于容忍十几度的姿态误差。
 */
int line_control_yaw(const line_control_cfg_t *cfg, int heading_error,
                     int *accum);

/*
 * 横移：一阶纠偏 + 时间抖动。
 * 需求量低于 lat_min 时攒进 *accum，攒够了发一个整脉冲，等效平均值不变。
 */
int line_control_strafe(const line_control_cfg_t *cfg, int lateral_error,
                        int *accum);

/* 前进速度取所有限制里最小的一个。远端只在这里起作用，绝不参与转向。 */
int line_control_speed(const line_control_cfg_t *cfg,
                       const line_observation_t *observation,
                       bool alert);

#ifdef __cplusplus
}
#endif
