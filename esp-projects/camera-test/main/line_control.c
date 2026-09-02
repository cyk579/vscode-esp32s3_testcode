#include "line_control.h"

#include <stdlib.h>

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

static int min_int(int a, int b)
{
    return a < b ? a : b;
}

/* 需求量低于 floor 时攒进 *accum，攒够了发一个整脉冲；等效平均值不变。 */
static int dither(int demand, int floor, int *accum)
{
    if (demand == 0) {
        *accum = 0;
        return 0;
    }
    if (abs(demand) >= floor) {
        *accum = 0;
        return demand;
    }
    *accum += demand;
    if (abs(*accum) >= floor) {
        const int pulse = *accum > 0 ? floor : -floor;
        *accum -= pulse;
        return pulse;
    }
    return 0;
}

int line_control_yaw(const line_control_cfg_t *cfg, int heading_error,
                     int *accum)
{
    if (cfg == NULL || accum == NULL || cfg->scale <= 0) {
        return 0;
    }
    if (abs(heading_error) <= cfg->heading_deadband) {
        *accum = 0;
        return 0;
    }
    /* 远端偏左 (heading < 0) 要左转 (turn > 0)，所以这里是负号。 */
    int target = -cfg->kh * heading_error / cfg->scale;
    target = clamp_int(target, cfg->turn_max);
    return dither(target, cfg->yaw_min, accum);
}

int line_control_strafe(const line_control_cfg_t *cfg, int lateral_error,
                        int *accum)
{
    if (cfg == NULL || accum == NULL || cfg->scale <= 0) {
        return 0;
    }
    if (abs(lateral_error) <= cfg->error_deadband) {
        *accum = 0;
        return 0;
    }
    /* 线在左 (error > 0) 要向左平移 (lat < 0)，所以这里是负号。 */
    int demand = -cfg->kp_lat * lateral_error / cfg->scale;
    demand = clamp_int(demand, cfg->lat_max);
    return dither(demand, cfg->lat_min, accum);
}

int line_control_speed(const line_control_cfg_t *cfg,
                       const line_observation_t *observation,
                       bool alert)
{
    if (cfg == NULL || observation == NULL) {
        return 0;
    }
    int speed = cfg->speed_fast;
    const int magnitude = abs(observation->lateral_error);
    if (magnitude >= cfg->error_large) {
        speed = min_int(speed, cfg->speed_crawl);
    } else if (magnitude >= cfg->error_medium) {
        speed = min_int(speed, cfg->speed_slow);
    } else if (magnitude > cfg->error_deadband) {
        speed = min_int(speed, cfg->speed_medium);
    }
    if (abs(observation->heading_error) >= cfg->heading_slow) {
        speed = min_int(speed, cfg->speed_slow);
    }
    if (abs(observation->far_error) >= cfg->far_slow) {
        speed = min_int(speed, cfg->speed_slow);
    }
    /* 远处看见折角或终点只降速，转向由状态机在事件足够近时才触发。 */
    if (observation->corner_direction != 0 || observation->finish_candidate) {
        speed = min_int(speed, cfg->speed_crawl);
    }
    if (observation->valid_rows < cfg->min_valid_rows + 2) {
        speed = min_int(speed, cfg->speed_medium);
    }
    if (alert) {
        speed = min_int(speed, cfg->speed_slow);
    }
    return speed;
}
