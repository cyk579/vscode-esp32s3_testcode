#pragma once

/*
 * 巡线控制律。纯函数，不依赖 ESP-IDF，可在 host 上测。
 *
 * 输入是 line_detect 给出的两个量（单位：画面宽度的千分之一）：
 *   lateral  + = 线在车右侧
 *   heading  + = 线向右倾（车头该往右拧）
 *
 * 输出是三轮混控的输入 (forward, turn, lat)，符号约定见 line_mixer.h：
 *   turn > 0 = 逆时针 = 左转
 *   lat  > 0 = 车体右移
 * 所以线偏右时 turn 取负、lat 取正。这两个符号搞反车就直接冲出赛道，
 * 因此它们由 test/harness.c 钉住。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 要在赛道上调的就是这几个数 ===== */
/* 速度。转弯时降到 CORNER，让下一帧还能看到线。 */
#define LINE_FORWARD_CRUISE 22
#define LINE_FORWARD_CORNER 15
/* 增益。turn = -KYAW*heading/1000，lat = +KLAT*lateral/1000。 */
#define LINE_GAIN_YAW 45
#define LINE_GAIN_LAT 55
/* 死区。直道上小幅噪声不动作，免得左右抖。 */
#define LINE_DEADBAND_HEADING 30
#define LINE_DEADBAND_LATERAL 45
/* 输出上限。turn=13 是三轮都能起转的最小值（a/d 起转 11，b 起转 13），
 * 15 留一点余量。 */
#define LINE_TURN_MAX 15
#define LINE_LAT_MAX 16
/* 车头歪得比这个多就先别平移：平移和转向会互相抵消。 */
#define LINE_STRAFE_HEADING_LIMIT 90

typedef struct {
    int forward;
    int turn;
    int lat;
} line_cmd_t;

/*
 * 由两个误差算出一条指令。
 *
 * 转向只看 heading，平移只看 lateral —— 全向底盘才能这么分工，也正是它
 * 不需要专门的"弯道识别"的原因：左转弯时远处那段横胶带把 far_x 拉到左边，
 * heading 自然变成大负数，turn 就是左转。
 */
void line_control_follow(int lateral, int heading, line_cmd_t *out);

/* 丢线后该往哪边转：返回 turn 值（正=左，负=右），0 表示没有依据。
 * 弯道处丢线是正常的（顶点走到车底下时线确实不在画面里）。 */
int line_control_search_turn(int lateral, int heading);

#ifdef __cplusplus
}
#endif
