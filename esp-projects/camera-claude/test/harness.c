/*
 * host 端回归测试：用合成画面钉住 line_detect / line_control / line_mixer。
 *
 * 这里只测能在 PC 上测的东西，也就是"符号对不对、数量级对不对"。符号错了车
 * 会直接冲出赛道，而这是唯一能在没有实车的情况下抓到的一类 bug。
 * 速度、增益的具体数值只能在赛道上调，不在这里断言。
 *
 * 编译运行：bash build.sh
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "line_control.h"
#include "line_detect.h"
#include "line_mixer.h"

#define W 120
#define H 80

static int g_failures;

static void check(bool ok, const char *what, const char *detail)
{
    if (ok) {
        printf("  ok    %s\n", what);
    } else {
        printf("  FAIL  %s   (%s)\n", what, detail);
        ++g_failures;
    }
}

/* ===== 合成画面 ===== */

static uint8_t g_frame[W * H * 2];

static void fill_white(void)
{
    memset(g_frame, 0xff, sizeof(g_frame));
}

static void set_dark(int x, int y)
{
    if (x < 0 || y < 0 || x >= W || y >= H) {
        return;
    }
    uint8_t *p = g_frame + ((size_t)y * W + (size_t)x) * 2U;
    p[0] = 0x00;
    p[1] = 0x00;
}

/* 画一条宽 thick 的竖直胶带，中心从 y=0 的 x_top 线性走到 y=H-1 的 x_bottom。 */
static void draw_tape(int x_top, int x_bottom, int thick)
{
    for (int y = 0; y < H; ++y) {
        const int cx = x_top + (x_bottom - x_top) * y / (H - 1);
        for (int d = -thick / 2; d <= thick / 2; ++d) {
            set_dark(cx + d, y);
        }
    }
}

/* 画一段横向胶带（直角弯的出弯支路）。 */
static void draw_tape_h(int y_centre, int x_lo, int x_hi, int thick)
{
    for (int x = x_lo; x <= x_hi; ++x) {
        for (int d = -thick / 2; d <= thick / 2; ++d) {
            set_dark(x, y_centre + d);
        }
    }
}

static int band_y(int percent)
{
    return H * percent / 100;
}

/* ===== 检测器 ===== */

static line_obs_t run(void)
{
    line_obs_t obs;
    memset(&obs, 0, sizeof(obs));
    const bool found = line_detect_run(g_frame, W, H, 110, &obs);
    obs.found = found;
    return obs;
}

static void t_straight_centered(void)
{
    printf("straight, centered\n");
    line_detect_reset();
    fill_white();
    draw_tape(W / 2, W / 2, 10);
    const line_obs_t o = run();
    char d[128];
    snprintf(d, sizeof(d), "found=%d lateral=%d heading=%d", o.found, o.lateral,
             o.heading);
    check(o.found, "找到线", d);
    check(abs(o.lateral) <= LINE_DEADBAND_LATERAL, "lateral 在死区内", d);
    check(abs(o.heading) <= LINE_DEADBAND_HEADING, "heading 在死区内", d);

    line_cmd_t c;
    line_control_follow(o.lateral, o.heading, &c);
    snprintf(d, sizeof(d), "f=%d t=%d l=%d", c.forward, c.turn, c.lat);
    check(c.turn == 0 && c.lat == 0, "直道不转不平移", d);
    check(c.forward == LINE_FORWARD_CRUISE, "直道全速", d);
}

static void t_offset_right(void)
{
    printf("平行但整体偏右（车该平移，不该转）\n");
    line_detect_reset();
    fill_white();
    const int x = W / 2 + W * 20 / 100;
    draw_tape(x, x, 10);
    const line_obs_t o = run();
    char d[128];
    snprintf(d, sizeof(d), "lateral=%d heading=%d", o.lateral, o.heading);
    check(o.found, "找到线", d);
    check(o.lateral > LINE_DEADBAND_LATERAL, "lateral 为正（线在右）", d);
    check(abs(o.heading) <= LINE_DEADBAND_HEADING,
          "heading 仍在死区（far-near 抵消了偏移）", d);

    line_cmd_t c;
    line_control_follow(o.lateral, o.heading, &c);
    snprintf(d, sizeof(d), "f=%d t=%d l=%d", c.forward, c.turn, c.lat);
    check(c.lat > 0, "lat 为正（车右移）", d);
    check(c.turn == 0, "不转向", d);
}

static void t_offset_left(void)
{
    printf("平行但整体偏左\n");
    line_detect_reset();
    fill_white();
    const int x = W / 2 - W * 20 / 100;
    draw_tape(x, x, 10);
    const line_obs_t o = run();
    line_cmd_t c;
    line_control_follow(o.lateral, o.heading, &c);
    char d[128];
    snprintf(d, sizeof(d), "lateral=%d lat=%d", o.lateral, c.lat);
    check(o.lateral < -LINE_DEADBAND_LATERAL, "lateral 为负", d);
    check(c.lat < 0, "lat 为负（车左移）", d);
}

static void t_tilt_right(void)
{
    printf("线向右倾（车该右转）\n");
    line_detect_reset();
    fill_white();
    draw_tape(W / 2 + W * 25 / 100, W / 2, 10);
    const line_obs_t o = run();
    char d[128];
    snprintf(d, sizeof(d), "near=%d far=%d heading=%d", o.near_x, o.far_x,
             o.heading);
    check(o.found && o.far_valid, "两条带都找到", d);
    check(o.heading > LINE_DEADBAND_HEADING, "heading 为正", d);

    line_cmd_t c;
    line_control_follow(o.lateral, o.heading, &c);
    snprintf(d, sizeof(d), "t=%d f=%d", c.turn, c.forward);
    check(c.turn < 0, "turn 为负 = 顺时针 = 右转", d);
    check(c.forward < LINE_FORWARD_CRUISE, "转弯降速", d);
}

static void t_tilt_left(void)
{
    printf("线向左倾（车该左转）\n");
    line_detect_reset();
    fill_white();
    draw_tape(W / 2 - W * 25 / 100, W / 2, 10);
    const line_obs_t o = run();
    line_cmd_t c;
    line_control_follow(o.lateral, o.heading, &c);
    char d[128];
    snprintf(d, sizeof(d), "heading=%d turn=%d", o.heading, c.turn);
    check(o.heading < -LINE_DEADBAND_HEADING, "heading 为负", d);
    check(c.turn > 0, "turn 为正 = 逆时针 = 左转", d);
}

static void t_corner_left_90(void)
{
    printf("向左的 90 度直角弯\n");
    line_detect_reset();
    fill_white();
    /* 竖直段从画面底部上来，到 far 带高度处折向左。 */
    const int vertex_y = band_y(LINE_FAR_BOTTOM_PERCENT);
    for (int y = vertex_y; y < H; ++y) {
        for (int d = -5; d <= 5; ++d) {
            set_dark(W / 2 + d, y);
        }
    }
    draw_tape_h(vertex_y, 2, W / 2, 10);

    const line_obs_t o = run();
    char d[160];
    snprintf(d, sizeof(d), "near=%d far=%d heading=%d far_valid=%d", o.near_x,
             o.far_x, o.heading, o.far_valid);
    check(o.found, "近处仍看到竖直段", d);
    check(o.heading < -LINE_DEADBAND_HEADING,
          "heading 明显为负（远处横支路在左）", d);

    line_cmd_t c;
    line_control_follow(o.lateral, o.heading, &c);
    snprintf(d, sizeof(d), "t=%d l=%d f=%d", c.turn, c.lat, c.forward);
    check(c.turn > 0, "左转", d);
    check(c.forward <= LINE_FORWARD_CORNER + 2, "降到接近弯道速度", d);
}

static void t_corner_right_acute(void)
{
    printf("向右的锐角急弯（起点后第一个弯）\n");
    line_detect_reset();
    fill_white();
    const int vertex_y = band_y(LINE_FAR_BOTTOM_PERCENT);
    for (int y = vertex_y; y < H; ++y) {
        for (int d = -5; d <= 5; ++d) {
            set_dark(W / 2 + d, y);
        }
    }
    /* 锐角：出弯支路斜着回到右上方。 */
    for (int x = W / 2; x < W - 2; ++x) {
        const int y = vertex_y - (x - W / 2) * 6 / 10;
        for (int d = -5; d <= 5; ++d) {
            set_dark(x, y + d);
        }
    }

    const line_obs_t o = run();
    line_cmd_t c;
    line_control_follow(o.lateral, o.heading, &c);
    char d[160];
    snprintf(d, sizeof(d), "near=%d far=%d heading=%d turn=%d", o.near_x,
             o.far_x, o.heading, c.turn);
    check(o.found, "找到线", d);
    check(o.heading > LINE_DEADBAND_HEADING, "heading 为正", d);
    check(c.turn < 0, "右转", d);
}

static void t_blank(void)
{
    printf("全白画面（线不在视野里）\n");
    line_detect_reset();
    fill_white();
    const line_obs_t o = run();
    check(!o.found, "报告丢线", "found 应为 false");
}

static void t_all_dark(void)
{
    printf("整片发黑（阈值漂了 / 相机糊了）\n");
    line_detect_reset();
    memset(g_frame, 0x00, sizeof(g_frame));
    const line_obs_t o = run();
    check(!o.found, "被 MAX_FILL 挡掉，不当成线", "found 应为 false");
}

static void t_reject_distractor(void)
{
    printf("窗口外的干扰暗块（桌腿、障碍物木板）\n");
    line_detect_reset();
    fill_white();
    /* 先给一帧干净的中心线，让窗口锁定在中间。 */
    draw_tape(W / 2, W / 2, 10);
    const line_obs_t lock = run();
    char d[160];
    snprintf(d, sizeof(d), "lock near_x=%d window=[%d,%d]", lock.near_x,
             lock.window_lo, lock.window_hi);
    check(lock.found, "先锁定中心线", d);

    /* 同一帧再在最左边加一大块暗区。窗口应该把它挡在外面。 */
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < 12; ++x) {
            set_dark(x, y);
        }
    }
    const line_obs_t o = run();
    snprintf(d, sizeof(d), "near_x=%d lateral=%d window=[%d,%d]", o.near_x,
             o.lateral, o.window_lo, o.window_hi);
    check(o.found, "仍跟住中心线", d);
    check(abs(o.lateral) <= LINE_DEADBAND_LATERAL, "质心没被拉向左边暗块", d);
}

static void t_window_grows_when_lost(void)
{
    printf("丢线后搜索窗口逐帧变宽\n");
    line_detect_reset();
    fill_white();
    draw_tape(W / 2, W / 2, 10);
    (void)run(); /* 锁定 */

    fill_white(); /* 线消失 */
    int prev_span = -1;
    bool grew = true;
    for (int i = 0; i < 4; ++i) {
        const line_obs_t o = run();
        const int span = o.window_hi - o.window_lo;
        if (prev_span >= 0 && span < prev_span) {
            grew = false;
        }
        prev_span = span;
    }
    char d[64];
    snprintf(d, sizeof(d), "最终窗口宽 %d", prev_span);
    check(grew, "窗口单调变宽（不会越找越窄）", d);
    check(prev_span <= W, "窗口不超出画面", d);
}

static void t_resolution_independent(void)
{
    printf("换分辨率不用改增益（千分比单位）\n");
    /* 同一个物理场景：线在右侧 20% 处。120x80 的结果上面已经测过，
     * 这里用一张 160x120 的画面比对千分比是否一致。 */
    enum { W2 = 160, H2 = 120 };
    static uint8_t big[W2 * H2 * 2];
    memset(big, 0xff, sizeof(big));
    const int x = W2 / 2 + W2 * 20 / 100;
    for (int y = 0; y < H2; ++y) {
        for (int dx = -6; dx <= 6; ++dx) {
            const int px = x + dx;
            if (px < 0 || px >= W2) { continue; }
            uint8_t *p = big + ((size_t)y * W2 + (size_t)px) * 2U;
            p[0] = 0x00;
            p[1] = 0x00;
        }
    }
    line_detect_reset();
    line_obs_t o;
    memset(&o, 0, sizeof(o));
    const bool found = line_detect_run(big, W2, H2, 110, &o);

    line_detect_reset();
    fill_white();
    const int xs = W / 2 + W * 20 / 100;
    draw_tape(xs, xs, 10);
    const line_obs_t small = run();

    char d[128];
    snprintf(d, sizeof(d), "160x120 lateral=%d, 120x80 lateral=%d", o.lateral,
             small.lateral);
    check(found && small.found, "两个分辨率都找到线", d);
    check(abs(o.lateral - small.lateral) <= 25, "lateral 千分比基本一致", d);
}

/* ===== 混控 ===== */

/* 与 line_follow.c 里的实车校准值保持一致。 */
static const line_mixer_cfg_t MIX_CFG = {
    .ceiling = 44,
    .floor_ad = 11,
    .floor_b = 13,
    .trim_a = 90,
    .trim_d = 100,
};

static void mix(int forward, int turn, int lat, int *a, int *b, int *d)
{
    line_mixer_out_t out;
    memset(&out, 0, sizeof(out));
    line_mixer_solve(forward, turn, lat, &MIX_CFG, &out);
    *a = out.a;
    *b = out.b;
    *d = out.d;
}

static void t_mixer_spin(void)
{
    printf("原地旋转：三个轮子都要真的转起来\n");
    int a, b, d;
    char msg[128];

    mix(0, LINE_TURN_MAX, 0, &a, &b, &d);
    snprintf(msg, sizeof(msg), "turn=+%d -> a=%d b=%d d=%d", LINE_TURN_MAX, a, b,
             d);
    check(abs(a) >= 11 && abs(d) >= 11 && abs(b) >= 13, "都过起转值", msg);
    check(a < 0 && d < 0 && b > 0, "左转时 a/d 反向、b 正向", msg);

    mix(0, -LINE_TURN_MAX, 0, &a, &b, &d);
    snprintf(msg, sizeof(msg), "turn=-%d -> a=%d b=%d d=%d", LINE_TURN_MAX, a, b,
             d);
    check(a > 0 && d > 0 && b < 0, "右转符号整体取反", msg);
}

static void t_mixer_strafe(void)
{
    printf("纯平移\n");
    int a, b, d;
    char msg[128];

    mix(0, 0, LINE_LAT_MAX, &a, &b, &d);
    snprintf(msg, sizeof(msg), "lat=+%d -> a=%d b=%d d=%d", LINE_LAT_MAX, a, b,
             d);
    check(a > 0 && d > 0 && b > 0, "右移时三轮同向", msg);
    check(abs(a) >= 11 && abs(d) >= 11 && abs(b) >= 13, "都过起转值", msg);
    check(abs(a) <= 44 && abs(b) <= 44 && abs(d) <= 44, "都在 PWM 上限内", msg);

    mix(0, 0, -LINE_LAT_MAX, &a, &b, &d);
    snprintf(msg, sizeof(msg), "lat=-%d -> a=%d b=%d d=%d", LINE_LAT_MAX, a, b,
             d);
    check(a < 0 && d < 0 && b < 0, "左移符号整体取反", msg);
}

static void t_mixer_forward(void)
{
    printf("纯直行\n");
    int a, b, d;
    char msg[128];
    mix(LINE_FORWARD_CRUISE, 0, 0, &a, &b, &d);
    snprintf(msg, sizeof(msg), "f=%d -> a=%d b=%d d=%d", LINE_FORWARD_CRUISE, a,
             b, d);
    check(a < 0 && d > 0, "a/d 反向（对置安装）", msg);
    check(abs(a) >= 11 && abs(d) >= 11, "过起转值", msg);
    check(b == 0, "后轮不出力", msg);
}

static void t_mixer_combined_scales(void)
{
    printf("直行+转向+平移叠加后不超上限\n");
    int a, b, d;
    char msg[128];
    mix(LINE_FORWARD_CRUISE, LINE_TURN_MAX, LINE_LAT_MAX, &a, &b, &d);
    snprintf(msg, sizeof(msg), "a=%d b=%d d=%d", a, b, d);
    check(abs(a) <= 44 && abs(b) <= 44 && abs(d) <= 44, "整体缩放到上限内", msg);
    check(a != 0 && b != 0 && d != 0, "缩放后没有轮子被压到 0", msg);
}

static void t_search_direction(void)
{
    printf("丢线找线方向\n");
    char msg[96];
    const int right = line_control_search_turn(0, 200);
    snprintf(msg, sizeof(msg), "heading=+200 -> search=%d", right);
    check(right < 0, "线右倾时往右找", msg);

    const int left = line_control_search_turn(0, -200);
    snprintf(msg, sizeof(msg), "heading=-200 -> search=%d", left);
    check(left > 0, "线左倾时往左找", msg);

    const int by_lat = line_control_search_turn(200, 0);
    snprintf(msg, sizeof(msg), "lateral=+200 -> search=%d", by_lat);
    check(by_lat < 0, "只有横向偏差时按偏差方向找", msg);

    check(line_control_search_turn(0, 0) == 0, "居中时没有依据", "应为 0");
}

int main(void)
{
    printf("=== camera-claude host 回归测试 ===\n\n");

    t_straight_centered();
    t_offset_right();
    t_offset_left();
    t_tilt_right();
    t_tilt_left();
    t_corner_left_90();
    t_corner_right_acute();
    t_blank();
    t_all_dark();
    t_reject_distractor();
    t_window_grows_when_lost();
    t_resolution_independent();
    t_mixer_spin();
    t_mixer_strafe();
    t_mixer_forward();
    t_mixer_combined_scales();
    t_search_direction();

    printf("\n");
    if (g_failures == 0) {
        printf("全部通过\n");
        return 0;
    }
    printf("%d 项失败\n", g_failures);
    return 1;
}
