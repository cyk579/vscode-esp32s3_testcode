/*
 * 巡线几何层的 host 回归测试。用 gcc 直接编译，不需要 ESP-IDF：
 *
 *     cd test && make && ./harness
 *
 * 合成帧能验证几何判据与状态语义（直线/偏移/斜线/90°弯/终点 T/彩色干扰），
 * 不能验证真实光照、噪声、镜头畸变和胶带边缘 —— 那些必须用真机采帧回放。
 */
#include "line_geometry.h"
#include "line_mixer.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_W 240
#define FRAME_H 160
#define THRESHOLD 60

static uint8_t g_frame[FRAME_W * FRAME_H * 2];
static int g_failures;
static int g_checks;

static void put_pixel(int x, int y, int r, int g, int b)
{
    if (x < 0 || y < 0 || x >= FRAME_W || y >= FRAME_H) {
        return;
    }
    const uint16_t value = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) |
                                      ((b & 0xF8) >> 3));
    uint8_t *pixel = g_frame + (((size_t)y * FRAME_W + (size_t)x) * 2);
    pixel[0] = (uint8_t)(value >> 8);
    pixel[1] = (uint8_t)value;
}

static void fill_frame(int r, int g, int b)
{
    for (int y = 0; y < FRAME_H; ++y) {
        for (int x = 0; x < FRAME_W; ++x) {
            put_pixel(x, y, r, g, b);
        }
    }
}

/* 画一条有宽度的线段：对每个像素做点到线段的距离判断。 */
static void draw_segment(double x0, double y0, double x1, double y1,
                         double thickness, int r, int g, int b)
{
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double length_squared = dx * dx + dy * dy;
    const double half = thickness / 2.0;
    for (int y = 0; y < FRAME_H; ++y) {
        for (int x = 0; x < FRAME_W; ++x) {
            double t = 0.0;
            if (length_squared > 0.0) {
                t = ((x - x0) * dx + (y - y0) * dy) / length_squared;
                t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
            }
            const double px = x0 + t * dx - x;
            const double py = y0 + t * dy - y;
            if (px * px + py * py <= half * half) {
                put_pixel(x, y, r, g, b);
            }
        }
    }
}

static void draw_disc(double cx, double cy, double radius, int r, int g, int b)
{
    for (int y = 0; y < FRAME_H; ++y) {
        for (int x = 0; x < FRAME_W; ++x) {
            const double dx = x - cx;
            const double dy = y - cy;
            if (dx * dx + dy * dy <= radius * radius) {
                put_pixel(x, y, r, g, b);
            }
        }
    }
}

static void check(const char *scenario, const char *what, bool ok, const char *detail)
{
    ++g_checks;
    if (!ok) {
        ++g_failures;
        printf("    FAIL  %-22s %s  (%s)\n", scenario, what, detail);
    }
}

static line_scan_cfg_t base_cfg(int expected_width)
{
    line_scan_cfg_t cfg = {0};
    cfg.width = FRAME_W;
    cfg.height = FRAME_H;
    cfg.threshold = THRESHOLD;
    cfg.use_history = true;
    cfg.seed_x = FRAME_W / 2;
    cfg.search_half_percent = LINE_SEARCH_HALF_PERCENT;
    cfg.corridor_x = -1;
    cfg.corridor_half = 0;
    cfg.expected_width = expected_width;
    cfg.mirror_x = false;
    cfg.saturation_guard = false;
    cfg.saturation_max = 60;
    return cfg;
}

static void report(const char *name, const line_observation_t *o)
{
    printf("  %-18s cand=%d rows=%2u seed=%3d w=%2d ey=%4d eth=%4d "
           "corner=%+d@%d finish=%d\n",
           name, o->candidate, o->valid_rows, o->seed_x, o->near_width,
           o->lateral_error, o->heading_error, o->corner_direction,
           o->corner_row_y, o->finish_candidate);
}

#define BAR_Y 112
#define LINE_X 120

static void run_case(const char *name, int w, line_observation_t *obs, bool guard)
{
    line_scan_cfg_t cfg = base_cfg(w);
    cfg.saturation_guard = guard;
    line_geometry_track(g_frame, &cfg, obs);
    report(name, obs);
}

static void scenario_straight(int w)
{
    char name[32];
    line_observation_t obs;
    snprintf(name, sizeof(name), "straight/w=%d", w);
    fill_frame(240, 240, 240);
    draw_segment(LINE_X, FRAME_H - 1, LINE_X, 36, w, 0, 0, 0);
    run_case(name, w, &obs, false);
    check(name, "candidate", obs.candidate, "no line found");
    check(name, "rows>=18", obs.valid_rows >= 18, "line tracked too short");
    check(name, "|ey|<=3", abs(obs.lateral_error) <= 3, "centred line has offset error");
    check(name, "|eth|<=4", abs(obs.heading_error) <= 4, "straight line has heading");
    check(name, "no corner", obs.corner_direction == 0, "phantom corner on a straight");
    check(name, "no finish", !obs.finish_candidate, "phantom finish on a straight");
}

static void scenario_offset(int w)
{
    char name[32];
    line_observation_t obs;
    snprintf(name, sizeof(name), "offset+20/w=%d", w);
    fill_frame(240, 240, 240);
    draw_segment(LINE_X + 20, FRAME_H - 1, LINE_X + 20, 36, w, 0, 0, 0);
    run_case(name, w, &obs, false);
    check(name, "candidate", obs.candidate, "no line found");
    check(name, "ey in [-22,-11]",
          obs.lateral_error <= -11 && obs.lateral_error >= -22, "wrong error scale/sign");
    check(name, "no corner", obs.corner_direction == 0, "phantom corner");
    check(name, "no finish", !obs.finish_candidate, "phantom finish");
}

/* heading 的绝对标度由 LINE_HEADING_ROWS 定义，控制器的 KH 必须跟它配套；
 * 这里断言的是符号和单调性，以及陡斜线能给出可用的量级。 */
static int scenario_tilted(int w, int dx)
{
    char name[40];
    line_observation_t obs;
    snprintf(name, sizeof(name), "tilt%d/w=%d", -dx, w);
    fill_frame(240, 240, 240);
    draw_segment(LINE_X, FRAME_H - 1, LINE_X + dx, 36, w, 0, 0, 0);
    run_case(name, w, &obs, false);
    check(name, "candidate", obs.candidate, "no line found");
    check(name, "eth<0", obs.heading_error < 0, "far end left must give eth<0");
    check(name, "no finish", !obs.finish_candidate, "phantom finish on a tilt");
    check(name, "no corner", obs.corner_direction == 0, "a tilt is not a corner");
    return obs.heading_error;
}

static void scenario_corner(int w, int direction)
{
    char name[32];
    line_observation_t obs;
    snprintf(name, sizeof(name), "corner%s/w=%d", direction < 0 ? "-L" : "-R", w);
    fill_frame(240, 240, 240);
    draw_segment(LINE_X, FRAME_H - 1, LINE_X, BAR_Y, w, 0, 0, 0);
    /* 单侧横条：一直伸出 ROI 边界，另一侧什么都没有。 */
    draw_segment(LINE_X, BAR_Y, direction < 0 ? 30 : 210, BAR_Y, w, 0, 0, 0);
    run_case(name, w, &obs, false);
    check(name, "candidate", obs.candidate, "lost the line at the corner");
    check(name, "corner reported", obs.corner_direction == direction,
          "a 90 deg corner must report exactly one side");
    check(name, "no finish", !obs.finish_candidate,
          "one-sided corner bar must not look like the finish T");
    check(name, "|ey|<=4", abs(obs.lateral_error) <= 4,
          "near band is below the bar and must stay clean");
    check(name, "|eth|<=8", abs(obs.heading_error) <= 8,
          "clipped bar centroid must not drag the heading");
}

static void scenario_finish_t(int w)
{
    char name[32];
    line_observation_t obs;
    snprintf(name, sizeof(name), "finish-T/w=%d", w);
    fill_frame(240, 240, 240);
    draw_segment(LINE_X, FRAME_H - 1, LINE_X, BAR_Y, w, 0, 0, 0);
    /* 终点横杆实测 10 cm，两侧各外伸约 5 cm，并不横穿整个 ROI。 */
    draw_segment(LINE_X - 36, BAR_Y, LINE_X + 36, BAR_Y, w, 0, 0, 0);
    run_case(name, w, &obs, false);
    check(name, "candidate", obs.candidate, "lost the line at the finish");
    check(name, "finish reported", obs.finish_candidate,
          "two-sided bar in the near band is the finish T");
    check(name, "no corner", obs.corner_direction == 0,
          "the finish T must not be reported as a corner");
}

static void scenario_colour_blob(int w)
{
    char name[32];
    line_observation_t obs;
    const int r = 150, g = 20, b = 20;
    uint8_t pixel[2];
    const uint16_t value = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) |
                                      ((b & 0xF8) >> 3));
    pixel[0] = (uint8_t)(value >> 8);
    pixel[1] = (uint8_t)value;
    printf("  dark-red blob: luma=%u threshold=%d -> counted as black without a guard\n",
           line_geometry_luma(pixel), THRESHOLD);

    fill_frame(240, 240, 240);
    draw_segment(LINE_X, FRAME_H - 1, LINE_X, 36, w, 0, 0, 0);
    draw_disc(LINE_X + 12, 130, 11, r, g, b);

    snprintf(name, sizeof(name), "blob/noguard/w=%d", w);
    run_case(name, w, &obs, false);

    snprintf(name, sizeof(name), "blob/guard/w=%d", w);
    run_case(name, w, &obs, true);
    check(name, "|ey|<=3", abs(obs.lateral_error) <= 3,
          "saturation guard must reject the coloured blob");
    check(name, "width clean", obs.near_width <= w + 3,
          "merged blob+line run means the guard did not fire");
}

/* 混控层单元测试：等比缩放必须保方向，反解必须能还原车体意图。 */
static line_mixer_cfg_t mixer_cfg(void)
{
    line_mixer_cfg_t cfg = {50, 11, 13, 100, 100};
    return cfg;
}

static void scenario_mixer(void)
{
    static const struct { int f, t, l; } cases[] = {
        {26, 0, 0}, {0, 19, 0}, {0, 0, 12}, {20, 8, 0}, {17, 19, 0},
        {24, 12, 10}, {-20, 0, 0}, {0, -19, 0},
    };
    const line_mixer_cfg_t cfg = mixer_cfg();

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const int f = cases[i].f, t = cases[i].t, l = cases[i].l;
        char name[40];
        line_mixer_out_t out = {0, 0, 0, false, false};
        int rf = 0, rt = 0, rl = 0;
        snprintf(name, sizeof(name), "mix(%d,%d,%d)", f, t, l);
        line_mixer_solve(f, t, l, &cfg, &out);
        line_mixer_body(out.a, out.b, out.d, &rf, &rt, &rl);
        printf("  %-16s a=%4d b=%4d d=%4d  scaled=%d dropped=%d -> (%d,%d,%d)\n",
               name, out.a, out.b, out.d, out.scaled, out.dropped, rf, rt, rl);
        check(name, "|wheel|<=ceiling",
              abs(out.a) <= cfg.ceiling && abs(out.b) <= cfg.ceiling &&
              abs(out.d) <= cfg.ceiling, "ceiling exceeded");
        check(name, "no sub-floor output",
              (out.a == 0 || abs(out.a) >= cfg.floor_ad) &&
              (out.d == 0 || abs(out.d) >= cfg.floor_ad) &&
              (out.b == 0 || abs(out.b) >= cfg.floor_b),
              "a wheel was left below its stiction floor");
        if (!out.scaled && !out.dropped) {
            check(name, "round trip", rf == f && rt == t && rl == l,
                  "inverse mix did not reproduce the intent");
        }
        /* 纯直行时后轮必须完全不动，这是 kiwi 混控的正确行为。 */
        if (t == 0 && l == 0) {
            check(name, "b idle on straight", out.b == 0,
                  "rear omni wheel must be idle for pure translation");
        }
    }

    /* 单边削顶会旋转指令向量，等比缩放不会：检查 (f,t) 的比例被保住。 */
    line_mixer_out_t out = {0, 0, 0, false, false};
    int rf = 0, rt = 0, rl = 0;
    line_mixer_solve(40, 20, 0, &cfg, &out);
    line_mixer_body(out.a, out.b, out.d, &rf, &rt, &rl);
    printf("  %-16s a=%4d b=%4d d=%4d  scaled=%d -> (%d,%d,%d)\n",
           "mix(40,20,0)", out.a, out.b, out.d, out.scaled, rf, rt, rl);
    /* 单边削顶会旋转指令向量，等比缩放不会：检查 (f,t) 的比例被保住。
     * 整数除法会带来几个单位的舍入，所以用交叉乘积的容差判断。 */
    check("mix-scale", "direction kept",
          rf > 0 && rt > 0 && abs(rf * 20 - rt * 40) <= 40,
          "scaling changed the forward:turn ratio");
    check("mix-scale", "no parasitic strafe", rl == 0,
          "scaling introduced sideways motion");
}

/* 绕摄像头旋转的指令必须让三个轮子都过起转值，而且实际比例要对得上：
 * f≈0、lat/turn ≈ a/(2L)。turn 给小了 a/d = lat - turn 会掉到起转值以下，
 * 只剩后轮在推，车就变成绕后轮甩而不是绕镜头转。 */
static void scenario_pivot(void)
{
    const line_mixer_cfg_t cfg = mixer_cfg();
    static const int pivots[] = {26, -26};

    for (size_t i = 0; i < sizeof(pivots) / sizeof(pivots[0]); ++i) {
        const int turn = pivots[i];
        const int lat = turn * 50 / 100;
        char name[24];
        line_mixer_out_t out = {0, 0, 0, false, false};
        int rf = 0, rt = 0, rl = 0;
        snprintf(name, sizeof(name), "pivot(%d)", turn);
        line_mixer_solve(0, turn, lat, &cfg, &out);
        line_mixer_body(out.a, out.b, out.d, &rf, &rt, &rl);
        printf("  %-16s a=%4d b=%4d d=%4d  scaled=%d -> (%d,%d,%d)\n",
               name, out.a, out.b, out.d, out.scaled, rf, rt, rl);
        check(name, "all wheels drive",
              out.a != 0 && out.b != 0 && out.d != 0,
              "a pivot command left a wheel below its stiction floor");
        check(name, "no net forward", abs(rf) <= 1, "pivot should not translate");
        check(name, "turn keeps sign", (rt > 0) == (turn > 0),
              "pivot rotation flipped direction");
        check(name, "lat tracks turn", rt != 0 && abs(rl * 100 / rt - 50) <= 15,
              "camera-pivot strafe ratio lost");
    }

    /* 偏航给小了就会退化成"只有后轮在推"。 */
    line_mixer_out_t weak = {0, 0, 0, false, false};
    line_mixer_solve(0, 19, 9, &cfg, &weak);
    printf("  %-16s a=%4d b=%4d d=%4d  (weak pivot degenerates)\n",
           "pivot(19)", weak.a, weak.b, weak.d);
    check("pivot-weak", "documents the floor", weak.a == 0 && weak.d == 0,
          "expected the weak pivot to drop A and D");
}

int main(int argc, char **argv)
{
    static const int widths[] = {7, 11, 15};

    /* 回放模式：直接吃 player.py --save-dir 存下来的大端 RGB565 .bin。
     * 只打印观测结果，不做断言 —— 真实帧的期望值要人眼配合 TFT 判断。 */
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            FILE *file = fopen(argv[i], "rb");
            if (file == NULL) {
                printf("cannot open %s\n", argv[i]);
                return 2;
            }
            const size_t read = fread(g_frame, 1, sizeof(g_frame), file);
            fclose(file);
            if (read != sizeof(g_frame)) {
                printf("%s: expected %zu bytes for %dx%d, got %zu\n",
                       argv[i], sizeof(g_frame), FRAME_W, FRAME_H, read);
                return 2;
            }
            line_observation_t obs;
            run_case(argv[i], 0, &obs, false);
        }
        return 0;
    }

    printf("line geometry host regression  frame=%dx%d threshold=%d\n",
           FRAME_W, FRAME_H, THRESHOLD);
    for (size_t i = 0; i < sizeof(widths) / sizeof(widths[0]); ++i) {
        const int w = widths[i];
        printf("\n-- line width %d px --\n", w);
        scenario_straight(w);
        scenario_offset(w);
        {
            const int steep = scenario_tilted(w, -40);
            const int shallow = scenario_tilted(w, -20);
            char name[24];
            snprintf(name, sizeof(name), "tilt-scale/w=%d", w);
            check(name, "steep<=-8", steep <= -8, "steep tilt heading too weak");
            check(name, "steep<shallow", steep < shallow,
                  "heading must grow with slope");
        }
        scenario_corner(w, -1);
        scenario_corner(w, +1);
        scenario_finish_t(w);
        scenario_colour_blob(w);
    }

    printf("\n-- mixer --\n");
    scenario_mixer();
    scenario_pivot();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
