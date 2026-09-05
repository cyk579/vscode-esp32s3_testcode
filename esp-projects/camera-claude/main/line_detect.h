#pragma once

/*
 * 黑线检测：两个横条带 + 一个跟随线的搜索窗口。不依赖 ESP-IDF，可在 host 上测。
 *
 * 只测量两个量：
 *   lateral = near_x - 画面中心   车体现在偏离线多少（左右平移能修）
 *   heading = far_x  - near_x     线在画面里的倾斜（车头方向不对，转身修）
 *
 * heading 用 far-near 而不是 far-中心：车与线平行但整体偏右时，两者都偏右，
 * 相减为 0，只出平移指令；若用 far-中心，车会转向、越过线、再转回来，来回振。
 *
 * 两个量都以画面宽度的千分之一为单位，所以增益不随解码分辨率变化。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 条带位置，占画面高度的百分比。near 贴着画面底部（离车最近）。 */
#define LINE_NEAR_TOP_PERCENT 84
#define LINE_NEAR_BOTTOM_PERCENT 99
#define LINE_FAR_TOP_PERCENT 38
#define LINE_FAR_BOTTOM_PERCENT 54

/* 搜索窗口半宽，占画面宽度的百分比。丢线时逐帧放宽到 MAX。 */
#define LINE_WINDOW_HALF_PERCENT 30
#define LINE_WINDOW_GROW_PERCENT 6
#define LINE_WINDOW_MAX_PERCENT 50

/* 条带里至少要有这么多暗像素才算看到线，占条带内取样点数的千分比。
 * 2cm 胶带在约 20cm 视野宽里约占 10% 宽度，整条带占满时约 100‰。 */
#define LINE_MIN_FILL_PERMILLE 12
/* 暗像素超过这个比例说明整片都暗（阴影、压到障碍物），不可信。 */
#define LINE_MAX_FILL_PERMILLE 600

typedef struct {
    bool found;        /* near 条带看到线；控制环只在此时更新误差 */
    bool far_valid;    /* far 条带也看到线；否则 heading 记 0 */
    int lateral;       /* 千分比，+ = 线在车右侧 */
    int heading;       /* 千分比，+ = 线向右倾（车该右转） */
    int near_x;        /* 像素，画面坐标，画叠加层用 */
    int far_x;
    int near_fill;     /* 千分比，诊断用 */
    int far_fill;
    int window_lo;     /* 本帧实际使用的窗口，画叠加层用 */
    int window_hi;
} line_obs_t;

/* 清空窗口记忆，把窗口放回画面正中。上电和停车后调用。 */
void line_detect_reset(void);

/*
 * 扫描一帧。frame 是大端 RGB565，threshold 是 camera_display 算好的灰度阈值
 * （0 表示这帧对比度不够，直接当没看到线）。
 * 返回值同 out->found。
 */
bool line_detect_run(const uint8_t *frame, int width, int height,
                     uint8_t threshold, line_obs_t *out);

#ifdef __cplusplus
}
#endif
