#pragma once

/*
 * 巡线的纯几何层：不依赖 ESP-IDF，可以用 host 编译器直接编译和测试。
 * camera_line_follow.c 只负责状态机、电机和日志；这里只负责"从一帧
 * RGB565 里把黑线的位置和形状读出来"。
 *
 * 帧格式与解码器输出一致：每像素 2 字节，大端 RGB565（高字节在前）。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- ROI 与扫描几何（ESP 端和 host 测试共用同一组常量） ---- */
#define LINE_ROI_LEFT_PERCENT 20
#define LINE_ROI_RIGHT_PERCENT 80
#define LINE_ROI_TOP_PERCENT 30
#define LINE_ROI_BOTTOM_PERCENT 100
#define LINE_NEAR_TOP_PERCENT 65
#define LINE_NEAR_BOTTOM_PERCENT 92
#define LINE_ROW_STEP 4
#define LINE_BOTTOM_SKIP_ROWS 1
#define LINE_SCAN_MAX_ROWS 64
#define LINE_SEARCH_HALF_PERCENT 20
#define LINE_SEARCH_HALF_MIN 6
#define LINE_SEARCH_HALF_MAX 80
/* 逐行搜索用上一行的斜率做预测，所以跳变门限可以比原来紧。 */
#define LINE_MAX_CENTER_JUMP_PERCENT 12
#define LINE_MIN_SEGMENT_WIDTH 3
#define LINE_MAX_SEGMENT_WIDTH_PERCENT 55
#define LINE_MIN_VALID_ROWS 4
#define LINE_SEED_MISS_ROWS 3

/* 近场取样窗口全部用固定行数，避免"基线随跟踪长度变化"导致前馈增益漂移。 */
#define LINE_NEAR_ROWS 3
#define LINE_HEADING_ROWS 10
#define LINE_FAR_ROWS 14

/* 线段形状分类的相对门限，全部以近场线宽 w 为基准：
 *   宽黑区   run_width  > LINE_WIDE_RATIO * w
 *   某侧敞开 ext_side  >= LINE_WIDE_OPEN_RATIO * w
 * 实测胶带约 1.5 cm、终点横杆 10 cm，两侧各外伸约 3.3w，落在 2w 门限之上；
 * 直角弯的横条只有一侧敞开。两个门限之间有 4 倍余量，所以 w 估偏也不翻判。 */
#define LINE_WIDE_RATIO 3
#define LINE_WIDE_OPEN_RATIO 2
#define LINE_WIDTH_FALLBACK_PERCENT 4
#define LINE_FINISH_STEM_ROWS 3

/* 单行黑色线段的形状分类。WIDE_* 表示该行的黑区宽度远超正常线宽，
 * 其质心是搜索窗口的产物而不是赛道位置，不能用于转向。 */
typedef enum {
    LINE_ROW_NORMAL = 0,
    LINE_ROW_WIDE_LEFT,
    LINE_ROW_WIDE_RIGHT,
    LINE_ROW_WIDE_BOTH,
} line_row_kind_t;

typedef struct {
    int center;
    int width;
    int ext_left;        /* 黑区相对 expected 向左的延伸量（px） */
    int ext_right;       /* 向右的延伸量（px） */
    bool clipped_left;   /* 黑区触到搜索窗口/ROI 左边界 */
    bool clipped_right;
    line_row_kind_t kind;
} line_segment_t;

typedef struct {
    uint16_t width;
    uint16_t height;
    int threshold;           /* luma <= threshold 判为黑；0 表示无效 */
    bool use_history;        /* 有可信历史 seed 时只在 seed 附近找线 */
    int seed_x;
    int search_half_percent; /* 由调用方按状态给出 */
    int corridor_x;          /* < 0 表示不启用走廊约束 */
    int corridor_half;
    int expected_width;      /* 近场线宽 EMA（px），0 表示未知 */
    bool mirror_x;
    bool saturation_guard;   /* 额外要求 max-min <= saturation_max */
    int saturation_max;
} line_scan_cfg_t;

typedef struct {
    bool candidate;
    bool near_line_visible;
    bool finish_candidate;
    int seed_x;
    int lateral_error;
    int heading_error;
    int corner_direction;    /* -1 左 / +1 右 / 0 无 */
    int corner_row_y;        /* 事件所在行，用于判断远近；-1 表示无 */
    int far_error;           /* 远端取样点误差，只用于限速 */
    int near_width;          /* 最底行的黑段宽度（px） */
    uint8_t near_normal_rows;/* 近场里形状正常的行数 */
    uint8_t valid_rows;
    uint8_t confidence;
    int threshold;
    int scan_bottom_y;       /* 实际扫描起始行，overlay 用 */
    int point_count;
    int16_t point_x[LINE_SCAN_MAX_ROWS];
    int16_t point_y[LINE_SCAN_MAX_ROWS];
} line_observation_t;

uint8_t line_geometry_luma(const uint8_t *pixel);

/* 正值表示赛道在画面中心左侧，与已实车验证的转向符号配套：
 * 线偏左 -> error > 0 -> turn > 0 -> 逆时针 -> 向左。 */
int line_geometry_error(int center, int image_width, bool mirror_x);

int line_geometry_positive_percent(int value, int percent, int minimum);

/* 从底部 seed 向上逐行跟踪黑线。返回值等于 observation->candidate。 */
bool line_geometry_track(const uint8_t *frame,
                         const line_scan_cfg_t *cfg,
                         line_observation_t *observation);

#ifdef __cplusplus
}
#endif
