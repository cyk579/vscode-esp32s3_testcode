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
#define LINE_ROI_LEFT_PERCENT 10
#define LINE_ROI_RIGHT_PERCENT 90
#define LINE_ROI_TOP_PERCENT 15
#define LINE_ROI_BOTTOM_PERCENT 100
#define LINE_LOCAL_CONTRAST_MIN 28
/* The tape is deliberately pure black.  An adaptive threshold may rise under
 * bright lighting, but it must not classify ordinary dark objects or a hand as
 * track pixels solely because the whole frame became dim. */
#define LINE_ABSOLUTE_BLACK_MAX_LUMA 88
#define LINE_NEAR_TOP_PERCENT 65
#define LINE_NEAR_BOTTOM_PERCENT 92
#define LINE_ROW_STEP 2
#define LINE_BOTTOM_SKIP_ROWS 0
#define LINE_SCAN_MAX_ROWS 64
#define LINE_SEARCH_HALF_PERCENT 26
#define LINE_SEARCH_HALF_MIN 6
#define LINE_SEARCH_HALF_MAX 80
/* 逐行搜索用上一行的斜率做预测，所以跳变门限可以比原来紧。 */
#define LINE_MAX_CENTER_JUMP_PERCENT 18
#define LINE_MIN_SEGMENT_WIDTH 2
#define LINE_BRIDGE_GAP_PIXELS 1
#define LINE_MAX_SEGMENT_WIDTH_PERCENT 55
#define LINE_MIN_VALID_ROWS 4
#define LINE_CORNER_MIN_VALID_ROWS 2
#define LINE_SEED_MISS_ROWS 6
#define LINE_TRACK_MISS_ROWS 3

/* 近场/航向/远场取样窗口按扫描高度的千分比折算成行数，而不是固定行数：
 * 控制帧可能是 120x80（480x320 输入）或 160x120（320x240 输入），固定行数
 * 会让同一物理姿态在两种尺寸下给出相差一倍的 heading。120x80 时分别得到
 * 3/6/14 行；heading 有意只看近场，避免直角弯远处分支把车带离当前线，
 * far_error 仍保留给速度限制。基线仍与跟踪长度无关。 */
#define LINE_NEAR_ROWS_PERMILLE 75
#define LINE_HEADING_ROWS_PERMILLE 150
#define LINE_FAR_ROWS_PERMILLE 350

/* 线段形状分类的相对门限，全部以近场线宽 w 为基准：
 *   宽黑区   run_width  > LINE_WIDE_RATIO * w
 *   某侧敞开 ext_side  >= LINE_WIDE_OPEN_RATIO * w
 * 实测胶带约 1.5 cm、终点横杆 10 cm，两侧各外伸约 3.3w，落在 2w 门限之上；
 * 直角弯的横条只有一侧敞开。两个门限之间有 4 倍余量，所以 w 估偏也不翻判。 */
#define LINE_WIDE_RATIO 3
#define LINE_WIDE_OPEN_RATIO 2
#define LINE_WIDTH_FALLBACK_PERCENT 4
#define LINE_FINISH_STEM_ROWS 3

/*
 * 摄像头相对车体的安装旋转。"扫描坐标系"永远是车体视角：sy 越大越靠近车，
 * sx 越大越靠车体右侧。缓冲区里的像素按这个枚举反查，不做整帧旋转拷贝。
 *
 * 判断方法见 test/README.md：绿点必须沿着胶带走，而不是横切胶带。
 */
typedef enum {
    LINE_ROTATE_0 = 0,   /* 缓冲区就是车体视角 */
    LINE_ROTATE_90,      /* 缓冲区顺时针转 90 度后才是车体视角 */
    LINE_ROTATE_180,
    LINE_ROTATE_270,
} line_rotation_t;

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
    line_rotation_t rotation;
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
    int corner_x;            /* 事件行的支路中心，用于预览 overlay */
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

/* 扫描坐标系的尺寸：90/270 度时和缓冲区的宽高互换。 */
int line_geometry_scan_width(const line_scan_cfg_t *cfg);
int line_geometry_scan_height(const line_scan_cfg_t *cfg);

/* 扫描坐标 -> 缓冲区坐标。overlay 要用它把点画回原始帧上。 */
void line_geometry_map(const line_scan_cfg_t *cfg, int sx, int sy,
                       int *bx, int *by);

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
