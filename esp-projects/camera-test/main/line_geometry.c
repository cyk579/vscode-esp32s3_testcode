#include "line_geometry.h"

#include <limits.h>
#include <stddef.h>
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

int line_geometry_positive_percent(int value, int percent, int minimum)
{
    int result = value * percent / 100;
    return result < minimum ? minimum : result;
}

uint8_t line_geometry_luma(const uint8_t *pixel)
{
    const uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    const int red = ((value >> 11) & 0x1F) * 255 / 31;
    const int green = ((value >> 5) & 0x3F) * 255 / 63;
    const int blue = (value & 0x1F) * 255 / 31;
    return (uint8_t)((77 * red + 150 * green + 29 * blue) >> 8);
}

/* 彩色物体（例如赛道板上的红球）亮度可能落在黑线阈值区间内，但通道差很大。 */
static int rgb565_saturation(const uint8_t *pixel)
{
    const uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    const int red = ((value >> 11) & 0x1F) * 255 / 31;
    const int green = ((value >> 5) & 0x3F) * 255 / 63;
    const int blue = (value & 0x1F) * 255 / 31;
    int high = red > green ? red : green;
    int low = red < green ? red : green;
    high = blue > high ? blue : high;
    low = blue < low ? blue : low;
    return high - low;
}

int line_geometry_error(int center, int image_width, bool mirror_x)
{
    int error = (image_width / 2 - center) * 100 / (image_width / 2);
    error = clamp_int(error, 100);
    return mirror_x ? -error : error;
}

int line_geometry_scan_width(const line_scan_cfg_t *cfg)
{
    if (cfg == NULL) {
        return 0;
    }
    return (cfg->rotation == LINE_ROTATE_90 || cfg->rotation == LINE_ROTATE_270) ?
           (int)cfg->height : (int)cfg->width;
}

int line_geometry_scan_height(const line_scan_cfg_t *cfg)
{
    if (cfg == NULL) {
        return 0;
    }
    return (cfg->rotation == LINE_ROTATE_90 || cfg->rotation == LINE_ROTATE_270) ?
           (int)cfg->width : (int)cfg->height;
}

void line_geometry_map(const line_scan_cfg_t *cfg, int sx, int sy,
                       int *bx, int *by)
{
    if (cfg == NULL || bx == NULL || by == NULL) {
        return;
    }
    const int w = (int)cfg->width;
    const int h = (int)cfg->height;
    switch (cfg->rotation) {
    case LINE_ROTATE_90:
        *bx = sy;
        *by = h - 1 - sx;
        break;
    case LINE_ROTATE_180:
        *bx = w - 1 - sx;
        *by = h - 1 - sy;
        break;
    case LINE_ROTATE_270:
        *bx = w - 1 - sy;
        *by = sx;
        break;
    case LINE_ROTATE_0:
    default:
        *bx = sx;
        *by = sy;
        break;
    }
}

/* Estimate the local floor brightness for one scan row.  The brightest few
 * samples are used so the black tape itself cannot pull the estimate down;
 * broad lamp shadows therefore move the threshold with the floor instead of
 * turning the whole row into a false black segment. */
static int row_background_luma(const uint8_t *frame,
                               const line_scan_cfg_t *cfg,
                               int sy)
{
    const int width = line_geometry_scan_width(cfg);
    const int roi_left = width * LINE_ROI_LEFT_PERCENT / 100;
    const int roi_right = (width * LINE_ROI_RIGHT_PERCENT / 100) - 1;
    if (roi_right <= roi_left) {
        return 0;
    }

    int brightest[4] = {0, 0, 0, 0};
    const int sample_count = 12;
    for (int i = 0; i < sample_count; ++i) {
        const int sx = roi_left + (roi_right - roi_left) * i /
                       (sample_count - 1);
        int bx = 0;
        int by = 0;
        line_geometry_map(cfg, sx, sy, &bx, &by);
        if (bx < 0 || by < 0 || bx >= (int)cfg->width || by >= (int)cfg->height) {
            continue;
        }
        const uint8_t *pixel = frame + (((size_t)by * cfg->width + (size_t)bx) * 2);
        const int value = line_geometry_luma(pixel);
        for (int j = 0; j < 4; ++j) {
            if (value > brightest[j]) {
                for (int k = 3; k > j; --k) {
                    brightest[k] = brightest[k - 1];
                }
                brightest[j] = value;
                break;
            }
        }
    }
    return (brightest[0] + brightest[1] + brightest[2] + brightest[3]) / 4;
}

static bool pixel_is_dark(const uint8_t *frame, const line_scan_cfg_t *cfg,
                          int sx, int sy, int row_threshold)
{
    int bx = 0;
    int by = 0;
    line_geometry_map(cfg, sx, sy, &bx, &by);
    if (bx < 0 || by < 0 || bx >= (int)cfg->width || by >= (int)cfg->height) {
        return false;
    }
    const uint8_t *pixel = frame + (((size_t)by * cfg->width + (size_t)bx) * 2);
    if (line_geometry_luma(pixel) > row_threshold) {
        return false;
    }
    if (cfg->saturation_guard && rgb565_saturation(pixel) > cfg->saturation_max) {
        return false;
    }
    return true;
}

/*
 * 单行搜索：返回离 reference 最近的合理黑段，并按"相对近场线宽"给出形状分类。
 *
 * 关键点：被搜索窗口边界截断的黑段，其质心由窗口位置决定而不是由赛道决定。
 * 直角弯的横条、终点 T 的横杆、大片阴影都会命中这种情况。调用方必须先看
 * kind，NORMAL 之外的行不能当成赛道中心点使用。
 */
static bool scan_segment(const uint8_t *frame,
                         const line_scan_cfg_t *cfg,
                         int y,
                         int expected_x,
                         int half_window,
                         int width_reference,
                         line_segment_t *segment)
{
    const int width = line_geometry_scan_width(cfg);
    if (frame == NULL || segment == NULL || y < 0 ||
        y >= line_geometry_scan_height(cfg)) {
        return false;
    }

    const line_segment_t empty = {0, 0, 0, 0, false, false, LINE_ROW_NORMAL};
    *segment = empty;
    const int roi_left = width * LINE_ROI_LEFT_PERCENT / 100;
    const int roi_right = (width * LINE_ROI_RIGHT_PERCENT / 100) - 1;
    int left = expected_x < 0 ? roi_left : expected_x - half_window;
    int right = expected_x < 0 ? roi_right : expected_x + half_window;
    left = left < roi_left ? roi_left : left;
    right = right > roi_right ? roi_right : right;
    if (left > right) {
        return false;
    }

    const int reference = expected_x < 0 ? width / 2 : expected_x;
    const int min_width = LINE_MIN_SEGMENT_WIDTH;
    /* 上界只用来挡掉"整条 ROI 都是黑的"这种帧：那既不是线也不是可用事件。 */
    const int max_width = line_geometry_positive_percent(width,
                                                         LINE_MAX_SEGMENT_WIDTH_PERCENT,
                                                         min_width + 2);
    int best_distance = INT_MAX;
    int best_start = 0;
    int best_end = 0;
    bool in_run = false;
    int run_start = 0;
    int row_threshold = cfg->threshold;
    const int background = row_background_luma(frame, cfg, y);
    if (background > LINE_LOCAL_CONTRAST_MIN) {
        const int local_threshold = background - LINE_LOCAL_CONTRAST_MIN;
        if (local_threshold < row_threshold) {
            row_threshold = local_threshold;
        }
    }
    if (row_threshold < 1) {
        row_threshold = 1;
    }

    for (int x = left; x <= right + 1; ++x) {
        bool dark = x <= right &&
                    pixel_is_dark(frame, cfg, x, y, row_threshold);
        /* A one-pixel JPEG/compression hole inside black tape must not split
         * the segment.  Only bridge a hole surrounded by dark pixels, never
         * an isolated dark sample. */
        if (!dark && x > left && x < right && LINE_BRIDGE_GAP_PIXELS == 1) {
            dark = pixel_is_dark(frame, cfg, x - 1, y, row_threshold) &&
                   pixel_is_dark(frame, cfg, x + 1, y, row_threshold);
        }
        if (dark && !in_run) {
            in_run = true;
            run_start = x;
        }
        if (!dark && in_run) {
            const int run_end = x - 1;
            const int run_width = run_end - run_start + 1;
            const int distance = abs((run_start + run_end) / 2 - reference);
            if (run_width >= min_width && run_width <= max_width &&
                distance < best_distance) {
                best_distance = distance;
                best_start = run_start;
                best_end = run_end;
            }
            in_run = false;
        }
    }

    if (best_distance == INT_MAX) {
        return false;
    }

    const int w_ref = width_reference < min_width ? min_width : width_reference;
    segment->center = (best_start + best_end) / 2;
    segment->width = best_end - best_start + 1;
    segment->ext_left = reference - best_start;
    segment->ext_right = best_end - reference;
    segment->clipped_left = best_start == left;
    segment->clipped_right = best_end == right;
    if (segment->width > LINE_WIDE_RATIO * w_ref) {
        const int open = LINE_WIDE_OPEN_RATIO * w_ref;
        const bool open_left = segment->ext_left >= open;
        const bool open_right = segment->ext_right >= open;
        if (open_left && open_right) {
            segment->kind = LINE_ROW_WIDE_BOTH;
        } else if (open_left) {
            segment->kind = LINE_ROW_WIDE_LEFT;
        } else if (open_right) {
            segment->kind = LINE_ROW_WIDE_RIGHT;
        }
    }
    return true;
}

static int average_points(const int16_t *xs, int first, int count)
{
    if (xs == NULL || count <= 0) {
        return 0;
    }
    int total = 0;
    for (int i = 0; i < count; ++i) {
        total += xs[first + i];
    }
    return total / count;
}

bool line_geometry_track(const uint8_t *frame,
                         const line_scan_cfg_t *cfg,
                         line_observation_t *observation)
{
    if (frame == NULL || cfg == NULL || observation == NULL) {
        return false;
    }
    const line_observation_t empty = {0};
    *observation = empty;
    observation->threshold = cfg->threshold;
    observation->corner_row_y = -1;
    observation->corner_x = -1;
    observation->scan_bottom_y = -1;
    const int width = line_geometry_scan_width(cfg);
    const int height = line_geometry_scan_height(cfg);
    if (width < 16 || height < 16 || cfg->threshold <= 0) {
        return false;
    }

    int top = height * LINE_ROI_TOP_PERCENT / 100;
    int roi_bottom = height * LINE_ROI_BOTTOM_PERCENT / 100;
    const int near_top = height * LINE_NEAR_TOP_PERCENT / 100;
    top = top < 0 ? 0 : top;
    roi_bottom = roi_bottom >= height ? height - 1 : roi_bottom;
    /* 首次获取和历史跟踪用同一个扫描底行。原来跟踪时从 92% 起扫，等于把
     * 离车最近的整整 10% 画面丢掉，"近点"其实一点也不近。 */
    const int bottom = roi_bottom - LINE_BOTTOM_SKIP_ROWS * LINE_ROW_STEP;
    const int seed_floor = bottom - LINE_SEED_MISS_ROWS * LINE_ROW_STEP;
    if (bottom <= top) {
        return false;
    }
    observation->scan_bottom_y = bottom;

    int half_window = line_geometry_positive_percent(width,
                                                    cfg->search_half_percent,
                                                    LINE_SEARCH_HALF_MIN);
    if (half_window > LINE_SEARCH_HALF_MAX) {
        half_window = LINE_SEARCH_HALF_MAX;
    }
    const int jump_limit = line_geometry_positive_percent(width,
                                                         LINE_MAX_CENTER_JUMP_PERCENT,
                                                         LINE_SEARCH_HALF_MIN * 2);

    int expected = cfg->use_history ? cfg->seed_x : width / 2;
    expected = expected < 0 ? 0 : (expected >= width ? width - 1 : expected);
    int w_ref = cfg->expected_width > 0 ? cfg->expected_width :
                line_geometry_positive_percent(width, LINE_WIDTH_FALLBACK_PERCENT,
                                               LINE_MIN_SEGMENT_WIDTH);
    int slope = 0;
    int point_count = 0;
    int misses = 0;
    int near_normal_rows = 0;
    int shape_event_rows = 0;
    int y = bottom;

    while (y >= top && point_count < LINE_SCAN_MAX_ROWS) {
        const bool seed_row = point_count == 0;
        const bool blind_seed = seed_row && !cfg->use_history;
        line_segment_t segment;
        if (!scan_segment(frame, cfg, y, blind_seed ? -1 : expected,
                          blind_seed ? width : half_window, w_ref, &segment)) {
            /* 底部若干行都允许没找到，之后每个空隙只容忍一行。 */
            if (seed_row && y > seed_floor) {
                y -= LINE_ROW_STEP;
                continue;
            }
            if (!seed_row && misses == 0) {
                ++misses;
                y -= LINE_ROW_STEP;
                continue;
            }
            break;
        }
        misses = 0;

        if (segment.kind != LINE_ROW_NORMAL) {
            /* 形状事件：质心不可用，记下方向和所在行就结束向上跟踪。
             * 双侧敞开 = 终点 T / 十字；单侧敞开 = 直角或锐角弯。 */
            observation->corner_row_y = y;
            observation->corner_x = segment.center;
            if (segment.kind == LINE_ROW_WIDE_BOTH) {
                observation->finish_candidate = point_count >= LINE_FINISH_STEM_ROWS &&
                                                y >= near_top;
            } else {
                /* corner_direction 是控制量，直接给出控制系方向。 */
                const int raw = segment.kind == LINE_ROW_WIDE_LEFT ? -1 : 1;
                observation->corner_direction = cfg->mirror_x ? -raw : raw;
                /* 锐角处底部几行可能仍是弯道横向投影，不能因第一行宽线
                 * 就结束跟踪；跳过少量形状事件，继续向上找真正的赛道段。 */
                if (point_count < LINE_MIN_VALID_ROWS &&
                    shape_event_rows < LINE_SEED_MISS_ROWS * 2) {
                    ++shape_event_rows;
                    y -= LINE_ROW_STEP;
                    continue;
                }
            }
            break;
        }

        const int predicted = point_count >= 2 ? expected + slope : expected;
        if (!seed_row && abs(segment.center - predicted) > jump_limit) {
            break;
        }
        if (cfg->corridor_x >= 0 &&
            abs(segment.center - cfg->corridor_x) > cfg->corridor_half) {
            break;
        }

        if (seed_row) {
            observation->near_width = segment.width;
            w_ref = (w_ref + segment.width) / 2;
            if (w_ref < LINE_MIN_SEGMENT_WIDTH) {
                w_ref = LINE_MIN_SEGMENT_WIDTH;
            }
        } else {
            slope = clamp_int(segment.center - expected, jump_limit);
        }
        observation->point_x[point_count] = (int16_t)segment.center;
        observation->point_y[point_count] = (int16_t)y;
        ++point_count;
        expected = segment.center;
        if (y >= near_top) {
            ++near_normal_rows;
        }
        y -= LINE_ROW_STEP;
    }

    observation->point_count = point_count;
    observation->valid_rows = (uint8_t)(point_count > 255 ? 255 : point_count);
    observation->near_normal_rows = (uint8_t)(near_normal_rows > 255 ? 255 :
                                              near_normal_rows);
    if (point_count > 0) {
        observation->seed_x = observation->point_x[0];
    }
    if (point_count < LINE_MIN_VALID_ROWS) {
        /* 锐角的近场横向投影会吃掉若干扫描行；只要已经有两行
         * 正常中心点，并且明确报告了靠近底部的单侧支路，就允许
         * 状态机进入 CORNER/TURN。普通丢线仍严格要求四行。 */
        const int corner_near_y = height * LINE_NEAR_BOTTOM_PERCENT / 100;
        if (observation->corner_direction == 0 ||
            observation->corner_row_y < corner_near_y ||
            point_count < LINE_CORNER_MIN_VALID_ROWS) {
            return false;
        }
    }

    const int near_count = point_count < LINE_NEAR_ROWS ? point_count : LINE_NEAR_ROWS;
    const int near_center = average_points(observation->point_x, 0, near_count);
    const int near_error = line_geometry_error(near_center, width, cfg->mirror_x);
    /* heading 取固定行号并按实际基线归一化：同一个物理姿态必须给出同一个
     * 数值。原来 far 取 points[] 末端，基线随跟踪长度变化，前馈增益会漂移，
     * point_count==4 时甚至恒等于 0。 */
    int heading_index = point_count - 1;
    if (heading_index > LINE_HEADING_ROWS) {
        heading_index = LINE_HEADING_ROWS;
    }
    int heading = 0;
    if (heading_index > 0) {
        const int local = line_geometry_error(observation->point_x[heading_index],
                                              width, cfg->mirror_x);
        heading = (near_error - local) * LINE_HEADING_ROWS / heading_index;
    }
    int far_index = point_count - 1;
    if (far_index > LINE_FAR_ROWS) {
        far_index = LINE_FAR_ROWS;
    }

    observation->candidate = true;
    observation->near_line_visible = near_normal_rows >= 2;
    observation->lateral_error = near_error;
    observation->heading_error = clamp_int(heading, 100);
    observation->far_error = line_geometry_error(observation->point_x[far_index],
                                                width, cfg->mirror_x);
    observation->confidence = (uint8_t)((point_count * 100) /
                                       (LINE_MIN_VALID_ROWS + 6));
    if (observation->confidence > 100) {
        observation->confidence = 100;
    }
    return true;
}
