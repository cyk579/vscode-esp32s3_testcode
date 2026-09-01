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

static bool pixel_is_dark(const uint8_t *pixel, const line_scan_cfg_t *cfg)
{
    if (line_geometry_luma(pixel) > cfg->threshold) {
        return false;
    }
    if (cfg->saturation_guard && rgb565_saturation(pixel) > cfg->saturation_max) {
        return false;
    }
    return true;
}

static bool scan_segment(const uint8_t *frame,
                         const line_scan_cfg_t *cfg,
                         int y,
                         int expected_x,
                         int half_window,
                         line_segment_t *segment)
{
    const int width = (int)cfg->width;
    if (frame == NULL || segment == NULL || y < 0 || y >= (int)cfg->height) {
        return false;
    }

    const line_segment_t empty = {0, 0, 0, 0, false, false, false, LINE_ROW_NORMAL};
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
    const int max_width = line_geometry_positive_percent(width,
                                                         LINE_MAX_SEGMENT_WIDTH_PERCENT,
                                                         min_width + 2);
    /* Finish width is relative to the local window, not the whole image. */
    const int window_width = right - left + 1;
    const int finish_width = line_geometry_positive_percent(window_width,
                                                            LINE_FINISH_WIDTH_PERCENT,
                                                            LINE_MIN_SEGMENT_WIDTH + 1);
    int best_distance = INT_MAX;
    line_segment_t best = empty;
    int best_wide_distance = INT_MAX;
    line_segment_t best_wide = empty;
    bool in_run = false;
    int run_start = 0;

    for (int x = left; x <= right + 1; ++x) {
        const bool dark = x <= right &&
                          pixel_is_dark(frame + (((size_t)y * cfg->width + (size_t)x) * 2),
                                        cfg);
        if (dark && !in_run) {
            in_run = true;
            run_start = x;
        }
        if (!dark && in_run) {
            const int run_end = x - 1;
            const int run_width = run_end - run_start + 1;
            const int run_center = (run_start + run_end) / 2;
            const int distance = abs(run_center - reference);
            if (run_width >= finish_width && distance < best_wide_distance) {
                best_wide_distance = distance;
                best_wide.center = run_center;
                best_wide.width = run_width;
                best_wide.ext_left = reference - run_start;
                best_wide.ext_right = run_end - reference;
                best_wide.clipped_left = run_start == left;
                best_wide.clipped_right = run_end == right;
                best_wide.wide = true;
            }
            if (run_width >= min_width && run_width <= max_width &&
                distance < best_distance) {
                best_distance = distance;
                best.center = run_center;
                best.width = run_width;
                best.ext_left = reference - run_start;
                best.ext_right = run_end - reference;
                best.clipped_left = run_start == left;
                best.clipped_right = run_end == right;
                best.wide = run_width >= finish_width;
            }
            in_run = false;
        }
    }

    if (best_distance == INT_MAX) {
        /* A broad local finish bar is still a valid seed for confirmation. */
        if (best_wide_distance == INT_MAX) {
            return false;
        }
        best = best_wide;
    }
    *segment = best;
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

/* 旧的支路探测：只在中心线最顶端点两侧各扫 3 行。对本赛道来说，唯一真正
 * 的"支路"是终点 T 口；所有拐角都是单侧事件，不该走这条路径。 */
static int branch_hint(const uint8_t *frame,
                       const line_scan_cfg_t *cfg,
                       const int16_t *xs,
                       const int16_t *ys,
                       int point_count)
{
    if (point_count < LINE_MIN_VALID_ROWS) {
        return 0;
    }

    const int end_x = xs[point_count - 1];
    const int end_y = ys[point_count - 1];
    const int offset = line_geometry_positive_percent((int)cfg->width,
                                                      LINE_BRANCH_OFFSET_PERCENT,
                                                      LINE_SEARCH_HALF_MIN * 2);
    const int half = line_geometry_positive_percent((int)cfg->width,
                                                    LINE_BRANCH_WINDOW_PERCENT,
                                                    LINE_SEARCH_HALF_MIN);
    const int min_offset = offset * LINE_BRANCH_MIN_OFFSET_PERCENT /
                           LINE_BRANCH_OFFSET_PERCENT;
    int found_direction = 0;
    for (int direction = -1; direction <= 1; direction += 2) {
        int hits = 0;
        for (int i = 0; i < LINE_BRANCH_ROWS; ++i) {
            int y = end_y + (i - 1) * LINE_ROW_STEP;
            if (y < 0) {
                y = 0;
            }
            if (y >= (int)cfg->height) {
                y = (int)cfg->height - 1;
            }
            line_segment_t branch;
            if (scan_segment(frame, cfg, y, end_x + direction * offset, half,
                             &branch) &&
                abs(branch.center - end_x) >= min_offset) {
                ++hits;
            }
        }
        if (hits >= 2) {
            if (found_direction != 0 && found_direction != direction) {
                return 0;
            }
            found_direction = direction;
        }
    }

    return found_direction;
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
    observation->scan_bottom_y = -1;
    const int width = (int)cfg->width;
    const int height = (int)cfg->height;
    if (width < 16 || height < 16 || cfg->threshold <= 0) {
        return false;
    }

    int top = height * LINE_ROI_TOP_PERCENT / 100;
    int roi_bottom = height * LINE_ROI_BOTTOM_PERCENT / 100;
    const int near_bottom = height * LINE_NEAR_BOTTOM_PERCENT / 100 -
                            LINE_BOTTOM_SKIP_ROWS * LINE_ROW_STEP;
    const int near_top = height * LINE_NEAR_TOP_PERCENT / 100;
    top = top < 0 ? 0 : top;
    roi_bottom = roi_bottom >= height ? height - 1 : roi_bottom;
    const int bottom = cfg->use_history ? near_bottom :
                       roi_bottom - LINE_BOTTOM_SKIP_ROWS * LINE_ROW_STEP;
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

    int expected = cfg->use_history ? cfg->seed_x : width / 2;
    expected = expected < 0 ? 0 : (expected >= width ? width - 1 : expected);
    int point_count = 0;
    int misses = 0;
    int wide_near_rows = 0;
    int y = bottom;

    const int jump_limit = line_geometry_positive_percent(width,
                                                         LINE_MAX_CENTER_JUMP_PERCENT,
                                                         LINE_SEARCH_HALF_MIN * 2);
    while (y >= top && point_count < LINE_SCAN_MAX_ROWS) {
        const bool seed_row = point_count == 0 && !cfg->use_history;
        line_segment_t segment;
        const bool found = scan_segment(frame, cfg, y,
                                        seed_row ? -1 : expected,
                                        seed_row ? width : half_window,
                                        &segment);
        if (!found) {
            /* Initial acquisition checks a few bottom rows, still using a
             * full-width nearest-to-centre seed search on each row. */
            if (seed_row && y > near_bottom) {
                y -= LINE_ROW_STEP;
                continue;
            }
            if (point_count > 0 && misses == 0) {
                ++misses;
                y -= LINE_ROW_STEP;
                continue;
            }
            break;
        }
        misses = 0;

        if (point_count > 0 && abs(segment.center - expected) > jump_limit) {
            break;
        }
        if (cfg->corridor_x >= 0 &&
            abs(segment.center - cfg->corridor_x) > cfg->corridor_half) {
            break;
        }

        if (point_count == 0) {
            observation->near_width = segment.width;
        }
        observation->point_x[point_count] = (int16_t)segment.center;
        observation->point_y[point_count] = (int16_t)y;
        ++point_count;
        expected = segment.center;
        if (segment.wide && y >= near_top && y <= near_bottom) {
            ++wide_near_rows;
        }
        y -= LINE_ROW_STEP;
    }

    observation->point_count = point_count;
    observation->valid_rows = (uint8_t)(point_count > 255 ? 255 : point_count);
    observation->finish_candidate = wide_near_rows >= LINE_FINISH_MIN_ROWS;
    if (point_count > 0) {
        observation->seed_x = observation->point_x[0];
    }
    if (point_count < LINE_MIN_VALID_ROWS) {
        return false;
    }

    const int near_count = point_count < 4 ? point_count : 4;
    const int far_count = near_count;
    const int near_center = average_points(observation->point_x, 0, near_count);
    const int far_center = average_points(observation->point_x,
                                         point_count - far_count, far_count);
    observation->candidate = true;
    observation->old_line_visible = point_count >= LINE_MIN_VALID_ROWS + 1;
    observation->lateral_error = line_geometry_error(near_center, width, cfg->mirror_x);
    observation->heading_error = observation->lateral_error -
                                 line_geometry_error(far_center, width, cfg->mirror_x);
    observation->confidence = (uint8_t)((point_count * 100) /
                                       (LINE_MIN_VALID_ROWS + 6));
    if (observation->confidence > 100) {
        observation->confidence = 100;
    }
    observation->corner_direction = branch_hint(frame, cfg, observation->point_x,
                                                observation->point_y, point_count);
    if (observation->corner_direction != 0) {
        observation->corner_row_y = observation->point_y[point_count - 1];
    }
    if (observation->finish_candidate &&
        abs(observation->seed_x - (cfg->use_history ? cfg->seed_x : width / 2)) >
            half_window) {
        observation->finish_candidate = false;
    }
    return true;
}
