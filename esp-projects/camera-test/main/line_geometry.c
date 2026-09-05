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

static int max_int(int a, int b)
{
    return a > b ? a : b;
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

/* The reference tracker is luma-only.  Keep this optional chroma check for
 * this vehicle because hands and coloured balls are common distractors. */
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
    if (image_width < 2) {
        return 0;
    }
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

static int pixel_luma_at(const uint8_t *frame, const line_scan_cfg_t *cfg,
                         int sx, int sy)
{
    int bx = 0;
    int by = 0;
    line_geometry_map(cfg, sx, sy, &bx, &by);
    if (bx < 0 || by < 0 || bx >= (int)cfg->width || by >= (int)cfg->height) {
        return 255;
    }
    return line_geometry_luma(frame + (((size_t)by * cfg->width +
                                        (size_t)bx) * 2U));
}

static bool pixel_is_dark(const uint8_t *frame, const line_scan_cfg_t *cfg,
                          int sx, int sy)
{
    int bx = 0;
    int by = 0;
    line_geometry_map(cfg, sx, sy, &bx, &by);
    if (bx < 0 || by < 0 || bx >= (int)cfg->width || by >= (int)cfg->height) {
        return false;
    }
    const uint8_t *pixel = frame + (((size_t)by * cfg->width +
                                     (size_t)bx) * 2U);
    const int threshold = cfg->threshold < LINE_ABSOLUTE_BLACK_MAX_LUMA ?
                          cfg->threshold : LINE_ABSOLUTE_BLACK_MAX_LUMA;
    if (line_geometry_luma(pixel) > threshold) {
        return false;
    }
    return !cfg->saturation_guard ||
           rgb565_saturation(pixel) <= cfg->saturation_max;
}

typedef struct {
    int start;
    int end;
    int center;
    int width;
    int luma;
} raw_run_t;

static bool run_is_better(const raw_run_t *run, const raw_run_t *best,
                          int reference)
{
    const int distance = abs(run->center - reference);
    const int best_distance = abs(best->center - reference);
    return distance < best_distance ||
           (distance == best_distance && run->luma < best->luma);
}

/* Find one continuous dark segment.  Position relative to the predicted
 * track wins; darkness is only a tie breaker. */
static bool find_run(const uint8_t *frame, const line_scan_cfg_t *cfg, int y,
                     int left, int right, int reference, int max_width,
                     raw_run_t *result)
{
    if (frame == NULL || cfg == NULL || result == NULL || left > right) {
        return false;
    }
    const int scan_width = line_geometry_scan_width(cfg);
    const int min_width = LINE_MIN_SEGMENT_WIDTH;
    left = left < 0 ? 0 : left;
    right = right >= scan_width ? scan_width - 1 : right;
    if (left > right || y < 0 || y >= line_geometry_scan_height(cfg)) {
        return false;
    }

    bool in_run = false;
    int run_start = 0;
    raw_run_t best = {0};
    bool found = false;
    for (int x = left; x <= right + 1; ++x) {
        bool dark = x <= right && pixel_is_dark(frame, cfg, x, y);
        if (!dark && x > left && x < right && LINE_BRIDGE_GAP_PIXELS == 1) {
            dark = pixel_is_dark(frame, cfg, x - 1, y) &&
                   pixel_is_dark(frame, cfg, x + 1, y);
        }
        if (dark && !in_run) {
            in_run = true;
            run_start = x;
        }
        if (!dark && in_run) {
            const int run_end = x - 1;
            const int run_width = run_end - run_start + 1;
            if (run_width >= min_width && run_width <= max_width) {
                raw_run_t run = {
                    .start = run_start,
                    .end = run_end,
                    .center = (run_start + run_end) / 2,
                    .width = run_width,
                    .luma = (pixel_luma_at(frame, cfg, run_start, y) +
                             pixel_luma_at(frame, cfg,
                                           (run_start + run_end) / 2, y) +
                             pixel_luma_at(frame, cfg, run_end, y)) / 3,
                };
                if (!found || run_is_better(&run, &best, reference)) {
                    best = run;
                    found = true;
                }
            }
            in_run = false;
        }
    }
    if (found) {
        *result = best;
    }
    return found;
}

/* Look for a wide segment joined to the predicted line.  This is the only
 * shape test: it distinguishes a one-sided branch from a two-sided finish
 * bar, while its centroid is never used as a steering point. */
static bool find_branch(const uint8_t *frame, const line_scan_cfg_t *cfg, int y,
                        int expected, int width_reference, raw_run_t *result)
{
    const int scan_width = line_geometry_scan_width(cfg);
    const int roi_left = scan_width * LINE_ROI_LEFT_PERCENT / 100;
    const int roi_right = scan_width * LINE_ROI_RIGHT_PERCENT / 100 - 1;
    const int reference_width = max_int(width_reference, LINE_MIN_SEGMENT_WIDTH);
    const int wide_min = max_int(reference_width * LINE_WIDE_RATIO,
                                 reference_width + 6);
    const int overlap = max_int(reference_width, 3);
    if (result == NULL) {
        return false;
    }

    bool in_run = false;
    int run_start = 0;
    raw_run_t best = {0};
    bool found = false;
    for (int x = roi_left; x <= roi_right + 1; ++x) {
        bool dark = x <= roi_right && pixel_is_dark(frame, cfg, x, y);
        if (!dark && x > roi_left && x < roi_right &&
            LINE_BRIDGE_GAP_PIXELS == 1) {
            dark = pixel_is_dark(frame, cfg, x - 1, y) &&
                   pixel_is_dark(frame, cfg, x + 1, y);
        }
        if (dark && !in_run) {
            in_run = true;
            run_start = x;
        }
        if (!dark && in_run) {
            const int run_end = x - 1;
            const int run_width = run_end - run_start + 1;
            const bool joins_expected = run_start <= expected + overlap &&
                                        run_end >= expected - overlap;
            if (joins_expected && run_width >= wide_min) {
                raw_run_t run = {
                    .start = run_start,
                    .end = run_end,
                    .center = (run_start + run_end) / 2,
                    .width = run_width,
                    .luma = (pixel_luma_at(frame, cfg, run_start, y) +
                             pixel_luma_at(frame, cfg,
                                           (run_start + run_end) / 2, y) +
                             pixel_luma_at(frame, cfg, run_end, y)) / 3,
                };
                if (!found || run.width > best.width ||
                    (run.width == best.width && run.luma < best.luma)) {
                    best = run;
                    found = true;
                }
            }
            in_run = false;
        }
    }
    if (found) {
        *result = best;
    }
    return found;
}

static int rows_for_permille(int height, int permille)
{
    const int rows = height * permille / (1000 * LINE_ROW_STEP);
    return rows < 1 ? 1 : rows;
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
    *observation = (line_observation_t){0};
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
    int bottom = height * LINE_ROI_BOTTOM_PERCENT / 100;
    top = top < 0 ? 0 : top;
    bottom = bottom >= height ? height - 1 : bottom;
    bottom -= LINE_BOTTOM_SKIP_ROWS * LINE_ROW_STEP;
    if (bottom <= top) {
        return false;
    }
    observation->scan_bottom_y = bottom;

    const int roi_left = width * LINE_ROI_LEFT_PERCENT / 100;
    const int roi_right = width * LINE_ROI_RIGHT_PERCENT / 100 - 1;
    int half_window = line_geometry_positive_percent(width,
                                                     cfg->search_half_percent,
                                                     LINE_SEARCH_HALF_MIN);
    if (half_window > LINE_SEARCH_HALF_MAX) {
        half_window = LINE_SEARCH_HALF_MAX;
    }
    const int jump_limit = line_geometry_positive_percent(
        width, LINE_MAX_CENTER_JUMP_PERCENT, LINE_SEARCH_HALF_MIN * 2);
    int expected = cfg->use_history ? cfg->seed_x : width / 2;
    expected = expected < roi_left ? roi_left : expected;
    expected = expected > roi_right ? roi_right : expected;
    int width_reference = cfg->expected_width > 0 ? cfg->expected_width :
                          line_geometry_positive_percent(
                              width, LINE_WIDTH_FALLBACK_PERCENT,
                              LINE_MIN_SEGMENT_WIDTH);
    int slope = 0;
    int point_count = 0;
    int misses = 0;
    int near_normal_rows = 0;
    const int near_top = height * LINE_NEAR_TOP_PERCENT / 100;
    const int seed_floor = bottom - LINE_SEED_MISS_ROWS * LINE_ROW_STEP;
    int y = bottom;

    while (y >= top && point_count < LINE_SCAN_MAX_ROWS) {
        const bool seed_row = point_count == 0;
        const bool blind_seed = seed_row && !cfg->use_history;
        const int reference = seed_row ? expected : expected + slope;
        raw_run_t branch;
        if (!seed_row && point_count >= LINE_CORNER_MIN_VALID_ROWS &&
            find_branch(frame, cfg, y, reference, width_reference, &branch)) {
            const int ext_left = expected - branch.start;
            const int ext_right = branch.end - expected;
            const int open = max_int(width_reference * LINE_WIDE_OPEN_RATIO,
                                     width_reference + 2);
            observation->corner_row_y = y;
            observation->corner_x = branch.center;
            if (ext_left >= open && ext_right >= open) {
                observation->finish_candidate = point_count >=
                                                LINE_FINISH_STEM_ROWS &&
                                                y >= near_top;
            } else if (ext_left >= open) {
                observation->corner_direction = cfg->mirror_x ? 1 : -1;
            } else if (ext_right >= open) {
                observation->corner_direction = cfg->mirror_x ? -1 : 1;
            } else {
                observation->corner_row_y = -1;
                observation->corner_x = -1;
            }
            if (observation->finish_candidate ||
                observation->corner_direction != 0) {
                break;
            }
        }

        int left = blind_seed ? roi_left : reference - half_window;
        int right = blind_seed ? roi_right : reference + half_window;
        left = left < roi_left ? roi_left : left;
        right = right > roi_right ? roi_right : right;
        const int normal_max_width = max_int(
            line_geometry_positive_percent(width, LINE_MAX_SEGMENT_WIDTH_PERCENT,
                                           LINE_MIN_SEGMENT_WIDTH + 2),
            width_reference * LINE_WIDE_RATIO - 1);
        raw_run_t run;
        if (!find_run(frame, cfg, y, left, right, reference,
                      normal_max_width, &run)) {
            if (seed_row && y > seed_floor) {
                y -= LINE_ROW_STEP;
                continue;
            }
            if (!seed_row && misses < LINE_TRACK_MISS_ROWS) {
                ++misses;
                y -= LINE_ROW_STEP;
                continue;
            }
            break;
        }
        misses = 0;

        const int predicted = seed_row ? expected : expected + slope;
        if (!seed_row && abs(run.center - predicted) > jump_limit) {
            break;
        }
        if (cfg->corridor_x >= 0 &&
            abs(run.center - cfg->corridor_x) > cfg->corridor_half) {
            break;
        }

        if (seed_row) {
            observation->near_width = run.width;
            width_reference = (width_reference + run.width) / 2;
            width_reference = max_int(width_reference, LINE_MIN_SEGMENT_WIDTH);
        } else {
            const int delta = run.center - expected;
            slope = (slope * 2 + clamp_int(delta, jump_limit)) / 3;
        }
        observation->point_x[point_count] = (int16_t)run.center;
        observation->point_y[point_count] = (int16_t)y;
        ++point_count;
        expected = run.center;
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
        const int corner_near_y = height * 70 / 100;
        if (observation->corner_direction == 0 &&
            !observation->finish_candidate) {
            return false;
        }
        if (observation->corner_row_y < corner_near_y ||
            point_count < LINE_CORNER_MIN_VALID_ROWS) {
            return false;
        }
    }

    const int near_rows = rows_for_permille(height, LINE_NEAR_ROWS_PERMILLE);
    const int heading_rows = rows_for_permille(height, LINE_HEADING_ROWS_PERMILLE);
    const int far_rows = rows_for_permille(height, LINE_FAR_ROWS_PERMILLE);
    const int near_count = point_count < near_rows ? point_count : near_rows;
    const int near_center = average_points(observation->point_x, 0, near_count);
    const int near_error = line_geometry_error(near_center, width, cfg->mirror_x);

    int heading_index = point_count - 1;
    if (heading_index > heading_rows) {
        heading_index = heading_rows;
    }
    int heading = 0;
    if (heading_index > 0) {
        const int far_error = line_geometry_error(
            observation->point_x[heading_index], width, cfg->mirror_x);
        heading = (near_error - far_error) * heading_rows / heading_index;
    }
    int far_index = point_count - 1;
    if (far_index > far_rows) {
        far_index = far_rows;
    }

    observation->candidate = true;
    observation->near_line_visible = near_normal_rows >= 2;
    observation->lateral_error = near_error;
    observation->heading_error = clamp_int(heading, 100);
    observation->far_error = line_geometry_error(observation->point_x[far_index],
                                                 width, cfg->mirror_x);
    int confidence = (point_count * 100) / (LINE_MIN_VALID_ROWS + 6);
    if (confidence > 100) {
        confidence = 100;
    }
    observation->confidence = (uint8_t)confidence;
    return true;
}
