#include "line_detect.h"

/* 窗口中心是唯一的跨帧状态：它让检测器跟住上一帧的线，从而自动忽略画面
 * 别处的暗块（障碍物木板、板边阴影）。丢线时窗口放宽而不是清零，避免
 * "一帧没看到就退回全画面搜索"导致的跳线。 */
static int s_window_center = -1;
static int s_window_half;

void line_detect_reset(void)
{
    s_window_center = -1;
    s_window_half = 0;
}

static uint8_t rgb565_luma(const uint8_t *pixel)
{
    const uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    const uint8_t red = (uint8_t)(((value >> 11) & 0x1fU) * 255U / 31U);
    const uint8_t green = (uint8_t)(((value >> 5) & 0x3fU) * 255U / 63U);
    const uint8_t blue = (uint8_t)((value & 0x1fU) * 255U / 31U);
    return (uint8_t)((77U * red + 150U * green + 29U * blue) >> 8);
}

static int clamp_int(int value, int low, int high)
{
    if (value < low) {
        return low;
    }
    return value > high ? high : value;
}

/*
 * 一个条带内暗像素的水平质心。窗口外的像素完全不看。
 * 返回是否可信；*fill 是暗像素占取样点的千分比。
 */
static bool band_centroid(const uint8_t *frame, int width,
                          int top, int bottom, int lo, int hi,
                          uint8_t threshold, int *centre, int *fill)
{
    long sum_x = 0;
    long dark = 0;
    long samples = 0;
    for (int y = top; y <= bottom; ++y) {
        const uint8_t *row = frame + (size_t)y * (size_t)width * 2U;
        for (int x = lo; x <= hi; ++x) {
            ++samples;
            if (rgb565_luma(row + (size_t)x * 2U) <= threshold) {
                sum_x += x;
                ++dark;
            }
        }
    }
    if (samples == 0) {
        *fill = 0;
        return false;
    }
    *fill = (int)(dark * 1000 / samples);
    if (*fill < LINE_MIN_FILL_PERMILLE || *fill > LINE_MAX_FILL_PERMILLE) {
        return false;
    }
    *centre = (int)(sum_x / dark);
    return true;
}

bool line_detect_run(const uint8_t *frame, int width, int height,
                     uint8_t threshold, line_obs_t *out)
{
    if (out == NULL) {
        return false;
    }
    *out = (line_obs_t){0};
    if (frame == NULL || width < 16 || height < 16 || threshold == 0) {
        return false;
    }

    if (s_window_center < 0) {
        s_window_center = width / 2;
        s_window_half = width * LINE_WINDOW_HALF_PERCENT / 100;
    }
    const int half_max = width * LINE_WINDOW_MAX_PERCENT / 100;
    const int lo = clamp_int(s_window_center - s_window_half, 0, width - 1);
    const int hi = clamp_int(s_window_center + s_window_half, 0, width - 1);
    out->window_lo = lo;
    out->window_hi = hi;

    const int near_top = clamp_int(height * LINE_NEAR_TOP_PERCENT / 100, 0, height - 1);
    const int near_bottom = clamp_int(height * LINE_NEAR_BOTTOM_PERCENT / 100, 0, height - 1);
    const int far_top = clamp_int(height * LINE_FAR_TOP_PERCENT / 100, 0, height - 1);
    const int far_bottom = clamp_int(height * LINE_FAR_BOTTOM_PERCENT / 100, 0, height - 1);

    int near_x = 0;
    int far_x = 0;
    const bool near_ok = band_centroid(frame, width, near_top, near_bottom,
                                       lo, hi, threshold, &near_x, &out->near_fill);
    const bool far_ok = band_centroid(frame, width, far_top, far_bottom,
                                      lo, hi, threshold, &far_x, &out->far_fill);

    if (!near_ok) {
        /* 近场没线：放宽窗口，但保留中心，下一帧还从这附近找。 */
        s_window_half += width * LINE_WINDOW_GROW_PERCENT / 100;
        if (s_window_half > half_max) {
            s_window_half = half_max;
        }
        return false;
    }

    s_window_center = near_x;
    s_window_half = width * LINE_WINDOW_HALF_PERCENT / 100;

    out->found = true;
    out->near_x = near_x;
    out->far_x = far_ok ? far_x : near_x;
    out->far_valid = far_ok;
    out->lateral = (near_x - width / 2) * 1000 / width;
    out->heading = far_ok ? (far_x - near_x) * 1000 / width : 0;
    return true;
}
