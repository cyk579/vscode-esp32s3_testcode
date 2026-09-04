#include "line_geometry.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define FRAME_W 120
#define FRAME_H 80
#define LINE_X (FRAME_W / 2)
#define LINE_WIDTH 6
#define THRESHOLD 120

static uint8_t frame[FRAME_W * FRAME_H * 2];

static void put_pixel(int x, int y, int r, int g, int b)
{
    if (x < 0 || y < 0 || x >= FRAME_W || y >= FRAME_H) {
        return;
    }
    const uint16_t value = (uint16_t)(((r & 0xf8) << 8) |
                                      ((g & 0xfc) << 3) |
                                      ((b & 0xf8) >> 3));
    uint8_t *pixel = frame + (((size_t)y * FRAME_W + (size_t)x) * 2U);
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

static void draw_vertical_line(void)
{
    const int half = LINE_WIDTH / 2;
    for (int y = 0; y < FRAME_H; ++y) {
        for (int x = LINE_X - half; x <= LINE_X + half; ++x) {
            put_pixel(x, y, 0, 0, 0);
        }
    }
}

static void draw_corner(int direction, int y)
{
    const int half = LINE_WIDTH / 2;
    const int end = direction < 0 ? 8 : FRAME_W - 9;
    for (int row = y - half; row <= y + half; ++row) {
        if (direction < 0) {
            for (int x = end; x <= LINE_X + half; ++x) {
                put_pixel(x, row, 0, 0, 0);
            }
        } else {
            for (int x = LINE_X - half; x <= end; ++x) {
                put_pixel(x, row, 0, 0, 0);
            }
        }
    }
}

static line_scan_cfg_t base_cfg(void)
{
    line_scan_cfg_t cfg = {0};
    cfg.width = FRAME_W;
    cfg.height = FRAME_H;
    cfg.threshold = THRESHOLD;
    cfg.use_history = false;
    cfg.seed_x = LINE_X;
    cfg.search_half_percent = LINE_SEARCH_HALF_PERCENT;
    cfg.corridor_x = -1;
    cfg.expected_width = LINE_WIDTH;
    cfg.rotation = LINE_ROTATE_0;
    return cfg;
}

static int check_corner(int direction, int y)
{
    line_observation_t observation;
    line_scan_cfg_t cfg = base_cfg();
    fill_frame(240, 240, 240);
    draw_vertical_line();
    draw_corner(direction, y);
    const bool candidate = line_geometry_track(frame, &cfg, &observation);
    printf("corner %s y=%d candidate=%d rows=%u event=%+d@%d\n",
           direction < 0 ? "left" : "right", y, candidate,
           (unsigned)observation.valid_rows, observation.corner_direction,
           observation.corner_row_y);
    return candidate && observation.corner_direction == direction ? 0 : 1;
}

/* The control frame is 160x120 for a 320x240 camera mode and 120x80 for
 * 480x320.  The same physical tilt must yield the same heading at both
 * sizes, otherwise the yaw gain silently changes with the negotiated mode. */
#define SCALE_MAX_W 160
#define SCALE_MAX_H 120
static uint8_t scale_frame[SCALE_MAX_W * SCALE_MAX_H * 2];

static int tilt_heading(int w, int h)
{
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            /* Line from bottom centre to 1/4 width at the top: identical
             * geometry at any resolution.  Tape is 5% of the width. */
            const int cx = w / 2 - (h - 1 - y) * (w / 4) / (h - 1);
            const int dark = (x - cx) * (x - cx) * 400 <= w * w;
            const uint16_t v = dark ? 0 : (uint16_t)0xf79e;
            uint8_t *p = scale_frame + (((size_t)y * w + (size_t)x) * 2U);
            p[0] = (uint8_t)(v >> 8);
            p[1] = (uint8_t)v;
        }
    }
    line_scan_cfg_t cfg = base_cfg();
    cfg.width = (uint16_t)w;
    cfg.height = (uint16_t)h;
    cfg.seed_x = w / 2;
    cfg.expected_width = w / 20;
    line_observation_t obs;
    const bool candidate = line_geometry_track(scale_frame, &cfg, &obs);
    printf("tilt %dx%d candidate=%d rows=%u ey=%d eth=%d far=%d\n", w, h,
           candidate, (unsigned)obs.valid_rows, obs.lateral_error,
           obs.heading_error, obs.far_error);
    return candidate ? obs.heading_error : 9999;
}

int main(void)
{
    int failures = 0;
    {
        const int small = tilt_heading(120, 80);
        const int large = tilt_heading(160, 120);
        if (small >= 0 || large >= 0 || small - large > 3 || large - small > 3) {
            printf("FAIL: heading scale differs between frame sizes\n");
            ++failures;
        }
    }
    failures += check_corner(-1, 31);
    failures += check_corner(1, 31);
    failures += check_corner(-1, 17);
    failures += check_corner(1, 17);

    line_observation_t straight;
    line_scan_cfg_t cfg = base_cfg();
    fill_frame(240, 240, 240);
    draw_vertical_line();
    const bool candidate = line_geometry_track(frame, &cfg, &straight);
    printf("straight candidate=%d rows=%u event=%+d\n",
           candidate, (unsigned)straight.valid_rows,
           straight.corner_direction);
    if (!candidate || straight.corner_direction != 0) {
        ++failures;
    }

    printf("low-resolution corner regression: %s\n",
           failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
