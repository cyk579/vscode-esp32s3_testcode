#include "line_mixer.h"

#include <stdlib.h>

static int magnitude_max(int a, int b, int d)
{
    int high = abs(a);
    if (abs(b) > high) {
        high = abs(b);
    }
    if (abs(d) > high) {
        high = abs(d);
    }
    return high;
}

void line_mixer_solve(int forward, int turn, int lat,
                      const line_mixer_cfg_t *cfg,
                      line_mixer_out_t *out)
{
    if (out == NULL || cfg == NULL) {
        return;
    }
    const int trim_a = cfg->trim_a > 0 ? cfg->trim_a : 100;
    const int trim_d = cfg->trim_d > 0 ? cfg->trim_d : 100;
    const int forward_a = forward * trim_a / 100;
    const int forward_d = forward * trim_d / 100;

    int a = -forward_a - turn + lat;
    int d = forward_d - turn + lat;
    int b = turn + 2 * lat;

    out->scaled = false;
    out->dropped = false;

    const int ceiling = cfg->ceiling > 0 ? cfg->ceiling : 100;
    const int high = magnitude_max(a, b, d);
    if (high > ceiling) {
        a = a * ceiling / high;
        b = b * ceiling / high;
        d = d * ceiling / high;
        out->scaled = true;
    }

    /* 起转值处理放在缩放之后：先保证方向正确，再决定哪些分量根本推不动。 */
    if (a != 0 && abs(a) < cfg->floor_ad) {
        a = 0;
        out->dropped = true;
    }
    if (d != 0 && abs(d) < cfg->floor_ad) {
        d = 0;
        out->dropped = true;
    }
    if (b != 0 && abs(b) < cfg->floor_b) {
        b = 0;
        out->dropped = true;
    }

    out->a = a;
    out->b = b;
    out->d = d;
}

void line_mixer_body(int a, int b, int d, int *forward, int *turn, int *lat)
{
    /* a = -f - t + l ; d = f - t + l ; b = t + 2l  的精确逆解。 */
    const int sum = a + d;
    const int lateral = (2 * b + sum) / 6;
    if (forward != NULL) {
        *forward = (d - a) / 2;
    }
    if (lat != NULL) {
        *lat = lateral;
    }
    if (turn != NULL) {
        *turn = lateral - sum / 2;
    }
}
