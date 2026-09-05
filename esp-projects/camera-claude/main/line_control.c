#include "line_control.h"

#include <stddef.h>

static int clamp_int(int v, int limit)
{
    if (v > limit) { return limit; }
    if (v < -limit) { return -limit; }
    return v;
}

static int abs_int(int v)
{
    return v < 0 ? -v : v;
}

static int deadband(int v, int band)
{
    if (v > -band && v < band) { return 0; }
    return v;
}

void line_control_follow(int lateral, int heading, line_cmd_t *out)
{
    if (out == NULL) { return; }

    const int hd = deadband(heading, LINE_DEADBAND_HEADING);
    const int lt = deadband(lateral, LINE_DEADBAND_LATERAL);

    /* 线向右倾 -> 车头要往右拧 -> turn 取负 */
    out->turn = clamp_int(-LINE_GAIN_YAW * hd / 1000, LINE_TURN_MAX);

    /* 线在右边 -> 车整体右移 -> lat 取正 */
    if (abs_int(hd) < LINE_STRAFE_HEADING_LIMIT) {
        out->lat = clamp_int(LINE_GAIN_LAT * lt / 1000, LINE_LAT_MAX);
    } else {
        out->lat = 0;
    }

    /* 越歪越慢，在 DEADBAND..STRAFE_LIMIT 之间线性从 CRUISE 降到 CORNER。 */
    const int mag = abs_int(hd);
    if (mag <= LINE_DEADBAND_HEADING) {
        out->forward = LINE_FORWARD_CRUISE;
    } else if (mag >= LINE_STRAFE_HEADING_LIMIT) {
        out->forward = LINE_FORWARD_CORNER;
    } else {
        const int span = LINE_STRAFE_HEADING_LIMIT - LINE_DEADBAND_HEADING;
        const int drop = (LINE_FORWARD_CRUISE - LINE_FORWARD_CORNER)
                         * (mag - LINE_DEADBAND_HEADING) / span;
        out->forward = LINE_FORWARD_CRUISE - drop;
    }
}

int line_control_search_turn(int lateral, int heading)
{
    /* 优先信 heading：丢线通常发生在弯道，弯道方向就是最后看到的倾斜方向。 */
    const int hd = deadband(heading, LINE_DEADBAND_HEADING);
    if (hd != 0) {
        return hd > 0 ? -LINE_TURN_MAX : LINE_TURN_MAX;
    }
    const int lt = deadband(lateral, LINE_DEADBAND_LATERAL);
    if (lt != 0) {
        return lt > 0 ? -LINE_TURN_MAX : LINE_TURN_MAX;
    }
    return 0;
}
