#include "gesture_control.h"
#include "control_config.h"
#include <math.h>
#include <string.h>
static void stop(gesture_control_t *c, drive_state_t state) {
    c->state = state; c->neutral_tracking = false;
    memset(c->pwm, 0, sizeof(c->pwm));
    memset(c->last_direction, 0, sizeof(c->last_direction));
    memset(c->at_zero, 0, sizeof(c->at_zero));
}
void gesture_control_init(gesture_control_t *c) { memset(c, 0, sizeof(*c)); }
void gesture_control_link(gesture_control_t *c, bool connected) {
    c->connected = connected; c->have_frame = false;
    stop(c, connected ? DRIVE_WAIT_NEUTRAL : DRIVE_OFF);
}
void gesture_control_fault(gesture_control_t *c, bool fault) {
    c->local_fault = fault;
    if (fault) stop(c, DRIVE_FAULT);
}
float gesture_axis(float a) {
    float x = (fabsf(a)-CONTROL_DEADZONE_DEG)/(CONTROL_FULL_TILT_DEG-CONTROL_DEADZONE_DEG);
    x = fminf(1.0f, fmaxf(0.0f, x));
    return copysignf(x, a);
}
void gesture_mix(float forward, float lateral, float yaw, float out[3]) {
    out[0] = -0.8660254f*forward - 0.5f*lateral - yaw;
    out[1] = -lateral + yaw;
    out[2] = 0.8660254f*forward - 0.5f*lateral - yaw;
    float peak = fmaxf(fabsf(out[0]), fmaxf(fabsf(out[1]), fabsf(out[2])));
    float scale = peak > 1 ? CONTROL_MAX_PWM/peak : CONTROL_MAX_PWM;
    for (int i=0; i<3; ++i) out[i] *= scale;
}
bool gesture_control_receive(gesture_control_t *c, const gesture_frame_t *f, uint32_t now) {
    if (!c->connected) return false;
    if (c->have_frame && (!gesture_sequence_newer(f->sequence, c->frame.sequence) ||
        (uint32_t)(f->uptime_ms-c->frame.uptime_ms) == 0 ||
        (uint32_t)(f->uptime_ms-c->frame.uptime_ms) >= 0x80000000u)) return false;
    if (c->have_frame && (uint32_t)(now-c->last_rx) >= CONTROL_TIMEOUT_MS) stop(c, DRIVE_FAULT);
    c->frame = *f; c->last_rx = now; c->have_frame = true;
    float p = f->pitch_cd/100.0f, r = f->roll_cd/100.0f, y = f->yaw_cd/100.0f;
    if (c->local_fault || !(f->flags & GESTURE_VALID) || (f->flags & GESTURE_ESTOP) ||
        fabsf(p) > CONTROL_MAX_TILT_DEG || fabsf(r) > CONTROL_MAX_TILT_DEG || fabsf(y) > CONTROL_MAX_TILT_DEG) {
        stop(c, DRIVE_FAULT); return true;
    }
    if (!(f->flags & GESTURE_HELD)) {
        bool was_ready = c->state == DRIVE_READY;
        if (c->state == DRIVE_ACTIVE) stop(c, DRIVE_WAIT_NEUTRAL);
        if (fabsf(p) <= CONTROL_DEADZONE_DEG && fabsf(r) <= CONTROL_DEADZONE_DEG &&
            fabsf(y) <= CONTROL_DEADZONE_DEG) {
            if (!c->neutral_tracking) { c->neutral_tracking = true; c->neutral_since = now; }
            c->state = was_ready || (uint32_t)(now-c->neutral_since) >= CONTROL_NEUTRAL_MS ? DRIVE_READY : DRIVE_WAIT_NEUTRAL;
        } else { c->neutral_tracking = false; c->state = DRIVE_WAIT_NEUTRAL; }
    } else {
        c->neutral_tracking = false;
        if (c->state == DRIVE_READY) c->state = DRIVE_ACTIVE;
    }
    return true;
}
void gesture_control_step(gesture_control_t *c, uint32_t now, float dt) {
    if (!c->connected) { stop(c, DRIVE_OFF); return; }
    if (c->local_fault || (c->have_frame && (uint32_t)(now-c->last_rx) >= CONTROL_TIMEOUT_MS) ||
        !isfinite(dt) || dt <= 0 || dt > 0.05f) { stop(c, DRIVE_FAULT); return; }
    if (c->state != DRIVE_ACTIVE) { memset(c->pwm, 0, sizeof(c->pwm)); return; }
    float target[3];
    gesture_mix(gesture_axis(c->frame.pitch_cd/100.0f), gesture_axis(c->frame.roll_cd/100.0f), gesture_axis(c->frame.yaw_cd/100.0f), target);
    float largest_change = 0;
    for (int wheel=0; wheel<3; ++wheel) {
        int direction = (target[wheel] > 0)-(target[wheel] < 0);
        bool reverse = direction && c->last_direction[wheel] && direction != c->last_direction[wheel];
        if (reverse && (!c->at_zero[wheel] || (uint32_t)(now-c->zero_since[wheel]) < CONTROL_REVERSE_MS)) target[wheel] = 0;
        largest_change = fmaxf(largest_change, fabsf(target[wheel]-c->pwm[wheel]));
    }
    float step_limit = CONTROL_SLEW_PWM_PER_SECOND*dt;
    float fraction = largest_change > step_limit ? step_limit/largest_change : 1;
    for (int wheel=0; wheel<3; ++wheel) {
        c->pwm[wheel] += fraction*(target[wheel]-c->pwm[wheel]);
        if (fabsf(c->pwm[wheel]) < 0.0001f) {
            c->pwm[wheel] = 0;
            if (!c->at_zero[wheel]) { c->zero_since[wheel] = now; c->at_zero[wheel] = true; }
        } else {
            c->last_direction[wheel] = (c->pwm[wheel] > 0) ? 1 : -1;
            c->at_zero[wheel] = false;
        }
    }
}
