#pragma once
#include "gesture_protocol.h"
typedef enum { DRIVE_OFF, DRIVE_WAIT_NEUTRAL, DRIVE_READY, DRIVE_ACTIVE, DRIVE_FAULT } drive_state_t;
typedef struct {
    bool connected, have_frame, neutral_tracking, local_fault;
    drive_state_t state;
    gesture_frame_t frame;
    uint32_t last_rx, neutral_since;
    float pwm[3]; /* A, B, D, before physical direction signs */
    int last_direction[3];
    uint32_t zero_since[3];
    bool at_zero[3];
} gesture_control_t;
void gesture_control_init(gesture_control_t *c);
void gesture_control_link(gesture_control_t *c, bool connected);
void gesture_control_fault(gesture_control_t *c, bool fault);
bool gesture_control_receive(gesture_control_t *c, const gesture_frame_t *f, uint32_t now);
void gesture_control_step(gesture_control_t *c, uint32_t now, float dt_seconds);
float gesture_axis(float angle_deg);
void gesture_mix(float forward, float lateral, float yaw, float out[3]);
