#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GESTURE_FRAME_SIZE 12
#define GESTURE_VERSION 1
#define GESTURE_VALID 1u
#define GESTURE_HELD 2u
#define GESTURE_ESTOP 4u
/* Same UUIDs in Android Protocol.kt. Group ID is an installation label, not a password. */
#define GESTURE_SERVICE_UUID "7f510001-1b15-4b85-9c13-8f08604a0001"
#define GESTURE_CONTROL_UUID "7f510002-1b15-4b85-9c13-8f08604a0001"
#define GESTURE_STATUS_UUID  "7f510003-1b15-4b85-9c13-8f08604a0001"
#define GESTURE_UUID_BYTES(n) 0x01,0x00,0x4a,0x60,0x08,0x8f,0x13,0x9c,0x85,0x4b,0x15,0x1b,n,0x00,0x51,0x7f

typedef struct {
    uint8_t flags;
    uint16_t sequence;
    int16_t pitch_cd; /* positive = forward tilt */
    int16_t roll_cd;  /* positive = left tilt */
    uint32_t uptime_ms;
} gesture_frame_t;
void gesture_encode(const gesture_frame_t *frame, uint8_t out[GESTURE_FRAME_SIZE]);
bool gesture_decode(const uint8_t *data, size_t size, gesture_frame_t *frame);
bool gesture_sequence_newer(uint16_t next, uint16_t previous);
