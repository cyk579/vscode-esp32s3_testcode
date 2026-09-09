#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { MEDIA_PLAY = 1, MEDIA_PAUSE, MEDIA_RESUME, MEDIA_STOP };
enum { MEDIA_IDLE, MEDIA_PLAYING, MEDIA_PAUSED, MEDIA_ERROR };
enum { MEDIA_OK, MEDIA_NO_USB, MEDIA_NO_STORAGE, MEDIA_CATALOG_MISMATCH,
       MEDIA_NO_TRACK, MEDIA_BAD_WAV, MEDIA_IO_ERROR };
#define MEDIA_COMMAND_BYTES 12
#define MEDIA_STATUS_BYTES 16
#define MEDIA_UUID_BYTES(n) 0x01,0x00,0x4a,0x60,0x08,0x8f,0x13,0x9c,0x85,0x4b,0x15,0x1b,n,0x00,0x52,0x7f
typedef struct { uint8_t op; uint16_t sequence, track; uint32_t catalog; } media_command_t;
typedef struct {
    uint8_t state, error, flags; /* flags: bit0 USB ready, bit1 storage ready */
    uint16_t sequence, track;
    uint32_t position_seconds, catalog;
} media_status_t;
bool media_decode(const uint8_t *bytes, size_t size, media_command_t *out);
void media_encode_status(const media_status_t *status, uint8_t out[MEDIA_STATUS_BYTES]);
