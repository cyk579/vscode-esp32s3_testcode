#pragma once
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint64_t samples;
    uint64_t sum_squares;
    uint32_t peak;
} pcm_level_t;

/* Signed 16-bit little-endian PCM, including interleaved channels.
 * An unmatched final byte is ignored. This is a level meter, not ASR. */
pcm_level_t pcm16le_measure(const void *data, size_t bytes);
