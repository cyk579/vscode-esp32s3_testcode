#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef struct { uint32_t rate, data_bytes; uint16_t channels, bits; long data_offset; } wav_info_t;
/* Validates RIFF chunks and leaves the file at PCM data. No allocation/decoding. */
bool wav_open_pcm(FILE *file, wav_info_t *info);
