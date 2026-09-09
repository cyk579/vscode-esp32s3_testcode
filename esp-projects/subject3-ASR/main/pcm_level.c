#include "pcm_level.h"

pcm_level_t pcm16le_measure(const void *data, size_t bytes)
{
    pcm_level_t result = {0};
    if (!data) return result;
    const uint8_t *p = data;
    for (size_t i = 0; i < bytes / 2; ++i) {
        uint32_t raw = (uint32_t)p[2 * i] | ((uint32_t)p[2 * i + 1] << 8);
        int32_t sample = raw >= 32768 ? (int32_t)raw - 65536 : (int32_t)raw;
        uint32_t magnitude = (uint32_t)(sample < 0 ? -sample : sample);
        if (magnitude > result.peak) result.peak = magnitude;
        result.sum_squares += (uint64_t)magnitude * magnitude;
        ++result.samples;
    }
    return result;
}
