#include "pcm_level.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    const unsigned char silence[] = {0, 0, 0, 0};
    pcm_level_t level = pcm16le_measure(silence, sizeof(silence));
    assert(level.samples == 2 && level.peak == 0 && level.sum_squares == 0);
    /* Deliberately unaligned; the final byte is an incomplete sample. */
    const unsigned char rails[] = {99, 0, 0x80, 0xff, 0x7f, 0xff};
    level = pcm16le_measure(rails + 1, sizeof(rails) - 1);
    assert(level.samples == 2 && level.peak == 32768);
    assert(level.sum_squares == UINT64_C(2147418113));
    const unsigned char wave[] = {0xe8, 3, 0x18, 0xfc}; /* +1000, -1000 */
    level = pcm16le_measure(wave, sizeof(wave));
    assert(level.samples == 2 && level.peak == 1000 && level.sum_squares == 2000000);
    assert(pcm16le_measure(NULL, 100).samples == 0);
    /* One second at 48kHz: the energy accumulator must exceed 32 bits. */
    unsigned char *loud = malloc(96000);
    assert(loud);
    for (size_t i = 0; i < 48000; ++i) { loud[2*i] = 0; loud[2*i+1] = 0x80; }
    level = pcm16le_measure(loud, 96000);
    assert(level.samples == 48000 && level.sum_squares == UINT64_C(51539607552000));
    free(loud);
    puts("PCM level tests passed");
    return 0;
}
