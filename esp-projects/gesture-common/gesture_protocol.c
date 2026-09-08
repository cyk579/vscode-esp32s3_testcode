#include "gesture_protocol.h"
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static void put16(uint8_t *p, uint16_t n) { p[0] = (uint8_t)n; p[1] = (uint8_t)(n >> 8); }
void gesture_encode(const gesture_frame_t *f, uint8_t out[GESTURE_FRAME_SIZE]) {
    out[0] = GESTURE_VERSION; out[1] = f->flags;
    put16(out + 2, f->sequence); put16(out + 4, (uint16_t)f->pitch_cd);
    put16(out + 6, (uint16_t)f->roll_cd);
    for (int i = 0; i < 4; ++i) out[8+i] = (uint8_t)(f->uptime_ms >> (8*i));
}
bool gesture_decode(const uint8_t *p, size_t n, gesture_frame_t *f) {
    if (!p || !f || n != GESTURE_FRAME_SIZE || p[0] != GESTURE_VERSION || (p[1] & ~7u)) return false;
    f->flags = p[1]; f->sequence = get16(p+2);
    /* Explicit signed conversion rather than relying on implementation-defined casts. */
    int pitch = get16(p+4), roll = get16(p+6);
    f->pitch_cd = (int16_t)(pitch > 32767 ? pitch-65536 : pitch);
    f->roll_cd = (int16_t)(roll > 32767 ? roll-65536 : roll);
    f->uptime_ms = 0;
    for (int i=0; i<4; ++i) f->uptime_ms |= (uint32_t)p[8+i] << (8*i);
    return f->pitch_cd >= -18000 && f->pitch_cd <= 18000 && f->roll_cd >= -18000 && f->roll_cd <= 18000;
}
bool gesture_sequence_newer(uint16_t next, uint16_t previous) {
    uint16_t delta = (uint16_t)(next-previous);
    return delta != 0 && delta < 0x8000u;
}
