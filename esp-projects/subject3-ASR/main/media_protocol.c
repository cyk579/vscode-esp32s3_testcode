#include "media_protocol.h"
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)le16(p) | ((uint32_t)le16(p+2) << 16); }
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, uint32_t v) { put16(p,(uint16_t)v); put16(p+2,(uint16_t)(v>>16)); }
bool media_decode(const uint8_t *b, size_t n, media_command_t *out) {
    if(!b || !out || n!=MEDIA_COMMAND_BYTES || b[0]!=1 || b[1]<MEDIA_PLAY || b[1]>MEDIA_STOP || b[6] || b[7]) return false;
    uint16_t track=le16(b+4);
    if((b[1]==MEDIA_PLAY && !track) || (b[1]!=MEDIA_PLAY && track)) return false;
    *out=(media_command_t){.op=b[1],.sequence=le16(b+2),.track=track,.catalog=le32(b+8)};
    return true;
}
void media_encode_status(const media_status_t *s, uint8_t out[MEDIA_STATUS_BYTES]) {
    out[0]=1; out[1]=s->state; out[2]=s->error; out[3]=s->flags;
    put16(out+4,s->sequence); put16(out+6,s->track);
    put32(out+8,s->position_seconds); put32(out+12,s->catalog);
}
