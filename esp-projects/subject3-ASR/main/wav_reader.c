#include "wav_reader.h"
#include <string.h>
static uint16_t le16(const unsigned char *p) { return (uint16_t)(p[0] | p[1]<<8); }
static uint32_t le32(const unsigned char *p) { return (uint32_t)le16(p) | (uint32_t)le16(p+2)<<16; }
bool wav_open_pcm(FILE *f, wav_info_t *info) {
    if(!f || !info || fseek(f,0,SEEK_END)) return false;
    long size=ftell(f);
    if(size<12 || fseek(f,0,SEEK_SET)) return false;
    unsigned char h[16];
    if(fread(h,1,12,f)!=12 || memcmp(h,"RIFF",4) || memcmp(h+8,"WAVE",4)) return false;
    uint64_t end=(uint64_t)le32(h+4)+8;
    if(end>(uint64_t)size || end<12) return false;
    wav_info_t w={0}; bool fmt=false, data=false;
    uint32_t bytes_per_second=0; uint16_t block=0;
    for(uint64_t pos=12; pos+8<=end;) {
        if(fseek(f,(long)pos,SEEK_SET) || fread(h,1,8,f)!=8) return false;
        uint32_t n=le32(h+4); uint64_t next=pos+8+n+(n&1u);
        if(next>end) return false;
        if(!memcmp(h,"fmt ",4)) {
            if(fmt || n<16 || fread(h,1,16,f)!=16 || le16(h)!=1) return false;
            fmt=true; w.channels=le16(h+2); w.rate=le32(h+4);
            bytes_per_second=le32(h+8); block=le16(h+12); w.bits=le16(h+14);
        } else if(!memcmp(h,"data",4)) {
            if(data) return false;
            data=true; w.data_offset=(long)(pos+8); w.data_bytes=n;
        }
        pos=next;
    }
    if(!fmt || !data || !w.data_bytes || w.bits!=16 || w.channels<1 || w.channels>2 ||
       w.rate<8000 || w.rate>48000 || block!=w.channels*2 ||
       bytes_per_second!=w.rate*block || w.data_bytes%block) return false;
    if(fseek(f,w.data_offset,SEEK_SET)) return false;
    *info=w; return true;
}
