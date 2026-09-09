#include "media_protocol.h"
#include "wav_reader.h"
#include <assert.h>
#include <string.h>

static unsigned char good[]={
 'R','I','F','F',40,0,0,0,'W','A','V','E',
 'f','m','t',' ',16,0,0,0,1,0,1,0,0x80,0x3e,0,0,0,0x7d,0,0,2,0,16,0,
 'd','a','t','a',4,0,0,0,0,0,0,0
};
static bool read_bytes(const unsigned char *bytes,size_t n) {
    FILE *f=tmpfile(); assert(f); assert(fwrite(bytes,1,n,f)==n);
    wav_info_t w; bool ok=wav_open_pcm(f,&w);
    if(ok) { assert(w.rate==16000 && w.channels==1 && w.data_bytes==4); assert(ftell(f)==w.data_offset); }
    fclose(f); return ok;
}
int main(void) {
    uint8_t packet[]={1,1,0x34,0x12,2,0,0,0,0x78,0x56,0x34,0x12};
    media_command_t cmd;
    assert(media_decode(packet,sizeof(packet),&cmd));
    assert(cmd.op==MEDIA_PLAY && cmd.track==2 && cmd.sequence==0x1234 && cmd.catalog==0x12345678);
    for(size_t n=0;n<sizeof(packet);++n) assert(!media_decode(packet,n,&cmd));
    packet[6]=1; assert(!media_decode(packet,sizeof(packet),&cmd)); packet[6]=0;
    packet[1]=MEDIA_PAUSE; assert(!media_decode(packet,sizeof(packet),&cmd));
    packet[4]=0; assert(media_decode(packet,sizeof(packet),&cmd));
    assert(read_bytes(good,sizeof(good)));
    for(size_t n=0;n<sizeof(good);++n) assert(!read_bytes(good,n));
    unsigned char corrupt[sizeof(good)]; memcpy(corrupt,good,sizeof(good));
    corrupt[20]=3; assert(!read_bytes(corrupt,sizeof(corrupt))); /* float, not PCM */
    memcpy(corrupt,good,sizeof(good)); corrupt[28]=1; assert(!read_bytes(corrupt,sizeof(corrupt)));
    memcpy(corrupt,good,sizeof(good)); corrupt[40]=0xff; assert(!read_bytes(corrupt,sizeof(corrupt)));
    /* Skip an odd-sized unknown chunk, including its pad byte. */
    unsigned char extra[sizeof(good)+10]; memcpy(extra,good,12); extra[4]=50;
    memcpy(extra+12,"JUNK\1\0\0\0x\0",10); memcpy(extra+22,good+12,sizeof(good)-12);
    assert(read_bytes(extra,sizeof(extra)));
    uint8_t status[16]; media_status_t s={.state=MEDIA_PLAYING,.flags=3,.sequence=0x1234,.track=2,.catalog=0x12345678};
    media_encode_status(&s,status);
    assert(status[0]==1 && status[1]==1 && status[3]==3 && status[4]==0x34 && status[12]==0x78);
    puts("Music protocol and RIFF validation passed"); return 0;
}
