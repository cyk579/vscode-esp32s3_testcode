#include "music_player.h"
#include "wav_reader.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "usb_stream.h"
#include <inttypes.h>
#include <stdatomic.h>
#include <stdlib.h>

static const char *TAG="music";
static QueueHandle_t commands;
static portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
static media_status_t published;
static atomic_bool usb_ready;
static atomic_uint usb_epoch;

static void publish(media_status_t *s, bool mounted) {
    s->flags=(atomic_load(&usb_ready)?1:0)|(mounted?2:0);
    portENTER_CRITICAL(&lock); published=*s; portEXIT_CRITICAL(&lock);
}
void music_player_status(uint8_t out[MEDIA_STATUS_BYTES]) {
    portENTER_CRITICAL(&lock); media_status_t s=published; portEXIT_CRITICAL(&lock);
    media_encode_status(&s,out);
}
bool music_player_submit(const media_command_t *command) {
    return commands && command && xQueueSend(commands,command,0)==pdTRUE;
}
static void usb_state(usb_stream_state_t event,void *arg) {
    (void)arg;
    atomic_store(&usb_ready,false);
    atomic_fetch_add(&usb_epoch,1);
    if(event!=STREAM_CONNECTED) { ESP_LOGW(TAG,"USB speaker disconnected"); return; }
    size_t count=0,current=0;
    if(uac_frame_size_list_get(STREAM_UAC_SPK,NULL,&count,&current)!=ESP_OK || !count || count>255) {
        ESP_LOGW(TAG,"No matching UAC1 speaker; check descriptors and output format"); return;
    }
    uac_frame_size_t *formats=calloc(count,sizeof(*formats));
    if(!formats) return;
    if(uac_frame_size_list_get(STREAM_UAC_SPK,formats,&count,&current)==ESP_OK) {
        for(size_t i=0;i<count;++i) ESP_LOGI(TAG,"speaker[%u]%s ch=%u bits=%u rate=%"PRIu32" range=%"PRIu32"..%"PRIu32,
            (unsigned)i,i==current?" SELECTED":"",formats[i].ch_num,formats[i].bit_resolution,
            formats[i].samples_frequence,formats[i].samples_frequence_min,formats[i].samples_frequence_max);
        if(current<count) {
            const uac_frame_size_t *f=&formats[current];
            bool rate=f->samples_frequence==CONFIG_SUBJECT3_SPEAKER_RATE ||
                (f->samples_frequence_min<=CONFIG_SUBJECT3_SPEAKER_RATE &&
                 f->samples_frequence_max>=CONFIG_SUBJECT3_SPEAKER_RATE);
            atomic_store(&usb_ready,f->ch_num==1 && f->bit_resolution==16 && rate);
        }
    }
    free(formats);
}
static void close_song(FILE **file) { if(*file) fclose(*file); *file=NULL; }
static void fail(media_status_t *s,FILE **file,uint8_t error) {
    close_song(file); s->state=MEDIA_ERROR; s->error=error;
}
static void player_task(void *arg) {
    (void)arg;
    media_status_t s={0}; FILE *song=NULL; wav_info_t wav={0};
    uint32_t remaining=0; unsigned play_epoch=0;
    const esp_vfs_spiffs_conf_t fs={.base_path="/music",.partition_label="music",
        .max_files=3,.format_if_mount_failed=false};
    bool mounted=esp_vfs_spiffs_register(&fs)==ESP_OK;
    if(mounted) {
        FILE *catalog=fopen("/music/catalog.id","r");
        if(!catalog || fscanf(catalog,"%"SCNx32,&s.catalog)!=1) mounted=false;
        if(catalog) fclose(catalog);
    }
    if(!mounted) { s.state=MEDIA_ERROR; s.error=MEDIA_NO_STORAGE; ESP_LOGW(TAG,"Music filesystem/catalog missing; motor control remains available"); }
    const uac_config_t config={.spk_ch_num=1,.spk_bit_resolution=16,
        .spk_samples_frequence=CONFIG_SUBJECT3_SPEAKER_RATE,.spk_buf_size=2048};
    esp_err_t err=uac_streaming_config(&config);
    if(err==ESP_OK) err=usb_streaming_state_register(usb_state,NULL);
    if(err==ESP_OK) err=usb_streaming_start();
    if(err!=ESP_OK) { s.state=MEDIA_ERROR; s.error=MEDIA_NO_USB; ESP_LOGE(TAG,"USB audio start: %s",esp_err_to_name(err)); }
    ESP_LOGI(TAG,"catalog=%08"PRIx32" rate=%d mono PCM16; no autoplay",s.catalog,CONFIG_SUBJECT3_SPEAKER_RATE);
    uint8_t pcm[640]; size_t buffered=0;
    int64_t write_started=0;
    for(;;) {
        if(song && (!atomic_load(&usb_ready) || play_epoch!=atomic_load(&usb_epoch))) {
            fail(&s,&song,MEDIA_NO_USB); buffered=0;
        }
        media_command_t command;
        if(xQueueReceive(commands,&command,pdMS_TO_TICKS(s.state==MEDIA_PLAYING?0:20))==pdTRUE) {
            s.sequence=command.sequence;
            if(command.op==MEDIA_STOP) {
                close_song(&song); buffered=0; s.state=MEDIA_IDLE; s.error=MEDIA_OK; s.position_seconds=0;
            } else if(command.op==MEDIA_PAUSE) {
                if(s.state==MEDIA_PLAYING) s.state=MEDIA_PAUSED;
            } else if(command.op==MEDIA_RESUME) {
                if(s.state==MEDIA_PAUSED && song && atomic_load(&usb_ready)) {
                    s.state=MEDIA_PLAYING; write_started=esp_timer_get_time();
                }
            } else if(command.op==MEDIA_PLAY) {
                /* Reject unavailable titles/catalogs without disturbing the current song. */
                uint8_t error=!mounted?MEDIA_NO_STORAGE:command.catalog!=s.catalog?MEDIA_CATALOG_MISMATCH:
                    !atomic_load(&usb_ready)?MEDIA_NO_USB:MEDIA_OK;
                char path[40]; snprintf(path,sizeof(path),"/music/%u.wav",command.track);
                FILE *next=error?NULL:fopen(path,"rb"); wav_info_t next_wav={0};
                if(!error && !next) error=MEDIA_NO_TRACK;
                if(!error && (!wav_open_pcm(next,&next_wav) || next_wav.channels!=1 || next_wav.rate!=CONFIG_SUBJECT3_SPEAKER_RATE)) error=MEDIA_BAD_WAV;
                if(error) { if(next) fclose(next); s.error=error; if(!song) s.state=MEDIA_ERROR; }
                else {
                    /* Flush the old track in the USB ring buffer only on an explicit new song. */
                    err=usb_streaming_control(STREAM_UAC_SPK,CTRL_SUSPEND,NULL);
                    if(err==ESP_OK) err=usb_streaming_control(STREAM_UAC_SPK,CTRL_RESUME,NULL);
                    if(err!=ESP_OK) { fclose(next); fail(&s,&song,MEDIA_NO_USB); buffered=0; }
                    else {
                        close_song(&song); song=next; wav=next_wav; remaining=wav.data_bytes; buffered=0;
                        play_epoch=atomic_load(&usb_epoch); s.state=MEDIA_PLAYING; s.error=MEDIA_OK;
                        s.track=command.track; s.position_seconds=0;
                    }
                }
            }
        }
        if(s.state==MEDIA_PLAYING && song) {
            if(!buffered && remaining) {
                write_started=esp_timer_get_time();
                buffered=remaining<sizeof(pcm)?remaining:sizeof(pcm);
                if(fread(pcm,1,buffered,song)!=buffered) { buffered=0; fail(&s,&song,MEDIA_IO_ERROR); }
                else {
                    for(size_t i=0;i<buffered;i+=2) {
                        int32_t sample=pcm[i]|((uint32_t)pcm[i+1]<<8);
                        if(sample>=32768) sample-=65536;
                        sample=sample*CONFIG_SUBJECT3_SPEAKER_GAIN/100;
                        uint16_t raw=(uint16_t)sample; pcm[i]=(uint8_t)raw; pcm[i+1]=(uint8_t)(raw>>8);
                    }
                }
            }
            if(buffered) {
                /* Bounded wait in a dedicated lower-priority task, never in BLE or motor tasks. */
                err=uac_spk_streaming_write(pcm,buffered,20);
                if(err==ESP_OK) { remaining-=(uint32_t)buffered; buffered=0; s.position_seconds=(wav.data_bytes-remaining)/(wav.rate*2); }
                else if(err!=ESP_ERR_TIMEOUT || esp_timer_get_time()-write_started>=2000000) {
                    buffered=0; fail(&s,&song,MEDIA_IO_ERROR);
                }
            }
            if(song && !remaining) { close_song(&song); s.state=MEDIA_IDLE; }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        publish(&s,mounted);
    }
}
esp_err_t music_player_start(void) {
    commands=xQueueCreate(8,sizeof(media_command_t));
    if(!commands) return ESP_ERR_NO_MEM;
    if(xTaskCreate(player_task,"music",6144,NULL,3,NULL)!=pdPASS) {
        vQueueDelete(commands); commands=NULL; return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
