#include "gesture_imu.h"
#include "control_config.h"
#include "FusionAhrs.h"
#include "FusionMath.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t device;
static gesture_imu_config_t config;
static gesture_imu_sample_t latest;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time()/1000); }
static esp_err_t read_reg(uint8_t reg, uint8_t *data, size_t size) {
    return i2c_master_transmit_receive(device, &reg, 1, data, size, 10);
}
static esp_err_t write_reg(uint8_t reg, uint8_t data) {
    uint8_t packet[] = {reg, data}; return i2c_master_transmit(device, packet, 2, 20);
}
static int16_t signed_be(const uint8_t *b) {
    int v = (b[0]<<8)|b[1]; return (int16_t)(v>32767 ? v-65536 : v);
}
static bool read_motion(FusionVector *acc, FusionVector *gyro) {
    uint8_t ready, b[14];
    if (read_reg(0x3a, &ready, 1) != ESP_OK || !(ready & 1) || read_reg(0x3b,b,sizeof(b)) != ESP_OK) return false;
    float a[3], g[3];
    for (int i=0; i<3; ++i) { a[i]=signed_be(b+2*i)/4096.0f; g[i]=signed_be(b+8+2*i)/65.5f; }
    for (int i=0; i<3; ++i) { acc->array[i]=a[config.axis[i]]*config.sign[i]; gyro->array[i]=g[config.axis[i]]*config.sign[i]; }
    return true;
}
static void publish(gesture_imu_sample_t sample) {
    portENTER_CRITICAL(&lock); latest = sample; portEXIT_CRITICAL(&lock);
}
static void imu_task(void *arg) {
    (void)arg;
    FusionAhrs ahrs;
    FusionVector offset = { .array = {0,0,0} };
    float sum[3] = {0}; unsigned calibration_count = 0;
    uint32_t previous = now_ms(), last_good = previous;
    bool calibrated = false;
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSettings settings = { .sampleRate=100, .convention=FusionConventionNwu,
        .gain=0.5f, .gyroscopeRange=500, .accelerationRejection=10, .magneticRejection=0, .rejectionTimeout=5 };
    FusionAhrsSetSettings(&ahrs, &settings);
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
        uint32_t now=now_ms(); FusionVector a,g;
        if (!read_motion(&a,&g)) {
            if ((uint32_t)(now-last_good) >= IMU_STALE_MS) {
                publish((gesture_imu_sample_t){0}); calibrated=false; calibration_count=0;
                memset(sum,0,sizeof(sum)); FusionAhrsRestart(&ahrs);
            }
            continue;
        }
        last_good=now;
        float dt=(uint32_t)(now-previous)/1000.0f; previous=now;
        if (!calibrated) {
            float norm=sqrtf(a.axis.x*a.axis.x+a.axis.y*a.axis.y+a.axis.z*a.axis.z);
            bool still=fabsf(norm-1)<0.08f && fabsf(a.axis.x)<0.12f && fabsf(a.axis.y)<0.12f && a.axis.z>0.8f &&
                fabsf(g.axis.x)<3 && fabsf(g.axis.y)<3 && fabsf(g.axis.z)<3;
            if (!still) { calibration_count=0; memset(sum,0,sizeof(sum)); continue; }
            for(int i=0;i<3;++i) sum[i]+=g.array[i];
            if (++calibration_count < 200) continue;
            for(int i=0;i<3;++i) offset.array[i]=sum[i]/calibration_count;
            calibrated=true; FusionAhrsRestart(&ahrs);
            ESP_LOGI("imu","Gyro bias calibrated; wait for attitude startup (3s)");
        }
        if (dt<=0 || dt>0.05f) { calibrated=false; calibration_count=0; memset(sum,0,sizeof(sum)); publish((gesture_imu_sample_t){0}); continue; }
        for(int i=0;i<3;++i) g.array[i]-=offset.array[i];
        FusionAhrsSetSamplePeriod(&ahrs,dt);
        FusionAhrsUpdateNoMagnetometer(&ahrs,g,a);
        FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        FusionAhrsFlags flags=FusionAhrsGetFlags(&ahrs);
        bool valid=!flags.startup && !flags.overrangeRecovery && isfinite(e.angle.pitch) && isfinite(e.angle.roll);
        publish((gesture_imu_sample_t){ .pitch=e.angle.pitch, .roll=-e.angle.roll,
            .body_roll=e.angle.roll, .body_pitch=e.angle.pitch, .valid=valid, .sampled_ms=now });
    }
}
esp_err_t gesture_imu_start(const gesture_imu_config_t *cfg) {
    if (bus || !cfg || cfg->sda<0 || cfg->scl<0 || cfg->sda==cfg->scl) return ESP_ERR_INVALID_ARG;
    for(int i=0;i<3;++i) {
        if(cfg->axis[i]<0 || cfg->axis[i]>2 || (cfg->sign[i]!=1 && cfg->sign[i]!=-1)) return ESP_ERR_INVALID_ARG;
        for(int j=0;j<i;++j) if(cfg->axis[i]==cfg->axis[j]) return ESP_ERR_INVALID_ARG;
    }
    int inversions=0;
    for(int i=0;i<3;++i) for(int j=i+1;j<3;++j) if(cfg->axis[i]>cfg->axis[j]) ++inversions;
    if(((inversions%2)?-1:1)*cfg->sign[0]*cfg->sign[1]*cfg->sign[2]!=1) return ESP_ERR_INVALID_ARG;
    config=*cfg;
    i2c_master_bus_config_t b={ .i2c_port=I2C_NUM_0, .sda_io_num=cfg->sda, .scl_io_num=cfg->scl,
        .clk_source=I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt=7, .flags.enable_internal_pullup=true };
    esp_err_t err=i2c_new_master_bus(&b,&bus); if(err!=ESP_OK) return err;
    uint16_t address=0;
    for(uint16_t a=0x68;a<=0x69;++a) if(i2c_master_probe(bus,a,20)==ESP_OK) { address=a; break; }
    if(!address) return ESP_ERR_NOT_FOUND;
    i2c_device_config_t d={ .dev_addr_length=I2C_ADDR_BIT_LEN_7, .device_address=address, .scl_speed_hz=100000 };
    err=i2c_master_bus_add_device(bus,&d,&device); if(err!=ESP_OK) return err;
    uint8_t who=0; err=read_reg(0x75,&who,1);
    if(err!=ESP_OK) return err;
    if(who!=0x70) { ESP_LOGE("imu","WHO_AM_I=0x%02x, expected MPU6500 0x70",who); return ESP_ERR_NOT_SUPPORTED; }
    if((err=write_reg(0x6b,0x80))!=ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(100));
    const uint8_t init[][2]={{0x6b,1},{0x6c,0},{0x1a,4},{0x19,9},{0x1b,8},{0x1c,16},{0x1d,4},{0x38,1}};
    for(size_t i=0;i<sizeof(init)/sizeof(init[0]);++i) if((err=write_reg(init[i][0],init[i][1]))!=ESP_OK) return err;
    return xTaskCreate(imu_task,"gesture_imu",4096,NULL,5,NULL)==pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
bool gesture_imu_latest(gesture_imu_sample_t *sample) {
    portENTER_CRITICAL(&lock); *sample=latest; portEXIT_CRITICAL(&lock);
    sample->valid = sample->valid && (uint32_t)(now_ms()-sample->sampled_ms)<IMU_STALE_MS;
    return sample->valid;
}
