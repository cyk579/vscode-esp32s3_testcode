#include "board_pins.h"
#include "motor.h"
#include "gesture_control.h"
#include "gesture_ble.h"
#include "gesture_imu.h"
#include "control_config.h"
#include "tft_st7735.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <stdatomic.h>

static gesture_control_t control;
static portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
static atomic_bool hardware_fault=true;
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time()/1000); }
static void link_changed(bool connected) {
    portENTER_CRITICAL(&lock); gesture_control_link(&control,connected); portEXIT_CRITICAL(&lock);
}
static void received(const gesture_frame_t *f) {
    uint32_t now=now_ms();
    portENTER_CRITICAL(&lock); gesture_control_receive(&control,f,now); portEXIT_CRITICAL(&lock);
}
/* Status: version,state,local_fault,reserved, pitch int16,roll int16, pwm A/B/D int8,reserved. */
static void status(uint8_t out[12]) {
    portENTER_CRITICAL(&lock); gesture_control_t c=control; portEXIT_CRITICAL(&lock);
    out[0]=1; out[1]=(uint8_t)c.state; out[2]=c.local_fault; out[3]=0;
    out[4]=(uint8_t)c.frame.pitch_cd; out[5]=(uint8_t)((uint16_t)c.frame.pitch_cd>>8);
    out[6]=(uint8_t)c.frame.roll_cd; out[7]=(uint8_t)((uint16_t)c.frame.roll_cd>>8);
    for(int i=0;i<3;++i) out[8+i]=(uint8_t)(int8_t)lroundf(c.pwm[i]);
    out[11]=0;
}
static void control_task(void *arg) {
    (void)arg; TickType_t wake=xTaskGetTickCount(); uint32_t previous=now_ms(), tilt_since=0;
    bool tilting=false;
    for(;;) {
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(10));
        uint32_t now=now_ms(); bool fault=hardware_fault;
#if ENABLE_CAR_IMU
        gesture_imu_sample_t s;
        bool valid=gesture_imu_latest(&s);
        if(!valid) fault=true;
        bool excessive=valid && (fabsf(s.body_roll)>CAR_TILT_LIMIT_DEG || fabsf(s.body_pitch)>CAR_TILT_LIMIT_DEG);
        if(excessive) {
            if(!tilting) { tilting=true; tilt_since=now; }
            if((uint32_t)(now-tilt_since)>=CAR_TILT_HOLD_MS) fault=true;
        } else tilting=false;
#else
        (void)tilting; (void)tilt_since;
#endif
        float dt=(uint32_t)(now-previous)/1000.0f; previous=now;
        portENTER_CRITICAL(&lock);
        gesture_control_fault(&control,fault); gesture_control_step(&control,now,dt);
        gesture_control_t snapshot=control;
        portEXIT_CRITICAL(&lock);
        if(!hardware_fault && motor_apply(snapshot.pwm,snapshot.state==DRIVE_ACTIVE)!=ESP_OK) {
            hardware_fault=true; motor_stop(); ESP_LOGE("car","Motor output failed; restart after inspection");
        }
    }
}
static void display_task(void *arg) {
    (void)arg;
#if ENABLE_TFT
    bool screen=tft_st7735_init();
#endif
    for(;;) {
        uint8_t s[12]; status(s);
        ESP_LOGI("car","state=%u fault=%u pwm=%d,%d,%d",s[1],s[2],(int8_t)s[8],(int8_t)s[9],(int8_t)s[10]);
#if ENABLE_TFT
        if(screen) {
            char a[28],b[28],c[28];
            snprintf(a,sizeof(a),"STATE %u FAULT %u",s[1],s[2]);
            snprintf(b,sizeof(b),"PWM %d %d %d",(int8_t)s[8],(int8_t)s[9],(int8_t)s[10]);
            snprintf(c,sizeof(c),"RELEASE LEVEL PRESS");
            const char *lines[]={"GESTURE CAR",a,b,c};
            screen=tft_st7735_draw_text_lines(lines,4,0xffff,0);
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
void app_main(void) {
    gesture_control_init(&control);
    const gesture_pin_t required[]={
        {"PWMA",PIN_PWMA,true},{"AIN1",PIN_AIN1,true},{"AIN2",PIN_AIN2,true},
        {"PWMB",PIN_PWMB,true},{"BIN1",PIN_BIN1,true},{"BIN2",PIN_BIN2,true},
        {"PWMD",PIN_PWMD,true},{"DIN1",PIN_DIN1,true},{"DIN2",PIN_DIN2,true},
        {"STBY",PIN_MOTOR_STBY,true},
#if ENABLE_CAR_IMU
        {"IMU SDA",PIN_IMU_SDA,true},{"IMU SCL",PIN_IMU_SCL,true},
#endif
#if ENABLE_TFT
        {"TFT SCLK",PIN_TFT_SCLK,true},{"TFT MOSI",PIN_TFT_MOSI,true},{"TFT DC",PIN_TFT_DC,true},
        {"TFT CS",PIN_TFT_CS,true},{"TFT RST",PIN_TFT_RST,true},
#endif
    };
    if(gesture_board_validate(required,sizeof(required)/sizeof(required[0]),ALLOW_STRAPPING_PINS)!=ESP_OK) {
        ESP_LOGE("car","CONFIG MISSING: no peripheral initialized. Fill board_pins.h, then rebuild."); return;
    }
    esp_err_t err=motor_init();
    if(err!=ESP_OK) { motor_stop(); ESP_LOGE("car","Motor init: %s",esp_err_to_name(err)); return; }
#if ENABLE_CAR_IMU
    gesture_imu_config_t imu={ .sda=PIN_IMU_SDA,.scl=PIN_IMU_SCL,
        .axis={IMU_AXIS_X,IMU_AXIS_Y,IMU_AXIS_Z},.sign={IMU_SIGN_X,IMU_SIGN_Y,IMU_SIGN_Z} };
    err=gesture_imu_start(&imu);
    if(err!=ESP_OK) { motor_stop(); ESP_LOGE("car","IMU init: %s",esp_err_to_name(err)); return; }
#endif
    hardware_fault=false;
    if(xTaskCreate(control_task,"drive",4096,NULL,6,NULL)!=pdPASS) { motor_stop(); return; }
    err=gesture_ble_server_start(link_changed,received,status);
    if(err!=ESP_OK) { hardware_fault=true; motor_stop(); ESP_LOGE("car","BLE init: %s",esp_err_to_name(err)); return; }
    xTaskCreate(display_task,"status",4096,NULL,2,NULL);
}
