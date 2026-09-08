#include "board_pins.h"
#include "gesture_ble.h"
#include "gesture_imu.h"
#include "control_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
static uint16_t sequence;
static bool pressed;
static uint32_t pressed_since;
static void link_changed(bool connected) {
    pressed=false; pressed_since=0;
    ESP_LOGI("remote","link=%d; release and level, then press",connected);
}
static void build_frame(gesture_frame_t *f) {
    uint32_t now=(uint32_t)(esp_timer_get_time()/1000);
    gesture_imu_sample_t s; bool valid=gesture_imu_latest(&s);
    /* Release is immediate. Press must stay low for >=50ms across sends. */
    if(gpio_get_level(PIN_ENABLE_BUTTON)) pressed=false;
    else if(!pressed) { pressed=true; pressed_since=now; }
    bool held=pressed && (uint32_t)(now-pressed_since)>=50;
    f->sequence=++sequence; f->uptime_ms=now;
    f->flags=(valid?GESTURE_VALID:0)|(held?GESTURE_HELD:0);
    f->pitch_cd=valid?(int16_t)lroundf(s.pitch*100*REMOTE_PITCH_SIGN):0;
    f->roll_cd=valid?(int16_t)lroundf(s.roll*100*REMOTE_ROLL_SIGN):0;
}
void app_main(void) {
    const gesture_pin_t pins[]={ {"IMU SDA",PIN_IMU_SDA,true},{"IMU SCL",PIN_IMU_SCL,true},{"ENABLE",PIN_ENABLE_BUTTON,false} };
    if(gesture_board_validate(pins,sizeof(pins)/sizeof(pins[0]),ALLOW_STRAPPING_PINS)!=ESP_OK) {
        ESP_LOGE("remote","CONFIG MISSING: fill board_pins.h before using the handheld"); return;
    }
    ESP_ERROR_CHECK(gpio_set_direction(PIN_ENABLE_BUTTON,GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(PIN_ENABLE_BUTTON,GPIO_PULLUP_ONLY));
    gesture_imu_config_t imu={ .sda=PIN_IMU_SDA,.scl=PIN_IMU_SCL,
        .axis={IMU_AXIS_X,IMU_AXIS_Y,IMU_AXIS_Z},.sign={IMU_SIGN_X,IMU_SIGN_Y,IMU_SIGN_Z} };
    esp_err_t err=gesture_imu_start(&imu);
    if(err!=ESP_OK) { ESP_LOGE("remote","IMU init: %s",esp_err_to_name(err)); return; }
    ESP_ERROR_CHECK(gesture_ble_client_start(link_changed,build_frame));
    for(;;) {
        gesture_imu_sample_t s; gesture_imu_latest(&s);
        ESP_LOGI("remote","valid=%d forward=%.1f left=%.1f",s.valid,s.pitch,s.roll);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
