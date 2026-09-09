#include "motor.h"
#include "board_pins.h"
#include "control_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>
static const int in1[]={PIN_AIN1,PIN_BIN1,PIN_DIN1}, in2[]={PIN_AIN2,PIN_BIN2,PIN_DIN2};
static const int pins[]={PIN_PWMA,PIN_PWMB,PIN_PWMD};
static const int signs[]={MOTOR_A_SIGN,MOTOR_B_SIGN,MOTOR_D_SIGN};
static bool initialized, standby_known;
static int directions[3];
static esp_err_t duty(int i,uint32_t value) {
    esp_err_t err=ledc_set_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)i,value);
    return err==ESP_OK ? ledc_update_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)i) : err;
}
void motor_stop(void) {
    if(standby_known) gpio_set_level(PIN_MOTOR_STBY,0);
    if(!initialized) return;
    for(int i=0;i<3;++i) { duty(i,0); gpio_set_level(in1[i],0); gpio_set_level(in2[i],0); directions[i]=0; }
}
esp_err_t motor_init(void) {
    esp_err_t err;
    if(PIN_MOTOR_STBY<0) return ESP_ERR_INVALID_ARG;
    for(int i=0;i<3;++i) if(pins[i]<0 || in1[i]<0 || in2[i]<0 || (signs[i]!=1 && signs[i]!=-1)) return ESP_ERR_INVALID_ARG;
    gpio_set_level(PIN_MOTOR_STBY,0);
    if((err=gpio_set_direction(PIN_MOTOR_STBY,GPIO_MODE_OUTPUT))!=ESP_OK) return err;
    standby_known=true;
    for(int i=0;i<3;++i) {
        gpio_set_level(in1[i],0); gpio_set_level(in2[i],0);
        if((err=gpio_set_direction(in1[i],GPIO_MODE_OUTPUT))!=ESP_OK) return err;
        if((err=gpio_set_direction(in2[i],GPIO_MODE_OUTPUT))!=ESP_OK) return err;
    }
    ledc_timer_config_t timer={ .speed_mode=LEDC_LOW_SPEED_MODE, .duty_resolution=LEDC_TIMER_13_BIT,
        .timer_num=LEDC_TIMER_0, .freq_hz=5000, .clk_cfg=LEDC_AUTO_CLK };
    if((err=ledc_timer_config(&timer))!=ESP_OK) return err;
    for(int i=0;i<3;++i) {
        ledc_channel_config_t ch={ .gpio_num=pins[i], .speed_mode=LEDC_LOW_SPEED_MODE,
            .channel=(ledc_channel_t)i, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 };
        if((err=ledc_channel_config(&ch))!=ESP_OK) return err;
    }
    initialized=true; motor_stop(); return ESP_OK;
}
esp_err_t motor_apply(const float pwm[3],bool enabled) {
    if(!initialized) return ESP_ERR_INVALID_STATE;
    if(!enabled) { motor_stop(); return ESP_OK; }
    for(int i=0;i<3;++i) if(!isfinite(pwm[i]) || fabsf(pwm[i])>CONTROL_MAX_PWM+0.01f) { motor_stop(); return ESP_ERR_INVALID_ARG; }
    esp_err_t err;
    for(int i=0;i<3;++i) {
        float p=pwm[i]*signs[i];
        int direction=(p>0)-(p<0);
        if(direction!=directions[i]) {
            if((err=duty(i,0))!=ESP_OK) goto failed;
            if((err=gpio_set_level(in1[i],direction>0))!=ESP_OK) goto failed;
            if((err=gpio_set_level(in2[i],direction<0))!=ESP_OK) goto failed;
            directions[i]=direction;
        }
        if((err=duty(i,(uint32_t)lroundf(fabsf(p)*8191/100)))!=ESP_OK) goto failed;
    }
    if((err=gpio_set_level(PIN_MOTOR_STBY,1))!=ESP_OK) goto failed;
    return ESP_OK;
failed:
    motor_stop(); return err;
}
