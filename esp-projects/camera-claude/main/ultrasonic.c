#include "ultrasonic.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* 引脚见 board_pins.h。本车 18/11，他们 3/14。 */
#define TRIG_PIN PIN_ULTRASONIC_TRIG
#define ECHO_PIN PIN_ULTRASONIC_ECHO

void ultrasonic_init(void) {
    // Trig 设置为输出，Echo 设置为输入
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(TRIG_PIN, 0); // 默认拉低
}

float ultrasonic_get_distance_cm(void) {
    // 1. 给 Trig 发送一个持续 10 微秒的高电平脉冲，触发测距
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10); 
    gpio_set_level(TRIG_PIN, 0);

    // 2. 等待 Echo 引脚变高 (超声波刚发出的时刻)
    // 加上 30ms 超时限制，防止传感器没接好导致死循环卡死程序
    int64_t timeout = esp_timer_get_time() + 30000; 
    while (gpio_get_level(ECHO_PIN) == 0 && esp_timer_get_time() < timeout) {}
    
    int64_t start_time = esp_timer_get_time();
    
    // 3. 等待 Echo 引脚变低 (超声波返回被接收的时刻)
    while (gpio_get_level(ECHO_PIN) == 1 && esp_timer_get_time() < timeout) {}
    
    int64_t end_time = esp_timer_get_time();
    
    // 4. 计算距离
    // 时间差(微秒) * 声速(0.034 厘米/微秒) / 2 (因为是来回双程)
    // 0.034 / 2 = 0.017
    float distance = (float)(end_time - start_time) * 0.017f;
    
    // 如果超过 400cm (4米)，通常超出了传感器的有效范围，返回 -1 表示无效
    if (distance > 400.0f) {
        return -1.0f;
    }
    return distance;
}