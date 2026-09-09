#include "ultrasonic.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static gpio_num_t trig_pin;
static gpio_num_t echo_pin;

static float one_read_cm(void)
{
    gpio_set_level(trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig_pin, 0);

    int64_t wait_start = esp_timer_get_time();
    while (!gpio_get_level(echo_pin)) {
        if (esp_timer_get_time() - wait_start > 30000) return -1.0f;
    }
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin)) {
        if (esp_timer_get_time() - pulse_start > 30000) return -1.0f;
    }
    return (float)(esp_timer_get_time() - pulse_start) / 58.0f;
}

void ultrasonic_init(gpio_num_t trig_gpio, gpio_num_t echo_gpio)
{
    trig_pin = trig_gpio;
    echo_pin = echo_gpio;
    gpio_reset_pin(trig_pin);
    gpio_set_direction(trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(trig_pin, 0);
    gpio_reset_pin(echo_pin);
    gpio_set_direction(echo_pin, GPIO_MODE_INPUT);
}

float ultrasonic_read_cm(void)
{
    float a = one_read_cm();
    esp_rom_delay_us(1500);
    float b = one_read_cm();
    esp_rom_delay_us(1500);
    float c = one_read_cm();
    int valid = (a >= 0.0f) + (b >= 0.0f) + (c >= 0.0f);
    if (valid == 0) return -1.0f;
    if (valid < 3) {
        float sum = (a >= 0.0f ? a : 0.0f) + (b >= 0.0f ? b : 0.0f) + (c >= 0.0f ? c : 0.0f);
        return sum / valid;
    }
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}
