#include "ultrasonic.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"

static gpio_num_t s_trig_pin;
static gpio_num_t s_echo_pin;

static float read_once_cm(void)
{
    gpio_set_level(s_trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(s_trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s_trig_pin, 0);

    const int64_t wait_start = esp_timer_get_time();
    while (!gpio_get_level(s_echo_pin)) {
        if (esp_timer_get_time() - wait_start > 30000) {
            return -1.0f;
        }
    }

    const int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(s_echo_pin)) {
        if (esp_timer_get_time() - pulse_start > 30000) {
            return -1.0f;
        }
    }
    return (float)(esp_timer_get_time() - pulse_start) / 58.0f;
}

void ultrasonic_init(gpio_num_t trig_gpio, gpio_num_t echo_gpio)
{
    s_trig_pin = trig_gpio;
    s_echo_pin = echo_gpio;
    gpio_reset_pin(s_trig_pin);
    gpio_set_direction(s_trig_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_trig_pin, 0);
    gpio_reset_pin(s_echo_pin);
    gpio_set_direction(s_echo_pin, GPIO_MODE_INPUT);
}

float ultrasonic_read_cm(void)
{
    const float a = read_once_cm();
    esp_rom_delay_us(1500);
    const float b = read_once_cm();
    esp_rom_delay_us(1500);
    const float c = read_once_cm();
    const int valid = (a >= 0.0f) + (b >= 0.0f) + (c >= 0.0f);
    if (valid == 0) {
        return -1.0f;
    }
    if (valid < 3) {
        const float sum = (a >= 0.0f ? a : 0.0f) +
                          (b >= 0.0f ? b : 0.0f) +
                          (c >= 0.0f ? c : 0.0f);
        return sum / (float)valid;
    }

    /* Three samples are enough here; use the median to reject one echo spike. */
    float low = a;
    float mid = b;
    float high = c;
    if (low > mid) { float t = low; low = mid; mid = t; }
    if (mid > high) { float t = mid; mid = high; high = t; }
    if (low > mid) { float t = low; low = mid; mid = t; }
    return mid;
}
