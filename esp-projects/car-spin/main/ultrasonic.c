#include "ultrasonic.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define ECHO_TIMEOUT_US 30000
#define MIN_DISTANCE_CM 2.0f
#define MAX_DISTANCE_CM 400.0f

static gpio_num_t trig_pin;
static gpio_num_t echo_pin;
static float history[3];
static uint8_t history_count;
static uint8_t history_index;

static float one_read_cm(ultrasonic_status_t *status)
{
    gpio_set_level(trig_pin, 0);

    /* 上一轮回波尚未结束时不重触发，避免把旧回波当作新测距结果。 */
    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin)) {
        if (esp_timer_get_time() - wait_start > ECHO_TIMEOUT_US) {
            *status = ULTRASONIC_ECHO_STUCK_HIGH;
            return -1.0f;
        }
    }
    esp_rom_delay_us(2);
    gpio_set_level(trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig_pin, 0);

    wait_start = esp_timer_get_time();
    while (!gpio_get_level(echo_pin)) {
        if (esp_timer_get_time() - wait_start > ECHO_TIMEOUT_US) {
            *status = ULTRASONIC_NO_ECHO;
            return -1.0f;
        }
    }
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin)) {
        if (esp_timer_get_time() - pulse_start > ECHO_TIMEOUT_US) {
            *status = ULTRASONIC_ECHO_STUCK_HIGH;
            return -1.0f;
        }
    }
    *status = ULTRASONIC_OK;
    return (float)(esp_timer_get_time() - pulse_start) / 58.0f;
}

static float median(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
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
    gpio_set_pull_mode(echo_pin, GPIO_PULLDOWN_ONLY);
    history_count = 0;
    history_index = 0;
}

float ultrasonic_read_cm(ultrasonic_status_t *status)
{
    float distance = one_read_cm(status);
    if (*status != ULTRASONIC_OK) return -1.0f;
    if (distance < MIN_DISTANCE_CM || distance > MAX_DISTANCE_CM) {
        *status = ULTRASONIC_OUT_OF_RANGE;
        return -1.0f;
    }

    history[history_index] = distance;
    history_index = (history_index + 1U) % 3U;
    if (history_count < 3U) ++history_count;
    if (history_count < 3U) return distance;
    return median(history[0], history[1], history[2]);
}

const char *ultrasonic_status_name(ultrasonic_status_t status)
{
    switch (status) {
    case ULTRASONIC_NO_ECHO: return "NO_ECHO";
    case ULTRASONIC_ECHO_STUCK_HIGH: return "ECHO_HIGH";
    case ULTRASONIC_OUT_OF_RANGE: return "OUT_OF_RANGE";
    default: return "OK";
    }
}
