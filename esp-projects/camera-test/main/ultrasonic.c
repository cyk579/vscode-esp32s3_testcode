#include "ultrasonic.h"

#include "esp_rom_sys.h"
#include "esp_timer.h"

/*
 * HC-SR04 timing.  The read is a busy-wait inside a high-priority task, so
 * every microsecond spent here is stolen from JPEG decoding and USB handling.
 * Keep the bounds tight:
 *   - the echo line normally rises well under 1 ms after the trigger; no rise
 *     within ULTRASONIC_ECHO_START_TIMEOUT_US means the sensor is absent.
 *   - the vehicle only needs "closer than OBSTACLE_DETECT_CM or not", so an
 *     echo longer than ULTRASONIC_ECHO_MAX_US (about 1.4 m) is reported as
 *     that distance instead of being waited out (a missing echo keeps the
 *     line high for ~38 ms on this sensor).
 * The previous median-of-three read could spend up to 180 ms per 70 ms period
 * when nothing was in range (or the sensor was unplugged), starving the decode
 * and USB tasks.  This spends at most 11 ms.  Spurious samples are rejected by
 * the caller's consecutive-sample confirmation instead of a median filter.
 */
#define ULTRASONIC_ECHO_START_TIMEOUT_US 3000
#define ULTRASONIC_ECHO_MAX_US 8000

static gpio_num_t s_trig_pin;
static gpio_num_t s_echo_pin;

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
    gpio_set_level(s_trig_pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(s_trig_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s_trig_pin, 0);

    const int64_t wait_start = esp_timer_get_time();
    while (!gpio_get_level(s_echo_pin)) {
        if (esp_timer_get_time() - wait_start > ULTRASONIC_ECHO_START_TIMEOUT_US) {
            return -1.0f;
        }
    }

    const int64_t pulse_start = esp_timer_get_time();
    int64_t pulse_us = 0;
    while (gpio_get_level(s_echo_pin)) {
        pulse_us = esp_timer_get_time() - pulse_start;
        if (pulse_us >= ULTRASONIC_ECHO_MAX_US) {
            break;
        }
    }
    return (float)pulse_us / 58.0f;
}
