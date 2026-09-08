#include "gesture_board.h"
#include "driver/gpio.h"
#include "esp_log.h"
esp_err_t gesture_board_validate(const gesture_pin_t *p, size_t n, bool straps) {
    bool valid = true;
    for (size_t i=0; i<n; ++i) {
        int g = p[i].gpio;
        bool exposed = (g >= 0 && g <= 21) || (g >= 38 && g <= 46);
        bool strap = g == 0 || g == 3 || g == 45 || g == 46;
        if (!exposed || g == 43 || g == 44 || (!straps && strap) ||
            !GPIO_IS_VALID_GPIO(g) || (p[i].output && !GPIO_IS_VALID_OUTPUT_GPIO(g))) {
            ESP_LOGE("pins", "%s=%d missing/reserved/unsupported; fill board_pins.h", p[i].name, g); valid = false;
        }
        for (size_t j=0; j<i; ++j) if (g >= 0 && g == p[j].gpio) {
            ESP_LOGE("pins", "%s conflicts with %s on GPIO%d", p[i].name, p[j].name, g); valid = false;
        }
    }
    return valid ? ESP_OK : ESP_ERR_INVALID_ARG;
}
