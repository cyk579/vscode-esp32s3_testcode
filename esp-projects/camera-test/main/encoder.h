#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    ENCODER_WHEEL_A = 0,
    ENCODER_WHEEL_B,
    ENCODER_WHEEL_D,
    ENCODER_WHEEL_COUNT,
};

/* Signed encoder telemetry.  rate_cps is quadrature counts per second; the
 * configured count-per-revolution value is reported for later calibration. */
typedef struct {
    bool ready;
    uint32_t sample_period_ms;
    uint32_t counts_per_revolution;
    int32_t delta[ENCODER_WHEEL_COUNT];
    int32_t rate_cps[ENCODER_WHEEL_COUNT];
    int32_t total[ENCODER_WHEEL_COUNT];
} encoder_snapshot_t;

/* Initialise the three PCNT quadrature inputs and their sampling task. */
esp_err_t encoder_init(void);

/* Copy the latest coherent snapshot.  Returns false when initialisation failed. */
bool encoder_get_snapshot(encoder_snapshot_t *snapshot);

/* Reset accumulated counts and the filtered rate without touching the motors. */
void encoder_reset_counts(void);

#ifdef __cplusplus
}
#endif
