#include "endpoint_ball.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PAN_SERVO_PIN GPIO_NUM_1
#define TILT_SERVO_PIN GPIO_NUM_2
#define SERVO_TIMER LEDC_TIMER_1
#define SERVO_PAN_CHANNEL LEDC_CHANNEL_3
#define SERVO_TILT_CHANNEL LEDC_CHANNEL_4
#define SERVO_RESOLUTION LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY ((1U << 14) - 1U)
#define SERVO_PERIOD_US 20000U

#define BALL_PAN_CENTER_US 1500U
#define BALL_TILT_HIGH_US 1750U
#define BALL_SERVO_SETTLE_MS 700U
#define BALL_SERVO_STAGGER_MS 700U
#define BALL_SEARCH_TURN_SPEED 0.15f
#define BALL_ALIGN_TURN_SPEED 0.14f
#define BALL_MIN_RUN_SPEED 0.11f
#define BALL_B_MIN_RUN_SPEED 0.13f
#define BALL_CHARGE_A_SPEED 0.28f
#define BALL_CHARGE_D_SPEED 0.30f
#define BALL_CHARGE_MS 2100U
#define BALL_SEARCH_TIMEOUT_MS 14000U
#define BALL_ALIGN_TIMEOUT_MS 5000U
#define BALL_ALIGN_TOLERANCE_PX 7
#define BALL_ALIGN_CONFIRM_FRAMES 4U
#define BALL_FRAME_TIMEOUT_MS 1200U

#define BALL_MIN_PIXELS 10
#define BALL_SCAN_TOP_PERCENT 5
#define BALL_SCAN_BOTTOM_PERCENT 90
#define BALL_RED_MIN_R 90
#define BALL_RED_CHANNEL_GAP 35
#define BALL_GREEN_MIN_G 70
#define BALL_GREEN_CHANNEL_GAP 8
#define BALL_MIN_LUMA 35
#define BALL_MIN_FILL_PERCENT 50
#define BALL_MIN_EDGE_PIXELS 6
#define BALL_MIN_BORDER_MARGIN 4
#define BALL_MAX_WIDTH_PERCENT 20
#define BALL_GRID_STEP 1
#define BALL_GRID_MAX_CELLS 20000

typedef enum {
    BALL_IDLE = 0,
    BALL_INIT,
    BALL_SEARCH_RED,
    BALL_ALIGN_RED,
    BALL_FORWARD_RED,
    BALL_BACK_RED,
    BALL_SEARCH_GREEN,
    BALL_ALIGN_GREEN,
    BALL_FORWARD_GREEN,
    BALL_BACK_GREEN,
    BALL_DONE,
} ball_phase_t;

typedef enum {
    BALL_COLOUR_RED = 0,
    BALL_COLOUR_GREEN,
} ball_colour_t;

typedef struct {
    bool valid;
    int x;
    int y;
    int width;
    int height;
    int pixels;
    int edge_score;
} ball_blob_t;

static const char *TAG = "endpoint_ball";
static endpoint_ball_drive_fn_t drive_fn;
static endpoint_ball_stop_fn_t stop_fn;
static endpoint_ball_control_fn_t pause_fn;

static bool servo_initialized;
static volatile bool route_completed;
static volatile bool endpoint_latched;
static bool endpoint_paused;

static volatile ball_phase_t ball_phase;
static int64_t ball_phase_start_us;
static volatile int64_t ball_last_frame_us;
static uint8_t ball_align_frames;
static int ball_x;
static int ball_y;
static int ball_width;
static int ball_height;
static int ball_frame_width;
static int ball_frame_height;
static bool ball_frame_logged;
static bool ball_target_valid;
static bool ball_target_is_red;
static int ball_search_direction;
static uint8_t ball_target_miss_frames;
static int ball_motor_a;
static int ball_motor_b;
static int ball_motor_d;

static uint8_t ball_visited[BALL_GRID_MAX_CELLS];
static uint16_t ball_queue[BALL_GRID_MAX_CELLS];

static void ball_drive_stop(void);
static void ball_begin(int64_t now);

static void ball_watchdog_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int64_t now = esp_timer_get_time();
        if (endpoint_latched && ball_phase != BALL_DONE && ball_last_frame_us != 0 &&
            now - ball_last_frame_us > (int64_t)BALL_FRAME_TIMEOUT_MS * 1000) {
            ball_phase = BALL_DONE;
            ball_drive_stop();
            ESP_LOGE(TAG, "ball camera frame timeout; vehicle stopped");
        }
    }
}

static uint32_t servo_pulse_to_duty(uint32_t pulse_us)
{
    return (SERVO_MAX_DUTY * pulse_us) / SERVO_PERIOD_US;
}

static void servo_set_pulse(ledc_channel_t channel, uint32_t pulse_us)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel,
                                  servo_pulse_to_duty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static esp_err_t ball_servo_init(void)
{
    if (servo_initialized) return ESP_OK;

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = SERVO_TIMER,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) return err;

    const ledc_channel_config_t channels[] = {
        {
            .gpio_num = PAN_SERVO_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = SERVO_PAN_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_TIMER,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = TILT_SERVO_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = SERVO_TILT_CHANNEL,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_TIMER,
            .duty = 0,
            .hpoint = 0,
        },
    };
    for (size_t index = 0; index < sizeof(channels) / sizeof(channels[0]); index++) {
        err = ledc_channel_config(&channels[index]);
        if (err != ESP_OK) return err;
    }

    servo_set_pulse(SERVO_PAN_CHANNEL, BALL_PAN_CENTER_US);
    ESP_LOGI(TAG, "ball pan servo enabled; waiting %ums before tilt",
             (unsigned)BALL_SERVO_STAGGER_MS);
    vTaskDelay(pdMS_TO_TICKS(BALL_SERVO_STAGGER_MS));
    servo_set_pulse(SERVO_TILT_CHANNEL, BALL_TILT_HIGH_US);
    servo_initialized = true;
    ESP_LOGI(TAG, "ball servos ready: pan=center tilt=high");
    return ESP_OK;
}

static void ball_drive_direct(float a, float b, float d)
{
    ball_motor_a = (int)(a * 100.0f);
    ball_motor_b = (int)(b * 100.0f);
    ball_motor_d = (int)(d * 100.0f);
    drive_fn(a, b, d);
}

static void ball_drive_spin(int direction, float speed)
{
    float magnitude = speed < BALL_MIN_RUN_SPEED ? BALL_MIN_RUN_SPEED : speed;
    float rear = magnitude < BALL_B_MIN_RUN_SPEED ? BALL_B_MIN_RUN_SPEED : magnitude;
    ball_drive_direct(-direction * magnitude, direction * rear,
                      -direction * magnitude);
}

static void ball_drive_charge(bool reverse)
{
    float sign = reverse ? -1.0f : 1.0f;
    ball_drive_direct(-sign * BALL_CHARGE_A_SPEED, 0.0f,
                      sign * BALL_CHARGE_D_SPEED);
}

static void ball_drive_stop(void)
{
    ball_motor_a = 0;
    ball_motor_b = 0;
    ball_motor_d = 0;
    stop_fn();
}

static const char *ball_phase_name(void)
{
    switch (ball_phase) {
    case BALL_INIT: return "INIT";
    case BALL_SEARCH_RED: return "SEARCH RED";
    case BALL_ALIGN_RED: return "ALIGN RED";
    case BALL_FORWARD_RED: return "FORWARD RED";
    case BALL_BACK_RED: return "BACK RED";
    case BALL_SEARCH_GREEN: return "SEARCH GREEN";
    case BALL_ALIGN_GREEN: return "ALIGN GREEN";
    case BALL_FORWARD_GREEN: return "FORWARD GREEN";
    case BALL_BACK_GREEN: return "BACK GREEN";
    case BALL_DONE: return "DONE";
    case BALL_IDLE:
    default: return "IDLE";
    }
}

static uint8_t ball_luma(const uint8_t *pixel, int *red, int *green, int *blue)
{
    uint16_t value = ((uint16_t)pixel[0] << 8) | pixel[1];
    int r = ((value >> 11) & 0x1f) * 255 / 31;
    int g = ((value >> 5) & 0x3f) * 255 / 63;
    int b = (value & 0x1f) * 255 / 31;
    if (red != NULL) *red = r;
    if (green != NULL) *green = g;
    if (blue != NULL) *blue = b;
    return (uint8_t)((77 * r + 150 * g + 29 * b) >> 8);
}

static bool ball_pixel_matches(const uint8_t *frame, uint16_t width,
                               uint16_t height, int x, int y,
                               ball_colour_t colour)
{
    if (frame == NULL || x < 0 || y < 0 || x >= (int)width || y >= (int)height) {
        return false;
    }
    const uint8_t *pixel = frame + (((size_t)y * width + (size_t)x) * 2U);
    int red = 0;
    int green = 0;
    int blue = 0;
    int luma = ball_luma(pixel, &red, &green, &blue);
    if (colour == BALL_COLOUR_RED) {
        return luma >= BALL_MIN_LUMA && red >= BALL_RED_MIN_R &&
               red > green + BALL_RED_CHANNEL_GAP &&
               red > blue + BALL_RED_CHANNEL_GAP;
    }
    return luma >= BALL_MIN_LUMA && green >= BALL_GREEN_MIN_G &&
           green > red + BALL_GREEN_CHANNEL_GAP &&
           green > blue + BALL_GREEN_CHANNEL_GAP;
}

static bool find_ball_blob(const uint8_t *frame, uint16_t width, uint16_t height,
                           ball_colour_t colour, ball_blob_t *blob)
{
    if (frame == NULL || blob == NULL || width < 16 || height < 16) return false;

    *blob = (ball_blob_t){0};
    int grid_w = ((int)width + BALL_GRID_STEP - 1) / BALL_GRID_STEP;
    int grid_h = ((int)height + BALL_GRID_STEP - 1) / BALL_GRID_STEP;
    size_t cells = (size_t)grid_w * (size_t)grid_h;
    if (cells > BALL_GRID_MAX_CELLS) return false;
    memset(ball_visited, 0, cells);

    int top = (int)height * BALL_SCAN_TOP_PERCENT / 100;
    int bottom = (int)height * BALL_SCAN_BOTTOM_PERCENT / 100;
    int best_score = 0;
    ball_blob_t best = {0};

    for (int grid_y = 0; grid_y < grid_h; grid_y++) {
        int seed_y = grid_y * BALL_GRID_STEP;
        if (seed_y < top || seed_y >= bottom) continue;
        for (int grid_x = 0; grid_x < grid_w; grid_x++) {
            size_t seed = (size_t)grid_y * grid_w + (size_t)grid_x;
            int seed_x = grid_x * BALL_GRID_STEP;
            if (ball_visited[seed] ||
                !ball_pixel_matches(frame, width, height, seed_x, seed_y, colour)) {
                continue;
            }

            ball_visited[seed] = 1;
            size_t head = 0;
            size_t tail = 0;
            ball_queue[tail++] = (uint16_t)seed;
            int count = 0;
            int sum_x = 0;
            int sum_y = 0;
            int left = width;
            int right = 0;
            int top_y = height;
            int bottom_y = 0;

            while (head < tail) {
                uint16_t cell = ball_queue[head++];
                int cell_grid_x = cell % grid_w;
                int cell_grid_y = cell / grid_w;
                int cell_x = cell_grid_x * BALL_GRID_STEP;
                int cell_y = cell_grid_y * BALL_GRID_STEP;
                count++;
                sum_x += cell_x;
                sum_y += cell_y;
                if (cell_x < left) left = cell_x;
                if (cell_x > right) right = cell_x;
                if (cell_y < top_y) top_y = cell_y;
                if (cell_y > bottom_y) bottom_y = cell_y;

                for (int delta_y = -1; delta_y <= 1; delta_y++) {
                    for (int delta_x = -1; delta_x <= 1; delta_x++) {
                        if (delta_x == 0 && delta_y == 0) continue;
                        int next_x = cell_grid_x + delta_x;
                        int next_y = cell_grid_y + delta_y;
                        if (next_x < 0 || next_y < 0 || next_x >= grid_w || next_y >= grid_h) {
                            continue;
                        }
                        size_t next = (size_t)next_y * grid_w + (size_t)next_x;
                        if (ball_visited[next]) continue;
                        int pixel_x = next_x * BALL_GRID_STEP;
                        int pixel_y = next_y * BALL_GRID_STEP;
                        if (pixel_y < top || pixel_y >= bottom ||
                            !ball_pixel_matches(frame, width, height, pixel_x, pixel_y,
                                                colour)) {
                            continue;
                        }
                        ball_visited[next] = 1;
                        if (tail < BALL_GRID_MAX_CELLS) ball_queue[tail++] = (uint16_t)next;
                    }
                }
            }

            int box_w = right - left + BALL_GRID_STEP;
            int box_h = bottom_y - top_y + BALL_GRID_STEP;
            int sample_box_w = (right - left) / BALL_GRID_STEP + 1;
            int sample_box_h = (bottom_y - top_y) / BALL_GRID_STEP + 1;
            int sample_box_area = sample_box_w * sample_box_h;
            int fill_percent = sample_box_area > 0 ? count * 100 / sample_box_area : 0;
            if (count < BALL_MIN_PIXELS || box_w < 4 || box_h < 4 ||
                left <= BALL_MIN_BORDER_MARGIN || top_y <= BALL_MIN_BORDER_MARGIN ||
                right >= (int)width - 1 - BALL_MIN_BORDER_MARGIN ||
                bottom_y >= (int)height - 1 - BALL_MIN_BORDER_MARGIN ||
                fill_percent < BALL_MIN_FILL_PERCENT ||
                box_w > (int)width * BALL_MAX_WIDTH_PERCENT / 100 ||
                box_h > (int)height * BALL_MAX_WIDTH_PERCENT / 100 ||
                box_w > box_h * 22 / 10 || box_h > box_w * 22 / 10) {
                continue;
            }

            int edge_score = 0;
            for (int y = top_y - 3; y <= bottom_y + 3; y += BALL_GRID_STEP) {
                for (int x = left - 3; x <= right + 3; x += BALL_GRID_STEP) {
                    if (x < 0 || y < 0 || x >= (int)width || y >= (int)height) continue;
                    if ((x >= left && x <= right && y >= top_y && y <= bottom_y) ||
                        ball_pixel_matches(frame, width, height, x, y, colour)) {
                        continue;
                    }
                    edge_score++;
                }
            }
            if (edge_score < BALL_MIN_EDGE_PIXELS) continue;

            int score = count * fill_percent + edge_score * 6;
            if (score > best_score) {
                best_score = score;
                best.valid = true;
                best.x = sum_x / count;
                best.y = sum_y / count;
                best.width = box_w;
                best.height = box_h;
                best.pixels = count * BALL_GRID_STEP * BALL_GRID_STEP;
                best.edge_score = edge_score;
            }
        }
    }

    if (best_score <= 0) return false;
    *blob = best;
    return true;
}

static bool ball_is_search_phase(void)
{
    return ball_phase == BALL_SEARCH_RED || ball_phase == BALL_SEARCH_GREEN;
}

static ball_colour_t ball_target_colour(void)
{
    return (ball_phase == BALL_SEARCH_RED || ball_phase == BALL_ALIGN_RED ||
            ball_phase == BALL_FORWARD_RED || ball_phase == BALL_BACK_RED) ?
           BALL_COLOUR_RED : BALL_COLOUR_GREEN;
}

static const char *ball_colour_name(ball_colour_t colour)
{
    return colour == BALL_COLOUR_RED ? "red" : "green";
}

static void ball_begin(int64_t now)
{
    if (ball_servo_init() != ESP_OK) {
        ESP_LOGE(TAG, "ball task disabled: servo init failed");
        ball_phase = BALL_DONE;
        ball_drive_stop();
        return;
    }
    servo_set_pulse(SERVO_PAN_CHANNEL, BALL_PAN_CENTER_US);
    servo_set_pulse(SERVO_TILT_CHANNEL, BALL_TILT_HIGH_US);
    ball_phase = BALL_INIT;
    ball_phase_start_us = now;
    ball_align_frames = 0;
    ball_target_valid = false;
    ball_x = 0;
    ball_y = 0;
    ball_width = 0;
    ball_height = 0;
    ball_frame_width = 0;
    ball_frame_height = 0;
    ball_frame_logged = false;
    ball_target_is_red = true;
    ball_search_direction = 1;
    ball_target_miss_frames = 0;
    ball_drive_stop();
    ESP_LOGW(TAG, "fixed forward finished; initializing ball task for %ums",
             (unsigned)BALL_SERVO_SETTLE_MS);
}

static void ball_next_search(int64_t now)
{
    ball_phase = BALL_SEARCH_GREEN;
    ball_phase_start_us = now;
    ball_align_frames = 0;
    ball_target_valid = false;
    ball_width = 0;
    ball_height = 0;
    ball_target_is_red = false;
    ball_search_direction = 1;
    ball_target_miss_frames = 0;
    ball_drive_stop();
    ESP_LOGI(TAG, "red complete; searching GREEN");
}

static void ball_begin_charge(int64_t now, ball_colour_t colour)
{
    ball_phase = colour == BALL_COLOUR_RED ? BALL_FORWARD_RED : BALL_FORWARD_GREEN;
    ball_phase_start_us = now;
    ball_align_frames = 0;
    ball_target_miss_frames = 0;
    ball_target_valid = false;
    ball_width = 0;
    ball_height = 0;
    ball_drive_charge(false);
    ESP_LOGI(TAG, "%s centered; charging forward for %ums",
             ball_colour_name(colour), (unsigned)BALL_CHARGE_MS);
}

static void ball_finish_charge(int64_t now, ball_colour_t colour)
{
    ball_phase = colour == BALL_COLOUR_RED ? BALL_BACK_RED : BALL_BACK_GREEN;
    ball_phase_start_us = now;
    ball_drive_charge(true);
    ESP_LOGI(TAG, "%s forward complete; reversing for %ums",
             ball_colour_name(colour), (unsigned)BALL_CHARGE_MS);
}

static void ball_process_frame(const uint8_t *frame, uint16_t width,
                               uint16_t height, int64_t now)
{
    if (frame == NULL || ball_phase == BALL_IDLE || ball_phase == BALL_DONE) return;

    if (ball_phase == BALL_INIT) {
        ball_drive_stop();
        if (now - ball_phase_start_us >= (int64_t)BALL_SERVO_SETTLE_MS * 1000) {
            ball_phase = BALL_SEARCH_RED;
            ball_phase_start_us = now;
            ESP_LOGI(TAG, "ball initialization complete; searching RED");
        }
        return;
    }

    ball_colour_t colour = ball_target_colour();
    bool red_target = colour == BALL_COLOUR_RED;
    if (ball_phase == BALL_FORWARD_RED || ball_phase == BALL_FORWARD_GREEN) {
        ball_drive_charge(false);
        if (now - ball_phase_start_us >= (int64_t)BALL_CHARGE_MS * 1000) {
            ball_finish_charge(now, colour);
        }
        return;
    }
    if (ball_phase == BALL_BACK_RED || ball_phase == BALL_BACK_GREEN) {
        ball_drive_charge(true);
        if (now - ball_phase_start_us >= (int64_t)BALL_CHARGE_MS * 1000) {
            ball_drive_stop();
            if (red_target) {
                ball_next_search(now);
            } else {
                ball_phase = BALL_DONE;
                ESP_LOGW(TAG, "red and green complete; vehicle stopped");
            }
        }
        return;
    }

    ball_blob_t blob = {0};
    bool found_ball = find_ball_blob(frame, width, height, colour, &blob);
    if (ball_is_search_phase()) {
        if (found_ball) {
            ball_x = blob.x;
            ball_y = blob.y;
            ball_width = blob.width;
            ball_height = blob.height;
            ball_target_valid = true;
            ball_target_is_red = red_target;
            ball_phase = red_target ? BALL_ALIGN_RED : BALL_ALIGN_GREEN;
            ball_phase_start_us = now;
            ball_align_frames = 0;
            ball_target_miss_frames = 0;
            ball_drive_stop();
            ESP_LOGI(TAG, "%s ball found at (%d,%d) pixels=%d; aligning",
                     ball_colour_name(colour), blob.x, blob.y, blob.pixels);
            return;
        }

        int64_t elapsed = now - ball_phase_start_us;
        if (elapsed > (int64_t)BALL_SEARCH_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "%s ball search timeout; stopping safely",
                     ball_colour_name(colour));
            ball_phase = BALL_DONE;
            ball_drive_stop();
        } else {
            if (elapsed > (int64_t)BALL_SEARCH_TIMEOUT_MS * 500 &&
                ball_search_direction > 0) {
                ball_search_direction = -1;
            }
            ball_drive_spin(ball_search_direction, BALL_SEARCH_TURN_SPEED);
        }
        return;
    }

    if (ball_phase == BALL_ALIGN_RED || ball_phase == BALL_ALIGN_GREEN) {
        if (found_ball) {
            ball_x = blob.x;
            ball_y = blob.y;
            ball_width = blob.width;
            ball_height = blob.height;
            ball_target_valid = true;
            ball_target_is_red = red_target;
            ball_target_miss_frames = 0;
        } else if (ball_target_miss_frames < 3U) {
            ball_target_miss_frames++;
        } else {
            ball_phase = red_target ? BALL_SEARCH_RED : BALL_SEARCH_GREEN;
            ball_phase_start_us = now;
            ball_target_valid = false;
            ball_width = 0;
            ball_height = 0;
            ESP_LOGW(TAG, "%s ball lost during alignment; resuming search",
                     ball_colour_name(colour));
            return;
        }

        if (!ball_target_valid) {
            ball_drive_spin(ball_search_direction, BALL_ALIGN_TURN_SPEED);
        } else {
            int error = ball_x - (int)width / 2;
            if (abs(error) <= BALL_ALIGN_TOLERANCE_PX) {
                if (ball_align_frames < BALL_ALIGN_CONFIRM_FRAMES) ball_align_frames++;
                ball_drive_stop();
            } else {
                ball_align_frames = 0;
                int direction = error > 0 ? -1 : 1;
                ball_drive_spin(direction, BALL_ALIGN_TURN_SPEED);
            }
            if (ball_align_frames >= BALL_ALIGN_CONFIRM_FRAMES) {
                ball_drive_stop();
                ball_align_frames = 0;
                ESP_LOGI(TAG, "%s centered ball=(%d,%d); preparing charge",
                         ball_colour_name(colour), ball_x, ball_y);
                ball_begin_charge(now, colour);
                return;
            }
        }

        if (now - ball_phase_start_us > (int64_t)BALL_ALIGN_TIMEOUT_MS * 1000) {
            ESP_LOGW(TAG, "%s alignment timeout; returning to search",
                     ball_colour_name(colour));
            ball_phase = red_target ? BALL_SEARCH_RED : BALL_SEARCH_GREEN;
            ball_phase_start_us = now;
            ball_target_valid = false;
            ball_width = 0;
            ball_height = 0;
        }
    }
}

esp_err_t endpoint_ball_init(endpoint_ball_drive_fn_t drive,
                             endpoint_ball_stop_fn_t stop,
                             endpoint_ball_control_fn_t pause)
{
    if (drive == NULL || stop == NULL || pause == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    drive_fn = drive;
    stop_fn = stop;
    pause_fn = pause;
    servo_initialized = false;
    route_completed = false;
    endpoint_latched = false;
    endpoint_paused = false;
    ball_phase = BALL_IDLE;
    ball_last_frame_us = 0;
    ball_x = 0;
    ball_y = 0;
    ball_width = 0;
    ball_height = 0;
    ball_frame_width = 0;
    ball_frame_height = 0;
    ball_frame_logged = false;
    ball_target_valid = false;
    ball_target_is_red = false;
    ball_motor_a = 0;
    ball_motor_b = 0;
    ball_motor_d = 0;
    if (xTaskCreate(ball_watchdog_task, "ball_watchdog", 2048, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void endpoint_ball_process_frame(const uint8_t *frame,
                                 uint16_t width,
                                 uint16_t height)
{
    if (drive_fn == NULL || stop_fn == NULL || pause_fn == NULL) return;

    int64_t now = esp_timer_get_time();
    if (!endpoint_latched || frame == NULL || width == 0 || height == 0) return;
    ball_last_frame_us = now;
    ball_frame_width = width;
    ball_frame_height = height;
    if (!ball_frame_logged) {
        ESP_LOGI(TAG, "ball recognition frame %ux%u RGB565-BE",
                 (unsigned)width, (unsigned)height);
        ball_frame_logged = true;
    }
    ball_process_frame(frame, width, height, now);
}

void endpoint_ball_start(void)
{
    if (drive_fn == NULL || stop_fn == NULL || pause_fn == NULL || route_completed) return;

    endpoint_paused = true;
    pause_fn();
    int64_t now = esp_timer_get_time();
    ball_begin(now);
    ball_last_frame_us = now;
    endpoint_latched = true;
    route_completed = true;
    ESP_LOGI(TAG, "fixed obstacle route complete; ball program active");
}

bool endpoint_ball_active(void)
{
    return route_completed;
}

void endpoint_ball_get_status(endpoint_ball_status_t *status)
{
    if (status == NULL) return;
    *status = (endpoint_ball_status_t){
        .route_completed = route_completed,
        .paused = endpoint_paused,
        .active = route_completed,
        .phase = ball_phase_name(),
        .target_valid = ball_target_valid,
        .target_is_red = ball_target_is_red,
        .target_x = ball_x,
        .target_y = ball_y,
        .target_width = ball_width,
        .target_height = ball_height,
        .target_frame_width = ball_frame_width,
        .target_frame_height = ball_frame_height,
        .motor_a = ball_motor_a,
        .motor_b = ball_motor_b,
        .motor_d = ball_motor_d,
    };
}
