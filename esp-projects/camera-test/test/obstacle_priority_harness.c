#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../main/camera_line_follow.c"

static int64_t s_fake_time_us = 1000000;

int64_t esp_timer_get_time(void) { return s_fake_time_us; }
esp_err_t esp_timer_create(const esp_timer_create_args_t *args,
                           esp_timer_handle_t *handle)
{
    (void)args;
    *handle = (esp_timer_handle_t)1;
    return ESP_OK;
}
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us)
{ (void)timer; (void)period_us; return ESP_OK; }
esp_err_t esp_timer_stop(esp_timer_handle_t timer)
{ (void)timer; return ESP_OK; }

esp_err_t gpio_reset_pin(gpio_num_t pin) { (void)pin; return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode)
{ (void)pin; (void)mode; return ESP_OK; }
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level)
{ (void)pin; (void)level; return ESP_OK; }
int gpio_get_level(gpio_num_t pin) { (void)pin; return 0; }

esp_err_t ledc_timer_config(const ledc_timer_config_t *cfg)
{ (void)cfg; return ESP_OK; }
esp_err_t ledc_channel_config(const ledc_channel_config_t *cfg)
{ (void)cfg; return ESP_OK; }
esp_err_t ledc_set_duty(ledc_mode_t mode, ledc_channel_t channel, uint32_t duty)
{ (void)mode; (void)channel; (void)duty; return ESP_OK; }
esp_err_t ledc_update_duty(ledc_mode_t mode, ledc_channel_t channel)
{ (void)mode; (void)channel; return ESP_OK; }

SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (SemaphoreHandle_t)1; }
int xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks)
{ (void)semaphore; (void)ticks; return pdTRUE; }
int xSemaphoreGive(SemaphoreHandle_t semaphore)
{ (void)semaphore; return pdTRUE; }
int xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack, void *arg,
                uint32_t priority, TaskHandle_t *handle)
{ (void)fn; (void)name; (void)stack; (void)arg; (void)priority; (void)handle; return pdPASS; }
int xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack,
                            void *arg, uint32_t priority, TaskHandle_t *handle,
                            int core)
{ (void)fn; (void)name; (void)stack; (void)arg; (void)priority; (void)handle; (void)core; return pdPASS; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
void vTaskDelete(TaskHandle_t task) { (void)task; }
void vTaskPrioritySet(TaskHandle_t task, uint32_t priority)
{ (void)task; (void)priority; }

void ultrasonic_init(gpio_num_t trig, gpio_num_t echo) { (void)trig; (void)echo; }
float ultrasonic_read_cm(void) { return -1.0f; }
void camera_display_get_pipeline_stats(camera_display_pipeline_stats_t *stats)
{ memset(stats, 0, sizeof(*stats)); }

bool line_geometry_track(const uint8_t *frame, const line_scan_cfg_t *cfg,
                         line_observation_t *observation)
{
    (void)frame;
    (void)cfg;
    memset(observation, 0, sizeof(*observation));
    return false;
}
int line_geometry_scan_width(const line_scan_cfg_t *cfg) { return cfg->width; }
int line_geometry_scan_height(const line_scan_cfg_t *cfg) { return cfg->height; }
void line_geometry_map(const line_scan_cfg_t *cfg, int sx, int sy, int *bx, int *by)
{ (void)cfg; *bx = sx; *by = sy; }
int line_geometry_positive_percent(int value, int percent, int minimum)
{ int out = value * percent / 100; return out < minimum ? minimum : out; }

int line_control_yaw_pd(const line_control_cfg_t *cfg, int error,
                        int derivative, int kd, int *accum)
{ (void)cfg; (void)error; (void)derivative; (void)kd; (void)accum; return 0; }
int line_control_strafe_pd(const line_control_cfg_t *cfg, int error,
                           int derivative, int kd, int *accum)
{ (void)cfg; (void)error; (void)derivative; (void)kd; (void)accum; return 0; }
int line_control_speed(const line_control_cfg_t *cfg,
                       const line_observation_t *observation, bool alert)
{ (void)observation; (void)alert; return cfg->speed_crawl; }
void line_mixer_solve(int forward, int turn, int lat,
                      const line_mixer_cfg_t *cfg, line_mixer_out_t *out)
{ (void)forward; (void)turn; (void)lat; (void)cfg; memset(out, 0, sizeof(*out)); }

int main(void)
{
    uint8_t frame[4] = {0};
    s_started = true;
    s_finished = false;
    s_armed = false;
    s_stby_enabled = false;
    s_obstacle_completed = false;
    s_obstacle_state = OBSTACLE_IDLE;
    s_obstacle_close_samples = 0;
    s_first_frame_us = s_fake_time_us;

    obstacle_record_sample(true, 12.0f);
    obstacle_record_sample(true, 12.0f);
    obstacle_record_sample(false, -1.0f);
    assert(s_obstacle_close_samples == OBSTACLE_CLOSE_CONFIRM_SAMPLES);

    camera_line_follow_process_frame(frame, 1, 1, 0, false);

    if (s_obstacle_state != OBSTACLE_BRAKE || !s_stby_enabled) {
        fprintf(stderr,
                "FAIL: unarmed close obstacle did not own control: state=%d STBY=%d\n",
                (int)s_obstacle_state, s_stby_enabled ? 1 : 0);
        return 1;
    }

    s_fake_time_us += (int64_t)AVOID_BRAKE_MS * 1000;
    camera_line_follow_process_frame(frame, 1, 1, 0, false);
    assert(s_obstacle_state == OBSTACLE_LEFT);
    assert(s_stby_enabled);

    ++s_fake_time_us;
    camera_line_follow_process_frame(frame, 1, 1, 0, false);
    assert(s_command_a == -AVOID_LEFT_A_SPEED);
    assert(s_command_b == -AVOID_LEFT_B_SPEED);
    assert(s_command_d == -AVOID_LEFT_D_SPEED);

    puts("PASS: unarmed close obstacle owns BRAKE and LEFT with STBY enabled");
    return 0;
}
