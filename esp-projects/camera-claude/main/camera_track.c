#include "camera_track.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "usb_stream.h"
#include "esp_heap_caps.h"
#include <math.h>
#include "esp_jpeg_dec.h"
#include "wifi_mjpeg.h"
#include <string.h>

#include "board_pins.h"

/* 引脚见 board_pins.h。本车俯仰=2、水平=1；他们俯仰=18、水平=8。
 * 他们那两个脚在本车上是超声波 TRIG(18) 和 TB6612 的 STBY(8)，
 * 照抄的话舵机 PWM 会去砍电机使能，三个轮子跟着 50Hz 抽。 */
#define SERVO_PITCH_PIN PIN_SERVO_PITCH
#define SERVO_YAW_PIN   PIN_SERVO_YAW

/* 巡线时摄像头要往下看地面。这个角度的正负取决于舵机装配方向，
 * 必须实测：屏幕/网页里看到天花板就把它改成负值。 */
#define SERVO_INIT_PITCH_DEG 30.0f
#define SERVO_INIT_YAW_DEG 0.0f

static const char *TAG = "CAM_TRACK";

#define CAM_FRAME_WIDTH   480
#define CAM_FRAME_HEIGHT  320
#define CAM_FRAME_INTERVAL 400000

#define CAM_XFER_BUF_SIZE  (32 * 1024)
#define CAM_FRAME_BUF_SIZE (120 * 1024)

#define CAM_LINE_LOST_CONFIRM_COUNT 3
#define CAM_CROP_ROWS_LINE 24   // 循迹时只看最下面 24 行
#define CAM_CROP_ROWS_BALL 320 // 找球时看下半个画面的 320 行

// ================= 全局变量 =================
static volatile float camera_track_error = 0.0f;
static volatile int camera_line_lost = 0;
static int camera_line_lost_count = 0;

// 🌟 找球状态变量
static volatile vision_mode_t current_vision_mode = VISION_MODE_LINE;
static volatile float ball_track_error = 0.0f;
static volatile int ball_detected = 0;
static int current_crop_rows = CAM_CROP_ROWS_LINE; 

static QueueHandle_t frame_queue = NULL;
#define FRAME_QUEUE_LENGTH 3

typedef struct {
    uint8_t *data;
    size_t len;
    uint16_t width;
    uint16_t height;
} proc_frame_t;

static QueueHandle_t proc_queue = NULL;
#define PROC_QUEUE_LENGTH 2

// ================= 舵机与状态接口 =================

static void servos_init(void) { 
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_TIMER_14_BIT, 
        .freq_hz          = 50,                
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t pitch_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_3,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PITCH_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&pitch_channel);

    ledc_channel_config_t yaw_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_4,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_YAW_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&yaw_channel);
}

void camera_set_pan_tilt(float yaw_angle, float pitch_angle) {
    if (yaw_angle < -90) yaw_angle = -90;
    if (yaw_angle > 90) yaw_angle = 90;
    if (pitch_angle < -90) pitch_angle = -90;
    if (pitch_angle > 90) pitch_angle = 90;

   uint32_t yaw_duty = (uint32_t)(1229 + (int)((yaw_angle / 90.0f) * 819));
    uint32_t pitch_duty = (uint32_t)(1229 + (int)((pitch_angle / 90.0f) * 819));

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, yaw_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, pitch_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

// 🌟 对外提供的找球接口
void camera_track_set_mode(vision_mode_t mode) {
    current_vision_mode = mode;
    if (mode == VISION_MODE_LINE) {
        current_crop_rows = CAM_CROP_ROWS_LINE; 
    } else {
        current_crop_rows = CAM_CROP_ROWS_BALL; 
    }
}
float get_ball_track_error(void) { return ball_track_error; }
int is_ball_detected(void) { return ball_detected; }
float get_camera_track_error(void) { return camera_track_error; }
int camera_track_is_line_lost(void) { return camera_line_lost; }
QueueHandle_t camera_track_get_frame_queue(void) { return frame_queue; }

static void update_line_lost_state(int line_found_this_frame) {
    if (line_found_this_frame) {
        camera_line_lost_count = 0;
        camera_line_lost = 0;
    } else {
        camera_line_lost_count++;
        if (camera_line_lost_count >= CAM_LINE_LOST_CONFIRM_COUNT) {
            camera_line_lost = 1;
            camera_line_lost_count = CAM_LINE_LOST_CONFIRM_COUNT;
        }
    }
}

static inline uint8_t ycbyry_get_y(const uint8_t *row, int x) {
    int pair = x >> 1;
    int off = (x & 1) ? 3 : 1;
    return row[pair * 4 + off];
}

// ================= USB 视频流回调 =================
static void camera_frame_cb(uvc_frame_t *frame, void *ptr) {
    if (frame_queue != NULL) {
        uint8_t *jpeg_copy = (uint8_t *)heap_caps_malloc(frame->data_bytes, MALLOC_CAP_SPIRAM);
        if (jpeg_copy) {
            memcpy(jpeg_copy, frame->data, frame->data_bytes);
            frame_item_t item = { .data = jpeg_copy, .len = frame->data_bytes };
            if (xQueueSend(frame_queue, &item, 0) != pdPASS) {
                free(jpeg_copy);
            }
        }
    }

    if (proc_queue != NULL) {
        proc_frame_t pf;
        pf.data = (uint8_t *)heap_caps_malloc(frame->data_bytes, MALLOC_CAP_SPIRAM);
        if (pf.data) {
            memcpy(pf.data, frame->data, frame->data_bytes);
            pf.len = frame->data_bytes;
            pf.width = frame->width;
            pf.height = frame->height;
            if (xQueueSend(proc_queue, &pf, 0) != pdPASS) {
                free(pf.data);
            }
        }
    }
}

// ================= 核心图像处理 =================
static void process_jpeg_frame(uint8_t *jpeg_data, size_t jpeg_len, uint16_t img_w, uint16_t img_h) {
    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
    config.output_type = JPEG_PIXEL_FORMAT_CbYCrY;
    config.clipper.width = img_w;
    config.clipper.height = current_crop_rows; // 🌟 动态裁剪高度
    /* esp_new_jpeg 的顺序是 解码 → 缩放 → 裁剪 → 旋转，裁剪锚在左上角。
     * 所以上面这 24 行取的是传感器原始画面【最上面】24 行，之后才 180° 旋转。
     * 地面要落在那 24 行里，摄像头必须倒装 —— 这句 rotate 就是倒装的补偿。
     *
     * 别因为「画面看着颠倒」就删掉它：删了不会把裁剪窗口挪到画面下半部分，
     * 裁的还是最上面 24 行，变的只是那 24 行里哪几行权重高。本车摄像头如果是
     * 正装的，最上面 24 行拍到的是地平线，调俯仰角救不回来，得物理倒装。 */
    config.rotate = JPEG_ROTATE_180D;

    jpeg_dec_handle_t jpeg_dec = NULL;
    jpeg_error_t res = jpeg_dec_open(&config, &jpeg_dec);

    if (res != JPEG_ERR_OK || jpeg_dec == NULL) {
        if(current_vision_mode == VISION_MODE_LINE) update_line_lost_state(0);
        return;
    }

    jpeg_dec_io_t jpeg_io = { .inbuf = jpeg_data, .inbuf_len = jpeg_len };
    jpeg_dec_header_info_t out_info;
    res = jpeg_dec_parse_header(jpeg_dec, &jpeg_io, &out_info);

    if (res != JPEG_ERR_OK) {
        jpeg_dec_close(jpeg_dec);
        if(current_vision_mode == VISION_MODE_LINE) update_line_lost_state(0);
        return;
    }

    int outbuf_len = 0;
    jpeg_dec_get_outbuf_len(jpeg_dec, &outbuf_len);
    if (outbuf_len <= 0) {
        jpeg_dec_close(jpeg_dec);
        if(current_vision_mode == VISION_MODE_LINE) update_line_lost_state(0);
        return;
    }

    uint8_t *yuv_buf = (uint8_t *)jpeg_calloc_align(outbuf_len, 16);
    if (yuv_buf == NULL) {
        jpeg_dec_close(jpeg_dec);
        if(current_vision_mode == VISION_MODE_LINE) update_line_lost_state(0);
        return;
    }

    jpeg_io.outbuf = yuv_buf;
    res = jpeg_dec_process(jpeg_dec, &jpeg_io);
    jpeg_dec_close(jpeg_dec);

    if (res != JPEG_ERR_OK) {
        jpeg_free_align(yuv_buf);
        if(current_vision_mode == VISION_MODE_LINE) update_line_lost_state(0);
        return;
    }

    int crop_w = out_info.width;
    int bytes_per_row = crop_w * 2;

    // =========================================================
    // 分支 1：黑线循迹模式
    // =========================================================
    if (current_vision_mode == VISION_MODE_LINE) {
        const int scan_rows = 8;
        const int ignore_margin = crop_w * 4 / 10;
        const int center_l = crop_w * 7 / 20;
        const int center_r = crop_w * 13 / 20;
        const int center_x_pos = crop_w / 2;
        const int max_dist = center_x_pos - ignore_margin;
        const int threshold = 120;
        const int min_center_weight = 200;
        const int turn_trigger_weight = 60;

        long sum_center_x = 0;
        long center_weight = 0, left_weight = 0, right_weight = 0;
        int found_any_black = 0;

        for (int img_y = 0; img_y < scan_rows; img_y++) {
            const uint8_t *row = &yuv_buf[img_y * bytes_per_row];
            int w_y = 100 - (img_y * 80 / scan_rows);
            if (w_y < 20) w_y = 20;

            for (int img_x = ignore_margin; img_x < crop_w - ignore_margin; img_x++) {
                if (ycbyry_get_y(row, img_x) < threshold) {
                    found_any_black = 1;
                    int dist_x = abs(img_x - center_x_pos);
                    int w_x = 100 - (dist_x * 80 / max_dist);
                    if (w_x < 20) w_x = 20;
                    int weight = (w_y * w_x) / 100;

                    if (img_x >= center_l && img_x <= center_r) {
                        sum_center_x += img_x * weight;
                        center_weight += weight;
                    } else if (img_x < center_l) {
                        left_weight += weight;
                    } else {
                        right_weight += weight;
                    }
                }
            }
        }

        if (center_weight >= min_center_weight) {
            float center_x = (float)sum_center_x / center_weight;
            float offset = center_x - (crop_w / 2.0f);
            float err = offset / ((crop_w * 0.12f) / 6.0f);
            err = fmaxf(-3.0f, fminf(3.0f, err));
            camera_track_error = camera_track_error * 0.60f + err * 0.40f;
            update_line_lost_state(1);
        } else if (!found_any_black) {
            update_line_lost_state(0);
        } else if (left_weight > right_weight * 2 && left_weight > turn_trigger_weight) {
            camera_track_error = -2.5f;
            update_line_lost_state(0);
        } else if (right_weight > left_weight * 2 && right_weight > turn_trigger_weight) {
            camera_track_error = 2.5f;
            update_line_lost_state(0);
        } else {
            update_line_lost_state(1);
        }
    } 
    // =========================================================
    // 分支 2：找球模式 (绿球 / 红球)
    // =========================================================
    else {
        long sum_x = 0;
        int target_pixel_count = 0;

        // YUV422 格式内存排列: [Cb0, Y0, Cr0, Y1], [Cb1, Y2, Cr1, Y3]...
        // 隔行隔点扫描，加快处理速度
        for (int y = 0; y < current_crop_rows; y += 2) {
            const uint8_t *row = &yuv_buf[y * bytes_per_row];
            for (int x = 0; x < crop_w; x += 2) {
                int pair = x >> 1;
                uint8_t cb = row[pair * 4 + 0];     // U 通道 (蓝色差)
                uint8_t y_luma = row[pair * 4 + 1]; // Y 通道 (亮度)
                uint8_t cr = row[pair * 4 + 2];     // V 通道 (红色差)

                int is_target = 0;

                if (current_vision_mode == VISION_MODE_GREEN_BALL) {
                    // 🟢 绿球特征：Cr极低(不红)，Cb适中偏低，亮度适中
                    if (cr < 115 && cb < 135 && y_luma > 15 && y_luma < 220) {
                        is_target = 1;
                    }
                } else if (current_vision_mode == VISION_MODE_RED_BALL) {
                    // 🔴 红球特征：Cr极高(非常红)，Cb偏低，亮度适中
                    if (cr > 160 && cb < 110 && y_luma > 30 && y_luma < 220) {
                        is_target = 1;
                    }
                }

                if (is_target) {
                    sum_x += x;
                    target_pixel_count++;
                }
            }
        }

        // 判定阈值：如果检测到的像素足够多，说明看到了完整的球
        if (target_pixel_count > 5) {
            ball_detected = 1;
            float center_x = (float)sum_x / target_pixel_count;
            float offset = center_x - (crop_w / 2.0f);
            
            // 计算偏差：负数代表在画面左侧，正数代表在右侧
            ball_track_error = offset / (crop_w / 6.0f); 
            ball_track_error = fmaxf(-3.0f, fminf(3.0f, ball_track_error));
            
            // 可以在终端打印一下色块数量，方便你调光
            // ESP_LOGI(TAG, "Ball Found! pixels:%d, err:%.2f", target_pixel_count, ball_track_error);
        } else {
            ball_detected = 0;
            ball_track_error = 0.0f;
        }
    }

    jpeg_free_align(yuv_buf);
}

// ================= 后台任务与启动 =================
static void camera_process_task(void *arg) {
    ESP_LOGI(TAG, "正在初始化 USB 摄像头...");

    uint8_t *xfer_buffer_a = (uint8_t *)heap_caps_malloc(CAM_XFER_BUF_SIZE, MALLOC_CAP_INTERNAL);
    uint8_t *xfer_buffer_b = (uint8_t *)heap_caps_malloc(CAM_XFER_BUF_SIZE, MALLOC_CAP_INTERNAL);
    uint8_t *frame_buffer  = (uint8_t *)heap_caps_malloc(CAM_FRAME_BUF_SIZE, MALLOC_CAP_SPIRAM);

    if (!xfer_buffer_a || !xfer_buffer_b || !frame_buffer) {
        ESP_LOGE(TAG, "内存分配失败!");
        vTaskDelete(NULL);
    }

    uvc_config_t uvc_config = {
        .frame_width = CAM_FRAME_WIDTH,
        .frame_height = CAM_FRAME_HEIGHT,
        .frame_interval = CAM_FRAME_INTERVAL,
        .xfer_buffer_size = CAM_XFER_BUF_SIZE,
        .xfer_buffer_a = xfer_buffer_a,
        .xfer_buffer_b = xfer_buffer_b,
        .frame_buffer_size = CAM_FRAME_BUF_SIZE,
        .frame_buffer = frame_buffer,
        .frame_cb = camera_frame_cb,
        .frame_cb_arg = NULL,
    };

    esp_err_t ret = uvc_streaming_config(&uvc_config);
    if (ret != ESP_OK) ESP_LOGE(TAG, "UVC 配置失败: %s", esp_err_to_name(ret));
    
    ret = usb_streaming_start();
    if (ret != ESP_OK) ESP_LOGE(TAG, "USB 视频流启动失败: %s", esp_err_to_name(ret));
    else ESP_LOGI(TAG, "摄像头已启动，正在等待图像帧...");

    while (1) {
        proc_frame_t pf;
        if (xQueueReceive(proc_queue, &pf, pdMS_TO_TICKS(2000))) {
            process_jpeg_frame(pf.data, pf.len, pf.width, pf.height);
            free(pf.data);
        }
    }
}

void camera_track_init(void) {
    servos_init();
    camera_set_pan_tilt(SERVO_INIT_YAW_DEG, SERVO_INIT_PITCH_DEG); // 初始舵机角度
    camera_track_error = 0.0f;
    camera_line_lost = 0;
    camera_line_lost_count = 0;
    
    current_vision_mode = VISION_MODE_LINE; // 默认循迹模式
    ball_detected = 0;

    frame_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(frame_item_t));
    proc_queue = xQueueCreate(PROC_QUEUE_LENGTH, sizeof(proc_frame_t));
}

void camera_track_start(void) {
    xTaskCreatePinnedToCore(camera_process_task, "cam_track_task", 8192, NULL, 5, NULL, 1);
}