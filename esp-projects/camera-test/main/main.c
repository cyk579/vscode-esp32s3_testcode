/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include "camera_display.h"
#if CONFIG_EXAMPLE_ENABLE_CAMERA_LINE_FOLLOW
#include "camera_line_follow.h"
#endif
#include "esp_log.h"
#include "tcp_server.h"
#include "libuvc/libuvc.h"
#include "libuvc_helper.h"
#include "libuvc_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"

static const char *TAG = "example";

#define USB_DISCONNECT_PIN  GPIO_NUM_0
#define USB_DIAGNOSTIC_WINDOW_MS 3000U

#if CONFIG_EXAMPLE_ENABLE_SERVO_TEST
#define PAN_SERVO_PIN       GPIO_NUM_1
#define TILT_SERVO_PIN      GPIO_NUM_2
#define SERVO_TIMER         LEDC_TIMER_1
#define SERVO_RESOLUTION    LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY      ((1U << 14) - 1U)
#define SERVO_PERIOD_US     20000U

static uint32_t servo_pulse_to_duty(uint32_t pulse_us)
{
    return (SERVO_MAX_DUTY * pulse_us) / SERVO_PERIOD_US;
}

static void servo_set_pulse(ledc_channel_t channel, uint32_t pulse_us)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,
                                  channel,
                                  servo_pulse_to_duty(pulse_us)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void servo_test(void)
{
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = SERVO_TIMER,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    const ledc_channel_config_t channels[] = {
        {
            .gpio_num = PAN_SERVO_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_TIMER,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = TILT_SERVO_PIN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = SERVO_TIMER,
            .duty = 0,
            .hpoint = 0,
        },
    };

    for (size_t i = 0; i < sizeof(channels) / sizeof(channels[0]); ++i) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }

    ESP_LOGI(TAG, "Servo test: center -> small movement -> center");
    servo_set_pulse(LEDC_CHANNEL_0, 1500);
    servo_set_pulse(LEDC_CHANNEL_1, 1500);
    vTaskDelay(pdMS_TO_TICKS(700));
    servo_set_pulse(LEDC_CHANNEL_0, 1250);
    servo_set_pulse(LEDC_CHANNEL_1, 1750);
    vTaskDelay(pdMS_TO_TICKS(700));
    servo_set_pulse(LEDC_CHANNEL_0, 1750);
    servo_set_pulse(LEDC_CHANNEL_1, 1250);
    vTaskDelay(pdMS_TO_TICKS(700));
    servo_set_pulse(LEDC_CHANNEL_0, 1500);
    servo_set_pulse(LEDC_CHANNEL_1, 1500);
}
#endif

#if (CONFIG_EXAMPLE_UVC_PROTOCOL_MODE_AUTO)
#define EXAMPLE_UVC_PROTOCOL_AUTO_COUNT     5
typedef struct {
    enum uvc_frame_format format;
    int width;
    int height;
    int fps;
    const char* name;
} uvc_stream_profile_t;

/* 巡线的扫描几何里有绝对像素量（行距、最小线宽、窗口上下限），所以协商到
 * 哪个分辨率会改变行为。这台摄像头的 480x320 描述符支持 15~25 FPS；优先
 * 请求 25 FPS，失败时再回退到摄像头可接受的速率。解码仍缩放成 240x160。 */
uvc_stream_profile_t uvc_stream_profiles[EXAMPLE_UVC_PROTOCOL_AUTO_COUNT] = {
    {UVC_FRAME_FORMAT_MJPEG, 480, 320, 25, "480x320, fps 25"},
    {UVC_FRAME_FORMAT_MJPEG, 480, 320,  0, "480x320, any fps"},
    {UVC_FRAME_FORMAT_MJPEG, 320, 240, 30, "320x240, fps 30"},
    {UVC_FRAME_FORMAT_MJPEG, 640, 480, 15, "640x480, fps 15"},
    {UVC_FRAME_FORMAT_MJPEG, 1280, 720,  0, "1280x720, any fps"}
};
#endif // CONFIG_EXAMPLE_UVC_PROTOCOL_MODE_AUTO

#if (CONFIG_EXAMPLE_UVC_PROTOCOL_MODE_CUSTOM)
#define FPS                 CONFIG_EXAMPLE_FPS_PARAM
#define WIDTH               CONFIG_EXAMPLE_WIDTH_PARAM
#define HEIGHT              CONFIG_EXAMPLE_HEIGHT_PARAM
#define FORMAT              CONFIG_EXAMPLE_FORMAT_PARAM
#endif // CONFIG_EXAMPLE_UVC_PROTOCOL_MODE_CUSTOM

// Attached camera can be filtered out based on (non-zero value of) PID, VID, SERIAL_NUMBER
#define PID 0
#define VID 0
#define SERIAL_NUMBER NULL

#define UVC_CHECK(exp) do {                 \
    uvc_error_t _err_ = (exp);              \
    if(_err_ < 0) {                         \
        ESP_LOGE(TAG, "UVC error: %s",      \
                 uvc_error_string(_err_));  \
        assert(0);                          \
    }                                       \
} while(0)

static SemaphoreHandle_t ready_to_uninstall_usb;
static EventGroupHandle_t app_flags;

// Handles common USB host library events
static void usb_lib_handler_task(void *args)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        // Give ready_to_uninstall_usb semaphore to indicate that USB Host library
        // can be deinitialized, and terminate this task.
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            xSemaphoreGive(ready_to_uninstall_usb);
        }
    }

    vTaskDelete(NULL);
}

static esp_err_t initialize_usb_host_lib(void)
{
    TaskHandle_t task_handle = NULL;

    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        return err;
    }

    ready_to_uninstall_usb = xSemaphoreCreateBinary();
    if (ready_to_uninstall_usb == NULL) {
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(usb_lib_handler_task, "usb_events", 4096, NULL, 2, &task_handle) != pdPASS) {
        vSemaphoreDelete(ready_to_uninstall_usb);
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void uninitialize_usb_host_lib(void)
{
    xSemaphoreTake(ready_to_uninstall_usb, portMAX_DELAY);
    vSemaphoreDelete(ready_to_uninstall_usb);

    if (usb_host_uninstall() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall usb_host");
    }
}

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void frame_callback(uvc_frame_t *frame, void *ptr)
{
    // The decoder task copies only the newest frame outside this UVC callback,
    // so USB reception is not stalled by JPEG processing or an optional TFT.
    camera_display_submit(frame->data, frame->data_bytes);

    // Stream received frame to the PC client, if enabled.
    tcp_server_send(frame->data, frame->data_bytes);
}

void button_callback(int button, int state, void *user_ptr)
{
    printf("button %d state %d\n", button, state);
}

static void libuvc_adapter_cb(libuvc_adapter_event_t event)
{
    xEventGroupSetBits(app_flags, event);
}

static EventBits_t wait_for_event(EventBits_t event)
{
    return xEventGroupWaitBits(app_flags, event, pdTRUE, pdFALSE, portMAX_DELAY) & event;
}

static void wait_for_camera_connection(void)
{
    while ((xEventGroupWaitBits(app_flags,
                                UVC_DEVICE_CONNECTED,
                                pdTRUE,
                                pdFALSE,
                                pdMS_TO_TICKS(5000)) & UVC_DEVICE_CONNECTED) == 0) {
        ESP_LOGW(TAG, "No USB camera detected: check 5 V/GND, then power off and swap the two data wires");
        static const uint8_t status[] = "STATUS:WAITING_FOR_UVC\n";
        tcp_server_send((uint8_t *)status, sizeof(status) - 1);
    }
    static const uint8_t connected[] = "STATUS:UVC_CONNECTED\n";
    tcp_server_send((uint8_t *)connected, sizeof(connected) - 1);
}

static uvc_error_t uvc_negotiate_stream_profile(uvc_device_handle_t *devh,
                                                uvc_stream_ctrl_t *ctrl)
{
    uvc_error_t res;
#if (CONFIG_EXAMPLE_UVC_PROTOCOL_MODE_AUTO)
    for (int idx = 0; idx < EXAMPLE_UVC_PROTOCOL_AUTO_COUNT; idx++) {
        int attempt = CONFIG_EXAMPLE_NEGOTIATION_ATTEMPTS;
        do {
            /*
            The uvc_get_stream_ctrl_format_size() function will attempt to set the desired format size.
            On first attempt, some cameras would reject the format, even if they support it.
            So we ask 3x by default. The second attempt is usually successful.
            */
            ESP_LOGI(TAG, "Negotiate streaming profile %s ...", uvc_stream_profiles[idx].name);
            res = uvc_get_stream_ctrl_format_size(devh,
                                                  ctrl,
                                                  uvc_stream_profiles[idx].format,
                                                  uvc_stream_profiles[idx].width,
                                                  uvc_stream_profiles[idx].height,
                                                  uvc_stream_profiles[idx].fps);
        } while (--attempt && !(UVC_SUCCESS == res));
        if (UVC_SUCCESS == res) {
            break;
        }
    }

#elif (CONFIG_EXAMPLE_UVC_PROTOCOL_MODE_CUSTOM)
    int attempt = CONFIG_EXAMPLE_NEGOTIATION_ATTEMPTS;
    while (attempt--) {
        ESP_LOGI(TAG, "Negotiate streaming profile %dx%d, %d fps ...", WIDTH, HEIGHT, FPS);
        res = uvc_get_stream_ctrl_format_size(devh, ctrl, FORMAT, WIDTH, HEIGHT, FPS);
        if (UVC_SUCCESS == res) {
            break;
        }
        ESP_LOGE(TAG, "Negotiation failed. Try again (%d) ...", attempt);
    }
#endif // CONFIG_EXAMPLE_UVC_PROTOCOL_MODE_CUSTOM

    if (UVC_SUCCESS == res) {
        ESP_LOGI(TAG, "Negotiation complete.");
    } else {
        ESP_LOGE(TAG, "Try another UVC USB device of change negotiation parameters.");
    }

    return res;
}

void app_main(void)
{
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;

    ESP_LOGI(TAG, "Camera test starting");
    ESP_LOGI(TAG, "USB wiring under test: GPIO19=D-, GPIO20=D+");
    ESP_LOGI(TAG, "Camera must also have 5 V VBUS and common GND");

#if CONFIG_EXAMPLE_ENABLE_CAMERA_LINE_FOLLOW
    esp_err_t line_follow_err = camera_line_follow_start();
    if (line_follow_err == ESP_OK) {
        camera_display_set_frame_callback(camera_line_follow_frame_callback, NULL);
        ESP_LOGI(TAG, "Camera black-line following enabled; TB6612 standby is held low until arm");
    } else {
        ESP_LOGE(TAG, "Camera black-line following disabled: %s", esp_err_to_name(line_follow_err));
    }
#else
    ESP_LOGI(TAG, "Camera black-line following disabled in menuconfig");
#endif

    esp_err_t display_err = camera_display_start();
    if (display_err == ESP_OK) {
#if CONFIG_EXAMPLE_ENABLE_TFT_PREVIEW
        ESP_LOGI(TAG, "Camera JPEG decoder ready; TFT preview enabled (160x128 landscape)");
#else
        ESP_LOGI(TAG, "Camera JPEG decoder ready; TFT preview disabled (competition mode)");
#endif
    } else {
        ESP_LOGW(TAG, "Camera JPEG decoder unavailable: %s; PC streaming remains available",
                 esp_err_to_name(display_err));
    }

    // Start the AP when streaming is enabled; this call is a no-op otherwise.
    tcp_server_start_ap();
#if CONFIG_EXAMPLE_ENABLE_STREAMING
    ESP_LOGI(TAG, "Wi-Fi AP initialization complete");
#else
    ESP_LOGI(TAG, "Wi-Fi streaming disabled; continuing without an AP");
#endif

    // Keep USB-Serial/JTAG alive briefly before GPIO19/20 become USB Host.
    ESP_LOGI(TAG, "Diagnostic window: USB Host starts in %u seconds",
             (unsigned)(USB_DIAGNOSTIC_WINDOW_MS / 1000U));
    vTaskDelay(pdMS_TO_TICKS(USB_DIAGNOSTIC_WINDOW_MS));

#if CONFIG_EXAMPLE_ENABLE_SERVO_TEST
    servo_test();
#else
    ESP_LOGI(TAG, "Servo test disabled; continuing to USB Host");
#endif

    app_flags = xEventGroupCreate();
    assert(app_flags);

    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(USB_DISCONNECT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));

    ESP_LOGI(TAG, "Installing USB Host (GPIO19=D-, GPIO20=D+)");
    esp_err_t usb_host_err = initialize_usb_host_lib();
    if (usb_host_err != ESP_OK) {
        ESP_LOGE(TAG, "USB Host initialization failed: %s", esp_err_to_name(usb_host_err));
        ESP_LOGE(TAG, "Wi-Fi AP remains active for diagnosis");
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    libuvc_adapter_config_t config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .callback = libuvc_adapter_cb
    };

    libuvc_adapter_set_config(&config);

    ESP_LOGI(TAG, "Initializing UVC library");
    UVC_CHECK(uvc_init(&ctx, NULL));

    // Streaming takes place only when enabled in menuconfig
    ESP_ERROR_CHECK(tcp_server_wait_for_connection());

    do {

        ESP_LOGI(TAG, "Waiting for USB UVC device connection ...");
        wait_for_camera_connection();

        if (uvc_find_device(ctx, &dev, PID, VID, SERIAL_NUMBER) != UVC_SUCCESS) {
            ESP_LOGW(TAG, "UVC device not found");
            static const uint8_t status[] = "STATUS:UVC_DEVICE_NOT_FOUND\n";
            tcp_server_send((uint8_t *)status, sizeof(status) - 1);
            continue; // Continue waiting for UVC device
        }
        ESP_LOGI(TAG, "UVC device found");
        static const uint8_t found[] = "STATUS:DEVICE_FOUND\n";
        tcp_server_send((uint8_t *)found, sizeof(found) - 1);

        // UVC Device open
        UVC_CHECK(uvc_open(dev, &devh));

        // VID/PID and descriptors prove that D+/D- are wired correctly.
        libuvc_adapter_print_descriptors(devh);

        uvc_set_button_callback(devh, button_callback, NULL);

        // Print known device information
        uvc_print_diag(devh, stderr);
        // Negotiate stream profile
        if (UVC_SUCCESS == uvc_negotiate_stream_profile(devh, &ctrl)) {
            // This camera exposes a 64-byte bulk endpoint. Keep each USB
            // transfer to one payload packet; larger transfers concatenate
            // several UVC headers and corrupt the JPEG byte stream.
            ctrl.dwMaxPayloadTransferSize = 64;

            uvc_print_stream_ctrl(&ctrl, stderr);

            UVC_CHECK(uvc_start_streaming(devh, &ctrl, frame_callback, NULL, 0));
            ESP_LOGI(TAG, "Streaming...");
            static const uint8_t streaming[] = "STATUS:STREAMING\n";
            tcp_server_send((uint8_t *)streaming, sizeof(streaming) - 1);

            wait_for_event(UVC_DEVICE_DISCONNECTED);

            uvc_stop_streaming(devh);
            ESP_LOGI(TAG, "Done streaming.");
        } else {
            static const uint8_t failed[] = "STATUS:NEGOTIATION_FAILED\n";
            tcp_server_send((uint8_t *)failed, sizeof(failed) - 1);
            wait_for_event(UVC_DEVICE_DISCONNECTED);
        }
        // UVC Device close
        uvc_close(devh);
    } while (gpio_get_level(USB_DISCONNECT_PIN) != 0);

    tcp_server_close_when_done();

    uvc_exit(ctx);
    ESP_LOGI(TAG, "UVC exited");

    uninitialize_usb_host_lib();

#if CONFIG_EXAMPLE_ENABLE_CAMERA_LINE_FOLLOW
    camera_line_follow_stop();
#endif

}
