#include "wifi_mjpeg.h"
#include "camera_track.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <sys/param.h>

static const char *TAG = "WIFI_MJPEG";

#define SSID     "ESP32_CAM"
#define PASSWORD "12345678"
#define PORT     8080

static QueueHandle_t s_frame_queue = NULL;

static void wifi_init_ap(void) {
    // ---------- 1. 初始化 NVS ----------
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ---------- 2. 初始化网络 ----------
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    // ---------- 3. 初始化 Wi-Fi ----------
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 设置协议兼容性
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

    // ---------- 4. 配置 AP ----------
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SSID,
            .password = PASSWORD,
            .ssid_len = strlen(SSID),
            .max_connection = 3,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = 6,                 // 避开常用信道
            .beacon_interval = 100,
        },
    };
    if (strlen(PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

    // ---------- 5. 启动 Wi-Fi ----------
    ESP_ERROR_CHECK(esp_wifi_start());

    // ---------- 6. 设置发射功率（必须在 esp_wifi_start 之后） ----------
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));  // 78 对应 20dBm

    ESP_LOGI(TAG, "Wi-Fi AP started, SSID: %s, Channel: %d, IP: 192.168.4.1", SSID, wifi_config.ap.channel);
}

// ---------- MJPEG 服务器任务 ----------
static void mjpeg_server_task(void *arg) {
    int listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket create failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port = htons(PORT)
    };
    if (bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    listen(listen_sock, 1);
    ESP_LOGI(TAG, "MJPEG server started on port %d", PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        if (client_sock < 0) {
            ESP_LOGE(TAG, "accept failed");
            continue;
        }
        ESP_LOGI(TAG, "Client connected");

        const char *header =
            "HTTP/1.0 200 OK\r\n"
            "Server: ESP32-CAM\r\n"
            "Cache-Control: no-store, no-cache, must-revalidate\r\n"
            "Pragma: no-cache\r\n"
            "Connection: close\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=--frame\r\n\r\n";
        send(client_sock, header, strlen(header), 0);

        while (1) {
            frame_item_t item;
            if (xQueueReceive(s_frame_queue, &item, portMAX_DELAY) == pdPASS) {
                char boundary[64];
                snprintf(boundary, sizeof(boundary),
                         "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
                         item.len);
                if (send(client_sock, boundary, strlen(boundary), 0) < 0) break;
                if (send(client_sock, item.data, item.len, 0) < 0) {
                    free(item.data);
                    break;
                }
                send(client_sock, "\r\n", 2, 0);
                free(item.data);
            } else {
                break;
            }
        }
        close(client_sock);
        ESP_LOGI(TAG, "Client disconnected");
    }
}

void wifi_mjpeg_start(QueueHandle_t frame_queue) {
    s_frame_queue = frame_queue;
    wifi_init_ap();
    xTaskCreate(mjpeg_server_task, "mjpeg_server", 8192, NULL, 6, NULL);
}