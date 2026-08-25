/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "tcp_server.h"

#define TAG "camera_stream"
#define PORT 2222
#define CAMERA_AP_SSID "ESP32-CAMERA-TEST"
#define CAMERA_AP_PASSWORD "camera123"

typedef struct {
    int sock;
    int listen_sock;
    RingbufHandle_t buffer;
    volatile bool close_request;
    bool is_active;
} tcp_server_t;

#ifdef CONFIG_EXAMPLE_ENABLE_STREAMING

static tcp_server_t *s_server;

static void socket_close(tcp_server_t *server)
{
    ESP_LOGI(TAG, "Closing socket");
    if (server->sock >= 0) {
        shutdown(server->sock, 0);
        close(server->sock);
        server->sock = -1;
    }
    if (server->listen_sock >= 0) {
        close(server->listen_sock);
        server->listen_sock = -1;
    }
}

static bool accept_client(tcp_server_t *server)
{
    ESP_LOGI(TAG, "Waiting for player.py at 192.168.4.1:%d", PORT);
    struct sockaddr_storage source_addr;
    socklen_t address_length = sizeof(source_addr);
    server->sock = accept(server->listen_sock,
                          (struct sockaddr *)&source_addr,
                          &address_length);
    if (server->sock < 0) {
        ESP_LOGE(TAG, "Accept failed: errno %d", errno);
        return false;
    }
    server->is_active = true;
    ESP_LOGI(TAG, "Image viewer connected");
    return true;
}

static void sender_task(void *arg)
{
    tcp_server_t *server = (tcp_server_t *)arg;

    while (true) {
        if (!server->is_active) {
            if (!accept_client(server)) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
        }

        size_t bytes_received = 0;
        char *payload = (char *)xRingbufferReceiveUpTo(
            server->buffer, &bytes_received, pdMS_TO_TICKS(2500), 20000);

        if (payload != NULL) {
            if (server->is_active) {
                const int sent = send(server->sock, payload, bytes_received, 0);
                if (sent < 0) {
                    ESP_LOGE(TAG, "Send failed: errno %d", errno);
                    server->is_active = false;
                    shutdown(server->sock, 0);
                    close(server->sock);
                    server->sock = -1;
                    ESP_LOGI(TAG, "Viewer disconnected; accepting a new connection");
                }
            }
            vRingbufferReturnItem(server->buffer, payload);
        }

        if (server->close_request) {
            socket_close(server);
            vRingbufferDelete(server->buffer);
            s_server = NULL;
            free(server);
            vTaskDelete(NULL);
        }
    }
}

esp_err_t tcp_server_send(uint8_t *payload, size_t size)
{
    if (s_server == NULL || !s_server->is_active) {
        return ESP_OK;
    }

    if (xRingbufferSend(s_server->buffer, payload, size, pdMS_TO_TICKS(1)) != pdTRUE) {
        ESP_LOGW(TAG, "Frame dropped: TCP ring buffer is full");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void wifi_softap_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    const wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CAMERA_AP_SSID,
            .ssid_len = sizeof(CAMERA_AP_SSID) - 1,
            .password = CAMERA_AP_PASSWORD,
            .channel = 6,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP ready: SSID=%s password=%s", CAMERA_AP_SSID,
             CAMERA_AP_PASSWORD);
    ESP_LOGI(TAG, "Connect the PC to this AP, then run player.py (192.168.4.1:%d)", PORT);
}

static esp_err_t create_server(tcp_server_t *server)
{
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    server->listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server->listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    const int reuse = 1;
    setsockopt(server->listen_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    if (bind(server->listen_sock, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(server->listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Unable to bind/listen: errno %d", errno);
        close(server->listen_sock);
        return ESP_FAIL;
    }

    server->sock = -1;
    ESP_LOGI(TAG, "TCP image service listening at 192.168.4.1:%d", PORT);
    return ESP_OK;
}

esp_err_t tcp_server_wait_for_connection(void)
{
    tcp_server_t *server = calloc(1, sizeof(tcp_server_t));
    if (server == NULL) {
        return ESP_ERR_NO_MEM;
    }
    server->buffer = xRingbufferCreate(100000, RINGBUF_TYPE_BYTEBUF);
    if (server->buffer == NULL) {
        free(server);
        return ESP_ERR_NO_MEM;
    }
    if (create_server(server) != ESP_OK) {
        vRingbufferDelete(server->buffer);
        free(server);
        return ESP_FAIL;
    }

    TaskHandle_t task_handle = NULL;
    if (xTaskCreate(sender_task, "camera_sender", 4096, server, 10, &task_handle) != pdPASS) {
        socket_close(server);
        vRingbufferDelete(server->buffer);
        free(server);
        return ESP_ERR_NO_MEM;
    }
    s_server = server;
    return ESP_OK;
}

void tcp_server_start_ap(void)
{
    wifi_softap_init();
}

void tcp_server_close_when_done(void)
{
    if (s_server != NULL) {
        s_server->close_request = true;
    }
}

#else

esp_err_t tcp_server_wait_for_connection(void)
{
    return ESP_OK;
}

void tcp_server_start_ap(void)
{
}

esp_err_t tcp_server_send(uint8_t *payload, size_t size)
{
    return ESP_OK;
}

void tcp_server_close_when_done(void)
{
}

#endif
