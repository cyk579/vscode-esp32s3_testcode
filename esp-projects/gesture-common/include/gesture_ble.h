#pragma once
#include "gesture_protocol.h"
#include "esp_err.h"
typedef void (*gesture_link_cb)(bool connected);
typedef void (*gesture_receive_cb)(const gesture_frame_t *frame);
typedef void (*gesture_build_cb)(gesture_frame_t *frame);
typedef void (*gesture_status_cb)(uint8_t out[12]);
/* Callbacks run in NimBLE host context: bounded work only, never motor/I2C operations. */
esp_err_t gesture_ble_server_start(gesture_link_cb link, gesture_receive_cb receive, gesture_status_cb status);
esp_err_t gesture_ble_client_start(gesture_link_cb link, gesture_build_cb build);
