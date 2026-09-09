#pragma once
#include "esp_err.h"
#include "media_protocol.h"
esp_err_t music_player_start(void);
bool music_player_submit(const media_command_t *command);
void music_player_status(uint8_t out[MEDIA_STATUS_BYTES]);
esp_err_t music_gatt_register(void);
void music_gatt_tick(void);
