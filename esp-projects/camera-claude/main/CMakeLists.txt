idf_component_register(SRCS "tft.c" "main.c" "motor.c" "pid.c" "track.c" "ultrasonic.c" "encoder.c" "camera_track.c" "wifi_mjpeg.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer driver usb usb_stream mbedtls esp_new_jpeg esp_wifi esp_netif lwip nvs_flash)
