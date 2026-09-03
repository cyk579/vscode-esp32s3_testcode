idf_component_register(SRCS "main.c" "tcp_server.c" "camera_display.c" "camera_line_follow.c" "line_geometry.c" "line_mixer.c" "line_control.c" "tft_st7735.c" "ultrasonic.c"
                    INCLUDE_DIRS ""
                    PRIV_REQUIRES
                        nvs_flash
                        usb
                        esp_ringbuf
                        esp_psram    # Required for CONFIG_SPIRAM
                        esp_timer
                        esp_wifi
                        esp_driver_gpio
                        esp_driver_ledc
                        esp_driver_spi
                        esp_jpeg
                        heap
                    )
