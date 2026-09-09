idf_component_register(SRCS "main.c" "endpoint_ball.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_spi esp_driver_ledc esp_psram esp_timer hal soc usb_stream esp_jpeg)
