idf_component_register(SRCS "main.c" "motor.c" "tft_st7735.c"
    INCLUDE_DIRS "." REQUIRES gesture-common esp_driver_gpio esp_driver_ledc esp_driver_spi esp_timer)
