idf_component_register(
    SRCS "main.c" "ultrasonic.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_ledc esp_timer esp_rom
)
