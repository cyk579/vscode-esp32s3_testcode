idf_component_register(SRCS "gesture_protocol.c" "gesture_control.c" "gesture_imu.c" "gesture_board.c" "gesture_ble.c"
    INCLUDE_DIRS "include" "third_party/Fusion"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_timer bt nvs_flash)
target_sources(${COMPONENT_LIB} PRIVATE "third_party/Fusion/FusionAhrs.c")
