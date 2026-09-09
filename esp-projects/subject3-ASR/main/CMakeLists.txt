idf_component_register(
    SRCS "main.c" "ble_car.c" "motor.c" "tft_st7735.c"
         "usb_mic_probe.c" "pcm_level.c" "media_protocol.c" "wav_reader.c"
         "music_player.c" "music_gatt.c"
    INCLUDE_DIRS "."
    REQUIRES gesture-common esp_driver_gpio esp_driver_ledc esp_driver_spi
             esp_timer usb_stream spiffs bt)

if(CONFIG_SUBJECT3_MUSIC)
    if(NOT CONFIG_SPIFFS_PAGE_SIZE EQUAL 1024)
        message(FATAL_ERROR "subject3 music requires CONFIG_SPIFFS_PAGE_SIZE=1024. Set Component config > SPIFFS Configuration > SPIFFS logical page size, then rebuild the music image.")
    endif()
    spiffs_create_partition_image(music "${CMAKE_CURRENT_LIST_DIR}/../musicfs" FLASH_IN_PROJECT)
endif()
