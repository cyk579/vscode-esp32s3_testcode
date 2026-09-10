# Install script for directory: C:/Espressif/frameworks/idf-extract/esp-idf-5.5.4

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/subject3-ASR")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/Espressif/tools/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin/xtensa-esp32s3-elf-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/xtensa/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_timer/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_pm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/bootloader/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esptool_py/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/partition_table/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_app_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_bootloader_format/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/app_update/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_partition/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/efuse/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/bootloader_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_mm/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/spi_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_system/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_common/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_rom/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/hal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/log/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/heap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_security/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_hw_support/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/freertos/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/newlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/pthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/cxx/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_driver_gpio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_ringbuf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_driver_i2c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_event/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/nvs_flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_phy/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_psram/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_driver_uart/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_driver_usb_serial_jtag/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_vfs_console/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/vfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/lwip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_netif_stack/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_netif/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/wpa_supplicant/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_coex/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_driver_spi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_gdbstub/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/bt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/gesture-common/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/esp_driver_ledc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/usb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/espressif__cmake_utilities/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/espressif__usb_stream/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/spiffs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance-resume/esp-idf/main/cmake_install.cmake")
endif()

