# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Espressif/frameworks/idf-extract/esp-idf-5.5.4/components/bootloader/subproject")
  file(MAKE_DIRECTORY "C:/Espressif/frameworks/idf-extract/esp-idf-5.5.4/components/bootloader/subproject")
endif()
file(MAKE_DIRECTORY
  "C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance/bootloader"
  "C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance/bootloader-prefix"
  "C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance/bootloader-prefix/tmp"
  "C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance/bootloader-prefix/src/bootloader-stamp"
  "C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance/bootloader-prefix/src"
  "C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Electronic_Design/vscode-esp32s3_testcode/esp-projects/subject3-ASR/build-dance/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
