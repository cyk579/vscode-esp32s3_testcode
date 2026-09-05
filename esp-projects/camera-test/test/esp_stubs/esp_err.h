/* 最小 ESP-IDF 桩头文件：只为了让 host 编译器能对 camera_line_follow.c
 * 做语法/类型检查（gcc -fsyntax-only）。这里不实现任何行为。 */
#pragma once
#include <stdint.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_STATE 0x103
static inline const char *esp_err_to_name(esp_err_t e) { (void)e; return "stub"; }
#define ESP_ERROR_CHECK(x) do { (void)(x); } while (0)
