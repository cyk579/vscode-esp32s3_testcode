#pragma once
#include <stddef.h>
#include "esp_err.h"
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_INTERNAL 2
#define MALLOC_CAP_8BIT 4
void *heap_caps_malloc(size_t size, unsigned caps);
void heap_caps_free(void *ptr);
