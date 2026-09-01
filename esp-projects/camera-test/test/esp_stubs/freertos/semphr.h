#pragma once
#include "FreeRTOS.h"
typedef void *SemaphoreHandle_t;
SemaphoreHandle_t xSemaphoreCreateMutex(void);
int xSemaphoreTake(SemaphoreHandle_t s, TickType_t t);
int xSemaphoreGive(SemaphoreHandle_t s);
