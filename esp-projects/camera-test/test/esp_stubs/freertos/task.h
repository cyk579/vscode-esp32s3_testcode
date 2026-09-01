#pragma once
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
int xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack,
                void *arg, uint32_t prio, TaskHandle_t *handle);
void vTaskDelay(TickType_t ticks);
