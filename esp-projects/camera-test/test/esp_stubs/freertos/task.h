#pragma once
#include "FreeRTOS.h"
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
int xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack,
                void *arg, uint32_t prio, TaskHandle_t *handle);
int xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack,
                            void *arg, uint32_t prio, TaskHandle_t *handle,
                            int core_id);
void vTaskDelay(TickType_t ticks);
void vTaskDelete(TaskHandle_t task);
void vTaskPrioritySet(TaskHandle_t task, uint32_t prio);
