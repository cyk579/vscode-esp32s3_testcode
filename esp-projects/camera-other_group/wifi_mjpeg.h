#ifndef _WIFI_MJPEG_H_
#define _WIFI_MJPEG_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// 初始化 Wi-Fi AP 并启动 MJPEG 流服务器
// 参数 frame_queue：由 camera_track 提供的帧队列
void wifi_mjpeg_start(QueueHandle_t frame_queue);

#endif