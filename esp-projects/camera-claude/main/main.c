#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"          // <-- 新增
#include <math.h>             // 🌟 新增：引入数学库以支持 fabsf()

#include "motor.h"
#include "track.h"
#include "pid.h"
#include "camera_track.h"
#include "track_config.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "wifi_mjpeg.h"
#include "tft.h"              // 🌟 新增：引入屏幕驱动头文件

#define TEST_MODE 6
#define PID_CONTROL_PERIOD_MS 10
#define PID_PRINT_PERIOD_MS 100

static volatile float g_obstacle_distance = 100.0f;

typedef enum {
    STATE_FOLLOWING = 0,
    STATE_AVOID_LEFT,
    STATE_AVOID_FORWARD,
    STATE_AVOID_RIGHT,
    STATE_STOPPED
} avoid_state_t;

static avoid_state_t g_avoid_state = STATE_FOLLOWING;
static uint32_t g_state_timer = 0;
static int g_avoidance_completed = 0;

#define AVOID_POWER 1600  //前进
#define STRAFE_POWER 2200  //横移左右轮
#define STRAFE_CORRECT 1200   //横移后轮
#define LEFT_MOVE_MS 900    //左轮移动时间
#define RIGHT_MOVE_MS (LEFT_MOVE_MS / 3)
#define FORWARD_MOVE_MS 700    //前进移动时间

static inline int line_reacquired_after_avoid(void) {
#ifdef USE_CAMERA_TRACK
    return !camera_track_is_line_lost();
#else
    return track_is_centered();
#endif
}

static inline int is_line_currently_lost(void) {
#ifdef USE_CAMERA_TRACK
    return camera_track_is_line_lost();
#else
    return track_is_all_zero();
#endif
}

void ultrasonic_task(void *pvParameters) {
    ultrasonic_init();
    while (1) {
        float dist = ultrasonic_get_distance_cm();
        g_obstacle_distance = (dist >= 0) ? dist : 999.0f;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
// ===================== 显示屏刷新任务 =====================
// ===================== 显示屏刷新任务 =====================
void display_task(void *pvParameters) {
    char buf[32];
    float sa, sb, sd;
    
    while (1) {
        // 1. 获取编码器速度
        encoder_get_speeds(&sa, &sb, &sd);
        
        // 2. 显示超声波距离
        sprintf(buf, "Dist: %.1f cm  ", g_obstacle_distance);
        tft_draw_string(0, 30, buf, TFT_WHITE, TFT_BLACK);
        
        // 3. 显示三个轮子速度 (这里换回你之前红外代码喜欢的格式)
        sprintf(buf, "SpdA: %.0f   ", sa * 100);
        tft_draw_string(0, 60, buf, TFT_GREEN, TFT_BLACK);
        
        sprintf(buf, "SpdB: %.0f   ", sb * 100);
        tft_draw_string(0, 80, buf, TFT_GREEN, TFT_BLACK);
        
        sprintf(buf, "SpdD: %.0f   ", sd * 100);
        tft_draw_string(0, 100, buf, TFT_GREEN, TFT_BLACK);
        
        // 每 100ms 刷新一次屏幕
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(500));

#if TEST_MODE == 1
    // ... 电机测试（略） ...
#elif TEST_MODE == 2
    // ... 红外测试（略） ...
#elif TEST_MODE == 3
// 🌟 1. 优先级最高：在一切开始前先初始化屏幕！抢占 DMA 内存
    tft_init();
    tft_clear(TFT_BLACK);
    tft_draw_string(0, 0, "SYSTEM STARTING...", TFT_YELLOW, TFT_BLACK);
    motor_init();
    pid_init();
    encoder_init();  // 🌟 新增：必须在循环前初始化编码器，为闭环和屏幕提供数据

    // 🌟 新增：创建屏幕显示任务 (分配 4096 字节栈空间，优先级设为 4)
    xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);

#ifdef USE_CAMERA_TRACK
    printf("\n********************************************\n");
    printf("*          摄像头循迹模式启动              *\n");
    printf("*  摄像头: JQ-CAM12-720P-V1                *\n");
    printf("*  舵机:   MG90S x2 (PITCH/YAW)           *\n");
    printf("*  Wi-Fi:  ESP32_CAM / 12345678           *\n");
    printf("*  浏览器访问: http://192.168.4.1:8080   *\n");
    printf("********************************************\n\n");

    camera_track_init();
    camera_track_start();

    QueueHandle_t frame_q = camera_track_get_frame_queue();
    if (frame_q) {
        wifi_mjpeg_start(frame_q);
    } else {
        ESP_LOGE("MAIN", "Failed to get frame queue, Wi-Fi stream not started");
    }
#else
    printf("\n[系统] 使用红外传感器循迹模式\n\n");
    track_init();
#endif

    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 5, NULL);

    int print_counter = 0;
    pid_motor_state_t motor_A, motor_B, motor_D;

    while (1) {
        int out1, out2, out3, out4;
#ifdef USE_CAMERA_TRACK
        float error = get_camera_track_error();
#else
        float error = get_track_error();
#endif

        float current_dist = g_obstacle_distance;
        switch (g_avoid_state) {
    case STATE_FOLLOWING:
    if (current_dist > 0 && current_dist < 5.0f && g_avoidance_completed == 0) {
        // 只在未避障时触发
        g_avoid_state = STATE_AVOID_LEFT;
        g_state_timer = 0;
    } else if (g_avoidance_completed && is_line_currently_lost()) {
        pid_stop();
        g_avoid_state = STATE_STOPPED;
    } else {
        pid_update(error);
    }
    break;
            case STATE_AVOID_LEFT:
                pid_manual_control(STRAFE_CORRECT, STRAFE_POWER, -STRAFE_CORRECT);
                if (++g_state_timer >= LEFT_MOVE_MS / PID_CONTROL_PERIOD_MS) {
                    pid_stop();
                    g_avoid_state = STATE_AVOID_FORWARD;
                    g_state_timer = 0;
                }
                break;
            case STATE_AVOID_FORWARD:
                pid_manual_control(AVOID_POWER, 0, AVOID_POWER);
                if (++g_state_timer >= FORWARD_MOVE_MS / PID_CONTROL_PERIOD_MS) {
                    pid_stop();
                    g_avoid_state = STATE_AVOID_RIGHT;
                    g_state_timer = 0;
                }
                break;
            case STATE_AVOID_RIGHT:
                pid_manual_control(-STRAFE_CORRECT, -STRAFE_POWER, STRAFE_CORRECT);
                g_state_timer++;
                if (g_state_timer >= RIGHT_MOVE_MS / PID_CONTROL_PERIOD_MS) {
                    if (line_reacquired_after_avoid()) {
                        pid_stop();
                        g_avoidance_completed = 1;
                        g_avoid_state = STATE_FOLLOWING;
                        g_state_timer = 0;
                    }
                }
                break;
            case STATE_STOPPED:
                pid_stop();
                break;
        }

        if (++print_counter >= PID_PRINT_PERIOD_MS / PID_CONTROL_PERIOD_MS) {
#ifndef USE_CAMERA_TRACK
            track_get_sensor_states(&out1, &out2, &out3, &out4);
#else
            out1 = out2 = out3 = out4 = 0;
#endif
            pid_get_motor_states(&motor_A, &motor_B, &motor_D);
            printf("态:%d 距:%.1fcm | 传:%d %d %d %d | 误:%.2f | A:%lu B:%lu D:%lu\n",
                   g_avoid_state, current_dist, out1, out2, out3, out4, error,
                   (unsigned long)motor_A.speed, (unsigned long)motor_B.speed, (unsigned long)motor_D.speed);
            print_counter = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(PID_CONTROL_PERIOD_MS));
    }
#elif TEST_MODE == 4
    // ... 超声波测试（略） ...
// ==================== 模式 5：识别并锁定绿球测试 ====================
// ==================== 模式 5：识别并锁定绿球测试 ====================
// ==================== 模式 5：识别并锁定绿球测试 ====================
// ==================== 模式 5：识别、瞄准并推球测试 ====================
#elif TEST_MODE == 5
    // 1. 初始化基础硬件
    tft_init();
    tft_clear(TFT_BLACK);
    tft_draw_string(0, 0, "PUSH BALL MODE", TFT_YELLOW, TFT_BLACK);
    motor_init();
    pid_init();
    encoder_init(); 
    
    // 2. 启动摄像头与 Wi-Fi
    camera_track_init();
    camera_track_start();
    QueueHandle_t frame_q = camera_track_get_frame_queue();
    if (frame_q) wifi_mjpeg_start(frame_q);
    camera_track_set_mode(VISION_MODE_GREEN_BALL);

    int print_counter = 0;
    int task_phase = 0;    // 0=绿球阶段, 1=红球阶段, 2=全部完成
    int task_step = 0;     // 每个phase内: 0=找球居中, 1=停稳, 2=推球+后退
    int action_timer = 0;
    int ball_just_seen = 0;

    while (1) {
        const char *ball_name = (task_phase == 0) ? "绿球" : "红球";

        if (task_phase == 2) {
            pid_stop();
            continue;
        }

        if (task_step == 0) {
            if (is_ball_detected()) {
                if (ball_just_seen == 0) {
                    pid_stop();
                    printf("🛑 视野中刚出现%s！先急刹车停稳...\n", ball_name);
                    ball_just_seen = 1;
                    vTaskDelay(pdMS_TO_TICKS(300));
                    continue;
                }

                float err = get_ball_track_error();
                if (fabsf(err) < 0.6f) {
                    pid_stop();
                    printf("🎯 %s已居中！准备进入下一步... | 误差: %.2f\n", ball_name, err);
                    action_timer = 0;
                    task_step = 1;
                } else {
                    int spin_speed = (int)(err * 150);
                    if (spin_speed > 0 && spin_speed < 800) spin_speed = 800;
                    if (spin_speed < 0 && spin_speed > -800) spin_speed = -800;
                    pid_manual_control(-spin_speed, 0, spin_speed);
                    if (++print_counter >= 10) {
                        printf("👀 正在微调对准%s... | 误差: %.2f | 转向速度: %d\n", ball_name, err, spin_speed);
                        print_counter = 0;
                    }
                }
            } else {
                ball_just_seen = 0;
                pid_manual_control(-1000, 0, 1000);
                if (++print_counter >= 10) {
                    printf("🔄 正在转圈搜索%s...\n", ball_name);
                    print_counter = 0;
                }
            }
        }
        else if (task_step == 1) {
            pid_stop();
            action_timer++;
            if (action_timer > 100) {
                printf("🚀 停稳完毕！开始向前冲刺推%s！\n", ball_name);
                action_timer = 0;
                task_step = 2;
            }
        }
        else if (task_step == 2) {
            if (action_timer < 120) {
                pid_manual_control(1800, 0, 1800);
            } else {
                pid_manual_control(-1800, 0, -1800);
            }
            action_timer++;
            if (action_timer >= 260) {
                pid_stop();
                printf("✅ %s已推入目标区域，小车已后退复位！\n", ball_name);
                if (task_phase == 0) {
                    task_phase = 1;
                    task_step = 0;
                    action_timer = 0;
                    ball_just_seen = 0;
                    camera_track_set_mode(VISION_MODE_RED_BALL);
                    printf("\n========== 切换到红球识别模式 ==========\n");
                    vTaskDelay(pdMS_TO_TICKS(500));
                } else {
                    task_phase = 2;
                    printf("\n🎉🎉🎉 全部任务完成！绿球+红球均已推入！\n");
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
// ==================== 模式 6：循迹避障 → 找球推球（完整流程） ====================
#elif TEST_MODE == 6
    tft_init();
    tft_clear(TFT_BLACK);
    tft_draw_string(0, 0, "FULL PIPELINE MODE", TFT_YELLOW, TFT_BLACK);
    motor_init();
    pid_init();
    encoder_init();
    xTaskCreate(display_task, "display_task", 4096, NULL, 4, NULL);

    camera_track_init();
    camera_track_start();
    QueueHandle_t frame_q = camera_track_get_frame_queue();
    if (frame_q) wifi_mjpeg_start(frame_q);

    xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, 5, NULL);

    printf("\n************************************************\n");
    printf("*     完整流水线模式启动！                       *\n");
    printf("*  阶段1: 摄像头循迹 + 避障                     *\n");
    printf("*  阶段2: 找绿球 → 推球 → 后退                   *\n");
    printf("*  阶段3: 找红球 → 推球 → 后退                   *\n");
    printf("************************************************\n\n");

    camera_track_set_mode(VISION_MODE_LINE);

    int pipeline_stage = 0;    // 0=循迹避障, 1=绿球推球, 2=红球推球, 3=全部完成
    int ball_task_phase = 0;   // 推球阶段内部: 0=当前球, 1=下一个球
    int ball_task_step = 0;    // 0=找球居中, 1=停稳, 2=推球+后退
    int ball_action_timer = 0;
    int ball_just_seen = 0;

    g_avoid_state = STATE_FOLLOWING;
    g_state_timer = 0;
    g_avoidance_completed = 0;

    while (1) {
        if (pipeline_stage == 0) {
            // ========== 阶段 1：循迹避障 ==========
            float error = get_camera_track_error();
            float current_dist = g_obstacle_distance;

            switch (g_avoid_state) {
                case STATE_FOLLOWING:
                    if (current_dist > 0 && current_dist < 8.0f && g_avoidance_completed == 0) {
                        g_avoid_state = STATE_AVOID_LEFT;
                        g_state_timer = 0;
                        printf("⚠️ 检测到障碍物，开始避障！\n");
                    } else if (g_avoidance_completed && is_line_currently_lost()) {
                        pid_stop();
                        g_avoid_state = STATE_STOPPED;
                        printf("🏁 循迹避障完成！切换到找球推球阶段...\n\n");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        camera_track_set_mode(VISION_MODE_GREEN_BALL);
                        ball_task_phase = 0;
                        ball_task_step = 0;
                        ball_action_timer = 0;
                        ball_just_seen = 0;
                        pipeline_stage = 1;
                    } else {
                        pid_update(error);
                    }
                    break;
                case STATE_AVOID_LEFT:
                    pid_manual_control(STRAFE_CORRECT, STRAFE_POWER, -STRAFE_CORRECT);
                    if (++g_state_timer >= LEFT_MOVE_MS / PID_CONTROL_PERIOD_MS) {
                        pid_stop();
                        g_avoid_state = STATE_AVOID_FORWARD;
                        g_state_timer = 0;
                    }
                    break;
                case STATE_AVOID_FORWARD:
                    pid_manual_control(AVOID_POWER, 0, AVOID_POWER);
                    if (++g_state_timer >= FORWARD_MOVE_MS / PID_CONTROL_PERIOD_MS) {
                        pid_stop();
                        g_avoid_state = STATE_AVOID_RIGHT;
                        g_state_timer = 0;
                    }
                    break;
                case STATE_AVOID_RIGHT:
                    pid_manual_control(-STRAFE_CORRECT, -STRAFE_POWER, STRAFE_CORRECT);
                    g_state_timer++;
                    if (g_state_timer >= RIGHT_MOVE_MS / PID_CONTROL_PERIOD_MS) {
                        if (line_reacquired_after_avoid()) {
                            pid_stop();
                            g_avoidance_completed = 1;
                            g_avoid_state = STATE_FOLLOWING;
                            g_state_timer = 0;
                            printf("✅ 避障完成，重新找到黑线，继续循迹！\n");
                        }
                    }
                    break;
                case STATE_STOPPED:
                    pid_stop();
                    break;
            }
        }
        else if (pipeline_stage == 1 || pipeline_stage == 2) {
            // ========== 阶段 2/3：推球（绿球 / 红球） ==========
            const char *ball_name = (ball_task_phase == 0) ? "绿球" : "红球";

            if (ball_task_step == 0) {
                if (is_ball_detected()) {
                    if (ball_just_seen == 0) {
                        pid_stop();
                        printf("🛑 视野中刚出现%s！先急刹车停稳...\n", ball_name);
                        ball_just_seen = 1;
                        vTaskDelay(pdMS_TO_TICKS(300));
                        continue;
                    }
                    float err = get_ball_track_error();
                    if (fabsf(err) < 0.6f) {
                        pid_stop();
                        printf("🎯 %s已居中！准备进入下一步... | 误差: %.2f\n", ball_name, err);
                        ball_action_timer = 0;
                        ball_task_step = 1;
                    } else {
                        int spin_speed = (int)(err * 150);
                        if (spin_speed > 0 && spin_speed < 850) spin_speed = 850;
                        if (spin_speed < 0 && spin_speed > -850) spin_speed = -850;
                        pid_manual_control(-spin_speed, 0, spin_speed);
                    }
                } else {
                    ball_just_seen = 0;
                    pid_manual_control(-1000, 0, 1000);
                }
            }
            else if (ball_task_step == 1) {
                pid_stop();
                ball_action_timer++;
                if (ball_action_timer > 100) {
                    printf("🚀 停稳完毕！开始向前冲刺推%s！\n", ball_name);
                    ball_action_timer = 0;
                    ball_task_step = 2;
                }
            }
            else if (ball_task_step == 2) {
                if (ball_action_timer < 105) {
                    pid_manual_control(1800, 0, 1800);
                } else {
                    pid_manual_control(-1800, 0, -1800);
                }
                ball_action_timer++;
                if (ball_action_timer >= 220) {
                    pid_stop();
                    printf("✅ %s已推入目标区域，小车已后退复位！\n", ball_name);
                    if (ball_task_phase == 0) {
                        ball_task_phase = 1;
                        ball_task_step = 0;
                        ball_action_timer = 0;
                        ball_just_seen = 0;
                        camera_track_set_mode(VISION_MODE_RED_BALL);
                        printf("\n========== 切换到红球识别模式 ==========\n");
                        vTaskDelay(pdMS_TO_TICKS(500));
                    } else {
                        pipeline_stage = 3;
                        printf("\n🎉🎉🎉 全部任务完成！绿球+红球均已推入！\n");
                    }
                }
            }
        }
        else {
            // ========== 阶段 4：全部完成，永久停车 ==========
            pid_stop();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
#endif
}