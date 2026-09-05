#include "track.h"
#include "board_pins.h"
#include "driver/gpio.h"

/* 摄像头模式下（track_config.h 定义了 USE_CAMERA_TRACK）track_init() 不会被
 * 调用，这四脚实际不配置。本车这几个脚和舵机、编码器占位脚重复，要切回红外
 * 模式前必须先确认接线，见 board_pins.h 的说明。 */
#define SENSOR_OUT1 PIN_IR_OUT1
#define SENSOR_OUT2 PIN_IR_OUT2
#define SENSOR_OUT3 PIN_IR_OUT3
#define SENSOR_OUT4 PIN_IR_OUT4

// 丢线确认次数：连续检测到全白的次数
#define LINE_LOST_CONFIRM_COUNT 3

static float last_error = 0.0f;
static int line_lost = 0;
static int line_lost_count = 0;

void track_init(void) {
    uint64_t pin_mask = (1ULL << SENSOR_OUT1) | (1ULL << SENSOR_OUT2) |
                        (1ULL << SENSOR_OUT3) | (1ULL << SENSOR_OUT4);
    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    last_error = 0.0f;
    line_lost = 0;
    line_lost_count = 0;
}

void track_get_sensor_states(int *out1, int *out2, int *out3, int *out4) {
    if (out1 != 0) *out1 = gpio_get_level(SENSOR_OUT1);
    if (out2 != 0) *out2 = gpio_get_level(SENSOR_OUT2);
    if (out3 != 0) *out3 = gpio_get_level(SENSOR_OUT3);
    if (out4 != 0) *out4 = gpio_get_level(SENSOR_OUT4);
}

int track_is_line_lost(void) {
    return line_lost;
}

int track_is_centered(void) {
    int out1, out2, out3, out4;
    track_get_sensor_states(&out1, &out2, &out3, &out4);
    // 只要中间两个传感器有一个看到黑线(0)就认为找回了线
    return (out2 == 0 || out3 == 0);
}

int track_is_all_zero(void) {
    int out1, out2, out3, out4;
    track_get_sensor_states(&out1, &out2, &out3, &out4);
    return (out1 == 0 && out2 == 0 && out3 == 0 && out4 == 0);
}

float get_track_error(void) {
    int out1, out2, out3, out4;
    const int weights[] = {3, 1, -1, -3};
    int weighted_sum = 0;
    int detected_count = 0;

    track_get_sensor_states(&out1, &out2, &out3, &out4);
    
    // 丢线确认机制：连续多次检测到全白才判定丢线
    if (out1 == 1 && out2 == 1 && out3 == 1 && out4 == 1) {
        line_lost_count++;
        if (line_lost_count >= LINE_LOST_CONFIRM_COUNT) {
            line_lost = 1;
        }
    } else {
        // 检测到非全白状态，重置计数器
        line_lost_count = 0;
        line_lost = 0;
    }

    // 新窄路线的标准居中状态：两侧白，中间两路黑。
    if (out1 == 1 && out2 == 0 && out3 == 0 && out4 == 1) {
        last_error = 0.0f;
        return last_error;
    }

    // 忽略不合理的单点跳变，保持上一次有效误差。
    if ((out1 == 0 && out2 == 0 && out3 == 1 && out4 == 0) ||
        (out1 == 0 && out2 == 1 && out3 == 0 && out4 == 0) ||
        (out1 == 1 && out2 == 1 && out3 == 0 && out4 == 1) ||
        (out1 == 1 && out2 == 0 && out3 == 1 && out4 == 1)) {
        return last_error;
    }

    const int outputs[] = {out1, out2, out3, out4};
    for (int index = 0; index < 4; index++) {
        if (outputs[index] == 0) {
            weighted_sum += weights[index];
            detected_count++;
        }
    }

    if (detected_count == 0 || detected_count == 4) {
        return last_error;
    }

    last_error = (float)weighted_sum / (float)detected_count;
    return last_error;
}