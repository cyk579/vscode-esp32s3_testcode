#include "track.h"
#include "board_pins.h"
#include "track_config.h"
#include "driver/gpio.h"

/* 本车的红外模块已经物理拆除，那四个脚现在给了别的外设：
 *   41/42 → D 轮编码器
 *   2/1   → 俯仰舵机 / 水平舵机
 * 摄像头模式下 track_init() 不会被调用，所以这四脚实际不会被配置，编译通过
 * 但代码是死的。
 *
 * 下面这句 #error 是为了拦住"注释掉 USE_CAMERA_TRACK 切回红外"这个操作 ——
 * 一旦切回去，track_init() 会把这四脚重新配成输入，D 轮编码器和两个舵机同时
 * 失效，而且现象是"车能跑但转向不对、摄像头不动"，很难往引脚上想。
 * 真要用红外循迹，先在 board_pins.h 里给它分配真正空闲的脚。 */
#ifndef USE_CAMERA_TRACK
#error "本车红外模块已拆除，PIN_IR_OUT1..4 现在是 D 轮编码器(41/42)和两个舵机(2/1)。切回红外模式前必须先在 board_pins.h 里重新分配这四个脚，再删掉这句 #error。"
#endif

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