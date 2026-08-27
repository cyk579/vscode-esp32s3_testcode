#include <stdbool.h>                                      // 使用 bool 类型。
#include <stdint.h>                                       // 使用 uint8_t、uint32_t 等定长整数类型。
#include <stdlib.h>                                       // 使用 abs()。
#include "driver/gpio.h"                                  // GPIO 驱动。
#include "driver/ledc.h"                                  // PWM 驱动。
#include "esp_log.h"                                      // 串口日志。
#include "esp_timer.h"                                    // 超声波微秒计时。
#include "esp_rom_sys.h"                                  // esp_rom_delay_us()。
#include "freertos/FreeRTOS.h"                            // FreeRTOS 基础定义。
#include "freertos/task.h"                                // FreeRTOS 任务和延时。
#include "tft_st7735.h"                                   // 沿用仓库已有 TFT 驱动。

/* ======================== 运行模式开关 ======================== */

#define FULL_RUN_ENABLE 0                                 // 0：巡线到 7 cm 后永久停车测试；1：执行完整避障并跑到 END。

/* ======================== 四路红外 ======================== */

#define OUT1_GPIO GPIO_NUM_41                             // OUT1：实车最右侧原始通道。
#define OUT2_GPIO GPIO_NUM_42                             // OUT2：实车右中原始通道。
#define OUT3_GPIO GPIO_NUM_2                              // OUT3：实车左中原始通道。
#define OUT4_GPIO GPIO_NUM_1                              // OUT4：实车最左侧原始通道。
#define IR_ACTIVE_LEVEL 0                                 // 当前模块黑线输出低电平。
#define REVERSE_SENSOR_ORDER 1                            // 实车 OUT1 在右、OUT4 在左，因此软件顺序反转。
#define CENTER_MASK 0x06U                                 // 软件左到右排列时，中间两路压线为 0110。
#define CONTROL_PERIOD_MS 5U                              // 红外读取和电机控制周期 5 ms。
#define REVERSE_CONFIRM_CYCLES 2U                         // 左右纠偏方向直接反转时连续确认 2 次，只增加约 5 ms 延迟。

/* ======================== 三路电机 ======================== */

#define A_PWM GPIO_NUM_9                                  // Motor A PWM。
#define A_IN1 GPIO_NUM_12                                 // Motor A 方向 1。
#define A_IN2 GPIO_NUM_10                                 // Motor A 方向 2。
#define B_PWM GPIO_NUM_4                                  // Motor B PWM。
#define B_IN1 GPIO_NUM_6                                  // Motor B 方向 1。
#define B_IN2 GPIO_NUM_5                                  // Motor B 方向 2。
#define D_PWM GPIO_NUM_16                                 // Motor D PWM。
#define D_IN1 GPIO_NUM_7                                  // Motor D 方向 1。
#define D_IN2 GPIO_NUM_15                                 // Motor D 方向 2。
#define STBY_GPIO GPIO_NUM_8                              // TB6612 总使能。

#define MOTOR_A_SIGN 1                                    // Motor A 接线方向校准。
#define MOTOR_B_SIGN 1                                    // Motor B 接线方向校准。
#define MOTOR_D_SIGN -1                                   // Motor D 当前实车方向校准。
#define YAW_SIGN 1                                        // 若所有左右转向完全相反，只把 1 改成 -1。

#define MOTOR_A_GAIN 100                                  // Motor A 机械增益百分比；直线长期偏航时可微调。
#define MOTOR_B_GAIN 100                                  // Motor B 机械增益百分比。
#define MOTOR_D_GAIN 100                                  // Motor D 机械增益百分比；直线长期偏航时可微调。

#define PWM_MAX 1023U                                     // 10 bit PWM 最大值。
#define MAX_OUTPUT 40                                     // 最终逻辑输出限制为 ±40%。

/* ======================== 简单巡线参数 ======================== */

#define STRAIGHT_SPEED 27                                 // 0110 居中时的前进速度。
#define SMALL_TURN_SPEED 23                               // 小偏差时的前进速度。
#define LARGE_TURN_SPEED 17                               // 大偏差/急弯时的前进速度。
#define SMALL_YAW 11                                      // 小偏差 yaw；保证 Motor B 不落入低 PWM 死区。
#define LARGE_YAW 18                                      // 大偏差 yaw。
#define LOST_FORWARD_SPEED 8                              // 丢线搜索时保留很小的前进量。
#define LOST_YAW 20                                       // 丢线搜索使用较强 yaw。
#define LOST_SEARCH_MS 320U                               // 最多搜索 320 ms，找不到就停车，禁止转一整圈。

/* ======================== 超声波 ======================== */

#define ULTRASONIC_TRIG GPIO_NUM_18                       // HC-SR04 TRIG。
#define ULTRASONIC_ECHO GPIO_NUM_11                       // HC-SR04 ECHO。
#define ULTRASONIC_PERIOD_MS 60U                          // 约 60 ms 测一次；HC-SR04 不建议再明显缩短。
#define ECHO_TIMEOUT_US 30000LL                           // 单次等待回波最多 30 ms。
#define MIN_DISTANCE_CM 2.0f                              // 小于 2 cm 判无效。
#define MAX_DISTANCE_CM 400.0f                            // 大于 400 cm 判无效。
#define OBSTACLE_DETECT_CM 7.0f                           // <=7 cm 触发停车/避障。
#define OBSTACLE_CLEAR_CM 15.0f                           // 左移后 >=15 cm 认为正前方已清障。

/* ======================== 避障 ======================== */

#define AVOID_BRAKE_MS 180U                               // 正式避障前先停车 180 ms。
#define AVOID_LATERAL_SPEED 22                            // 横移 L：A/D≈L/2，B≈-L。
#define AVOID_LEFT_MIN_MS 250U                            // 左移至少 250 ms。
#define AVOID_LEFT_NO_ECHO_CLEAR_MS 450U                  // 左移 450 ms 后若无回波，也允许视为已移出障碍物波束。
#define AVOID_LEFT_TIMEOUT_MS 1800U                       // 左移最长 1.8 s。
#define AVOID_FORWARD_SPEED 18                            // 绕过障碍时向前速度。
#define AVOID_FORWARD_MS 750U                             // 绕过障碍向前约 0.75 s。
#define AVOID_RIGHT_MIN_MS 200U                           // 回移至少 200 ms，防止刚碰到外侧线就退出。
#define AVOID_RIGHT_TIMEOUT_MS 1800U                      // 回移最长 1.8 s。

/* ======================== 终点与日志 ======================== */

#define END_CONFIRM_MS 30U                                // 避障完成后，原始四路全黑约 30 ms 即 END。
#define DISPLAY_PERIOD_MS 100U                            // TFT 200 ms 刷一次即可调试，避免整屏刷新过重。
#define LOG_PERIOD_MS 100U                                // 串口每 100 ms 输出一行。
#define START_DELAY_MS 1500U                              // 上电后等待 1.5 s。

typedef struct {                                          // 电机硬件描述。
    gpio_num_t in1;                                       // 方向脚 1。
    gpio_num_t in2;                                       // 方向脚 2。
    ledc_channel_t channel;                               // PWM 通道。
    int sign;                                             // 电气方向校准。
    int gain;                                             // 机械增益百分比。
} motor_t;                                                // 电机结构结束。

typedef enum {                                            // 超声波状态。
    US_OK,                                                // 测距正常。
    US_NO_ECHO,                                           // 没收到回波。
    US_ECHO_HIGH,                                         // ECHO 卡高。
    US_OUT_OF_RANGE                                       // 距离超范围。
} us_status_t;                                            // 超声波状态结束。

typedef enum {                                            // 小车运行状态。
    STATE_LINE,                                           // 正常巡线。
    STATE_BRAKE,                                          // 障碍前停车。
    STATE_LEFT,                                           // 左横移。
    STATE_FORWARD,                                        // 绕障向前。
    STATE_RIGHT,                                          // 右横移回线。
    STATE_DISTANCE_STOP,                                  // 7 cm 测距测试停车。
    STATE_FAIL_STOP,                                      // 避障异常安全停车。
    STATE_END                                             // T 型终点停车。
} car_state_t;                                            // 运行状态结束。

static const char *TAG = "car_simple";                    // 串口日志 TAG。
static const gpio_num_t sensors[4] = {                    // 红外原始 GPIO 顺序。
    OUT1_GPIO,                                            // 原始 OUT1。
    OUT2_GPIO,                                            // 原始 OUT2。
    OUT3_GPIO,                                            // 原始 OUT3。
    OUT4_GPIO                                             // 原始 OUT4。
};                                                        // 红外数组结束。
static const int weights[4] = {-3, -1, 1, 3};            // 软件左到右权重：左负、右正。
static const motor_t motor_a = {A_IN1, A_IN2, LEDC_CHANNEL_0, MOTOR_A_SIGN, MOTOR_A_GAIN}; // Motor A。
static const motor_t motor_b = {B_IN1, B_IN2, LEDC_CHANNEL_1, MOTOR_B_SIGN, MOTOR_B_GAIN}; // Motor B。
static const motor_t motor_d = {D_IN1, D_IN2, LEDC_CHANNEL_2, MOTOR_D_SIGN, MOTOR_D_GAIN}; // Motor D。

static volatile tft_status_t display_status = {           // TFT 共享状态。
    -1.0f,                                                // 初始距离未知。
    0,                                                    // 初始 mask。
    0,                                                    // 初始 error。
    0,                                                    // 初始 yaw，沿用 TFT 的 TURN 字段。
    0,                                                    // 初始 A。
    0,                                                    // 初始 B。
    0,                                                    // 初始 D。
    "TEST"                                                // FULL_RUN_ENABLE=0 时开机显示 TEST。
};                                                        // TFT 状态结束。

static volatile float us_distance_cm = -1.0f;             // 最新一次超声波距离。
static volatile us_status_t us_status = US_NO_ECHO;       // 最新一次超声波状态。
static volatile uint32_t us_sequence = 0;                 // 每产生一次新测距结果就递增。
static car_state_t car_state = STATE_LINE;                // 初始进入巡线状态。

static int clamp_int(int value, int limit)                // 整数限幅。
{                                                         // 限幅函数开始。
    if (value > limit) return limit;                      // 超过正限幅。
    if (value < -limit) return -limit;                    // 超过负限幅。
    return value;                                         // 范围内保持原值。
}                                                         // 限幅函数结束。

static int sign_int(int value)                            // 返回 -1、0、+1。
{                                                         // 符号函数开始。
    return (value > 0) - (value < 0);                    // 用两个布尔表达式得到符号。
}                                                         // 符号函数结束。

static void motor_set(const motor_t *motor, int logical_speed) // 输出一个电机。
{                                                         // 电机输出函数开始。
    static int last_direction[3] = {0, 0, 0};            // 记录三路电机上一次电气方向。
    int scaled = logical_speed * motor->gain / 100;       // 先做简单机械增益校准。
    scaled = clamp_int(scaled, MAX_OUTPUT);               // 增益后再次限幅。
    int electrical = scaled * motor->sign;                // 转换成真实电气方向。
    int direction = sign_int(electrical);                 // 得到 -1/0/+1 方向。
    int index = (int)motor->channel;                      // 通道 0/1/2 作为方向历史下标。
    if (direction != last_direction[index]) {             // 只有方向真正变化时才先撤掉 PWM。
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, 0); // 换向前 PWM=0。
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel); // 立即生效。
        gpio_set_level(motor->in1, electrical > 0);       // 正向时 IN1=1。
        gpio_set_level(motor->in2, electrical < 0);       // 反向时 IN2=1；停止时两脚均 0。
        last_direction[index] = direction;                // 保存新方向。
    }                                                     // 换向处理结束。
    uint32_t duty = PWM_MAX * (uint32_t)abs(electrical) / 100U; // 百分比转换成 10 bit PWM。
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel, duty); // 设置占空比。
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel); // 立即更新。
}                                                         // 电机输出函数结束。

static void drive_body(int forward, int lateral, int yaw, int *a, int *b, int *d) // 三轮 60°/kiwi 向量分解。
{                                                         // 底盘分解开始。
    int z = yaw * YAW_SIGN;                               // 统一处理整车左右 yaw 方向校准。
    int half_lateral = lateral / 2;                       // 横移时 A/D 各承担约一半分量。
    *a = clamp_int(-forward + half_lateral + z, MAX_OUTPUT); // A：前进、横移、yaw 叠加。
    *b = clamp_int(-lateral + z, MAX_OUTPUT);             // B：不承担 forward，承担完整 lateral 和 yaw。
    *d = clamp_int(+forward + half_lateral + z, MAX_OUTPUT); // D：与 A 对称。
    motor_set(&motor_a, *a);                              // 输出 A。
    motor_set(&motor_b, *b);                              // 输出 B。
    motor_set(&motor_d, *d);                              // 输出 D。
}                                                         // 底盘分解结束。

static void stop_car(int *a, int *b, int *d)              // 统一停车。
{                                                         // 停车函数开始。
    drive_body(0, 0, 0, a, b, d);                        // forward/lateral/yaw 全部为 0。
}                                                         // 停车函数结束。

static void hardware_init(void)                           // 初始化电机、红外、超声波 GPIO。
{                                                         // 硬件初始化开始。
    const gpio_num_t outputs[] = {                        // 普通输出 GPIO。
        A_IN1, A_IN2, B_IN1, B_IN2, D_IN1, D_IN2, STBY_GPIO // A/B/D 方向脚和 STBY。
    };                                                    // 输出数组结束。
    for (size_t i = 0; i < sizeof(outputs) / sizeof(outputs[0]); ++i) { // 逐个配置普通输出。
        gpio_reset_pin(outputs[i]);                       // 复位旧 GPIO 配置。
        gpio_set_direction(outputs[i], GPIO_MODE_OUTPUT); // 设置为输出。
    }                                                     // 普通输出初始化结束。
    for (size_t i = 0; i < 4; ++i) {                     // 配置四路红外输入。
        gpio_reset_pin(sensors[i]);                       // 复位红外 GPIO。
        gpio_set_direction(sensors[i], GPIO_MODE_INPUT);  // 设置为输入。
    }                                                     // 红外输入初始化结束。
    const ledc_timer_config_t timer = {                   // PWM 定时器配置。
        .speed_mode = LEDC_LOW_SPEED_MODE,                // 低速 LEDC。
        .duty_resolution = LEDC_TIMER_10_BIT,             // 10 bit 分辨率。
        .timer_num = LEDC_TIMER_0,                        // 定时器 0。
        .freq_hz = 2000,                                  // 2 kHz，沿用已验证设置。
        .clk_cfg = LEDC_AUTO_CLK                          // 自动时钟。
    };                                                    // PWM 定时器配置结束。
    ESP_ERROR_CHECK(ledc_timer_config(&timer));           // 应用 PWM 定时器配置。
    const gpio_num_t pwm_gpio[3] = {A_PWM, B_PWM, D_PWM}; // A/B/D PWM GPIO。
    for (int i = 0; i < 3; ++i) {                        // 配置三路 PWM。
        const ledc_channel_config_t channel = {           // 当前 PWM 通道配置。
            .gpio_num = pwm_gpio[i],                      // 当前 PWM GPIO。
            .speed_mode = LEDC_LOW_SPEED_MODE,            // 与定时器一致。
            .channel = (ledc_channel_t)i,                 // A/B/D 使用通道 0/1/2。
            .intr_type = LEDC_INTR_DISABLE,               // 不使用 PWM 中断。
            .timer_sel = LEDC_TIMER_0,                    // 使用定时器 0。
            .duty = 0                                     // 初始 PWM=0。
        };                                                // 当前通道配置结束。
        ESP_ERROR_CHECK(ledc_channel_config(&channel));   // 应用当前 PWM 通道配置。
    }                                                     // 三路 PWM 初始化结束。
    gpio_set_level(STBY_GPIO, 1);                         // 使能 TB6612。
    gpio_reset_pin(ULTRASONIC_TRIG);                      // 复位超声波 TRIG。
    gpio_set_direction(ULTRASONIC_TRIG, GPIO_MODE_OUTPUT); // TRIG 设置为输出。
    gpio_set_level(ULTRASONIC_TRIG, 0);                   // TRIG 初始拉低。
    gpio_reset_pin(ULTRASONIC_ECHO);                      // 复位超声波 ECHO。
    gpio_set_direction(ULTRASONIC_ECHO, GPIO_MODE_INPUT); // ECHO 设置为输入。
    gpio_set_pull_mode(ULTRASONIC_ECHO, GPIO_PULLDOWN_ONLY); // ECHO 下拉，断线时避免悬空。
}                                                         // 硬件初始化结束。

static uint8_t read_active_mask(void)                     // 直接读取四路红外，5 ms 一次，不做均值滤波。
{                                                         // 红外读取开始。
    uint8_t mask = 0;                                     // 初始四位 ACTIVE=0000。
    for (size_t connector = 0; connector < 4; ++connector) { // 逐个读取 OUT1~OUT4。
        size_t position = REVERSE_SENSOR_ORDER ? 3U - connector : connector; // 换算成软件左到右位置。
        if (gpio_get_level(sensors[connector]) == IR_ACTIVE_LEVEL) { // 当前探头压到黑线。
            mask |= (uint8_t)(1U << position);            // 对应 ACTIVE bit 置 1。
        }                                                 // 当前探头判断结束。
    }                                                     // 四路读取结束。
    return mask;                                          // 返回软件左到右的 4 位 ACTIVE。
}                                                         // 红外读取函数结束。

static int line_error(uint8_t mask)                       // 用 4 位 ACTIVE 计算离散横向误差。
{                                                         // 误差计算开始。
    int sum = 0;                                          // 权重和。
    int count = 0;                                        // 黑线探头数量。
    for (int i = 0; i < 4; ++i) {                        // 遍历软件左到右四个 bit。
        if (mask & (1U << i)) {                           // 当前 bit 检测到黑线。
            sum += weights[i];                            // 累加 -3/-1/+1/+3。
            ++count;                                      // 有效探头数 +1。
        }                                                 // 当前 bit 处理结束。
    }                                                     // 四个 bit 处理结束。
    return count ? sum * 10 / count : 0;                  // 输出约 -30~+30；完全丢线返回 0。
}                                                         // 误差计算结束。

static int anti_reverse_error(int raw_error)              // 只抑制“左纠偏下一帧立刻变右纠偏”的抖动，不做平均滤波。
{                                                         // 反向去抖开始。
    static int accepted_sign = 0;                         // 当前已经接受的纠偏方向。
    static int pending_sign = 0;                          // 正在等待确认的新方向。
    static uint8_t pending_count = 0;                     // 新方向已经连续出现的次数。
    int new_sign = sign_int(raw_error);                   // 当前误差方向。
    if (new_sign == 0) {                                  // 已回到中心时。
        accepted_sign = 0;                                // 立即接受中心。
        pending_sign = 0;                                 // 清除待确认方向。
        pending_count = 0;                                // 清除待确认次数。
        return 0;                                         // 直接输出 0 误差。
    }                                                     // 中心处理结束。
    if (accepted_sign == 0 || new_sign == accepted_sign) { // 从中心出发或仍在同一侧时。
        accepted_sign = new_sign;                         // 立即接受当前方向。
        pending_sign = 0;                                 // 不再需要待确认方向。
        pending_count = 0;                                // 清零待确认次数。
        return raw_error;                                 // 直接采用当前误差。
    }                                                     // 同方向处理结束。
    if (new_sign != pending_sign) {                       // 第一次看到相反方向。
        pending_sign = new_sign;                          // 记录这个相反方向。
        pending_count = 1;                                // 记录出现一次。
        return 0;                                         // 这一帧先直行，不立即反打。
    }                                                     // 第一次反向处理结束。
    if (++pending_count >= REVERSE_CONFIRM_CYCLES) {      // 相反方向连续出现到确认次数。
        accepted_sign = new_sign;                         // 正式接受新方向。
        pending_sign = 0;                                 // 清除待确认方向。
        pending_count = 0;                                // 清除计数。
        return raw_error;                                 // 输出新的真实误差。
    }                                                     // 反向确认结束。
    return 0;                                             // 尚未确认时继续直行一帧。
}                                                         // 反向去抖结束。

static bool raw_all_black(void)                           // 直接读取原始四路，判断 T 型横杠。
{                                                         // T 型判断开始。
    for (size_t i = 0; i < 4; ++i) {                     // 检查四个原始通道。
        if (gpio_get_level(sensors[i]) != IR_ACTIVE_LEVEL) return false; // 任意一路不是黑线就不是终点横杠。
    }                                                     // 四路检查结束。
    return true;                                          // 四路同时黑线。
}                                                         // T 型判断结束。

static const char *us_name(us_status_t status)            // 超声波状态转字符串。
{                                                         // 超声波状态字符串开始。
    if (status == US_OK) return "OK";                     // 正常。
    if (status == US_NO_ECHO) return "NO_ECHO";           // 无回波。
    if (status == US_ECHO_HIGH) return "ECHO_HIGH";       // ECHO 卡高。
    return "OUT_RANGE";                                   // 超范围。
}                                                         // 超声波状态字符串结束。

static float ultrasonic_read_once(us_status_t *status)    // 单次 HC-SR04 测距，不做三点中值，便于 7 cm 快速响应。
{                                                         // 测距函数开始。
    *status = US_NO_ECHO;                                 // 默认状态为无回波。
    gpio_set_level(ULTRASONIC_TRIG, 0);                   // 确保 TRIG 为低。
    int64_t wait_start = esp_timer_get_time();            // 记录等待旧 ECHO 结束的时刻。
    while (gpio_get_level(ULTRASONIC_ECHO)) {             // 如果 ECHO 尚未拉低则等待。
        if (esp_timer_get_time() - wait_start > ECHO_TIMEOUT_US) { // 等待超过 30 ms。
            *status = US_ECHO_HIGH;                       // 标记 ECHO 卡高。
            return -1.0f;                                 // 返回无效距离。
        }                                                 // 卡高判断结束。
    }                                                     // 等待旧 ECHO 结束。
    esp_rom_delay_us(2);                                  // TRIG 先保持低约 2 us。
    gpio_set_level(ULTRASONIC_TRIG, 1);                   // TRIG 拉高。
    esp_rom_delay_us(10);                                 // 保持 10 us。
    gpio_set_level(ULTRASONIC_TRIG, 0);                   // TRIG 拉低完成触发。
    wait_start = esp_timer_get_time();                    // 记录等待新 ECHO 上升沿时刻。
    while (!gpio_get_level(ULTRASONIC_ECHO)) {            // 等待 ECHO 上升。
        if (esp_timer_get_time() - wait_start > ECHO_TIMEOUT_US) { // 30 ms 没收到回波。
            *status = US_NO_ECHO;                         // 标记无回波。
            return -1.0f;                                 // 返回无效距离。
        }                                                 // 无回波判断结束。
    }                                                     // 等待 ECHO 上升结束。
    int64_t pulse_start = esp_timer_get_time();           // 记录 ECHO 高电平起点。
    while (gpio_get_level(ULTRASONIC_ECHO)) {             // 等待 ECHO 回落。
        if (esp_timer_get_time() - pulse_start > ECHO_TIMEOUT_US) { // 高电平超过 30 ms。
            *status = US_ECHO_HIGH;                       // 标记卡高。
            return -1.0f;                                 // 返回无效距离。
        }                                                 // 卡高判断结束。
    }                                                     // 等待 ECHO 回落结束。
    float distance = (float)(esp_timer_get_time() - pulse_start) / 58.0f; // 脉宽换算成厘米。
    if (distance < MIN_DISTANCE_CM || distance > MAX_DISTANCE_CM) { // 检查有效范围。
        *status = US_OUT_OF_RANGE;                        // 标记超范围。
        return -1.0f;                                     // 超范围不参与控制。
    }                                                     // 有效范围检查结束。
    *status = US_OK;                                      // 本次测距正常。
    return distance;                                      // 返回当前距离。
}                                                         // 单次测距结束。

static const char *state_name(car_state_t state)          // 运行状态转 TFT/串口字符串。
{                                                         // 状态字符串开始。
    if (state == STATE_BRAKE) return "AV-L";              // 障碍前停车先显示 AV-L。
    if (state == STATE_LEFT) return "AV-L";               // 左移。
    if (state == STATE_FORWARD) return "AV-F";            // 绕障前进。
    if (state == STATE_RIGHT) return "AV-R";              // 右移。
    if (state == STATE_DISTANCE_STOP) return "DIST";      // 7 cm 测距断点。
    if (state == STATE_FAIL_STOP) return "FAIL";          // 避障失败安全停车。
    if (state == STATE_END) return "END";                 // T 型终点。
    return FULL_RUN_ENABLE ? "LINE" : "TEST";             // 正常巡线时显示 LINE 或 TEST。
}                                                         // 状态字符串结束。

static void ultrasonic_task(void *arg)                    // 超声波独立任务，避免阻塞 5 ms 红外控制。
{                                                         // 超声波任务开始。
    (void)arg;                                            // 忽略任务参数。
    while (true) {                                        // 永久测距。
        us_status_t status = US_NO_ECHO;                  // 本次状态变量。
        float distance = ultrasonic_read_once(&status);   // 执行一次测距。
        us_distance_cm = distance;                        // 保存最新距离。
        us_status = status;                               // 保存最新状态。
        display_status.distance_cm = distance;            // TFT 更新距离。
        ++us_sequence;                                    // 标记得到新测距。
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));  // 等待约 60 ms 后再触发。
    }                                                     // 永久测距循环结束。
}                                                         // 超声波任务结束。

static void display_task(void *arg)                       // TFT 独立任务。
{                                                         // TFT 任务开始。
    (void)arg;                                            // 忽略任务参数。
    if (!tft_st7735_init()) vTaskDelete(NULL);            // TFT 初始化失败就停止显示任务。
    while (true) {                                        // 永久刷新。
        tft_status_t snapshot = {                         // 建立当前显示快照。
            display_status.distance_cm,                   // 距离。
            display_status.ir_mask,                       // ACTIVE。
            display_status.error,                         // 误差。
            display_status.turn,                          // yaw。
            display_status.motor_a,                       // A。
            display_status.motor_b,                       // B。
            display_status.motor_d,                       // D。
            display_status.mode                           // 状态。
        };                                                // 快照结束。
        tft_st7735_show(&snapshot);                       // 绘制当前状态。
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_PERIOD_MS));     // 约 200 ms 更新一次。
    }                                                     // 永久刷新循环结束。
}                                                         // TFT 任务结束。

void app_main(void)                                       // ESP-IDF 主入口。
{                                                         // 主函数开始。
    hardware_init();                                      // 初始化所有硬件。
    xTaskCreate(ultrasonic_task, "ultrasonic", 2048, NULL, 1, NULL); // 启动超声波独立任务。
    xTaskCreate(display_task, "tft", 4096, NULL, 1, NULL); // 启动 TFT 独立任务。
    vTaskPrioritySet(NULL, 3);                            // 提高主控制任务优先级，让 5 ms 巡线优先于 TFT。
    int a = 0;                                            // 当前 A 逻辑输出。
    int b = 0;                                            // 当前 B 逻辑输出。
    int d = 0;                                            // 当前 D 逻辑输出。
    int yaw = 0;                                          // 当前 yaw 命令。
    int last_yaw_sign = 1;                                // 最后一次正确纠偏 yaw 方向。
    uint32_t lost_ms = 0;                                 // 连续丢线时间。
    uint32_t state_ms = 0;                                // 当前避障状态时间。
    uint32_t left_shift_ms = 0;                           // 左移持续时间。
    uint32_t end_ms = 0;                                  // END 原始四黑持续时间。
    uint32_t log_ms = LOG_PERIOD_MS;                      // 日志计时。
    uint32_t last_us_sequence = 0;                        // 已处理的超声波序号。
    bool line_seen = false;                               // 上电后是否真正看到过赛道。
    bool end_armed = false;                               // 只有避障完成后才允许识别 END，避免前面的 60°/90°弯误停。
    stop_car(&a, &b, &d);                                 // 上电明确停车。
    vTaskDelay(pdMS_TO_TICKS(START_DELAY_MS));            // 等待放车。
    TickType_t last_wake = xTaskGetTickCount();           // 5 ms 固定周期基准。

    while (true) {                                        // 主控制永久循环。
        uint8_t mask = read_active_mask();                // 直接读取当前 4 位 ACTIVE。
        int raw_error = line_error(mask);                 // 根据 4 位 ACTIVE 得到离散误差。
        int error = anti_reverse_error(raw_error);        // 仅对直接左右反打做 2 次确认。
        bool end_raw = raw_all_black();                   // 读取原始四路判断 T 型横杠。
        float distance = us_distance_cm;                  // 取最新距离。
        us_status_t current_us_status = us_status;        // 取最新超声波状态。
        uint32_t current_us_sequence = us_sequence;       // 取最新超声波序号。
        bool us_new = current_us_sequence != last_us_sequence; // 判断是否有新测距。
        bool us_valid = current_us_status == US_OK && distance > 0.0f; // 判断新距离是否有效。
        if (us_new) last_us_sequence = current_us_sequence; // 标记该帧测距已经处理。
        if (mask != 0U) line_seen = true;                 // 看到过任意黑线后才允许运动。

        if (car_state == STATE_LINE && line_seen && us_new && us_valid && distance <= OBSTACLE_DETECT_CM) { // <=7 cm 立即触发。
            stop_car(&a, &b, &d);                         // 立即停车。
            yaw = 0;                                      // 清零 yaw。
            lost_ms = 0;                                  // 清零丢线计时。
            state_ms = 0;                                 // 清零状态计时。
            if (FULL_RUN_ENABLE) car_state = STATE_BRAKE; // 完整模式：先刹车再避障。
            else car_state = STATE_DISTANCE_STOP;         // 测距模式：永久停车直到复位。
            ESP_LOGW(TAG, "BREAKPOINT 7CM dist=%.1fcm FULL_RUN=%d", (double)distance, FULL_RUN_ENABLE); // 打印 7 cm 断点。
        }                                                 // 7 cm 触发逻辑结束。

        if (car_state == STATE_END) {                     // 已到最终 T 型终点。
            stop_car(&a, &b, &d);                         // 永久停车。
            yaw = 0;                                      // 清零 yaw。
        } else if (car_state == STATE_DISTANCE_STOP) {    // 测距测试模式已到 7 cm。
            stop_car(&a, &b, &d);                         // 永久停车。
            yaw = 0;                                      // 清零 yaw。
        } else if (car_state == STATE_FAIL_STOP) {        // 避障异常。
            stop_car(&a, &b, &d);                         // 安全停车。
            yaw = 0;                                      // 清零 yaw。
        } else if (car_state == STATE_BRAKE) {            // 正式避障前停车。
            stop_car(&a, &b, &d);                         // 保持停车。
            yaw = 0;                                      // 保持 yaw=0。
            if (state_ms >= AVOID_BRAKE_MS) {             // 停够 180 ms。
                car_state = STATE_LEFT;                   // 开始左横移。
                state_ms = 0;                             // 清零新状态计时。
                ESP_LOGI(TAG, "AVOID LEFT start");         // 打印左移开始。
            }                                             // 刹车结束判断完成。
        } else if (car_state == STATE_LEFT) {             // 左横移。
            drive_body(0, AVOID_LATERAL_SPEED, 0, &a, &b, &d); // 输出约 [+L/2,-L,+L/2]，不附加 yaw。
            yaw = 0;                                      // 明确无 yaw。
            bool far_clear = us_new && us_valid && distance >= OBSTACLE_CLEAR_CM; // 新有效距离 >=15 cm。
            bool no_echo_clear = us_new && current_us_status == US_NO_ECHO && state_ms >= AVOID_LEFT_NO_ECHO_CLEAR_MS; // 已左移较久且障碍退出波束。
            if (state_ms >= AVOID_LEFT_MIN_MS && (far_clear || no_echo_clear)) { // 满足清障条件。
                left_shift_ms = state_ms;                 // 记录左移时间。
                car_state = STATE_FORWARD;                // 进入绕障前进。
                state_ms = 0;                             // 清零新状态计时。
                ESP_LOGI(TAG, "LEFT done time=%lums clear=%d noecho=%d", (unsigned long)left_shift_ms, far_clear, no_echo_clear); // 打印左移结果。
            } else if (state_ms >= AVOID_LEFT_TIMEOUT_MS) { // 左移超时仍未清障。
                car_state = STATE_FAIL_STOP;              // 直接安全停车，不盲目前进。
                ESP_LOGE(TAG, "LEFT timeout -> FAIL STOP"); // 打印失败原因。
            }                                             // 左移状态判断结束。
        } else if (car_state == STATE_FORWARD) {          // 绕过障碍向前。
            drive_body(AVOID_FORWARD_SPEED, 0, 0, &a, &b, &d); // 只给 forward。
            yaw = 0;                                      // 不附加 yaw。
            if (state_ms >= AVOID_FORWARD_MS) {           // 前进约 750 ms。
                car_state = STATE_RIGHT;                  // 开始右横移回路线。
                state_ms = 0;                             // 清零状态计时。
                ESP_LOGI(TAG, "AVOID RIGHT start left=%lums", (unsigned long)left_shift_ms); // 打印回移目标时间。
            }                                             // 前进结束判断完成。
        } else if (car_state == STATE_RIGHT) {            // 右横移。
            drive_body(0, -AVOID_LATERAL_SPEED, 0, &a, &b, &d); // 横移方向反向。
            yaw = 0;                                      // 不附加 yaw。
            bool centered = mask == CENTER_MASK;          // 中间两路重新稳定压线时认为回到赛道。
            bool matched_time = left_shift_ms > 0U && state_ms >= left_shift_ms; // 或按左移时间对称返回。
            if (state_ms >= AVOID_RIGHT_MIN_MS && (centered || matched_time)) { // 满足回线条件。
                car_state = STATE_LINE;                   // 恢复正常巡线。
                end_armed = true;                         // 现在才允许识别赛道最后的 T 型 END。
                state_ms = 0;                             // 清零避障计时。
                lost_ms = 0;                              // 清零丢线计时。
                yaw = 0;                                  // 从 0 yaw 重新巡线。
                ESP_LOGI(TAG, "AVOID done; END armed");    // 打印避障完成。
            } else if (state_ms >= AVOID_RIGHT_TIMEOUT_MS) { // 回移超时。
                car_state = STATE_FAIL_STOP;              // 安全停车。
                ESP_LOGE(TAG, "RIGHT timeout -> FAIL STOP"); // 打印失败原因。
            }                                             // 右移状态判断结束。
        } else if (!line_seen) {                          // 上电后还没看到赛道。
            stop_car(&a, &b, &d);                         // 无条件停车。
            yaw = 0;                                      // yaw=0。
        } else if (end_armed && end_raw) {                // 只有避障完成后才判断最终 T 型 END。
            end_ms += CONTROL_PERIOD_MS;                  // 累计四路原始全黑时间。
            stop_car(&a, &b, &d);                         // 一看到终点横杠就先停车确认，防止冲过。
            yaw = 0;                                      // 清零 yaw。
            if (end_ms >= END_CONFIRM_MS) {               // 持续约 30 ms。
                car_state = STATE_END;                    // 锁存最终 END。
                ESP_LOGW(TAG, "END confirmed");            // 打印终点确认。
            }                                             // END 确认结束。
        } else if (mask == 0U) {                          // 已经巡线但当前完全丢线。
            end_ms = 0;                                   // 不是终点，清零 END 计时。
            lost_ms += CONTROL_PERIOD_MS;                 // 累计丢线时间。
            if (lost_ms <= LOST_SEARCH_MS) {              // 只允许搜索 320 ms。
                yaw = last_yaw_sign * LOST_YAW;           // 沿最后一次真实纠偏方向继续找线。
                drive_body(LOST_FORWARD_SPEED, 0, yaw, &a, &b, &d); // 小前进 + 正确纯 yaw，适合 60°/90°急弯。
            } else {                                      // 320 ms 仍找不到线。
                stop_car(&a, &b, &d);                     // 停车，绝不转一整圈。
                yaw = 0;                                  // 清零 yaw。
            }                                             // 丢线搜索结束。
        } else {                                          // 正常看到黑线，执行简单离散纠偏。
            end_ms = 0;                                   // 非终点横杠时清零 END 计时。
            lost_ms = 0;                                  // 看到线即清零丢线计时。
            int abs_error = abs(error);                   // 偏差绝对值。
            int forward = STRAIGHT_SPEED;                 // 默认直行速度。
            yaw = 0;                                      // 默认不转向。
            if (abs_error == 0) {                         // 0110 或其他左右对称组合。
                forward = STRAIGHT_SPEED;                 // 正常直行。
                yaw = 0;                                  // 不纠偏。
            } else if (abs_error <= 10) {                 // 内侧单探头或轻微偏差。
                forward = SMALL_TURN_SPEED;               // 略微降速。
                yaw = (error < 0 ? +SMALL_YAW : -SMALL_YAW); // 线在左(error<0)就左转；线在右就右转。
            } else {                                      // 外侧探头/大偏差/急弯。
                forward = LARGE_TURN_SPEED;               // 明显降速防止冲弯。
                yaw = (error < 0 ? +LARGE_YAW : -LARGE_YAW); // 大幅纠偏，但不原地暴力旋转。
            }                                             // 三档纠偏结束。
            if (yaw != 0) last_yaw_sign = sign_int(yaw); // 记录最后一次正确 yaw 方向供急弯丢线搜索。
            drive_body(forward, 0, yaw, &a, &b, &d);      // 巡线只允许 forward+yaw，不混入 lateral。
        }                                                 // 主状态控制结束。

        display_status.distance_cm = distance;            // TFT 距离。
        display_status.ir_mask = mask;                    // TFT 显示 4 位 ACTIVE。
        display_status.error = error;                     // TFT 显示纠偏误差。
        display_status.turn = yaw;                        // TFT TURN 行显示真正 yaw。
        display_status.motor_a = a;                       // TFT A。
        display_status.motor_b = b;                       // TFT B。
        display_status.motor_d = d;                       // TFT D。
        display_status.mode = state_name(car_state);      // TFT 显示 TEST/LINE/AV-L/AV-F/AV-R/DIST/FAIL/END。

        if (log_ms >= LOG_PERIOD_MS) {                    // 到达日志周期。
            ESP_LOGI(TAG, "mode=%s mask=%u%u%u%u rawErr=%d err=%d yaw=%d dist=%.1f us=%s motor=[%d,%d,%d] lost=%lums endArm=%d", // 输出完整调试信息。
                     state_name(car_state),               // 当前状态。
                     mask & 1U,                           // 软件最左 bit。
                     (mask >> 1) & 1U,                    // 左中 bit。
                     (mask >> 2) & 1U,                    // 右中 bit。
                     (mask >> 3) & 1U,                    // 软件最右 bit。
                     raw_error,                           // 原始 4 位误差。
                     error,                               // 反向去抖后的控制误差。
                     yaw,                                 // yaw。
                     (double)distance,                    // 距离。
                     us_name(current_us_status),          // 超声波状态。
                     a, b, d,                             // 三轮逻辑输出。
                     (unsigned long)lost_ms,              // 丢线时间。
                     end_armed);                          // 是否已允许 END。
            log_ms = 0;                                   // 清零日志计时。
        }                                                 // 日志结束。

        log_ms += CONTROL_PERIOD_MS;                      // 累加日志时间。
        if (car_state == STATE_BRAKE || car_state == STATE_LEFT || car_state == STATE_FORWARD || car_state == STATE_RIGHT) { // 真正避障阶段才计时。
            state_ms += CONTROL_PERIOD_MS;                // 累加当前避障状态时间。
        }                                                 // 避障计时结束。
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS)); // 固定 5 ms 控制周期。
    }                                                     // 主循环结束。
}                                                         // app_main 结束。
