#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "jpeg_decoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "hal/usb_dwc_ll.h"
#include "soc/usb_dwc_struct.h"
#include "usb_stream.h"
#include "board_pins.h"
#include "endpoint_ball.h"

static const char *TAG = "cam";

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_PWR_GLITCH: return "POWER_GLITCH";
    default: return "OTHER";
    }
}

// ==================== LCD 屏幕 (ST7735, 128x160) ====================
// 本车接线集中在 board_pins.h。
#define LCD_WIDTH      128
#define LCD_HEIGHT     160

#define CAM_DISP_W     128
#define CAM_DISP_H     96
#define CAM_DISP_X     0
#define CAM_DISP_Y     32

// ==================== 超声波模块 ====================
#define SOUND_SPEED_CM_PER_US  0.0343f
#define MAX_RANGE_US           60000
static volatile float lcd_ultrasonic_cm = -1.0f;

// ==================== USB 摄像头缓冲 ====================
#define XFER_BUF_SIZE   (160 * 1024)
#define JPEG_BUF_SIZE   (1024 * 1024)

static spi_device_handle_t lcd_spi = NULL;

/**
 * @brief 向 LCD 发送一段字节（命令或数据）
 * @param dc  0=命令（D/C 拉低），1=数据（D/C 拉高）
 * @param buf 待发送数据首地址
 * @param len 字节数
 * @note  轮询式 SPI 传输，函数返回时数据已发完；传输失败直接 abort
 */
static void lcd_write_bytes(int dc, const uint8_t *buf, size_t len)
{
    gpio_set_level(LCD_DC_GPIO, dc);
    spi_transaction_t t = {0};
    t.length = len * 8;
    t.tx_buffer = buf;
    ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_spi, &t));
}

/** @brief 发送单字节 LCD 命令（D/C=0） */
static void lcd_cmd(uint8_t cmd) { lcd_write_bytes(0, &cmd, 1); }

/**
 * @brief 设置后续像素写入的矩形区域，并进入写显存状态
 * @param x0,y0 左上角坐标（含）
 * @param x1,y1 右下角坐标（含）
 * @note  依次发 0x2A(列地址)/0x2B(行地址)/0x2C(写显存)，返回后可连续写 RGB565 数据
 */
static void lcd_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    uint8_t d[4];
    lcd_cmd(0x2A);
    d[0] = 0; d[1] = x0; d[2] = 0; d[3] = x1;
    lcd_write_bytes(1, d, 4);
    lcd_cmd(0x2B);
    d[0] = 0; d[1] = y0; d[2] = 0; d[3] = y1;
    lcd_write_bytes(1, d, 4);
    lcd_cmd(0x2C);
}

/**
 * @brief 用单一颜色填满整屏
 * @param color RGB565 颜色，大端字节序（与屏幕线序一致）
 * @note  以 64 像素为一批循环发送，不需要整屏大小的缓冲
 */
static void lcd_fill(uint16_t color)
{
    /* ST7735 expects RGB565 high byte first.  ESP32 is little-endian, so
     * sending a uint16_t array directly reverses every pixel's byte order. */
    uint8_t buf[64 * 2];
    for (int i = 0; i < 64; i++) {
        buf[i * 2] = (uint8_t)(color >> 8);
        buf[i * 2 + 1] = (uint8_t)color;
    }
    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    gpio_set_level(LCD_DC_GPIO, 1);
    int total = LCD_WIDTH * LCD_HEIGHT;
    for (int i = 0; i < total; i += 64) {
        spi_transaction_t t = {0};
        t.length = sizeof(buf) * 8;
        t.tx_buffer = buf;
        ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_spi, &t));
    }
}

/**
 * @brief 把摄像头画面贴到屏幕上的固定显示区（左上角 CAM_DISP_X/CAM_DISP_Y）
 * @param rgb565 w*h 个 RGB565 大端像素
 * @param w,h    图像宽高，正常应为 CAM_DISP_W/CAM_DISP_H
 * @note  分片发送，每片最多 1024 字节
 */
static void lcd_blit_cam(const uint8_t *rgb565, int w, int h)
{
    lcd_set_window(CAM_DISP_X, CAM_DISP_Y, CAM_DISP_X + w - 1, CAM_DISP_Y + h - 1);
    gpio_set_level(LCD_DC_GPIO, 1);
    int total = w * h * 2;
    int off = 0;
    while (off < total) {
        int n = total - off;
        if (n > 1024) n = 1024;
        spi_transaction_t t = {0};
        t.length = n * 8;
        t.tx_buffer = rgb565 + off;
        ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_spi, &t));
        off += n;
    }
}

#define LCD_STATUS_HEIGHT 24

static const char lcd_font_chars[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-/=%:.";
static const uint8_t lcd_font[][5] = {
    {0x00,0x00,0x00,0x00,0x00},
    {0x3e,0x51,0x49,0x45,0x3e}, {0x00,0x42,0x7f,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4b,0x31},
    {0x18,0x14,0x12,0x7f,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3c,0x4a,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1e},
    {0x7e,0x11,0x11,0x11,0x7e}, {0x7f,0x49,0x49,0x49,0x36},
    {0x3e,0x41,0x41,0x41,0x22}, {0x7f,0x41,0x41,0x22,0x1c},
    {0x7f,0x49,0x49,0x49,0x41}, {0x7f,0x09,0x09,0x09,0x01},
    {0x3e,0x41,0x49,0x49,0x7a}, {0x7f,0x08,0x08,0x08,0x7f},
    {0x00,0x41,0x7f,0x41,0x00}, {0x20,0x40,0x41,0x3f,0x01},
    {0x7f,0x08,0x14,0x22,0x41}, {0x7f,0x40,0x40,0x40,0x40},
    {0x7f,0x02,0x0c,0x02,0x7f}, {0x7f,0x04,0x08,0x10,0x7f},
    {0x3e,0x41,0x41,0x41,0x3e}, {0x7f,0x09,0x09,0x09,0x06},
    {0x3e,0x41,0x51,0x21,0x5e}, {0x7f,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7f,0x01,0x01},
    {0x3f,0x40,0x40,0x40,0x3f}, {0x1f,0x20,0x40,0x20,0x1f},
    {0x3f,0x40,0x38,0x40,0x3f}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
    {0x08,0x08,0x3e,0x08,0x08}, {0x08,0x08,0x08,0x08,0x08},
    {0x20,0x10,0x08,0x04,0x02}, {0x14,0x14,0x14,0x14,0x14},
    {0x62,0x64,0x08,0x13,0x23}, {0x00,0x36,0x36,0x00,0x00},
    {0x00,0x60,0x60,0x00,0x00},
};

typedef struct {
    const char *state;
    bool armed;
    bool stby;
    int motor_a;
    int motor_b;
    int motor_d;
    float ultrasonic_cm;
    bool candidate;
    int threshold;
    int seed_x;
    int valid_rows;
    int confidence;
    const char *ball_phase;
} lcd_status_t;

static void lcd_status_char(uint8_t *buffer, int x, int y,
                            char character, uint16_t color)
{
    const char *found = strchr(lcd_font_chars, character);
    if (found == NULL) found = lcd_font_chars;
    const uint8_t *glyph = lcd_font[found - lcd_font_chars];
    for (int column = 0; column < 5; column++) {
        for (int row = 0; row < 7; row++) {
            if ((glyph[column] & (1U << row)) == 0) continue;
            int pixel_x = x + column;
            int pixel_y = y + row;
            if (pixel_x < 0 || pixel_x >= LCD_WIDTH ||
                pixel_y < 0 || pixel_y >= LCD_STATUS_HEIGHT) continue;
            size_t offset = ((size_t)pixel_y * LCD_WIDTH + pixel_x) * 2;
            buffer[offset] = (uint8_t)(color >> 8);
            buffer[offset + 1] = (uint8_t)color;
        }
    }
}

static void lcd_status_line(uint8_t *buffer, int row, const char *text)
{
    int x = 1;
    for (const char *cursor = text; *cursor != '\0' && x + 5 < LCD_WIDTH; cursor++) {
        lcd_status_char(buffer, x, row * 8, *cursor, 0xffff);
        x += 6;
    }
}

static void lcd_write_status_zone(int y, const uint8_t *buffer)
{
    const size_t buffer_size = LCD_WIDTH * LCD_STATUS_HEIGHT * 2;
    lcd_set_window(0, y, LCD_WIDTH - 1, y + LCD_STATUS_HEIGHT - 1);
    gpio_set_level(LCD_DC_GPIO, 1);
    for (size_t offset = 0; offset < buffer_size; offset += 1024) {
        size_t length = buffer_size - offset;
        if (length > 1024) length = 1024;
        spi_transaction_t transaction = {0};
        transaction.length = length * 8;
        transaction.tx_buffer = buffer + offset;
        ESP_ERROR_CHECK(spi_device_polling_transmit(lcd_spi, &transaction));
    }
}

static void lcd_show_status(const lcd_status_t *status)
{
    static uint8_t top[LCD_WIDTH * LCD_STATUS_HEIGHT * 2];
    static uint8_t bottom[LCD_WIDTH * LCD_STATUS_HEIGHT * 2];
    char line[32];
    memset(top, 0, sizeof(top));
    memset(bottom, 0, sizeof(bottom));

    snprintf(line, sizeof(line), "STATE %s", status->state);
    lcd_status_line(top, 0, line);
    snprintf(line, sizeof(line), "ARM %d STBY %d",
             status->armed ? 1 : 0, status->stby ? 1 : 0);
    lcd_status_line(top, 1, line);
    snprintf(line, sizeof(line), "M A%+03d B%+03d D%+03d",
             status->motor_a, status->motor_b, status->motor_d);
    lcd_status_line(top, 2, line);

    if (status->ultrasonic_cm >= 0.0f) {
        int distance_x10 = (int)(status->ultrasonic_cm * 10.0f + 0.5f);
        snprintf(line, sizeof(line), "US %d.%d C%d T%d", distance_x10 / 10,
                 distance_x10 % 10, status->candidate ? 1 : 0, status->threshold);
    } else {
        snprintf(line, sizeof(line), "US -- C%d T%d",
                 status->candidate ? 1 : 0, status->threshold);
    }
    lcd_status_line(bottom, 0, line);
    snprintf(line, sizeof(line), "SEED %d V%02d Q%03d", status->seed_x,
             status->valid_rows, status->confidence);
    lcd_status_line(bottom, 1, line);
    snprintf(line, sizeof(line), "BALL %s", status->ball_phase);
    lcd_status_line(bottom, 2, line);

    lcd_write_status_zone(0, top);
    lcd_write_status_zone(CAM_DISP_Y + CAM_DISP_H, bottom);
}

/**
 * @brief 初始化 LCD：GPIO、SPI2 总线与设备、ST7735 上电序列，最后清屏为黑
 * @note  独占 SPI2_HOST，SPI 时钟 10MHz，像素格式 RGB565；MADCTL=0x08 只设 BGR、不做镜像，
 *        因此屏幕上显示的是摄像头原始画面，左右与实际相反（循迹误差另做镜像修正，
 *        见 CAM_IMAGE_MIRROR）
 */
static void lcd_init(void)
{
    ESP_LOGI(TAG, "LCD init: CS=%d SCK=%d MOSI=%d DC=%d RST=%d",
             LCD_CS_GPIO, LCD_SCK_GPIO, LCD_MOSI_GPIO, LCD_DC_GPIO, LCD_RST_GPIO);
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LCD_RST_GPIO) | (1ULL << LCD_DC_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(LCD_RST_GPIO, 1);
    gpio_set_level(LCD_DC_GPIO, 1);

    spi_bus_config_t bus = {
        .mosi_io_num = LCD_MOSI_GPIO,
        .miso_io_num = -1,
        .sclk_io_num = LCD_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev = {
        /* Match camera-claude's 10 MHz setting for the wiring check. */
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = LCD_CS_GPIO,
        .queue_size = 4,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &lcd_spi));

    gpio_set_level(LCD_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(150));
    lcd_cmd(0x11); vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t d[8];
    lcd_cmd(0xB1); d[0]=0x01; d[1]=0x2C; d[2]=0x2D; lcd_write_bytes(1, d, 3);
    lcd_cmd(0xB2); d[0]=0x01; d[1]=0x2C; d[2]=0x2D; lcd_write_bytes(1, d, 3);
    lcd_cmd(0xB3); d[0]=0x01; d[1]=0x2C; d[2]=0x2D;
                  d[3]=0x01; d[4]=0x2C; d[5]=0x2D; lcd_write_bytes(1, d, 6);
    lcd_cmd(0xB4); d[0]=0x07; lcd_write_bytes(1, d, 1);
    lcd_cmd(0xC0); d[0]=0xA2; d[1]=0x02; d[2]=0x84; lcd_write_bytes(1, d, 3);
    lcd_cmd(0xC1); d[0]=0xC5; lcd_write_bytes(1, d, 1);
    lcd_cmd(0xC2); d[0]=0x0A; d[1]=0x00; lcd_write_bytes(1, d, 2);
    lcd_cmd(0xC3); d[0]=0x8A; d[1]=0x2A; lcd_write_bytes(1, d, 2);
    lcd_cmd(0xC4); d[0]=0x8A; d[1]=0xEE; lcd_write_bytes(1, d, 2);
    lcd_cmd(0xC5); d[0]=0x0E; lcd_write_bytes(1, d, 1);
    lcd_cmd(0x20);
    lcd_cmd(0x36); d[0]=0x08; lcd_write_bytes(1, d, 1);
    lcd_cmd(0x3A); d[0]=0x05; lcd_write_bytes(1, d, 1);
    lcd_cmd(0x13); vTaskDelay(pdMS_TO_TICKS(20));
    lcd_cmd(0x29); vTaskDelay(pdMS_TO_TICKS(100));

    lcd_fill(0x0000);
    ESP_LOGI(TAG, "LCD init commands and black frame sent (no LCD readback)");
}

// ==================== 超声波 ====================
/**
 * @brief 初始化超声波模块引脚：TRIG 为输出（初始低电平），ECHO 为输入（带下拉）
 */
static void ultrasonic_init(void)
{
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << TRIG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_cfg));
    gpio_set_level(TRIG_GPIO, 0);

    gpio_config_t echo_cfg = {
        .pin_bit_mask = (1ULL << ECHO_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_cfg));
}

/**
 * @brief 触发一次测距并返回前方距离
 * @return >0   距离（cm）
 * @return -1.0 超时：等不到回波上升沿，或回波持续为高超过 MAX_RANGE_US(60ms)
 * @return -2.0 测距前 ECHO 就已是高电平，通常是接线或模块异常
 * @note  两个等待阶段都是忙等（不让出 CPU），最坏阻塞约 120ms，
 *        所以避障任务的优先级必须低于显示任务，否则会拖慢画面刷新
 */
static float measure_distance_cm(void)
{
    // 空闲时回波线应为低电平；若一直为高说明接线/模块异常
    if (gpio_get_level(ECHO_GPIO) != 0) {
        lcd_ultrasonic_cm = -2.0f;
        return -2.0f;
    }

    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(20);
    gpio_set_level(TRIG_GPIO, 0);

    // 等待回波变高（没有脉冲 -> -1，说明模块没响应/没触发）
    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - wait_start > MAX_RANGE_US) {
            lcd_ultrasonic_cm = -1.0f;
            return -1.0f;
        }
    }
    // 回波变高，等待变低，计算持续时间
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - echo_start > MAX_RANGE_US) {
            lcd_ultrasonic_cm = -1.0f;
            return -1.0f;
        }
    }
    int64_t duration_us = esp_timer_get_time() - echo_start;
    if (duration_us <= 0) {
        lcd_ultrasonic_cm = -1.0f;
        return -1.0f;
    }
    float distance_cm = duration_us * SOUND_SPEED_CM_PER_US / 2.0f;
    lcd_ultrasonic_cm = distance_cm;
    return distance_cm;
}

// ==================== 三个电机接线（麦克纳姆轮） ====================
// 引脚见 board_pins.h：M1=本车左前D，M2=后B，M3=右前A。

#define PWM_FREQ_HZ    1000
#define MAX_DUTY       1023      // 10bit 占空比满量程
#define MIN_WHEEL_DUTY 0.08f     // 轮子只要被命令转动，占空比不低于此值（防低占空比卡死）

typedef struct {
    gpio_num_t enc_a, enc_b, ina, inb, pwm;
} motor_t;

// 电机顺序：M1=左前(+60°)、M2=后轮(180°)、M3=右前(-60°)
// 保留原组方向系数；它是原组的实测值，本车上须架空检查各轮转向。
// 直行解算为 M1 负、M3 正；乘下列系数后，本车 D/A 两路都是 IN1=0、IN2=1。
static const int motor_dir[3] = { 1, 1, -1 };
static const motor_t motors[] = {
    { .enc_a = M1_ENC_A, .enc_b = M1_ENC_B, .ina = M1_INA, .inb = M1_INB, .pwm = M1_PWM },
    { .enc_a = M2_ENC_A, .enc_b = M2_ENC_B, .ina = M2_INA, .inb = M2_INB, .pwm = M2_PWM },
    { .enc_a = M3_ENC_A, .enc_b = M3_ENC_B, .ina = M3_INA, .inb = M3_INB, .pwm = M3_PWM },
};
#define MOTOR_COUNT (sizeof(motors) / sizeof(motors[0]))
static volatile int lcd_motor_percent[3];

/**
 * @brief 初始化三个电机：编码器引脚配为输入、方向引脚配为输出，每路 PWM 建一个 LEDC 通道
 * @note  LEDC 低速模式 + TIMER_0，10bit 分辨率、频率 PWM_FREQ_HZ；
 *        LEDC 通道号与电机下标一一对应（0/1/2），set_motor_speed() 直接用下标当通道号。
 *        编码器引脚只做了输入配置，当前代码并未读取脉冲计数（无速度闭环）
 */
static void motor_init(void)
{
    // 本车 STBY 接 GPIO8；先保持待机，方向和 PWM 初始化后才使能。
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY_GPIO, 0));
    uint64_t enc_mask = 0, dir_mask = (1ULL << MOTOR_STBY_GPIO);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        enc_mask |= (1ULL << motors[i].enc_a) | (1ULL << motors[i].enc_b);
        dir_mask |= (1ULL << motors[i].ina) | (1ULL << motors[i].inb);
    }
    gpio_config_t in_cfg = {
        .pin_bit_mask = enc_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in_cfg));
    gpio_config_t out_cfg = {
        .pin_bit_mask = dir_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ESP_ERROR_CHECK(gpio_set_level(motors[i].ina, 0));
        ESP_ERROR_CHECK(gpio_set_level(motors[i].inb, 0));
    }

    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = motors[i].pwm,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY_GPIO, 1));
}

/**
 * @brief 设置单个电机的转向与占空比（所有运动最终都经过这里）
 * @param idx   电机下标：0=M1 左前(+60°)、1=M2 后轮(180°)、2=M3 右前(-60°)
 * @param speed -1.0~1.0，正=正转、负=反转；|speed|<0.001 视为停止
 * @note  先乘 motor_dir[idx] 修正接线极性；停止时两个方向脚同时拉低 —— 是自由滑行而非刹车。
 *        非零时占空比不低于 MIN_WHEEL_DUTY，避免占空比太小电机只嗡嗡响却不转
 */
static void set_motor_speed(int idx, float speed)
{
    lcd_motor_percent[idx] = (int)lroundf(speed * 100.0f);
    float eff = motor_dir[idx] * speed;
    if (eff > -0.001f && eff < 0.001f) {
        gpio_set_level(motors[idx].ina, 0);
        gpio_set_level(motors[idx].inb, 0);
    } else if (eff >= 0) {
        gpio_set_level(motors[idx].ina, 1);
        gpio_set_level(motors[idx].inb, 0);
    } else {
        gpio_set_level(motors[idx].ina, 0);
        gpio_set_level(motors[idx].inb, 1);
    }
    uint32_t duty = (uint32_t)((eff < 0 ? -eff : eff) * MAX_DUTY);
    if (duty > MAX_DUTY) duty = MAX_DUTY;
    if (eff > 0.001f || eff < -0.001f) {
        uint32_t min_duty = (uint32_t)(MIN_WHEEL_DUTY * MAX_DUTY);
        if (duty < min_duty) duty = min_duty;   // 要转就必须给够启动动力
    }
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)idx));
}

static TaskHandle_t line_follow_task_handle = NULL;

static void endpoint_ball_drive(float a, float b, float d)
{
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY_GPIO, 1));
    set_motor_speed(0, -d);
    set_motor_speed(1, b);
    set_motor_speed(2, -a);
}

static void endpoint_ball_stop(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) set_motor_speed(i, 0.0f);
    ESP_ERROR_CHECK(gpio_set_level(MOTOR_STBY_GPIO, 0));
}

static void endpoint_ball_pause_controllers(void)
{
    if (line_follow_task_handle != NULL) vTaskSuspend(line_follow_task_handle);
    endpoint_ball_stop();
}

// ==================== 摄像头循迹（动态检测窗 + 固定转向基准） ====================
// 检测窗：以上一帧线的位置为中心水平移动（中心被钳制在画面中部），
//         帮助大弯/突然偏移时更快锁定，同时减少其它区域干扰；
// 转向误差：始终以固定的“车头参考点”为基准（不随窗口移动），
//         否则线永远在窗口中心、误差恒为 0 就不会转向
#define BOX_Y0_FRAC             0.79f   // 扫描带上边界（相对高度）
#define BOX_Y1_FRAC             0.94f   // 扫描带下边界（相对高度）
#define TRACK_WIN_W_FRAC        0.50f   // 动态检测窗宽度（相对画面宽）
#define TRACK_CENTER_MIN_FRAC   0.30f   // 窗口中心允许的最小 x
#define TRACK_CENTER_MAX_FRAC   0.70f   // 窗口中心允许的最大 x
#define STEER_REF_FRAC          0.53f   // 固定转向基准点 x（车头参考）
#define ERR_SCALE_FRAC          0.30f   // 误差归一化尺度（约等于原框半宽）
#define RECOVERY_X_MARGIN       0.12f   // 主窗左右扩展的“找回带”宽度
#define LINE_LUMA_MAX_CAP       170     // 自适应阈值上限（防过曝误判）
#define ALL_BLACK_RATIO         0.90f   // 黑像素占比达到此值视为“全黑停车”
#define LINE_LOST_RATIO         0.01f   // 框内黑像素占比低于此值视为丢线
#define CAM_IMAGE_MIRROR        1       // 1=画面水平翻转（实测左右反了）
#define LINE_MIN_BLACK_PIXELS   4       // 框内至少多少个黑像素才算找到线
#define BASE_SPEED              0.28f   // 循迹前进速度（0~1）
#define CTRL_PERIOD_MS          50      // 控制周期

static volatile int vision_ready = 0;        // 处理过第一帧后才允许出车
static volatile float line_err = 0.0f;       // -1~1：负=线偏左，正=线偏右
static volatile int line_found = 0;          // 1=找到黑线
static volatile float line_black_ratio = 0.0f;  // 扫描区内黑像素占比
static volatile float line_cx = -1.0f;       // 加权线中心（图像 x）
static volatile float line_angle_deg = 0.0f; // 偏转角（度），OpenMV 风格
static volatile float line_row_top_img = -1.0f;    // 黑线簇顶部行（图像坐标，用于描线）
static volatile float line_row_bottom_img = -1.0f; // 黑线簇底部行（图像坐标，用于描线）
static volatile uint32_t line_luma_thr = 150;      // 本帧判定黑线用的亮度阈值
static float track_cx = -1.0f;                     // 上一帧线位置（原始坐标，用于生成检测窗）
static volatile float track_win_x0f = 0.30f;       // 当前检测窗左边界（相对宽度，供画框）
static volatile float track_win_x1f = 0.70f;       // 当前检测窗右边界（相对宽度，供画框）

/**
 * @brief RGB565 大端像素 -> 灰度亮度
 * @param p 指向 2 字节像素（p[0]=RRRRRGGG，p[1]=GGGBBBBB）
 * @return 0~255 的亮度（BT.601 加权 0.299R+0.587G+0.114B）
 * @note  5/6bit 分量直接左移补零、未做低位复制，所以纯白算出来约 250 而不是 255，
 *        设定亮度阈值时按这个量程理解
 */
static uint32_t pixel_luma(const uint8_t *p)
{
    uint8_t r = p[0] & 0xF8;
    uint8_t g = ((p[0] & 0x07) << 5) | ((p[1] & 0xE0) >> 3);
    uint8_t b = (p[1] & 0x1F) << 3;
    return (r * 299u + g * 587u + b * 114u) / 1000u;
}

// ==================== 小球检测接口 ====================
#if 0  // 本工程只复现巡线避障；原组找球代码保留作参考，不参与编译和运行。
// 检测结果只描述目标，不直接驱动车体；后续接近/推球任务可读取这些量。
typedef enum {
    BALL_NONE = 0,
    BALL_COLORED,
    BALL_GOAL,
} ball_type_t;

typedef struct {
    ball_type_t type;
    float cx;
    float cy;
    float radius;
    float confidence;
} ball_target_t;

static volatile ball_target_t ball_targets[2] = {
    { .type = BALL_COLORED, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f },
    { .type = BALL_COLORED, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f },
};
static volatile ball_target_t goal_target = {
    .type = BALL_NONE, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f
};

static void rgb565_components(const uint8_t *p, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = p[0] & 0xF8;
    *g = (uint8_t)(((p[0] & 0x07) << 5) | ((p[1] & 0xE0) >> 3));
    *b = (p[1] & 0x1F) << 3;
}

static uint8_t pixel_saturation(const uint8_t *p)
{
    uint8_t r, g, b;
    rgb565_components(p, &r, &g, &b);
    uint8_t maxc = r > g ? (r > b ? r : b) : (g > b ? g : b);
    uint8_t minc = r < g ? (r < b ? r : b) : (g < b ? g : b);
    return maxc - minc;
}

/**
 * @brief 在摄像头图像中搜索两个颜色类别的小球。
 *
 * 当前允许白球，使用圆周边缘为主、颜色为辅的筛选方式，排除明显黑色赛道。
 * 结果写入两个小球目标；当小球进入画面顶部 20% 时，同时搜索远处黑色球门。
 * approach_ball_task / push_ball_task 使用。
 */
#if 0
static void detect_ball_from_rgb565_legacy(const uint8_t *buf, uint32_t w, uint32_t h)
{
    ball_target_t best[2] = {
        { .type = BALL_COLORED, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f },
        { .type = BALL_COLORED, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f },
    };
    if (!buf || w < 40 || h < 40) {
        ball_targets[0] = best[0];
        ball_targets[1] = best[1];
        return;
    }

    // 只搜索画面高度 15% 到 80%，排除远处顶部背景和过近底部干扰。
    int y_min = (int)(h * 0.15f);
    int y_max = (int)(h * 0.80f);
    const int step = 6;
    const int ring_samples = 16;
    const float pi = 3.14159265f;

    for (int cy = y_min; cy < y_max; cy += step) {
        for (int cx = 12; cx < (int)w - 12; cx += step) {
            for (int radius = 8; radius <= 38; radius += 4) {
                if (cx - radius - 5 < 0 || cx + radius + 5 >= (int)w ||
                    cy - radius - 5 < 0 || cy + radius + 5 >= (int)h) continue;

                float inside_luma = 0.0f;
                float inside_sat = 0.0f;
                float ring_luma = 0.0f;
                float outer_luma = 0.0f;
                int ring_dark = 0;
                int ring_color = 0;

                for (int i = 0; i < ring_samples; i++) {
                    float a = 2.0f * pi * (float)i / (float)ring_samples;
                    int ix = cx + (int)(cosf(a) * radius * 0.45f);
                    int iy = cy + (int)(sinf(a) * radius * 0.45f);
                    int rx = cx + (int)(cosf(a) * radius);
                    int ry = cy + (int)(sinf(a) * radius);
                    int ox = cx + (int)(cosf(a) * (radius + 5));
                    int oy = cy + (int)(sinf(a) * (radius + 5));
                    const uint8_t *inside = buf + ((size_t)iy * w + ix) * 2;
                    const uint8_t *ring = buf + ((size_t)ry * w + rx) * 2;
                    const uint8_t *outer = buf + ((size_t)oy * w + ox) * 2;
                    inside_luma += (float)pixel_luma(inside);
                    inside_sat += (float)pixel_saturation(inside);
                    ring_luma += (float)pixel_luma(ring);
                    outer_luma += (float)pixel_luma(outer);
                    if (pixel_luma(ring) + 18 < pixel_luma(inside)) ring_dark++;
                    if (pixel_saturation(inside) > pixel_saturation(outer) + 18 ||
                        pixel_luma(inside) + 25 < pixel_luma(outer)) ring_color++;
                }

                inside_luma /= ring_samples;
                inside_sat /= ring_samples;
                ring_luma /= ring_samples;
                outer_luma /= ring_samples;
                float edge = fabsf(inside_luma - ring_luma) / 255.0f;
                float background_edge = fabsf(inside_luma - outer_luma) / 255.0f;
                // 边缘是主判据；颜色只作为辅助，白球因此可以被保留。
                float color_score = (inside_sat > 42.0f ? (inside_sat - 42.0f) / 100.0f : 0.0f);
                float not_black_score = inside_luma > 35.0f ? (inside_luma - 35.0f) / 120.0f : 0.0f;
                float dark_edge_score = (float)ring_dark / ring_samples;
                float color_edge_score = (float)ring_color / ring_samples;
                if (color_score > 1.0f) color_score = 1.0f;
                if (not_black_score > 1.0f) not_black_score = 1.0f;
                float bright_score = inside_luma > 150.0f ?
                                     (inside_luma - 150.0f) / 100.0f : 0.0f;
                if (bright_score > 1.0f) bright_score = 1.0f;
                float score = 0.48f * edge + 0.20f * color_edge_score +
                              0.16f * color_score + 0.10f * bright_score +
                              0.06f * not_black_score;
                if (inside_luma <= 35.0f || score < 0.34f) continue;

                int slot = -1;
                if (score > best[0].confidence) slot = 0;
                else if (score > best[1].confidence) slot = 1;
                if (slot < 0) continue;
                // 不保存同一个球的相邻半径候选，优先保留两个不同位置的球。
                for (int existing = 0; existing < 2; existing++) {
                    if (best[existing].confidence > 0.0f) {
                        float dx = (float)cx - best[existing].cx;
                        float dy = (float)cy - best[existing].cy;
                        if (dx * dx + dy * dy < (float)(radius * radius)) {
                            slot = -1;
                            break;
                        }
                    }
                }
                if (slot < 0) continue;
                best[slot] = (ball_target_t){
                    .type = BALL_COLORED, .cx = (float)cx, .cy = (float)cy,
                    .radius = (float)radius, .confidence = score
                };
            }
        }
    }
    if (best[1].confidence > best[0].confidence) {
        ball_target_t temp = best[0];
        best[0] = best[1];
        best[1] = temp;
    }
    ball_targets[0] = best[0];
    ball_targets[1] = best[1];

    ball_target_t goal = {
        .type = BALL_NONE, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f
    };
    int goal_y_max = (int)(h * 0.20f);
    for (int cy = 8; cy < goal_y_max; cy += 6) {
        for (int cx = 12; cx < (int)w - 12; cx += 6) {
            for (int radius = 10; radius <= 38; radius += 4) {
                if (cy - radius - 4 < 0 || cy + radius + 4 >= (int)h) continue;
                float dark_sum = 0.0f;
                float outer_sum = 0.0f;
                int edge_count = 0;
                for (int i = 0; i < ring_samples; i++) {
                    float a = 2.0f * pi * (float)i / (float)ring_samples;
                    int ix = cx + (int)(cosf(a) * radius * 0.45f);
                    int iy = cy + (int)(sinf(a) * radius * 0.45f);
                    int ox = cx + (int)(cosf(a) * (radius + 4));
                    int oy = cy + (int)(sinf(a) * (radius + 4));
                    const uint8_t *inside = buf + ((size_t)iy * w + ix) * 2;
                    const uint8_t *outer = buf + ((size_t)oy * w + ox) * 2;
                    uint32_t inner_luma = pixel_luma(inside);
                    uint32_t outer_luma = pixel_luma(outer);
                    dark_sum += (float)inner_luma;
                    outer_sum += (float)outer_luma;
                    if (outer_luma > inner_luma + 20) edge_count++;
                }
                float dark_luma = dark_sum / ring_samples;
                float outer_luma = outer_sum / ring_samples;
                float dark_score = dark_luma < 75.0f ? (75.0f - dark_luma) / 75.0f : 0.0f;
                float contrast_score = outer_luma > dark_luma ?
                                       (outer_luma - dark_luma) / 180.0f : 0.0f;
                float edge_score = (float)edge_count / ring_samples;
                float score = 0.40f * edge_score + 0.35f * dark_score + 0.25f * contrast_score;
                if (score > goal.confidence && score >= 0.45f) {
                    goal = (ball_target_t){
                        .type = BALL_GOAL, .cx = (float)cx, .cy = (float)cy,
                        .radius = (float)radius, .confidence = score
                    };
                }
            }
        }
    }
    goal_target = goal;
}
#endif

// 参考 1.c：弱边缘也参与投票，不用颜色筛掉白球。
#define BALL_HOUGH_ROI_TOP       0.20f
#define BALL_HOUGH_ROI_BOTTOM    1.00f
#define BALL_HOUGH_GRAD_THRESH   18
#define BALL_HOUGH_RMIN          5
#define BALL_HOUGH_RMAX          42       // 原 60 的 70%，限制最大识别半径
#define BALL_HOUGH_RSTEP         2
#define BALL_HOUGH_VOTE_MIN      20
#define BALL_TARGET_CONF_MIN     ((float)BALL_HOUGH_VOTE_MIN / 80.0f)
#define BALL_HOUGH_EDGE_MAX      8000
#define BALL_HOUGH_EVERY_N       2

static uint8_t *ball_gray = NULL;
static int16_t *ball_edge_x = NULL, *ball_edge_y = NULL;
static int16_t *ball_edge_gx = NULL, *ball_edge_gy = NULL;
static uint16_t *ball_acc = NULL;
static uint32_t *ball_dist_acc = NULL;
static size_t ball_hough_cap = 0;
static int ball_hough_frame = 0;

static bool ensure_ball_hough_buffers(size_t npix)
{
    if (npix <= ball_hough_cap && ball_gray && ball_acc && ball_dist_acc &&
        ball_edge_x && ball_edge_y && ball_edge_gx && ball_edge_gy) return true;
    if (ball_gray) { heap_caps_free(ball_gray); ball_gray = NULL; }
    if (ball_acc) { heap_caps_free(ball_acc); ball_acc = NULL; }
    if (ball_dist_acc) { heap_caps_free(ball_dist_acc); ball_dist_acc = NULL; }
    if (ball_edge_x) { heap_caps_free(ball_edge_x); ball_edge_x = NULL; }
    if (ball_edge_y) { heap_caps_free(ball_edge_y); ball_edge_y = NULL; }
    if (ball_edge_gx) { heap_caps_free(ball_edge_gx); ball_edge_gx = NULL; }
    if (ball_edge_gy) { heap_caps_free(ball_edge_gy); ball_edge_gy = NULL; }
    ball_hough_cap = 0;
    ball_gray = heap_caps_malloc(npix, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ball_acc = heap_caps_malloc(npix * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ball_dist_acc = heap_caps_malloc(npix * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ball_edge_x = heap_caps_malloc(BALL_HOUGH_EDGE_MAX * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ball_edge_y = heap_caps_malloc(BALL_HOUGH_EDGE_MAX * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ball_edge_gx = heap_caps_malloc(BALL_HOUGH_EDGE_MAX * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ball_edge_gy = heap_caps_malloc(BALL_HOUGH_EDGE_MAX * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ball_gray || !ball_acc || !ball_dist_acc || !ball_edge_x || !ball_edge_y ||
        !ball_edge_gx || !ball_edge_gy) return false;
    ball_hough_cap = npix;
    return true;
}

static uint32_t ball_hough_edge(const uint8_t *gray, uint32_t w, int x, int y)
{
    const uint8_t *r0 = gray + (size_t)(y - 1) * w;
    const uint8_t *r1 = gray + (size_t)y * w;
    const uint8_t *r2 = gray + (size_t)(y + 1) * w;
    int gx = (int)r1[x + 1] - (int)r1[x - 1];
    int gy = (int)r2[x] - (int)r0[x];
    return (uint32_t)abs(gx) + (uint32_t)abs(gy);
}

static void detect_ball_from_rgb565(const uint8_t *buf, uint32_t w, uint32_t h)
{
    if (!buf || w < 48 || h < 48) return;
    if (++ball_hough_frame % BALL_HOUGH_EVERY_N != 0) return;
    ball_targets[0] = (ball_target_t){ .type = BALL_COLORED, .cx = -1.0f, .cy = -1.0f };
    ball_targets[1] = (ball_target_t){ .type = BALL_COLORED, .cx = -1.0f, .cy = -1.0f };
    goal_target = (ball_target_t){ .type = BALL_NONE, .cx = -1.0f, .cy = -1.0f };
    if (!ensure_ball_hough_buffers((size_t)w * h)) return;

    ball_target_t best = {
        .type = BALL_COLORED, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f
    };
    int y0 = (int)(h * BALL_HOUGH_ROI_TOP);
    int y1 = (int)(h * BALL_HOUGH_ROI_BOTTOM);
    if (y0 < 1) y0 = 1;
    if (y1 > (int)h - 1) y1 = (int)h - 1;

    for (int y = y1; y >= y0 - 1; y--) {
        const uint8_t *row = buf + (size_t)y * w * 2;
        uint8_t *gray = ball_gray + (size_t)y * w;
        for (uint32_t x = 0; x < w; x++) gray[x] = (uint8_t)pixel_luma(row + x * 2);
    }

    int edge_count = 0;
    for (int y = y1 - 1; y >= y0 && edge_count < BALL_HOUGH_EDGE_MAX; y--) {
        for (int x = 1; x < (int)w - 1 && edge_count < BALL_HOUGH_EDGE_MAX; x++) {
            int gx = (int)ball_gray[(size_t)y * w + x + 1] - (int)ball_gray[(size_t)y * w + x - 1];
            int gy = (int)ball_gray[(size_t)(y + 1) * w + x] - (int)ball_gray[(size_t)(y - 1) * w + x];
            if (abs(gx) + abs(gy) < BALL_HOUGH_GRAD_THRESH) continue;
            ball_edge_x[edge_count] = (int16_t)x;
            ball_edge_y[edge_count] = (int16_t)y;
            ball_edge_gx[edge_count] = (int16_t)gx;
            ball_edge_gy[edge_count++] = (int16_t)gy;
        }
    }

    memset(ball_acc, 0, (size_t)w * h * sizeof(uint16_t));
    memset(ball_dist_acc, 0, (size_t)w * h * sizeof(uint32_t));
    for (int e = 0; e < edge_count; e++) {
        int gx = ball_edge_gx[e], gy = ball_edge_gy[e];
        int mag = abs(gx) + abs(gy);
        if (mag == 0) continue;
        int32_t ux = ((int32_t)gx << 8) / mag;
        int32_t uy = ((int32_t)gy << 8) / mag;
        for (int sign = -1; sign <= 1; sign += 2) {
            for (int r = BALL_HOUGH_RMIN; r <= BALL_HOUGH_RMAX; r += BALL_HOUGH_RSTEP) {
                int cx = ball_edge_x[e] - (int)((sign * ux * r) >> 8);
                int cy = ball_edge_y[e] - (int)((sign * uy * r) >> 8);
                if (cx < 0 || cx >= (int)w || cy < y0 || cy >= y1) continue;
                size_t index = (size_t)cy * w + cx;
                if (ball_acc[index] < UINT16_MAX) ball_acc[index]++;
                ball_dist_acc[index] += (uint32_t)r;
            }
        }
    }

    {
        int best_vote = 0, best_x = -1, best_y = -1, best_r = 0;
        for (int y = y1 - 1; y >= y0; y--) {
            for (int x = 0; x < (int)w; x++) {
                size_t index = (size_t)y * w + x;
                int vote = ball_acc[index];
                if (vote > best_vote) {
                    best_vote = vote;
                    best_x = x;
                    best_y = y;
                    best_r = vote ? (int)(ball_dist_acc[index] / (uint32_t)vote) : 0;
                }
            }
        }
        if (best_vote >= BALL_HOUGH_VOTE_MIN && best_x >= 0) {
            best = (ball_target_t){
                .type = BALL_COLORED, .cx = (float)best_x, .cy = (float)best_y,
                .radius = (float)best_r, .confidence = (float)best_vote / 80.0f
            };
            if (best.confidence > 1.0f) best.confidence = 1.0f;
        }
    }

    // 近处阶段（80%~100%）搜索黑色球门块：黑色内部、较亮外部和明显边界。
    ball_target_t goal = { .type = BALL_NONE, .cx = -1.0f, .cy = -1.0f,
                           .radius = 0.0f, .confidence = 0.0f };
    int goal_y0 = (int)(h * 0.80f);
    for (int cy = goal_y0 + 8; cy < (int)h - 8; cy += 6) {
        for (int cx = 12; cx < (int)w - 12; cx += 6) {
            for (int radius = 8; radius <= 60; radius += 4) {
                if (cy - radius < goal_y0 || cy + radius >= (int)h ||
                    cx - radius < 0 || cx + radius >= (int)w) continue;
                int dark_count = 0;
                int contrast_count = 0;
                int samples = 16;
                int dark_sum = 0;
                for (int i = 0; i < samples; i++) {
                    float a = 2.0f * 3.14159265f * (float)i / samples;
                    int ix = cx + (int)(cosf(a) * radius * 0.45f);
                    int iy = cy + (int)(sinf(a) * radius * 0.45f);
                    int ox = cx + (int)(cosf(a) * radius);
                    int oy = cy + (int)(sinf(a) * radius);
                    int inner = (int)pixel_luma(buf + ((size_t)iy * w + ix) * 2);
                    int outer = (int)pixel_luma(buf + ((size_t)oy * w + ox) * 2);
                    dark_sum += inner;
                    if (inner < 80) dark_count++;
                    if (outer > inner + 20) contrast_count++;
                }
                float dark_score = (float)dark_count / samples;
                float contrast_score = (float)contrast_count / samples;
                float score = 0.60f * dark_score + 0.40f * contrast_score;
                if (score > goal.confidence && score >= 0.55f) {
                    goal = (ball_target_t){ .type = BALL_GOAL, .cx = (float)cx,
                        .cy = (float)cy, .radius = (float)radius, .confidence = score };
                }
                (void)dark_sum;
            }
        }
    }
    goal_target = goal;
    ball_targets[0] = best;
    ball_targets[1] = (ball_target_t){
        .type = BALL_COLORED, .cx = -1.0f, .cy = -1.0f,
        .radius = 0.0f, .confidence = 0.0f
    };
}

#if 0
static uint32_t ball_sobel_magnitude(const uint8_t *buf, uint32_t w, uint32_t h, int x, int y)
{
    if (x <= 0 || y <= 0 || x >= (int)w - 1 || y >= (int)h - 1) return 0;
    const uint8_t *p00 = buf + ((size_t)(y - 1) * w + (x - 1)) * 2;
    const uint8_t *p01 = buf + ((size_t)(y - 1) * w + x) * 2;
    const uint8_t *p02 = buf + ((size_t)(y - 1) * w + (x + 1)) * 2;
    const uint8_t *p10 = buf + ((size_t)y * w + (x - 1)) * 2;
    const uint8_t *p12 = buf + ((size_t)y * w + (x + 1)) * 2;
    const uint8_t *p20 = buf + ((size_t)(y + 1) * w + (x - 1)) * 2;
    const uint8_t *p21 = buf + ((size_t)(y + 1) * w + x) * 2;
    const uint8_t *p22 = buf + ((size_t)(y + 1) * w + (x + 1)) * 2;
    int gx = (int)pixel_luma(p02) + 2 * (int)pixel_luma(p12) + (int)pixel_luma(p22)
           - (int)pixel_luma(p00) - 2 * (int)pixel_luma(p10) - (int)pixel_luma(p20);
    int gy = (int)pixel_luma(p20) + 2 * (int)pixel_luma(p21) + (int)pixel_luma(p22)
           - (int)pixel_luma(p00) - 2 * (int)pixel_luma(p01) - (int)pixel_luma(p02);
    return (uint32_t)(abs(gx) + abs(gy));
}

static void detect_ball_from_rgb565(const uint8_t *buf, uint32_t w, uint32_t h)
{
    ball_target_t best_white = {
        .type = BALL_WHITE, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f
    };
    ball_target_t best_non_white = {
        .type = BALL_NON_WHITE, .cx = -1.0f, .cy = -1.0f, .radius = 0.0f, .confidence = 0.0f
    };
    if (!buf || w < 48 || h < 48) {
        ball_targets[0] = best_white;
        ball_targets[1] = best_non_white;
        return;
    }

    // Hough-style search: each candidate circle receives votes from Sobel edge pixels
    // sampled around its circumference. The top 15% is excluded; the lower 90% is used.
    const int center_step = 6;
    const int radius_step = 4;
    const int ring_samples = 24;
    const float pi = 3.14159265f;
    int y_min = (int)(h * 0.15f);
    int y_max = (int)(h * 0.90f);
    for (int cy = y_min; cy < y_max; cy += center_step) {
        for (int cx = 12; cx < (int)w - 12; cx += center_step) {
            for (int radius = 8; radius <= 38; radius += radius_step) {
                if (cx - radius - 2 < 1 || cx + radius + 2 >= (int)w - 1 ||
                    cy - radius - 2 < 1 || cy + radius + 2 >= (int)h - 1) continue;

                uint32_t edge_sum = 0;
                int edge_votes = 0;
                float center_luma = 0.0f;
                float center_sat = 0.0f;
                for (int i = 0; i < ring_samples; i++) {
                    float a = 2.0f * pi * (float)i / (float)ring_samples;
                    int rx = cx + (int)(cosf(a) * radius);
                    int ry = cy + (int)(sinf(a) * radius);
                    uint32_t edge = ball_sobel_magnitude(buf, w, h, rx, ry);
                    edge_sum += edge;
                    if (edge >= 100) edge_votes++;
                    int ix = cx + (int)(cosf(a) * radius * 0.35f);
                    int iy = cy + (int)(sinf(a) * radius * 0.35f);
                    const uint8_t *inside = buf + ((size_t)iy * w + ix) * 2;
                    center_luma += (float)pixel_luma(inside);
                    center_sat += (float)pixel_saturation(inside);
                }

                center_luma /= ring_samples;
                center_sat /= ring_samples;
                float edge_score = (float)edge_votes / (float)ring_samples;
                float strength_score = (float)edge_sum / (float)(ring_samples * 700);
                if (strength_score > 1.0f) strength_score = 1.0f;
                float round_score = 0.65f * edge_score + 0.35f * strength_score;
                if (round_score < 0.32f) continue;

                // White balls use their edge against the white floor; colored balls use
                // the same circle votes plus center saturation/brightness difference.
                float white_score = round_score * (center_luma > 165.0f ? 1.0f : 0.35f);
                float non_white_score = round_score *
                    (center_sat > 24.0f || center_luma < 175.0f ? 1.0f : 0.30f);
                bool is_white = white_score > non_white_score && center_sat < 45.0f;
                ball_target_t *best = is_white ? &best_white : &best_non_white;
                float score = is_white ? white_score : non_white_score;
                float minimum = best == &best_white ? 0.42f : 0.38f;
                if (score < minimum || score <= best->confidence) continue;
                best->cx = (float)cx;
                best->cy = (float)cy;
                best->radius = (float)radius;
                best->confidence = score > 1.0f ? 1.0f : score;
            }
        }
    }
    ball_targets[0] = best_white;
    ball_targets[1] = best_non_white;
}
#endif

/** @brief 供后续接近小球/推球任务读取指定类型的小球目标。 */
static ball_target_t get_ball_target(ball_type_t type)
{
    if (type == BALL_COLORED) return ball_targets[0];
    if (type == BALL_GOAL) return goal_target;
    return (ball_target_t){ .type = BALL_NONE, .cx = -1.0f, .cy = -1.0f };
}

// ==================== 找球/接近小球测试任务 ====================
#define BALL_SEARCH_PERIOD_MS       80
#define BALL_APPROACH_SPEED         0.16f
#define BALL_ALIGN_SPEED            0.08f
#define BALL_ALIGN_PULSE_MS         40
#define BALL_FINE_TURN_GAIN         0.06f
#define BALL_ANGLE_LIMIT_DEG        35.0f
#define BALL_HORIZONTAL_FOV_DEG     110.0f
#define BALL_STOP_Y_FRAC            0.80f
#define BALL_STOP_X_HALF_FRAC       0.125f
#define BALL_VISION_TIMEOUT_MS      400

typedef enum {
    BALL_TASK_SEARCH = 0,
    BALL_TASK_ALIGN,
    BALL_TASK_APPROACH,
    BALL_TASK_PUSH_READY,
} ball_task_state_t;

static volatile ball_task_state_t ball_task_state = BALL_TASK_SEARCH;
static volatile ball_type_t selected_ball_type = BALL_NONE;
static volatile uint16_t vision_w = 0, vision_h = 0;
static volatile int64_t last_vision_time = 0;

static void stop_all_motors(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) set_motor_speed(i, 0.0f);
}

static ball_target_t select_nearest_ball(void)
{
    ball_target_t first = ball_targets[0];
    ball_target_t second = ball_targets[1];
    ball_target_t selected = { .type = BALL_NONE, .cx = -1.0f, .cy = -1.0f };
    if (first.type == BALL_COLORED && first.confidence >= BALL_TARGET_CONF_MIN) selected = first;
    if (second.type == BALL_COLORED && second.confidence >= BALL_TARGET_CONF_MIN &&
        (selected.type == BALL_NONE || second.radius > selected.radius)) selected = second;
    return selected;
}

static bool ball_is_in_stop_region(const ball_target_t *target, uint32_t w, uint32_t h)
{
    if (!target || target->type == BALL_NONE || w == 0 || h == 0) return false;
    float x_error = fabsf(target->cx / (float)w - 0.5f);
    float y_from_bottom = target->cy / (float)h;
    return x_error <= BALL_STOP_X_HALF_FRAC && y_from_bottom >= BALL_STOP_Y_FRAC;
}

/**
 * @brief 找球测试任务：选择较近小球，转向对准后前进，进入底部窄区域后停车。
 *
 * BALL_TASK_PUSH_READY 是后续推球入洞任务的接入口；本阶段只负责把车停到球前。
 */
static void ball_approach_task(void *arg)
{
    (void)arg;
    int align_direction = 0;
    int64_t align_end_time = 0;
    for (;;) {
        ball_target_t target = select_nearest_ball();
        int64_t now = esp_timer_get_time();
        bool vision_stale = last_vision_time == 0 ||
                            now - last_vision_time > (int64_t)BALL_VISION_TIMEOUT_MS * 1000;
        if (!vision_ready || vision_w == 0 || vision_h == 0 || vision_stale ||
            target.type == BALL_NONE) {
            ball_task_state = BALL_TASK_SEARCH;
            selected_ball_type = BALL_NONE;
            align_end_time = 0;
            align_direction = 0;
            stop_all_motors();
            vTaskDelay(pdMS_TO_TICKS(BALL_SEARCH_PERIOD_MS));
            continue;
        }

        // 近处区域定义为画面80%~100%。此时停止追小球，切换为球门目标。
        ball_target_t goal = get_ball_target(BALL_GOAL);
        bool goal_mode = target.cy / (float)vision_h >= 0.80f;
        if (goal_mode) {
            if (goal.type != BALL_GOAL || goal.confidence < 0.55f) {
                ball_task_state = BALL_TASK_SEARCH;
                selected_ball_type = target.type;
                align_end_time = 0;
                align_direction = 0;
                stop_all_motors();
                vTaskDelay(pdMS_TO_TICKS(BALL_SEARCH_PERIOD_MS));
                continue;
            }
            target = goal;
        }

        selected_ball_type = target.type;
        float x_error = target.cx / (float)vision_w - 0.5f;
        if (CAM_IMAGE_MIRROR) x_error = -x_error;
        float angle_deg = x_error * BALL_HORIZONTAL_FOV_DEG;

        if (align_end_time > 0 && now < align_end_time) {
            // 一次修正脉冲期间保持固定方向，不使用中途的旧视觉结果反复改向。
            float omega = align_direction * BALL_ALIGN_SPEED;
            ball_task_state = BALL_TASK_ALIGN;
            set_motor_speed(0, omega);
            set_motor_speed(1, omega);
            set_motor_speed(2, omega);
        } else if (align_end_time > 0) {
            // 40 ms 到达后先停车；下一轮循环重新读取球心，确认偏角后再决定。
            align_end_time = 0;
            align_direction = 0;
            ball_task_state = BALL_TASK_SEARCH;
            stop_all_motors();
        } else if (!goal_mode && ball_is_in_stop_region(&target, vision_w, vision_h)) {
            ball_task_state = BALL_TASK_PUSH_READY;
            stop_all_motors();
        } else if (fabsf(angle_deg) > BALL_ANGLE_LIMIT_DEG) {
            ball_task_state = BALL_TASK_ALIGN;
            align_direction = x_error > 0.0f ? 1 : -1;
            align_end_time = now + (int64_t)BALL_ALIGN_PULSE_MS * 1000;
            float omega = align_direction * BALL_ALIGN_SPEED;
            set_motor_speed(0, omega);
            set_motor_speed(1, omega);
            set_motor_speed(2, omega);
        } else {
            ball_task_state = BALL_TASK_APPROACH;
            float omega = x_error * BALL_FINE_TURN_GAIN;
            float vx = BALL_APPROACH_SPEED;
            set_motor_speed(0, -0.8660254f * vx + 1.5f * omega);
            set_motor_speed(1, 0.0f);
            set_motor_speed(2,  0.8660254f * vx + 1.5f * omega);
        }

        static int log_count = 0;
        if (++log_count >= 10) {
            log_count = 0;
            ESP_LOGI(TAG, "ball: type=%d r=%.1f conf=%.2f x=%.2f angle=%.1f state=%d",
                     (int)target.type, target.radius, target.confidence,
                     x_error, angle_deg, (int)ball_task_state);
        }
        vTaskDelay(pdMS_TO_TICKS(BALL_SEARCH_PERIOD_MS));
    }
}

#endif  // 停用原组找球检测与运动任务

/**
 * @brief 从一帧解码后的画面里找出黑线并算出转向误差 —— 循迹的“感知”环节
 *
 * @param buf 解码后的图像，w*h 个 RGB565 大端像素（swap_color_bytes=1 的输出）
 * @param w,h 图像宽、高（像素）
 * @return 无；结果全部写进下面列出的全局量，供 line_follow_task 读取
 *
 * 处理步骤：
 *  1. 扫描带：只看 y ∈ [h*BOX_Y0_FRAC, h*BOX_Y1_FRAC) 这条横带，也就是画面底部最贴近
 *     车头的区域 —— 越靠下代表越近的路况，转向反应最及时；
 *  2. 动态检测窗：以上一帧的线位置 track_cx 为中心（首帧退化为 w*STEER_REF_FRAC），
 *     中心钳制在 [TRACK_CENTER_MIN_FRAC, TRACK_CENTER_MAX_FRAC]*w，
 *     宽度 w*TRACK_WIN_W_FRAC。窗口跟着线走，大弯或突然偏移时锁定更快，也少受两侧干扰；
 *  3. 自适应阈值：统计窗内亮度 min/max，thr = min + (max-min)*20%，
 *     再夹到 [30, LINE_LUMA_MAX_CAP]（上限用来防止过曝时把地面也判成黑线）；
 *  4. 把窗内亮度 < thr 的像素记为黑，累加求出 x 方向质心 cx 和黑像素占比；
 *  5. 窗内没线（黑点数 < LINE_MIN_BLACK_PIXELS 或占比 < LINE_LOST_RATIO）时，
 *     把窗口左右各外扩 w*RECOVERY_X_MARGIN 再扫一遍（“找回带”）。找回带里有线就照常输出
 *     误差，让车带着方向指引继续修正；只有找回带也没线，才判定为丢线；
 *  6. 误差 = (cx - w*STEER_REF_FRAC) / (w*ERR_SCALE_FRAC)，夹到 ±1。
 *
 * 最关键的一点：转向基准点固定在 w*STEER_REF_FRAC，不随检测窗移动。检测窗决定“往哪看”，
 * 基准点决定“往哪转”；若拿窗口中心当基准，线永远落在窗口中心、误差恒为 0，车就永远不转向。
 *
 * 写入的全局量（都是给控制/显示用的“输出接口”）：
 *  - line_found        1=本帧找到线，0=丢线（控制层据此进入盲搜）
 *  - line_err          -1~1 的转向误差，已按 CAM_IMAGE_MIRROR 翻转符号，符号以实车视角为准：
 *                      正=线在基准点右侧（该往右修），负=在左侧
 *  - line_cx           黑像素质心 x，原始图像坐标（未镜像），只用于屏幕描点
 *  - line_black_ratio  实际统计区域内的黑像素占比，≥ALL_BLACK_RATIO 时控制层判为全黑停车
 *  - line_luma_thr     本帧使用的亮度阈值，叠加显示时复用同一阈值描绿点
 *  - line_row_top_img / line_row_bottom_img  扫描带上下边界行（图像坐标，供描线）
 *  - line_angle_deg    由归一化误差反算的等效偏角，仅用于日志观察，不参与控制
 *  - track_cx          本帧线位置，作为下一帧检测窗的中心
 *  - track_win_x0f / track_win_x1f  本帧检测窗左右边界（相对宽度，供画蓝框）
 *
 * @note 判定丢线返回时 track_cx 保持上一帧的值，下一帧仍在原处附近继续找。
 *       这些全局量是各自独立的 volatile 变量、写入不加锁，控制任务偶尔会读到相邻两帧的
 *       混合结果；控制周期(50ms)远慢于帧抖动且误差另有平滑，实测可以接受
 */
static void detect_line_from_rgb565(const uint8_t *buf, uint32_t w, uint32_t h)
{
    int y0 = (int)(h * BOX_Y0_FRAC);
    int y1 = (int)(h * BOX_Y1_FRAC);
    if (y1 <= y0) y1 = y0 + 1;
    if (y1 > (int)h) y1 = h;

    // —— 动态检测窗：以上一帧线位置为中心（首帧用固定基准点），中心钳制在画面中部 ——
    float center = track_cx;
    if (center < 0.0f) center = w * STEER_REF_FRAC;
    float cmin = w * TRACK_CENTER_MIN_FRAC;
    float cmax = w * TRACK_CENTER_MAX_FRAC;
    if (center < cmin) center = cmin;
    if (center > cmax) center = cmax;

    float half_w = w * TRACK_WIN_W_FRAC / 2.0f;
    int x0 = (int)(center - half_w);
    int x1 = (int)(center + half_w);
    if (x0 < 0) x0 = 0;
    if (x1 > (int)w) x1 = w;
    if (x1 <= x0) x1 = x0 + 1;

    track_win_x0f = (float)x0 / (float)w;
    track_win_x1f = (float)x1 / (float)w;

    // 阈值：统计检测窗内的亮度范围，取“最暗~最亮”区间的 20% 处（与下面的 20u/100u 对应）
    uint32_t lmin = 255, lmax = 0;
    for (int y = y0; y < y1; y++) {
        const uint8_t *row = buf + (size_t)y * w * 2;
        for (int x = x0; x < x1; x++) {
            uint32_t luma = pixel_luma(row + x * 2);
            if (luma < lmin) lmin = luma;
            if (luma > lmax) lmax = luma;
        }
    }
    uint32_t thr = lmin + (lmax - lmin) * 20u / 100u;
    if (thr < 30) thr = 30;
    if (thr > LINE_LUMA_MAX_CAP) thr = LINE_LUMA_MAX_CAP;
    line_luma_thr = thr;

    // 转向基准固定不变（不随窗口移动）
    float fixed_ref = w * STEER_REF_FRAC;
    float err_scale = w * ERR_SCALE_FRAC;

    // 统计窗内黑像素，算质心 x（原始坐标）
    int64_t sum_x = 0;
    uint32_t black_cnt = 0, sample_cnt = 0;
    for (int y = y0; y < y1; y++) {
        const uint8_t *row = buf + (size_t)y * w * 2;
        for (int x = x0; x < x1; x++) {
            uint32_t luma = pixel_luma(row + x * 2);
            sample_cnt++;
            if (luma < thr) {
                sum_x += x;
                black_cnt++;
            }
        }
    }
    line_black_ratio = sample_cnt ? (float)black_cnt / (float)sample_cnt : 0.0f;

    // 窗内没有找到线 -> 用更宽的“找回带”扫一次（以当前窗口中心扩展）：
    // 找回带里能找到线就带方向指引继续转，避免盲目原地转
    if (black_cnt < LINE_MIN_BLACK_PIXELS || line_black_ratio < LINE_LOST_RATIO) {
        int rx0 = x0 - (int)(w * RECOVERY_X_MARGIN);
        int rx1 = x1 + (int)(w * RECOVERY_X_MARGIN);
        if (rx0 < 0) rx0 = 0;
        if (rx1 > (int)w) rx1 = w;

        int64_t rsum = 0;
        uint32_t rcnt = 0, rsamp = 0;
        for (int y = y0; y < y1; y++) {
            const uint8_t *row = buf + (size_t)y * w * 2;
            for (int x = rx0; x < rx1; x++) {
                uint32_t luma = pixel_luma(row + x * 2);
                rsamp++;
                if (luma < thr) {
                    rsum += x;
                    rcnt++;
                }
            }
        }

        if (rcnt < LINE_MIN_BLACK_PIXELS ||
            (rsamp > 0 && (float)rcnt / (float)rsamp < LINE_LOST_RATIO)) {
            line_found = 0;   // 找回带里也没有线，才进入盲搜
            return;
        }

        float rcx = (float)rsum / (float)rcnt;
        float rerr = (rcx - fixed_ref) / err_scale;
        track_cx = rcx;
        line_found = 1;
        line_cx = rcx;
        line_row_top_img = (float)y0;
        line_row_bottom_img = (float)y1 - 1;
        // 摄像头水平镜像时只翻转误差符号，固定基准点始终是判定中心
        line_err = CAM_IMAGE_MIRROR ? -rerr : rerr;
        if (line_err < -1.0f) line_err = -1.0f;
        if (line_err > 1.0f) line_err = 1.0f;
        line_angle_deg = -atanf(rerr) * 180.0f / (float)M_PI;
        line_black_ratio = rsamp ? (float)rcnt / (float)rsamp : 0.0f;
        return;
    }

    float cx = (float)sum_x / (float)black_cnt;
    track_cx = cx;

    line_found = 1;
    line_cx = cx;
    line_row_top_img = (float)y0;
    line_row_bottom_img = (float)y1 - 1;
    // 线中心偏哪边，误差就带符号：偏左为负、偏右为正，控制让它回到固定基准点
    float err_raw = (cx - fixed_ref) / err_scale;
    line_err = CAM_IMAGE_MIRROR ? -err_raw : err_raw;
    if (line_err < -1.0f) line_err = -1.0f;
    if (line_err > 1.0f) line_err = 1.0f;
    line_angle_deg = -atanf(err_raw) * 180.0f / (float)M_PI;
}

// ==================== LCD 标注识别到的黑线 ====================
// 颜色按 RGB565 大端字节序定义（与屏幕线序一致）
#define MARKER_GREEN_BE   0x07E0u   // 亮绿：描出算法认定的黑线像素
#define MARKER_YELLOW_BE  0xFFE0u   // 黄色十字：标出误差计算用的质心
#define MARKER_RED_BE     0xF800u   // 红色横线：没找到线时的提示
#define MARKER_CYAN_BE    0x07FFu   // 青色：中间/上方 ROI 框
#define MARKER_BLUE_BE    0x001Fu   // 蓝色：底部（权重最大）ROI 框

/**
 * @brief 在显示缓冲里画一个标注像素，越界自动忽略
 * @param buf      CAM_DISP_W*CAM_DISP_H 的 RGB565 大端缓冲
 * @param x,y      显示坐标（左上角为原点）
 * @param color_be RGB565 颜色，大端字节序
 */
static void draw_marker_px(uint8_t *buf, int x, int y, uint16_t color_be)
{
    if (x < 0 || x >= CAM_DISP_W || y < 0 || y >= CAM_DISP_H) return;
    uint8_t *p = buf + ((size_t)y * CAM_DISP_W + x) * 2;
    p[0] = color_be >> 8;
    p[1] = color_be & 0xFF;
}

static void overlay_ball_target(uint8_t *scaled,
                                const endpoint_ball_status_t *status)
{
    if (!status->target_valid || status->target_frame_width <= 0 ||
        status->target_frame_height <= 0) return;

    int center_x = status->target_x * CAM_DISP_W / status->target_frame_width;
    int center_y = status->target_y * CAM_DISP_H / status->target_frame_height;
    int half_width = status->target_width * CAM_DISP_W /
                     status->target_frame_width / 2 + 2;
    int half_height = status->target_height * CAM_DISP_H /
                      status->target_frame_height / 2 + 2;
    int left = center_x - half_width;
    int right = center_x + half_width;
    int top = center_y - half_height;
    int bottom = center_y + half_height;
    uint16_t color = status->target_is_red ? MARKER_RED_BE : MARKER_GREEN_BE;

    for (int x = left; x <= right; x++) {
        draw_marker_px(scaled, x, top, color);
        draw_marker_px(scaled, x, bottom, color);
    }
    for (int y = top; y <= bottom; y++) {
        draw_marker_px(scaled, left, y, color);
        draw_marker_px(scaled, right, y, color);
    }
    for (int offset = -3; offset <= 3; offset++) {
        draw_marker_px(scaled, center_x + offset, center_y, color);
        draw_marker_px(scaled, center_x, center_y + offset, color);
    }
}

/**
 * @brief 画出本帧的动态检测窗（蓝色矩形）和固定转向基准线（青色竖线）
 * @param scaled 128x96 的 RGB565 大端显示缓冲，原地修改
 * @note  矩形由 track_win_x0f/track_win_x1f 与 BOX_Y0/Y1_FRAC 决定，随线左右移动；
 *        青线是 STEER_REF_FRAC，黑线压到青线上才算居中。屏幕不做镜像，
 *        所以画面上的左右与实车左右相反，而 line_err 已经修正过镜像
 */
static void draw_roi_rects(uint8_t *scaled)
{
    int dx0 = (int)(track_win_x0f * CAM_DISP_W);
    int dx1 = (int)(track_win_x1f * CAM_DISP_W);
    int dy0 = (int)(BOX_Y0_FRAC * CAM_DISP_H);
    int dy1 = (int)(BOX_Y1_FRAC * CAM_DISP_H);
    if (dx0 < 0) dx0 = 0;
    if (dx1 >= CAM_DISP_W) dx1 = CAM_DISP_W - 1;
    if (dy0 < 0) dy0 = 0;
    if (dy1 >= CAM_DISP_H) dy1 = CAM_DISP_H - 1;

    for (int x = dx0; x <= dx1; x++) {
        draw_marker_px(scaled, x, dy0, MARKER_BLUE_BE);
        draw_marker_px(scaled, x, dy1, MARKER_BLUE_BE);
    }
    for (int y = dy0; y <= dy1; y++) {
        draw_marker_px(scaled, dx0, y, MARKER_BLUE_BE);
        draw_marker_px(scaled, dx1, y, MARKER_BLUE_BE);
    }

    // 固定转向基准点：线回到这条竖线才表示居中
    int dref = (int)(STEER_REF_FRAC * CAM_DISP_W);
    if (dref < 0) dref = 0;
    if (dref >= CAM_DISP_W) dref = CAM_DISP_W - 1;
    for (int y = 0; y < CAM_DISP_H; y++) {
        draw_marker_px(scaled, dref, y, MARKER_CYAN_BE);
    }
}

/**
 * @brief 把循迹识别结果叠加到显示缓冲：检测窗、基准线、黑线像素、质心十字
 * @param scaled      128x96 的 RGB565 大端显示缓冲，原地修改
 * @param img_w,img_h 解码图像的宽高，用来把图像坐标换算成显示坐标
 * @note  丢线时只在画面底部画一条红色横线提示，不再描点。
 *        绿色描点扫的是扫描带对应的整行像素（不限于蓝框内），窗外的深色物体也会被涂绿；
 *        调参时别把绿色当成“已进入误差计算”的依据 —— 真正参与计算的范围是蓝框
 */
static void overlay_detected_line(uint8_t *scaled, uint32_t img_w, uint32_t img_h)
{
    // 先画识别区域框
    draw_roi_rects(scaled);

    if (!line_found) {
        // 没找到黑线：画面底部画一条红色横线提示
        for (int x = 0; x < CAM_DISP_W; x++) {
            draw_marker_px(scaled, x, CAM_DISP_H - 1, MARKER_RED_BE);
        }
        return;
    }

    // 把算法认定属于黑线的像素（最近黑线簇所在行、且亮度低于阈值）描成亮绿色
    int top_dy = (int)(line_row_top_img * CAM_DISP_H / img_h);
    int bot_dy = (int)(line_row_bottom_img * CAM_DISP_H / img_h);
    if (top_dy < 0) top_dy = 0;
    if (bot_dy >= CAM_DISP_H) bot_dy = CAM_DISP_H - 1;
    uint32_t thr = line_luma_thr;

    for (int dy = top_dy; dy <= bot_dy; dy++) {
        uint8_t *row = scaled + (size_t)dy * CAM_DISP_W * 2;
        for (int dx = 0; dx < CAM_DISP_W; dx++) {
            uint8_t r = row[dx * 2] & 0xF8;
            uint8_t g = ((row[dx * 2] & 0x07) << 5) | ((row[dx * 2 + 1] & 0xE0) >> 3);
            uint8_t b = (row[dx * 2 + 1] & 0x1F) << 3;
            uint32_t luma = (r * 299u + g * 587u + b * 114u) / 1000u;
            if (luma < thr) {
                row[dx * 2] = MARKER_GREEN_BE >> 8;
                row[dx * 2 + 1] = MARKER_GREEN_BE & 0xFF;
            }
        }
    }

    // 黄色十字：误差计算用的质心位置（贴近车头的一段）
    int dxc = (int)(line_cx * CAM_DISP_W / img_w);
    int dyc = bot_dy;
    if (dxc < 0) dxc = 0;
    if (dxc >= CAM_DISP_W) dxc = CAM_DISP_W - 1;
    for (int i = -2; i <= 2; i++) {
        draw_marker_px(scaled, dxc + i, dyc, MARKER_YELLOW_BE);
        draw_marker_px(scaled, dxc, dyc + i, MARKER_YELLOW_BE);
    }
}

#if 0  // 找球叠加层停用，巡线只绘制 overlay_detected_line。
#define MARKER_BALL_FIRST_BE  0xF81Fu
#define MARKER_BALL_SECOND_BE 0x07FFu
#define MARKER_GOAL_BE        0xF800u
#define MARKER_NO_DETECT_BE   0xF800u
#define MARKER_BALL_ZONE_BE   0x07E0u
#define MARKER_GOAL_ZONE_BE   0xFFE0u

static void draw_detection_zones(uint8_t *scaled)
{
    int y_remote_end = (int)(CAM_DISP_H * 0.20f);
    int y_ball_end = (int)(CAM_DISP_H * 0.80f);
    for (int x = 0; x < CAM_DISP_W; x++) {
        draw_marker_px(scaled, x, y_remote_end, MARKER_NO_DETECT_BE);
        draw_marker_px(scaled, x, y_ball_end, MARKER_BALL_ZONE_BE);
    }
    for (int x = 0; x < CAM_DISP_W; x += 4) {
        draw_marker_px(scaled, x, 0, MARKER_NO_DETECT_BE);
        draw_marker_px(scaled, x, CAM_DISP_H - 1, MARKER_GOAL_ZONE_BE);
    }
}

/** @brief 将检测到的小球轮廓、球心和类型标记叠加到 LCD 缩略图。 */
static void overlay_detected_ball(uint8_t *scaled, uint32_t img_w, uint32_t img_h)
{
    if (img_w == 0 || img_h == 0) return;
    draw_detection_zones(scaled);
    for (int target_index = 0; target_index < 2; target_index++) {
        ball_target_t target = ball_targets[target_index];
        if (target.type == BALL_NONE || target.confidence <= 0.0f) continue;

        int cx = (int)(target.cx * CAM_DISP_W / img_w);
        int cy = (int)(target.cy * CAM_DISP_H / img_h);
        int radius = (int)(target.radius * CAM_DISP_W / img_w);
        uint16_t color = target_index == 0 ? MARKER_BALL_FIRST_BE : MARKER_BALL_SECOND_BE;
        if (radius < 3) radius = 3;

        // 24 个等角度采样点近似圆轮廓，避免引入额外图形库。
        for (int i = 0; i < 24; i++) {
            float angle = 2.0f * (float)M_PI * (float)i / 24.0f;
            draw_marker_px(scaled, cx + (int)(cosf(angle) * radius),
                           cy + (int)(sinf(angle) * radius), color);
        }
        for (int i = -3; i <= 3; i++) {
            draw_marker_px(scaled, cx + i, cy, color);
            draw_marker_px(scaled, cx, cy + i, color);
        }
    }

    ball_target_t goal = get_ball_target(BALL_GOAL);
    if (goal.type == BALL_GOAL && goal.confidence >= 0.45f) {
        int cx = (int)(goal.cx * CAM_DISP_W / img_w);
        int cy = (int)(goal.cy * CAM_DISP_H / img_h);
        int radius = (int)(goal.radius * CAM_DISP_W / img_w);
        if (radius < 3) radius = 3;
        for (int i = 0; i < 24; i++) {
            float angle = 2.0f * (float)M_PI * (float)i / 24.0f;
            draw_marker_px(scaled, cx + (int)(cosf(angle) * radius),
                           cy + (int)(sinf(angle) * radius), MARKER_GOAL_BE);
        }
    }
}

#endif  // 停用找球叠加层

// ==================== 循迹控制参数（开关式短促转向，不是连续 PID） ====================
// 注意：PID_KP/KI/KD/PID_OMEGA_MAX/PID_INTEGRAL_MAX 是早期连续 PID 方案留下的参数，
//       当前 line_follow_task 用的是“回差触发 + 定时短促转向 + 原地观察”的开关式控制，
//       代码里并未引用这几个宏。要调循迹手感请改 STEER_TRIGGER_ERR / STEER_DEADBAND /
//       TURN_PULSE_MS / TURN_PULSE_SPEED / OBSERVE_HOLD_MS / ERR_SMOOTH_K / BASE_SPEED
#define PID_KP              0.15f
#define PID_KI              0.01f
#define PID_KD              0.30f
#define PID_OMEGA_MAX       0.08f   // 转向输出上限
#define PID_INTEGRAL_MAX    0.40f
#define TURN_PULSE_MS       140     // 每次短促转向的最长持续时间（减小单次转向幅度）
#define TURN_PULSE_SPEED    0.20f  // 短促转向速度
#define OBSERVE_HOLD_MS     180    // 一次转向后原地停住的观察时间（0.1s）
#define SEARCH_TURN_SPEED   0.18f  // 丢线时搜索转向速度
#define STEER_DEADBAND      0.22f   // 回差退出阈值：|err|<此值停止转向
#define STEER_TRIGGER_ERR   0.40f   // 回差触发阈值：|err|>=此值才开始转向
#define ERR_SMOOTH_K        0.40f   // 误差平滑系数（0~1，越小越平滑）

static volatile int avoid_active = 0;       // 1=避障任务接管电机（循迹任务暂停）

/**
 * @brief 循迹控制任务：把视觉误差变成三个轮子的速度 —— 循迹的“决策 + 执行”环节
 *
 * @param arg 未使用（xTaskCreate 要求的形参）
 * @return 不返回；死循环，控制周期 CTRL_PERIOD_MS
 *
 * 输入：vision_ready / line_found / line_err / line_black_ratio（均由
 *       detect_line_from_rgb565 写入），以及 avoid_active（避障是否已接管电机）。
 * 输出：直接调 set_motor_speed() 驱动 M1/M2/M3。
 *
 * 状态判定按优先级自上而下：
 *  1. !vision_ready                  还没处理完第一帧 -> 三轮全停，避免上电就盲跑；
 *  2. avoid_active                   避障接管 -> 本任务完全不碰电机；
 *  3. !line_found                    丢线 -> 原地按 last_omega_dir 方向以 SEARCH_TURN_SPEED
 *                                    搜索；沿最后一次转向的方向找，比随机乱转更容易找回来；
 *  4. black_ratio ≥ ALL_BLACK_RATIO  画面几乎全黑（终点区/压上大片黑）-> 停车；
 *  5. 其它                           正常循迹。
 *
 * 正常循迹不是连续比例控制，而是“走 - 停 - 看 - 转”的节拍，用来压住小车的惯性过冲：
 *  - 误差平滑：smooth_err += ERR_SMOOTH_K*(err - smooth_err)，抑制单帧跳变造成的左右乱摆；
 *  - 回差（施密特）：|smooth_err| ≥ STEER_TRIGGER_ERR 且已过观察期才进入转向；转向中一旦
 *    |line_err| < STEER_DEADBAND，或本次已转满 TURN_PULSE_MS，就退出。触发阈值大于退出死区，
 *    所以不会在阈值附近反复进出转向；
 *  - 转向段：vx=0 原地转，速度 TURN_PULSE_SPEED，方向取平滑误差的符号并记进 last_omega_dir
 *    （丢线搜索要复用这个方向）；
 *  - 转完先原地停 OBSERVE_HOLD_MS 看清新画面，观察期结束才以 BASE_SPEED 继续直行。
 *
 * 轮速解算（M1 左前 +60°、M2 后轮 180°、M3 右前 -60°）：
 *  - 原地旋转（vx=0 且 ω≠0）：三轮同速同向；
 *  - 其余情况：M1 = -0.866*vx + 1.5*ω，M2 = 0（后轮不驱动、自由随动），
 *    M3 = +0.866*vx + 1.5*ω。当前状态机不会同时给出 vx≠0 和 ω≠0，
 *    所以这一支实际只用于纯直行，1.5*ω 项是留给以后“边走边转”的
 */
static void line_follow_task(void *arg)
{
    int last_omega_dir = 1;  // 最近一次转向输出的方向（+1/-1），丢线时保持该方向
    int64_t pulse_start = 0; // 本次短促转向开始时刻
    int64_t hold_end = 0;    // 转向结束后的原地观察截止时刻
    float smooth_err = 0.0f; // 平滑后的误差
    int smooth_init = 0;     // 平滑器是否已初始化
    int in_turn = 0;         // 是否处于“转向修正”中（配合回差）
    int log_cnt = 0;

    for (;;) {
        if (!vision_ready) {
            for (int i = 0; i < MOTOR_COUNT; i++) set_motor_speed(i, 0.0f);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // 避障接管期间，循迹任务不碰电机（控制权交给避障任务）
        if (avoid_active) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        float vx, omega;

        if (!line_found) {
            // 丢线：保持上一次转向方向原地搜索，直到找回黑线
            vx = 0.0f;
            omega = last_omega_dir * SEARCH_TURN_SPEED;
            smooth_init = 0;
            in_turn = 0;
        } else if (line_black_ratio >= ALL_BLACK_RATIO) {
            // 全黑停车
            vx = 0.0f;
            omega = 0.0f;
            smooth_init = 0;
            in_turn = 0;
        } else {
            float err = line_err;
            int64_t now = esp_timer_get_time();

            // 误差平滑：抑制单帧抖动导致的方向反复
            if (!smooth_init) {
                smooth_err = err;
                smooth_init = 1;
            } else {
                smooth_err += ERR_SMOOTH_K * (err - smooth_err);
            }
            float serr = smooth_err;

            // 进入转向：平滑误差超阈值，且已过原地观察期
            if (!in_turn && fabsf(serr) >= STEER_TRIGGER_ERR &&
                now >= hold_end) {
                in_turn = 1;
                pulse_start = now;
            }
            // 退出转向：原始误差回到死区，或本次短促转向达到最大时长
            if (in_turn && (fabsf(err) < STEER_DEADBAND ||
                            (now - pulse_start) >= (int64_t)TURN_PULSE_MS * 1000)) {
                in_turn = 0;
                hold_end = now + (int64_t)OBSERVE_HOLD_MS * 1000;   // 原地观察 0.1s
            }

            if (!in_turn) {
                if (now < hold_end) {
                    // 转向后原地停住观察 0.1s，避免惯性连续转向
                    vx = 0.0f;
                    omega = 0.0f;
                } else {
                    // 观察结束：正常直行
                    vx = BASE_SPEED;
                    omega = 0.0f;
                }
            } else {
                // 短促转向：固定方向、固定时长的一小段修正
                int dir = (serr >= 0.0f) ? 1 : -1;
                last_omega_dir = dir;
                omega = dir * TURN_PULSE_SPEED;
                vx = 0.0f;
            }
        }

        // 轮速解算：原地旋转（vx=0且ω≠0）三轮同速同向；其它情况只用两个前轮
        float speeds[MOTOR_COUNT];
        if (vx == 0.0f && omega != 0.0f) {
            speeds[0] = omega;
            speeds[1] = omega;
            speeds[2] = omega;
        } else {
            speeds[0] = -0.8660254f * vx + 1.5f * omega;
            speeds[1] = 0.0f;
            speeds[2] =  0.8660254f * vx + 1.5f * omega;
        }
        for (int i = 0; i < MOTOR_COUNT; i++) set_motor_speed(i, speeds[i]);

        if (++log_cnt >= 4) {
            log_cnt = 0;
            ESP_LOGI(TAG, "line: found=%d err=%.2f ang=%.1f black=%.0f%% vx=%.2f w=%.2f | M1=%.2f M2=%.2f M3=%.2f",
                     line_found, line_err, line_angle_deg, line_black_ratio * 100.0f,
                     vx, omega, speeds[0], speeds[1], speeds[2]);
        }
        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

// ==================== 超声波避障（参考之前红外版逻辑，条件一致） ====================
// 流程：连续两次距离 < 触发值 -> 制动 -> 定时左平移 -> 定时前进 -> 定时右平移 -> 恢复循迹；
// 硬编码避障路线结束后再固定前进 1s，随后直接进入找球程序。
#define ULTRASONIC_MIN_CM              2.0f
#define ULTRASONIC_MAX_CM            400.0f
#define ULTRASONIC_PERIOD_MS          20U
#define AVOID_TRIGGER_CM              6.0f   // 距离小于此值触发避障
#define AVOID_CLOSE_CONFIRM_SAMPLES    2U
#define AVOID_BRAKE_MS               500U
#define AVOID_LEFT_MS               1600U
#define AVOID_FWD_MS                1800U
#define AVOID_RIGHT_MS              1100U

// 原测试参数按百分比归一化；A/B/D 分轮设置，避免统一系数破坏实测比例。
#define AVOID_LEFT_A_SPEED            0.22f
#define AVOID_LEFT_B_SPEED            0.35f
#define AVOID_LEFT_D_SPEED            0.24f
#define AVOID_RIGHT_A_SPEED           0.18f
#define AVOID_RIGHT_B_SPEED           0.35f
#define AVOID_RIGHT_D_SPEED           0.24f
#define AVOID_FWD_A_SPEED             0.24f
#define AVOID_FWD_D_SPEED             0.27f
#define POST_AVOID_FORWARD_MS        1000U
#define POST_AVOID_FORWARD_COEF       0.8660254f
#define BALL_POWER_RECOVERY_MS       1000U

typedef enum {
    AV_NORMAL,
    AV_BRAKE,
    AV_LEFT,
    AV_FWD,
    AV_RIGHT,
    AV_POST_FWD,
} avoid_state_t;

/**
 * @brief 按物理轮 A/B/D 的独立速度驱动避障动作
 * @param a A 轮逻辑速度（M3）
 * @param b B 轮逻辑速度（M2）
 * @param d D 轮逻辑速度（M1）
 */
static void set_avoid_wheel_speed(float a, float b, float d)
{
    set_motor_speed(0, d);  // M1 = D
    set_motor_speed(1, b);  // M2 = B
    set_motor_speed(2, a);  // M3 = A
}

/**
 * @brief 超声波避障任务：按固定时序绕开障碍并直接切换到找球程序
 *
 * @param arg 未使用
 * @return 不返回；死循环，每 50ms 测一次距离
 *
 * 状态流转：
 *   AV_NORMAL --连续 AVOID_CLOSE_CONFIRM_SAMPLES 次 dist<AVOID_TRIGGER_CM--> AV_BRAKE
 *   --> AV_LEFT（固定左平移 AVOID_LEFT_MS）--> AV_FWD（固定前进 AVOID_FWD_MS）
 *   --> AV_RIGHT（固定右平移 AVOID_RIGHT_MS）
 *   --> AV_POST_FWD（固定前进 POST_AVOID_FORWARD_MS）--> 找球程序。
 *
 * 和循迹的分工：一进 AV_BRAKE 就置 avoid_active=1，line_follow_task 立刻松手，整个避障过程
 * 由本任务独占电机；最后固定前进结束后直接启动找球程序，不恢复巡线。
 */
static void avoid_task(void *arg)
{
    avoid_state_t st = AV_NORMAL;
    int64_t phase_start = 0;
    int trig_cnt = 0;    // 连续几次测到障碍才触发（防噪声误判）
    int dbg_cnt = 0;     // 调试打印计数

    for (;;) {
        float dist = measure_distance_cm();
        bool valid_dist = dist >= ULTRASONIC_MIN_CM && dist <= ULTRASONIC_MAX_CM;
        bool has_line = line_found;

        switch (st) {
        case AV_NORMAL:
            if (!avoid_active) {
                if (valid_dist && dist < AVOID_TRIGGER_CM) {
                    if (++trig_cnt >= AVOID_CLOSE_CONFIRM_SAMPLES) {
                        ESP_LOGI(TAG, "avoid: trigger d=%.1fcm -> brake", dist);
                        avoid_active = 1;      // 立即关闭摄像头寻线控制
                        if (line_follow_task_handle != NULL) {
                            vTaskSuspend(line_follow_task_handle);
                            ESP_LOGI(TAG, "avoid: line-follow task permanently suspended");
                        }
                        trig_cnt = 0;
                        st = AV_BRAKE;
                        phase_start = esp_timer_get_time();
                    }
                } else {
                    trig_cnt = 0;
                }
            }
            break;

        case AV_BRAKE:
            set_avoid_wheel_speed(0.0f, 0.0f, 0.0f);
            if (esp_timer_get_time() - phase_start >= AVOID_BRAKE_MS * 1000) {
                ESP_LOGI(TAG, "avoid: brake done -> left translate");
                st = AV_LEFT;
                phase_start = esp_timer_get_time();
            }
            break;

        case AV_LEFT:
            set_avoid_wheel_speed(AVOID_LEFT_A_SPEED, -AVOID_LEFT_B_SPEED,
                                  AVOID_LEFT_D_SPEED);
            if (esp_timer_get_time() - phase_start >= AVOID_LEFT_MS * 1000) {
                ESP_LOGI(TAG, "avoid: left fixed-time done -> forward dash");
                st = AV_FWD;
                phase_start = esp_timer_get_time();
            }
            break;

        case AV_FWD:
            // 前冲：直行一段固定时间（寻线仍关闭）
            set_avoid_wheel_speed(AVOID_FWD_A_SPEED, 0.0f, -AVOID_FWD_D_SPEED);
            if (esp_timer_get_time() - phase_start >= AVOID_FWD_MS * 1000) {
                ESP_LOGI(TAG, "avoid: forward done -> right translate");
                st = AV_RIGHT;
                phase_start = esp_timer_get_time();
            }
            break;

        case AV_RIGHT:
            // 右平移固定时间，结束后直接进入最后固定前进
            set_avoid_wheel_speed(-AVOID_RIGHT_A_SPEED, AVOID_RIGHT_B_SPEED,
                                  -AVOID_RIGHT_D_SPEED);
            if (esp_timer_get_time() - phase_start >= AVOID_RIGHT_MS * 1000) {
                ESP_LOGI(TAG, "avoid: right fixed-time done -> final forward %ums",
                         (unsigned)POST_AVOID_FORWARD_MS);
                st = AV_POST_FWD;
                phase_start = esp_timer_get_time();
            }
            break;

        case AV_POST_FWD: {
            float wheel_speed = POST_AVOID_FORWARD_COEF * BASE_SPEED;
            set_avoid_wheel_speed(wheel_speed, 0.0f, -wheel_speed);
            if (esp_timer_get_time() - phase_start >= POST_AVOID_FORWARD_MS * 1000) {
                ESP_LOGI(TAG, "avoid: final forward done -> power recovery %ums",
                         (unsigned)BALL_POWER_RECOVERY_MS);
                endpoint_ball_stop();
                vTaskDelay(pdMS_TO_TICKS(BALL_POWER_RECOVERY_MS));
                ESP_LOGI(TAG, "avoid: power recovered -> start ball program");
                endpoint_ball_start();
                vTaskDelete(NULL);
                return;
            }
            break;
        }
        }

        if (++dbg_cnt >= 3) {
            dbg_cnt = 0;
            ESP_LOGI(TAG, "us dist=%.1fcm echo=%d (st=%d line=%d active=%d)",
                     dist, gpio_get_level(ECHO_GPIO), (int)st, has_line ? 1 : 0, avoid_active);
        }
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));
    }
}

// ==================== 帧缓冲 + 显示任务 ====================
static SemaphoreHandle_t frame_sem = NULL;
static uint8_t *jpeg_buf = NULL;
static volatile size_t jpeg_len = 0;
static volatile uint16_t cam_w = 0, cam_h = 0;
static volatile int64_t last_frame_time = 0;

/**
 * @brief 显示 + 视觉任务：取一帧 JPEG -> 解码 -> 循迹检测 -> 缩放叠加 -> 送 LCD
 *
 * @param arg 未使用
 * @return 不返回；每次等到 frame_sem（由 camera_frame_cb 释放）就处理一帧
 *
 * 流程：esp_jpeg 以 1/2 比例把 MJPEG 解成 RGB565（swap_color_bytes=1，字节序直接可送屏），
 * 解码缓冲按需从 PSRAM 分配、并按见过的最大尺寸复用；接着对整幅解码图跑
 * detect_line_from_rgb565() 并置 vision_ready=1（这是循迹任务允许出车的总开关）；
 * 然后最近邻缩放到 CAM_DISP_W*CAM_DISP_H，叠加识别标注，最后贴到屏幕上。
 *
 * @note 循迹检测跑在解码后的整幅图上（分辨率高于屏幕），标注只画进缩放后的显示副本，
 *       不会污染判定用的数据。
 *       jpeg_buf 没有加锁：解码期间若来了新帧，回调会直接覆盖同一块缓冲，表现为偶发的
 *       解码失败（decode_fail 计数），当前靠丢帧容忍；每 2s 打印一次解码统计
 */
static void camera_display_task(void *arg)
{
    uint8_t *dec_buf = NULL;
    size_t dec_cap = 0;
    static uint8_t scaled[CAM_DISP_W * CAM_DISP_H * 2];
    int64_t last_stat = 0;
    int64_t last_status_lcd = 0;
    uint32_t decode_ok = 0, decode_fail = 0;

    for (;;) {
        if (xSemaphoreTake(frame_sem, portMAX_DELAY) != pdTRUE) continue;
        size_t len = jpeg_len;
        if (len == 0) continue;

        bool ball_mode = endpoint_ball_active();
        esp_jpeg_image_cfg_t cfg = {0};
        cfg.indata = jpeg_buf;
        cfg.indata_size = len;
        cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
        cfg.out_scale = ball_mode ? JPEG_IMAGE_SCALE_1_4 : JPEG_IMAGE_SCALE_1_2;
        cfg.flags.swap_color_bytes = 1;

        esp_jpeg_image_output_t info;
        if (esp_jpeg_get_image_info(&cfg, &info) != ESP_OK) { decode_fail++; continue; }
        uint32_t w = info.width, h = info.height;
        if (w == 0 || h == 0) { decode_fail++; continue; }

        size_t need = (size_t)w * h * 2;
        if (need > dec_cap) {
            if (dec_buf) heap_caps_free(dec_buf);
            dec_buf = heap_caps_malloc(need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!dec_buf) { dec_cap = 0; continue; }
            dec_cap = need;
        }
        cfg.outbuf = dec_buf;
        cfg.outbuf_size = dec_cap;

        esp_jpeg_image_output_t out;
        if (esp_jpeg_decode(&cfg, &out) != ESP_OK) { decode_fail++; continue; }
        decode_ok++;
        w = out.width; h = out.height;
        if (w == 0 || h == 0) continue;

        // 只运行原组黑线检测，供巡线和避障找回线使用。
        if (ball_mode) {
            endpoint_ball_process_frame(dec_buf, w, h);
        } else {
            detect_line_from_rgb565(dec_buf, w, h);
            vision_ready = 1;
        }

        // 最近邻缩放到屏幕显示区大小（只影响显示，循迹判定用的是上面的原始分辨率）
        for (int dy = 0; dy < CAM_DISP_H; dy++) {
            uint32_t sy = (uint32_t)((uint64_t)dy * h / CAM_DISP_H);
            const uint8_t *src_row = dec_buf + (size_t)sy * w * 2;
            uint8_t *dst = scaled + (size_t)dy * CAM_DISP_W * 2;
            for (int dx = 0; dx < CAM_DISP_W; dx++) {
                uint32_t sx = (uint32_t)((uint64_t)dx * w / CAM_DISP_W);
                const uint8_t *p = src_row + sx * 2;
                dst[dx * 2] = p[0];
                dst[dx * 2 + 1] = p[1];
            }
        }
        endpoint_ball_status_t endpoint_status;
        endpoint_ball_get_status(&endpoint_status);
        bool ball_active = endpoint_ball_active();
        // 在画面上标注识别到的黑线
        if (ball_active) {
            overlay_ball_target(scaled, &endpoint_status);
        } else {
            overlay_detected_line(scaled, w, h);
        }
        lcd_blit_cam(scaled, CAM_DISP_W, CAM_DISP_H);

        int64_t now = esp_timer_get_time();
        if (now - last_status_lcd >= 200000) {
            const char *state = ball_active ? "BALL" :
                                endpoint_status.paused ? "STOP" :
                                avoid_active ? "AVOID" :
                                !vision_ready ? "WAIT" :
                                !line_found ? "LOST" :
                                line_black_ratio >= ALL_BLACK_RATIO ? "STOP" : "NORMAL";
            int motor_a = ball_active ? endpoint_status.motor_a : lcd_motor_percent[2];
            int motor_b = ball_active ? endpoint_status.motor_b : lcd_motor_percent[1];
            int motor_d = ball_active ? endpoint_status.motor_d : lcd_motor_percent[0];
            int black_percent = (int)(line_black_ratio * 100.0f + 0.5f);
            lcd_status_t status = {
                .state = state,
                .armed = vision_ready != 0,
                .stby = gpio_get_level(MOTOR_STBY_GPIO) != 0,
                .motor_a = motor_a,
                .motor_b = motor_b,
                .motor_d = motor_d,
                .ultrasonic_cm = lcd_ultrasonic_cm,
                .candidate = line_found != 0,
                .threshold = (int)line_luma_thr,
                .seed_x = (int)line_cx,
                .valid_rows = line_found ? 1 : 0,
                .confidence = black_percent,
                .ball_phase = endpoint_status.phase,
            };
            lcd_show_status(&status);
            last_status_lcd = now;
        }
        if (now - last_stat > 2000000) {
            last_stat = now;
            ESP_LOGI(TAG, "decode ok=%lu fail=%lu dim=%lux%lu",
                     (unsigned long)decode_ok, (unsigned long)decode_fail,
                     (unsigned long)w, (unsigned long)h);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== USB 端口诊断 ====================
/**
 * @brief 打印 USB 主机端口状态：是否检测到连接、D+/D- 线状态、端口使能、协商速度
 * @note  只读寄存器、不改任何配置；直连 DWC OTG 的 HPRT 寄存器，
 *        用来排查“摄像头插上了但枚举不出来”这类问题
 */
static void usb_port_diag(void)
{
    usb_dwc_dev_t *hw = &USB_DWC;
    bool conn = usb_dwc_ll_hprt_get_conn_status(hw);
    uint32_t line = usb_dwc_ll_hprt_get_pwr_line_status(hw);
    bool en = usb_dwc_ll_hprt_get_port_en(hw);
    uint32_t spd = usb_dwc_ll_hprt_get_speed(hw);
    ESP_LOGI(TAG, "USB port diag: conn=%d line=%lu(0=SE0 1=K 2=J 3=SE1) en=%d spd=%lu",
             conn, (unsigned long)line, en, (unsigned long)spd);
}

// ==================== 官方 espressif/usb_stream 回调 ====================
/**
 * @brief usb_stream 的每帧回调：把 MJPEG 数据拷进 jpeg_buf 并唤醒显示任务
 * @param frame    UVC 帧，frame->data 为 JPEG 码流，只在回调期间有效，必须拷走
 * @param user_ptr 注册时传入的参数，未使用
 * @note  运行在 usb_stream 内部任务上下文，要尽快返回，不要在这里做解码。
 *        超过 JPEG_BUF_SIZE 的部分会被截断；frame_sem 是计数上限 1 的信号量，
 *        显示任务处理不过来时新帧直接覆盖旧帧（主动丢帧，保证画面是最新的）
 */
static void camera_frame_cb(uvc_frame_t *frame, void *user_ptr)
{
    if (!frame || !frame->data || frame->data_bytes == 0) return;
    size_t len = frame->data_bytes;
    if (len > JPEG_BUF_SIZE) len = JPEG_BUF_SIZE;
    memcpy(jpeg_buf, frame->data, len);
    jpeg_len = len;
    cam_w = frame->width;
    cam_h = frame->height;
    last_frame_time = esp_timer_get_time();
    xSemaphoreGive(frame_sem);
}

/**
 * @brief usb_stream 连接状态回调：连上时把摄像头支持的分辨率与帧间隔全部打印出来
 * @param state    STREAM_CONNECTED 表示已连接，其它值按断开处理
 * @param user_ptr 注册时传入的参数，未使用
 * @note  打印出来的分辨率列表用于核对 app_main 里 uvc_config_t 的 frame_width/height
 *        是不是摄像头真正支持的组合
 */
static void stream_state_cb(usb_stream_state_t state, void *user_ptr)
{
    if (state == STREAM_CONNECTED) {
        ESP_LOGI(TAG, "USB camera connected (official usb_stream)");
        size_t list_num = 0;
        if (uvc_frame_size_list_get(NULL, &list_num, NULL) == ESP_OK && list_num > 0) {
            uvc_frame_size_t *list = malloc(list_num * sizeof(uvc_frame_size_t));
            if (list) {
                size_t cur = 0;
                if (uvc_frame_size_list_get(list, &list_num, &cur) == ESP_OK) {
                    for (size_t i = 0; i < list_num; i++) {
                        ESP_LOGI(TAG, "camera frame[%u]: %ux%u interval=%lu",
                                 (unsigned)i, list[i].width, list[i].height,
                                 (unsigned long)list[i].interval);
                    }
                }
                free(list);
            }
        }
    } else {
        ESP_LOGW(TAG, "USB camera disconnected");
    }
}

// ==================== 主流程 ====================
/**
 * @brief 程序入口：按依赖顺序初始化外设，拉起三个任务，然后进入健康监控循环
 *
 * 顺序与理由：
 *   LCD、超声波初始化 -> 分配 1MB JPEG 缓冲和帧信号量 -> 显示/视觉任务 cam_lcd（prio 5）
 *   -> 电机初始化 -> 循迹任务 line_follow（prio 6，高于显示任务，保证控制周期稳定）
 *   -> 避障任务 avoid（prio 3 最低，因为测距是忙等，不能抢画面刷新的时间）
 *   -> 分配 USB 传输/帧缓冲（优先 PSRAM，失败退回内部 RAM）-> 等 2s 让独立供电的摄像头稳定
 *   -> 配置并启动 usb_stream，阻塞等摄像头连上。
 *
 * @note 循迹任务在 vision_ready 置起之前不驱动电机，所以先建任务、后连摄像头不会让车乱跑。
 *       连上之后主循环只做每 5s 一次的健康打印（USB 端口状态、当前分辨率、最后一帧距今多久）
 */
void app_main(void)
{
    esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGW(TAG, "boot reset reason=%s (%d)",
             reset_reason_name(reset_reason), (int)reset_reason);
    lcd_init();
    lcd_fill(0x0000);
    ultrasonic_init();

    jpeg_buf = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(jpeg_buf);
    frame_sem = xSemaphoreCreateCounting(1, 0);
    assert(frame_sem);
    xTaskCreate(camera_display_task, "cam_lcd", 8192, NULL, 5, NULL);

    motor_init();
    ESP_ERROR_CHECK(endpoint_ball_init(endpoint_ball_drive, endpoint_ball_stop,
                                       endpoint_ball_pause_controllers));
    // 原巡线和避障任务保持独立；终点确认后由找球模块暂停并接管。
    xTaskCreate(line_follow_task, "line_follow", 4096, NULL, 6, &line_follow_task_handle);
    xTaskCreate(avoid_task, "avoid", 4096, NULL, 3, NULL);

    uint8_t *xfer_a = heap_caps_malloc(XFER_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!xfer_a) xfer_a = heap_caps_malloc(XFER_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    assert(xfer_a);
    uint8_t *xfer_b = heap_caps_malloc(XFER_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!xfer_b) xfer_b = heap_caps_malloc(XFER_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    assert(xfer_b);
    uint8_t *frame_buf = heap_caps_malloc(XFER_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!frame_buf) frame_buf = heap_caps_malloc(XFER_BUF_SIZE, MALLOC_CAP_8BIT);
    assert(frame_buf);

    // 摄像头独立供电，上电后先等 2 秒让设备稳定
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "init official espressif/usb_stream component...");
    esp_log_level_set("USB_STREAM", ESP_LOG_WARN);
    esp_log_level_set("USB HOST", ESP_LOG_WARN);
    esp_log_level_set("HUB", ESP_LOG_WARN);
    esp_log_level_set("ENUM", ESP_LOG_WARN);
    esp_log_level_set("USBH", ESP_LOG_WARN);
    esp_log_level_set("HCD DWC", ESP_LOG_WARN);

    uvc_config_t uvc = {
        .frame_width = 480,
        .frame_height = 320,
        .frame_interval = FPS2INTERVAL(15),
        .xfer_buffer_size = XFER_BUF_SIZE,
        .xfer_buffer_a = xfer_a,
        .xfer_buffer_b = xfer_b,
        .frame_buffer_size = XFER_BUF_SIZE,
        .frame_buffer = frame_buf,
        .frame_cb = camera_frame_cb,
        .frame_cb_arg = NULL,
    };
    ESP_ERROR_CHECK(uvc_streaming_config(&uvc));
    ESP_ERROR_CHECK(usb_streaming_state_register(stream_state_cb, NULL));
    ESP_ERROR_CHECK(usb_streaming_start());

    ESP_LOGI(TAG, "waiting for USB camera...");
    usb_port_diag();
    ESP_ERROR_CHECK(usb_streaming_connect_wait(portMAX_DELAY));
    usb_port_diag();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        usb_port_diag();
        ESP_LOGI(TAG, "cam %ux%u, last frame %.1fs ago",
                 cam_w, cam_h,
                 (esp_timer_get_time() - last_frame_time) / 1e6f);
    }
}
