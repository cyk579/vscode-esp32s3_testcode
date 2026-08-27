#include "tft_st7735.h"                                  // 引入 TFT 状态结构和接口。
#include <stdio.h>                                        // 使用 snprintf()。
#include <string.h>                                       // 使用 memset()。
#include "driver/gpio.h"                                  // GPIO 驱动。
#include "driver/spi_master.h"                            // SPI 主机驱动。
#include "esp_log.h"                                      // 日志输出。
#include "freertos/FreeRTOS.h"                            // FreeRTOS 基础定义。
#include "freertos/task.h"                                // 使用 vTaskDelay()。

#define TFT_SCLK GPIO_NUM_13                              // 保持原仓库 SCLK=GPIO13。
#define TFT_MOSI GPIO_NUM_14                              // 保持原仓库 MOSI=GPIO14。
#define TFT_DC   GPIO_NUM_21                              // 保持原仓库 D/C=GPIO21。
#define TFT_CS   GPIO_NUM_47                              // 保持原仓库 CS=GPIO47。
#define TFT_RST  GPIO_NUM_38                              // 保持原仓库 RST=GPIO38。
#define TFT_W 128                                         // 保持原仓库 ST7735 宽度 128。
#define TFT_H 160                                         // 保持原仓库 ST7735 高度 160。
#define TFT_SPI_HZ 8000000                                // 保持原仓库 8 MHz；先保证稳定，不用超频换刷新率。
#define TEXT_H 7                                          // 当前字模实际高度 7 像素。
#define ROW_CLEAR_H 10                                    // 每次刷新一行前清除 10 像素高度，防止旧字符残留。

static spi_device_handle_t dev;                           // 保存 ST7735 SPI 设备句柄。
static const char *TAG = "tft";                           // TFT 日志 TAG。

static void write_bytes(const void *data_ptr, size_t len, int dc) // 通过 SPI 发送命令或数据。
{                                                         // SPI 发送函数开始。
    gpio_set_level(TFT_DC, dc);                           // dc=0 表示命令，dc=1 表示像素/参数数据。
    spi_transaction_t transaction = {                    // 建立一个同步 SPI 事务。
        .length = len * 8U,                               // ESP-IDF 的 length 单位是 bit。
        .tx_buffer = data_ptr                             // 指向待发送数据。
    };                                                    // SPI 事务结构结束。
    ESP_ERROR_CHECK(spi_device_transmit(dev, &transaction)); // 同步发送，错误时立即报告。
}                                                         // SPI 发送函数结束。

static void command(uint8_t cmd)                          // 发送一个 ST7735 命令字节。
{                                                         // 命令发送开始。
    write_bytes(&cmd, 1U, 0);                             // D/C=0，发送 1 字节命令。
}                                                         // 命令发送结束。

static void data(const uint8_t *bytes, size_t len)        // 发送 ST7735 数据。
{                                                         // 数据发送开始。
    write_bytes(bytes, len, 1);                           // D/C=1，发送参数或像素数据。
}                                                         // 数据发送结束。

static void set_window(int x0, int y0, int x1, int y1)   // 设置接下来写像素的矩形区域。
{                                                         // 设置窗口开始。
    uint8_t bytes[4];                                     // 保存两个 16 bit 坐标的高低字节。
    command(0x2A);                                        // CASET：设置列地址。
    bytes[0] = 0;                                         // x0 高字节；128 宽屏幕始终为 0。
    bytes[1] = (uint8_t)x0;                               // x0 低字节。
    bytes[2] = 0;                                         // x1 高字节。
    bytes[3] = (uint8_t)x1;                               // x1 低字节。
    data(bytes, sizeof(bytes));                           // 发送列范围。
    command(0x2B);                                        // RASET：设置行地址。
    bytes[0] = 0;                                         // y0 高字节；160 高屏幕始终为 0。
    bytes[1] = (uint8_t)y0;                               // y0 低字节。
    bytes[2] = 0;                                         // y1 高字节。
    bytes[3] = (uint8_t)y1;                               // y1 低字节。
    data(bytes, sizeof(bytes));                           // 发送行范围。
    command(0x2C);                                        // RAMWR：后续数据写入该窗口。
}                                                         // 设置窗口结束。

static void fill_rect(int x0, int y0, int x1, int y1, uint16_t color) // 填充一个矩形，而不是每帧清整个屏幕。
{                                                         // 矩形填充开始。
    if (x0 < 0) x0 = 0;                                  // 限制左边界。
    if (y0 < 0) y0 = 0;                                  // 限制上边界。
    if (x1 >= TFT_W) x1 = TFT_W - 1;                     // 限制右边界。
    if (y1 >= TFT_H) y1 = TFT_H - 1;                     // 限制下边界。
    if (x0 > x1 || y0 > y1) return;                      // 无效矩形直接返回。
    int width = x1 - x0 + 1;                             // 计算矩形宽度。
    static uint8_t line[TFT_W * 2];                      // 最大一行 RGB565 数据缓冲。
    for (int x = 0; x < width; ++x) {                    // 生成一行固定颜色。
        line[2 * x] = (uint8_t)(color >> 8);             // RGB565 高字节。
        line[2 * x + 1] = (uint8_t)color;                // RGB565 低字节。
    }                                                     // 一行颜色生成结束。
    set_window(x0, y0, x1, y1);                          // 设置矩形写入区域。
    for (int y = y0; y <= y1; ++y) {                    // 逐行填充矩形。
        data(line, (size_t)width * 2U);                   // 发送当前一行像素。
    }                                                     // 矩形逐行发送结束。
}                                                         // 矩形填充结束。

static void fill_screen(uint16_t color)                   // 只在初始化时使用整屏填充。
{                                                         // 整屏填充开始。
    fill_rect(0, 0, TFT_W - 1, TFT_H - 1, color);        // 填满 128×160。
}                                                         // 整屏填充结束。

static void clear_status_row(int y)                       // 清除某一行状态文本所在的小区域。
{                                                         // 行清除开始。
    fill_rect(0, y, TFT_W - 1, y + ROW_CLEAR_H - 1, 0x0000); // 只清 10 像素高，而不是整屏清除。
}                                                         // 行清除结束。

static const uint8_t *glyph(char c)                       // 返回 5×7 英文字模。
{                                                         // 字模函数开始。
    static uint8_t g[5];                                  // 每个字符 5 列。
    memset(g, 0, sizeof(g));                              // 未定义字符默认显示为空白。
    switch (c) {                                          // 根据字符建立点阵。
    case '0': g[0]=0x3e;g[1]=0x51;g[2]=0x49;g[3]=0x45;g[4]=0x3e;break; // 数字 0。
    case '1': g[0]=0x00;g[1]=0x42;g[2]=0x7f;g[3]=0x40;g[4]=0x00;break; // 数字 1。
    case '2': g[0]=0x62;g[1]=0x51;g[2]=0x49;g[3]=0x49;g[4]=0x46;break; // 数字 2。
    case '3': g[0]=0x22;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x36;break; // 数字 3。
    case '4': g[0]=0x18;g[1]=0x14;g[2]=0x12;g[3]=0x7f;g[4]=0x10;break; // 数字 4。
    case '5': g[0]=0x2f;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x31;break; // 数字 5。
    case '6': g[0]=0x3e;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x32;break; // 数字 6。
    case '7': g[0]=0x01;g[1]=0x71;g[2]=0x09;g[3]=0x05;g[4]=0x03;break; // 数字 7。
    case '8': g[0]=0x36;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x36;break; // 数字 8。
    case '9': g[0]=0x26;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x3e;break; // 数字 9。
    case 'A': g[0]=0x7e;g[1]=0x11;g[2]=0x11;g[3]=0x11;g[4]=0x7e;break; // A。
    case 'B': g[0]=0x7f;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x36;break; // B。
    case 'C': g[0]=0x3e;g[1]=0x41;g[2]=0x41;g[3]=0x41;g[4]=0x22;break; // C。
    case 'D': g[0]=0x7f;g[1]=0x41;g[2]=0x41;g[3]=0x22;g[4]=0x1c;break; // D。
    case 'E': g[0]=0x7f;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x41;break; // E。
    case 'F': g[0]=0x7f;g[1]=0x09;g[2]=0x09;g[3]=0x09;g[4]=0x01;break; // F。
    case 'I': g[0]=0x00;g[1]=0x41;g[2]=0x7f;g[3]=0x41;g[4]=0x00;break; // I。
    case 'L': g[0]=0x7f;g[1]=0x40;g[2]=0x40;g[3]=0x40;g[4]=0x40;break; // L。
    case 'M': g[0]=0x7f;g[1]=0x02;g[2]=0x0c;g[3]=0x02;g[4]=0x7f;break; // M。
    case 'N': g[0]=0x7f;g[1]=0x02;g[2]=0x0c;g[3]=0x10;g[4]=0x7f;break; // N。
    case 'O': g[0]=0x3e;g[1]=0x41;g[2]=0x41;g[3]=0x41;g[4]=0x3e;break; // O。
    case 'R': g[0]=0x7f;g[1]=0x09;g[2]=0x19;g[3]=0x29;g[4]=0x46;break; // R。
    case 'S': g[0]=0x46;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x31;break; // S。
    case 'T': g[0]=0x01;g[1]=0x01;g[2]=0x7f;g[3]=0x01;g[4]=0x01;break; // T。
    case 'U': g[0]=0x3f;g[1]=0x40;g[2]=0x40;g[3]=0x40;g[4]=0x3f;break; // U。
    case 'V': g[0]=0x1f;g[1]=0x20;g[2]=0x40;g[3]=0x20;g[4]=0x1f;break; // V。
    case ':': g[0]=0x00;g[1]=0x36;g[2]=0x36;g[3]=0x00;g[4]=0x00;break; // 冒号。
    case '-': g[0]=0x08;g[1]=0x08;g[2]=0x08;g[3]=0x08;g[4]=0x08;break; // 减号。
    case '.': g[0]=0x00;g[1]=0x60;g[2]=0x60;g[3]=0x00;g[4]=0x00;break; // 小数点。
    case '/': g[0]=0x20;g[1]=0x10;g[2]=0x08;g[3]=0x04;g[4]=0x02;break; // 斜杠。
    }                                                     // 字模选择结束。
    return g;                                             // 返回 5 列点阵。
}                                                         // 字模函数结束。

static void text(int x, int y, const char *string, uint16_t color) // 绘制 5×7 字符串。
{                                                         // 文本绘制开始。
    uint8_t pixels[5 * TEXT_H * 2];                      // 一个字符的 RGB565 像素缓冲。
    while (*string) {                                     // 逐个字符绘制。
        const uint8_t *g = glyph(*string++);              // 取得当前字符字模。
        memset(pixels, 0, sizeof(pixels));                // 字符背景默认为黑色。
        for (int row = 0; row < TEXT_H; ++row) {         // 遍历字符 7 行。
            for (int col = 0; col < 5; ++col) {          // 遍历字符 5 列。
                if (g[col] & (1U << row)) {               // 当前像素需要点亮。
                    size_t index = (size_t)(row * 5 + col) * 2U; // 计算 RGB565 缓冲下标。
                    pixels[index] = (uint8_t)(color >> 8); // 高字节。
                    pixels[index + 1U] = (uint8_t)color;  // 低字节。
                }                                         // 当前像素判断结束。
            }                                             // 当前行列遍历结束。
        }                                                 // 整个字符点阵生成结束。
        set_window(x, y, x + 4, y + TEXT_H - 1);         // 设置 5×7 字符区域。
        data(pixels, sizeof(pixels));                     // 一次写入完整字符。
        x += 6;                                           // 下一个字符右移 6 像素，留下 1 像素间距。
    }                                                     // 字符串绘制结束。
}                                                         // 文本绘制函数结束。

bool tft_st7735_init(void)                                // 初始化 ST7735 屏幕。
{                                                         // TFT 初始化开始。
    gpio_reset_pin(TFT_DC);                               // 复位 D/C GPIO。
    gpio_set_direction(TFT_DC, GPIO_MODE_OUTPUT);         // D/C 设置为输出。
    gpio_reset_pin(TFT_CS);                               // 复位 CS GPIO。
    gpio_set_direction(TFT_CS, GPIO_MODE_OUTPUT);         // CS 设置为输出。
    gpio_reset_pin(TFT_RST);                              // 复位 RST GPIO。
    gpio_set_direction(TFT_RST, GPIO_MODE_OUTPUT);        // RST 设置为输出。
    spi_bus_config_t bus = {                              // 配置 SPI2 总线。
        .sclk_io_num = TFT_SCLK,                          // SCLK=GPIO13。
        .mosi_io_num = TFT_MOSI,                          // MOSI=GPIO14。
        .miso_io_num = -1,                                // 屏幕只写不读，不使用 MISO。
        .quadwp_io_num = -1,                              // 不使用 Quad WP。
        .quadhd_io_num = -1,                              // 不使用 Quad HD。
        .max_transfer_sz = TFT_W * 2                      // 最大单事务为一整行 256 字节。
    };                                                    // SPI 总线配置结束。
    esp_err_t error = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO); // 初始化 SPI2，并启用自动 DMA。
    if (error != ESP_OK && error != ESP_ERR_INVALID_STATE) { // SPI 初始化真正失败。
        ESP_LOGE(TAG, "SPI init failed: %s", esp_err_to_name(error)); // 打印失败原因。
        return false;                                     // 返回初始化失败。
    }                                                     // SPI 初始化检查结束。
    spi_device_interface_config_t config = {              // 配置 ST7735 SPI 设备。
        .clock_speed_hz = TFT_SPI_HZ,                     // 保持 8 MHz，现阶段不靠超频提升刷新。
        .mode = 0,                                        // ST7735 使用 SPI mode 0。
        .spics_io_num = TFT_CS,                           // CS=GPIO47。
        .queue_size = 1                                   // 当前驱动使用同步单事务。
    };                                                    // SPI 设备配置结束。
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &config, &dev)); // 把 ST7735 添加到 SPI 总线。
    gpio_set_level(TFT_RST, 0);                           // 硬件复位拉低。
    vTaskDelay(pdMS_TO_TICKS(20));                        // 保持低 20 ms。
    gpio_set_level(TFT_RST, 1);                           // 硬件复位释放。
    vTaskDelay(pdMS_TO_TICKS(120));                       // 等待屏幕复位完成。
    command(0x01);                                        // SWRESET：软件复位。
    vTaskDelay(pdMS_TO_TICKS(120));                       // 等待软件复位。
    command(0x11);                                        // SLPOUT：退出休眠。
    vTaskDelay(pdMS_TO_TICKS(120));                       // 等待退出休眠。
    uint8_t pixel_format = 0x05;                          // 0x05：16 bit RGB565，与原仓库一致。
    command(0x3A);                                        // COLMOD：设置像素格式。
    data(&pixel_format, 1U);                              // 写入 RGB565。
    uint8_t madctl = 0xC8;                                // 保持原仓库方向/BGR 配置；若当前画面方向正确就不要改。
    command(0x36);                                        // MADCTL：设置扫描方向/颜色顺序。
    data(&madctl, 1U);                                    // 写入 0xC8。
    command(0x29);                                        // DISPON：打开显示。
    vTaskDelay(pdMS_TO_TICKS(20));                        // 等待显示稳定。
    fill_screen(0x0000);                                  // 只在初始化时清一次整屏。
    return true;                                          // TFT 初始化成功。
}                                                         // TFT 初始化结束。

void tft_st7735_show(const tft_status_t *status)          // 刷新调试状态。
{                                                         // 状态刷新开始。
    char line[32];                                        // 文本格式化缓冲。

    clear_status_row(4);                                  // 只清距离所在行。
    if (status->distance_cm < 0.0f) {                     // 当前无有效距离。
        text(2, 4, "DIST:---CM", 0xFFFF);                 // 显示 ---。
    } else {                                              // 当前距离有效。
        snprintf(line, sizeof(line), "DIST:%3.0fCM", status->distance_cm); // 格式化整数厘米。
        text(2, 4, line, 0xFFFF);                         // 显示距离。
    }                                                     // 距离显示结束。

    clear_status_row(24);                                 // 清红外状态行。
    snprintf(line, sizeof(line), "IR:%u%u%u%u",           // 格式化 4 位 ACTIVE。
             status->ir_mask & 1U,                        // 左侧 bit。
             (status->ir_mask >> 1) & 1U,                 // 左中 bit。
             (status->ir_mask >> 2) & 1U,                 // 右中 bit。
             (status->ir_mask >> 3) & 1U);                // 右侧 bit。
    text(2, 24, line, 0x07E0);                            // 绿色显示红外状态。

    clear_status_row(44);                                 // 清误差行。
    snprintf(line, sizeof(line), "ERR:%d", status->error); // 格式化 error。
    text(2, 44, line, 0xFFE0);                            // 黄色显示 error。

    clear_status_row(64);                                 // 清 yaw 行。
    snprintf(line, sizeof(line), "TURN:%d", status->turn); // 格式化真实 yaw。
    text(2, 64, line, 0xFFE0);                            // 黄色显示 yaw。

    clear_status_row(84);                                 // 清 A/B 行。
    snprintf(line, sizeof(line), "A:%d B:%d", status->motor_a, status->motor_b); // 格式化 A/B。
    text(2, 84, line, 0xF81F);                            // 紫色显示 A/B。

    clear_status_row(104);                                // 清 D 行。
    snprintf(line, sizeof(line), "D:%d", status->motor_d); // 格式化 D。
    text(2, 104, line, 0xF81F);                           // 紫色显示 D。

    clear_status_row(124);                                // 清模式行，防止 LINE→END 等变短后残留旧字符。
    text(2, 124, status->mode ? status->mode : "LINE", 0xFFFF); // 显示 TEST/LINE/AV-L/AV-F/AV-R/DIST/FAIL/END。
}                                                         // 状态刷新结束。
