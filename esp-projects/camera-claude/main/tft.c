#include "tft.h"
#include "board_pins.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <string.h>

/* 引脚见 board_pins.h。SPI2_HOST 和本车原固件一致。 */
#define TFT_HOST    SPI2_HOST
#define PIN_NUM_MISO PIN_TFT_MISO
#define PIN_NUM_MOSI PIN_TFT_MOSI
#define PIN_NUM_CLK  PIN_TFT_CLK
#define PIN_NUM_CS   PIN_TFT_CS
#define PIN_NUM_DC   PIN_TFT_DC
#define PIN_NUM_RST  PIN_TFT_RST

/* 面板变体差异：他们的屏用 0xC8 竖屏 + 行列偏移 (2,1)，本车原固件实测是
 * 0xA0 横屏 + 偏移 (0,0)。他们整套文字坐标是按 128 宽竖屏排的，所以这里
 * 先保留他们的值，画面不对时改这三个宏（README「屏幕不对怎么调」一节）。 */
#define TFT_MADCTL_VALUE 0xC8
#define TFT_COL_OFFSET 2
#define TFT_ROW_OFFSET 1
static spi_device_handle_t spi;

// ST7735 常用指令
#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5
#define ST7735_INVOFF  0x20
#define ST7735_MADCTL  0x36
#define ST7735_COLMOD  0x3A
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_DISPON  0x29

// 发送指令
static void tft_send_cmd(uint8_t cmd) {
    gpio_set_level(PIN_NUM_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_polling_transmit(spi, &t);
}

// 发送数据
static void tft_send_data(uint8_t data) {
    gpio_set_level(PIN_NUM_DC, 1);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    spi_device_polling_transmit(spi, &t);
}

// 设置显示窗口
static void tft_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    tft_send_cmd(ST7735_CASET);
    tft_send_data(0x00);
    tft_send_data(x1 + TFT_COL_OFFSET); // 偏移修正
    tft_send_data(0x00);
    tft_send_data(x2 + TFT_COL_OFFSET);

    tft_send_cmd(ST7735_RASET);
    tft_send_data(0x00);
    tft_send_data(y1 + TFT_ROW_OFFSET);
    tft_send_data(0x00);
    tft_send_data(y2 + TFT_ROW_OFFSET);

    tft_send_cmd(ST7735_RAMWR);
}

void tft_init(void) {
    // 1. 初始化 GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<PIN_NUM_DC) | (1ULL<<PIN_NUM_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    // 2. 初始化 SPI 总线
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096  // 🌟 核心修改：缩减内存申请，防止底层崩溃
    };
    // 🌟 核心修改：强制捕捉错误！如果总线罢工，终端会直接爆红报错
    ESP_ERROR_CHECK(spi_bus_initialize(TFT_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10MHz
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
    };
    // 🌟 核心修改：强制捕捉错误！
    ESP_ERROR_CHECK(spi_bus_add_device(TFT_HOST, &devcfg, &spi));

    // 3. 复位屏
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 4. 初始化序列
    tft_send_cmd(ST7735_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    tft_send_cmd(ST7735_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    tft_send_cmd(ST7735_FRMCTR1); tft_send_data(0x01); tft_send_data(0x2C); tft_send_data(0x2D);
    tft_send_cmd(ST7735_FRMCTR2); tft_send_data(0x01); tft_send_data(0x2C); tft_send_data(0x2D);
    tft_send_cmd(ST7735_INVCTR);  tft_send_data(0x07);
    tft_send_cmd(ST7735_PWCTR1);  tft_send_data(0xA2); tft_send_data(0x02); tft_send_data(0x84);
    tft_send_cmd(ST7735_PWCTR2);  tft_send_data(0xC5);
    tft_send_cmd(ST7735_PWCTR3);  tft_send_data(0x0A); tft_send_data(0x00);
    tft_send_cmd(ST7735_PWCTR4);  tft_send_data(0x8A); tft_send_data(0x2A);
    tft_send_cmd(ST7735_PWCTR5);  tft_send_data(0x8A); tft_send_data(0xEE);
    tft_send_cmd(ST7735_VMCTR1);  tft_send_data(0x0E);
    tft_send_cmd(ST7735_MADCTL);  tft_send_data(TFT_MADCTL_VALUE); // 扫描方向
    tft_send_cmd(ST7735_COLMOD);  tft_send_data(0x05); // 16-bit color
    tft_send_cmd(ST7735_DISPON);
    
    tft_clear(TFT_BLACK);
}

void tft_clear(uint16_t color) {
    tft_set_window(0, 0, 127, 159);
    uint16_t *buf = heap_caps_malloc(128 * 2, MALLOC_CAP_DMA);
    for(int i=0; i<128; i++) buf[i] = (color >> 8) | (color << 8);
    gpio_set_level(PIN_NUM_DC, 1);
    for(int y=0; y<160; y++) {
        spi_transaction_t t = {
            .length = 128 * 16,
            .tx_buffer = buf,
        };
        spi_device_polling_transmit(spi, &t);
    }
    heap_caps_free(buf);
}

// 简单的 ASCII 字符集实现 (通常放在单独的头文件)
extern const uint8_t ascii_font8x16[95][16];

void tft_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg_color) {
    if (c < 32 || c > 126) return;
    const uint8_t *bitmap = ascii_font8x16[c - 32];
    tft_set_window(x, y, x + 7, y + 15);
    gpio_set_level(PIN_NUM_DC, 1);
    uint16_t data[8];
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 8; j++) {
            data[j] = (bitmap[i] & (0x80 >> j)) ? (color >> 8 | color << 8) : (bg_color >> 8 | bg_color << 8);
        }
        spi_transaction_t t = {
            .length = 8 * 16,
            .tx_buffer = data,
        };
        spi_device_polling_transmit(spi, &t);
    }
}

void tft_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg_color) {
    while (*str) {
        tft_draw_char(x, y, *str++, color, bg_color);
        x += 8;
        if (x > 120) { x = 0; y += 16; }
    }
}

// 为了代码简洁，这里直接嵌入一个精简的字体表
const uint8_t ascii_font8x16[95][16] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // (space)
    {0x00,0x00,0x18,0x3C,0x3C,0x3C,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00}, // !
    // ... 此处省略大量字体数据，实际应用中我会使用更完整的字库 ...
    // 下面提供数字 0-9 的关键数据以保证基本功能
    ['0'-32] = {0x00,0x00,0x3E,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x3E,0x00,0x00,0x00},
    ['1'-32] = {0x00,0x00,0x08,0x18,0x28,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x3E,0x00,0x00,0x00},
    ['2'-32] = {0x00,0x00,0x3E,0x41,0x01,0x01,0x01,0x02,0x0C,0x10,0x20,0x41,0x7F,0x00,0x00,0x00},
    ['3'-32] = {0x00,0x00,0x3E,0x41,0x01,0x01,0x1E,0x01,0x01,0x01,0x01,0x41,0x3E,0x00,0x00,0x00},
    ['4'-32] = {0x00,0x00,0x02,0x06,0x0A,0x12,0x22,0x42,0x7F,0x02,0x02,0x02,0x07,0x00,0x00,0x00},
    ['5'-32] = {0x00,0x00,0x7F,0x40,0x40,0x40,0x7E,0x01,0x01,0x01,0x01,0x41,0x3E,0x00,0x00,0x00},
    ['6'-32] = {0x00,0x00,0x1E,0x20,0x40,0x40,0x7E,0x41,0x41,0x41,0x41,0x41,0x3E,0x00,0x00,0x00},
    ['7'-32] = {0x00,0x00,0x7F,0x01,0x01,0x01,0x02,0x04,0x08,0x10,0x10,0x10,0x10,0x00,0x00,0x00},
    ['8'-32] = {0x00,0x00,0x3E,0x41,0x41,0x41,0x3E,0x41,0x41,0x41,0x41,0x41,0x3E,0x00,0x00,0x00},
    ['9'-32] = {0x00,0x00,0x3E,0x41,0x41,0x41,0x41,0x41,0x7F,0x01,0x01,0x02,0x1C,0x00,0x00,0x00},
    ['.'-32] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00},
    [':'-32] = {0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00},
    ['A'-32] = {0x00,0x00,0x18,0x24,0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x42,0x42,0x00,0x00,0x00},
    ['B'-32] = {0x00,0x00,0x7C,0x42,0x42,0x42,0x7C,0x42,0x42,0x42,0x42,0x42,0x7C,0x00,0x00,0x00},
    ['D'-32] = {0x00,0x00,0x78,0x44,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x44,0x78,0x00,0x00,0x00},
    ['E'-32] = {0x00,0x00,0x7E,0x40,0x40,0x40,0x78,0x40,0x40,0x40,0x40,0x40,0x7E,0x00,0x00,0x00},
    ['S'-32] = {0x00,0x00,0x3E,0x41,0x40,0x40,0x3E,0x01,0x01,0x01,0x01,0x41,0x3E,0x00,0x00,0x00},
    ['c'-32] = {0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x40,0x40,0x40,0x40,0x41,0x3E,0x00,0x00,0x00},
    ['m'-32] = {0x00,0x00,0x00,0x00,0x00,0x00,0x66,0x99,0x99,0x99,0x99,0x99,0x99,0x00,0x00,0x00},
};
