#include "tft_st7735.h"
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TFT_SCLK GPIO_NUM_13
#define TFT_MOSI GPIO_NUM_14
#define TFT_DC   GPIO_NUM_21
#define TFT_CS   GPIO_NUM_47
#define TFT_RST  GPIO_NUM_38
#define TFT_W 128
#define TFT_H 160

static spi_device_handle_t dev;
static const char *TAG = "tft";

static void write_bytes(const void *data, size_t len, int dc)
{
    gpio_set_level(TFT_DC, dc);
    spi_transaction_t t = {.length = len * 8, .tx_buffer = data};
    ESP_ERROR_CHECK(spi_device_transmit(dev, &t));
}

static void command(uint8_t cmd)
{
    write_bytes(&cmd, 1, 0);
}

static void data(const uint8_t *bytes, size_t len)
{
    write_bytes(bytes, len, 1);
}

static void set_window(int x0, int y0, int x1, int y1)
{
    uint8_t b[4];
    command(0x2A); b[0] = 0; b[1] = x0; b[2] = 0; b[3] = x1; data(b, 4);
    command(0x2B); b[0] = 0; b[1] = y0; b[2] = 0; b[3] = y1; data(b, 4);
    command(0x2C);
}

static void fill(uint16_t color)
{
    static uint8_t line[TFT_W * 2];
    for (int i = 0; i < TFT_W; ++i) { line[2 * i] = color >> 8; line[2 * i + 1] = color; }
    set_window(0, 0, TFT_W - 1, TFT_H - 1);
    for (int y = 0; y < TFT_H; ++y) data(line, sizeof(line));
}

static const uint8_t *glyph(char c)
{
    static uint8_t g[5];
    memset(g, 0, sizeof(g));
    switch (c) {
    case '0': g[0]=0x3e;g[1]=0x51;g[2]=0x49;g[3]=0x45;g[4]=0x3e;break;
    case '1': g[0]=0x00;g[1]=0x42;g[2]=0x7f;g[3]=0x40;g[4]=0x00;break;
    case '2': g[0]=0x62;g[1]=0x51;g[2]=0x49;g[3]=0x49;g[4]=0x46;break;
    case '3': g[0]=0x22;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x36;break;
    case '4': g[0]=0x18;g[1]=0x14;g[2]=0x12;g[3]=0x7f;g[4]=0x10;break;
    case '5': g[0]=0x2f;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x31;break;
    case '6': g[0]=0x3e;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x32;break;
    case '7': g[0]=0x01;g[1]=0x71;g[2]=0x09;g[3]=0x05;g[4]=0x03;break;
    case '8': g[0]=0x36;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x36;break;
    case '9': g[0]=0x26;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x3e;break;
    case 'A': g[0]=0x7e;g[1]=0x11;g[2]=0x11;g[3]=0x11;g[4]=0x7e;break;
    case 'B': g[0]=0x7f;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x36;break;
    case 'D': g[0]=0x7f;g[1]=0x41;g[2]=0x41;g[3]=0x22;g[4]=0x1c;break;
    case 'E': g[0]=0x7f;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x41;break;
    case 'I': g[0]=0x00;g[1]=0x41;g[2]=0x7f;g[3]=0x41;g[4]=0x00;break;
    case 'M': g[0]=0x7f;g[1]=0x02;g[2]=0x0c;g[3]=0x02;g[4]=0x7f;break;
    case 'R': g[0]=0x7f;g[1]=0x09;g[2]=0x19;g[3]=0x29;g[4]=0x46;break;
    case 'S': g[0]=0x46;g[1]=0x49;g[2]=0x49;g[3]=0x49;g[4]=0x31;break;
    case 'T': g[0]=0x01;g[1]=0x01;g[2]=0x7f;g[3]=0x01;g[4]=0x01;break;
    case 'U': g[0]=0x3f;g[1]=0x40;g[2]=0x40;g[3]=0x40;g[4]=0x3f;break;
    case 'C': g[0]=0x3e;g[1]=0x41;g[2]=0x41;g[3]=0x41;g[4]=0x22;break;
    case 'L': g[0]=0x7f;g[1]=0x40;g[2]=0x40;g[3]=0x40;g[4]=0x40;break;
    case 'N': g[0]=0x7f;g[1]=0x02;g[2]=0x0c;g[3]=0x10;g[4]=0x7f;break;
    case 'O': g[0]=0x3e;g[1]=0x41;g[2]=0x41;g[3]=0x41;g[4]=0x3e;break;
    case 'V': g[0]=0x1f;g[1]=0x20;g[2]=0x40;g[3]=0x20;g[4]=0x1f;break;
    case 'F': g[0]=0x7f;g[1]=0x09;g[2]=0x09;g[3]=0x09;g[4]=0x01;break;
    case ':': g[0]=0x00;g[1]=0x36;g[2]=0x36;g[3]=0x00;g[4]=0x00;break;
    case '-': g[0]=0x08;g[1]=0x08;g[2]=0x08;g[3]=0x08;g[4]=0x08;break;
    case '.': g[0]=0x00;g[1]=0x60;g[2]=0x60;g[3]=0x00;g[4]=0x00;break;
    case '/': g[0]=0x20;g[1]=0x10;g[2]=0x08;g[3]=0x04;g[4]=0x02;break;
    }
    return g;
}

static void text(int x, int y, const char *s, uint16_t color)
{
    uint8_t px[5 * 7 * 2];
    while (*s) {
        const uint8_t *g = glyph(*s++);
        memset(px, 0, sizeof(px));
        for (int row = 0; row < 7; ++row) for (int col = 0; col < 5; ++col)
            if (g[col] & (1U << row)) { px[(row * 5 + col) * 2] = color >> 8; px[(row * 5 + col) * 2 + 1] = color; }
        set_window(x, y, x + 4, y + 6); data(px, sizeof(px));
        x += 6;
    }
}

bool tft_st7735_init(void)
{
    gpio_reset_pin(TFT_DC); gpio_set_direction(TFT_DC, GPIO_MODE_OUTPUT);
    gpio_reset_pin(TFT_CS); gpio_set_direction(TFT_CS, GPIO_MODE_OUTPUT);
    gpio_reset_pin(TFT_RST); gpio_set_direction(TFT_RST, GPIO_MODE_OUTPUT);
    spi_bus_config_t bus = {.sclk_io_num = TFT_SCLK, .mosi_io_num = TFT_MOSI,
        .miso_io_num = -1, .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = TFT_W * 2};
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "SPI init failed: %s", esp_err_to_name(err)); return false; }
    spi_device_interface_config_t cfg = {.clock_speed_hz = 8000000, .mode = 0, .spics_io_num = TFT_CS, .queue_size = 1};
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &cfg, &dev));
    gpio_set_level(TFT_RST, 0); vTaskDelay(pdMS_TO_TICKS(20)); gpio_set_level(TFT_RST, 1); vTaskDelay(pdMS_TO_TICKS(120));
    command(0x01); vTaskDelay(pdMS_TO_TICKS(120));
    command(0x11); vTaskDelay(pdMS_TO_TICKS(120));
    uint8_t madctl = 0xC8, color = 0x05; command(0x3A); data(&color, 1); command(0x36); data(&madctl, 1);
    command(0x29); vTaskDelay(pdMS_TO_TICKS(20));
    fill(0x0000);
    return true;
}

void tft_st7735_show(const tft_status_t *s)
{
    char line[32];
    fill(0x0000);
    if (s->distance_cm < 0.0f) text(2, 4, "DIST:---CM", 0xFFFF);
    else { snprintf(line, sizeof(line), "DIST:%3.0fCM", s->distance_cm); text(2, 4, line, 0xFFFF); }
    snprintf(line, sizeof(line), "IR:%u%u%u%u", s->ir_mask & 1, (s->ir_mask >> 1) & 1, (s->ir_mask >> 2) & 1, (s->ir_mask >> 3) & 1); text(2, 24, line, 0x07E0);
    snprintf(line, sizeof(line), "ERR:%d", s->error); text(2, 44, line, 0xFFE0);
    snprintf(line, sizeof(line), "TURN:%d", s->turn); text(2, 64, line, 0xFFE0);
    snprintf(line, sizeof(line), "A:%d B:%d", s->motor_a, s->motor_b); text(2, 84, line, 0xF81F);
    snprintf(line, sizeof(line), "D:%d", s->motor_d); text(2, 104, line, 0xF81F);
    text(2, 124, s->mode ? s->mode : "LINE", 0xFFFF);
}
