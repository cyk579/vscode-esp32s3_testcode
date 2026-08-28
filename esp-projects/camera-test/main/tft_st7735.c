#include "tft_st7735.h"

#include <stddef.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TFT_SCLK GPIO_NUM_13
#define TFT_MOSI GPIO_NUM_14
#define TFT_DC   GPIO_NUM_21
#define TFT_CS   GPIO_NUM_47
#define TFT_RST  GPIO_NUM_38

#define TFT_SPI_CLOCK_HZ 20000000
#define TFT_MADCTL_LANDSCAPE 0xA8

static const char *TAG = "tft";
static spi_device_handle_t s_tft;
static bool s_initialized;

static esp_err_t write_bytes(const void *data, size_t len, int dc)
{
    gpio_set_level(TFT_DC, dc);
    spi_transaction_t transaction = {
        .length = len * 8,
        .tx_buffer = data,
    };
    return spi_device_transmit(s_tft, &transaction);
}

static esp_err_t write_command(uint8_t command)
{
    return write_bytes(&command, 1, 0);
}

static esp_err_t write_data(const uint8_t *data, size_t len)
{
    return write_bytes(data, len, 1);
}

static esp_err_t set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t values[4];
    esp_err_t err;

    err = write_command(0x2A);
    if (err != ESP_OK) {
        return err;
    }
    values[0] = x0 >> 8;
    values[1] = x0 & 0xFF;
    values[2] = x1 >> 8;
    values[3] = x1 & 0xFF;
    err = write_data(values, sizeof(values));
    if (err != ESP_OK) {
        return err;
    }

    err = write_command(0x2B);
    if (err != ESP_OK) {
        return err;
    }
    values[0] = y0 >> 8;
    values[1] = y0 & 0xFF;
    values[2] = y1 >> 8;
    values[3] = y1 & 0xFF;
    err = write_data(values, sizeof(values));
    if (err != ESP_OK) {
        return err;
    }
    return write_command(0x2C);
}

bool tft_st7735_fill(uint16_t color)
{
    if (!s_initialized) {
        return false;
    }

    uint8_t line[TFT_ST7735_WIDTH * 2];
    for (size_t x = 0; x < TFT_ST7735_WIDTH; ++x) {
        line[x * 2] = color >> 8;
        line[x * 2 + 1] = color & 0xFF;
    }

    if (set_window(0, 0, TFT_ST7735_WIDTH - 1, TFT_ST7735_HEIGHT - 1) != ESP_OK) {
        return false;
    }
    for (size_t y = 0; y < TFT_ST7735_HEIGHT; ++y) {
        if (write_data(line, sizeof(line)) != ESP_OK) {
            return false;
        }
    }
    return true;
}

bool tft_st7735_draw_rgb565(const uint8_t *rgb565_big_endian,
                             uint16_t width,
                             uint16_t height)
{
    if (!s_initialized || rgb565_big_endian == NULL || width == 0 || height == 0 ||
        width > TFT_ST7735_WIDTH || height > TFT_ST7735_HEIGHT) {
        return false;
    }

    const uint16_t x0 = (TFT_ST7735_WIDTH - width) / 2;
    const uint16_t y0 = (TFT_ST7735_HEIGHT - height) / 2;
    if (set_window(x0, y0, x0 + width - 1, y0 + height - 1) != ESP_OK) {
        return false;
    }

    const size_t line_bytes = (size_t)width * 2;
    for (size_t y = 0; y < height; ++y) {
        if (write_data(rgb565_big_endian + y * line_bytes, line_bytes) != ESP_OK) {
            return false;
        }
    }
    return true;
}

bool tft_st7735_init(void)
{
    if (s_initialized) {
        return true;
    }

    gpio_reset_pin(TFT_DC);
    gpio_set_direction(TFT_DC, GPIO_MODE_OUTPUT);
    gpio_reset_pin(TFT_CS);
    gpio_set_direction(TFT_CS, GPIO_MODE_OUTPUT);
    gpio_reset_pin(TFT_RST);
    gpio_set_direction(TFT_RST, GPIO_MODE_OUTPUT);

    const spi_bus_config_t bus_config = {
        .sclk_io_num = TFT_SCLK,
        .mosi_io_num = TFT_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TFT_ST7735_WIDTH * 2,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }

    const spi_device_interface_config_t device_config = {
        .clock_speed_hz = TFT_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = TFT_CS,
        .queue_size = 1,
    };
    err = spi_bus_add_device(SPI2_HOST, &device_config, &s_tft);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device init failed: %s", esp_err_to_name(err));
        return false;
    }

    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    uint8_t color_mode = 0x05;  // RGB565
    uint8_t madctl = TFT_MADCTL_LANDSCAPE;
    if (write_command(0x01) != ESP_OK) {
        ESP_LOGE(TAG, "ST7735 software reset failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(120));
    if (write_command(0x11) != ESP_OK) {
        ESP_LOGE(TAG, "ST7735 sleep-out command failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(120));
    if (write_command(0x3A) != ESP_OK ||
        write_data(&color_mode, 1) != ESP_OK ||
        write_command(0x36) != ESP_OK ||
        write_data(&madctl, 1) != ESP_OK ||
        write_command(0x29) != ESP_OK) {
        ESP_LOGE(TAG, "ST7735 init command failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    s_initialized = true;
    if (!tft_st7735_fill(0x0000)) {
        ESP_LOGE(TAG, "ST7735 clear failed");
        s_initialized = false;
        return false;
    }
    ESP_LOGI(TAG, "ST7735 initialized on SPI2 (%dx%d landscape)",
             TFT_ST7735_WIDTH, TFT_ST7735_HEIGHT);
    return true;
}
