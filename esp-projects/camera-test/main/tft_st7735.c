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

#define TFT_SPI_CLOCK_HZ 8000000
/* MX+MY+MV landscape; this panel uses RGB order (BGR bit cleared). */
#define TFT_MADCTL_LANDSCAPE 0xA0
#define TFT_TEXT_MARGIN_X 4U
#define TFT_TEXT_MARGIN_Y 4U

static const char *TAG = "tft";
static spi_device_handle_t s_tft;
static bool s_initialized;

/* Small 5x7 ASCII subset used by the diagnostic page.  Rows are packed with
 * bit 4 at the left; the renderer streams one TFT row at a time and never
 * allocates a full-screen buffer. */
static const uint8_t s_font[][7] = {
    {0x0e, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11}, /* A */
    {0x1e, 0x11, 0x11, 0x1e, 0x11, 0x11, 0x1e}, /* B */
    {0x0e, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0e}, /* C */
    {0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e}, /* D */
    {0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f}, /* E */
    {0x0e, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0f}, /* G */
    {0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11}, /* H */
    {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1f}, /* I */
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f}, /* L */
    {0x11, 0x1b, 0x15, 0x15, 0x11, 0x11, 0x11}, /* M */
    {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}, /* N */
    {0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e}, /* O */
    {0x1e, 0x11, 0x11, 0x1e, 0x10, 0x10, 0x10}, /* P */
    {0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11}, /* R */
    {0x0f, 0x10, 0x10, 0x0e, 0x01, 0x01, 0x1e}, /* S */
    {0x1f, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}, /* T */
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e}, /* U */
    {0x11, 0x11, 0x0a, 0x04, 0x04, 0x04, 0x04}, /* Y */
    {0x0e, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0e}, /* 0 */
    {0x04, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x0e}, /* 1 */
    {0x0e, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1f}, /* 2 */
    {0x1e, 0x01, 0x01, 0x0e, 0x01, 0x01, 0x1e}, /* 3 */
    {0x02, 0x06, 0x0a, 0x12, 0x1f, 0x02, 0x02}, /* 4 */
    {0x1f, 0x10, 0x10, 0x1e, 0x01, 0x01, 0x1e}, /* 5 */
    {0x0e, 0x10, 0x10, 0x1e, 0x11, 0x11, 0x0e}, /* 6 */
    {0x1f, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}, /* 7 */
    {0x0e, 0x11, 0x11, 0x0e, 0x11, 0x11, 0x0e}, /* 8 */
    {0x0e, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x0e}, /* 9 */
    {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00}, /* : */
    {0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00}, /* - */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* space */
    {0x1f, 0x11, 0x05, 0x02, 0x04, 0x00, 0x04}, /* fallback */
};

static const uint8_t *font_glyph(char ch)
{
    if (ch >= 'a' && ch <= 'z') {
        ch = (char)(ch - ('a' - 'A'));
    }
    switch (ch) {
    case 'A': return s_font[0];
    case 'B': return s_font[1];
    case 'C': return s_font[2];
    case 'D': return s_font[3];
    case 'E': return s_font[4];
    case 'G': return s_font[5];
    case 'H': return s_font[6];
    case 'I': return s_font[7];
    case 'L': return s_font[8];
    case 'M': return s_font[9];
    case 'N': return s_font[10];
    case 'O': return s_font[11];
    case 'P': return s_font[12];
    case 'R': return s_font[13];
    case 'S': return s_font[14];
    case 'T': return s_font[15];
    case 'U': return s_font[16];
    case 'Y': return s_font[17];
    case '0': return s_font[18];
    case '1': return s_font[19];
    case '2': return s_font[20];
    case '3': return s_font[21];
    case '4': return s_font[22];
    case '5': return s_font[23];
    case '6': return s_font[24];
    case '7': return s_font[25];
    case '8': return s_font[26];
    case '9': return s_font[27];
    case ':': return s_font[28];
    case '-': return s_font[29];
    case ' ': return s_font[30];
    default: return s_font[31];
    }
}

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

bool tft_st7735_draw_rgb565_2x(const uint8_t *rgb565_big_endian,
                              uint16_t source_width,
                              uint16_t source_height)
{
    const uint16_t width = source_width / 2;
    const uint16_t height = source_height / 2;
    if (!s_initialized || rgb565_big_endian == NULL || width == 0 || height == 0 ||
        width > TFT_ST7735_WIDTH || height > TFT_ST7735_HEIGHT) {
        return false;
    }

    const uint16_t x0 = (TFT_ST7735_WIDTH - width) / 2;
    const uint16_t y0 = (TFT_ST7735_HEIGHT - height) / 2;
    if (set_window(x0, y0, x0 + width - 1, y0 + height - 1) != ESP_OK) {
        return false;
    }

    uint8_t line[TFT_ST7735_WIDTH * 2];
    for (uint16_t y = 0; y < height; ++y) {
        const uint8_t *source = rgb565_big_endian +
                                (size_t)(y * 2) * source_width * 2;
        for (uint16_t x = 0; x < width; ++x) {
            line[x * 2] = source[x * 4];
            line[x * 2 + 1] = source[x * 4 + 1];
        }
        if (write_data(line, (size_t)width * 2) != ESP_OK) {
            return false;
        }
    }
    return true;
}

bool tft_st7735_draw_rgb565_2x_crop(const uint8_t *rgb565_big_endian,
                                    uint16_t source_width,
                                    uint16_t source_height,
                                    uint16_t crop_left,
                                    uint16_t crop_top,
                                    uint16_t crop_right,
                                    uint16_t crop_bottom,
                                    uint16_t blank_color)
{
    const uint16_t width = source_width / 2;
    const uint16_t height = source_height / 2;
    if (!s_initialized || rgb565_big_endian == NULL || width == 0 || height == 0 ||
        width > TFT_ST7735_WIDTH || height > TFT_ST7735_HEIGHT) {
        return false;
    }

    if (crop_left >= source_width) crop_left = source_width - 1;
    if (crop_top >= source_height) crop_top = source_height - 1;
    if (crop_right >= source_width) crop_right = source_width - 1;
    if (crop_bottom >= source_height) crop_bottom = source_height - 1;
    if (crop_left > crop_right || crop_top > crop_bottom) {
        return false;
    }

    const uint16_t x0 = (TFT_ST7735_WIDTH - width) / 2;
    const uint16_t y0 = (TFT_ST7735_HEIGHT - height) / 2;
    if (set_window(x0, y0, x0 + width - 1, y0 + height - 1) != ESP_OK) {
        return false;
    }

    uint8_t line[TFT_ST7735_WIDTH * 2];
    const uint8_t blank_hi = (uint8_t)(blank_color >> 8);
    const uint8_t blank_lo = (uint8_t)blank_color;
    for (uint16_t y = 0; y < height; ++y) {
        const uint16_t source_y = y * 2;
        const bool row_visible = source_y >= crop_top && source_y <= crop_bottom;
        const uint8_t *source = rgb565_big_endian +
                                (size_t)source_y * source_width * 2;
        for (uint16_t x = 0; x < width; ++x) {
            const uint16_t source_x = x * 2;
            const size_t out = (size_t)x * 2;
            if (row_visible && source_x >= crop_left && source_x <= crop_right) {
                const uint8_t *pixel = source + (size_t)source_x * 2;
                line[out] = pixel[0];
                line[out + 1] = pixel[1];
            } else {
                line[out] = blank_hi;
                line[out + 1] = blank_lo;
            }
        }
        if (write_data(line, (size_t)width * 2) != ESP_OK) {
            return false;
        }
    }
    return true;
}

bool tft_st7735_draw_text_lines(const char *const lines[],
                                size_t line_count,
                                uint16_t foreground,
                                uint16_t background)
{
    if (!s_initialized || lines == NULL) {
        return false;
    }

    const size_t rows = (TFT_ST7735_HEIGHT - TFT_TEXT_MARGIN_Y) / 8;
    const size_t columns = (TFT_ST7735_WIDTH - TFT_TEXT_MARGIN_X) / 6;
    if (line_count > rows) {
        line_count = rows;
    }
    const size_t draw_rows = line_count;
    uint8_t line[TFT_ST7735_WIDTH * 2];
    for (size_t text_row = 0; text_row < draw_rows; ++text_row) {
        const uint16_t y0 = (uint16_t)(TFT_TEXT_MARGIN_Y + text_row * 8U);
        if (set_window(0, y0, TFT_ST7735_WIDTH - 1, y0 + 7U) != ESP_OK) {
            return false;
        }
        const char *text = lines[text_row];
        for (uint8_t glyph_row = 0; glyph_row < 8U; ++glyph_row) {
            for (size_t x = 0; x < TFT_ST7735_WIDTH; ++x) {
                line[x * 2] = (uint8_t)(background >> 8);
                line[x * 2 + 1] = (uint8_t)background;
            }
            if (text != NULL && glyph_row < 7U) {
                for (size_t column = 0; column < columns; ++column) {
                    const char ch = text[column];
                    if (ch == '\0') {
                        break;
                    }
                    const uint8_t *glyph = font_glyph(ch);
                    const uint8_t bits = glyph[glyph_row];
                    for (size_t glyph_x = 0; glyph_x < 5; ++glyph_x) {
                        if ((bits & (uint8_t)(1U << (4U - glyph_x))) == 0) {
                            continue;
                        }
                        const size_t x = TFT_TEXT_MARGIN_X + column * 6 + glyph_x;
                        if (x < TFT_ST7735_WIDTH) {
                            line[x * 2] = (uint8_t)(foreground >> 8);
                            line[x * 2 + 1] = (uint8_t)foreground;
                        }
                    }
                }
            }
            if (write_data(line, sizeof(line)) != ESP_OK) {
                return false;
            }
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
