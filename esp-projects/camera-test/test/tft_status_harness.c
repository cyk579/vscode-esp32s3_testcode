#include <assert.h>
#include <stdio.h>

#include "../main/tft_st7735.c"

/* Host-only panel: exercise the actual renderer and SPI output, not a second
 * font implementation. No framebuffer is added to the firmware. */
static uint16_t panel[TFT_ST7735_HEIGHT][TFT_ST7735_WIDTH];
static unsigned x0, x1, y0, y1, px, py;
static int dc;
static uint8_t command;

esp_err_t gpio_reset_pin(gpio_num_t pin) { return ESP_OK; }
esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode) { return ESP_OK; }
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level)
{ if (pin == TFT_DC) dc = (int)level; return ESP_OK; }
void vTaskDelay(TickType_t ticks) {}
esp_err_t spi_bus_initialize(int host, const spi_bus_config_t *cfg, int dma)
{ return ESP_OK; }
esp_err_t spi_bus_add_device(int host, const spi_device_interface_config_t *cfg,
                            spi_device_handle_t *device)
{ *device = (spi_device_handle_t)1; return ESP_OK; }

esp_err_t spi_device_transmit(spi_device_handle_t device, spi_transaction_t *tx)
{
    const uint8_t *data = tx->tx_buffer;
    const size_t bytes = tx->length / 8;
    if (!dc) {
        assert(bytes == 1);
        command = data[0];
        if (command == 0x2c) { px = x0; py = y0; }
    } else if (command == 0x2a || command == 0x2b) {
        assert(bytes == 4);
        const unsigned start = (unsigned)data[0] * 256 + data[1];
        const unsigned end = (unsigned)data[2] * 256 + data[3];
        if (command == 0x2a) { x0 = start; x1 = end; }
        else { y0 = start; y1 = end; }
        assert(x0 <= x1 && x1 < TFT_ST7735_WIDTH);
        assert(y0 <= y1 && y1 < TFT_ST7735_HEIGHT);
    } else if (command == 0x2c) {
        assert(bytes % 2 == 0);
        for (size_t i = 0; i < bytes; i += 2) {
            assert(px <= x1 && py <= y1);
            panel[py][px] = (uint16_t)((unsigned)data[i] * 256 + data[i + 1]);
            if (++px > x1) { px = x0; ++py; }
        }
    }
    return ESP_OK;
}

static int check_glyph(const char *name, unsigned row, unsigned column,
                       const uint8_t expected[7])
{
    for (unsigned y = 0; y < 7; ++y) {
        for (unsigned x = 0; x < 5; ++x) {
            const uint16_t color = (expected[y] & (1U << (4 - x))) ? 0xffff : 0;
            if (panel[TFT_TEXT_MARGIN_Y + row * 8 + y]
                     [TFT_TEXT_MARGIN_X + column * 6 + x] != color) {
                fprintf(stderr, "FAIL: %s is not rendered correctly\n", name);
                return 1;
            }
        }
    }
    return 0;
}

int main(void)
{
    const char *lines[] = {
        "STATE:FINDBALL", "STATE:T_FINISH", "STATE:AVOID_F", "STATE:BRAKE"
    };
    assert(tft_st7735_init());
    assert(tft_st7735_draw_text_lines(lines, 4, 0xffff, 0));
    int failures = 0;
    failures += check_glyph("FINDBALL/F", 0, 6,
                             (uint8_t[]){31,16,16,30,16,16,16});
    failures += check_glyph("T_FINISH/_", 1, 7,
                             (uint8_t[]){0,0,0,0,0,0,31});
    failures += check_glyph("AVOID/V", 2, 7,
                             (uint8_t[]){17,17,17,17,17,10,4});
    failures += check_glyph("BRAKE/K", 3, 9,
                             (uint8_t[]){17,18,20,24,20,18,17});
    const char *page[] = {
        "STATE:FINDBALL", "ARM:1 STBY:1", "M A:-27 B:0 D:27",
        "LAT:0 HEAD:0", "TURN:0", "CAM:25 CTRL:10", "DROP:0 CD:0",
        "US:--", "T:DONE 3-3"
    };
    assert(tft_st7735_draw_text_lines(page, 9, 0xffff, 0));
    failures += check_glyph("T result on ninth row", 8, 2,
                             (uint8_t[]){30,17,17,17,17,17,30});
    if (failures) return 1;
    puts("PASS: TFT renders FINDBALL, T_FINISH and obstacle state glyphs");
    return 0;
}
