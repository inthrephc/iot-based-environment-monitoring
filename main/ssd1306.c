#include "ssd1306.h"

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static const uint8_t font5x7_digits[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
};

static const uint8_t font5x7_upper[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x04,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
};

static void font_for_char(char c, uint8_t out[5])
{
    memset(out, 0, 5);
    if (c >= '0' && c <= '9') {
        memcpy(out, font5x7_digits[c - '0'], 5);
        return;
    }
    if (c >= 'A' && c <= 'Z') {
        memcpy(out, font5x7_upper[c - 'A'], 5);
        return;
    }
    switch (c) {
        case ':': out[1]=0x36; out[3]=0x36; break;
        case '.': out[2]=0x60; break;
        case '%': out[0]=0x63; out[2]=0x08; out[4]=0x63; break;
        case '-': out[1]=0x08; out[2]=0x08; out[3]=0x08; break;
        case '/': out[4]=0x03; out[3]=0x0C; out[2]=0x30; out[1]=0x60; break;
        case '>': out[0]=0x41; out[1]=0x22; out[2]=0x14; out[3]=0x08; break;
        case '_': out[4]=0x80; break;
        case ' ': default: break;
    }
}

static esp_err_t ssd1306_cmd(uint8_t cmd)
{
    uint8_t packet[2] = {0x00, cmd};
    return i2c_master_transmit(s_dev, packet, sizeof(packet), 100);
}

static esp_err_t ssd1306_cmds(const uint8_t *cmds, size_t n)
{
    uint8_t packet[33];
    if (n > 32) return ESP_ERR_INVALID_SIZE;
    packet[0] = 0x00;
    memcpy(&packet[1], cmds, n);
    return i2c_master_transmit(s_dev, packet, n + 1, 100);
}

esp_err_t ssd1306_init(i2c_master_bus_handle_t bus, uint8_t address)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &s_dev);
    if (err != ESP_OK) return err;

    static const uint8_t init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00,
        0x40, 0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40,
        0xA4, 0xA6, 0xAF
    };
    err = ssd1306_cmds(init_seq, sizeof(init_seq));
    if (err != ESP_OK) return err;
    ssd1306_clear();
    ssd1306_update();
    return ESP_OK;
}

void ssd1306_clear(void)
{
    memset(s_buffer, 0, sizeof(s_buffer));
}

static void set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) return;
    size_t idx = (size_t)x + (size_t)(y / 8) * SSD1306_WIDTH;
    uint8_t mask = 1U << (y % 8);
    if (on) s_buffer[idx] |= mask;
    else s_buffer[idx] &= (uint8_t)~mask;
}

static void draw_char(int x, int y, char c, bool scale2)
{
    uint8_t glyph[5];
    font_for_char(c, glyph);
    int scale = scale2 ? 2 : 1;
    for (int col = 0; col < 5; ++col) {
        for (int row = 0; row < 7; ++row) {
            bool on = (glyph[col] >> row) & 1U;
            if (scale == 1) set_pixel(x + col, y + row, on);
            else {
                for (int dx = 0; dx < 2; ++dx)
                    for (int dy = 0; dy < 2; ++dy)
                        set_pixel(x + col*2 + dx, y + row*2 + dy, on);
            }
        }
    }
}

void ssd1306_draw_text(int x, int y, const char *text, bool scale2)
{
    int step = scale2 ? 12 : 6;
    while (*text && x < SSD1306_WIDTH) {
        draw_char(x, y, *text++, scale2);
        x += step;
    }
}

void ssd1306_update(void)
{
    for (uint8_t page = 0; page < 8; ++page) {
        uint8_t cmds[] = {(uint8_t)(0xB0 | page), 0x00, 0x10};
        if (ssd1306_cmds(cmds, sizeof(cmds)) != ESP_OK) return;

        uint8_t packet[129];
        packet[0] = 0x40;
        memcpy(&packet[1], &s_buffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
        if (i2c_master_transmit(s_dev, packet, sizeof(packet), 100) != ESP_OK) return;
    }
}
