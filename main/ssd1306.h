#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

esp_err_t ssd1306_init(i2c_master_bus_handle_t bus, uint8_t address);
void ssd1306_clear(void);
void ssd1306_draw_text(int x, int y, const char *text, bool scale2);
void ssd1306_update(void);

#ifdef __cplusplus
}
#endif
