#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize GPIO for DHT22 (AM2302) 3-wire module in Open-Drain mode.
 * Call this once at startup to avoid repeated gpio_config calls.
 * @param gpio_num GPIO connected to DATA.
 */
esp_err_t dht22_init(int gpio_num);

/**
 * Read a DHT22 (AM2302) 3-wire module.
 * @param gpio_num GPIO connected to DATA.
 * @param temperature_c output temperature in Celsius.
 * @param humidity_pct output relative humidity in percent.
 */
esp_err_t dht22_read(int gpio_num, float *temperature_c, float *humidity_pct);

#ifdef __cplusplus
}
#endif
