#include "dht22.h"

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static portMUX_TYPE dht_spinlock = portMUX_INITIALIZER_UNLOCKED;
static int s_configured_pin = -1;

static bool wait_for_level(gpio_num_t pin, int level, int timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) {
            return false;
        }
    }
    return true;
}

static int64_t wait_level_duration(gpio_num_t pin, int level, int timeout_us)
{
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) == level) {
        if ((esp_timer_get_time() - start) > timeout_us) {
            return -1;
        }
    }
    return esp_timer_get_time() - start;
}

esp_err_t dht22_init(int gpio_num)
{
    gpio_num_t pin = (gpio_num_t)gpio_num;

    // Cấu hình Open-Drain hai chiều: vừa Output vừa Input, có trở kéo Pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t cfg_err = gpio_config(&io_conf);
    if (cfg_err == ESP_OK) {
        s_configured_pin = gpio_num;
        gpio_set_level(pin, 1); // Trạng thái nghỉ: nhả bus lên mức cao
    }
    return cfg_err;
}

esp_err_t dht22_read(int gpio_num, float *temperature_c, float *humidity_pct)
{
    if (!temperature_c || !humidity_pct) {
        return ESP_ERR_INVALID_ARG;
    }

    // Chỉ cấu hình GPIO nếu chân chưa từng được khởi tạo
    if (s_configured_pin != gpio_num) {
        esp_err_t cfg_err = dht22_init(gpio_num);
        if (cfg_err != ESP_OK) {
            return cfg_err;
        }
    }

    gpio_num_t pin = (gpio_num_t)gpio_num;

    // 1. Gửi Start Signal: Kéo LOW >= 18ms (dùng 20ms cho ổn định)
    gpio_set_level(pin, 0);
    esp_rom_delay_us(20000);

    // 2. Nhả bus về mức HIGH (trở pull-up kéo lên)
    gpio_set_level(pin, 1);
    esp_rom_delay_us(30);

    // 3. BẮT ĐẦU CRITICAL SECTION: Chỉ bao bọc phần đọc xung nhạy cảm với thời gian
    portENTER_CRITICAL(&dht_spinlock);

    // Cảm biến phản hồi: kéo LOW ~80us rồi HIGH ~80us
    if (!wait_for_level(pin, 0, 100)) {
        portEXIT_CRITICAL(&dht_spinlock);
        return ESP_ERR_TIMEOUT;
    }
    if (wait_level_duration(pin, 0, 120) < 0) {
        portEXIT_CRITICAL(&dht_spinlock);
        return ESP_ERR_TIMEOUT;
    }
    if (wait_level_duration(pin, 1, 120) < 0) {
        portEXIT_CRITICAL(&dht_spinlock);
        return ESP_ERR_TIMEOUT;
    }

    // 4. Đọc 40 bit dữ liệu (5 bytes)
    uint8_t data[5] = {0};

    for (int i = 0; i < 40; ++i) {
        // Mỗi bit bắt đầu bằng khoảng ~50us LOW
        if (wait_level_duration(pin, 0, 100) < 0) {
            portEXIT_CRITICAL(&dht_spinlock);
            return ESP_ERR_TIMEOUT;
        }

        // Độ dài mức HIGH quyết định bit 0 (~26-28us) hay bit 1 (~70us)
        int64_t high_us = wait_level_duration(pin, 1, 120);
        if (high_us < 0) {
            portEXIT_CRITICAL(&dht_spinlock);
            return ESP_ERR_TIMEOUT;
        }

        data[i / 8] <<= 1;
        if (high_us > 45) {
            data[i / 8] |= 1;
        }
    }

    // Kết thúc critical section ngay sau khi nhận đủ 40 bit
    portEXIT_CRITICAL(&dht_spinlock);
    gpio_set_level(pin, 1); // Đảm bảo nhả bus về trạng thái nghỉ (idle HIGH)

    // 5. Kiểm tra Checksum
    uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_h = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_t = ((uint16_t)(data[2] & 0x7F) << 8) | data[3];

    float h = raw_h / 10.0f;
    float t = raw_t / 10.0f;
    if (data[2] & 0x80) {
        t = -t;
    }

    if (h < 0.0f || h > 100.0f || t < -40.0f || t > 80.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *humidity_pct = h;
    *temperature_c = t;
    return ESP_OK;
}