#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/adc_channel.h"
#include "esp_rom_sys.h"

#include "dht22.h"
#include "ssd1306.h"

// =============================
// Hardware pin map (ESP32 WROOM-32, 30-pin DevKit)
// =============================
#define PIN_DHT22       GPIO_NUM_4
#define PIN_MQ135_AO    GPIO_NUM_34
#define PIN_OLED_SDA    GPIO_NUM_21
#define PIN_OLED_SCL    GPIO_NUM_22
#define PIN_RELAY       GPIO_NUM_16
#define PIN_BUZZER      GPIO_NUM_17
#define PIN_LED_RED     GPIO_NUM_25
#define PIN_LED_GREEN   GPIO_NUM_26
#define PIN_BUTTON      GPIO_NUM_27

#define I2C_PORT        I2C_NUM_0
#define OLED_ADDR_PRIMARY   0x3C
#define OLED_ADDR_SECONDARY 0x3D
#define ADC_UNIT_USED   ADC_UNIT_1
#define ADC_CHANNEL_MQ  ADC_CHANNEL_6   // GPIO34 on classic ESP32

// Relay modules vary. Set 1 for the common LOW-level trigger module.
#define RELAY_ACTIVE_LOW 0

// =============================
// Project configuration
// =============================
#define WIFI_SSID       "Ky Tuc Xa DHFPT"
#define WIFI_PASSWORD   ""

#define SAMPLE_PERIOD_MS       2500
#define OLED_PAGE_PERIOD_MS    5000
#define MQ_WARMUP_MS           60000
#define BUTTON_DEBOUNCE_MS     40
#define BUTTON_LONG_MS         1500
#define SNOOZE_MS              60000

// =============================
// Motor & Relay Protection Configuration
// =============================
#define FAN_MIN_RUN_TIME_MS    60000   // Chạy tối thiểu 60s khi đã bật để bảo vệ cuộn dây và tản nhiệt
#define FAN_MIN_REST_TIME_MS   60000   // Nghỉ tối thiểu 60s khi đã tắt trước khi bật lại để bảo vệ tụ đề & relay
#define FAN_TRIGGER_PERSIST    2       // Cần 2 mẫu liên tiếp (5s) ở Level 3 mới kích hoạt quạt (chống giật nhiễu)
#define TEMP_HYSTERESIS_C      0.6f    // Độ trễ nhiệt độ khi phục hồi về mức thấp hơn
#define HUM_HYSTERESIS_PCT     3.0f    // Độ trễ độ ẩm khi phục hồi về mức thấp hơn

#define NVS_NAMESPACE          "envmon"
#define NVS_KEY_THRESHOLDS     "thresholds"
#define NVS_KEY_MQ_R0          "mq_r0"
#define CONFIG_VERSION         1

#define MQ_DEFAULT_R0          40.0f    // Default R0 in kOhm for clean air
#define MQ_FILTER_SAMPLES      20       // 20 * 2.5s = 50 seconds moving average
#define MQ_RL_KOHM             10.0f    // Load resistor on standard MQ breakout (~10 kOhm)

static const char *TAG = "ENV_MON";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_wifi_retry_count = 0;

static SemaphoreHandle_t s_state_mutex;
static SemaphoreHandle_t s_oled_mutex;

static adc_oneshot_unit_handle_t s_adc_handle;
static i2c_master_bus_handle_t s_i2c_bus;
static httpd_handle_t s_http_server = NULL;
static bool s_oled_ready = false;

static volatile bool s_selftest_requested = false;
static volatile bool s_mq_calibrate_requested = false;

// =============================
// Data structures
// =============================
typedef enum {
    ENV_LEVEL_1_NORMAL = 1,
    ENV_LEVEL_2_NOTICE = 2,
    ENV_LEVEL_3_WARNING = 3,
    ENV_LEVEL_4_CRITICAL = 4,
} env_level_t;

typedef enum {
    FAN_AUTO = 0,
    FAN_FORCE_ON,
    FAN_FORCE_OFF,
} fan_mode_t;

typedef struct {
    uint32_t version;
    float fan_temp_on;
    float fan_temp_off;
    float fan_hum_on;
    float fan_hum_off;
    int fan_gas_on;
    int fan_gas_off;
    float crit_temp;
    float crit_hum;
    int crit_gas;
} thresholds_t;

typedef struct {
    float temperature_c;
    float humidity_pct;
    float heat_index_c;
    float mold_risk_pct;
    int comfort_level;
    int mq_raw;
    float mq_rs;
    float mq_r0;
    float mq_ratio_k;
    env_level_t temp_level;
    env_level_t hum_level;
    env_level_t mq_level;
    bool dht_valid;
    bool fan_on;
    bool fan_locked;           // Đang trong thời gian bảo vệ Min-Run hoặc Min-Rest
    int fan_protect_rem_s;     // Thời gian bảo vệ còn lại (giây)
    bool alarm;
    bool warning;
    env_level_t env_level;
    bool wifi_connected;
    int wifi_rssi;
    bool selftest_active;
    int64_t snooze_until_us;
    int64_t boot_us;
    bool warmup_done;
    fan_mode_t fan_mode;
} app_state_t;

static thresholds_t g_thr = {
    .version = CONFIG_VERSION,
    .fan_temp_on = 30.0f,
    .fan_temp_off = 28.0f,
    .fan_hum_on = 70.0f,
    .fan_hum_off = 65.0f,
    // These are raw ADC values, NOT ppm. Tune after MQ-135 warm-up/calibration.
    .fan_gas_on = 1800,
    .fan_gas_off = 1500,
    .crit_temp = 35.0f,
    .crit_hum = 85.0f,
    .crit_gas = 2500,
};

static app_state_t g_state = {
    .temperature_c = NAN,
    .humidity_pct = NAN,
    .heat_index_c = NAN,
    .mold_risk_pct = NAN,
    .comfort_level = 0,
    .mq_raw = 0,
    .mq_rs = 0.0f,
    .mq_r0 = MQ_DEFAULT_R0,
    .mq_ratio_k = 1.0f,
    .temp_level = ENV_LEVEL_1_NORMAL,
    .hum_level = ENV_LEVEL_1_NORMAL,
    .mq_level = ENV_LEVEL_1_NORMAL,
    .dht_valid = false,
    .fan_on = false,
    .fan_locked = false,
    .fan_protect_rem_s = 0,
    .alarm = false,
    .warning = false,
    .env_level = ENV_LEVEL_1_NORMAL,
    .wifi_connected = false,
    .wifi_rssi = 0,
    .selftest_active = false,
    .snooze_until_us = 0,
    .boot_us = 0,
    .warmup_done = false,
    .fan_mode = FAN_AUTO,
};

static float s_mq_r0 = MQ_DEFAULT_R0;
static float s_mq_samples[MQ_FILTER_SAMPLES] = {0};
static int s_mq_sample_count = 0;
static int s_mq_sample_idx = 0;

static env_level_t s_temp_prev_level = ENV_LEVEL_1_NORMAL;
static env_level_t s_hum_prev_level = ENV_LEVEL_1_NORMAL;
static env_level_t s_mq_prev_level = ENV_LEVEL_1_NORMAL;

static int64_t s_fan_last_on_us = 0;
static int64_t s_fan_last_off_us = -((int64_t)FAN_MIN_REST_TIME_MS * 1000);
static bool s_fan_is_on = false;
static int s_warning_persist_count = 0;

// =============================
// Utility
// =============================
static void lock_state(void) { xSemaphoreTake(s_state_mutex, portMAX_DELAY); }
static void unlock_state(void) { xSemaphoreGive(s_state_mutex); }

static const char *fan_mode_str(fan_mode_t mode)
{
    switch (mode) {
        case FAN_FORCE_ON: return "ON";
        case FAN_FORCE_OFF: return "OFF";
        default: return "AUTO";
    }
}

static const char *env_level_str(env_level_t lvl)
{
    switch (lvl) {
        case ENV_LEVEL_4_CRITICAL: return "CRIT";
        case ENV_LEVEL_3_WARNING:  return "WARN";
        case ENV_LEVEL_2_NOTICE:   return "NOTICE";
        default:                   return "NORM";
    }
}

// Xác định mức nhiệt độ kèm Hysteresis (0.6°C) chống rung ngưỡng
static env_level_t get_temp_level(float t, env_level_t prev_lvl)
{
    if (!isfinite(t)) return ENV_LEVEL_1_NORMAL;
    const float hys = TEMP_HYSTERESIS_C;

    switch (prev_lvl) {
        case ENV_LEVEL_4_CRITICAL:
            if (t <= 33.0f - hys && t >= 21.0f + hys) {
                if (t <= 31.0f - hys && t >= 23.0f + hys) {
                    if (t <= 29.0f - hys && t >= 25.0f + hys) return ENV_LEVEL_1_NORMAL;
                    return ENV_LEVEL_2_NOTICE;
                }
                return ENV_LEVEL_3_WARNING;
            }
            return ENV_LEVEL_4_CRITICAL;

        case ENV_LEVEL_3_WARNING:
            if (t < 21.0f || t > 33.0f) return ENV_LEVEL_4_CRITICAL;
            if (t <= 31.0f - hys && t >= 23.0f + hys) {
                if (t <= 29.0f - hys && t >= 25.0f + hys) return ENV_LEVEL_1_NORMAL;
                return ENV_LEVEL_2_NOTICE;
            }
            return ENV_LEVEL_3_WARNING;

        case ENV_LEVEL_2_NOTICE:
            if (t < 21.0f || t > 33.0f) return ENV_LEVEL_4_CRITICAL;
            if (t < 23.0f || t > 31.0f) return ENV_LEVEL_3_WARNING;
            if (t <= 29.0f - hys && t >= 25.0f + hys) {
                return ENV_LEVEL_1_NORMAL;
            }
            return ENV_LEVEL_2_NOTICE;

        case ENV_LEVEL_1_NORMAL:
        default:
            if (t < 21.0f || t > 33.0f) return ENV_LEVEL_4_CRITICAL;
            if (t < 23.0f || t > 31.0f) return ENV_LEVEL_3_WARNING;
            if (t < 25.0f || t > 29.0f) return ENV_LEVEL_2_NOTICE;
            return ENV_LEVEL_1_NORMAL;
    }
}

// Xác định mức độ ẩm kèm Hysteresis (3.0%) chống rung ngưỡng
static env_level_t get_hum_level(float h, env_level_t prev_lvl)
{
    if (!isfinite(h)) return ENV_LEVEL_1_NORMAL;
    const float hys = HUM_HYSTERESIS_PCT;

    switch (prev_lvl) {
        case ENV_LEVEL_4_CRITICAL:
            if (h <= 85.0f - hys && h >= 30.0f + hys) {
                if (h <= 75.0f - hys && h >= 40.0f + hys) {
                    if (h <= 70.0f - hys && h >= 50.0f + hys) return ENV_LEVEL_1_NORMAL;
                    return ENV_LEVEL_2_NOTICE;
                }
                return ENV_LEVEL_3_WARNING;
            }
            return ENV_LEVEL_4_CRITICAL;

        case ENV_LEVEL_3_WARNING:
            if (h < 30.0f || h > 85.0f) return ENV_LEVEL_4_CRITICAL;
            if (h <= 75.0f - hys && h >= 40.0f + hys) {
                if (h <= 70.0f - hys && h >= 50.0f + hys) return ENV_LEVEL_1_NORMAL;
                return ENV_LEVEL_2_NOTICE;
            }
            return ENV_LEVEL_3_WARNING;

        case ENV_LEVEL_2_NOTICE:
            if (h < 30.0f || h > 85.0f) return ENV_LEVEL_4_CRITICAL;
            if (h < 40.0f || h > 75.0f) return ENV_LEVEL_3_WARNING;
            if (h <= 70.0f - hys && h >= 50.0f + hys) {
                return ENV_LEVEL_1_NORMAL;
            }
            return ENV_LEVEL_2_NOTICE;

        case ENV_LEVEL_1_NORMAL:
        default:
            if (h < 30.0f || h > 85.0f) return ENV_LEVEL_4_CRITICAL;
            if (h < 40.0f || h > 75.0f) return ENV_LEVEL_3_WARNING;
            if (h < 50.0f || h > 70.0f) return ENV_LEVEL_2_NOTICE;
            return ENV_LEVEL_1_NORMAL;
    }
}

// Tính điện trở cảm biến Rs (kOhm) từ ADC raw
static float mq135_calculate_rs(int raw)
{
    if (raw < 15) raw = 15;
    if (raw > 4080) raw = 4080;
    return MQ_RL_KOHM * (4095.0f - (float)raw) / (float)raw;
}

// Bộ lọc trung bình trượt 50s (20 mẫu x 2.5s) cho tỷ số K = Rs/R0
static float mq_filter_update(float new_rs, float r0)
{
    if (r0 <= 0.1f) r0 = MQ_DEFAULT_R0;
    float new_k = new_rs / r0;

    // Khởi tạo buffer tức thì ở mẫu đầu tiên để tránh trễ khởi động
    if (s_mq_sample_count == 0) {
        for (int i = 0; i < MQ_FILTER_SAMPLES; i++) {
            s_mq_samples[i] = new_k;
        }
        s_mq_sample_count = MQ_FILTER_SAMPLES;
        return new_k;
    }

    s_mq_samples[s_mq_sample_idx] = new_k;
    s_mq_sample_idx = (s_mq_sample_idx + 1) % MQ_FILTER_SAMPLES;

    float sum = 0.0f;
    for (int i = 0; i < MQ_FILTER_SAMPLES; i++) {
        sum += s_mq_samples[i];
    }
    return sum / (float)MQ_FILTER_SAMPLES;
}

// Xác định 4 mức chất lượng không khí của MQ-135 kèm Hysteresis (0.02)
static env_level_t get_mq_level(float k, env_level_t prev_lvl)
{
    const float hys = 0.02f; // Trễ phục hồi để chống dao động chập chờn

    switch (prev_lvl) {
        case ENV_LEVEL_4_CRITICAL:
            if (k >= 0.45f + hys) {
                if (k >= 0.65f + hys) {
                    if (k >= 0.85f + hys) return ENV_LEVEL_1_NORMAL;
                    return ENV_LEVEL_2_NOTICE;
                }
                return ENV_LEVEL_3_WARNING;
            }
            return ENV_LEVEL_4_CRITICAL;

        case ENV_LEVEL_3_WARNING:
            if (k < 0.45f) return ENV_LEVEL_4_CRITICAL;
            if (k >= 0.65f + hys) {
                if (k >= 0.85f + hys) return ENV_LEVEL_1_NORMAL;
                return ENV_LEVEL_2_NOTICE;
            }
            return ENV_LEVEL_3_WARNING;

        case ENV_LEVEL_2_NOTICE:
            if (k < 0.45f) return ENV_LEVEL_4_CRITICAL;
            if (k < 0.65f) return ENV_LEVEL_3_WARNING;
            if (k >= 0.85f + hys) return ENV_LEVEL_1_NORMAL;
            return ENV_LEVEL_2_NOTICE;

        case ENV_LEVEL_1_NORMAL:
        default:
            if (k < 0.45f) return ENV_LEVEL_4_CRITICAL;
            if (k < 0.65f) return ENV_LEVEL_3_WARNING;
            if (k < 0.85f) return ENV_LEVEL_2_NOTICE;
            return ENV_LEVEL_1_NORMAL;
    }
}

// Hợp nhất đa cảm biến (Sensor Fusion) có quy tắc cộng hưởng bất lợi (Synergy)
static env_level_t evaluate_combined_env_level(env_level_t lt, env_level_t lh, env_level_t lmq)
{
    int max_lvl = (int)lt;
    if ((int)lh > max_lvl) max_lvl = (int)lh;
    if ((int)lmq > max_lvl) max_lvl = (int)lmq;

    if (max_lvl >= (int)ENV_LEVEL_4_CRITICAL) {
        return ENV_LEVEL_4_CRITICAL;
    }

    int count_notice = (lt >= ENV_LEVEL_2_NOTICE ? 1 : 0) +
                       (lh >= ENV_LEVEL_2_NOTICE ? 1 : 0) +
                       (lmq >= ENV_LEVEL_2_NOTICE ? 1 : 0);

    int count_warning = (lt >= ENV_LEVEL_3_WARNING ? 1 : 0) +
                        (lh >= ENV_LEVEL_3_WARNING ? 1 : 0) +
                        (lmq >= ENV_LEVEL_3_WARNING ? 1 : 0);

    // Kịch bản A: Nóng + ẩm + khí bí cùng xuất hiện (cả 3 ở mức 2, hoặc 2 yếu tố có MQ)
    // -> Nâng lên Warning (Mức 3) để kích hoạt quạt thông gió
    if (max_lvl == (int)ENV_LEVEL_2_NOTICE) {
        if (count_notice >= 3 || (count_notice >= 2 && lmq >= ENV_LEVEL_2_NOTICE)) {
            return ENV_LEVEL_3_WARNING;
        }
    }

    // Kịch bản B: Có từ 2 cảm biến cùng ở Mức 3 -> Nâng lên Critical (Mức 4)
    if (count_warning >= 2) {
        return ENV_LEVEL_4_CRITICAL;
    }

    return (env_level_t)max_lvl;
}

static bool alarm_snoozed(void)
{
    bool result;
    lock_state();
    result = esp_timer_get_time() < g_state.snooze_until_us;
    unlock_state();
    return result;
}

static bool is_finite(float x) { return isfinite(x); }

// Standard NOAA/Rothfusz Heat Index approximation.
static float heat_index_c(float temp_c, float rh)
{
    float tf = temp_c * 9.0f / 5.0f + 32.0f;
    if (tf < 80.0f || rh < 40.0f) return temp_c;

    float hi = -42.379f
             + 2.04901523f * tf
             + 10.14333127f * rh
             - 0.22475541f * tf * rh
             - 0.00683783f * tf * tf
             - 0.05481717f * rh * rh
             + 0.00122874f * tf * tf * rh
             + 0.00085282f * tf * rh * rh
             - 0.00000199f * tf * tf * rh * rh;

    if (rh < 13.0f && tf >= 80.0f && tf <= 112.0f) {
        float adj = ((13.0f - rh) / 4.0f) * sqrtf((17.0f - fabsf(tf - 95.0f)) / 17.0f);
        hi -= adj;
    } else if (rh > 85.0f && tf >= 80.0f && tf <= 87.0f) {
        float adj = ((rh - 85.0f) / 10.0f) * ((87.0f - tf) / 5.0f);
        hi += adj;
    }
    return (hi - 32.0f) * 5.0f / 9.0f;
}

// Heuristic indoor mold risk score, not a certified mold model.
static float mold_risk_score(float temp_c, float rh)
{
    float score = 0.0f;
    if (rh >= 60.0f) score += (rh - 60.0f) * 1.5f;
    if (temp_c >= 15.0f && temp_c <= 30.0f && rh >= 70.0f) score += 25.0f;
    if (temp_c >= 20.0f && temp_c <= 28.0f && rh >= 80.0f) score += 25.0f;
    if (score < 0.0f) score = 0.0f;
    if (score > 100.0f) score = 100.0f;
    return score;
}

static int comfort_level(float temp_c, float rh, float hi_c)
{
    if (!isfinite(temp_c) || !isfinite(rh)) return 0;
    float discomfort = 0.0f;
    discomfort += fabsf(temp_c - 24.0f) * 4.0f;
    discomfort += fabsf(rh - 50.0f) * 0.5f;
    discomfort += fmaxf(0.0f, hi_c - 28.0f) * 4.0f;
    if (discomfort < 20.0f) return 3; // comfortable
    if (discomfort < 50.0f) return 2; // acceptable
    if (discomfort < 85.0f) return 1; // uncomfortable
    return 0;                          // poor
}

// =============================
// Hardware outputs
// =============================
static void relay_set(bool on)
{
#if RELAY_ACTIVE_LOW
    gpio_set_level(PIN_RELAY, on ? 0 : 1);
#else
    gpio_set_level(PIN_RELAY, on ? 1 : 0);
#endif
}

static uint32_t s_current_buzzer_hz = 0;

static void buzzer_stop(void)
{
    if (s_current_buzzer_hz != 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        s_current_buzzer_hz = 0;
    }
}

static void buzzer_tone(uint32_t hz, uint32_t duty)
{
    if (hz == 0) {
        buzzer_stop();
        return;
    }
    if (s_current_buzzer_hz != hz) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, hz);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        s_current_buzzer_hz = hz;
    }
}

static void alarm_task(void *arg)
{
    while (1) {
        lock_state();
        bool selftest = g_state.selftest_active;
        env_level_t level = g_state.env_level;
        env_level_t mq_lvl = g_state.mq_level;
        bool snoozed = esp_timer_get_time() < g_state.snooze_until_us;
        unlock_state();

        if (selftest) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;

        switch (level) {
            case ENV_LEVEL_4_CRITICAL: {
                // 4. Mức cảnh báo cao:
                // - LED Đỏ nhấp nháy (chu kỳ 700ms: 350ms BẬT / 350ms TẮT)
                // - LED Xanh tắt
                // - Buzzer: CHỈ bíp ngắt quãng khi nồng độ KHÍ (MQ-135) ở mức nguy hiểm (Mức 4).
                //   Khi nhiệt độ hoặc độ ẩm ở mức cảnh báo cao -> Buzzer KHÔNG bíp (chỉ cảnh báo qua LED và Quạt).
                bool blink = ((now_ms / 350) % 2) == 0;
                gpio_set_level(PIN_LED_GREEN, 0);
                gpio_set_level(PIN_LED_RED, blink ? 1 : 0);

                bool gas_critical = (mq_lvl >= ENV_LEVEL_4_CRITICAL);
                if (!snoozed && blink && gas_critical) {
                    buzzer_tone(2400, 128);
                } else {
                    buzzer_stop();
                }
                break;
            }
            case ENV_LEVEL_3_WARNING: {
                // 3. Mức cảnh báo:
                // - LED Xanh và LED Đỏ nhấp nháy xen kẽ (chu kỳ 800ms: 400ms mỗi LED)
                // - Buzzer tắt
                bool blink = ((now_ms / 400) % 2) == 0;
                gpio_set_level(PIN_LED_GREEN, blink ? 1 : 0);
                gpio_set_level(PIN_LED_RED, blink ? 0 : 1);
                buzzer_stop();
                break;
            }
            case ENV_LEVEL_2_NOTICE: {
                // 2. Mức chú ý:
                // - LED Xanh nhấp nháy (chu kỳ 1 giây: 500ms BẬT / 500ms TẮT)
                // - LED Đỏ tắt
                // - Buzzer tắt
                bool blink = ((now_ms / 500) % 2) == 0;
                gpio_set_level(PIN_LED_GREEN, blink ? 1 : 0);
                gpio_set_level(PIN_LED_RED, 0);
                buzzer_stop();
                break;
            }
            case ENV_LEVEL_1_NORMAL:
            default: {
                // 1. Mức bình thường:
                // - LED Xanh bật liên tục
                // - LED Đỏ tắt
                // - Buzzer tắt
                gpio_set_level(PIN_LED_GREEN, 1);
                gpio_set_level(PIN_LED_RED, 0);
                buzzer_stop();
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void self_test_sequence(void)
{
    lock_state();
    g_state.selftest_active = true;
    unlock_state();

    gpio_set_level(PIN_LED_GREEN, 1);
    gpio_set_level(PIN_LED_RED, 0);
    buzzer_tone(1000, 128);
    vTaskDelay(pdMS_TO_TICKS(250));

    gpio_set_level(PIN_LED_GREEN, 0);
    gpio_set_level(PIN_LED_RED, 1);
    buzzer_tone(1800, 128);
    vTaskDelay(pdMS_TO_TICKS(250));

    gpio_set_level(PIN_LED_RED, 0);
    gpio_set_level(PIN_LED_GREEN, 1);
    buzzer_stop();

    // Briefly exercise the relay/fan.
    relay_set(true);
    vTaskDelay(pdMS_TO_TICKS(500));
    relay_set(false);

    lock_state();
    g_state.selftest_active = false;
    unlock_state();
}

// =============================
// NVS threshold storage
// =============================
static void thresholds_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        return;
    }
    size_t len = sizeof(g_thr);
    thresholds_t loaded;
    esp_err_t err = nvs_get_blob(nvs, NVS_KEY_THRESHOLDS, &loaded, &len);
    nvs_close(nvs);
    if (err == ESP_OK && len == sizeof(loaded) && loaded.version == CONFIG_VERSION) {
        g_thr = loaded;
    }
}

static void thresholds_save(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_blob(nvs, NVS_KEY_THRESHOLDS, &g_thr, sizeof(g_thr));
    nvs_commit(nvs);
    nvs_close(nvs);
}

static void mq_r0_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "No NVS R0 found, using default R0=%.2f kOhm", s_mq_r0);
        return;
    }
    size_t len = sizeof(float);
    float loaded_r0 = 0.0f;
    esp_err_t err = nvs_get_blob(nvs, NVS_KEY_MQ_R0, &loaded_r0, &len);
    nvs_close(nvs);
    if (err == ESP_OK && len == sizeof(float) && loaded_r0 > 0.1f && loaded_r0 < 1000.0f) {
        s_mq_r0 = loaded_r0;
        ESP_LOGI(TAG, "Loaded MQ-135 R0 from NVS: %.2f kOhm", s_mq_r0);
    } else {
        ESP_LOGI(TAG, "Using default MQ-135 R0: %.2f kOhm", s_mq_r0);
    }
}

static void mq_r0_save(float r0)
{
    if (r0 <= 0.1f || r0 > 1000.0f) return;
    s_mq_r0 = r0;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_blob(nvs, NVS_KEY_MQ_R0, &s_mq_r0, sizeof(float));
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Saved MQ-135 R0 to NVS: %.2f kOhm", s_mq_r0);
}

// =============================
// ADC MQ-135
// =============================
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_USED,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_MQ, &chan_cfg));
}

static int adc_read_mq_raw(void)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc_handle, ADC_CHANNEL_MQ, &raw) != ESP_OK) return 0;
    return raw;
}

// =============================
// GPIO / LEDC / I2C
// =============================
static void gpio_init_all(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_RELAY) |
                        (1ULL << PIN_LED_RED) |
                        (1ULL << PIN_LED_GREEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    gpio_set_level(PIN_LED_RED, 0);
    gpio_set_level(PIN_LED_GREEN, 0);
    relay_set(false);

    // Simple hardware sanity check: both LEDs should flash at boot.
    // This runs before OLED/Wi-Fi initialization, so a peripheral problem
    // cannot hide a basic GPIO wiring problem.
    gpio_set_level(PIN_LED_RED, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(PIN_LED_RED, 0);
    gpio_set_level(PIN_LED_GREEN, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(PIN_LED_GREEN, 0);

    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << PIN_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));

    // Khởi tạo chân DHT22 ở chế độ Open-Drain 1 lần duy nhất để tránh conflict GPIO
    ESP_ERROR_CHECK(dht22_init(PIN_DHT22));
}

static void buzzer_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num = PIN_BUZZER,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    buzzer_stop();
}

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = PIN_OLED_SDA,
        .scl_io_num = PIN_OLED_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        s_oled_ready = false;
        return;
    }

    uint8_t oled_addr = OLED_ADDR_PRIMARY;
    err = ssd1306_init(s_i2c_bus, OLED_ADDR_PRIMARY);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OLED not responding at 0x%02X: %s; trying 0x%02X",
                 OLED_ADDR_PRIMARY, esp_err_to_name(err), OLED_ADDR_SECONDARY);
        err = ssd1306_init(s_i2c_bus, OLED_ADDR_SECONDARY);
        oled_addr = OLED_ADDR_SECONDARY;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED init failed at both 0x%02X and 0x%02X: %s",
                 OLED_ADDR_PRIMARY, OLED_ADDR_SECONDARY, esp_err_to_name(err));
        ESP_LOGE(TAG, "Check OLED VCC/GND/SDA(GPIO21)/SCL(GPIO22). Firmware will continue without OLED.");
        s_oled_ready = false;
        return;
    }

    s_oled_ready = true;
    ESP_LOGI(TAG, "OLED ready at I2C address 0x%02X", oled_addr);
}

// =============================
// Wi-Fi
// =============================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason: %d, retry %d/10)", disconn->reason, s_wifi_retry_count + 1);
        lock_state();
        g_state.wifi_connected = false;
        unlock_state();
        if (s_wifi_retry_count < 10) {
            esp_wifi_connect();
            s_wifi_retry_count++;
        } else {
            ESP_LOGW(TAG, "Wi-Fi failed after 10 attempts (check SSID/password, ensure 2.4GHz). Running offline.");
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_wifi_retry_count = 0;
        lock_state();
        g_state.wifi_connected = true;
        unlock_state();
        ESP_LOGI(TAG, "Wi-Fi IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(15000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected");
    } else {
        ESP_LOGW(TAG, "Wi-Fi not connected. Sensor/control still runs locally.");
    }
}

static void wifi_rssi_update(void)
{
    wifi_ap_record_t ap;
    int rssi = 0;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;
    lock_state();
    g_state.wifi_rssi = rssi;
    unlock_state();
}

// =============================
// HTTP server (mobile-friendly local dashboard/API)
// =============================
static esp_err_t send_text(httpd_req_t *req, const char *content_type, const char *body)
{
    httpd_resp_set_type(req, content_type);
    httpd_resp_sendstr(req, body);
    return ESP_OK;
}

static esp_err_t root_get(httpd_req_t *req)
{
    static const char html[] =
        "<!doctype html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>ESP32 Environment Monitor</title>"
        "<style>body{font-family:Arial,sans-serif;margin:16px;background:#f5f7fa}"
        ".card{background:white;border-radius:12px;padding:14px;margin:10px 0;box-shadow:0 1px 5px #ccc}"
        "button{padding:10px;margin:4px;border:0;border-radius:8px;cursor:pointer}input{width:90px;padding:6px}"
        ".btn-calib{background:#ff9800;color:white;font-weight:bold}"
        "pre{white-space:pre-wrap}</style></head><body>"
        "<h2>IoT Environment Monitor</h2>"
        "<div class='card'><pre id='s'>Loading...</pre></div>"
        "<div class='card'><b>Controls</b><br>"
        "<button onclick=cmd('auto')>AUTO</button>"
        "<button onclick=cmd('on')>ON</button>"
        "<button onclick=cmd('off')>OFF</button>"
        "<button onclick=cmd('snooze')>SNOOZE</button>"
        "<button onclick=cmd('test')>SELF TEST</button>"
        "<button class='btn-calib' onclick=\"if(confirm('Calibrate R0 in clean air?'))cmd('calib')\">CALIBRATE R0 (CLEAN AIR)</button></div>"
        "<div class='card'><b>Thresholds</b><br>"
        "Temp ON <input id='to' value='30'> Temp OFF <input id='tf' value='28'><br>"
        "Hum ON <input id='ho' value='70'> Hum OFF <input id='hf' value='65'><br>"
        "Gas ON <input id='go' value='1800'> Gas OFF <input id='gf' value='1500'><br>"
        "Critical T <input id='tc' value='35'> Critical H <input id='hc' value='85'> Critical Gas <input id='gc' value='2500'><br>"
        "<button onclick=save()>SAVE</button></div>"
        "<script>"
        "async function refresh(){let j=await (await fetch('/api/state')).json();"
        "document.getElementById('s').textContent=JSON.stringify(j,null,2)}"
        "async function cmd(x){await fetch('/api/control?cmd='+x);refresh()}"
        "async function save(){let q=new URLSearchParams({to:to.value,tf:tf.value,ho:ho.value,hf:hf.value,go:go.value,gf:gf.value,tc:tc.value,hc:hc.value,gc:gc.value});await fetch('/api/thresholds?'+q);refresh()}"
        "refresh();setInterval(refresh,2000);</script></body></html>";
    return send_text(req, "text/html; charset=utf-8", html);
}

static esp_err_t state_get(httpd_req_t *req)
{
    app_state_t s;
    lock_state(); s = g_state; unlock_state();

    char json[1350];
    snprintf(json, sizeof(json),
        "{\"temperature_c\":%.1f,\"humidity_pct\":%.1f,\"heat_index_c\":%.1f,"
        "\"mold_risk_pct\":%.1f,\"comfort_level\":%d,\"mq_raw\":%d,"
        "\"mq_rs\":%.2f,\"mq_r0\":%.2f,\"mq_ratio_k\":%.2f,\"mq_level\":%d,"
        "\"temp_level\":%d,\"hum_level\":%d,"
        "\"fan\":%s,\"fan_mode\":\"%s\",\"fan_locked\":%s,\"fan_protect_rem_s\":%d,"
        "\"alarm\":%s,\"warning\":%s,"
        "\"env_level\":%d,\"env_level_str\":\"%s\","
        "\"snoozed\":%s,\"wifi\":%s,\"rssi\":%d,\"warmup_done\":%s}",
        s.temperature_c, s.humidity_pct, s.heat_index_c, s.mold_risk_pct,
        s.comfort_level, s.mq_raw,
        s.mq_rs, s.mq_r0, s.mq_ratio_k, (int)s.mq_level,
        (int)s.temp_level, (int)s.hum_level,
        s.fan_on ? "true" : "false", fan_mode_str(s.fan_mode),
        s.fan_locked ? "true" : "false", s.fan_protect_rem_s,
        s.alarm ? "true" : "false", s.warning ? "true" : "false",
        (int)s.env_level, env_level_str(s.env_level),
        alarm_snoozed() ? "true" : "false",
        s.wifi_connected ? "true" : "false", s.wifi_rssi,
        s.warmup_done ? "true" : "false");

    return send_text(req, "application/json", json);
}

static esp_err_t control_get(httpd_req_t *req)
{
    char query[128];
    char cmd[32] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "cmd", cmd, sizeof(cmd));
    }

    int64_t now_us = esp_timer_get_time();
    if (strcmp(cmd, "auto") == 0) {
        lock_state(); g_state.fan_mode = FAN_AUTO; unlock_state();
    } else if (strcmp(cmd, "on") == 0) {
        lock_state();
        g_state.fan_mode = FAN_FORCE_ON;
        g_state.fan_on = true;
        g_state.fan_locked = false;
        g_state.fan_protect_rem_s = 0;
        unlock_state();
        s_fan_is_on = true;
        s_fan_last_on_us = now_us;
        relay_set(true);
    } else if (strcmp(cmd, "off") == 0) {
        lock_state();
        g_state.fan_mode = FAN_FORCE_OFF;
        g_state.fan_on = false;
        g_state.fan_locked = false;
        g_state.fan_protect_rem_s = 0;
        unlock_state();
        s_fan_is_on = false;
        s_fan_last_off_us = now_us;
        relay_set(false);
    } else if (strcmp(cmd, "snooze") == 0) {
        lock_state();
        g_state.snooze_until_us = now_us + ((int64_t)SNOOZE_MS * 1000);
        if (g_state.fan_mode == FAN_FORCE_ON) {
            g_state.fan_mode = FAN_AUTO;
        }
        g_state.fan_on = false;
        g_state.fan_locked = false;
        g_state.fan_protect_rem_s = 0;
        unlock_state();
        s_fan_is_on = false;
        s_fan_last_off_us = now_us;
        relay_set(false);
        buzzer_stop();
    } else if (strcmp(cmd, "test") == 0) {
        s_selftest_requested = true;
    } else if (strcmp(cmd, "calib") == 0 || strcmp(cmd, "calibrate") == 0) {
        s_mq_calibrate_requested = true;
    }

    return send_text(req, "application/json", "{\"ok\":true}");
}

static bool query_float(const char *query, const char *key, float *dst)
{
    char value[32];
    if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) return false;
    char *end = NULL;
    float v = strtof(value, &end);
    if (end == value || !isfinite(v)) return false;
    *dst = v;
    return true;
}

static bool query_int(const char *query, const char *key, int *dst)
{
    char value[32];
    if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK) return false;
    char *end = NULL;
    long v = strtol(value, &end, 10);
    if (end == value || v < -1000000 || v > 1000000) return false;
    *dst = (int)v;
    return true;
}

static esp_err_t thresholds_get(httpd_req_t *req)
{
    char query[512];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        lock_state();
        query_float(query, "to", &g_thr.fan_temp_on);
        query_float(query, "tf", &g_thr.fan_temp_off);
        query_float(query, "ho", &g_thr.fan_hum_on);
        query_float(query, "hf", &g_thr.fan_hum_off);
        query_int(query, "go", &g_thr.fan_gas_on);
        query_int(query, "gf", &g_thr.fan_gas_off);
        query_float(query, "tc", &g_thr.crit_temp);
        query_float(query, "hc", &g_thr.crit_hum);
        query_int(query, "gc", &g_thr.crit_gas);
        unlock_state();
        thresholds_save();
    }
    return send_text(req, "application/json", "{\"saved\":true}");
}

static void http_register(httpd_handle_t server, const httpd_uri_t *uri)
{
    esp_err_t err = httpd_register_uri_handler(server, uri);
    if (err != ESP_OK) ESP_LOGE(TAG, "URI registration failed: %s", esp_err_to_name(err));
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    if (httpd_start(&s_http_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server failed");
        return;
    }

    static const httpd_uri_t root = {
        .uri = "/", .method = HTTP_GET, .handler = root_get, .user_ctx = NULL
    };
    static const httpd_uri_t state = {
        .uri = "/api/state", .method = HTTP_GET, .handler = state_get, .user_ctx = NULL
    };
    static const httpd_uri_t control = {
        .uri = "/api/control", .method = HTTP_GET, .handler = control_get, .user_ctx = NULL
    };
    static const httpd_uri_t thresholds = {
        .uri = "/api/thresholds", .method = HTTP_GET, .handler = thresholds_get, .user_ctx = NULL
    };

    http_register(s_http_server, &root);
    http_register(s_http_server, &state);
    http_register(s_http_server, &control);
    http_register(s_http_server, &thresholds);
}

// =============================
// Push button task
// =============================
static void button_task(void *arg)
{
    int last_level = gpio_get_level(PIN_BUTTON);
    int64_t pressed_at = 0;
    bool is_pressed = false;

    ESP_LOGI(TAG, "Button task started on GPIO%d (initial level: %d)", PIN_BUTTON, last_level);

    while (1) {
        int now_level = gpio_get_level(PIN_BUTTON);
        int64_t t = esp_timer_get_time();

        // Nút nối GPIO27 -> GND, pull-up nội:
        // Nhấn: 1 -> 0 (sườn xuống)
        // Thả:  0 -> 1 (sườn lên)
        if (last_level == 1 && now_level == 0) {
            pressed_at = t;
            is_pressed = true;
            ESP_LOGI(TAG, "Button PRESSED (GPIO%d = 0)", PIN_BUTTON);
        } else if (last_level == 0 && now_level == 1 && is_pressed) {
            int64_t duration_ms = (t - pressed_at) / 1000;
            is_pressed = false;
            ESP_LOGI(TAG, "Button RELEASED after %lld ms", duration_ms);

            if (duration_ms >= 5000) {
                // Nhấn giữ >= 5 giây: Hiệu chuẩn R0 trong không khí sạch
                ESP_LOGI(TAG, "Button very long-press (>= 5s) -> Calibrating Clean Air R0");
                s_mq_calibrate_requested = true;
            } else if (duration_ms >= BUTTON_LONG_MS) {
                // Nhấn giữ 1.5s - 5s: Self-test
                ESP_LOGI(TAG, "Button long-press -> Requesting Self-Test");
                s_selftest_requested = true;
            } else if (duration_ms >= BUTTON_DEBOUNCE_MS) {
                // Nhấn nhả nhanh: Tắt còi, snooze cảnh báo 60s
                ESP_LOGI(TAG, "Button short-press -> Resetting Alarm & Turning OFF Fan (snooze %d s)",
                         SNOOZE_MS / 1000);

                lock_state();
                g_state.snooze_until_us = t + ((int64_t)SNOOZE_MS * 1000);
                if (g_state.fan_mode == FAN_FORCE_ON) {
                    g_state.fan_mode = FAN_AUTO;
                }
                g_state.fan_on = false;
                g_state.fan_locked = false;
                g_state.fan_protect_rem_s = 0;
                unlock_state();

                // Lập tức tắt quạt và còi phần cứng ngay lập tức
                s_fan_is_on = false;
                s_fan_last_off_us = t;
                relay_set(false);
                buzzer_stop();
            }
            pressed_at = 0;
        }

        last_level = now_level;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================
// Sensor/control task
// =============================
static void sensor_control_task(void *arg)
{
    int64_t next_sample = esp_timer_get_time();
    while (1) {
        if (s_selftest_requested) {
            s_selftest_requested = false;
            self_test_sequence();
        }

        int64_t now = esp_timer_get_time();
        bool warmup_done = (now - g_state.boot_us) >= ((int64_t)MQ_WARMUP_MS * 1000);

        // DHT22: keep reads >2 s apart.
        float temp = NAN, hum = NAN;
        esp_err_t dht_err = dht22_read(PIN_DHT22, &temp, &hum);
        int mq_raw = adc_read_mq_raw();
        float mq_rs = mq135_calculate_rs(mq_raw);

        // Xử lý yêu cầu hiệu chuẩn trong không khí sạch
        if (s_mq_calibrate_requested) {
            s_mq_calibrate_requested = false;
            mq_r0_save(mq_rs);
            for (int i = 0; i < MQ_FILTER_SAMPLES; i++) {
                s_mq_samples[i] = 1.0f;
            }
            s_mq_sample_count = MQ_FILTER_SAMPLES;
            s_mq_prev_level = ENV_LEVEL_1_NORMAL;
            ESP_LOGW(TAG, ">>> CALIBRATION SUCCESS: R0 set to %.2f kOhm (Raw ADC=%d) <<<", s_mq_r0, mq_raw);
        }

        // Lọc trung bình trượt 50s (20 mẫu) và xác định Level MQ-135
        float mq_k = mq_filter_update(mq_rs, s_mq_r0);
        env_level_t mq_lvl = get_mq_level(mq_k, s_mq_prev_level);
        s_mq_prev_level = mq_lvl;

        env_level_t temp_lvl = ENV_LEVEL_1_NORMAL;
        env_level_t hum_lvl = ENV_LEVEL_1_NORMAL;
        bool dht_ok = (dht_err == ESP_OK);

        if (dht_ok) {
            temp_lvl = get_temp_level(temp, s_temp_prev_level);
            hum_lvl = get_hum_level(hum, s_hum_prev_level);
            s_temp_prev_level = temp_lvl;
            s_hum_prev_level = hum_lvl;
        }

        // Hợp nhất đa cảm biến với hiệu ứng cộng hưởng
        env_level_t level = evaluate_combined_env_level(temp_lvl, hum_lvl, mq_lvl);

        lock_state();
        if (dht_ok) {
            g_state.temperature_c = temp;
            g_state.humidity_pct = hum;
            g_state.heat_index_c = heat_index_c(temp, hum);
            g_state.mold_risk_pct = mold_risk_score(temp, hum);
            g_state.comfort_level = comfort_level(temp, hum, g_state.heat_index_c);
            g_state.dht_valid = true;
        } else {
            ESP_LOGW(TAG, "DHT22 read failed: %s", esp_err_to_name(dht_err));
        }
        g_state.mq_raw = mq_raw;
        g_state.mq_rs = mq_rs;
        g_state.mq_r0 = s_mq_r0;
        g_state.mq_ratio_k = mq_k;
        g_state.temp_level = temp_lvl;
        g_state.hum_level = hum_lvl;
        g_state.mq_level = mq_lvl;
        g_state.warmup_done = warmup_done;
        app_state_t s = g_state;
        unlock_state();

        wifi_rssi_update();

        // 1. Đánh giá nhu cầu bật quạt từ môi trường (Debounce xác nhận 2 mẫu liên tiếp)
        // Yêu cầu: Ở mức 3 và mức 4, quạt KHÔNG bật khi nhiệt độ (hoặc độ ẩm hoặc cả hai) đang ở ngưỡng dưới
        // - Ngưỡng dưới nhiệt độ: Mức 3 [21.0 - 23.0°C] hoặc Mức 4 [< 21.0°C]
        // - Ngưỡng dưới độ ẩm: Mức 3 [30.0 - 40.0%] hoặc Mức 4 [< 30.0%]
        bool is_temp_lower = dht_ok && ((temp_lvl >= ENV_LEVEL_3_WARNING && temp < 25.0f) || (temp < 23.0f));
        bool is_hum_lower  = dht_ok && ((hum_lvl >= ENV_LEVEL_3_WARNING && hum < 50.0f) || (hum < 40.0f));

        bool desired_fan = false;
        if (is_temp_lower || is_hum_lower) {
            // Nhiệt độ hoặc độ ẩm (hoặc cả hai) đang ở ngưỡng dưới -> Quạt không bật (Standby)
            s_warning_persist_count = 0;
            desired_fan = false;
        } else if (level >= ENV_LEVEL_4_CRITICAL) {
            // Mức 4 do ngưỡng trên (nhiệt độ > 33°C, độ ẩm > 85%) hoặc nồng độ khí gas nguy hiểm (K < 0.45)
            desired_fan = true;
            s_warning_persist_count = FAN_TRIGGER_PERSIST;
        } else if (level >= ENV_LEVEL_3_WARNING) {
            // Mức 3 do ngưỡng trên (nhiệt độ 31-33°C, độ ẩm 75-85%) hoặc khí gas cảnh báo (K 0.45-0.65)
            if (++s_warning_persist_count >= FAN_TRIGGER_PERSIST) {
                desired_fan = true;
            }
        } else {
            s_warning_persist_count = 0;
            desired_fan = false;
        }

        // 2. Thuật toán Anti-Short-Cycle bảo vệ động cơ quạt và rơ-le
        bool fan_on = s_fan_is_on;
        bool fan_locked = false;
        int fan_protect_rem_s = 0;
        int64_t now_us = esp_timer_get_time();

        switch (s.fan_mode) {
            case FAN_FORCE_ON:
                fan_on = true;
                if (!s_fan_is_on) {
                    s_fan_last_on_us = now_us;
                }
                break;

            case FAN_FORCE_OFF:
                fan_on = false;
                if (s_fan_is_on) {
                    s_fan_last_off_us = now_us;
                }
                break;

            case FAN_AUTO:
            default:
                if (alarm_snoozed()) {
                    // Người dùng can thiệp tắt khẩn cấp -> Ngắt ngay lập tức
                    if (s_fan_is_on) {
                        fan_on = false;
                        s_fan_last_off_us = now_us;
                        ESP_LOGI(TAG, "Fan shut down immediately by USER SNOOZE");
                    }
                } else if (s_fan_is_on) {
                    // Quạt ĐANG CHẠY: nếu môi trường hết cảnh báo (desired_fan == false)
                    if (!desired_fan) {
                        int64_t run_time_ms = (now_us - s_fan_last_on_us) / 1000;
                        if (run_time_ms >= FAN_MIN_RUN_TIME_MS) {
                            fan_on = false;
                            s_fan_last_off_us = now_us;
                            ESP_LOGI(TAG, "Fan MIN_RUN reached (%lld ms) -> Turning OFF", run_time_ms);
                        } else {
                            // Khóa bảo vệ MIN_RUN: quạt tiếp tục chạy cho đủ thời gian
                            fan_on = true;
                            fan_locked = true;
                            fan_protect_rem_s = (int)((FAN_MIN_RUN_TIME_MS - run_time_ms) / 1000) + 1;
                        }
                    } else {
                        fan_on = true;
                    }
                } else {
                    // Quạt ĐANG NGHỈ: nếu môi trường muốn bật quạt (desired_fan == true)
                    if (desired_fan) {
                        int64_t rest_time_ms = (now_us - s_fan_last_off_us) / 1000;
                        if (rest_time_ms >= FAN_MIN_REST_TIME_MS) {
                            fan_on = true;
                            s_fan_last_on_us = now_us;
                            ESP_LOGI(TAG, "Fan MIN_REST reached (%lld ms) -> Turning ON", rest_time_ms);
                        } else {
                            // Khóa bảo vệ MIN_REST: quạt tiếp tục nghỉ cho đủ thời gian
                            fan_on = false;
                            fan_locked = true;
                            fan_protect_rem_s = (int)((FAN_MIN_REST_TIME_MS - rest_time_ms) / 1000) + 1;
                        }
                    } else {
                        fan_on = false;
                    }
                }
                break;
        }

        s_fan_is_on = fan_on;
        relay_set(fan_on);

        lock_state();
        g_state.fan_on = fan_on;
        g_state.fan_locked = fan_locked;
        g_state.fan_protect_rem_s = fan_protect_rem_s;
        g_state.env_level = level;
        g_state.alarm = (mq_lvl >= ENV_LEVEL_4_CRITICAL);
        g_state.warning = (level >= ENV_LEVEL_2_NOTICE);
        unlock_state();

        if (dht_ok) {
            ESP_LOGI(TAG, "T=%.1fC(L%d%s) RH=%.1f%%(L%d%s) K=%.2f(L%d) -> Level=%s(%d) Fan=%s(%s%ds) Mode=%s RSSI=%d",
                     temp, (int)temp_lvl, is_temp_lower ? ":LOW" : "",
                     hum, (int)hum_lvl, is_hum_lower ? ":LOW" : "",
                     mq_k, (int)mq_lvl,
                     env_level_str(level), (int)level, fan_on ? "ON" : "OFF",
                     fan_locked ? "PROT:" : "", fan_locked ? fan_protect_rem_s : 0,
                     fan_mode_str(s.fan_mode), s.wifi_rssi);
        } else {
            ESP_LOGI(TAG, "DHT FAIL | K=%.2f(L%d) -> Level=%s(%d) Fan=%s Mode=%s RSSI=%d",
                     mq_k, (int)mq_lvl, env_level_str(level), (int)level, fan_on ? "ON" : "OFF", fan_mode_str(s.fan_mode), s.wifi_rssi);
        }

        next_sample += ((int64_t)SAMPLE_PERIOD_MS * 1000);
        int64_t delay_us = next_sample - esp_timer_get_time();
        if (delay_us < 0) delay_us = 0;
        vTaskDelay(pdMS_TO_TICKS((uint32_t)(delay_us / 1000)));
    }
}

// =============================
// OLED task
// =============================
static void oled_task(void *arg)
{
    int page = 0;
    while (1) {
        app_state_t s;
        lock_state(); s = g_state; unlock_state();

        if (!s_oled_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (xSemaphoreTake(s_oled_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            char line[32];
            ssd1306_clear();
            if (page == 0) {
                ssd1306_draw_text(0, 0, "ENVIRONMENT", false);
                if (s.dht_valid) {
                    snprintf(line, sizeof(line), "TEMP %.1fC (L%d)", s.temperature_c, (int)s.temp_level);
                    ssd1306_draw_text(0, 16, line, false);
                    snprintf(line, sizeof(line), "HUM  %.1f%% (L%d)", s.humidity_pct, (int)s.hum_level);
                    ssd1306_draw_text(0, 28, line, false);
                    snprintf(line, sizeof(line), "HEAT %.1fC", s.heat_index_c);
                    ssd1306_draw_text(0, 40, line, false);
                    snprintf(line, sizeof(line), "MOLD %.0f%%", s.mold_risk_pct);
                    ssd1306_draw_text(0, 52, line, false);
                } else {
                    ssd1306_draw_text(0, 20, "DHT ERROR", true);
                }
            } else if (page == 1) {
                ssd1306_draw_text(0, 0, "AIR & FAN", false);
                snprintf(line, sizeof(line), "K:%.2f L%d", s.mq_ratio_k, (int)s.mq_level);
                ssd1306_draw_text(0, 14, line, false);
                snprintf(line, sizeof(line), "Rs:%.1fk R0:%.1fk", s.mq_rs, s.mq_r0);
                ssd1306_draw_text(0, 26, line, false);
                if (s.fan_locked && s.fan_protect_rem_s > 0) {
                    snprintf(line, sizeof(line), "FAN %s (%ds)", s.fan_on ? "ON" : "OFF", s.fan_protect_rem_s);
                } else {
                    snprintf(line, sizeof(line), "FAN %s (%s)", s.fan_on ? "ON" : "OFF", fan_mode_str(s.fan_mode));
                }
                ssd1306_draw_text(0, 38, line, false);
                snprintf(line, sizeof(line), "SYS LVL %d %s", (int)s.env_level, alarm_snoozed() ? "SNOOZE" : env_level_str(s.env_level));
                ssd1306_draw_text(0, 50, line, false);
            } else {
                ssd1306_draw_text(0, 0, "SYSTEM", false);
                snprintf(line, sizeof(line), "WIFI %s", s.wifi_connected ? "OK" : "OFF");
                ssd1306_draw_text(0, 14, line, false);
                snprintf(line, sizeof(line), "RSSI %d", s.wifi_rssi);
                ssd1306_draw_text(0, 26, line, false);
                snprintf(line, sizeof(line), "WARM %s", s.warmup_done ? "OK" : "WAIT");
                ssd1306_draw_text(0, 38, line, false);
                snprintf(line, sizeof(line), "RAW ADC %d", s.mq_raw);
                ssd1306_draw_text(0, 50, line, false);
            }
            ssd1306_update();
            xSemaphoreGive(s_oled_mutex);
        }

        page = (page + 1) % 3;
        vTaskDelay(pdMS_TO_TICKS(OLED_PAGE_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting IoT Environment Monitor - ESP-IDF");

    s_state_mutex = xSemaphoreCreateMutex();
    s_oled_mutex = xSemaphoreCreateMutex();
    configASSERT(s_state_mutex && s_oled_mutex);

    g_state.boot_us = esp_timer_get_time();

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);
    thresholds_load();
    mq_r0_load();

    gpio_init_all();
    buzzer_init();
    adc_init();
    i2c_init();

    wifi_init_sta();
    start_webserver();

    xTaskCreate(sensor_control_task, "sensor_ctrl", 8192, NULL, 5, NULL);
    xTaskCreate(oled_task, "oled", 4096, NULL, 4, NULL);
    xTaskCreate(button_task, "button", 2048, NULL, 4, NULL);
    xTaskCreate(alarm_task, "alarm", 2048, NULL, 4, NULL);

    // Keep the main task alive.
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
