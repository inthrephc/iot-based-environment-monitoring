/*
 * =========================================================================================
 *   HỆ THỐNG GIÁM SÁT MÔI TRƯỜNG IOT (IoT-Based Environment Monitoring) - ARDUINO ESP32
 * =========================================================================================
 *   Phiên bản chuyển đổi (Migrate) từ ESP-IDF sang Arduino IDE (.ino)
 *   Bảo toàn 100% logic điều khiển, thuật toán Sensor Fusion, Anti-Short-Cycle & REST API
 * 
 *   CÁC THƯ VIỆN CẦN CÀI ĐẶT TRÊN ARDUINO IDE (Vào Sketch -> Include Library -> Manage Libraries):
 *   1. "DHT sensor library" bởi Adafruit (chọn "Install All" khi được hỏi cài thêm Adafruit Unified Sensor)
 *   2. "Adafruit SSD1306" bởi Adafruit (chọn "Install All" khi được hỏi cài thêm Adafruit GFX Library)
 * 
 *   CÁC THƯ VIỆN ĐÃ TÍCH HỢP SẴN TRONG GÓI ESP32 ARDUINO CORE:
 *   - WiFi.h
 *   - WebServer.h
 *   - Preferences.h (Giao diện NVS lưu Flash vĩnh viễn)
 *   - Wire.h
 * =========================================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <math.h>
#include <esp_timer.h>

// =========================================================================================
// 1. SƠ ĐỒ CHÂN PHẦN CỨNG (ESP32 WROOM-32, 30-pin DevKit)
// =========================================================================================
#define PIN_DHT22            4    // Chân DATA của DHT22 (AM2302)
#define PIN_MQ135_AO        34    // Chân Analog AO của MQ-135 (ADC1_CH6 qua cầu phân áp 10k/10k)
#define PIN_OLED_SDA        21    // Dây dữ liệu SDA I2C màn hình OLED SSD1306
#define PIN_OLED_SCL        22    // Dây xung nhịp SCL I2C màn hình OLED SSD1306
#define PIN_RELAY           16    // Chân điều khiển Relay quạt hút
#define PIN_BUZZER          17    // Chân còi chip thụ động (Passive Buzzer PWM)
#define PIN_LED_RED         25    // LED Đỏ cảnh báo (qua điện trở 220 Ohm)
#define PIN_LED_GREEN       26    // LED Xanh bình thường (qua điện trở 220 Ohm)
#define PIN_BUTTON          27    // Nút nhấn vật lý (nối GND, bật pull-up nội)

// Cấu hình loại Relay: 0 = Kích mức Cao (Active-HIGH), 1 = Kích mức Thấp (Active-LOW)
#define RELAY_ACTIVE_LOW     0

// Cấu hình I2C OLED SSD1306
#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT        64
#define OLED_RESET           -1
#define OLED_ADDR_PRIMARY   0x3C
#define OLED_ADDR_SECONDARY 0x3D

// =========================================================================================
// 2. CẤU HÌNH THỜI GIAN & THÔNG SỐ VẬN HÀNH
// =========================================================================================
#define WIFI_SSID           "Ky Tuc Xa DHFPT"
#define WIFI_PASSWORD       ""

#define SAMPLE_PERIOD_MS    2500   // Chu kỳ lấy mẫu cảm biến (2.5 giây)
#define OLED_PAGE_PERIOD_MS 5000   // Thời gian xoay vòng trang OLED (5 giây)
#define MQ_WARMUP_MS        60000  // Thời gian sấy nóng cảm biến MQ-135 (60 giây)
#define BUTTON_DEBOUNCE_MS  40     // Bộ lọc chống dội nút nhấn (40 ms)
#define BUTTON_LONG_MS      1500   // Ngưỡng nhấn giữ Self-Test (1.5s - 5.0s)
#define SNOOZE_MS           60000  // Thời gian tạm hoãn/im lặng cảnh báo (60 giây)

// Cấu hình cơ chế bảo vệ động cơ quạt và rơ-le (Anti-Short-Cycle)
#define FAN_MIN_RUN_TIME_MS  60000 // Chạy tối thiểu 60s khi đã bật để bảo vệ cuộn dây và tản nhiệt
#define FAN_MIN_REST_TIME_MS 60000 // Nghỉ tối thiểu 60s khi đã tắt trước khi bật lại để bảo vệ tụ đề & tiếp điểm relay
#define FAN_TRIGGER_PERSIST  2     // Cần 2 mẫu đo liên tiếp (5s) ở Level 3 mới bật quạt (chống nhiễu tức thời)
#define TEMP_HYSTERESIS_C    0.6f  // Độ trễ nhiệt độ khi phục hồi về mức thấp hơn
#define HUM_HYSTERESIS_PCT   3.0f  // Độ trễ độ ẩm khi phục hồi về mức thấp hơn

#define NVS_NAMESPACE       "envmon"
#define NVS_KEY_THRESHOLDS  "thresholds"
#define NVS_KEY_MQ_R0       "mq_r0"
#define CONFIG_VERSION      1

#define MQ_DEFAULT_R0       40.0f  // Giá trị mốc chuẩn R0 mặc định trong không khí sạch (kOhm)
#define MQ_FILTER_SAMPLES   20     // 20 mẫu x 2.5s = Bộ lọc trung bình trượt 50 giây cho tỉ lệ K
#define MQ_RL_KOHM          10.0f  // Điện trở tải trên mạch breakout MQ-135 (~10 kOhm)

// =========================================================================================
// 3. ĐỊNH NGHĨA DỮ LIỆU & ENUM
// =========================================================================================
enum env_level_t {
    ENV_LEVEL_1_NORMAL = 1,
    ENV_LEVEL_2_NOTICE = 2,
    ENV_LEVEL_3_WARNING = 3,
    ENV_LEVEL_4_CRITICAL = 4,
};

enum fan_mode_t {
    FAN_AUTO = 0,
    FAN_FORCE_ON,
    FAN_FORCE_OFF,
};

// Cấu trúc lưu ngưỡng trong bộ nhớ Flash (NVS)
struct thresholds_t {
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
};

// Cấu trúc trạng thái toàn cục của hệ thống (Telemetry State)
struct app_state_t {
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
    bool fan_locked;           // Đang trong thời gian khóa bảo vệ Min-Run hoặc Min-Rest
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
};

// =========================================================================================
// 4. BIẾN TOÀN CỤC & ĐỐI TƯỢNG PHẦN CỨNG
// =========================================================================================
static thresholds_t g_thr = {
    .version = CONFIG_VERSION,
    .fan_temp_on = 30.0f,
    .fan_temp_off = 28.0f,
    .fan_hum_on = 70.0f,
    .fan_hum_off = 65.0f,
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

static SemaphoreHandle_t s_state_mutex = NULL;
static SemaphoreHandle_t s_oled_mutex = NULL;

static bool s_oled_ready = false;
static volatile bool s_selftest_requested = false;
static volatile bool s_mq_calibrate_requested = false;

// Đối tượng phần cứng & thư viện
static DHT dht(PIN_DHT22, DHT22);
static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
static WebServer server(80);
static Preferences prefs;

// =========================================================================================
// 5. CÁC HÀM TIỆN ÍCH & ĐỒNG BỘ TRẠNG THÁI
// =========================================================================================
static void lock_state(void) {
    if (s_state_mutex) xSemaphoreTake(s_state_mutex, portMAX_DELAY);
}

static void unlock_state(void) {
    if (s_state_mutex) xSemaphoreGive(s_state_mutex);
}

static const char *fan_mode_str(fan_mode_t mode) {
    switch (mode) {
        case FAN_FORCE_ON:  return "ON";
        case FAN_FORCE_OFF: return "OFF";
        default:            return "AUTO";
    }
}

static const char *env_level_str(env_level_t lvl) {
    switch (lvl) {
        case ENV_LEVEL_4_CRITICAL: return "CRIT";
        case ENV_LEVEL_3_WARNING:  return "WARN";
        case ENV_LEVEL_2_NOTICE:   return "NOTICE";
        default:                   return "NORM";
    }
}

static bool alarm_snoozed(void) {
    bool result;
    lock_state();
    result = esp_timer_get_time() < g_state.snooze_until_us;
    unlock_state();
    return result;
}

// =========================================================================================
// 6. THUẬT TOÁN TÍNH TOÁN & PHÂN LOẠI MÔI TRƯỜNG (KÈM VÙNG TRỄ HYSTERESIS)
// =========================================================================================

// Xác định mức nhiệt độ kèm Hysteresis (0.6°C) chống rung ngưỡng
static env_level_t get_temp_level(float t, env_level_t prev_lvl) {
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
static env_level_t get_hum_level(float h, env_level_t prev_lvl) {
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
static float mq135_calculate_rs(int raw) {
    if (raw < 15) raw = 15;
    if (raw > 4080) raw = 4080;
    return MQ_RL_KOHM * (4095.0f - (float)raw) / (float)raw;
}

// Bộ lọc trung bình trượt 50s (20 mẫu x 2.5s) cho tỉ số K = Rs/R0
static float mq_filter_update(float new_rs, float r0) {
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
static env_level_t get_mq_level(float k, env_level_t prev_lvl) {
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
static env_level_t evaluate_combined_env_level(env_level_t lt, env_level_t lh, env_level_t lmq) {
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

// Chỉ số nhiệt Heat Index theo hồi quy NOAA / Rothfusz
static float heat_index_c(float temp_c, float rh) {
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

// Nguy cơ nấm mốc trong nhà Mold Risk Score (0 - 100%)
static float mold_risk_score(float temp_c, float rh) {
    float score = 0.0f;
    if (rh >= 60.0f) score += (rh - 60.0f) * 1.5f;
    if (temp_c >= 15.0f && temp_c <= 30.0f && rh >= 70.0f) score += 25.0f;
    if (temp_c >= 20.0f && temp_c <= 28.0f && rh >= 80.0f) score += 25.0f;
    if (score < 0.0f) score = 0.0f;
    if (score > 100.0f) score = 100.0f;
    return score;
}

// Điểm thoải mái nhiệt độ Comfort Level (0: Kém, 1: Khó chịu, 2: Chấp nhận, 3: Thoải mái)
static int comfort_level(float temp_c, float rh, float hi_c) {
    if (!isfinite(temp_c) || !isfinite(rh)) return 0;
    float discomfort = 0.0f;
    discomfort += fabsf(temp_c - 24.0f) * 4.0f;
    discomfort += fabsf(rh - 50.0f) * 0.5f;
    discomfort += fmaxf(0.0f, hi_c - 28.0f) * 4.0f;
    if (discomfort < 20.0f) return 3; // comfortable
    if (discomfort < 50.0f) return 2; // acceptable
    if (discomfort < 85.0f) return 1; // uncomfortable
    return 0;                         // poor
}

// =========================================================================================
// 7. ĐIỀU KHIỂN THIẾT BỊ NGOẠI VI (RELAY, BUZZER, LED, OLED)
// =========================================================================================
static void relay_set(bool on) {
#if RELAY_ACTIVE_LOW
    digitalWrite(PIN_RELAY, on ? LOW : HIGH);
#else
    digitalWrite(PIN_RELAY, on ? HIGH : LOW);
#endif
}

static uint32_t s_current_buzzer_hz = 0;

static void buzzer_stop(void) {
    if (s_current_buzzer_hz != 0) {
        noTone(PIN_BUZZER);
        digitalWrite(PIN_BUZZER, LOW);
        s_current_buzzer_hz = 0;
    }
}

static void buzzer_tone(uint32_t hz) {
    if (hz == 0) {
        buzzer_stop();
        return;
    }
    if (s_current_buzzer_hz != hz) {
        tone(PIN_BUZZER, hz);
        s_current_buzzer_hz = hz;
    }
}

static void oled_clear(void) {
    if (!s_oled_ready) return;
    display.clearDisplay();
}

static void oled_draw_text(int x, int y, const char *text, bool scale2 = false) {
    if (!s_oled_ready) return;
    display.setTextSize(scale2 ? 2 : 1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x, y);
    display.print(text);
}

static void oled_update(void) {
    if (!s_oled_ready) return;
    display.display();
}

// Quy trình tự kiểm tra ngoại vi (Hardware Self-Test)
static void self_test_sequence(void) {
    lock_state();
    g_state.selftest_active = true;
    unlock_state();

    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
    buzzer_tone(1000);
    vTaskDelay(pdMS_TO_TICKS(250));

    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_RED, HIGH);
    buzzer_tone(1800);
    vTaskDelay(pdMS_TO_TICKS(250));

    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, HIGH);
    buzzer_stop();

    // Kích hoạt ngắn relay quạt để kiểm tra tiếp điểm
    relay_set(true);
    vTaskDelay(pdMS_TO_TICKS(500));
    relay_set(false);

    lock_state();
    g_state.selftest_active = false;
    unlock_state();
}

// =========================================================================================
// 8. LƯU TRỮ THÔNG SỐ VÀO BỘ NHỚ FLASH NVS (PREFERENCES)
// =========================================================================================
static void thresholds_load(void) {
    if (prefs.begin(NVS_NAMESPACE, true)) {
        thresholds_t loaded;
        size_t len = prefs.getBytes(NVS_KEY_THRESHOLDS, &loaded, sizeof(loaded));
        if (len == sizeof(thresholds_t) && loaded.version == CONFIG_VERSION) {
            g_thr = loaded;
            Serial.println("[ENV_MON] Loaded thresholds from NVS");
        }
        prefs.end();
    }
}

static void thresholds_save(void) {
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putBytes(NVS_KEY_THRESHOLDS, &g_thr, sizeof(thresholds_t));
        prefs.end();
        Serial.println("[ENV_MON] Saved thresholds to NVS");
    }
}

static void mq_r0_load(void) {
    if (prefs.begin(NVS_NAMESPACE, true)) {
        float loaded_r0 = 0.0f;
        size_t len = prefs.getBytes(NVS_KEY_MQ_R0, &loaded_r0, sizeof(float));
        if (len == sizeof(float) && loaded_r0 > 0.1f && loaded_r0 < 1000.0f) {
            s_mq_r0 = loaded_r0;
            Serial.printf("[ENV_MON] Loaded MQ-135 R0 from NVS: %.2f kOhm\n", s_mq_r0);
            prefs.end();
            return;
        }
        prefs.end();
    }
    Serial.printf("[ENV_MON] Using default MQ-135 R0: %.2f kOhm\n", s_mq_r0);
}

static void mq_r0_save(float r0) {
    if (r0 <= 0.1f || r0 > 1000.0f) return;
    s_mq_r0 = r0;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putBytes(NVS_KEY_MQ_R0, &s_mq_r0, sizeof(float));
        prefs.end();
        Serial.printf("[ENV_MON] Saved MQ-135 R0 to NVS: %.2f kOhm\n", s_mq_r0);
    }
}

// =========================================================================================
// 9. GIAO DIỆN WEB DASHBOARD VÀ REST API
// =========================================================================================
static void handleRoot(void) {
    static const char html[] PROGMEM =
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

    server.send(200, "text/html; charset=utf-8", html);
}

static void handleState(void) {
    app_state_t s;
    lock_state();
    s = g_state;
    unlock_state();

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
        isnan(s.temperature_c) ? 0.0f : s.temperature_c,
        isnan(s.humidity_pct) ? 0.0f : s.humidity_pct,
        isnan(s.heat_index_c) ? 0.0f : s.heat_index_c,
        isnan(s.mold_risk_pct) ? 0.0f : s.mold_risk_pct,
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

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", json);
}

static void handleControl(void) {
    String cmd = "";
    if (server.hasArg("cmd")) {
        cmd = server.arg("cmd");
    }

    int64_t now_us = esp_timer_get_time();
    if (cmd == "auto") {
        lock_state(); g_state.fan_mode = FAN_AUTO; unlock_state();
    } else if (cmd == "on") {
        lock_state();
        g_state.fan_mode = FAN_FORCE_ON;
        g_state.fan_on = true;
        g_state.fan_locked = false;
        g_state.fan_protect_rem_s = 0;
        unlock_state();
        s_fan_is_on = true;
        s_fan_last_on_us = now_us;
        relay_set(true);
    } else if (cmd == "off") {
        lock_state();
        g_state.fan_mode = FAN_FORCE_OFF;
        g_state.fan_on = false;
        g_state.fan_locked = false;
        g_state.fan_protect_rem_s = 0;
        unlock_state();
        s_fan_is_on = false;
        s_fan_last_off_us = now_us;
        relay_set(false);
    } else if (cmd == "snooze") {
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
    } else if (cmd == "test") {
        s_selftest_requested = true;
    } else if (cmd == "calib" || cmd == "calibrate") {
        s_mq_calibrate_requested = true;
    }

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleThresholds(void) {
    lock_state();
    if (server.hasArg("to")) g_thr.fan_temp_on = server.arg("to").toFloat();
    if (server.hasArg("tf")) g_thr.fan_temp_off = server.arg("tf").toFloat();
    if (server.hasArg("ho")) g_thr.fan_hum_on = server.arg("ho").toFloat();
    if (server.hasArg("hf")) g_thr.fan_hum_off = server.arg("hf").toFloat();
    if (server.hasArg("go")) g_thr.fan_gas_on = server.arg("go").toInt();
    if (server.hasArg("gf")) g_thr.fan_gas_off = server.arg("gf").toInt();
    if (server.hasArg("tc")) g_thr.crit_temp = server.arg("tc").toFloat();
    if (server.hasArg("hc")) g_thr.crit_hum = server.arg("hc").toFloat();
    if (server.hasArg("gc")) g_thr.crit_gas = server.arg("gc").toInt();
    unlock_state();
    thresholds_save();

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"saved\":true}");
}

// =========================================================================================
// 10. CÁC TÁC VỤ FREERTOS CHẠY SONG SONG TRONG HỆ THỐNG
// =========================================================================================

// Tác vụ cảnh báo LED & Còi Buzzer (chu kỳ 50 ms)
static void alarm_task(void *arg) {
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
                // Mức 4: LED Đỏ chớp nhanh (350ms ON / 350ms OFF), LED Xanh tắt
                // Còi Buzzer: CHỈ bíp ngắt quãng khi nồng độ KHÍ (MQ-135) ở Mức 4.
                // Nếu chỉ nóng/ẩm thì còi im lặng để tránh ô nhiễm tiếng ồn.
                bool blink = ((now_ms / 350) % 2) == 0;
                digitalWrite(PIN_LED_GREEN, LOW);
                digitalWrite(PIN_LED_RED, blink ? HIGH : LOW);

                bool gas_critical = (mq_lvl >= ENV_LEVEL_4_CRITICAL);
                if (!snoozed && blink && gas_critical) {
                    buzzer_tone(2400);
                } else {
                    buzzer_stop();
                }
                break;
            }
            case ENV_LEVEL_3_WARNING: {
                // Mức 3: LED Xanh & Đỏ nhấp nháy xen kẽ (chu kỳ 800ms: 400ms mỗi LED)
                // Còi Buzzer tắt
                bool blink = ((now_ms / 400) % 2) == 0;
                digitalWrite(PIN_LED_GREEN, blink ? HIGH : LOW);
                digitalWrite(PIN_LED_RED, blink ? LOW : HIGH);
                buzzer_stop();
                break;
            }
            case ENV_LEVEL_2_NOTICE: {
                // Mức 2: LED Xanh nhấp nháy đều (chu kỳ 1 giây: 500ms ON / 500ms OFF), LED Đỏ tắt
                // Còi Buzzer tắt
                bool blink = ((now_ms / 500) % 2) == 0;
                digitalWrite(PIN_LED_GREEN, blink ? HIGH : LOW);
                digitalWrite(PIN_LED_RED, LOW);
                buzzer_stop();
                break;
            }
            case ENV_LEVEL_1_NORMAL:
            default: {
                // Mức 1: LED Xanh bật sáng đứng liên tục, LED Đỏ tắt, Còi Buzzer tắt
                digitalWrite(PIN_LED_GREEN, HIGH);
                digitalWrite(PIN_LED_RED, LOW);
                buzzer_stop();
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Tác vụ nút nhấn vật lý đa chức năng (chu kỳ 10 ms)
static void button_task(void *arg) {
    int last_level = digitalRead(PIN_BUTTON);
    int64_t pressed_at = 0;
    bool is_pressed = false;

    Serial.printf("[ENV_MON] Button task started on GPIO%d (initial: %d)\n", PIN_BUTTON, last_level);

    while (1) {
        int now_level = digitalRead(PIN_BUTTON);
        int64_t t = esp_timer_get_time();

        // Nút nhấn nối GPIO27 -> GND, pull-up nội:
        // Nhấn: 1 -> 0 (sườn xuống)
        // Thả:  0 -> 1 (sườn lên)
        if (last_level == 1 && now_level == 0) {
            pressed_at = t;
            is_pressed = true;
            Serial.printf("[ENV_MON] Button PRESSED (GPIO%d = 0)\n", PIN_BUTTON);
        } else if (last_level == 0 && now_level == 1 && is_pressed) {
            int64_t duration_ms = (t - pressed_at) / 1000;
            is_pressed = false;
            Serial.printf("[ENV_MON] Button RELEASED after %lld ms\n", duration_ms);

            if (duration_ms >= 5000) {
                // Nhấn giữ >= 5 giây: Cân chỉnh R0 trong không khí sạch
                Serial.println("[ENV_MON] Button very long-press (>= 5s) -> Calibrating Clean Air R0");
                s_mq_calibrate_requested = true;
            } else if (duration_ms >= BUTTON_LONG_MS) {
                // Nhấn giữ 1.5s - 5.0s: Chạy tự kiểm tra Self-Test
                Serial.println("[ENV_MON] Button long-press -> Requesting Self-Test");
                s_selftest_requested = true;
            } else if (duration_ms >= BUTTON_DEBOUNCE_MS) {
                // Nhấn nhả nhanh (40ms - 1500ms): Tắt còi, snooze cảnh báo và cưỡng bức tắt quạt 60s
                Serial.printf("[ENV_MON] Button short-press -> Reset Alarm & Snooze %d s\n", SNOOZE_MS / 1000);

                lock_state();
                g_state.snooze_until_us = t + ((int64_t)SNOOZE_MS * 1000);
                if (g_state.fan_mode == FAN_FORCE_ON) {
                    g_state.fan_mode = FAN_AUTO;
                }
                g_state.fan_on = false;
                g_state.fan_locked = false;
                g_state.fan_protect_rem_s = 0;
                unlock_state();

                // Lập tức ngắt quạt và còi phần cứng ngay lập tức
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

// Tác vụ màn hình OLED 3 trang tự động luân chuyển (chu kỳ 5 giây)
static void oled_task(void *arg) {
    int page = 0;
    while (1) {
        app_state_t s;
        lock_state();
        s = g_state;
        unlock_state();

        if (!s_oled_ready) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (xSemaphoreTake(s_oled_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            char line[32];
            oled_clear();
            if (page == 0) {
                // Trang 0: Thông số môi trường
                oled_draw_text(0, 0, "ENVIRONMENT", false);
                if (s.dht_valid) {
                    snprintf(line, sizeof(line), "TEMP %.1fC (L%d)", s.temperature_c, (int)s.temp_level);
                    oled_draw_text(0, 16, line, false);
                    snprintf(line, sizeof(line), "HUM  %.1f%% (L%d)", s.humidity_pct, (int)s.hum_level);
                    oled_draw_text(0, 28, line, false);
                    snprintf(line, sizeof(line), "HEAT %.1fC", s.heat_index_c);
                    oled_draw_text(0, 40, line, false);
                    snprintf(line, sizeof(line), "MOLD %.0f%%", s.mold_risk_pct);
                    oled_draw_text(0, 52, line, false);
                } else {
                    oled_draw_text(0, 20, "DHT ERROR", true);
                }
            } else if (page == 1) {
                // Trang 1: Khí Gas & Trạng thái quạt
                oled_draw_text(0, 0, "AIR & FAN", false);
                snprintf(line, sizeof(line), "K:%.2f L%d", s.mq_ratio_k, (int)s.mq_level);
                oled_draw_text(0, 14, line, false);
                snprintf(line, sizeof(line), "Rs:%.1fk R0:%.1fk", s.mq_rs, s.mq_r0);
                oled_draw_text(0, 26, line, false);
                if (s.fan_locked && s.fan_protect_rem_s > 0) {
                    snprintf(line, sizeof(line), "FAN %s (%ds)", s.fan_on ? "ON" : "OFF", s.fan_protect_rem_s);
                } else {
                    snprintf(line, sizeof(line), "FAN %s (%s)", s.fan_on ? "ON" : "OFF", fan_mode_str(s.fan_mode));
                }
                oled_draw_text(0, 38, line, false);
                snprintf(line, sizeof(line), "SYS LVL %d %s", (int)s.env_level, alarm_snoozed() ? "SNOOZE" : env_level_str(s.env_level));
                oled_draw_text(0, 50, line, false);
            } else {
                // Trang 2: Trạng thái hệ thống
                oled_draw_text(0, 0, "SYSTEM", false);
                snprintf(line, sizeof(line), "WIFI %s", s.wifi_connected ? "OK" : "OFF");
                oled_draw_text(0, 14, line, false);
                snprintf(line, sizeof(line), "RSSI %d", s.wifi_rssi);
                oled_draw_text(0, 26, line, false);
                snprintf(line, sizeof(line), "WARM %s", s.warmup_done ? "OK" : "WAIT");
                oled_draw_text(0, 38, line, false);
                snprintf(line, sizeof(line), "RAW ADC %d", s.mq_raw);
                oled_draw_text(0, 50, line, false);
            }
            oled_update();
            xSemaphoreGive(s_oled_mutex);
        }

        page = (page + 1) % 3;
        vTaskDelay(pdMS_TO_TICKS(OLED_PAGE_PERIOD_MS));
    }
}

// Tác vụ thu thập cảm biến, Sensor Fusion & Điều khiển quạt Anti-Short-Cycle (chu kỳ 2.5 giây)
static void sensor_control_task(void *arg) {
    int64_t next_sample = esp_timer_get_time();

    while (1) {
        if (s_selftest_requested) {
            s_selftest_requested = false;
            self_test_sequence();
        }

        int64_t now = esp_timer_get_time();
        bool warmup_done = (now - g_state.boot_us) >= ((int64_t)MQ_WARMUP_MS * 1000);

        // 1. Đọc dữ liệu cảm biến DHT22
        float temp = dht.readTemperature();
        float hum = dht.readHumidity();
        bool dht_ok = !isnan(temp) && !isnan(hum);

        // 2. Đọc tín hiệu Analog ADC từ cảm biến MQ-135
        int mq_raw = analogRead(PIN_MQ135_AO);
        float mq_rs = mq135_calculate_rs(mq_raw);

        // Xử lý yêu cầu hiệu chuẩn R0 trong không khí sạch
        if (s_mq_calibrate_requested) {
            s_mq_calibrate_requested = false;
            mq_r0_save(mq_rs);
            for (int i = 0; i < MQ_FILTER_SAMPLES; i++) {
                s_mq_samples[i] = 1.0f;
            }
            s_mq_sample_count = MQ_FILTER_SAMPLES;
            s_mq_prev_level = ENV_LEVEL_1_NORMAL;
            Serial.printf("[ENV_MON] >>> CALIBRATION SUCCESS: R0 set to %.2f kOhm (Raw ADC=%d) <<<\n", s_mq_r0, mq_raw);
        }

        // Lọc trung bình trượt 50s (20 mẫu) và xác định Level MQ-135
        float mq_k = mq_filter_update(mq_rs, s_mq_r0);
        env_level_t mq_lvl = get_mq_level(mq_k, s_mq_prev_level);
        s_mq_prev_level = mq_lvl;

        env_level_t temp_lvl = ENV_LEVEL_1_NORMAL;
        env_level_t hum_lvl = ENV_LEVEL_1_NORMAL;

        if (dht_ok) {
            temp_lvl = get_temp_level(temp, s_temp_prev_level);
            hum_lvl = get_hum_level(hum, s_hum_prev_level);
            s_temp_prev_level = temp_lvl;
            s_hum_prev_level = hum_lvl;
        }

        // Hợp nhất đa cảm biến với hiệu ứng cộng hưởng (Synergy Escalation)
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
            g_state.dht_valid = false;
            Serial.println("[ENV_MON] DHT22 read failed (returned NaN)");
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

        // 3. Đánh giá nhu cầu bật quạt từ môi trường (Xác nhận 2 mẫu liên tiếp)
        // Chặn bật quạt ở ngưỡng dưới (Lower-Threshold Fan Inhibit):
        // Ở mức 3 và mức 4, quạt KHÔNG bật nếu nhiệt độ (<23°C / <25°C) hoặc độ ẩm (<40% / <50%) đang thấp
        bool is_temp_lower = dht_ok && ((temp_lvl >= ENV_LEVEL_3_WARNING && temp < 25.0f) || (temp < 23.0f));
        bool is_hum_lower  = dht_ok && ((hum_lvl >= ENV_LEVEL_3_WARNING && hum < 50.0f) || (hum < 40.0f));

        bool desired_fan = false;
        if (is_temp_lower || is_hum_lower) {
            s_warning_persist_count = 0;
            desired_fan = false;
        } else if (level >= ENV_LEVEL_4_CRITICAL) {
            desired_fan = true;
            s_warning_persist_count = FAN_TRIGGER_PERSIST;
        } else if (level >= ENV_LEVEL_3_WARNING) {
            if (++s_warning_persist_count >= FAN_TRIGGER_PERSIST) {
                desired_fan = true;
            }
        } else {
            s_warning_persist_count = 0;
            desired_fan = false;
        }

        // 4. Thuật toán Anti-Short-Cycle bảo vệ động cơ quạt và relay
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
                    // Người dùng bấm Snooze -> Ngắt quạt ngay lập tức
                    if (s_fan_is_on) {
                        fan_on = false;
                        s_fan_last_off_us = now_us;
                        Serial.println("[ENV_MON] Fan shut down immediately by USER SNOOZE");
                    }
                } else if (s_fan_is_on) {
                    // Quạt ĐANG CHẠY: nếu môi trường hết cảnh báo (desired_fan == false)
                    if (!desired_fan) {
                        int64_t run_time_ms = (now_us - s_fan_last_on_us) / 1000;
                        if (run_time_ms >= FAN_MIN_RUN_TIME_MS) {
                            fan_on = false;
                            s_fan_last_off_us = now_us;
                            Serial.printf("[ENV_MON] Fan MIN_RUN reached (%lld ms) -> Turning OFF\n", run_time_ms);
                        } else {
                            // Khóa bảo vệ MIN_RUN: quạt tiếp tục chạy cho đủ thời gian 60s
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
                            Serial.printf("[ENV_MON] Fan MIN_REST reached (%lld ms) -> Turning ON\n", rest_time_ms);
                        } else {
                            // Khóa bảo vệ MIN_REST: quạt tiếp tục nghỉ cho đủ thời gian 60s
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
            Serial.printf("[ENV_MON] T=%.1fC(L%d%s) RH=%.1f%%(L%d%s) K=%.2f(L%d) -> Level=%s(%d) Fan=%s(%s%ds) Mode=%s RSSI=%d\n",
                          temp, (int)temp_lvl, is_temp_lower ? ":LOW" : "",
                          hum, (int)hum_lvl, is_hum_lower ? ":LOW" : "",
                          mq_k, (int)mq_lvl,
                          env_level_str(level), (int)level, fan_on ? "ON" : "OFF",
                          fan_locked ? "PROT:" : "", fan_locked ? fan_protect_rem_s : 0,
                          fan_mode_str(s.fan_mode), s.wifi_rssi);
        } else {
            Serial.printf("[ENV_MON] DHT FAIL | K=%.2f(L%d) -> Level=%s(%d) Fan=%s Mode=%s RSSI=%d\n",
                          mq_k, (int)mq_lvl, env_level_str(level), (int)level, fan_on ? "ON" : "OFF", fan_mode_str(s.fan_mode), s.wifi_rssi);
        }

        next_sample += ((int64_t)SAMPLE_PERIOD_MS * 1000);
        int64_t delay_us = next_sample - esp_timer_get_time();
        if (delay_us < 0) delay_us = 0;
        vTaskDelay(pdMS_TO_TICKS((uint32_t)(delay_us / 1000)));
    }
}

// =========================================================================================
// 11. KHỞI TẠO HỆ THỐNG (SETUP) & VÒNG LẶP CHÍNH (LOOP)
// =========================================================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================================");
    Serial.println("  IoT-Based Environment Monitoring (Arduino ESP32)");
    Serial.println("========================================================\n");

    s_state_mutex = xSemaphoreCreateMutex();
    s_oled_mutex  = xSemaphoreCreateMutex();
    assert(s_state_mutex != NULL && s_oled_mutex != NULL);

    g_state.boot_us = esp_timer_get_time();

    // 1. Nạp ngưỡng và giá trị R0 từ bộ nhớ NVS Flash
    thresholds_load();
    mq_r0_load();

    // 2. Khởi tạo GPIO phần cứng
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    relay_set(false);
    buzzer_stop();

    // Tự kiểm tra phần cứng đơn giản tại lúc khởi động: chớp 2 LED
    digitalWrite(PIN_LED_RED, HIGH);
    delay(300);
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, HIGH);
    delay(300);
    digitalWrite(PIN_LED_GREEN, LOW);

    // 3. Khởi tạo ADC đọc MQ-135 (12-bit, suy hao 11/12 dB)
    analogReadResolution(12);
#if defined(SOC_ADC_ATTEN_DB_12)
    analogSetAttenuation(ADC_12db);
#else
    analogSetAttenuation(ADC_11db);
#endif

    // 4. Khởi tạo cảm biến nhiệt ẩm DHT22
    dht.begin();

    // 5. Khởi tạo giao tiếp I2C & màn hình OLED SSD1306
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL, 100000);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_PRIMARY)) {
        Serial.printf("[ENV_MON] OLED not responding at 0x%02X, trying 0x%02X...\n", OLED_ADDR_PRIMARY, OLED_ADDR_SECONDARY);
        if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR_SECONDARY)) {
            Serial.printf("[ENV_MON] OLED init failed at both 0x%02X and 0x%02X. Continuing without OLED.\n",
                          OLED_ADDR_PRIMARY, OLED_ADDR_SECONDARY);
            s_oled_ready = false;
        } else {
            s_oled_ready = true;
            Serial.printf("[ENV_MON] OLED ready at 0x%02X\n", OLED_ADDR_SECONDARY);
        }
    } else {
        s_oled_ready = true;
        Serial.printf("[ENV_MON] OLED ready at 0x%02X\n", OLED_ADDR_PRIMARY);
    }

    if (s_oled_ready) {
        display.clearDisplay();
        display.display();
    }

    // 6. Kết nối Wi-Fi Station
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[ENV_MON] Connecting to Wi-Fi");
    unsigned long start_wifi = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_wifi < 15000)) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[ENV_MON] Wi-Fi connected! IP address: ");
        Serial.println(WiFi.localIP());
        lock_state();
        g_state.wifi_connected = true;
        g_state.wifi_rssi = WiFi.RSSI();
        unlock_state();
    } else {
        Serial.println("[ENV_MON] Wi-Fi connection timed out. Running in offline mode.");
        lock_state();
        g_state.wifi_connected = false;
        g_state.wifi_rssi = 0;
        unlock_state();
    }

    // 7. Khởi động Web Server và đăng ký các REST API endpoints
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/state", HTTP_GET, handleState);
    server.on("/api/control", HTTP_GET, handleControl);
    server.on("/api/thresholds", HTTP_GET, handleThresholds);
    server.begin();
    Serial.println("[ENV_MON] HTTP REST server started on port 80");

    // 8. Tạo các FreeRTOS Tasks đa nhiệm chạy độc lập
    xTaskCreatePinnedToCore(sensor_control_task, "sensor_ctrl", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(oled_task, "oled", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(button_task, "button", 2048, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(alarm_task, "alarm", 2048, NULL, 4, NULL, 1);
}

void loop() {
    // Phục vụ các yêu cầu HTTP từ Web Dashboard hoặc Mobile App
    server.handleClient();

    // Định kỳ cập nhật trạng thái kết nối Wi-Fi và cường độ tín hiệu RSSI
    static unsigned long last_rssi_check = 0;
    if (millis() - last_rssi_check >= 3000) {
        last_rssi_check = millis();
        lock_state();
        if (WiFi.status() == WL_CONNECTED) {
            g_state.wifi_connected = true;
            g_state.wifi_rssi = WiFi.RSSI();
        } else {
            g_state.wifi_connected = false;
            g_state.wifi_rssi = 0;
        }
        unlock_state();
    }

    delay(5);
}
