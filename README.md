# IoT Environment Monitor - ESP-IDF

ESP32 WROOM-32 (30-pin) firmware for the group's IoT-Based Environment Monitoring project.

## Table of Contents
- [Features](#features)
- [Pin Mapping](#pin-mapping)
- [Power Architecture & Wiring Safety](#power-architecture--wiring-safety)
- [Environmental Classification & Alert Levels](#environmental-classification--alert-levels)
- [Anti-Short-Cycle Smart Fan Control](#anti-short-cycle-smart-fan-control)
- [Sensor Processing & Multi-Sensor Fusion](#sensor-processing--multi-sensor-fusion)
- [Push Button Operation](#push-button-operation)
- [OLED Multi-Page Interface](#oled-multi-page-interface)
- [Web Dashboard & REST API](#web-dashboard--rest-api)
- [NVS Non-Volatile Storage](#nvs-non-volatile-storage)
- [Hardware Diagnostics & Boot Sequence](#hardware-diagnostics--boot-sequence)
- [Build, Configuration & Flash Guide](#build-configuration--flash-guide)

---

## Features

- **Multi-Sensor Environmental Sensing**:
  - DHT22 (AM2302) for high-precision temperature and relative humidity measurement.
  - MQ-135 air quality gas sensor measuring hazardous gases, air staleness, and smoke.
  - Calculated **Heat Index** (NOAA Rothfusz regression) and **Indoor Mold Risk Index** (0–100%).
  - Real-time **Comfort Score** evaluation.
- **Smart Anti-Short-Cycle Relay/Fan Automation**:
  - Minimum Run Time (60 s) and Minimum Rest Time (60 s) protect the fan motor and relay contacts.
  - Hysteresis thresholds and multi-sample persistence prevent rapid bouncing/relay chatter.
  - Immediate user snooze override.
- **Multi-Sensor Fusion & Synergy Escalation**:
  - Evaluates temperature, humidity, and gas air quality levels concurrently.
  - Synergy logic detects combined stuffy/hot air or compound risks and escalates alerts automatically.
- **4-Level Safety Indication**:
  - Visual status via Green and Red LEDs (solid, single blink, alternating blink, fast alert).
  - Active audio alert via 3.3V passive buzzer with PWM tone generation (triggers strictly on critical gas/smoke).
- **Physical Push Button Controls**:
  - Short press: Snooze alarm and force fan off for 60 s.
  - Long press (1.5 s – 5 s): Trigger hardware self-test routine.
  - Very long press (≥ 5 s): Zero/calibrate MQ-135 baseline ($R_0$) in clean air.
- **SSD1306 0.96" I2C OLED Display**:
  - 3 auto-cycling display screens (Environment, Air & Fan, System Status) every 5 seconds.
  - Dual address auto-detection (`0x3C` and `0x3D`) with non-blocking error tolerance.
- **Embedded Web Dashboard & REST API**:
  - Mobile-responsive web UI served directly from ESP32 flash (`/`).
  - Real-time telemetry JSON endpoint (`/api/state`).
  - Remote control endpoint (`/api/control`) for manual fan override, snooze, self-test, and calibration.
  - Remote threshold tuning endpoint (`/api/thresholds`) saved to non-volatile storage.
- **NVS Non-Volatile Storage**:
  - Persists custom fan/alarm thresholds and calibrated MQ-135 $R_0$ values across power cycles.

---

## Pin Mapping

ESP32 WROOM-32 (30-pin DevKit):

| Peripheral / Signal | ESP32 GPIO | Description / Conditioning |
| :--- | :--- | :--- |
| **DHT22 DATA** | `GPIO4` | Single-bus bidirectional data line, configured Open-Drain |
| **MQ-135 AO** | `GPIO34` (ADC1_CH6) | Analog output connected through **10kΩ / 10kΩ voltage divider** |
| **OLED SSD1306 SDA** | `GPIO21` | I2C Data (internal pull-ups enabled, 100 kHz) |
| **OLED SSD1306 SCL** | `GPIO22` | I2C Clock (internal pull-ups enabled, 100 kHz) |
| **Relay IN** | `GPIO16` | Digital output driving relay module (`RELAY_ACTIVE_LOW 0` by default) |
| **Passive Buzzer** | `GPIO17` | Driven via LEDC PWM (Timer 0, Channel 0, 2000–2400 Hz) |
| **Red LED** | `GPIO25` | Output through 220Ω current-limiting resistor to anode |
| **Green LED** | `GPIO26` | Output through 220Ω current-limiting resistor to anode |
| **Push Button** | `GPIO27` | Connected to GND, internal pull-up enabled (Active-LOW) |

---

## Power Architecture & Wiring Safety

1. **MB102 Breadboard Power Supply Module (+5V Rail)**:
   - Powers the **MQ-135 internal heater coil** (requires steady 5V ~150 mA).
   - Powers the **Relay module VCC** and **exhaust fan motor**.
2. **ESP32 3.3V Pin**:
   - Powers the **DHT22 sensor** and **SSD1306 OLED display**.
3. **Passive Buzzer (3.3V)**:
   - Driven directly from `GPIO17` via LEDC PWM (low current draw).
4. **Common Ground**:
   - **ESP32 GND and MB102 GND must be connected together.** Floating grounds will cause erroneous ADC readings and I2C/relay glitching.
5. **Voltage Divider on MQ-135 AO**:
   - The MQ-135 runs on 5V, meaning its analog output pin (AO) can swing up to ~5V.
   - ESP32 ADC pins are **NOT 5V tolerant** (max 3.3V, ADC linear range up to ~2.5V with 12 dB attenuation).
   - Wire a **10kΩ / 10kΩ resistor divider** between MQ-135 AO and GND, tapping the midpoint to `GPIO34`.

---

## Environmental Classification & Alert Levels

The firmware continuously classifies the environment into 4 distinct operational states:

| Level | Name | Trigger Condition | Visual Indicator (LEDs) | Audio Alert (Buzzer) | Fan Action |
| :---: | :---: | :--- | :--- | :--- | :--- |
| **1** | **NORMAL** | Temperature: 25.0°C – 29.0°C<br>Humidity: 50% – 70%<br>Gas Ratio $K \ge 0.85$ | **Green LED ON solid**<br>Red LED OFF | Silent | Standby (OFF) |
| **2** | **NOTICE** | Temperature: 23–25°C or 29–31°C<br>Humidity: 40–50% or 70–75%<br>Gas Ratio $K$: 0.65 – 0.85 | **Green LED blinking** (1.0 s period: 500 ms ON / 500 ms OFF)<br>Red LED OFF | Silent | Standby (OFF unless synergy triggered) |
| **3** | **WARNING** | Temperature: 21–23°C or 31–33°C<br>Humidity: 30–40% or 75–85%<br>Gas Ratio $K$: 0.45 – 0.65 | **Green & Red LEDs alternating** (800 ms period: 400 ms Green / 400 ms Red) | Silent | **Ventilation Fan ON** (after 2 consecutive samples)<br>*(Standby/OFF nếu nhiệt độ hoặc độ ẩm đang ở ngưỡng dưới)* |
| **4** | **CRITICAL** | Temperature: <21°C or >33°C<br>Humidity: <30% or >85%<br>Gas Ratio $K < 0.45$ (Dangerous Gas/Smoke) | **Red LED flashing** (700 ms period: 350 ms ON / 350 ms OFF)<br>Green LED OFF | **2400 Hz tone beeps synchronously with Red LED**<br>*(Only sounds if Gas is Level 4; silent for Temp/Hum critical to avoid noise nuisance)* | **Ventilation Fan ON immediately**<br>*(Standby/OFF nếu nhiệt độ hoặc độ ẩm đang ở ngưỡng dưới)* |

> [!NOTE]
> All level transitions incorporate **Hysteresis** (0.6°C for temperature, 3.0% for humidity, and 0.02 for gas ratio $K$) to prevent rapid oscillation around boundary thresholds.

---

## Anti-Short-Cycle Smart Fan Control

To protect fan motors against burnout and prevent relay contact pitting from frequent switching, the firmware implements an industrial-grade **Anti-Short-Cycle** state machine:

- **Lower-Threshold Fan Inhibit**:
  Ở Mức 3 (Warning) và Mức 4 (Critical), quạt **KHÔNG BẬT** nếu nhiệt độ (hoặc độ ẩm hoặc cả hai) đang ở ngưỡng dưới (Lạnh: $<23^\circ\text{C}$ / $<21^\circ\text{C}$; Khô: $<40\%$ / $<30\%$) để tránh làm không gian lạnh hoặc khô hơn. Quạt chỉ tự động kích hoạt khi có nhiệt độ cao (nóng), độ ẩm cao (nồm ẩm/nấm mốc) hoặc nồng độ khí gas nguy hại.
- **Minimum Run Time (`FAN_MIN_RUN_TIME_MS = 60000` ms)**:
  Once the fan turns ON automatically, it must run for at least 60 seconds before it can turn OFF, even if sensor readings return to normal.
- **Minimum Rest Time (`FAN_MIN_REST_TIME_MS = 60000` ms)**:
  Once the fan turns OFF, it must remain resting for at least 60 seconds before it can be re-triggered.
- **Trigger Persistence (`FAN_TRIGGER_PERSIST = 2` samples / 5 s)**:
  At Level 3 (Warning), the condition must persist across 2 consecutive measurement cycles before the fan engages.
- **Emergency User Override**:
  Pressing the physical button or sending `snooze`/`off` via the Web UI immediately overrides and shuts down the fan and buzzer without waiting for lock timers.

---

## Sensor Processing & Multi-Sensor Fusion

### 1. DHT22 (AM2302)
- Read cycle: Every 2.5 seconds (sample period).
- Calculates:
  - **NOAA/Rothfusz Heat Index** ($^\circ\text{C}$): Apparent temperature considering evaporative cooling limits.
  - **Indoor Mold Risk Score** (0–100%): Empirical assessment based on sustained high humidity ($\ge 60\%$) and moderate temperatures.
  - **Thermal Comfort Score**: 0 (Poor), 1 (Uncomfortable), 2 (Acceptable), 3 (Comfortable).

### 2. MQ-135 Gas Sensor
- **Sensor Resistance ($R_s$)**:
  $$R_s = R_L \times \frac{4095 - \text{ADC}}{\text{ADC}} \quad (R_L = 10.0\text{ k}\Omega)$$
- **Air Quality Ratio ($K$)**:
  $$K = \frac{R_s}{R_0}$$
  A higher $K$ indicates clean air; lower $K$ indicates high gas/VOC/smoke concentration.
- **Moving Average Filter**: Ratio $K$ is smoothed over 20 samples (50-second moving window) to eliminate transient noise spikes.
- **Warm-Up Gate**: Enforces a 60-second hardware pre-heat timer upon boot before relying on gas readings for automatic control.

### 3. Multi-Sensor Synergy Fusion
The global environment level is derived through `evaluate_combined_env_level()`:
1. **Base Level**: $\max(\text{Temp Level}, \text{Hum Level}, \text{MQ Level})$.
2. **Synergy Rule A (Stuffy/Humid Air Escalation)**: If all 3 metrics are at Level 2 (Notice), or 2 metrics are at Level 2 with one being the gas sensor, the system escalates to **Level 3 (Warning)** to initiate ventilation.
3. **Synergy Rule B (Compound Hazard Escalation)**: If 2 or more sensors simultaneously reach Level 3 (Warning), the system escalates to **Level 4 (Critical)**.

---

## Push Button Operation

The push button on `GPIO27` (active-LOW with internal pull-up and 40 ms software debounce) supports 3 interaction modes:

| Duration | Action | Firmware Response |
| :--- | :--- | :--- |
| **Short Press**<br>*(40 ms – 1500 ms)* | **Snooze / Silence** | Silences the buzzer, forces fan OFF, and activates a 60-second snooze window (`SNOOZE_MS`). |
| **Long Press**<br>*(1.5 s – 5.0 s)* | **Self-Test Routine** | Runs peripheral test: Green LED (250 ms, 1000 Hz tone) $\rightarrow$ Red LED (250 ms, 1800 Hz tone) $\rightarrow$ clicks relay ON for 500 ms. |
| **Very Long Press**<br>*( $\ge$ 5.0 s)* | **Clean Air Calibration** | Measures current $R_s$, saves it as new baseline reference $R_0$ into NVS flash, and resets filter buffers. |

---

## OLED Multi-Page Interface

The SSD1306 128x64 display automatically cycles through 3 information pages every 5 seconds (`OLED_PAGE_PERIOD_MS`):

````
+---------------------+    +---------------------+    +---------------------+
| ENVIRONMENT         |    | AIR & FAN           |    | SYSTEM              |
| TEMP 28.4C (L1)     | -> | K:0.92 L1           | -> | WIFI OK             |
| HUM  62.1% (L1)     |    | Rs:36.8k R0:40.0k   |    | RSSI -58            |
| HEAT 30.1C          |    | FAN OFF (AUTO)      |    | WARM OK             |
| MOLD 15%            |    | SYS LVL 1 NORM      |    | RAW ADC 842         |
+---------------------+    +---------------------+    +---------------------+
       Page 0                     Page 1                     Page 2
````

- If the OLED is disconnected or fails I2C communication, the firmware logs a warning and continues running headlessly without halting.

---

## Web Dashboard & REST API

When Wi-Fi is connected, the ESP32 hosts an HTTP server on port 80. Obtain the assigned IP address from the serial monitor logs:
```
I (xxxx) ENV_MON: Wi-Fi IP: 192.168.x.x
```
Navigate to `http://<ESP32-IP>/` in any browser on the local network.

### Web Dashboard Features
- Live system status cards with automated AJAX refresh (2-second interval).
- Interactive control buttons: `AUTO`, `ON`, `OFF`, `SNOOZE`, `SELF TEST`, and `CALIBRATE R0 (CLEAN AIR)`.
- Online threshold adjustment form with `SAVE` button.

### REST API Endpoints

#### 1. Telemetry State
`GET /api/state`
```json
{
  "temperature_c": 28.4,
  "humidity_pct": 62.1,
  "heat_index_c": 30.1,
  "mold_risk_pct": 15.0,
  "comfort_level": 3,
  "mq_raw": 842,
  "mq_rs": 36.82,
  "mq_r0": 40.00,
  "mq_ratio_k": 0.92,
  "mq_level": 1,
  "temp_level": 1,
  "hum_level": 1,
  "fan": false,
  "fan_mode": "AUTO",
  "fan_locked": false,
  "fan_protect_rem_s": 0,
  "alarm": false,
  "warning": false,
  "env_level": 1,
  "env_level_str": "NORM",
  "snoozed": false,
  "wifi": true,
  "rssi": -58,
  "warmup_done": true
}
```

#### 2. Device Control
`GET /api/control?cmd=<command>`
- `auto`: Switch fan back to automatic mode.
- `on`: Manually force fan ON.
- `off`: Manually force fan OFF.
- `snooze`: Mute buzzer and turn off fan for 60 seconds.
- `test`: Trigger hardware self-test routine.
- `calib` or `calibrate`: Calibrate MQ-135 $R_0$ in clean air and store to NVS.

#### 3. Threshold Configuration
`GET /api/thresholds?to=...&tf=...&ho=...&hf=...&go=...&gf=...&tc=...&hc=...&gc=...`
- `to`: Fan Temp ON trigger (°C)
- `tf`: Fan Temp OFF trigger (°C)
- `ho`: Fan Humidity ON trigger (%)
- `hf`: Fan Humidity OFF trigger (%)
- `go`: Fan Gas Raw ADC ON trigger
- `gf`: Fan Gas Raw ADC OFF trigger
- `tc`: Critical Temperature (°C)
- `hc`: Critical Humidity (%)
- `gc`: Critical Gas Raw ADC

---

## NVS Non-Volatile Storage

The firmware uses the ESP-IDF Non-Volatile Storage (NVS) library under the namespace `"envmon"`:
- **`thresholds`**: Binary blob storing user-configured temperature, humidity, and gas triggers.
- **`mq_r0`**: Floating point value storing calibrated clean-air baseline resistance $R_0$.

Data is preserved across power resets and reboots. If NVS is unformatted or corrupted, the firmware initializes default thresholds automatically.

---

## Hardware Diagnostics & Boot Sequence

Upon boot, the firmware executes a safety sanity test before initializing I2C or Wi-Fi:
1. **LED Sequence**:
   - Red LED turns ON for ~300 ms, then OFF.
   - Green LED turns ON for ~300 ms, then OFF.
2. **I2C OLED Probe**:
   - Attempts address `0x3C`. If no ACK is received, automatically tries `0x3D`.
   - If no display is detected, logs error and proceeds headlessly without crashing.
3. **Wi-Fi Connection Check**:
   - Tries connecting up to 10 times. If connection fails, the device continues fully operational in offline local mode.

### Wiring Troubleshooting
- **Neither LED flashes at boot**: Check LED polarity, verify 220Ω resistor connection to `GPIO25`/`GPIO26`, and verify ESP32 GND.
- **LEDs flash but OLED stays blank**: Verify OLED VCC is connected to **3.3V** (not 5V), check SDA on `GPIO21`, SCL on `GPIO22`.
- **Relay inverted (ON when it should be OFF)**: Toggle `#define RELAY_ACTIVE_LOW` in `main/main.c` (`0` for Active-High, `1` for Active-Low).
- **Erratic MQ-135 ADC readings**: Ensure MB102 power ground and ESP32 ground are securely tied together, and ensure 10kΩ/10kΩ divider is installed.

---

## Build, Configuration & Flash Guide

### Prerequisites
- ESP-IDF v5.x or v6.x toolchain installed.
- VS Code with official Espressif IDF extension (optional, or use command line).

### Step-by-Step Instructions

1. **Configure Wi-Fi Credentials**:
   Open `main/main.c` and update:
   ```c
   #define WIFI_SSID       "YOUR_WIFI_SSID"
   #define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
   ```

2. **Select ESP32 Target**:
   ```bash
   idf.py set-target esp32
   ```

3. **Build Firmware**:
   ```bash
   idf.py build
   ```

4. **Flash to ESP32**:
   ```bash
   idf.py -p COM_PORT flash
   ```
   *(Replace `COM_PORT` with your serial port, e.g., `COM3` on Windows or `/dev/ttyUSB0` on Linux).*

5. **Open Serial Monitor**:
   ```bash
   idf.py -p COM_PORT monitor
   ```
   *(Press `Ctrl+]` to exit the monitor).*
