# Hệ Thống Giám Sát Môi Trường IoT - ESP-IDF

Mã nguồn firmware trên ESP32 WROOM-32 (bản 30 chân) dành cho dự án Giám sát môi trường thông minh dựa trên IoT.

## Mục lục
- [1. Tính năng nổi bật](#1-tính-năng-nổi-bật)
- [2. Sơ đồ nối chân (Pin Mapping)](#2-sơ-đồ-nối-chân-pin-mapping)
- [3. Cấu trúc nguồn điện & An toàn đấu dây](#3-cấu-trúc-nguồn-điện--an-toàn-đấu-dây)
- [4. Phân loại môi trường & Các cấp độ cảnh báo](#4-phân-loại-môi-trường--các-cấp-độ-cảnh-báo)
- [5. Điều khiển quạt thông minh chống bật/tắt liên tục (Anti-Short-Cycle)](#5-điều-khiển-quạt-thông-minh-chống-bậttắt-liên-tục-anti-short-cycle)
- [6. Xử lý cảm biến & Thuật toán kết hợp (Multi-Sensor Fusion)](#6-xử-lý-cảm-biến--thuật-toán-kết-hợp-multi-sensor-fusion)
- [7. Hướng dẫn sử dụng nút nhấn](#7-hướng-dẫn-sử-dụng-nút-nhấn)
- [8. Giao diện màn hình OLED đa trang](#8-giao-diện-màn-hình-oled-đa-trang)
- [9. Giao diện Web Dashboard & REST API](#9-giao-diện-web-dashboard--rest-api)
- [10. Lưu trữ bộ nhớ NVS (Non-Volatile Storage)](#10-lưu-trữ-bộ-nhớ-nvs-non-volatile-storage)
- [11. Tự kiểm tra phần cứng & Quá trình khởi động](#11-tự-kiểm-tra-phần-cứng--quá-trình-khởi-động)

---

## 1. Tính năng nổi bật

- **Cảm biến môi trường đa thông số**:
  - DHT22 (AM2302) đo nhiệt độ và độ ẩm tương đối với độ chính xác cao.
  - Cảm biến khí gas MQ-135 phát hiện các khí độc hại, không khí tù đọng/ngột ngạt và khói.
  - Tự động tính toán **Chỉ số nhiệt Heat Index** (theo hồi quy NOAA Rothfusz) và **Nguy cơ nấm mốc trong nhà** (0–100%).
  - Đánh giá **Điểm thoải mái (Comfort Score)** theo thời gian thực.
- **Tự động hóa quạt/relay thông minh chống bật/tắt liên tục (Anti-Short-Cycle)**:
  - Thời gian chạy tối thiểu (60 giây) và thời gian nghỉ tối thiểu (60 giây) giúp bảo vệ động cơ quạt và tiếp điểm relay.
  - Vùng trễ (Hysteresis) và kiểm tra nhiều mẫu liên tiếp giúp chống việc đóng/ngắt rơ-le liên tục khi chỉ số dao động quanh ngưỡng.
  - Cho phép người dùng tạm dừng cảnh báo (snooze) và tắt cưỡng bức quạt ngay lập tức.
- **Kết hợp đa cảm biến & Nâng cấp cảnh báo tương hỗ (Multi-Sensor Fusion)**:
  - Đánh giá đồng thời cả 3 yếu tố: nhiệt độ, độ ẩm và nồng độ khí gas.
  - Thuật toán nhận diện tình trạng không khí vừa nóng vừa bí hoặc các nguy cơ kết hợp để tự động kích hoạt mức cảnh báo cao hơn.
- **Hệ thống cảnh báo an toàn 4 cấp độ**:
  - Đèn LED trực quan với 2 màu Xanh và Đỏ (sáng đứng, chớp đơn, chớp luân phiên, chớp nhanh báo động).
  - Còi chip thụ động (Passive Buzzer 3.3V) phát âm thanh PWM (chỉ kêu khi khí gas/khói đạt mức nguy hiểm, tránh gây ồn không cần thiết).
- **Điều khiển tiện lợi bằng nút nhấn vật lý**:
  - Nhấn nhanh: Tạm hoãn báo động (Snooze) và cưỡng bức tắt quạt trong 60 giây.
  - Nhấn giữ (1.5 s – 5 s): Chạy quy trình tự kiểm tra phần cứng (Self-Test).
  - Nhấn giữ lâu (≥ 5 s): Cân chỉnh (Calibrate) mốc không khí sạch ($R_0$) cho cảm biến MQ-135.
- **Màn hình OLED SSD1306 0.96" I2C**:
  - Tự động luân chuyển giữa 3 trang hiển thị (Môi trường, Khí gas & Quạt, Trạng thái hệ thống) mỗi 5 giây.
  - Tự động dò tìm địa chỉ I2C (`0x3C` và `0x3D`), không làm treo hệ thống nếu mất kết nối màn hình.
- **Tích hợp Web Dashboard & REST API**:
  - Giao diện web trực quan, tương thích tốt với điện thoại, chạy trực tiếp từ bộ nhớ flash của ESP32 (`/`).
  - Cung cấp API dữ liệu thời gian thực dạng JSON (`/api/state`).
  - API điều khiển từ xa (`/api/control`) hỗ trợ bật/tắt quạt thủ công, tạm dừng cảnh báo (snooze), tự kiểm tra và cân chỉnh cảm biến.
  - API cài đặt ngưỡng kích hoạt (`/api/thresholds`) lưu trực tiếp vào bộ nhớ flash.
- **Lưu trữ bộ nhớ NVS (Non-Volatile Storage)**:
  - Lưu lại toàn bộ ngưỡng kích hoạt quạt/cảnh báo và giá trị $R_0$ đã cân chỉnh của cảm biến MQ-135, không bị mất dữ liệu khi mất điện hay khởi động lại.

---

## 2. Sơ đồ nối chân (Pin Mapping)

Áp dụng cho board ESP32 WROOM-32 (loại 30 chân DevKit):

| Ngoại vi / Tín hiệu | Chân ESP32 GPIO | Mô tả / Lưu ý mạch |
| :--- | :--- | :--- |
| **DHT22 DATA** | `GPIO4` | Dây truyền dữ liệu 1-wire hai chiều, cấu hình ngõ ra cực thu hở (Open-Drain) |
| **MQ-135 AO** | `GPIO34` (ADC1_CH6) | Tín hiệu Analog, bắt buộc nối qua **cầu phân áp 10kΩ / 10kΩ** |
| **OLED SSD1306 SDA** | `GPIO21` | Dây dữ liệu I2C (đã bật điện trở kéo lên nội, tần số 100 kHz) |
| **OLED SSD1306 SCL** | `GPIO22` | Xung nhịp I2C (đã bật điện trở kéo lên nội, tần số 100 kHz) |
| **Relay IN** | `GPIO16` | Chân Digital điều khiển module relay (mặc định `RELAY_ACTIVE_LOW 0`) |
| **Còi chip (Buzzer)** | `GPIO17` | Còi thụ động phát âm qua xung LEDC PWM (Timer 0, Channel 0, tần số 2000–2400 Hz) |
| **LED Đỏ** | `GPIO25` | Nối qua điện trở hạn dòng 220Ω vào cực Anode (+) |
| **LED Xanh** | `GPIO26` | Nối qua điện trở hạn dòng 220Ω vào cực Anode (+) |
| **Nút nhấn** | `GPIO27` | Nối vào chân GND, bật điện trở kéo lên nội (Active-LOW) |

---

## 3. Cấu trúc nguồn điện & An toàn đấu dây

1. **Module nguồn Breadboard MB102 (Đường nguồn +5V)**:
   - Cấp nguồn cho **dây sấy bên trong của cảm biến MQ-135** (cần nguồn 5V ổn định, dòng tiêu thụ khoảng 150 mA).
   - Cấp nguồn **VCC cho module Relay** và **động cơ quạt thông gió**.
2. **Chân nguồn 3.3V của ESP32**:
   - Cấp nguồn cho **cảm biến nhiệt ẩm DHT22** và **màn hình OLED SSD1306**.
3. **Còi chip thụ động (Passive Buzzer 3.3V)**:
   - Nối trực tiếp từ chân `GPIO17` qua bộ điều khiển xung PWM LEDC (dòng tiêu thụ rất nhỏ).
4. **Nối chung Mass (Common Ground)**:
   - **Bắt buộc phải nối chân GND của ESP32 với chân GND của module nguồn MB102.** Nếu không nối chung mass, chân ADC đọc tín hiệu sẽ bị sai lệch nghiêm trọng và giao tiếp I2C cũng như relay có thể hoạt động chập chờn.
5. **Cầu phân áp cho chân AO của cảm biến MQ-135**:
   - Cảm biến MQ-135 chạy nguồn 5V nên ngõ ra analog (AO) có thể xuất điện áp lên đến gần 5V.
   - Các chân ADC trên ESP32 **KHÔNG chịu được mức điện áp 5V** (tối đa chỉ 3.3V, dải đo tuyến tính tốt nhất đến khoảng ~2.5V khi cấu hình suy hao 12 dB).
   - Hãy mắc một **cầu phân áp gồm 2 điện trở 10kΩ / 10kΩ** từ chân AO của MQ-135 xuống GND, sau đó lấy điểm chính giữa nối vào chân `GPIO34`.

---

## 4. Phân loại môi trường & Các cấp độ cảnh báo

Firmware liên tục phân tích và chia trạng thái môi trường thành 4 cấp độ:

| Cấp độ | Tên gọi | Điều kiện kích hoạt | Tín hiệu đèn LED | Còi báo động (Buzzer) | Trạng thái quạt |
| :---: | :---: | :--- | :--- | :--- | :--- |
| **1** | **BÌNH THƯỜNG (NORMAL)** | Nhiệt độ: 25.0°C – 29.0°C<br>Độ ẩm: 50% – 70%<br>Tỉ lệ khí gas $K \ge 0.85$ | **LED Xanh SÁNG ĐỨNG**<br>LED Đỏ TẮT | Im lặng | Quạt ở chế độ chờ (TẮT) |
| **2** | **CẦN LƯU Ý (NOTICE)** | Nhiệt độ: 23–25°C hoặc 29–31°C<br>Độ ẩm: 40–50% hoặc 70–75%<br>Tỉ lệ khí gas $K$: 0.65 – 0.85 | **LED Xanh CHỚP ĐỀU** (chu kỳ 1.0 giây: 500 ms Sáng / 500 ms Tắt)<br>LED Đỏ TẮT | Im lặng | Quạt ở chế độ chờ (TẮT, trừ khi kích hoạt thuật toán cộng hưởng) |
| **3** | **CẢNH BÁO (WARNING)** | Nhiệt độ: 21–23°C hoặc 31–33°C<br>Độ ẩm: 30–40% hoặc 75–85%<br>Tỉ lệ khí gas $K$: 0.45 – 0.65 | **LED Xanh & Đỏ CHỚP LUÂN PHIÊN** (chu kỳ 800 ms: 400 ms Xanh / 400 ms Đỏ) | Im lặng | **BẬT QUẠT THÔNG GIÓ** (sau 2 lần đo vượt ngưỡng liên tiếp)<br>*(Lưu ý: Quạt sẽ KHÔNG BẬT nếu nhiệt độ hoặc độ ẩm đang ở ngưỡng dưới/lạnh khô)* |
| **4** | **NGUY HIỂM (CRITICAL)** | Nhiệt độ: <21°C hoặc >33°C<br>Độ ẩm: <30% hoặc >85%<br>Tỉ lệ khí gas $K < 0.45$ (Khí độc / Khói nguy hiểm) | **LED Đỏ CHỚP NHANH** (chu kỳ 700 ms: 350 ms Sáng / 350 ms Tắt)<br>LED Xanh TẮT | **Hú còi 2400 Hz ngắt quãng đồng bộ theo nhịp chớp của LED Đỏ**<br>*(Chỉ hú còi khi khí gas đạt Mức 4; nếu chỉ do Nhiệt độ/Độ ẩm đạt Mức 4 thì còi vẫn im lặng để tránh làm phiền)* | **BẬT QUẠT THÔNG GIÓ NGAY LẬP TỨC**<br>*(Lưu ý: Quạt sẽ KHÔNG BẬT nếu nhiệt độ hoặc độ ẩm đang ở ngưỡng dưới/lạnh khô)* |

> [!NOTE]
> Mọi bước chuyển đổi cấp độ đều áp dụng **Vùng trễ (Hysteresis)** (0.6°C cho nhiệt độ, 3.0% cho độ ẩm, và 0.02 cho tỉ lệ khí gas $K$) để tránh hiện tượng quạt/đèn chuyển trạng thái liên tục khi giá trị đo dao động sát mép ngưỡng.

---

## 5. Điều khiển quạt thông minh chống bật/tắt liên tục (Anti-Short-Cycle)

Để bảo vệ động cơ quạt không bị quá nhiệt, cháy hỏng và chống mòn tiếp điểm rơ-le do đóng cắt quá dày, hệ thống áp dụng cơ chế điều khiển **Anti-Short-Cycle** chuẩn công nghiệp:

- **Chặn bật quạt ở ngưỡng dưới (Lower-Threshold Fan Inhibit)**:
  Ở Mức 3 (Cảnh báo) và Mức 4 (Nguy hiểm), quạt **KHÔNG TỰ ĐỘNG BẬT** nếu nhiệt độ hoặc độ ẩm đang ở ngưỡng dưới (Trời lạnh: $<23^\circ\text{C}$ / $<21^\circ\text{C}$; Trời khô: $<40\%$ / $<30\%$) để tránh làm phòng càng lạnh hoặc khô hơn. Quạt chỉ tự động bật khi nhiệt độ cao (quá nóng), độ ẩm cao (nồm ẩm/ẩm mốc) hoặc nồng độ khí gas nguy hại.
- **Thời gian chạy tối thiểu (`FAN_MIN_RUN_TIME_MS = 60000` ms - 60 giây)**:
  Khi quạt đã tự động BẬT, nó phải chạy đủ ít nhất 60 giây mới được phép TẮT, kể cả khi các chỉ số cảm biến đã quay lại mức an toàn.
- **Thời gian nghỉ tối thiểu (`FAN_MIN_REST_TIME_MS = 60000` ms - 60 giây)**:
  Khi quạt vừa TẮT, nó phải nghỉ ít nhất 60 giây trước khi có thể được kích hoạt bật trở lại.
- **Kiểm tra độ trễ nhiều mẫu đo (`FAN_TRIGGER_PERSIST = 2` mẫu / 5 giây)**:
  Ở Mức 3 (Cảnh báo), điều kiện vượt ngưỡng phải duy trì liên tục trong 2 chu kỳ đo liên tiếp thì quạt mới bật, giúp loại trừ các trường hợp cảm biến bị nhiễu tức thời.
- **Ghi đè thủ công khẩn cấp (Emergency User Override)**:
  Bất cứ lúc nào người dùng nhấn nút vật lý hoặc bấm lệnh `snooze` / `off` trên giao diện Web, hệ thống sẽ ngay lập tức tắt quạt và còi mà không cần đợi hết thời gian khóa bảo vệ.

---

## 6. Xử lý cảm biến & Thuật toán kết hợp (Multi-Sensor Fusion)

### 6.1. Cảm biến DHT22 (AM2302)
- Chu kỳ đọc: Mỗi 2.5 giây một lần.
- Dữ liệu tính toán:
  - **Chỉ số nhiệt Heat Index** ($^\circ\text{C}$): Nhiệt độ cảm nhận thực tế của cơ thể con người (tính theo công thức hồi quy NOAA/Rothfusz).
  - **Nguy cơ nấm mốc trong nhà Mold Risk Score** (0–100%): Đánh giá nguy cơ phát sinh nấm mốc dựa trên thời gian độ ẩm duy trì ở mức cao ($\ge 60\%$) cùng nhiệt độ thích hợp.
  - **Điểm thoải mái nhiệt độ Comfort Score**: 0 (Rất khó chịu), 1 (Khó chịu), 2 (Chấp nhận được), 3 (Dễ chịu, thoải mái).

### 6.2. Cảm biến khí gas MQ-135
- **Điện trở của cảm biến ($R_s$)**:
  $$R_s = R_L \times \frac{4095 - \text{ADC}}{\text{ADC}} \quad (R_L = 10.0\text{ k}\Omega)$$
- **Tỉ lệ chất lượng không khí ($K$)**:
  $$K = \frac{R_s}{R_0}$$
  Chỉ số $K$ càng cao biểu thị không khí càng trong lành; $K$ càng giảm chứng tỏ nồng độ khí độc, hợp chất hữu cơ dễ bay hơi (VOC) hoặc khói càng đậm đặc.
- **Bộ lọc trung bình trượt (Moving Average Filter)**: Tỉ lệ $K$ được làm mịn qua 20 mẫu đo liên tiếp (cửa sổ trượt 50 giây) để triệt tiêu các xung nhiễu điện tử ngẫu nhiên.
- **Khóa sấy nóng cảm biến (Warm-Up Gate)**: Tự động đếm 60 giây sấy nóng sau khi cấp điện trước khi bắt đầu dùng dữ liệu khí gas để điều khiển tự động.

### 6.3. Thuật toán kết hợp cảm biến tương hỗ (Multi-Sensor Synergy Fusion)
Cấp độ môi trường tổng thể được tính toán qua hàm `evaluate_combined_env_level()`:
1. **Mức cơ sở**: Lấy giá trị lớn nhất trong 3 yếu tố: $\max(\text{Mức Nhiệt độ}, \text{Mức Độ ẩm}, \text{Mức Khí gas})$.
2. **Quy tắc tương hỗ A (Cộng hưởng không khí ngột ngạt/bí bách)**: Nếu cả 3 thông số đều ở Mức 2 (Lưu ý), hoặc có 2 thông số ở Mức 2 trong đó có cảm biến khí gas, hệ thống sẽ tự động nâng lên **Mức 3 (Cảnh báo)** để bật quạt thông gió.
3. **Quy tắc tương hỗ B (Cộng hưởng nguy cơ kép)**: Nếu có từ 2 cảm biến trở lên cùng chạm Mức 3 (Cảnh báo), hệ thống sẽ lập tức nâng thẳng lên **Mức 4 (Nguy hiểm)**.

---

## 7. Hướng dẫn sử dụng nút nhấn

Nút nhấn nối vào chân `GPIO27` (nối GND, có điện trở kéo lên nội và bộ lọc chống dội phím 40 ms bằng phần mềm) hỗ trợ 3 thao tác:

| Thời gian nhấn | Thao tác | Phản hồi của hệ thống |
| :--- | :--- | :--- |
| **Nhấn nhanh**<br>*(40 ms – 1500 ms)* | **Tạm hoãn / Im lặng (Snooze)** | Tắt ngay tiếng còi hú, cưỡng bức tắt quạt và kích hoạt thời gian im lặng 60 giây (`SNOOZE_MS`). |
| **Nhấn giữ**<br>*(1.5 s – 5.0 s)* | **Tự kiểm tra (Self-Test)** | Chạy bài kiểm tra toàn bộ ngoại vi: Đèn Xanh bật (250 ms, còi kêu 1000 Hz) $\rightarrow$ Đèn Đỏ bật (250 ms, còi kêu 1800 Hz) $\rightarrow$ Đóng relay bật quạt trong 500 ms. |
| **Nhấn giữ lâu**<br>*( $\ge$ 5.0 s)* | **Cân chỉnh không khí sạch** | Đo giá trị $R_s$ tại thời điểm đó, lưu làm giá trị mốc chuẩn $R_0$ mới vào bộ nhớ flash NVS và đặt lại bộ đệm lọc. |

---

## 8. Giao diện màn hình OLED đa trang

Màn hình SSD1306 độ phân giải 128x64 sẽ tự động chuyển đổi qua lại giữa 3 trang thông tin sau mỗi 5 giây (`OLED_PAGE_PERIOD_MS`):

````
+---------------------+    +---------------------+    +---------------------+
| ENVIRONMENT         |    | AIR & FAN           |    | SYSTEM              |
| TEMP 28.4C (L1)     | -> | K:0.92 L1           | -> | WIFI OK             |
| HUM  62.1% (L1)     |    | Rs:36.8k R0:40.0k   |    | RSSI -58            |
| HEAT 30.1C          |    | FAN OFF (AUTO)      |    | WARM OK             |
| MOLD 15%            |    | SYS LVL 1 NORM      |    | RAW ADC 842         |
+---------------------+    +---------------------+    +---------------------+
       Trang 0                     Trang 1                     Trang 2
````

- Nếu màn hình bị rút ra hoặc đường truyền I2C gặp sự cố, firmware sẽ ghi nhật ký cảnh báo và tiếp tục vận hành bình thường, hoàn toàn không làm đứng hay treo chương trình.

---

## 9. Giao diện Web Dashboard & REST API

Khi kết nối Wi-Fi thành công, ESP32 sẽ khởi chạy một máy chủ HTTP ở cổng 80. Bạn có thể xem địa chỉ IP được cấp qua màn hình theo dõi Serial:
```
I (xxxx) ENV_MON: Wi-Fi IP: 192.168.x.x
```
Mở trình duyệt trên máy tính hoặc điện thoại trong cùng mạng Wi-Fi và truy cập vào: `http://<IP-CỦA-ESP32>/`.

### 9.1. Các tính năng trên giao diện Web
- Thẻ thông tin hiển thị trạng thái hệ thống theo thời gian thực (tự động cập nhật qua AJAX mỗi 2 giây).
- Các nút bấm điều khiển tiện lợi: `AUTO` (Tự động), `ON` (Bật quạt), `OFF` (Tắt quạt), `SNOOZE` (Tạm dừng), `SELF TEST` (Tự kiểm tra phần cứng) và `CALIBRATE R0 (CLEAN AIR)` (Cân chỉnh mốc không khí sạch).
- Biểu mẫu trực quan cho phép chỉnh sửa các ngưỡng nhiệt độ, độ ẩm, khí gas và bấm nút `SAVE` để lưu vào bộ nhớ.

### 9.2. Các điểm cuối REST API (REST Endpoints)

#### 9.2.1. Dữ liệu trạng thái cảm biến (Telemetry State)
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

#### 9.2.2. Điều khiển thiết bị (Device Control)
`GET /api/control?cmd=<command>`
- `auto`: Đưa quạt về chế độ tự động theo cảm biến.
- `on`: Cưỡng bức bật quạt thủ công.
- `off`: Cưỡng bức tắt quạt thủ công.
- `snooze`: Tắt còi báo động và tắt quạt trong 60 giây.
- `test`: Kích hoạt quy trình tự kiểm tra phần cứng (Self-Test).
- `calib` hoặc `calibrate`: Cân chỉnh mốc $R_0$ của cảm biến MQ-135 trong không khí sạch và lưu vào bộ nhớ flash NVS.

#### 9.2.3. Cài đặt ngưỡng cảnh báo (Threshold Configuration)
`GET /api/thresholds?to=...&tf=...&ho=...&hf=...&go=...&gf=...&tc=...&hc=...&gc=...`
- `to`: Ngưỡng nhiệt độ bật quạt (°C)
- `tf`: Ngưỡng nhiệt độ tắt quạt (°C)
- `ho`: Ngưỡng độ ẩm bật quạt (%)
- `hf`: Ngưỡng độ ẩm tắt quạt (%)
- `go`: Ngưỡng giá trị ADC thô khí gas bật quạt
- `gf`: Ngưỡng giá trị ADC thô khí gas tắt quạt
- `tc`: Ngưỡng nhiệt độ nguy hiểm (°C)
- `hc`: Ngưỡng độ ẩm nguy hiểm (%)
- `gc`: Ngưỡng giá trị ADC thô khí gas nguy hiểm

---

## 10. Lưu trữ bộ nhớ NVS (Non-Volatile Storage)

Firmware sử dụng thư viện NVS của ESP-IDF với phân vùng mang tên `"envmon"`:
- **`thresholds`**: Khối dữ liệu nhị phân lưu toàn bộ các ngưỡng kích hoạt nhiệt độ, độ ẩm và khí gas do người dùng tùy chỉnh.
- **`mq_r0`**: Giá trị số thực (float) lưu điện trở mốc chuẩn không khí sạch $R_0$ đã cân chỉnh.

Dữ liệu được lưu vĩnh viễn trên chip và không bị mất đi khi mất điện hay khởi động lại thiết bị. Nếu bộ nhớ NVS bị trống hoặc dữ liệu lỗi, firmware sẽ tự động nạp lại các giá trị mặc định an toàn.

---

## 11. Tự kiểm tra phần cứng & Quá trình khởi động

Mỗi khi khởi động lại, firmware sẽ thực hiện một chuỗi kiểm tra an toàn trước khi kích hoạt giao tiếp I2C hoặc Wi-Fi:
1. **Kiểm tra đèn LED**:
   - Đèn LED Đỏ sáng ~300 ms rồi tắt.
   - Đèn LED Xanh sáng ~300 ms rồi tắt.
2. **Dò tìm màn hình OLED I2C**:
   - Thử liên lạc với địa chỉ `0x3C`. Nếu không nhận được phản hồi ACK, chip sẽ tự động chuyển sang thử địa chỉ `0x3D`.
   - Nếu vẫn không tìm thấy màn hình, firmware ghi log lỗi và tiếp tục chạy hệ thống bình thường mà không bị treo.
3. **Kiểm tra kết nối mạng Wi-Fi**:
   - Thử kết nối tối đa 10 lần. Nếu thất bại, thiết bị vẫn tiếp tục hoạt động đầy đủ ở chế độ ngoại tuyến (Offline).

### 11.1. Xử lý sự cố phần cứng & Đấu dây
- **Cả hai đèn LED đều không nháy khi bật nguồn**: Kiểm tra lại chiều chân Anode (+)/Cathode (-) của LED, kiểm tra điện trở 220Ω nối vào `GPIO25`/`GPIO26` và chân GND của ESP32.
- **Đèn LED có nháy nhưng màn hình OLED không lên**: Đảm bảo chân VCC của màn hình OLED được nối vào nguồn **3.3V** (không cắm nhầm nguồn 5V), kiểm tra dây SDA nối đúng chân `GPIO21` và dây SCL nối đúng chân `GPIO22`.
- **Relay bị ngược trạng thái (lúc cần tắt thì bật, lúc cần bật thì tắt)**: Đổi giá trị `#define RELAY_ACTIVE_LOW` trong file `main/main.c` (`0` cho loại kích mức Cao / Active-High, `1` cho loại kích mức Thấp / Active-Low).
- **Giá trị ADC của MQ-135 bị nhảy lung tung hoặc sai lệch lớn**: Đảm bảo chân GND của nguồn MB102 đã được nối chung chắc chắn với chân GND của ESP32, đồng thời kiểm tra lại cầu phân áp 10kΩ / 10kΩ.
