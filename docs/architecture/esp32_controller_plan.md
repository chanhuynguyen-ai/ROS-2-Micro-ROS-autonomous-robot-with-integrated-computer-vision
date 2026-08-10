# Kế Hoạch Triển Khai: Bộ Điều Khiển ESP32 (micro-ROS)

## 1. Tổng Quan

Tài liệu này mô tả thiết kế và kế hoạch triển khai firmware ESP32 để nhận lệnh điều khiển từ Raspberry Pi 5 và điều khiển 4 động cơ bánh xe.

### Đầu ra từ Raspberry Pi 5 (Input cho ESP32)

Raspberry Pi publish topic `/cmd_vel` kiểu `geometry_msgs/Twist`:

```
/cmd_vel
  linear.x  = v     [m/s]    — vận tốc tiến (>0: tiến, <0: lùi)
  angular.z = omega [rad/s]  — vận tốc góc  (>0: rẽ trái, <0: rẽ phải)
```

Pipeline trên Pi trước khi ra `/cmd_vel`:

```
ncnn_inference_node
    → ipm_transform_node  (/avs/telemetry_realworld)
        → control_node    (/avs/control_error: ε_x, ε_y, θ, κ)
            → pure_pursuit_node  (/cmd_vel: v, ω)
```

---

## 2. Kiến Trúc Hệ Thống ESP32

```
Raspberry Pi 5                          ESP32 (micro-ROS)
──────────────────────                  ──────────────────────────────
pure_pursuit_node                       micro-ROS agent ← Serial UART / UDP
    │                                       │
    └──→ /cmd_vel (v, ω) ─────────────────→ cmd_vel_callback()
                                            │
                                    ┌───────▼────────┐
                                    │  Kinematics    │
                                    │  v_L = v-ω*W/2 │
                                    │  v_R = v+ω*W/2 │
                                    └───────┬────────┘
                                            │
                                    ┌───────▼────────┐
                                    │  PWM Output    │
                                    │  (LEDC/MCPWM)  │
                                    └───────┬────────┘
                                            │
                              ┌─────────────┴─────────────┐
                         Left side                    Right side
                       FL + RL motors               FR + RR motors
```

---

## 3. Toán Học Kinematics (Skid-Steer 4WD)

Xe 4 bánh chủ động chia thành 2 nhóm: **trái** (FL + RL) và **phải** (FR + RR).

### 3.1 Tính vận tốc từng bên

```
v_left  = v - omega * (W / 2)   [m/s]
v_right = v + omega * (W / 2)   [m/s]
```

| Ký hiệu | Ý nghĩa | Ghi chú |
|---------|---------|---------|
| `v` | Vận tốc tiến | Từ `/cmd_vel` linear.x |
| `omega` | Vận tốc góc | Từ `/cmd_vel` angular.z |
| `W` | Khoảng cách tâm bánh trái–phải | **Đo thực tế trên xe** |

### 3.2 Chuyển đổi sang PWM

```
pwm_left  = clamp(v_left  / v_max * PWM_MAX,  -PWM_MAX, +PWM_MAX)
pwm_right = clamp(v_right / v_max * PWM_MAX,  -PWM_MAX, +PWM_MAX)
```

| Ký hiệu | Giá trị đề xuất | Ghi chú |
|---------|-----------------|---------|
| `v_max` | 0.5 m/s | Phải khớp với `v_max` trong `pure_pursuit_node` |
| `PWM_MAX` | 1023 (10-bit) | Tuỳ driver, thường 255 (8-bit) hoặc 1023 (10-bit) |

### 3.3 Kiểm tra hướng quay theo trường hợp

| Lệnh | v | ω | v_left | v_right | Hành vi |
|------|---|---|--------|---------|---------|
| Tiến thẳng | + | 0 | + | + | Cả 4 bánh tiến |
| Rẽ trái | + | + | nhỏ hơn | lớn hơn | Trái chậm, phải nhanh |
| Rẽ phải | + | − | lớn hơn | nhỏ hơn | Trái nhanh, phải chậm |
| Quay tại chỗ | 0 | + | − | + | Trái lùi, phải tiến |

---

## 4. Cấu Trúc Firmware

```
esp32_firmware/
├── CMakeLists.txt                  ← ESP-IDF top-level
└── main/
    ├── CMakeLists.txt              ← idf_component_register
    ├── main.c                      ← Entry point, micro-ROS init, executor loop
    ├── cmd_vel_subscriber.c/.h     ← Subscribe /cmd_vel + watchdog
    └── motor_driver.c/.h           ← Kinematics + LEDC PWM output
```

---

## 5. Mô Tả Chi Tiết Từng Module

### 5.1 `main.c` — Entry Point & micro-ROS Executor

**Trách nhiệm:**
- Khởi tạo micro-ROS transport (Serial UART hoặc UDP WiFi)
- Tạo ROS2 node `esp32_drive_node`
- Đăng ký subscriber `/cmd_vel`
- Chạy vòng lặp `rclc_executor_spin_some()` mỗi 10ms
- Gọi `motor_driver_init()` và kiểm tra watchdog

**Pseudocode:**
```c
app_main():
    micro_ros_transport_init()          // Serial UART @921600 baud
    rclc_support_init()
    rclc_node_init("esp32_drive_node")
    rclc_subscription_init("/cmd_vel", Twist, cmd_vel_callback)
    motor_driver_init()

    while(true):
        rclc_executor_spin_some(10ms)
        watchdog_check()                // dừng xe nếu cmd cũ > 500ms
```

---

### 5.2 `cmd_vel_subscriber.c` — Subscriber & Watchdog

**Trách nhiệm:**
- Callback khi nhận `/cmd_vel`: trích xuất `v`, `omega` → gọi `motor_set_velocity()`
- Watchdog: nếu không nhận cmd trong `CMD_TIMEOUT_MS = 500ms` → `motor_stop_all()`

**Pseudocode:**
```c
static int64_t last_cmd_time_ms = 0;

cmd_vel_callback(msg):
    v     = msg.linear.x
    omega = msg.angular.z
    last_cmd_time_ms = esp_timer_get_time_ms()
    motor_set_velocity(v, omega)

watchdog_check():
    if (now - last_cmd_time_ms) > CMD_TIMEOUT_MS:
        motor_stop_all()
```

---

### 5.3 `motor_driver.c` — Kinematics + PWM

**Trách nhiệm:**
- Tính `v_left`, `v_right` từ `(v, omega)`
- Chuyển đổi sang duty cycle PWM
- Điều khiển chiều quay qua GPIO DIR
- Xuất PWM qua ESP-IDF LEDC driver

**Pseudocode:**
```c
motor_set_velocity(v, omega):
    v_left  = v - omega * (WHEEL_SEPARATION_M / 2)
    v_right = v + omega * (WHEEL_SEPARATION_M / 2)

    pwm_left  = clamp(v_left  / V_MAX_MS * PWM_MAX_DUTY)
    pwm_right = clamp(v_right / V_MAX_MS * PWM_MAX_DUTY)

    // LEFT: FL + RL
    gpio_set_level(FL_DIR, pwm_left >= 0 ? FORWARD : BACKWARD)
    ledc_set_duty(FL_CH, abs(pwm_left))

    gpio_set_level(RL_DIR, pwm_left >= 0 ? FORWARD : BACKWARD)
    ledc_set_duty(RL_CH, abs(pwm_left))

    // RIGHT: FR + RR
    gpio_set_level(FR_DIR, pwm_right >= 0 ? FORWARD : BACKWARD)
    ledc_set_duty(FR_CH, abs(pwm_right))

    gpio_set_level(RR_DIR, pwm_right >= 0 ? FORWARD : BACKWARD)
    ledc_set_duty(RR_CH, abs(pwm_right))
```

---

## 6. Transport Layer

### Lựa chọn (cần xác nhận)

| Transport | Ưu điểm | Nhược điểm | Khuyến nghị |
|-----------|---------|-----------|-------------|
| **Serial UART** | Latency thấp ~1ms, ổn định, không cần WiFi | Cần dây Pi ↔ ESP32 | ✅ **Khuyến nghị** |
| UDP WiFi | Không dây | Latency cao hơn, phụ thuộc WiFi | Dùng khi không muốn dây |

### Kết nối Serial UART

```
Raspberry Pi 5          ESP32
GPIO14 (TX) ──────────→ GPIO16 (RX)
GPIO15 (RX) ←────────── GPIO17 (TX)
GND         ────────── GND
```

micro-ROS agent chạy trên Pi:
```bash
ros2 run micro_ros_agent micro_ros_agent serial \
    --dev /dev/ttyAMA0 -b 921600 -v4
```

---

## 7. Thông Số Cần Xác Nhận Trước Khi Triển Khai

> [!IMPORTANT]
> Các thông số sau **bắt buộc phải đo/xác nhận** trước khi viết code thực tế:

| # | Thông số | Ký hiệu | Cần làm |
|---|---------|---------|---------|
| 1 | Khoảng cách tâm bánh trái–phải | `W` | Đo thực tế trên xe (m) |
| 2 | Tốc độ tối đa thực tế | `v_max` | Đo thực nghiệm (phải khớp Pi) |
| 3 | Motor driver IC đang dùng | — | L298N / TB6612 / BTS7960? |
| 4 | Transport method | — | Serial UART hay WiFi UDP? |
| 5 | Encoder | — | Có hay không? (ảnh hưởng thiết kế PID vòng trong) |
| 6 | GPIO pins | — | Xác nhận sơ đồ nối dây thực tế |

> [!NOTE]
> Nếu **có encoder**: thêm PID vòng trong (inner loop) trên ESP32 để kiểm soát tốc độ từng bánh chính xác hơn, tránh drift khi xe đi thẳng.
> Nếu **không có encoder**: chạy open-loop PWM, cần tinh chỉnh `V_MAX_MS` thực nghiệm.

---

## 8. Kế Hoạch Test (Verification)

### Test 1 — Kiểm tra motor driver (không cần Pi)
Publish `/cmd_vel` thủ công từ laptop:
```bash
export ROS_DOMAIN_ID=20

# Tiến thẳng
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Rẽ trái
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}"

# Dừng
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```
✅ Kiểm tra: 4 bánh quay đúng chiều và tốc độ tương đối.

### Test 2 — Kiểm tra watchdog
Dừng publish `/cmd_vel` → sau 500ms ESP32 phải tự dừng tất cả motor.

### Test 3 — Kiểm tra kinematics
- Rẽ trái (ω > 0): `v_left < v_right` → trái chậm, phải nhanh ✅
- Rẽ phải (ω < 0): `v_left > v_right` → trái nhanh, phải chậm ✅
- Quay tại chỗ (v=0, ω>0): trái lùi, phải tiến ✅

### Test 4 — Full Integration
Chạy toàn bộ stack trên Pi → quan sát xe bám làn trong thực tế.

---

## 9. Các Bước Triển Khai

- [ ] **Bước 1:** Xác nhận 6 thông số trong mục 7
- [ ] **Bước 2:** Tạo project ESP-IDF + thêm micro-ROS component
- [ ] **Bước 3:** Implement `motor_driver.c` với LEDC PWM
- [ ] **Bước 4:** Implement `cmd_vel_subscriber.c` + watchdog
- [ ] **Bước 5:** Implement `main.c` — micro-ROS init + executor
- [ ] **Bước 6:** Flash firmware, Test 1 & Test 2
- [ ] **Bước 7:** Kết nối Pi ↔ ESP32 qua UART, chạy micro-ROS agent
- [ ] **Bước 8:** Test 3 & Test 4 — full integration
