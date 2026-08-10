# Báo cáo Chi tiết: Thiết kế Dữ liệu và Schema JSON (AVS)

Báo cáo này tập trung phân tích chi tiết **Phần III, Mục 5: Thiết kế dữ liệu và schema JSON** dựa trên mã nguồn thực tế của hệ thống AVS. Tài liệu này làm rõ thiết kế cấu trúc dữ liệu, kiểu dữ liệu, mô tả chi tiết từng trường trong các bản tin JSON truyền qua các topic ROS2, và các kỹ thuật lập trình C++ liên quan đến xử lý JSON.

---

## 1. Triết lý Thiết kế Dữ liệu dạng JSON trong AVS

Để tối ưu hóa sự phối hợp giữa các node ROS2 C++ hiệu năng cao và Dashboard Web (React/FastAPI), hệ thống AVS sử dụng **JSON (JavaScript Object Notation)** làm định dạng trao đổi thông tin chính. 
- Dữ liệu được tuần tự hóa (serialize) thành chuỗi văn bản (String) trước khi gửi qua topic ROS2 và được giải tuần tự hóa (deserialize) ở node nhận.
- Các tọa độ điểm ảnh được định nghĩa dạng Pixel (số nguyên hoặc số thực), trong khi tọa độ thực địa luôn sử dụng đơn vị **milimet (mm)** và góc lái sử dụng **radian (rad)** để đảm bảo tính chuẩn hóa toán học.

---

## 2. Chi tiết Schema JSON của các Topic chính

### 2.1. Schema JSON của Topic `/avs/telemetry` (Perception)
Bản tin này do node `ncnn_inference_node` phát ra sau khi suy luận AI để cung cấp thông tin thô của các đối tượng nhận diện trong hệ tọa độ ảnh.

#### Sơ đồ cấu trúc phân cấp (JSON Hierarchy):
```
Root (Object)
├── input_fps (Number, Float) - Tần số khung hình camera nhận được
├── processing_fps (Number, Float) - Tần số khung hình AI xử lý được
├── publish_fps (Number, Float) - Tần số khung hình thực tế phát đi
├── bridge_latency_ms (Number, Float) - Thời gian convert ROS sang OpenCV Mat
├── inference_latency_ms (Number, Float) - Thời gian suy luận mạng NCNN
├── post_processing_latency_ms (Number, Float) - Thời gian trích contour và tạo JSON
├── contour_time_ms (Number, Float) - Thời gian chạy thuật toán tìm biên contour
├── node_total_latency_ms (Number, Float) - Tổng thời gian chạy của node ở frame trước
├── output_age_ms (Number, Float) - Tổng độ trễ end-to-end từ lúc capture đến khi publish
├── detections (Object) - Số lượng đối tượng phát hiện theo lớp
│   ├── main-lane (Integer)
│   ├── other-lane (Integer)
│   ├── turn-lane (Integer)
│   └── ... (các lớp khác trong số 19 lớp)
└── objects (Array) - Danh sách các đối tượng phát hiện
    └── Object i (Object)
        ├── label (Integer) - Mã số nhãn lớp (0-18)
        ├── class_name (String) - Tên nhãn lớp tương ứng
        ├── prob (Number, Float) - Độ tin cậy nhận diện (0.0 - 1.0)
        ├── id (String) - Mã định danh duy nhất của đối tượng (sinh bởi tracker)
        ├── track_id (String) - Mã định danh bám vết đối tượng
        ├── box (Array of 4 Numbers) - Bounding Box định dạng [x_min, y_min, width, height]
        └── polygons (Array of Arrays) - Mảng 3 chiều chứa tọa độ pixel [u, v] của các đa giác vùng biên
```

#### Ví dụ Payload thực tế:
```json
{
  "input_fps": 30.0,
  "processing_fps": 28.5,
  "publish_fps": 28.5,
  "bridge_latency_ms": 1.25,
  "inference_latency_ms": 23.10,
  "post_processing_latency_ms": 3.80,
  "contour_time_ms": 1.45,
  "node_total_latency_ms": 29.80,
  "output_age_ms": 31.20,
  "detections": {
    "main-lane": 1,
    "solid-white": 2,
    "vehicle": 0
  },
  "objects": [
    {
      "label": 3,
      "class_name": "main-lane",
      "prob": 0.92,
      "id": "main_lane_0",
      "track_id": "main_lane_0",
      "box": [100, 200, 440, 280],
      "polygons": [
        [[100, 480], [220, 200], [320, 200], [540, 480]]
      ]
    }
  ]
}
```

---

### 2.2. Schema JSON của Topic `/avs/telemetry_realworld` (IPM)
Node `ipm_transform_node` nhận dữ liệu từ `/avs/telemetry`, giữ nguyên các trường metadata hiệu năng, bổ sung tọa độ thực thế giới thực của đa giác (`polygons_real_world`), trích xuất centerline (`waypoints`) và nạp các hệ số đa thức khớp được.

#### Các trường bổ sung/sửa đổi trong mảng `objects`:
- **`polygons_real_world`** (Array of Arrays): Mảng 3 chiều chứa các điểm tọa độ thực tế $(X_w, Y_w)$ đơn vị milimet trên mặt đường (gốc xe).
- **`waypoints`** (Array of Arrays): Danh sách điểm tâm làn đường rời rạc dạng $[X, Y]$ tính từ gốc xe hướng ra xa.
- **`polynomial`** (Object): Chứa 4 hệ số của đa thức khớp bậc 3:
  - `a0` (Float): Hệ số tự do (sai số lệch ngang tại gốc xe $e_x$ tính bằng mm).
  - `a1` (Float): Tiếp tuyến tại gốc xe (liên quan trực tiếp đến góc lệch hướng $\theta$).
  - `a2` (Float): Độ cong bậc 2 (liên quan tới curvature $\kappa \approx 2a_2$).
  - `a3` (Float): Hệ số bậc 3.
- **`lateral_offset_mm`** (Float): Khoảng lệch ngang mượt mà tại gốc xe.
- **`heading_angle_rad`** (Float): Góc hướng lệch mượt mà tại gốc xe.
- **`curvature_inv_mm`** (Float): Độ cong làn đường tại gốc xe ($1/\text{mm}$).
- **`lookahead_d_mm`** (Float): Khoảng cách xem trước động được chọn cho frame này.
- **`lookahead_x_mm`** (Float): Tọa độ lệch ngang của điểm đích tương ứng với khoảng xem trước.
- **`lookahead_theta_rad`** (Float): Góc lái mục tiêu bám đuổi tương ứng tại khoảng xem trước.

*Lưu ý: Node IPM hỗ trợ xử lý đặc thù cho làn rẽ (`turn-lane` ứng với nhãn lớp 10 hoặc 17 tùy phiên bản mô hình). Đối với làn rẽ, đa thức khớp theo phương ngang $y(x) = b_3x^3 + b_2x^2 + b_1x + b_0$, các trường đa thức sẽ biểu diễn các hệ số $b_i$.*

#### Ví dụ Payload thực tế:
```json
{
  "input_fps": 30.0,
  "processing_fps": 28.5,
  "objects": [
    {
      "label": 3,
      "class_name": "main-lane",
      "id": "main_lane_0",
      "polygons_real_world": [
        [[-300.5, 300.0], [-100.2, 1200.0], [100.2, 1200.0], [300.5, 300.0]]
      ],
      "waypoints": [
        [0.0, 300.0], [0.0, 500.0], [0.0, 700.0], [0.0, 900.0], [0.0, 1100.0]
      ],
      "polynomial": {
        "a3": 0.00000001,
        "a2": 0.000002,
        "a1": 0.015,
        "a0": -8.5
      },
      "lateral_offset_mm": -8.5,
      "heading_angle_rad": 0.015,
      "curvature_inv_mm": 0.000004,
      "lookahead_d_mm": 350.0,
      "lookahead_x_mm": -3.2,
      "lookahead_theta_rad": -0.009
    }
  ]
}
```

---

### 2.3. Schema JSON của Topic `/avs/control_error` (Control Output)
Do node `control_node` phát ra. Đây là đầu ra tinh gọn nhất của hệ CV cung cấp cho bộ điều khiển lái.

#### Danh sách trường dữ liệu:
- **`epsilon_x_mm`** (Number, Float): Sai số lệch ngang tại điểm nhìn trước (Look-ahead point). Giá trị âm biểu thị lệch trái, dương biểu thị lệch phải.
- **`epsilon_y_mm`** (Number, Float): Khoảng cách nhìn trước dọc thực tế (trùng với $e_y$).
- **`theta_rad`** (Number, Float): Sai số góc hướng tại điểm nhìn trước.
- **`curvature`** (Number, Float): Độ cong quỹ đạo tại điểm nhìn trước ($1/\text{mm}$).
- **`target_label`** (String): Nhãn làn đường đang bám theo (ví dụ: `main-lane`, `turn-lane`).
- **`lane_state`** (String): Trạng thái của bộ bám làn (ví dụ: `TRACKING`, `TURN_APPROACH`, `TURNING`, `RECOVERY`).
- **`trajectory_valid`** (Boolean): Cờ xác thực quỹ đạo (`true` nếu quỹ đạo bám tin cậy, `false` nếu mất làn hoàn toàn).

#### Ví dụ Payload thực tế:
```json
{
  "epsilon_x_mm": -10.4,
  "epsilon_y_mm": 350.0,
  "theta_rad": -0.029,
  "curvature": 0.000005,
  "target_label": "main-lane",
  "lane_state": "TRACKING",
  "trajectory_valid": true
}
```

---

## 3. Các Schema JSON hỗ trợ khác

### 3.1. Schema JSON của Topic `/avs/route_intent` (Ý định dẫn hướng)
Do Dashboard gửi xuống để tác động vào máy trạng thái quyết định của `control_node`:
```json
{
  "intent": "TURN_RIGHT",
  "source": "web_dashboard",
  "seq": 42
}
```

### 3.2. Schema JSON của Topic `/avs/lane_state` (Debug Quỹ đạo mượt)
Do `control_node` gửi lên Dashboard để vẽ đè quỹ đạo mượt (Active Blended Trajectory) lên video giám sát:
```json
{
  "state": "TRACKING",
  "trajectory_points": [
    [-8.5, 0.0], [-5.2, 100.0], [-2.1, 200.0], [0.5, 300.0], [2.8, 400.0]
  ]
}
```

---

## 4. Kỹ thuật Lập trình C++ Xử lý JSON trong ROS2 Workspace

Mã nguồn AVS sử dụng thư viện **nlohmann/json** để phân tích cú pháp JSON trong C++. Để đảm bảo các node không bị dừng đột ngột (crash) khi gặp chuỗi JSON bị lỗi hoặc thiếu trường, hệ thống áp dụng các kỹ thuật sau:

### 4.1. Kỹ thuật Đọc an toàn (Safe Retrieval với giá trị mặc định)
Không truy cập trực tiếp bằng toán tử ngoặc vuông `obj["key"]` nếu không chắc chắn trường đó tồn tại, vì sẽ gây ra ngoại lệ `json.exception.out_of_range`. Thay vào đó, sử dụng phương thức `value()` để cung cấp fallback:
```cpp
// Đọc an toàn với giá trị mặc định nếu trường không tồn tại hoặc bị null
int label = obj.value("label", -1);
std::string class_name = obj.value("class_name", "");
```

### 4.2. Cơ chế Bẫy lỗi Phân tích cú pháp (Exception Catching)
Trong quá trình chuyển đổi chuỗi String từ topic ROS2 sang đối tượng JSON, luôn đặt mã nguồn trong khối `try-catch` để xử lý các gói tin lỗi đường truyền:
```cpp
try {
    // Phân tích chuỗi JSON nhận từ Subscriber
    nlohmann::json telemetry = nlohmann::json::parse(msg->data);
    
    // Thực hiện các xử lý IPM hoặc Control tại đây...
    
} catch (const nlohmann::json::parse_error& e) {
    RCLCPP_ERROR(this->get_logger(), "JSON Parse Error: %s", e.what());
    // Tránh crash node và đưa ra giải pháp fallback
} catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Standard Exception: %s", e.what());
}
```
