# Báo cáo Chi tiết: Thiết kế Hệ thống Topic ROS2 (AVS)

Báo cáo này tập trung phân tích chi tiết **Phần III, Mục 4: Thiết kế topic ROS2** dựa trên thiết kế và mã nguồn thực tế của hệ thống AVS. Tài liệu này làm rõ cơ chế truyền thông giữa các node, chi tiết từng topic, kiểu dữ liệu tương ứng, vai trò vật lý của các topic, và cấu trúc dữ liệu JSON được tuần tự hóa qua các cổng giao tiếp này.

---

## 1. Tổng quan về Cơ chế Truyền thông trong AVS

Hệ thống AVS tận dụng cơ chế **Publish/Subscribe bất đồng bộ** của ROS2 Humble chạy trên nền middleware DDS. Để cân bằng giữa tính linh hoạt của chu trình phát triển nhanh và tính hiệu quả của băng thông mạng, thiết kế hệ thống chia làm hai nhóm kiểu dữ liệu:
1. **Kiểu Message ROS2 Tiêu chuẩn (Standard Messages):** Sử dụng các kiểu dữ liệu có cấu trúc định sẵn trong thư viện ROS2 (`sensor_msgs`, `nav_msgs`) đối với các luồng hình ảnh thô và ảnh nén nhằm tương thích với các driver camera chuẩn và thư viện xử lý ảnh `cv_bridge`.
2. **Kiểu Message JSON tuần tự hóa qua String (JSON over String Messages):** Đối với các dữ liệu telemetry, đa giác vùng làn, waypoints và sai số điều khiển, hệ thống đóng gói dữ liệu dưới dạng cấu trúc JSON, sau đó publish qua tin nhắn chuỗi ký tự tiêu chuẩn `std_msgs/msg/String`.

### Đánh giá Đặc điểm của Giải pháp Dùng JSON qua `std_msgs/String`:
- **Ưu điểm:**
  - **Cực kỳ linh hoạt:** Dễ dàng bổ sung, thay đổi các trường dữ liệu (ví dụ thêm thông số trễ mới, thay đổi số lượng waypoint) mà không cần định nghĩa lại tệp tin `.msg` tùy biến và biên dịch lại (recompile) workspace ROS2.
  - **Thân thiện với Dashboard Web:** Backend FastAPI có thể đọc trực tiếp chuỗi JSON từ topic ROS2 và chuyển thẳng tiếp (forward) lên giao diện người dùng qua WebSockets mà không cần giải mã và dựng lại schema ở tầng web.
  - **Thuận tiện debug:** Lập trình viên có thể dùng lệnh `ros2 topic echo /topic_name` để đọc dữ liệu dạng văn bản trực quan mà không cần cài đặt các plugin tin nhắn tùy biến trên máy tính giám sát.
- **Nhược điểm:**
  - **Overhead CPU:** Đòi hỏi CPU phải thực hiện tuần tự hóa (serialize) ở node publish và giải tuần tự hóa (deserialize) ở node subscribe qua các thư viện C++ JSON (như `nlohmann/json`), tốn tài nguyên hơn so với kiểu nhị phân (binary serialization) của ROS2.
  - **Không chặt kiểu dữ liệu (Loose Coupling):** Các node không được kiểm soát kiểu dữ liệu nghiêm ngặt ở mức biên dịch, dễ dẫn đến lỗi runtime nếu cấu trúc JSON bị thay đổi ở một node mà node subscribe chưa được cập nhật tương ứng.

---

## 2. Chi tiết Thiết kế các Topic ROS2

### 2.1. Nhóm Topic Hình ảnh (Image Topics)

#### 2.1.1. Topic `/camera/image_raw`
- **Kiểu dữ liệu:** `sensor_msgs/msg/Image`
- **Node Publish:** `video_publisher_node`
- **Node Subscribe:** `ncnn_inference_node`, `video_test_node`
- **Vai trò:** Truyền tải luồng ma trận điểm ảnh thô (không nén) kích thước $640 \times 480$ ở định dạng hệ màu `bgr8`. Đây là đầu vào trực tiếp cho node nhận diện AI xử lý trên cùng một bo mạch Raspberry Pi 5 để tránh suy hao chất lượng ảnh do nén.

#### 2.1.2. Topic `/camera/image_raw/compressed`
- **Kiểu dữ liệu:** `sensor_msgs/msg/CompressedImage`
- **Node Publish:** `video_publisher_node`
- **Node Subscribe:** `web_dashboard_backend` (trong container Web Dashboard)
- **Vai trò:** Chứa luồng ảnh đã được nén bằng thuật toán **JPEG (chất lượng nén 80%)**. Nhờ kích thước gói tin nhỏ hơn tới 15-20 lần so với ảnh thô, luồng ảnh nén này được truyền tải không dây (Wifi) từ xe tự hành về Laptop chạy Dashboard của lập trình viên để hiển thị thời gian thực mà không gây trễ mạng hoặc nghẽn DDS.

---

### 2.2. Nhóm Topic Telemetry và Nhận diện (Perception Telemetry Topics)

#### 2.2.1. Topic `/avs/telemetry`
- **Kiểu dữ liệu:** `std_msgs/msg/String` (Chứa chuỗi JSON)
- **Node Publish:** `ncnn_inference_node`
- **Node Subscribe:** `ipm_transform_node`, `web_dashboard_backend`
- **Vai trò:** Truyền tải danh sách các đối tượng nhận diện và các đa giác (polygons) phân vùng của chúng nằm trên hệ tọa độ pixel ảnh 2D ($u, v$).
- **Cấu trúc trường dữ liệu JSON mẫu:**
  ```json
  {
    "input_fps": 30.0,
    "processing_fps": 28.5,
    "publish_fps": 28.5,
    "bridge_latency_ms": 1.2,
    "inference_latency_ms": 22.4,
    "post_processing_latency_ms": 4.1,
    "detections": {
      "main-lane": 1,
      "solid-white": 2,
      "vehicle": 0
    },
    "objects": [
      {
        "label": 3,
        "prob": 0.89,
        "id": "main_lane_0",
        "box": [120, 240, 400, 240],
        "polygons": [
          [[120, 480], [200, 240], [320, 240], [520, 480]]
        ]
      }
    ]
  }
  ```

---

### 2.3. Nhóm Topic Telemetry Thế giới Thực (World-Space Telemetry Topics)

#### 2.3.1. Topic `/avs/telemetry_realworld`
- **Kiểu dữ liệu:** `std_msgs/msg/String` (Chứa chuỗi JSON)
- **Node Publish:** `ipm_transform_node`
- **Node Subscribe:** `control_node`, `web_dashboard_backend`
- **Vai trò:** Truyền tải tọa độ thế giới (đơn vị: milimet) của các đối tượng sau khi đi qua phép biến đổi IPM Homography. Đồng thời chứa các waypoint của đường tâm làn đường và các hệ số đa thức khớp đường cong làn đường.
- **Cấu trúc trường dữ liệu JSON mẫu:**
  ```json
  {
    "objects": [
      {
        "id": "main_lane_0",
        "class_name": "main-lane",
        "world_polygons": [
          [[-350, 400], [-150, 1500], [150, 1500], [350, 400]]
        ],
        "waypoints": [
          [0, 400], [0, 600], [0, 800], [0, 1000], [0, 1200]
        ],
        "polynomial_coefficients": [0.0, 0.0, 0.0, 0.0]
      }
    ]
  }
  ```

---

### 2.4. Nhóm Topic Ý định Điều hướng và Điều khiển (Navigation & Control Topics)

#### 2.4.1. Topic `/avs/route_intent`
- **Kiểu dữ liệu:** `std_msgs/msg/String` (Chứa chuỗi JSON)
- **Node Publish:** `web_dashboard_backend` (Lệnh từ người dùng)
- **Node Subscribe:** `control_node`
- **Vai trò:** Chỉ thị ý định đường đi sắp tới cho xe. Khi xe tiếp cận ngã rẽ có nhiều làn ứng viên (`turn-lane`), node quyết định sẽ dựa vào intent này để lựa chọn đường tâm phù hợp.
- **Các giá trị lệnh chính:**
  - `{"intent": "FOLLOW_MAIN"}`: Giữ làn chính (Mặc định).
  - `{"intent": "TURN_LEFT"}`: Ưu tiên rẽ trái tại giao lộ.
  - `{"intent": "TURN_RIGHT"}`: Ưu tiên rẽ phải tại giao lộ.
  - `{"intent": "LANE_CHANGE"}`: Yêu cầu chuyển làn sang làn kế bên.

#### 2.4.2. Topic `/avs/control_error`
- **Kiểu dữ liệu:** `std_msgs/msg/String` (Chứa chuỗi JSON)
- **Node Publish:** `control_node`
- **Node Subscribe:** Bộ điều khiển hạ tầng (như `pure_pursuit_node` hoặc micro-ROS agent)
- **Vai trò:** Là điểm kết thúc của hệ thống thị giác AVS. Topic này xuất bản sai số hình học tức thời tính toán tại khoảng cách nhìn trước để bộ điều khiển xe thực thi lái.
- **Cấu trúc trường dữ liệu JSON mẫu:**
  ```json
  {
    "epsilon_x_mm": -12.5,
    "epsilon_y_mm": 350.0,
    "theta_rad": 0.054,
    "curvature": 0.00012,
    "target_label": "main-lane",
    "lane_state": "TRACKING",
    "trajectory_valid": true
  }
  ```

#### 2.4.3. Topic `/avs/lane_state`
- **Kiểu dữ liệu:** `std_msgs/msg/String` (Chứa chuỗi JSON)
- **Node Publish:** `control_node`
- **Node Subscribe:** `web_dashboard_backend`
- **Vai trò:** Cung cấp thông tin debug nâng cao về trạng thái làn đường và danh sách các điểm tọa độ thế giới của quỹ đạo mượt hiện tại để vẽ đè (overlay) lên luồng video trên Dashboard phục vụ giám sát trực quan.

---

## 3. Bảng Tổng hợp Đặc tả các Topic trong Hệ thống AVS

| Tên Topic ROS2 | Kiểu Tin nhắn | Node Publish (Source) | Node Subscribe (Sink) | Tần số (Hz) | Vai trò chính |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `video_publisher_node` | `ncnn_inference_node` | $30$ | Luồng ảnh thô BGR8 đầu vào AI |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | `video_publisher_node` | `web_dashboard_backend` | $30$ | Luồng ảnh nén JPEG cho Dashboard |
| `/avs/telemetry` | `std_msgs/msg/String` | `ncnn_inference_node` | `ipm_transform_node`, Dashboard | $25-30$ | Đa giác đối tượng dạng pixel (JSON) |
| `/avs/telemetry_realworld` | `std_msgs/msg/String` | `ipm_transform_node` | `control_node`, Dashboard | $25-30$ | Đa giác đối tượng thế giới mm (JSON) |
| `/avs/route_intent` | `std_msgs/msg/String` | `web_dashboard_backend` | `control_node` | Event-based | Ý định đường đi từ người dùng (JSON) |
| `/avs/control_error` | `std_msgs/msg/String` | `control_node` | Bộ điều khiển bám làn (Pure Pursuit) | $25-30$ | Sai số lệch ngang, lệch hướng (JSON) |
| `/avs/lane_state` | `std_msgs/msg/String` | `control_node` | `web_dashboard_backend` | $25-30$ | Quỹ đạo mượt debug vẽ đè (JSON) |
