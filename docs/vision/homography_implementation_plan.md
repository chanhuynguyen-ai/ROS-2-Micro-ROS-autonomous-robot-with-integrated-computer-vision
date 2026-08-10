# Kế Hoạch Triển Khai: Pixel → Tọa Độ Thực (Homography IPM)

## 1. Tổng Quan Kiến Trúc

```
┌─────────────────────────────────────────────────────────────────┐
│                    LAPTOP (Acer Nitro 5)                        │
│                                                                 │
│  ┌─────────────────────────────────────┐                        │
│  │  Web Dashboard (http://localhost:8000)│                       │
│  │                                     │                        │
│  │  [Stream] [Settings] [Calibration]  │ ← nút mới             │
│  │                                     │                        │
│  │  Chế độ Calibration:                │                        │
│  │  - Hiển thị 1 frame camera          │                        │
│  │  - Click 4 điểm trên ảnh           │                        │
│  │  - Nhập tọa độ thực (mm)           │                        │
│  │  - Tính ma trận H                  │                        │
│  │  - Gửi H → config/calibration.json │                        │
│  └────────────────┬────────────────────┘                        │
│                   │ API: POST /api/calibration                  │
│  ┌────────────────▼────────────────────┐                        │
│  │  FastAPI Backend (main.py)          │                        │
│  │  - Nhận 4 cặp điểm (pixel, mm)     │                        │
│  │  - Tính H = getPerspectiveTransform │                        │
│  │  - Lưu H vào config/calibration.json│                        │
│  └─────────────────────────────────────┘                        │
└─────────────────────────────────────────────────────────────────┘
                        │
          config/calibration.json (volume-mounted)
                        │
┌───────────────────────▼─────────────────────────────────────────┐
│                  RASPBERRY PI 5                                  │
│                                                                  │
│  ┌────────────────┐    ┌──────────────────┐    ┌──────────────┐ │
│  │video_publisher  │───→│ncnn_inference    │───→│lane_transform│ │
│  │_node            │    │_node             │    │_node (MỚI)   │ │
│  │                 │    │                  │    │              │ │
│  │ /camera/        │    │ /avs/telemetry   │    │Đọc calib.json│ │
│  │  image_raw      │    │ (JSON + polygons)│    │Transform H   │ │
│  └────────────────┘    └──────────────────┘    │Fit polynomial│ │
│                                                 │Publish model │ │
│                                                 │              │ │
│                                                 │ /avs/        │ │
│                                                 │  lane_model  │ │
│                                                 └──────────────┘ │
└──────────────────────────────────────────────────────────────────┘
```

---

## 2. Hệ Tọa Độ

```
      Y (mm) — hướng trước xe (dọc / longitudinal)
      ↑
      │
      O────────→ X (mm) — sang phải (ngang / lateral)
    (0,0)
```

- **X dương:** sang phải so với xe
- **Y dương:** phía trước xe
- **Gốc (0,0):** điểm chiếu camera xuống mặt đường

---

## 3. Các Thành Phần Cần Triển Khai

### 3.1 Frontend — Chế Độ Calibration (Web Dashboard)

**File cần sửa:** `web_dashboard/frontend/index.html` (và CSS/JS liên quan)

**Chức năng:**
1. Thêm nút **[Calibration]** trên thanh điều khiển
2. Khi bấm, chuyển sang chế độ calibration:
   - Hiển thị **1 frame tĩnh** (freeze frame) từ camera stream
   - Người dùng **click 4 điểm** trên ảnh → hiển thị dấu chấm + số thứ tự
   - Mỗi điểm: nhập tọa độ thực **(X_mm, Y_mm)** qua ô input
   - Nút **[Tính & Lưu]** → gửi dữ liệu lên backend
   - Hiển thị kết quả: "Calibration thành công" hoặc lỗi

**Lưu ý UI:**
- Tọa độ pixel được lấy dựa trên kích thước ảnh gốc (640×480), không phải kích thước hiển thị trên trình duyệt
- Cần tính tỉ lệ: `pixel_x = click_x * (640 / display_width)`

---

### 3.2 Backend — API Calibration (FastAPI)

**File cần sửa:** `web_dashboard/backend/main.py`

**API mới:**

```
POST /api/calibration
Body: {
    "points": [
        {"pixel": [u1, v1], "world": [x1_mm, y1_mm]},
        {"pixel": [u2, v2], "world": [x2_mm, y2_mm]},
        {"pixel": [u3, v3], "world": [x3_mm, y3_mm]},
        {"pixel": [u4, v4], "world": [x4_mm, y4_mm]}
    ]
}
Response: {
    "status": "success",
    "homography_matrix": [[h11,h12,h13],[h21,h22,h23],[h31,h32,h33]]
}
```

**Logic:**
```python
import numpy as np
import cv2

src = np.float32([[u1,v1], [u2,v2], [u3,v3], [u4,v4]])
dst = np.float32([[x1,y1], [x2,y2], [x3,y3], [x4,y4]])
H = cv2.getPerspectiveTransform(src, dst)

# Lưu vào config/calibration.json
{
    "homography_matrix": H.tolist(),
    "src_points_pixel": src.tolist(),
    "dst_points_mm": dst.tolist(),
    "image_size": [640, 480],
    "calibrated_at": "2026-06-10T21:00:00"
}
```

**API bổ sung:**
```
GET /api/calibration        → Trả về calibration hiện tại (nếu có)
GET /api/calibration/frame  → Trả về 1 frame JPEG tĩnh để hiển thị khi calibrate
```

---

### 3.3 Config File — `config/calibration.json`

```json
{
    "homography_matrix": [
        [0.85, -0.02, -120.5],
        [0.01,  1.65, -180.0],
        [0.00001, 0.0008, 1.0]
    ],
    "src_points_pixel": [[185,120], [455,120], [410,350], [230,350]],
    "dst_points_mm": [[0,0], [297,0], [297,210], [0,210]],
    "image_size": [640, 480],
    "calibrated_at": "2026-06-10T21:00:00"
}
```

File này nằm trong folder `config/` đã được volume-mount trong Docker → Pi đọc được trực tiếp.

---

### 3.4 ROS2 Node Mới — `lane_transform_node` (C++)

**File mới:** `ros2_ws/src/avs_perception/src/lane_transform_node.cpp`

**Subscribe:** `/avs/telemetry` (std_msgs/String — JSON chứa polygons)

**Publish:** `/avs/lane_model` (std_msgs/String — JSON chứa polynomial + offset + heading)

**Logic xử lý mỗi frame:**

```
1. Parse JSON telemetry → lọc objects có label = main-lane (2), other-lane (3), turn-lane (6)
2. Với mỗi làn đường:
   a. Lấy polygon contour (pixel)
   b. Trích xuất centerline bằng phương pháp midpoint:
      - Với mỗi hàng pixel (v), tìm min_u và max_u trong contour
      - center_u = (min_u + max_u) / 2
      - → Tập điểm centerline: {(center_u, v)}
   c. Transform pixel → mm qua ma trận H:
      - perspectiveTransform({(center_u, v)}, H) → {(X_mm, Y_mm)}
   d. Fit đa thức bậc 3: x(y) = a₃y³ + a₂y² + a₁y + a₀
      - Dùng cv::solve() với ma trận Vandermonde
   e. Trích xuất:
      - lateral_offset = a₀ (mm)
      - heading_angle = arctan(a₁) (rad)
      - curvature = 2·a₂ (1/mm)
3. Publish JSON lên /avs/lane_model
```

**Output JSON mẫu:**
```json
{
    "timestamp": 1718042400.123,
    "lanes": [
        {
            "class": "main-lane",
            "label": 2,
            "polynomial": {"a3": 1e-7, "a2": -0.0003, "a1": 0.015, "a0": 85.2},
            "lateral_offset_mm": 85.2,
            "heading_angle_rad": 0.015,
            "curvature_inv_mm": -0.0006,
            "num_points": 120,
            "y_range_mm": [50, 1200]
        },
        {
            "class": "other-lane",
            "label": 3,
            "polynomial": {"a3": 8e-8, "a2": -0.0002, "a1": 0.012, "a0": -180.5},
            "lateral_offset_mm": -180.5,
            "heading_angle_rad": 0.012,
            "curvature_inv_mm": -0.0004,
            "num_points": 95,
            "y_range_mm": [80, 1100]
        }
    ],
    "calibrated": true
}
```

---

### 3.5 Cập Nhật CMakeLists.txt

Thêm executable mới cho `lane_transform_node`:

```cmake
# Lane Transform Node
add_executable(lane_transform_node src/lane_transform_node.cpp)
ament_target_dependencies(lane_transform_node rclcpp std_msgs)
target_link_libraries(lane_transform_node ${OpenCV_LIBRARIES})

install(TARGETS lane_transform_node
  DESTINATION lib/${PROJECT_NAME})
```

---

### 3.6 Cập Nhật Docker Compose

Thêm lệnh khởi chạy `lane_transform_node` — có thể chạy chung container `avs_perception` hoặc tạo container riêng.

**Phương án đơn giản:** chạy song song trong `avs_perception_container`:
```yaml
command: bash -c "cd /workspace/ros2_ws && rm -rf build install log && colcon build --symlink-install && source install/setup.bash && ros2 run avs_perception ncnn_inference_node & ros2 run avs_perception lane_transform_node"
```

**Phương án tách biệt (khuyến nghị):** thêm container riêng để có thể restart độc lập.

---

## 4. Thứ Tự Triển Khai

| Phase | Công việc | Files |
|-------|----------|-------|
| **Phase 1** | Backend API calibration + config file | `main.py`, `config/calibration.json` |
| **Phase 2** | Frontend UI chế độ calibration | `frontend/index.html`, JS/CSS |
| **Phase 3** | `lane_transform_node` C++ (core logic) | `lane_transform_node.cpp`, `CMakeLists.txt` |
| **Phase 4** | Cập nhật Docker Compose + test end-to-end | `docker-compose.yml`, `docker-compose.prod.yml` |
| **Phase 5** | Kết nối output `/avs/lane_model` với bộ điều khiển PID | `coordinatesPID/` integration |

---

## 5. Quy Trình Sử Dụng (User Flow)

### Calibration (1 lần):
```
1. Đặt vật tham chiếu (kích thước đã biết) trên mặt đường trước camera
2. Mở web dashboard trên laptop: http://<hostname>:8000
3. Bấm nút [Calibration]
4. Ảnh camera freeze → click 4 góc của vật tham chiếu
5. Nhập tọa độ thực (mm) cho 4 điểm
6. Bấm [Tính & Lưu]
7. Ma trận H được lưu vào config/calibration.json
8. lane_transform_node tự động reload config
```

### Vận hành (liên tục):
```
1. Camera stream → video_publisher_node
2. YOLO segmentation → ncnn_inference_node → /avs/telemetry
3. Lane transform → lane_transform_node → /avs/lane_model
4. Bộ điều khiển đọc /avs/lane_model → tính Twist → điều khiển xe
```

---

## 6. Xác Minh & Kiểm Thử

| Bước kiểm tra | Cách thực hiện |
|---------------|----------------|
| Calibration API | Gửi 4 cặp điểm qua Postman/curl, kiểm tra `calibration.json` |
| Transform chính xác | Đặt vật ở vị trí đã biết, so sánh output mm với đo thực tế |
| Polynomial fit | Hiển thị đường cong fit overlay lên ảnh BEV trên dashboard |
| End-to-end | Chạy xe trên băng rôn, kiểm tra lateral offset thay đổi hợp lý |
