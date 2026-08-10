# Báo cáo Chi tiết: Dashboard và Công cụ Giám sát (AVS)

Báo cáo này tập trung phân tích chi tiết **Phần III, Mục 10: Dashboard và công cụ giám sát** dựa trên mã nguồn thực tế của ứng dụng Web Dashboard nằm trong thư mục `web_dashboard`. Tài liệu này làm rõ kiến trúc cầu nối trung gian (Web Bridge), cơ chế stream video overlay, các endpoint giao tiếp API REST, WebSocket truyền telemetry, và công cụ căn chỉnh camera (online calibration).

---

## 1. Kiến trúc Tổng quan của Web Dashboard

Hệ thống Dashboard giám sát của AVS được thiết kế dưới dạng ứng dụng Web hai tầng (Client-Server) độc lập, tối ưu hóa giao diện hiển thị cho trình duyệt Firefox:

```
[ROS2 DDS Network] ──► (WebBridgeNode) ── (FastAPI + Uvicorn) 
                             │                    │ (REST APIs & MJPEG)
                             ▼                    ▼
                      [WebSockets] ──────────► [Browser Frontend]
                     (JSON Telemetry)         (HTML / CSS / JS + Chart.js)
```

### 1.1. Backend FastAPI & Uvicorn Bridge (`backend/main.py`)
- Backend đóng vai trò là một **ROS2 Node lai** (`WebBridgeNode`).
- Khi FastAPI khởi động, nó tạo ra một luồng chạy nền (daemon thread) độc lập chạy hàm `rclcpp::spin()` để lắng nghe các topic ROS2. Luồng chính của FastAPI (chạy bởi Uvicorn) tập trung phản hồi các yêu cầu HTTP và quản lý các kết nối thời gian thực.
- Cầu nối này đăng ký nhận dữ liệu từ các topic:
  - `/camera/image_raw/compressed`: Luồng ảnh nén JPEG từ camera.
  - `/avs/telemetry_realworld`: Dữ liệu đa giác thực và thông số hiệu năng của AI.
  - `/avs/lane_state`: Quỹ đạo đường đi mượt mà đã trộn.
  - `/avs/control_error`: Sai số điều khiển bám làn tức thời.

### 1.2. Frontend Pure JS (`frontend/`)
- Được xây dựng bằng HTML5, CSS3, và JavaScript thuần (Vanilla JS) để đảm bảo độ nhẹ tối đa và không phụ thuộc vào các framework cồng kềnh như React/Angular, giúp tải nhanh trên các máy tính giám sát.
- Sử dụng thư viện **Chart.js** để vẽ đồ thị động biểu diễn sai số lệch ngang ($e_x$) và góc lệch ($\theta$) thời gian thực.

---

## 2. Các Luồng Dữ liệu và Endpoints API chính

### 2.1. Luồng Stream Video Vẽ đè Đa giác (/api/stream)
- Endpoint `/api/stream` cung cấp luồng video dạng **MJPEG (multipart/x-mixed-replace)** truyền thống.
- **Quy trình xử lý ảnh thô (Overlay Rendering):**
  1. Khi nhận được frame JPEG từ topic `/camera/image_raw/compressed`, backend giải mã frame thành ma trận OpenCV Mat.
  2. Dựa trên dữ liệu nhận diện `objects` từ topic telemetry, backend sao chép ảnh thô tạo lớp phủ `overlay` và vẽ các đa giác làn đường bằng hàm `cv2.fillPoly` theo màu đặc trưng của từng lớp.
  3. Trộn ảnh thô và lớp phủ màu bằng hàm `cv2.addWeighted` với hệ số mờ $\alpha = 0.4$ để hiển thị làn đường trong suốt, giúp lập trình viên vẫn nhìn rõ mặt đường phía dưới.
  4. Vẽ hộp bao và nhãn chữ xác suất nhận diện cho từng đối tượng.
  5. Nén ảnh ngược lại định dạng JPEG (quality 80%) và gửi đến frontend.
- **Tối ưu hóa tránh block:** Phép vẽ đè OpenCV rất tốn CPU. Để tránh làm nghẽn Event Loop đơn luồng của Python FastAPI, backend thực thi hàm vẽ đè thông qua luồng chạy riêng:
  `frame_data = await asyncio.to_thread(process_frame, latest_jpeg_frame, latest_telemetry)`

### 2.2. Kênh Truyền Telemetry thời gian thực (/ws)
- Frontend thiết lập kết nối **WebSocket** hai chiều tới `/ws`.
- Mỗi khi có gói telemetry, sai số hay quỹ đạo mới cập nhật từ ROS2, backend tự động broadcast tin nhắn JSON tới toàn bộ các trình duyệt đang kết nối để cập nhật giao diện (tần số $30\text{ Hz}$).

### 2.3. APIs Thiết lập Lệnh Điều khiển Lái
- **Gửi Ý đồ Điều hướng (`POST /api/route_intent`):** Nhận lệnh rẽ/chuyển làn từ các nút bấm trên giao diện, đóng gói JSON và publish trực tiếp vào topic `/avs/route_intent`.
- **Kích hoạt/Vô hiệu hóa Robot (`POST /api/arm`):** Gửi tín hiệu kích hoạt động cơ vật lý bằng cách publish `{"cmd": "arm"}` hoặc `{"cmd": "disarm"}` vào topic `/avs/cmd`.

---

## 3. Cấu hình Động và Công cụ Hiệu chuẩn Camera (Calibration Tool)

Dashboard tích hợp các công cụ chuyên sâu giúp lập trình viên can thiệp trực tiếp vào hoạt động của robot ở runtime mà không cần gõ lệnh terminal:

### 3.1. Cầu nối Tham số Động (ROS2 Parameter Bridge)
Backend FastAPI khởi tạo 2 service client để gọi các dịch vụ cấu hình động của ROS2:
- **`SetParameters` gọi tới `/ncnn_inference_node/set_parameters`:** 
  Cho phép người dùng kéo thanh trượt (slider) trên Web để điều chỉnh tức thì ngưỡng tin cậy phân vùng `prob_threshold` và ngưỡng triệt tiêu hộp bao trùng lặp `nms_threshold`.
- **`SetParameters` gọi tới `/video_publisher_node/set_parameters`:**
  Cho phép thay đổi nguồn ảnh đầu vào từ camera vật lý sang tệp tin video `.mp4` kiểm thử khác.

### 3.2. Quy trình Hiệu chuẩn IPM Trực tuyến (Online Calibration)
Dashboard cung cấp một quy trình căn chỉnh Homography trực quan để thiết lập phép IPM:

```
[Chụp Ảnh Snapshot] ──► [Chọn 4 Điểm trên Web UI] ──► [Nhập Tọa độ Thực mm]
                                                             │
                                                             ▼ cv2.getPerspectiveTransform()
[Ma trận H Mới] ◄── [Ghi vào calibration.json] ◄── [Tính toán Homography]
```

1. Người dùng bấm chụp snapshot khung hình hiện tại qua REST API `/api/calibration/frame`.
2. Trên giao diện Web, người dùng click chọn 4 điểm trên mặt đường (thường là góc các ô gạch hoặc vạch kẻ sân thí nghiệm). Các tọa độ pixel $(u_i, v_i)$ được ghi nhận.
3. Người dùng nhập tọa độ thực thế giới tương ứng $(X_i, Y_i)$ đơn vị milimet của 4 điểm này ngoài thực địa.
4. Backend nhận dữ liệu gửi lên qua yêu cầu `POST /api/calibration`, sử dụng hàm OpenCV:
   $$H = \text{cv2.getPerspectiveTransform(src\_points, dst\_points)}$$
   để tính toán ma trận biến đổi Homography $3 \times 3$ mới.
5. Ma trận mới cùng các điểm hiệu chuẩn được lưu vào tệp `/workspace/config/calibration.json`.
6. Node `ipm_transform_node.cpp` phát hiện tệp cấu hình thay đổi và thực hiện **hot-reload** tự động nạp lại ma trận mới, hoàn thành quá trình hiệu chuẩn trực tuyến mà không cần khởi động lại ROS2.

---

## 4. Các Chỉ số Giám sát trên Giao diện Người dùng

Giao diện Web hiển thị tập trung các thông số vận hành then chốt:
- **Thông số Hiệu năng:** Hiển thị FPS camera, FPS AI xử lý, và bảng chi tiết các độ trễ thành phần:
  - `inference_latency_ms` (suy luận AI)
  - `post_processing_latency_ms` (trích contour)
  - `bridge_latency_ms` (chuyển đổi định dạng ảnh)
  - `publish_latency_ms` (phát tin nhắn qua DDS)
- **Sai số Lái:** Biểu đồ đường của Chart.js cập nhật liên tục sai số lệch ngang ($e_x$) và góc lệch hướng ($\theta$) để giám sát tính ổn định của bộ bám làn.
- **Thông tin Làn:** Làn đường mục tiêu hiện tại (`target_label`), trạng thái máy trạng thái quyết định (`lane_state`) và trạng thái hợp lệ của đường đi (`trajectory_valid`).
