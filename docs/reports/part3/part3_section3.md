# Báo cáo Chi tiết: Thiết kế các Node và Vai trò từng Node (AVS)

Báo cáo này tập trung phân tích chi tiết **Phần III, Mục 3: Thiết kế node và vai trò từng node** dựa trên mã nguồn thực tế của package `avs_perception`. Tài liệu này làm rõ thiết kế lớp (class design), kiến trúc xử lý bên trong (như đa luồng, cơ chế tracking, lọc nhiễu, làm mượt và máy trạng thái quyết định), các tham số và các cổng giao tiếp (Publisher/Subscriber) của từng node.

---

## 1. Bản đồ Tổng quan về các Node ROS2 trong Package

Hệ thống thị giác máy tính và hình học đường đi của AVS bao gồm 5 node ROS2 C++ cốt lõi được điều khiển bởi file launch. Mối quan hệ và luồng hoạt động giữa các node tuân thủ nghiêm ngặt nguyên lý chia tách chức năng (Decoupled Nodes):

```
                       [config.json]                 [calibration.json]
                             │                                │
                             ▼ (Cấu hình nguồn)               ▼ (Hot-reload)
[Camera / Video] ──► (VideoPublisherNode) ──────────► (NCNNInferenceNode)
                             │                                │
                             ▼ (/camera/.../compressed)       ▼ (/avs/telemetry)
                      [Web Dashboard] ◄────────────── (IPMTransformNode)
                             ▲                                │
                             │ (/avs/route_intent)            ▼ (/avs/telemetry_realworld)
                             └─────────────────────── (LaneErrorNode)
                                                              │
                                                              ▼ (/avs/control_error)
                                                        [Downstream Control]
```

---

## 2. Chi tiết Thiết kế và Vai trò của từng Node

### 2.1. Node `VideoPublisherNode` (Đọc và Phát hình ảnh)
- **File nguồn:** `src/video_publisher_node.cpp`
- **Class C++:** `VideoPublisherNode`
- **Vai trò:** Đọc nguồn video thô từ camera vật lý V4L2 hoặc file video thử nghiệm `.mp4`, sau đó đóng gói và phát dưới dạng tin nhắn ảnh thô (`Image`) và ảnh nén (`CompressedImage`).

#### 2.1.1. Các kỹ thuật triển khai cốt lõi:
1. **Background Capture Thread (Tách biệt luồng đọc ảnh):**
   - Đọc ảnh từ camera (`cv::VideoCapture::read()`) có thể bị block do trễ đồng bộ phần cứng. Để tránh block luồng chính (Main Thread) của ROS2, node khởi chạy một luồng chạy nền độc lập `capture_loop()`.
2. **Buffer Overwrite size = 1 (Chống tích lũy trễ):**
   - Khi chạy camera trực tiếp (`is_camera_source_ = true`), luồng nền liên tục ghi đè frame mới nhất vào bộ đệm `latest_frame_`. Nếu node xử lý AI phía sau bị chậm, các frame cũ trong bộ đệm của camera sẽ bị bỏ qua thay vì bị dồn ứ trong hàng đợi, đảm bảo thời gian thực tuyệt đối (Zero Frame Lag).
3. **Producer-Consumer Synchronization (Đồng bộ cho File Video):**
   - Khi chạy file `.mp4` để kiểm thử, luồng nền sẽ tạm dừng giải mã frame tiếp theo cho đến khi frame hiện tại được timer callback consume xong (`new_frame_available_ == false`), giúp tái hiện đúng tốc độ khung hình gốc của video.
4. **Hardware-Accelerated MJPEG Decoding:**
   - Cưỡng bức cấu hình định dạng pixel `MJPG` cho camera USB để giải mã phần cứng trực tiếp, giảm đáng kể tải CPU khi thu nhận ảnh phân giải $640 \times 480$ ở $30\text{ fps}$.

#### 2.1.2. Tham số Node (ROS2 Parameters):
- `video_path` (`string`): Đường dẫn camera `/dev/video*` hoặc file `.mp4`.
- `loop` (`bool`): Tự động phát lại video từ đầu khi kết thúc (mặc định `true`).
- `fps_override` (`double`): Ghi đè FPS phát. Nếu đặt $\le 0.0$, hệ thống tự nhận diện FPS gốc.
- `camera_width`, `camera_height`, `camera_fps` (`int`): Kích thước và FPS cấu hình cho camera.

---

### 2.2. Node `NCNNInferenceNode` (suy luận phân vùng AI)
- **File nguồn:** `src/ncnn_inference_node.cpp`
- **Class C++:** `NCNNInferenceNode`
- **Vai trò:** Đăng ký nhận ảnh từ camera, thực hiện tiền xử lý, chạy mô hình phân vùng qua NCNN, chạy thuật toán bám vết đối tượng 2D, trích xuất đa giác vùng làn và publish telemetry không gian ảnh.

#### 2.2.1. Các kỹ thuật triển khai cốt lõi:
1. **Greedy 2D IoU Tracking (Bám vết đối tượng):**
   - Để giữ tính liên tục của các làn đường và xe cộ qua các frame, node triển khai bộ bám vết IoU tham lam. Tính toán chỉ số giao chéo (IoU) giữa bounding box của các đối tượng phát hiện ở frame hiện tại với các vết bám (`Track`) đang hoạt động ở frame trước.
   - Nếu $\text{IoU} \ge 0.3$, đối tượng được gán ID bám vết duy nhất (ví dụ: `main_lane_0`, `vehicle_3`). Các vết bám không được khớp trong 5 frame liên tiếp sẽ bị giải phóng.
2. **Contour Extraction thay cho Full Mask (Tiết kiệm băng thông):**
   - Sử dụng hàm `cv::findContours` để rút trích các đa giác biên (`polygons` dạng danh sách điểm) của mặt nạ phân vùng. Việc này giúp giảm kích thước payload telemetry từ hàng trăm KB xuống dưới $5\text{ KB}$.
3. **Previous-Frame Telemetry Reporting (Báo cáo trễ chéo):**
   - Để đo đạc chính xác thời gian đóng gói JSON (`json_finalize_latency`) và thời gian phát dữ liệu qua DDS (`publish_latency`) mà không làm méo mó các chỉ số này do chính hành động đo đạc gây ra, node đính kèm số liệu đo của **frame liền trước** vào gói tin telemetry của frame hiện tại.

#### 2.2.2. Tham số Node:
- `model_param_path` & `model_bin_path` (`string`): Đường dẫn tệp cấu hình và trọng số mạng NCNN.
- `prob_threshold` & `nms_threshold` (`float`): Ngưỡng tin cậy phân loại và ngưỡng lọc chồng lấn hộp bao.
- `num_threads` (`int`): Số luồng CPU tối đa cấp cho NCNN (mặc định `4` nhân vật lý).

---

### 2.3. Node `IPMTransformNode` (Biến đổi hình học IPM)
- **File nguồn:** `src/ipm_transform_node.cpp`
- **Class C++:** `IPMTransformNode`
- **Vai trò:** Biến đổi các điểm đa giác từ tọa độ ảnh (pixel) sang tọa độ thực mặt đường (mm) sử dụng ma trận Homography, trích xuất đường tâm làn đường và khớp đa thức bậc 3.

#### 2.3.1. Các kỹ thuật triển khai cốt lõi:
1. **Calibration Hot-Reload (Nạp động cấu hình):**
   - Node liên tục giám sát tệp `calibration.json`. Nếu phát hiện tệp thay đổi thời gian sửa đổi (Modified Time), node tự động nạp lại ma trận Homography $H$ ngay ở chu kỳ callback tiếp theo mà không cần khởi động lại toàn bộ hệ thống ROS2.
2. **Dynamic Look-ahead Distance (Khoảng cách xem trước động):**
   - Đăng ký nhận thông tin tốc độ hiện tại từ `/odom_raw`. Tính toán khoảng cách bám đuổi $d = v \cdot T_{preview}$ và kẹp trong khoảng $[d_{min}, d_{max}]$. Khi xe chạy nhanh, khoảng xem trước tự động kéo dài ra để cua mượt; khi xe chạy chậm, khoảng xem trước co lại để bám cua gắt.
3. **Robust Waypoint Sweeping & SVD Fitting:**
   - Triển khai quét lát cắt dọc ($Y$-sweep) hoặc ngang ($X$-sweep) để lấy tọa độ trung điểm. Lọc bỏ các lát cắt bị biến dạng (bề rộng lớn hơn 1.3 lần bề rộng trung vị). Giải hệ phương trình bình phương tối thiểu khớp đa thức bậc 3 bằng phân tích SVD thông qua hàm `cv::solve(..., cv::DECOMP_SVD)` để triệt tiêu hiện tượng kỳ dị ma trận.

#### 2.3.2. Tham số Node:
- `calibration_file_path` (`string`): Đường dẫn tệp cấu hình Homography.
- `lookahead_T_preview` (`double`): Thời gian xem trước (mặc định $0.15\text{ s}$).
- `lookahead_d_min_mm` & `lookahead_d_max_mm` (`double`): Khoảng cách xem trước tối thiểu ($120\text{ mm}$) và tối đa ($450\text{ mm}$).

---

### 2.4. Node `LaneErrorNode` (Khối Quyết định và Tính sai số)
- **File nguồn:** `src/control_node.cpp`
- **Class C++:** `LaneErrorNode`
- **Vai trò:** Khối hậu xử lý hình học và ra quyết định. Node theo dõi `route_intent` từ Dashboard, chọn làn đường mục tiêu phù hợp, trộn làm mượt quỹ đạo theo thời gian và xuất sai số cuối cùng cho bộ điều khiển.

#### 2.4.1. Kiến trúc lớp phân tầng nội bộ (Helper Classes):
Để tránh một file mã nguồn cồng kềnh khó kiểm thử, logic xử lý của `LaneErrorNode` được module hóa thành 4 lớp C++ độc lập:
1. **`PathObservationBuilder`**:
   - Nhận chuỗi JSON thực địa từ `/avs/telemetry_realworld`, phân tích cú pháp để trích xuất danh sách các làn đường quan sát được (`main-lane`, `other-lane`, `turn-lane`) và phân loại vạch kẻ đường ngăn cách.
2. **`TrajectoryPlanner`**:
   - Chạy ở mọi frame. Dựa trên trạng thái quyết định hiện tại và thông tin từ `PathObservationBuilder`, planner lập kế hoạch tạo ra một quỹ đạo ứng viên (Candidate Trajectory) bám theo làn phù hợp. Khi rẽ, nó tự động chọn làn rẽ tối ưu (ví dụ: rẽ phải chọn làn gần hơn, rẽ trái chọn làn xa hơn).
3. **`TrajectoryNormalizer` (Làm mượt không gian - thời gian):**
   - Nhận quỹ đạo ứng viên từ planner, thực hiện phép trộn động học Spatial Blending với quỹ đạo đã cam kết ở frame trước. Kỹ thuật này giúp loại bỏ hoàn toàn các dao động nhỏ do nhiễu phân vùng ảnh.
4. **`TrajectoryManager` (Máy trạng thái quyết định):**
   - Quản lý **Decision State Machine** chuyển đổi giữa các trạng thái điều hướng (`FOLLOW_MAIN`, `TURN_RIGHT`, `TURN_LEFT`, `LANE_CHANGE`, `BLOCKED`, `RECOVERY`). Lớp này quản lý các cửa sổ thời gian (hold window) để chống nhiễu chuyển đổi trạng thái liên tục (hysteresis).

#### 2.4.2. Trích xuất sai số điều khiển:
- Node xác định điểm mục tiêu $P_{target}(x_{target}, y_{target})$ trên quỹ đạo bám đường mượt mà cuối cùng nằm cách xe một khoảng bằng lookahead distance.
- Từ điểm này, node suy ra các tham số hình học:
  - Lệch ngang: $e_x = x_{target}$ (lệch phải $>0$, lệch trái $<0$).
  - Lệch dọc: $e_y = y_{target}$.
  - Góc lệch hướng: $\theta = \arctan(a_1)$ từ hệ số tiếp tuyến đa thức tại đầu xe hoặc $\theta = \operatorname{atan2}(x_{target}, y_{target})$.
- Các thông số này được publish qua topic `/avs/control_error`.

#### 2.4.3. Tham số Node:
- `turn_proximity_mm` (`double`): Khoảng cách tiếp cận ngã rẽ để chuyển trạng thái ($500\text{ mm}$).
- `turn_done_mm` (`double`): Khoảng cách tối thiểu đi sâu vào làn mới để xác nhận hoàn thành rẽ ($200\text{ mm}$).
- `theta_done_rad` (`double`): Góc hướng lệch tối đa so với làn mới để xác nhận hoàn tất đi thẳng ($0.1\text{ rad}$).

---

### 2.5. Node `VideoTestNode` (Kiểm thử ngoại tuyến)
- **File nguồn:** `src/video_test_node.cpp`
- **Class C++:** `VideoTestNode`
- **Vai trò:** Chạy ngoại tuyến trên PC phát triển, đọc trực tiếp file video `.mp4` và chạy mô hình phân vùng để đo đạc chính xác các chỉ số FPS xử lý và latency của mô hình AI mà không cần khởi động luồng camera USB thực tế. Node này cũng ghi luồng video kết quả ra file phục vụ phân tích trực quan.

---

## 3. Tổng hợp Bảng đặc tả cổng giao tiếp của các Node

| Tên Node | Subscribe Topics | Publish Topics | Đọc Cấu hình / File |
| :--- | :--- | :--- | :--- |
| **`video_publisher_node`** | *(Không có)* | `/camera/image_raw` <br> `/camera/image_raw/compressed` | `config.json` |
| **`ncnn_inference_node`** | `/camera/image_raw` | `/avs/telemetry` | Trọng số NCNN (`.param`/`.bin`) |
| **`ipm_transform_node`** | `/avs/telemetry` <br> `/odom_raw` | `/avs/telemetry_realworld` | `calibration.json` |
| **`control_node`** | `/avs/telemetry_realworld` <br> `/avs/route_intent` <br> `/avs/cmd` <br> `/odom_raw` | `/avs/control_error` <br> `/avs/lane_state` | *(Thông số máy trạng thái)* |
| **`video_test_node`** | *(Không có - tự đọc file)* | *(Không có - tự ghi file)* | `config.json`, Trọng số NCNN |
