# Báo cáo Chi tiết: Thiết kế Package ROS2 avs_perception

Báo cáo này tập trung phân tích chi tiết **Phần III, Mục 2: Thiết kế package ROS2 avs_perception** dựa trên đặc tả hệ thống AVS. Tài liệu này làm rõ cấu trúc thư mục, các phụ thuộc hệ thống, thiết kế cấu hình biên dịch (`CMakeLists.txt` & `package.xml`), cơ chế khởi chạy (`launch`), và các kỹ thuật tối ưu hóa mức biên dịch của package thị giác máy tính cốt lõi này.

---

## 1. Tổng quan về Package avs_perception

Package `avs_perception` là trung tâm xử lý thị giác máy tính và hình học đường đi của dự án AVS. Nó thực hiện toàn bộ chu trình từ thu nhận ảnh nén, suy luận phân vùng (instance segmentation), biến đổi IPM sang tọa độ thực, quy hoạch và làm mượt quỹ đạo, cho đến việc tính toán sai số hình học cung cấp cho bộ điều khiển xe tự hành.

### Thông tin cơ bản:
- **Tên Package:** `avs_perception`
- **Hệ thống xây dựng (Build System):** `ament_cmake` (chuẩn C++ của ROS2 Humble).
- **Giấy phép (License):** MIT.
- **Maintainer:** goln (longhuynh.ai.contact@gmail.com).
- **Mục tiêu thiết kế:** Đảm bảo độ trễ xử lý thấp, tối ưu hóa tối đa năng lực CPU đa nhân trên Raspberry Pi 5 thông qua thư viện NCNN và biên dịch C++ tối ưu hóa phần cứng.

---

## 2. Cấu trúc Thư mục và File trong Package

Package được tổ chức theo chuẩn cấu trúc của một ROS2 C++ package kết hợp với thư viện xử lý ngoài (NCNN):

```
avs_perception/
├── CMakeLists.txt                  # File cấu hình biên dịch CMake chính
├── package.xml                     # Định nghĩa metadata, thông tin dependencies của package
├── include/                        # Thư mục chứa các file header (.hpp)
│   └── avs_perception/
│       └── yolo26_seg.hpp          # Định nghĩa lớp YOLO26Seg bao bọc thư viện NCNN
├── src/                            # Thư mục chứa mã nguồn C++ (.cpp)
│   ├── yolo26_seg.cpp              # Implement lớp YOLO26Seg (tiền xử lý, suy luận NCNN, hậu xử lý mask)
│   ├── ncnn_inference_node.cpp     # Node ROS2 nhận ảnh raw/compressed, suy luận AI và publish telemetry điểm ảnh
│   ├── ipm_transform_node.cpp      # Node ROS2 biến đổi tọa độ IPM, trích centerline và khớp đa thức bậc 3
│   ├── control_node.cpp            # Node ROS2 (LaneErrorNode) quản lý quỹ đạo, blending và xuất sai số ex, ey, theta
│   ├── video_publisher_node.cpp    # Node ROS2 đọc camera V4L2/video file và publish ảnh nén
│   └── video_test_node.cpp         # Node ROS2 chạy ngoại tuyến (offline profile) đo latency và FPS
├── launch/                         # Thư mục chứa cấu hình khởi chạy hệ thống
│   └── perception.launch.py        # File launch Python điều phối hoạt động toàn bộ hệ thống
└── scripts/                        # Thư mục chứa các script hỗ trợ lượng hóa và xuất model
    ├── export_ncnn.py              # Script xuất model YOLO PyTorch sang ONNX và NCNN
    └── reexport_and_quantize.py    # Script xuất lại và lượng hóa INT8 mô hình qua tập hiệu chuẩn
```

---

## 3. Thiết kế Quản lý Phụ thuộc và Cấu hình Biên dịch

### 3.1. Phân tích `package.xml`
File `package.xml` định nghĩa các package ROS2 Humble cần thiết cho quá trình build và chạy của `avs_perception`:
- **`ament_cmake`**: Công cụ build chính.
- **`rclcpp`**: Thư viện ROS2 C++ client.
- **`sensor_msgs`**: Chứa kiểu tin nhắn hình ảnh (`Image`, `CompressedImage`).
- **`std_msgs`**: Chứa tin nhắn dạng chuỗi (`String`) phục vụ truyền JSON.
- **`geometry_msgs`**: Kiểu tin nhắn hình học (như `Point`, `Twist` phục vụ tương thích điều khiển).
- **`nav_msgs`**: Kiểu tin nhắn phục vụ điều hướng (như `Path`, `Odometry`).
- **`cv_bridge`**: Cầu nối chuyển đổi ma trận ảnh giữa ROS2 Image Message và OpenCV `cv::Mat`.
- **`image_transport`**: Hỗ trợ tối ưu hóa và nén đường truyền hình ảnh.

### 3.2. Thiết kế biên dịch trong `CMakeLists.txt`
File `CMakeLists.txt` được thiết kế tối ưu hóa cực kỳ chặt chẽ nhằm tận dụng phần cứng biên:

#### 3.2.1. Cấu hình cờ biên dịch Release tối ưu hóa CPU ARM64
Nếu hệ thống phát hiện build kiểu `Release` trên cấu trúc chip ARM64 (Raspberry Pi 5):
```cmake
add_compile_options(
  -O3                           # Tối ưu hóa hiệu năng biên dịch cao nhất của GCC
  -march=armv8.2-a              # Biên dịch đích danh tập lệnh ARMv8.2-A (Pi 5)
  -mtune=cortex-a76             # Tối ưu hóa sơ đồ đường ống lệnh (pipeline) riêng cho nhân Cortex-A76
  -ffast-math                   # Tăng tốc các phép tính toán học dấu phẩy động (bỏ qua một số kiểm tra IEEE)
  -funroll-loops                # Trải vòng lặp để giảm overhead nhánh nhảy rẽ nhánh của CPU
)
```
*Đối với máy tính xách tay phát triển (x86_64), hệ thống tự động fallback về cờ `-O3 -ffast-math -funroll-loops` chung.*

#### 3.2.2. Tìm kiếm Thư viện Hệ thống ngoài ROS2
- **OpenCV**: Dùng để xử lý ma trận ảnh đầu vào, tìm contour, vẽ debug và giải mã.
- **NCNN**: Kiểm tra các đường dẫn cục bộ (ví dụ cài đặt chung thư mục dự án `/home/goln/SimpleSysIDV/ncnn`) trước khi tìm trong hệ thống (`/usr/lib`, `/usr/local/lib`) nhằm đảm bảo khả năng build di động (portable).

#### 3.2.3. Thiết kế Shared Library để tái sử dụng mã nguồn
Thay vì biên dịch file wrapper YOLO26 ở tất cả các node gây phình kích cỡ file thực thi và trùng lặp ký hiệu (duplicate symbols), hệ thống tạo ra một thư viện dùng chung:
```cmake
add_library(yolo26_seg_lib src/yolo26_seg.cpp)
target_link_libraries(yolo26_seg_lib ncnn ${OpenCV_LIBRARIES})
```
Các node chạy thực thi (`ncnn_inference_node`, `video_test_node`) chỉ cần liên kết tĩnh với `yolo26_seg_lib`, giúp tăng tốc độ build đáng kể.

---

## 4. Chi tiết các Node và Thành phần lõi

| File nguồn | Tên Node ROS2 | Vai trò / Nhiệm vụ chính |
| :--- | :--- | :--- |
| `yolo26_seg.cpp` | *(Shared Lib)* | Bao bọc mô hình NCNN; thực hiện co dãn ảnh về $320\times 320$, chuẩn hóa giá trị $[0,1]$, suy luận mạng nơ-ron, giải mã bounding box, lọc NMS, tổ hợp tuyến tính khôi phục mặt nạ phân vùng nhị phân cục bộ. |
| `ncnn_inference_node.cpp` | `ncnn_inference_node` | Nhận ảnh nén/thô, chạy suy luận qua thư viện `yolo26_seg_lib`, trích polygon đường bao vùng làn đường, đóng gói JSON telemetry (Pixel Space) và publish. |
| `ipm_transform_node.cpp` | `ipm_transform_node` | Áp dụng IPM Homography chuyển polygon ảnh sang mm thực tế; thực hiện thuật toán sweep tìm centerline rời rạc; dùng SVD khớp đa thức bậc 3 dọc $x(y)$ và ngang $y(x)$ để publish telemetry thực địa. |
| `control_node.cpp` | `control_node` | Node ra quyết định (Decision Node); theo dõi intent và trạng thái xe để chọn lane mục tiêu; áp dụng blending làm mượt quỹ đạo theo thời gian; publish sai số $e_x$, $e_y$, $\theta$ và trạng thái debug làn. |
| `video_publisher_node.cpp`| `video_publisher_node`| Đọc camera vật lý USB hoặc đọc video mô phỏng để publish dữ liệu ảnh nén lên luồng ROS2. |
| `video_test_node.cpp` | `video_test_node` | Chạy offline trên video thử nghiệm, kiểm thử chức năng nhận diện, đo đạc độ trễ chi tiết của mô hình suy luận và xuất kết quả. |

---

## 5. Thiết kế Cơ chế khởi chạy hệ thống (Launch Mechanism)

Để quản lý cấu hình và đồng bộ các node một cách linh hoạt, package cung cấp file launch Python **`perception.launch.py`**.

### Các tham số cấu hình chính trong file Launch:
1. **`mode`**: Định nghĩa chế độ chạy của hệ thống:
   - `live`: Chạy node suy luận thời gian thực (`ncnn_inference_node`) kết hợp với luồng camera USB thực tế.
   - `test`: Khởi chạy node profiling video offline (`video_test_node`) để kiểm tra hiệu năng hệ thống trên tệp video mẫu.
2. **`model_param_path` & `model_bin_path`**: Đường dẫn tới mô hình phân vùng lượng hóa INT8 của NCNN.
3. **Cấu hình IPM & Look-ahead (ở node `ipm_transform_node`):**
   - `calibration_file_path`: `/workspace/config/calibration.json`.
   - `lookahead_T_preview`: Hệ số thời gian xem trước (mặc định $0.15\text{ s}$) để điều chỉnh khoảng cách xem trước động: $d = v \cdot T_{preview}$.
   - `lookahead_d_min_mm` & `lookahead_d_max_mm`: Giới hạn tối thiểu ($120\text{ mm}$) và tối đa ($450\text{ mm}$) của khoảng cách xem trước.
4. **Cấu hình máy trạng thái rẽ (ở node `control_node`):**
   - `turn_proximity_mm`: Khoảng cách từ xe tới ngã rẽ để kích hoạt trạng thái chuẩn bị rẽ ($500\text{ mm}$).
   - `turn_done_mm`: Khoảng cách vượt qua ngã rẽ để xác nhận hoàn tất ($200\text{ mm}$).
   - `theta_done_rad`: Ngưỡng góc hướng ($0.1\text{ rad} \approx 5.7^\circ$) để xác nhận xe đã đi thẳng vào làn mới sau khi rẽ.

---

## 6. Kỹ thuật Tối ưu hóa Đặc thù trong Thiết kế Package

Package `avs_perception` được tối ưu hóa đặc biệt theo triết lý CPU-Centric (tối ưu hóa luồng tính toán trên CPU biên):
1. **Chia sẻ Thư viện Wrapper (`yolo26_seg_lib`)**:
   - Tránh biên dịch mã nguồn xử lý AI nhiều lần, giúp giảm dung lượng của các file thực thi nhị phân đầu ra khi cài đặt lên Raspberry Pi 5.
2. **Multi-threading và Tối ưu NCNN**:
   - Bên trong `yolo26_seg.cpp`, cấu hình `net.opt.num_threads = 3` để phân phối tính toán suy luận song song trên 3 nhân vật lý Cortex-A76 mạnh mẽ của Pi 5, chừa lại 1 nhân cho các tác vụ truyền thông ROS2, IPM và Dashboard.
   - Bật các tùy chọn tối ưu hóa bộ nhớ đệm và lượng hóa của NCNN: `use_fp16_packed`, `use_fp16_storage`, `use_int8_inference`.
3. **Giảm thiểu kích thước Payload truyền thông**:
   - Thay vì truyền tải các mặt nạ phân vùng dạng ảnh nhị phân đầy đủ kích thước ($640 \times 480$ pixel ở định dạng ảnh xám chiếm $307.2\text{ KB}$ mỗi frame) qua lại giữa các node ROS2, node nhận diện chuyển đổi mặt nạ thành danh sách đa giác tối giản (Polygon Points - chỉ gồm các cặp điểm góc). Điều này làm giảm kích thước gói tin qua topic `/avs/telemetry` xuống dưới $5\text{ KB}$, giảm tải hoàn toàn băng thông bộ nhớ RAM khi trao đổi dữ liệu.
