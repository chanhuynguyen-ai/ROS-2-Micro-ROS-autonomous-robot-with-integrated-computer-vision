# Báo cáo Chi tiết: Container hóa và Cấu hình Docker (AVS)

Báo cáo này tập trung phân tích chi tiết **Phần III, Mục 8: Docker hóa hệ thống** dựa trên các tệp tin cấu hình Docker vật lý (`docker/Dockerfile`, `docker-compose.yml`, và `docker-compose.prod.yml`). Tài liệu này phân tích chi tiết quy trình xây dựng image, các tham số tối ưu hóa phần cứng ARM64, và giải pháp điều phối (orchestration) đa container để tối ưu hóa hiệu năng truyền tải dữ liệu và tương tác phần cứng.

---

## 1. Triết lý áp dụng Docker trong Hệ thống AVS

Việc sử dụng Docker để đóng gói hệ thống AVS trên Raspberry Pi 5 và PC phát triển mang lại các lợi ích cốt lõi sau:
- **Chuẩn hóa môi trường:** Tránh xung đột phiên bản của các thư viện phụ thuộc cực kỳ nhạy cảm như OpenCV, Protobuf, và ROS2 Humble. Lập trình viên không cần cài đặt ROS2 trực tiếp trên máy Host.
- **Tương thích chéo hệ điều hành:** Cho phép chạy phần mềm nhận diện ROS2 Humble (vốn chạy mặc định trên Ubuntu 22.04 LTS) trực tiếp trên hệ điều hành Raspberry Pi OS (Debian-based) của Pi 5 mà không gặp trở ngại tương thích.
- **Triển khai tức thì (One-click Deployment):** Đóng gói toàn bộ mã nguồn biên dịch và môi trường vào một Image duy nhất, giúp tăng tốc độ phân phối phần mềm lên các xe robot trong dây chuyền sản xuất.

---

## 2. Chi tiết Thiết kế Dockerfile và Tối ưu hóa Image

Tệp tin Dockerfile đặt tại `docker/Dockerfile` sử dụng chiến lược xây dựng tối ưu hóa CPU-only cho kiến trúc ARM64 (Raspberry Pi 5):

### 2.1. Phân tích các bước xây dựng trong Dockerfile
1. **Base Image:** Sử dụng `ros:humble-ros-base` làm nền tảng. Đây là image chính thức của OSRF chứa sẵn các thư viện lõi của ROS2 Humble (như `rclcpp`, `std_msgs`, `sensor_msgs`), giúp giảm dung lượng image so với việc cài đặt ROS từ image Ubuntu thô.
2. **Cài đặt các gói phụ thuộc hệ thống (System Dependencies):**
   - Các gói biên dịch mã nguồn: `build-essential`, `cmake`, `git`, `pkg-config`.
   - Các thư viện xử lý ảnh và giao tiếp ROS: `libopencv-dev` (OpenCV C++), `python3-opencv` (OpenCV Python), `ros-humble-cv-bridge` (chuyển đổi Mat sang Image Message), `ros-humble-image-transport` (nén ảnh).
   - Thư viện xử lý song song và AI: `libomp-dev` (OpenMP cho xử lý đa luồng), `protobuf-compiler` & `libprotobuf-dev` (hỗ trợ định dạng NCNN).
3. **Cơ chế nạp môi trường ROS2 tự động:**
   Tệp `docker/entrypoint.sh` tự động thực thi khi container khởi chạy để source môi trường `/opt/ros/humble/setup.bash`, giúp các biến môi trường ROS2 luôn sẵn sàng cho các lệnh chạy node phía sau.

### 2.2. Thuật toán tối ưu hóa biên dịch NCNN từ mã nguồn
Một trong những bước quan trọng nhất trong Dockerfile là tự động tải và tự biên dịch thư viện NCNN phiên bản `20240820` với các cờ tối ưu hóa phần cứng nghiêm ngặt:
```dockerfile
RUN cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr \
          -DNCNN_VULKAN=OFF \
          -DNCNN_BUILD_EXAMPLES=OFF \
          -DNCNN_BUILD_TOOLS=OFF \
          -DNCNN_BUILD_BENCHMARK=OFF \
          -DNCNN_SHARED_LIB=ON \
          -DNCNN_ARM_NEON=ON \
          ..
```
- **`-DNCNN_VULKAN=OFF`:** Tắt hoàn toàn hỗ trợ GPU Vulkan. Điều này ép hệ thống chạy suy luận AI thuần túy trên CPU, giúp giảm độ trễ sao chép dữ liệu giữa RAM CPU và VRAM GPU, đồng thời tránh quá nhiệt trên mạch Pi 5.
- **`-DNCNN_ARM_NEON=ON`:** Kích hoạt tập lệnh tối ưu hóa vector hóa **ARM NEON SIMD** trên CPU Cortex-A76 của Raspberry Pi 5. Giúp tăng tốc độ xử lý ma trận và tích chập lên gấp 3-4 lần.
- **`-DNCNN_SHARED_LIB=ON`:** Tạo thư viện liên kết động `.so` giúp chia sẻ tài nguyên bộ nhớ khi nhiều node ROS2 cùng gọi đến thư viện nhận diện AI.

---

## 3. Điều phối Đa Container qua Docker Compose

Hệ thống phân rã thành 3 dịch vụ container chạy song song để đảm bảo tính module hóa và ổn định cao:

```
                            [Mạng LAN / Internet]
                                      │
                     Dịch vụ Web: web_dashboard (Port 8000)
                                      │ (Internal Websocket / ROS2 DDS)
                                      ▼
[Thiết bị Camera] ──► Dịch vụ Camera: video_publisher (Mount /dev)
                                      │ (Shared Memory IPC / ROS2 Topic)
                                      ▼
                     Dịch vụ AI & Geometry: avs_perception
```

### 3.1. Phân tích chức năng các Dịch vụ trong Compose

1. **`avs_perception` (Container Nhận diện & Hình học):**
   - Đảm nhiệm biên dịch workspace ROS2 bằng lệnh `colcon build --symlink-install`.
   - Chạy đồng thời 3 node: `ncnn_inference_node`, `ipm_transform_node`, và `control_node` (LaneErrorNode).
2. **`video_publisher` (Container Camera):**
   - Độc lập đọc luồng ảnh từ thiết bị camera phần cứng vật lý và phát lên mạng ROS2.
   - Thiết kế đồng bộ hóa khởi động: Container này liên tục kiểm tra tệp tin `install/setup.bash` của container `avs_perception` tạo ra. Khi quá trình biên dịch hoàn tất, nó mới tự động chạy `video_publisher_node`, tránh lỗi thiếu thư viện khi chạy song song.
3. **`web_dashboard` (Container Giao diện Giám sát):**
   - Chạy máy chủ backend FastAPI (`main.py`) trên cổng `8000` để trung chuyển dữ liệu telemetry và video nén lên trình duyệt Firefox của người dùng.

---

## 4. Các giải pháp Tối ưu hóa Hiệu năng Container hóa Đặc thù

Để hệ thống chạy mượt mà ở tần số $30\text{ fps}$ trên phần cứng Raspberry Pi 5 hạn chế, Docker Compose áp dụng các cấu hình hệ thống chuyên sâu sau:

### 4.1. Chế độ Mạng Host (`network_mode: host`)
- **Cách hoạt động:** Container sử dụng trực tiếp ngăn xếp mạng (Network Stack) của hệ điều hành Host thay vì tạo mạng ảo Bridge.
- **Lợi ích:** Loại bỏ hoàn toàn overhead định tuyến mạng và NAT của Docker. Cho phép giao thức truyền thông ROS2 DDS tự động phát hiện (Auto-discovery) và kết nối trực tiếp với các thiết bị ROS2 khác trong cùng mạng LAN (ví dụ: máy tính giám sát của kỹ sư) mà không cần cấu hình định tuyến cổng phức tạp.

### 4.2. Bộ nhớ Dùng chung Host IPC (`ipc: host`)
- **Cách hoạt động:** Cho phép các container chia sẻ phân đoạn bộ nhớ dùng chung (Shared Memory segments) với máy Host và giữa các container với nhau.
- **Lợi ích:** Việc truyền ảnh thô kích thước lớn ($640 \times 480 \times 3$ bytes) qua DDS giữa container `video_publisher` và container `avs_perception` được thực hiện trực tiếp thông qua cơ chế Zero-copy Shared Memory. Điều này triệt tiêu hoàn toàn độ trễ đóng gói gói tin mạng cục bộ (Socket loopback overhead), giúp tiết kiệm tải CPU đáng kể.

### 4.3. Đặc quyền truy cập phần cứng (`privileged: true` và Mount `/dev`)
- **`privileged: true`:** Cấp quyền quản trị hệ thống của Host cho container, cho phép container can thiệp vào các tài nguyên phần cứng cấp thấp.
- **Mount `/dev`:** Việc gắn kết toàn bộ thư mục `/dev` từ Host vào container giúp `video_publisher` truy cập trực tiếp vào các nút thiết bị camera vật lý `/dev/video*`. Nhờ đó, OpenCV bên trong container có thể thiết lập các thông số phần cứng V4L2 của camera USB (chất lượng ảnh, kích thước, định dạng MJPEG) như thể đang chạy trực tiếp trên hệ điều hành Host.

### 4.4. Phân vùng miền truyền thông ROS (`ROS_DOMAIN_ID=20`)
- Thiết lập biến môi trường `ROS_DOMAIN_ID=20` đồng nhất trên cả 3 container. Điều này đảm bảo toàn bộ các node của AVS giao tiếp khép kín trong miền ID 20, tránh bị lẫn hoặc nhiễu tín hiệu DDS với các robot hoặc thiết bị ROS2 khác đang hoạt động trong cùng mạng LAN của phòng thí nghiệm.
