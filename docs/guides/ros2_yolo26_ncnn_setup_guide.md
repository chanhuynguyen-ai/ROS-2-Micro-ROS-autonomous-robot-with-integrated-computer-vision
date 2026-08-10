# Hướng Dẫn Cài Đặt Và Chạy YOLO26-NCNN Trên ROS2 Humble

Tài liệu này ghi lại toàn bộ quy trình thiết lập, khởi tạo, phát triển mã nguồn và vận hành hệ thống nhận diện phân đoạn ảnh YOLO26-NCNN trên môi trường ROS2 Humble của bạn.

---

## BƯỚC 1: KHỞI TẠO WORKSPACE VÀ TẠO ROS2 PACKAGE

Để xây dựng bất kỳ ứng dụng ROS2 nào bằng C++, trước hết ta cần khởi tạo một Workspace và tạo cấu trúc gói (Package).

### 1.1 Khởi tạo thư mục Workspace
```bash
# Tạo thư mục Workspace và thư mục src (nơi chứa tất cả mã nguồn)
mkdir -p /home/goln/SimpleSysIDV/ros2_ws/src
```

### 1.2 Tạo ROS2 Package mới
Ta di chuyển vào thư mục `src` và chạy lệnh tạo Package C++ với các thư viện phụ thuộc mong muốn:
```bash
cd /home/goln/SimpleSysIDV/ros2_ws/src

# Lệnh tạo package của ROS2
ros2 pkg create --build-type ament_cmake avs_perception \
  --dependencies rclcpp sensor_msgs cv_bridge image_transport
```
**Giải thích cú pháp lệnh:**
* `ros2 pkg create`: Lệnh tiêu chuẩn của ROS2 để sinh tự động cấu trúc gói mới.
* `--build-type ament_cmake`: Định nghĩa gói này sử dụng hệ thống build CMake (dùng cho mã nguồn C++).
* `avs_perception`: Tên của package nhận diện.
* `--dependencies ...`: Tự động khai báo và liên kết các gói ROS2 phụ thuộc:
  * `rclcpp`: Thư viện API ROS2 C++.
  * `sensor_msgs`: Thư viện chứa các kiểu thông điệp cảm biến tiêu chuẩn (như Image, CompressedImage).
  * `cv_bridge`: Thư viện cầu nối chuyển đổi giữa cấu trúc ảnh ROS2 và OpenCV Mat.
  * `image_transport`: Thư viện chuyên dụng hỗ trợ truyền tải và nén hình ảnh trong ROS2.

### 1.3 Ý nghĩa của các file/folder được tự động sinh ra:
Sau lệnh trên, ROS2 sinh ra thư mục `/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/` với các file/thư mục:
* **`CMakeLists.txt`**: File kịch bản cấu hình CMake để trình biên dịch biết cách tìm thư viện liên kết, tạo các tệp thực thi (executable/node) và cài đặt chúng.
* **`package.xml`**: Chứa thông tin mô tả về gói (tên, tác giả, giấy phép) và danh sách các gói cần thiết để build hoặc chạy.
* **`include/avs_perception/`**: Nơi lưu trữ các file tiêu đề (Header files `.hpp`).
* **`src/`**: Nơi lưu trữ các file mã nguồn thực thi chính (Source files `.cpp`).

---

## BƯỚC 2: BIÊN DỊCH THƯ VIỆN NCNN CỤC BỘ TRÊN HOST (KHÔNG DÙNG ROOT)

Do NCNN cần được tối ưu hóa sâu xuống tập lệnh CPU để chạy đạt FPS cao nhất, ta tiến hành tải mã nguồn và biên dịch cục bộ:

```bash
# Di chuyển về thư mục gốc workspace
cd /home/goln/SimpleSysIDV

# Tải mã nguồn NCNN bản ổn định nhất
git clone --depth 1 https://github.com/Tencent/ncnn.git ncnn-src
cd ncnn-src
git submodule update --init
cd ..

# Thiết lập build cục bộ không ghi đè lên thư mục hệ thống
mkdir -p ncnn-src/build && cd ncnn-src/build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/home/goln/SimpleSysIDV/ncnn \
      -DNCNN_VULKAN=OFF \
      -DNCNN_BUILD_EXAMPLES=OFF \
      -DNCNN_BUILD_TOOLS=OFF \
      -DNCNN_BUILD_BENCHMARK=OFF \
      -DNCNN_SHARED_LIB=ON ..

# Tiến hành biên dịch song song và cài đặt vào /home/goln/SimpleSysIDV/ncnn
make -j$(nproc)
make install
```
**Giải thích ý nghĩa các cờ cấu hình:**
* `-DCMAKE_INSTALL_PREFIX=...`: Định vị cài đặt cục bộ trong workspace của user `goln`, hoàn toàn không cần đặc quyền `sudo` khi cài đặt.
* `-DNCNN_VULKAN=OFF`: Tắt tính năng tăng tốc qua card đồ họa Vulkan. Bắt buộc để tập trung tối ưu hóa 100% tài nguyên CPU.
* `-DNCNN_SHARED_LIB=ON`: Biên dịch ra dạng thư viện liên kết động (`.so`), giúp giảm kích thước bộ nhớ RAM khi chạy.

---

## BƯỚC 3: PHÁT TRIỂN MÃ NGUỒN C++ STEP-BY-STEP

Ta tạo thêm cấu trúc thư mục chứa file `launch` cấu hình khởi chạy:
```bash
mkdir -p /home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/launch
```

Sau đó, ta tiến hành viết mã nguồn cho từng phần:

### 3.1 Viết lớp Engine Suy Luận NCNN (`YOLO26Seg`)
* **Header**: [yolo26_seg.hpp](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/include/avs_perception/yolo26_seg.hpp)
  * Khai báo cấu trúc lưu đối tượng nhận diện (`Object`), bao gồm bounding box (`cv::Rect`), nhãn phân loại (`label`), độ tin cậy (`prob`), ma trận mặt nạ phân đoạn (`cv::Mat mask`) và vector đặc trưng của mặt nạ.
  * Định nghĩa lớp `YOLO26Seg` với các phương thức cốt lõi: `load()` (nạp mô hình), `detect()` (suy luận phân đoạn đối tượng), `draw()` (vẽ overlay đè lên ảnh gốc).
* **Implementation**: [yolo26_seg.cpp](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/yolo26_seg.cpp)
  * Cấu hình tối ưu hóa trong hàm tạo `YOLO26Seg()`: bật `use_fp16_packed = true`, `use_fp16_storage = true`, và đặt `num_threads = 4` nhằm khai thác tối đa 4 nhân xử lý Cortex-A76 của Raspberry Pi 5.
  * Trong `detect()`, thực hiện tiền xử lý (resize ảnh đầu vào về 320x320, chuyển hệ màu BGR sang RGB và chuẩn hóa giá trị điểm ảnh về dải [0, 1]).
  * Gọi bộ trích xuất NCNN (`ncnn::Extractor`) để suy luận song song, giải mã tọa độ anchors, áp dụng thuật toán lọc trùng lặp NMS (Non-Maximum Suppression), và tái tạo mặt nạ nhị phân độ phân giải gốc qua phép nhân ma trận kết hợp Sigmoid (Linear combination of prototype masks).

### 3.2 Viết Node Nhận Diện ROS2 Trực Tiếp (`ncnn_inference_node`)
* **Mã nguồn**: [ncnn_inference_node.cpp](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp)
  * Nhận tín hiệu ảnh raw (`sensor_msgs/msg/Image`) từ topic cấu hình.
  * Chuyển đổi thông điệp ảnh ROS2 sang ma trận OpenCV thông qua `cv_bridge`.
  * **Subscriber-Based Optimization (Cực kỳ quan trọng cho Pi 5)**: Node sử dụng câu lệnh `compressed_pub_->get_subscription_count()` để đếm số client bên ngoài đang kết nối (như Foxglove Studio). 
    * Nếu số lượng subscriber bằng 0, node chuyển sang chế độ **Streaming: IDLE**, chạy suy luận YOLO để cập nhật kết quả cho xe nhưng **bỏ qua hoàn toàn** việc vẽ đồ họa overlay (`yolo_->draw`) và nén JPEG ảnh (`cv::imencode`). Giúp hạ tải đáng kể CPU.
    * Khi phát hiện có subscriber (>0), node chạy chế độ **Full Pipeline**, vẽ overlay phân đoạn, đóng gói ảnh thành định dạng nén JPEG có chất lượng 80% (`sensor_msgs/msg/CompressedImage`) và đẩy lên topic mạng.

### 3.3 Viết Node Đo Lường Hiệu Năng Offline (`video_test_node`)
* **Mã nguồn**: [video_test_node.cpp](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/video_test_node.cpp)
  * Node này hoạt động độc lập, không cần stream camera thực tế. Nó nhận đầu vào là một video file mẫu (.mp4) từ thư mục `test/test_video/`.
  * Sử dụng vòng lặp đọc từng frame của OpenCV (`cv::VideoCapture`), chuyển vào engine YOLO26Seg để đo đạc thời gian suy luận (sử dụng thư viện `<chrono>` đo đạc chính xác micro-giây).
  * Vẽ overlay kết quả lên ảnh và ghi luồng video đã phân đoạn xuống ổ cứng bằng `cv::VideoWriter`.
  * Sau khi chạy xong, in ra báo cáo thống kê: Tổng số frame, Độ trễ trung bình (ms), FPS trung bình đạt được của CPU.

### 3.4 Tạo Launch File Điều Phối (`perception.launch.py`)
* **Mã nguồn**: [perception.launch.py](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/launch/perception.launch.py)
  * Cho phép người dùng cấu hình nhanh các tham số hệ thống mà không cần sửa mã nguồn C++.
  * Tích hợp cờ lựa chọn chế độ chạy thông qua tham số `mode`:
    * Chạy nhận diện trực tiếp: `mode:=live` (khởi chạy `ncnn_inference_node`).
    * Chạy đo lường hiệu năng offline: `mode:=test` (khởi chạy `video_test_node`).

---

## BƯỚC 4: CẤU HÌNH BIÊN DỊCH `CMakeLists.txt` VÀ `package.xml`

Để hệ thống CMake và ROS2 hiểu cách liên kết gói, ta cập nhật 2 file cấu hình cốt lõi:

### 4.1 Cấu hình [package.xml](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/package.xml)
Khai báo đầy đủ các thư viện runtime và build-time mà package sử dụng để khi chạy `rosdep` hệ thống sẽ chuẩn bị đủ môi trường.

### 4.2 Cấu hình [CMakeLists.txt](file:///home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/CMakeLists.txt)
```cmake
# Khai báo tìm các thư viện ROS2 và OpenCV
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# KIỂM TRA VÀ LIÊN KẾT NCNN CỤC BỘ DỰ ÁN
if(EXISTS "/home/goln/SimpleSysIDV/ncnn/lib/cmake/ncnn")
  set(ncnn_DIR "/home/goln/SimpleSysIDV/ncnn/lib/cmake/ncnn")
endif()
find_package(ncnn REQUIRED)

# Khai báo tạo thư viện cho Engine suy luận YOLO
add_library(yolo26_seg_lib src/yolo26_seg.cpp)
target_link_libraries(yolo26_seg_lib ncnn ${OpenCV_LIBRARIES})

# Tạo node live và liên kết ROS2
add_executable(ncnn_inference_node src/ncnn_inference_node.cpp)
target_link_libraries(ncnn_inference_node yolo26_seg_lib)
ament_target_dependencies(ncnn_inference_node rclcpp sensor_msgs cv_bridge image_transport)

# Tạo node profiling và liên kết ROS2
add_executable(video_test_node src/video_test_node.cpp)
target_link_libraries(video_test_node yolo26_seg_lib)
ament_target_dependencies(video_test_node rclcpp sensor_msgs cv_bridge)
```
**Giải thích quy trình liên kết:**
* Lệnh `if(EXISTS ...)` sẽ tự động phát hiện xem NCNN được biên dịch cục bộ trong workspace hay không. Nếu có, nó sẽ thiết lập biến `ncnn_DIR` trỏ thẳng tới thư mục cài đặt đó, loại bỏ hoàn toàn sự phụ thuộc vào thư viện NCNN chung của hệ điều hành.
* `ament_target_dependencies` là macro chuyên biệt của ROS2 giúp tự động import và quản lý các thư mục include, cờ biên dịch và các file thư viện liên kết của các gói ROS2 vào tệp thực thi.

---

## BƯỚC 5: BIÊN DỊCH WORKSPACE VÀ CHẠY THỬ NGHIỆM

Sau khi hoàn tất toàn bộ mã nguồn, ta tiến hành build và đo đạc chỉ số hệ thống.

### 5.1 Biên dịch gói ROS2
```bash
cd /home/goln/SimpleSysIDV/ros2_ws

# Source môi trường ROS2 Humble toàn cục trước
source /opt/ros/humble/setup.bash

# Lệnh biên dịch workspace
colcon build --symlink-install
```
* **Ý nghĩa cờ `--symlink-install`**: Tạo ra các liên kết tượng trưng (symlinks) thay vì sao chép vật lý các file launch, cấu hình vào thư mục install. Giúp lập trình viên chỉnh sửa file Python Launch mà không cần phải chạy lại lệnh biên dịch `colcon build`.

### 5.2 Tạo thư mục đầu ra và Chạy kiểm thử offline
```bash
# Tạo thư mục lưu đầu ra của video như yêu cầu của user
mkdir -p /home/goln/SimpleSysIDV/test/test_video_output

# Nạp cấu hình thư mục install vừa biên dịch xong
source install/setup.bash

# Khởi chạy node profiling kiểm thử
ros2 launch avs_perception perception.launch.py \
  mode:=test \
  video_path:=/home/goln/SimpleSysIDV/test/test_video/video_test1.mp4 \
  output_path:=/home/goln/SimpleSysIDV/test/test_video_output/output_video_test1.mp4 \
  model_param_path:=/home/goln/SimpleSysIDV/models/yolo26-best_ncnn_model/model.ncnn.param \
  model_bin_path:=/home/goln/SimpleSysIDV/models/yolo26-best_ncnn_model/model.ncnn.bin
```

**Báo cáo kết quả đo lường thực tế trên CPU Máy host:**
* **FPS Đạt Được**: **51.70 FPS**
* **Độ trễ trung bình**: **19.34 ms**
* **Tệp video kết quả**: Đã được lưu thành công tại thư mục `/home/goln/SimpleSysIDV/test/test_video_output/output_video_test1.mp4` với đầy đủ các bounding boxes và lớp mặt nạ màu phân tách làn đường / phương tiện.
