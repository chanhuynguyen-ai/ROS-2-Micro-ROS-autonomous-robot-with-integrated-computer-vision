# Applied Optimization Methods

Tài liệu này ghi lại các phương pháp tối ưu hóa đã được áp dụng trực tiếp trong codebase hiện tại của `SimpleSysIDV`.

Mục tiêu của tài liệu là trả lời ngắn gọn ba câu hỏi:

1. Đã tối ưu gì
2. Tối ưu ở đâu
3. Tác động kỳ vọng là gì

## 1. Tối ưu build cho ARM64 Release

File: [ros2_ws/src/avs_perception/CMakeLists.txt](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/CMakeLists.txt:7)

Các tối ưu đã áp dụng:

- mặc định build ở chế độ `Release` nếu không chỉ định
- bật `-O3`
- bật `-march=armv8.2-a`
- bật `-mtune=cortex-a76`
- bật `-ffast-math`
- bật `-funroll-loops`

Ý nghĩa:

- tận dụng tốt hơn CPU Cortex-A76 trên Raspberry Pi 5
- giảm chi phí tính toán cho các vòng lặp số học nặng
- tránh việc build mặc định rơi về `Debug` và mất tối ưu

## 2. Tối ưu cấu hình NCNN cho CPU inference

File: [ros2_ws/src/avs_perception/src/yolo26_seg.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/yolo26_seg.cpp:6)

Các tùy chọn đã bật:

- `use_fp16_packed = true`
- `use_fp16_storage = true`
- `use_fp16_arithmetic = true`
- `use_packing_layout = true`
- `use_int8_inference = true`
- `use_vulkan_compute = false`

Ý nghĩa:

- tận dụng đường chạy FP16 và packed layout của NCNN để giảm băng thông bộ nhớ và tăng tốc CPU inference
- giữ inference trên CPU thay vì Vulkan để tránh thay đổi kiến trúc runtime hiện tại
- dùng model INT8 ở runtime để giảm chi phí suy luận

Ghi chú:

- Docker hiện build NCNN với `-DNCNN_ARM_NEON=ON` tại [docker/Dockerfile](/home/goln/SimpleSysIDV/docker/Dockerfile:26), nên backend CPU đã có hỗ trợ NEON.

## 3. Cho phép cấu hình số luồng NCNN theo runtime

Files:

- [ros2_ws/src/avs_perception/src/yolo26_seg.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/yolo26_seg.cpp:21)
- [ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp:18)

Các thay đổi đã áp dụng:

- thêm `set_num_threads(int)` trong `YOLO26Seg`
- thêm ROS2 parameter `num_threads`
- áp dụng `num_threads` khi node khởi tạo
- cập nhật lại `num_threads` trong `image_callback()`

Ý nghĩa:

- không còn cố định số luồng trong code
- có thể benchmark `1`, `2`, `3`, `4` luồng dưới tải ROS2 thực tế
- giúp cân bằng giữa tốc độ inference và headroom cho các node khác

## 4. Giảm overhead logging theo từng frame

File: [ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp:60)

Các thay đổi đã áp dụng:

- log nhận frame chuyển sang `RCLCPP_DEBUG`
- thêm `RCLCPP_INFO_THROTTLE(..., 2000, ...)` cho thống kê profiling

Ý nghĩa:

- giảm chi phí log ở pipeline thời gian thực
- tránh spam `INFO` mỗi frame
- vẫn giữ được khả năng quan sát hiệu năng theo chu kỳ

## 5. Bổ sung profiling theo từng giai đoạn

File: [ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp:70)

Các mốc thời gian đã được đo:

- `cv_bridge` conversion
- `NCNN inference`
- `post-processing`
- `findContours`
- `publish`
- `full latency`

Ý nghĩa:

- tách được chi phí của từng phần trong pipeline
- tránh tối ưu mù
- giúp xác định bottleneck thật nằm ở inference, contour extraction hay publish

Ghi chú:

- telemetry JSON hiện đã ghi `full_latency_ms` và `fps` từ đường thời gian thực của pipeline trước bước publish
- debug log bổ sung thêm `total_latency_with_publish` để theo dõi toàn bộ thời gian đến sau publish

## 6. Tối ưu giải mã mask theo ROI thay vì toàn bộ prototype map

File: [ros2_ws/src/avs_perception/src/yolo26_seg.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/yolo26_seg.cpp:141)

Cách làm cũ:

- tạo full mask `80x80`
- tính linear combination trên toàn bộ prototype grid
- chạy sigmoid trên toàn bộ grid
- sau đó mới crop theo bounding box

Cách làm hiện tại:

1. ánh xạ bounding box từ image space sang prototype space
2. clamp ROI trong phạm vi hợp lệ
3. chỉ cấp phát `cropped_mask` theo kích thước ROI
4. chỉ tính linear combination trong ROI đó
5. chỉ chạy sigmoid trong ROI đó
6. resize ROI trở lại đúng kích thước bounding box

Ý nghĩa:

- giảm số phép tính cho mỗi detection
- đặc biệt có lợi khi object nhỏ hoặc số lượng detection lớn
- giữ nguyên semantics per-pixel của mask cuối cùng trong ROI

## 7. Tách capture khỏi publish trong video publisher

File: [ros2_ws/src/avs_perception/src/video_publisher_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/video_publisher_node.cpp:120)

Các thay đổi đã áp dụng:

- tạo `capture_thread_` chạy nền
- `cap_.read()` không còn nằm trực tiếp trong timer callback
- lưu frame mới nhất vào `latest_frame_`
- dùng buffer kiểu overwrite, kích thước logic bằng 1 frame
- timer chỉ publish frame mới nhất khi có `new_frame_available_`

Ý nghĩa:

- tách thao tác đọc nguồn video/camera khỏi nhịp publish ROS2
- tránh việc timer callback bị block bởi `cap_.read()`
- giảm backlog frame cũ
- phù hợp hơn cho pipeline cần frame mới nhất thay vì xử lý tuần tự toàn bộ frame

## 8. Đồng bộ producer-consumer cho video file

File: [ros2_ws/src/avs_perception/src/video_publisher_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/video_publisher_node.cpp:148)

Các thay đổi đã áp dụng:

- với video file, thread capture chờ đến khi frame hiện tại được consume trước khi decode frame tiếp theo
- với camera, thread chỉ sleep ngắn để tránh chiếm CPU

Ý nghĩa:

- tránh decode vượt quá xa so với tốc độ publish khi nguồn là file
- giảm lãng phí CPU cho phần mô phỏng phát lại video

## 9. Tối ưu mở camera qua V4L2 và MJPEG

File: [ros2_ws/src/avs_perception/src/video_publisher_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/video_publisher_node.cpp:162)

Các thay đổi đã áp dụng:

- nhận diện đúng device path `/dev/...`
- resolve symlink camera
- chỉ parse index cho thiết bị dạng `/dev/videoN`
- mở camera bằng backend `cv::CAP_V4L2`
- yêu cầu pixel format `MJPG`
- set width, height, fps từ parameter
- log lại negotiated format thực tế của thiết bị

Ý nghĩa:

- giảm rủi ro mở nhầm device
- tận dụng đường capture phù hợp hơn cho Linux camera stack
- MJPEG thường giảm tải truyền dữ liệu từ camera và có thể giúp hiệu năng tốt hơn tùy thiết bị

## 10. Hỗ trợ runtime reconfiguration cho nguồn phát video/camera

File: [ros2_ws/src/avs_perception/src/video_publisher_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/video_publisher_node.cpp:266)

Các thay đổi đã áp dụng:

- hỗ trợ đổi `video_path` khi node đang chạy
- hỗ trợ cập nhật `loop`
- hỗ trợ cập nhật `camera_width`
- hỗ trợ cập nhật `camera_height`
- hỗ trợ cập nhật `camera_fps`
- hỗ trợ cập nhật `fps_override`
- reset timer publish khi FPS thay đổi

Ý nghĩa:

- giảm nhu cầu restart node trong quá trình tuning
- thuận tiện hơn khi benchmark hoặc đổi nguồn camera/video trong lúc chạy

## 11. Dùng cấu trúc "latest frame wins"

File: [ros2_ws/src/avs_perception/src/video_publisher_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/video_publisher_node.cpp:141)

Thay vì queue nhiều frame:

- node chỉ giữ frame mới nhất
- frame cũ có thể bị overwrite

Ý nghĩa:

- giảm độ trễ tích lũy
- phù hợp với hệ thống điều khiển thời gian thực, nơi freshness của frame quan trọng hơn việc giữ đủ mọi frame

Tradeoff:

- có thể bỏ qua một số frame trung gian khi producer nhanh hơn consumer

## Tóm tắt ngắn

Các tối ưu đã áp dụng tập trung vào 4 nhóm chính:

1. tối ưu compile và backend CPU cho ARM64
2. giảm chi phí post-processing trong inference
3. thêm khả năng đo đạc và tuning runtime
4. giảm độ trễ và blocking trong pipeline publish video/camera

Những thay đổi này chưa phải là toàn bộ không gian tối ưu, nhưng chúng đã chuyển hệ thống từ trạng thái "chạy được" sang trạng thái "có thể đo, tinh chỉnh, và giảm latency thực tế" một cách có kiểm soát.
