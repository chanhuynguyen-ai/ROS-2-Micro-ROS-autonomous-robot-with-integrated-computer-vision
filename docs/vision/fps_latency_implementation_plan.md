# Kế Hoạch Triển Khai Đo FPS Và Latency

## 1. Mục Tiêu

Triển khai lại cơ chế đo FPS và latency trong `ncnn_inference_node.cpp` để:

- không còn dùng một biến `fps` duy nhất đại diện cho toàn hệ thống
- tách rõ từng loại FPS theo ý nghĩa sử dụng
- tách rõ từng khoảng latency theo từng công đoạn xử lý
- làm cho log và telemetry phản ánh đúng ý nghĩa kỹ thuật của từng tham số

Phạm vi triển khai dựa trên tài liệu [fps_latency_measurement.md](./fps_latency_measurement.md).

## 2. Phạm Vi Sửa Đổi

File chính cần sửa:

- [ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp:60)

Phạm vi logic:

- đổi cách đo FPS
- đổi tên các metric latency cho rõ nghĩa
- thêm metric mới còn thiếu
- cập nhật JSON telemetry output
- cập nhật log `RCLCPP_DEBUG` và `RCLCPP_INFO_THROTTLE`

## 3. Kết Quả Mong Muốn

Sau khi triển khai, node cần xuất được các chỉ số sau:

- `input_fps`
- `processing_fps`
- `publish_fps`
- `bridge_latency_ms`
- `inference_latency_ms`
- `post_processing_latency_ms`
- `contour_time_ms`
- `json_finalize_latency_ms`
- `publish_latency_ms`
- `node_processing_latency_ms`
- `node_total_latency_ms`
- `input_age_ms`
- `output_age_ms`

## 4. Nguyên Tắc Triển Khai

### 4.1 Tách Hai Nhóm Clock

Sử dụng hai nhóm thời gian khác nhau:

- `std::chrono::steady_clock` cho mọi duration nội bộ trong node
- ROS time tương thích với `msg->header.stamp` cho các metric dạng age

Mục tiêu:

- duration nội bộ không bị ảnh hưởng bởi chỉnh giờ hệ thống
- metric `input_age_ms` và `output_age_ms` dùng cùng hệ thời gian với message stamp

### 4.2 Không Giữ Lại Cách Gọi Mơ Hồ

Tránh các tên quá chung như:

- `fps`
- `full_latency`
- `total_latency_with_publish`

Thay bằng các tên phản ánh đúng phạm vi đo.

### 4.3 Giữ Tương Thích Từng Bước

Triển khai theo từng pha nhỏ để:

- dễ kiểm chứng
- tránh làm hỏng luồng publish hiện tại
- có thể so sánh metric cũ và mới trong thời gian ngắn nếu cần

## 5. Kế Hoạch Triển Khai Theo Pha

## 5.1 Pha 1: Chuẩn Hóa Mốc Thời Gian Nội Bộ

Mục tiêu:

- thay các mốc đo duration nội bộ sang `std::chrono::steady_clock`
- giữ nguyên flow xử lý hiện tại

Việc cần làm:

- đổi `start_time`, `bridge_end`, `inference_end`, `post_start`, `post_end`, `publish_start`, `publish_end` sang `steady_clock`
- giữ nguyên công thức duration hiện tại để tránh thay đổi logic lớn ngay lập tức

Kết quả sau pha này:

- hệ thống vẫn chạy như cũ
- duration nội bộ ổn định hơn về mặt đo lường

## 5.2 Pha 2: Đổi Tên Metric Latency Hiện Có

Mục tiêu:

- làm cho tên metric khớp đúng với tài liệu đo lường

Việc cần làm:

- đổi `bridge_latency` thành `bridge_latency_ms`
- đổi `inference_latency` thành `inference_latency_ms`
- đổi `post_latency` thành `post_processing_latency_ms`
- đổi `contour_time` thành `contour_time_ms`
- đổi `publish_latency` thành `publish_latency_ms`
- đổi `full_latency` thành `node_processing_latency_ms`
- đổi `total_latency_with_publish` thành `node_total_latency_ms`

Kết quả sau pha này:

- code dễ đọc hơn
- log và telemetry bớt gây hiểu sai

## 5.3 Pha 3: Thêm `json_finalize_latency_ms`

Mục tiêu:

- tách riêng chi phí ghép `json_str` cuối cùng

Việc cần làm:

- thêm mốc `json_start`
- đo khoảng `post_end -> json_end`
- xuất metric `json_finalize_latency_ms`

Lưu ý:

- metric này không nên gộp vào `post_processing_latency_ms`
- mục tiêu là nhìn rõ chi phí build payload cuối

Kết quả sau pha này:

- biết rõ phần hậu xử lý và phần ghép JSON đang tốn bao nhiêu riêng biệt

## 5.4 Pha 4: Thêm `input_age_ms` Và `output_age_ms`

Mục tiêu:

- đo độ tuổi của frame khi vào node và khi node phát xong output

Việc cần làm:

- lấy thời gian ROS tại đầu callback
- lấy thời gian ROS tại sau `publish()`
- tính:
  - `input_age_ms = callback_start_ros - msg->header.stamp`
  - `output_age_ms = publish_end_ros - msg->header.stamp`

Điều kiện cần xác nhận:

- `msg->header.stamp` thực sự do camera hoặc driver đóng dấu hợp lệ

Kết quả sau pha này:

- có metric gần đúng cho camera-to-perception-output latency
- có thể phát hiện backlog nếu age tăng dần

## 5.5 Pha 5: Thêm `input_fps`

Mục tiêu:

- đo tốc độ frame đi vào node

Việc cần làm:

- lưu `previous_msg_stamp`
- mỗi frame tính:
  - `input_frame_period = current_msg_stamp - previous_msg_stamp`
  - `input_fps = 1 / input_frame_period`

Fallback nếu timestamp nguồn không tin cậy:

- dùng thời điểm callback bắt đầu giữa hai frame liên tiếp

Kết quả sau pha này:

- biết camera hoặc upstream source đang cấp dữ liệu ở tốc độ bao nhiêu

## 5.6 Pha 6: Thêm `processing_fps`

Mục tiêu:

- đo số frame xử lý hoàn tất trong một cửa sổ trượt 1 giây

Việc cần làm:

- chọn cấu trúc lưu timestamp các callback hoàn tất
- mỗi khi xử lý xong một frame:
  - thêm timestamp hiện tại
  - loại bỏ mọi timestamp cũ hơn 1 giây
  - số phần tử còn lại là `processing_fps`

Gợi ý implementation:

- dùng `std::deque<steady_clock::time_point>`

Kết quả sau pha này:

- có FPS thực tế của perception node theo năng lực xử lý

## 5.7 Pha 7: Thêm `publish_fps`

Mục tiêu:

- đo số message publish hoàn tất trong một cửa sổ trượt 1 giây

Việc cần làm:

- chọn cấu trúc lưu timestamp các lần publish hoàn tất
- mỗi lần `publish()` return:
  - thêm timestamp hiện tại
  - loại bỏ timestamp cũ hơn 1 giây
  - số phần tử còn lại là `publish_fps`

Kết quả sau pha này:

- biết output telemetry thực sự được phát ra với tốc độ bao nhiêu

## 5.8 Pha 8: Cập Nhật JSON Telemetry

Mục tiêu:

- thay các field cũ bằng bộ metric mới rõ nghĩa hơn

Việc cần làm:

- bỏ hoặc giảm ưu tiên field `fps`
- bỏ hoặc đổi tên field `full_latency_ms`
- thêm đầy đủ các field metric mới

Đề xuất field:

- `input_fps`
- `processing_fps`
- `publish_fps`
- `bridge_latency_ms`
- `inference_latency_ms`
- `post_processing_latency_ms`
- `contour_time_ms`
- `json_finalize_latency_ms`
- `publish_latency_ms`
- `node_processing_latency_ms`
- `node_total_latency_ms`
- `input_age_ms`
- `output_age_ms`

Kết quả sau pha này:

- dashboard và consumer nhận được dữ liệu đúng nghĩa hơn

## 5.9 Pha 9: Cập Nhật Log Runtime

Mục tiêu:

- làm log dễ đọc và đúng ý nghĩa

Việc cần làm:

- cập nhật `RCLCPP_DEBUG` để hiển thị các metric nội bộ quan trọng
- cập nhật `RCLCPP_INFO_THROTTLE` để hiển thị:
  - `input_fps`
  - `processing_fps`
  - `publish_fps`
  - `node_total_latency_ms`
  - `inference_latency_ms`
  - `output_age_ms`

Kết quả sau pha này:

- đọc log là hiểu được node đang chậm ở đầu vào, ở xử lý hay ở publish

## 6. Thay Đổi Cấu Trúc Dữ Liệu Trong Class

Để hỗ trợ metric mới, class cần thêm state:

- `previous_msg_stamp`
- cờ đánh dấu frame đầu tiên đã có stamp trước đó hay chưa
- `std::deque` lưu thời điểm callback hoàn tất
- `std::deque` lưu thời điểm publish hoàn tất

Có thể cần thêm helper function:

- hàm cắt cửa sổ 1 giây cho deque
- hàm chuyển duration sang ms
- hàm tính FPS từ deque

## 7. Tiêu Chí Hoàn Thành

Implementation được xem là hoàn thành khi:

- không còn dùng `fps = 1000 / latency` làm FPS chính trong telemetry
- metric mới xuất hiện đầy đủ trong JSON
- log runtime dùng tên metric mới
- các mốc đo khớp với tài liệu `fps_latency_measurement.md`
- code build thành công
- node chạy được mà không làm thay đổi luồng inference chính

## 8. Kiểm Thử Sau Triển Khai

## 8.1 Kiểm Thử Logic Metric

Cần xác nhận:

- `bridge_latency_ms >= 0`
- `inference_latency_ms >= 0`
- `post_processing_latency_ms >= 0`
- `json_finalize_latency_ms >= 0`
- `publish_latency_ms >= 0`
- `node_total_latency_ms >= node_processing_latency_ms`
- `output_age_ms >= input_age_ms` trong điều kiện timestamp hợp lệ

## 8.2 Kiểm Thử Tính Nhất Quán FPS

Cần quan sát:

- `input_fps` gần với FPS camera khi pipeline không backlog
- `processing_fps` giảm khi inference nặng hơn
- `publish_fps` không vượt quá `processing_fps`

## 8.3 Kiểm Thử Tình Huống Backlog

Tạo tình huống tải cao để quan sát:

- `input_fps` giữ ổn định
- `processing_fps` giảm
- `input_age_ms` tăng dần
- `output_age_ms` tăng dần

Điều này xác nhận metric mới phản ánh đúng tình trạng nghẽn.

## 9. Rủi Ro Và Lưu Ý

### 9.1 Độ Tin Cậy Của `msg->header.stamp`

Nếu stamp đầu vào không phải timestamp capture thật, thì:

- `input_age_ms` và `output_age_ms` vẫn có ích
- nhưng không nên diễn giải chúng là latency quang học tuyệt đối của camera

### 9.2 Chi Phí Tính Metric

Việc thêm metric không nên làm tăng tải đáng kể.

Nguyên tắc:

- chỉ dùng vài timestamp và deque nhỏ
- không thêm cấp phát lớn trong callback
- không thêm serialization dư thừa

### 9.3 Tương Thích Dashboard

Nếu dashboard đang đọc field cũ như:

- `fps`
- `full_latency_ms`

thì cần có kế hoạch cập nhật consumer tương ứng hoặc duy trì field cũ tạm thời trong giai đoạn chuyển tiếp.

## 10. Thứ Tự Thực Thi Khuyến Nghị

Thứ tự triển khai an toàn:

1. đổi clock nội bộ sang `steady_clock`
2. đổi tên metric latency hiện có
3. thêm `json_finalize_latency_ms`
4. thêm `input_age_ms` và `output_age_ms`
5. thêm `input_fps`
6. thêm `processing_fps`
7. thêm `publish_fps`
8. cập nhật JSON telemetry
9. cập nhật log
10. kiểm thử bằng dữ liệu chạy thật

## 11. Tóm Tắt

Mục tiêu của kế hoạch này là biến hệ đo hiện tại từ:

- một bộ metric chủ yếu phục vụ local profiling

thành:

- một bộ metric vừa phục vụ profiling nội bộ
- vừa phản ánh đúng hơn tốc độ input, năng lực xử lý, tốc độ publish và độ tuổi thực tế của frame
