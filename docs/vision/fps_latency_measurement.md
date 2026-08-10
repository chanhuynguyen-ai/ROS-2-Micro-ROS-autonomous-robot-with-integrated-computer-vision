# Thay Đổi Cách Đo FPS Và Ý Nghĩa Các Tham Số

## 1. Mục Tiêu

Tài liệu này chỉ tập trung vào:

- thay đổi cách đo FPS
- ý nghĩa của từng tham số đo
- mỗi tham số được đo từ công đoạn nào đến công đoạn nào
- ý nghĩa của từng công đoạn trong pipeline

Phần mô tả dựa trên flow hiện tại của [ncnn_inference_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp:60).

## 2. Pipeline Xử Lý Hiện Tại

Một frame hiện đi qua các công đoạn chính sau:

1. Callback nhận `sensor_msgs::msg::Image`
2. Chuyển ảnh ROS2 sang `cv::Mat` bằng `cv_bridge`
3. Chạy `yolo_->detect()`
4. Hậu xử lý kết quả:
   - đếm object theo class
   - trích contour từ mask
   - build `detections_json`
   - build `objects_json`
5. Ghép `json_str` cuối cùng
6. Tạo `telemetry_msg`
7. Gọi `telemetry_pub_->publish()`

## 3. Vấn Đề Với Cách Đo FPS Cũ

Hiện tại mã đang dùng:

```text
fps = 1000.0 / full_latency
fps_with_publish = 1000.0 / total_latency_with_publish
```

Hai giá trị này chỉ phản ánh:

- thời gian xử lý của một frame đơn lẻ
- khả năng xử lý tức thời của callback

Hai giá trị này không phản ánh chính xác:

- FPS thực tế của luồng ảnh đi vào node
- FPS thực tế của số frame node xử lý xong trong 1 giây
- FPS thực tế của số message được publish ra trong 1 giây

Vì vậy cần đổi cách hiểu:

- `1000 / latency` chỉ là tốc độ xử lý tức thời theo một frame
- FPS thực tế phải được đo bằng số frame hoàn tất trong một khoảng thời gian cố định, ví dụ 1 giây

## 4. Cách Đo FPS Mới Đề Xuất

Thay vì chỉ dùng một biến `fps`, nên tách thành 3 loại FPS.

### 4.1 `input_fps`

`input_fps` đo tốc độ frame đi vào node.

Khoảng đo:

- từ `msg->header.stamp` của frame trước
- đến `msg->header.stamp` của frame hiện tại

Công thức:

```text
input_frame_period = current_msg_stamp - previous_msg_stamp
input_fps = 1 / input_frame_period
```

Ý nghĩa:

- cho biết camera hoặc upstream source đang cấp dữ liệu với tốc độ bao nhiêu
- giúp phát hiện camera tụt FPS hoặc upstream bị nghẽn

Nếu `msg->header.stamp` không đáng tin, có thể thay bằng:

- thời điểm bắt đầu callback frame trước
- thời điểm bắt đầu callback frame hiện tại

### 4.2 `processing_fps`

`processing_fps` đo số frame node xử lý xong trong một cửa sổ thời gian, ví dụ 1 giây.

Khoảng đo:

- đếm số callback hoàn tất trong 1 giây

Công thức:

```text
processing_fps = completed_callbacks_in_last_1s
```

Ý nghĩa:

- cho biết năng lực xử lý thực tế của node
- đây là FPS nên dùng khi đánh giá perception node có theo kịp luồng ảnh hay không

### 4.3 `publish_fps`

`publish_fps` đo số message telemetry được publish thành công từ phía publisher trong 1 giây.

Khoảng đo:

- đếm số lần `telemetry_pub_->publish()` được gọi xong trong 1 giây

Công thức:

```text
publish_fps = published_messages_in_last_1s
```

Ý nghĩa:

- cho biết tốc độ node thật sự phát kết quả ra ngoài
- hữu ích khi cần biết perception output có ổn định hay không

## 5. Các Tham Số Đo Và Ý Nghĩa

## 5.1 `bridge_latency_ms`

Khoảng đo:

- từ lúc bắt đầu callback
- đến lúc kết thúc `cv_bridge::toCvCopy()`

Tức là:

```text
callback_start -> bridge_end
```

Ý nghĩa công đoạn:

- callback start: node đã nhận frame để bắt đầu xử lý
- `cv_bridge`: chuyển từ message ROS2 sang dữ liệu ảnh OpenCV để inference dùng được

Ý nghĩa tham số:

- đo chi phí nhận và chuyển đổi dữ liệu ảnh đầu vào
- giúp biết bottleneck có nằm ở bước copy/chuyển format ảnh hay không

## 5.2 `inference_latency_ms`

Khoảng đo:

- từ lúc kết thúc `cv_bridge`
- đến lúc kết thúc `yolo_->detect()`

Tức là:

```text
bridge_end -> inference_end
```

Ý nghĩa công đoạn:

- đây là bước chạy model NCNN để sinh ra detection và segmentation mask

Ý nghĩa tham số:

- đo chi phí inference thuần của model
- đây là chỉ số quan trọng nhất để tối ưu model và số thread

## 5.3 `post_processing_latency_ms`

Khoảng đo:

- từ lúc bắt đầu hậu xử lý
- đến lúc kết thúc phần build dữ liệu trung gian sau inference

Tức là:

```text
post_start -> post_end
```

Ý nghĩa công đoạn:

- đếm object theo class
- duyệt danh sách object
- chạy `cv::findContours()` trên từng mask
- build `detections_json`
- build `objects_json`

Ý nghĩa tham số:

- đo chi phí xử lý kết quả model trước khi đóng gói output
- giúp biết phần contour hoặc vòng lặp JSON có đang quá nặng hay không

## 5.4 `contour_time_ms`

Khoảng đo:

- cộng dồn thời gian của từng lần gọi `cv::findContours()`

Tức là:

```text
sum(contour_start -> contour_end for each object)
```

Ý nghĩa công đoạn:

- trích polygon từ segmentation mask của từng object

Ý nghĩa tham số:

- là chỉ số con bên trong hậu xử lý
- dùng để xác định riêng chi phí contour extraction

## 5.5 `json_finalize_latency_ms`

Tham số này nên được thêm mới.

Khoảng đo:

- từ lúc kết thúc `post_end`
- đến lúc ghép xong `json_str`

Tức là:

```text
post_end -> json_end
```

Ý nghĩa công đoạn:

- ghép toàn bộ chuỗi JSON cuối cùng để chuẩn bị publish

Ý nghĩa tham số:

- tách riêng chi phí build chuỗi output cuối
- tránh gộp nhầm phần này vào `post_processing_latency_ms`

## 5.6 `publish_latency_ms`

Khoảng đo:

- từ trước khi gọi `telemetry_pub_->publish()`
- đến sau khi hàm `publish()` trả về

Tức là:

```text
publish_start -> publish_end
```

Ý nghĩa công đoạn:

- tạo ROS2 publish request ở phía publisher
- đẩy message vào tầng publish của ROS2

Ý nghĩa tham số:

- đo chi phí publish cục bộ ở phía node hiện tại
- không phải thời gian dashboard nhận được dữ liệu

## 5.7 `node_processing_latency_ms`

Tham số này nên thay cho `full_latency` hiện tại.

Khoảng đo:

- từ lúc bắt đầu callback
- đến lúc kết thúc hậu xử lý

Tức là:

```text
callback_start -> post_end
```

Ý nghĩa công đoạn:

- bao gồm nhận frame trong callback, convert ảnh, inference, hậu xử lý
- chưa gồm ghép JSON cuối, tạo message, publish

Ý nghĩa tham số:

- đo thời gian xử lý nội bộ chính của perception node
- phù hợp để profile performance của thuật toán

## 5.8 `node_total_latency_ms`

Tham số này nên thay cho `total_latency_with_publish`.

Khoảng đo:

- từ lúc bắt đầu callback
- đến sau khi `publish()` trả về

Tức là:

```text
callback_start -> publish_end
```

Ý nghĩa công đoạn:

- bao gồm toàn bộ pipeline cục bộ trong node
- gồm convert ảnh, inference, hậu xử lý, ghép JSON, tạo message và publish

Ý nghĩa tham số:

- phản ánh tổng chi phí xử lý của node cho một frame
- vẫn chưa phải end-to-end latency của toàn hệ thống

## 5.9 `input_age_ms`

Tham số này nên được thêm mới.

Khoảng đo:

- từ `msg->header.stamp`
- đến thời điểm callback bắt đầu

Tức là:

```text
msg->header.stamp -> callback_start_ros
```

Ý nghĩa công đoạn:

- đây là khoảng thời gian frame đã tồn tại trước khi node bắt đầu xử lý nó

Ý nghĩa tham số:

- cho biết frame vào node còn "mới" hay đã "già"
- giúp phát hiện queue, buffering hoặc backlog trước perception node

## 5.10 `output_age_ms`

Tham số này nên được thêm mới.

Khoảng đo:

- từ `msg->header.stamp`
- đến sau khi `publish()` trả về

Tức là:

```text
msg->header.stamp -> publish_end_ros
```

Ý nghĩa công đoạn:

- đây là toàn bộ tuổi của frame tính đến khi node đã phát xong output

Ý nghĩa tham số:

- nếu timestamp đầu vào là timestamp capture thật, đây là chỉ số gần đúng nhất cho camera-to-output latency ở phía perception node

## 6. Ý Nghĩa Của Từng Công Đoạn Trong Pipeline

### 6.1 Callback Receive

Ý nghĩa:

- frame đã tới perception node
- đây là điểm bắt đầu xử lý cục bộ

### 6.2 Image Conversion

Ý nghĩa:

- chuyển dữ liệu từ ROS image message sang định dạng OpenCV dùng cho model

### 6.3 Inference

Ý nghĩa:

- model NCNN chạy suy luận để tạo detection và mask

### 6.4 Post-processing

Ý nghĩa:

- biến output thô của model thành dữ liệu có thể dùng cho hệ thống khác
- gồm contour extraction, thống kê object và chuẩn bị dữ liệu JSON trung gian

### 6.5 JSON Finalization

Ý nghĩa:

- ghép dữ liệu trung gian thành payload hoàn chỉnh để truyền ra ngoài

### 6.6 Publish

Ý nghĩa:

- phát payload sang topic ROS2
- đây là điểm kết thúc xử lý cục bộ của node

## 7. Bộ Tham Số Nên Dùng Sau Khi Chỉnh

Để rõ ràng và dễ theo dõi, nên dùng bộ tham số sau:

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

## 8. Kết Luận

Thay đổi quan trọng nhất là không dùng duy nhất `fps = 1000 / latency` để đại diện cho FPS của hệ thống nữa.

Thay vào đó:

- `input_fps` trả lời tốc độ frame đi vào
- `processing_fps` trả lời node xử lý được bao nhiêu frame mỗi giây
- `publish_fps` trả lời node phát được bao nhiêu kết quả mỗi giây

Còn các tham số latency phải được hiểu đúng theo mốc bắt đầu và mốc kết thúc của từng công đoạn, thay vì gộp chung thành một con số "latency" duy nhất.
