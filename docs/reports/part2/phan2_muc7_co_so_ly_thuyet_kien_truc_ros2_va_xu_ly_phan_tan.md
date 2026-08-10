# PHẦN II.7. Cơ sở lý thuyết kiến trúc ROS2 và xử lý phân tán

## 1. Mở đầu

Hệ thống Computer Vision của AVS không được triển khai như một chương trình đơn khối duy nhất, mà được tổ chức thành nhiều khối chức năng độc lập chạy trên ROS2. Cách tổ chức này phản ánh đúng bản chất của bài toán: perception thời gian thực không chỉ gồm suy luận mô hình, mà còn bao gồm thu nhận ảnh, biến đổi hình học, trích xuất đại lượng điều hướng và công cụ giám sát.

Trong repo hiện tại, pipeline được chia thành các node chính:

- `video_publisher_node`
- `ncnn_inference_node`
- `ipm_transform_node`
- `control_node`

cùng với một số thành phần phụ trợ như:

- `video_test_node`
- web dashboard

Các node này không gọi hàm trực tiếp của nhau như trong một thư viện nội bộ, mà trao đổi thông qua cơ chế publish/subscribe của ROS2. Vì vậy, để hiểu đúng kiến trúc triển khai của hệ thống, cần nắm rõ ba lớp khái niệm:

- `node`
- `topic`
- `message`

và cách chúng tạo thành một pipeline xử lý phân tán.

## 2. Vì sao hệ thống cần kiến trúc ROS2

### 2.1. Tính mô-đun của bài toán

Pipeline AVS có các tầng xử lý rất khác nhau về bản chất:

- tầng nguồn ảnh: đọc camera hoặc video file
- tầng perception: suy luận segmentation bằng NCNN
- tầng IPM/homography: đổi tọa độ ảnh sang hệ tọa độ thực
- tầng hậu xử lý hình học: chọn lane, suy ra `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`

Nếu dồn tất cả vào một tiến trình lớn, hệ thống sẽ gặp nhiều vấn đề:

- khó debug từng phần
- khó thay thế một khối mà không ảnh hưởng khối khác
- khó đo độ trễ riêng từng tầng
- khó tái sử dụng dữ liệu trung gian cho dashboard hoặc node khác

ROS2 giải quyết điều này bằng cách cho phép chia hệ thống thành các thực thể độc lập nhưng vẫn giao tiếp theo chuẩn chung.

### 2.2. Yêu cầu thời gian thực mềm

Perception của AVS là hệ thống thời gian thực mềm (`soft real-time`), nghĩa là:

- cần phản hồi đủ nhanh để phục vụ điều hướng
- nhưng không yêu cầu deadline cứng tuyệt đối như điều khiển mức firmware

Trong bối cảnh đó, ROS2 phù hợp vì:

- hỗ trợ xử lý bất đồng bộ qua callback
- cho phép nhiều node chạy song song
- có thể triển khai phân tán theo tiến trình, theo container hoặc theo máy
- dễ chèn logging, profiling, replay và monitoring

### 2.3. Mô hình giao tiếp dữ liệu chảy

Pipeline perception có bản chất của một **dataflow system**: mỗi tầng nhận dữ liệu từ tầng trước, xử lý, rồi phát kết quả sang tầng sau. ROS2 tự nhiên phù hợp với mô hình này vì topic đóng vai trò như các kênh dữ liệu một chiều giữa các khối.

Về mặt khái niệm, toàn pipeline hiện tại có thể viết gọn:

$$
\texttt{/camera/image\_raw}
\rightarrow
\texttt{/avs/telemetry}
\rightarrow
\texttt{/avs/telemetry\_realworld}
\rightarrow
\texttt{/avs/control\_error}
$$

Đây là một chuỗi biến đổi trạng thái tuần tự nhưng tách rời ở mức tiến trình.

## 3. Khái niệm node trong ROS2

### 3.1. Định nghĩa

Trong ROS2, `node` là đơn vị thực thi logic cơ bản. Một node thường:

- có tên riêng
- có thể publish dữ liệu
- có thể subscribe dữ liệu
- có tham số cấu hình
- có logger và clock riêng

Về mặt triển khai C++, các node trong repo đều kế thừa từ:

```cpp
rclcpp::Node
```

Ví dụ:

- `Node("video_publisher_node")`
- `Node("ncnn_inference_node")`
- `Node("ipm_transform_node")`
- `Node("control_node")`

### 3.2. Node như một tiến trình chức năng

Trong ngữ cảnh báo cáo này, có thể xem node là một **tiến trình chức năng** đảm nhiệm một vai trò chuyên biệt trong pipeline.

Ví dụ:

- `video_publisher_node` chỉ chịu trách nhiệm sinh luồng ảnh đầu vào
- `ncnn_inference_node` chỉ chịu trách nhiệm segmentation và telemetry pixel-space
- `ipm_transform_node` chỉ chịu trách nhiệm homography và geometry world-space
- `control_node` chỉ chịu trách nhiệm chọn lane và publish sai số hình học đầu ra

Việc tách vai trò rõ ràng như vậy có ý nghĩa kiến trúc quan trọng: mỗi node là một điểm biên logic có đầu vào và đầu ra xác định.

### 3.3. Node và callback

Node ROS2 vận hành chủ yếu theo cơ chế callback. Khi có message mới tới từ topic đã subscribe, ROS2 đánh thức callback tương ứng để xử lý.

Ví dụ trong repo:

- ảnh mới trên `/camera/image_raw` kích hoạt `image_callback()` của `ncnn_inference_node`
- telemetry mới trên `/avs/telemetry` kích hoạt `telemetry_callback()` của `ipm_transform_node`
- telemetry world-space trên `/avs/telemetry_realworld` kích hoạt `telemetry_callback()` của `control_node`

Mô hình này làm cho pipeline tự nhiên mang tính bất đồng bộ, thay vì phải xây một vòng lặp polling thủ công.

## 4. Khái niệm topic trong ROS2

### 4.1. Định nghĩa

`Topic` là kênh truyền dữ liệu theo mô hình publish/subscribe. Một hoặc nhiều publisher có thể phát message lên topic, và một hoặc nhiều subscriber có thể nhận message từ topic đó.

Một topic trong ROS2 được xác định bởi:

- tên topic
- kiểu message

Ví dụ:

- `/camera/image_raw`
- `/avs/telemetry`
- `/avs/telemetry_realworld`
- `/avs/control_error`

### 4.2. Topic như một liên kết lỏng

Điểm mạnh quan trọng của topic là **liên kết lỏng** (`loose coupling`). Publisher không cần biết cụ thể subscriber là ai, và subscriber cũng không cần biết publisher nằm trong file hay tiến trình nào.

Điều này mang lại nhiều lợi ích:

- dễ thêm dashboard subscribe để quan sát mà không sửa pipeline chính
- dễ thay `video_publisher_node` bằng camera thật hoặc ROS bag replay
- dễ thay backend suy luận trong `ncnn_inference_node` mà không đổi giao diện logic với tầng sau

Về mặt kiến trúc phần mềm, topic chính là lớp trừu tượng giao tiếp giữa các node.

### 4.3. Dòng dữ liệu trong pipeline hiện tại

Hệ thống hiện có các topic chính:

- `/camera/image_raw`
  - kiểu: `sensor_msgs/msg/Image`
  - vai trò: ảnh đầu vào cho perception

- `/camera/image_raw/compressed`
  - kiểu: `sensor_msgs/msg/CompressedImage`
  - vai trò: luồng quan sát phục vụ dashboard hoặc debug

- `/avs/telemetry`
  - kiểu: `std_msgs/msg/String`
  - vai trò: telemetry perception trong hệ pixel

- `/avs/telemetry_realworld`
  - kiểu: `std_msgs/msg/String`
  - vai trò: telemetry sau homography, waypoint và polynomial trong hệ `mm`

- `/avs/control_error`
  - kiểu: `std_msgs/msg/String`
  - vai trò: đầu ra sai số hình học cuối của phạm vi báo cáo

- `/avs/lane_state`
  - kiểu: `std_msgs/msg/String`
  - vai trò: trạng thái lane selection để debug

- `/avs/route_intent`
  - kiểu: `std_msgs/msg/String`
  - vai trò: chỉ thị chiến lược chọn lane

- `/avs/cmd`
  - kiểu: `std_msgs/msg/String`
  - vai trò: lệnh điều khiển mức logic như lane change hoặc turn

- `/odom_raw`
  - kiểu: `nav_msgs/msg/Odometry`
  - vai trò: cung cấp vận tốc cho look-ahead động

## 5. Khái niệm message trong ROS2

### 5.1. Định nghĩa

`Message` là kiểu dữ liệu trao đổi giữa các node qua topic. Mỗi message có schema rõ ràng do ROS định nghĩa hoặc do người dùng tự tạo.

Trong repo hiện tại, các kiểu message chính là:

- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CompressedImage`
- `nav_msgs/msg/Odometry`
- `std_msgs/msg/String`

### 5.2. Message ảnh

`sensor_msgs/msg/Image` chứa:

- metadata header
- kích thước ảnh
- encoding
- mảng bytes dữ liệu ảnh

Đây là message phù hợp cho tầng perception vì:

- giữ được ảnh raw đầy đủ
- dễ chuyển sang `cv::Mat` bằng `cv_bridge`
- tương thích tốt với camera, replay và xử lý OpenCV

`sensor_msgs/msg/CompressedImage` dùng cho luồng nén JPEG để:

- giảm băng thông khi chỉ cần quan sát
- phù hợp hơn cho dashboard web

### 5.3. Message odometry

`nav_msgs/msg/Odometry` là message có cấu trúc chặt hơn nhiều so với JSON string. Trong hệ thống này, nó được dùng cho dữ liệu vận tốc trên `/odom_raw`.

Lý do dùng message chuẩn ở đây rất rõ:

- dữ liệu có cấu trúc ổn định
- nhiều thành phần ROS khác đã hiểu kiểu này
- không cần schema linh hoạt động như telemetry perception

### 5.4. `std_msgs/String` như một vỏ bọc message

Điểm đáng chú ý của hệ thống AVS là nhiều topic nghiệp vụ cốt lõi không dùng custom ROS message, mà dùng:

```cpp
std_msgs::msg::String
```

bên trong chứa dữ liệu JSON.

Nghĩa là ở mức ROS, message chỉ là một chuỗi. Còn schema thực tế được áp đặt ở tầng ứng dụng, ví dụ:

- `detections`
- `objects`
- `polygons`
- `polygons_real_world`
- `waypoints`
- `polynomial`
- `epsilon_x_mm`

Đây là một quyết định kiến trúc có đánh đổi rõ ràng, sẽ được phân tích ở phần sau.

## 6. Mô hình publish/subscribe và xử lý phân tán

### 6.1. Publish/subscribe thay cho gọi hàm trực tiếp

Trong chương trình đơn khối, khối A thường gọi trực tiếp hàm của khối B:

$$
B = f(A)
$$

Nhưng trong ROS2, khối A publish một message, còn khối B subscribe topic đó. Khi đó giao tiếp trở thành:

$$
\text{Node A} \xrightarrow{\text{publish}} \text{Topic} \xrightarrow{\text{subscribe}} \text{Node B}
$$

Điều này tạo ra một tầng trung gian truyền thông. Về mặt lý thuyết, đây là một dạng **message-passing architecture**, nơi trạng thái hệ thống lan truyền qua các gói dữ liệu thay vì qua shared memory hoặc lời gọi hàm đồng bộ.

### 6.2. Tính phân tán

“Xử lý phân tán” trong bối cảnh báo cáo này không nhất thiết phải hiểu là nhiều máy khác nhau. Nó bao gồm cả:

- nhiều tiến trình trên cùng một máy
- nhiều container trong cùng một hệ thống
- hoặc mở rộng ra nhiều máy nếu cần

Repo hiện tại đã triển khai theo hướng này:

- container `avs_perception` chạy các node perception chính
- container `video_publisher` chạy node sinh ảnh
- container `web_dashboard` chạy giao diện quan sát

Chúng giao tiếp qua ROS2 trên `network_mode: host`, cùng `ROS_DOMAIN_ID=20`. Đây là một ví dụ điển hình của phân tán ở mức dịch vụ trong cùng một host.

### 6.3. Đồng bộ thời gian và dòng dữ liệu

Trong pipeline perception, dữ liệu không được đồng bộ bằng shared clock cứng, mà dựa trên thứ tự publish/subscribe và timestamp của message. `ncnn_inference_node` thậm chí còn đo:

- `input_age_ms`
- `input_fps`
- `full_latency_ms`

Điều đó cho thấy kiến trúc phân tán luôn gắn với một vấn đề quan trọng: **độ trễ truyền và độ trễ xử lý không còn bằng 0**. Khi các tầng tách rời, cần quan sát rõ:

- message đến trễ bao nhiêu
- có backlog hay không
- khối nào là bottleneck

## 7. Lý do tách node theo pipeline

### 7.1. Tách đúng theo bản chất thuật toán

Pipeline hiện tại được chia thành bốn tầng:

1. nguồn ảnh
2. suy luận perception
3. biến đổi hình học
4. suy ra sai số đầu ra

Sự tách này không phải chỉ để “code cho đẹp”, mà bám sát bản chất của bài toán. Mỗi tầng có:

- đầu vào khác nhau
- kiểu xử lý khác nhau
- tài nguyên tính toán khác nhau
- tiêu chí đánh giá khác nhau

Ví dụ:

- `video_publisher_node` thiên về I/O và camera/video timing
- `ncnn_inference_node` thiên về compute nặng CPU
- `ipm_transform_node` thiên về geometry và JSON transform
- `control_node` thiên về decision logic và chọn trajectory

### 7.2. Dễ debug

Khi pipeline bị lỗi, tách node giúp xác định vấn đề nhanh hơn:

- nếu `/camera/image_raw` tốt nhưng `/avs/telemetry` rỗng, lỗi ở perception
- nếu `/avs/telemetry` có polygon nhưng `/avs/telemetry_realworld` sai, lỗi ở homography hoặc geometry
- nếu telemetry world-space đúng nhưng `/avs/control_error` dao động, lỗi ở chọn lane hoặc hậu xử lý hình học

Đây là một lợi thế rất lớn so với thiết kế đơn khối, nơi mọi thứ bị trộn trong cùng một call stack.

### 7.3. Dễ thay thế từng khối

Khi giao diện giữa các node đã ổn định qua topic, có thể thay một node mà không cần viết lại toàn hệ thống.

Ví dụ:

- thay `video_publisher_node` bằng camera driver khác
- thay `ncnn_inference_node` bằng backend ONNX/TensorRT giả sử sau này đổi nền tảng
- thay logic trong `control_node` mà không đụng tới perception

Về lý thuyết, đây là nguyên tắc **separation of concerns** và **interface-based integration**.

### 7.4. Dễ đo latency theo tầng

Với hệ thời gian thực mềm, đo độ trễ từng khối là điều bắt buộc. Khi chia node, mỗi tầng có thể:

- tự log thời gian xử lý riêng
- publish telemetry chứa profiling
- benchmark độc lập bằng video offline hoặc replay

Repo hiện tại phản ánh rõ điều đó:

- `ncnn_inference_node` đo bridge latency, inference latency, contour time, full latency
- `video_test_node` dùng cho kiểm thử offline
- `ipm_transform_node` tách riêng để quan sát geometry downstream

Nếu gom toàn bộ vào một tiến trình, việc tách nguyên nhân latency sẽ khó hơn rất nhiều.

### 7.5. Tạo khả năng mở rộng hệ thống

Kiến trúc node-based cho phép sau này bổ sung thêm các thành phần mà không phá vỡ pipeline cũ, ví dụ:

- node lưu log ROS bag
- node visualization 3D
- node fusion với lidar hoặc IMU
- node controller thực thụ đọc `/avs/control_error`

Nghĩa là kiến trúc hiện tại không chỉ phục vụ chức năng hiện tại, mà còn mở đường cho mở rộng sau này.

## 8. Đặc điểm của việc dùng JSON qua `std_msgs/String`

### 8.1. Mô hình dữ liệu thực tế

Các topic như:

- `/avs/telemetry`
- `/avs/telemetry_realworld`
- `/avs/control_error`
- `/avs/lane_state`

đều dùng `std_msgs/String` chứa JSON thay vì custom message.

Về mặt logic, đây là một **application-level schema over a generic transport**: ROS chỉ vận chuyển chuỗi, còn ngữ nghĩa thực sự do JSON định nghĩa.

### 8.2. Ưu điểm: linh hoạt và dễ mở rộng

JSON có ưu điểm rất rõ trong giai đoạn phát triển nhanh:

- dễ thêm trường mới mà không cần sửa `.msg`
- không cần regenerate ROS interfaces
- dễ in log ra chuỗi để đọc trực tiếp
- dễ đẩy thẳng sang dashboard web hoặc backend Python

Ví dụ, trong quá trình phát triển, hệ thống có thể thêm dần các trường như:

- `polygons_real_world`
- `waypoints`
- `lookahead_d_mm`
- `trajectory_valid`
- `selected_lane_id`

mà không phải rebuild toàn bộ interface message mỗi lần thay schema.

### 8.3. Ưu điểm: phù hợp với web dashboard

Dashboard web và backend Python tự nhiên làm việc tốt với JSON. Khi telemetry đã ở dạng JSON string:

- backend chỉ cần parse chuỗi
- frontend dễ hiển thị các object động
- không cần một lớp chuyển đổi message ROS custom sang JSON riêng

Điều này đặc biệt hữu ích với hệ thống AVS vì dashboard là một phần quan trọng của quy trình calibrate, debug và quan sát runtime.

### 8.4. Ưu điểm: hỗ trợ schema mềm trong giai đoạn khám phá

Ở giai đoạn hệ thống còn thay đổi nhanh, schema dữ liệu thường chưa ổn định. JSON cho phép:

- bỏ qua trường chưa có
- thêm trường tùy tình huống
- mang dữ liệu dị thể trong cùng một object

Ví dụ, object lane có thể chứa:

- `polygons`
- `polygons_real_world`
- `waypoints`
- `polynomial`

trong khi object vehicle có thể không cần toàn bộ các trường đó. JSON thích hợp hơn message typed cứng trong giai đoạn như vậy.

### 8.5. Nhược điểm: không chặt kiểu ở mức ROS

Nhược điểm lớn nhất là ROS không còn kiểm tra chặt schema nghiệp vụ. Ở mức middleware, mọi thứ chỉ là một chuỗi. Hệ quả:

- lỗi tên trường chỉ lộ ra khi runtime
- thiếu trường hoặc sai kiểu khó phát hiện sớm
- refactor schema khó an toàn hơn custom message

Ví dụ, nếu đổi `waypoints` thành tên khác hoặc thiếu `label`, subscriber downstream chỉ phát hiện khi parse JSON, không có kiểm tra compile-time.

### 8.6. Nhược điểm: overhead serialize/deserialize

Mỗi lần node publish JSON string, hệ thống phải:

1. tạo object dữ liệu trong bộ nhớ
2. serialize thành text JSON
3. truyền qua topic như chuỗi
4. parse lại thành object ở node tiếp theo

Chi phí này bao gồm:

- CPU cho stringify và parse
- cấp phát bộ nhớ động
- payload lớn hơn so với binary typed message

Trong perception thời gian thực, overhead này không thể xem nhẹ, nhất là khi:

- số lượng polygon lớn
- mỗi polygon có nhiều điểm
- telemetry được publish mỗi frame

### 8.7. Nhược điểm: khó tối ưu băng thông hơn message typed

JSON là định dạng text tự mô tả, nên:

- dài hơn binary
- lặp lại tên trường nhiều lần
- khó tận dụng tối đa memory layout tĩnh

Nếu hệ thống về sau cần throughput cao hơn hoặc chạy trên mạng chậm hơn, custom message hoặc schema nhị phân sẽ có lợi hơn.

### 8.8. Khi nào JSON là lựa chọn đúng

Trong bối cảnh hiện tại của AVS, dùng JSON qua `std_msgs/String` vẫn là lựa chọn hợp lý vì:

- schema còn đang tiến hóa
- dashboard web là thành phần quan trọng
- cần debug nhanh và quan sát trực tiếp
- phạm vi hệ thống hiện tại chủ yếu trong một host/container cluster nhỏ

Nói cách khác, hệ thống đang tối ưu cho **tốc độ phát triển và khả năng quan sát**, chấp nhận đánh đổi một phần hiệu quả truyền thông và độ chặt kiểu.

## 9. Kiến trúc phân tán trong repo hiện tại

### 9.1. Tầng nguồn ảnh

`video_publisher_node` phát:

- `/camera/image_raw`
- `/camera/image_raw/compressed`

Node này đóng vai trò producer của toàn pipeline. Nó có cơ chế thread nền giữ `latest frame wins`, cho thấy ngay từ tầng đầu vào hệ thống đã áp dụng tư duy xử lý bất đồng bộ để ưu tiên frame mới nhất thay vì xử lý tuần tự mọi frame cũ.

### 9.2. Tầng perception

`ncnn_inference_node` subscribe:

- `/camera/image_raw`

và publish:

- `/avs/telemetry`

Đây là biên giữa:

- dữ liệu ảnh dày đặc
- và dữ liệu nhận thức cấu trúc

Từ sau node này, pipeline không còn phụ thuộc trực tiếp vào tensor ảnh, mà làm việc trên đối tượng, box, polygon và metadata profiling.

### 9.3. Tầng hình học world-space

`ipm_transform_node` subscribe:

- `/avs/telemetry`
- `/odom_raw`

và publish:

- `/avs/telemetry_realworld`

Node này là ví dụ rõ ràng cho xử lý phân tán đa nguồn (`multi-input distributed stage`): nó không chỉ nhận dữ liệu perception, mà còn nhập vận tốc từ odometry để tính look-ahead động. Điều này cho thấy một node trong ROS2 có thể hợp nhất nhiều luồng dữ liệu logic khác nhau.

### 9.4. Tầng hậu xử lý hình học

`control_node` subscribe:

- `/avs/telemetry_realworld`
- `/avs/route_intent`
- `/avs/cmd`
- `/odom_raw`

và publish:

- `/avs/control_error`
- `/avs/lane_state`

Node này cho thấy tầng cuối của phạm vi báo cáo không phải một controller động học, mà là một **geometric decision and output node**. Nó sử dụng dữ liệu từ nhiều topic để chọn lane phù hợp và sinh đầu ra dạng sai số hình học.

## 10. ROS2 như một lớp tích hợp hệ thống

ROS2 trong dự án này không chỉ là “thư viện truyền message”, mà là lớp tích hợp toàn hệ thống. Nó đảm nhiệm đồng thời:

- định danh các node
- tổ chức các topic
- quản lý callback runtime
- hỗ trợ parameter động
- cung cấp môi trường thuận lợi cho debug và monitoring

Ví dụ:

- `num_threads`, `prob_threshold`, `nms_threshold` là parameter của node inference
- `calibration_file_path`, `lookahead_T_preview` là parameter của node IPM
- các node có logger riêng để theo dõi runtime

Nhờ vậy, kiến trúc ROS2 vừa hỗ trợ thực thi, vừa hỗ trợ vận hành và tuning.

## 11. Hạn chế và đánh đổi của kiến trúc phân tán

Mặc dù có nhiều ưu điểm, việc tách node cũng tạo ra một số chi phí và rủi ro.

### 11.1. Tăng độ trễ truyền giữa các tầng

Mỗi biên node-topic-node đều thêm:

- chi phí serialize
- chi phí copy
- chi phí scheduling callback
- chi phí queueing

Do đó, thiết kế phân tán luôn phải đánh đổi với latency tổng.

### 11.2. Tăng độ phức tạp khi đảm bảo nhất quán dữ liệu

Khi nhiều node cùng subscribe nhiều topic khác nhau, vấn đề đồng bộ logic xuất hiện:

- odometry đến sớm hay muộn hơn telemetry
- route intent thay đổi giữa hai frame
- calibration file đổi trong lúc pipeline đang chạy

Hệ thống hiện tại xử lý phần lớn theo hướng thực dụng, ví dụ:

- đọc giá trị mới nhất có sẵn
- reload calibration khi file thay đổi
- ưu tiên latest frame

Đây là cách hợp lý cho hệ soft real-time, nhưng vẫn là một nguồn cần theo dõi kỹ.

### 11.3. Khó tối ưu cực hạn nếu schema chưa ổn định

JSON qua `std_msgs/String` giúp phát triển nhanh, nhưng nếu sau này mục tiêu chuyển sang tối ưu cực hạn cho latency và băng thông, hệ thống có thể cần:

- custom ROS messages
- schema gọn hơn
- hoặc giảm lượng polygon/waypoint publish

Nghĩa là kiến trúc hiện tại rất tốt cho giai đoạn phát triển và nghiên cứu hệ thống, nhưng vẫn để ngỏ khả năng tinh chỉnh sâu hơn khi đi vào tối ưu hóa sản phẩm.

## 12. Kết luận

Kiến trúc ROS2 của hệ thống AVS dựa trên việc chia pipeline perception thành các node độc lập giao tiếp qua topic theo mô hình publish/subscribe. Về mặt lý thuyết, đây là một dạng kiến trúc xử lý phân tán theo dòng dữ liệu, trong đó mỗi node thực hiện một phép biến đổi riêng trên dữ liệu vào và phát kết quả cho tầng sau. Cách tổ chức này đặc biệt phù hợp với perception thời gian thực vì nó hỗ trợ tách mối quan tâm, đo latency theo tầng, thay thế từng khối độc lập và tích hợp thuận lợi với dashboard giám sát.

Trong triển khai hiện tại, kiến trúc đó được thể hiện rất rõ qua chuỗi:

- `video_publisher_node`
- `ncnn_inference_node`
- `ipm_transform_node`
- `control_node`

với các topic trung gian như `/camera/image_raw`, `/avs/telemetry`, `/avs/telemetry_realworld` và `/avs/control_error`.

Một đặc điểm nổi bật của hệ thống là sử dụng JSON qua `std_msgs/String` cho nhiều topic nghiệp vụ. Giải pháp này hy sinh một phần độ chặt kiểu và hiệu quả truyền thông, nhưng đổi lại đạt được tính linh hoạt cao, khả năng mở rộng schema nhanh và sự tương thích rất tốt với dashboard web và quy trình debug. Vì vậy, xét trong bối cảnh hiện tại của AVS, đây là một quyết định kiến trúc thực dụng và hợp lý.
