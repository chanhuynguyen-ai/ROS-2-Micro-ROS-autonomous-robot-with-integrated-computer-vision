# Dàn ý Báo Cáo Hệ Thống Thị Giác AVS

Tài liệu này là dàn ý chi tiết để viết báo cáo cho hệ thống thị giác máy tính hiện tại của AVS. Phạm vi báo cáo dừng ở lớp xử lý ảnh, biến đổi tọa độ, trích xuất hình học làn đường và publish các tham số `e_x`, `e_y`, `theta` qua topic ROS2. Không bao gồm phần chuyển động, động học, lý thuyết điều khiển, `cmd_vel`, micro-ROS hay firmware ESP32.

---

## PHẦN I. GIỚI THIỆU CHUNG

### 1. Bối cảnh bài toán
- Trình bày bài toán xe tự hành bám làn trong môi trường có:
  - làn chính (`main-lane`)
  - làn kế bên (`other-lane`)
  - làn rẽ (`turn-lane`)
  - vạch phân cách liền/đứt
  - biển báo và phương tiện
- Nêu khó khăn của bài toán:
  - ảnh phối cảnh camera không trực tiếp phản ánh khoảng cách thực
  - làn đường có thể cong, đứt đoạn hoặc bị che khuất
  - dữ liệu đầu ra hình học cần ổn định theo thời gian, không được giật
  - hệ thống phải chạy thời gian thực trên phần cứng biên

### 2. Mục tiêu hệ thống hiện tại
- Nhận ảnh từ camera hoặc video.
- Chạy mô hình phân đoạn ảnh thời gian thực trên CPU.
- Trích xuất đa giác/mặt nạ của các đối tượng liên quan đến làn đường.
- Biến đổi từ tọa độ ảnh sang tọa độ thực trên mặt đường bằng homography.
- Tạo centerline và đường biểu diễn hình học cho lane đang được chọn.
- Tính và publish các tham số:
  - `epsilon_x_mm`
  - `epsilon_y_mm`
  - `theta_rad`
- Cung cấp telemetry và dữ liệu debug qua ROS2 và dashboard web.

### 3. Phạm vi báo cáo
- Bao gồm:
  - mô hình nhận diện và phân đoạn
  - tiền xử lý, hậu xử lý
  - IPM/homography
  - trích xuất centerline
  - fit đa thức
  - suy ra tham số hình học đầu ra
  - thiết kế ROS2 node/topic
  - Docker hóa hệ thống
  - tối ưu hóa hiệu năng
- Không bao gồm:
  - mọi nội dung về điều khiển
  - mọi nội dung về chuyển động và động học
  - ESP32/micro-ROS

### 4. Cấu trúc báo cáo đề xuất
- Phần lý thuyết.
- Phần triển khai hệ thống.
- Phần đánh giá hiệu năng và hạn chế.

---

## PHẦN II. CƠ SỞ LÝ THUYẾT

### 1. Bài toán phân đoạn ngữ nghĩa và instance segmentation

#### 1.1. Khái niệm
- Trình bày sự khác nhau giữa:
  - phân loại ảnh
  - phát hiện đối tượng
  - phân đoạn ngữ nghĩa
  - instance segmentation
- Giải thích vì sao hệ thống này cần phân đoạn thay vì chỉ dùng bounding box:
  - cần hình dạng vùng làn
  - cần polygon để biến đổi sang tọa độ thực
  - cần trích centerline từ vùng làn

#### 1.2. Đầu vào và đầu ra của mô hình
- Ảnh đầu vào kích thước gốc từ camera, sau đó resize về kích thước mạng.
- Đầu ra của mô hình gồm:
  - thông tin lớp
  - độ tin cậy
  - bounding box
  - hệ số mask
  - prototype masks

#### 1.3. Công thức tiền xử lý
- Chuẩn hóa giá trị điểm ảnh:
  $$
  I_{norm}(u,v,c)=\frac{I_{raw}(u,v,c)}{255}
  $$
- Nêu thêm các phép biến đổi nếu có:
  - resize
  - đổi hệ màu BGR sang RGB
  - sắp xếp tensor theo định dạng đầu vào của mạng

#### 1.4. Công thức tái tạo mặt nạ phân đoạn
- Biểu diễn prototype masks:
  $$
  P \in \mathbb{R}^{K \times H_p \times W_p}
  $$
- Vector hệ số mask của một đối tượng:
  $$
  C \in \mathbb{R}^{K}
  $$
- Mặt nạ thô:
  $$
  M_{raw}(x,y)=\sum_{i=1}^{K} C_i P_i(x,y)
  $$
- Hàm sigmoid:
  $$
  M(x,y)=\sigma(M_{raw}(x,y))=\frac{1}{1+e^{-M_{raw}(x,y)}}
  $$
- Nhị phân hóa:
  $$
  M_{bin}(x,y)=
  \begin{cases}
  1, & M(x,y)\ge \tau \\
  0, & M(x,y)<\tau
  \end{cases}
  $$
- Mô tả thêm vai trò của ngưỡng `prob_threshold` và `nms_threshold`.

#### 1.5. Lọc trùng lặp bằng NMS
- Nêu khái niệm IoU giữa hai box:
  $$
  IoU(A,B)=\frac{|A \cap B|}{|A \cup B|}
  $$
- Trình bày ý nghĩa của NMS trong việc loại bỏ detection chồng lấn.

### 2. Cơ sở lý thuyết lượng hóa và tối ưu suy luận

#### 2.1. Động cơ lượng hóa
- Giải thích vì sao cần giảm từ FP32 sang INT8:
  - giảm băng thông bộ nhớ
  - tăng tốc suy luận CPU
  - giảm chi phí năng lượng

#### 2.2. Khái niệm lượng hóa tuyến tính
- Công thức lượng hóa:
  $$
  q=\text{round}\left(\frac{x}{s}\right)+z
  $$
- Công thức giải lượng hóa:
  $$
  x \approx s(q-z)
  $$
- Giải thích:
  - `s`: scale
  - `z`: zero-point

#### 2.3. Post-Training Quantization
- Nêu quy trình PTQ:
  - dùng mô hình FP32 đã huấn luyện
  - chạy tập calibration
  - ước lượng dải giá trị activation
  - sinh mô hình INT8
- Có thể nêu ngắn gọn vai trò của calibration theo KL-divergence nếu muốn viết sâu.

### 3. Biến đổi phối cảnh ngược bằng homography

#### 3.1. Nhu cầu chuyển từ ảnh sang mặt phẳng thực
- Giải thích vì sao sai số pixel không phù hợp để điều hướng trực tiếp.
- Nêu lợi ích của hệ tọa độ thực:
  - có đơn vị mm
  - dễ so sánh giữa các frame
  - dễ suy ra sai số hình học

#### 3.2. Mô hình homography phẳng
- Với điểm ảnh:
  $$
  p=\begin{bmatrix}u\\v\\1\end{bmatrix}
  $$
- Và điểm trên mặt đất:
  $$
  P=\begin{bmatrix}X\\Y\\1\end{bmatrix}
  $$
- Quan hệ:
  $$
  \lambda P = H p
  $$
  với
  $$
  H \in \mathbb{R}^{3 \times 3}
  $$
- Suy ra:
  $$
  X=\frac{h_{11}u+h_{12}v+h_{13}}{h_{31}u+h_{32}v+h_{33}}
  $$
  $$
  Y=\frac{h_{21}u+h_{22}v+h_{23}}{h_{31}u+h_{32}v+h_{33}}
  $$

#### 3.3. Hệ tọa độ gắn với xe
- Mô tả rõ:
  - gốc tọa độ tại điểm chiếu mép dưới chính giữa ảnh xuống mặt đường
  - trục `X`: ngang, dương về bên phải
  - trục `Y`: dọc, dương về phía trước
- Đây là cơ sở để định nghĩa `e_x`, `e_y`, `theta`.

#### 3.4. Hiệu chuẩn homography
- Trình bày ý tưởng dùng các cặp điểm ảnh - điểm thực để tính ma trận `H`.
- Có thể nêu phương pháp tổng quát:
  - chọn tối thiểu 4 cặp điểm không thẳng hàng
  - giải hệ tuyến tính để tìm `H`
  - lưu vào file cấu hình `calibration.json`

### 4. Trích xuất centerline từ polygon làn

#### 4.1. Biểu diễn vùng làn bằng đa giác
- Sau phân đoạn, mỗi lane được biểu diễn bằng tập contour/polygon.
- Mục tiêu là chuyển polygon thành một đường tâm đại diện cho hướng đi.

#### 4.2. Kỹ thuật quét lát cắt cho làn dọc
- Với `main-lane` và `other-lane`, quét theo các mức `Y`.
- Tại mỗi mức `Y_i`, tìm hai giao điểm biên trái và biên phải:
  $$
  x_{mid}(Y_i)=\frac{x_{left}(Y_i)+x_{right}(Y_i)}{2}
  $$
- Tập các điểm giữa tạo thành centerline rời rạc.

#### 4.3. Kỹ thuật quét lát cắt cho làn ngang/rẽ
- Với `turn-lane`, quét theo các mức `X`.
- Tại mỗi mức `X_i`, tính:
  $$
  y_{mid}(X_i)=\frac{y_{bottom}(X_i)+y_{top}(X_i)}{2}
  $$

#### 4.4. Lọc lát cắt bị phình và nhiễu
- Nêu hiện tượng polygon bị nở rộng cục bộ do segmentation noise.
- Giải thích ý tưởng:
  - tính bề rộng trung vị của lane
  - phát hiện lát cắt có bề rộng vượt ngưỡng
  - hiệu chỉnh tâm bằng thông tin lân cận hoặc xu thế toàn cục
- Phần này nên viết theo trực giác thuật toán, không cần sa vào chứng minh.

### 5. Khớp đường cong bằng bình phương tối thiểu

#### 5.1. Mô hình đa thức cho làn dọc
- Dùng cho `main-lane` và `other-lane`:
  $$
  x(y)=a_3y^3+a_2y^2+a_1y+a_0
  $$

#### 5.2. Mô hình đa thức cho làn ngang
- Dùng cho `turn-lane`:
  $$
  y(x)=b_3x^3+b_2x^2+b_1x+b_0
  $$

#### 5.3. Bài toán bình phương tối thiểu
- Với tập điểm quan sát $(y_i,x_i)$, cực tiểu hóa:
  $$
  \min_{\mathbf{a}} \sum_{i=1}^{N}(x_i-\hat{x}(y_i))^2
  $$
- Hoặc dạng ma trận:
  $$
  \min_{\mathbf{a}} \|A\mathbf{a}-\mathbf{x}\|_2^2
  $$
- Nêu việc giải bằng SVD để ổn định số.

### 6. Suy ra các tham số hình học đầu ra

#### 6.1. Sai số lệch ngang `e_x`
- Với điểm tham chiếu được chọn trên centerline hoặc từ đa thức làn dọc:
  $$
  e_x = x_{target}
  $$
  hoặc tại gốc gần xe:
  $$
  e_x = a_0
  $$
- Ý nghĩa vật lý: độ lệch trái/phải của điểm đích so với tâm xe.

#### 6.2. Sai số dọc `e_y`
- Khoảng cách từ gốc xe tới điểm tham chiếu trên centerline:
  $$
  e_y = y_{target}
  $$
- Ý nghĩa:
  - phản ánh độ xa của điểm hình học đang được chọn
  - là thành phần dọc trong hệ tọa độ gắn với xe

#### 6.3. Sai số góc `theta`
- Nếu dùng điểm đích:
  $$
  \theta = \operatorname{atan2}(e_x,e_y)
  $$
- Nếu dùng tiếp tuyến đa thức tại vị trí xét:
  $$
  \theta = \arctan\left(\frac{dx}{dy}\right)
  $$
- Giải thích mối liên hệ giữa hai cách biểu diễn này trong hệ tọa độ gắn với xe.

### 7. Cơ sở lý thuyết kiến trúc ROS2 và xử lý phân tán

#### 7.1. Khái niệm node, topic, message
- Định nghĩa ngắn gọn:
  - node là tiến trình chức năng
  - topic là kênh truyền pub/sub
  - message là kiểu dữ liệu trao đổi

#### 7.2. Lý do tách node theo pipeline
- Dễ debug.
- Dễ thay thế từng khối.
- Tách perception, IPM và khối suy ra tham số đầu ra.
- Cho phép đo latency từng tầng.

#### 7.3. Đặc điểm của việc dùng JSON qua `std_msgs/String`
- Ưu điểm:
  - linh hoạt
  - dễ mở rộng schema
  - thuận tiện cho dashboard web
- Nhược điểm:
  - không chặt kiểu dữ liệu bằng custom ROS message
  - có overhead serialize/deserialize

---

## PHẦN III. THIẾT KẾ TRIỂN KHAI HỆ THỐNG

### 1. Kiến trúc tổng quan của hệ thống hiện tại

#### 1.1. Sơ đồ khối tổng quan
- Nên vẽ sơ đồ:
  `Video/Camera -> video_publisher_node -> ncnn_inference_node -> ipm_transform_node -> control_node -> dashboard`
- Nên thể hiện thêm:
  - file calibration
  - route intent từ dashboard
  - Docker container bao quanh các thành phần

#### 1.2. Luồng dữ liệu chính
- Ảnh/video được publish lên ROS2.
- Node perception chạy NCNN và xuất telemetry pixel-space.
- Node IPM chuyển polygon sang world frame, trích centerline và waypoint.
- Node cuối suy ra các tham số hình học và publish `e_x`, `e_y`, `theta`.
- Dashboard subscribe để hiển thị và gửi route intent.

### 2. Thiết kế package ROS2 `avs_perception`

#### 2.1. Mục đích package
- Chứa toàn bộ pipeline thị giác và xử lý hình học hiện tại.

#### 2.2. Thành phần mã nguồn chính
- `video_publisher_node.cpp`
- `ncnn_inference_node.cpp`
- `ipm_transform_node.cpp`
- `control_node.cpp`
- `video_test_node.cpp`
- `yolo26_seg.cpp/.hpp`

#### 2.3. Phụ thuộc chính
- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `nav_msgs`
- `cv_bridge`
- `image_transport`
- `OpenCV`
- `ncnn`

### 3. Thiết kế node và vai trò từng node

#### 3.1. `video_publisher_node`
- Vai trò:
  - đọc video file hoặc camera V4L2
  - publish ảnh raw và compressed
- Nội dung nên mô tả:
  - hỗ trợ nguồn `/dev/video*` hoặc file `.mp4`
  - dùng thread nền để luôn giữ frame mới nhất
  - cấu hình FPS và độ phân giải bằng parameter

#### 3.2. `ncnn_inference_node`
- Vai trò:
  - nhận ảnh raw
  - chạy segmentation bằng NCNN
  - sinh telemetry JSON ở không gian ảnh
- Các bước xử lý nên mô tả:
  - nhận `sensor_msgs/Image`
  - chuyển sang `cv::Mat`
  - tiền xử lý ảnh
  - chạy model
  - contour extraction
  - build JSON gồm `detections`, `objects`, `polygons`, latency và FPS
- Nên nêu rõ đầu ra chưa phải tọa độ thực mà vẫn là pixel-space telemetry.

#### 3.3. `ipm_transform_node`
- Vai trò:
  - đọc `calibration.json`
  - áp dụng homography cho polygon
  - trích centerline và fit lane
  - publish telemetry world-space
- Các bước xử lý:
  - reload calibration nếu file thay đổi
  - chuyển từng điểm polygon từ pixel sang mm
  - sinh `waypoints` cho lane dọc và lane rẽ
  - gắn thêm dữ liệu đa thức và waypoint vào JSON

#### 3.4. `control_node`
- Lưu ý trong báo cáo:
  - không mô tả node này như bộ điều khiển
  - chỉ mô tả đây là khối hậu xử lý hình học và publish tham số đầu ra
- Các nhiệm vụ chính:
  - nhận `telemetry_realworld`
  - nhận `route_intent`
  - chọn lane hoặc tập điểm hình học phù hợp
  - publish `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`
  - publish thêm `lane_state` để debug

#### 3.5. `video_test_node`
- Vai trò:
  - kiểm thử offline bằng video
  - đo FPS và latency
  - xuất video kết quả nếu cần

### 4. Thiết kế topic ROS2

#### 4.1. Các topic ảnh
- `/camera/image_raw`
  - kiểu: `sensor_msgs/msg/Image`
  - vai trò: ảnh đầu vào cho perception
- `/camera/image_raw/compressed`
  - kiểu: `sensor_msgs/msg/CompressedImage`
  - vai trò: phục vụ quan sát trên dashboard

#### 4.2. Topic telemetry perception
- `/avs/telemetry`
  - kiểu: `std_msgs/msg/String`
  - nội dung:
    - số lượng detection theo lớp
    - từng object với `label`, `prob`, `box`, `polygons`
    - các chỉ số latency và FPS

#### 4.3. Topic telemetry sau IPM
- `/avs/telemetry_realworld`
  - kiểu: `std_msgs/msg/String`
  - nội dung:
    - polygon trong world frame
    - `waypoints`
    - hệ số đa thức nếu có
    - thông tin làn đã chuẩn hóa cho khối hậu xử lý

#### 4.4. Topic route intent
- `/avs/route_intent`
  - kiểu: `std_msgs/msg/String`
  - chức năng:
    - cho biết nhánh xử lý hình học nào cần ưu tiên khi có nhiều lane ứng viên
  - ví dụ schema:
    - `intent`
    - `source`
    - `seq`

#### 4.5. Topic đầu ra cuối của hệ CV
- `/avs/control_error`
  - kiểu: `std_msgs/msg/String`
  - là điểm kết thúc của phạm vi báo cáo
  - các trường trọng tâm:
    - `epsilon_x_mm`
    - `epsilon_y_mm`
    - `theta_rad`
  - có thể nhắc thêm:
    - `trajectory_valid`
    - `lane_state`

#### 4.6. Topic debug trạng thái
- `/avs/lane_state`
  - kiểu: `std_msgs/msg/String`
  - dùng để:
    - hiển thị lane đang được chọn
    - hiển thị thông tin hợp lệ hoặc không hợp lệ
    - hiển thị các điểm hình học đang được dùng

### 5. Thiết kế dữ liệu và schema JSON

#### 5.1. JSON của `/avs/telemetry`
- Gợi ý các mục cần mô tả khi viết báo cáo:
  - metadata FPS/latency
  - `detections`
  - `objects`
  - polygon của từng object

#### 5.2. JSON của `/avs/telemetry_realworld`
- Các trường cần nêu:
  - object id
  - class name
  - world polygons
  - `waypoints`
  - dữ liệu lane phục vụ khối hậu xử lý

#### 5.3. JSON của `/avs/control_error`
- Các trường trọng tâm:
  - `epsilon_x_mm`
  - `epsilon_y_mm`
  - `theta_rad`
- Các trường phụ để debug:
  - `target_label`
  - `lane_state`
  - `trajectory_valid`

### 6. Thiết kế thuật toán trong từng tầng

#### 6.1. Tầng perception
- Outline khi viết:
  - nhận frame
  - inference
  - contour extraction
  - build telemetry JSON
  - publish

#### 6.2. Tầng IPM
- Outline khi viết:
  - nạp homography
  - kiểm tra cập nhật calibration
  - đổi pixel sang world
  - trích centerline
  - fit polynomial
  - publish telemetry thực địa

#### 6.3. Tầng hậu xử lý hình học
- Outline khi viết:
  - nhận observation frame
  - phân loại lane và marking
  - chọn lane theo `route_intent`
  - chọn tập điểm hoặc đường biểu diễn hình học phù hợp
  - tính `e_x`, `e_y`, `theta`
  - publish kết quả

### 7. Logic chọn lane trong phạm vi CV

#### 7.1. Các tình huống xử lý chính
- ưu tiên `main-lane`
- chọn `other-lane` khi cần
- chọn `turn-lane` khi có lane rẽ tương ứng
- loại bỏ trường hợp bị vạch liền ngăn cách nếu có dùng rule này trong mô tả

#### 7.2. Điều kiện chọn lane
- Nhận `route_intent`.
- Có hay không có lane phù hợp.
- Có hay không có vạch liền chặn chuyển làn/rẽ.
- Chất lượng dữ liệu hình học hiện tại còn hợp lệ hay không.

#### 7.3. Vai trò của logic chọn lane
- Không phát lệnh điều khiển.
- Chỉ quyết định lane nào và điểm hình học nào sẽ được quy đổi thành tham số hình học đầu ra.

### 8. Docker hóa hệ thống

#### 8.1. Mục tiêu dùng Docker
- Chuẩn hóa môi trường build và runtime.
- Giảm sai khác giữa máy phát triển và máy triển khai.
- Dễ tái lập pipeline ROS2 + OpenCV + NCNN.

#### 8.2. Kiến trúc container hiện tại
- `avs_perception`
  - build workspace
  - chạy `ncnn_inference_node`, `ipm_transform_node`, `control_node`
- `video_publisher`
  - publish nguồn camera/video
- `web_dashboard`
  - chạy backend dashboard và bridge dữ liệu

#### 8.3. Các cấu hình Docker quan trọng
- `network_mode: host`
  - giúp ROS2 DDS giao tiếp thuận lợi
- `ipc: host`
  - giảm hạn chế chia sẻ bộ nhớ
- `privileged: true`
  - hỗ trợ truy cập thiết bị camera
- `volume mounts`
  - mount `ros2_ws`, `models`, `test`, `config`

#### 8.4. Luồng khởi chạy bằng Docker Compose
- Nên mô tả:
  - container build workspace
  - source `install/setup.bash`
  - chạy các node chính
  - dashboard subscribe dữ liệu và phục vụ giao diện web

### 9. Các kỹ thuật tối ưu hóa hệ thống

#### 9.1. Tối ưu ở mức mô hình
- Xuất mô hình sang NCNN.
- Dùng mô hình INT8.
- Chọn kích thước đầu vào phù hợp để cân bằng tốc độ và độ chính xác.

#### 9.2. Tối ưu ở mức suy luận CPU
- Cấu hình số luồng `num_threads`.
- Dùng build `Release`.
- Dùng cờ tối ưu:
  - `-O3`
  - `-ffast-math`
  - `-funroll-loops`
  - tối ưu kiến trúc ARM khi triển khai trên Raspberry Pi 5

#### 9.3. Tối ưu ở mức xử lý ảnh
- Chỉ trích contour cần thiết.
- Không truyền nguyên mask ảnh đầy đủ qua topic.
- Dùng polygon thay vì tensor lớn để giảm băng thông.

#### 9.4. Tối ưu ở mức camera/video input
- Dùng camera V4L2.
- Cấu hình MJPEG để giảm tải decode.
- Tách capture thread khỏi publish timer.
- Giữ buffer kích thước 1 để luôn ưu tiên frame mới nhất.

#### 9.5. Tối ưu ở mức ROS2 và pipeline
- Tách node theo tầng để đo latency riêng.
- Dùng queue nhỏ cho ảnh đầu vào để hạn chế backlog.
- Dùng telemetry JSON để thuận tiện debug nhanh.

#### 9.6. Tối ưu ở mức hình học
- Lọc lát cắt bị phình.
- Chỉ lấy waypoint theo bước cố định.
- Làm mượt dữ liệu hình học giữa các frame để giảm dao động đầu ra.

### 10. Dashboard và công cụ giám sát

#### 10.1. Vai trò dashboard
- Quan sát ảnh nén.
- Theo dõi telemetry world-space.
- Theo dõi `lane_state` và `control_error`.
- Gửi `route_intent`.

#### 10.2. Nội dung hiển thị nên mô tả trong báo cáo
- ảnh camera
- trạng thái lane
- các điểm hình học đang được chọn
- sai số hình học đầu ra
- thông số calibration/homography

#### 10.3. Vai trò của dashboard trong quy trình phát triển
- hỗ trợ debug online
- kiểm tra logic route intent
- kiểm tra hiệu lực của calibration

---

## PHẦN IV. ĐÁNH GIÁ HỆ THỐNG

### 1. Chỉ số đánh giá hiệu năng

#### 1.1. FPS
- `input_fps`
- `processing_fps`
- `publish_fps`

#### 1.2. Độ trễ từng khối
- `bridge_latency_ms`
- `inference_latency_ms`
- `post_processing_latency_ms`
- `json_finalize_latency_ms`
- `publish_latency_ms`
- `node_total_latency_ms`
- `output_age_ms`

#### 1.3. Chỉ số chất lượng đầu ra hình học
- độ ổn định của `e_x`, `e_y`, `theta`
- mức nhảy giữa các frame
- độ tin cậy khi lane bị đứt đoạn hoặc bị che

### 2. Kịch bản thử nghiệm nên báo cáo
- Chạy video offline.
- Chạy camera trực tiếp.
- Đường thẳng.
- Đường cong.
- Ngã rẽ có `turn-lane`.
- Chuyển làn với vạch đứt và vạch liền.

### 3. Kết quả cần trình bày
- FPS trung bình.
- Độ trễ trung bình và cực đại.
- Ảnh minh họa:
  - segmentation mask
  - polygon sau contour
  - world-space waypoints
  - output `e_x`, `e_y`, `theta`

### 4. Hạn chế hiện tại
- JSON qua `std_msgs/String` chưa tối ưu kiểu dữ liệu.
- Chất lượng homography phụ thuộc mạnh vào calibration.
- Segmentation sai có thể kéo theo lỗi IPM và centerline.
- Hệ thống hiện mới dừng ở mức publish tham số hình học, chưa đi sang phần điều khiển trong phạm vi báo cáo này.

### 5. Hướng phát triển tiếp theo
- Chuẩn hóa custom ROS message thay cho JSON.
- Tối ưu thêm perception và contour extraction.
- Cải thiện độ bền vững của centerline khi lane dropout dài hơn.
- Nếu cần, tách báo cáo điều khiển thành một tài liệu độc lập ở giai đoạn sau.

---

## PHẦN V. PHỤ LỤC ĐỀ XUẤT

### 1. Danh sách lớp đối tượng của mô hình
- Liệt kê các class hiện đang dùng trong mô hình segmentation.

### 2. Bảng tham số cấu hình chính
- `prob_threshold`
- `nms_threshold`
- `num_threads`
- `camera_width`, `camera_height`, `camera_fps`

### 3. Bảng node - topic - message type
- Nên lập bảng 4 cột:
  - node
  - subscribe topic
  - publish topic
  - kiểu message

### 4. Các sơ đồ nên đưa vào báo cáo hoàn chỉnh
- sơ đồ khối tổng quan hệ thống
- flowchart pipeline perception -> IPM -> suy ra tham số -> `control_error`
- sơ đồ ROS2 node/topic
- sơ đồ Docker/container

### 5. Danh sách hình ảnh minh họa nên chèn
- ảnh camera gốc
- ảnh segmentation
- ảnh polygon/contour
- ảnh BEV sau homography
- ảnh centerline/waypoint
- ảnh dashboard

---

## Gợi ý cách viết từ dàn ý này

### 1. Với phần lý thuyết
- Mỗi mục nên viết theo thứ tự:
  - bài toán
  - khái niệm
  - công thức
  - ý nghĩa vật lý
  - liên hệ với hệ thống AVS

### 2. Với phần triển khai
- Mỗi mục nên viết theo thứ tự:
  - mục tiêu của module
  - input
  - xử lý chính
  - output
  - vai trò trong toàn pipeline

### 3. Với phần đánh giá
- Nên tách rõ:
  - đánh giá hiệu năng tính toán
  - đánh giá chất lượng hình học đầu ra
  - đánh giá tính ổn định toàn pipeline
