# PHẦN II.2. Cơ sở lý thuyết lượng hóa và tối ưu suy luận

## 1. Mở đầu

Trong hệ thống thị giác máy tính thời gian thực, chi phí suy luận của mô hình học sâu thường chiếm phần lớn tổng độ trễ pipeline. Điều này đặc biệt rõ trên phần cứng biên như Raspberry Pi 5, nơi tài nguyên tính toán, băng thông bộ nhớ và công suất điện đều hữu hạn. Vì vậy, ngoài việc lựa chọn kiến trúc mạng phù hợp, hệ thống cần các kỹ thuật giảm chi phí suy luận mà vẫn giữ được độ chính xác đủ tốt cho bài toán bám làn và nhận biết đối tượng.

Trong repo hiện tại, node `ncnn_inference_node` nạp mô hình NCNN INT8 và chạy suy luận trên CPU thông qua lớp `YOLO26Seg`. Theo mã nguồn hiện tại:

- mô hình được nạp từ `best_ncnn_model_int8`
- đầu vào được resize về `320x320`
- suy luận chạy trên CPU, không dùng Vulkan
- runtime bật `use_int8_inference = true`
- đồng thời tận dụng các tối ưu `FP16` và `packing layout`
- số luồng xử lý được cấu hình qua tham số `num_threads`

Do đó, phần cơ sở lý thuyết này không chỉ trình bày nguyên lý lượng hóa nói chung, mà còn giải thích vì sao các kỹ thuật đó phù hợp với kiến trúc suy luận của hệ thống hiện tại.

## 2. Động cơ lượng hóa trong hệ thống nhúng thời gian thực

### 2.1. Giới hạn của biểu diễn FP32

Trong huấn luyện, trọng số và activation thường được biểu diễn dưới dạng số thực dấu phẩy động 32 bit (`FP32`). Cách biểu diễn này cho độ chính xác cao và dải động lớn, nhưng có ba nhược điểm khi triển khai ở biên:

- mỗi phần tử chiếm `4 byte`, làm tăng lưu lượng đọc/ghi bộ nhớ
- phép toán dấu phẩy động thường tốn tài nguyên hơn số nguyên
- cache CPU bị áp lực lớn hơn khi tensor trung gian quá lớn

Với các mô hình segmentation, chi phí không chỉ nằm ở backbone mà còn ở:

- feature map trung gian
- tensor đầu ra cho box, class score, mask coefficient
- prototype masks dùng để tái tạo mặt nạ từng đối tượng

Nếu toàn bộ pipeline đều giữ ở `FP32`, hệ thống dễ gặp các vấn đề:

- FPS thấp
- độ trễ tăng mạnh khi nhiều object xuất hiện
- CPU bị chiếm dụng cao, làm giảm headroom cho ROS2, IPM và hậu xử lý hình học

### 2.2. Mục tiêu của lượng hóa

Lượng hóa (`quantization`) là kỹ thuật ánh xạ các giá trị số thực liên tục sang một tập giá trị rời rạc có độ rộng bit thấp hơn, phổ biến là `INT8`. Mục tiêu gồm:

- giảm dung lượng mô hình
- giảm băng thông bộ nhớ
- tăng khả năng tận dụng SIMD/NEON trên CPU
- giảm độ trễ suy luận
- giảm năng lượng tiêu thụ trên mỗi frame

Trên phần cứng CPU ARM, lợi ích lớn nhất thường không đến từ việc “số nguyên luôn nhanh hơn dấu phẩy động” một cách tuyệt đối, mà đến từ tổng hợp các yếu tố:

- tensor nhỏ hơn nên ít tốn cache miss hơn
- nhiều phép toán được vector hóa tốt hơn
- dữ liệu ít hơn nên giảm chi phí di chuyển qua memory hierarchy

Đối với hệ thống AVS, điều này có ý nghĩa trực tiếp vì perception không chạy đơn lẻ mà nằm trong một pipeline ROS2 nhiều node. Mỗi mili giây tiết kiệm ở khối suy luận đều có thể chuyển thành:

- giảm end-to-end latency
- tăng tính ổn định nhịp publish
- giảm hiện tượng backlog frame

## 3. Khái niệm lượng hóa tuyến tính

### 3.1. Ánh xạ từ số thực sang số nguyên

Lượng hóa tuyến tính dùng hai tham số:

- `s` (`scale`)
- `z` (`zero-point`)

để ánh xạ một giá trị thực `x` sang giá trị nguyên lượng hóa `q`:

$$
q=\operatorname{round}\left(\frac{x}{s}\right)+z
$$

Trong đó:

- `x` là giá trị thực ban đầu
- `q` là giá trị nguyên sau lượng hóa
- `s > 0` là hệ số co giãn
- `z` là điểm zero-point sao cho giá trị thực `0` có thể được biểu diễn gần đúng trong miền số nguyên

Với dữ liệu `INT8`, miền giá trị thường là:

$$
q \in [-128,127]
$$

hoặc với biểu diễn unsigned:

$$
q \in [0,255]
$$

### 3.2. Giải lượng hóa

Khi cần khôi phục gần đúng về miền thực, ta dùng:

$$
x \approx s(q-z)
$$

Biểu thức này được gọi là giải lượng hóa (`dequantization`). Trong thực tế, nhiều backend suy luận không giải lượng hóa toàn bộ tensor về `FP32`, mà thực hiện tích chập hay nhân ma trận trong miền số nguyên, sau đó chỉ chuyển đổi ở những điểm cần thiết. Đây là cơ sở giúp giảm chi phí tính toán.

### 3.3. Ý nghĩa của `scale`

`scale` quyết định độ phân giải của miền lượng hóa. Nếu `x_{min}` và `x_{max}` là cận dưới và cận trên của tensor thực, một cách chọn điển hình là:

$$
s=\frac{x_{max}-x_{min}}{q_{max}-q_{min}}
$$

Khi `s` nhỏ:

- biểu diễn tinh hơn
- nhưng dễ tràn miền nếu dải dữ liệu thực quá rộng

Khi `s` lớn:

- giảm nguy cơ clipping
- nhưng tăng sai số làm tròn

Vì vậy, việc ước lượng đúng dải động của tensor là bước rất quan trọng trong lượng hóa.

### 3.4. Vai trò của `zero-point`

`zero-point` cho phép ánh xạ giá trị thực `0` vào đúng hoặc gần đúng một giá trị nguyên. Điều này đặc biệt hữu ích khi activation hoặc input không đối xứng quanh 0.

Nếu miền lượng hóa là bất đối xứng, zero-point có thể được tính như:

$$
z=\operatorname{round}\left(q_{min}-\frac{x_{min}}{s}\right)
$$

Sau đó được chặn trong miền biểu diễn hợp lệ của `INT8`.

Trong nhiều trường hợp thực tế:

- trọng số thường dùng lượng hóa đối xứng quanh 0
- activation thường dùng lượng hóa bất đối xứng

Lý do là phân bố trọng số thường cân bằng hơn, còn activation sau ReLU hoặc sigmoid thường lệch về một phía.

## 4. Sai số do lượng hóa

### 4.1. Sai số làm tròn

Vì số thực được ánh xạ sang tập rời rạc, nên luôn tồn tại sai số:

$$
\varepsilon = x - s(q-z)
$$

Sai số này đến từ:

- làm tròn (`rounding`)
- clipping nếu giá trị vượt quá dải biểu diễn
- sai lệch dải động ước lượng khi calibration không đủ đại diện

### 4.2. Tích lũy sai số qua nhiều lớp

Một lớp riêng lẻ có thể chỉ gây sai số nhỏ, nhưng qua nhiều tầng tích chập, cộng, chuẩn hóa và activation, sai số có thể tích lũy. Đối với bài toán segmentation, hậu quả có thể xuất hiện ở nhiều mức:

- score lớp thay đổi nhẹ, làm detection qua hoặc không qua `prob_threshold`
- box dịch chuyển vài pixel sau bước decode
- hệ số mask coefficient sai khác làm biên segmentation bị méo
- polygon trích xuất sau `findContours()` kém ổn định hơn

Điều này rất quan trọng đối với hệ AVS vì đầu ra cuối không dừng ở detection, mà còn đi tiếp qua:

- biến đổi homography
- centerline extraction
- fitting hình học
- suy ra `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`

Nghĩa là sai số lượng hóa ở perception có thể lan truyền thành sai số hình học ở lớp trên.

### 4.3. Đánh đổi tốc độ và độ chính xác

Lượng hóa luôn là một bài toán đánh đổi:

- bit-width càng thấp thì tốc độ và tiết kiệm bộ nhớ càng tốt
- nhưng sai số xấp xỉ càng lớn

Trong triển khai thực tế, mục tiêu không phải là giữ đầu ra `INT8` giống tuyệt đối `FP32`, mà là:

- sai số đủ nhỏ để không làm hỏng quyết định downstream
- throughput đủ cao để hệ thống chạy thời gian thực ổn định

Với bài toán xe tự hành ở tốc độ thấp trong môi trường có cấu trúc, một mô hình `INT8` có thể tốt hơn mô hình `FP32` nếu:

- hình học làn vẫn ổn định
- độ trễ giảm rõ rệt
- tổng chất lượng điều hướng theo chuỗi xử lý tốt hơn

## 5. Lượng hóa theo tensor và theo kênh

### 5.1. Per-tensor quantization

Toàn bộ tensor dùng chung một cặp `(s, z)`:

$$
q_i=\operatorname{round}\left(\frac{x_i}{s}\right)+z
$$

Ưu điểm:

- đơn giản
- ít metadata
- dễ triển khai

Nhược điểm:

- một vài kênh có biên độ lớn có thể làm giảm độ phân giải của các kênh nhỏ hơn

### 5.2. Per-channel quantization

Mỗi kênh có scale riêng:

$$
q_{i,c}=\operatorname{round}\left(\frac{x_{i,c}}{s_c}\right)+z_c
$$

Ưu điểm:

- giảm sai số lượng hóa cho trọng số convolution
- đặc biệt hiệu quả khi các kênh có phân bố rất khác nhau

Nhược điểm:

- phức tạp hơn
- backend runtime phải hỗ trợ tốt

Trong các backend tối ưu cho CNN như NCNN, việc dùng lượng hóa theo kênh cho trọng số là một thực hành phổ biến vì giữ được nhiều độ chính xác hơn so với chỉ lượng hóa theo tensor.

## 6. Post-Training Quantization

### 6.1. Khái niệm

`Post-Training Quantization` (PTQ) là phương pháp lượng hóa mô hình sau khi quá trình huấn luyện FP32 đã hoàn tất. Quy trình tổng quát:

1. huấn luyện hoặc nhận mô hình gốc ở `FP32`
2. chạy mô hình trên một tập calibration đại diện
3. ước lượng dải động của trọng số và activation
4. sinh ra mô hình lượng hóa `INT8`
5. kiểm tra lại độ chính xác và hiệu năng

Ưu điểm lớn nhất của PTQ là:

- không cần huấn luyện lại toàn bộ mô hình
- quy trình triển khai nhanh hơn
- phù hợp khi mục tiêu là đẩy mô hình xuống thiết bị biên càng sớm càng tốt

### 6.2. Calibration

Calibration là bước quan sát activation của mô hình trên một tập dữ liệu đại diện để ước lượng:

- `x_{min}`
- `x_{max}`
- hoặc histogram phân bố

Từ đó hệ thống tính:

- scale cho từng tensor hoặc từng kênh
- zero-point tương ứng

Nếu tập calibration không đại diện cho phân phối vận hành thật, mô hình `INT8` có thể bị giảm chất lượng mạnh. Với hệ thống AVS, tập calibration cần phản ánh đủ các tình huống:

- ánh sáng khác nhau
- lane thẳng, cong, rẽ
- mức độ che khuất khác nhau
- số lượng object ít và nhiều

### 6.3. Vai trò của KL-divergence

Một cách chọn ngưỡng lượng hóa phổ biến là tối ưu theo `KL-divergence`, tức là tìm dải biểu diễn sao cho phân bố sau lượng hóa gần với phân bố gốc nhất. Về trực giác:

- nếu cắt dải quá hẹp thì nhiều giá trị bị clipping
- nếu dải quá rộng thì bước lượng tử quá thô

Tối ưu theo KL-divergence giúp chọn điểm cân bằng giữa hai yếu tố này.

Không phải mọi workflow PTQ đều bắt buộc dùng KL-divergence, nhưng đây là một nguyên lý quan trọng khi muốn mô hình INT8 giữ được hành vi gần với mô hình FP32.

### 6.4. Hạn chế của PTQ

PTQ rất thực dụng nhưng có các giới hạn:

- nhạy với chất lượng tập calibration
- một số lớp hoặc activation khó lượng hóa hơn
- mô hình segmentation thường nhạy hơn classification vì cần giữ cấu trúc không gian chi tiết

Với các đầu ra mask, sai số nhỏ ở activation vẫn có thể làm thay đổi biên mask sau sigmoid và thresholding. Do đó khi đánh giá PTQ cho hệ này, không nên chỉ nhìn mAP hay accuracy tổng quát, mà còn phải nhìn:

- độ mượt của polygon
- độ ổn định centerline
- sai số hình học sau IPM

## 7. Quantization-Aware Training

### 7.1. Khái niệm

`Quantization-Aware Training` (QAT) mô phỏng sai số lượng hóa ngay trong lúc huấn luyện. Thay vì huấn luyện `FP32` xong rồi mới ép sang `INT8`, QAT buộc mô hình học cách thích nghi với nhiễu lượng hóa từ đầu.

Về mặt khái niệm, trong forward pass, một số tensor được chèn bước giả lượng hóa:

$$
\hat{x}=s\left(\operatorname{round}\left(\frac{x}{s}\right)+z-z\right)
$$

nhưng gradient vẫn được lan truyền xấp xỉ trong backward pass.

### 7.2. So sánh PTQ và QAT

PTQ:

- nhanh hơn
- dễ triển khai hơn
- phù hợp khi cần đưa mô hình lên thiết bị sớm

QAT:

- tốn công huấn luyện hơn
- nhưng thường giữ độ chính xác tốt hơn, nhất là với mô hình nhạy lượng hóa

Trong bối cảnh repo hiện tại, việc dùng mô hình `best_ncnn_model_int8` cho thấy pipeline đang nghiêng về hướng triển khai thực dụng kiểu PTQ hoặc workflow đã sinh sẵn model INT8. Nếu sau này chất lượng segmentation giảm đáng kể khi đổi từ FP32 sang INT8, QAT là hướng cần cân nhắc.

## 8. Tối ưu suy luận ở mức runtime

Lượng hóa chỉ là một phần. Tốc độ thực tế của hệ thống còn phụ thuộc mạnh vào runtime suy luận và cách tổ chức bộ nhớ, luồng xử lý, cũng như chi phí hậu xử lý.

### 8.1. Tối ưu bằng backend NCNN

Hệ thống hiện dùng NCNN, là backend được tối ưu mạnh cho thiết bị biên và CPU ARM. Một số điểm quan trọng trong mã hiện tại:

- `use_vulkan_compute = false`
- `use_fp16_packed = true`
- `use_fp16_storage = true`
- `use_fp16_arithmetic = true`
- `use_packing_layout = true`
- `use_int8_inference = true`

Ý nghĩa lý thuyết:

- `packing layout` giúp sắp xếp dữ liệu phù hợp với vector register, tăng hiệu quả SIMD
- `FP16 storage` và `FP16 packed` giảm băng thông bộ nhớ cho các tensor trung gian
- `INT8 inference` cho phép các lớp hỗ trợ chạy trong miền số nguyên thấp bit

Ở đây cần phân biệt rõ:

- lượng hóa mô hình là biến đổi biểu diễn dữ liệu và tham số
- tối ưu runtime là cách backend khai thác phần cứng để thực thi mô hình đó hiệu quả hơn

Hai thành phần này bổ sung cho nhau. Mô hình INT8 chưa chắc đã nhanh nếu runtime không tối ưu; ngược lại runtime tốt nhưng vẫn giữ FP32 toàn bộ thì vẫn bị áp lực bộ nhớ lớn.

### 8.2. Tối ưu số luồng

Chi phí suy luận trên CPU phụ thuộc vào số luồng thực thi `num_threads`. Nếu gọi `T_1` là thời gian chạy một luồng, thời gian chạy nhiều luồng không lý tưởng bằng:

$$
T_n \ne \frac{T_1}{n}
$$

do còn tồn tại:

- overhead đồng bộ
- tranh chấp cache
- tranh chấp băng thông bộ nhớ
- cạnh tranh CPU với các node ROS2 khác

Vì vậy, tăng số luồng không luôn làm giảm end-to-end latency. Trên một pipeline perception hoàn chỉnh, dùng ít hơn số lõi tối đa đôi khi lại tốt hơn vì:

- để CPU cho `cv_bridge`, contour extraction và publish
- giảm jitter scheduler
- giảm nguy cơ thermal throttling

Việc repo cho phép cấu hình `num_threads` ở runtime là một quyết định đúng về mặt hệ thống, vì tối ưu thực sự phải dựa trên benchmark toàn pipeline chứ không chỉ benchmark inference đơn lẻ.

### 8.3. Tối ưu độ phân giải đầu vào

Trong mã hiện tại, ảnh được resize về:

$$
320 \times 320
$$

Với CNN, chi phí tính toán của nhiều lớp tích chập tăng gần tỷ lệ với số điểm ảnh đầu vào. Nếu tăng kích thước không gian lên `640x640`, số phần tử tăng xấp xỉ gấp 4 lần:

$$
\frac{640 \cdot 640}{320 \cdot 320}=4
$$

Do đó, chọn input size là một điểm cân bằng giữa:

- chi tiết không gian của mask
- khả năng phân biệt vật thể nhỏ
- tốc độ suy luận

Trong hệ AVS, `320x320` là lựa chọn hợp lý khi ưu tiên thời gian thực trên CPU và downstream chủ yếu cần hình học lane đủ ổn định hơn là biên segmentation quá sắc nét.

### 8.4. Tối ưu tiền xử lý

Tiền xử lý hiện tại gồm:

- resize
- đổi kênh màu `BGR -> RGB`
- chuẩn hóa:

$$
I_{norm}(u,v,c)=\frac{I_{raw}(u,v,c)}{255}
$$

Về lý thuyết, tiền xử lý càng sát format đầu vào nội bộ của backend thì càng giảm copy và chuyển đổi trung gian. Việc dùng `ncnn::Mat::from_pixels_resize(...)` ngay từ đầu giúp gom:

- đọc pixel
- đổi định dạng
- resize

vào pipeline phù hợp với NCNN, giảm một phần overhead so với việc tạo nhiều tensor trung gian bên ngoài.

### 8.5. Tối ưu hậu xử lý

Trong segmentation, hậu xử lý có thể trở thành bottleneck thật sự sau khi phần inference lõi đã được tăng tốc. Hệ hiện tại có các bước:

1. giải mã box, class score, mask coefficient
2. NMS
3. tái tạo mask từ prototype
4. `findContours()`
5. tuần tự hóa polygon sang JSON

Khi inference lõi được giảm chi phí nhờ INT8, phần hậu xử lý có thể chiếm tỷ trọng lớn hơn trong tổng latency. Đây là hiện tượng thường gặp: tối ưu một khâu làm lộ ra bottleneck mới ở khâu khác.

## 9. Giải mã mask và ý nghĩa tối ưu ROI

### 9.1. Công thức tái tạo mask

Với tensor prototype:

$$
P \in \mathbb{R}^{K \times H_p \times W_p}
$$

và vector hệ số mask của một detection:

$$
C \in \mathbb{R}^{K}
$$

mặt nạ thô tại vị trí `(x,y)`:

$$
M_{raw}(x,y)=\sum_{i=1}^{K} C_i P_i(x,y)
$$

Sau đó áp dụng sigmoid:

$$
M(x,y)=\sigma(M_{raw}(x,y))=\frac{1}{1+e^{-M_{raw}(x,y)}}
$$

và nhị phân hóa:

$$
M_{bin}(x,y)=
\begin{cases}
1, & M(x,y)\ge 0.5 \\
0, & M(x,y)<0.5
\end{cases}
$$

### 9.2. Tối ưu chỉ tính trong ROI

Nếu tính toàn bộ prototype map cho mọi detection, số phép toán tăng theo:

$$
O(K \cdot H_p \cdot W_p \cdot N)
$$

với `N` là số detection.

Nhưng với nhiều object nhỏ, phần lớn diện tích prototype map là không liên quan tới box hiện tại. Vì vậy, chiến lược chỉ giải mã trong vùng `ROI` của bounding box giúp giảm chi phí xuống gần:

$$
O\left(K \cdot \sum_{j=1}^{N} A_j\right)
$$

trong đó `A_j` là diện tích ROI của detection thứ `j` trong prototype space.

Về mặt lý thuyết, đây là một ví dụ điển hình của tối ưu theo miền tính toán hữu ích:

- không đổi ý nghĩa toán học trong vùng cần dùng
- nhưng loại bỏ phép tính ở vùng chắc chắn bị bỏ đi sau đó

Trong repo hiện tại, đây là tối ưu rất hợp lý vì mask cuối cùng chỉ được dùng trong bounding box rồi mới trích contour.

## 10. Tối ưu ở mức biên dịch và kiến trúc phần cứng

### 10.1. Build `Release`

Nếu pipeline bị build ở `Debug`, trình biên dịch thường bỏ qua nhiều tối ưu mạnh, làm sai lệch hoàn toàn đánh giá hiệu năng. Do đó việc mặc định build `Release` là yêu cầu cơ bản cho một hệ thống thời gian thực.

### 10.2. Cờ tối ưu số học

Các cờ như:

- `-O3`
- `-ffast-math`
- `-funroll-loops`

giúp trình biên dịch:

- mở rộng tối ưu nội tuyến
- sắp xếp lại biểu thức số học
- giảm chi phí vòng lặp trong các đoạn xử lý tensor hoặc pixel

Tuy nhiên, `-ffast-math` cũng có nghĩa là chấp nhận một số đánh đổi về tính chặt chẽ IEEE-754 để đổi lấy tốc độ. Với pipeline perception thời gian thực, đây thường là đánh đổi chấp nhận được nếu đầu ra cuối vẫn ổn định.

### 10.3. Tối ưu kiến trúc ARM

Các cờ:

- `-march=armv8.2-a`
- `-mtune=cortex-a76`

cho phép binary được tối ưu sát hơn với đặc tính vi kiến trúc CPU mục tiêu. Khi kết hợp với backend NCNN có hỗ trợ `NEON`, hệ thống khai thác tốt hơn:

- vector instruction
- pipeline thực thi của lõi ARM
- cache behavior đặc thù phần cứng đích

Đây là phần quan trọng trong tối ưu suy luận vì hiệu năng thực không chỉ do thuật toán mạng quyết định, mà còn do độ phù hợp giữa mã máy sinh ra và CPU chạy nó.

## 11. Cách đánh giá hiệu quả lượng hóa trong hệ AVS

Đánh giá lượng hóa không nên dừng ở việc “mô hình chạy nhanh hơn bao nhiêu”. Với hệ AVS, cần đánh giá đồng thời ba lớp chỉ số.

### 11.1. Chỉ số mô hình

- mAP hoặc IoU segmentation nếu có bộ test gán nhãn
- độ chính xác lớp lane và vehicle
- chất lượng mask biên

### 11.2. Chỉ số runtime

- `inference_latency_ms`
- `full_latency_ms`
- FPS trung bình
- p95 hoặc p99 latency
- CPU utilization theo số luồng

### 11.3. Chỉ số hệ thống cuối

- độ ổn định polygon
- độ mượt centerline
- dao động của `epsilon_x_mm`
- dao động của `theta_rad`
- tỷ lệ frame bị bỏ hoặc đến muộn

Một mô hình INT8 được xem là thành công nếu tổng thể hệ thống tốt hơn mô hình FP32 trên các chỉ số cuối này, kể cả khi có giảm nhẹ độ chính xác thuần học sâu.

## 12. Kết luận

Lượng hóa là cơ sở trọng tâm giúp mô hình segmentation chạy thời gian thực trên phần cứng biên. Về bản chất, kỹ thuật này chuyển biểu diễn từ `FP32` sang miền bit thấp hơn, điển hình là `INT8`, thông qua các tham số `scale` và `zero-point`, từ đó giảm băng thông bộ nhớ và tăng hiệu quả tính toán. Tuy nhiên, lợi ích của lượng hóa chỉ phát huy đầy đủ khi đi kèm với backend suy luận tối ưu, cấu hình luồng hợp lý, kích thước đầu vào phù hợp và hậu xử lý được kiểm soát chi phí.

Trong hệ thống AVS hiện tại, phần lý thuyết này gắn trực tiếp với các quyết định triển khai đang có:

- dùng mô hình NCNN INT8
- chạy suy luận CPU thay vì Vulkan
- bật các tối ưu `FP16`, `packing layout`, `INT8 inference`
- cho phép tinh chỉnh `num_threads`
- giữ input ở `320x320`
- giảm chi phí giải mã mask bằng ROI

Do đó, “lượng hóa và tối ưu suy luận” không phải là hai chủ đề tách rời, mà là một chuỗi quyết định thống nhất nhằm đạt mục tiêu cốt lõi của hệ thống: duy trì đầu ra perception đủ chính xác, đủ ổn định và đủ nhanh để phục vụ điều hướng thời gian thực.
