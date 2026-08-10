# PHẦN II.5. Khớp đường cong bằng bình phương tối thiểu

## 1. Mở đầu

Sau khi hệ thống đã:

- phân đoạn được vùng làn đường
- biến đổi polygon từ ảnh sang hệ tọa độ thực bằng homography
- trích xuất được các waypoint nằm gần centerline

thì vẫn còn một bài toán quan trọng: làm sao biểu diễn hình dạng lane bằng một mô hình toán học gọn, trơn và ổn định theo thời gian. Nếu chỉ giữ nguyên tập waypoint rời rạc, dữ liệu đầu ra sẽ:

- nhiễu theo từng frame
- khó nội suy ở các khoảng nhìn trước
- khó suy ra trực tiếp các đại lượng như lệch ngang, góc hướng và độ cong

Vì vậy, hệ thống AVS dùng **khớp đường cong bằng bình phương tối thiểu** để xấp xỉ lane bằng một hàm đa thức bậc 3. Đây là bước biến tập điểm đo được từ perception thành một mô hình hình học liên tục có thể phục vụ điều hướng.

## 2. Vì sao cần khớp đường cong

Waypoint trích từ polygon lane là dữ liệu đo thực nghiệm. Dữ liệu này thường có các đặc điểm:

- không đều theo khoảng cách
- có nhiễu từ segmentation
- có thể thiếu điểm ở một số vùng
- dao động nhẹ giữa các frame liên tiếp

Nếu điều khiển bám lane dùng trực tiếp các điểm này, sai số đầu ra sẽ dễ bị giật. Thay vào đó, ta cần một hàm xấp xỉ toàn cục sao cho:

- vẫn bám theo xu hướng chung của lane
- làm trơn nhiễu cục bộ
- cho phép đánh giá vị trí lane ở mọi khoảng nhìn trước
- cho phép tính đạo hàm để suy ra góc và độ cong

Least-squares polynomial fitting giải quyết đúng nhu cầu này.

## 3. Mô hình đa thức dùng trong hệ thống

### 3.1. Làn dọc: `x(y)`

Với các lane kéo dài theo phương dọc của xe, như `main-lane` và `other-lane`, hệ thống biểu diễn vị trí ngang `x` như một hàm của khoảng cách dọc `y`:

$$
x(y)=a_3y^3+a_2y^2+a_1y+a_0
$$

Đây là mô hình được nêu trong `../avs_system_report_outline.md` và cũng là mô hình đang được hiện thực trong hàm:

- `fit_polynomial_xy()`

Tên hàm `xy` ở đây nên hiểu là:

- đầu vào waypoint có dạng `(x, y)`
- biến độc lập là `y`
- biến phụ thuộc là `x`

Lựa chọn `x(y)` là hợp lý cho lane dọc vì:

- các điểm lane trải dài chủ yếu theo trục `Y`
- tại cùng một giá trị `Y`, lane thường chỉ có một giá trị `X` trung tâm
- khó mô tả ngược lại bằng `y(x)` khi lane gần thẳng đứng

### 3.2. Làn ngang hoặc rẽ: `y(x)`

Với lane rẽ hoặc lane trải theo phương ngang, hệ thống đổi vai trò biến số:

$$
y(x)=b_3x^3+b_2x^2+b_1x+b_0
$$

Trong code, mô hình này được thực hiện bởi:

- `fit_polynomial_yx()`

Ở đây:

- biến độc lập là `x`
- biến phụ thuộc là `y`

Lựa chọn này tránh hiện tượng biểu diễn lane rẽ bằng `x(y)` trở nên không đơn trị hoặc rất nhạy số.

### 3.3. Vì sao chọn đa thức bậc 3

Đa thức bậc 3 là điểm cân bằng tốt giữa năng lực biểu diễn và độ ổn định:

- bậc 1 chỉ mô tả được đường thẳng
- bậc 2 mô tả được cong đơn giản nhưng thiếu linh hoạt khi hình dạng lane đổi độ cong
- bậc 3 cho phép mô hình hóa cả độ cong lẫn sự thay đổi của độ cong
- bậc cao hơn dễ overfit với waypoint nhiễu

Trong môi trường AVS hiện tại, bậc 3 là lựa chọn thực dụng và đủ cho đa số lane cong nhẹ đến vừa.

## 4. Bài toán bình phương tối thiểu

### 4.1. Phát biểu bài toán cho làn dọc

Giả sử ta có tập waypoint thực trong world frame:

$$
(x_1,y_1), (x_2,y_2), \dots, (x_N,y_N)
$$

và muốn khớp mô hình:

$$
\hat{x}(y)=a_3y^3+a_2y^2+a_1y+a_0
$$

thì bài toán bình phương tối thiểu là:

$$
\min_{\mathbf{a}} \sum_{i=1}^{N}\left(x_i-\hat{x}(y_i)\right)^2
$$

với:

$$
\mathbf{a}=
\begin{bmatrix}
a_3\\a_2\\a_1\\a_0
\end{bmatrix}
$$

Mục tiêu là tìm bộ hệ số làm tổng bình phương sai số nhỏ nhất.

### 4.2. Dạng ma trận

Ta xây dựng ma trận thiết kế:

$$
A=
\begin{bmatrix}
y_1^3 & y_1^2 & y_1 & 1\\
y_2^3 & y_2^2 & y_2 & 1\\
\vdots & \vdots & \vdots & \vdots\\
y_N^3 & y_N^2 & y_N & 1
\end{bmatrix}
$$

vector hệ số:

$$
\mathbf{a}=
\begin{bmatrix}
a_3\\a_2\\a_1\\a_0
\end{bmatrix}
$$

và vector quan sát:

$$
\mathbf{x}=
\begin{bmatrix}
x_1\\x_2\\ \vdots \\x_N
\end{bmatrix}
$$

Khi đó bài toán trở thành:

$$
\min_{\mathbf{a}} \|A\mathbf{a}-\mathbf{x}\|_2^2
$$

Đây chính là dạng được dùng trong tài liệu lý thuyết và cũng là cách cài đặt thực tế trong `ipm_transform_node.cpp`.

### 4.3. Dạng cho làn ngang

Tương tự, với lane rẽ cần mô hình:

$$
\hat{y}(x)=b_3x^3+b_2x^2+b_1x+b_0
$$

ta có:

$$
A=
\begin{bmatrix}
x_1^3 & x_1^2 & x_1 & 1\\
x_2^3 & x_2^2 & x_2 & 1\\
\vdots & \vdots & \vdots & \vdots\\
x_N^3 & x_N^2 & x_N & 1
\end{bmatrix},
\qquad
\mathbf{b}=
\begin{bmatrix}
b_3\\b_2\\b_1\\b_0
\end{bmatrix},
\qquad
\mathbf{y}=
\begin{bmatrix}
y_1\\y_2\\ \vdots \\y_N
\end{bmatrix}
$$

và cực tiểu hóa:

$$
\min_{\mathbf{b}} \|A\mathbf{b}-\mathbf{y}\|_2^2
$$

## 5. Vì sao dùng bình phương tối thiểu

Bình phương tối thiểu phù hợp cho bài toán lane fitting vì:

- số waypoint thường nhiều hơn số ẩn
- dữ liệu có nhiễu nên không nên ép đường cong đi qua mọi điểm
- cần lời giải ổn định theo nghĩa toàn cục
- dễ giải nhanh bằng thư viện tuyến tính chuẩn

Ý nghĩa trực quan của phương pháp là:

- mỗi waypoint “kéo” đường cong về phía mình
- lời giải cuối cùng là đường cong cân bằng tốt nhất giữa các waypoint
- các sai số lớn bị phạt mạnh hơn do bị bình phương

Đây là lý do least squares thường được dùng trong lane model fitting, sensor fusion và path approximation.

## 6. Cách hệ thống hiện tại hiện thực least squares

### 6.1. Hàm `fit_polynomial_xy()`

Trong file `ros2_ws/src/avs_perception/src/ipm_transform_node.cpp`, hàm `fit_polynomial_xy()` nhận vào một vector `Waypoint`, trong đó mỗi waypoint có:

- `x`
- `y`

Nếu số waypoint:

- nhỏ hơn `2`: trả về toàn bộ hệ số bằng `0`
- từ `2` đến `3`: fit tuyến tính
- từ `4` trở lên: fit đa thức bậc 3

Với trường hợp bậc 3, code tạo:

- ma trận `A` kích thước `N x 4`
- vector `B` kích thước `N x 1`

trong đó mỗi dòng của `A` là:

$$
[y_i^3,\; y_i^2,\; y_i,\; 1]
$$

và:

$$
B_i = x_i
$$

Sau đó lời giải được tính bằng:

`cv::solve(A, B, C, cv::DECOMP_SVD)`

Cuối cùng:

- `C(0)` là `a3`
- `C(1)` là `a2`
- `C(2)` là `a1`
- `C(3)` là `a0`

### 6.2. Hàm `fit_polynomial_yx()`

Hàm `fit_polynomial_yx()` có cấu trúc hoàn toàn tương tự, chỉ khác:

- biến độc lập là `x`
- biến phụ thuộc là `y`

Mỗi dòng của ma trận thiết kế là:

$$
[x_i^3,\; x_i^2,\; x_i,\; 1]
$$

và:

$$
B_i = y_i
$$

Do đó, về mặt toán học, hai hàm fit là hai phiên bản đối xứng của cùng một nguyên lý least squares.

## 7. Giải bằng SVD để ổn định số

### 7.1. Lý do không giải trực tiếp phương trình chuẩn

Về lý thuyết, bài toán least squares có thể giải qua phương trình chuẩn:

$$
A^TA\mathbf{a}=A^T\mathbf{x}
$$

và:

$$
\mathbf{a}=(A^TA)^{-1}A^T\mathbf{x}
$$

Tuy nhiên, cách này có nhược điểm:

- khuếch đại sai số số học
- dễ mất ổn định khi các cột của `A` gần phụ thuộc tuyến tính
- đặc biệt nhạy khi giá trị `x`, `y` lớn hoặc waypoint phân bố kém

### 7.2. Lợi ích của SVD

Trong hệ thống hiện tại, OpenCV được yêu cầu giải bằng:

`cv::DECOMP_SVD`

SVD có ưu điểm:

- ổn định số hơn
- xử lý tốt hệ overdetermined
- bền hơn khi dữ liệu gần suy biến
- phù hợp với dữ liệu waypoint có nhiễu hoặc phân bố chưa đều

Điều này khớp với ghi chú trong `../avs_system_report_outline.md` rằng phần này nên “nêu việc giải bằng SVD để ổn định số”.

## 8. Fallback fit tuyến tính khi số điểm ít

Một chi tiết rất quan trọng trong triển khai hiện tại là hệ thống **không cố fit bậc 3 khi dữ liệu quá ít**.

Nếu chỉ có `2` hoặc `3` waypoint, code chuyển sang mô hình tuyến tính:

### 8.1. Cho làn dọc

$$
x(y)=a_1y+a_0
$$

khi đó:

- `a3 = 0`
- `a2 = 0`

### 8.2. Cho làn ngang

$$
y(x)=b_1x+b_0
$$

khi đó:

- hệ số bậc 3 và bậc 2 bị đặt bằng `0`

Lý do của fallback này là hợp lý:

- fit bậc 3 với quá ít điểm dễ tạo đường cong vô nghĩa
- lane quan sát ngắn vẫn có thể xấp xỉ tốt bằng đường thẳng cục bộ
- bảo toàn tính liên tục của pipeline khi dữ liệu perception bị thiếu

## 9. Mối liên hệ giữa fitting và smoothing trong hệ thống

Least squares trong hệ thống hiện tại không hoạt động độc lập. Nó nằm trong một chuỗi ổn định hóa gồm ba lớp:

1. lọc waypoint trùng hoặc gần trùng
2. smoothing không gian trên waypoint
3. smoothing theo thời gian trên các điểm neo và trên đại lượng điều khiển

### 9.1. Gộp waypoint trùng

Sau khi centerline được trích ra từ nhiều polygon, hệ thống:

- sắp xếp waypoint theo trục độc lập
- gộp các waypoint có cùng `y` hoặc cùng `x`
- trung bình vị trí nếu sai khác đủ nhỏ

Mục đích là tránh việc một lát cắt sinh nhiều điểm gần trùng làm bias phép fit.

### 9.2. Spatial smoothing

Nếu có ít nhất 3 waypoint, hệ thống làm trơn cục bộ bằng trung bình trượt 3 điểm:

Ví dụ với lane dọc:

$$
x_i^{smooth}=\frac{x_{i-1}+x_i+x_{i+1}}{3}
$$

Điều này làm giảm nhiễu cao tần trước khi fit.

### 9.3. Temporal smoothing trên các điểm neo

Sau khi fit một đa thức thô (`raw_coeffs`), hệ thống không dùng trực tiếp luôn. Thay vào đó, nó:

- lấy mẫu lại trên lưới cố định mỗi `100 mm`
- nội suy giá trị từ đa thức thô
- trộn với giá trị cùng vị trí của frame trước bằng hệ số:

$$
\alpha = 0.25
$$

Công thức smoothing:

$$
z_{smooth} = \alpha z_{raw} + (1-\alpha) z_{prev}
$$

với `z` là:

- `x` nếu đang fit `x(y)`
- `y` nếu đang fit `y(x)`

Sau đó hệ thống fit lại đa thức lần hai trên tập điểm đã được smooth theo thời gian. Đây là một chi tiết rất quan trọng: **đa thức cuối cùng publish ra là đa thức sau smoothing, không phải đa thức raw ban đầu**.

### 9.4. Regenerate waypoints từ đa thức cuối

Khi đã có hệ số cuối, hệ thống tái sinh `waypoints` đều theo bước `100 mm` từ chính đa thức này. Nhờ vậy:

- waypoints đầu ra trơn hơn
- nhất quán với polynomial publish ra
- phù hợp hơn cho control và dashboard

## 10. Ý nghĩa hình học của các hệ số

### 10.1. Với mô hình `x(y)`

Cho:

$$
x(y)=a_3y^3+a_2y^2+a_1y+a_0
$$

tại vùng gần xe, các hệ số có ý nghĩa:

- `a0`: lệch ngang tại `y = 0`
- `a1`: độ dốc cục bộ của lane, dùng để suy ra góc hướng
- `a2`: liên quan đến độ cong
- `a3`: mô tả sự thay đổi của độ cong

Hệ thống hiện tại dùng:

$$
\text{lateral\_offset} = a_0
$$

$$
\text{heading\_angle} = \arctan(a_1)
$$

$$
\text{curvature} = 2a_2
$$

### 10.2. Với mô hình `y(x)`

Tương tự, khi lane được biểu diễn theo:

$$
y(x)=b_3x^3+b_2x^2+b_1x+b_0
$$

thì:

- `b0` là giá trị `y` tại `x = 0`
- `b1` liên hệ với góc tiếp tuyến
- `b2` liên hệ với độ cong cục bộ

Trong code hiện tại, các hệ số này vẫn được ghi vào JSON dưới tên trường:

- `a3`
- `a2`
- `a1`
- `a0`

ngay cả khi về mặt toán học lane đang được fit theo `y(x)`. Đây là quy ước triển khai để thống nhất schema JSON, không có nghĩa biến số độc lập luôn là `y`.

## 11. Liên hệ trực tiếp với điều hướng

Sau bước fit, hệ thống có một mô hình lane liên tục. Từ đó có thể:

- đánh giá vị trí lane tại khoảng nhìn trước tùy ý
- tính sai số lệch ngang
- tính góc hướng của lane
- ước lượng độ cong
- tạo waypoints mượt cho khối điều khiển

Điều này biến least-squares fitting thành bước chuyển đổi từ perception geometry sang control geometry.

Ví dụ, với lane dọc:

$$
x(d_{lookahead})=a_3d^3+a_2d^2+a_1d+a_0
$$

cho phép lấy trực tiếp độ lệch ngang tại khoảng nhìn trước `d`.

## 12. Hạn chế và các điểm cần lưu ý

### 12.1. Nhạy với waypoint ngoại lai

Least squares tối thiểu hóa tổng bình phương sai số nên khá nhạy với outlier. Nếu có waypoint lệch mạnh do segmentation lỗi, đường cong fit có thể bị kéo sai đáng kể.

### 12.2. Không có ràng buộc hình học cứng

Phép fit hiện tại là fit tự do, không ép:

- bề rộng lane danh nghĩa
- tính trơn bậc cao
- liên tục với frame trước ở mức đạo hàm

Hệ thống bù lại bằng smoothing trước và sau fit.

### 12.3. Phụ thuộc mạnh vào chất lượng IPM

Nếu homography sai, waypoint world-space sai thì least squares chỉ fit tốt một dữ liệu sai. Vì vậy chất lượng fitting không thể tách rời chất lượng segmentation và IPM.

### 12.4. Thang giá trị có thể lớn

Do `x`, `y` tính bằng mm và có thể lên đến vài trăm hoặc vài nghìn, các cột `y^3` hoặc `x^3` có độ lớn rất lớn. Đây là một lý do nữa khiến việc giải bằng SVD thay vì công thức nghịch đảo trực tiếp là lựa chọn đúng.

## 13. Kết luận

Khớp đường cong bằng bình phương tối thiểu là bước cốt lõi để chuyển tập waypoint rời rạc của lane thành một mô hình đa thức liên tục, gọn và có thể dùng trực tiếp cho điều hướng. Trong hệ thống AVS, lane dọc được biểu diễn bằng `x(y)`, lane ngang hoặc lane rẽ được biểu diễn bằng `y(x)`, và các hệ số được suy ra bằng cách giải bài toán least squares trên ma trận Vandermonde.

Triển khai hiện tại trong `ipm_transform_node.cpp` đã hiện thực đầy đủ tư tưởng này:

- fit cubic khi có đủ điểm
- fallback tuyến tính khi điểm ít
- giải bằng `cv::solve(..., cv::DECOMP_SVD)` để ổn định số
- kết hợp smoothing không gian và thời gian để giảm rung
- xuất ra polynomial, waypoint và các đại lượng hình học phục vụ control

Do đó, trong báo cáo hệ thống, phần “khớp đường cong bằng bình phương tối thiểu” nên được trình bày như chiếc cầu nối giữa world-space perception và lane model phục vụ điều hướng.
