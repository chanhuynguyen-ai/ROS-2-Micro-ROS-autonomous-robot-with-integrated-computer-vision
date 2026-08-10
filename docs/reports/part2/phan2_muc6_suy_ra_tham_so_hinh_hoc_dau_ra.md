# PHẦN II.6. Suy ra các tham số hình học đầu ra

Tài liệu này triển khai chi tiết cho mục `Phần II - Mục 6` trong [../avs_system_report_outline.md](/home/goln/SimpleSysIDV/../avs_system_report_outline.md). Mục tiêu là diễn giải rõ cách suy ra ba đại lượng đầu ra hình học của hệ thống:

- `epsilon_x_mm`
- `epsilon_y_mm`
- `theta_rad`

Phần trình bày dưới đây không dừng ở mức công thức tổng quát, mà bám theo đúng pipeline hiện có:

1. polygon làn trong ảnh
2. homography sang hệ tọa độ thực
3. trích centerline
4. fit đa thức
5. chọn điểm look-ahead hoặc target point
6. quy đổi thành `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`

Ngoài mô hình lý thuyết, tài liệu cũng chỉ ra cách hệ thống đang hiện thực trong [ipm_transform_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ipm_transform_node.cpp:723) và [control_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/control_node.cpp:1541).

## 1. Ý nghĩa của bài toán

Sau khi lane được phân đoạn và đưa về hệ tọa độ thực, hệ thống vẫn chưa thể publish trực tiếp cho lớp điều khiển. Bộ điều khiển không cần toàn bộ polygon hay toàn bộ centerline, mà cần một số ít đại lượng hình học cô đọng, ổn định và có ý nghĩa vật lý.

Ba đại lượng đó là:

- `epsilon_x_mm`: sai số lệch ngang của mục tiêu hình học so với tâm xe
- `epsilon_y_mm`: khoảng nhìn trước dọc theo hệ trục xe
- `theta_rad`: sai số góc hướng của mục tiêu hoặc của quỹ đạo so với trục tiến của xe

Về bản chất, đây là bước nén biểu diễn:

$$
\text{lane geometry} \longrightarrow (e_x, e_y, \theta)
$$

Nếu bước này ổn định, phần điều hướng phía sau có thể hoạt động mà không cần biết lane được biểu diễn ban đầu bằng mask, polygon hay đa thức.

## 2. Hệ tọa độ dùng để định nghĩa đầu ra

Toàn bộ các đại lượng đầu ra được định nghĩa trong hệ tọa độ gắn với xe trên mặt phẳng đường:

- gốc tọa độ `O` nằm tại vị trí tham chiếu của xe trên mặt đường
- trục `X` dương sang phải
- trục `Y` dương về phía trước

Với một điểm mục tiêu bất kỳ:

$$
P_t = (X_t, Y_t)
$$

thì:

- `X_t > 0`: mục tiêu ở bên phải xe
- `X_t < 0`: mục tiêu ở bên trái xe
- `Y_t > 0`: mục tiêu nằm phía trước xe

Trong hệ trục này, ba đầu ra hình học được hiểu trực quan như sau:

$$
e_x = X_t,\qquad e_y = Y_t,\qquad \theta = \text{góc lệch của hướng mục tiêu}
$$

Điểm quan trọng là mọi giá trị đều mang đơn vị vật lý thực:

- `e_x`, `e_y` tính bằng `mm`
- `theta` tính bằng `rad`

## 3. Nguồn gốc của điểm mục tiêu hình học

Trong hệ thống hiện tại, điểm mục tiêu không đi thẳng từ ảnh. Nó được sinh ra qua các bước trung gian:

1. lane polygon được biến đổi sang world frame bằng homography
2. polygon được rút thành `waypoints` trên centerline
3. `waypoints` được fit thành đa thức bậc 3
4. từ đa thức hoặc từ quỹ đạo rời rạc, hệ thống chọn một điểm nhìn trước `look-ahead`

Do đó, bản chất của `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad` là các đại lượng đo trên:

- một điểm look-ahead
- hoặc một target point nội suy trên active trajectory

Đây là lý do cần tách rõ hai lớp:

- lớp `ipm_transform_node`: sinh lane geometry sơ cấp
- lớp `control_node`: chọn trajectory đang hoạt động và publish đầu ra cuối cùng

## 4. Suy luận cho lane dọc: `main-lane` và `other-lane`

### 4.1. Mô hình hình học

Với lane dọc, hệ thống biểu diễn centerline bằng đa thức:

$$
x(y)=a_3 y^3+a_2 y^2+a_1 y+a_0
$$

trong đó:

- `y` là khoảng cách tiến về phía trước
- `x(y)` là độ lệch ngang của centerline tại vị trí đó

Sau khi fit xong, node IPM ghi các hệ số vào trường `polynomial.a3..a0`, đồng thời dùng chính đa thức này để sinh các `waypoints` mượt.

### 4.2. Sai số lệch ngang gần xe

Tại vị trí gốc xe `y = 0`, ta có:

$$
x(0)=a_0
$$

Vì vậy:

$$
e_{x,0}=a_0
$$

Đây là sai số lệch ngang cục bộ tại vùng gần xe. Trong code hiện tại, đại lượng này được lưu dưới tên:

- `lateral_offset_mm`

và được tính trực tiếp từ:

$$
\text{lateral\_offset\_mm}=a_0
$$

theo logic tại [ipm_transform_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ipm_transform_node.cpp:858).

### 4.3. Góc hướng gần xe

Đạo hàm của đa thức lane dọc là:

$$
\frac{dx}{dy}=3a_3 y^2+2a_2 y+a_1
$$

Tại gốc xe `y = 0`:

$$
\left.\frac{dx}{dy}\right|_{y=0}=a_1
$$

Do đó, góc tiếp tuyến cục bộ gần xe là:

$$
\theta_0=\arctan(a_1)
$$

Trong code, giá trị này được lưu dưới tên:

- `heading_angle_rad`

theo phép tính:

$$
\text{heading\_angle\_rad}=\arctan(a_1)
$$

ở [ipm_transform_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ipm_transform_node.cpp:859).

### 4.4. Độ cong cục bộ

Nếu xét gần gốc và giả thiết góc nhỏ, thành phần bậc hai của đa thức chi phối độ cong sơ bộ:

$$
\frac{d^2x}{dy^2}=6a_3 y + 2a_2
$$

tại `y = 0`:

$$
\left.\frac{d^2x}{dy^2}\right|_{y=0}=2a_2
$$

Hệ thống hiện tại xuất xấp xỉ này dưới trường:

$$
\text{curvature\_inv\_mm} \approx 2a_2
$$

Đây chưa phải nội dung chính của mục 6, nhưng nó cho thấy `e_x`, `e_y`, `theta` được suy ra cùng một nền hình học thống nhất.

## 5. Chọn điểm look-ahead cho lane dọc

### 5.1. Điểm tham chiếu không lấy ngay tại gốc

Nếu chỉ dùng `a_0` và `a_1`, đầu ra sẽ quá nhạy với nhiễu ở vùng rất gần xe. Vì vậy, hệ thống chọn một khoảng nhìn trước:

$$
d_{la} > 0
$$

và đánh giá hình học tại đó.

Trong `ipm_transform_node`, giá trị này được lấy bởi:

$$
d_{la}=\text{compute\_lookahead\_d()}
$$

rồi tính điểm look-ahead:

$$
x_{la}=x(d_{la})=a_3 d_{la}^3+a_2 d_{la}^2+a_1 d_{la}+a_0
$$

theo [ipm_transform_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ipm_transform_node.cpp:878).

### 5.2. Suy ra đầu ra hình học từ look-ahead

Từ điểm:

$$
P_{la}=(x_{la}, d_{la})
$$

ta có thể định nghĩa trực tiếp:

$$
e_x = x_{la}
$$

$$
e_y = d_{la}
$$

và góc nối từ gốc xe tới điểm đích:

$$
\theta = \operatorname{atan2}(x_{la}, d_{la})
$$

Đây chính là cách mà node IPM đang ghi vào telemetry:

- `lookahead_x_mm = x_la`
- `lookahead_d_mm = d_la`
- `lookahead_theta_rad = atan2(x_la, d_la)`

theo [ipm_transform_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ipm_transform_node.cpp:878).

### 5.3. Ý nghĩa vật lý

Với định nghĩa trên:

- `e_x` là độ lệch ngang của điểm đích nhìn trước
- `e_y` là tầm nhìn trước trên trục dọc
- `theta` là góc ngắm từ tâm xe đến điểm đích

Điểm cần lưu ý là:

$$
\theta=\operatorname{atan2}(e_x,e_y)
$$

không hoàn toàn giống với góc tiếp tuyến cục bộ:

$$
\theta_0=\arctan\left(\frac{dx}{dy}\Big|_{y=0}\right)=\arctan(a_1)
$$

Hai đại lượng này chỉ gần nhau khi:

- lane đủ mượt
- độ cong nhỏ
- look-ahead không quá xa

Vì vậy, trong báo cáo nên nêu rõ hệ thống hiện tại ưu tiên **góc ngắm đến điểm look-ahead** cho lane dọc, thay vì chỉ dùng đạo hàm tại gốc.

## 6. Suy luận cho lane rẽ: `turn-lane`

### 6.1. Mô hình hình học

Với lane rẽ, hệ thống không fit `x(y)` mà fit:

$$
y(x)=b_3 x^3+b_2 x^2+b_1 x+b_0
$$

Lý do là hình học lane rẽ thường thuận tiện hơn khi xem `y` là hàm của `x`, đặc biệt khi đường rẽ trải ngang theo phương `X`.

### 6.2. Đại lượng gốc của lane rẽ

Tại `x = 0`, ta có:

$$
y(0)=b_0
$$

Giá trị này được hiểu là độ tiến dọc của tâm lane khi cắt qua trục dọc của xe. Trong code hiện tại, nó được ghi dưới tên:

- `longitudinal_offset_mm`

theo phép gán:

$$
\text{longitudinal\_offset\_mm}=b_0
$$

tại [ipm_transform_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ipm_transform_node.cpp:723).

### 6.3. Góc hướng của lane rẽ

Đạo hàm của lane rẽ là:

$$
\frac{dy}{dx}=3b_3 x^2+2b_2 x+b_1
$$

tại `x = 0`:

$$
\left.\frac{dy}{dx}\right|_{x=0}=b_1
$$

Nếu cần biểu diễn góc tiếp tuyến theo quy ước hệ trục xe với trục chuẩn là `+Y`, một cách gần đúng đang được dùng là:

$$
\theta_t=\arctan(b_1)
$$

và giá trị này được xuất ra:

- `heading_angle_rad`
- `lookahead_theta_rad`

theo [ipm_transform_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/ipm_transform_node.cpp:723).

### 6.4. Suy ra `e_x`, `e_y`, `theta` cho lane rẽ

Trong triển khai hiện tại, lane rẽ không sinh `lookahead_x_mm` theo đa thức như lane dọc. Node IPM chọn:

$$
e_x = 0
$$

$$
e_y = b_0
$$

$$
\theta = \arctan(b_1)
$$

và đóng gói tạm dưới dạng:

- `lookahead_x_mm = 0`
- `longitudinal_offset_mm = b_0`
- `lookahead_theta_rad = atan(b_1)`

Về mặt hình học, đây là một chuẩn hóa hợp lý cho lane rẽ vì mục tiêu tham chiếu được quy về giao điểm giữa centerline rẽ và trục dọc của xe.

## 7. Từ lane geometry sang đầu ra cuối trong `control_node`

### 7.1. Tầng publish cuối không dùng trực tiếp đa thức

Node publish `/avs/control_error` không bắt buộc đọc lại đa thức để tính lỗi. Thay vào đó, nó làm việc với:

- `active trajectory`
- hoặc các giá trị precomputed từ IPM

Nếu trajectory có danh sách điểm hợp lệ, `control_node` sẽ nội suy một target point trên quỹ đạo tại khoảng cách look-ahead. Logic này nằm tại [control_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/control_node.cpp:1541).

### 7.2. Công thức nội suy trên active trajectory

Giả sử active trajectory là dãy điểm:

$$
\{P_1, P_2, \dots, P_N\}, \qquad P_i=(x_i,y_i)
$$

Node thêm gốc xe:

$$
P_0=(0,0)
$$

Sau đó tính tổng chiều dài cung dọc theo polyline. Khi tổng chiều dài tích lũy lần đầu vượt `d_la`, node nội suy tuyến tính trong đoạn hiện tại để nhận điểm đích:

$$
P_t=(x_t,y_t)
$$

Khi đó đầu ra cuối được publish là:

$$
\epsilon_x = x_t
$$

$$
\epsilon_y = y_t
$$

$$
\theta = \operatorname{atan2}(x_t,y_t)
$$

theo [control_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/control_node.cpp:1579) và [control_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/control_node.cpp:3439).

### 7.3. Trường hợp dùng giá trị precomputed

Nếu trajectory chưa đủ điểm nhưng object đã có sẵn các trường:

- `lookahead_x_mm`
- `lookahead_d_mm`
- `lookahead_theta_rad`

thì `control_node` dùng luôn các giá trị đó làm:

$$
\epsilon_x = \text{lookahead\_x\_mm}
$$

$$
\epsilon_y = \text{lookahead\_d\_mm}
$$

$$
\theta = \text{lookahead\_theta\_rad}
$$

Nếu là turn-lane dạng precomputed, node dùng:

$$
\epsilon_x = 0,\qquad \epsilon_y = \text{longitudinal\_offset\_mm},\qquad \theta=\text{lookahead\_theta\_rad}
$$

theo [control_node.cpp](/home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/src/control_node.cpp:3445).

## 8. Tổng hợp công thức nên đưa vào báo cáo chính

Để phần `2.6` trong báo cáo tổng dễ đọc nhưng vẫn đúng với triển khai, nên trình bày theo hai lớp sau.

### 8.1. Công thức tổng quát

Với điểm mục tiêu hình học trong hệ trục xe:

$$
P_t=(x_t,y_t)
$$

ta định nghĩa:

$$
e_x=x_t
$$

$$
e_y=y_t
$$

$$
\theta=\operatorname{atan2}(x_t,y_t)
$$

Đây là định nghĩa thống nhất nhất, vì áp dụng được cho:

- điểm lấy từ đa thức lane dọc
- điểm lấy từ lane rẽ đã chuẩn hóa
- điểm nội suy trên active trajectory

### 8.2. Suy ra từ đa thức lane dọc

Nếu lane được mô tả bởi:

$$
x(y)=a_3 y^3+a_2 y^2+a_1 y+a_0
$$

và chọn khoảng nhìn trước `d_la`, thì:

$$
e_x = x(d_{la}) = a_3 d_{la}^3+a_2 d_{la}^2+a_1 d_{la}+a_0
$$

$$
e_y = d_{la}
$$

$$
\theta = \operatorname{atan2}(e_x,e_y)
$$

Trong trường hợp chỉ xét rất gần xe:

$$
e_x \approx a_0,\qquad \theta \approx \arctan(a_1)
$$

### 8.3. Suy ra từ đa thức lane rẽ

Nếu lane rẽ được mô tả bởi:

$$
y(x)=b_3 x^3+b_2 x^2+b_1 x+b_0
$$

và chọn chuẩn hóa theo giao điểm với trục xe `x = 0`, thì:

$$
e_x = 0
$$

$$
e_y = y(0)=b_0
$$

$$
\theta \approx \arctan(b_1)
$$

Đây chính là dạng mà triển khai hiện tại đang sử dụng.

## 9. Nhận xét kỹ thuật quan trọng

### 9.1. `theta` hiện tại là góc nhìn trước, không thuần là góc tiếp tuyến

Đối với lane dọc, telemetry IPM đang tính:

$$
\theta=\operatorname{atan2}(x_{la},d_{la})
$$

chứ không dùng trực tiếp:

$$
\arctan\left(\frac{dx}{dy}\right)
$$

Điều này giúp đầu ra phản ánh trực tiếp hướng cần lái tới điểm mục tiêu, nhưng cũng có nghĩa là `theta_rad` phụ thuộc vào lựa chọn `lookahead_d_mm`.

### 9.2. `epsilon_y_mm` là khoảng mục tiêu hình học, không phải sai số dọc theo nghĩa điều khiển bám vị trí

Trong hệ thống này:

- `epsilon_y_mm` chủ yếu đóng vai trò khoảng nhìn trước
- không phải một sai số cần triệt tiêu về `0`

Do đó, khi viết báo cáo nên giải thích đây là **thành phần dọc của điểm mục tiêu**, không nên mô tả như “lỗi dọc” theo nghĩa bám đích cuối.

### 9.3. Tầng IPM và tầng control không mâu thuẫn nhau

Hai tầng đang làm hai việc khác nhau:

- `ipm_transform_node` suy ra lane geometry sơ cấp từ một lane object
- `control_node` chọn target cuối trên trajectory đang active

Vì vậy, tài liệu nên nêu rõ:

- phần lý thuyết của mục 6 có thể bắt đầu từ điểm mục tiêu tổng quát `P_t`
- phần triển khai giải thích `P_t` được sinh ra từ đa thức hoặc từ active trajectory

## 10. Kết luận

Ba đầu ra `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad` của hệ thống AVS thực chất là một biểu diễn tối giản của mục tiêu hình học trong hệ tọa độ gắn với xe.

Với lane dọc, hệ thống suy ra các đại lượng này bằng cách fit đa thức `x(y)`, chọn khoảng nhìn trước `d_la`, rồi tính:

$$
e_x=x(d_{la}),\qquad e_y=d_{la},\qquad \theta=\operatorname{atan2}(x(d_{la}), d_{la})
$$

Với lane rẽ, hệ thống dùng đa thức `y(x)` và chuẩn hóa theo trục xe:

$$
e_x=0,\qquad e_y=b_0,\qquad \theta\approx\arctan(b_1)
$$

Ở tầng publish cuối, `control_node` tổng quát hóa bước này bằng cách nội suy trực tiếp một target point trên active trajectory rồi ánh xạ:

$$
(x_t,y_t)\longrightarrow (\epsilon_x,\epsilon_y,\theta)
$$

với:

$$
\epsilon_x=x_t,\qquad \epsilon_y=y_t,\qquad \theta=\operatorname{atan2}(x_t,y_t)
$$

Đây là mô hình diễn giải gọn nhất nhưng vẫn bám sát mã nguồn hiện tại và phù hợp để đưa vào báo cáo chính thức.
