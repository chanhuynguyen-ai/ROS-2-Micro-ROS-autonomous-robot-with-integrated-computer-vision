# PHẦN II.4. Trích xuất centerline từ polygon làn

## 1. Mở đầu

Trong hệ thống AVS, mô hình segmentation không sinh ra trực tiếp một đường centerline, mà sinh ra một **vùng làn** dưới dạng mask, sau đó được chuyển thành contour và polygon. Tuy nhiên, lớp điều hướng phía sau không làm việc trực tiếp với một vùng diện tích. Điều nó cần là một biểu diễn dạng đường, ổn định theo thời gian, phản ánh được trục chuyển động trung tâm của làn.

Vì vậy, bài toán trung gian cần giải là:

- đầu vào: một hoặc nhiều polygon biểu diễn vùng làn
- đầu ra: một tập waypoint nằm trên trục giữa của vùng đó

Tập waypoint này chính là **centerline rời rạc**, dùng cho các bước:

- fit đa thức hình học
- suy ra độ lệch ngang và góc hướng
- tính điểm look-ahead
- lựa chọn trajectory trong node điều hướng

Trong code hiện tại, bước này được thực hiện ở `ipm_transform_node.cpp` sau khi polygon đã được biến đổi từ hệ pixel sang hệ tọa độ thực bằng homography. Do đó, toàn bộ centerline được trích xuất trực tiếp trong **world frame** với đơn vị `mm`, thay vì trên ảnh gốc.

## 2. Vì sao cần centerline thay vì polygon

Polygon chứa thông tin đầy đủ hơn centerline, nhưng cũng cồng kềnh hơn và khó dùng trực tiếp cho điều hướng. Một vùng làn dù chính xác đến đâu vẫn chưa trả lời được ngay các câu hỏi:

- xe đang lệch sang trái hay phải bao nhiêu
- hướng tiếp tuyến của làn tại vùng gần xe là gì
- độ cong cục bộ của làn thay đổi ra sao
- nên chọn điểm nhìn trước nào để sinh lệnh điều khiển

Ngược lại, nếu mỗi làn được rút gọn thành một đường trung tâm:

- vị trí hình học trở nên rõ ràng
- việc fit đa thức trở nên trực tiếp
- có thể nội suy và làm mượt theo thời gian
- dễ tính sai số `e_x`, `e_y`, `theta`

Nói cách khác, polygon là biểu diễn tốt cho perception, còn centerline là biểu diễn phù hợp cho geometry và planning cục bộ.

## 3. Đầu vào thực tế của bước trích centerline

Pipeline hiện tại có chuỗi xử lý:

$$
\text{segmentation} \rightarrow \text{mask} \rightarrow \text{contour/polygon} \rightarrow \text{homography} \rightarrow \text{centerline}
$$

Trong `ncnn_inference_node`, mỗi object lane được publish với trường:

- `polygons`

là danh sách các polygon trong hệ pixel.

Sau đó `ipm_transform_node` đọc từng điểm `[u, v]`, áp dụng homography và tạo:

- `polygons_real_world`

Mỗi polygon lúc này là một tập điểm:

$$
\mathcal{P} = \{(X_i, Y_i)\}_{i=1}^{N}
$$

trong hệ tọa độ gắn với xe:

- `X`: ngang, dương sang phải
- `Y`: dọc, dương về phía trước

Việc trích centerline được thực hiện trên tập điểm này. Đây là quyết định quan trọng vì:

- khoảng cách giữa các lát cắt có thể đặt theo `mm`
- bề rộng làn được đo bằng đại lượng vật lý thật
- thuật toán ít phụ thuộc hơn vào phối cảnh ảnh

## 4. Biểu diễn vùng làn bằng polygon

### 4.1. Polygon như một đường bao kín

Một lane sau segmentation thường được biểu diễn bởi contour ngoài, sau đó chuyển thành polygon. Về hình học, có thể xem polygon là một đường bao kín:

$$
\mathcal{P} = (P_1, P_2, \dots, P_N), \quad P_i = (x_i, y_i)
$$

trong đó cạnh thứ `i` nối:

$$
P_i \rightarrow P_{i+1}
$$

và cạnh cuối nối:

$$
P_N \rightarrow P_1
$$

Toàn bộ miền nằm bên trong đường bao này đại diện cho diện tích lane mà segmentation tin rằng thuộc cùng một thực thể.

### 4.2. Từ vùng diện tích sang trục giữa

Nếu lane đủ “ống” và không tự cắt, thì tại mỗi lát cắt vuông góc với hướng quét, vùng lane thường giao với lát cắt đó tại hai biên:

- biên trái và biên phải đối với lane dọc
- biên dưới và biên trên đối với lane rẽ/ngang

Khi đó điểm giữa của hai biên là ước lượng tự nhiên cho centerline tại vị trí lát cắt đó.

Đây chính là ý tưởng cốt lõi của phương pháp midpoint extraction: thay vì tìm skeleton toàn cục của mask, hệ thống lấy các giao điểm với một họ lát cắt song song, rồi tính trung điểm của từng lát.

## 5. Phân loại hình học lane trong hệ thống

Repo hiện tại xử lý ba lớp lane chính:

- `main-lane` với `label = 3`
- `other-lane` với `label = 4`
- `turn-lane` với `label = 10`

Hai nhóm đầu thường kéo dài theo trục tiến `Y`, nên centerline thuận tiện được biểu diễn dưới dạng:

$$
x = x(y)
$$

Ngược lại, `turn-lane` có thể nằm ngang hoặc cong mạnh theo phương ngang, nên biểu diễn phù hợp hơn là:

$$
y = y(x)
$$

Do đó, hệ thống tách thành hai chiến lược quét:

- quét theo `Y` cho lane dọc
- quét theo `X` cho lane rẽ

Đây là quyết định quan trọng để tránh trường hợp một hàm hình học không còn đơn trị theo biến độc lập được chọn.

## 6. Kỹ thuật quét lát cắt cho làn dọc

### 6.1. Miền quét

Với một polygon lane dọc trong world frame, thuật toán trước hết tìm:

$$
Y_{min} = \min_i Y_i, \qquad Y_{max} = \max_i Y_i
$$

Sau đó tạo các mức quét:

$$
Y_k = Y_{start} + k \cdot \Delta Y
$$

với:

- `\Delta Y = 100 mm` trong code hiện tại
- `Y_start` là bội số đầu tiên của `100 mm` lớn hơn hoặc bằng `Y_min`

Tập mức quét rời rạc này giúp centerline có spacing đều theo không gian thực.

### 6.2. Giao điểm của lát cắt với polygon

Xét lát cắt ngang:

$$
y = Y_k
$$

Với mỗi cạnh polygon nối:

$$
(x_1, y_1) \rightarrow (x_2, y_2)
$$

nếu lát cắt đi qua khoảng tung độ của cạnh, tức là:

$$
(y_1 \le Y_k \le y_2) \quad \text{hoặc} \quad (y_2 \le Y_k \le y_1)
$$

thì tọa độ giao điểm theo phép nội suy tuyến tính là:

$$
x_{int} = x_1 + \frac{(Y_k - y_1)(x_2 - x_1)}{y_2 - y_1}
$$

Thuật toán lặp qua toàn bộ các cạnh và thu được tập các giao điểm:

$$
\{x_{int}^{(1)}, x_{int}^{(2)}, \dots, x_{int}^{(m)}\}
$$

### 6.3. Chọn biên trái và biên phải

Trong cách triển khai hiện tại, thay vì ghép cặp toàn bộ giao điểm, node lấy trực tiếp:

$$
x_{left}(Y_k) = \min_j x_{int}^{(j)}
$$

$$
x_{right}(Y_k) = \max_j x_{int}^{(j)}
$$

Từ đó bề rộng lát cắt là:

$$
w(Y_k) = x_{right}(Y_k) - x_{left}(Y_k)
$$

và tâm lát cắt:

$$
x_{mid}(Y_k) = \frac{x_{left}(Y_k) + x_{right}(Y_k)}{2}
$$

Điểm waypoint thu được là:

$$
W_k = \big(x_{mid}(Y_k),\, Y_k\big)
$$

### 6.4. Ý nghĩa của phép lấy min-max

Với polygon lane đơn liên thông và tương đối “đầy”, việc lấy giao điểm trái nhất và phải nhất hoạt động tốt vì:

- nó giữ đúng hai biên ngoài của lane
- không phụ thuộc vào thứ tự các điểm contour
- đơn giản hơn nhiều so với skeletonization

Tuy nhiên, nếu polygon có các nhánh phụ, phần phình hoặc nhiễu lồi cục bộ, thì:

- `x_left`
- `x_right`

có thể bị kéo ra xa khỏi bề rộng lane thực. Đây chính là lý do cần cơ chế lọc lát cắt bị phình ở các bước sau.

## 7. Kỹ thuật quét lát cắt cho làn ngang hoặc lane rẽ

### 7.1. Lý do phải đổi trục quét

Với `turn-lane`, nếu vẫn quét theo `Y` thì nhiều lát cắt có thể:

- cắt lane ở rất ít điểm
- không cắt liên tục
- hoặc một giá trị `Y` tương ứng với nhiều giá trị `X`

Do đó node đổi sang quét dọc theo trục `X`.

### 7.2. Miền quét

Từ polygon lane rẽ, hệ thống tìm:

$$
X_{min} = \min_i X_i, \qquad X_{max} = \max_i X_i
$$

và sinh các lát cắt:

$$
X_k = X_{start} + k \cdot \Delta X
$$

với:

- `\Delta X = 100 mm`

### 7.3. Công thức giao điểm

Xét đường thẳng dọc:

$$
x = X_k
$$

Với cạnh:

$$
(x_1, y_1) \rightarrow (x_2, y_2)
$$

nếu:

$$
(x_1 \le X_k \le x_2) \quad \text{hoặc} \quad (x_2 \le X_k \le x_1)
$$

thì giao điểm có tung độ:

$$
y_{int} = y_1 + \frac{(X_k - x_1)(y_2 - y_1)}{x_2 - x_1}
$$

Từ các giao điểm, node chọn:

$$
y_{bottom}(X_k) = \min_j y_{int}^{(j)}
$$

$$
y_{top}(X_k) = \max_j y_{int}^{(j)}
$$

Bề rộng theo trục quét:

$$
w(X_k) = y_{top}(X_k) - y_{bottom}(X_k)
$$

Tâm lát cắt:

$$
y_{mid}(X_k) = \frac{y_{bottom}(X_k) + y_{top}(X_k)}{2}
$$

Waypoint tương ứng:

$$
W_k = \big(X_k,\, y_{mid}(X_k)\big)
$$

### 7.4. Kết quả hình học

Tập điểm thu được mô tả một centerline rời rạc theo biến `X`, là cơ sở để fit đa thức:

$$
y(x)=b_3x^3+b_2x^2+b_1x+b_0
$$

đúng với bản chất của lane rẽ trong hệ tọa độ thực.

## 8. Phát hiện lát cắt bị phình và nhiễu

### 8.1. Nguồn gốc của hiện tượng phình

Sau segmentation và homography, polygon có thể bị méo vì nhiều nguyên nhân:

- mask bị nở cục bộ do nhiễu dự đoán
- biên lane bị kéo dính sang đối tượng lân cận
- contour có các răng cưa nhỏ
- phép biến đổi homography khuếch đại sai lệch pixel ở vùng xa

Khi đó, một số lát cắt cho bề rộng lớn bất thường so với phần còn lại của lane. Nếu vẫn lấy trung điểm trực tiếp, centerline sẽ bị kéo lệch đáng kể.

### 8.2. Bề rộng trung vị như một tham số chuẩn

Thuật toán thu bề rộng của toàn bộ lát cắt:

$$
\{w_1, w_2, \dots, w_n\}
$$

rồi sắp xếp để lấy trung vị:

$$
w_{median} = \operatorname{median}(w_1, w_2, \dots, w_n)
$$

Trung vị được chọn thay vì trung bình vì:

- ít nhạy với các lát bị phình cực đoan
- phản ánh tốt “bề rộng điển hình” của lane

Trong code hiện tại, nếu:

$$
w_{median} < 10 \text{ mm}
$$

thì hệ thống ép:

$$
w_{median} = 400 \text{ mm}
$$

để tránh các tình huống suy biến do polygon quá nhỏ hoặc lỗi dữ liệu.

### 8.3. Tiêu chuẩn phát hiện slice bất thường

Một lát cắt bị xem là phình nếu:

$$
w_i > 1.3 \, w_{median}
$$

Ngưỡng `1.3` hiện là heuristic thực dụng:

- đủ nhạy để bắt các trường hợp nở rõ rệt
- nhưng chưa quá chặt để loại bỏ mọi biến thiên tự nhiên của lane cong

Kết quả là mỗi slice được gắn nhãn:

- sạch (`clean`)
- bị phình (`bloated`)

## 9. Ước lượng centerline cho slice bị phình

### 9.1. Vấn đề của trung điểm thuần túy

Nếu một biên lane bị “rò” ra ngoài do segmentation, trung điểm:

$$
\frac{x_{left}+x_{right}}{2}
$$

hoặc:

$$
\frac{y_{bottom}+y_{top}}{2}
$$

sẽ dịch chuyển khỏi centerline thật dù biên còn lại vẫn đúng. Vì vậy, hệ thống không tin hoàn toàn vào midpoint của các lát bị phình.

### 9.2. Local sliding window

Đối với mỗi slice bị phình, node quan sát một cửa sổ lân cận cỡ:

- `±3` slices

và thu các midpoint của các slice sạch trong vùng đó. Nếu tồn tại đủ slice sạch, hệ thống lấy **trung vị cục bộ** của các midpoint này làm ước lượng tâm tham chiếu:

$$
c_{local} = \operatorname{median}\big(\text{clean mids in neighborhood}\big)
$$

Ưu điểm:

- giữ được xu hướng cong cục bộ của lane
- ít bị ảnh hưởng bởi một vài slice nhiễu

Đây là một lựa chọn hợp lý hơn dùng trung bình, vì median trong cửa sổ cục bộ bền hơn với outlier.

### 9.3. Global linear trend làm phương án dự phòng

Nếu toàn bộ vùng lân cận đều bị phình, node dựng một xu thế tuyến tính toàn cục trên các slice sạch.

Với lane dọc, xu thế có dạng:

$$
x = m y + c
$$

Với lane rẽ:

$$
y = m x + c
$$

Các tham số được tính theo bình phương tối thiểu từ tập midpoint sạch. Xu thế này không nhằm mô tả chính xác độ cong toàn bộ lane, mà chỉ làm **fallback** để cung cấp một ước lượng center gần hợp lý khi local neighborhood không đủ tin cậy.

### 9.4. Cắt lại biên theo bề rộng danh định

Sau khi có tâm tham chiếu `c`, node so sánh biên thực tế của slice với biên kỳ vọng:

- `c - w_median/2`
- `c + w_median/2`

Từ đó đánh giá xem bên nào bị rò nhiều hơn. Nếu độ lệch hai phía gần tương đương, thuật toán giả định slice bị nở đối xứng và đặt tâm đúng tại `c`. Nếu một phía lệch nhiều hơn, nó “clip” phía đó về gần bề rộng danh định rồi tính lại midpoint.

Ý tưởng trực giác là:

- nếu lane nở đều hai bên, giữ nguyên tâm tham chiếu
- nếu chỉ một biên bị sai, tin biên còn lại nhiều hơn và hiệu chỉnh tâm về phía nó

Đây là một cơ chế đơn giản nhưng rất phù hợp với dữ liệu lane thực tế, nơi lỗi segmentation thường làm trôi một biên nhiều hơn biên còn lại.

## 10. Hợp nhất waypoint từ nhiều polygon

### 10.1. Nhu cầu hợp nhất

Một object lane có thể chứa nhiều polygon trong `polygons_real_world`, ví dụ:

- lane bị đứt đoạn
- contour ngoài sinh ra nhiều thành phần
- có nhiễu làm lane tách thành các mảnh

Node hiện tại xử lý từng polygon, trích waypoint cho từng polygon, rồi gộp toàn bộ lại vào `all_waypoints`.

### 10.2. Sắp xếp và loại trùng

Với lane dọc, waypoint được sắp theo `Y` tăng dần. Với lane rẽ, waypoint được sắp theo `X` tăng dần.

Nếu hai waypoint có cùng mức quét:

- cùng `Y` với lane dọc
- cùng `X` với lane rẽ

thì hệ thống không giữ nguyên cả hai một cách mù quáng. Nó kiểm tra độ lệch theo trục còn lại:

- nếu chênh lệch nhỏ hơn `300 mm`, lấy trung bình để gộp
- nếu chênh lệch quá lớn, giữ thành hai điểm riêng

Điều này giúp:

- giảm trùng lặp khi nhiều polygon cùng đại diện cho một lane
- nhưng vẫn giữ khả năng biểu diễn khi có hai nhánh hình học thực sự khác nhau

## 11. Làm mượt không gian cho centerline rời rạc

Sau khi có `unique_waypoints`, node áp dụng làm mượt không gian cục bộ bằng trung bình trượt bậc ba.

Với lane dọc:

$$
x_i^{smooth} = \frac{x_{i-1}+x_i+x_{i+1}}{3}
$$

Với lane rẽ:

$$
y_i^{smooth} = \frac{y_{i-1}+y_i+y_{i+1}}{3}
$$

Phép làm mượt này có vai trò:

- giảm rung cục bộ do biên polygon lởm chởm
- làm centerline đều hơn trước khi fit đa thức
- vẫn giữ được cấu trúc cong chậm của lane

Vì chỉ làm mượt trên láng giềng gần, thuật toán không làm biến mất xu hướng hình học chính như một bộ lọc quá mạnh.

## 12. Quan hệ giữa centerline và fit đa thức

Centerline rời rạc chưa phải đầu ra cuối. Nó là dữ liệu quan sát cho bước xấp xỉ hình học:

- lane dọc: fit `x(y)`
- lane rẽ: fit `y(x)`

Điều này rất quan trọng vì:

- centerline thô có thể thiếu ở một vài lát
- khoảng cách giữa các điểm có thể không phủ kín toàn miền
- phép fit giúp tạo một biểu diễn liên tục, khả vi và dễ nội suy

Do đó, centerline extraction là bước cầu nối giữa biểu diễn hình học rời rạc của perception và biểu diễn hàm liên tục dùng trong điều hướng.

## 13. Ưu điểm của phương pháp lát cắt so với skeletonization

Có nhiều cách lấy centerline từ một vùng, ví dụ:

- morphological skeleton
- medial axis transform
- distance transform rồi lần theo trục xương

Tuy nhiên, trong hệ AVS, phương pháp quét lát cắt có nhiều lợi thế thực dụng:

- dễ cài đặt trên polygon world-space
- không cần raster hóa lại mask ở độ phân giải mới
- dễ kiểm soát theo đơn vị `mm`
- trực tiếp tạo waypoint theo spacing cố định
- dễ kết hợp với heuristic lọc bề rộng

Skeletonization thường mạnh với hình dạng bất quy tắc phức tạp, nhưng cũng dễ tạo nhánh phụ khi vùng bị nhiễu. Với lane trong môi trường có cấu trúc, cách quét lát cắt là lựa chọn đơn giản hơn và ổn định hơn cho mục tiêu điều hướng.

## 14. Hạn chế của phương pháp hiện tại

Mặc dù hiệu quả, phương pháp này vẫn có các giới hạn bản chất.

### 14.1. Phụ thuộc vào chất lượng polygon

Nếu polygon:

- bị đứt quá mạnh
- tự cắt
- có lỗ lớn
- hoặc dính với vùng khác

thì giao điểm của lát cắt có thể không còn phản ánh đúng hai biên lane.

### 14.2. Giả định lane đủ “dày”

Phương pháp trung điểm ngầm giả định rằng mỗi lát cắt cắt qua một bề rộng lane đủ rõ. Nếu polygon rất mảnh hoặc méo mạnh, bề rộng đo được trở nên kém ổn định.

### 14.3. Heuristic vẫn là heuristic

Các ngưỡng như:

- `step = 100 mm`
- `1.3 * w_median`
- cửa sổ `±3`
- ngưỡng gộp `300 mm`

là các lựa chọn thực nghiệm. Chúng hợp lý với bối cảnh hiện tại, nhưng không phải định luật tổng quát cho mọi loại đường hay mọi camera setup.

### 14.4. Xu thế toàn cục tuyến tính chỉ là fallback thô

Khi toàn bộ vùng lân cận đều nhiễu, việc dùng:

$$
x = my + c \quad \text{hoặc} \quad y = mx + c
$$

chỉ cho một xấp xỉ tuyến tính. Với đoạn cua gắt, fallback này có thể không phản ánh đúng độ cong cục bộ.

## 15. Ý nghĩa trong toàn bộ pipeline

Nếu viết gọn toàn pipeline hình học, ta có:

$$
\text{mask} \rightarrow \text{polygon} \rightarrow \text{homography} \rightarrow \text{centerline} \rightarrow \text{polynomial} \rightarrow \text{control metrics}
$$

Trong chuỗi này, centerline extraction là tầng chuyển đổi then chốt:

- từ vùng diện tích sang đường đại diện
- từ dữ liệu perception dày đặc sang dữ liệu điều hướng gọn
- từ mô tả hình dạng cục bộ sang mô hình hình học liên tục

Nếu bước này không ổn định, mọi đại lượng phía sau như:

- `lateral_offset_mm`
- `longitudinal_offset_mm`
- `heading_angle_rad`
- `lookahead_x_mm`

đều sẽ dao động theo. Vì vậy, về mặt hệ thống, centerline extraction không phải một chi tiết phụ của hậu xử lý, mà là một thành phần quyết định chất lượng điều hướng.

## 16. Kết luận

Trích xuất centerline từ polygon làn là bước rút gọn hình học quan trọng nhất sau homography trong hệ AVS. Về nguyên lý, thuật toán xem polygon lane như một miền kín, quét nó bằng các lát cắt song song, tìm hai biên tại mỗi lát và lấy trung điểm để tạo centerline rời rạc. Với lane dọc, phép quét được thực hiện theo `Y`; với lane rẽ, phép quét được thực hiện theo `X`.

Điểm mạnh của triển khai hiện tại không chỉ nằm ở công thức midpoint, mà còn ở các cơ chế làm cho midpoint đó bền hơn với nhiễu:

- dùng bề rộng trung vị để xác định kích thước lane danh định
- phát hiện lát cắt bị phình
- hiệu chỉnh bằng cửa sổ cục bộ và xu thế toàn cục
- gộp waypoint giữa nhiều polygon
- làm mượt không gian trước khi fit đa thức

Nhờ vậy, centerline thu được không chỉ là một tập điểm trung bình đơn giản, mà là một biểu diễn hình học đã qua kiểm soát nhiễu, đủ phù hợp để phục vụ fitting, look-ahead và điều hướng thời gian thực trên phần cứng biên.
