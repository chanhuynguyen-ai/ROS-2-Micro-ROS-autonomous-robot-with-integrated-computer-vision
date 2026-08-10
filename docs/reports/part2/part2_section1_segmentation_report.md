# PHẦN II.1. Bài toán phân đoạn ngữ nghĩa và instance segmentation

## 1. Mở đầu

Trong hệ thống Computer Vision của AVS, bước đầu tiên và quan trọng nhất là biến ảnh đầu vào từ camera thành biểu diễn hình học có ý nghĩa cho điều hướng. Hệ thống không chỉ cần biết "có gì trong ảnh", mà còn cần biết chính xác "vùng nào trong ảnh thuộc về làn nào, vạch nào, biển nào hoặc phương tiện nào". Vì lý do đó, bài toán phù hợp không phải chỉ là phân loại ảnh hoặc phát hiện đối tượng bằng hộp bao, mà là bài toán phân đoạn ảnh.

Ở mức triển khai hiện tại, hệ thống sử dụng một mô hình phân đoạn theo kiểu YOLO segmentation chạy trên NCNN. Mô hình này sinh ra cho mỗi đối tượng một bounding box, một nhãn lớp, một độ tin cậy và một mặt nạ nhị phân riêng. Về bản chất, đây là kiến trúc **instance segmentation**. Tuy nhiên, vì các đối tượng cần quan tâm đều thuộc các lớp mang ý nghĩa ngữ nghĩa rõ ràng như `main-lane`, `other-lane`, `turn-lane`, `stop-line`, `solid-white`, `dashed-yellow`, `vehicle`, nên đầu ra của mô hình đồng thời cũng cung cấp thông tin **ngữ nghĩa theo từng pixel**. Do đó, trong báo cáo này cần trình bày song song hai khía cạnh:

- phân đoạn ngữ nghĩa để giải thích việc gán nhãn theo từng vùng ảnh
- instance segmentation để giải thích cơ chế tách riêng từng thực thể, tái tạo mask và hậu xử lý trong hệ thống thực

## 2. Phân biệt các bài toán thị giác máy tính liên quan

### 2.1. Phân loại ảnh

Phân loại ảnh nhận toàn bộ ảnh đầu vào và xuất ra một hoặc một vài nhãn tổng quát. Ví dụ, mô hình có thể dự đoán rằng ảnh chứa "đường giao thông" hoặc "có xe". Dạng đầu ra này không cho biết vị trí cụ thể của đối tượng trong ảnh, càng không cho biết hình dạng của làn đường. Vì vậy, phân loại ảnh không thể dùng trực tiếp cho bài toán bám làn.

### 2.2. Phát hiện đối tượng

Phát hiện đối tượng gán cho mỗi đối tượng một nhãn lớp và một bounding box. Cách tiếp cận này phù hợp khi chỉ cần biết vị trí xấp xỉ của biển báo hoặc xe phía trước. Tuy nhiên, với làn đường và vùng mặt đường có hình dạng kéo dài, cong, thay đổi bề rộng theo phối cảnh, bounding box là biểu diễn quá thô. Hai nguyên nhân chính là:

- bounding box không khớp với biên thực của làn
- bounding box không cho phép suy ra trực tiếp polygon hay centerline

Do đó, chỉ dùng detection sẽ làm suy giảm mạnh chất lượng bước biến đổi tọa độ và fit hình học ở các tầng sau.

### 2.3. Phân đoạn ngữ nghĩa

Phân đoạn ngữ nghĩa gán cho mỗi pixel một nhãn lớp. Khi đó, mọi pixel thuộc cùng một lớp, ví dụ `main-lane`, sẽ được xem là cùng loại về mặt ngữ nghĩa. Kết quả thu được là một bản đồ nhãn dày đặc trên toàn ảnh.

Ưu điểm của semantic segmentation là:

- mô tả được hình dạng vùng làn đầy đủ hơn nhiều so với bounding box
- thể hiện tốt các cấu trúc mảnh, kéo dài hoặc cong
- thuận lợi cho các bước biến đổi hình học theo từng pixel hoặc từng contour

Tuy nhiên, semantic segmentation thuần túy không phân biệt được hai thực thể cùng lớp nếu chúng xuất hiện tách rời. Ví dụ, nếu có hai vùng `vehicle` hoặc hai vùng lane cùng loại nhưng tách biệt, semantic segmentation chỉ cho biết chúng cùng nhãn chứ không gán danh tính riêng cho từng vùng.

### 2.4. Instance segmentation

Instance segmentation là sự kết hợp giữa detection và segmentation. Mỗi đối tượng được biểu diễn bởi:

- nhãn lớp
- độ tin cậy
- bounding box
- mặt nạ phân đoạn riêng

Khác với semantic segmentation thuần túy, instance segmentation cho phép tách riêng từng thực thể cùng lớp. Điều này đặc biệt hữu ích khi:

- cần theo dõi từng đối tượng theo thời gian
- cần loại bỏ các phát hiện trùng lặp bằng NMS
- cần xử lý từng polygon riêng biệt sau khi trích contour

Trong hệ thống AVS hiện tại, mô hình phân đoạn thuộc nhóm này. Mỗi detection sau cùng có một `mask` riêng, sau đó được chuyển thành `polygons` để publish qua topic `/avs/telemetry`.

## 3. Vì sao hệ thống hiện tại cần segmentation thay vì chỉ dùng bounding box

Đầu ra cuối của perception trong hệ thống AVS không dừng ở mức "phát hiện có làn", mà phải tạo được các đại lượng hình học như centerline, đa thức xấp xỉ làn và sai số `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`. Muốn làm được điều đó, hệ thống cần biết hình dạng thật của vùng làn trên ảnh.

Nếu chỉ có bounding box, các bước sau sẽ gặp vấn đề:

- không xác định chính xác hai biên trái và phải của lane
- không thể quét lát cắt để lấy trung điểm theo từng mức `Y` hoặc `X`
- không thể tạo polygon đáng tin cậy để biến đổi sang mặt phẳng thực
- không thể khử tốt các đoạn lane bị cong hoặc nở cục bộ

Ngược lại, segmentation cung cấp vùng diện tích của làn đường. Từ vùng này có thể:

- trích contour ngoài bằng `cv::findContours()`
- chuyển contour thành polygon
- biến đổi các điểm polygon qua homography
- lấy centerline từ lát cắt trên polygon hoặc trên mask
- fit đa thức để thu được biểu diễn hình học ổn định theo thời gian

Nói cách khác, segmentation là đầu vào bắt buộc để nối tầng perception với tầng hình học và điều hướng.

## 4. Vai trò của semantic segmentation và instance segmentation trong hệ thống AVS

Về mặt lý thuyết, bài toán cần thông tin ngữ nghĩa theo từng pixel để biết vùng nào là làn chính, làn phụ, làn rẽ, vạch dừng hoặc vật thể khác. Đây là bản chất của semantic segmentation.

Về mặt triển khai, hệ thống dùng một mô hình instance segmentation để thu được từng đối tượng riêng biệt. Cách làm này mang lại ba lợi ích thực tế:

1. Giữ được ý nghĩa ngữ nghĩa theo lớp.
2. Tách riêng từng thực thể để hậu xử lý linh hoạt hơn.
3. Phù hợp với pipeline YOLO segmentation tối ưu cho suy luận thời gian thực trên CPU với NCNN.

Có thể xem hệ thống hiện tại là một pipeline **instance segmentation phục vụ cho bài toán semantic understanding và lane geometry extraction**.

## 5. Đầu vào và đầu ra của mô hình trong hệ thống hiện tại

### 5.1. Ảnh đầu vào

Node `ncnn_inference_node` nhận ảnh BGR từ topic ROS2 đầu vào. Ảnh này có kích thước gốc phụ thuộc camera hoặc video phát lại. Trong hàm `YOLO26Seg::detect()`, ảnh được resize về kích thước cố định:

$$
320 \times 320
$$

trước khi đưa vào mạng.

Quá trình tiền xử lý hiện tại gồm:

- chuyển không gian màu từ BGR sang RGB
- resize về `320 x 320`
- chuẩn hóa giá trị pixel về khoảng `[0,1]`

Nếu gọi ảnh gốc là $I_{raw}(u,v,c)$ thì ảnh chuẩn hóa là:

$$
I_{norm}(u,v,c)=\frac{I_{raw}(u,v,c)}{255}
$$

Trong mã nguồn, phép chuẩn hóa được thực hiện bằng:

$$
\text{norm\_vals} = \left[\frac{1}{255}, \frac{1}{255}, \frac{1}{255}\right]
$$

### 5.2. Đầu ra tensor của mô hình

Từ mã nguồn `yolo26_seg.cpp`, mô hình xuất ra hai tensor chính:

- `out0`: tensor detection và mask coefficients cho `2100` anchor, trong đó mỗi anchor mang `55` giá trị theo logic giải mã hiện tại
- `out1`: tensor prototype masks có kích thước `32 x 80 x 80`

Ý nghĩa của `out0` như sau:

- 4 phần tử đầu: tham số hộp bao `(c_x, c_y, w, h)`
- 19 phần tử tiếp theo: điểm số lớp
- 32 phần tử cuối: hệ số mask của đối tượng

Tổng số chiều cho mỗi anchor là:

$$
4 + 19 + 32 = 55
$$

Trong triển khai hiện tại, phần xử lý truy cập theo chỉ số hàng của `out0` để lấy box, class score và 32 hệ số mask cho từng proposal.

Ý nghĩa của `out1` là tập các prototype mask dùng chung cho toàn bộ đối tượng:

$$
P \in \mathbb{R}^{K \times H_p \times W_p}
$$

với:

- $K = 32$
- $H_p = 80$
- $W_p = 80$

Mỗi đối tượng không mang một mask đầy đủ ngay từ đầu, mà chỉ mang một vector hệ số:

$$
C \in \mathbb{R}^{32}
$$

Mask cuối cùng được tái tạo bằng tổ hợp tuyến tính của 32 prototype này.

### 5.3. Các lớp đối tượng đang dùng

Hệ thống hiện tại khai báo 19 lớp:

1. `dashed-white`
2. `dashed-yellow`
3. `double-solid-white`
4. `main-lane`
5. `other-lane`
6. `parking-zone`
7. `sign-no-left`
8. `sign-no-parking`
9. `sign-no-right`
10. `sign-parking`
11. `sign-stop`
12. `sign-turn-left`
13. `sign-turn-right`
14. `solid-white`
15. `solid-yellow`
16. `start`
17. `stop-line`
18. `turn-lane`
19. `vehicle`

Trong số đó, các lớp phục vụ trực tiếp cho bài toán trích xuất hình học là:

- `main-lane`
- `other-lane`
- `turn-lane`
- `stop-line`
- các lớp vạch như `solid-white`, `dashed-white`, `solid-yellow`, `dashed-yellow`

## 6. Quy trình giải mã detection và mặt nạ phân đoạn

### 6.1. Giải mã proposal

Với mỗi anchor, mô hình trả ra:

- tâm hộp bao $(c_x, c_y)$
- bề rộng $w$
- bề cao $h$
- điểm số lớp cho 19 lớp
- vector hệ số mask dài 32 chiều

Hệ thống chọn lớp có điểm cao nhất:

$$
\hat{c} = \arg\max_{c \in \{1,\dots,19\}} s_c
$$

và độ tin cậy tương ứng:

$$
\hat{s} = \max_{c \in \{1,\dots,19\}} s_c
$$

Proposal chỉ được giữ lại nếu:

$$
\hat{s} > \text{prob\_threshold}
$$

Trong cấu hình mặc định hiện tại:

$$
\text{prob\_threshold} = 0.25
$$

Từ biểu diễn tâm-kích thước, bounding box được đổi sang góc trên trái:

$$
x = c_x - \frac{w}{2}, \qquad y = c_y - \frac{h}{2}
$$

### 6.2. Sắp xếp và lọc bằng NMS

Sau khi tạo danh sách proposal, hệ thống sắp xếp giảm dần theo xác suất rồi áp dụng Non-Maximum Suppression.

Với hai box $A$ và $B$, chỉ số giao trên hợp được tính:

$$
IoU(A,B)=\frac{|A \cap B|}{|A \cup B|}
$$

Nếu proposal đang xét có IoU lớn hơn ngưỡng với một proposal tốt hơn đã được giữ lại, proposal đó sẽ bị loại. Mục đích của bước này là:

- tránh nhiều detection chồng lấn cho cùng một đối tượng
- giảm số mask cần giải mã
- ổn định dữ liệu đầu ra cho hậu xử lý và tracking

Ngưỡng mặc định hiện tại là:

$$
\text{nms\_threshold} = 0.45
$$

### 6.3. Ánh xạ box về ảnh gốc

Mạng suy luận trên ảnh `320 x 320`, nhưng các bước hình học sau đó cần làm việc trên kích thước ảnh thật. Vì vậy, sau NMS, bounding box được scale ngược về ảnh gốc:

$$
\text{scale}_x = \frac{W_{img}}{320}, \qquad \text{scale}_y = \frac{H_{img}}{320}
$$

Từ đó:

$$
x' = x \cdot \text{scale}_x,\quad
y' = y \cdot \text{scale}_y,\quad
w' = w \cdot \text{scale}_x,\quad
h' = h \cdot \text{scale}_y
$$

Sau khi scale, box tiếp tục được chặn biên để không vượt ra ngoài ảnh.

## 7. Công thức tái tạo mặt nạ phân đoạn

### 7.1. Biểu diễn prototype masks

Cho tensor prototype:

$$
P \in \mathbb{R}^{K \times H_p \times W_p}
$$

trong đó:

- $K = 32$
- $H_p = 80$
- $W_p = 80$

Mỗi kênh $P_i(x,y)$ là một prototype mask cơ sở.

### 7.2. Vector hệ số mask của từng đối tượng

Với mỗi detection được giữ lại sau NMS, mô hình cung cấp vector:

$$
C = [C_1, C_2, \dots, C_K] \in \mathbb{R}^{K}
$$

Vector này được lưu trong trường `mask_feats` của cấu trúc `Object`.

### 7.3. Mặt nạ thô

Mặt nạ thô của đối tượng trước chuẩn hóa được tính bằng tổ hợp tuyến tính:

$$
M_{raw}(x,y)=\sum_{i=1}^{K} C_i P_i(x,y)
$$

Đây là cơ chế phổ biến của các mô hình YOLO segmentation: toàn ảnh dùng chung một tập prototype, còn từng object chỉ cần học một vector hệ số ngắn gọn.

### 7.4. Hàm sigmoid

Sau khi tổ hợp tuyến tính, hệ thống áp dụng sigmoid:

$$
M(x,y)=\sigma(M_{raw}(x,y))=\frac{1}{1+e^{-M_{raw}(x,y)}}
$$

Giá trị $M(x,y)$ lúc này nằm trong khoảng $(0,1)$ và có thể hiểu là xác suất pixel thuộc về object đang xét.

### 7.5. Nhị phân hóa mặt nạ

Trong triển khai hiện tại, ngưỡng nhị phân hóa mask là `0.5`. Mặt nạ nhị phân được tạo bởi:

$$
M_{bin}(x,y)=
\begin{cases}
1, & M(x,y)\ge 0.5 \\
0, & M(x,y)<0.5
\end{cases}
$$

Trong mã C++, giá trị `1` được lưu dưới dạng `255` trên ảnh `CV_8UC1`, còn `0` là nền.

## 8. Tối ưu giải mã mask theo ROI trong hệ thống hiện tại

Một điểm quan trọng của triển khai hiện tại là hệ thống **không giải mã toàn bộ mask `80 x 80` cho mọi detection rồi mới cắt theo box**. Thay vào đó, hàm `decode_mask()` thực hiện giải mã trên **vùng ROI tương ứng với bounding box trong không gian prototype**. Đây là một tối ưu đúng với tài liệu `../../vision/applied_optimization_methods.md`.

### 8.1. Ánh xạ ROI từ ảnh gốc sang prototype space

Giả sử box sau khi scale về ảnh gốc là:

$$
\text{rect}=(x,y,w,h)
$$

Tỷ lệ ánh xạ từ ảnh gốc sang prototype space là:

$$
\alpha_x=\frac{W_p}{W_{img}}, \qquad \alpha_y=\frac{H_p}{H_{img}}
$$

Khi đó ROI trong không gian prototype được tính:

$$
r_x = \text{round}(x \alpha_x), \quad
r_y = \text{round}(y \alpha_y), \quad
r_w = \text{round}(w \alpha_x), \quad
r_h = \text{round}(h \alpha_y)
$$

ROI này tiếp tục được chặn biên để nằm trọn trong `80 x 80`.

### 8.2. Tổ hợp tuyến tính chỉ trong ROI

Thay vì tính $M_{raw}$ trên toàn bộ lưới prototype, hệ thống chỉ tính trên ROI:

$$
M_{raw}^{ROI}(x,y)=\sum_{i=1}^{K} C_i P_i(x,y),
\qquad (x,y)\in ROI
$$

Điều này làm giảm:

- số phép nhân cộng
- lượng bộ nhớ tạm
- thời gian chạy sigmoid

Tối ưu này đặc biệt hữu ích trên CPU biên như Raspberry Pi khi số detection tăng.

### 8.3. Resize và đặt lại vào ảnh gốc

Sau khi có mặt nạ xác suất trong ROI prototype, hệ thống:

1. áp dụng sigmoid trong ROI
2. resize ROI mask về đúng kích thước box trên ảnh gốc
3. chèn kết quả vào đúng vị trí của box trong ảnh nhị phân kích thước đầy đủ

Kết quả cuối cùng là một `cv::Mat` kiểu `CV_8UC1` có cùng kích thước với ảnh gốc, trong đó chỉ vùng thuộc object có giá trị 255.

## 9. Từ mask nhị phân đến polygon phục vụ hậu xử lý hình học

Sau khi nhận được `mask` cho từng object, node `ncnn_inference_node` tiếp tục dùng:

$$
\texttt{cv::findContours(mask, contours, hierarchy, cv::RETR\_EXTERNAL, cv::CHAIN\_APPROX\_SIMPLE)}
$$

để trích contour ngoài của từng vùng. Việc chỉ lấy contour ngoài (`RETR_EXTERNAL`) phù hợp với mục tiêu của hệ thống là:

- giảm số điểm cần truyền
- giữ biên ngoài của đối tượng
- thuận tiện chuyển sang polygon cho JSON telemetry

Kết quả được publish qua trường `polygons` trong từng object của topic `/avs/telemetry`. Đây là cầu nối trực tiếp giữa bước segmentation và các bước:

- biến đổi phối cảnh ngược
- trích xuất centerline
- fit đường cong
- tính sai số hình học

Như vậy, trong hệ thống hiện tại, **mask nhị phân không phải đầu ra cuối**, mà là trung gian để sinh ra contour/polygon có giá trị hình học cao hơn.

## 10. Liên hệ trực tiếp với bài toán làn đường

### 10.1. Với `main-lane` và `other-lane`

Hai lớp này thường có hình dạng kéo dài theo phương dọc ảnh và bị biến dạng bởi phối cảnh. Segmentation cho phép giữ được hai biên lane, từ đó có thể:

- quét theo các mức `Y`
- lấy trung điểm giữa hai biên
- tạo centerline
- fit đa thức $x(y)$

Nếu chỉ dùng bounding box, toàn bộ thông tin cong của lane sẽ bị mất.

### 10.2. Với `turn-lane`

`turn-lane` thường có hình dạng ngang hoặc cong mạnh. Segmentation cung cấp vùng lane để hệ thống có thể quét theo `X`, lấy các điểm giữa và fit hàm $y(x)$. Đây là điều gần như không thể thực hiện chính xác nếu chỉ có detection box.

### 10.3. Với `stop-line` và các loại vạch

Các lớp vạch như `stop-line`, `solid-white`, `dashed-yellow` có hình học mảnh và định hướng rõ. Segmentation giúp:

- xác định chiều dài và hướng của vạch
- tách khỏi lane lân cận
- dùng làm tham chiếu bổ sung trong nhận thức ngữ cảnh đường

### 10.4. Với `vehicle` và biển báo

Dù các đối tượng như xe hoặc biển báo về lý thuyết có thể chỉ cần detection, việc dùng chung mô hình segmentation giúp thống nhất pipeline, đồng thời vẫn giữ khả năng:

- tách riêng từng đối tượng cùng lớp
- theo dõi object theo `track_id`
- trực quan hóa overlay theo mask

## 11. Ý nghĩa của các ngưỡng trong hệ thống

### 11.1. `prob_threshold`

Ngưỡng này kiểm soát việc proposal nào được chấp nhận trước NMS. Nếu đặt quá thấp:

- số proposal tăng mạnh
- tăng chi phí NMS
- tăng số mask phải giải mã
- tăng nguy cơ nhiễu ở các lớp lane và vạch

Nếu đặt quá cao:

- bỏ sót lane mờ hoặc xa
- làm đứt đoạn polygon
- khiến centerline thiếu ổn định

Do đó, `prob_threshold = 0.25` là điểm cân bằng giữa recall và độ ổn định trong cấu hình hiện tại.

### 11.2. `nms_threshold`

Ngưỡng này kiểm soát mức độ chấp nhận chồng lấn giữa hai proposal. Nếu quá thấp:

- các detection gần nhau có thể bị loại nhầm
- lane hoặc vạch dài có thể bị mất một phần

Nếu quá cao:

- giữ lại nhiều object trùng lặp
- sinh nhiều mask chồng nhau
- tăng tải contour extraction và tracking

Mức `0.45` hiện tại là một giá trị trung dung thường dùng cho YOLO segmentation.

### 11.3. Ngưỡng mask `0.5`

Ngưỡng nhị phân hóa mask ảnh hưởng trực tiếp đến hình dạng contour:

- ngưỡng thấp làm mask nở rộng, biên dễ phình
- ngưỡng cao làm mask co lại, có thể bị đứt

Sai lệch này sẽ lan truyền sang bước trích centerline và homography. Vì vậy, dù `0.5` là giá trị hợp lý mặc định, đây vẫn là thông số ảnh hưởng mạnh đến chất lượng hình học đầu ra.

## 12. Ưu điểm của cách tiếp cận hiện tại

Hệ thống hiện tại có các ưu điểm sau:

- dùng instance segmentation nên vừa có nhãn lớp vừa có mask riêng cho từng object
- phù hợp với bài toán lane geometry, nơi hình dạng vùng quan trọng hơn box
- có thể chuyển trực tiếp từ mask sang contour và polygon
- hỗ trợ tracking object theo từng frame
- đã tối ưu giải mã mask theo ROI để giảm tải CPU
- chạy được bằng NCNN với INT8 và FP16 storage trên phần cứng biên

Quan trọng hơn, toàn bộ pipeline này tương thích tốt với các bước sau trong hệ thống AVS: IPM, centerline extraction, polynomial fitting và suy ra `e_x`, `e_y`, `theta`.

## 13. Hạn chế và các điểm cần lưu ý khi viết báo cáo

Khi trình bày phần này trong báo cáo chính, cần nêu rõ rằng segmentation dù mạnh hơn detection nhưng vẫn có các nguồn sai số:

- mask bị nở cục bộ do nhiễu
- lane bị đứt khi ánh sáng kém hoặc bề mặt phản quang
- biên lane bị lệch ở vùng xa do độ phân giải prototype thấp (`80 x 80`)
- box sai sẽ kéo theo ROI mask sai
- NMS và ngưỡng xác suất có thể làm mất object nhỏ hoặc mờ

Một điểm kỹ thuật quan trọng là prototype mask chỉ có độ phân giải `80 x 80`, sau đó mới được resize về ảnh gốc. Điều này tạo ra giới hạn tự nhiên về độ sắc nét của biên mask. Với các cấu trúc mảnh như vạch lane hoặc stop-line, sai lệch vài pixel ở biên ảnh có thể chuyển thành sai lệch đáng kể sau homography nếu điểm nằm xa camera.

## 14. Kết luận

Bài toán perception của hệ thống AVS về bản chất đòi hỏi thông tin phân đoạn theo pixel để phục vụ trích xuất hình học làn đường. Vì vậy, segmentation là lựa chọn đúng về mặt lý thuyết. Trong triển khai thực tế, hệ thống sử dụng một mô hình instance segmentation kiểu YOLO trên NCNN, với đầu ra gồm box, class score, 32 hệ số mask và prototype masks `32 x 80 x 80`. Từ đó, hệ thống tái tạo mặt nạ nhị phân cho từng object, trích contour thành polygon và chuyển tiếp cho các khối xử lý tọa độ thực.

Nói ngắn gọn, semantic segmentation cung cấp ý nghĩa ngữ nghĩa theo từng vùng ảnh, còn instance segmentation cung cấp cơ chế triển khai cụ thể để biến ý nghĩa đó thành các thực thể độc lập có thể lọc, theo dõi, biến đổi hình học và sử dụng cho điều hướng thời gian thực.
