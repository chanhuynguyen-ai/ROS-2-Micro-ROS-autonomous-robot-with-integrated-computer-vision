# Chương 2. Cơ sở lý thuyết đang áp dụng

Chương này giải thích **từ gốc** các khái niệm lý thuyết mà hệ thống dùng, theo đúng thứ tự chúng xuất hiện trong pipeline: segmentation → homography/IPM → centerline → polynomial fit → lookahead → trajectory → control error. Mỗi mục có phần "ý tưởng" (why) trước khi vào công thức (how), và có ít nhất một ví dụ số minh hoạ. Các ví dụ số trong chương này dùng số liệu **giả định để minh hoạ công thức**, không phải log thật từ hệ thống — chương 3-6 mới đối chiếu với hằng số/ngưỡng thật trong code.

## 2.1. Instance segmentation — vì sao cần "mask" chứ không chỉ "box"

Có ba mức bài toán thị giác máy tính hay bị nhầm với nhau:

- **Object detection**: chỉ trả bounding box (hình chữ nhật bao quanh vật thể) + label + confidence. Đủ dùng nếu vật thể có hình dạng gần với hình chữ nhật (biển báo, xe).
- **Semantic segmentation**: tô màu từng pixel theo class, nhưng không phân biệt "vật thể nào với vật thể nào" nếu hai vật cùng class đứng cạnh nhau.
- **Instance segmentation** (cái hệ thống này dùng): vừa phát hiện từng object riêng biệt, vừa trả về mask (vùng pixel thuộc về đúng object đó).

Với lane/marking, bounding box gần như vô dụng: một làn đường là một dải dài, cong, chiếm gần hết chiều cao ảnh — hình chữ nhật bao quanh nó gần như bao luôn cả làn bên cạnh. Hệ thống cần biết **chính xác hình dạng vùng** của làn để sau này rút ra đường tim làn (centerline). Đó là lý do bắt buộc phải dùng instance segmentation, không phải detection thường.

Output của một detection sau segmentation:

| Trường | Ý nghĩa |
|---|---|
| `label` | ID lớp, ví dụ `6 = main-lane` |
| `prob` | Điểm tin cậy (confidence), 0-1 |
| `box` | Bounding box trong ảnh (dùng cho NMS/tracking, không dùng để tính hình học lane) |
| `mask` | Vùng nhị phân (0/255) đánh dấu pixel nào thuộc object |
| `polygons` | Đường viền (contour) của mask, đã xấp xỉ thành đa giác để nhẹ hơn lưu trữ pixel-by-pixel |

### 2.1.1. Mô hình sinh ra mask như thế nào (YOLO-segmentation kiểu coefficient + prototype)

Một cách "vẽ mask" ngây thơ là cho mô hình xuất thẳng một ảnh nhị phân kích thước đầy đủ cho mỗi object — rất tốn bộ nhớ/tính toán nếu có nhiều object. Cách mà YOLO-segmentation (và hệ thống này) dùng thông minh hơn: mô hình học ra một tập nhỏ **"prototype mask"** dùng chung cho cả ảnh (ví dụ 32 lớp mask cơ sở, kích thước nhỏ 80×80), và với mỗi object chỉ cần học ra **32 hệ số** để pha trộn tuyến tính (linear combination) các prototype đó thành mask riêng của nó:

```text
mask_value(pixel) = Σ_{c=0..31} coeff[c] * prototype[c][pixel]
mask(pixel)        = sigmoid(mask_value(pixel))
mask_nhị_phân       = 255 nếu mask(pixel) > 0.5, ngược lại 0
```

Ý tưởng giống hệt "trộn màu": thay vì lưu một bức ảnh lớn, chỉ cần lưu 32 con số (công thức pha trộn) và một bộ "màu cơ sở" dùng chung.

**Ví dụ số minh hoạ** (rút gọn, chỉ 1 kênh khác 0 để dễ tính tay): giả sử một object `main-lane` có bounding box `(x=100, y=200, w=60, h=80)` trên ảnh gốc `640×480`. Prototype có kích thước `80×80`, nên toạ độ box phải quy đổi tỉ lệ sang không gian prototype trước:

```text
x_scale = 80/640 = 0.125       y_scale = 80/480 ≈ 0.1667
rx = round(100 × 0.125) = 13   ry = round(200 × 0.1667) = 33
rw = round(60  × 0.125) = 8    rh = round(80  × 0.1667) = 13
```

Tại một pixel trong vùng ROI 13×8 đó, giả sử chỉ hệ số kênh 0 đáng kể: `coeff[0] = 2.0`, giá trị prototype tại pixel đó `proto[0][pixel] = 0.3`, các kênh còn lại ≈ 0:

```text
mask_value = 2.0 × 0.3 = 0.6
sigmoid(0.6) = 1 / (1 + e^-0.6) ≈ 0.6457
0.6457 > 0.5  →  pixel này thuộc mask (giá trị 255)
```

Sau đó ROI 13×8 được resize (nội suy song tuyến — bilinear) về đúng kích thước bounding box gốc `60×80` để khớp lại vào ảnh — vì prototype được tính ở độ phân giải thấp còn box thì ở độ phân giải ảnh gốc.

### 2.1.2. NMS (Non-Maximum Suppression) — vì sao cần

Mô hình detection thường sinh ra **nhiều đề xuất chồng lấn** cho cùng một vật thể thật (ví dụ 5-6 box hơi lệch nhau đều "nhìn thấy" cùng một làn đường). NMS là bước lọc để chỉ giữ lại đề xuất tốt nhất, loại các đề xuất trùng lặp:

1. Sắp xếp mọi đề xuất theo `prob` giảm dần.
2. Lấy đề xuất có `prob` cao nhất, giữ lại.
3. Loại mọi đề xuất khác có độ chồng lấn (IoU — xem 2.1.3) với nó lớn hơn một ngưỡng (`nms_threshold`).
4. Lặp lại với đề xuất tốt nhất còn lại, cho tới khi hết.

### 2.1.3. IoU (Intersection over Union) — thước đo độ chồng lấn dùng xuyên suốt hệ thống

IoU là tỉ lệ giữa diện tích giao nhau và diện tích hợp của hai hình chữ nhật — giá trị từ 0 (không chạm nhau) đến 1 (trùng khít). Đây là công cụ được dùng lại ở **hai chỗ khác nhau** trong hệ thống: NMS (lọc trùng trong cùng 1 frame) và tracking (nối object giữa các frame, xem 2.1.4).

```text
IoU(A, B) = diện_tích(A ∩ B) / diện_tích(A ∪ B)
```

**Ví dụ số**: box A `(x=100, y=200, w=60, h=80)` (góc trên-trái tới góc dưới-phải: `100..160, 200..280`), box B `(x=110, y=205, w=60, h=80)` (`110..170, 205..285`) — hai box lệch nhau nhẹ (mô phỏng cùng một làn ở hai frame liên tiếp, xe/mask dịch nhẹ):

```text
Diện tích A = Diện tích B = 60 × 80 = 4800

Giao theo X: min(160,170) − max(100,110) = 160 − 110 = 50
Giao theo Y: min(280,285) − max(200,205) = 280 − 205 = 75
Diện tích giao = 50 × 75 = 3750

Diện tích hợp = 4800 + 4800 − 3750 = 5850
IoU = 3750 / 5850 ≈ 0.641
```

### 2.1.4. Tracking 2D bằng IoU — vì sao cần "track_id" xuyên suốt nhiều frame

Nếu mỗi frame hệ thống coi mọi object là "mới hoàn toàn", các bước phía sau (đặc biệt là làm mượt theo thời gian ở IPM, và hysteresis chọn làn ở control_node) sẽ không biết "làn này ở frame trước và frame này có phải cùng một làn hay không". Giải pháp là **object tracking đơn giản kiểu greedy theo IoU**:

1. Với mỗi cặp (object frame hiện tại, track cũ) **cùng label**, tính IoU giữa bounding box.
2. Giữ lại các cặp có `IoU ≥ ngưỡng` làm ứng viên ghép.
3. Ghép tham lam (greedy): xét các cặp theo IoU giảm dần, cặp nào cả hai phía (track và detection) đều chưa được dùng thì ghép luôn. Đây **không phải** thuật toán tối ưu toàn cục (như Hungarian algorithm) — chỉ là xấp xỉ nhanh, đủ tốt cho tần suất frame cao và object di chuyển chậm giữa hai frame liên tiếp.
4. Track không được ghép trong nhiều frame liên tiếp thì bị xoá; detection không ghép được với track nào thì trở thành track mới với `track_id` mới.

Với ví dụ IoU ở trên (`0.641`), nếu ngưỡng match là `0.3` thì cặp này được ghép — object frame hiện tại **giữ nguyên track_id cũ** thay vì bị coi là một làn mới xuất hiện. Đây chính là cơ chế giúp các tầng phía sau "biết" đây vẫn là cùng một làn đang bám, dù hình dạng mask mỗi frame hơi khác nhau.

## 2.2. Camera phối cảnh và Homography — vì sao không thể dùng thẳng toạ độ pixel

### 2.2.1. Vấn đề: ảnh phối cảnh làm méo khoảng cách

Một camera chiếu thế giới 3D lên mặt phẳng ảnh 2D theo nguyên lý phối cảnh (pinhole camera): vật càng xa thì càng nhỏ, và — quan trọng hơn với hệ thống này — **khoảng cách thật giữa hai điểm trên mặt đường không tỉ lệ tuyến tính với khoảng cách pixel giữa hai điểm ảnh tương ứng**. Hai vạch kẻ song song ngoài đời trông như hội tụ về một điểm (điểm tụ - vanishing point) trên ảnh. Vì vậy: không thể lấy toạ độ pixel `(u, v)` của một điểm rồi coi nó là "lệch bao nhiêu mm" — cần một phép biến đổi đúng để "duỗi thẳng" phối cảnh đó ra.

### 2.2.2. Homography — công cụ toán học để làm việc đó

Nếu **mặt đường là một mặt phẳng** (giả định hợp lý với đường đi trong nhà/sân trong phạm vi ngắn) và **camera cố định** (góc nhìn không đổi), thì tồn tại một phép biến đổi tuyến tính trong toạ độ thuần nhất (homogeneous coordinates) ánh xạ *chính xác* từ điểm pixel trên ảnh sang điểm thật trên mặt phẳng đó. Phép biến đổi này gọi là **homography**, biểu diễn bằng một ma trận 3×3:

```text
H = [ h11 h12 h13 ]
    [ h21 h22 h23 ]
    [ h31 h32 h33 ]
```

Với điểm pixel `(u, v)`, điểm world tương ứng `(X, Y)` (đơn vị mm, trong hệ toạ độ xe ở mục 1.2) được tính bằng:

```text
w = h31*u + h32*v + h33
X = (h11*u + h12*v + h13) / w
Y = (h21*u + h22*v + h23) / w
```

Việc chia cho `w` (thay vì chỉ nhân ma trận thường) chính là phần "phối cảnh" của phép biến đổi — nó mô phỏng đúng hiệu ứng "càng xa càng nén lại" của camera thật. Đây gọi là **Inverse Perspective Mapping (IPM)**: từ ảnh phối cảnh, dựng lại góc nhìn "từ trên xuống" (bird's-eye view) của mặt đường.

Ma trận `H` không tính online — nó được đo một lần bằng calibration (dùng các điểm mốc đã biết khoảng cách thật) và lưu trong `config/calibration.json`. **Runtime không warp lại toàn bộ ảnh thành ảnh bird's-eye view** (tốn CPU); hệ thống chỉ áp công thức trên cho từng đỉnh polygon đã detect được — nhẹ hơn rất nhiều trên phần cứng CPU-only như Raspberry Pi 5.

**Ví dụ số minh hoạ** (H giả định đơn giản để tính tay được, không phải calibration thật):

```text
H = [ 1   0    -160  ]
    [ 0   1    -100  ]
    [ 0   0.002 -0.2 ]
```

Với điểm pixel `(u, v) = (160, 600)`:

```text
w = 0.002×600 - 0.2 = 1.0
X = (160 - 160) / 1.0 = 0 mm
Y = (600 - 100) / 1.0 = 500 mm
```

→ điểm ở chính giữa ảnh (`u=160`), cách xe 500 mm về phía trước, không lệch trái/phải — hợp lý nếu đây là một điểm nằm đúng tim ảnh và tim làn.

Với điểm pixel `(u, v) = (200, 700)` (dịch sang phải và xuống dưới ảnh — tức gần xe hơn):

```text
w = 0.002×700 - 0.2 = 1.2
X = (200 - 160) / 1.2 ≈ 33.3 mm
Y = (700 - 100) / 1.2 = 500 mm
```

→ cùng khoảng cách 500 mm về phía trước nhưng lệch phải 33.3 mm — minh hoạ việc một cạnh polygon nghiêng trên ảnh sẽ được "duỗi thẳng" đúng tỉ lệ khi qua homography.

### 2.2.3. Giả định và giới hạn của homography

- Mặt đường phải đủ phẳng (địa hình gồ ghề sẽ làm sai lệch).
- Camera phải cố định — nếu góc camera đổi (rung, xê dịch), `H` cũ không còn đúng nữa.
- Calibration phải khớp với góc lắp camera hiện tại; nếu file `calibration.json` bị ghi đè bằng calibration cho góc camera khác, mọi toạ độ world phía sau sẽ sai toàn bộ mà không có cảnh báo rõ ràng ở tầng ứng dụng.
- Với `u, v` nằm trên hoặc gần "đường chân trời" (horizon — nơi `w ≈ 0`), phép chia sẽ cho ra toạ độ world tiến tới vô cực. Đây là lý do bắt buộc phải có bước lọc vùng hợp lệ trước khi chiếu — xem 2.3.

## 2.3. Vùng hợp lệ (BEV valid region) và cắt polygon

### 2.3.1. Vì sao phải cắt trước khi chiếu

Vì phép chia cho `w` "nổ" gần đường chân trời, một polygon lane có cạnh vắt qua vùng gần chân trời (rất thường gặp — làn đường luôn kéo dài xa dần lên phía trên ảnh) sẽ tạo ra vài đỉnh có toạ độ world khổng lồ và vô nghĩa nếu chiếu thẳng. Giải pháp không phải là "loại bỏ toàn bộ polygon" (sẽ mất luôn phần dữ liệu hợp lệ gần xe), mà là **cắt (clip) polygon tại đúng đường biên hợp lệ**, giữ lại phần polygon nằm trong vùng chiếu an toàn, trước khi áp công thức homography.

Vùng hợp lệ được giới hạn bởi các tham số:

- `bev_horizon_margin_px`: biên an toàn tính bằng pixel, phía dưới đường chân trời một khoảng để tránh vùng gần `w≈0`.
- `bev_y_max_mm`, `bev_x_abs_max_mm`: giới hạn khoảng cách hợp lý phía trước và hai bên trong không gian world — loại các điểm world quá xa để tránh nhiễu (dù phần world này về mặt toán không "nổ" nhưng dữ liệu quá xa camera vốn đã kém tin cậy).

### 2.3.2. Thuật toán cắt polygon: Sutherland–Hodgman

Đây là một thuật toán kinh điển trong đồ hoạ máy tính để cắt một đa giác bởi một **nửa mặt phẳng** (half-plane, tức "phần nằm về một phía của một đường thẳng"). Ý tưởng: duyệt lần lượt từng cạnh của polygon gốc; với mỗi cạnh nối hai đỉnh `cur → next`:

- Nếu `cur` nằm trong vùng hợp lệ (`f(cur) ≥ 0` với `f` là hàm khoảng cách có dấu tới đường biên) → giữ `cur`.
- Nếu dấu của `f` đổi giữa `cur` và `next` (một điểm trong, một điểm ngoài vùng hợp lệ) → thêm vào polygon kết quả điểm giao cắt giữa cạnh đó và đường biên, nội suy tuyến tính theo tỉ lệ `t = f(cur) / (f(cur) − f(next))`.

Lặp lại thuật toán này lần lượt cho **từng đường biên** của vùng hợp lệ (biên chân trời trong không gian pixel, rồi bốn biên hình chữ nhật trong không gian world sau khi đã chiếu) sẽ cho ra polygon đã được "gọt" gọn trong vùng hợp lệ, với các cạnh cắt được nội suy chính xác tại biên thay vì bị vứt bỏ thô bạo.

Hệ quả thực tế quan trọng: một polygon lane cắt ngang qua đường chân trời **không bị loại bỏ hoàn toàn** — nó chỉ mất phần phía trên chân trời, phần còn lại (gần xe, đáng tin cậy) vẫn được giữ và chiếu bình thường.

## 2.4. Trích centerline (đường tim làn) từ polygon

### 2.4.1. Vì sao cần bước này

Segmentation cho ra một **vùng diện tích** (polygon lane), nhưng để lái xe, hệ thống cần một **đường một chiều** (centerline/waypoints) chạy dọc giữa làn đó — đó mới là thứ có thể tham số hoá thành `x(y)` hoặc `y(x)` để tính lệch/góc/độ cong.

### 2.4.2. Thuật toán: quét lát (slicing)

Ý tưởng: cắt polygon thành nhiều "lát" song song, tại mỗi lát tìm hai mép trái/phải của làn, lấy điểm giữa (midpoint) làm một điểm trên centerline.

- Với `main-lane`/`other-lane`: làn chạy dọc theo hướng tiến của xe, nên **quét theo trục Y** (từng mức `y` cách nhau 100 mm) — tại mỗi mức `y`, duyệt qua tất cả các cạnh của polygon, tìm những cạnh cắt ngang mức `y` đó (nội suy điểm cắt theo tỉ lệ tuyến tính trên cạnh), lấy `x` nhỏ nhất/lớn nhất trong các điểm cắt làm mép trái/phải, rồi tính `x_mid = (x_left + x_right)/2`.
- Với `turn-lane`: làn rẽ thường trải ngang/chéo so với hướng tiến, nếu vẫn quét theo Y sẽ có nhiều lát chỉ cắt qua polygon một lần hoặc không cắt — không ổn định. Vì vậy `turn-lane` được **quét theo trục X**, tìm mép trên/dưới (`y_bottom`, `y_top`), lấy `y_mid`.

```text
main/other lane: quét theo Y          turn lane: quét theo X
Y                                     Y
^     centerline                      ^  ----- centerline
|        |                            |
+-----> X                             +-----> X
```

### 2.4.3. Vấn đề "lát bị phình" (bloated) và cách sửa

Ở khu vực giao lộ hoặc khi hai làn dính mask vào nhau, một số lát có bề rộng (`width = x_right - x_left`) lớn bất thường so với các lát xung quanh — nếu lấy midpoint trực tiếp, centerline sẽ bị kéo lệch sai. Hệ thống phát hiện lát bất thường bằng cách so với **median độ rộng** của toàn bộ các lát:

```text
lát bị "bloated" nếu width > 1.3 × median_width
```

Với các lát bị đánh dấu bloated, midpoint không lấy trực tiếp mà được sửa bằng một trong hai cách:

1. Nếu xung quanh (cửa sổ trượt local) có đủ lát "sạch" (không bloated) → lấy **median** của các midpoint sạch đó làm giá trị thay thế.
2. Nếu toàn bộ cửa sổ local cũng bloated (ví dụ cả một đoạn dài bị lẫn mask) → dùng xu hướng tuyến tính toàn cục (global linear trend, fit `x = m·y + c` bằng least-square trên các lát sạch còn lại) để ước lượng midpoint tại vị trí đó.

Ý tưởng chung: luôn ưu tiên dữ liệu thật cục bộ, chỉ "ngoại suy" bằng xu hướng toàn cục khi không còn lựa chọn nào khác — giảm thiểu việc bịa dữ liệu sai ở vùng nhiễu.

## 2.5. Fit polynomial — biến tập điểm rời rạc thành một hàm số mượt

### 2.5.1. Vì sao fit polynomial bậc 3

Sau khi có một chuỗi điểm centerline rời rạc (mỗi 100 mm một điểm), hệ thống cần một **hàm số liên tục** để: (a) có thể tính giá trị tại bất kỳ khoảng cách nào (không chỉ đúng các mốc 100 mm đã đo), và (b) có thể rút ra các đại lượng đạo hàm bậc nhất (góc/heading) và bậc hai (độ cong/curvature) một cách trực tiếp từ hệ số.

Đa thức bậc 3 là lựa chọn kinh điển cho biên dạng đường (cùng họ với "cubic spline" hay dùng trong robotics): đủ bậc tự do để mô tả một đoạn đường cong nhẹ + đổi độ cong, nhưng không "quá khớp" (overfit) theo từng điểm nhiễu nhỏ như đa thức bậc cao hơn sẽ dễ mắc phải.

```text
main-lane/other-lane:  x(y) = a3·y³ + a2·y² + a1·y + a0
turn-lane:             y(x) = a3·x³ + a2·x² + a1·x + a0
```

### 2.5.2. Least-squares fit bằng ma trận Vandermonde + SVD

Với `n` điểm dữ liệu `(y_i, x_i)`, hệ thống không giải đúng hệ phương trình (vì `n` thường nhiều hơn 4 ẩn số `a0..a3`, dữ liệu lại có nhiễu) — mà giải bài toán **bình phương tối thiểu** (least squares): tìm bộ hệ số `a` sao cho tổng bình phương sai số dự đoán so với dữ liệu thật là nhỏ nhất. Biểu diễn dưới dạng ma trận:

```text
A · a = b

A = [ y1³ y1² y1 1 ]     a = [a3]     b = [x1]
    [ y2³ y2² y2 1 ]         [a2]         [x2]
    [ ...          ]         [a1]         [...]
    [ yn³ yn² yn 1 ]         [a0]         [xn]
```

`A` gọi là **ma trận Vandermonde** (mỗi hàng là các luỹ thừa tăng dần của một điểm dữ liệu). Hệ thống giải bài toán này bằng `cv::solve(A, b, DECOMP_SVD)` — dùng phân tích giá trị kỳ dị (Singular Value Decomposition), một phương pháp số ổn định để giải least-squares kể cả khi `A` gần suy biến (ví dụ các điểm `y_i` quá gần nhau).

Nếu chỉ có `2` hoặc `3` điểm (không đủ 4 để xác định trọn vẹn bậc 3), hệ thống hạ bậc xuống fit tuyến tính `x = a1·y + a0` (ép `a3 = a2 = 0`) — vẫn cho ra một hướng đi hợp lý thay vì từ chối tính toán. Nếu ít hơn 2 điểm, không đủ dữ liệu để fit gì cả.

### 2.5.3. Ý nghĩa vật lý của từng hệ số (gần xe, tức `y ≈ 0`)

Đây là phần quan trọng nhất để hiểu vì sao chỉ vài hệ số đa thức lại đủ để suy ra "lệch ngang", "góc lái", "độ cong":

- `a0 = x(0)`: giá trị của `x` tại `y = 0` — chính là **lệch ngang (lateral offset)** ngay tại vị trí xe.
- `atan(a1)`: `a1` là đạo hàm bậc nhất `dx/dy` tại `y=0` (vì đạo hàm của `a3y³+a2y²+a1y+a0` tại `y=0` chỉ còn `a1`) — đây là độ dốc của đường cong ngay tại xe; lấy `atan` để đổi từ "độ dốc" sang **góc (heading angle)** tính bằng radian.
- `2·a2`: đạo hàm bậc hai tại `y=0` của đa thức là `6a3·y + 2a2`, tại `y=0` còn lại `2a2` — xấp xỉ **độ cong (curvature)** cục bộ ngay tại xe (độ cong thật của một đường cong là tỉ số phức tạp hơn liên quan đạo hàm bậc 1 và bậc 2, nhưng khi đạo hàm bậc 1 nhỏ — tức xe gần như thẳng hàng với làn — `2a2` là một xấp xỉ tốt và rẻ để tính).

### 2.5.4. Ví dụ số: từ polygon tới hệ số đa thức

Cho một polygon lane world-frame (đã qua bước homography + clip ở 2.2-2.3), rộng 700 mm, lệch trái dần đều theo Y (mô phỏng xe đang hơi lệch khỏi tim làn, và làn "xoay" nhẹ so với trục xe):

```text
P0=(-350,200)   P1=(350,200)   P2=(400,800)   P3=(-300,800)
```

Quét `Y` từ 200 đến 800, bước 100 (7 lát). Nội suy tuyến tính hai cạnh bên: cạnh phải `x_right(y) = 350 + (y-200)/12`, cạnh trái `x_left(y) = -350 + (y-200)/12`. Mọi lát đều có `width = 700` (đồng nhất — không lát nào bị "bloated" vì median cũng là 700, ngưỡng bloat sẽ là `1.3×700=910`). Midpoint tại vài mốc:

```text
y=200  → x_mid = 0
y=500  → x_mid = 25
y=800  → x_mid = 50
```

Quan hệ này thực chất tuyến tính hoàn hảo: `x_mid(y) = (y-200)/12`. Fit đa thức bậc 3 qua SVD trên 7 điểm thẳng hàng này sẽ cho:

```text
a3 ≈ 0        a2 ≈ 0
a1 = 1/12 ≈ 0.08333
a0 = -200/12 ≈ -16.667
```

Từ đó suy ra (lưu ý: `a0` ở đây là **ngoại suy** ra `y=0`, nằm ngoài vùng dữ liệu thật 200-800 mm — cần hiểu đây là ước lượng từ xu hướng đo được, không phải một điểm đo trực tiếp):

```text
lateral_offset_mm  = a0            ≈ -16.7 mm
heading_angle_rad  = atan(a1)      = atan(0.0833) ≈ 0.0831 rad (≈ 4.76°)
curvature_inv_mm   = 2·a2          = 0            (đường thẳng, đúng vì polygon dựng tuyến tính)
```

## 2.6. Làm mượt theo thời gian (temporal smoothing) — EMA

Dù đã lọc lát bloated, dữ liệu centerline vẫn dao động nhẹ frame-qua-frame do nhiễu mask. Hệ thống dùng **trung bình động hàm mũ** (Exponential Moving Average — EMA), một công thức làm mượt rất phổ biến vì chỉ cần lưu một giá trị "trạng thái mượt" duy nhất, không cần giữ lịch sử nhiều frame:

```text
giá_trị_mượt_mới = alpha × giá_trị_thô_hiện_tại + (1 − alpha) × giá_trị_mượt_trước_đó
```

`alpha` càng nhỏ thì càng "ì" (mượt nhưng phản ứng chậm với thay đổi thật); càng lớn thì càng bám sát dữ liệu mới (ít trễ nhưng dễ rung theo nhiễu). Hệ thống dùng `alpha = 0.25` — nghiêng về phía ổn định, chấp nhận độ trễ nhỏ để đổi lấy một đường lái không giật.

**Ví dụ số**: nếu giá trị `heading_angle` đã mượt ở frame trước là `0.10 rad`, và giá trị thô đo được ở frame này là `0.0831 rad` (từ ví dụ 2.5.4):

```text
smoothed = 0.25 × 0.0831 + 0.75 × 0.10 = 0.0958 rad
```

→ giá trị mượt chỉ dịch một phần về phía số đo mới, không nhảy thẳng tới `0.0831` — đây chính là cơ chế chống rung giữa các frame liên tiếp.

## 2.7. Lookahead (khoảng cách nhìn trước) — vì sao không dùng điểm ngay tại xe

Nếu bộ điều khiển chỉ nhìn vào điểm ngay tại `y=0` (lệch ngang hiện tại), nó sẽ phản ứng "cận thị" — chỉ sửa lỗi đã xảy ra, không dự đoán trước hướng đường sắp tới, dễ gây dao động (lái quá tay rồi lại sửa quá tay). Ý tưởng "lookahead" (giống triết lý của thuật toán Pure Pursuit kinh điển trong robotics) là chọn một điểm ở phía trước, cách xe một khoảng `d`, rồi lái theo hướng tới điểm đó — vừa sửa lỗi hiện tại vừa "đón đầu" hình dạng đường sắp tới.

Khoảng cách lookahead không cố định — xe đi nhanh cần nhìn xa hơn (giống người lái xe nhìn xa hơn khi chạy tốc độ cao):

```text
d = clamp(tốc_độ_hiện_tại × T_preview, d_min, d_max)
```

`T_preview` là "thời gian nhìn trước" (bao nhiêu giây tương lai); nhân với tốc độ ra khoảng cách. `clamp` để tránh lookahead quá gần (mất ổn định ở tốc độ thấp) hoặc quá xa (vượt ra ngoài vùng dữ liệu tin cậy ở tốc độ cao).

Với hàm `x(y)` đã fit ở 2.5.4, và giả sử tốc độ hiện tại tương ứng lookahead `d = 225 mm` (nằm trong khoảng dữ liệu 200-800mm, tức vẫn là nội suy chứ không phải ngoại suy):

```text
x(225) = 0.0833×225 - 16.667 ≈ 2.08 mm
theta_lookahead = atan2(2.08, 225) ≈ 0.0092 rad (≈ 0.53°)
```

Hai giá trị `x(225)` và `theta_lookahead` này chính là kiểu đại lượng được ghi vào `lookahead_x_mm`/`lookahead_theta_rad` trong JSON world-frame.

## 2.8. Trajectory planning: nối các đoạn đường bằng Bezier

### 2.8.1. Bài toán: cần một đường nối mượt giữa hai làn

Khi xe cần rẽ hoặc đổi làn, cần một đường đi nối từ điểm trên làn hiện tại sang điểm trên làn đích — nhưng nối bằng đường thẳng sẽ tạo ra một góc gãy đột ngột tại điểm nối (xe phải bẻ lái tức thời, không khả thi về động lực học). Cần một đường **cong mượt, có hướng ra/vào khớp với hướng của từng làn tại điểm nối**.

### 2.8.2. Đường cong Bezier bậc 3 (cubic Bezier)

Một cubic Bezier được định nghĩa bởi 4 điểm điều khiển `P0, P1, P2, P3`, và một tham số `t` chạy từ 0 đến 1:

```text
B(t) = (1-t)³·P0 + 3(1-t)²t·P1 + 3(1-t)t²·P2 + t³·P3
```

Ý nghĩa hình học đơn giản (không cần nhớ công thức, chỉ cần nhớ vai trò từng điểm):

- `P0`: điểm bắt đầu (trên làn hiện tại).
- `P3`: điểm kết thúc (trên làn đích).
- `P1`: nằm trên tiếp tuyến (hướng đi) tại `P0` — quyết định đường cong "rời khỏi" `P0` theo đúng hướng làn hiện tại, không bị gãy góc.
- `P2`: nằm trên tiếp tuyến tại `P3` — quyết định đường cong "đi vào" `P3` theo đúng hướng làn đích.

```text
làn hiện tại ---- P0 .. P1 .. P2 .. P3 ---- làn đích
                    (đường cong Bezier nối mượt hai hướng)
```

Vì `P1`/`P2` được đặt theo tiếp tuyến tại đầu mút, đường Bezier đảm bảo **liên tục về hướng (heading)** tại cả điểm bắt đầu lẫn điểm kết thúc — không có góc gãy.

### 2.8.3. Resample theo arc-length

Sau khi có đường Bezier (một hàm liên tục theo `t`), hệ thống lấy mẫu lại thành một chuỗi điểm rời rạc cách đều nhau theo **độ dài cung thực tế** (arc length), mặc định mỗi 100 mm — chứ không phải cách đều theo `t` (vì `t` không tỉ lệ tuyến tính với độ dài cung khi đường cong). Nhờ vậy các bước tính toán phía sau (nội suy tìm điểm lookahead, tính độ cong 3 điểm) đều làm việc trên một chuỗi điểm có khoảng cách đều đặn, dễ dự đoán.

## 2.9. Trajectory candidate → normalized → committed: cơ chế "bộ nhớ" chống giật

Mục 1.6 đã giới thiệu 4 tầng khái niệm. Về mặt lý thuyết, đây là một dạng **bộ lọc trạng thái có trễ (stateful hysteresis filter)** áp trên toàn bộ đường đi, chứ không chỉ trên từng con số đơn lẻ như EMA ở mục 2.6:

- **Chuẩn hoá (normalize)**: nếu trajectory ứng viên (candidate) cùng "loại hình học" (topology — ví dụ cùng là follow-main, hay cùng là turn-left) với trajectory đang bám ở frame trước, hai đường được **căn chỉnh theo arc-length rồi trộn có trọng số** — trọng số nghiêng dần về phía candidate mới khi đi xa dần theo `s` (điểm gần xe tin tưởng cao trajectory cũ đã ổn định, điểm xa tin tưởng candidate mới vì nó phản ánh dữ liệu mới nhất về phía trước). Nếu khác loại hình học — không trộn (passthrough), vì trộn hai đường có "ý nghĩa lái" khác nhau (ví dụ đang đi thẳng trộn với đang rẽ) sẽ tạo ra một đường vô nghĩa về mặt vật lý.
- **Chốt (commit)**: một bộ máy trạng thái riêng so sánh candidate đã chuẩn hoá với trajectory đang active, dùng vài chỉ số (độ lệch ngang RMS, độ lệch góc RMS, tỉ lệ chồng lấn...) để quyết định: giữ nguyên (đường cũ vẫn đủ tốt), cập nhật mềm (khác biệt vừa phải), hay chốt hẳn trajectory mới (khác biệt lớn, hoặc ý định lái vừa đổi).

Chi tiết công thức/ngưỡng số thật của cả hai bước này được trình bày trong chương 5 (đây là phần phức tạp và đặc thù nhất của `control_node`, nên tách riêng thay vì đưa vào chương lý thuyết chung).

## 2.10. Control error: bộ số cuối cùng gửi cho downstream

Sau khi có một active trajectory duy nhất, hệ thống không gửi cả đường đi cho bộ điều khiển — chỉ gửi một vài con số hình học tại đúng điểm lookahead, đủ để một bộ điều khiển kiểu Pure Pursuit/PD phía sau dùng:

- **`epsilon_x_mm`, `epsilon_y_mm`**: toạ độ của điểm lookahead trong hệ toạ độ xe (mục 1.2) — về bản chất đây là "sai số vị trí" (cross-track theo trục X, along-track theo trục Y) mà bộ điều khiển cần triệt tiêu.
- **`theta_rad`**: góc giữa hướng xe (trục Y) và hướng tới điểm lookahead — `atan2(epsilon_x_mm, epsilon_y_mm)`. Đây là sai số góc (heading error) mà nhiều bộ điều khiển lái dùng làm tín hiệu chính (ví dụ Pure Pursuit thuần tuý chỉ cần góc này + khoảng cách lookahead để tính độ cong lái cần thiết).
- **`curvature_inv_mm`**: độ cong cục bộ của đường đi tại vùng lookahead — cho biết đường sắp tới cong gấp hay thoải, giúp bộ điều khiển "đón đầu" thay vì chỉ phản ứng theo sai số hiện tại.

Nếu trajectory hiện tại đã có sẵn các giá trị "precomputed" (được IPM tính sẵn từ polynomial, xem chương 4-5), hệ thống ưu tiên dùng thẳng các giá trị đó thay vì tính lại từ đầu trên chuỗi điểm rời rạc — vừa nhanh hơn, vừa nhất quán với con số mà tầng trước đã tính.
