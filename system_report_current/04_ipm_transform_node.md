# Chương 5. `ipm_transform_node`

## 5.1. Vai trò

`ipm_transform_node` là nơi chuyển dữ liệu từ "hình học trong ảnh" sang "hình học trong thế giới thực" — tầng quan trọng nhất về mặt lý thuyết toán (homography, centerline, polynomial fit — đã giải thích từ gốc ở chương 2 mục 2.2-2.7). Chương này mô tả **chính xác cách các công thức đó được cài đặt** trong code, kèm ví dụ số cụ thể.

```text
/avs/telemetry
-> polygon pixel
-> cắt theo vùng hợp lệ (Sutherland-Hodgman) + chiếu qua homography
-> polygons_real_world (mm)
-> quét lát tìm centerline
-> fit polynomial + làm mượt
-> /avs/telemetry_realworld
```

File chính: `ros2_ws/src/avs_perception/src/ipm_transform_node.cpp`.

## 5.2. Calibration — đọc và tự động reload

Node đọc `calibration_file_path` (tham số ROS2, mặc định `/workspace/config/calibration.json`), file phải chứa key `homography_matrix` dạng mảng 3×3, ánh xạ **row-major** vào ma trận `H` trong code (`H[i][j]` là hàng `i` cột `j`, đúng như công thức ở chương 2 mục 2.2.2).

Node lưu lại `last_write_time` của file lúc đọc; ở đầu mỗi lần xử lý telemetry, nó so sánh mtime hiện tại của file với giá trị đã lưu — nếu khác, đọc lại calibration ngay lập tức mà **không cần restart node**. Nếu file không tồn tại hoặc sai format, node log cảnh báo (giới hạn tần suất, tránh spam log) và **không publish gì cho frame đó** — đây là hành vi "fail-safe im lặng" cần lưu ý khi debug: nếu `/avs/telemetry_realworld` đột nhiên không có dữ liệu, calibration là nghi phạm đầu tiên cần kiểm tra.

## 5.3. Odometry và lookahead động

Node subscribe `/odom_raw`. Tốc độ được quy đổi:

```text
current_speed_mms = |odom.twist.twist.linear.x| × 2500.0
```

(`linear.x` là giá trị đã chuẩn hoá về khoảng `[0,1]`; hệ số `2500` tương ứng `1.0` chuẩn hoá = 2.5 m/s.)

Khoảng cách lookahead động (công thức và ý nghĩa đã giải thích ở chương 2 mục 2.7):

```text
d = clamp(current_speed_mms × T_preview, d_min_mm, d_max_mm)
```

**Giá trị thực tế dùng khi chạy** (đọc từ tham số ROS2, khai báo trong constructor) là `T_preview = 0.15 s`, `d_min = 120 mm`, `d_max = 450 mm`. Trong code còn có một bộ giá trị mặc định khác gán sẵn trên khai báo biến thành viên (`T_preview_ = 0.5`, `d_min_mm_ = 150`, `d_max_mm_ = 600`) — đây chỉ là giá trị khởi tạo tạm thời trước khi constructor đọc và ghi đè bằng giá trị tham số thật ngay sau đó, nên **giá trị vận hành thực tế luôn là bộ số đầu** (`0.15/120/450`) trừ khi launch file truyền tham số khác đi. Không nên nhầm lẫn hai bộ số này khi đọc code.

## 5.4. Chuyển polygon sang world frame — cắt trước, chiếu sau

Với mỗi object có `polygons`, node dựng `polygons_real_world` theo đúng lý thuyết ở chương 2 mục 2.3 (Sutherland-Hodgman), cụ thể gồm 3 bước liên tiếp trong `clip_and_project`:

1. **Cắt trong không gian pixel theo biên chân trời**: đường chân trời ứng với hàng ảnh `v` sao cho hàng thứ 3 của `H` triệt tiêu (`H[2][0]·u + H[2][1]·v + H[2][2] = 0`), cộng thêm biên an toàn `bev_horizon_margin_px`. Polygon được cắt (Sutherland-Hodgman) chỉ giữ phần nằm dưới đường biên này — phần "quá gần/ trên chân trời" (nơi phép chia phối cảnh sẽ nổ số) bị loại trước khi chiếu, không phải sau.
2. **Chiếu mọi điểm còn lại qua homography** — đúng công thức chương 2 mục 2.2.2.
3. **Cắt trong không gian world theo 4 biên hình chữ nhật hợp lệ**: `Y ≥ y_min` (mặc định 0), `Y ≤ bev_y_max_mm`, `X ≤ bev_x_abs_max_mm`, `X ≥ -bev_x_abs_max_mm` — mỗi lần cắt gọi lại đúng thuật toán Sutherland-Hodgman với một cặp hệ số biên khác.

Với polygon kín (marking dạng vùng, ≥3 điểm) thuật toán clip nối cạnh cuối về cạnh đầu (`closed=true`); với polyline (marking dạng đường, chỉ 2 điểm) không nối vòng (`closed=false`).

Toạ độ world sau chiếu được làm tròn 1 chữ số thập phân cho JSON gọn hơn. Polygon còn lại dưới 3 điểm sau khi cắt sẽ không được dùng để trích centerline.

**Ví dụ số** (dùng lại H giả định ở chương 2 mục 2.2.2 để minh hoạ, không phải calibration thật):

```text
H = [ 1   0    -160  ]     horizon_v(u) = -(0·u - 0.2)/0.002 = 100
    [ 0   1    -100  ]     → mọi pixel có v < 100+margin(10)=110 bị coi là "trên chân trời", loại ngay ở bước 1
    [ 0   0.002 -0.2 ]

Điểm pixel (u,v)=(160,600):  w=0.002·600-0.2=1.0 → X=0mm,    Y=500mm
Điểm pixel (u,v)=(200,700):  w=0.002·700-0.2=1.2 → X≈33.3mm, Y=500mm
```

## 5.5. Trích centerline cho `main-lane` và `other-lane`

Với label `3` hoặc `4`, thuật toán quét theo `Y` (lý thuyết ở chương 2 mục 2.4.2):

1. Sort các polygon world hợp lệ của object theo số điểm giảm dần (ưu tiên xử lý polygon "đầy đủ" nhất trước, nếu object có nhiều mảnh polygon rời rạc).
2. Quét `y` từ mức làm tròn lên gần nhất của `100mm` tới `max_y`, bước `100mm`.
3. Tại mỗi mức `y`: duyệt mọi cạnh polygon (kể cả cạnh không phải "trái/phải" theo trực giác — thuật toán không giả định polygon lồi), tìm các cạnh cắt ngang mức `y` (bỏ qua cạnh gần như nằm ngang, chênh lệch `y` hai đầu `<1e-5`), nội suy tuyến tính điểm cắt `x_int` trên mỗi cạnh cắt được.
4. Lấy `x_left = min(mọi x_int)`, `x_right = max(mọi x_int)`, `x_mid = (x_left+x_right)/2`.
5. Tính `w_median` — median độ rộng của toàn bộ các lát (nếu `w_median < 10mm`, ép về `400mm` để tránh chia cho số gần 0 hoặc lane rác gây méo ngưỡng).
6. Đánh dấu lát "bloated" nếu `width > 1.3 × w_median` (đúng công thức chương 2 mục 2.4.3).
7. Fit xu hướng tuyến tính toàn cục `x = m·y + c` bằng least-square thủ công trên các lát sạch (không dùng `cv::solve`, tự tính bằng công thức hồi quy tuyến tính đơn giản).
8. Sửa các lát bloated: tìm cửa sổ local `±3` lát quanh lát đó — nếu có lát sạch trong cửa sổ, lấy **median** midpoint của chúng làm `local_center`; nếu cả cửa sổ đều bloated, dùng giá trị từ xu hướng toàn cục tại `y` đó. Sau đó so sánh `left_dev`/`right_dev` (độ lệch giữa biên trái/phải thực tế so với biên "kỳ vọng" quanh `local_center` với bề rộng `w_median`):
   - nếu `|left_dev - right_dev| < 50mm` → coi là phình đối xứng cả hai bên, lấy thẳng `final_x = local_center`;
   - nếu lệch trái nhiều hơn → có khả năng mask dính sang trái (ví dụ lẫn làn bên cạnh) → clip bớt biên trái: `final_x = (local_center - w_median/2 + x_right) / 2`;
   - ngược lại (lệch phải nhiều hơn) → clip biên phải theo công thức đối xứng.
9. Gộp toàn bộ waypoint, sort theo `Y`.
10. Làm mượt không gian: trung bình 3 điểm liên tiếp (chỉ áp dụng cho các điểm nội bộ, giữ nguyên 2 điểm đầu/cuối) — xem chương 2 mục 2.6 phần "smoothing không gian".
11. Fit polynomial `x(y)` lần 1 trên các điểm đã mượt không gian (chương 2 mục 2.5).
12. Làm mượt theo thời gian bằng EMA (`alpha=0.25`) theo `track_id`, áp dụng tại các "điểm neo" (anchor point) theo lưới toạ độ quét cố định (`y` tròn 100mm) — không áp trực tiếp lên hệ số đa thức mà áp lên từng điểm neo trước, sau đó **fit lại polynomial lần 2** trên các điểm neo đã mượt để ra hệ số cuối cùng.
13. Regenerate lại một chuỗi waypoint mượt theo polynomial vừa fit, bước `100mm`, giới hạn tối đa `128` điểm mỗi object (`kMaxWaypointsPerObject`, một cap an toàn tránh sinh vô hạn điểm nếu dữ liệu bất thường).

Ngoài EMA cho toạ độ waypoint, còn có một EMA riêng (cùng `alpha=0.25`) áp trực tiếp lên 3 đại lượng control-output cuối cùng (`lateral_offset`/`longitudinal_offset`, `heading_angle`, `curvature`) — tách biệt với EMA áp lên waypoint ở bước 12. Trạng thái làm mượt theo `track_id` được dọn dẹp nếu track đó không xuất hiện trong hơn 15 frame liên tiếp.

Kết quả ghi vào object: `waypoints`, `polynomial`, `lateral_offset_mm`, `heading_angle_rad`, `curvature_inv_mm`, `lookahead_d_mm`, `lookahead_x_mm`, `lookahead_theta_rad`.

## 5.6. Trích centerline cho `turn-lane`

Với label `20`, thuật toán đối xứng hoàn toàn với mục 5.5 nhưng đổi trục quét: quét theo `X`, tìm mép dưới/trên `y_bottom`/`y_top`, fit `y(x)`, cùng ngưỡng bloat `1.3×median`, cùng cơ chế sửa bằng local-median hoặc global-trend. Lý do đổi trục: `turn-lane` thường trải ngang/chéo so với hướng tiến của xe (chương 2 mục 2.4.2) — nếu ép quét theo `Y` như làn thường, nhiều lát sẽ cắt polygon 0 hoặc nhiều lần một cách không ổn định.

Các trường control cho `turn-lane` (khác ý nghĩa so với main/other-lane vì trục đảo ngược):

- `longitudinal_offset_mm` lấy từ hệ số chặn (`a0`) của `y(x)` — tương tự vai trò `lateral_offset_mm` nhưng theo trục dọc, vì với turn-lane "lệch theo hướng tiến" mới là đại lượng có ý nghĩa gần điểm rẽ.
- `heading_angle_rad = atan(a1)`.
- `curvature_inv_mm = 2·a2`.
- `lookahead_x_mm = 0.0` (không có ý nghĩa với turn-lane vì hàm là `y(x)` không phải `x(y)`).
- `lookahead_d_mm` vẫn lấy từ công thức lookahead động chung (mục 5.3).
- `lookahead_theta_rad` lấy từ giá trị heading đã làm mượt.

## 5.7. Xoá lane không đủ waypoint

Sau toàn bộ xử lý, node xoá mọi object lane (main/other/turn) có `waypoints.size() < 2`. Đây là safeguard đã nêu ở chương 3 mục 3.3 — nếu không xoá, các trường offset mặc định `0.0` có thể bị downstream hiểu nhầm là "làn đang ngay trước xe, lệch 0mm" thay vì "không đủ dữ liệu để đo".

## 5.8. Debug centerline

Nếu tham số `publish_debug_centerline = true`, mỗi object lane có thêm trường `debug_centerline` (chỉ xuất hiện trên topic debug, xoá trước khi publish topic production) gồm 3 mảng, hữu ích khi cần xem lại từng bước thuật toán slicing:

- `slices`: mỗi phần tử là `{y, x_left, x_right}` (main/other-lane) hoặc `{x, y_bottom, y_top}` (turn-lane), làm tròn 1 chữ số thập phân — đây là dữ liệu **ngay sau bước 4** ở mục 5.5 (trước khi lọc bloat).
- `raw_midpoints`: midpoint trước khi lọc lát bloated.
- `filtered_midpoints`: midpoint sau khi lọc bloat (bước 8), **trước** khi làm mượt không gian/thời gian (bước 10-12).

Trường `debug_centerline` bị xoá khỏi object trước khi publish lên `/avs/telemetry_realworld` (topic production) — chỉ tồn tại trên `/avs/ipm_debug`.

## 5.9. Ví dụ số: polygon → waypoint → hệ số → lookahead

Dùng lại ví dụ ở chương 2 mục 2.5.4 và 2.7, trình bày lại theo đúng thứ tự bước của node này. Polygon world-frame (rộng đều 700mm, lệch trái dần):

```text
P0=(-350,200)  P1=(350,200)  P2=(400,800)  P3=(-300,800)
```

Quét `Y=200..800` bước 100 → 7 lát, mọi lát `width=700` (không lát nào bloated vì `median=700`, ngưỡng bloat `=910`). Midpoint: `x_mid(y) = (y-200)/12` — ví dụ `y=200→0`, `y=500→25`, `y=800→50`.

Fit `x(y)` bậc 3 (SVD trên 7 điểm thẳng hàng): `a3≈0, a2≈0, a1=1/12≈0.08333, a0=-200/12≈-16.667`.

```text
lateral_offset_mm  = a0       ≈ -16.7 mm
heading_angle_rad  = atan(a1) ≈ 0.0831 rad (≈4.76°)
curvature_inv_mm   = 2·a2     = 0
```

Nếu `heading_angle` đã mượt frame trước là `0.10 rad`, EMA (`alpha=0.25`) cho frame này: `0.25×0.0831 + 0.75×0.10 = 0.0958 rad`.

Giả sử `current_speed_mms=1500` → `lookahead_d = clamp(1500×0.15, 120, 450) = 225mm` (nằm trong khoảng dữ liệu 200-800mm, tức nội suy chứ không ngoại suy). `x(225) = 0.0833×225-16.667 ≈ 2.08mm`, `lookahead_theta_rad = atan2(2.08,225) ≈ 0.0092 rad (≈0.53°)` — đây là `lookahead_x_mm`/`lookahead_theta_rad` được ghi vào JSON.
