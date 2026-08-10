# Lý Thuyết: Chuyển Đổi Tọa Độ Pixel → Tọa Độ Thực Bằng Homography

## 1. Bài Toán

Camera gắn trên xe nhìn xuống đường. Ảnh camera bị **méo phối cảnh** (perspective distortion): vật ở xa trông nhỏ hơn vật ở gần, các đường song song hội tụ về 1 điểm.

**Mục tiêu:** Chuyển tọa độ pixel `(u, v)` trên ảnh camera thành tọa độ thực `(X_mm, Y_mm)` trên mặt đường, đơn vị milimet.

---

## 2. Homography Là Gì?

Homography là một **phép biến đổi hình học** ánh xạ điểm từ mặt phẳng này sang mặt phẳng khác. Trong trường hợp của chúng ta:

- **Mặt phẳng nguồn:** Ảnh camera (pixel)
- **Mặt phẳng đích:** Mặt đường thực (mm)

Phép biến đổi được biểu diễn bằng **ma trận H kích thước 3×3**:

```
┌ X_mm ┐       ┌ h11  h12  h13 ┐   ┌ u ┐
│ Y_mm │ = s · │ h21  h22  h23 │ × │ v │
└  1   ┘       └ h31  h32  h33 ┘   └ 1 ┘

Trong đó:
  (u, v)       = tọa độ pixel trên ảnh camera
  (X_mm, Y_mm) = tọa độ thực trên mặt đường (mm)
  s            = hệ số tỉ lệ (tính tự động)
  H            = ma trận homography 3×3
```

Sau khi nhân ma trận:
```
X_mm = (h11·u + h12·v + h13) / (h31·u + h32·v + h33)
Y_mm = (h21·u + h22·v + h23) / (h31·u + h32·v + h33)
```

---

## 3. Cách Tính Ma Trận H (Calibration)

Ma trận H có 8 ẩn số (h33 chuẩn hóa = 1). Cần tối thiểu **4 cặp điểm** tương ứng giữa ảnh và thực tế.

### Ví dụ minh họa:

Đặt một vật hình chữ nhật (kích thước đã biết) trên mặt đường, trước camera:

```
Ảnh camera (pixel):                Mặt đường thực (mm):
                                   
    P1●───────●P2                  P1●───────────●P2
     \         /                   │              │
      \       /     ──────→        │              │
       \     /       H             │              │
    P4●───────●P3                  P4●───────────●P3

  Hình thang (do phối cảnh)        Hình chữ nhật (kích thước thực)
```

| Điểm | Pixel (u, v) | Thực (X_mm, Y_mm) |
|------|-------------|-------------------|
| P1 | (185, 120) | (0, 0) |
| P2 | (455, 120) | (297, 0) |
| P3 | (410, 350) | (297, 210) |
| P4 | (230, 350) | (0, 210) |

OpenCV tính H từ 4 cặp điểm này:
```cpp
Mat H = getPerspectiveTransform(src_points, dst_points);
```

---

## 4. Hệ Tọa Độ Thực (Vehicle-Centric)

```
      Y (mm) — hướng trước xe (dọc / longitudinal)
      ↑
      │
      │    ← làn đường trải dài theo Y
      │
      O────────→ X (mm) — sang phải (ngang / lateral)
    (0,0)
    = điểm chiếu camera xuống mặt đường
```

- **X (mm):** Độ lệch ngang (lateral offset). X > 0 = sang phải, X < 0 = sang trái.
- **Y (mm):** Khoảng cách phía trước xe (longitudinal). Y > 0 = phía trước.

---

## 5. Fit Đa Thức Bậc 3

Sau khi chuyển contour làn đường từ pixel sang mm, ta có tập điểm `{(Xᵢ, Yᵢ)}` cho mỗi làn.

Vì làn đường trải dài theo trục **Y (dọc)**, ta biểu diễn vị trí ngang **X** theo hàm của **Y**:

```
x(y) = a₃·y³ + a₂·y² + a₁·y + a₀
```

### Ý nghĩa các hệ số tại vị trí xe (y = 0):

| Hệ số | Ý nghĩa | Công thức |
|-------|---------|-----------|
| **a₀** | **Lateral offset** — Độ lệch ngang so với làn (mm) | `x(0) = a₀` |
| **a₁** | **Heading angle** — Góc giữa hướng xe và đường cong (rad) | `ψ = arctan(a₁)` |
| **a₂** | **Curvature** — Độ cong của đường tại vị trí xe (1/mm) | `κ = 2·a₂` |
| **a₃** | **Rate of curvature change** — Tốc độ thay đổi độ cong | Dùng cho dự đoán xa |

### Minh họa trực quan:

```
      Y (mm)
      ↑
      │         ╱ ← đường cong thực tế
      │        ╱
      │       ╱
      │      │
      │     │
      │    │ 
      O──│──────→ X (mm)
         │
         ← a₀ = lateral offset (khoảng cách ngang tại y=0)
         
         ψ = arctan(a₁) = góc tiếp tuyến tại y=0
```

---

## 6. Phương Pháp Fit: Least Squares

Cho N điểm `{(Xᵢ, Yᵢ)}`, tìm `[a₃, a₂, a₁, a₀]` sao cho tổng bình phương sai số nhỏ nhất:

```
minimize Σᵢ (Xᵢ - a₃·Yᵢ³ - a₂·Yᵢ² - a₁·Yᵢ - a₀)²
```

Viết dưới dạng ma trận: **A · c = b**

```
    ┌ Y₁³  Y₁²  Y₁  1 ┐       ┌ a₃ ┐       ┌ X₁ ┐
A = │ Y₂³  Y₂²  Y₂  1 │,  c = │ a₂ │,  b = │ X₂ │
    │  ⋮    ⋮    ⋮   ⋮ │       │ a₁ │       │  ⋮ │
    └ Yₙ³  Yₙ²  Yₙ  1 ┘       └ a₀ ┘       └ Xₙ ┘
```

Nghiệm: `c = (AᵀA)⁻¹ · Aᵀ · b`

Trong OpenCV C++:
```cpp
cv::Mat A(N, 4, CV_64F);
cv::Mat b(N, 1, CV_64F);
for (int i = 0; i < N; i++) {
    double y = points[i].y;
    A.at<double>(i, 0) = y*y*y;
    A.at<double>(i, 1) = y*y;
    A.at<double>(i, 2) = y;
    A.at<double>(i, 3) = 1.0;
    b.at<double>(i, 0) = points[i].x;
}
cv::Mat coeffs;
cv::solve(A, b, coeffs, cv::DECOMP_QR);
// coeffs = [a3, a2, a1, a0]
```

---

## 7. Trích Xuất Centerline Từ Mask Phân Đoạn

Segmentation mask cho ra **vùng diện tích** của làn đường (main-lane, other-lane, turn-lane). Để fit polynomial, cần trích xuất **đường trung tâm (centerline)**.

### Phương pháp: Midpoint Extraction

```
Mask làn đường tại mỗi hàng pixel (row):

Row 100: ....████████....    → leftmost=50, rightmost=110 → center=80
Row 120: ...██████████...    → leftmost=45, rightmost=115 → center=80
Row 140: ..████████████..    → leftmost=40, rightmost=120 → center=80
Row 160: .██████████████.    → leftmost=35, rightmost=125 → center=80
```

Cho mỗi hàng `v` có pixel thuộc mask, lấy trung bình X:
```
center_u = (leftmost_u + rightmost_u) / 2
→ Điểm centerline: (center_u, v) trong pixel
→ Transform qua H: (X_mm, Y_mm) trong mm
```

---

## 8. Giả Định & Hạn Chế

| Giả định | Hệ quả |
|----------|--------|
| Mặt đường phẳng | Nếu mặt đường có độ dốc hoặc gồ ghề, tọa độ mm sẽ bị sai |
| Camera cố định trên xe | Nếu camera bị xê dịch, cần calibrate lại |
| Vùng nhìn thấy hữu hạn | Polynomial chỉ chính xác trong phạm vi camera nhìn thấy |

Với dự án hiện tại (băng rôn in trên sàn phẳng), tất cả giả định đều thỏa mãn.
