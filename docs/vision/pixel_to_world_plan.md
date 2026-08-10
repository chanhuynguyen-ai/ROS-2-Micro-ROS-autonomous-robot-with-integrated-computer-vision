# Kế Hoạch Chuyển Đổi Tọa Độ Pixel → Tọa Độ Thực (mm)

## 1. Mục Tiêu

Chuyển đổi kết quả phân đoạn làn đường (main-lane, other-lane, turn-lane) từ **tọa độ pixel** sang **tọa độ thế giới thực (mm)**, sau đó:
1. Fit đa thức bậc 3 cho mỗi làn đường
2. Trích xuất: **độ lệch ngang (lateral offset)**, **độ lệch dọc (longitudinal offset)**, **góc xoay (heading angle)**

---

## 2. Tổng Quan Pipeline

```
Segmentation Mask (pixel) 
    → Trích xuất contour/skeleton của làn đường
    → Chuyển đổi pixel → tọa độ thực (mm) trên mặt phẳng đường
    → Fit đa thức bậc 3: y(x) = a₃x³ + a₂x² + a₁x + a₀
    → Trích xuất: lateral offset (a₀), heading angle (arctan(a₁)), curvature (2·a₂)
```

---

## 3. Ba Phương Án Đề Xuất

### 3.1 Phương Án A: Homography IPM (Inverse Perspective Mapping) — ⭐ KHUYẾN NGHỊ

**Nguyên lý:** Sử dụng ma trận phối cảnh (Homography 3×3) để warp ảnh camera thành ảnh nhìn từ trên xuống (Bird's Eye View - BEV). Trong BEV, 1 pixel tương ứng tỉ lệ cố định với khoảng cách thực (mm/pixel).

**Ưu điểm:**
- Đơn giản nhất, không cần biết thông số camera nội tại (intrinsic)
- Chỉ cần chọn 4 điểm tương ứng giữa ảnh gốc và mặt phẳng thực
- Tính toán nhanh, phù hợp chạy real-time trên Pi 5
- OpenCV hỗ trợ sẵn: `getPerspectiveTransform()` + `warpPerspective()`

**Nhược điểm:**
- Giả định mặt đường phẳng (flat road assumption)
- Cần hiệu chuẩn lại nếu thay đổi góc gắn camera

**Các bước thực hiện:**

#### Bước 1: Hiệu chuẩn Homography (1 lần)
```
1. Đặt xe trên mặt phẳng, đặt tấm bảng hiệu chuẩn (checkerboard hoặc 4 điểm đánh dấu)
   trên mặt đường với khoảng cách đã biết (ví dụ: hình chữ nhật 500mm × 800mm)
2. Chụp ảnh từ camera
3. Chọn 4 điểm nguồn (src) trên ảnh camera (pixel)
4. Định nghĩa 4 điểm đích (dst) tương ứng trong tọa độ thực (mm)
5. Tính: H = cv::getPerspectiveTransform(src, dst)
6. Lưu ma trận H vào config/calibration.json
```

#### Bước 2: Chuyển đổi tọa độ tại runtime
```cpp
// Với mỗi điểm pixel (u, v) trên contour làn đường:
cv::Mat pt_pixel = (cv::Mat_<double>(3,1) << u, v, 1.0);
cv::Mat pt_world = H * pt_pixel;
double X_mm = pt_world.at<double>(0) / pt_world.at<double>(2);
double Y_mm = pt_world.at<double>(1) / pt_world.at<double>(2);
```
> **Lưu ý:** Không cần warp toàn bộ ảnh. Chỉ cần transform các điểm contour → tiết kiệm CPU.

#### Bước 3: Fit đa thức bậc 3
```
Với tập điểm {(X_i, Y_i)} của mỗi làn:
  Y(X) = a₃X³ + a₂X² + a₁X + a₀
Sử dụng Least Squares hoặc cv::polyfit tương đương.
```

#### Bước 4: Trích xuất thông số điều khiển
```
Tại vị trí xe (X=0):
  - Lateral offset  = a₀ (mm)
  - Heading angle   = arctan(a₁) (rad)
  - Curvature       = 2·a₂ (1/mm)
```

**Độ chính xác ước tính:** ±10-30mm trong phạm vi 1-2m trước xe

---

### 3.2 Phương Án B: Camera Calibration Đầy Đủ (Intrinsic + Extrinsic)

**Nguyên lý:** Sử dụng mô hình pinhole camera đầy đủ với:
- **Intrinsic (K):** focal length (fx, fy), optical center (cx, cy), distortion coefficients
- **Extrinsic (R, T):** chiều cao camera, góc pitch/yaw/roll so với mặt đường

Từ đó tính chính xác tọa độ 3D trên mặt phẳng đường (Z=0) cho mỗi pixel.

**Ưu điểm:**
- Chính xác nhất (có thể đạt ±5mm ở khoảng cách gần)
- Xử lý được méo ống kính (lens distortion)
- Có thể mở rộng cho bề mặt không phẳng (nếu có thêm thông tin depth)

**Nhược điểm:**
- Phức tạp hơn để setup (cần checkerboard calibration)
- Cần đo chính xác chiều cao và góc nghiêng camera
- Nhạy cảm với rung lắc thay đổi extrinsic

**Các bước thực hiện:**

#### Bước 1: Calibrate Intrinsic
```bash
# Chụp 15-20 ảnh checkerboard ở các góc khác nhau
# Sử dụng OpenCV calibrateCamera() để tính K và distortion coefficients
```

#### Bước 2: Xác định Extrinsic
```
- Đo chiều cao camera so với mặt đường: h (mm)
- Đo góc pitch (nghiêng xuống): α (rad)
- Tính ma trận xoay R và vector tịnh tiến T
```

#### Bước 3: Back-projection lên mặt phẳng đường
```
Với mỗi pixel (u, v):
  ray = K⁻¹ · [u, v, 1]ᵀ           // tia chiếu trong hệ camera
  ray_world = Rᵀ · ray              // chuyển sang hệ thế giới
  t = -h / ray_world.z              // giao với mặt phẳng Z=0
  X_world = ray_world.x * t (mm)
  Y_world = ray_world.y * t (mm)
```

**Độ chính xác ước tính:** ±5-15mm trong phạm vi 1-2m

---

### 3.3 Phương Án C: Fixed-Height Geometric Projection (Đơn giản hóa)

**Nguyên lý:** Giả định camera cố định ở chiều cao `h` và góc pitch `α` đã biết. Sử dụng công thức hình học đơn giản (tam giác đồng dạng) để tính khoảng cách thực mà không cần calibrate đầy đủ.

**Ưu điểm:**
- Cực kỳ đơn giản, không cần calibration phức tạp
- Tính toán nhanh nhất
- Dễ hiểu và debug

**Nhược điểm:**
- Độ chính xác thấp nhất (không xử lý lens distortion)
- Chỉ chính xác ở vùng trung tâm ảnh, sai số lớn ở biên

**Công thức cốt lõi:**
```
Khoảng cách dọc (longitudinal):
  D_long = h * f / (v - v₀)

Khoảng cách ngang (lateral):
  D_lat = D_long * (u - u₀) / f

Trong đó:
  h   = chiều cao camera (mm)
  f   = focal length (pixel)
  v₀  = optical center Y (pixel, thường = image_height/2)
  u₀  = optical center X (pixel, thường = image_width/2)
  (u,v) = tọa độ pixel
```

**Độ chính xác ước tính:** ±30-80mm

---

## 4. Bảng So Sánh

| Tiêu chí | A: Homography IPM | B: Full Calibration | C: Fixed-Height |
|---|---|---|---|
| **Độ phức tạp setup** | Thấp (4 điểm) | Cao (checkerboard) | Rất thấp (đo tay) |
| **Độ chính xác** | ±10-30mm | ±5-15mm | ±30-80mm |
| **Thời gian tính toán** | ~0.1ms/frame | ~0.2ms/frame | ~0.05ms/frame |
| **Xử lý lens distortion** | Không | Có | Không |
| **Giả định flat road** | Có | Có (cơ bản) | Có |
| **Phù hợp Pi 5** | ✅ Rất phù hợp | ✅ Phù hợp | ✅ Phù hợp |
| **Dễ mở rộng** | Trung bình | Cao | Thấp |

---

## 5. Đề Xuất: Phương Án A (Homography IPM)

### Lý do chọn:
1. **Cân bằng tốt nhất** giữa độ chính xác và độ phức tạp
2. **Không cần warp toàn bộ ảnh** — chỉ transform các điểm contour → cực nhanh
3. **Camera USB cố định** trên xe → calibrate 1 lần là đủ
4. **Đường thử nghiệm phẳng** → giả định flat road hoàn toàn hợp lý
5. **Dễ tích hợp** vào pipeline C++ hiện có trong `ncnn_inference_node`

### Hệ tọa độ đề xuất (Vehicle-Centric BEV):

```
        X (dọc, hướng trước xe, mm)
        ↑
        |
        |     ← main-lane
        |
  ------+-----→ Y (ngang, sang phải, mm)
        |
      (0,0) = tâm trước xe (vị trí camera chiếu xuống đường)
```

### Output cho bộ điều khiển:

Mỗi làn đường sẽ được biểu diễn bằng:
```json
{
  "lane_id": "main-lane",
  "polynomial": {
    "a3": 0.000001,
    "a2": -0.0005,
    "a1": 0.02,
    "a0": 150.0
  },
  "lateral_offset_mm": 150.0,
  "heading_angle_rad": 0.02,
  "curvature_inv_mm": -0.001,
  "confidence": 0.92,
  "valid_range_mm": [100, 1500]
}
```

---

## 6. Quy Trình Triển Khai (Nếu Chọn Phương Án A)

### Phase 1: Calibration Tool
- Viết script Python để chọn 4 điểm trên ảnh camera
- Nhập tọa độ thực tương ứng (mm)
- Tính và lưu ma trận H vào `config/calibration.json`

### Phase 2: Lane Skeleton Extraction
- Từ segmentation mask → trích xuất đường trung tâm (skeleton/centerline) của mỗi làn
- Phương pháp: morphological thinning hoặc lấy trung bình contour trái-phải

### Phase 3: Coordinate Transform + Polynomial Fit
- Transform điểm skeleton từ pixel → mm qua ma trận H
- Fit đa thức bậc 3 bằng Least Squares
- Trích xuất a₀ (lateral offset), arctan(a₁) (heading angle)

### Phase 4: Tích hợp vào ROS2 Pipeline
- Thêm module mới trong `ncnn_inference_node.cpp` hoặc tạo node riêng
- Publish kết quả dạng JSON trên topic `/avs/lane_model`
- Kết nối với bộ điều khiển PID/Stanley trong `coordinatesPID`

---

## 7. Yêu Cầu Phần Cứng Cho Calibration

| Vật liệu | Mục đích |
|---|---|
| Tấm bìa/giấy A3 với 4 dấu chấm | Làm điểm tham chiếu trên mặt đường |
| Thước đo (>1m) | Đo khoảng cách thực giữa các điểm |
| Camera USB cố định trên xe | Chụp ảnh calibration |

---

## 8. Tham Khảo

- **Inverse Perspective Mapping (IPM):** Kỹ thuật chuẩn trong autonomous driving, warp ảnh camera thành BEV bằng homography
- **OpenCV Functions:** `getPerspectiveTransform()`, `perspectiveTransform()`, `warpPerspective()`
- **Polynomial Fitting:** `cv::solve()` hoặc Eigen least squares cho fit bậc 3
- **Lane Model Standard:** ISO 17361 — lane representation bằng cubic polynomial là tiêu chuẩn công nghiệp

---

## 9. BEV Valid Region — Horizon Clipping (Plan E1, 2026-07-06)

Homography chỉ là ánh xạ mặt đường hợp lệ **dưới đường chân trời**; pixel tại/trên horizon (`w = h20*u + h21*v + h22 → 0`) back-project ra ±hàng trăm mét hoặc ra sau xe. `ipm_transform_node` clip mọi polygon theo vùng BEV hợp lệ trước khi extract waypoint — logic thuần trong `ros2_ws/src/avs_perception/include/avs_perception/bev_region.hpp` (`BevRegion::clip_and_project`, Sutherland–Hodgman: clip pixel-space theo horizon → chiếu qua H → clip world-space theo box; homography bảo toàn đường thẳng nên nội suy cạnh trước khi chiếu là chính xác).

Tham số ROS (default trong `bev_region.hpp`):

| Param | Default | Ý nghĩa |
|---|---|---|
| `bev_horizon_margin_px` | 10.0 | Chỉ nhận pixel dưới horizon ít nhất margin này (`v >= v_h(u) + margin`, `v_h(u) = -(h20*u + h22)/h21`) |
| `bev_y_max_mm` | 8000.0 | Tầm nhìn trước hữu dụng; `y_min = 0` cố định (không nhận điểm sau xe) |
| `bev_x_abs_max_mm` | 4000.0 | Nửa bề rộng ngang vùng BEV |

Lưới an toàn: `kMaxWaypointsPerObject = 128`/object (WARN khi chạm — dấu hiệu dữ liệu upstream bất thường). Chi tiết + bằng chứng: `../plans/plan_E_ipm_horizon_and_far_field.md`.
