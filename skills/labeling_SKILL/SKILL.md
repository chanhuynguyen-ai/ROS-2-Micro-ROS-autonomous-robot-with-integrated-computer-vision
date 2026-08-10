# 🏷️ Skill: Lane & Marking Labeling Guidelines for AVS

Tài liệu này định nghĩa quy tắc gán nhãn dữ liệu phân đoạn ảnh (Semantic Segmentation) cho dự án AI Autonomous Vision System (AVS). Quy tắc này đảm bảo tập dữ liệu huấn luyện (Dataset) chất lượng cao, giúp mô hình **YOLO26-seg** (`ros2_ws/src/avs_perception/src/yolo26_seg.cpp`) đạt độ chính xác tối đa và tối ưu hóa cho hệ thống lập kế hoạch & điều khiển ROS2.

---

## 1. Danh sách các lớp phân đoạn (Segmentation Classes)

Model hiện tại có **22 class**. Nguồn sự thật duy nhất cho danh sách/ID là `config/label_mapping.json` (build sinh `label_mapping.hpp`) + `models/best_ncnn_model/metadata.yaml` — bảng dưới đây chỉ là tóm tắt, nếu lệch với `label_mapping.json` thì file đó luôn thắng.

### Làn đường (Lanes) — quan trọng nhất cho decision/planning
*   **`main-lane` (ID 6, Làn chính):** Làn đường hiện tại mà xe đang đi (Ego-lane). Đây là làn động (dynamic), đóng vai trò là làn bám mục tiêu chính của xe.
*   **`other-lane` (ID 7, Làn bên cạnh):** Các làn đường liền kề nằm cùng chiều hoặc ngược chiều với làn chính.
*   **`turn-lane` (ID 20, Làn rẽ):** Làn đường được thiết lập cho các cua rẽ trái/phải tại ngã tư. **Bẫy quan trọng nhất của repo: turn-lane = 20, không phải 17 (`solid-yellow`) hay 10 (`sign-no-parking`)** — xem `CLAUDE.md`.

### Vạch kẻ đường (Markings)
*   **`solid-white` (ID 16):** Vạch kẻ đường nét liền màu trắng.
*   **`solid-yellow` (ID 17):** Vạch kẻ đường nét liền màu vàng.
*   **`dashed-white` (ID 0):** Vạch kẻ đường nét đứt màu trắng.
*   **`dashed-yellow` (ID 1):** Vạch kẻ đường nét đứt màu vàng.
*   **`double-solid-white` (ID 2):** Vạch kẻ đường nét liền đôi màu trắng.
*   **`stop-line` (ID 19, Vạch dừng):** Vạch ngang đường màu trắng báo hiệu xe phải dừng trước đèn đỏ hoặc tại giao lộ. Lưu ý: hiện **không** dùng để kích hoạt rẽ/phát hiện giao lộ/chuyển làn trong decision system (xem `AGENTS.md`).

### Đèn tín hiệu (Lights)
*   **`light_green` (ID 3)**, **`light_red` (ID 4)**, **`light_yellow` (ID 5)**.

### Biển báo (Signs)
*   **`sign-no-left` (9)**, **`sign-no-parking` (10)**, **`sign-no-right` (11)**, **`sign-parking` (12)**, **`sign-stop` (13)**, **`sign-turn-left` (14)**, **`sign-turn-right` (15)**.

### Ô đỗ xe (Parking)
*   **`parking-zone` (ID 8, Ô đỗ xe):** Vùng diện tích hình chữ nhật của ô đỗ xe (bao gồm cả ký hiệu chữ P vẽ trên mặt đất). Tên đúng trong `label_mapping.json` là `parking-zone`, không phải `parking-slot`.

### Khác
*   **`start` (ID 18):** Vạch/điểm xuất phát.
*   **`vehicle` (ID 21):** Các phương tiện giao thông khác trên đường (xe máy, ô tô, xe robot khác).

---

## 2. Quy tắc gán nhãn làn đường tại ngã tư (Intersections)

Để bộ điều khiển (PID/Stanley) chuyển đổi mượt mà giữa đi thẳng và rẽ, đồng thời nhận diện giao lộ chính xác, quy tắc gán nhãn được quy định như sau:

> [!IMPORTANT]
> **KHÔNG ĐƯỢC vẽ liền mạch các làn đường (`main-lane`, `other-lane`) xuyên thẳng qua ngã tư.** 

```text
                  │     │     │
                  │other│main │
                  │-lane│-lane│
                  │(Xa) │(Xa) │
──────────────────┘     │     └──────────────────
   GIAO LỘ (KHÔNG gán nhãn làn nào đi xuyên qua)
──────────────────┐     │     ┌──────────────────
                  │other│main │
                  │-lane│-lane│
                  │(Gần)│(Gần)│
                  │  ▲  │  ▲  │
                  │     │[Xe] │
```

### Quy tắc chi tiết:
1.  **Ngắt làn tại vạch dừng:** Mặt nạ của `main-lane` và `other-lane` ở đường hiện tại phải dừng chính xác tại vạch dừng (`stop-line`) trước ngã tư.
2.  **Khởi động lại làn ở phía đối diện:** Làn đường thẳng hướng ở phía bên kia ngã tư sẽ bắt đầu được gán nhãn là `main-lane` (và các làn bên cạnh là `other-lane`) ngay khi nó bắt đầu xuất hiện lại.
3.  **Làn rẽ (`turn-lane`):** Gán nhãn bắt đầu từ vạch dừng của làn rẽ hiện tại, đi theo hình vòng cung rẽ của đường sa bàn, và kết thúc tại điểm nhập làn của đường mới.

---

## 3. Quy tắc gán nhãn vạch dừng (`stop-line`) & Phân biệt với `solid-white`

`stop-line` (vạch dừng ngang) và `solid-white` (vạch dọc liền trắng) đều là các dải sơn màu trắng trên đường. Để tránh mô hình AI bị nhầm lẫn giữa hai lớp này, cần tuân thủ nghiêm ngặt các quy tắc gán nhãn sau:

### 3.1 Quy tắc gán nhãn nhãn học (Annotation Rules)
*   **Gán nhãn đúng hướng (Orientation):** 
    *   `stop-line` luôn có hướng **ngang/vuông góc** (transverse) với hướng chuyển động của xe.
    *   `solid-white` luôn có hướng **dọc/song song** (longitudinal) with hướng di chuyển của xe.
*   **Phân tách ranh giới rõ ràng:**
    *   Tại ngã tư, nơi vạch dọc biên đường (`solid-white`) tiếp giáp với vạch dừng (`stop-line`), **không được vẽ chồng lấn** hai vùng nhãn lên nhau.
    *   Hãy vẽ vạch `stop-line` chạm hoặc giữ khoảng cách tối thiểu 1-2 pixel với `solid-white` để tránh thuật toán học nhầm rằng hai vạch này là một.
*   **Giới hạn vùng vẽ:** Chỉ gán nhãn `stop-line` cho phần vạch ngang trực tiếp chắn làn xe. Không vẽ lan sang lề đường hay vạch đi bộ.

```text
       ĐÚNG: Phân tách rõ ràng              SAI: Vẽ chồng lấn lên nhau
       
          │ solid-white                         │ solid-white
          │                                     │
    ──────┴────── stop-line               ──────┼────── stop-line (Chồng lấn)
                                                │
```

---

## 4. Giải pháp kỹ thuật xử lý nhầm lẫn ở Hậu xử lý (Post-Processing)

Dù mô hình YOLO đã học ngữ cảnh toàn cục (Global Context) để phân biệt rất tốt dựa vào góc nghiêng và hình dáng, bộ lọc hậu xử lý hình học (Geometric Filter) trong code ROS2 vẫn cần được triển khai để tăng độ tin cậy:

### 4.1 Lọc theo tỷ lệ khung hộp giới hạn (Bounding Box Aspect Ratio)
Do vạch dừng nằm ngang và vạch biên dọc nằm dọc, sau khi tìm được contour của mặt nạ:
*   **Vạch dừng (`stop-line`):** Chiều rộng (Width) trong ảnh BEV sẽ lớn hơn nhiều so với chiều cao (Height).
    $$\text{Aspect Ratio} = \frac{W}{H} > 2.0$$
*   **Vạch biên (`solid-white`):** Chiều cao (dọc theo trục Y của xe) sẽ lớn hơn chiều rộng.
    $$\text{Aspect Ratio} = \frac{W}{H} < 0.5$$

### 4.2 Lọc theo góc tiếp tuyến (PCA / Orientation Angle)
Sử dụng phân tích thành phần chính (PCA) hoặc khớp đường thẳng cho contour để tìm góc hướng $\theta$ của vạch so với trục ngang của xe:
*   Nếu góc lệch hướng so với trục ngang (X-axis) nằm trong khoảng $[-15^\circ, +15^\circ] \rightarrow$ Xác nhận là `stop-line`.
*   If góc lệch hướng gần vuông góc với trục ngang (song song trục Y-axis) $\rightarrow$ Xác nhận là `solid-white`.

### 4.3 Xác thực bằng vị trí (Spatial Validation)
*   `stop-line` luôn cắt ngang qua phần cuối của làn đường `main-lane`. Nếu phát hiện thấy một vùng mask nghi ngờ là `stop-line` nhưng lại nằm song song bên cạnh `main-lane`, hệ thống sẽ tự động phân loại lại hoặc loại bỏ.

---

## 5. Quy tắc gán nhãn & xử lý ô đỗ xe (`parking-zone`)

Không giống các làn đường dài vô tận được biểu diễn bằng đa thức bám đường cong, ô đỗ xe (`parking-zone`, ID 8 — không phải `parking-slot`, tên đó không tồn tại trong `label_mapping.json`) là một vùng hình chữ nhật khép kín, được định vị như một điểm đích (Goal Pose) cố định.

### 5.1 Quy tắc gán nhãn
*   Vẽ đa giác gán nhãn bao phủ **toàn bộ diện tích hình chữ nhật** của ô đỗ xe.
*   Tô kín ô bao gồm cả chữ P vẽ trên mặt đất bên trong. Không cần phân tách chữ P ra làm nhãn riêng.

### 5.2 Giải thuật trích xuất tọa độ mục tiêu (Goal Pose Extraction)
Do không thể fit đường cong cho ô đỗ xe dạng hộp, ta cần trích xuất một điểm đích có tọa độ 2D và hướng góc quay `Goal Pose: (x_g, y_g, theta_g)` từ mặt nạ:
1.  **Tìm hộp bao xoay tối ưu (Oriented Bounding Box):**
    Sử dụng hàm OpenCV `cv::minAreaRect` trên contour của mặt nạ `parking-zone` để tìm:
    *   Tâm hình học của ô đỗ trên ảnh: $C(u_c, v_c)$
    *   Chiều dài và chiều rộng của ô đỗ: $(W, H)$
    *   Góc nghiêng của ô đỗ trên ảnh: $\theta_{pixel}$
2.  **Chuyển đổi qua Homography sang tọa độ thực (mm):**
    *   Tâm ảnh $C(u_c, v_c) \rightarrow P_{target}(X_g, Y_g)$ (Tọa độ thực tế ảo tâm ô đỗ).
    *   Góc ảnh $\theta_{pixel} \rightarrow \theta_{slot}$ (Góc xoay thực tế của ô đỗ).
3.  **Xác định hướng đi vào ô:**
    Từ kích thước $W$ và $H$, xác định cạnh ngắn hơn là lối vào (entrance). Định nghĩa Goal Pose nằm ở tâm ô đỗ xe, có hướng $\theta_g = \theta_{slot}$ dọc theo chiều dọc của ô đỗ.

---

## 6. Tích hợp Điều khiển & Lập hành trình (State Machine Integration) — ĐỀ XUẤT, CHƯA TRIỂN KHAI

**Bảng dưới đây là thiết kế đề xuất, không phải trạng thái đang chạy trong `control_node.cpp` hiện tại.** Không có `PoseController`/parking state machine nào trong code (`rg "PoseController|parking"` trong `control_node.cpp` không ra kết quả). Ngoài ra một số hàng mâu thuẫn trực tiếp với nguyên tắc kiến trúc hiện hành trong `CLAUDE.md`/`AGENTS.md`:

- `stop-line` **hiện KHÔNG** được dùng để kích hoạt phanh/dừng, phát hiện giao lộ, hay bất kỳ quyết định lái nào (đây là chủ đích thiết kế giai đoạn hiện tại, không phải thiếu sót).
- Phát hiện giao lộ/vào-thoát khúc cua hiện dựa hoàn toàn vào hình học `turn-lane` (label 20) và cơ chế trajectory manager/`TrajectoryLatch` trong `control_node.cpp` (xem `docs/architecture/decision_sys.md` và `system_report_current/05_control_node_va_control_error.md`), phức tạp hơn nhiều so với bảng state đơn giản dưới đây.

| Trạng thái (State) | Nhận diện đầu vào | Hành động điều khiển | Trạng thái triển khai |
|---|---|---|---|
| **Bám làn (Lane Following)** | Có `main-lane` độ dài tốt. | Fit đa thức bám tâm `main-lane`. | Đã triển khai (dạng phức tạp hơn, xem `control_node.cpp`). |
| **Gặp vạch dừng (Stop Line Detected)** | Có `stop-line` phía trước. | (Đề xuất) Kiểm tra đèn giao thông → phanh/giảm tốc. | **Chưa triển khai** — `stop-line` không dùng để quyết định lái hiện tại. |
| **Vào/thoát giao lộ** | Hình học `turn-lane` xuất hiện/biến mất. | Trajectory planner + `TrajectoryLatch` xử lý (phức tạp hơn nhiều bảng này). | Đã triển khai, khác cơ chế mô tả ở đây — xem chương 6 `system_report_current/05_...md`. |
| **Đỗ xe (Parking Mode)** | Có nhãn `parking-zone` trong tầm quét gần. | (Đề xuất) Bộ điều khiển Goal Pose `(x_g, y_g, theta_g)` riêng. | **Chưa triển khai** — không có `PoseController`/parking logic trong `control_node.cpp`. |
