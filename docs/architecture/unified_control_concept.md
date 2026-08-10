# Ý Tưởng Thiết Kế: Hệ Thống Điều Khiển Đồng Nhất (Unified Control Concept)

Tài liệu này mô tả ý tưởng đồng nhất hóa đầu vào cho bộ điều khiển xe tự hành bằng cách quy đổi mọi dạng đường cong làn đường về các điểm mục tiêu (waypoint) cục bộ, tham chiếu trực tiếp với một hệ tọa độ gốc cố định gắn trên xe.

---

## 1. Hệ Tọa Độ Gốc Cố Định (Fixed Robot-Centric Frame)

Gốc tọa độ được đặt cố định tại **điểm chiếu của mép dưới - chính giữa khung hình camera xuống mặt đất** (tương ứng với tâm trước mũi xe).

*   **Trục X (Trục ngang / Lateral):** Nằm ngang, hướng sang bên phải xe.
    *   $X > 0$: Lệch về bên phải xe.
    *   $X < 0$: Lệch về bên trái xe.
*   **Trục Y (Trục dọc / Longitudinal):** Nằm dọc, hướng thẳng về phía trước theo hướng mũi xe.
    *   $Y > 0$: Khoảng cách phía trước xe.
*   **Gốc tọa độ $O(0, 0)$:** Tâm đầu xe. Hướng tiến của xe luôn trùng với tia $OY$.

---

## 2. Quy Trình Trích Xuất Waypoint Từ Các Đường Cong

Sau khi qua ma trận Homography IPM để chuyển đổi từ pixel sang tọa độ thực (mm), hệ thống tiến hành fit các đa thức riêng biệt:

1.  **Làn đi thẳng (main-lane, other-lane):**
    *   Fit theo dạng: $x(y) = a_3y^3 + a_2y^2 + a_1y + a_0$
    *   Cách lấy tập hợp waypoint: Tính giá trị $x$ bằng cách chạy $y$ tăng dần từ khoảng cách gần nhất nhìn thấy ($y_{min}$) đến xa nhất ($y_{max}$).
2.  **Làn rẽ ngang (turn-lane):**
    *   Fit theo dạng: $y(x) = b_3x^3 + b_2x^2 + b_1x + b_0$
    *   Cách lấy tập hợp waypoint: Tính giá trị $y$ bằng cách chạy $x$ tăng dần/giảm dần dọc theo chiều rộng của làn rẽ.

### Xác định Waypoint Mục Tiêu $P_{target}(X_t, Y_t)$

Từ tập hợp các waypoint của làn đường hiện tại đang chọn để bám (active lane):
*   Hệ thống chọn ra waypoint **gần mép dưới ảnh nhất** (tương ứng điểm trên làn đường gần đầu xe nhất trong tọa độ thực).
*   Điểm này được ký hiệu là: $P_{target} = (X_t, Y_t)$ (đơn vị: mm).

---

## 3. Mục Tiêu Điều Khiển (Control Objective)

Mục tiêu cốt lõi của bộ điều khiển (PID, Stanley, hay Pure Pursuit) là tạo ra góc lái và tốc độ sao cho:

$$\text{Điều khiển xe để } P_{target}(X_t, Y_t) \to O(0, 0)$$

Tức là kéo điểm bắt đầu của làn đường trùng khít với tâm mũi xe. Khi điểm này dịch chuyển do đường cong hoặc ngã rẽ, bộ điều khiển sẽ tự động bẻ lái để bám đuổi theo điểm này.

---

## 4. Gợi Ý Các Tham Số Đầu Vào Cho Bộ Điều Khiển

Dựa trên tọa độ điểm mục tiêu $P_{target}(X_t, Y_t)$ và các hệ số đa thức, các tham số đầu vào được trích xuất đồng nhất bao gồm:

### 4.1. Sai số lệch ngang (Lateral Error / Cross-Track Error - $e_x$)
*   **Ý nghĩa:** Khoảng cách lệch trái/phải của làn đường so với tâm xe.
*   **Công thức:**
    $$e_x = X_t \text{ (mm)}$$
*   *Nếu $e_x > 0$: Làn đường lệch sang phải (xe đang lệch trái) $\to$ Đánh lái sang phải.*
*   *Nếu $e_x < 0$: Làn đường lệch sang trái (xe đang lệch phải) $\to$ Đánh lái sang trái.*

### 4.2. Sai số lệch dọc (Longitudinal Error - $e_y$)
*   **Ý nghĩa:** Khoảng cách từ đầu xe đến điểm bắt đầu của làn đường.
*   **Công thức:**
    $$e_y = Y_t \text{ (mm)}$$
*   **Ứng dụng:**
    *   Sử dụng làm khoảng cách xem trước (Look-ahead distance) trong các bộ điều khiển hình học (như Pure Pursuit).
    *   Điều tiết tốc độ: Nếu $e_y$ quá nhỏ (ngã rẽ nằm sát đầu xe) $\to$ Giảm tốc độ để vào cua an toàn.

### 4.3. Sai số góc hướng (Heading Error - $e_{\theta}$)
*   **Ý nghĩa:** Góc lệch giữa hướng đầu xe (trục Y) và hướng tiếp tuyến của làn đường tại điểm $P_{target}$.
*   **Cách tính đồng nhất bằng Vector (Tránh phân biệt dạng đa thức):**
    *   Tìm vector tiếp tuyến $\vec{T} = (dx, dy)$ tại $P_{target}$:
        *   Với đa thức dọc $x(y)$: $dx = (3a_3Y_t^2 + 2a_2Y_t + a_1) \cdot dy$, chọn $dy = 1.0$.
        *   Với đa thức ngang $y(x)$: $dy = (3b_3X_t^2 + 2b_2X_t + b_1) \cdot dx$, chọn $dx = 1.0$ (hoặc $-1.0$ tùy hướng rẽ).
    *   Vector hướng xe: $\vec{V} = (0, 1)$ (hướng dọc theo trục Y).
    *   Góc lệch hướng:
        $$e_{\theta} = \text{atan2}(dx, dy) \text{ (rad)}$$

### 4.4. Độ cong làn đường (Curvature - $\kappa$)
*   **Ý nghĩa:** Độ cong vật lý của làn đường tại điểm đích, phục vụ cho điều khiển bù lái trước (Feed-forward steering control).
*   **Công thức:**
    *   Với làn dọc: $\kappa \approx 2a_2$ (tại vị trí gần xe).
    *   Với làn ngang: $\kappa \approx 2b_2$ (tại vị trí gần xe).

---

## 5. Ưu Điểm của Thiết Kế Đồng Nhất Này

1.  **Độc lập thuật toán:** Bộ điều khiển không cần biết cấu trúc toán học của làn đường ($x(y)$ hay $y(x)$). Nó chỉ cần nhận đầu vào là một điểm tọa độ $(X_t, Y_t)$ và một góc lệch tiếp tuyến $e_{\theta}$.
2.  **Mượt mà khi chuyển đổi trạng thái:** Khi chuyển từ bám làn thẳng sang bám làn rẽ, điểm mục tiêu $P_{target}$ chỉ chuyển dịch tọa độ trên mặt phẳng 2D. Bộ điều khiển phản ứng liên tục mà không bị gián đoạn hay đổi chế độ logic.
3.  **Dễ dàng Debug:** Ta có thể vẽ trực tiếp tọa độ $P_{target}$ và vector tiếp tuyến $\vec{T}$ lên màn hình mô phỏng hoặc dashboard để kiểm tra tính đúng đắn của dữ liệu trước khi gửi sang điều khiển động cơ.
