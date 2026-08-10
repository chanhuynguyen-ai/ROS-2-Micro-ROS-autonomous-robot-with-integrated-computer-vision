# Hướng Dẫn Thiết Kế Bộ Điều Khiển Pure Pursuit Từ Sai Số Làn Đường AVS

Tài liệu này hướng dẫn cách chuyển đổi các tham số sai số hình học đầu ra từ hệ thống thị giác AVS (`/avs/control_error`) thành lệnh điều khiển góc lái ($\omega$) và tốc độ ($v$) cho robot bằng thuật toán **Pure Pursuit**.

---

## 1. Các Tham Số Đầu Vào Từ AVS

Hệ thống AVS xuất dữ liệu dưới dạng chuỗi JSON lên topic `/avs/control_error` với các trường cốt lõi sau:

| Ký hiệu toán học | Trường JSON | Đơn vị | Ý nghĩa vật lý |
| :--- | :--- | :--- | :--- |
| $e_x$ | `epsilon_x_mm` | $\text{mm}$ | Sai lệch ngang (lateral error) từ tâm robot đến đường đi. Dương nếu mục tiêu ở bên phải xe. |
| $e_y$ | `epsilon_y_mm` | $\text{mm}$ | Khoảng cách nhìn trước dọc theo hướng xe, đóng vai trò là khoảng cách nhìn trước $L_d$. |
| $\theta$ | `theta_rad` | $\text{rad}$ | Sai số góc hướng (heading error) giữa xe và hướng làn đường. |
| $\kappa$ (hoặc $k$) | `curvature_inv_mm` | $\text{mm}^{-1}$ | Độ cong vật lý của làn đường hiện tại. Bằng 0 nếu đi thẳng. |

---

## 2. Thiết Kế Bộ Điều Khiển Hướng (Lateral Control - Tính $\omega$)

Thuật toán Pure Pursuit coi chuyển động của xe như một **cung tròn** nối từ tâm xe đến điểm nhìn trước (waypoint mục tiêu) cách xe một khoảng $L_d$.

### 2.1. Quy đổi đơn vị sang Hệ SI (Mét)
Trước khi tính toán, hãy chuyển đổi các tham số từ milimét sang mét để phù hợp với chuẩn của ROS:
*   Sai lệch ngang:
    $$e_{x,m} = \frac{e_x}{1000.0} \quad (\text{m})$$
*   Khoảng cách nhìn trước:
    $$L_d = e_{y,m} = \frac{e_y}{1000.0} \quad (\text{m})$$

### 2.2. Công thức toán học của Pure Pursuit
Từ mối quan hệ hình học trên cung tròn:
1.  Bán kính quay vòng của cung tròn $R$:
    $$R = \frac{L_d^2}{2 e_{x,m}} \quad (\text{m})$$
2.  Độ cong quỹ đạo mong muốn của xe $\gamma$:
    $$\gamma = \frac{1}{R} = \frac{2 e_{x,m}}{L_d^2} \quad (\text{m}^{-1})$$
3.  Vận tốc góc điều khiển $\omega$ (yaw rate) tỉ lệ thuận với vận tốc dài $v$:
    $$\omega = v \cdot \gamma = \frac{2 \cdot v \cdot e_{x,m}}{L_d^2} \quad (\text{rad/s})$$

> [!CAUTION]
> **Tránh chia cho 0:** Nếu $L_d$ tiến sát về 0, phép tính sẽ bị lỗi. Cần đặt một ngưỡng tối thiểu (ví dụ $L_{d,min} = 0.05\text{ m}$) để bảo vệ.

---

## 3. Thiết Kế Điều Khiển Tốc Độ Thích Ứng (Longitudinal Control - Tính $v$)

Để đảm bảo xe không bị lật hoặc trượt bánh khi đi qua các khúc cua gắt hoặc khi lệch hướng lớn, vận tốc dài mong muốn $v$ cần được tự động giảm thiểu dựa trên $\theta$ và $\kappa$.

### 3.1. Quy đổi độ cong sang hệ mét
$$\kappa_m = \kappa \times 1000.0 \quad (\text{m}^{-1})$$

### 3.2. Công thức tính vận tốc thích ứng
$$v_{target} = \frac{v_{max} \cdot \cos(\theta)}{1.0 + k_c \cdot |\kappa_m|} \quad (\text{m/s})$$

Trong đó:
*   $v_{max}$: Vận tốc tối đa chạy trên đường thẳng (ví dụ $0.5\text{ m/s}$).
*   $k_c$: Hệ số phạt độ cong (càng lớn thì xe giảm tốc càng nhiều khi vào cua, khuyến nghị từ $200.0$ đến $500.0$).
*   $\cos(\theta)$: Giảm tốc khi đầu xe lệch góc quá nhiều so với hướng làn.

Sau đó giới hạn $v_{target}$ trong khoảng chạy an toàn $[v_{min}, v_{max}]$ để động cơ không bị tắt đột ngột:
$$v = \text{clamp}(v_{target}, v_{min}, v_{max})$$

---

## 4. Mã Nguồn Tham Khảo (Python & C++)

### 4.1. Thực thi bằng C++ (ROS2 Node)
```cpp
#include <cmath>
#include <algorithm>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

// Cấu hình các giới hạn vật lý
double v_max = 0.5;      // m/s
double v_min = 0.1;      // m/s
double k_c = 300.0;      // Phạt độ cong
double omega_max = 2.0;  // rad/s
double Ld_min = 0.05;    // m (Tránh chia cho 0)

// Hàm xử lý khi nhận được dữ liệu từ /avs/control_error
void processControlError(const std::string& json_str, double& out_v, double& out_omega) {
    auto err = json::parse(json_str);
    
    // Quy đổi đơn vị mm -> m
    double e_x = err.value("epsilon_x_mm", 0.0) / 1000.0;
    double L_d = err.value("epsilon_y_mm", 300.0) / 1000.0;
    double theta = err.value("theta_rad", 0.0);
    double kappa = err.value("curvature_inv_mm", 0.0) * 1000.0; // 1/mm -> 1/m

    // 1. Tính toán vận tốc dài v
    double v = (v_max * std::cos(theta)) / (1.0 + k_c * std::abs(kappa));
    out_v = std::clamp(v, v_min, v_max);

    // 2. Tính toán vận tốc góc omega (Pure Pursuit)
    if (L_d > Ld_min) {
        out_omega = (2.0 * out_v * e_x) / (L_d * L_d);
    } else {
        out_omega = 0.0;
    }
    out_omega = std::clamp(out_omega, -omega_max, omega_max);
}
```

### 4.2. Thực thi bằng Python (ROS2 Node)
```python
import math
import json

# Tham số cấu hình
V_MAX = 0.5       # m/s
V_MIN = 0.1       # m/s
K_C = 300.0       # Phạt độ cong
OMEGA_MAX = 2.0   # rad/s
LD_MIN = 0.05     # m

def calculate_pure_pursuit(json_data_str):
    try:
        data = json.loads(json_data_str)
        
        # Đọc dữ liệu và quy đổi sang hệ SI (m)
        e_x = data.get("epsilon_x_mm", 0.0) / 1000.0
        L_d = data.get("epsilon_y_mm", 300.0) / 1000.0
        theta = data.get("theta_rad", 0.0)
        kappa = data.get("curvature_inv_mm", 0.0) * 1000.0
        
        # 1. Tính toán v
        v = (V_MAX * math.cos(theta)) / (1.0 + K_C * abs(kappa))
        v = max(V_MIN, min(v, V_MAX))
        
        # 2. Tính toán omega
        if L_d > LD_MIN:
            omega = (2.0 * v * e_x) / (L_d * L_d)
        else:
            omega = 0.0
        omega = max(-OMEGA_MAX, min(omega, OMEGA_MAX))
        
        return v, omega
    except Exception as e:
        print(f"Error parsing control error: {e}")
        return 0.0, 0.0
```

---

## 5. Quy Trình Khởi Chạy Và Cơ Chế An Toàn (Khuyến Nghị)

Để đảm bảo an toàn tuyệt đối cho mô hình xe chạy thực tế ngoài thực địa, bộ điều khiển của bên thứ ba cần tích hợp thêm các cơ chế sau:

1.  **Lắng nghe nút bấm trạng thái điều khiển:**
    *   Subscribe thêm topic `/avs/cmd`.
    *   Chỉ khi nhận được `{"cmd": "arm"}` mới gửi lệnh vận tốc thực tế tới động cơ. 
    *   Khi nhận được `{"cmd": "disarm"}` hoặc khi khởi động hệ thống (mặc định SAFE MODE), bắt buộc phải phát đi `/cmd_vel` với các giá trị `0` (`linear.x = 0, angular.z = 0`) để dừng xe.
2.  **Thiết lập Watchdog (Mất kết nối khẩn cấp):**
    *   Cần giám sát thời gian trễ của gói tin nhận được từ `/avs/control_error`.
    *   Nếu sau **$0.5\text{ giây}$** không nhận được dữ liệu mới từ camera (mất tín hiệu thị giác), bộ điều khiển phải tự động ngắt đầu ra và đưa vận tốc xe về 0 ngay lập tức để tránh xe bị mất kiểm soát lao tự do.
