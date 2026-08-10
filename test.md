Dưới góc độ toán học máy tính, hàm `cv2.getPerspectiveTransform(src, dst)` của OpenCV giải quyết bài toán tìm ma trận Homography $H$ ($3 \times 3$) bằng phương pháp **Biến đổi tuyến tính trực tiếp (Direct Linear Transform - DLT)**.

Dưới đây là mô tả quy trình tính toán chi tiết dựa trên các điểm giả định lấy từ chính tệp cấu hình thực tế của hệ thống (`calibration.json`).

---

### 1. Thiết lập các điểm giả định (4 cặp điểm không thẳng hàng)

* **Tọa độ nguồn (Pixel Space - src):**
  * $p_1 = (145, 250)$ (trên-trái)
  * $p_2 = (511, 257)$ (trên-phải)
  * $p_3 = (597, 306)$ (dưới-phải)
  * $p_4 = (58, 294)$ (dưới-trái)

* **Tọa độ đích tương ứng (World Space - dst - mm):**
  * $P_1 = (-196, 510)$
  * $P_2 = (196, 510)$
  * $P_3 = (196, 310)$
  * $P_4 = (-196, 310)$

---

### 2. Thiết lập mô hình toán học của OpenCV

Ma trận Homography $H$ có dạng:
$$
H = \begin{bmatrix}
h_{11} & h_{12} & h_{13} \\
h_{21} & h_{22} & h_{23} \\
h_{31} & h_{32} & h_{33}
\end{bmatrix}
$$
Vì phép chiếu là đồng dạng (tỷ lệ), ma trận này chỉ có **8 bậc tự do**. OpenCV thực hiện chuẩn hóa bằng cách đặt phần tử góc dưới cùng $h_{33} = 1$. Khi đó, ta cần tìm 8 ẩn số: $[h_{11}, h_{12}, h_{13}, h_{21}, h_{22}, h_{23}, h_{31}, h_{32}]^T$.

Với mỗi cặp điểm tương ứng $(u_i, v_i) \rightarrow (X_i, Y_i)$, ta có phương trình:
$$
X_i = \frac{h_{11}u_i + h_{12}v_i + h_{13}}{h_{31}u_i + h_{32}v_i + 1} \quad , \quad Y_i = \frac{h_{21}u_i + h_{22}v_i + h_{23}}{h_{31}u_i + h_{32}v_i + 1}
$$

Nhân chéo mẫu số và chuyển vế, ta rút ra 2 phương trình tuyến tính bậc nhất cho mỗi cặp điểm:
1. $u_i h_{11} + v_i h_{12} + h_{13} - u_i X_i h_{31} - v_i X_i h_{32} = X_i$
2. $u_i h_{21} + v_i h_{22} + h_{23} - u_i Y_i h_{31} - v_i Y_i h_{32} = Y_i$

---

### 3. Quy trình tính toán từng bước của OpenCV

#### Bước 3.1: Thay thế các điểm giả định vào hệ phương trình
Với 4 cặp điểm đã cho, OpenCV tiến hành thế số để tạo hệ 8 phương trình. 
* **Ví dụ với điểm số 1 ($p_1(145, 250) \rightarrow P_1(-196, 510)$):**
  * Phương trình 1 (trục X):
    $$145 h_{11} + 250 h_{12} + h_{13} - (145 \cdot -196) h_{31} - (250 \cdot -196) h_{32} = -196$$
    $$\Leftrightarrow 145 h_{11} + 250 h_{12} + h_{13} + 28420 h_{31} + 49000 h_{32} = -196$$
  * Phương trình 2 (trục Y):
    $$145 h_{21} + 250 h_{22} + h_{23} - (145 \cdot 510) h_{31} - (250 \cdot 510) h_{32} = 510$$
    $$\Leftrightarrow 145 h_{21} + 250 h_{22} + h_{23} - 73950 h_{31} - 127500 h_{32} = 510$$

Thực hiện tương tự thế số cho các điểm 2, 3, và 4 để có đủ 8 phương trình.

#### Bước 3.2: Dựng ma trận hệ phương trình $A \cdot h = b$
Hệ phương trình tuyến tính được biểu diễn dưới dạng ma trận:
$$
\begin{bmatrix}
u_1 & v_1 & 1 & 0 & 0 & 0 & -u_1 X_1 & -v_1 X_1 \\
0 & 0 & 0 & u_1 & v_1 & 1 & -u_1 Y_1 & -v_1 Y_1 \\
u_2 & v_2 & 1 & 0 & 0 & 0 & -u_2 X_2 & -v_2 X_2 \\
0 & 0 & 0 & u_2 & v_2 & 1 & -u_2 Y_2 & -v_2 Y_2 \\
u_3 & v_3 & 1 & 0 & 0 & 0 & -u_3 X_3 & -v_3 X_3 \\
0 & 0 & 0 & u_3 & v_3 & 1 & -u_3 Y_3 & -v_3 Y_3 \\
u_4 & v_4 & 1 & 0 & 0 & 0 & -u_4 X_4 & -v_4 X_4 \\
0 & 0 & 0 & u_4 & v_4 & 1 & -u_4 Y_4 & -v_4 Y_4
\end{bmatrix}
\begin{bmatrix}
h_{11} \\ h_{12} \\ h_{13} \\ h_{21} \\ h_{22} \\ h_{23} \\ h_{31} \\ h_{32}
\end{bmatrix}
=
\begin{bmatrix}
X_1 \\ Y_1 \\ X_2 \\ Y_2 \\ X_3 \\ Y_3 \\ X_4 \\ Y_4
\end{bmatrix}
$$

Thế các số cụ thể vào ma trận $A$ ($8 \times 8$) và vector tự do $b$ ($8 \times 1$):
$$
A = \begin{bmatrix}
145 & 250 & 1 & 0 & 0 & 0 & 28420 & 49000 \\
0 & 0 & 0 & 145 & 250 & 1 & -73950 & -127500 \\
511 & 257 & 1 & 0 & 0 & 0 & -100156 & -50372 \\
0 & 0 & 0 & 511 & 257 & 1 & -260610 & -131070 \\
597 & 306 & 1 & 0 & 0 & 0 & -117012 & -59976 \\
0 & 0 & 0 & 597 & 306 & 1 & -185070 & -94860 \\
58 & 294 & 1 & 0 & 0 & 0 & 11368 & 57624 \\
0 & 0 & 0 & 58 & 294 & 1 & -17980 & -91140
\end{bmatrix}
, \quad
b = \begin{bmatrix}
-196 \\ 510 \\ 196 \\ 510 \\ 196 \\ 310 \\ -196 \\ 310
\end{bmatrix}
$$

#### Bước 3.3: Giải hệ phương trình tuyến tính
Do $A$ là ma trận vuông cấp 8 và 4 điểm đầu vào không thẳng hàng (độc lập tuyến tính), ma trận $A$ khả nghịch. OpenCV sử dụng bộ giải phương trình tuyến tính (ví dụ như phân rã LU hoặc phân rã QR) để tìm vector ẩn số $h$:
$$
h = A^{-1} \cdot b
$$

#### Bước 3.4: Trích xuất ma trận H cuối cùng
Sau khi tìm được vector hệ số $h$, OpenCV sắp xếp các giá trị vào ma trận $3 \times 3$ và đặt $h_{33} = 1.0$:
$$
H = \begin{bmatrix}
h[0] & h[1] & h[2] \\
h[3] & h[4] & h[5] \\
h[6] & h[7] & 1.0
\end{bmatrix}
$$
Kết quả này chính là ma trận `homography_matrix` được lưu vào tệp JSON và node C++ sử dụng ở runtime để nhân tọa độ điểm ảnh.