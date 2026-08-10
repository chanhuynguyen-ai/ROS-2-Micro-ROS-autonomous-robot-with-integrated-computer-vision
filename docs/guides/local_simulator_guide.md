# Hướng dẫn Chạy Mô Phỏng AVS (Local Post-Inference Simulator)

Tài liệu này hướng dẫn chi tiết cách khởi chạy và vận hành hệ thống mô phỏng `local_post_inference_simulator`. Hệ thống này dùng để cô lập và kiểm tra logic nội suy hình học (IPM) cùng thuật toán lập quỹ đạo (control / planning), hoàn toàn độc lập với NCNN (AI Model).

---

## 1. Khởi chạy Hệ thống Mô phỏng qua Giao diện (UI)

Để sử dụng trình mô phỏng bằng giao diện Web, bạn cần mở **3 cửa sổ Terminal độc lập** để chạy lần lượt 3 tiến trình sau:

### Terminal 1: Chạy Node Nội suy Hình học (IPM Transform)
Chuyển đổi các tọa độ Pixel sang tọa độ thực tế (World/BEV).
```bash
cd ~/SimpleSysIDV
source ros2_ws/install_user/setup.bash
export ROS_DOMAIN_ID=20
ros2 run avs_perception ipm_transform_node --ros-args -p calibration_file_path:=$(pwd)/config/calibration.json
```
*(Lưu ý: Parameter `calibration_file_path` là bắt buộc khi chạy ngoài container để trỏ đúng đường dẫn file config).*

### Terminal 2: Chạy Node Ra quyết định (Control Node)
Thực hiện lựa chọn làn, xử lý state (ý định) và nội suy quỹ đạo.
```bash
cd ~/SimpleSysIDV
source ros2_ws/install_user/setup.bash
export ROS_DOMAIN_ID=20
ros2 run avs_perception control_node
```

### Terminal 3: Chạy FastAPI Simulator Backend
Backend sẽ đóng vai trò nhận request từ trình duyệt, giả lập Topic Inference và bắn vào mạng ROS.
```bash
cd ~/SimpleSysIDV
export ROS_DOMAIN_ID=20
.venv/bin/python3 -m uvicorn tools.local_post_inference_simulator.backend.main:app --host 0.0.0.0 --port 8001
```

Sau khi 3 terminal báo chạy thành công, hãy mở **trình duyệt Firefox** và truy cập:
👉 `http://localhost:8001/`

---

## 2. Cách thao tác trên Web UI

### A. Tải kịch bản (Load Scenario)
1. Trong cột bên trái (phần **Scenario**), nhấn nút **Import JSON**.
2. Trình duyệt sẽ mở hộp thoại chọn file. Điều hướng đến thư mục:
   `tools/local_post_inference_simulator/fixtures/`
3. Chọn một kịch bản có sẵn (ví dụ: `follow_main_straight.json`, `lane_change_solid_blocked.json`).
4. Nhấn Open. Các đối tượng (làn đường, vạch kẻ) sẽ được vẽ lên Canvas ở chính giữa.

### B. Bắt đầu Mô phỏng (Chạy Pipeline)
Nhìn sang cột bên phải (phần **Run (ROS backend)**), bạn có 3 thao tác chính:
1. **Load Scenario**: Phải nhấn nút này trước để gửi kịch bản từ Canvas xuống Backend ROS.
2. **Step IPM**: Chỉ chạy một khung hình đi qua phần nội suy hình học (IPM), **bỏ qua Control Node**. Dùng để test calibration và nội suy điểm trung tâm của làn.
3. **Step** hoặc **Play**: Chạy qua toàn bộ pipeline (bao gồm cả Control Node) để sinh quỹ đạo thực tế.

### C. Xem kết quả quỹ đạo (BEV / World View)
Cuộn xuống phần "BEV / World View", bạn có thể tích chọn để quan sát quá trình sinh quỹ đạo của hệ thống:
- 🟧 **Candidate Trajectory** (cam đứt nét): Quỹ đạo thô ban đầu dựa theo ý định lái.
- 🟪 **Normalized Trajectory** (tím đứt nét): Quỹ đạo sau khi làm mượt khoảng cách điểm.
- 🟩 **Committed Trajectory** (xanh lá liền nét): **Đây là quỹ đạo chốt cuối cùng mà xe sẽ đi theo**. Quỹ đạo này được trả về xuống motor/vi điều khiển.

---

## 3. Chạy Kiểm tra Tự động (Regression Test) qua Command Line

Nếu bạn không cần xem giao diện và chỉ muốn kiểm tra toàn bộ kịch bản một cách tự động (CI/Regression):

### Chạy Offline (Không cần bật ROS/Backend)
Thích hợp để CI hoặc kiểm tra nhanh code logic trong bộ test.
```bash
cd ~/SimpleSysIDV
pytest -v -m "not ros" test/local_post_inference_simulator/test_regression.py
```

### Chạy Live với ROS (Yêu cầu bật 3 Terminal ở mục 1)
Đảm bảo luồng ROS thông qua Topic chạy ổn định y hệt lúc lên xe thực tế.
```bash
cd ~/SimpleSysIDV
export ROS_DOMAIN_ID=20
pytest -v -m ros test/local_post_inference_simulator/test_regression.py
```

---

*Lưu ý: Bất kỳ khi nào sửa code C++ của các node ROS, bạn cần biên dịch lại bằng `colcon build` trước khi khởi động lại các tiến trình.*
