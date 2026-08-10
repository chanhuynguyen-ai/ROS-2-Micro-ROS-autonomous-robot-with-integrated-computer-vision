# Chương 7. Giới hạn, rủi ro và kiểm thử

## 7.1. Giới hạn hiện tại

### Phụ thuộc calibration

IPM phụ thuộc hoàn toàn vào `homography_matrix` trong `config/calibration.json` (chương 5 mục 5.2). Nếu camera lệch góc, mặt đường không đủ phẳng, hoặc calibration sai/không khớp góc lắp hiện tại, mọi toạ độ world-frame phía sau đều sai — và hệ thống **không có cơ chế tự phát hiện** calibration sai theo kiểu "sanity check số liệu", chỉ phát hiện được file thiếu/sai định dạng (khi đó không publish gì cả, xem checklist ở 7.4).

### Phụ thuộc segmentation

Nếu mask lane bị đứt, nhiều lane bị nhập chung một vùng, hoặc nhầm label, các bước centerline/planner có safeguard (lọc lát bloated, EMA, xoá lane thiếu waypoint...) nhưng không thể khôi phục hoàn toàn hình học đúng nếu dữ liệu gốc đã sai đáng kể.

### `stop-line` chưa tham gia decision hiện tại

`stop-line` được phát hiện và giữ trong telemetry/marking, nhưng hiện **không** dùng để: kích hoạt rẽ, phát hiện giao lộ/T-junction, hay quyết định đổi làn. Đây là chủ đích của thiết kế hiện tại (phát hiện T-junction hoàn toàn dựa vào hình học turn-lane, chương 6 mục 6.6), không phải thiếu sót cần bổ sung ngay.

### `curvature_inv_mm` là tên field mang tính lịch sử (legacy)

Trong code, "curvature" được tính bằng một trong hai cách tuỳ ngữ cảnh: `2·a2` (từ hệ số polynomial, chương 2 mục 2.5.3) hoặc công thức Menger `2·cross/(a·b·c)` từ 3 điểm lân cận (chương 6 mục 6.13.1). Tên field `curvature_inv_mm` giữ nguyên để tương thích ngược với controller downstream, nhưng cần hiểu đây là một đại lượng **kiểu curvature xấp xỉ cục bộ**, không phải một bán kính cong nghịch đảo đã chuẩn hoá chặt chẽ theo định nghĩa toán học đầy đủ của độ cong.

### Helper legacy vẫn chạy song song, không phải dead code

`LegacyLaneModel` vẫn chạy song song với pipeline mới (`PathObservationBuilder`/`TrajectoryPlanner`/`TrajectoryNormalizer`/`TrajectoryManager`). Đây **không phải** code chết — `control_node` vẫn dùng nó cho: split/select lane kiểu legacy (chương 6 mục 6.6), bộ đếm T-junction, kiểm tra `is_turn_blocked_by_solid`, `evaluate_trajectory_at_lookahead` (chương 6 mục 6.13.1). Khi refactor, không được xoá các hàm này chỉ vì tên có chữ "legacy".

### Bug đã biết, chưa fix trong `TrajectoryLatch` (phát hiện qua code review 2026-08-03, xem chương 6 mục 6.16)

Đây là các vấn đề thực chất trong code, không phải chỉ lệch tài liệu — mô tả hành vi ở mục 6.16 là hành vi **theo thiết kế/code hiện tại**, chưa phải hành vi đã verify đúng trên xe thật, do các bug sau còn mở:

- **Hệ số đổi tốc độ sai**: `current_speed_mms_ = |twist.linear.x| * 2500.0` tại `control_node.cpp` — `/odom_raw.twist.linear.x` là m/s nên hệ số đúng phải là `1000.0`, không phải `2500.0`. Hệ quả: `turn_latch_progress_mm_` tích luỹ nhanh gấp 2.5 lần thực tế, latch bị release (`latch_path_consumed`) sớm hơn nhiều so với quãng đường xe thực sự đã đi.
- **`/odom_raw` hiện không có node nào publish trong deployment production** (`docker-compose.prod.yml` chỉ chạy 3 node perception, không có node publish odometry). Nếu đúng như vậy trên deployment đang dùng, `current_speed_mms_` luôn bằng `0`, `turn_latch_progress_mm_` không bao giờ tăng, và path latch được replay y nguyên mỗi frame cho tới khi hết `latch_deadline_s()` (10-30s) — trong lúc đó `RECOVERY` và direct-IPM fallback đều bị khoá vì latch đang active.
- Cơ chế `latch_blocked_by_marking` (giải phóng latch khi gặp marking cấm) chỉ thực sự trip được cho `TURN_LEFT` tại T-junction — với `TURN_RIGHT` hoặc `TURN_LEFT` ngoài T-junction, đường thoát an toàn này gần như không kích hoạt được.
- Exempt turn-lane khỏi `LaneLegalityGate` (mục 6.5/7.2) hiện đang bỏ qua **cả verdict HARD** (solid-yellow), không chỉ soft-illegal như comment trong code mô tả.
- Vị trí gốc `progress_mm=0` của latch được giả định là điểm đầu trajectory đã chốt, nhưng điểm đó có thể lệch tới vài trăm mm so với vị trí xe thực tế — có thể gây `epsilon_x` nhảy bậc đột ngột ngay frame bắt đầu latch.

Trước khi tin tưởng hành vi rẽ tại giao lộ khi turn-lane mất khỏi tầm nhìn, cần fix các bug trên và verify lại bằng replay multi-frame thật (không phải test một frame đơn lẻ).

### Vài chi tiết cài đặt dễ gây hiểu nhầm khi đọc code (phát hiện khi đối chiếu code thật cho báo cáo này)

- Comment mô tả kích thước tensor `out0` trong `yolo26_seg.cpp` ghi `"(44 x 2100)"`, nhưng logic decode thực tế **suy `num_classes` động** từ shape thật của `out0` (không hard-code), kèm safety check đối chiếu với `class_names.size()` (`return -1` nếu lệch). Với model 22-class hiện tại, `out0` thực chất là `58 hàng × 2100 cột` (`4+22+32=58`) — số `44` trong comment lỗi thời với cả model cũ (19-class, `55`) lẫn model hiện tại, không lấy `44` làm chuẩn khi đọc/sửa code (chương 4 mục 4.4).
- Tham số ROS2 `output_topic` (`/camera/segmented_image/compressed`) được khai báo trong `ncnn_inference_node` nhưng **không có publisher tương ứng dùng nó** trong node này — có thể là tham số dự phòng cho một node hiển thị khác, hoặc còn sót lại từ refactor trước (chương 4 mục 4.2).
- Hàm `publish_control_error_from_trajectory` thực chất chỉ có **2 nhánh xử lý** (precomputed-control / evaluate-tại-lookahead), không phải 3 nhánh tách biệt như cách diễn đạt "3 trường hợp" dễ gây hiểu ở các bản mô tả cũ — trường hợp "trajectory invalid" chỉ là giá trị mặc định khi không rơi vào 2 nhánh trên, không phải một nhánh code riêng (chương 6 mục 6.13).
- Latency ghi trong `/avs/telemetry` (`json_finalize_latency_ms`, `publish_latency_ms`, `node_total_latency_ms`) là latency đo được ở **frame ngay trước đó**, không phải frame đang publish — chủ đích thiết kế, không phải bug (chương 4 mục 4.8).
- `DecisionState` (`state_`) là giá trị **suy ra** từ trajectory vừa chốt trong frame, luôn trễ một nhịp so với `current_intent_` — không dùng `state_` để gate các bộ đếm dropout/abort trong code mới (chương 6 mục 6.2).

## 7.2. Các bất biến không được phá vỡ

- `turn-lane` phải là label `20`, không phải `17` (`17 = solid-yellow`) hay `10` (`10 = sign-no-parking`).
- `turn-lane` không bao giờ bị `LaneLegalityGate` loại khỏi output đã lọc, kể cả khi verdict là illegal — chỉ verdict được ghi nhận, không được dùng để ẩn lane (chương 6 mục 6.5).
- Mỗi frame chỉ publish một active trajectory duy nhất qua `/avs/control_error` (chương 1 mục 1.6).
- `/avs/route_intent` là nguồn chính cho ý định lái; `/avs/cmd` chỉ dùng cho lệnh hệ thống/legacy.
- `stop-line` không được dùng làm trigger rẽ/giao lộ/đổi làn trong giai đoạn hiện tại.
- Không sửa controller downstream khi chỉ đang thay đổi phần perception/decision tới `/avs/control_error` — schema (tên field, đơn vị, dấu) của topic này đã đóng băng.
- Khi đổi label, phải cập nhật đồng bộ: `config/label_mapping.json`, `models/best_ncnn_model/metadata.yaml`, `tools/local_post_inference_simulator/label_mapping.json` (nếu liên quan), và test.
- Mọi thay đổi vào `control_node.cpp` hoặc `ipm_transform_node.cpp` phải liệt kê cụ thể hàm/hành vi bị đổi và được duyệt trước khi code (quy tắc dự án, xem `CLAUDE.md`).

## 7.3. Kiểm thử nên chạy khi sửa hệ thống

Khi sửa logic decision/planning:

```bash
pytest -q test/decision_system
```

Khi sửa C++ node trong `ros2_ws/src/avs_perception/src/` hoặc header planning:

```bash
cd ros2_ws
colcon --log-base log_user build --symlink-install --packages-select avs_perception --build-base build_user --install-base install_user
```

Nếu sửa logic thuần thuật toán trajectory (không cần ROS runtime), ưu tiên viết/chạy gtest trong `ros2_ws/src/avs_perception/test/decision_trajectory_test.cpp` — theo chính sách D3 (2026-07-05): `test/decision_system/decision_harness.py` là mirror Python đã đóng băng, không thêm logic planner mới vào đó nữa; logic mới thuộc về header `trajectory_*.hpp` và test bằng gtest offline.

Lưu ý: build user layout sẽ thay đổi artifact trong `ros2_ws/build_user`, `ros2_ws/install_user`, `ros2_ws/log_user` — đây là build artifact, không phải source change, không cần revert.

## 7.4. Checklist đọc lỗi runtime

Khi `/avs/control_error` bất thường, kiểm tra theo đúng thứ tự sau (đi ngược từ input tới output để khoanh vùng tầng nào đang sai):

1. **`/avs/telemetry`**: model có detect đúng label và polygon hợp lý không (chương 4).
2. **`/avs/telemetry_realworld`**: `polygons_real_world`, `waypoints`, `lookahead_x_mm`, `lookahead_d_mm` có hợp lý không — nếu topic này trống hoàn toàn, nghi ngờ đầu tiên là calibration (chương 5 mục 5.2).
3. **`/avs/lane_state`**: `decision_state`, `route_intent`, `trajectory_kind`, `normalization_mode`, `hold_reason`, `replan_reason`, `control_source` — bộ field này cho biết chính xác đang ở bước nào trong luồng xử lý (chương 6 mục 6.3, 6.9, 6.11).
4. **`yellow_gate`**: lane có đang bị lọc bởi vạch vàng solid/dashed không (chương 6 mục 6.5).
5. **`active_trajectory_points`**: trajectory có liên tục, đúng hướng, không nhảy làn đột ngột không.
6. **`/odom_raw`**: tốc độ có đang làm lookahead bị kẹp ở `d_min`/`d_max` không (chương 5 mục 5.3).

## 7.5. Hướng phát triển hợp lý

- Tách dần helper legacy khỏi `control_node` sau khi test coverage đủ rộng cho từng hàm tương ứng trong pipeline mới.
- Chuẩn hoá schema JSON telemetry bằng message/interface ROS2 typed thay vì `std_msgs/String` + JSON thủ công, nếu pipeline đã đủ ổn định để chịu chi phí migrate.
- Thêm replay test nhiều frame cho các case giao lộ, dropout, và lane-change — vì lỗi ở trajectory manager/hysteresis chỉ lộ ra khi replay nhiều frame liên tiếp, test một frame đơn lẻ không đủ để kết luận đúng/sai.
- Thêm visualization cho 3 stage `candidate`/`normalized`/`committed` trajectory để debug trực quan hành vi của normalizer/manager thay vì chỉ đọc số trong `/avs/lane_state`.
- Nếu tương lai cần dùng `stop-line`/luật giao thông khác, nên thêm như một layer rule riêng biệt, không trộn vào logic phát hiện hình học lane hiện tại.
