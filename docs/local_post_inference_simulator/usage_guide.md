# Hướng Dẫn Sử Dụng Local Post-Inference Simulator

Tài liệu này hướng dẫn cách dùng simulator hiện tại trong `tools/local_post_inference_simulator/` để debug pipeline hậu-inference của AVS theo runtime ROS thật.

Simulator này dùng cho các bài toán:

- kiểm tra hình học IPM/pixel-to-world
- kiểm tra lane centerline/waypoint
- kiểm tra decision/planning/control error
- replay nhiều frame để xem hysteresis, dropout, lane switch, trajectory debug

Simulator này không chạy model inference thật. Frontend chỉ tạo scenario JSON và gửi vào ROS pipeline thật; source-of-truth planning vẫn là `control_node.cpp`.

## 1. Thành phần chính

Luồng dữ liệu chính:

```text
scenario JSON / canvas
-> FastAPI simulator backend
-> synthetic /avs/telemetry
-> ipm_transform_node
-> /avs/telemetry_realworld
-> control_node
-> /avs/lane_state + /avs/control_error
-> frontend preview + BEV + JSON debug
```

Thư mục liên quan:

- `tools/local_post_inference_simulator/backend/`: backend bridge và scenario runner
- `tools/local_post_inference_simulator/frontend/`: UI canvas
- `tools/local_post_inference_simulator/fixtures/`: fixture mẫu
- `tools/local_post_inference_simulator/README.md`: tóm tắt nhanh
- `docs/local_post_inference_simulator/plan.md`: scope, rationale, phase history

## 2. Khi nào dùng mode nào

### `Step IPM`

Dùng khi muốn kiểm tra riêng phần hình học:

- object pixel có đi qua `ipm_transform_node` đúng không
- BEV/world polygon có hợp lý không
- waypoint/centerline từ IPM có đúng không
- calibration có đang lệch không

Mode này không chạy `control_node`. Vì vậy:

- không có `/avs/lane_state`
- không có `/avs/control_error`
- không có debug trajectory thật để overlay

### `Step`

Dùng khi muốn chạy full pipeline một frame:

- route intent
- IPM
- decision/planning
- lane state
- control error
- debug trajectories `candidate` / `normalized` / `committed`

### `Play`

Dùng khi muốn replay nhiều frame liên tiếp để xem:

- hysteresis
- lane switching
- jitter
- blocked behavior
- replan count
- hành vi manager qua nhiều frame

## 3. Chuẩn bị môi trường

### 3.1 Build package perception

Khuyến nghị dùng layout user đã có trong repo:

```bash
cd /home/goln/SimpleSysIDV/ros2_ws
colcon --log-base log_user build --symlink-install --packages-select avs_perception --build-base build_user --install-base install_user
```

### 3.2 Source workspace

Mỗi terminal chạy ROS cần source:

```bash
cd /home/goln/SimpleSysIDV/ros2_ws
source install_user/setup.bash
```

### 3.3 Chọn `ROS_DOMAIN_ID`

Tất cả terminal phải dùng cùng một `ROS_DOMAIN_ID`.

Ví dụ:

```bash
export ROS_DOMAIN_ID=57
```

## 4. Khởi chạy hệ thống

### 4.1 Chạy `ipm_transform_node`

Nếu chạy ngoài Docker, phải override calibration path vì default `/workspace/...` chỉ đúng trong container.

```bash
cd /home/goln/SimpleSysIDV
source ros2_ws/install_user/setup.bash
export ROS_DOMAIN_ID=57
ros2 run avs_perception ipm_transform_node --ros-args -p calibration_file_path:=$(pwd)/config/calibration.json
```

### 4.2 Chạy `control_node`

Chỉ cần cho `Step` và `Play`. `Step IPM` không cần node này.

```bash
cd /home/goln/SimpleSysIDV
source ros2_ws/install_user/setup.bash
export ROS_DOMAIN_ID=57
ros2 run avs_perception control_node
```

### 4.3 Chạy backend simulator

```bash
cd /home/goln/SimpleSysIDV
export ROS_DOMAIN_ID=57
.venv/bin/python3 -m uvicorn tools.local_post_inference_simulator.backend.main:app --host 0.0.0.0 --port 8001
```

### 4.4 Mở frontend

Mở:

```text
http://localhost:8001/
```

## 5. Quy trình sử dụng cơ bản

### Cách A: dùng fixture có sẵn

Đây là cách nhanh nhất để xác nhận simulator đang hoạt động.

1. Mở frontend.
2. Import một fixture từ `tools/local_post_inference_simulator/fixtures/`.
3. Bấm `Load`.
4. Chọn:
   - `Step IPM` nếu chỉ muốn xem IPM
   - `Step` nếu muốn xem full pipeline
   - `Play` nếu muốn replay nhiều frame

Fixture nên thử đầu tiên:

- `follow_main_straight.json`
- `follow_main_curve.json`
- `turn_right_two_lanes.json`
- `lane_change_solid_blocked.json`

### Cách B: tự vẽ scenario mới

1. Chọn class trong panel class.
2. Chọn kiểu shape:
   - polygon cho lane vùng
   - polyline cho marking
3. Click từng điểm trên canvas để vẽ.
4. Nhấn `Enter` hoặc double-click để kết thúc shape.
5. Chọn object để kéo điểm chỉnh lại nếu cần.
6. Thêm frame mới bằng duplicate frame khi muốn tạo replay nhiều frame.
7. Export ra JSON nếu muốn lưu fixture.

## 6. Các class nên dùng

Trong simulator hiện tại, nhóm class hữu ích nhất cho decision/planning là:

- `main-lane`
- `other-lane`
- `turn-lane`
- `dashed-white`
- `dashed-yellow`
- `double-solid-white`
- `solid-white`
- `solid-yellow`
- `stop-line`

Lưu ý quan trọng:

- `turn-lane` đúng là label `20` (model 22 class; `17` là `solid-yellow`)
- `stop-line` chỉ để quan sát, không dùng để kích hoạt rẽ hay quyết định lane trong phase hiện tại

## 7. Route intent và ý nghĩa

Simulator dùng `/avs/route_intent` làm nguồn intent lái.

Các intent thường dùng:

- `follow_main`
- `turn_left`
- `turn_right`
- `lane_change_left`
- `lane_change_right`

Có 2 mức cấu hình:

- route intent toàn scenario
- route intent riêng từng frame

Nếu frame có route intent riêng, nó sẽ override giá trị toàn scenario cho frame đó.

## 8. Ý nghĩa các nút điều khiển

### `Load`

- nạp scenario từ frontend vào backend
- reset runner state để sẵn sàng chạy

### `Step IPM`

- publish synthetic telemetry cho đúng 1 frame
- đợi `/avs/telemetry_realworld`
- không chạy logic control

Dùng để xác nhận:

- object có đi qua IPM không
- BEV có đúng không
- world geometry có hợp lý không

### `Step`

- publish route intent
- publish synthetic telemetry
- đợi `telemetry_realworld`, `lane_state`, `control_error`
- lưu output mới nhất cho panel JSON và BEV overlay

### `Play`

- chạy lần lượt toàn bộ frame theo playback loop
- dùng để xem behavior nhiều frame

Trong lúc `Play`:

- frame đang chọn để edit có viền xanh
- frame đang được ROS xử lý có viền đỏ

### `Pause`

- dừng playback tạm thời
- giữ scenario hiện tại

### `Stop`

- dừng playback
- reset frame index về 0

## 9. Cách đọc 3 panel quan trọng

### 9.1 Preview Image

Đây là ảnh 2D pixel-space của frame đang gửi vào pipeline.

Dùng panel này để kiểm tra:

- shape có đúng không
- object có bị vẽ lệch không
- class đã chọn có khớp hình mình muốn không

Nếu preview đã sai thì chưa cần xem tiếp BEV hay lane state.

### 9.2 BEV / World View

Đây là output world-space sau IPM.

Trong ảnh:

- `X+` là sang phải xe
- `Y+` là phía trước xe

Panel này giúp xem:

- polygon/lane sau IPM có hợp lý không
- waypoint/centerline có ổn không
- trajectory debug có bám đúng geometry không

### 9.3 Runner Status / Output JSON

Hai panel JSON dùng để xem:

- frame hiện tại
- mode đang chạy
- `lane_state`
- `control_error`
- report metrics sau khi `Play`

Nếu cần raw lane state mới nhất để inspect riêng, có thể gọi:

```text
GET /api/lane_state/latest
```

## 10. Debug trajectory overlay

Panel BEV có 3 checkbox:

- `candidate trajectory`
- `normalized trajectory`
- `committed trajectory`

Ý nghĩa:

- overlay đọc trực tiếp từ `debug_trajectories` trong `/avs/lane_state`
- backend/frontend không tự tính lại planning
- đây là dữ liệu production do `control_node` publish

Màu mặc định:

- candidate: cam đứt nét
- normalized: tím đứt nét
- committed: xanh lá liền nét

### Khi nào sẽ thấy trajectory

Chỉ thấy khi:

- dùng `Step` hoặc `Play`
- `control_node` đang chạy
- frame đó có `debug_trajectories` hợp lệ

### Khi nào sẽ không thấy trajectory

Simulator hiện có hint text trên ảnh để phân biệt nguyên nhân:

- đang dùng `Step IPM`, nên `control_node` chưa chạy
- `lane_state` có nhưng thiếu `debug_trajectories`
- stage được bật không có điểm hợp lệ trong frame đó

Nếu muốn debug planning mà chỉ dùng `Step IPM`, đó là sai workflow.

## 11. Quản lý nhiều frame

Simulator hỗ trợ multi-frame replay cơ bản.

Có thể dùng để test:

- object dropout
- lane đổi dần qua từng frame
- route intent đổi theo frame
- hysteresis của planner

Workflow khuyến nghị:

1. Tạo frame 1 đúng scene gốc.
2. Duplicate frame.
3. Chỉnh nhẹ geometry hoặc marking ở frame sau.
4. Lặp lại cho đủ chuỗi frame.
5. Chạy `Play`.
6. Mở report sau khi chạy xong.

## 12. ID-swap control

Để mô phỏng tracker đổi ID giữa các frame:

1. Chọn frame cần sửa.
2. Trong bảng object, double-click vào ô `Id`.
3. Nhập ID mới.

Cơ chế này chỉ đổi ID trong frame hiện tại, không đổi frame khác.

Dùng cho các bài test:

- tracking continuity
- smoothing theo `track_id`
- robustness khi object giữ geometry nhưng đổi identity

## 13. Hai chế độ load scenario: `direct` và `rasterized`

Scenario backend hỗ trợ:

- `mode=direct`
- `mode=rasterized`

### `direct`

- polygon từ UI đi thẳng vào payload telemetry
- phù hợp để debug nhanh geometry/scenario logic

### `rasterized`

- polygon được rasterize thành mask rồi contour lại trước khi publish
- gần với luồng sau-inference hơn khi muốn mô phỏng artifacts kiểu contour

Nếu chưa có lý do rõ ràng, nên bắt đầu với `direct`.

## 14. API hữu ích khi debug không qua browser

### Load scenario

```bash
curl -X POST "http://localhost:8001/api/scenarios/load?mode=direct" \
  -H "Content-Type: application/json" \
  --data @tools/local_post_inference_simulator/fixtures/follow_main_straight.json
```

### Chạy 1 frame IPM-only

```bash
curl -X POST http://localhost:8001/api/scenarios/step_ipm
```

### Chạy 1 frame full pipeline

```bash
curl -X POST http://localhost:8001/api/scenarios/step
```

### Xem BEV

```bash
curl "http://localhost:8001/api/ipm/bev?show_committed_trajectory=true" --output bev.jpg
```

### Xem lane state mới nhất

```bash
curl http://localhost:8001/api/lane_state/latest
```

### Xem status

```bash
curl http://localhost:8001/api/scenarios/status
```

## 15. Quy trình debug khuyến nghị

### Case 1: nghi lỗi IPM/homography

Đi theo thứ tự:

1. Import fixture hoặc tự vẽ scene.
2. `Load`
3. `Step IPM`
4. Xem `Preview`
5. Xem `BEV`
6. Xem `GET /api/ipm/latest`

Nếu lỗi đã xuất hiện ở đây thì chưa cần xem `control_node`.

### Case 2: nghi lỗi decision/planning

Đi theo thứ tự:

1. Xác nhận `Step IPM` cho geometry ổn trước.
2. Bật `control_node`.
3. Dùng `Step`.
4. Xem:
   - `lane_state`
   - `control_error`
   - BEV trajectory overlay
5. Nếu cần multi-frame behavior thì chạy `Play`.

### Case 3: nghi lỗi hysteresis / manager / lane switch

1. Dùng scenario nhiều frame.
2. Chạy `Play`.
3. Mở report cuối run.
4. So sánh:
   - `selected_lane_switch_count`
   - `trajectory_kind_switch_count`
   - `replan_count`
   - `invalid_frame_count`
   - `jitter_epsilon_x_mm`
   - `jitter_theta_rad`

## 16. Các lỗi thường gặp và cách xử lý

### BEV trống hoàn toàn

Kiểm tra:

- `ipm_transform_node` có đang chạy không
- `ROS_DOMAIN_ID` có khớp giữa các terminal không
- calibration path có override đúng chưa
- preview pixel có object thật không

### `Step IPM` chạy nhưng không có trajectory overlay

Đây là đúng behavior.

Lý do:

- `Step IPM` không chạy `control_node`
- không có `/avs/lane_state`
- không có `debug_trajectories`

Muốn xem trajectory overlay phải dùng `Step` hoặc `Play`.

### `Step` bị timeout hoặc không có lane state

Kiểm tra:

- `control_node` có đang chạy không
- route intent có được publish và ack không
- `ipm_transform_node` có đang publish `telemetry_realworld` không
- terminal ROS có log warning/error gì không

### Geometry world-space rất méo hoặc lộn hướng

Kiểm tra:

- calibration file có đúng file production hiện tại không
- object pixel có đi xuyên qua vùng vanishing row không
- lane geometry có phi vật lý không

### Playback chạy nhưng frame đỏ không như kỳ vọng

Hiện tại:

- viền xanh là frame đang chọn để edit
- viền đỏ là frame backend đang xử lý khi `Play`

Nếu đang bấm `Step` thủ công thì không dùng đỏ để biểu diễn sau khi step đã xong.

## 17. Test và xác nhận simulator

### Chạy test simulator + decision

```bash
cd /home/goln/SimpleSysIDV
pytest -q test/local_post_inference_simulator test/decision_system
```

### Chạy regression live với ROS

```bash
cd /home/goln/SimpleSysIDV
export ROS_DOMAIN_ID=57
pytest -v -m ros test/local_post_inference_simulator/test_regression.py
```

### Chạy CLI không qua browser

```bash
cd /home/goln/SimpleSysIDV
export ROS_DOMAIN_ID=57
python tools/local_post_inference_simulator/backend/run_scenario.py \
  tools/local_post_inference_simulator/fixtures/follow_main_straight.json
```

## 18. Giới hạn hiện tại

Các giới hạn đã biết:

- chưa có timeline scrub/kéo-thả
- frontend không có JS unit test runner riêng
- simulator không thay thế closed-loop vehicle simulator
- UI chỉ phục vụ debug kỹ thuật, không tối ưu cho demo sản phẩm

Nhưng ở trạng thái hiện tại, simulator đã đủ để:

- dựng scenario có kiểm soát
- replay multi-frame cơ bản
- debug geometry IPM
- debug trajectory pipeline
- so sánh hành vi planner trước/sau refactor

## 19. Tài liệu nên đọc kèm

- `tools/local_post_inference_simulator/README.md`
- `docs/local_post_inference_simulator/plan.md`
- `docs/local_post_inference_simulator/scenario_refactor_mapping.md`
- `docs/architecture/decision_sys.md`
- `docs/architecture/decision_trajectory_refactor_roadmap.md`
- `docs/vision/homography_implementation_plan.md`
