# Local Post-Inference Simulator

Mô phỏng và test pipeline sau khi nhận diện ảnh (Inference) của hệ thống AVS.

## Mục tiêu
Cách ly các thuật toán xử lý hình học (IPM/Homography), lập quỹ đạo (trajectory planning), trích xuất đường centerline và quyết định (decision system) khỏi mô hình học máy thực tế (NCNN). Người dùng có thể chạy thử các kịch bản vẽ tay để debug phần planning/decision.

## Cấu trúc thư mục
- `backend/`: FastAPI server chạy cổng `8001` làm cầu nối gửi dữ liệu giả lập vào ROS2.
- `frontend/`: UI canvas HTML/CSS/JS thuần, được `backend/main.py` mount tĩnh tại `/`. Frontend chỉ tạo shape/scenario JSON và gọi backend bridge, không tự tính IPM/planning.
- `fixtures/`: Các kịch bản vẽ bằng pixel space được định nghĩa dưới dạng JSON.
- `label_mapping.json`: Mapping giữa class label và class name dùng chung, cũng được serve qua `GET /api/label_mapping` để frontend không hard-code lại danh sách class.

## Cách vận hành

### 1. Khởi chạy các Node ROS2 trong Workspace
Để chạy Phase 3 IPM-only, chỉ cần `ipm_transform_node` đang chạy. Để chạy full pipeline qua control, chạy thêm `control_node`.

```bash
# Terminal 1: Chạy IPM Transform Node (chạy từ thư mục gốc repo)
# Ngoài Docker BẮT BUỘC override calibration_file_path — default /workspace/... chỉ đúng trong container,
# thiếu nó node sẽ log "Calibration file does not exist" và bỏ qua IPM.
source ros2_ws/install_user/setup.bash
export ROS_DOMAIN_ID=20
ros2 run avs_perception ipm_transform_node --ros-args \
    -p calibration_file_path:=$(pwd)/config/calibration.json \
    -p publish_debug_centerline:=true

# Terminal 2: Chạy Control Node (chỉ cần cho full pipeline /api/scenarios/step)
source ros2_ws/install_user/setup.bash
export ROS_DOMAIN_ID=20
ros2 run avs_perception control_node
```

### 2. Khởi chạy FastAPI Simulator Backend
```bash
export ROS_DOMAIN_ID=20
.venv/bin/python3 -m uvicorn tools.local_post_inference_simulator.backend.main:app --host 0.0.0.0 --port 8001
```

### 3. Mở UI canvas
Mở `http://localhost:8001/` bằng Firefox. UI cho phép vẽ polygon (lane) / polyline (marking), cấu hình route intent, quản lý nhiều frame, export/import scenario JSON, và có các nút Load/Step/Step IPM/Play/Pause/Stop gọi thẳng backend bridge ở trên.

## Các API endpoint chính
- `GET /api/label_mapping`: Trả về danh sách `{label, class_name, color}` đọc từ `label_mapping.json`, dùng chung cho frontend canvas (single source of truth, không hard-code lại).
- `POST /api/scenarios/load?mode=rasterized`: Nạp kịch bản scenario JSON (hỗ trợ chế độ `direct` hoặc `rasterized`).
- `POST /api/scenarios/step_ipm`: Chạy 1 frame qua synthetic `/avs/telemetry` -> production `ipm_transform_node` -> `/avs/telemetry_realworld`. Endpoint này không cần `control_node`.
- `POST /api/scenarios/step`: Chạy thử 1 frame tiếp theo trong kịch bản.
- `POST /api/scenarios/play`: Chạy tự động (playback loop) toàn bộ kịch bản.
- `POST /api/scenarios/pause`: Tạm dừng playback.
- `POST /api/scenarios/stop`: Dừng playback và quay lại frame 0.
- `GET /api/scenarios/status`: Xem trạng thái runner hiện tại.
- `GET /api/scenarios/preview`: Trả về ảnh preview JPEG của frame hiện tại được vẽ trên Canvas 2D nền đen để kiểm tra trực quan.
- `GET /api/ipm/latest`: Xem raw `/avs/telemetry_realworld`, summary `polygons_real_world`, waypoint/centerline và bounds.
- `GET /api/ipm/bev`: Trả về ảnh BEV/world view từ output IPM mới nhất, với trục `X+` sang phải xe và `Y+` phía trước xe. Query param `show_candidate_trajectory`/`show_normalized_trajectory`/`show_committed_trajectory` (bool) vẽ thêm debug trajectory (candidate/normalized/committed) đọc trực tiếp từ `debug_trajectories` trong `/avs/lane_state` do `control_node` thật publish — không tính lại planning trong simulator.
- `GET /api/lane_state/latest`: Trả về raw payload `/avs/lane_state` mới nhất (kèm `debug_trajectories`) từ lần full `step()` gần nhất.
- `GET /api/ipm/calibration`: Kiểm tra calibration đang dùng cho scenario hiện tại, mặc định `config/calibration.json`.

**Lưu ý khi chạy `ipm_transform_node` ngoài Docker**: default param `calibration_file_path` là `/workspace/config/calibration.json`, chỉ đúng trong container. Chạy ngoài container phải override, ví dụ:

```bash
ros2 run avs_perception ipm_transform_node --ros-args -p calibration_file_path:=$(pwd)/config/calibration.json
```

## Phase 3: IPM-only quick check

Luồng này dùng để kiểm tra hình học trước khi đưa dữ liệu vào planner:

```text
fixture JSON
-> /api/scenarios/load
-> /api/scenarios/step_ipm
-> /avs/telemetry
-> ipm_transform_node thật
-> /avs/telemetry_realworld
-> /api/ipm/latest + /api/ipm/bev
```

`step_ipm` chỉ đợi timestamp matching từ `/avs/telemetry_realworld`, nên lỗi route intent hoặc `control_node` không làm nhiễu kết quả IPM. Summary trả về gồm:

- `polygons_real_world`
- `waypoints` nếu IPM node tạo centerline cho lane object
- `world_bounds`/`pixel_bounds`
- các field control/debug do IPM node publish như `lookahead_x_mm`, `heading_angle_rad`, `polynomial`

## Phase 1: UI Canvas Vẽ Class Mask

Frontend tối thiểu để vẽ scene, theo đúng scope trong `docs/local_post_inference_simulator/plan.md` (Phase 1):

- Canvas nền đen, vẽ polygon (lane) bằng click từng đỉnh + double-click/Enter để đóng, vẽ polyline (marking) tương tự với tối thiểu 2 điểm.
- Chế độ "Select / Edit" để chọn object đã vẽ, kéo từng điểm để chỉnh shape, hoặc xoá object (phím Delete/Backspace hoặc nút trong bảng object).
- Panel class lấy trực tiếp từ `GET /api/label_mapping`, lọc còn 9 class trong scope (`main-lane`, `other-lane`, `turn-lane`, `dashed-white`, `dashed-yellow`, `double-solid-white`, `solid-white`, `solid-yellow`, `stop-line`); mỗi object có `id` ổn định (auto-gen `class_name_N` nếu bỏ trống, giữ nguyên khi duplicate frame) để phục vụ test hysteresis/ID swap ở Phase 6.
- Panel route intent theo đúng 5 giá trị trong plan: `follow_main`, `turn_left`, `turn_right`, `lane_change_left`, `lane_change_right`.
- Quản lý nhiều frame ở mức cơ bản (duplicate/xoá frame, chọn frame đang vẽ) để scenario JSON tương thích ngay với `frames[]` mà `step`/`play` cần; timeline đầy đủ (dropout, ID swap có kiểm soát, route intent theo từng frame) vẫn để dành cho Phase 6.
- Export/import scenario JSON đúng `ScenarioSchema`/`ObjectSchema` ở `backend/scenario_schema.py` — checkbox "Enabled" trên object chỉ là state phía client để loại object đó khỏi payload xuất ra, không phải field trong schema.
- Nút Load/Step/Step IPM/Play/Pause/Stop gọi thẳng các endpoint có sẵn ở trên; frontend không tự tính IPM hay centerline, chỉ hiển thị lại `/api/scenarios/preview` và `/api/ipm/bev` do backend render.

Tiêu chí hoàn thành theo plan: vẽ được một scene lane cơ bản, và export → import giữ nguyên object id/class/points. Đã verify phần backend (`/api/label_mapping`, `/api/scenarios/load` với fixture, `/api/scenarios/preview`) qua `curl`; phần vẽ tay trên canvas/export-import trong trình duyệt cần người dùng tự kiểm tra bằng Firefox trước khi coi Phase 1 là "done" trong lịch sử commit.

## Phase 5/6: Debug Trajectory Overlay Và ID-swap Control

- Panel "BEV / World View" có 3 checkbox `candidate trajectory` (cam đứt nét) / `normalized trajectory` (tím đứt nét) / `committed trajectory` (xanh lá liền nét, mặc định bật) để vẽ debug trajectory từ `control_node` lên ảnh BEV. Dữ liệu lấy nguyên từ `debug_trajectories` trong `/avs/lane_state`, không tính lại ở frontend/backend.
- Double-click vào ô Id trong bảng "Objects trong frame hiện tại" để đổi id của object đó — chỉ áp dụng cho frame đang chọn, giữ nguyên hình học, dùng để dựng scenario test ID-swap giữa các frame (Phase 6) mà không cần sửa tay JSON export.
- Khi Play, frame đang được ROS xử lý (`runner.current_frame_idx`) được tô viền đỏ trong danh sách Frames, tách biệt với frame đang chọn để sửa (viền xanh).

## Phase 7: CLI Regression Runner

Chạy một fixture không cần browser:

```bash
export ROS_DOMAIN_ID=20
python tools/local_post_inference_simulator/backend/run_scenario.py \
    tools/local_post_inference_simulator/fixtures/follow_main_straight.json
```

Chạy toàn bộ regression suite (offline, không cần ROS):

```bash
pytest -v -m "not ros" test/local_post_inference_simulator/test_regression.py
```

Chạy live với ROS (sau khi đã start `ipm_transform_node` + `control_node` + backend):

```bash
export ROS_DOMAIN_ID=20
pytest -v -m ros test/local_post_inference_simulator/test_regression.py
```

## Kịch Bản Từ Video Thật (`fixtures/real_*.json`)

Các fixture `real_*` được sinh từ inference NCNN **thật** trên `test/test_video/video_test1.mp4` (không vẽ tay): mask polygon giữ **nguyên vẹn từng contour** từ `/avs/telemetry` (không đơn giản hóa, không scale — canvas 640x480 trùng độ phân giải video), `id` là `track_id` thật của tracker, `confidence` là `prob` thật.

Quy trình tái tạo (khi có video mới):

```bash
# 1. Chạy inference thật trên video, phát chậm 5x để CPU xử lý được ~mọi frame
ros2 run avs_perception video_publisher_node --ros-args \
    -p video_path:=$(pwd)/test/test_video/video_test1.mp4 -p loop:=false -p fps_override:=4.0
ros2 run avs_perception ncnn_inference_node --ros-args \
    -p model_param_path:=$(pwd)/models/best_ncnn_model/model.ncnn.param \
    -p model_bin_path:=$(pwd)/models/best_ncnn_model/model.ncnn.bin

# 2. Ghi lại telemetry
python3 tools/local_post_inference_simulator/backend/capture_real_telemetry.py \
    --output capture.jsonl --duration 420

# 3. Cắt đoạn video thành scenario (thời gian tính theo giây VIDEO)
python3 tools/local_post_inference_simulator/backend/telemetry_to_scenario.py \
    --input capture.jsonl --start-s 9.0 --end-s 15.5 \
    --name real_intersection_gap_crossing --target-fps 0 \
    --output tools/local_post_inference_simulator/fixtures/real_intersection_gap_crossing.json
```

Object nhiều contour dùng field `polygons_px` (toàn bộ contour, verbatim); `points_px` giữ contour đầu cho canvas editor. `mask_to_objects.py` ưu tiên `polygons_px` khi build payload nên pipeline nhận đúng từng contour như production; các contour phụ được vẽ read-only trên canvas.

Danh sách fixture thật hiện có (cửa sổ giây-video trong `source.video_window_s` của từng file):

| Fixture | Nội dung |
|---|---|
| `real_follow_main_straight.json` | Đi thẳng ổn định, 1 main-lane chạm đáy ảnh |
| `real_intersection_approach.json` | Tiếp cận giao lộ 1: main-lane gần co dần, dải main-lane xa, turn-lane + stop-line chớp |
| `real_intersection_gap_crossing.json` | **Băng qua giữa giao lộ 1**: mất lane chứa xe, chỉ còn main-lane bên kia (kịch bản "mất path") |
| `real_intersection2_turn_lanes.json` | Tiếp cận giao lộ 2: 2 main-lane, tới 3-4 turn-lane, stop-line |
| `real_intersection2_gap_crossing.json` | Băng qua giữa giao lộ 2 + turn-lane flicker trong gap |
| `real_main_dropout_blip.json` | Mất toàn bộ main-lane 1 frame giữa đường thẳng (test dropout hold) |

## Phase 8: Integration with Refactor Roadmap

Mỗi fixture trong `fixtures/` được gắn vào ít nhất một phase trong `docs/architecture/decision_trajectory_refactor_roadmap.md` để làm regression guard. Chi tiết mapping xem tại:

- [`docs/local_post_inference_simulator/scenario_refactor_mapping.md`](../../../docs/local_post_inference_simulator/scenario_refactor_mapping.md)

**Mapping nhanh**:

| Fixture | Refactor Phase được bảo vệ |
|---|---|
| `follow_main_straight.json` | Phase 9 (Replay/Regression) |
| `follow_main_curve.json` | Phase 9 + Phase 5 (Normalizer) |
| `intersection_follow_main.json` | Phase 4 (Lane Selection Geometry) |
| `lane_change_solid_blocked.json` | Phase 8 (Marking Gate & Blocked Behavior) |
| `turn_right_two_lanes.json` | Phase 1 (Runtime Flow) + Phase 2 (Intent/State) |

**Khi thêm phase refactor mới**: tạo hoặc cập nhật fixture tương ứng, thêm `assertions` block, cập nhật mapping doc. Đảm bảo:

```bash
pytest -v test/local_post_inference_simulator/ | grep "SKIPPED"
# Không được có SKIPPED do thiếu assertions
```

