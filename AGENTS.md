# AGENTS.md

Hướng dẫn này dành cho coding agents làm việc trong repo `SimpleSysIDV`. Mục tiêu là giúp agent đọc đúng hệ thống hiện tại, không dựa vào giả định cũ, và tránh phá vỡ pipeline ROS2 đang dùng cho AVS.

## Tổng Quan Hệ Thống

AVS là hệ thống vision/control chạy ROS2 Humble, tối ưu cho Raspberry Pi 5 CPU-only. Pipeline chính:

```text
camera/video
-> ncnn_inference_node
-> /avs/telemetry
-> ipm_transform_node
-> /avs/telemetry_realworld
-> control_node
-> /avs/control_error + /avs/lane_state
-> controller/ESP32 downstream
```

Các node chính nằm trong `ros2_ws/src/avs_perception/src/`:

- `ncnn_inference_node.cpp`: chạy NCNN segmentation, publish object/mask/polygon telemetry.
- `ipm_transform_node.cpp`: chuyển pixel/polygon sang world frame, trích waypoint/centerline.
- `control_node.cpp`: route intent, lane selection, trajectory planning, normalization, manager, control error.
- `video_publisher_node.cpp`: phát video/camera cho test.
- `video_test_node.cpp`: profiling offline.
- `yolo26_seg.cpp`: inference engine wrapper.

Dashboard hiện tại ở `web_dashboard/` chỉ là UI runtime/bridge. Không nhét simulator hoặc tooling debug lớn vào dashboard chính nếu không được yêu cầu.

## Tài Liệu Source-Of-Truth

Đọc theo thứ tự khi làm decision/planning:

1. `docs/architecture/decision_sys.md`
2. `docs/architecture/trajectory_planning_memory_proposal.md`
3. `docs/architecture/decision_trajectory_refactor_roadmap.md`
4. `docs/local_post_inference_simulator/plan.md` nếu làm simulator hậu-inference

Đọc theo thứ tự khi làm vision/IPM:

1. `docs/vision/homography_theory.md`
2. `docs/vision/homography_implementation_plan.md`
3. `docs/vision/pixel_to_world_plan.md`

Đọc khi cần vận hành/deploy:

- `docs/guides/deployment_guide.md`
- `docs/guides/ros2_yolo26_ncnn_setup_guide.md`
- `docs/guides/pure_pursuit_guide.md`

Lưu ý: nhiều tài liệu từng nằm trực tiếp dưới `docs/` đã được chuyển vào `docs/architecture`, `docs/vision`, `docs/guides`, `docs/reports`. Luôn dùng đường dẫn hiện tại.

`system_report_current/` (00_tong_quan → 06_gioi_han_va_kiem_thu) là báo cáo đối chiếu trực tiếp với code hiện tại, phạm vi dừng ở `/avs/control_error` (không phân tích controller downstream). Cập nhật gần nhất 2026-08-03. Dùng khi cần tra cứu công thức/hằng số/JSON schema cụ thể mà không cần đọc lại toàn bộ source.

## Label Mapping Hiện Tại

Source-of-truth là `config/label_mapping.json` (sinh ra `label_mapping.hpp` với các hằng `LABEL_*` lúc build) cùng `models/best_ncnn_model/metadata.yaml`. Bản sao thủ công cần sync khi đổi label: `class_names` trong `yolo26_seg.hpp`, `CLASS_NAMES`/`CLASS_COLORS` trong `web_dashboard/backend/main.py` + `web_dashboard/frontend/app.js`, `DEFAULT_CLASS_NAMES` trong `tools/local_post_inference_simulator/backend/mask_to_objects.py`, `CLASS_NAMES` trong `ros2_ws/src/avs_controlsystem/`, hằng `LABEL_*` trong `test/decision_system/decision_harness.py`, và label số trong fixtures.

```text
0  dashed-white
1  dashed-yellow
2  double-solid-white
3  light_green
4  light_red
5  light_yellow
6  main-lane
7  other-lane
8  parking-zone
9  sign-no-left
10 sign-no-parking
11 sign-no-right
12 sign-parking
13 sign-stop
14 sign-turn-left
15 sign-turn-right
16 solid-white
17 solid-yellow
18 start
19 stop-line
20 turn-lane
21 vehicle
```

Decision/planning labels quan trọng:

- `main-lane = 6`
- `other-lane = 7`
- `turn-lane = 20`
- `dashed-white = 0`
- `dashed-yellow = 1`
- `double-solid-white = 2`
- `solid-white = 16`
- `solid-yellow = 17`
- `stop-line = 19`

Invariant bắt buộc: `turn-lane` phải là `20` ở inference, IPM, control, test và simulator (model 22 class, đổi từ `17` của model 19 class cũ). Repo từng có regression dùng `turn-lane = 10` trong `ipm_transform_node.cpp`; nếu thấy `label == 10` hoặc `label == 17` được dùng như turn-lane thì đó là regression — `10` hiện là `sign-no-parking`, `17` là `solid-yellow`.

## Nguyên Tắc Kiến Trúc

- Không đổi controller downstream nếu không có yêu cầu rõ. Giữ contract `/avs/control_error`.
- Decision/planning phải đi theo hướng `path observation -> candidate trajectory -> normalized trajectory -> committed active trajectory`.
- Mỗi frame chỉ publish một active trajectory cho controller.
- `stop-line` không được dùng để kích hoạt rẽ, phát hiện giao lộ, phát hiện T-junction hoặc quyết định chuyển làn trong phase hiện tại.
- `/avs/route_intent` là nguồn intent lái; `/avs/cmd` chỉ nên dùng cho lệnh hệ thống/legacy compatibility.
- Simulator hậu-inference phải ưu tiên ROS-first: UI chỉ vẽ/cấu hình, pipeline chính chạy qua ROS node/topic thật hoặc synthetic ROS publisher.
- Không copy planner logic sang frontend/backend simulator. Source-of-truth planning là C++ `control_node` chạy trong ROS; Python harness chỉ để test/đối chiếu.

## Build Và Test

Build package chính:

```bash
cd ros2_ws
colcon build --symlink-install --packages-select avs_perception
```

Nếu muốn tránh đụng build/install/log mặc định, dùng layout user hiện có:

```bash
cd ros2_ws
colcon --log-base log_user build --symlink-install --packages-select avs_perception --build-base build_user --install-base install_user
```

Build user layout gần nhất đã chạy xong thành công với `build_user`, `install_user`, `log_user` và return code `0`. Các lần build kiểu này sẽ làm dirty các file trong `ros2_ws/build_user/`, `ros2_ws/install_user/`, `ros2_ws/log_user/`; coi đó là build artifact, không phải source change.

Test decision system:

```bash
pytest -q test/decision_system
```

Khi sửa C++ node trong `ros2_ws/src/avs_perception/src/`, tối thiểu nên chạy:

```bash
pytest -q test/decision_system
cd ros2_ws
colcon build --symlink-install --packages-select avs_perception
```

Không chạy hoặc sửa trong `ncnn-src/` trừ khi nhiệm vụ liên quan trực tiếp đến thư viện NCNN upstream.

Nếu cần chạy node từ user install layout sau build:

```bash
cd ros2_ws
source install_user/setup.bash
```

## Quy Tắc Làm Việc Với Code

- Dùng `rg`/`rg --files` để tìm file và symbol.
- Không sửa build artifacts trong `build/`, `install/`, `log/`, `ros2_ws/build*`, `ros2_ws/install*`, `ros2_ws/log*`. Nếu chúng xuất hiện trong `git status` sau build, ghi nhận nhưng không revert trừ khi user yêu cầu.
- Không revert thay đổi chưa rõ nguồn gốc. Repo có thể đang dirty.
- Khi refactor `control_node.cpp`, thêm test/fixture trước nếu behavior có thể đổi.
- Khi gom label constants, cập nhật đồng bộ inference, IPM, control, test, fixtures và các bản sao class list liệt kê ở mục Label Mapping.
- Khi thêm tooling local, đặt ngoài dashboard chính, ví dụ `tools/local_post_inference_simulator/`, và tài liệu trong `docs/local_post_inference_simulator/`.

## Kỹ Năng/Tham Chiếu Nội Bộ

Các skill trong repo có thể hữu ích:

- `skills/camera_SKILL/SKILL.md`: camera, udev, device mapping.
- `skills/docker_SKILL/SKILL.md`: Docker/deployment.
- `skills/data_transport_SKILL/SKILL.md`: ROS2 phân tán, Pi-to-laptop.
- `skills/labeling_SKILL/SKILL.md`: quy tắc annotation lane/marking.
- `skills/decision_trajectory_SKILL/SKILL.md`: gotchas cho `control_node.cpp`/`trajectory_*.hpp` — `TrajectoryLatch`, gate diagnostic lane-change, turn-lane exempt legality gate, bug đã biết chưa fix. Đọc trước khi sửa vùng decision/trajectory.
- `skills/karpathy-guidelines/SKILL.md`: coding style, simplicity-first.

## Các Bẫy Dễ Sai

- Nhầm `turn-lane` label `20` với `17`/`10`. `20` là đúng theo model 22 class hiện tại; `17` là `solid-yellow`, `10` là `sign-no-parking`.
- Nhầm docs root cũ với docs đã phân loại trong `docs/architecture` và `docs/vision`.
- Bypass active trajectory bằng direct IPM lookahead mà không debug rõ `control_source`.
- Test một frame rồi kết luận memory/hysteresis đúng. Các lỗi trajectory manager cần multi-frame replay.
- Viết simulator logic riêng khiến kết quả khác production ROS pipeline.
- `TrajectoryLatch` (frozen turn execution) có bug đã biết chưa fix tính đến 2026-08-03: hệ số đổi tốc độ odom sai (`*2500.0` thay vì `*1000.0`), `/odom_raw` không được publish trong `docker-compose.prod.yml`, marking-abort không trip ngoài `TURN_LEFT`/T-junction, exempt legality bỏ qua cả verdict HARD. Xem `skills/decision_trajectory_SKILL/SKILL.md` và `system_report_current/06_gioi_han_va_kiem_thu.md` trước khi tin hành vi rẽ tại giao lộ đã đúng.
