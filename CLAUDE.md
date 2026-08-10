# CLAUDE.md

Ghi chú làm việc cho Claude Code trong repo `SimpleSysIDV`. Đây là bản tóm tắt hành vi/quy tắc quan trọng nhất; nguồn đầy đủ và chi tiết hơn nằm ở `AGENTS.md` — đọc `AGENTS.md` khi cần chi tiết docs source-of-truth, danh sách node, hoặc quy trình build/test.

## Hệ Thống

AVS: vision/control ROS2 Humble, CPU-only trên Raspberry Pi 5. Pipeline:

```
camera/video -> ncnn_inference_node -> /avs/telemetry -> ipm_transform_node
-> /avs/telemetry_realworld -> control_node -> /avs/control_error + /avs/lane_state
-> controller/ESP32
```

Node chính: `ros2_ws/src/avs_perception/src/{ncnn_inference_node,ipm_transform_node,control_node,video_publisher_node,video_test_node,yolo26_seg}.cpp`.

`web_dashboard/` chỉ là UI runtime/bridge — không nhét simulator/tooling debug lớn vào đó; dùng `tools/local_post_inference_simulator/` cho tooling local, docs tương ứng ở `docs/local_post_inference_simulator/`.

## Label Mapping — Bẫy Quan Trọng Nhất

Source-of-truth: `config/label_mapping.json` (build sinh `label_mapping.hpp` với hằng `LABEL_*`) + `models/best_ncnn_model/metadata.yaml`. Bản sao thủ công phải sync khi đổi label: `class_names` trong `yolo26_seg.hpp`, `CLASS_NAMES` trong `web_dashboard/backend/main.py` + `app.js`, `DEFAULT_CLASS_NAMES` trong simulator `mask_to_objects.py`, `CLASS_NAMES` trong `avs_controlsystem/`, hằng `LABEL_*` trong `test/decision_system/decision_harness.py`, và label số trong fixtures.

Model hiện tại 22 class (thêm `light_green/light_red/light_yellow` ở 3–5, mọi label cũ >= 3 dịch +3).

Quan trọng nhất: **`turn-lane = 20`**, không phải `17` (model 19 class cũ) hay `10`. Repo từng có regression dùng `10` làm turn-lane trong `ipm_transform_node.cpp` — nếu thấy `label == 10` hoặc `label == 17` xử lý như turn-lane, đó là bug cần fix (`17 = solid-yellow`, `10 = sign-no-parking`).

Labels khác dùng trong decision/planning: `main-lane=6`, `other-lane=7`, `dashed-white=0`, `dashed-yellow=1`, `double-solid-white=2`, `solid-white=16`, `solid-yellow=17`, `stop-line=19`.

## Nguyên Tắc Kiến Trúc

- Không đổi controller downstream nếu không có yêu cầu rõ ràng; giữ contract `/avs/control_error`.
- Contract `/avs/control_error` đóng băng: không đổi schema (tên field, đơn vị, dấu) và không đổi cách tính theo hướng phá tương thích controller tầng thấp (Pure Pursuit/PD/ESP32).
- Mọi thay đổi vào `control_node.cpp` hoặc `ipm_transform_node.cpp` phải liệt kê cụ thể (hàm/hành vi đổi) và được user duyệt trước khi code.
- Khi làm decision/trajectory: đọc `docs/plans/README.md` trước, tuân thủ Ràng Buộc Toàn Cục và Gate Hoàn Thành trong đó; chỉ báo hoàn thành phase khi checklist của plan tương ứng pass đủ.
- Decision/planning theo hướng: `path observation -> candidate trajectory -> normalized trajectory -> committed active trajectory`. Mỗi frame chỉ một active trajectory.
- `stop-line` KHÔNG được dùng để kích hoạt rẽ / phát hiện giao lộ / phát hiện T-junction / chuyển làn (giai đoạn hiện tại).
- `/avs/route_intent` là nguồn intent lái; `/avs/cmd` chỉ cho lệnh hệ thống/legacy.
- Simulator hậu-inference ưu tiên ROS-first: UI chỉ vẽ/cấu hình, pipeline chạy qua ROS node/topic thật hoặc synthetic ROS publisher. Không copy planner logic sang frontend/backend simulator — source-of-truth planning là C++ `control_node`; Python harness chỉ để test/đối chiếu.

## Build & Test

```bash
cd ros2_ws
colcon build --symlink-install --packages-select avs_perception
```

Layout user (tránh đụng build/install/log mặc định — đây là layout đang được dùng gần nhất, đã build thành công):

```bash
cd ros2_ws
colcon --log-base log_user build --symlink-install --packages-select avs_perception --build-base build_user --install-base install_user
```

Test decision system:

```bash
pytest -q test/decision_system
```

Sau khi sửa C++ node trong `ros2_ws/src/avs_perception/src/`, tối thiểu chạy cả `pytest -q test/decision_system` và `colcon build --symlink-install --packages-select avs_perception`.

**Chính sách test decision/trajectory (chốt Plan D3, 2026-07-05):** `test/decision_system/decision_harness.py` là mirror Python **ĐÃ ĐÓNG BĂNG** — KHÔNG thêm logic planner mới vào harness. Logic decision/trajectory giờ header-only (`ros2_ws/src/avs_perception/include/avs_perception/trajectory_*.hpp`); mọi test logic thuần MỚI viết bằng **gtest** against header thật trong `ros2_ws/src/avs_perception/test/decision_trajectory_test.cpp` (chạy offline, không cần ROS). Khi sửa một mảng logic, chuyển test mảng đó sang gtest rồi xóa phần mirror tương ứng — mirror teo dần, không migrate big-bang.

Không sửa/chạy trong `ncnn-src/` trừ khi task liên quan trực tiếp thư viện NCNN upstream.

## Quy Tắc Repo Hygiene

- Không sửa build artifacts: `build/`, `install/`, `log/`, `ros2_ws/build*`, `ros2_ws/install*`, `ros2_ws/log*`. Nếu chúng dirty sau build, ghi nhận nhưng không revert trừ khi user yêu cầu.
- Không revert thay đổi chưa rõ nguồn gốc — repo có thể đang dirty từ trước (kiểm tra `git status` trước các thao tác có thể mất dữ liệu).
- Refactor `control_node.cpp`: thêm test/fixture trước nếu behavior có thể đổi.
- Gom label constants: cập nhật đồng bộ inference, IPM, control, test, `label_mapping.json`, simulator docs.
- Dùng `rg`/`rg --files` để tìm file/symbol.

## Đọc Docs Theo Nhiệm Vụ

Decision/planning (theo thứ tự): `docs/architecture/decision_sys.md` → `docs/architecture/trajectory_planning_memory_proposal.md` → `docs/architecture/decision_trajectory_refactor_roadmap.md` → (`docs/local_post_inference_simulator/plan.md` nếu làm simulator).

Vision/IPM (theo thứ tự): `docs/vision/homography_theory.md` → `docs/vision/homography_implementation_plan.md` → `docs/vision/pixel_to_world_plan.md`.

Vận hành/deploy: `docs/guides/deployment_guide.md`, `docs/guides/ros2_yolo26_ncnn_setup_guide.md`, `docs/guides/pure_pursuit_guide.md`.

Docs cũ từng ở thẳng dưới `docs/` đã chuyển vào `docs/architecture`, `docs/vision`, `docs/guides`, `docs/reports` — luôn dùng path hiện tại, đừng tin path cũ trong lịch sử/memory.

`system_report_current/` là báo cáo hệ thống đối chiếu trực tiếp với code hiện tại (00_tong_quan → 06_gioi_han_va_kiem_thu), phạm vi dừng ở `/avs/control_error` (không phân tích controller downstream). Cập nhật gần nhất 2026-08-03, đồng bộ với `TrajectoryLatch`/gate diagnostic mới nhất — dùng khi cần tra cứu chi tiết công thức/hằng số/JSON schema mà không phải đọc lại toàn bộ code.

## Skills (`skills/`)

Đọc trước khi làm task liên quan:

- `skills/camera_SKILL/SKILL.md` — camera, udev, device mapping, hardware Pi 5/ESP32.
- `skills/docker_SKILL/SKILL.md` — kiến trúc 3 container Docker, deployment.
- `skills/data_transport_SKILL/SKILL.md` — ROS2 phân tán Pi-to-laptop, DDS/Zenoh, bandwidth.
- `skills/labeling_SKILL/SKILL.md` — quy tắc annotation lane/marking, định nghĩa class segmentation.
- `skills/decision_trajectory_SKILL/SKILL.md` — gotchas cho `control_node.cpp`/`trajectory_*.hpp`: `TrajectoryLatch`, gate diagnostic lane-change, turn-lane exempt khỏi legality gate, kèm bug đã biết chưa fix. Đọc trước khi sửa vùng này.
- `skills/karpathy-guidelines/SKILL.md` — coding style: simplicity-first, surgical changes, surface assumptions trước khi code. Áp dụng mặc định cho mọi task code trong repo này.

## Các Bẫy Dễ Sai

- `turn-lane` label `20` vs `17`/`10` (xem trên).
- Nhầm docs root cũ với docs đã phân loại trong `docs/architecture`, `docs/vision`, `docs/guides`.
- Bypass active trajectory bằng direct IPM lookahead mà không debug rõ `control_source`.
- Test một frame rồi kết luận memory/hysteresis đúng — lỗi trajectory manager cần multi-frame replay.
- Viết simulator logic riêng khiến kết quả khác production ROS pipeline.
- `TrajectoryLatch` (frozen turn execution, `trajectory_latch.hpp`) có bug đã biết chưa fix tính đến 2026-08-03: hệ số đổi tốc độ odom sai (`*2500.0` thay vì `*1000.0`), `/odom_raw` không được publish trong `docker-compose.prod.yml`, marking-abort không trip ngoài `TURN_LEFT`/T-junction, exempt legality bỏ qua cả verdict HARD. Đừng coi hành vi rẽ khi turn-lane mất khỏi tầm nhìn là đã verify đúng — xem `skills/decision_trajectory_SKILL/SKILL.md` và `system_report_current/06_gioi_han_va_kiem_thu.md`.
