# Plan D: Mở Rộng Regression + Trả Nợ Kỹ Thuật

Tương ứng roadmap refactor **Phase 9 (phần còn thiếu) + Phase 3 (dọn label) + Phase 10 (tách file)**. Làm CUỐI CÙNG, sau khi behavior đã đúng và ổn định (A, B, C xong) — đúng khuyến cáo roadmap: không tách file khi logic còn sai.

## 1. Hiện Trạng

- Live regression: 6/6 fixture pass nhưng thiếu các scenario roadmap Phase 9 yêu cầu: **dropout giữa maneuver**, **ID swap/recovery**, turn left two lanes, lane change dashed allowed (bản live — hiện chỉ có bản unit fixture).
- Magic label còn sót: `label != 17` tại `control_node.cpp:266` và `:3090` (phải dùng `LABEL_TURN_LANE`).
- `decision_harness.py` 1666 dòng mirror logic C++ — mỗi plan A/B/C đều phải sửa đôi; rủi ro lệch tăng dần.
- `control_node.cpp` 3661 dòng chứa toàn bộ pipeline — Phase 10 chưa làm.
- Bảng label trong `decision_sys_implementation_plan.md` là mapping cũ (`turn-lane=10`, `solid-white=6`...) — bẫy tài liệu cho người đọc sau.

## 2. Hạng Mục

### D1. Mở rộng scenario regression (Phase 9)

Fixture live mới trong `tools/local_post_inference_simulator/fixtures/` + assertion trong `test/local_post_inference_simulator/test_regression.py`:

| Fixture | Nội dung | Assertion chính |
|---|---|---|
| `turn_dropout_mid_maneuver.json` | Đang rẽ phải, turn-lane mất 3 frame giữa chừng | không rơi về follow_main giữa maneuver; hoàn tất rẽ; `replan_count` không tăng vì dropout |
| `lane_id_swap.json` | main-lane đổi `id`/`track_id` giữa 2 frame, hình học liên tục | `selected_lane_id` đổi tối đa 1 lần, không replan, trajectory liên tục |
| `turn_left_two_lanes_live.json` | port fixture unit sang live | chọn lane xa hơn, một active trajectory |
| `lane_change_dashed_allowed_live.json` | port fixture unit sang live | commit lane change, hoàn tất, quay về follow_main |

Mỗi fixture thêm dòng vào `docs/local_post_inference_simulator/scenario_refactor_mapping.md`.

Report replay bổ sung metric còn thiếu theo roadmap: `control_source_count` per kind, `blocked/recovery event list` (nếu harness chưa xuất).

### D2. Dọn label + docs (Phase 3 còn sót)

- Thay 2 chỗ `!= 17` bằng `LABEL_TURN_LANE`. (Thay đổi vào `control_node.cpp` — nhỏ, thuần cơ học, vẫn liệt kê xin duyệt theo ràng buộc toàn cục #2.)
- Thêm cảnh báo đầu bảng label trong `decision_sys_implementation_plan.md`: mapping trong đó đã lỗi thời, trỏ về `models/best_ncnn_model/metadata.yaml` + `label_mapping.json`. (Sửa docs, không sửa code.)
- Kiểm tra `label_mapping.json` và simulator docs đồng bộ mapping runtime.

### D3. Chính sách Python harness

**ĐÃ CHỐT (2026-07-05, user duyệt): Phương án 3 — đóng băng harness, logic mới → gtest.**

Bối cảnh khi chốt: sau khi D4 tách file, logic decision/trajectory nằm header-only (`trajectory_planner.hpp`, `trajectory_normalizer.hpp`, `trajectory_manager.hpp`...) và `ros2_ws/src/avs_perception/test/decision_trajectory_test.cpp` đã test C++ thật **offline, không cần ROS** (11 test pass). Vì A/B/C đã xong (churn logic gần kết thúc), rủi ro drift của mirror Python giờ thấp, nên không migrate big-bang. Chính sách:

- `decision_harness.py` (mirror 2111 dòng) **đóng băng**: KHÔNG thêm logic planner mới vào harness. 91 test Python hiện có giữ nguyên (đang pass, nhanh) cho tới khi teo dần tự nhiên.
- Mọi test logic thuần MỚI viết bằng **gtest** against header thật (`decision_trajectory_test.cpp`), không mirror sang Python.
- Khi cần sửa một mảng logic: chuyển đúng test của mảng đó sang gtest rồi xóa phần mirror tương ứng → tiến tới "replay client thuần" (phương án 1 cũ) mà không có cú migrate rủi ro.
- "Test tương ứng pass" của gate = `ament_add_gtest(decision_trajectory_test ...)` trong `BUILD_TESTING` (CMakeLists.txt) chạy xanh.

Hai phương án cân nhắc ban đầu (giữ lại để tham khảo):

1. **Harness là replay client thuần**: harness không tái hiện logic planner nữa mà chỉ feed fixture qua node C++ thật (hạ tầng live-ROS đã có) và đọc debug topic; các test logic thuần chuyển dần sang live hoặc test C++ nhỏ (gtest) cho helper hình học. Ưu: hết rủi ro lệch. Nhược (framing cũ, nay đã lỗi thời — gtest chạy offline không cần ROS): "test chậm hơn, cần ROS". Chi phí thực: migrate ~91 test ngay.
2. **Giữ harness mirror nhưng thêm consistency test**: một test chạy cùng fixture qua cả harness và node C++ live, so từng field debug. Ưu: giữ tốc độ unit test. Nhược: vẫn phải sửa đôi + consistency test cần ROS live, ngược hướng giảm nợ.

### D4. Tách file (Phase 10) — chỉ khi A/B/C đã merge và regression xanh ổn định

Thứ tự tách (build + full test sau MỖI file, không tách hàng loạt):

1. `decision_types.hpp` (enum, struct thuần — không logic)
2. `path_observation.hpp/cpp`
3. `trajectory_planner.hpp/cpp`
4. `trajectory_normalizer.hpp/cpp`
5. `trajectory_manager.hpp/cpp`
6. `control_error_projector.hpp/cpp`
7. `control_node.cpp` chỉ còn ROS orchestration + parameter loading

Ràng buộc kỹ thuật: cập nhật `CMakeLists.txt` từng bước; không include `json.hpp` lan vào header mới nếu tránh được (giữ raw JSON ở boundary parse); mỗi bước tách là một commit riêng, diff move-only (kiểm bằng `git diff --color-moved`), không sửa logic trong commit tách file.

## 3. Thiết Kế Unit Test

- D1: chính các assertion live regression trong bảng trên (mỗi fixture ≥ 3 assertion: hành vi chuỗi, invariant một trajectory, metric jitter/replan).
- D2: test nhỏ `test_no_magic_turn_lane_literal` — grep source khẳng định không còn literal `17` ngoài định nghĩa constant (chạy trong pytest bằng subprocess `rg`).
- D3 (nếu chọn phương án 2): `test_harness_matches_cxx_node` chạy fixture chuẩn qua cả hai, so `selected_lane_id`, `trajectory_kind`, `manager_action`, `decision_state`, sai số `epsilon_x_mm` < 1mm.
- D4: KHÔNG test mới — tiêu chí là toàn bộ test hiện có pass nguyên trạng sau mỗi commit tách file, và symbol map trước/sau giống nhau (move-only).

## 4. Điều Kiện Ràng Buộc Hoàn Thành

- [x] D1: **13/13** fixture live pass (vượt 10/10 yêu cầu: 6 cũ + 4 mới D1 + 3 T-junction Plan B); mapping doc cập nhật. *(verified 2026-07-05, `AVS_REQUIRE_LIVE_ROS=1`.)*
- [x] D2: grep sạch magic label (test `test_no_magic_turn_lane_literal` trong `test_plan_d_debt.py` — khóa cả label-vs-literal chung, không chỉ `17`; đã bắt & sửa thêm `legacy_lane_model.hpp` dùng raw `2/13/14` → `LABEL_*`); docs đã gắn cảnh báo mapping cũ (`decision_sys_implementation_plan.md:19`).
- [x] D3: phương án được user chốt (P3 — đóng băng harness, logic mới → gtest; 2026-07-05); test tương ứng = `decision_trajectory_test.cpp` (11 test) pass.
- [~] D4: tách file DONE — `control_node.cpp` 3661→**1114 dòng**, 6 header extract, gtest offline chạy. **CHƯA đạt** mục tiêu soft `< ~700 dòng` (phần dư là `telemetry_callback` orchestration — thu gọn thêm = đụng logic, cần user duyệt riêng). **CHƯA tách thành các commit move-only riêng** (hiện là 1 khối working-tree; D4 yêu cầu mỗi file 1 commit `--color-moved`).
- [x] Gate chung README pass: `colcon build` clean + `pytest` 93/93 + gtest 11/11 + live 13/13. *(verified 2026-07-05.)*
- [~] Danh sách thay đổi logic (control_node.cpp/header decision: D2 label relabel, C3 dropout-window gộp) — **chờ user duyệt qua bước commit** (user tự commit sau review, theo yêu cầu phiên này).

## 5. Rủi Ro

- Tách file dễ va chạm nếu có nhánh khác đang sửa `control_node.cpp` — chỉ bắt đầu D4 khi không còn plan nào khác mở trên file này.
- Fixture ID-swap phụ thuộc cách `ipm_transform_node` cấp `id`/`track_id`; nếu cần chỉnh IPM để tái hiện scenario, phải hỏi user trước (ràng buộc toàn cục #2 — IPM cũng thuộc diện duyệt).
