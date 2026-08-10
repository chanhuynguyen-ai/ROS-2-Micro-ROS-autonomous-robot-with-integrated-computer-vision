# Plan A: Planner Sinh Candidate Theo Intent Ở Mọi Frame + Intent Latch

Tương ứng roadmap refactor **Phase 1 + Phase 2**. Đây là plan ưu tiên cao nhất vì lỗi kiến trúc ở đây trực tiếp làm xe **không rẽ / không chuyển làn đúng lúc**: intent đã nhận nhưng bị nuốt hoặc bị reset.

## 1. Hiện Trạng (anchor code, khảo sát 2026-07-03)

Tất cả trong `ros2_ws/src/avs_perception/src/control_node.cpp`:

- **Intent bị bỏ qua khi state chưa chuyển maneuver**: nhánh `switch (active_eval_state)` case `FOLLOW_MAIN/RECOVERY/BLOCKED` (~dòng 1894–1908) có comment nguyên văn *"Always follow-main in this branch, ignore pending route intent until maneuver starts"*. Planner chỉ được gọi `plan_follow_main` dù `current_intent_` đang là `TURN_RIGHT`.
- **Maneuver kích hoạt bằng khoảng cách trong state machine**: `update_lane_state` (~dòng 3352) chỉ chuyển sang `TURN_*` khi `long_off < turn_proximity_mm_` (`turn_is_close`). Roadmap Phase 1 quy định: khoảng cách chỉ được là feasibility/commit guard trong manager, không phải điều kiện "có plan hay không".
- **Reset intent khi mất turn-lane 1 frame**: trong `update_lane_state`, khi `state_ == TURN_*` và `turn_lane_cand == nullptr` → `state_ = FOLLOW_MAIN; current_intent_ = FOLLOW_MAIN` ngay lập tức ("Turn-lane lost"). Vi phạm roadmap Phase 2: *"Không reset maneuver khi perception dropout ngắn hạn; dùng hold window"*. (Manager có `hold_maneuver_fallback` 1 frame ở tầng trajectory, nhưng intent ở tầng trên đã bị xóa nên frame sau planner không plan turn nữa — hold vô nghĩa.)
- **Chưa có helper trung tâm** `plan_candidate_for_intent(obs, intent, previous_state)`; dispatch planner nằm rải trong 3 nhánh switch (dòng ~1894, ~2004, ~2124).
- Intent đã có enum `RouteIntent` nhưng khi vào `TrajectoryManager::update` lại truyền `current_intent` dạng **string** và so sánh string với `trajectory_kind_name` (~dòng 1305–1312).
- `/avs/route_intent` chưa dùng `seq` để phân biệt "intent mới cùng loại".

## 2. Thay Đổi Dự Kiến (cần user duyệt trước khi code — đều trong `control_node.cpp`)

### A1. Helper trung tâm `plan_candidate_for_intent`

```cpp
PlannedTrajectory plan_candidate_for_intent(
    const PathObservationFrame& obs,
    RouteIntent intent,
    const CommittedTrajectoryState& previous_state,
    const TJunctionContext& t_ctx,          // is_t, t_junction_pending
    const std::string& last_main_track_id);
```

- Mapping cố định: `FOLLOW_MAIN → plan_follow_main`, `TURN_RIGHT → plan_turn_right`, `TURN_LEFT → plan_turn_left`, `LANE_CHANGE_* → plan_lane_change_*`.
- Được gọi **một lần mỗi frame**, trước khi state machine đánh giá feasibility. Ba nhánh switch hiện tại hợp về một flow duy nhất: `build obs → plan candidate theo intent → normalize → manager → project control`.
- Các planner con đã tồn tại (dòng 423–658), không viết lại logic bên trong ở plan này — chỉ đổi cách dispatch.

### A2. State machine chỉ còn vai trò feasibility/completion

- `update_lane_state` không quyết định "planner plan gì". Nó cung cấp cho manager: `maneuver_feasible` (turn-lane/other-lane có và không bị block), `maneuver_complete`, `blocked`.
- `turn_proximity_mm_` chuyển thành **commit guard** trong manager: candidate turn hợp lệ nhưng chưa đủ gần → manager `HOLD` follow-main committed trajectory, debug `hold_reason = "turn_not_in_range"`. Planner vẫn plan turn candidate và debug `candidate_trajectory_kind = "turn_right"` mỗi frame.

### A3. Intent latch + hold window

- Thêm `pending_intent_` (latched) tách khỏi `active_maneuver_` (đang thực thi).
- Intent chỉ được clear khi: (a) maneuver hoàn tất theo điều kiện completion hiện có, (b) user gửi intent mới/`follow_main`, (c) blocked quá `intent_abort_frames_` (parameter, đề xuất mặc định 30 frame) — clear kèm `replan_reason = "intent_aborted_blocked"`.
- Mất turn-lane N frame (`maneuver_dropout_hold_frames_`, đề xuất mặc định 5, đồng bộ với hold window của manager) → giữ intent + giữ committed maneuver trajectory; quá N frame → rơi về follow_main nhưng **giữ intent latched** để re-acquire khi turn-lane xuất hiện lại; quá `intent_abort_frames_` → clear intent.
- Mọi threshold mới là parameter ROS có tên rõ, không magic number.

### A4. Intent enum + seq xuyên suốt

- `TrajectoryManager::update` nhận `RouteIntent` enum + `intent_seq` thay cho string. So sánh "intent mới" = `(intent, seq)` đổi.
- Parse `seq` từ JSON `/avs/route_intent` (schema đã định nghĩa trong `decision_sys.md`); thiếu `seq` → coi mỗi message là seq tăng dần (tương thích ngược).
- Legacy path `LEGACY_TURN`/`LEGACY_LANE_CHANGE` (dòng ~1682–1698, ~1814–1838): giữ nguyên hành vi, chỉ map về enum sớm hơn; không mở rộng.

### A5. Debug fields bổ sung trên `/avs/lane_state`

- `pending_intent`, `intent_seq`, `intent_age_frames`, `maneuver_dropout_counter`, `hold_reason` (đã có — bảo đảm được set ở mọi nhánh mới).

**Không đổi**: `/avs/control_error` schema/cách tính; `ControlErrorProjector`/`publish_control_error_from_trajectory`; gate direct-IPM (Phase 7 đã có — chỉ thêm điều kiện đã ghi trong roadmap: không dùng direct-IPM khi có intent maneuver pending, nếu chưa có thì bổ sung ở plan này, đánh dấu rõ trong diff để duyệt).

## 3. Thiết Kế Unit Test (viết TRƯỚC khi sửa runtime)

Vị trí: `test/decision_system/test_plan_a_intent_planning.py` (dùng `decision_harness.py` hiện có; nếu harness thiếu API thì mở rộng harness trong cùng PR).

| Test | Setup | Assertion |
|---|---|---|
| `test_pending_turn_intent_plans_turn_candidate_every_frame` | Fixture có main-lane + turn-lane phải xa hơn `turn_proximity_mm_`; gửi intent `turn_right` | Mỗi frame: `candidate_trajectory_kind == "turn_right"`, `manager_action == "HOLD_CURRENT"`, `hold_reason == "turn_not_in_range"`, active trajectory vẫn là follow_main |
| `test_turn_commits_when_in_range` | Chuỗi frame turn-lane tiến dần vào trong `turn_proximity_mm_` | Đúng frame vào range: `manager_action == "COMMIT_NEW"`, `trajectory_kind == "turn_right"`, `replan_reason == "intent_change"` (hoặc reason mới đã duyệt) |
| `test_turn_lane_dropout_keeps_intent_within_hold_window` | Đang turn, turn-lane biến mất 3 frame rồi quay lại | Trong 3 frame: intent vẫn `turn_right`, committed trajectory kind vẫn turn; sau khi quay lại: tiếp tục turn, KHÔNG có commit follow_main xen giữa |
| `test_turn_lane_dropout_beyond_window_falls_back_but_relatches` | Turn-lane mất 8 frame (> hold window) rồi xuất hiện lại trước `intent_abort_frames_` | Rơi về follow_main có `replan_reason` rõ; khi turn-lane xuất hiện lại: planner lại sinh candidate turn và commit lại |
| `test_intent_abort_after_blocked_timeout` | Intent `lane_change_left` nhưng solid marking chặn liên tục > `intent_abort_frames_` | Intent bị clear, `replan_reason == "intent_aborted_blocked"`, xe vẫn follow_main liên tục không gián đoạn |
| `test_new_seq_same_intent_triggers_replan` | intent `turn_right` seq=1 hoàn tất, gửi `turn_right` seq=2 | Manager coi là trigger mới, plan turn lần hai |
| `test_invalid_intent_does_not_drop_committed_trajectory` | Đang follow_main ổn định, gửi JSON intent rác | Fallback `FOLLOW_MAIN`, committed trajectory không bị clear, không có replan |
| `test_follow_main_unchanged_without_intent` (regression guard) | Fixture `follow_main_straight`/`follow_main_curve` hiện có | Output từng frame khớp baseline trước refactor (chạy baseline snapshot trước khi sửa) |
| `test_single_active_trajectory_invariant` | Mọi fixture trên | Mỗi frame đúng một active trajectory; `control_source` chỉ một giá trị/frame |

Fixture mới cần tạo: `turn_intent_far_lane.json` (turn-lane ngoài proximity), `turn_dropout_relatch.json` (chuỗi ≥ 15 frame), `lane_change_blocked_timeout.json` (≥ 35 frame). Format theo fixture hiện có trong `test/decision_system/fixtures/`.

## 4. Điều Kiện Ràng Buộc Hoàn Thành (ngoài Gate chung trong README)

- [x] Baseline snapshot metric (jitter `epsilon_x_mm`, `theta_rad`, lane switch count, replan count) trên 6 fixture simulator được lưu **trước** khi sửa, so sánh **sau** khi sửa: follow_main không xấu đi (roadmap Phase 0 đã có hạ tầng — tái dùng).
- [x] Toàn bộ test bảng trên pass + 42 test cũ pass không sửa expected value.
- [x] 6/6 live regression fixture pass (`AVS_REQUIRE_LIVE_ROS=1 pytest -m ros test/local_post_inference_simulator/test_regression.py`).
- [x] Grep xác nhận: không còn nhánh nào hard-code plan follow_main khi `current_intent_`/`pending_intent_` là maneuver; comment *"ignore pending route intent"* bị xóa.
- [x] `/avs/lane_state` giải thích được đầy đủ chuỗi: intent nhận → candidate mỗi frame → lý do hold → commit → complete/abort.
- [x] Diff `control_node.cpp` được user duyệt trước khi merge (theo ràng buộc toàn cục #2).

## 5. Rủi Ro

- Hợp nhất 3 nhánh switch là thay đổi lớn nhất vào `control_node.cpp` từ trước tới nay → làm theo 2 bước nhỏ: (1) thêm `plan_candidate_for_intent` + gọi song song chỉ để ghi debug `candidate_trajectory_kind` (không đổi output), chạy full regression; (2) chuyển output sang flow mới, so baseline.
- Hold window tương tác với manager `hold_maneuver_fallback` hiện có (1 frame) — phải thống nhất một nguồn đếm dropout, tránh hai bộ đếm lệch nhau.
- Python harness (`decision_harness.py`) phải cập nhật cùng lúc, nếu không unit test sẽ test hành vi khác runtime C++.
