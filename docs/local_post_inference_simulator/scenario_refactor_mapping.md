# Scenario Fixture → Refactor Roadmap Mapping

Tài liệu này ánh xạ mỗi fixture trong `tools/local_post_inference_simulator/fixtures/` tới ít nhất một phase trong `docs/architecture/decision_trajectory_refactor_roadmap.md`, theo đúng yêu cầu Phase 8 của simulator plan.

**Nguyên tắc**: mỗi scenario fixture là một regression guard — khi một phase refactor thay đổi behavior, fixture tương ứng phải vẫn pass assertions nếu refactor đúng, hoặc fail rõ ràng nếu contract bị phá.

---

## Coverage Status

| Refactor Phase | Fixture bảo vệ | Trạng thái |
|---|---|---|
| Phase 0 (Baseline) | — | ⚪ Intentionally skipped — baseline snapshot, không đổi behavior |
| Phase 1 (Runtime Flow) | `turn_right_two_lanes.json` | ✅ Covered |
| Phase 2 (Intent/State) | `turn_right_two_lanes.json` | ✅ Covered |
| Phase 3 (PathObservationBuilder) | — | ⚪ Intentionally skipped — refactor nội bộ, behavior quan sát không đổi |
| Phase 4 (Lane Selection Geometry) | `intersection_follow_main.json`, `t_junction_no_stopline.json`, `t_junction_not_triggered_by_stopline.json` | ✅ Covered — pass live (bao gồm R3: T-junction detect thuần geometry, không do stop-line) |
| Phase 5 (Normalizer Arc-Length) | `follow_main_curve.json` | ✅ Covered — pass live |
| Phase 6 (Manager Replan Policy) | `follow_main_dropout.json` | ✅ Covered — pass live |
| Phase 7 (ControlErrorProjector) | `follow_main_straight.json`, `turn_right_two_lanes.json` | ✅ Covered — `expected_control_source` đã thêm, verify live: `direct_ipm` khi follow straight, `trajectory_manager` khi turn (không bao giờ bypass active trajectory lúc turn). Xem chi tiết bên dưới. |
| Phase 8 (Marking Gate) | `lane_change_solid_blocked.json`, `t_junction_turn_left_blocked_by_solid.json` | ✅ Covered — pass live (bao gồm R4: turn_left qua T-junction bị block bởi solid marking) |
| Phase 9 (Replay/Regression) | `follow_main_straight.json`, `follow_main_curve.json`, `turn_dropout_mid_maneuver.json`, `lane_id_swap.json`, `turn_left_two_lanes_live.json`, `lane_change_dashed_allowed_live.json` | ✅ Covered — pass live |
| Phase 10 (Tách File) | — | ⚪ Intentionally skipped — tách file, không đổi behavior |

**Chú thích**:
- ✅ Covered = có fixture với assertions bảo vệ behavior của phase, đã verify live pass
- 🔴 = fixture/assertion đã viết nhưng khi chạy live-ROS thật (`pytest -m ros`) đang FAIL — coverage tồn tại về mặt hạ tầng nhưng KHÔNG chứng minh được behavior đúng; cần điều tra riêng, xem "Live Regression Status"
- ⚪ Intentionally skipped = phase này refactor nội bộ/tách file/baseline, không có behavior quan sát mới cần guard riêng

**Lưu ý quan trọng**: trước đây toàn bộ 6 test `-m ros` trong `test_regression.py` bị skip âm thầm do một bug đường dẫn (`workspace_dir` trỏ ra ngoài repo nên node ROS không bao giờ start) — nghĩa là các dòng ✅ Covered ở trên (trước khi sửa bug này) chưa từng thực sự được chạy live. Sau khi sửa bug skip và điều tra/sửa lần lượt 4 fixture còn lại (geometry vanishing-row, id collision xuyên fixture gây rò state trong `ipm_transform_node`, thiếu relabel `BLOCKED_FOLLOW_MAIN`, và metric `replan_count` đếm sai trong harness), **cả 6/6 fixture hiện pass live** (`AVS_REQUIRE_LIVE_ROS=1 pytest -m ros test/local_post_inference_simulator/test_regression.py`). Xem "Live Regression Status" bên dưới để biết chi tiết từng fixture.

---

## Chi Tiết Theo Từng Fixture

### `follow_main_straight.json`

**Refactor Phase bảo vệ**: Phase 9 — Replay Test và Scenario Regression

**Scenario**: `main-lane` rộng thẳng + `main-lane` phía trước (hai polygon nối nhau), route intent `follow_main`.

**Assertions**:
- `expected_selected_lane` = một main-lane object
- `max_lane_switch_count: 0` — không được đổi lane giữa các frame
- `max_jitter_epsilon_x_mm: 50.0` — lateral jitter thấp trên đường thẳng
- `max_jitter_theta_rad: 0.1`
- `expected_blocked_state: false`
- `expected_trajectory_kind: "follow_main"`

**Lý do**: Baseline regression. Nếu bất kỳ refactor nào (normalizer, manager, planner) làm hỏng follow-main thẳng, fixture này phải phát hiện ngay.

---

### `follow_main_curve.json`

**Refactor Phase bảo vệ**: Phase 9 + Phase 5 (Normalizer theo Arc-Length)

**Scenario**: `main-lane` cong (polygon lệch dần) qua nhiều frame, route intent `follow_main`.

**Assertions**:
- `expected_selected_lane` = main-lane object
- `max_lane_switch_count: 0`
- `max_jitter_epsilon_x_mm: 60.0` — cho phép jitter cao hơn một chút so với thẳng do cong
- `max_jitter_theta_rad: 0.15`
- `expected_blocked_state: false`
- `expected_trajectory_kind: "follow_main"`

**Lý do**: Sau khi Phase 5 refactor normalizer sang arc-length, trajectory curve phải mượt hơn, không lag hoặc snap. Fixture này xác nhận jitter không tăng sau refactor.

---

### `follow_main_dropout.json`

**Refactor Phase bảo vệ**: Phase 6 — Manager Replan Policy

**Scenario**: 10 frame — 3 frame main-lane ổn định → 3 frame dropout (objects rỗng) → 4 frame phục hồi. Route intent `follow_main`.

**Frame breakdown**:
- Frames 1–3: `fmd_main_lane_current` visible, polygon dịch nhẹ (±2px) mỗi frame
- Frames 4–6: `objects: []` — mô phỏng dropout/miss-detection ngắn hạn
- Frames 7–10: `fmd_main_lane_current` visible trở lại, confidence 0.95→1.0

**Assertions**:
- `expected_selected_lane: "fmd_main_lane_current"` — sau recovery, lane phải về đúng id cũ
- `max_lane_switch_count: 1` — cho phép tối đa 1 switch (lúc recovery từ dropout về lane)
- `max_jitter_epsilon_x_mm: 80.0` — jitter cao hơn do dropout
- `max_jitter_theta_rad: 0.2`
- `expected_blocked_state: false`
- `expected_trajectory_kind: "follow_main"`
- `max_replan_count: 4` — hold window ≤5 frame; sau dropout 3 frame phải replan khi phục hồi. Metric này đếm **số lần `replan_reason` đổi giá trị so với frame trước** (cùng pattern với `selected_lane_switch_count`/`trajectory_kind_switch_count`), không phải số frame có `replan_reason != "none"` — harness từng đếm sai kiểu sau, khiến giá trị luôn ≈ tổng số frame hợp lệ (10) bất kể manager có thực sự replan hay không.
- `max_invalid_frame_count: 3` — đúng bằng số frame dropout (frames 4–6)

**Lý do**: Phase 6 yêu cầu test "dropout ngắn hạn", "persistent dropout sau hold window", và "hold vs replan policy" (roadmap:334-338). 3 frame dropout nằm trong hold window (≤5 frame theo control_node.cpp:1329), nên manager phải hold chứ không clear trajectory. Khi lane phục hồi ở frame 7, manager replan với candidate mới. Fixture này bắt các regression sau refactor Phase 6 làm hold window sai, replan quá sớm, hoặc không recovery được sau dropout.

---

### `intersection_follow_main.json`

**Refactor Phase bảo vệ**: Phase 4 — Chuẩn Hóa Chọn Lane Theo Geometry và Hysteresis

**Scenario**: `main-lane` hiện tại + `main-lane` phía trước + một `stop-line` ở giữa, route intent `follow_main`.

**Assertions**:
- `expected_selected_lane: "ifm_main_lane_current"` — phải chọn lane gần xe, không nhảy sang main_ahead
- `max_lane_switch_count: 0`
- `max_jitter_epsilon_x_mm: 50.0`
- `max_jitter_theta_rad: 0.1`
- `expected_blocked_state: false` — stop-line **không** được trigger blocked state
- `expected_trajectory_kind: "follow_main"`

**Lý do**: Bảo vệ invariant cốt lõi: **stop-line không được dùng để kích hoạt rẽ, phát hiện giao lộ, phát hiện T-junction hoặc quyết định chuyển làn**. Nếu Phase 4 refactor làm lane selection nhảy khi có stop-line, fixture fail ngay.

---

### `lane_change_solid_blocked.json`

**Refactor Phase bảo vệ**: Phase 8 refactor — Marking Gate và Blocked Behavior

**Scenario**: `main-lane` bên phải + `other-lane` bên trái + `solid-white` marking ở giữa, route intent `lane_change_left`.

**Assertions**:
- `expected_selected_lane: "lcsb_main_lane_1"` — giữ main-lane, không chuyển sang other-lane
- `max_lane_switch_count: 0`
- `max_jitter_epsilon_x_mm: 50.0`
- `max_jitter_theta_rad: 0.1`
- `expected_blocked_state: true` — solid marking trigger `blocked_by_marking=true`
- `expected_trajectory_kind: "blocked_follow_main"` — khi bị blocked, trajectory_kind_name() trả về `"blocked_follow_main"` (control_node.cpp:149), không phải `"follow_main"`

**Lý do**: Core contract của Phase 8 refactor: `is_lane_change_blocked_by_solid` phải hoạt động trong corridor giữa hai lane. `expected_trajectory_kind: "blocked_follow_main"` là khớp chính xác với enum `TrajectoryKind::BLOCKED_FOLLOW_MAIN` trong C++ — không phải string tùy ý.

---

### `t_junction_no_stopline.json`

**Refactor Phase bảo vệ**: Phase 4 (Lane Selection Geometry) — R3 trong `docs/plans/plan_B_lane_rule_conformance.md`

**Scenario**: `main-lane` cụt (không có `main_ahead`) + hai `turn-lane` dàn trải rộng (>2000mm giữa min/max turn_x, world-Y gần với điểm cuối main) tạo hình học T-junction, route intent `turn_right`, **không có object `stop-line` nào trong scene**.

**Assertions**:
- `expected_selected_lane: "tjns_turn_right"` — sau khi hysteresis xác nhận, phải commit rẽ phải
- `max_lane_switch_count: 1` — đúng 1 lần chuyển từ main sang turn khi T-junction được confirm
- `expected_blocked_state: false`
- `expected_trajectory_kind: "turn_right"`

**Bổ sung ngoài schema chung**: test `test_t_junction_detected_purely_from_geometry_no_stopline` (`test/local_post_inference_simulator/test_t_junction_live.py`) đọc trực tiếp `hold_reason` từ `/avs/lane_state` qua từng frame và assert 2 frame đầu `hold_reason == "t_junction_pending"` (geometry khớp nhưng `t_junction_counter_` chưa đủ 3) rồi frame cuối `hold_reason == ""` (đã confirm) — chứng minh `detect_t_junction` được kích hoạt thuần túy bởi geometry (`is_t_geom`), không phụ thuộc bất kỳ object stop-line nào vì scene này không có stop-line.

---

### `t_junction_not_triggered_by_stopline.json`

**Refactor Phase bảo vệ**: Phase 4 — R3 (mặt đối chứng)

**Scenario**: Cùng hình học turn-lane dàn trải như fixture trên (đủ điều kiện "trông giống" T-junction), nhưng có thêm `main_ahead` + một object `stop-line` thật ở giữa main_current/main_ahead, route intent `follow_main`.

**Assertions**:
- `expected_selected_lane: "tjnt_main_current"` — không được nhảy sang turn-lane
- `max_lane_switch_count: 0`
- `expected_blocked_state: false`
- `expected_trajectory_kind: "follow_main"`

**Bổ sung ngoài schema chung**: test `test_t_junction_not_triggered_by_stopline_alone` assert `hold_reason` không bao giờ là `"t_junction_pending"` ở bất kỳ frame nào — vì `main_ahead` tồn tại nên điều kiện đầu tiên của `detect_t_junction` (`main_current && !main_ahead`) không bao giờ đúng, bất kể có stop-line hay turn-lane dàn trải cỡ nào. Đây là phần bù cần thiết của R3: chứng minh riêng object `stop-line` không đủ để tự nó kích hoạt detection.

---

### `t_junction_turn_left_blocked_by_solid.json`

**Refactor Phase bảo vệ**: Phase 8 (Marking Gate) — R4

**Scenario**: Cùng hình học T-junction như trên (route intent `turn_left`), cộng thêm một `solid-white` marking (label 13) đặt chắn ngay corridor mà `plan_transition` phải đi qua khi rẽ trái (world X≈[-172,+6], Y≈[249,472] — bao trùm phần đầu của Bezier transition path ngay sau khi rời main-lane).

**Assertions**:
- `expected_selected_lane: "tjlb_main_current"` — giữ nguyên main, không commit rẽ trái
- `max_lane_switch_count: 0`
- `expected_blocked_state: true`
- `expected_trajectory_kind: "blocked_follow_main"`

**Bổ sung ngoài schema chung**: test `test_t_junction_turn_left_blocked_by_solid_marking` assert frame cuối `hold_reason == "blocked_by_marking"`, `blocked_by_marking is True`, `trajectory_kind == "blocked_follow_main"`, `selected_lane_id == "tjlb_main_current"`.

**Lưu ý thiết kế fixture (đáng ghi lại vì phản trực giác)**: lần thử đầu tiên đặt marking chỉ dựa trên vị trí world của TURN-LANE waypoints (giả định trajectory rẽ trái ≈ raw waypoints của turn-lane) — sai, vì `is_turn_blocked_by_solid` kiểm tra trajectory THỰC TẾ do `plan_transition` sinh ra (Bezier từ P0 gần main-lane đến P3 ~1200mm dọc turn-lane), và đoạn đầu của Bezier đó bám khá sát world-X≈0 (gần main-lane) trong khoảng world-Y ngắn ngay sau điểm rời main, chứ không đi theo hình dạng turn-lane thô. Marking đặt đúng theo turn-lane waypoints (world X≈-91,Y≈333, kích thước ~90x40mm) nằm ngoài path Bezier thật (~20mm lệch) nên không bị phát hiện block trong lần chạy live đầu tiên. Đã sửa bằng cách tính lại vùng world mà đoạn đầu Bezier thực sự đi qua (dò bằng cách suy ngược từng bước `plan_transition`: P0, P1, P2, P3, rồi sample công thức Bezier tại nhiều t nhỏ) và mở rộng marking pixel-space bao phủ vùng world X≈[-172,+6] Y≈[249,472] — đủ rộng để chắc chắn bắt được đoạn resampled-path bất kể sai số ước lượng tay. Bài học: **không được suy ra trajectory rẽ thật từ raw turn-lane waypoints khi thiết kế fixture blocking — phải dựa trên hình học mà `plan_transition`/Bezier thực sự tạo ra.**

---

### `turn_right_two_lanes.json`

**Refactor Phase bảo vệ**: Phase 1 (Runtime Flow) + Phase 2 (Intent/State Normalization)

**Scenario**: `main-lane` bên trái + `turn-lane` (label **20**) bên phải, 2 frame (turn-lane dịch nhẹ), route intent `turn_right`.

**Assertions**:
- `expected_selected_lane: "trtl_turn_lane_1"` — phải chọn turn-lane, không giữ main-lane
- `max_lane_switch_count: 0` — không swap giữa 2 frame
- `max_jitter_epsilon_x_mm: 50.0`
- `max_jitter_theta_rad: 0.1`
- `expected_blocked_state: false`
- `expected_trajectory_kind: "turn_right"` — khớp `trajectory_kind_name(TrajectoryKind::TURN_RIGHT)` = `"turn_right"` (control_node.cpp:145)

**Lý do**: Bảo vệ hai invariant:
1. Route intent `turn_right` phải được planner thực thi ngay, không bị state machine `FOLLOW_MAIN` che khuất (Phase 1).
2. `turn-lane` label phải là **20** (model 22 class) — không phải 17 (`solid-yellow`, model 19 class cũ) hay 10 (`sign-no-parking`). Nếu IPM hoặc control node còn logic cũ, planner không nhận turn-lane hợp lệ và fixture fail ở `expected_selected_lane` (Phase 2 — invariant label mapping).

---

### `turn_dropout_mid_maneuver.json`

**Refactor Phase bảo vệ**: Phase 9 (Replay/Regression) — gap Phase 9 nêu trong `docs/plans/plan_D_regression_and_debt.md` (D1)

**Scenario**: Đang rẽ phải (`turn_right`), `tdm_turn_lane_1` biến mất khỏi perception đúng 3 frame giữa chừng (frame 3–5), rồi xuất hiện lại (frame 6–8). `tdm_main_lane_1` luôn hiện diện.

**Assertions**:
- `expected_selected_lane: "tdm_turn_lane_1"` — sau dropout phải quay lại đúng turn-lane cũ, không rơi về main-lane
- `max_lane_switch_count: 1`
- `max_jitter_epsilon_x_mm: 80.0`
- `max_jitter_theta_rad: 0.2`
- `expected_blocked_state: false`
- `expected_trajectory_kind: "turn_right"` — không được rơi về `follow_main` giữa maneuver
- `max_invalid_frame_count: 0` — dropout 3 frame nằm trong `maneuver_dropout_hold_frames=5`, nên không frame nào được tính invalid

**Lý do**: Bảo vệ đúng invariant D1 yêu cầu: dropout ngắn hạn giữa maneuver rẽ không được làm rớt về `follow_main` hay tăng invalid/replan count, miễn dropout nằm trong hold window.

---

### `lane_id_swap.json`

**Refactor Phase bảo vệ**: Phase 9 (Replay/Regression) — gap Phase 9 (D1)

**Scenario**: `main-lane` đổi `id` object (`lis_main_lane_a` → `lis_main_lane_b`) giữa frame 3 và 4 trong khi hình học (polygon world position) liên tục — mô phỏng tracker cấp lại ID mới cho cùng một lane vật lý. Route intent `follow_main`.

**Assertions**:
- `max_lane_switch_count: 1` — id đổi tối đa gây 1 lần "switch" (do so sánh theo id), không được flicker nhiều lần
- `max_replan_count: 1`
- `max_jitter_epsilon_x_mm: 50.0`
- `max_jitter_theta_rad: 0.1`
- `expected_blocked_state: false`
- `expected_trajectory_kind: "follow_main"`

**Lý do**: Bảo vệ invariant D1: đổi `id`/`track_id` khi hình học liên tục không được coi là mất lane hoàn toàn hay gây replan lặp — chỉ một lần chuyển đổi hợp lý sang id mới.

---

### `turn_left_two_lanes_live.json`

**Refactor Phase bảo vệ**: Phase 9 (Replay/Regression) — port live của `test/decision_system/fixtures/turn_left_two_lanes.json` (D1)

**Scenario**: Hai `turn-lane` candidate cho rẽ trái (`tltl_turn_lane_further`, `tltl_turn_lane_closer`) cùng `tltl_main_lane_1`, 2 frame ổn định, route intent `turn_left`.

**Assertions**:
- `expected_selected_lane: "tltl_turn_lane_further"` — với `turn_left`, phải chọn candidate xa hơn (theo nearest-point distance), không phải gần hơn
- `max_lane_switch_count: 0`
- `max_jitter_epsilon_x_mm: 80.0`
- `max_jitter_theta_rad: 0.15`
- `expected_blocked_state: false`
- `expected_trajectory_kind: "turn_left"`

**Lý do**: Đối chứng trực tiếp với `turn_right_two_lanes.json` (chọn gần hơn) — cùng một helper `select_turn_lane_obs`/`select_turn_lane` nhưng đảo chiều tiêu chí theo `is_turn_right`. Bảo vệ để refactor không làm gộp nhầm logic near/far giữa hai hướng rẽ.

---

### `lane_change_dashed_allowed_live.json`

**Refactor Phase bảo vệ**: Phase 9 (Replay/Regression) — port live của lane-change unit fixture + case abort-timeout (D1)

**Scenario**: `lcda_main_lane_1` + `lcda_dashed_white` (label 0) trong corridor + `lcda_other_lane_left` chỉ xuất hiện ở frame 1–3, route intent `lane_change_left`, 37 frame. Từ frame 4 trở đi other-lane biến mất hoàn toàn (không quay lại) — mô phỏng target lane bị mất dài hạn sau khi đã commit.

**Hành vi live thực đo** (không phải giả định — đã chạy qua `ScenarioRunner` thật và log per-frame để xác nhận trước khi viết assertion):
- Frame 1–6: `dashed-white` không block → commit `lane_change_left`, `selected_lane_id = "lcda_other_lane_left"` (giữ qua hold window dù input đã mất từ frame 4, vì nằm trong `maneuver_dropout_hold_frames`)
- Frame 7: dropout vượt hold window → 1 frame invalid (`selected_lane_id: None`, `trajectory_kind: "unknown"`), phát sinh recovery event `persistent_invalid_clear`
- Frame 8–37: abort hẳn về `follow_main` trên `lcda_main_lane_1`, ổn định tới hết fixture (34 frame liên tục, vượt quá `intent_abort_frames=30`)

**Assertions**:
- `expected_selected_lane: "lcda_main_lane_1"` — trạng thái cuối cùng phải quay về main-lane
- `expected_trajectory_kind: "follow_main"` — đã hoàn tất abort, không còn kẹt ở `lane_change_left`/`unknown`
- `expected_blocked_state: false`
- `max_lane_switch_count: 1`
- `max_replan_count: 4`
- `max_invalid_frame_count: 1`
- `max_jitter_epsilon_x_mm: 60.0`
- `max_jitter_theta_rad: 0.5`
- `expected_control_source: "trajectory_manager"`

**Lý do**: D1 yêu cầu bảo vệ "commit lane change, hoàn tất, quay về follow_main". Fixture này đi xa hơn phiên bản unit gốc: thay vì chỉ port trực tiếp fixture ngắn, nó còn bọc thêm case abort dài hạn (mất target lane sau commit) để bảo vệ luôn nhánh `intent_abort_frames` — nếu refactor sau này làm mất khả năng tự phục hồi về `follow_main` khi target lane biến mất vĩnh viễn, fixture fail ngay ở `expected_trajectory_kind`.

---

## Invariant Quan Trọng Được Bảo Vệ Chung

1. **turn-lane = 20**: bắt regression label mapping qua `turn_right_two_lanes`.
2. **stop-line không trigger decision**: bảo vệ bởi `intersection_follow_main`.
3. **Một active trajectory duy nhất mỗi frame**: bảo vệ qua `expected_trajectory_kind` nhất quán.
4. **`/avs/route_intent` là nguồn intent**: tất cả fixture đều set intent qua `route_intent` field của scenario.
5. **Hold window ≤5 frame**: bảo vệ bởi `follow_main_dropout` (3 frame dropout phải hold, không clear).
6. **`blocked_follow_main` là kind riêng biệt**: bắt regression nếu blocked behavior bị merge vào `follow_main` thông thường.

---

## Phase Intentionally Skipped

| Phase | Lý do không cần fixture riêng |
|---|---|
| Phase 0 (Baseline) | Snapshot đo lường, không thay đổi behavior. Các fixture hiện tại là baseline sau khi pipeline ổn. |
| Phase 3 (PathObservationBuilder) | Refactor internal parsing — output `/avs/lane_state` và `/avs/control_error` không đổi nếu refactor đúng. Covered gián tiếp qua mọi fixture. |
| Phase 10 (Tách File) | Tách `.cpp` thành module — zero behavior change. Build test (`colcon build`) là verification. |

### Phase 7 (ControlErrorProjector) — Đã đóng gap

Trước đây field `control_source` trong `/avs/lane_state` và `/avs/control_error` bị hardcode `"trajectory_manager"` bất kể nhánh nào chạy thật, nên không assert được invariant "direct-IPM không bypass active trajectory lúc turn/lane-change". Đã sửa trong `control_node.cpp`: `control_source` giờ được set động theo nhánh `use_direct_lookahead` thực tế (`"direct_ipm"` / `"trajectory_manager"`), và `AssertionsSchema`/`evaluate_assertions` đã có field `expected_control_source`.

Verify live trên ROS graph thật:
- `follow_main_straight.json` → `control_source: "direct_ipm"` (đúng — không có lane tiếp diễn, dùng direct IPM shortcut)
- `turn_right_two_lanes.json` → `control_source: "trajectory_manager"` (đúng — direct-IPM không bao giờ kích hoạt khi đang turn)

## Live Regression Status (cập nhật sau khi sửa bug hạ tầng test)

Trước đây `test_regression.py` có bug đường dẫn (`workspace_dir` thừa một cấp `../`, trỏ ra `/home/<user>/ros2_ws` thay vì `<repo>/ros2_ws`) khiến `setup_script` không bao giờ tồn tại — toàn bộ 6 test `-m ros` skip âm thầm với message "Required ROS2 nodes ... are not subscribing", dễ khiến người đọc tưởng là do môi trường chứ không phải do node chưa từng được start. Đã sửa path này, cùng với tên tham số calibration sai (`calibration_file_path` chứ không phải `calibration_path`) và một race condition trong `ScenarioRunner` khiến report có thể chưa sẵn sàng khi `is_playing` đã về `False`.

Sau khi sửa, chạy `pytest -m ros -q test/local_post_inference_simulator/test_regression.py` cho kết quả thật (không còn skip do bug hạ tầng — có thể vẫn skip nếu ROS graph không start được trong môi trường cụ thể, ví dụ `ROS_LOG_DIR` không ghi được). Lần chạy đầu tiên lộ ra **4/6 fixture fail** — các gap này đã được điều tra và sửa từng case, kết quả tại thời điểm đó (`AVS_REQUIRE_LIVE_ROS=1 pytest -m ros -q test/local_post_inference_simulator/test_regression.py`) là **6/6 PASS**. Sau khi Plan D (D1) thêm 4 fixture live mới để lấp gap Phase 9, kết quả hiện tại là **10/10 PASS**:

| Fixture | Kết quả live | Ghi chú |
|---|---|---|
| `follow_main_straight.json` | ✅ PASS | Bao gồm `expected_control_source: direct_ipm` |
| `follow_main_curve.json` | ✅ PASS | |
| `follow_main_dropout.json` | ✅ PASS | Fail ban đầu (`max_replan_count` kỳ vọng ≤4, thực tế 10) là bug **metric harness**, không phải manager: `ros_scenario_runner.py` đếm mọi frame có `replan_reason != "none"` thay vì đếm số lần giá trị *đổi* so với frame trước (đúng theo định nghĩa metric trong roadmap và cùng pattern với `selected_lane_switch_count`/`trajectory_kind_switch_count`). `TrajectoryManager::update()` luôn gán một reason cụ thể ở mọi frame hợp lệ (không có nhánh nào set `"none"`), nên kiểu đếm cũ luôn ra ≈ tổng số frame. Sửa cách đếm, không đổi `control_node.cpp`. |
| `intersection_follow_main.json` | ✅ PASS | Fail ban đầu (`expected_selected_lane` kỳ vọng `main_lane_current`, thực tế chọn `main_lane_ahead`) là bug **geometry fixture**: `main_lane_ahead` ở quá gần `main_lane_current` theo world-Y, khiến scoring chọn nhầm lane phía trước. Đã nới khoảng cách world-Y giữa hai lane trong fixture. |
| `lane_change_solid_blocked.json` | ✅ PASS | Fail ban đầu (`expected_blocked_state`/`expected_trajectory_kind` kỳ vọng blocked, thực tế không vào blocked state) do 2 nguyên nhân: (1) geometry fixture cũ không tạo corridor thực tế giữa 2 lane cho `is_lane_change_blocked_by_solid` — đã thiết kế lại bằng nghịch đảo homography (`H⁻¹`) từ tọa độ world mong muốn; (2) bug production thật trong `control_node.cpp`: `plan_follow_main()` luôn gán `TrajectoryKind::FOLLOW_MAIN` bất kể state, nên nhánh BLOCKED không bao giờ báo cáo đúng `blocked_follow_main` — đã fix bằng relabel trong nhánh `DecisionState::BLOCKED` (control_node.cpp:~1906). |
| `turn_right_two_lanes.json` | ✅ PASS | Fail ban đầu (turn chưa commit, log "Transition safety check failed! Lateral distance: 166.1, Heading diff: 0.93 rad") là bug **geometry fixture**: 2 đỉnh polygon của `main_lane_1` (nay `trtl_main_lane_1`) nằm ở pixel row v=120, vắt qua "singular vanishing row" (v≈151 với `calibration.json` hiện tại) của phép biến đổi homography nghịch, khiến world-Y tính ra âm/phi vật lý và đầu độc gate `heading_diff`/`lat_dist` trong `plan_transition()`. Đã nudge v=120→200. Việc verify full-suite còn lộ thêm bug thứ hai (xem ngay dưới). |
| `turn_dropout_mid_maneuver.json` | ✅ PASS | Mới thêm (D1). Verify pass ngay lần chạy đầu — không phát hiện gap production. |
| `lane_id_swap.json` | ✅ PASS | Mới thêm (D1). Verify pass ngay lần chạy đầu. |
| `turn_left_two_lanes_live.json` | ✅ PASS | Mới thêm (D1), port live của `test/decision_system/fixtures/turn_left_two_lanes.json`. Verify pass ngay lần chạy đầu. |
| `lane_change_dashed_allowed_live.json` | ✅ PASS | Mới thêm (D1). Assertions được viết **sau khi** chạy scenario thật qua `ScenarioRunner` và log per-frame `selected_lane_id`/`trajectory_kind`/`control_source` (không đoán số) — xem chi tiết hành vi ở mục fixture bên trên. |

Ngoài các fixture-specific fix trên, verify full-suite còn lộ ra một bug hạ tầng test **xuyên fixture**: `ipm_transform_node.cpp` giữ state smoothing tạm thời (EMA) theo `track_id` (`track_states_`, sống suốt vòng đời process — trong pytest là suốt cả file test vì node module-scoped). Nhiều fixture dùng chung id object generic như `main_lane_1`/`main_lane_current`/`solid_white_1`, nên state smoothing của fixture chạy trước bị rò sang fixture chạy sau có cùng id, làm kết quả phụ thuộc thứ tự chạy test (điển hình: `turn_right_two_lanes.json` PASS khi chạy riêng nhưng FAIL khi chạy sau `lane_change_solid_blocked.json` trong cùng session). Đây là bug test-hygiene, không phải bug production (track_id thật do tracker sinh, không đụng hàng giữa các session không liên quan). Đã sửa bằng cách prefix mọi object id theo từng fixture để đảm bảo unique toàn cục: `fmc_*` (`follow_main_curve.json`), `fmd_*` (`follow_main_dropout.json`), `fms_*` (`follow_main_straight.json`), `ifm_*` (`intersection_follow_main.json`), `lcsb_*` (`lane_change_solid_blocked.json`), `trtl_*` (`turn_right_two_lanes.json`) — áp dụng cho cả `objects[].id` lẫn `expected_selected_lane` trong assertions.

---

## Cách Chạy Regression

### Offline (không cần ROS)

```bash
pytest -v -m "not ros" test/local_post_inference_simulator/
```

`test_assertions_evaluation_logic` (kiểm tra logic `evaluate_assertions` trên mock) đã tách sang `test_regression_mock.py` — không còn nằm trong `test_regression.py`, nên phải chạy trên cả thư mục chứ không chỉ file `test_regression.py`. Đặt `AVS_REQUIRE_LIVE_ROS=1` trước lệnh `pytest -m ros` (xem dưới) nếu muốn skip biến thành fail cứng thay vì skip âm thầm.

### Live (cần ROS running)

```bash
# Terminal 1: IPM node
source ros2_ws/install_user/setup.bash && export ROS_DOMAIN_ID=20
ros2 run avs_perception ipm_transform_node

# Terminal 2: Control node
source ros2_ws/install_user/setup.bash && export ROS_DOMAIN_ID=20
ros2 run avs_perception control_node

# Terminal 3: Backend simulator
export ROS_DOMAIN_ID=20
.venv/bin/python3 -m uvicorn tools.local_post_inference_simulator.backend.main:app \
    --host 0.0.0.0 --port 8001

# Terminal 4: Run full regression (AVS_REQUIRE_LIVE_ROS=1 turns a "not subscribing" skip
# into a hard fail instead of a silent skip — use it whenever you expect live mode to work)
export ROS_DOMAIN_ID=20
AVS_REQUIRE_LIVE_ROS=1 pytest -v -m ros test/local_post_inference_simulator/test_regression.py
```

### CLI runner cho từng fixture

```bash
export ROS_DOMAIN_ID=20
# Test Phase 6 dropout scenario
python tools/local_post_inference_simulator/backend/run_scenario.py \
    tools/local_post_inference_simulator/fixtures/follow_main_dropout.json \
    --output /tmp/report_dropout.json

# Test Phase 8 marking gate
python tools/local_post_inference_simulator/backend/run_scenario.py \
    tools/local_post_inference_simulator/fixtures/lane_change_solid_blocked.json
```

---

## Quy Tắc Mở Rộng

Khi thêm phase refactor mới vào `decision_trajectory_refactor_roadmap.md`, phải:

1. Tạo hoặc cập nhật fixture trong `fixtures/`.
2. Thêm `assertions` block với đủ field cần bảo vệ.
3. Cập nhật bảng mapping và coverage status trong file này.
4. Nếu cần assertion field mới (ví dụ `expected_control_source`), mở rộng `AssertionsSchema` và `evaluate_assertions` trong `scenario_schema.py`.
5. Xác nhận không còn fixture skip do thiếu assertions:

```bash
pytest -v test/local_post_inference_simulator/ | grep "SKIPPED"
# Tất cả SKIPPED phải là "Required ROS2 nodes are not subscribing"
# Không được có "no assertions" hoặc "scenario.assertions is None"
```
