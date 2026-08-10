# Plan B: Bảo Đảm Đúng Luật Chọn Lane Theo `decision_sys.md`

Tương ứng roadmap refactor **Phase 4 + Phase 8**. Mục tiêu: mọi rule nghiệp vụ trong `../architecture/decision_sys.md` được thực thi đúng và có test chứng minh, đặc biệt các rule quyết định "rẽ đúng làn, chuyển làn đúng luật".

Phụ thuộc: Plan A xong trước (nếu intent còn bị nuốt thì test rule rẽ/chuyển làn không có ý nghĩa end-to-end).

## 1. Luật Cần Bảo Đảm (trích `decision_sys.md`)

| # | Rule | Trạng thái code hiện tại |
|---|---|---|
| R1 | `turn_right`: 2 turn-lane hợp lệ → chọn lane **gần hơn** | Có `select_turn_lane_obs` (control_node.cpp:859) — cần audit metric gần/xa |
| R2 | `turn_left`: 2 turn-lane hợp lệ → chọn lane **xa hơn** | Như trên |
| R3 | T-junction phát hiện bằng hình học, không dùng stop-line | Có `detect_t_junction` (:1714) — cần audit điều kiện |
| R4 | T-junction `turn_left` chỉ cho phép khi không có solid marking chặn | Chỉ check `is_turn_blocked_by_solid` khi `is_t` trong `update_lane_state` (:3391) — cần xác nhận đúng phạm vi và có test |
| R5 | Lane change bị chặn bởi `solid-white(13)`, `solid-yellow(14)`, `double-solid-white(2)` trong corridor giữa 2 lane | Có `is_lane_change_blocked_by_solid_obs` (:757) — cần audit corridor |
| R6 | Lane change được phép với `dashed-white(0)`, `dashed-yellow(1)` | Có fixture `lane_change_dashed_allowed.json` |
| R7 | Solid NGOÀI corridor không được block | Chưa có test |
| R8 | Blocked → `decision_state=BLOCKED`, `blocked_by_marking=true`, `trajectory_kind=blocked_follow_main`, active trajectory duy nhất theo main | Có relabel `BLOCKED_FOLLOW_MAIN` (đã fix trong live regression) |
| R9 | Follow main qua ngã tư: nối main hiện tại → main phía trước bằng MỘT line smooth; không nối xa khi chưa thấy main phía trước | Có fixture `follow_main_intersection.json` |
| R10 | `lane_change_left/right`: chọn `other-lane` theo lateral so với main; nhiều other-lane → chọn lane gần main nhất | Có `select_other_lane_obs` (:671) — cần audit |

Lưu ý label: dùng constant hiện có; mapping runtime là `main-lane=3`, `other-lane=4`, `turn-lane=17`, `dashed-white=0`, `dashed-yellow=1`, `double-solid-white=2`, `solid-white=13`, `solid-yellow=14`, `stop-line=16`. (Bảng label trong `decision_sys_implementation_plan.md` là mapping cũ — KHÔNG dùng.)

## 2. Bước 1: Audit (không đổi code)

Deliverable: bảng audit ngắn trong PR description hoặc file `plan_B_audit_notes.md`, mỗi rule R1–R10 ghi: hàm thực thi, metric/threshold dùng, kết luận ĐÚNG/SAI/THIẾU so với `decision_sys.md`, kèm test hiện có cover hay không.

Điểm audit trọng tâm:

- **Metric gần/xa của turn-lane** (R1/R2): roadmap Phase 4 yêu cầu không dùng average-x cho lane cong mạnh; phải dùng median hoặc đoạn gần xe; kiểm tra `select_turn_lane_obs` và `select_turn_lane` (2 bản — C++ observation-based và candidate-based) có nhất quán không.
- **`detect_t_junction`** (R3): đủ 3 điều kiện hình học (main phía trước không tiếp tục hợp lệ, có turn-lane ngang/chéo, không có main đối diện)? Có lẫn stop-line không?
- **Corridor marking gate** (R5/R7): vùng xét là corridor giữa 2 centerline vùng gần xe, hay chỉ check lateral thô? Solid ngoài corridor có bị tính nhầm?
- **Hai bản logic trùng nhau**: nhiều helper tồn tại cả bản `*_obs` (LaneObservation) lẫn bản LaneCandidate cũ (vd `select_other_lane` :671 vs dùng trong `update_lane_state`). Audit phải liệt kê cặp trùng và xác nhận không lệch behavior — đây là nguồn bug "planner chọn lane A, state machine đánh giá lane B".

## 3. Bước 2: Sửa Theo Kết Quả Audit (cần user duyệt danh sách sửa)

Chỉ sửa những gì audit chứng minh sai/thiếu. Dự kiến các nhóm sửa (sẽ chốt lại sau audit):

- Thống nhất một metric gần/xa cho turn-lane (khoảng cách dọc theo hướng tiến từ ego tới điểm vào lane, không phải average x), gom threshold thành parameter.
- Hợp nhất hoặc đồng bộ cặp helper trùng (`*_obs` vs candidate-based); hướng ưu tiên: bản `*_obs` là source-of-truth, bản cũ gọi lại bản mới.
- Corridor check: xây corridor từ 2 centerline (main → target) trong vùng `[0, corridor_check_y_mm]`, marking solid phải giao corridor mới block.
- Chính sách "không thấy marking": giữ mặc định hiện tại (không coi là block) nhưng debug `marking_confidence_low=true` — đúng ghi chú rủi ro trong implementation plan.

## 4. Thiết Kế Unit Test

Vị trí: `test/decision_system/test_plan_b_lane_rules.py` + fixture mới trong `test/decision_system/fixtures/`.

| Test | Fixture | Assertion |
|---|---|---|
| `test_turn_right_two_lanes_picks_nearer` | `turn_right_two_lanes.json` (đã có) + fixture mới `turn_right_two_lanes_same_side.json` (2 turn-lane CÙNG phía phải, cách nhau ≥ 800mm) | `selected_lane_id` là lane gần hơn; ổn định qua ≥ 5 frame (không nhảy giữa 2 lane) |
| `test_turn_left_two_lanes_picks_farther` | tương tự cho trái | lane xa hơn được chọn, ổn định |
| `test_turn_lane_selection_stable_under_jitter` | fixture 2 turn-lane với nhiễu vị trí ±30mm mỗi frame | `selected_lane_id` không đổi trong cả chuỗi |
| `test_t_junction_detected_without_stopline` | `t_junction_no_stopline.json` (mới): main kết thúc, 2 turn-lane ngang, KHÔNG có stop-line object | `is_t` (qua debug field) = true; intent `turn_right` chọn lane gần |
| `test_t_junction_not_triggered_by_stopline_alone` | fixture main tiếp tục bình thường + có stop-line | không phát hiện T-junction, không đổi behavior |
| `test_t_junction_left_blocked_by_solid` | T-junction + solid marking chắn hướng rẽ trái, intent `turn_left` | `decision_state == BLOCKED`, không sinh trajectory rẽ trái, active trajectory theo main |
| `test_lane_change_blocked_solid_in_corridor` | `lane_change_solid_blocked.json` (đã có) | giữ pass (regression) |
| `test_lane_change_allowed_solid_outside_corridor` | `lane_change_solid_outside_corridor.json` (mới): solid song song nhưng ở phía đối diện của target lane | lane change ĐƯỢC phép, `blocked_by_marking == false` |
| `test_lane_change_allowed_dashed` | `lane_change_dashed_allowed.json` (đã có) | giữ pass |
| `test_lane_change_no_marking_policy` | fixture không có marking nào giữa 2 lane | allowed theo policy + debug confidence-low được set |
| `test_other_lane_nearest_lateral_picked` | fixture 2 other-lane cùng bên trái | chọn lane gần main nhất theo lateral |
| `test_intersection_no_far_connect_without_main_ahead` | fixture chỉ có main hiện tại đứt ở xa | trajectory chỉ theo main hiện tại, không có đoạn nối "đoán" |
| `test_helper_pair_consistency` | chạy cùng input qua bản `*_obs` và bản candidate (nếu sau audit vẫn còn 2 bản) | hai bản chọn cùng lane |

Mỗi fixture mới phải được thêm vào mapping `../local_post_inference_simulator/scenario_refactor_mapping.md` nếu đồng thời dùng cho live regression.

## 5. Điều Kiện Ràng Buộc Hoàn Thành

- [x] Bảng audit R1–R10 hoàn chỉnh trong `plan_B_audit_notes.md`; kết luận không có rule nào sai/thiếu nên không phát sinh danh sách sửa `control_node.cpp` cần duyệt (chờ user xác nhận đã đọc/đồng ý audit).
- [x] Toàn bộ test bảng §4 pass (`test/decision_system/test_plan_b_lane_rules.py` + fixtures liên quan trong `test/decision_system/` và `tools/local_post_inference_simulator/fixtures/`); test cũ pass nguyên trạng — `pytest -q test/decision_system` 76/76.
- [x] Gate chung README pass: `colcon build` clean, `pytest -q test/decision_system` 76/76, `AVS_REQUIRE_LIVE_ROS=1 pytest -m ros test/local_post_inference_simulator/` 12/12.
- [x] Không còn cặp helper trùng thiếu test consistency — `select_turn_lane_obs`/`select_turn_lane` và `select_other_lane_obs`/`select_other_lane` vẫn tồn tại song song nhưng bị khóa bằng `test_turn_lane_helper_pair_thresholds_match`/`test_other_lane_helper_pair_thresholds_match` (nay assert cả threshold side-gate lẫn công thức `min_dist` near/far).
- [x] Mọi threshold hình học dùng trong R1–R10 là hằng số đã có sẵn trong code từ trước, không có magic number mới do Plan B thêm vào.
- [x] Không rule nào tham chiếu `stop-line` trong decision path — xác nhận qua `test_t_junction_detection_never_references_stop_line` + `grep STOP_LINE\|stop_line`.

## 6. Rủi Ro

- Fixture tổng hợp có thể không tái hiện đúng hình học perception thật (polygon méo, waypoint thưa) → sau khi pass fixture tĩnh, chạy lại các video test có sẵn qua simulator để đối chiếu bằng mắt overlay.
- Sửa metric gần/xa có thể đổi lane được chọn trên fixture cũ → nếu fixture cũ sai theo luật thì sửa fixture (giải thích và xin duyệt), không "nắn" code cho khớp fixture sai.
