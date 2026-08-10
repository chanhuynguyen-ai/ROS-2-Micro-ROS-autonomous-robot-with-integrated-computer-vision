# Roadmap Refactor Decision Và Trajectory Planning

Tài liệu này là kế hoạch refactor hệ thống chọn lane và lập trajectory từ hiện trạng runtime hiện tại sang kiến trúc đúng với:

- `./decision_sys.md`
- `./trajectory_planning_memory_proposal.md`
- `./decision_sys_implementation_plan.md`
- `./trajectory_planning_memory_implementation_plan.md`

Mục tiêu không phải viết lại toàn bộ ngay, mà là đưa hệ thống về đúng nguyên tắc:

- perception cung cấp `path observation`
- planner luôn sinh candidate trajectory mỗi frame
- normalizer chuẩn hóa candidate với trajectory đã commit
- manager quyết định hold/update/replan
- controller chỉ nhận một `active trajectory` duy nhất
- route intent chỉ là trigger nghiệp vụ, không bị state machine cũ làm lệch nghĩa

## Kết Luận Kỹ Thuật

Không nên chuyển sang deep learning hoặc machine learning để học hành vi lái xe ở giai đoạn này. Vấn đề hiện tại chủ yếu là kiến trúc runtime chưa thực thi đúng tài liệu, không phải do hướng rule-based/geometric sai.

Deep learning có thể dùng sau cho các bài toán hẹp như lane association, confidence estimation hoặc junction classification. Không nên dùng end-to-end behavior learning để thay planner khi hệ thống chưa có logging, dataset, metric và safety gate đủ mạnh.

## Nguyên Tắc Refactor

- Mỗi phase phải build được bằng `colcon build --symlink-install --packages-select avs_perception`.
- Mỗi phase phải chạy được `pytest -q test/decision_system`.
- Không đổi contract `/avs/control_error` nếu chưa có phase riêng cho controller downstream.
- Không để logic planning phụ thuộc trực tiếp vào callback ROS hoặc raw JSON nếu có thể tách được.
- Không publish nhiều trajectory song song xuống controller.
- Không dùng `stop-line` để kích hoạt rẽ, phát hiện giao lộ, phát hiện T-junction hoặc quyết định chuyển làn.
- Không để direct IPM lookahead âm thầm bypass `active trajectory` mà không có debug field rõ ràng.
- Không refactor lớn khi chưa có fixture hoặc replay test bảo vệ behavior tương ứng.

## Kiến Trúc Mục Tiêu

Runtime mỗi frame:

```text
/avs/telemetry_realworld
-> PathObservationBuilder
-> TrajectoryPlanner(plan_candidate_for_intent)
-> TrajectoryNormalizer
-> TrajectoryManager
-> ControlErrorProjector
-> /avs/control_error
```

Các khối chính:

- `PathObservationBuilder`: parse telemetry thành lane/marking observation đã chuẩn hóa.
- `TrajectoryPlanner`: sinh candidate trajectory theo route intent hiện tại.
- `TrajectoryNormalizer`: blend và kiểm soát hình học candidate với committed trajectory.
- `TrajectoryManager`: quyết định hold/update/replan/recovery/blocked.
- `ControlErrorProjector`: tính `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`, `curvature_inv_mm` từ active trajectory.

## Phase 0: Baseline Và Metric Trước Khi Refactor

### Mục tiêu

Đóng băng hiện trạng để biết refactor làm tốt hơn hay xấu hơn.

### Việc cần làm

- Chạy `pytest -q test/decision_system` và lưu kết quả.
- Build package bằng lệnh chuẩn.
- Thu ít nhất 3 log hoặc replay case:
  - follow main đường thẳng/đường cong
  - rẽ trái/phải hoặc T-junction
  - lane change blocked/allowed nếu có dữ liệu
- Ghi lại các metric runtime:
  - số lần `selected_lane_id` đổi trong mỗi đoạn
  - số lần `trajectory_kind` đổi
  - jitter của `epsilon_x_mm` và `theta_rad`
  - số frame `trajectory_valid=false`
  - số lần `replan_reason` thay đổi

### Lưu ý khi code/refactor

- Không sửa behavior ở phase này.
- Nếu cần thêm logging, chỉ thêm field debug không ảnh hưởng output điều khiển.
- Không xóa logic cũ trước khi có baseline.

### Lưu ý khi build/test

- Chạy test unit hiện có trước và sau khi thêm debug field.
- Nếu build fail do warning hoặc include, sửa tối thiểu, không refactor lan rộng.

### Tiêu chí hoàn thành

- Có baseline test và baseline runtime để so sánh.
- Biết rõ case nào đang sai: chọn lane sai, trajectory rung, replan sai, hay output controller bypass planner.

## Phase 1: Tách Runtime Flow Khỏi State Machine Cũ

### Mục tiêu

Loại mâu thuẫn lớn nhất: khi `state_ == FOLLOW_MAIN`, runtime hiện vẫn ép planner chỉ plan `follow_main` và bỏ qua pending route intent. Sau phase này, planner phải sinh candidate theo intent hiện tại ở mọi frame.

### Việc cần làm

- Tạo helper trung tâm:

```cpp
PlannedTrajectory plan_candidate_for_intent(
    const PathObservationFrame& obs,
    RouteIntent intent,
    const CommittedTrajectoryState& previous_state);
```

- Mapping intent:
  - `FOLLOW_MAIN` -> `plan_follow_main`
  - `TURN_RIGHT` -> `plan_turn_right`
  - `TURN_LEFT` -> `plan_turn_left`
  - `LANE_CHANGE_LEFT` -> `plan_lane_change_left`
  - `LANE_CHANGE_RIGHT` -> `plan_lane_change_right`
- State machine không được quyết định "có plan hay không". Nó chỉ cung cấp feasibility/blocked/completion signal cho manager.
- `TrajectoryManager` mới là nơi quyết định candidate có được commit không.

### Lưu ý khi code/refactor

- Không để nhánh `FOLLOW_MAIN` hard-code `current_intent_str = "follow_main"` nếu `current_intent_` đang là turn/lane change.
- Không kích hoạt maneuver bằng khoảng cách gần giao lộ trong planner. Khoảng cách chỉ nên là feasibility hoặc commit guard.
- Giữ fallback an toàn: nếu intent maneuver chưa đủ dữ liệu, manager có thể hold `follow_main`, nhưng candidate/decision debug phải nói rõ lý do.

### Lưu ý khi build/test

- Thêm unit test cho case: intent `turn_right` đã nhận, state vẫn `FOLLOW_MAIN`, planner vẫn sinh candidate turn nếu có turn-lane hợp lệ.
- Chạy `pytest -q test/decision_system`.
- Build `avs_perception` sau khi đổi flow callback.

### Tiêu chí hoàn thành

- Mọi frame đều đi qua cùng flow: build obs -> plan candidate -> normalize -> manager -> project control.
- Route intent không còn bị bỏ qua chỉ vì state hiện tại chưa chuyển maneuver.

## Phase 2: Chuẩn Hóa Intent, State Và Commit Policy

### Mục tiêu

Tách nghĩa của `route_intent`, `decision_state`, `trajectory_kind` và `manager_action`. Hiện các khái niệm này đang bị trộn, dẫn tới rẽ/chuyển làn dễ bị reset hoặc fallback sai.

### Việc cần làm

- Định nghĩa rõ:
  - `route_intent`: mục tiêu nghiệp vụ từ user/planner.
  - `decision_state`: trạng thái an toàn/khả dụng hiện tại.
  - `trajectory_kind`: loại trajectory đang commit.
  - `manager_action`: hành động với trajectory ở frame hiện tại.
- Thêm field debug:
  - `candidate_trajectory_kind`
  - `manager_action`
  - `candidate_valid`
  - `commit_allowed`
  - `hold_reason`
- Chỉ reset `current_intent_` về `FOLLOW_MAIN` khi maneuver thật sự hoàn tất, không reset chỉ vì tạm mất turn-lane một frame.

### Lưu ý khi code/refactor

- Không dùng `decision_state` để suy ngược route intent.
- Không để `TrajectoryManager` so sánh intent bằng string nếu đã có enum ổn định.
- Không reset maneuver khi perception dropout ngắn hạn; dùng hold window.

### Lưu ý khi build/test

- Test dropout trong lúc đang turn/lane change, không chỉ dropout follow-main.
- Test intent không hợp lệ không làm mất committed trajectory đang an toàn.

### Tiêu chí hoàn thành

- Debug cho biết rõ vì sao một candidate maneuver chưa được commit.
- Maneuver không bị rơi về `FOLLOW_MAIN` chỉ vì miss detection ngắn hạn.

## Phase 3: Refactor PathObservationBuilder Thành Nguồn Sự Thật

### Mục tiêu

Planner không đọc raw JSON rải rác. Tất cả lane/marking dùng cho decision phải đi qua observation model thống nhất.

### Việc cần làm

- Đưa các field cần thiết vào `LaneObservation`:
  - `lane_id`
  - `label`
  - `class_name`
  - `points`
  - `confidence`
  - `start_s/end_s` hoặc `nearest_y/farthest_y`
  - `lateral_ref_x`
  - `heading_start/heading_end`
  - `has_precomputed_control`
- Đưa marking vào `MarkingObservation` có polyline/polygon chuẩn.
- Không để các helper như chọn turn-lane, chọn other-lane đọc trực tiếp `raw_obj` nếu field đã có trong observation.

### Lưu ý khi code/refactor

- Giữ `raw_obj` tạm thời để tương thích, nhưng coi là fallback, không là data path chính.
- Không dùng magic label rải rác; gom label vào constant/enum.
- Cẩn thận label hiện tại trong code/model đang dùng `turn-lane = 17`, trong một số docs cũ có mapping khác.
- Cần xử lý mismatch đang tồn tại: `models/best_ncnn_model/metadata.yaml`, `ncnn_inference_node.cpp`, `control_node.cpp` và test/harness dùng `turn-lane = 17`, nhưng `ipm_transform_node.cpp` vẫn còn logic cũ kiểm tra `turn-lane = 10`. Nếu không sửa, IPM sẽ không coi object `turn-lane` thật là lane rẽ, dẫn tới không quét/fitting theo hướng turn-lane, không sinh đúng waypoint/centerline rẽ, và planner phía sau có thể không nhận được path rẽ hợp lệ.
- Phase này phải tạo một source-of-truth duy nhất cho label mapping, ví dụ `LabelId::TURN_LANE = 17`, rồi thay mọi magic number liên quan trong IPM/control/test bằng constant đó.

### Lưu ý khi build/test

- Thêm fixture có object thiếu `id`, có `track_id`, và object có ID `obj_*` để test stable ID logic.
- Test `waypoints` rỗng nhưng có precomputed control.

### Tiêu chí hoàn thành

- Planner có thể hoạt động chủ yếu trên `PathObservationFrame`.
- Sự khác biệt giữa C++ runtime và Python harness giảm xuống mức tối thiểu.

## Phase 4: Chuẩn Hóa Chọn Lane Theo Geometry Và Hysteresis

### Mục tiêu

Làm lane selection ổn định hơn trước nhiễu perception, ID đổi, lane bị đứt hoặc nhiều lane cùng label.

### Việc cần làm

- Refactor chọn `main_current`:
  - dùng score gồm lateral gần ego, start y, heading, overlap với committed trajectory
  - ưu tiên stable ID nếu còn hợp lý
  - không giữ sticky ID nếu lane bắt đầu quá xa hoặc lệch topology
- Refactor chọn `main_ahead`:
  - kiểm tra longitudinal gap
  - lateral jump
  - heading continuity
  - confidence và độ dài lane
- Refactor chọn `turn-lane`:
  - chọn theo side/hướng intent
  - right chọn gần hơn, left chọn xa hơn theo metric nhất quán
  - T-junction có rule riêng nhưng không phụ thuộc stop-line
- Refactor chọn `other-lane`:
  - dùng lateral relative với `main_current`
  - kiểm tra parallelism
  - kiểm tra overlap vùng gần xe

### Lưu ý khi code/refactor

- Mọi threshold phải gom thành parameter hoặc constant có tên rõ.
- Không chọn lane chỉ dựa trên average x nếu lane cong mạnh; dùng median hoặc đoạn gần xe.
- Không snap sang lane mới nếu committed trajectory hiện tại vẫn valid và deviation thấp.

### Lưu ý khi build/test

- Thêm fixture nhiều main-lane bị đứt đoạn qua giao lộ.
- Thêm fixture hai turn-lane cùng phía nhưng khoảng cách khác nhau.
- Thêm fixture lane ID đổi giữa hai frame.

### Tiêu chí hoàn thành

- `selected_lane_id` ít nhảy hơn trên cùng một đoạn đường.
- Turn/lane-change chọn đúng lane theo rule trong docs.

## Phase 5: Normalizer Theo Arc-Length Và Progress

### Mục tiêu

Thay blend theo index bằng normalization theo arc-length/progress thực sự. Đây là điểm chính để giảm rung trajectory.

### Việc cần làm

- Resample previous committed trajectory và current candidate theo `s`.
- Ước lượng `progress_s_mm` của xe trên committed trajectory.
- Cắt/align đoạn còn lại của previous trajectory với candidate mới.
- Tính metric:
  - lateral deviation
  - heading deviation
  - curvature deviation
  - overlap length
- Blend:
  - gần xe ưu tiên previous
  - xa xe ưu tiên current nếu confidence đủ cao
  - confidence thấp thì giữ previous lâu hơn
- Sau blend, kiểm tra:
  - heading continuity
  - curvature bound
  - minimum path length

### Lưu ý khi code/refactor

- Không giả định `i * 100mm` luôn đúng nếu path bị append/truncate khác nhau.
- Không append phần dư của previous path một cách mù quáng nếu candidate đã rẽ topology khác.
- Không blend hai trajectory khác `trajectory_kind` trừ khi manager đã quyết định đó là transition hợp lệ.

### Lưu ý khi build/test

- Test hai frame giống nhau lệch nhẹ 20-50 mm: output phải mượt, không snap.
- Test candidate khác topology rõ rệt: manager phải commit/replan hoặc hold, không blend thành đường lai sai.
- Test dropout: normalized trajectory không mất ngay trong hold window.

### Tiêu chí hoàn thành

- Jitter `epsilon_x_mm` và `theta_rad` giảm trên replay.
- Active trajectory vẫn theo kịp đường cong thật, không bị lag quá mức.

## Phase 6: Manager Replan Policy Chuẩn

### Mục tiêu

Manager quyết định ổn định và giải thích được: hold, update, commit, blocked, recovery.

### Việc cần làm

- Đổi `calculate_path_deviation` từ chỉ `abs(x)` sang metric tổng hợp:
  - lateral RMS
  - heading RMS
  - curvature max/RMS
  - overlap ratio
  - target lane/topology change
- Replan chỉ khi:
  - intent seq mới
  - maneuver hoàn tất
  - current trajectory invalid thật sự
  - blocked bởi rule
  - topology mismatch vượt ngưỡng
- Hold khi:
  - dropout ngắn hạn
  - candidate mới chỉ khác nhẹ
  - maneuver candidate tạm mất nhưng committed maneuver vẫn còn hợp lệ
- Recovery khi:
  - dropout quá hold window
  - active trajectory quá ngắn
  - geometry vượt curvature/heading limit an toàn

### Lưu ý khi code/refactor

- Không so sánh intent bằng text; dùng enum và `seq` nếu route intent có.
- Không commit candidate chỉ vì deviation lớn nếu candidate confidence thấp.
- Không giữ stale path vô hạn; hold window nên theo frame và/hoặc thời gian/quãng đường.

### Lưu ý khi build/test

- Test candidate confidence thấp nhưng deviation lớn.
- Test intent mới cùng loại nhưng seq mới.
- Test persistent dropout sau hold window phải vào recovery.

### Tiêu chí hoàn thành

- Mọi lần đổi trajectory đều có `replan_reason` rõ.
- Không đổi path liên tục theo nhiễu frame.

## Phase 7: ControlErrorProjector Và Loại Bypass Không Kiểm Soát

### Mục tiêu

Đảm bảo output controller thật sự đến từ `active trajectory` đã commit, hoặc nếu dùng direct IPM fallback thì phải được kiểm soát và debug rõ.

### Việc cần làm

- Tách `publish_control_error_from_trajectory` thành `ControlErrorProjector`.
- Projector chỉ nhận `CommittedTrajectoryState` hoặc `ActiveTrajectory`.
- Nếu giữ direct IPM policy:
  - thêm `control_source`
  - chỉ dùng khi trajectory kind là `FOLLOW_MAIN`
  - không dùng khi có intent maneuver pending hoặc active
  - phải kiểm tra candidate/committed trajectory match direct IPM bằng metric rõ
- Ưu tiên về lâu dài: direct IPM là observation/control hint, không bypass planner.

### Lưu ý khi code/refactor

- Không để `/avs/control_error` publish từ hai nguồn khác nhau trong cùng callback.
- Không tính `theta_rad` chỉ từ vector origin -> lookahead nếu trajectory cong; nên dùng tiếp tuyến tại lookahead.
- Kiểm tra đơn vị `curvature_inv_mm`: tên hiện tại dễ gây nhầm giữa curvature và inverse curvature.

### Lưu ý khi build/test

- Test debug `control_source` trên follow-main và turn.
- Test turn/lane-change không bị direct IPM override.

### Tiêu chí hoàn thành

- Có đúng một nguồn xuất control error ở mỗi frame.
- Dashboard/log nhìn được controller đang bám active trajectory hay fallback đặc biệt.

## Phase 8: Marking Gate Và Blocked Behavior

### Mục tiêu

Hoàn thiện rule chuyển làn/rẽ trái bị chặn bởi solid marking theo tài liệu.

### Việc cần làm

- Chuẩn hóa marking labels:
  - solid: `solid-white`, `solid-yellow`, `double-solid-white`
  - dashed: `dashed-white`, `dashed-yellow`
- `is_lane_change_blocked_by_solid` làm việc trên corridor giữa current lane và target lane.
- Blocked behavior:
  - không sinh line chuyển làn/rẽ trái
  - commit/fallback về follow-main
  - publish `decision_state=BLOCKED`
  - publish `blocked_by_marking=true`
  - giữ route intent hoặc clear intent theo chính sách rõ ràng

### Lưu ý khi code/refactor

- Không coi không thấy dashed là được phép nếu có solid confidence cao trong corridor.
- Không block chỉ vì solid ở ngoài corridor.
- Không dùng stop-line trong logic này.

### Lưu ý khi build/test

- Test solid nằm giữa lane.
- Test solid nằm ngoài corridor.
- Test dashed giữa lane.
- Test không thấy marking: behavior phải theo policy đã chọn và debug rõ confidence thấp.

### Tiêu chí hoàn thành

- Lane change blocked/allowed đúng fixture.
- Blocked không làm controller nhận trajectory chuyển làn sai.

## Phase 9: Replay Test Và Scenario Regression

### Mục tiêu

Đưa validation từ unit fixture tĩnh sang chuỗi frame gần với runtime thật.

### Việc cần làm

- Tạo replay runner đọc JSON lines hoặc bag-export telemetry.
- Mỗi frame chạy:
  - observation
  - planner
  - normalizer
  - manager
  - projector
- Xuất report:
  - jitter stats
  - selected lane switches
  - replan count
  - invalid frame count
  - control source count
  - blocked/recovery events
- Thêm scenario regression:
  - follow main straight
  - follow main curve
  - intersection follow-main
  - right turn two lanes
  - left turn two lanes
  - lane change solid blocked
  - lane change dashed allowed
  - dropout during maneuver
  - ID swap/recovery

### Lưu ý khi code/refactor

- Không chỉ assert một frame cuối; assert chuỗi hành vi.
- Test phải kiểm tra không có lane switch/replan bất thường.
- Fixture nên nhỏ, dễ đọc, nhưng đủ nhiều frame để bắt lỗi memory.

### Lưu ý khi build/test

- Chạy `pytest -q test/decision_system` trong mỗi phase.
- Nếu thêm test replay nặng, tách marker để có thể chạy quick test và full test.

### Tiêu chí hoàn thành

- Có regression bắt được các lỗi từng thấy trên robot/video.
- Có metric định lượng trước/sau refactor.

## Phase 10: Tách File Và Dọn Nợ Kỹ Thuật

### Mục tiêu

Sau khi behavior ổn, tách `control_node.cpp` thành module dễ maintain.

### Việc cần làm

- Tách file theo trách nhiệm:
  - `path_observation.hpp/cpp`
  - `trajectory_planner.hpp/cpp`
  - `trajectory_normalizer.hpp/cpp`
  - `trajectory_manager.hpp/cpp`
  - `control_error_projector.hpp/cpp`
  - `decision_types.hpp`
- `control_node.cpp` chỉ còn:
  - ROS subscription/publish
  - parameter loading
  - gọi pipeline
- Đồng bộ Python harness hoặc thay bằng test C++ nếu phù hợp.

### Lưu ý khi code/refactor

- Không tách file trước khi behavior được test bảo vệ; nếu không sẽ khó biết lỗi do move code hay do logic.
- Giữ API nhỏ, dùng struct rõ ràng, tránh truyền raw JSON qua nhiều lớp.
- Nếu dùng C++ header, tránh include `json.hpp` lan rộng nếu không cần.

### Lưu ý khi build/test

- Cập nhật `CMakeLists.txt` theo từng file mới.
- Build sau mỗi lần tách module, không tách hàng loạt.
- Chạy unit/replay test sau mỗi module move.

### Tiêu chí hoàn thành

- `control_node.cpp` ngắn và chỉ làm orchestration ROS.
- Core decision/planning có thể test offline không cần ROS node.

## Thứ Tự Ưu Tiên Nếu Muốn Làm Nhanh

Nếu cần cải thiện nhanh trước khi refactor đầy đủ, làm theo thứ tự:

1. Phase 1: không bỏ qua route intent khi state đang `FOLLOW_MAIN`.
2. Phase 7: debug hoặc hạn chế direct IPM bypass.
3. Phase 6: sửa manager deviation/replan policy.
4. Phase 5: normalizer theo arc-length.
5. Phase 9: replay regression.

Không nên bắt đầu bằng Phase 10, vì tách file khi logic còn sai chỉ làm lỗi khó truy vết hơn.

## Checklist Sau Mỗi Phase

- `pytest -q test/decision_system` pass.
- `colcon build --symlink-install --packages-select avs_perception` pass.
- `/avs/lane_state` có đủ debug để giải thích quyết định.
- `/avs/control_error` vẫn giữ schema downstream.
- Không có thêm nguồn publish trajectory/control error song song.
- Không có logic mới dùng `stop-line` cho decision.
- Không có reset intent/state không giải thích được trong log.

## Rủi Ro Chính

- Threshold geometry hiện tại có thể đang được tune theo dữ liệu cụ thể; refactor cần replay để tránh regression.
- Mapping label giữa docs cũ và telemetry thực tế cần xác nhận trước khi gom constant.
- Mismatch `turn-lane = 10` trong IPM so với `turn-lane = 17` trong model/control là rủi ro cao: rẽ trái/phải có thể fail ngay từ tầng world/centerline extraction dù decision planner phía sau đúng.
- Nếu perception không cung cấp stable ID, lane association/hysteresis phải dựa nhiều hơn vào geometry.
- Nếu không có log chuỗi frame thực, rất khó chứng minh normalizer/manager tốt hơn unit fixture.

## Khi Nào Mới Nên Dùng ML/DL Bổ Trợ

Chỉ cân nhắc sau khi pipeline trên có logging và metric ổn định.

Ứng dụng ML/DL hợp lý:

- lane association qua frame
- confidence estimation cho lane/path
- classifier nhận diện T-junction/intersection topology
- model đề xuất candidate maneuver cho planner kiểm duyệt

Không khuyến nghị:

- end-to-end steering từ ảnh
- học trực tiếp quyết định chuyển làn/rẽ mà không có rule safety gate
- thay manager bằng model black-box
