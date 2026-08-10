# Implementation Plan: Trajectory Planning Memory

Tài liệu này chuyển nội dung trong `./trajectory_planning_memory_proposal.md` thành kế hoạch triển khai cụ thể theo từng phase. Mục tiêu là biến decision layer hiện tại từ mô hình `line theo frame` sang mô hình `plan every frame + normalize + commit + controlled replan`.

Sau mỗi phase nên dừng lại review trước khi đi tiếp để tránh gộp quá nhiều thay đổi khó kiểm soát vào một lần.

## Mục tiêu triển khai

- Ở mọi frame đều có bước `trajectory planning`, kể cả khi xe chỉ đang `follow_main` trên đường thẳng hoặc đường cong.
- `line` từ perception chỉ là `path observation`, không đi thẳng xuống controller.
- Mỗi frame đều có bước `curve normalization` để giảm nhiễu trước khi xuất `active trajectory`.
- Hệ thống luôn giữ đúng một `active trajectory` cho controller bám.
- Mặc định xe `follow_main`; chỉ `replan` khi có trigger hợp lệ như user intent, biển báo/rule, blocked, hoặc trajectory mất hiệu lực.
- Luật chọn lane vẫn phải bám đúng `./decision_sys.md`.

## Nguyên tắc chung

- Mỗi phase phải giữ `control_node` build được và chạy được.
- Mỗi phase phải giữ contract output `/avs/control_error` hiện tại để không phải đổi controller downstream.
- Ưu tiên tách phần hình học và quản lý trajectory ra khỏi callback ROS để có thể test offline.
- Không để planner logic nằm rải rác trong callback xử lý telemetry; gom thành helper/class rõ ràng.
- Chỉ publish một `active trajectory` duy nhất ở mỗi frame.

## Kiến trúc code mục tiêu

Nên tách thành các khối logic sau:

- `PathObservationBuilder`
  - trích lane/marking từ telemetry thành dữ liệu planning
- `TrajectoryPlanner`
  - sinh `candidate trajectory` theo intent hiện tại
- `TrajectoryNormalizer`
  - chuẩn hóa candidate hiện tại với trajectory đã chuẩn hóa của frame trước
- `TrajectoryManager`
  - giữ trajectory đã commit, quyết định hold/update/replan
- `ControlErrorProjector`
  - chuyển `active trajectory` thành `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`, `curvature_inv_mm`

Nếu chưa muốn tạo nhiều file ngay, vẫn nên tách bằng class/helper trong cùng module trước, sau đó mới refactor tiếp.

## Phase 1: Chuẩn hóa Data Model và Runtime State

### Mục tiêu

- Tạo data model đủ cho planning có bộ nhớ.
- Chưa thay sâu hành vi runtime, nhưng chuẩn bị đầy đủ struct/state để các phase sau cắm vào.

### Thay đổi chính

- Thêm các enum/struct nội bộ:
  - `RouteIntent`: `FOLLOW_MAIN`, `TURN_RIGHT`, `TURN_LEFT`, `LANE_CHANGE_LEFT`, `LANE_CHANGE_RIGHT`
  - `DecisionState`: `FOLLOW_MAIN`, `TURN_RIGHT`, `TURN_LEFT`, `LANE_CHANGE`, `BLOCKED`, `RECOVERY`
  - `TrajectoryKind`: `FOLLOW_MAIN`, `TURN_RIGHT`, `TURN_LEFT`, `LANE_CHANGE_LEFT`, `LANE_CHANGE_RIGHT`, `BLOCKED_FOLLOW_MAIN`
  - `Point2D`: `x_mm`, `y_mm`
  - `LaneObservation`: `lane_id`, `class_name`, `points`, `confidence`, `heading_hint`, `curvature_hint`
  - `MarkingObservation`: `marking_id`, `class_name`, `points`, `confidence`
  - `PathObservationFrame`: tập lane/marking đã parse từ telemetry frame hiện tại
  - `PlannedTrajectory`: points, source lane ids, target lane id, trajectory kind, confidence, valid
  - `CommittedTrajectoryState`: trajectory đang dùng, `progress_s_mm`, `remaining_s_mm`, `last_good_frame`, `replan_reason`
- Giữ subscriber `/avs/route_intent` như plan trong `./decision_sys_implementation_plan.md`.
- Mở rộng `/avs/lane_state` để phản ánh:
  - `route_intent`
  - `decision_state`
  - `trajectory_kind`
  - `committed_trajectory_id`
  - `replan_reason`

### Tiêu chí hoàn thành

- Build thành công.
- Runtime state đủ để lưu trajectory đã chuẩn hóa giữa các frame.
- Chưa đổi output điều khiển ngoài mong muốn.

### Review checkpoint

- Review tên và vai trò từng struct.
- Review dữ liệu nào là observation, dữ liệu nào là committed state.

## Phase 2: Path Observation Builder

### Mục tiêu

- Tách hẳn bước đọc telemetry thành `path observation` chuẩn cho planner.
- Không để planner phải đọc trực tiếp JSON/raw object phân đoạn.

### Thay đổi chính

- Viết helper/class `PathObservationBuilder`:
  - parse telemetry
  - gom `main-lane`, `other-lane`, `turn-lane`, `marking`
  - chuyển mỗi object thành polyline/waypoint trong vehicle frame
- Chuẩn hóa dữ liệu đầu vào:
  - resample polyline nếu cần
  - loại điểm trùng hoặc quá sát
  - đảm bảo thứ tự điểm theo hướng tiến dọc trục `Y`
- Tính metadata nhẹ phục vụ planning:
  - `nearest_y_mm`
  - `farthest_y_mm`
  - lateral representative
  - heading đầu/cuối
  - confidence cơ bản theo số điểm/chiều dài lane

### Tiêu chí hoàn thành

- Có thể build được `PathObservationFrame` nhất quán từ một telemetry frame.
- Planner và normalizer ở phase sau không cần biết chi tiết raw telemetry schema.

### Review checkpoint

- Review quy tắc sắp xếp waypoint theo hướng tiến.
- Review tiêu chí confidence cơ bản của lane observation.

## Phase 3: Follow Main Planning Ở Mọi Frame

### Mục tiêu

- Triển khai đúng tinh thần mới: ngay cả khi không có turn/lane change, planner vẫn phải sinh `FOLLOW_MAIN_PLAN` mỗi frame.
- `follow_main` trở thành maneuver mặc định có planning đầy đủ.

### Thay đổi chính

- Viết `TrajectoryPlanner::plan_follow_main(observation_frame, previous_state)`.
- Logic tối thiểu:
  - chọn `main_current`
  - nếu có `main_ahead` hợp lệ thì nối topology để kéo dài path
  - nếu chỉ có `main_current` thì vẫn sinh path theo `main_current`
  - nếu perception yếu nhưng trajectory cũ còn hợp lệ thì planner vẫn trả về candidate có thể normalize với path cũ
- Tách logic chọn `main_current` và `main_ahead` thành helper riêng.
- Sample output candidate theo bước cố định, ví dụ 50-100 mm.

### Tiêu chí hoàn thành

- Ở mọi frame có `main-lane`, planner luôn trả ra được một candidate `FOLLOW_MAIN`.
- Đường thẳng và đường cong bình thường không còn đi theo flow “đọc lane trực tiếp”.
- Mỗi frame bắt đầu có khái niệm `candidate trajectory` riêng với output controller.

### Review checkpoint

- Review tiêu chí chọn `main_current` và `main_ahead`.
- Review chiều dài tối thiểu của candidate path.

## Phase 4: Trajectory Normalization Mỗi Frame

### Mục tiêu

- Thêm lớp chuẩn hóa đường cong/path giữa frame hiện tại và trajectory đã chuẩn hóa của frame trước.
- Giảm rung trước khi output xuống controller.

### Thay đổi chính

- Viết class/helper `TrajectoryNormalizer`.
- Đầu vào:
  - `current_raw_candidate`
  - `previous_normalized_trajectory`
  - confidence của observation hiện tại
- Thuật toán tối thiểu nên gồm:
  - resample hai path theo cùng chuẩn arc-length
  - tìm vùng chồng lắp gần xe
  - tính sai lệch lateral / heading / curvature
  - blend mềm giữa path cũ và path mới
  - kiểm tra lại continuity hình học sau blend
- Tạo output:
  - `normalized_candidate`
  - `normalization_metrics`
  - `normalization_mode`

### Gợi ý triển khai phiên bản đầu

- Phiên bản đầu không cần tối ưu quá sâu.
- Có thể dùng:
  - weight gần xe ưu tiên `previous`
  - weight xa xe ưu tiên `current`
  - nếu confidence hiện tại thấp thì tăng trọng số `previous`
- Sau blend, fit lại đoạn nối bằng cubic Hermite nếu cần để tránh gãy tiếp tuyến.

### Tiêu chí hoàn thành

- Khi lane rung nhẹ giữa các frame, `active trajectory` giảm giật rõ rệt.
- Khi perception ổn định, normalized path vẫn bám hình học mới đủ nhanh.
- Khi chỉ thiếu lane ngắn hạn, normalized path không bị đứt ngay.

### Review checkpoint

- Review công thức blend.
- Review cách đánh giá curvature/heading continuity.
- Review tham số resample step và vùng overlap.

## Phase 5: Trajectory Manager và Chính Sách Commit/Hold/Replan

### Mục tiêu

- Bổ sung bộ nhớ runtime thực sự cho trajectory.
- Tách rõ ba hành vi: giữ path cũ, cập nhật mềm path cũ, hoặc replan sang path mới.

### Thay đổi chính

- Viết `TrajectoryManager`.
- Manager nhận:
  - intent hiện tại
  - normalized candidate frame hiện tại
  - committed trajectory state frame trước
  - trigger nghiệp vụ mới nếu có
- Manager phải quyết định một trong các action:
  - `HOLD_CURRENT`
  - `UPDATE_CURRENT`
  - `COMMIT_NEW`
  - `ENTER_BLOCKED`
  - `ENTER_RECOVERY`
- Thêm các guard:
  - không replan chỉ vì candidate mới “đẹp hơn chút ít”
  - không bỏ trajectory cũ nếu chỉ dropout ngắn hạn
  - chỉ replan khi có trigger hợp lệ hoặc trajectory cũ mất hiệu lực
- Theo dõi:
  - `progress_s_mm`
  - `remaining_s_mm`
  - `last_good_observation_frame`
  - `dropout_hold_counter`

### Tiêu chí hoàn thành

- Hệ thống giữ được trajectory ổn định qua nhiều frame.
- Không đổi lane/path liên tục theo nhiễu perception.
- Có log/debug rõ mỗi lần manager quyết định replan.

### Review checkpoint

- Review điều kiện `commit new`.
- Review hold window theo frame hoặc theo quãng đường.
- Review điều kiện “trajectory mất hiệu lực”.

## Phase 6: Tích hợp `follow_main` Mặc Định Với Manager

### Mục tiêu

- Hoàn tất flow mặc định: khi không có trigger mới, xe luôn tiếp tục `follow_main` nhưng vẫn có planning, normalization và memory.

### Thay đổi chính

- Runtime flow mỗi frame:
  1. build observation
  2. plan candidate theo intent hiện tại
  3. nếu không có intent mới thì candidate mặc định là `FOLLOW_MAIN`
  4. normalize candidate với trajectory cũ
  5. manager chọn trajectory cuối cùng
  6. publish đúng một `active trajectory`
- Khi không có trigger:
  - giữ `route_intent = FOLLOW_MAIN`
  - không đổi sang maneuver khác
  - chỉ cập nhật mềm đường dẫn main theo observation mới

### Tiêu chí hoàn thành

- Đường thẳng, đường cong và dropout ngắn hạn đều chạy qua cùng một flow planning thống nhất.
- Không còn nhánh logic “case đơn giản thì khỏi plan”.

### Review checkpoint

- Review fallback khi observation quá yếu nhưng chưa nên replan.
- Review output debug để chắc có thể thấy manager đang hold hay update.

## Phase 7: Turn Planning Theo Rule Lane Selection

### Mục tiêu

- Thêm `turn_right` và `turn_left` lên kiến trúc mới.
- Replan chỉ xảy ra khi có `route_intent` hoặc trigger hợp lệ.

### Thay đổi chính

- Triển khai trong `TrajectoryPlanner`:
  - `plan_turn_right`
  - `plan_turn_left`
- Bám đúng rule từ `./decision_sys.md`:
  - `turn_right`: nếu có 2 lane hợp lệ, chọn lane gần hơn
  - `turn_left`: nếu có 2 lane hợp lệ, chọn lane xa hơn
- Tạo candidate trajectory rẽ từ lane hiện tại sang `turn-lane` mục tiêu.
- Candidate này sau đó vẫn đi qua `TrajectoryNormalizer` và `TrajectoryManager` giống hệt `follow_main`.

### Tiêu chí hoàn thành

- Khi có intent rẽ, manager commit trajectory rẽ mới thay vì bám main.
- Sau khi commit, path rẽ vẫn được chuẩn hóa mỗi frame như follow main.

### Review checkpoint

- Review metric gần/xa để chọn `turn-lane`.
- Review điều kiện đủ tin cậy để commit turn trajectory.

## Phase 8: Lane Change Planning Và Marking Gate

### Mục tiêu

- Đưa `lane_change_left/right` vào cùng cơ chế planning-memory-normalization.
- Chỉ cho phép commit lane change khi hợp lệ theo marking rule.

### Thay đổi chính

- Triển khai:
  - `plan_lane_change_left`
  - `plan_lane_change_right`
- Chọn `other-lane` mục tiêu theo lateral.
- Viết helper `is_lane_change_blocked_by_solid(...)`.
- Nếu bị block:
  - không commit lane change
  - manager giữ hoặc quay về `FOLLOW_MAIN`
  - debug ghi rõ `blocked_by_marking = true`
- Nếu hợp lệ:
  - sinh candidate chuyển làn
  - normalizer làm mượt
  - manager commit trajectory mới

### Tiêu chí hoàn thành

- Lane change không bị commit khi có solid marking.
- Lane change được commit khi dashed hoặc không có solid ngăn cách rõ ràng.
- Khi blocked, xe vẫn có `active trajectory` duy nhất theo main.

### Review checkpoint

- Review vùng hình học kiểm tra marking giữa hai lane.
- Review khi nào lane change được coi là hoàn tất để trở lại `FOLLOW_MAIN`.

## Phase 9: Recovery, Dropout và Trajectory Invalidity

### Mục tiêu

- Hoàn thiện behavior khi perception kém hoặc trajectory hiện tại thật sự không còn dùng được.

### Thay đổi chính

- Định nghĩa rõ các trạng thái:
  - `TEMPORARY_DROPOUT`
  - `BLOCKED`
  - `RECOVERY`
- Thêm rule:
  - dropout ngắn hạn -> giữ trajectory cũ trong hold window
  - perception quay lại -> blend mềm với observation mới
  - trajectory hết chiều dài phía trước hoặc lệch xa quá ngưỡng -> cho phép recovery/replan
- Recovery phase đầu có thể đơn giản:
  - ưu tiên quay về `FOLLOW_MAIN` nếu observation đủ tốt
  - nếu chưa đủ tốt, giữ path ngắn an toàn hoặc publish trạng thái invalid rõ ràng

### Tiêu chí hoàn thành

- Mất lane ngắn hạn không làm xe đổi hướng đột ngột.
- Có quy tắc rõ để thoát khỏi trajectory cũ khi nó thật sự không còn hợp lệ.

### Review checkpoint

- Review ngưỡng dropout hold.
- Review ngưỡng lateral/heading error để coi trajectory là invalid.

## Phase 10: Control Projection và Debug Telemetry

### Mục tiêu

- Hoàn thiện bước chiếu `active trajectory` sang `/avs/control_error`.
- Mở rộng debug để tuning được normalizer và manager.

### Thay đổi chính

- Viết hoặc refactor `ControlErrorProjector`:
  - lấy lookahead point từ `active trajectory`
  - tính `epsilon_x_mm`, `epsilon_y_mm`
  - tính `theta_rad`
  - tính `curvature_inv_mm`
- Mở rộng debug `/avs/lane_state`:
  - `trajectory_kind`
  - `committed_trajectory_id`
  - `normalization_mode`
  - `trajectory_confidence`
  - `progress_s_mm`
  - `remaining_s_mm`
  - `dropout_hold_counter`
  - `replan_reason`
  - `active_trajectory_points`

### Tiêu chí hoàn thành

- Controller downstream chỉ thấy một trajectory ổn định hơn trước.
- Log/debug đủ để xác minh hệ thống đang:
  - follow main
  - hold path cũ
  - update mềm
  - hay commit maneuver mới

### Review checkpoint

- Review kích thước payload debug.
- Review field nào thực sự cần publish mọi frame, field nào chỉ cần log nội bộ.

## Phase 11: Unit Test, Fixture Offline và Runtime Verification

### Mục tiêu

- Có bộ test và fixture đủ để verify toàn bộ kiến trúc mới trước khi chạy xe thật.

### Thay đổi chính

- Tạo fixture JSON cho các scenario:
  - follow main đường thẳng
  - follow main đường cong
  - follow main qua giao lộ
  - turn right có 2 turn-lane
  - turn left có 2 turn-lane
  - lane change bị solid block
  - lane change được phép với dashed
  - dropout 2-5 frame rồi phục hồi
- Viết test cho:
  - `PathObservationBuilder`
  - `TrajectoryPlanner`
  - `TrajectoryNormalizer`
  - `TrajectoryManager`
- Kiểm tra các invariant:
  - mỗi frame chỉ publish một `active trajectory`
  - không replan nếu không có trigger hợp lệ
  - mặc định luôn quay về hoặc giữ `FOLLOW_MAIN`

### Tiêu chí hoàn thành

- Có cách chạy test offline lặp lại được.
- Có thể nhìn log/debug và giải thích vì sao một trajectory được giữ hay bị thay.

### Review checkpoint

- Review coverage của fixture với proposal và `decision_sys.md`.
- Review case nào còn thiếu để tái hiện nhiễu perception thật.

## Thứ tự triển khai khuyến nghị

Nếu muốn giảm rủi ro, nên làm theo thứ tự:

1. Phase 1
2. Phase 2
3. Phase 3
4. Phase 4
5. Phase 5
6. Phase 6
7. Phase 10
8. Phase 7
9. Phase 8
10. Phase 9
11. Phase 11

Lý do:

- làm `follow_main` có planning + normalization + memory trước
- xác nhận flow mặc định ổn định
- sau đó mới gắn turn và lane change vào cùng kiến trúc

## Kết quả mong đợi sau khi hoàn tất

Sau khi hoàn thành plan này, hệ thống sẽ có các đặc tính:

- planning là cơ chế mặc định ở mọi frame
- `follow_main` không còn là case đặc biệt đi tắt
- path cho controller được chuẩn hóa liên tục, ít rung hơn
- trajectory có bộ nhớ qua nhiều frame
- replan chỉ xảy ra khi có trigger hợp lệ
- toàn bộ turn/lane change/follow main dùng chung một kiến trúc trajectory thống nhất
