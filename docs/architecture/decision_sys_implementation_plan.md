# Implementation Plan: Decision System

Tài liệu này chuyển nội dung thiết kế trong `./decision_sys.md` thành kế hoạch triển khai theo từng phase. Sau khi hoàn thành mỗi phase, dừng lại để review trước khi đi tiếp phase kế tiếp.

## Mục tiêu triển khai

- Thay logic chọn lane trực tiếp trong `control_node.cpp` bằng một decision layer có cấu trúc rõ ràng.
- Nhận route intent từ `/avs/route_intent`, không dùng `/avs/cmd` cho ý định rẽ/chuyển làn.
- Không dùng `stop-line` để kích hoạt rẽ, phát hiện ngã tư, phát hiện ngã ba chữ T, hoặc quyết định chuyển làn.
- Mỗi frame/quyết định chỉ sinh đúng một `active trajectory` để tính `/avs/control_error`.
- Giữ nguyên contract downstream của `/avs/control_error` để không phải đổi bộ điều khiển Pure Pursuit/PD.

## Nguyên tắc chung

- Mỗi phase phải build được bằng `colcon build --symlink-install --packages-select avs_perception`.
- Mỗi phase phải giữ node `control_node` chạy được, kể cả khi một số rule chưa được bật.
- Ưu tiên tách helper/data model khỏi callback ROS để có thể test bằng JSON mẫu.
- Các label ID không viết rải rác bằng magic number; gom về constant/enum theo mapping hiện có.
  > **Cảnh báo**: bảng mapping dưới đây đã LỖI THỜI (ví dụ `turn-lane = 10` — giá trị đúng hiện tại là `17`). Source-of-truth runtime là `models/best_ncnn_model/metadata.yaml` + `class_names` trong `ncnn_inference_node.cpp`/`yolo26_seg.hpp`, mirror ở `tools/local_post_inference_simulator/label_mapping.json`. Đừng dùng số trong bảng này để code hoặc debug.
  - `main-lane = 3`
  - `other-lane = 4`
  - `solid-white = 6`
  - `solid-yellow = 7`
  - `stop-line = 9`
  - `turn-lane = 10`
  - `double-solid-white = 2`
  - `dashed-white = 0`
  - `dashed-yellow = 1`

## Phase 1: Chuẩn hóa Intent và Data Model

### Mục tiêu

- Tách parsing telemetry/intent khỏi state machine cũ.
- Thêm subscriber `/avs/route_intent` nhưng chưa đổi sâu logic điều khiển.
- Giữ behavior mặc định là `follow_main`.

### Thay đổi chính

- Trong `control_node.cpp`, thêm các enum/struct nội bộ:
  - `RouteIntent`: `FOLLOW_MAIN`, `TURN_RIGHT`, `TURN_LEFT`, `LANE_CHANGE_LEFT`, `LANE_CHANGE_RIGHT`.
  - `DecisionState`: `FOLLOW_MAIN`, `TURN_RIGHT`, `TURN_LEFT`, `LANE_CHANGE`, `BLOCKED`, `RECOVERY`.
  - `LaneCandidate`: label, class name, waypoints, polynomial, offsets, lookahead fields, bbox/range nếu có.
  - `MarkingCandidate`: label, class name, polygon/waypoints/range.
- Subscribe `/avs/route_intent` dạng `std_msgs::msg::String`.
- Parse intent JSON:
  - Nếu intent không hợp lệ hoặc thiếu, fallback về `FOLLOW_MAIN`.
  - Bỏ hoàn toàn intent `straight`; nếu nhận `straight` từ nguồn cũ thì map về `FOLLOW_MAIN` kèm warning throttle.
- Giữ `/avs/cmd` cho lệnh hệ thống hiện có nếu cần, nhưng không dùng nó để quyết định rẽ/chuyển làn.

### Tiêu chí hoàn thành

- Build thành công.
- Khi không có `/avs/route_intent`, node vẫn publish `/avs/control_error` theo `main-lane`.
- `/avs/lane_state` có thêm `route_intent` và `decision_state`.
- Không còn dependency logic rẽ vào `stop-line` trong flow mới của phase này.

### Review checkpoint

- Review schema `/avs/route_intent`.
- Review cách fallback intent không hợp lệ.
- Review việc giữ `/avs/cmd` chỉ cho lệnh hệ thống.

## Phase 2: Active Trajectory Data Path

### Mục tiêu

- Tạo đường dữ liệu mới: telemetry lane/marking -> một `active trajectory` -> `/avs/control_error`.
- Phase này chỉ cần hỗ trợ `follow_main`, chưa cần rẽ/chuyển làn phức tạp.

### Thay đổi chính

- Thêm struct `Point2D` và `ActiveTrajectory`:
  - `points`: danh sách `(x_mm, y_mm)`.
  - `source_labels`: label/lane đã dùng để sinh trajectory.
  - `trajectory_kind`: `follow_main`, `turn_right`, `turn_left`, `lane_change`, `blocked_follow_main`.
  - `valid`: trajectory có đủ điểm để controller bám hay không.
- Viết helper:
  - `extract_lane_candidates(telemetry)`.
  - `extract_marking_candidates(telemetry)`.
  - `build_follow_main_trajectory(main_lane)`.
  - `compute_control_error_from_trajectory(active_trajectory, lookahead_d_mm)`.
- `compute_control_error_from_trajectory` tính:
  - `epsilon_x_mm`, `epsilon_y_mm` từ waypoint lookahead.
  - `theta_rad` từ tiếp tuyến cục bộ.
  - `curvature_inv_mm` từ fit/đạo hàm cục bộ quanh waypoint; nếu không đủ điểm thì dùng `0.0`.
- Publish debug `active_trajectory_points` trong `/avs/lane_state`, giới hạn số điểm nếu payload quá lớn.

### Tiêu chí hoàn thành

- `follow_main` dùng `active trajectory` thay vì đọc trực tiếp `lookahead_x_mm` từ object.
- Mỗi frame chỉ có một `active trajectory` được chọn.
- Nếu không có `main-lane`, node không publish error giả; publish lane_state với `valid=false` hoặc giữ fallback hiện tại có cảnh báo throttle.

### Review checkpoint

- Review format debug của `active_trajectory_points`.
- Review công thức tính `theta_rad` và `curvature_inv_mm` từ polyline.
- Review invariant: chỉ một trajectory đi ra controller.

## Phase 3: Smoothing và Nối Main-Lane Qua Giao Lộ

### Mục tiêu

- Hỗ trợ `follow_main` qua đường cong và ngã tư theo đúng rule: không cần intent `straight`.
- Không phụ thuộc `stop-line`.

### Thay đổi chính

- Viết helper chọn `main-lane` hiện tại và `main-lane` mục tiêu phía trước:
  - Ưu tiên lane có điểm gần xe nhất và hướng theo trục `Y`.
  - Nếu có nhiều đoạn `main-lane`, chọn đoạn hiện tại gần xe và đoạn phía trước có `Y` lớn hơn.
- Viết `connect_two_lanes_smooth(current_lane, target_lane)`:
  - Dùng cubic Bezier hoặc cubic Hermite.
  - Điểm đầu lấy từ đoạn cuối lane hiện tại theo hướng tiến.
  - Điểm cuối lấy từ đoạn đầu lane mục tiêu phía trước.
  - Tiếp tuyến đầu/cuối lấy từ vài điểm gần endpoint.
- Nếu chưa thấy lane mục tiêu phía trước đủ tin cậy, chỉ follow `main-lane` hiện tại.
- Không dùng `stop-line` để quyết định có nối hay không.

### Tiêu chí hoàn thành

- `follow_main` trên đường thẳng/đường cong vẫn hoạt động.
- `follow_main` qua ngã tư tạo đúng một line smooth nối `main-lane` hiện tại sang `main-lane` phía trước khi đủ dữ liệu.
- Không tạo line nối xa nếu chỉ thấy một đoạn main-lane.
- Build thành công.

### Review checkpoint

- Review điều kiện chọn lane hiện tại/lane phía trước.
- Review Bezier/Hermite endpoint và tangent.
- Review các trường hợp không đủ dữ liệu.

## Phase 4: Turn Right, Turn Left và T-Junction

### Mục tiêu

- Triển khai chọn `turn-lane` cho rẽ phải/rẽ trái và ngã ba chữ T.
- Vẫn đảm bảo chỉ có một `active trajectory`.

### Thay đổi chính

- Viết helper phân loại turn-lane theo hình học BEV:
  - Lấy centerline/polyline của từng `turn-lane`.
  - Tính vị trí lateral đại diện bằng median/endpoint gần xe.
  - Tính điểm gần xe nhất và khoảng cách tới mép phải/trái vehicle frame.
- `turn_right`:
  - Lọc các `turn-lane` nằm về phía phải/hướng rẽ phải.
  - Nếu có 2 lane hợp lệ, chọn lane gần hơn.
  - Sinh một line smooth từ lane hiện tại sang lane đã chọn.
- `turn_left`:
  - Lọc các `turn-lane` tương ứng hướng rẽ trái.
  - Nếu có 2 lane hợp lệ, chọn lane xa hơn.
  - Sinh một line smooth từ lane hiện tại sang lane đã chọn.
- T-Junction:
  - Phát hiện bằng hình học: `main-lane` phía trước không tiếp tục hợp lệ, có `turn-lane` chạy ngang/chéo phía trước, không có `main-lane` đối diện như ngã tư.
  - Không yêu cầu `stop-line`.
  - Với `turn_right`, chọn lane gần hơn.
  - Với `turn_left`, chọn lane xa hơn nếu không bị solid marking chặn; marking gate chi tiết sẽ hoàn thiện ở Phase 5.

### Tiêu chí hoàn thành

- `turn_right` chọn đúng lane gần hơn khi có 2 `turn-lane`.
- `turn_left` chọn đúng lane xa hơn khi có 2 `turn-lane`.
- T-Junction không có `stop-line` vẫn chọn được hướng theo intent.
- Nếu không tìm được lane hợp lệ, không publish trajectory rẽ giả; fallback có cảnh báo rõ trong `/avs/lane_state`.

### Review checkpoint

- Review metric gần/xa của `turn-lane`.
- Review rule nhận diện T-Junction bằng hình học.
- Review fallback khi intent rẽ nhưng perception chưa đủ dữ liệu.

## Phase 5: Lane Change và Solid/Dashed Marking Gate

### Mục tiêu

- Chặn chuyển làn khi có solid marking giữa `main-lane` và `other-lane`.
- Cho phép chuyển làn khi marking giữa lane là dashed hoặc không có solid ngăn cách rõ ràng.

### Thay đổi chính

- Viết helper chọn `other-lane` mục tiêu:
  - `lane_change_left`: chọn `other-lane` có lateral nhỏ hơn `main-lane`.
  - `lane_change_right`: chọn `other-lane` có lateral lớn hơn `main-lane`.
  - Nếu có nhiều `other-lane`, chọn lane gần `main-lane` nhất theo lateral.
- Viết `is_lane_change_blocked_by_solid(main_lane, target_lane, markings)`:
  - Xét vùng gần xe và đoạn giữa hai centerline.
  - Nếu marking solid cắt/nằm giữa hai lane trong vùng xét, return blocked.
  - Dashed marking không block.
- Khi blocked:
  - `decision_state = BLOCKED`.
  - `blocked_by_marking = true`.
  - `trajectory_kind = blocked_follow_main`.
  - Active trajectory duy nhất vẫn là `follow_main`.
- Khi allowed:
  - Sinh một line smooth từ `main-lane` sang `other-lane` mục tiêu.

### Tiêu chí hoàn thành

- Chuyển làn bị chặn bởi `solid-white`, `solid-yellow`, `double-solid-white`.
- Chuyển làn được phép với `dashed-white`, `dashed-yellow`.
- Khi blocked, controller chỉ nhận line follow main.
- `/avs/lane_state` thể hiện rõ lane mục tiêu, marking block, và trajectory kind.

### Review checkpoint

- Review vùng hình học dùng để kiểm tra marking giữa hai lane.
- Review behavior khi không thấy marking.
- Review fallback khi không thấy `other-lane` mục tiêu.

## Phase 6: Debug, Logging và Công Cụ Test JSON

### Mục tiêu

- Làm hệ thống dễ kiểm thử offline trước khi chạy robot thật.
- Không cần camera hoặc ROS runtime đầy đủ để kiểm tra rule chính.

### Thay đổi chính

- Thêm thư mục test fixture nếu chưa có, ví dụ `ros2_ws/src/avs_perception/test/decision_fixtures/`.
- Tạo JSON mẫu cho các scenario:
  - Follow main đường thẳng.
  - Follow main qua ngã tư.
  - Turn right có 2 turn-lane.
  - Turn left có 2 turn-lane.
  - T-Junction không stop-line.
  - Lane change bị solid chặn.
  - Lane change được phép với dashed.
- Tách decision helper đủ độc lập để có thể gọi bằng unit test hoặc executable test nhỏ.
- Thêm log throttle cho các failure mode:
  - Không có lane mục tiêu.
  - Intent không hợp lệ.
  - Trajectory không đủ điểm.
  - Lane change bị blocked.

### Tiêu chí hoàn thành

- Có fixture JSON đủ để review thủ công.
- Có cách chạy test offline lặp lại được.
- Debug output đủ để nhìn thấy vì sao hệ thống chọn một trajectory.

### Review checkpoint

- Review fixture JSON có phủ đúng các rule trong `./decision_sys.md`.
- Review debug fields có đủ cho dashboard/log.
- Review thông báo lỗi có đủ rõ để tuning perception.

## Phase 7: Build, Integration và Runtime Verification

### Mục tiêu

- Xác minh toàn bộ pipeline perception -> decision -> control error.
- Chuẩn bị checklist chạy robot thật.

### Thay đổi chính

- Chạy build:
  - `cd ros2_ws`
  - `colcon build --symlink-install --packages-select avs_perception`
- Chạy node với dữ liệu mẫu hoặc video offline.
- Kiểm tra topic:
  - `/avs/telemetry_realworld`
  - `/avs/route_intent`
  - `/avs/control_error`
  - `/avs/lane_state`
- Nếu dashboard cần overlay trajectory, bổ sung consumer cho `active_trajectory_points` ở phase riêng sau review.

### Tiêu chí hoàn thành

- Build sạch.
- `/avs/control_error` luôn được tính từ một `active trajectory`.
- Không có logic hiện tại nào dùng `stop-line` để quyết định rẽ/chuyển làn.
- Tất cả scenario fixture cho kết quả đúng theo expected output.
- Runtime không publish nhiều trajectory ứng viên cho controller.

### Review checkpoint

- Review log/topic output theo từng scenario.
- Review độ mượt của trajectory trên overlay/video.
- Review danh sách việc còn lại trước khi chạy robot thật.

## Thứ tự review đề xuất

1. Review Phase 1 trước khi thêm smoothing để khóa interface `/avs/route_intent`.
2. Review Phase 2 để khóa contract `active trajectory` và `/avs/control_error`.
3. Review Phase 3 để khóa cách nối main-lane qua giao lộ.
4. Review Phase 4 để khóa rule turn-lane và T-Junction.
5. Review Phase 5 để khóa rule solid/dashed cho chuyển làn.
6. Review Phase 6-7 để khóa fixture, test offline và checklist runtime.

## Rủi ro cần theo dõi

- Telemetry hiện tại có thể chỉ giữ một object cho mỗi label trong `control_node.cpp`; implementation mới cần xử lý nhiều object cùng label để chọn đúng lane gần/xa.
- `turn-lane` hiện được fit theo `y(x)` trong `ipm_transform_node.cpp`; decision layer nên dùng `waypoints` như polyline chung để tránh phụ thuộc dạng polynomial.
- Nếu `active_trajectory_points` publish quá dài, payload `/avs/lane_state` có thể lớn; cần giới hạn số điểm debug hoặc publish topic debug riêng.
- Khi không thấy marking, rule mặc định trong plan là không coi là solid block; cần review trên dữ liệu thực để tránh chuyển làn quá tự tin.
- Smoothing quá mạnh có thể làm trajectory cắt qua vùng ngoài làn; cần overlay kiểm tra sau Phase 3-5.
