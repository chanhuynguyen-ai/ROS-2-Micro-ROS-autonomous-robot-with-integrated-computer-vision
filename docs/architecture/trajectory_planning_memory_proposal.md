# Đề Xuất: Trajectory Planning Có Bộ Nhớ Và Chuẩn Hóa Đường Cong Mỗi Frame

## Mục tiêu đính chính

- Không đi theo hướng “chỉ khi sắp rẽ hoặc sắp qua giao lộ mới lập kế hoạch”.
- Ở mọi frame, hệ thống đều phải thực hiện bước `trajectory planning`, kể cả khi xe đang chạy trên một đoạn thẳng, một đoạn cong bình thường, hay chưa có giao lộ phía trước.
- `line` nhìn thấy từ perception không còn được coi là output cuối cùng để bám trực tiếp. Trong decision/planning, `line` phải được coi là `path input` để sinh ra `active trajectory` cho xe bám, tương tự triết lý xe dò line nhưng ở mức có kế hoạch.
- Ở mỗi frame, đường cong/path quan sát được phải được chuẩn hóa để giảm nhiễu trước khi dùng làm đường dẫn cho controller.
- Nếu không có lệnh từ người dùng hoặc biển báo thì mặc định xe luôn `follow_main`.
- Chỉ khi có `route_intent` mới từ người dùng hoặc có biển báo/rule tác động thì hệ thống mới `replan` sang maneuver khác, và vẫn phải tuân thủ luật chọn lane trong `./decision_sys.md`.

## Thay đổi tư duy cốt lõi

Điểm cốt lõi không phải là:

- frame hiện tại thấy lane nào thì bám lane đó
- chỉ nối smooth line khi gặp giao lộ

Mà phải là:

- mọi frame đều sinh hoặc cập nhật một `planned path`
- mọi frame đều chuẩn hóa path đó để tạo ra một `normalized trajectory`
- controller luôn bám `normalized committed trajectory`
- hệ thống chỉ đổi kế hoạch khi có lý do nghiệp vụ rõ ràng

Nói ngắn gọn:

- perception cung cấp `path observation`
- planner biến observation thành `planned trajectory`
- normalizer làm mượt và ổn định trajectory theo thời gian
- manager quyết định giữ trajectory hiện tại hay replan

## Nguyên tắc hành vi mong muốn

### 1. Luôn lập kế hoạch ở mọi frame

Ngay cả trong các tình huống sau, planner vẫn phải chạy:

- đường thẳng, chỉ thấy `main-lane`
- đường cong, chỉ thấy `main-lane`
- đoạn không có giao lộ
- đang đi giữa giao lộ
- đang chuyển làn
- perception thiếu lane ngắn hạn

Khác biệt là:

- trong tình huống bình thường, planner có thể tiếp tục xác nhận và cập nhật mềm cùng một `follow_main plan`
- trong tình huống có intent hoặc biển báo, planner sẽ tạo ứng viên mới và manager quyết định replan

### 2. `follow_main` là plan mặc định

Nếu không có điều kiện đặc biệt:

- không có lệnh người dùng
- không có biển báo/rule kích hoạt maneuver khác
- không có tình huống blocked buộc đổi chiến lược

thì hành vi mặc định luôn là:

- `route_intent = follow_main`
- path mặc định là path của `main-lane` sau khi chuẩn hóa

Điều này phải đúng cả khi:

- xe đang đi thẳng
- xe đang ôm cua
- xe đang đi xuyên qua giao lộ nhưng vẫn tiếp tục theo main

### 3. Chỉ replan khi có trigger hợp lệ

Không replan chỉ vì:

- frame hiện tại thấy lane khác rõ hơn
- path hiện tại hơi nhiễu
- topology tức thời thay đổi nhẹ

Chỉ replan khi có ít nhất một trigger hợp lệ:

- người dùng gửi `route_intent` mới
- biển báo hoặc luật giao thông yêu cầu đổi hành vi
- trajectory hiện tại mất hiệu lực thực sự
- trajectory hiện tại bị chặn bởi rule lane/marking

## Vấn đề của cách tiếp cận theo line tức thời

Nếu coi output của từng frame là line để bám ngay, hệ thống sẽ gặp các vấn đề:

- đường cong bị rung do nhiễu detection theo từng frame
- khi lane bị thiếu ngắn hạn, output dễ bị đứt
- khi vào giao lộ, path có thể biến mất hoặc nhảy lane
- khi perception nhìn thấy lane khác rõ hơn, xe có thể bị kéo sai hướng
- khó thống nhất logic giữa follow main, turn, lane change và recovery

Do đó, cần chuyển từ:

- `frame-by-frame line following`

sang:

- `plan every frame + normalize + commit + controlled replan`

## Kiến trúc đề xuất

## 1. Pipeline tổng quát

```text
telemetry_realworld
-> lane/path extraction
-> path normalization per frame
-> candidate trajectory generation
-> trajectory commitment / hold / replan
-> active trajectory for controller
-> control_error
```

## 2. Ba lớp chính

### A. PathObservationBuilder

Nhiệm vụ:

- nhận lane/marking từ telemetry
- chuyển dữ liệu nhìn thấy ở frame hiện tại thành `path observation` cục bộ
- xác định các thực thể phục vụ planning:
  - `main_current`
  - `main_ahead` nếu có
  - `other_lane_left/right`
  - `turn_lane_left/right`
  - marking ngăn cách
- biểu diễn mỗi lane dưới dạng polyline/curve trong vehicle frame

Điểm quan trọng:

- layer này chưa quyết định xe sẽ đi theo lane nào
- layer này chỉ tạo quan sát hình học để planner sử dụng

### B. TrajectoryPlanner

Nhiệm vụ:

- luôn chạy ở mọi frame
- sinh `candidate trajectory` tương ứng với intent hiện tại
- khi không có intent đặc biệt thì mặc định sinh `FOLLOW_MAIN_PLAN`
- tuân thủ luật chọn lane trong `./decision_sys.md`

Ví dụ:

- `follow_main`
  - lấy `main_current` làm path cơ sở
  - nếu có `main_ahead` thì nối topology để kéo dài path theo main
  - nếu chỉ thấy `main_current` thì vẫn plan path theo main hiện tại
- `turn_right`
  - chọn `turn-lane` phía phải
  - nếu có 2 lane hợp lệ thì chọn lane gần hơn
- `turn_left`
  - chọn `turn-lane` phía trái
  - nếu có 2 lane hợp lệ thì chọn lane xa hơn
  - nếu bị solid marking chặn thì không commit maneuver trái
- `lane_change_left/right`
  - chọn `other-lane` theo lateral
  - kiểm tra marking solid/dashed đúng theo spec

### C. TrajectoryManager

Nhiệm vụ:

- giữ `active committed trajectory`
- quyết định:
  - cập nhật mềm trajectory hiện tại
  - giữ trajectory hiện tại
  - hay replan sang trajectory mới
- ngăn việc đổi path liên tục theo nhiễu từng frame

Manager là nơi áp dụng:

- hysteresis
- hold window
- commit policy
- replan trigger

## Chuẩn hóa đường cong mỗi frame

## 1. Mục đích

Path/lane quan sát được ở từng frame thường nhiễu do:

- segmentation mask rung
- centerline extraction không ổn định
- homography và fit curve dao động
- thiếu điểm cục bộ

Vì vậy, trước khi đưa sang controller, mỗi frame cần có bước `curve normalization`.

Mục tiêu của chuẩn hóa:

- giảm rung hình học giữa các frame
- giữ tính liên tục về hướng và độ cong
- tránh giật khi lane observation thay đổi nhẹ
- tạo một path đủ mượt để xe bám như xe dò line, nhưng trên nền path đã được lập kế hoạch

## 2. Quan điểm chuẩn hóa

`line` hay lane của frame hiện tại không nên đi thẳng xuống controller.

Thay vào đó:

1. dùng lane/path hiện tại để tạo `raw candidate path`
2. so sánh với `normalized trajectory` của frame trước
3. thực hiện chuẩn hóa theo thời gian và hình học
4. lưu kết quả thành `normalized trajectory` mới
5. controller chỉ bám `normalized trajectory`

Đây chính là phần “bộ nhớ” quan trọng:

- bộ nhớ không chỉ dùng khi mất lane
- bộ nhớ được dùng liên tục ở mọi frame để ổn định đường dẫn

## 3. Đề xuất phương pháp chuẩn hóa

Ý tưởng dùng đường cong của frame trước để chuẩn hóa frame hiện tại là đúng hướng. Có thể triển khai theo mô hình tổng quát hơn như sau:

### Bước 1. Tạo path tham chiếu theo arc-length

- resample path hiện tại theo bước cố định, ví dụ 50-100 mm
- resample trajectory đã chuẩn hóa của frame trước theo cùng chuẩn `s`
- đưa hai path về cùng hệ tham số arc-length để so sánh ổn định

### Bước 2. Căn chỉnh phần chồng lắp gần xe

- ưu tiên vùng gần xe vì đó là phần controller dùng ngay
- tìm đoạn chồng lắp giữa:
  - `previous_normalized_trajectory`
  - `current_raw_candidate`
- đánh giá:
  - sai lệch lateral
  - sai lệch heading
  - sai lệch curvature

### Bước 3. Blend mềm theo không gian và thời gian

Thay vì chọn cứng “dùng path cũ” hoặc “dùng path mới”, dùng blend có trọng số:

- gần xe: ưu tiên tính ổn định của path trước
- xa xe: cho phép nhận hình học mới nhiều hơn
- khi confidence perception cao: tăng trọng số path mới
- khi confidence perception thấp hoặc path mới nhiễu: tăng trọng số path cũ

Có thể mô tả:

- `normalized_path(s) = w_prev(s) * prev_path(s) + w_cur(s) * cur_path(s)`

với điều kiện:

- `w_prev(s) + w_cur(s) = 1`
- vùng gần xe có `w_prev` cao hơn
- vùng xa có `w_cur` cao hơn nếu observation mới đáng tin

### Bước 4. Ràng buộc độ cong và tiếp tuyến

Sau khi blend, cần kiểm tra lại:

- continuity vị trí
- continuity heading
- continuity curvature
- giới hạn curvature hợp lệ cho xe

Nếu cần, fit lại bằng:

- cubic Hermite
- cubic Bezier
- spline có ràng buộc đạo hàm

để tránh gãy góc hoặc cong quá mức.

### Bước 5. Lưu trajectory đã chuẩn hóa

Kết quả cuối cùng của frame là:

- `normalized_committed_trajectory`

Trajectory này được lưu lại để dùng cho frame kế tiếp, kể cả khi frame sau:

- ít điểm hơn
- bị nhiễu hơn
- hoặc mất lane ngắn hạn

## 4. Hướng tối ưu hơn đề xuất ban đầu

So với cách chỉ “dùng đường cong frame trước so với frame hiện tại”, phương án tối ưu hơn là:

- không làm việc trên fit curve thuần túy của từng frame
- mà làm việc trên `committed trajectory` đã được chuẩn hóa và resample ổn định

Lý do:

- fit curve của từng frame có thể đổi mạnh theo số lượng điểm và ROI
- `committed trajectory` là object ổn định hơn về mặt điều khiển
- dễ thêm hysteresis, progress tracking và replan policy

Vì vậy, bộ nhớ nên lưu:

- trajectory đã chuẩn hóa
- confidence của từng đoạn hoặc toàn path
- thời điểm/frame tạo ra trajectory đó
- tiến độ xe đang đi đến đâu trên trajectory

thay vì chỉ lưu các hệ số curve fit rời rạc.

## Chính sách mặc định và replan

## 1. Chính sách mặc định

Nếu không có trigger mới:

- giữ `route_intent = follow_main`
- planner tiếp tục plan theo `main`
- normalizer tiếp tục làm mượt path main qua từng frame
- manager tiếp tục commit cùng một hướng đi

Điểm này áp dụng cả khi:

- `main-lane` là đường thẳng
- `main-lane` là đường cong
- `main-lane` bị đứt ngắn hạn
- `main-lane` đang đi xuyên qua giao lộ

## 2. Trigger replan

Replan chỉ nên xảy ra khi có một trong các điều kiện:

- user đổi `route_intent`
- biển báo/rule yêu cầu đổi hướng đi
- maneuver hiện tại hoàn tất và có intent kế tiếp
- trajectory hiện tại không còn hợp lệ về topology
- trajectory hiện tại bị blocked bởi rule lane/marking

Không replan khi chỉ có:

- lane rung nhẹ
- frame mới có path nhìn đẹp hơn một chút
- missing detection ngắn hạn nhưng trajectory cũ vẫn còn hợp lệ

## 3. Fallback khi không có trigger

Khi không có trigger đặc biệt:

- tiếp tục follow `main`
- tiếp tục dùng trajectory đã chuẩn hóa gần nhất
- chỉ suy giảm confidence hoặc rút ngắn hold nếu perception tiếp tục kém

## Áp dụng cho từng loại maneuver

## 1. Follow Main

Đây là trường hợp mặc định và phải được đối xử như một maneuver đầy đủ, không phải case đơn giản bỏ qua planner.

Planner mỗi frame vẫn phải:

- lấy `main_current`
- nối với `main_ahead` nếu có
- tạo candidate path theo topology hiện tại
- chuẩn hóa với trajectory của frame trước
- lưu lại thành `active trajectory`

Khi đi qua giao lộ nhưng vẫn theo main:

- planner vẫn phải coi đây là `follow_main plan`
- không cần đợi đến sát giao lộ mới sinh path
- phải duy trì một main path liên tục qua vùng mất line

## 2. Turn Right / Turn Left

Khi có intent rẽ:

- planner sinh candidate trajectory mới theo luật chọn lane trong `decision_sys.md`
- manager replan từ `follow_main` sang `turn plan`
- sau khi commit, trajectory rẽ tiếp tục được chuẩn hóa ở mọi frame giống như follow main

Điểm quan trọng:

- cơ chế chuẩn hóa là thống nhất
- không có pipeline riêng chỉ dành cho giao lộ

## 3. Lane Change

Khi có intent chuyển làn:

- planner kiểm tra lane mục tiêu và marking
- nếu hợp lệ thì sinh trajectory chuyển làn
- manager commit trajectory này
- trong các frame tiếp theo vẫn tiếp tục chuẩn hóa trajectory chuyển làn theo cùng cơ chế

Điểm mạnh:

- xe không bị đổi mục tiêu lane mỗi frame
- xe không bị rơi path khi đang ở giữa hai lane

## 4. Mất lane ngắn hạn

Khi perception tạm thời mất lane:

- nếu chưa có trigger replan mới thì vẫn giữ intent cũ
- vẫn dùng trajectory đã chuẩn hóa gần nhất trong hold window
- khi observation quay lại thì blend mềm với path mới

Như vậy, dropout chỉ là một trạng thái confidence thấp của planning, không phải lý do tự động đổi hướng đi.

## Dữ liệu trạng thái nên lưu

Để hiện thực hóa cơ chế trên, manager nên lưu tối thiểu:

- `committed_trajectory_id`
- `trajectory_kind`
- `route_intent`
- `normalized_points`
- `progress_s_mm`
- `remaining_s_mm`
- `trajectory_confidence`
- `last_good_observation_frame`
- `replan_reason`
- `source_lane_ids`
- `target_lane_id`

Có thể thêm metadata cho chuẩn hóa:

- `normalization_mode`
- `blend_weight_prev`
- `blend_weight_cur`
- `curvature_stats`
- `dropout_hold_counter`

## Thuật toán runtime đề xuất

Ở mỗi frame:

1. build `path observation` từ lane/marking hiện thấy
2. sinh `candidate trajectory` theo `route_intent` hiện tại
3. nếu không có intent mới thì mặc định candidate chính là `follow_main`
4. lấy `committed trajectory` của frame trước làm reference
5. chuẩn hóa candidate hiện tại với reference trước đó
6. đánh giá trajectory hiện tại còn hợp lệ hay cần replan
7. nếu không có trigger replan thì cập nhật mềm trajectory hiện tại
8. nếu có trigger replan hợp lệ thì commit trajectory mới
9. publish một `active trajectory` duy nhất cho controller

## Quan hệ với `./decision_sys.md`

Tài liệu này không thay luật chọn lane trong `./decision_sys.md`.

Ngược lại, tài liệu này bổ sung cách thực thi ổn định hơn:

- `decision_sys.md` trả lời câu hỏi: chọn lane/trajectory nào theo rule nghiệp vụ
- tài liệu này trả lời câu hỏi: sau khi có rule đó thì làm sao duy trì trajectory liên tục, mượt và ít nhiễu qua nhiều frame

Các luật phải giữ nguyên:

- mặc định là `follow_main`
- `turn_right` chọn lane gần hơn nếu có 2 lane hợp lệ
- `turn_left` chọn lane xa hơn nếu có 2 lane hợp lệ
- `lane_change` phải tuân thủ solid/dashed marking
- mỗi thời điểm chỉ publish một `active trajectory`

## Kỳ vọng đầu ra

Sau khi áp dụng cách làm này, hệ thống nên đạt được:

- luôn có bước planning ở mọi frame, không phụ thuộc có giao lộ hay không
- `follow_main` trở thành maneuver mặc định có planning đầy đủ
- đường dẫn cho controller ổn định hơn, ít rung hơn
- cùng một cơ chế áp dụng cho đường thẳng, đường cong, giao lộ, rẽ và chuyển làn
- không đổi trajectory chỉ vì nhiễu tức thời
- khi không có trigger mới thì xe luôn tiếp tục bám `main`

## Kết luận

Đính chính quan trọng nhất là:

- không được hiểu trajectory planning như cơ chế chỉ dùng cho turn/giao lộ
- mà phải coi đó là cơ chế mặc định của toàn bộ việc bám đường

Ở mọi frame:

- luôn plan
- luôn chuẩn hóa curve/path
- luôn lưu trajectory đã chuẩn hóa làm đường dẫn cho xe bám

Và ở tầng quyết định:

- mặc định `follow_main`
- chỉ replan khi có lệnh người dùng hoặc biển báo/rule hợp lệ
- mọi replan phải tuân thủ luật chọn lane trong `./decision_sys.md`
