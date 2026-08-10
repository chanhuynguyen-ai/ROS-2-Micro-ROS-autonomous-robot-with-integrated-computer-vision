# Plan: Local Post-Inference Simulator

## Mục Tiêu

Tạo một giao diện local đơn giản, tách hoàn toàn khỏi dashboard chính, để mô phỏng và test pipeline sau inference:

```text
synthetic binary class frame
-> post-inference object/mask representation
-> pixel-to-world / IPM
-> lane centerline extraction
-> path observation
-> trajectory planning / normalization / manager
-> control error + debug trajectory
```

Giao diện này không dùng model inference thật. Người dùng vẽ hoặc cấu hình các class trực tiếp trên một frame nhị phân/label mask. Mục tiêu là cô lập và debug phần hình học, trích xuất line, chọn lane, lập plan từ line/path và behavior manager.

## Đánh Giá Đề Xuất

Đề xuất này đúng hướng và nên làm trước hoặc song song với refactor decision/trajectory. Lý do:

- Tách lỗi perception model khỏi lỗi hậu xử lý hình học.
- Tạo scenario có kiểm soát: đường cong, giao lộ, T-junction, lane change, solid/dashed marking, dropout.
- Cho phép replay cùng một case nhiều lần để so sánh trước/sau refactor.
- Giúp kiểm tra trực quan centerline, path observation, active trajectory và control error.
- Giảm phụ thuộc vào camera, video và model NCNN khi debug planning, nhưng vẫn giữ ROS graph để mô phỏng đúng runtime thật.

Điểm cần giữ chặt: simulator phải gọi lại thuật toán production hoặc module dùng chung. Nếu viết lại thuật toán riêng trong simulator, kết quả đẹp trên UI có thể không phản ánh lỗi thật trong `control_node.cpp` hoặc `ipm_transform_node.cpp`.

## Quyết Định Đã Chốt

- Simulator ưu tiên chạy hoàn toàn trong ROS, vì mục tiêu là mô phỏng lại phần xử lý của hệ thống thật sau inference.
- UI local chỉ là công cụ vẽ/cấu hình scenario và xem debug; pipeline xử lý chính đi qua ROS node/topic/service.
- Label mapping dùng theo model hiện tại và code inference hiện tại:
  - `0`: `dashed-white`
  - `1`: `dashed-yellow`
  - `2`: `double-solid-white`
  - `3`: `light_green`
  - `4`: `light_red`
  - `5`: `light_yellow`
  - `6`: `main-lane`
  - `7`: `other-lane`
  - `8`: `parking-zone`
  - `9`: `sign-no-left`
  - `10`: `sign-no-parking`
  - `11`: `sign-no-right`
  - `12`: `sign-parking`
  - `13`: `sign-stop`
  - `14`: `sign-turn-left`
  - `15`: `sign-turn-right`
  - `16`: `solid-white`
  - `17`: `solid-yellow`
  - `18`: `start`
  - `19`: `stop-line`
  - `20`: `turn-lane`
  - `21`: `vehicle`
- Calibration dùng chung cấu hình hiện tại, mặc định là calibration production như `config/calibration.json`.
- Output sau inference dùng cấu trúc theo format của model/pipeline hiện tại: object list có `label`, `class_name`, confidence, bbox/polygon/mask metadata, sau đó đi qua IPM/postprocess như runtime thật.
- Planner source-of-truth nên là C++ production path trong ROS, không phải Python harness. Python harness chỉ dùng cho unit test nhanh hoặc đối chiếu, không là logic chính của simulator.

Lưu ý: Mismatch `turn-lane = 10` so với `17` đã được fix và có test bảo vệ (`turn_right_two_lanes.json` fixture). Không còn rủi ro về sai lệch label này trong hệ thống.

## Phạm Vi Ban Đầu

### Có Trong Scope

- Local web UI riêng, không gắn vào dashboard chính.
- Canvas nền đen.
- Người dùng vẽ các vùng/class bằng pixel value hoặc class label.
- Cấu hình class:
  - `main-lane`
  - `other-lane`
  - `turn-lane`
  - `dashed-white`
  - `dashed-yellow`
  - `solid-white`
  - `solid-yellow`
  - `double-solid-white`
  - `stop-line` chỉ để quan sát, không dùng cho decision.
- Sinh synthetic frame/mask và object JSON tương đương output sau inference.
- Chạy lại các bước từ pixel-to-world/IPM trở đi.
- Hiển thị:
  - frame nhị phân/class mask
  - BEV/world points
  - extracted centerline
  - selected lane/path observation
  - planned candidate trajectory
  - committed active trajectory
  - control error
  - lane_state/debug JSON
- Export/import scenario JSON.
- Replay nhiều frame để test memory/dropout/hysteresis.

### Ngoài Scope Ban Đầu

- Không chạy NCNN/TFLite inference.
- Không thay dashboard chính.
- Không mô phỏng động học xe closed-loop đầy đủ.
- Không cần UI đẹp như product; ưu tiên rõ dữ liệu và lặp lại được.
- Không sửa planner bằng logic riêng trong frontend.

## Kiến Trúc Đề Xuất

```text
tools/local_post_inference_simulator/
  backend/
    main.py
    scenario_schema.py
    mask_to_objects.py
    ros_bridge.py
    ros_scenario_runner.py
  frontend/
    index.html
    app.js
    style.css
  fixtures/
    follow_main_straight.json
    follow_main_curve.json
    intersection_follow_main.json
    turn_right_two_lanes.json
    lane_change_solid_blocked.json
  README.md
```

Tài liệu:

```text
docs/local_post_inference_simulator/
  plan.md
  scenario_schema.md
  implementation_notes.md
```

Nguyên tắc:

- Tool nằm ngoài `web_dashboard` để không làm dashboard chính phình ra.
- Backend có thể dùng FastAPI nhẹ như dashboard hiện tại, nhưng chạy port riêng và chỉ làm UI bridge.
- Frontend dùng HTML/CSS/JS thuần để giảm dependency.
- Pipeline xử lý chính phải đi qua ROS topic/service hoặc ROS node giả lập inference, không chạy planner riêng trong frontend/backend.
- Nếu cần adapter nội bộ, adapter chỉ được làm nhiệm vụ chuyển scenario thành message/payload đúng format production.

## Data Model

### Scenario JSON

Scenario nên mô tả được cả một frame đơn và chuỗi frame.

```json
{
  "name": "lane_change_solid_blocked",
  "canvas": {
    "width": 640,
    "height": 480
  },
  "calibration": {
    "source": "config/calibration.json"
  },
  "route_intent": {
    "intent": "lane_change_left",
    "seq": 1
  },
  "frames": [
    {
      "frame_id": 1,
      "objects": [
        {
          "id": "main_lane_1",
          "class_name": "main-lane",
          "label": 6,
          "shape": "polygon",
          "points_px": [[300, 470], [340, 470], [340, 120], [300, 120]],
          "confidence": 1.0
        }
      ]
    }
  ]
}
```

### Binary/Class Frame

Giao diện hiển thị nền đen. Các object do người dùng vẽ được encode bằng pixel value theo label hoặc class id:

- background: `0`
- mỗi class có `pixel_value` riêng trong mask layer
- nếu cần phân biệt instance cùng class, lưu instance ở scenario JSON, không chỉ dựa vào pixel value

Lưu ý quan trọng: nếu chỉ dùng một mask nhị phân duy nhất thì không phân biệt được nhiều class. Vì vậy nên gọi là "class-valued mask" hoặc "label mask": nền đen, pixel khác 0 đại diện class/object.

## Phase 0: Xác Nhận Contract Và Label Mapping

### Mục tiêu

Chốt format input/output để simulator không lệch production và sửa các mismatch label có thể làm pipeline ROS thật sai ngay từ đầu.

### Việc cần làm

- Xác nhận label mapping đang dùng thực tế trong telemetry:
  - `main-lane`
  - `other-lane`
  - `turn-lane`
  - solid/dashed markings
  - `stop-line`
- Xác định input chuẩn sau inference:
  - polygon pixel?
  - mask bitmap?
  - `polygons_real_world`?
  - `waypoints`?
- Xác định module production nào sẽ được gọi lại:
  - IPM transform
  - centerline extraction
  - decision/planning
- [x] Đã hoàn thành sửa mismatch `turn-lane = 10` thành `17` trong `ipm_transform_node.cpp` và có test bảo vệ.
- Tạo constant/source-of-truth label mapping dùng chung cho simulator và production code.

### Lưu ý khi code

- Không hard-code label từ docs cũ nếu code/model hiện tại khác.
- Tạo một file mapping duy nhất, dùng chung cho frontend/backend.
- Nếu production hiện chưa tách module, phase này chỉ tạo adapter tạm nhưng phải ghi rõ nợ kỹ thuật.

### Lưu ý khi build/test

- Chạy `pytest -q test/decision_system` để biết baseline.
- Build ROS bắt buộc nếu phase này sửa label constant hoặc IPM logic.

### Tiêu chí hoàn thành

- Có bảng label mapping chính xác.
- Có schema scenario v1.
- Biết rõ simulator sẽ gọi production code qua cách nào.
- `turn-lane` được xử lý thống nhất là label `20` trong inference, IPM, control và test (model 22 class).

## Phase 1: UI Canvas Vẽ Class Mask

### Mục tiêu

Tạo giao diện local tối thiểu để vẽ scene.

### Việc cần làm

- Canvas nền đen.
- Tool vẽ:
  - polygon lane
  - polyline marking
  - brush/eraser nếu cần
  - drag point để chỉnh shape
- Panel class:
  - chọn class
  - set instance id
  - set confidence
  - set visible/enabled
- Panel route intent:
  - `follow_main`
  - `turn_left`
  - `turn_right`
  - `lane_change_left`
  - `lane_change_right`
- Export/import scenario JSON.

### Lưu ý khi code

- Frontend chỉ tạo shape/mask, không tự tính planning.
- Lưu tọa độ shape ở pixel space và để backend xử lý pipeline.
- Mỗi object phải có stable id để test hysteresis/ID swap.
- Không phụ thuộc ROS trong frontend.
- Frontend không gọi planner/IPM trực tiếp; mọi run scenario phải gửi request tới backend bridge để backend kích hoạt ROS pipeline.

### Lưu ý khi build/test

- Test editor bằng browser local không cần ROS graph, nhưng test "Run" scenario phải có ROS graph simulation đang chạy.
- Kiểm tra export -> import giữ nguyên object id, class, points.

### Tiêu chí hoàn thành

- Người dùng vẽ được một scene lane cơ bản.
- Export JSON có thể reload lại chính xác.

## Phase 2: Mask/Object Generator Sau Inference

### Mục tiêu

Biến shape người dùng vẽ thành dữ liệu tương đương output sau inference và publish vào ROS theo format production.

### Việc cần làm

- Rasterize object thành label mask.
- Tạo object list:
  - `id`
  - `label`
  - `class_name`
  - `confidence`
  - `polygons`
  - `bbox`
  - optional binary mask hoặc RLE nếu cần
- Tạo synthetic post-inference message/payload giống `/avs/telemetry` hoặc topic trung gian hiện tại của inference node.
- Tạo node/publisher giả lập inference, ví dụ `synthetic_inference_node`, publish dữ liệu vào đầu vào của IPM/postprocess.
- Cho phép preview:
  - label mask
  - object contours
  - bbox

### Lưu ý khi code

- Giữ object instance riêng dù pixel value cùng class.
- Không làm mất polygon gốc khi rasterize.
- Nếu production cần contour từ mask, backend nên có mode "polygon input" và "mask-derived contour" để so sánh.
- Payload publish ra ROS phải giữ đúng field mà `ipm_transform_node` đang đọc, không tạo schema mới nếu chưa có adapter rõ ràng.

### Lưu ý khi build/test

- Test nhiều object cùng class không bị merge sai nếu người dùng muốn giữ instance riêng.
- Test object chạm nhau để biết post-process có merge hay tách.

### Tiêu chí hoàn thành

- Sinh được payload tương đương inference output để đưa vào IPM/postprocess.
- Publish được payload đó vào ROS topic simulation.

## Phase 3: Pixel-To-World/IPM Adapter

### Mục tiêu

Chạy lại phần chuyển đổi hệ tọa độ sang thế giới thực bằng ROS node IPM thật hoặc node production-equivalent.

### Việc cần làm

- Load calibration từ `config/calibration.json` hoặc file do user chọn.
- Gọi lại logic homography/IPM đang dùng trong hệ thống thông qua ROS.
- Subscribe output IPM/postprocess để lấy telemetry real-world.
- Output:
  - `polygons_real_world`
  - waypoint/centerline nếu production hiện tạo ở IPM node
  - debug transform metrics
- Hiển thị BEV/world view trên canvas phụ.

### Lưu ý khi code

- Không viết công thức homography riêng khác production nếu có thể tránh.
- Nếu phải port logic sang Python/JS, cần test so sánh với output C++.
- Phải hiển thị rõ hệ trục: `X > 0` sang phải xe, `Y > 0` phía trước xe.
- Không bypass `ipm_transform_node` trong mode chính. Python/OpenCV transform chỉ được dùng cho debug phụ hoặc so sánh.

### Lưu ý khi build/test

- Dùng một polygon đơn giản để kiểm tra transform có đúng hướng.
- So sánh vài điểm pixel -> world với IPM node hiện tại nếu có thể.

### Tiêu chí hoàn thành

- Synthetic lane có polygon/world points đúng trực quan trong BEV.
- Có thể debug lỗi calibration/homography tách khỏi inference.

## Phase 4: Centerline Extraction Và Line Debug

### Mục tiêu

Test riêng phần trích xuất line/centerline từ polygon lane.

### Việc cần làm

- Chạy lại thuật toán:
  - lane dọc quét theo `Y`
  - turn-lane quét theo `X` hoặc theo logic hiện tại
  - lọc midpoint nhiễu
  - fit/waypoint generation
- Hiển thị:
  - polygon world
  - scan slices
  - raw midpoint
  - filtered midpoint
  - final waypoints/centerline
- Cho phép bật/tắt từng overlay.

### Lưu ý khi code

- Đây là phần phải lấy từ hệ thống hiện tại nhiều nhất có thể.
- Không chỉ vẽ line đẹp từ polygon trong frontend.
- Cần expose intermediate debug để biết lỗi nằm ở polygon, scan slice, filter hay fit.

### Lưu ý khi build/test

- Test lane thẳng, lane cong, lane phình, lane đứt đoạn.
- Test turn-lane ngang/chéo.
- Test polygon tự cắt hoặc quá nhỏ phải fail rõ.

### Tiêu chí hoàn thành

- Trích xuất được centerline từ scene vẽ.
- Người dùng nhìn được vì sao line sai.

## Phase 5: Planning Adapter Từ Line/Path

### Mục tiêu

Đưa output line/waypoints vào decision/planning hiện tại qua ROS để test chọn lane và lập plan bằng production control node.

### Việc cần làm

- Publish telemetry real-world hoặc replay output IPM vào input của `control_node`.
- Publish `/avs/route_intent` từ UI/backend.
- Subscribe `/avs/lane_state` và `/avs/control_error`.
- Ghi lại `PathObservationBuilder`, `TrajectoryPlanner`, `TrajectoryNormalizer`, `TrajectoryManager` thông qua debug output của node thật.
- Output:
  - selected lane id
  - candidate trajectory
  - normalized trajectory
  - committed active trajectory
  - replan reason
  - blocked reason
  - control error

### Lưu ý khi code

- Không copy logic planner sang simulator.
- Vì planner còn nằm trong `control_node.cpp`, source-of-truth giai đoạn đầu là chạy `control_node` thật trong ROS simulation.
- Phải giữ state giữa frame khi replay để test memory đúng.
- Python harness trong `test/decision_system` chỉ dùng để assert nhanh hoặc so sánh, không dùng làm planner chính cho simulator.

### Lưu ý khi build/test

- Chạy lại các fixture trong `test/decision_system` để đối chiếu logic unit.
- Chạy ROS graph simulation để xác nhận output thực từ `/avs/lane_state` và `/avs/control_error`.
- Thêm fixture simulator tương đương các scenario UI.
- Nếu ROS graph khó chạy trong test tự động, vẫn có thể dùng Python harness cho unit/CI phụ, nhưng không được thay thế run ROS simulation trong quá trình đánh giá behavior chính.

### Tiêu chí hoàn thành

- Từ scene vẽ, simulator tạo được active trajectory và control error như runtime.
- Có thể tái hiện bug chọn lane/lập plan mà không cần camera.

## Phase 6: Multi-Frame Replay Và Scenario Editor

### Mục tiêu

Test memory, dropout, ID swap và hysteresis qua chuỗi frame.

### Việc cần làm

- Timeline frame:
  - duplicate frame
  - edit frame
  - enable/disable object theo frame
  - đổi id theo frame để test ID instability
  - thay route intent tại frame bất kỳ
- Playback:
  - step frame
  - play/pause
  - reset manager state
  - export run report
- Metrics:
  - selected lane switch count
  - trajectory kind switch count
  - replan count
  - invalid frame count
  - jitter `epsilon_x_mm`
  - jitter `theta_rad`

### Lưu ý khi code

- State manager phải reset được rõ ràng giữa các run.
- Scenario file phải lưu cả route intent sequence.
- Không dùng animation frontend để giả lập logic; backend phải chạy từng frame deterministically.
- Replay chính phải đưa từng frame qua ROS synthetic inference publisher để giữ đúng state/timing của production nodes.

### Lưu ý khi build/test

- Test dropout 1-5 frame và dropout dài hơn hold window.
- Test đang lane-change thì target lane biến mất ngắn hạn.
- Test main lane ID đổi nhưng hình học gần như cũ.

### Tiêu chí hoàn thành

- Simulator bắt được lỗi memory/replan mà unit one-frame không bắt được.

## Phase 7: Regression Fixture Và CLI Runner

### Mục tiêu

Biến scenario từ UI thành regression test chạy không cần browser.

### Việc cần làm

- CLI:

```bash
python tools/local_post_inference_simulator/backend/run_scenario.py fixtures/follow_main_curve.json
```

- Output report JSON:
  - per-frame decision
  - control error
  - metrics
  - pass/fail expected assertions
- Scenario expected assertions:
  - expected selected lane
  - max lane switch count
  - max jitter
  - expected blocked state
  - expected trajectory kind

### Lưu ý khi code

- UI và CLI phải dùng cùng backend pipeline.
- Assertions nên bắt hành vi chính, không quá chặt vào từng pixel nếu thuật toán smoothing thay đổi.

### Lưu ý khi build/test

- Tích hợp vào `pytest` ở mức lightweight.
- Full replay có thể để marker riêng nếu chậm.

### Tiêu chí hoàn thành

- Mỗi bug planning có thể lưu thành scenario regression.
- Refactor decision/trajectory có test bảo vệ rõ hơn.

## Phase 8: Tích Hợp Với Refactor Roadmap

### Mục tiêu

Dùng simulator làm công cụ bảo vệ các phase trong `docs/decision_trajectory_refactor_roadmap.md`.

### Việc cần làm

- Mỗi phase refactor phải có ít nhất một scenario simulator tương ứng.
- Khi tách module production, simulator adapter chuyển từ harness tạm sang shared module thật.
- Dashboard chính chỉ consume debug output, không nhập editor simulator.

### Lưu ý khi code

- Không để simulator trở thành dependency runtime của robot.
- Không để UI simulator quyết định behavior khác production.
- Nếu phát hiện mismatch giữa simulator và robot, ưu tiên sửa shared module hoặc adapter, không patch riêng UI.

### Tiêu chí hoàn thành

- Simulator trở thành công cụ regression chính cho post-inference pipeline.
- Refactor planner có thể kiểm chứng nhanh bằng scenario vẽ tay.

## Thứ Tự Ưu Tiên Triển Khai

### Ưu tiên 1: Sửa contract label và ROS entrypoint

Làm trước:

1. Xác nhận và gom label mapping theo model hiện tại.
2. (Đã hoàn thành) Sửa mismatch `turn-lane = 10` trong IPM sang `turn-lane = 17`.
3. Xác định topic input mà synthetic inference node sẽ publish vào.
4. Xác định topic output cần subscribe:
   - output IPM/real-world telemetry
   - `/avs/lane_state`
   - `/avs/control_error`

Lý do: nếu label và ROS entrypoint sai, mọi UI/simulator phía trên đều tạo dữ liệu không đi đúng pipeline thật.

### Ưu tiên 2: ROS synthetic inference node tối thiểu

Làm tiếp:

1. Tạo scenario JSON tối thiểu một frame.
2. Tạo node/publisher giả lập output inference theo format hiện tại.
3. Publish object `main-lane`, `other-lane`, `turn-lane`, marking vào ROS.
4. Chạy qua IPM và control node thật.

Lý do: chứng minh đường ống ROS hoạt động trước khi đầu tư UI.

### Ưu tiên 3: UI vẽ scene và export/import

Làm sau khi ROS pipeline tối thiểu chạy được:

1. Canvas vẽ polygon/polyline.
2. Class panel theo label mapping.
3. Route intent panel.
4. Export/import scenario JSON.
5. Nút "Run in ROS" gọi backend bridge.

Lý do: UI chỉ có giá trị khi dữ liệu vẽ được chạy qua pipeline thật.

### Ưu tiên 4: Overlay debug và telemetry viewer

Làm khi đã có output ROS:

1. Hiển thị mask/object synthetic.
2. Hiển thị BEV/world output từ IPM.
3. Hiển thị centerline/waypoints.
4. Hiển thị candidate/committed trajectory từ `/avs/lane_state`.
5. Hiển thị `/avs/control_error`.

Lý do: mục tiêu chính của simulator là quan sát sai ở tầng nào.

### Ưu tiên 5: Multi-frame replay và regression

Làm cuối MVP:

1. Timeline nhiều frame.
2. Dropout/ID swap/noise controls.
3. Replay deterministically qua ROS.
4. Export report và assertion.
5. Tích hợp pytest/CI nhẹ nếu phù hợp.

Lý do: memory/hysteresis chỉ kiểm chứng được sau khi single-frame path đúng.

## Thứ Tự Đề Xuất Triển Khai Các Phase

Thứ tự dưới đây là thứ tự triển khai khuyến nghị. Không nên làm đúng theo số phase một cách máy móc nếu dependency chưa sẵn sàng; ưu tiên phải là chứng minh ROS pipeline chạy đúng trước, rồi mới mở rộng UI.

### Bước 1: Phase 0

Làm đầu tiên.

Mục tiêu:

- Chốt label mapping.
- (Đã hoàn thành) Sửa mismatch `turn-lane = 10` sang `turn-lane = 17` trong IPM.
- Chốt topic input/output cho synthetic inference.
- Chốt calibration dùng chung.

Lý do:

- Đây là contract nền. Nếu sai, mọi phase sau đều có thể cho kết quả sai dù UI hoạt động.

### Bước 2: Phase 2, bản tối thiểu không UI

Làm ngay sau Phase 0.

Mục tiêu:

- Tạo scenario JSON tay.
- Tạo synthetic inference publisher/node.
- Publish object list theo format output model hiện tại.
- Chưa cần canvas editor.

Lý do:

- Cần chứng minh có thể đưa dữ liệu giả vào đúng đầu pipeline ROS trước khi đầu tư giao diện.

### Bước 3: Phase 3

Làm sau khi synthetic inference publisher có thể phát dữ liệu.

Mục tiêu:

- Chạy production `ipm_transform_node`.
- Dùng calibration chung.
- Subscribe output world/realworld telemetry.
- Hiển thị hoặc log world polygon/waypoint cơ bản.

Lý do:

- Đây là điểm kiểm tra quan trọng nhất cho post-inference geometry. Nếu IPM chưa đúng, planner phía sau không có dữ liệu đáng tin.

### Bước 4: Phase 5, bản ROS tối thiểu

Làm sau khi IPM output ổn.

Mục tiêu:

- Chạy production `control_node`.
- Publish `/avs/route_intent`.
- Subscribe `/avs/lane_state` và `/avs/control_error`.
- Kiểm tra một scene đơn có sinh active trajectory/control error.

Lý do:

- Đây là end-to-end post-inference path tối thiểu: synthetic inference -> IPM -> control node.

### Bước 5: Phase 4

Làm sau khi có output IPM và trước khi mở rộng scenario phức tạp.

Mục tiêu:

- Expose debug centerline extraction.
- Hiển thị polygon world, scan midpoint, filtered midpoint, final waypoint.
- Đặc biệt kiểm tra `turn-lane` label `20` có được quét/fitting đúng.

Lý do:

- Nếu line sai, cần biết sai ở polygon, scan, filter, fit hay planner.

### Bước 6: Phase 1

Làm sau khi pipeline ROS tối thiểu đã chứng minh chạy được.

Mục tiêu:

- Xây UI canvas.
- Vẽ polygon/polyline.
- Chọn class/instance/confidence.
- Export/import scenario.
- Nút "Run in ROS" gọi backend bridge.

Lý do:

- UI có giá trị khi nó điều khiển được pipeline thật. Làm UI quá sớm dễ tạo công cụ đẹp nhưng chưa kiểm chứng được runtime.

### Bước 7: Phase 6

Làm sau khi single-frame UI + ROS run ổn.

Mục tiêu:

- Timeline nhiều frame.
- Dropout, ID swap, enable/disable object.
- Replay qua ROS synthetic publisher.
- Reset state giữa run.

Lý do:

- Memory, hysteresis và replan policy chỉ bộc lộ qua chuỗi frame.

### Bước 8: Phase 7

Làm sau khi multi-frame replay ổn.

Mục tiêu:

- CLI runner không cần browser.
- Report JSON.
- Expected assertions.
- Tích hợp pytest nhẹ.

Lý do:

- Khi scenario đã ổn, cần biến nó thành regression để bảo vệ refactor.

### Bước 9: Phase 8

Làm xuyên suốt nhưng hoàn thiện sau Phase 7.

Mục tiêu:

- Gắn simulator vào roadmap refactor.
- Mỗi phase refactor có scenario bảo vệ.
- Chuyển adapter tạm sang shared production module khi core được tách.

Lý do:

- Simulator không chỉ là tool debug, mà là regression harness cho refactor decision/trajectory.

### Thứ Tự Rút Gọn Cho MVP

Nếu cần MVP nhanh nhất, làm theo chuỗi:

```text
Phase 0
-> Phase 2 minimal
-> Phase 3
-> Phase 5 minimal
-> Phase 4 debug overlay
-> Phase 1 UI
```

Chưa làm trong MVP đầu:

- Phase 6 multi-frame editor
- Phase 7 CLI regression
- phần hoàn chỉnh của Phase 8

## Đề Xuất Stack

### Khuyến nghị

- Backend/UI bridge: FastAPI + Python, local-only, chạy port riêng.
- Frontend: HTML/CSS/JS thuần với canvas.
- Scenario: JSON.
- Optional image/mask processing: OpenCV + NumPy.
- ROS simulation:
  - synthetic inference publisher/node
  - production `ipm_transform_node`
  - production `control_node`
  - route intent publisher
  - telemetry/lane_state/control_error subscribers
- Planning source-of-truth:
  - giai đoạn đầu và MVP dùng C++ `control_node` thật trong ROS
  - Python harness chỉ dùng cho unit/regression đối chiếu
  - mục tiêu dài hạn là shared C++ core để ROS node và simulator cùng gọi dễ hơn

### Lý do

- Repo đã có FastAPI dashboard nên dễ reuse cách chạy local.
- HTML canvas đủ cho vẽ polygon/polyline.
- Python thuận tiện để rasterize mask, load JSON, tính metric và chạy regression.
- ROS-first giúp kết quả mô phỏng phản ánh đúng runtime thật, đặc biệt các lỗi topic/schema/state.
- Tách khỏi dashboard chính để không ảnh hưởng vận hành robot.

## Rủi Ro Và Cách Giảm

- Rủi ro: simulator dùng logic khác production.
  - Giảm bằng cách chạy production ROS node thật trong mode simulation.
- Rủi ro: label mapping sai.
  - Giảm bằng Phase 0 và một file mapping dùng chung.
- Rủi ro: ROS graph simulation phức tạp hơn local-only.
  - Giảm bằng synthetic inference node tối thiểu trước, UI làm sau.
- Rủi ro: UI vẽ polygon quá lý tưởng, không giống mask nhiễu.
  - Thêm noise/dropout/erosion/dilation ở phase sau.
- Rủi ro: test một frame không bắt lỗi memory.
  - Làm multi-frame replay từ Phase 6.
- Rủi ro: tool quá phức tạp ngay từ đầu.
  - MVP chỉ cần polygon editor, route intent, run pipeline, overlay centerline/trajectory.

## MVP Đề Xuất

MVP nên gồm:

1. Canvas vẽ polygon/polyline cho lane/marking.
2. Export/import scenario JSON.
3. Rasterize label mask và object list.
4. Publish object list vào ROS synthetic inference topic.
5. Chạy production IPM + control node trong ROS simulation.
6. Hiển thị centerline, active trajectory, selected lane, control error.

Chưa cần:

- timeline phức tạp
- mô phỏng động học xe
- UI styling cầu kỳ

## Quyết Định Cho Các Câu Hỏi Cũ

- Simulator ưu tiên chạy trong ROS, không phải local-only pipeline.
- Label mapping dùng theo model hiện tại; `turn-lane = 20` là giá trị đúng cho model/control hiện tại.
- Calibration dùng chung production calibration, mặc định `config/calibration.json`.
- Output sau inference dùng format object/polygon/mask metadata hiện tại của model/pipeline.
- Planner source-of-truth là C++ `control_node` thật trong ROS. Python harness chỉ là công cụ test phụ.

## Trạng Thái Triển Khai (Cập Nhật Sau Review)

Hệ thống hiện tại đã đáp ứng phần lớn MVP và nhiều phase chính, nhưng chưa đạt đầy đủ toàn bộ mục tiêu của plan. Điểm mạnh là simulator đã tách khỏi dashboard, có UI riêng, backend FastAPI, ROS bridge, fixture, CLI runner và test.

### Đã đáp ứng tốt (Đã xác minh)
- **Label `turn-lane = 20`**: Đã dùng nhất quán và chính xác trong IPM. Nhánh xử lý dùng `LABEL_TURN_LANE` (sinh từ `config/label_mapping.json`). Đã có test bảo vệ (`turn_right_two_lanes.json`).
- **Schema**: Có schema đầy đủ cho scenario, frame, route intent, assertions.
- **Post-inference Generator**: Backend đã sinh object payload (direct/rasterized) chuẩn xác (label, class_name, prob, box, polygons, track_id).
- **ROS Bridge**: Backend đã publish/subscribe tốt với các topic thực tế (`/avs/telemetry`, `/avs/route_intent`, `/avs/cmd`, `/avs/telemetry_realworld`, `/avs/lane_state`, `/avs/control_error`).
- **Pipeline Execution**: Đã có IPM-only step qua production node, và full pipeline (route intent -> telemetry -> IPM -> control outputs).
- **UI Canvas**: Hỗ trợ vẽ/edit polygon/polyline, class scope đúng, route intents, duplicate frame, export/import.
- **Regression & CLI**: Có CLI runner và regression assertions chạy tốt.
- **Roadmap Mapping**: Có fixture mapping với refactor roadmap và đã tự ghi nhận gap Phase 7.

### Đã đáp ứng (Cập nhật — đã sửa và xác minh live trong phiên này)
- **Single source-of-truth label mapping**: Đã chuyển thành `config/label_mapping.json` ở repo root + codegen (`generate_label_mapping.py` chạy qua CMake custom command) sinh `avs_perception/label_mapping.hpp`. C++ (`ipm_transform_node.cpp`, `control_node.cpp`, `ncnn_inference_node.cpp`) và Python simulator (`mask_to_objects.py`) đều đọc từ cùng một nguồn. Build `colcon` xác nhận thành công.
- **Synthetic Inference Node**: Đã tách thành process ROS độc lập (`tools/local_post_inference_simulator/ros2/synthetic_inference_node.py`), spawn qua `subprocess.Popen` từ backend FastAPI, CLI (`run_scenario.py`) và test fixture — không còn nhúng trong tiến trình FastAPI. Topic trung gian `/avs/sim/synthetic_payload` tách bridge khỏi việc giả lập metadata inference.
- **Gap Phase 7 (control_source)**: Đã sửa. Trước đó field `control_source` bị hardcode `"trajectory_manager"` bất kể nhánh nào được dùng (bug che giấu chính xác invariant "không bypass active trajectory" mà Phase 7 cần bảo vệ). Nay `control_source` được set động theo nhánh `use_direct_lookahead` thực tế (`"direct_ipm"` hoặc `"trajectory_manager"`) qua tham số truyền vào `publish_control_error_from_trajectory` + member `last_control_source_` dùng lại trong `publish_lane_state`. Đã xác minh **live trên ROS graph thật** (không chỉ đọc code tĩnh): `follow_main_straight.json` → `control_source: "direct_ipm"`; `turn_right_two_lanes.json` → `control_source: "trajectory_manager"` (đúng như kỳ vọng, direct-IPM không bao giờ kích hoạt khi đang rẽ). Đã thêm assertion `expected_control_source: "direct_ipm"` vào `follow_main_straight.json` để regression-lock.
- **Live ROS Graph End-to-end Test**: Đã thực sự chạy được (trước đây, dù `test_regression.py` có vẻ đã "wire up" auto-spawn node, **toàn bộ 6 test live đều skip âm thầm** do một bug tính đường dẫn — `workspace_dir` thừa một cấp `../` nên trỏ ra ngoài repo, khiến `setup_script` không tồn tại và node không bao giờ được start). Đã sửa: đường dẫn `workspace_dir`, tên tham số calibration (`calibration_file_path` chứ không phải `calibration_path`, và giá trị mặc định `/workspace/config/calibration.json` chỉ đúng trong Docker container — ngoài container phải override), thời gian chờ khởi động node, và một race condition trong `ScenarioRunner` (cờ `is_playing` chuyển `False` sớm hơn lúc `latest_report` thực sự được set, do `step()` gọi `ros2 param set` đồng bộ mất tới 1s trước khi trả quyền điều khiển — đã thêm `ScenarioRunner.wait_until_stopped()` join thread thay vì poll cờ).
- **Fixture vs hành vi thực tế của `control_node`**: Sau khi live test chạy thật, 4/6 fixture fail lúc đầu (`follow_main_dropout.json`, `intersection_follow_main.json`, `lane_change_solid_blocked.json`, `turn_right_two_lanes.json`) — đã điều tra và sửa hết, **hiện 6/6 pass live**. Gap thật gồm: 3 fixture có geometry pixel phi vật lý (vắt qua vanishing row của homography nghịch, hoặc 2 lane world-Y quá gần nhau); 1 bug production thật trong `control_node.cpp` (`plan_follow_main()` không bao giờ gán `TrajectoryKind::BLOCKED_FOLLOW_MAIN`, đã fix bằng relabel trong nhánh BLOCKED); 1 bug metric trong `ros_scenario_runner.py` (`replan_count` đếm sai — đếm mọi frame `replan_reason != "none"` thay vì đếm số lần đổi giá trị); và 1 bug hạ tầng test xuyên fixture (nhiều fixture dùng chung object id generic như `main_lane_1`, khiến state smoothing theo-track-id trong `ipm_transform_node.cpp` rò rỉ giữa các fixture không liên quan trong cùng session — đã fix bằng cách prefix id theo từng fixture). Chi tiết từng case ở `docs/local_post_inference_simulator/scenario_refactor_mapping.md` (mục "Live Regression Status").

### Đã đáp ứng (Cập nhật — phiên làm việc tiếp theo, đã build và xác minh live trong browser thật)
- **Debug Trajectory Pipeline**: Đã đóng gap. `ros_scenario_runner.py` giờ lưu `lane_state` (kèm `debug_trajectories`) vào `_latest_ipm_output` sau mỗi full `step()`. `ipm_adapter.py` có `draw_debug_trajectories()` vẽ trực tiếp 3 stage `candidate`/`normalized`/`committed` (điểm world mm lấy nguyên từ `control_node`, không tính lại) lên ảnh BEV đã có, với màu/style riêng (candidate = cam đứt nét, normalized = tím đứt nét, committed = xanh lá liền nét). `GET /api/ipm/bev` nhận thêm 3 query param `show_candidate_trajectory`/`show_normalized_trajectory`/`show_committed_trajectory`; có thêm `GET /api/lane_state/latest` để lấy raw payload. Frontend (`index.html`/`app.js`) có 3 checkbox tương ứng kèm legend màu, mặc định bật `committed`. Đã xác minh **live**: chạy thật `ipm_transform_node` + `control_node` (`ROS_DOMAIN_ID=57`, calibration override `/home/goln/SimpleSysIDV/config/calibration.json` vì default `/workspace/...` chỉ đúng trong Docker) + FastAPI backend, load fixture `turn_right_two_lanes.json`, step qua ROS thật ra `control_source: trajectory_manager` và `debug_trajectories` có đủ 3 stage 15 điểm mỗi stage; ảnh BEV JPEG trả về từ `/api/ipm/bev` cho thấy candidate (cam đứt nét) tách khỏi committed (xanh) trước khi hội tụ — bằng chứng hình học thật, không phải giả lập. Dùng Playwright (Chromium headless) mở `http://localhost:8001/`, import scenario, bấm Load/Step qua UI thật, tick 3 checkbox, xác nhận `src` của `<img id="bev-image">` đổi đúng query param và ảnh render đủ 3 màu (screenshot lưu lại). Không sửa `control_node.cpp`/`ipm_transform_node.cpp`; simulator chỉ đọc lại field đã publish sẵn.
  - **Review fix (không dùng số liệu tĩnh)**: workflow `Step IPM` (`/api/scenarios/step_ipm`, không chạy `control_node`) trước đó khiến 3 checkbox trajectory bật lên mà không vẽ được gì, dễ hiểu nhầm là production không publish `debug_trajectories`. Đã sửa: `step_ipm()` giờ set tường minh `lane_state: None` (thay vì thiếu hẳn key), và `draw_debug_trajectories()` vẽ hint đỏ trực tiếp lên ảnh BEV khi checkbox bật nhưng không có `debug_trajectories` để vẽ, giải thích rõ nguyên nhân (cần Step/Play qua `control_node`, không phải Step IPM). Thêm test `test_render_bev_image_trajectory_overlay_hints_when_no_debug_data` (`test_ipm_adapter.py`) làm regression guard.
- **Multi-frame Replay Controls (một phần)**: Thêm control chuyên dụng cho ID-swap: double-click ô Id trong bảng object (`data-editable="id"`) mở prompt đổi id — chỉ áp dụng cho object trong frame hiện tại, giữ nguyên hình học, dùng để mô phỏng tracker đổi id giữa các frame mà không cần sửa tay JSON export. Thêm chỉ báo "frame đang chạy" (`.frame-chip.running`, viền đỏ), tách biệt với frame đang chọn để edit (`.frame-chip.active`, viền xanh) — verify bằng Playwright cho thấy 2 chip khác màu cùng lúc khi Play một fixture 2-frame trong lúc đang chọn frame khác để sửa. Chưa làm: timeline dạng scrub/kéo-thả trực quan (kéo con trỏ để tua) — vẫn chỉ có click từng frame chip hoặc Step/Play tuần tự.
  - **Review fix (bug hiển thị thật, không chỉ UX gap)**: chỉ báo ban đầu tô đỏ theo `runner.current_frame_idx`, nhưng `step()` tăng `current_frame_idx` ngay khi vừa xong (trước khi trả quyền điều khiển), và `get_status()` dùng chung `self._lock` với `step()` (giữ khoá suốt round-trip ROS) nên **không bao giờ** quan sát được trạng thái giữa chừng — poll luôn thấy state *sau* step, tức chip đỏ trỏ sang frame kế tiếp chứ không phải frame đang chạy. Đã sửa bằng field mới `running_frame_idx`: set trong `step()` ngay khi resolve frame (trước khi làm ROS I/O), đọc trong `get_status()` **không qua `self._lock`** (attribute read đơn giản atomic dưới GIL) nên không bị chặn bởi step() đang chạy dở; reset về `None` trong `stop()`. Xác minh live: poll trực tiếp `/api/scenarios/status` mỗi 80ms khi Play fixture 10-frame (`follow_main_dropout.json`) qua ROS thật cho kết quả `running_frame_idx` luôn bằng `current_frame_idx - 1` ở mọi mẫu (đúng frame vừa xử lý, không lệch/không nhảy cóc), và về `None` khi dừng — không còn là số liệu tĩnh, đã thấy hành vi đúng qua nhiều frame liên tiếp. Thêm test `test_running_frame_idx_reflects_frame_in_flight_not_next_frame` (`test_control_adapter.py`) làm regression guard cho semantics này.
- **Regression test coverage cho phần mới (review fix)**: Trước đó chỉ có Playwright thủ công, không có pytest chống regression cho: query param mới của `/api/ipm/bev`, dashed overlay theo từng stage, contract `lane_state` giữa `step()`/`step_ipm()`, và semantics `running_frame_idx`. Đã thêm 5 test mới: `test_ipm_adapter.py::test_render_bev_image_trajectory_overlay_only_draws_requested_stage` (lọc đúng stage theo checkbox), `test_render_bev_image_trajectory_overlay_no_op_when_all_flags_off`, `test_render_bev_image_trajectory_overlay_hints_when_no_debug_data`; `test_control_adapter.py::test_step_stores_lane_state_with_debug_trajectories_for_bev_overlay`, `test_step_ipm_reports_lane_state_none_not_missing_key`, `test_running_frame_idx_reflects_frame_in_flight_not_next_frame`. Toàn bộ dùng `mock_ros_bridge` (không cần ROS thật), đã xác nhận từng test fail đúng cách khi revert fix tương ứng (không phải test tautological). ID-swap/export vẫn chỉ có Playwright + xác nhận thủ công vì repo chưa có JS test runner — không dựng thêm hạ tầng test JS chỉ cho 2 hàm nhỏ này.

### Review round 2 (sau khi phiên trên đã merge) — 2 finding, cả 2 đã sửa và verify live
- **`get_status()` vẫn race, response tự mâu thuẫn**: `running_frame_idx` chụp trước `self._lock`, nhưng field `is_playing` trong response lại đọc `self.is_playing` **lần thứ hai**, bên trong `_lock`. Nếu `stop()`/`pause()` đổi cờ giữa 2 lần đọc đó, response có thể ra `is_playing: false` kèm `running_frame_idx` cũ còn sót lại — 2 field cùng response nhưng thuộc 2 thời điểm khác nhau. Đã sửa `get_status()` (`ros_scenario_runner.py`) để chỉ đọc `self.is_playing` **một lần duy nhất** (`is_playing_snapshot`), dùng snapshot đó cho cả điều kiện gate `running_frame_idx` lẫn field `is_playing` trong response — không còn đọc lại lần 2 trong `_lock`. Đồng thời thêm phòng thủ lớp 2 ở frontend (`app.js`): `nextRunningIdx` giờ gate thêm `runner.is_playing === true`, không chỉ dựa `running_frame_idx != null`, để dù backend có regress cũng không tô nhầm chip đỏ. Test mới `test_control_adapter.py::test_get_status_is_playing_and_running_frame_idx_never_contradict` mô phỏng race bằng cách swap class của runner sang subclass có `is_playing` là property trả về giá trị khác nhau mỗi lần đọc (`iter([True, False, False, False])`) — deterministic, không phụ thuộc timing thread thật. Đã xác nhận rigor: revert tạm fix, test fail đúng kiểu `assert 0 is None` (tái hiện đúng bug `is_playing: False` + `running_frame_idx: 0`), sau đó restore lại fix, test pass.
- **Hint "thiếu debug_trajectories" gán sai nguyên nhân cho mọi trường hợp `drawn_any == False`**: Trước đó chỉ có 1 hint chung, luôn đổ lỗi "đang dùng Step IPM", kể cả khi `lane_state` có tồn tại nhưng thiếu field `debug_trajectories`, hoặc field đó có nhưng stage đang bật không có điểm hợp lệ trong frame này — 2 case sau không liên quan gì tới Step IPM. Đã sửa `draw_debug_trajectories()` (`ipm_adapter.py`) tách 3 hint riêng theo nguyên nhân thật: (1) `lane_state is None` → đúng là do Step IPM chưa chạy `control_node`; (2) `lane_state` tồn tại nhưng `debug_trajectories` không phải list → hint trung lập "kiểm tra log control_node" (**không** còn đổ cho `control_source = direct_ipm` như bản nháp đầu — đã kiểm tra ngược `control_node.cpp`: `debug_trajectories` được push ở nhánh trajectory-planning chạy **gần như mọi decision cycle**, độc lập với việc `publish_control_error_from_trajectory` có bypass sang `"direct_ipm"` hay không cho control_error của frame đó; xác nhận **live**: fixture `follow_main_straight.json` có `control_source: "direct_ipm"` nhưng `debug_trajectories` vẫn đủ 3 stage — nên giả thuyết ban đầu sai, đã bỏ để tránh hint sai sự thật); (3) `debug_trajectories` là list nhưng không có điểm hợp lệ cho stage đang bật → hint "không có điểm hợp lệ cho stage đã chọn". Test mới `test_ipm_adapter.py::test_render_bev_image_trajectory_overlay_hint_distinguishes_cause` phân biệt 3 case bằng so sánh pixel (không OCR text). Verify live: ảnh BEV thật từ `/api/ipm/bev` sau `step_ipm()` render đúng hint case (1) ("dang dung Step IPM, control_node chua chay"); ảnh BEV thật sau `step()` full ROS trên `follow_main_straight.json` (`control_source: direct_ipm`) vẽ đủ 3 stage trajectory bình thường, không hiện hint nào — đúng như phát hiện ở trên.
- **Regression suite sau review round 2**: `pytest -q test/local_post_inference_simulator/ test/decision_system` → 64 passed. `node --check app.js` → OK. `pytest -m ros test_regression.py` (live ROS graph thật, `ROS_DOMAIN_ID=57`) → 6/6 passed, không bị ảnh hưởng bởi 2 fix trên.

### Chưa đáp ứng đầy đủ (Cần bổ sung/hoàn thiện)
- **Timeline scrub/kéo-thả**: Chưa có thanh trượt kéo-thả để tua nhanh giữa nhiều frame; hiện chỉ click từng frame chip hoặc Step/Play tuần tự.
