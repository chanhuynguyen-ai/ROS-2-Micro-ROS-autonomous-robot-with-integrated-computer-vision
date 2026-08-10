# Kế Hoạch Hệ Thống Quyết Định

## Tóm tắt

- Xây dựng lớp quyết định nằm giữa `/avs/telemetry_realworld` và `/avs/control_error`.
- Hệ thống không đổi bộ điều khiển hiện có; decision layer chỉ chọn và sinh ra một `active trajectory` đã được làm mượt, sau đó quy đổi về `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`, `curvature_inv_mm`.
- Ý định lái lấy từ node/topic route riêng, không mở rộng `/avs/cmd`.
- Mỗi thời điểm quyết định chỉ được publish một line duy nhất cho xe bám. Các lane/trajectory ứng viên chỉ được dùng nội bộ để chọn.

## Public Interfaces

- Thêm topic route intent mới: `/avs/route_intent` dạng JSON.
- Schema tối thiểu:

```json
{
  "intent": "follow_main | turn_right | turn_left | lane_change_left | lane_change_right",
  "source": "manual | planner",
  "seq": 1
}
```

- Không cần intent `straight`.
  - Nếu xe tiếp tục đi thẳng, đi theo đường cong, hoặc đi tiếp qua ngã tư thì vẫn là `follow_main`.
  - Theo quy tắc gán nhãn, `main-lane` là lane hiện tại/ego lane và cũng là mục tiêu mặc định để xe tiếp tục bám.

- Giữ output `/avs/control_error` hiện có:
  - `lane_state`
  - `target_label`
  - `epsilon_x_mm`
  - `epsilon_y_mm`
  - `theta_rad`
  - `curvature_inv_mm`
  - `lookahead_d_mm`

- Mở rộng debug `/avs/lane_state` để thêm:
  - `route_intent`
  - `decision_state`
  - `selected_lane_id`
  - `blocked_by_marking`
  - `trajectory_kind`
  - `active_trajectory_points`

## Thay đổi triển khai

- Trong `control_node.cpp`, thay logic chọn object trực tiếp bằng `DecisionEngine`:
  - Subscribe `/avs/telemetry_realworld` và `/avs/route_intent`.
  - Phân loại trạng thái: `FOLLOW_MAIN`, `TURN_RIGHT`, `TURN_LEFT`, `LANE_CHANGE`, `BLOCKED`, `RECOVERY`.
  - Chọn lane theo intent, vạch kẻ, và hình học BEV.
  - Không dùng `stop-line` làm điều kiện kích hoạt hoặc phát hiện giao lộ trong phase hiện tại.

- Chuẩn hóa dữ liệu lane/marking từ telemetry:
  - Mỗi lane có centerline/waypoints trong vehicle frame `(X,Y)`.
  - `main-lane` và `other-lane` có thể được fit theo đường dọc `x(y)`.
  - `turn-lane` có thể được fit theo hướng ngang/chéo, nhưng khi đưa vào decision layer sẽ được coi là polyline chung.
  - Marking solid gồm `solid-white`, `solid-yellow`, `double-solid-white`.
  - Marking dashed gồm `dashed-white`, `dashed-yellow`.

## Quy tắc quyết định

### Follow Main

- `follow_main` là mặc định khi không có intent rẽ hoặc chuyển làn.
- Xe bám `main-lane` trong các trường hợp:
  - Đường thẳng.
  - Đường cong.
  - Tiếp tục qua ngã tư.
- Tại ngã tư, theo quy tắc gán nhãn, `main-lane` hiện tại và `main-lane` phía bên kia giao lộ không được vẽ liền mạch qua vùng giao lộ. Decision layer sẽ sinh một line smooth nối từ `main-lane` hiện tại sang `main-lane` mục tiêu phía trước.
- Nếu chỉ thấy `main-lane` hiện tại và chưa thấy rõ `main-lane` phía trước, tiếp tục follow `main-lane` hiện tại và không tạo line nối xa không chắc chắn.

### Turn Right

- Với intent `turn_right`, chọn `turn-lane` ở phía mép phải BEV/vehicle frame.
- Nếu có 2 `turn-lane` hợp lệ cho hướng rẽ phải, chọn lane gần hơn để nhập đúng làn.
- Sinh một line smooth duy nhất nối từ lane hiện tại sang `turn-lane` đã chọn.

### Turn Left

- Với intent `turn_left`, chọn `turn-lane` tương ứng hướng rẽ trái.
- Nếu có 2 `turn-lane` hợp lệ, chọn lane xa hơn để nhập đúng làn.
- Sinh một line smooth duy nhất nối từ lane hiện tại sang `turn-lane` đã chọn.

### Lane Change

- Với `lane_change_left` hoặc `lane_change_right`, xác định `other-lane` mục tiêu theo vị trí lateral so với `main-lane`.
- Trước khi sinh line chuyển làn, kiểm tra marking nằm giữa `main-lane` và `other-lane` mục tiêu ở vùng gần xe.
- Nếu có một trong các marking solid giữa hai lane:
  - Không được chuyển làn.
  - Publish trạng thái `BLOCKED`.
  - Tiếp tục sinh line duy nhất theo `main-lane`.
- Nếu marking giữa hai lane là dashed, hoặc không có solid ngăn cách rõ ràng:
  - Cho phép chuyển làn.
  - Sinh một line smooth duy nhất nối từ `main-lane` sang `other-lane` mục tiêu.

### T-Junction

- Không phụ thuộc vào `stop-line` để phát hiện ngã ba chữ T.
- Phát hiện T-junction bằng hình học lane:
  - `main-lane` phía trước không tiếp tục hợp lệ.
  - Có các `turn-lane` chạy ngang/chéo phía trước.
  - Không có `main-lane` đối diện hợp lệ như ngã tư.
- Nếu intent là `turn_right`, chọn `turn-lane` gần hơn theo mép phải BEV.
- Nếu intent là `turn_left`:
  - Chỉ cho phép khi không có solid marking ngăn hướng nhập/rẽ.
  - Khi hợp lệ, chọn `turn-lane` xa hơn để đi đúng làn.
- Nếu `turn_left` bị chặn bởi solid marking, decision layer không sinh line rẽ trái. Hệ thống chuyển sang `BLOCKED` hoặc fallback sang `turn_right` nếu higher-level planner cho phép.

## Luật Hợp Lệ Lane Theo Vạch Vàng (Solid-Yellow / Dashed-Yellow)

- Chỉ được vẽ path cho lane (không phân biệt `main-lane`, `other-lane`, `turn-lane`) nằm **bên phải** vạch vàng khi vạch có hướng dọc theo chiều cao frame, hoặc nằm **bên dưới** vạch vàng khi vạch nằm ngang theo chiều rộng frame.
- Cả `solid-yellow` và `dashed-yellow` đều là vạch phân định hai chiều (cập nhật 2026-07-18). Khác biệt duy nhất: `dashed-yellow` cho phép intent `lane_change_*` chủ động vượt qua (lane đích soft-illegal không bị lọc khi lane-change active); `solid-yellow` không bao giờ. Turn intent KHÔNG được miễn với cả hai loại vạch.
- An toàn trên luật cứng: xe đang ở bên sai vạch vàng (kể cả do lane_change chủ động qua `dashed-yellow`) thì cơ chế tự về làn đúng vẫn giám sát và kéo về sau debounce, bất kể loại vạch. "Làn đúng" xác định bằng hình học vạch vàng (verdict LEGAL), KHÔNG phải nhãn `main-lane` (`main-lane` chỉ cho biết làn hiện tại của xe).
- Hai case dọc/ngang là trường hợp riêng của một luật thống nhất: định hướng vạch theo quy ước (dọc/chéo → hướng ra xa xe `d.y > 0`; gần ngang → hướng sang phải `d.x > 0`), lane hợp lệ khi nằm bên phải vector hướng (signed side test).
- Gate áp lên lane đích của mọi intent; KHÔNG áp lên đường transition nối (đường nối khi rẽ được phép đi qua vùng giao lộ).
- Không thấy `solid-yellow`: giữ kết quả đánh giá gần nhất trong hold window; quá hold hoặc chưa từng thấy → permissive (không lọc lane nào).
- Lane đang bám bị đánh giá không hợp lệ ổn định (qua debounce) và tồn tại lane hợp lệ: sinh transition tự động về lane hợp lệ gần nhất (`legality_return`). Không bao giờ tự vứt path khi không có lane hợp lệ thay thế.
- Chi tiết hình học, memory, và tích hợp: `../plans/plan_F_solid_yellow_legality_gate.md`.

## Chính sách Stop-Line

- `stop-line` chỉ dùng cho logic dừng xe khi có hệ thống đèn giao thông, biển báo, hoặc rule ưu tiên giao thông.
- Phase hiện tại chưa xây dựng traffic light/sign recognition, nên `stop-line` không tham gia:
  - Kích hoạt rẽ.
  - Phát hiện ngã tư.
  - Phát hiện ngã ba chữ T.
  - Quyết định chuyển làn.
- Tại các giao lộ không có `stop-line`, decision layer vẫn phải quyết định dựa trên hình học lane/turn-lane và route intent.

## Sinh Trajectory Mượt

- Tại mỗi frame/quyết định, chỉ có đúng một `active trajectory`.
- `active trajectory` có thể được tạo từ:
  - Lane hiện tại.
  - Một lane mục tiêu duy nhất.
  - Một đoạn nối transition smooth giữa hai lane.
- Output cuối cùng cho controller luôn là một polyline/curve duy nhất, không publish nhiều line song song.
- Đoạn nối dùng cubic Bezier hoặc cubic Hermite với ràng buộc tiếp tuyến tại điểm đầu/cuối để tránh gãy góc.
- Sample trajectory theo bước cố định trong tọa độ mm, ví dụ 50-100 mm.
- Tính lookahead point trên `active trajectory` theo `compute_lookahead_d()`.
- Tính output controller từ `active trajectory`:
  - `epsilon_x_mm`, `epsilon_y_mm` lấy từ waypoint lookahead.
  - `theta_rad` lấy từ tiếp tuyến tại waypoint lookahead.
  - `curvature_inv_mm` lấy từ đạo hàm/fit cục bộ quanh waypoint.

## Test Plan

- Unit test decision:
  - `follow_main` trên đường thẳng tạo một trajectory theo `main-lane`.
  - `follow_main` trên đường cong vẫn bám `main-lane`, không cần intent `straight`.
  - `follow_main` qua ngã tư nối `main-lane` hiện tại với `main-lane` phía bên kia bằng một trajectory duy nhất.
  - `turn_right` tại ngã tư có 2 `turn-lane` chọn lane gần hơn.
  - `turn_left` tại ngã tư có 2 `turn-lane` chọn lane xa hơn.
  - `lane_change_left/right` bị chặn khi có `solid-white`, `solid-yellow`, hoặc `double-solid-white` giữa `main-lane` và `other-lane`.
  - `lane_change_left/right` được phép khi marking giữa lane là dashed.
  - T-junction không có `stop-line` vẫn quyết định được hướng rẽ dựa trên hình học lane.
  - Mỗi scenario chỉ publish một `active trajectory`.

- Integration test:
  - Feed JSON mẫu vào `/avs/telemetry_realworld` và `/avs/route_intent`, kiểm tra `/avs/control_error` không bị nhảy đột ngột khi đổi từ follow main sang turn/lane-change.
  - Kiểm tra `/avs/lane_state` phản ánh đúng `decision_state`, selected lane và lý do blocked.
  - Kiểm tra các frame có nhiều lane ứng viên nhưng output debug/controller chỉ có một active trajectory.

- Runtime verification:
  - Overlay debug `active trajectory` lên dashboard/video để nhìn được line sau smoothing.
  - Chạy `colcon build --symlink-install` sau khi implement.
  - Chạy test offline bằng video/json mẫu trước khi chạy robot thật.

## Assumptions

- Hệ tọa độ dùng theo tài liệu hiện có: `X > 0` sang phải xe, `Y > 0` phía trước xe.
- Quy tắc chọn `turn-lane` gần/xa dùng không gian BEV sau homography, theo mép frame/vehicle frame.
- `/avs/cmd` tiếp tục dành cho arm/disarm/resume hoặc lệnh hệ thống; route intent dùng `/avs/route_intent`.
- Bộ điều khiển downstream tiếp tục consume `/avs/control_error`; không đổi thuật toán Pure Pursuit/PD trong plan này.
