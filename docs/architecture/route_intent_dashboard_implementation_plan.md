# Kế Hoạch Triển Khai Route Intent Từ Dashboard

## 1. Mục Tiêu

Thay thế hoàn toàn panel `arm/disarm` cũ trên dashboard bằng panel gửi `route intent` từ người dùng để điều khiển hành vi:

- `follow_main`
- `turn_left`
- `turn_right`
- `lane_change_left`
- `lane_change_right`

Mục tiêu vận hành:

- mặc định xe bám `main-lane`
- intent từ người dùng là `latched`
- nếu intent chưa khả thi ở frame hiện tại, hệ thống vẫn giữ intent đó
- trong thời gian chưa khả thi, xe tiếp tục `follow_main`
- khi look-ahead và luật trong [decision_sys.md](/home/goln/SimpleSysIDV/./decision_sys.md:1) cho phép, decision layer mới sinh line nối để rẽ hoặc chuyển làn

## 2. Nguyên Tắc Thiết Kế

### 2.1 Không Dùng Lại `/avs/cmd`

Theo [decision_sys.md](/home/goln/SimpleSysIDV/./decision_sys.md:1):

- `/avs/cmd` không dùng cho route intent
- route intent phải đi qua topic riêng `/avs/route_intent`

Vì vậy plan này sẽ:

- loại bỏ vai trò `arm/disarm` khỏi dashboard
- không tái sử dụng endpoint `POST /api/arm`
- thêm API/backend path mới chỉ để publish vào `/avs/route_intent`

### 2.2 Intent Là Latched

Intent từ UI không phải lệnh tức thời một lần.

Sau khi người dùng bấm:

- intent đang chọn sẽ được giữ làm `current requested intent`
- decision layer liên tục xét lại intent đó ở các frame tiếp theo
- nếu chưa đủ điều kiện hình học hoặc bị luật cấm, xe vẫn bám `main-lane`
- khi điều kiện hợp lệ xuất hiện, decision layer mới thực hiện rẽ hoặc chuyển làn

### 2.3 Luật Điều Khiển Luôn Ưu Tiên

UI chỉ gửi `ý định`.

Decision layer mới là nơi quyết định:

- có được thực hiện intent hay không
- thực hiện ở frame nào
- lane mục tiêu nào được chọn
- có bị `blocked` bởi solid marking hay không

Điều này giữ đúng nguyên tắc:

- người dùng không điều khiển trực tiếp trajectory
- người dùng chỉ yêu cầu hành vi
- hệ thống thực thi theo luật trong `decision_sys.md`

## 3. Hành Vi Mong Muốn

## 3.1 Trạng Thái Mặc Định

Khi hệ thống khởi động:

- intent mặc định là `follow_main`
- dashboard hiển thị `Follow Main` là trạng thái đang chọn
- decision layer bám `main-lane`

## 3.2 Khi Người Dùng Bấm Rẽ Hoặc Chuyển Làn

Ví dụ với `turn_left` hoặc `lane_change_left`:

1. Dashboard gửi intent mới lên backend
2. Backend publish JSON vào `/avs/route_intent`
3. Decision layer lưu intent mới
4. Decision layer tiếp tục đánh giá intent trên mỗi frame telemetry
5. Nếu hiện tại chưa khả thi:
   - không sinh trajectory rẽ/chuyển làn
   - tiếp tục `follow_main`
   - trạng thái debug cần cho thấy intent vẫn đang chờ thực hiện
6. Khi khả thi:
   - chọn lane mục tiêu đúng theo luật
   - sinh một `active trajectory` duy nhất
   - bắt đầu rẽ hoặc chuyển làn

## 3.3 Khi Intent Không Hợp Lệ Trong Nhiều Frame

Nếu intent bị chặn bởi luật hoặc chưa đủ hình học:

- intent vẫn được giữ
- node không tự xóa intent ngay lập tức
- output điều khiển vẫn bám `main-lane`

Ví dụ:

- `lane_change_left` nhưng giữa `main-lane` và `other-lane` là `solid-white`
- `turn_left` nhưng chưa nhìn thấy `turn-lane` hợp lệ

Trong các trường hợp đó:

- UI vẫn hiển thị intent đang được yêu cầu
- debug state nên thể hiện rõ `requested intent` khác với `executed state`

## 3.4 Khi Nào Quay Lại `follow_main`

Vì intent là `latched`, cần cho phép người dùng chủ động trả hệ thống về mặc định.

Do đó panel mới phải có nút:

- `Follow Main`

Nút này có vai trò:

- xóa yêu cầu rẽ/chuyển làn hiện tại
- đưa hệ thống về trạng thái mặc định

## 4. Giao Diện Dashboard Cần Thay Đổi

## 4.1 Thay Panel `arm/disarm` Thành `Route Intent`

Panel cũ:

- tiêu đề `Vehicle Motion Control`
- badge `ARMED/DISARMED`
- 2 nút `ARM` và `DISARM`

Panel mới nên là:

- tiêu đề `Route Intent`
- badge hiển thị intent hiện tại hoặc trạng thái `FOLLOW MAIN`
- 5 nút:
  - `Follow Main`
  - `Turn Left`
  - `Turn Right`
  - `Lane Change Left`
  - `Lane Change Right`

## 4.2 Trạng Thái Nút

Nút nên phản ánh rõ:

- intent nào đang được chọn
- intent nào đang là mặc định

Hành vi UI đề xuất:

- nút intent đang được chọn có trạng thái active
- `Follow Main` active khi không có intent rẽ/chuyển làn
- khi người dùng chọn intent khác, UI đổi active ngay sau khi backend xác nhận publish thành công

## 4.3 Thông Tin Trạng Thái Trong Panel

Panel nên hiển thị ít nhất 3 lớp thông tin:

- `Requested Intent`: intent mà người dùng đang giữ
- `Decision State`: trạng thái thực thi hiện tại do node publish
- `Blocked/Waiting Reason`: lý do chưa thực hiện được intent, nếu có

Thông tin này có thể lấy từ:

- `route_intent`
- `decision_state`
- `blocked_by_marking`
- `selected_lane_id`
- `trajectory_kind`

## 5. Backend Bridge Cần Thay Đổi

## 5.1 Thêm Publisher `/avs/route_intent`

Trong `web_dashboard/backend/main.py`, thêm publisher ROS2 mới:

- topic: `/avs/route_intent`
- type: `std_msgs/msg/String`

Payload JSON theo `decision_sys.md`:

```json
{
  "intent": "turn_left",
  "source": "manual",
  "seq": 12
}
```

## 5.2 Thêm API Mới Cho Dashboard

Thêm endpoint mới, ví dụ:

- `POST /api/route_intent`

Request tối thiểu:

```text
intent=follow_main|turn_left|turn_right|lane_change_left|lane_change_right
```

Việc backend cần làm:

- validate intent
- sinh `seq` tăng dần
- gắn `source = "manual"`
- publish JSON lên `/avs/route_intent`
- trả response cho frontend

## 5.3 Loại Bỏ Hoặc Giảm Vai Trò API `arm`

Vì người dùng xác nhận không cần `arm/disarm` nữa:

- panel `arm/disarm` trên UI sẽ bị thay thế
- endpoint `/api/arm` có thể giữ tạm để không phá code cũ ngay
- nhưng không còn được frontend gọi đến

Trong pha cleanup sau:

- có thể xóa hẳn publisher `/avs/cmd`
- xóa endpoint `/api/arm`

## 6. Frontend Cần Thay Đổi

## 6.1 `index.html`

Thay toàn bộ panel `arm-panel` bằng panel `route-intent-panel`.

Nội dung mới nên có:

- badge trạng thái hiện tại
- mô tả ngắn:
  - xe mặc định bám `main-lane`
  - user intent là latched
  - system chỉ thực thi khi luật và hình học cho phép
- 5 nút intent

## 6.2 `app.js`

Thay logic:

- `btn-arm`
- `btn-disarm`
- `setArmState()`
- `sendArmCmd()`

bằng logic mới:

- `setRequestedIntent()`
- `sendRouteIntent(intent)`
- `setIntentButtonState(intent)`

Frontend cần làm rõ hai loại state:

- state do user yêu cầu
- state do telemetry report từ decision layer

Không nên dùng chỉ một badge cho cả hai nếu điều đó gây nhầm lẫn.

## 6.3 `style.css`

Cần đổi style từ `arm-panel` sang `route-intent-panel`.

Nên dùng layout rõ ràng:

- hàng đầu cho `Follow Main`
- hàng thứ hai cho `Turn Left` và `Turn Right`
- hàng thứ ba cho `Lane Change Left` và `Lane Change Right`

Màu sắc nên phản ánh loại hành vi:

- `Follow Main`: trung tính hoặc xanh lá
- `Turn`: màu cảnh báo định hướng
- `Lane Change`: màu khác với turn để tránh nhầm

## 7. Decision Layer Cần Hỗ Trợ Gì

## 7.1 Subscribe Route Intent

Decision node hoặc `control_node.cpp` cần:

- subscribe `/avs/route_intent`
- parse JSON intent
- lưu `current_route_intent`
- lưu `source`
- lưu `seq` nếu cần chống lặp hoặc debug

## 7.2 Tách `Requested Intent` Và `Executed Behavior`

Đây là phần quan trọng nhất của implementation.

Hệ thống phải phân biệt:

- `requested_intent`: intent do user gửi
- `decision_state`: trạng thái decision thực tế đang chạy

Ví dụ:

- `requested_intent = lane_change_left`
- `decision_state = FOLLOW_MAIN`

được xem là hợp lệ nếu:

- lane change chưa khả thi
- hoặc đang bị chặn bởi solid marking

## 7.3 Khi Nào Được Thực Thi Intent

Triển khai phải bám đúng luật trong `decision_sys.md`:

- `turn_left` và `turn_right` chỉ thực hiện khi có `turn-lane` hợp lệ
- `lane_change_left/right` chỉ thực hiện khi lane mục tiêu tồn tại và không bị solid marking ngăn cách
- nếu chưa hợp lệ thì vẫn `follow_main`

Nói cách khác:

- intent không ép controller chuyển trạng thái ngay khi nhận lệnh
- intent chỉ đặt mục tiêu ưu tiên cho decision layer

## 7.4 Điều Kiện Hoàn Thành Intent

Plan nên định nghĩa sớm tiêu chí sau để tránh hành vi mơ hồ:

- khi chuyển làn xong, có tự về `follow_main` không
- khi rẽ xong, có tự về `follow_main` không

Đề xuất triển khai:

- sau khi maneuver hoàn tất ổn định, decision layer tự reset về `follow_main`
- đồng thời publish trạng thái mới để UI cập nhật

Lý do:

- tránh việc intent cũ bám dai và tái kích hoạt không mong muốn ở khúc cua sau

Nếu muốn giữ intent ngay cả sau khi hoàn thành maneuver thì cần viết thêm một policy riêng, nhưng đó không phải hành vi an toàn mặc định.

## 8. Mở Rộng Telemetry Và Debug

Để UI hiển thị rõ intent latched và trạng thái thực thi, nên mở rộng hoặc xác nhận `/avs/lane_state` có các field:

- `route_intent`
- `decision_state`
- `selected_lane_id`
- `blocked_by_marking`
- `trajectory_kind`
- `active_trajectory_points`

Nên cân nhắc thêm:

- `intent_source`
- `intent_seq`
- `intent_status`

Trong đó `intent_status` có thể là:

- `active`
- `waiting`
- `blocked`
- `executing`
- `completed`

Field này không bắt buộc nhưng rất hữu ích cho UI.

## 9. Kế Hoạch Triển Khai Theo Pha

## 9.1 Pha 1: Chuẩn Hóa Interface Intent

Mục tiêu:

- chốt schema JSON trên `/avs/route_intent`
- chốt danh sách intent hợp lệ

Kết quả:

- backend và decision node dùng cùng schema

## 9.2 Pha 2: Thêm ROS Publisher Và API Ở Backend

Mục tiêu:

- dashboard có thể gửi route intent lên ROS2

Việc cần làm:

- thêm publisher `/avs/route_intent`
- thêm endpoint `POST /api/route_intent`
- sinh `seq`
- validate input

## 9.3 Pha 3: Thay UI `arm/disarm` Bằng UI `Route Intent`

Mục tiêu:

- tận dụng đúng vị trí panel cũ
- bỏ hẳn nút `ARM` và `DISARM`

Việc cần làm:

- sửa `index.html`
- sửa `style.css`
- sửa `app.js`

## 9.4 Pha 4: Kết Nối Decision Layer Với Intent Latched

Mục tiêu:

- decision layer nhận intent mới và giữ nó qua nhiều frame

Việc cần làm:

- subscribe `/avs/route_intent`
- lưu `requested_intent`
- tách `requested_intent` khỏi `decision_state`

## 9.5 Pha 5: Áp Luật Quyết Định Khi Intent Chưa Khả Thi

Mục tiêu:

- luôn ưu tiên luật trong `decision_sys.md`

Việc cần làm:

- nếu chưa đủ lane geometry hoặc bị solid marking chặn:
  - giữ intent
  - tiếp tục follow main
- nếu đủ điều kiện:
  - sinh line nối phù hợp
  - chuyển sang maneuver thực thi

## 9.6 Pha 6: Cập Nhật Telemetry Cho UI

Mục tiêu:

- UI nhìn ra được khác biệt giữa yêu cầu và thực thi

Việc cần làm:

- publish đầy đủ field debug qua `/avs/lane_state`
- frontend render `requested intent`, `decision_state`, `blocked/waiting`

## 9.7 Pha 7: Cleanup Logic Cũ

Mục tiêu:

- loại bỏ phần `arm/disarm` không còn dùng

Việc cần làm:

- gỡ event handler cũ trong frontend
- gỡ publisher `/avs/cmd` khỏi web bridge nếu không còn nơi dùng
- xóa endpoint `/api/arm` sau khi xác nhận không còn dependency

## 10. Các Quyết Định Kỹ Thuật Cần Giữ Nhất Quán

### 10.1 Danh Sách Intent Hợp Lệ

Danh sách chốt:

- `follow_main`
- `turn_left`
- `turn_right`
- `lane_change_left`
- `lane_change_right`

### 10.2 Source Của Intent

Đối với dashboard:

- `source = manual`

Điều này cho phép sau này coexist với planner:

- `source = planner`

### 10.3 Chính Sách Khi Intent Đang Chờ

Chính sách đã chốt:

- intent được giữ lại
- xe tiếp tục `follow_main`
- decision layer chờ đến khi khả thi mới thực hiện

### 10.4 Chính Sách Khi Intent Xong

Đề xuất:

- reset về `follow_main`

Nếu không reset, hệ thống dễ tái thực hiện intent cũ ở ngữ cảnh mới không mong muốn.

## 11. Tiêu Chí Hoàn Thành

Implementation được xem là hoàn thành khi:

- dashboard không còn nút `ARM/DISARM`
- dashboard có panel `Route Intent` với 5 nút
- backend publish được `/avs/route_intent`
- decision layer giữ được intent theo kiểu latched
- khi intent chưa khả thi, xe vẫn bám `main-lane`
- khi intent trở nên khả thi, decision layer sinh trajectory rẽ/chuyển làn theo luật
- UI hiển thị được trạng thái yêu cầu và trạng thái thực thi

## 12. Test Plan

## 12.1 UI Test

Cần kiểm tra:

- bấm từng nút gửi đúng API
- nút active đổi đúng sau khi gửi thành công
- trạng thái hiển thị không nhầm giữa `requested` và `executing`

## 12.2 Backend Test

Cần kiểm tra:

- endpoint chỉ chấp nhận 5 intent hợp lệ
- JSON publish đúng schema
- `seq` tăng đúng mỗi lần gửi

## 12.3 Decision Integration Test

Các case tối thiểu:

1. `follow_main`
   - hệ thống bám `main-lane`

2. `lane_change_left` khi chưa thấy `other-lane`
   - vẫn `follow_main`
   - giữ intent chờ

3. `lane_change_left` khi thấy `other-lane` nhưng có solid marking
   - vẫn `follow_main`
   - báo `blocked`

4. `lane_change_left` khi thấy `other-lane` và không có solid marking
   - sinh line nối chuyển làn

5. `turn_left` khi chưa thấy `turn-lane`
   - vẫn `follow_main`
   - giữ intent chờ

6. `turn_left` khi thấy `turn-lane` hợp lệ
   - sinh line nối để rẽ trái

7. sau khi maneuver hoàn tất
   - reset về `follow_main`

## 12.4 Runtime Verification

Cần quan sát trên dashboard:

- `requested intent`
- `decision_state`
- `selected_lane_id`
- `active_trajectory_points`

Đặc biệt cần nhìn được tình huống:

- user đã yêu cầu rẽ/chuyển làn
- nhưng line nối chỉ xuất hiện khi điều kiện thực sự hợp lệ

## 13. Tóm Tắt

Đề xuất của bạn là hợp lý và phù hợp với kiến trúc hiện có nếu triển khai theo hướng:

- dashboard chỉ gửi `route intent`
- decision layer giữ intent theo kiểu latched
- luật trong `decision_sys.md` luôn là nguồn quyết định cuối cùng
- khi chưa khả thi thì tiếp tục `follow_main`
- khi khả thi mới sinh trajectory để rẽ hoặc chuyển làn

Điểm quan trọng nhất của implementation là tách rõ:

- thứ người dùng yêu cầu
- và thứ hệ thống thực sự được phép làm ở frame hiện tại
