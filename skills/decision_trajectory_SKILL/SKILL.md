# 🧭 Skill: Decision & Trajectory System (`control_node`)

Đọc skill này TRƯỚC khi sửa `ros2_ws/src/avs_perception/src/control_node.cpp` hoặc bất kỳ header nào trong `ros2_ws/src/avs_perception/include/avs_perception/trajectory_*.hpp`, `lane_legality.hpp`, `legacy_lane_model.hpp`. Đây là bản "quick reference + gotchas cho agent", KHÔNG phải spec đầy đủ — spec đầy đủ nằm ở `docs/architecture/decision_sys.md` (đọc theo thứ tự trong `AGENTS.md` mục "Đọc Docs Theo Nhiệm Vụ") và `system_report_current/05_control_node_va_control_error.md` + `06_gioi_han_va_kiem_thu.md` (đối chiếu trực tiếp với code, cập nhật gần nhất 2026-08-03).

## 1. Luồng 4 tầng (tóm tắt)

`path observation -> candidate trajectory -> normalized trajectory -> committed active trajectory`. Mỗi frame chỉ một `active trajectory` được publish gián tiếp qua `/avs/control_error`. Chi tiết đầy đủ: `docs/architecture/decision_sys.md`.

## 2. `LegacyLaneModel` vẫn chạy song song — KHÔNG phải dead code

`control_node.cpp` vẫn dùng `LegacyLaneModel` cho: split/select lane kiểu legacy, bộ đếm T-junction, `is_turn_blocked_by_solid`, `evaluate_trajectory_at_lookahead`, và (mới) `diagnose_other_lane_gates`. Không xoá các hàm này chỉ vì tên có chữ "legacy" — xem `system_report_current/06_gioi_han_va_kiem_thu.md` mục "Helper legacy vẫn chạy song song".

## 3. `TrajectoryLatch` — frozen turn execution (`trajectory_latch.hpp`)

Khi turn-lane biến mất khỏi tầm nhìn ngay lúc gần điểm rẽ (xe đã commit vào turn), path rẽ cuối cùng quan sát được bị "đóng băng" và replay open-loop theo odometry (`progress_mm += current_speed_mms_ * dt`), thay vì để planner replan theo candidate follow_main mới. Hướng suy ra từ tiếp tuyến path, KHÔNG dùng cảm biến yaw — sai lệch bám đường thực tế trong lúc latch là vô hình với hệ thống.

Field debug: `turn_latch_active`, `turn_latch_progress_mm`, `turn_latch_length_mm`, `turn_latch_elapsed_s`, `turn_latch_release_reason`, `turn_latch_observed_span_deg`, `turn_latch_extended_span_deg`, `turn_latch_extension_mm`, `turn_latch_deadline_s`, `turn_latch_heading_turned_deg` trên `/avs/lane_state` — dùng khi debug "tại sao xe rẽ không phản ứng lỗi lateral" hoặc "xe rẽ cụt/lặp vòng ở giao lộ".

**ĐÃ ĐO VÀ SỬA 2026-08-05 — hai mục dưới đây trước kia là bug, giờ là sự thật đã kiểm chứng:**
- `odom_speed_scale` default giờ là **1000.0** (trước 2500.0). Đo trên xe: tích phân `|twist.linear.x|` qua 48m khớp quãng đường `pose.position` đi được tới **1.0001**, mà position tính bằng mét → `linear.x` là m/s đúng chuẩn `nav_msgs/Odometry`. Đừng "hiệu chỉnh" lại bằng phỏng đoán.
  Scale sai 2.5× chính là gốc của triệu chứng "rẽ được 1 tí rồi path thẳng ra, xe đi thẳng qua giao lộ": `heading_at()` báo đã xoay 63° (đúng ngưỡng gate góc) khi xe mới xoay 25° — mà 25° cũng xấp xỉ độ lệch của làn cũ so với xe, nên **gate thẳng hàng mở cùng lúc**. Hai gate cùng nhả ở 1/4 cú rẽ.
- `/odom_raw` **có thật và tin được**, do `YB_Car_Node` (ESP32 qua micro-ROS) publish — không phải `cmdvel_odom_node.py`. Cả `twist.linear.x` lẫn quaternion yaw đều dùng được để khép vòng.

**Vẫn còn mở:**
- `latch_blocked_by_marking` (đường thoát an toàn khi gặp marking cấm) chỉ thực sự trip cho `TURN_LEFT` tại T-junction, không hoạt động cho `TURN_RIGHT` hay `TURN_LEFT` ngoài T-junction.
- `progress_mm=0` được giả định là vị trí xe hiện tại, nhưng điểm đầu trajectory đã chốt có thể lệch vài trăm mm so với xe thật → có thể gây `epsilon_x` nhảy bậc ngay frame bắt đầu latch.

Chi tiết đầy đủ cơ chế + các bug trên: `system_report_current/05_control_node_va_control_error.md` mục 6.16, `system_report_current/06_gioi_han_va_kiem_thu.md` mục "Bug đã biết, chưa fix trong TrajectoryLatch".

### 3b. Điều kiện nhả latch — tại sao KHÔNG được nhả khi "thấy lane mới" (2026-08-04)

Có 3 đường nhả: `latch_path_consumed` (hết path), `latch_timeout` (backstop ≥10s), và `latch_new_lane_acquired` (mới).

Đường thứ ba dùng `TrajectoryLatch::turn_complete(...)` — **hai gate phải cùng đúng**, và đây là chỗ dễ "cải tiến" sai nhất:

- **Gate góc** — góc xoay **đo thật** từ `/odom_raw` kể từ lúc đóng latch (`EgoMotion::frame_delta_from_ros_yaw(current_yaw_rad_, turn_latch_start_yaw_rad_)`) ≥ `turn_latch_release_min_span_frac` (**0.9** từ 2026-08-06; 0.7 mở gate ở 63° của cú rẽ 90° nên cắt cụt mọi cú) × target. Datum là lúc latch ĐÓNG — xem 3b-sexies, đừng đổi sang lúc commit.
  **Đừng quay lại dùng `heading_at(path, progress_mm)`** (bản cũ, đổi 2026-08-05). Nó vừa thừa hưởng mọi sai số của `progress_mm`, vừa chỉ là góc chord của **một đoạn 100mm đơn lẻ** — đo trên xe thấy dao động ±25° giữa các frame liên tiếp (`12.5, 4.7, 1.6, 26.3, 25.8, 19.4, 12.5, 4.3, 34.4, 61.6`), và có ca trả **70.1° trong khi toàn bộ path đóng băng chỉ có 61.8°**, tức mở gate trên góc mà path không hề chứa. Yaw đo thì đơn điệu qua cú rẽ và không mang theo thang đo quãng đường nào để sai.
  Mất `/odom_raw` → giá trị đứng yên → gate ngừng mở, rơi về `path_consumed`/deadline. Đúng hướng an toàn, vì gate này chỉ được phép nhả **sớm hơn** hai điều kiện kia.
- **Gate thẳng hàng** — main-lane quan sát được lệch khỏi trục xe < `turn_latch_release_max_lane_heading_deg` (25°).

**Đừng bỏ gate góc để nhả ngay khi thấy main-lane.** Latch đóng băng đúng lúc turn-lane rời khỏi khung hình, mà lúc đó làn cũ đi thẳng qua giao lộ không chỉ còn nhìn thấy được mà còn **thẳng hàng 0° với xe** — gate thẳng hàng một mình sẽ trip ngay frame đầu và huỷ cú rẽ trước khi nó bắt đầu. Ngược lại, gate góc một mình lại nhả vào khung hình rỗng. Hai gate bù nhau theo thời gian: đầu cua gate góc chặn, cuối cua làn cũ đã quay ~90° sang hông nên trượt gate thẳng hàng.

Gate này chỉ có thể làm latch nhả **sớm hơn** hai điều kiện cũ, không bao giờ muộn hơn — nên sai số `odom_speed_scale` (mà nó dùng chung qua `progress_mm`) không tạo ra chế độ hỏng mới.

Test: `LatchReleaseGate.*` trong `decision_trajectory_test.cpp`, gồm ca replay nhiều frame `StaysShutAcrossTheWholeApproachWithALaneAlwaysInView` — một frame đơn lẻ không chứng minh được gì ở đây vì lỗi cần phòng chính là "một frame sớm trip nhầm".

### 3b-bis. `extend_to_turn_angle` — cạm bẫy hình học (2026-08-05)

Hàm này kéo dài cung rẽ quan sát được tới đủ `turn_latch_target_heading_deg` rồi nối 700mm runout thẳng. Nó đã gây ra nhiều chế độ hỏng nặng; đọc trước khi chỉnh:

- **`turn_latch_min_radius_mm` (800) PHẢI lớn hơn lookahead của pure pursuit** (`ipm_transform_node` publish `lookahead_d_mm = 600`). Vạch thật trên map cong tới 362–459mm, nên hằng số này **thực sự bóp méo** hình học thật — và buộc phải vậy. Đã thử hạ về 350 cho đúng hình học: giao lộ 90° thành **154°**, vì khi bán kính cong nhỏ hơn lookahead thì điểm ngắm nằm sang phía bên kia đường tròn và lệnh lái bão hoà. Muốn hạ thì phải hạ lookahead cùng lúc.
- **Đuôi path IPM không tin được**, méo theo cả hai chiều: có ca bẻ quặt ngược (`-76 → -72 → -66`), có ca cong quá đà (đo được `-103°` cho giao lộ 90°). `trim_flared_tip` cắt về đoạn xoay xa nhất — xử lý được chiều bẻ ngược, **không** xử lý được chiều cong quá.
- **Cung quan sát ngắn không suy ra được bán kính.** `min_observed_span_deg = 40`: ở mức 15 cũ, một quan sát 20.6° từng đẻ ra **4.7m cung bịa**, xe quét vòng bán kính 2m suốt 18 giây.
- Fit dùng **bình phương tối thiểu** (`fit_circle_ls`) trên cửa sổ đuôi, không phải đường tròn qua 3 điểm. Đừng đổi ngược lại: 3 điểm cố định một đường tròn nên fit không thể "bất đồng" với chúng, nó nuốt trọn nhiễu vào kết quả, mà điểm cuối — chỗ đọc tiếp tuyến — lại là điểm tệ nhất trong ba.

Cách debug đúng vùng này: bắt `active_trajectory_points` của frame **ngay trước** khi latch đóng (đó chính là `observed`, đầu vào thật của hàm), rồi chạy lại hàm offline trên chính path đó. Đoán nhánh nào trả `pts` là mất thời gian — có 7 nhánh `return pts`.

### 3b-ter. Vào latch, và cái được đóng băng (2026-08-05, run10–run12)

Ba lỗi xếp chồng, mỗi cái che lỗi sau. Đã sửa, đừng gỡ:

- **Debounce vào latch** (`turn_latch_enter_dropout_frames`, mặc định 4). Trước đây latch đóng ở frame ĐẦU TIÊN không có turn-lane candidate, không debounce. Đo run10: intent rẽ trái thấy vạch 73% frame, các lần mất chủ yếu **1 frame** (29 lần) và 2–3 frame (8 lần), mất thật (≥5 frame) chỉ 14 lần; rẽ phải cả run chỉ mất 1 frame — **đó mới là lý do rẽ phải vốn chạy được**, không phải hình học. Repo đã có sẵn ngân sách `maneuver_dropout_hold_frames=10` và latch cắt ngang nó ở frame 1.
- **Đóng băng bản chụp lúc CÒN thấy vạch**, không phải path ở cuối cửa sổ chờ. Trong 4 frame chờ, path commit vẫn bị `soft_update` bám vào mảnh lane còn sót sau khi vạch mất — tức nhiễu. Đo: latch ngay cho `obs_span` 70–86° (không ca nào >90); latch sau 4 frame cho 38–146° với 4/11 vượt 90. Path đã cong 146° cho giao lộ 90° thì **không được nối dài**, xe lái nguyên đường cong đó và xoay lố.
- **Guard khớp tròn dùng residual, KHÔNG dùng tiếp tuyến-vs-psi.** Guard cũ (lệch tiếp tuyến >15° thì từ chối) **không thể thoả mãn về cấu tạo**: `psi` là góc dây cung trên cửa sổ hữu hạn, tiếp tuyến lấy tại đầu mút, mà trên cung tròn góc dây cung bằng tiếp tuyến ở GIỮA đoạn nó chắn — lệch cỡ nửa góc cung, tức 15–25° ở bán kính thật 310–613mm. Nó phạt path cong nhiều nên rẽ trái dính nặng nhất: loại 7/8 cú, trong khi ca lọt duy nhất đo 14.6° còn các ca bị loại đo 16.6–27.4°. Thay bằng RMS sai số điểm-tới-đường-tròn ≤30mm: 13 ca lành đo 2.8–12.7mm, ca hỏng thật đo 64.5mm. Sau khi đổi: rẽ trái nối đủ 90° **8/8**.

Phương pháp đã cho kết luận: `turn_monitor2.py` ghi mỗi frame cả `active_trajectory_points` (committed) lẫn `debug_trajectories[stage=candidate]` ra jsonl, rồi `guard_probe.cpp` chạy lại đúng chuỗi guard thật trên path đã bắt và in ra path thoát ở nhánh nào. `extend_to_turn_angle` có nhiều nhánh `return pts` — đoán nhánh nào là vô ích.

### 3b-quater. Vòng đóng bằng yaw đo — CÓ code nhưng TẮT mặc định (2026-08-05, run13–run14)

`re_express(pts, progress_mm, heading_error_rad)` nhận thêm tham số nghiêng frame phát path, điều khiển bằng `turn_latch_heading_correction_max_deg` — **mặc định 0, tức tắt**.

Đừng bật lại rồi dò ngưỡng. Nó **chống lại `min_radius_mm`, không phải chống trôi**: path đóng băng bị ép bán kính 800mm (bắt buộc, vì bán kính < lookahead 600mm làm pure pursuit bão hoà lái — xem 3b-bis), trong khi giao lộ thật khớp 310–613mm. Xe bám giao lộ thật xoay nhiều hơn trên mỗi mm so với path quy định, nên `turned − assumed` dương một cách hệ thống và vòng đóng đọc nhầm thành "đang xoay lố". Đo run14 ở 40°: rẽ phải sai số trung vị **+18.9°**, bị nghiêng ngược lại, về đích ở 61–85° thay vì 90, và tầng điều khiển bão hoà thành "3 bánh dừng 1 bánh chạy" khi bị đòi góc lái lớn. Rẽ trái tốt lên nhưng **do trùng hợp** — quan sát rẽ trái hay vượt 90° nên không được nối dài, và vòng đóng vô tình triệt tiêu đúng phần cong dư đó.

Muốn dùng được thì cần tham chiếu **không kế thừa** kẹp bán kính — ví dụ so góc xoay thật với tỉ lệ quãng đường đã đi trên tổng path nhân góc tổng, thay vì tiếp tuyến tại `progress_mm`.

### 3b-sexies. Datum của gate góc = LÚC LATCH ĐÓNG, đừng đổi sang lúc commit (2026-08-06, run19–run21)

**ĐÃ GỠ VÀ KIỂM CHỨNG (run21):** sau khi khôi phục datum về lúc latch đóng, sai
số giữa `turn_latch_heading_turned_deg` và yaw đo độc lập là **0.0° trên cả 4/4
cú rẽ** (biên độ 76° → **0°**). Góc thật lúc nhả co từ 70–154° (median 111) về
77–111° (median 80), và cú 111° duy nhất là ca có skip. Rẽ phải qua ba lượt cùng
một giao lộ: 69.7° (cụt) → 107.3° (lố) → **82.5/83.6°**.

Hệ quả dùng được: với datum đúng, `turn_latch_release_min_span_frac` **ánh xạ
trực tiếp sang góc nhả thật** (0.9 → nhả ở ~81°). Trước đây nó là núm vặn vào
một đại lượng nhiễu, nên dò ngưỡng là vô nghĩa.


`turn_latch_start_yaw_rad_ = current_yaw_rad_` tại thời điểm latch đóng. Đã thử
đổi sang mốc "lúc manager commit cú rẽ" (để gộp cả swing tiếp cận trước khi vạch
rời khung hình) và **phải gỡ** — đo 9 cú latch trên xe:

- Sai số so với yaw tích phân độc lập từ `/odom_raw`: **−66 đến +9°, biên độ 76°**,
  dấu ngẫu nhiên trên CẢ hai chiều rẽ. Không phải thiên lệch có cấu trúc để bù —
  phần swing mà bản chụp bắt được phụ thuộc vào chỗ manager tình cờ commit.
- Kết quả: góc thật lúc nhả trải 70–154°, chỉ **1/9 cú** nằm trong 90±10°.
- Hai cú rẽ trái có hình học đóng băng gần như y hệt (obs_span −82.8 vs −82.0,
  path 2000 vs 1940mm) cho ra 92.6° và 154.2° — **biến động nằm ở thời điểm gate
  nhả, không nằm ở path**.

Lý lẽ "commit được gate bằng `turn_proximity_mm` nên nó nằm ở giao lộ" nghe hợp lý
nhưng sai trên xe thật: commit xảy ra ở vị trí thay đổi rất rộng.

**Cách phát hiện lớp lỗi này:** đo yaw ĐỘC LẬP (tích phân quaternion `/odom_raw`
trong script quan sát) rồi đặt cạnh `turn_latch_heading_turned_deg`. Đọc lại số của
chính hệ thống thì một datum sai trông y hệt một cú rẽ sai. Công cụ:
`tools/turn_observation/`.

### 3b-septies. Xe xoay hơn path đóng băng 1.4–2.1× (2026-08-06, còn mở)

Đo 3 cú không có skip: path quy định 90.0° → xoay thật 124.0° (1.38×); 66.2° →
136.4° (2.06×); 36.7° → 69.7° (1.90×). Nghĩa là giả định open-loop `progress_mm`
→ heading qua tiếp tuyến path **sai 40–100%** — đây mới là gốc, `skip-to-runout`
chỉ vá triệu chứng.

Giả thuyết chưa xác nhận: `turn_latch_min_radius_mm=800` ép cung rộng hơn hình học
thật (310–613mm). **Ràng buộc "800 phải > lookahead 600" ở mục 3b-bis đã hết hiệu
lực** với cấu hình đang chạy: compose trên Pi ép `lookahead_d_min/max=140` và
`cascade_controller_v4` là PD lateral/heading, **không có Pure Pursuit** — không
còn vòng tròn lookahead nào để bão hoà. Nếu quay lại controller có Pure Pursuit thì
ràng buộc cũ sống lại.

**`skip-to-runout` bị giới hạn bởi tần số lấy mẫu, không phải ngưỡng.** Nó bắn
đúng ở frame đầu tiên có `|turned| >= target`, nhưng `update_turn_latch` chạy
trong `telemetry_callback` (~12 FPS, có frame rớt) trong khi xe xoay tới **64°/s**
lúc giữa cua. Đo run21: frame `-89.4°` rồi frame kế tiếp cách **0.27s** đã là
`-106.6°` — trượt 17.2° chỉ vì một khoảng rớt frame. Muốn bắn đúng 90° phải dẫn
trước theo tốc độ xoay (`turned + yaw_rate × lead`), không phải hạ ngưỡng.

**`skip-to-runout` không dừng được cú xoay.** Sau khi skip, `control_error` phát
`theta_rad = 0` và `epsilon_x_mm = 0` (runout thẳng, `re_express` đặt xe đúng trên
path), mà xe vẫn xoay thêm **0 / +7.6 / +13.1 / +32.4°**. Đó là trễ vật lý, không
phải lệnh lái. Hai lần cùng thời lượng (1.44s vs 1.49s) chênh 4× về góc ⇒ **hằng số
bù cố định không dùng được**, phải theo tốc độ xoay (`ego_yaw_rate_dps` có sẵn trên
`/avs/lane_state`).

### 3b-quinquies. Còn mở: quan sát vượt góc giao lộ

**ĐÃ HẾT tính đến 2026-08-06:** run19+run20 đo **0/492 frame** có |span| > 100° (obs_span lúc latch: −89.4 / −82.8 / −82.0 / −79.4 / −73.0 / −70.7 / −42.8 / +66.7 / +55.3). Nhiều khả năng nhờ debounce vào latch + chụp bản quan sát lúc còn thấy vạch ở mục 3b-ter. Giữ đoạn dưới làm lịch sử, đừng "sửa" lại vấn đề đã tự khỏi.

Nền cũ: **30% path commit trước lúc latch có |span| > 100° cho giao lộ 90°, tối đa 170°** (đo run13 và run14). Khi quan sát đã vượt target thì `extend_to_turn_angle` trả về nguyên trạng, path đóng băng mang nguyên độ cong bịa đó và xe xoay lố. `trim_flared_tip` chỉ cắt đuôi **bẻ ngược**, không cắt đuôi **cong quá đà** — méo đầu xa IPM đi cả hai chiều. Hướng chưa thử: cắt path tại điểm nó chạm đúng góc target.

### 3c. `post_latch_stub` — không bao giờ nhả vào khoảng trống

Lúc latch nhả, `committed_state_.trajectory` là phần đuôi re-express đã gần cạn (thường 1-2 điểm, có khi rỗng), trong khi nhánh dropout của `plan_follow_main` cần ≥2 điểm để bridge. Thiếu → trajectory invalid → manager vào RECOVERY → **mất path hẳn** cho tới khi perception bắt được đường mới. Đó chính là triệu chứng "rẽ xong mất path" báo 2026-08-04.

`release_turn_latch` giờ seed một đoạn thẳng 1500mm phía trước xe (`normalization_mode = "post_latch_stub"`, confidence 0.3) khi path còn <2 điểm. Hợp lệ về hình học vì runout của latch vốn đã thẳng — đây là phần nối tiếp của cung vừa bám, không phải path bịa. Có lane thật trong frame thì stub bị thay ngay trước khi dùng tới.

**Stub có hạn frame, bắt buộc.** Không có hạn thì stub tự tái sinh vô tận: nhánh dropout của `plan_follow_main` re-anchor path cũ và trả về **valid**, nên manager không bao giờ thấy dropout, không bao giờ vào RECOVERY — xe cứ chạy thẳng đơ theo stub và lệch dần. Triệu chứng "path thẳng đơ, không bám tâm làn" báo 2026-08-04 chính là cái này. `control_node` đếm `post_latch_stub_frames_` từ lúc nhả latch: gặp `follow_main_candidate.from_direct_observation == true` thì bàn giao và tắt bộ đếm; quá `post_latch_stub_max_frames` (param, mặc định 15) thì invalidate trajectory với `replan_reason = "post_latch_stub_expired"` → RECOVERY. Trong lúc đếm có `RCLCPP_WARN_THROTTLE` mỗi 500ms ghi rõ đang chạy stub chứ không bám lane.

Thấy `post_latch_stub` kéo dài nhiều frame trên `/avs/lane_state` = perception không nhận được đường mới sau giao lộ, đi debug phía perception chứ không phải latch.

## 4. Gate diagnostic cho lane-change (`legacy_lane_model.hpp`)

Khi lane-change hold vì `lane_change_target_not_detected`, `LegacyLaneModel::diagnose_other_lane_gates` báo cáo debug-only 4 hard gate (`side_gate`, `parallel_gate`, `distance_gate`, `corridor_gate`) đang pass/fail cho từng ứng viên `other-lane`, publish qua field `lane_change_gate_debug` trên `/avs/lane_state`. Dùng field này đầu tiên khi debug "vì sao đổi làn không tìm được target dù other-lane có vẻ đã detect".

## 5. Turn-lane exempt khỏi `LaneLegalityGate` (`lane_legality.hpp`)

Lane label `turn-lane` (20) luôn được giữ trong output đã lọc, verdict vẫn tính bình thường cho debug nhưng bước loại bỏ bị bỏ qua — turn selector cần thấy candidate này mới hoạt động, bị `ILLEGAL` (băng qua vạch phân cách) là chuyện dự kiến.

**Bug đã biết:** exempt hiện đang bỏ qua CẢ verdict HARD (solid-yellow), không chỉ soft-illegal như comment code mô tả — kết hợp với bug #3 ở mục trên (marking-abort không trip ngoài TURN_LEFT/T-junction), một turn-lane bên kia vạch solid-yellow có thể được chọn mà không còn gate nào chặn cho các trường hợp đó.

## 5b. `TrajectoryNormalizer::normalize` — turn dùng trọng số cố định, không ramp (2026-08-04)

Trước 2026-08-04, `normalize()` dùng chung một công thức ramp-theo-khoảng-cách (`w_cur` tăng dần 0.05–0.2 → 0.2–0.9 theo confidence trong 3m đầu) cho MỌI `trajectory_kind`. Từ 2026-08-04 chỉ còn `LANE_CHANGE_*` dùng ramp đó; `TURN_*` và `FOLLOW_MAIN` dùng trọng số cố định, đều suốt path:

- `TrajectoryNormalizer::turn_blend_prev_weight` (mặc định 0.75) cho `TURN_LEFT`/`TURN_RIGHT` — connector rẽ ngắn (~1-2m) nên ramp cũ chưa kịp ổn định đã hết path → hình dạng rung frame-to-frame.
- `TrajectoryNormalizer::follow_main_blend_prev_weight` (mặc định **0.0** = publish thẳng waypoint quan sát được) cho `FOLLOW_MAIN` — xem 5d.

Cả hai tunable live qua ROS param cùng tên (đọc lại mỗi frame trong `control_node.cpp`, cùng chỗ với `turn_bezier_handle_scale_mult`/`turn_lateral_bulge_mult`).

**Bẫy nền:** blend trộn toạ độ x/y thô, nhưng `prev_pts` ở vehicle-frame của frame TRƯỚC còn `cur_pts` ở frame hiện tại. `project_point_to_path` chỉ trả về arc-length vô hướng — bù được xe tiến bao xa DỌC path, **không bù xoay/lệch ngang của frame**. Mọi trọng số `w_prev` khác 0 đều mang theo sai số này, và sai số bùng lên đúng lúc xe đánh lái mạnh. Đừng tăng các knob trên mà không cân nhắc điều đó.

## 5b-bis. Hình dạng đầu nối rẽ TRÁI và PHẢI khác nhau — đừng gộp lại (2026-08-06)

Bốn hằng số, hai cặp:

| | Rẽ trái | Rẽ phải |
|---|---|---|
| `turn_bezier_handle_scale_mult[_right]` | 1.5 | **1.0** |
| `turn_lateral_bulge_mult[_right]` | 0.355 | **0.0** |

Rẽ trái băng qua giao lộ nên có chỗ để giữ hướng cũ rồi bẻ muộn, vòng ra giữa
ngã tư. Rẽ phải ôm góc gần từ làn phải — **không có chỗ nào bên ngoài để lấn**.
Cùng một hình dạng cho cả hai chiều là sai từ gốc.

Cả hai núm làm rộng cung đều được thêm ngày 2026-08-04 (`360c6b0` → `e7c50cc` →
`aed49ed` → `c399c3e`) để tune cho rẽ trái, rồi áp cho cả hai chiều mà không ai
xét tới bất đối xứng. Giá trị rẽ phải hiện tại (1.0 / 0.0) **chính là hình học
trước `360c6b0`**, khi `plan_turn_generic` gọi
`plan_transition(from, turn_pts, kTurnMaxHeadingDiffRad)` không kèm tham số nào.
Gtest `TurnBulgeAsymmetry.RightTurnKeepsThePreWideningConnectorShape` khoá đúng
điều đó bằng cách so từng điểm.

Triệu chứng khi gộp chung (đo run24 + user báo trên sa bàn): path rẽ phải phát
`theta_rad` −0.05…−0.12 — chỉ sang **trái** trong khi intent là rẽ phải — và
controller đáp lại bằng tới **0.50 rad/s xoay trái** trước khi bắt đầu xoay
phải. Xe lái ra khỏi chính góc cua nó đang vào, rồi không đủ góc.

`turn_lateral_bulge_mult_right` **nhận giá trị âm**: lật bụng cong sang phía
trong, path nghiêng vào cua ngay khi rời xe (đúng hình "hugs the inside corner"
mà comment của `plan_transition` mô tả). Chưa phải mặc định, nhưng dò live được
nếu 0.0 vẫn còn rộng.

**Đừng sửa cung rẽ phải bằng `turn_latch_min_radius_mm`.** Đã thử run25: hạ từ
800 xuống 400 làm bán kính phát ra đổi **7mm** (827 → 834). Kẹp đó chỉ nâng
những fit chặt hơn ngưỡng, mà phần lớn fit vốn đã trên 400 — độ rộng nằm trong
hình dạng đầu nối, không phải ở kẹp.

## 5c. `plan_follow_main`'s dropout hold re-anchors at vehicle (2026-08-04)

Trước 2026-08-04, khi `select_main_current` không thấy lane nào trong frame (mất hẳn quan sát - hay xảy ra đúng lúc xe rẽ/cua gắt), `plan_follow_main` replay `prev_state.trajectory.points` y nguyên tọa độ vehicle-frame cũ, không re-anchor theo vị trí xe hiện tại - path đóng băng đó nhìn như "một đường thẳng theo hướng cũ" đâm ra khỏi lane khi xe tiếp tục di chuyển/xoay. Fix: nhánh dropout giờ bridge lại từ vehicle `(0,0)` bằng `plan_transition(ego_stub, prev_state.trajectory.points)` (cùng cơ chế bridge 600mm-gap đã có), coi path cũ là "lane đích" ước lượng chứ không phải path cố định để replay. Nếu `plan_transition` reject (heading/lat quá lệch), vẫn fallback về replay thô như trước. Trường hợp mất quan sát hoàn toàn không còn gì để bridge (không có path cũ hợp lệ) giữ nguyên hành vi decay-confidence cũ, không đổi.

## 5d. FOLLOW_MAIN bám waypoint + luôn neo tại xe (2026-08-04)

Triệu chứng đã fix: khi xe rẽ/đánh lái quá mạnh, path bám main lane bị kéo lệch khỏi lane. Ba nguyên nhân cộng dồn, sửa cả ba:

1. **Blend liên-frame** (`trajectory_normalizer.hpp`): `w_prev ≈ 0.8` ngay đoạn gần xe — chính đoạn Pure Pursuit lấy lookahead — trộn hình học cũ đã sai frame vào path. Giờ `follow_main_blend_prev_weight = 0.0` → path = waypoint quan sát được. Xem bẫy frame ở 5b.
2. **Bridge ăn mất 1200mm waypoint đầu** (`plan_follow_main` + `plan_transition`): `plan_transition` **vứt bỏ waypoint target trước `split_idx_target`** (~1200mm dọc lane) rồi thay bằng mẫu Bezier tổng hợp theo heading xe. Nghĩa là mọi lần bridge, đoạn gần xe — chính đoạn controller lái theo — **không còn là tâm làn** mà là đường cong tự vẽ, cắt qua lane và xê dịch theo độ lệch của xe.

   Chốt 2026-08-04: FOLLOW_MAIN **không neo tại xe** trong trường hợp thường. Path = waypoint tâm làn quan sát được, y nguyên. Độ lệch của xe thuộc về control error (`epsilon_x`), không được nắn vào hình dạng path. Ngoại lệ duy nhất còn bridge: gap giữa giao lộ (`raw_path.front().y > kBridgeMinLaneStartYMm`) — lúc đó không có waypoint nào gần xe để bám, và path lơ lửng ngoài xa làm sập overlap metric của manager.

   Bridge gap đó giờ dùng **`bridge_gap_to_lane`** (`trajectory_planner.hpp`, public), KHÔNG dùng `plan_transition` nữa. Khác biệt cốt lõi: `bridge_gap_to_lane` chỉ lấp đúng khoảng từ xe `(0,0)` tới waypoint đầu tiên (Bezier bậc 3, tiếp tuyến cuối = tiếp tuyến đầu của lane) rồi nối **toàn bộ** waypoint quan sát được — không bỏ điểm nào. `plan_transition` thì vứt mọi điểm target trước `split_idx_target`, và có bẫy nặng hơn ở `trajectory_planner.hpp:887`: khi target ngắn hơn 1200mm nó fallback `split_idx_target = target_pts.size() / 2`, **ăn mất một nửa quan sát**. Ngay sau khi rẽ xong, lane mới thường vừa xa (>600mm → bridge kích hoạt) vừa ngắn (<1200mm → fallback chia đôi) — đúng combo gây "sau cua lệch nặng". `bridge_gap_to_lane` từ chối (trả rỗng) khi heading lệch >110° hoặc waypoint đầu lệch ngang >1500mm; lúc đó `plan_follow_main` giữ nguyên `raw_path` chưa bridge.

   Nhánh dropout ở 5c vẫn dùng `plan_transition` — có chủ đích, đừng đổi: nó phải re-anchor **cả path nhớ** theo pose mới, không phải lấp một gap.

   **Đừng "cải tiến" bằng cách neo path vào xe lần nữa** — đã thử 2026-08-04 và phải revert: nó chính là thứ làm path lệch khỏi waypoint.
3. **Manager đóng băng path cũ** (`trajectory_manager.hpp`): nhánh `low_confidence_deviation_hold` giữ path cũ tới `low_conf_hold_frames` (5) khi candidate lệch nhiều + confidence thấp — đúng combo xảy ra khi rẽ mạnh (frame xoay → lệch nhiều; lane còn thấy ngắn/cong → confidence thấp). Giờ bỏ qua gate này khi candidate là `FOLLOW_MAIN` có `from_direct_observation == true`. Gate vẫn áp dụng bình thường cho mọi trường hợp khác.

`PlannedTrajectory::from_direct_observation` (mới, `decision_types.hpp`) = true khi geometry đến từ lane thật quan sát được frame này, false khi đang hold theo trí nhớ.

**Đường cong tâm làn đã có sẵn — đừng thêm spline.** `ipm_transform_node.cpp:156` (`extract_centerline_waypoints_y/_x`, `step_mm = 100.0`) trích waypoint tâm làn cách nhau 100mm và `fit_polynomial_xy` (`:492`) fit đa thức cho chúng. Waypoint tới `control_node` đã LÀ đường cong lấy mẫu dày, đúng bằng bước của `resample_path(…, 100.0)` — nội suy thẳng giữa hai điểm cách 100mm không tạo sai lệch đáng kể. Cần path bám tâm làn thì chỉ việc dùng nguyên waypoint, không fit lại gì cả.

**Lưu ý test:** mirror Python `decision_harness.py:1649` vẫn giữ công thức ramp CŨ cho mọi kind — pytest pass KHÔNG chứng minh gì về các thay đổi trên. Coverage thật nằm ở gtest (`TrajectoryNormalizerBlend.*`, `TrajectoryManagerHold.*`, `FollowMainBridge.*`). Mirror không có test nào assert trọng số blend bằng số nên không cần xoá gì.

## 5e. `EgoMotion` — bù xoay frame cho path nhớ (`ego_motion.hpp`, 2026-08-04)

Triệu chứng: path vẽ lệch khỏi lane khi xe rẽ/cua gắt, và xe chạy lệch theo. Nguyên nhân gốc là **bẫy frame ở 5b** nhưng ở mức hệ thống: MỌI path sống qua nhiều frame đều nằm ở vehicle-frame của frame tạo ra nó, và không có chỗ nào trong pipeline sửa phần **xoay**. `project_point_to_path` / `plan_transition` chỉ khôi phục được xe tiến bao xa DỌC path (vô hướng). Xoay mới là số hạng lớn: 15° yaw đẩy điểm cách 1000mm lệch ngang ~260mm, trong khi 1/4 giây di chuyển chỉ ~100mm và gần như dọc path.

**Bẫy đã vấp phải — đọc kỹ trước khi "cải tiến" chỗ này.** Bản đầu xoay `committed_state_.trajectory.points` **tại chỗ** ở đầu `telemetry_callback`, để vá hết 4 nơi replay bằng 1 chỗ sửa (blend normalizer `turn_blend_prev_weight`=0.75, 3 nhánh hold của manager, dropout replay của `plan_follow_main`). **Sai, và đã gây regression trên xe.** Lý do: `committed_state_` không chỉ là hình học để vẽ — nó là **mốc so sánh của mọi ngưỡng quyết định** (`lateral_rms_mm`, `overlap_ratio`, `topology_changed` đều so candidate frame này với committed frame trước). Toàn bộ `replan_lateral_rms_mm` / `hold_lateral_rms_mm` / `min_overlap_ratio` được tune trên trí nhớ **chưa xoay**. Xoay nó = đổi đầu vào của ngưỡng mà không đổi ngưỡng.

Triệu chứng thực tế của regression đó (2026-08-04, đã revert): (1) rẽ xong delay mấy giây mới lập path — trí nhớ world-fixed trông khớp đường mới nên ở lì trong `deviation_below_threshold` + `hold_maneuver_fallback`, hai nhánh này thay nhau reset `dropout_hold_counter` về 0 (`trajectory_manager.hpp:131`) nên cửa sổ hold kéo dài vượt thiết kế; (2) đang rẽ thì mất path, nhảy sang main lane bên kia giao lộ — chiều ngược lại, deviation phình lên → `excessive_deviation_replan`.

Fix đúng: **chỉ xoay bản path đem publish, không bao giờ chạm `committed_state_`.** `committed_yaw_rad_` = yaw lúc hình học committed được **làm mới từ quan sát**; nó đứng yên suốt mọi HOLD và chỉ nhảy khi geometry đổi. Phát hiện "đã làm mới" bằng cách so `committed_state_.trajectory.points` trước/sau `TrajectoryManager::update` — nhánh HOLD gán `previous_state.trajectory` nguyên si nên points giống hệt; không dựa vào chuỗi `reason` (dễ vỡ khi đổi tên). Latch active luôn tính là fresh vì `re_express` đã dựng lại arc ở frame hiện tại. Tính chất quan trọng: bù ego-motion giờ **decision-neutral** — sai nhất thì path vẽ hơi lệch, không thể gây mất path hay kéo dài hold.

**Bẫy dấu — quan trọng nhất ở mục này.** ROS yaw CCW-positive quanh +z ⇒ rẽ TRÁI làm yaw TĂNG. Vehicle-frame ở repo này có +x sang PHẢI ⇒ góc path `atan2(dx, dy)` (xem `TrajectoryLatch::terminal_heading_rad`) là rẽ-PHẢI-dương. Hai quy ước ngược nhau, nên `EgoMotion::frame_delta_from_ros_yaw` có dấu trừ. Đảo dấu chỗ này không phải "không bù" mà là **xoay sai gấp đôi**. Gtest `EgoMotion.SignAgreesWithPathHeadingConvention` khoá đúng bất biến đó — rẽ trái 20° ⇒ path nhớ phải quay sang phải đúng +20°.

Yaw lấy từ `pose.pose.orientation` (quaternion), không phải `twist.angular.z`. **Cẩn thận với lý lẽ "quaternion vô thứ nguyên nên miễn nhiễm lỗi đơn vị `odom_speed_scale`" — lý lẽ đó chỉ đúng nếu yaw được ĐO.** `cmdvel_odom_node.py:82-91` (node publish Odometry duy nhất trong repo) tích phân `theta += wz*dt` rồi mới đóng gói thành quaternion, tức quaternion thừa hưởng nguyên lỗi tỉ lệ của `angular.z`. User xác nhận `/odom_raw` trên xe đến từ encoder/IMU thật chứ không phải node này — nếu điều đó đổi thì bù xoay sai biên độ. **Dấu của `/odom_raw` vẫn CHƯA kiểm chứng trên phần cứng.**

**Bù latency: mặc định TẮT.** Quan sát cũ `output_age_ms` (~150-250ms trên Pi) khi controller lái theo nó, nên ngoại suy `yaw_rate_rps_` (EMA, chỉ dùng ở đây) qua khoảng tuổi đó, clamp bằng `latency_compensation_max_s` (0.4). Tắt mặc định vì nó là **ngoại suy** và đổi trực tiếp **giá trị** `epsilon_x_mm`/`theta_rad` (không đổi schema) một lượng chưa có phép đo nào trên xe xác nhận — 0.4s × tốc độ xoay khi rẽ là hàng chục độ. Áp lên `published_state`, sau phép xoay ego. Thiếu `output_age_ms` → 0 → no-op.

Param live-tune: `ego_yaw_compensation_enabled` (true), `latency_compensation_enabled` (**false**), `latency_compensation_max_s` (0.4). Diagnostic trên `/avs/lane_state`: `ego_yaw_deg`, `ego_yaw_delta_deg`, `ego_yaw_rate_dps`, `latency_yaw_deg`, `observation_age_ms`. `ego_yaw_delta_deg` **phình dần rồi snap về ~0** = đang hold rồi được làm mới — đọc trực tiếp được xe đang coasting trên trí nhớ bao lâu. Đứng im đúng 0 trong lúc xe rõ ràng xoay ⇒ `/odom_raw` không tới (`ego_yaw_deg` cũng đứng im).

Chưa verify trên Pi tính đến 2026-08-04.

## 6. Checklist debug runtime nhanh

Đi ngược từ input tới output: `/avs/telemetry` (label/polygon) → `/avs/telemetry_realworld` (waypoints/lookahead) → `/avs/lane_state` (`decision_state`, `route_intent`, `trajectory_kind`, `normalization_mode`, `hold_reason`, `replan_reason`, `control_source`, `turn_latch_*`, `lane_change_gate_debug`, `ego_yaw_*`) → `yellow_gate` → `active_trajectory_points` → `/odom_raw`. Chi tiết: `system_report_current/06_gioi_han_va_kiem_thu.md` mục 7.4.

## 7. Quy tắc bắt buộc khi sửa

- Mọi thay đổi vào `control_node.cpp` phải liệt kê cụ thể hàm/hành vi đổi và được user duyệt trước khi code (xem `CLAUDE.md`).
- Test một frame rồi kết luận memory/hysteresis/latch đúng là sai — lỗi trajectory manager/latch chỉ lộ ra khi replay nhiều frame liên tiếp.
- Logic decision/trajectory thuần thuật toán mới → viết gtest trong `ros2_ws/src/avs_perception/test/decision_trajectory_test.cpp`, KHÔNG thêm vào `test/decision_system/decision_harness.py` (mirror đã đóng băng, chính sách Plan D3 2026-07-05).
- `stop-line` không được dùng để kích hoạt rẽ/giao lộ/chuyển làn trong giai đoạn hiện tại — nếu thấy code mới dùng `stop-line` cho việc này, đó là regression hoặc vi phạm nguyên tắc kiến trúc, không phải tính năng.
