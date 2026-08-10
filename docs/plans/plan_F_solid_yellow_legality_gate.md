# Plan F — Solid-Yellow Legality Gate (đi đúng làn theo vạch vàng)

Trạng thái: **F1+F2+F3 CODE XONG 2026-07-18** (danh sách §5 đã duyệt; gate dây
vào `control_node.cpp` + auto-return qua `LegalityAutoReturn`; build sạch layout
user, **61 gtest pass** (34 `PlanFLegality.*`), 96 pytest nguyên trạng — fixtures
hiện không chứa yellow nên gate permissive, đúng nguyên tắc không đổi behavior
khi không có tín hiệu). **Còn mở:** (1) regression live / fixture thật có yellow
để điền metric F2 (§8); (2) quyết định nghiệp vụ §9 — auto-return vượt ngược
solid-yellow hiện BỊ marking gate chặn (chủ đích không sửa
`is_lane_change_blocked_by_solid_obs`), chỉ hoàn tất được qua dashed-yellow;
xem test `AutoReturnAcrossSolidYellowStaysBlocked`.

## 1. Mục tiêu & Luật nghiệp vụ

Bổ sung vào luật chọn lane của `../architecture/decision_sys.md`:

> Chỉ được vẽ path cho lane (main-lane, other-lane, turn-lane — không phân biệt)
> nằm **bên phải** solid-yellow khi vạch có hướng dọc theo chiều cao frame,
> hoặc nằm **bên dưới** solid-yellow khi vạch nằm ngang theo chiều rộng frame.

Mục đích: xe luôn đi đúng làn (giao thông bên phải, solid-yellow là vạch phân
chia hai chiều), kể cả khi intent + hình học lane đơn thuần cho phép chọn lane
ngược chiều.

Quyết định thiết kế đã chốt với user (2026-07-18):

1. Phương pháp: **signed side test** trên polyline yellow có định hướng
   (không dùng ray-crossing từ ego).
2. Phạm vi: gate áp lên **mọi candidate lane** (main/other/turn-lane đích).
   KHÔNG áp lên đường transition nối (đường nối khi rẽ được phép đi qua vùng
   giao lộ).
3. Dropout yellow: **memory + hold window** (giữ reference cũ ≤ hold frames,
   quá hold → permissive/UNKNOWN).
4. Current lane illegal ổn định: **tự động về đúng làn** (forced transition
   sang lane hợp lệ gần nhất, có debounce).

Cập nhật 2026-07-18 (dashed-yellow, đã chốt với user):

5. `dashed-yellow` cũng là vạch phân định hai chiều, tham gia gate y hệt
   `solid-yellow` nhưng là **soft reference**: lane illegal-vì-dashed được
   miễn lọc CHỈ khi intent `lane_change_*` active (được phép chủ động vượt
   vạch đứt vàng, phục vụ né vật cản/chuyển nhiều làn sau này); turn và
   follow_main không được miễn. `solid-yellow` là hard: không intent nào
   được miễn.
6. An toàn trên luật: sau khi vượt dashed sang bên sai, auto-return vẫn giám
   sát và kéo về theo debounce chuẩn, bất kể loại vạch. Làn đúng = verdict
   LEGAL theo hình học (main hoặc other), không phải nhãn `main-lane`.
7. Không cần merge dash phân mảnh: quy tắc labeling tô `dashed-yellow` thành
   MỘT mask dài bao trùm cả dải vạch (như solid), mỗi detection là một object
   dài — quality gate giữ nguyên.

## 2. Thiết kế hình học

### 2.1 Hợp nhất hai case dọc/ngang thành một công thức

Trong vehicle frame (`X>0` phải, `Y>0` trước): nếu định hướng vạch bằng vector
`d` theo quy ước

- vạch dọc/chéo rõ: chọn dấu sao cho `d.y > 0` (hướng ra xa xe),
- vạch gần ngang: chọn dấu sao cho `d.x > 0` (hướng sang phải),

thì cả hai luật gộp thành một: **lane hợp lệ ⇔ điểm lane nằm bên phải `d`**,
tức `cross_z(d, p − p0) = d.x·(p.y − p0.y) − d.y·(p.x − p0.x) < 0`.

- `d = (0,1)` (dọc): `cross_z = −(p.x − p0.x) < 0 ⇔ p.x > p0.x` → bên phải. ✓
- `d = (1,0)` (ngang): `cross_z = (p.y − p0.y) < 0 ⇔ p.y < p0.y` → bên dưới. ✓

### 2.2 Quy ước dấu của `d` — β-band tránh gián đoạn ở vạch ngang

Vạch không có hướng nội tại nên phải chọn dấu `±d`. Nếu chỉ dùng "lật khi
`d.y < 0`" thì vạch gần ngang chúc nhẹ xuống (`d = (1, −0.05)`) bị lật thành
gần `−X` → verdict đảo so với vạch chúc nhẹ lên → gián đoạn đúng tại vùng
ngang, là case quan trọng của luật. Fix bằng **β-band**:

```
angle_from_x = |atan2(|d.y|, |d.x|)|        # 0 = ngang thuần, 90° = dọc thuần
if angle_from_x < BETA_DEG (mặc định 20°):  # vạch "gần ngang"
    chọn dấu sao cho d.x > 0                # hợp lệ = bên dưới, bất kể chúc nhẹ lên/xuống
else:                                       # vạch dọc/chéo
    chọn dấu sao cho d.y > 0                # hợp lệ = bên phải
```

Gián đoạn chuyển từ vùng ngang (nguy hiểm, hay gặp ở T-junction/crossroad)
sang biên chéo ~[20°] (hiếm); per-marking hysteresis frame-to-frame (giữ phân
loại band cũ khi góc nằm trong ±5° quanh biên) làm mượt nốt biên này.

### 2.3 Xây polyline yellow tin được — PCA re-sort

`PathObservationBuilder` sort marking theo `y` tăng dần
(`path_observation.hpp:148`) → vạch **ngang** bị sort theo nhiễu `y` nhỏ thành
polyline zigzag, không dùng trực tiếp được. Gate tự xử lý từ waypoints thô:

1. PCA trên waypoints của marking → trục chính `u` (đại diện hướng vạch).
2. Chọn dấu `u` theo quy ước β-band (§2.2).
3. Re-sort waypoints theo tọa độ chiếu lên `u` → polyline đơn điệu theo trục
   chính, giữ được độ cong.
4. Nguồn điểm: `MarkingObservation.points` (waypoints); nếu thiếu/`< 2` điểm,
   gate tự đọc `raw_obj["polygons_real_world"]` làm fallback (marking gate
   hiện có cũng xét polygon — `trajectory_planner.hpp:545`), không sửa
   `PathObservationBuilder`.
5. Marking bị loại nếu: `< 2` điểm sau dedup, tổng chiều dài `< 300 mm`,
   `confidence <` ngưỡng (tái dùng ngưỡng marking-confidence hiện có của
   lane-change gate), hoặc **PCA linearity guard**: tỉ lệ eigenvalue
   `λ1/λ2 < 4` (marking hình L/góc/đốm gần đẳng hướng → hướng PCA là nhiễu,
   không dùng làm reference).

### 2.4 Verdict cho một lane

Cho từng lane candidate, với từng yellow đã chuẩn hóa:

1. **Applicability**: chiếu các điểm lane lên trục `u` của yellow. Nếu khoảng
   chiếu của lane không giao `[proj_min − EXT, proj_max + EXT]` của yellow
   (`EXT = 500 mm`) → yellow này **không áp dụng** cho lane (không phán xử
   lane nằm ngoài phạm vi vạch, tránh extrapolate vô hạn).
2. Nếu nhiều yellow áp dụng → dùng yellow **gần nhất** (khoảng cách vuông góc
   trung bình nhỏ nhất). Không AND tất cả — tại giao lộ có thể thấy đồng thời
   vạch của đường hiện tại và đường đích, mỗi lane chịu luật của vạch gần nó.
3. **Signed test**: chỉ xét các điểm lane có hình chiếu nằm trong
   `[proj_min − EXT, proj_max + EXT]` của yellow (điểm ngoài span không được
   tham gia — tránh nearest-segment extrapolation chi phối verdict với lane
   dài). Với mỗi điểm hợp lệ, tìm segment gần nhất trên polyline yellow, tính
   `signed_dist = cross_z(d̂, p − p0)` với **`d̂` là unit vector** của segment
   (định hướng theo §2.2) — kết quả là khoảng cách vuông góc có dấu, đơn vị
   mm, không phụ thuộc chiều dài segment. Signed distance của lane = trung
   bình các điểm tham gia.
4. Verdict:
   - `mean_signed < −MARGIN` (bên phải/dưới, `MARGIN = 100 mm`) → `LEGAL`
   - `mean_signed > +MARGIN` → `ILLEGAL`
   - `|mean_signed| ≤ MARGIN` → `UNKNOWN` (dead-zone, giữ verdict cũ nếu có)
5. Không có yellow nào áp dụng (kể cả từ memory) → `UNKNOWN`.

`UNKNOWN` = **permissive** (không lọc) — giữ nguyên behavior hiện tại khi
không có thông tin, đúng nguyên tắc không đổi behavior khi chưa có tín hiệu.

### 2.5 Memory + hold window

`LaneLegalityGate` (stateful, header-only):

- Frame có ≥1 yellow đạt quality gate → refresh `yellow_ref_` (danh sách
  polyline chuẩn hóa + band class), `yellow_age_frames_ = 0`.
- Frame mất yellow → `yellow_age_frames_++`; nếu `≤ YELLOW_HOLD_FRAMES (10)`
  vẫn đánh giá lane mới bằng reference cũ. Quá hold → mọi lane `UNKNOWN`.
- Giới hạn đã biết: reference cũ không được bù chuyển động ego (không có
  odometry trong node) — sai lệch chủ yếu theo trục Y (~100–300 mm cho 10
  frame @14FPS), chấp nhận được cho gate lateral; ghi nhận trong docs.
- Per-lane hysteresis: `lane_id → {verdict, streak}`; verdict chỉ lật khi dấu
  mới xuất hiện `≥ 2` frame liên tiếp. Lane id mới/đổi → đánh giá tươi từ
  hình học.

## 3. Tích hợp pipeline (không phá cấu trúc)

Gate là **stage lọc mới** ngay sau khi parse telemetry, TRƯỚC mọi consumer
lane (kể cả legacy path), để không tồn tại hai "thế giới quan" lane khác nhau
trong cùng một frame (review codex #4):

```
lanes     = LegacyLaneModel::extract_lane_candidates(telemetry)   // giữ nguyên
obs_frame = PathObservationBuilder::build(telemetry)              // DỜI LÊN trước update_lane_state
report    = legality_gate_.evaluate(obs_frame)                    // NEW
obs_plan  = LaneLegalityGate::filter(obs_frame, report, exempt)   // NEW
lanes     = filter_legacy_lanes(lanes, report, exempt)            // NEW – cùng verdict, cùng exempt
update_lane_state(lanes, ...); select_turn_lane(lanes, ...)       // giữ nguyên, nhận tập đã lọc
plan_follow_main(obs_plan, ...); plan_candidate_for_intent(obs_plan, ...)
```

- `exempt = last_main_track_id_` (lane đang bám): **không** bị loại ở cả hai
  đường (tránh mất path đột ngột) — đi vào nhánh auto-return §4.
- Lane `ILLEGAL` biến mất khỏi mọi consumer ⇒ luật chọn hiện có (gần/xa
  turn-lane, side/parallel/distance gate, marking gate) và các hold/fallback
  (`turn_not_in_range`, `lane_change_target_not_detected`) chạy y nguyên và
  **nhất quán**: target illegal = target không tồn tại → rơi đúng vào các
  hold_reason hiện có, không tạo vòng replan `intent_change`.
- **Nhánh direct-IPM fallback** (xuất control trực tiếp từ
  `main_current->raw_obj` khi không có active trajectory,
  `control_node.cpp:575` vùng lân cận): vì `main_current` được chọn từ tập
  `lanes` đã lọc nên không thể trỏ tới lane illegal (trừ lane exempt — chủ
  đích). Không sửa logic nhánh này, chỉ xác nhận bằng test §7.
- Với `turn_left` tại crossroad, gate tự loại turn-lane ngược chiều (nằm trái/
  trên vạch của đường đích) → củng cố luật "chọn lane xa" thay vì thay nó.
- Khi lane duy nhất bị lọc → planner không có candidate → `TrajectoryManager`
  giữ committed trajectory qua dropout hold window rồi mới RECOVERY — đây là
  hành vi dropout chuẩn hiện có, chấp nhận và assert trong test (codex #11).
- State machine `DecisionState` hiện có giữ nguyên — legality là **cây quyết
  định per-frame** ở tầng chọn candidate, không phải state mới.
- **Soft/hard (dashed vs solid)**: mỗi `YellowReference` mang cờ `soft`
  (dashed-yellow). Verdict ILLEGAL nhớ kèm nguồn soft/hard qua hysteresis.
  `filter`/`filter_legacy` nhận cờ `allow_soft_illegal` — control_node truyền
  `true` khi `current_intent_` hoặc committed maneuver là `LANE_CHANGE_*`
  (lane đích bên kia vạch đứt vàng được giữ lại cho planner chọn), `false`
  cho mọi trường hợp khác. Marking gate lane-change hiện có giữ nguyên
  (dashed vốn không block) — không sửa `is_lane_change_blocked_by_solid_obs`.

## 4. Auto-return về làn hợp lệ

Thiết kế theo **internal intent override** (sửa theo review codex #1/#2:
không sinh trajectory kind lệch với intent mà manager nhìn thấy — sinh một
intent nội bộ thật sự, chảy qua đúng máy móc maneuver hiện có):

```
Điều kiện xét: current_intent_ == FOLLOW_MAIN, không có committed maneuver,
               không có internal override đang active.

main-lane đang bám ILLEGAL?
  ├─ không → illegal_streak = 0
  └─ có → illegal_streak++
      illegal_streak ≥ LEGALITY_RETURN_DEBOUNCE (5)?
        ├─ chưa → giữ lane, chỉ set debug flag
        └─ rồi → tồn tại lane LEGAL (main/other) qua được các gate
                 lane-change hiện có? (side/parallel/distance)
              ├─ không → giữ lane + flag (không bao giờ tự vứt path)
              └─ có → set current_intent_ = LANE_CHANGE_LEFT/RIGHT
                      (theo hướng lateral về lane legal),
                      intent_source_ = "legality_gate" (nội bộ)
```

- Intent nội bộ đi qua **nguyên vẹn** đường maneuver hiện có: latch, hold
  window, marking gate, manager commit (`manager_intent` = intent override →
  không còn mâu thuẫn `committed_intent` vs `current_intent_` ở frame sau),
  completion/abort logic — không cần sửa `TrajectoryManager`, không cần
  thêm `replan_reason` mới vào whitelist (reason tự nhiên là
  `intent_change`).
- Nguồn gốc legality ghi ở debug: `route_intent_source = "legality_gate"` và
  `legality_return_active = true` trong `/avs/lane_state`; KHÔNG publish
  ngược lên `/avs/route_intent` (topic đó là của planner/manual).
- Override tự hủy (về FOLLOW_MAIN) khi: maneuver hoàn tất theo lifecycle
  hiện có, HOẶC lane đích không còn legal/tồn tại, HOẶC user intent thật đến
  từ `/avs/route_intent` (intent thật luôn thắng override nội bộ).
- Debounce 5 frame đồng bộ với debounce intent hiện có; chặn misdetection
  chớp tắt của yellow gây lane-change tự phát.
- Kill-switch riêng: param `legality_return_enabled` (default `true`, tắt
  được độc lập với gate lọc `legality_gate_enabled`).

## 5. Danh sách thay đổi cụ thể (CẦN USER DUYỆT trước khi code)

| # | File | Thay đổi |
|---|---|---|
| 1 | `include/avs_perception/lane_legality.hpp` | **MỚI**, header-only: PCA (kèm linearity guard λ1/λ2) + β-band orientation, polygon fallback từ `raw_obj`, applicability theo projection span, signed test (unit vector, mm), `LaneLegalityGate` (yellow memory + hold, per-lane hysteresis), `filter()` cho `PathObservationFrame` và cho `std::vector<LaneCandidate>` (legacy), `nearest_legal_lane()` cho auto-return |
| 2 | `include/avs_perception/trajectory_planner.hpp` | Không đổi logic (auto-return dùng đường lane-change hiện có qua intent override — bỏ hàm `plan_forced_lane_return` từng dự kiến); chỉ thêm 1 dòng `FRIEND_TEST(PlanFLegality, FilteredFrameRemovesLaneChangeTarget)` theo cơ chế test-access sẵn có (đã làm ở F1) |
| 3 | `include/avs_perception/trajectory_manager.hpp` | **KHÔNG đổi** (intent override làm manager thấy intent nhất quán; `replan_reason` giữ whitelist hiện có) |
| 4 | `src/control_node.cpp` | (a) member `LaneLegalityGate legality_gate_;`, `illegal_current_streak_`, `legality_intent_override_` (cờ + hướng); (b) **dời** `PathObservationBuilder::build` (`:349`) lên trước `update_lane_state` (`:348`); gọi `evaluate` + `filter` cho CẢ `obs_frame` VÀ vector `lanes` legacy (`:285`) với cùng verdict, exempt `last_main_track_id_` — mọi consumer phía sau (`select_turn_lane :345`, `update_lane_state :348`, planner `:367/:370`, direct-IPM fallback `:575`) tự nhận tập đã lọc, không sửa logic từng chỗ; (c) khối auto-return: set `current_intent_ = LANE_CHANGE_*` nội bộ khi đủ điều kiện §4, tự hủy override theo lifecycle §4 (chèn trong vùng xử lý intent, trước `:359`); (d) `publish_lane_state`: thêm object debug `yellow_gate` {`visible`, `age_frames`, `lane_legality`, `illegal_current_streak`, `legality_return_active`, `route_intent_source`} (additive, không đổi field cũ); (e) declare params: `legality_gate_enabled` (true), `legality_return_enabled` (true), `legality_margin_mm` (100), `legality_yellow_hold_frames` (10), `legality_return_debounce_frames` (5), `legality_beta_deg` (20) |
| 5 | `test/decision_trajectory_test.cpp` | gtest mới (danh sách §7) |
| 6 | `docs/architecture/decision_sys.md` | thêm mục "Luật Hợp Lệ Lane Theo Solid-Yellow" (đã thêm cùng plan này) |

KHÔNG đụng: `ipm_transform_node.cpp`, contract `/avs/control_error`,
`decision_harness.py` (mirror ĐÓNG BĂNG), label constants, `/avs/route_intent`
(override là nội bộ, không publish lên topic).

## 6. Phases

- **F1** — `lane_legality.hpp` + toàn bộ gtest hình học/memory thuần (không
  đụng `control_node.cpp`; build + test offline được ngay). Không cần duyệt
  thêm (file mới, không đổi behavior runtime).
- **F2** — tích hợp filter vào `control_node.cpp` (mục 3a/3b/3d/3e) sau khi
  user duyệt §5. Gate lọc hoạt động, auto-return chưa bật nhánh.
- **F3** — auto-return (3c) + gtest multi-frame + regression fixture.

## 7. Test plan (gtest — `decision_trajectory_test.cpp`)

Hình học:
1. Yellow dọc: lane phải `LEGAL`, lane trái `ILLEGAL`.
2. Yellow ngang: lane dưới `LEGAL`, lane trên `ILLEGAL`.
3. Yellow chéo 45°: đúng phía, không phụ thuộc thứ tự waypoint đầu vào.
4. Yellow gần ngang chúc nhẹ xuống (`d.y < 0` trước chuẩn hóa): vẫn "dưới =
   LEGAL" (β-band, không gián đoạn).
5. Yellow ngang bị sort-by-y zigzag (input như `PathObservationBuilder` thật):
   PCA re-sort cho verdict đúng.
6. Yellow cong (polyline cong chữ C nhẹ): điểm test theo segment gần nhất.
7. Lane ngoài phạm vi chiếu (applicability): `UNKNOWN`, không lọc.
8. Hai yellow (đường hiện tại + đường đích crossroad): mỗi lane xử theo vạch
   gần nhất; turn-lane ngược chiều bị `ILLEGAL`, turn-lane đúng chiều `LEGAL`.
9. Dead-zone margin: lane đè lên vạch → `UNKNOWN`, giữ verdict cũ.

Memory/hysteresis:
10. Mất yellow ≤ 10 frame: verdict giữ nguyên từ reference cũ; frame 11:
    mọi lane `UNKNOWN`.
11. Verdict flip cần 2 frame ổn định (per-lane hysteresis).

Memory/ID churn:
12. Lane id đổi giữa frame (id fallback `obj_<label>_<index>` bị reorder):
    verdict tươi từ hình học vẫn đúng, hysteresis reset không gây flip sai;
    exempt theo id cũ không exempt nhầm lane mới khác vị trí.

Tích hợp (qua filter + planner trên frame lọc):
13. `follow_main`: main-lane MỚI xuất hiện bên trái yellow dọc (không phải
    lane exempt đang bám) → bị lọc → planner không chọn; nếu là lane duy nhất
    → manager giữ committed qua dropout hold rồi RECOVERY (assert đúng hành
    vi dropout chuẩn, codex #11).
14. Lane exempt (đang bám) illegal → KHÔNG bị lọc, path giữ nguyên, chỉ debug
    flag (khớp semantics exempt + auto-return).
15. `lane_change_left` sang other-lane bên kia yellow → other-lane bị lọc ở
    cả hai đường (legacy + obs) → fallback follow-main với
    `hold_reason = lane_change_target_not_detected`, không lặp replan
    `intent_change`.
16. `turn_left` crossroad 2 turn-lane: lane ngược chiều bị lọc, chọn lane còn
    lại (khớp luật xa/gần).
17. Direct-IPM fallback: khi control rơi về nhánh direct IPM, `main_current`
    không bao giờ là lane illegal không-exempt (gate áp trước selection).

Auto-return (F3, multi-frame):
18. Main đang bám `ILLEGAL` 5 frame + có lane legal → intent nội bộ
    `LANE_CHANGE_*` được set, manager commit trajectory lane-change và GIỮ
    qua các frame sau (không bị `intent_change` hủy ở frame kế — codex #2),
    debug `legality_return_active = true`.
19. `ILLEGAL` 4 frame rồi legal lại → không auto-return.
20. Đang committed turn/lane-change thật → auto-return bị suppress.
21. Intent thật từ `/avs/route_intent` đến giữa lúc override active → intent
    thật thắng, override hủy.
22. `legality_return_enabled = false` → chỉ flag debug, không transition.

Gate hoàn thành: theo Gate Hoàn Thành Chung của `README.md` (colcon build
layout user, `pytest -q test/decision_system` pass nguyên trạng, regression
live khi F2/F3 đổi behavior runtime, bảng bằng chứng §8).

## 8. Bảng bằng chứng (điền khi báo xong từng phase)

| Phase | Fixture/metric | Trước | Sau | Test mới (tên + output) | Diff anchor |
|---|---|---|---|---|---|
| F1 | gtest suite / pytest decision_system | 44 gtest, 96 pytest pass | 49 gtest pass (22 test `PlanFLegality.*`, gộp một số case §7 vào chung test), 96 pytest pass nguyên trạng | `PlanFLegality.*` — `decision_trajectory_test` 49 PASSED (2026-07-18, build_user layout) | `lane_legality.hpp` (mới), `decision_trajectory_test.cpp` (append), `trajectory_planner.hpp` (+1 FRIEND_TEST) |
| F2 | fixture decision_system (16 file, không chứa yellow — labels 0/3/4/13/17) | 96 pytest pass | 96 pytest pass nguyên trạng (gate permissive khi không có yellow); metric "frame chọn lane trái yellow = 0" cần fixture thật có yellow — CÒN MỞ, đo khi capture real_* mới | `ControlNodeFiltersBeforeAllConsumers`, `ControlNodeParamDefaults` — 61 PASSED (2026-07-18) | `control_node.cpp` (evaluate+filter trước mọi consumer, ngay sau extract; debug `yellow_gate` trong publish_lane_state; params `legality_*`) |
| F3 | multi-frame gtest tổng hợp (dashed-yellow divider) | — | trigger đúng frame debounce=5, hướng đúng, manager COMMIT lane_change và KHÔNG replan `intent_change` frame kế | `AutoReturnTriggersAndManagerCommitsAndHolds`, `AutoReturnNotTriggeredWhenIllegalClearsBeforeDebounce`, `AutoReturnSuppressedWhenIneligible`, `AutoReturnHoldsLaneWhenNoLegalTarget`, `AutoReturnAcrossSolidYellowStaysBlocked`, `ControlNodeRealIntentClearsOverride` — 61 PASSED | `lane_legality.hpp` (`LegalityAutoReturn`), `control_node.cpp` (khối auto-return sau `update_lane_state`, hủy override trong route_intent_callback + cmd reset) |

## 9. Rủi ro & giới hạn đã biết

- Yellow reference giữ qua hold window không bù ego-motion (không odometry) —
  lệch ~100–300 mm dọc trục Y; chấp nhận với gate lateral, theo dõi qua debug.
- Misdetection solid-yellow ↔ dashed-yellow của model sẽ bật/tắt gate; đã
  chặn bằng quality gate + hysteresis + debounce, nhưng cần quan sát fixture
  thật (F2 evidence).
- Lane id không ổn định giữa frame → per-lane hysteresis mất tác dụng khi id
  đổi; verdict tươi từ hình học vẫn đúng, chỉ mất phần làm mượt.
- Gián đoạn quy ước dấu tại biên β (~20° so với phương ngang): hiếm gặp trên
  track; hysteresis band ±5° quanh biên.
- Dời `PathObservationBuilder::build` lên trước `update_lane_state` là thay
  đổi thứ tự thuần (build không có side effect, chỉ đọc telemetry) — xác nhận
  bằng test cũ pass nguyên trạng khi `legality_gate_enabled = false`.
- **Auto-return vượt ngược solid-yellow bị chặn (2026-07-18, cần user quyết):**
  intent nội bộ đi qua marking gate hiện có, mà
  `is_lane_change_blocked_by_solid_obs` coi solid-yellow là blocking → khi vạch
  phân cách là solid-yellow, lane-change về làn đúng bị `blocked_by_marking`
  (giữ follow_main, abort sau `intent_abort_frames`, rồi debounce re-trigger —
  xe KHÔNG bao giờ tự vẽ path cắt vạch liền). Auto-return chỉ hoàn tất qua
  dashed-yellow hoặc khi không có vạch liền giữa hai lane. Đây là hệ quả chủ
  đích của "không sửa marking gate" (§3); nếu muốn cho phép cắt vạch liền để
  về đúng làn, cần user duyệt một exemption riêng cho intent
  `legality_gate`-sourced. Pin bằng test `AutoReturnAcrossSolidYellowStaysBlocked`.
- Khi `legality_gate_enabled = false`: filter trả về frame nguyên vẹn, mọi
  hành vi cũ giữ nguyên bit-for-bit (đường rollback trên Pi).

## 10. Review log

- 2026-07-18: codex (codex-cli 0.144.1, read-only) review bản draft đầu — 12
  finding, tất cả đã xử lý trong bản này: #1/#2 → auto-return đổi sang
  internal intent override (§4), #3/#4 → filter áp cho cả legacy `lanes` +
  xác nhận direct-IPM qua test 17, #5 → test 13/14 tách exempt/không-exempt,
  #6 → unit vector cho signed distance, #7 → chỉ average điểm trong span,
  #8 → PCA linearity guard, #9 → polygon fallback, #10 → test 12 ID churn,
  #11 → test 13 assert dropout-hold semantics, #12 → §5 bổ sung đầy đủ file
  và ghi rõ file KHÔNG đổi.
- 2026-07-18: codex review lượt 2 trên code F1 — 4 blocker + 3 minor, đã fix
  hết trong F1: (1) verdict cũ bị giữ vô hạn khi lane rời applicability span
  → phân biệt dead-zone (giữ) vs not-applicable (decay qua flip hysteresis,
  test `LaneLeavingSpanDecaysToUnknown`); (2) thiếu confidence gate cho yellow
  → param `min_yellow_confidence=0.30` (test `LowConfidenceYellowIgnored`);
  (3) hold all-or-nothing khi 1 trong 2 vạch dropout → per-reference hold
  theo `marking_id` (test `PartialYellowDropoutHoldsMissingRef`); (4) polygon
  fallback tạo segment xuyên bề dày mask → collapse-to-centerline binning
  theo trục chính, áp cho mọi nguồn điểm (test
  `PolygonFallbackRectangleCenterline`); (5) guard NaN tại ingestion (test
  `NanPointsAreIgnored`).
