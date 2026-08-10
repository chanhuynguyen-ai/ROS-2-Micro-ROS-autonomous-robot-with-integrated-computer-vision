# Plan E — IPM Horizon Clipping & Far-Field Robustness (root-fix BEV/path lệch + rẽ không kích hoạt ở ngã 4)

> Trạng thái: **E1 + E2 ĐÃ LAND (2026-07-06, user đã duyệt §2)** — bảng bằng chứng §4 đã điền đủ, gate pass. Fixture đa-frame + assertions crossroads (§3.3/§6) ĐÃ XONG cùng ngày, xác nhận cơ chế `t_junction_counter_` commit turn_right qua nhiều frame. E3 vẫn TÙY CHỌN chờ số liệu `crossroads_sharp.json`.
> Lịch sử: v2 duyệt ngày 2026-07-06; v3 (bản này) cập nhật sau khi land: mục §2.E1.2-3 đổi từ "loại bỏ điểm" sang "clip cạnh có nội suy" — xem ghi chú deviation trong §2.E1.
> v2 sửa lại chẩn đoán decision-layer của v1: v1 quy sai cho "Plan A chưa land"; thực tế Plan A–D đã land (commit `447eda2`…`4c4b3df`). Cơ chế thật đã trace bằng số liệu ở §1.4–1.5.
> Docs vision liên quan: `../vision/homography_theory.md`, `../vision/pixel_to_world_plan.md`, `../vision/homography_implementation_plan.md`.

## 0. Vì sao có plan này

Chạy step trên fixture ngã 4 (`tools/local_post_inference_simulator/fixtures/crossroads.json`, `route_intent = turn_right`) cho kết quả BEV/World View hỏng: các đoạn trắng/vàng chỉ là sợi ngắn, green (active trajectory) rất ngắn, và xe KHÔNG rẽ dù intent `turn_right`. Làn thẳng đơn thì path mượt; đường cong và rẽ thì path vẽ lệch khỏi làn. Output tham chiếu ("trước"): `tools/local_post_inference_simulator/outputs/latest_test_IPM_output.json` (đã dời từ `fixtures/` — xem §4).

## 1. Chẩn đoán (mọi con số dưới đây tái lập được từ 2 file fixture/output ở trên)

### 1.1 Nguyên nhân gốc: IPM thiếu horizon clipping

Homography trong `config/calibration.json`:

```
H = [ -0.696146  -0.047086   238.6976 ]
    [ -0.036671   0.746617  -509.1544 ]
    [  8.2689e-5 -6.6191e-3   1.0      ]
```

Đường chân trời (nơi mẫu số `w = h20*u + h21*v + h22 → 0`) nằm tại **v ≈ 151–159 px** (ảnh cao 480):

- u=0   → v_horizon ≈ 151.1
- u=320 → v_horizon ≈ 155.1
- u=640 → v_horizon ≈ 159.1

Mọi pixel có `v` nhỏ hơn ~155 (phía trên horizon) back-project ra **|vô cực| hoặc âm** (sau lưng xe). Guard duy nhất hiện tại là `std::abs(w) > 1e-6` tại `ros2_ws/src/avs_perception/src/ipm_transform_node.cpp:609` — **quá yếu**: `w` nhỏ nhưng > 1e-6 vẫn cho X,Y nổ tới hàng chục–hàng trăm mét, và khi pixel vượt horizon thì đổi dấu (ra sau xe).

### 1.2 Bằng chứng từ crossroads.json → latest_test_IPM_output.json

| object | pixel v | world y (mm) | world_point / waypoint | đánh giá |
|---|---|---|---|---|
| main-lane_1 (gần) | 287–453 | [92, 357] | 54 / 3 | ✅ hợp lệ |
| **main-lane_2** (xa) | 45–152 | **[-19930, -668]** | 54 / **193** | ❌ 19.9m sau lưng xe |
| **other-lane_2** | 37–154 | **[-480968, -633]** | 136 / **4803** | ❌ -480 mét, 4803 waypoint rác |
| **solid-yellow_3** | 46–152 | [-85691, **272959**] | 78 | ❌ ±hàng trăm mét |
| turn-lane_1 | 155–198 | [1212, 16745] | 40 / 85 | ⚠ sát horizon, phình lớn |
| turn-lane_4 | 163–210 | [1033, 12351] | 12 / 46 | ⚠ phình lớn (waypoint tới y=12206) |
| turn-lane_2/3 | 163–266 | [448, 948] | — / 4,3 | ✅ hợp lệ |

`ipm_summary.world_bounds`: `y ∈ [-480967.9, 272958.7]`, `x ∈ [-17108.6, 73696.1]` → trải **~750 mét**.

### 1.3 Cơ chế gây triệu chứng hiển thị (BEV + green path)

1. **BEV hiển thị tệ (sợi trắng/vàng ngắn)**: world_bounds trải ±hàng trăm mét → auto-scale của BEV zoom out cực đại → làn gần thật (0.1–0.4m) co lại thành sợi tí xíu. Hệ quả rendering; **sửa nguồn (IPM) là sửa được**, không cần đụng renderer.
2. **Green (active trajectory) rất ngắn**: `committed_trajectory_id = main-lane_2`. `active_trajectory_points` có 52 điểm, y từ **-19900 → 300mm**; 51/52 điểm ở y≈-19000mm (ngoài khung), chỉ ~1 điểm gần y≈300mm hiện ra.
3. **Làn thẳng mượt, cong/rẽ lệch**: làn thẳng đơn nằm trọn dưới horizon (v>155) → chiếu bounded → path mượt. Khi đường cong/rẽ, phần xa của làn dâng lên gần horizon trong ảnh (v→155) → nổ số → kéo polynomial fit + waypoints lệch khỏi làn.

### 1.4 Chuỗi nhân quả khiến rẽ KHÔNG kích hoạt (trace từng bước, số thật)

**Đính chính so với v1**: KHÔNG phải "Plan A chưa land". Dispatcher `TrajectoryPlanner::plan_candidate_for_intent` tồn tại (`trajectory_planner.hpp:308`) và được gọi tại `control_node.cpp:362`. Chuỗi lỗi thật:

1. **Chọn nhầm main lane**: `select_main_current` (`trajectory_planner.hpp:742-795`) score `= |start_x| + 0.5*start_y` (dòng 777). Với dữ liệu bị nổ:
   - `main-lane_1`: start=(18.6, 100) → score **68.6**
   - `main-lane_2`: start=(211, **-19900**) → score **-9739** ← thắng sai vì start_y âm khổng lồ
2. **Turn-lane selection vẫn đúng**: `select_turn_lane_obs` chọn `turn-lane_3` (avg_x=200 RIGHT, min_dist=527) — không phải mắt xích lỗi.
3. **Transition guard reject**: `plan_transition` (`trajectory_planner.hpp:654`) reject khi `heading_diff > 40°`. Heading của `main-lane_2` = -6.7° vs `turn-lane_3` = 41.0° → diff **47.8° > 40° → REJECT** (trả về `{}`).
4. **Fallback nuốt cú rẽ**: `plan_turn_generic` thấy transition fail → fallback `plan_follow_main`, gắn kind `FOLLOW_MAIN` (`trajectory_planner.hpp:164-167`) → candidate/normalized/committed đều `follow_main` trên path rác của main-lane_2 (khớp `debug_trajectories` trong output: cả 3 stage đều `follow_main`, 204 điểm từ [211,-19900]).
5. **`hold_reason` rỗng** vì `is_turn_commit_ready` pass (turn-lane_3 `longitudinal_offset_mm=402.7` < `turn_proximity_mm_`) — state machine không hold, nó commit đúng cái candidate đã bị fallback.

### 1.5 Counterfactual (bằng chứng E1 là root-fix, tính từ chính output hiện tại)

Nếu chọn đúng `main-lane_1` (điều sẽ xảy ra sau E1, vì main-lane_2 mất hết waypoint hợp lệ):

- `plan_transition(main-lane_1, turn-lane_3)`: lat_dist=81mm, heading 2.9° vs 41.0° → diff **38.2° < 40° → PASS** → turn candidate ĐƯỢC sinh.

⚠ Margin chỉ **1.8°** dưới guard 40° → mong manh với ngã 4 vuông hơn (turn lane càng vuông góc, heading waypoint càng lớn). Đây là lý do có E3 (tùy chọn, cần duyệt riêng) ở §2.

## 2. Phương án cải thiện

Chốt hướng (user đã duyệt sơ bộ): **Valid-region clip tại nguồn IPM** + **guard chống nhiễu ở selection**. Không đổi phương pháp chiếu (vẫn planar homography).

### E1 — IPM valid-region clip (root-fix, ưu tiên 1) — ✅ ĐÃ LAND

**File:** `ros2_ws/src/avs_perception/src/ipm_transform_node.cpp` + header mới `include/avs_perception/bev_region.hpp`
**Đã implement (khớp danh sách duyệt, trừ deviation ghi rõ ở mục 2-3):**

1. **Hằng vùng BEV hợp lệ** — struct `BevRegion` trong `bev_region.hpp`, param ROS `bev_horizon_margin_px` (default 10), `bev_y_max_mm` (8000), `bev_x_abs_max_mm` (4000); `y_min = 0` cố định. Horizon theo cột u: `v_h(u) = -(h20*u + h22)/h21` (`BevRegion::horizon_v`).
2. **⚠ DEVIATION (đã kiểm chứng bằng regression):** bản duyệt ghi "**loại bỏ** điểm" ngoài vùng. Implement đầu tiên đúng nguyên văn (vertex-drop) làm **vỡ 2 fixture regression** (`follow_main_straight`, `lane_id_swap`): fixture direct-mode chỉ có 4 đỉnh góc, polygon vắt qua horizon bị drop 2 đỉnh trên → còn 2 điểm → suy biến → mất toàn bộ waypoint → không chọn được lane. Bản land dùng **clip cạnh có nội suy giao điểm (Sutherland–Hodgman)** — `BevRegion::clip_and_project`: (a) clip pixel-space theo đường horizon+margin, (b) chiếu qua H (homography bảo toàn đường thẳng nên nội suy trước khi chiếu là chính xác hình học), (c) clip world-space theo box `[y_min,y_max]×[±x_max]`. Đây là cách trung thành với ý định gốc "giữ phần hợp lệ" của polygon. Guard `abs(w) > 1e-6` giữ nguyên làm lớp bảo hiểm cuối.
3. **Polygon suy biến sau clip**: rule "≥ 3 điểm" chuyển xuống bước extract waypoint (skip poly < 3 điểm trước `extract_centerline_waypoints_{x,y}`) thay vì xóa `polygons_real_world` — vì marking polyline 2 điểm vẫn cần cho marking gate.
4. **Hard-cap waypoint** `kMaxWaypointsPerObject = 128` (`bev_region.hpp`) guard cả 2 vòng regenerate (turn-lane x-sweep + main/other y-sweep) + `RCLCPP_WARN_THROTTLE` khi chạm cap.
5. **Logic clip là header thuần** `bev_region.hpp` (không ROS) — gtest test trực tiếp (`BevRegion.*`).

**Không đổi:** schema `/avs/control_error`, công thức homography, đơn vị/trục. Contract test pass (xem §4).

### E2 — Guard chống nhiễu ở selection (ưu tiên 2, sau E1) — ✅ ĐÃ LAND

**File:** `ros2_ws/src/avs_perception/include/avs_perception/trajectory_planner.hpp` (runtime candidate path) **và** `ros2_ws/src/avs_perception/include/avs_perception/legacy_lane_model.hpp` (runtime state-machine path — cả hai đều được `control_node.cpp` dùng: `split_main_lanes` tại `control_node.cpp:315`, `plan_candidate_for_intent` tại `:362`).
**Đã implement (đúng danh sách duyệt):**

1. Hằng chung `kMinPlausibleLaneStartYMm = 0` / `kMaxPlausibleLaneEndYMm = 10000` trong `decision_types.hpp`; reject trước khi score, đồng bộ ở cả hai chỗ:
   - `TrajectoryPlanner::select_main_current`: loại candidate có `points.front().y < 0` (bắt đầu sau xe) hoặc `points.back().y > 10000`.
   - `LegacyLaneModel::split_main_lanes`: cùng điều kiện trên `raw_obj["waypoints"]`.
   - Sau E1 phần lớn tự hết; guard này là defense-in-depth định nghĩa rõ ràng, không dựa vào IPM.
2. KHÔNG thay luật chọn lane trong `../architecture/decision_sys.md`; mọi thay đổi luật đi qua Plan B.

### E3 — Transition guard cho cú rẽ gắt (TÙY CHỌN — quyết định sau khi có số liệu E1)

§1.5 cho thấy fixture ngã 4 hiện tại chỉ pass guard 40° với margin 1.8°. Ngã 4 vuông góc hơn (turn-lane heading → 60–90°) sẽ bị `plan_transition` reject → rẽ lại rơi về follow_main dù dữ liệu sạch.

**Quy trình quyết định (không code trước):** sau khi E1+E2 land, chạy step lại trên crossroads + biến thể "vuông hơn" (fixture mới `crossroads_sharp.json`). Nếu biến thể vuông fail đúng như dự đoán → đề xuất phương án cụ thể (vd: guard theo `is_turn` — nới heading limit cho TURN_*, giữ 40° cho LANE_CHANGE; hoặc đo heading tại điểm nối P0/P3 thay vì 2 điểm đầu path) và **xin duyệt riêng** trước khi sửa `trajectory_planner.hpp:630-740`.

## 3. Test & Fixture (test/fixture trước — Ràng Buộc #6)

1. **gtest offline (chính, theo chính sách Plan D3)** — thêm vào `ros2_ws/src/avs_perception/test/decision_trajectory_test.cpp` (hoặc file test mới cho bev_region):
   - `BevRegion.RejectsAboveHorizon` — pixel v < v_h(u)+margin → loại.
   - `BevRegion.RejectsBehindVehicleAndFarField` — Y<0, Y>y_max, |X|>x_max → loại; điểm hợp lệ → giữ.
   - `BevRegion.WaypointHardCap` — polygon dài bất thường → waypoint ≤ cap.
   - `PlanE.SelectMainRejectsNegativeStartY` (E2) — observation 2 main lane, một cái start_y=-19900 → chọn lane start_y=100.
   - `PlanE.CrossroadsTurnRightPlansTurnCandidate` — dựng `PathObservationFrame` từ waypoints ĐÃ CLIP của fixture crossroads (main-lane_1 + turn-lane_3) → `plan_candidate_for_intent(TURN_RIGHT)` trả `trajectory_kind == TURN_RIGHT`, không phải fallback follow_main.
2. **Regression fixture crossroads**: thêm crossroads vào bộ replay regression với assertion:
   - `world_bounds`: |y| ≤ 10000mm, |x| ≤ 5000mm.
   - Không object nào `waypoint_count > 128`.
   - `committed_trajectory_id == "main-lane_1"` (không phải main-lane_2).
   - `candidate_trajectory_kind == "turn_right"` (sau E1; nếu vẫn fail vì guard 40° → đó là bằng chứng kích hoạt E3, ghi vào báo cáo).
3. **Mở rộng fixture đa-frame** — ✅ ĐÃ LÀM 2026-07-06: `fixtures/crossroads_multiframe.json` (5 frame). Đơn giản hóa có chủ đích so với đề xuất gốc: lặp lại NGUYÊN VẸN cùng 1 frame geometry 5 lần (không dịch polygon dần) — đủ để kích hoạt `t_junction_counter_ >= 3` (`legacy_lane_model.hpp:221`) qua biên giới is_t_geom→is_t thật, tức đúng cơ chế đa-frame cần test, nhưng KHÔNG mô phỏng xe tiến gần liên tục (đó vẫn là việc của §7.1 nếu cần sau này). Verify thật qua `AVS_REQUIRE_LIVE_ROS=1` + mode `direct` (đúng mode `test_regression.py` dùng): frame 1–2 `decision_state=FOLLOW_MAIN, hold_reason=t_junction_pending`; frame 3–5 `decision_state=TURN_RIGHT, selected_lane_id=turn-lane_3, hold_reason=""`, ổn định không flip-flop lại (`selected_lane_switch_count=1`, `replan_count=2`, `invalid_frame_count=0`).
4. **Không hồi quy làn thẳng**: fixture làn thẳng đơn hiện có pass nguyên trạng (path vẫn mượt) — chứng minh clip không cắt nhầm vùng hợp lệ. 6/6 fixture regression cũ giữ pass.
5. **Regression live** (phase đổi behavior runtime):
   ```bash
   AVS_REQUIRE_LIVE_ROS=1 pytest -m ros test/local_post_inference_simulator/test_regression.py
   ```

## 4. Gate Hoàn Thành & Bằng Chứng (bắt buộc — không có bảng bằng chứng thì KHÔNG được báo "xong")

Ngoài Gate chung trong `README.md` (colcon build layout user + `pytest -q test/decision_system` + regression live + contract `/avs/control_error` không đổi), **mỗi phase E1/E2/E3 khi báo hoàn thành phải nộp kèm bảng bằng chứng** theo mẫu dưới — cột "trước" đã điền sẵn từ output hiện tại, cột "sau" điền từ việc chạy step lại trên chính `crossroads.json` và trích số từ `latest_test_IPM_output.json` mới:

| Metric (từ step output crossroads) | Trước (2026-07-06) | Sau (2026-07-06, land E1+E2) | Đạt khi | Kết quả |
|---|---|---|---|---|
| `ipm_summary.world_bounds.y` | [-480968, 272959] | **[92.5, 6104.6]** | trong [0, ~8000] | ✅ |
| `ipm_summary.world_bounds.x` | [-17109, 73696] | **[-3344.3, 2350.4]** | trong [±~4000] | ✅ |
| max `waypoint_count` một object | 4803 (other-lane_2) | **29 (turn-lane_1)** | ≤ 128 | ✅ |
| `committed_trajectory_id` | main-lane_2 | **main-lane_1** | main-lane_1 | ✅ |
| `candidate_trajectory_kind` | follow_main | **turn_right** | turn_right | ✅ (guard 40° KHÔNG chặn — counterfactual §1.5 xác nhận live) |
| `active_trajectory_points` y-range | [-19900, 300] | **[100, 300]** (3 điểm) | toàn bộ ≥ 0 | ✅ |
| gtest mới (§3.1) | chưa có | **18/18 pass** (7 test Plan E: `BevRegion.RejectsAboveHorizon`, `.RejectsBehindVehicleAndFarField`, `.CrossroadsFarPatchRejected`, `.StraddlingPolygonKeepsValidPart`, `.WaypointHardCapWiredIntoIpmNode`, `PlanE.SelectMainRejectsNegativeStartY`, `PlanE.CrossroadsTurnRightPlansTurnCandidate`) | tất cả pass | ✅ |
| fixture regression live | 6/6 pass (bộ cũ) | **15 passed, 0 skipped** (`AVS_REQUIRE_LIVE_ROS=1`; +`crossroads.json` sau khi thêm assertions, +`crossroads_multiframe.json` mới) | vẫn pass | ✅ |
| `pytest -q test/decision_system` | 93 pass | **93 pass** | pass | ✅ |
| colcon build (layout user) | pass | **pass** | pass | ✅ |
| `crossroads_multiframe.json` (5 frame, mode `direct`) frame 1-2 | — | **`decision_state=FOLLOW_MAIN`, `hold_reason=t_junction_pending`** | khớp §1.4/§1.5 hold trước confirm | ✅ |
| `crossroads_multiframe.json` frame 3-5 | — | **`decision_state=TURN_RIGHT`, `selected_lane_id=turn-lane_3`, `hold_reason=""`, `control_source=trajectory_manager`** | commit đúng sau `t_junction_counter_>=3` | ✅ |
| `crossroads_multiframe.json` metrics tổng | — | **`selected_lane_switch_count=1`, `trajectory_kind_switch_count=1`, `replan_count=2`, `invalid_frame_count=0`, jitter=0.0** | không flip-flop sau commit | ✅ |

File output "sau": `tools/local_post_inference_simulator/outputs/crossroads_after_planE.json` (chạy `ScenarioRunner.step()` mode rasterized — cùng mode với bản chạy "trước" của user, đã dời sang `tools/local_post_inference_simulator/outputs/latest_test_IPM_output.json` vì nằm trong `fixtures/` sẽ bị regression sweep quét nhầm như scenario và fail schema).

Diff anchor (đối chiếu §2):
- MỚI `include/avs_perception/bev_region.hpp` — `BevRegion` (horizon_v, accepts_pixel, accepts_world, clip_and_project + Sutherland–Hodgman), `kMaxWaypointsPerObject`.
- `src/ipm_transform_node.cpp` — khai báo 3 param `bev_*` (constructor); vòng chiếu pixel→world thay bằng `clip_and_project`; skip poly < 3 điểm ở 2 nhánh extract; cap + WARN_THROTTLE ở 2 vòng regenerate; member `bev_region_`.
- `include/avs_perception/decision_types.hpp` — thêm `kMinPlausibleLaneStartYMm`, `kMaxPlausibleLaneEndYMm`.
- `include/avs_perception/trajectory_planner.hpp` — guard reject trong `select_main_current`.
- `include/avs_perception/legacy_lane_model.hpp` — guard reject trong `split_main_lanes`.
- `test/decision_trajectory_test.cpp` — 7 test mới.
- MỚI `fixtures/crossroads.json` — thêm block `assertions` (không đổi frame/geometry).
- MỚI `fixtures/crossroads_multiframe.json` — fixture đa-frame (§3.3), 5 frame lặp cùng geometry + `assertions` riêng cho path commit turn_right.
- Ngoài danh sách duyệt: **không có** (deviation edge-clip đã ghi ở §2.E1.2; 2 fixture trên thuộc mục "land-tasks còn lại" §6, không phải thay đổi code hành vi).

Debug `/avs/lane_state` lần chạy "sau" tự giải thích được: `selected_lane_id=main-lane_1`, `candidate=turn_right`, nhưng `decision_state=FOLLOW_MAIN` với **`hold_reason=t_junction_pending`** — state machine giữ cú rẽ vì geometry 1 frame chưa xác nhận được giao lộ (main-ahead bên kia ngã 4 nằm trên horizon nên không nhìn thấy). Đây là hành vi giữ-an-toàn có chủ đích của control_node (`control_node.cpp:395-400`), KHÔNG phải lỗi E1/E2.

**✅ Xác nhận đa-frame ĐÃ LÀM (2026-07-06)**, không còn là known-gap mở: `fixtures/crossroads_multiframe.json` (§3.3) chứng minh bằng chạy thật rằng khi cùng geometry lặp lại đủ 3 frame, `t_junction_counter_` (`legacy_lane_model.hpp:163,215,221`) vượt ngưỡng 3, `is_t_geom→is_t` chuyển true, `t_junction_pending` tắt, và commit đúng `TURN_RIGHT`/`turn-lane_3` — khớp counterfactual §1.5. Quy mô còn lại (mô phỏng polygon dịch dần khi xe tiến gần thay vì lặp nguyên khung) là mở rộng tùy chọn, không chặn gate, chuyển cho Plan B nếu cần độ chân thực cao hơn.

> **CẬP NHẬT 2026-07-07 — fixture đã sửa geometry, timeline hold không còn tái hiện với `crossroads_multiframe.json`:** các object phía bên kia ngã 4 (`main-lane_2`, `other-lane_2`, `solid-yellow_3`) trước đây vẽ Ở TRÊN đường horizon (y_px 37–154 < horizon v≈151–159) nên không thể chiếu xuống ground plane — vô hình với planner, làm ngã 4 trông như T-junction cụt. Fixture đã được dời các object này xuống dưới horizon (v≈181–207, chiếu ra world y≈[1100,2200]mm). Hệ quả đúng-thiết-kế: `main_ahead` giờ nhìn thấy → `is_t_geom=false` → KHÔNG còn hold `t_junction_pending`; intent `turn_right` commit `turn-lane_3` ngay frame đầu (intent-driven qua `plan_turn_generic`), và intent `follow_main` merge `main-lane_1`→`main-lane_2` qua `select_main_ahead`/`merge_lanes` (active trajectory y=[100,2100]mm, 21 điểm). Timeline "frame 1-2 hold, frame 3 commit" ở §3.3/§4 là bằng chứng lịch sử với geometry cũ; muốn tái hiện path `t_junction_counter_` cần fixture T-junction thật (main lane cụt, không có main ahead). `crossroads.json` 1-frame vẫn giữ geometry cũ (far lanes trên horizon) nên evidence table §4 của nó vẫn tái hiện được.

## 5. Thứ tự thực hiện

| Phase | Nội dung | Điều kiện bắt đầu | Điều kiện xong | Trạng thái |
|---|---|---|---|---|
| E1 | IPM valid-region clip + hard-cap waypoint + hàm thuần testable | User duyệt danh sách §2.E1 | Bảng §4 (các dòng world_bounds, waypoint cap, y-range) + gtest BevRegion.* pass | ✅ XONG 2026-07-06 |
| E2 | Guard reject-lane phi lý ở planner + legacy model | E1 xong | Bảng §4 (committed_trajectory_id) + gtest PlanE.SelectMain* pass | ✅ XONG 2026-07-06 |
| E3 | (tùy chọn) transition guard cho rẽ gắt | Có số liệu sau E1+E2 trên `crossroads_sharp.json` chứng minh cần; user duyệt phương án riêng | candidate_trajectory_kind = turn_right trên cả crossroads thường và sharp | ⏳ chờ số liệu |

Không bắt đầu E2/E3 trước E1 (làm trước sẽ che triệu chứng thay vì trị gốc).

## 6. Việc cần làm khi land

- ~~Cập nhật bảng plan trong `docs/plans/README.md`~~ (đã thêm dòng E; đã cập nhật trạng thái E1/E2 xong).
- ~~Ghi param BEV region vào `../vision/pixel_to_world_plan.md`~~ (đã thêm mục "BEV valid region (Plan E1)").
- ~~Cross-check hằng label không đổi (turn-lane=17)~~ (không file label nào bị đụng).
- ~~CÒN LẠI: thêm assertions vào `fixtures/crossroads.json` ... + fixture đa-frame §3.3~~ — ✅ XONG 2026-07-06: `crossroads.json` có `assertions` (`expected_selected_lane=main-lane_1`, `expected_trajectory_kind=follow_main`, `expected_control_source=trajectory_manager`, `max_replan_count=0`) khớp đúng hành vi hold-an-toàn 1-frame (không chốt trên `hold_reason` vì schema `AssertionsSchema` không có field đó); `crossroads_multiframe.json` (mới) có assertions riêng cho path commit turn_right. Regression live: **15 passed, 0 skipped**.
- CÒN LẠI (không chặn gate, chuyển Plan B nếu cần): fixture đa-frame với polygon dịch dần mô phỏng xe tiến gần liên tục (thay vì lặp nguyên khung 5 lần như hiện tại).

## 7. Hướng tối ưu decision/planning tiếp theo (đề xuất — CHƯA cam kết, mỗi mục đều phải có tiêu chí verify như §4 khi thành plan riêng)

Xếp theo tác động tới mục tiêu chung của `README.md` ("rẽ đúng, chuyển làn đúng, đi đúng làn"):

1. **Multi-frame crossroads regression (nối dài Plan D Phase 9)**: fixture 1 frame không chứng minh được commit→hold→complete của maneuver. Thêm kịch bản replay 10–20 frame tiến vào ngã 4 + rẽ, assert theo từng frame: `decision_state` chuyển FOLLOW_MAIN→TURN_RIGHT đúng thời điểm, không flip-flop. Verify: assertion per-frame trong regression, không phải xem mắt.
2. **Sanity gate tập trung ở PathObservationBuilder**: thay vì rải guard ở từng selector (E2), một lớp validate observation duy nhất (y-range, waypoint spacing, monotonic) đánh dấu `lane.suspect=true` để mọi selector bỏ qua thống nhất. Verify: gtest cho builder + đếm selector không còn guard trùng lặp.
3. **Turn transition geometry cho ngã 4 vuông (chính là E3 mở rộng)**: đo heading tại điểm nối thay vì 2 điểm đầu path; hoặc P3 chọn theo arc-length trên turn-lane sau khi vào cua. Verify: fixture `crossroads_sharp.json` (heading 60–90°) plan được TURN_*.
4. **Commit gating dựa confidence + deviation liên tục (nối Plan C)**: hiện `is_turn_commit_ready` chỉ dùng `long_off < turn_proximity_mm`; bổ sung điều kiện candidate turn valid N frame liên tiếp trước khi commit để tránh commit vào candidate vừa fallback. Verify: replay dropout 1–2 frame giữa maneuver không mất cú rẽ (fixture Plan B/D đã có khung).
5. **BEV debug overlay vùng hợp lệ**: simulator vẽ đường horizon + biên bev region lên cả camera view và BEV để khi vẽ fixture, user thấy ngay polygon nào sẽ bị clip. Chỉ đụng `tools/local_post_inference_simulator/` (không đụng web_dashboard). Verify: screenshot trước/sau trong docs simulator.
