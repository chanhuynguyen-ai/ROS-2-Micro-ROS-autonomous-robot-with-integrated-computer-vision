# Plan C: Normalizer Arc-Length + Manager Replan Policy

Tương ứng roadmap refactor **Phase 5 + Phase 6**. Mục tiêu: trajectory ổn định, ít rung, và mọi lần đổi trajectory giải thích được — điều kiện để xe "đi đúng làn" mượt qua nhiều frame thay vì đúng từng frame đơn lẻ.

Phụ thuộc: Plan A (flow intent thống nhất). Nên làm sau Plan B để rule đã đúng trước khi tinh chỉnh độ ổn định.

## 1. Hiện Trạng (anchor `control_node.cpp`, khảo sát 2026-07-03)

### TrajectoryNormalizer (dòng 1208–1278)

- Blend theo **index**: `s = i * 100.0` — giả định cả hai path luôn resample đúng 100mm và điểm i của path cũ tương ứng điểm i của path mới. Roadmap Phase 5 cấm giả định này (path bị append/truncate khác nhau, xe đã tiến lên giữa 2 frame nên điểm 0 của path mới ≠ điểm 0 của path cũ).
- **Append đuôi path dài hơn một cách mù quáng** (dòng ~1261–1270): nếu previous dài hơn, đuôi previous được nối thẳng vào — có thể tạo path lai sai topology khi candidate đã rẽ hướng khác (roadmap cấm rõ).
- Không có ước lượng `progress_s_mm` của xe trên committed trajectory → không cắt phần đã đi qua trước khi blend.
- Không kiểm tra continuity heading/curvature bound/minimum length sau blend.
- Không có guard "không blend hai trajectory khác `trajectory_kind`".

### TrajectoryManager (dòng 1287–1494)

- `calculate_path_deviation` (:1479) = mean |Δx| theo index chung — bỏ qua heading, curvature, overlap, topology; và lệch index làm metric vô nghĩa khi 2 path khác chiều dài.
- So intent bằng **string** (`trajectory_kind_name(prev) != current_intent`) (:1305) — Plan A đổi sang enum+seq; plan này kế thừa.
- `path_diff > 800.0` → **replan bất kể confidence** của candidate (roadmap Phase 6: không commit candidate confidence thấp chỉ vì deviation lớn).
- Hold window dropout = 5 frame hard-code; maneuver fallback grace = 1 frame; các ngưỡng 800/50 mm là magic number.
- `progress_s_mm` gần như luôn 0 hoặc copy nguyên — chưa có tracking tiến độ thực.

## 2. Thay Đổi Dự Kiến (cần user duyệt trước khi code)

### C1. Resample + progress alignment (Phase 5)

- Helper `resample_by_arclength(points, step_mm)` — dùng chung cho candidate và committed (step 100mm, parameter).
- Ước lượng `progress_s_mm`: chiếu ego (gốc vehicle frame, x=0,y=0) lên committed trajectory → điểm gần nhất → arc-length từ đầu path. Cập nhật mỗi frame vào `CommittedTrajectoryState.progress_s_mm` / `remaining_s_mm`.
- Trước khi blend: cắt committed từ `progress_s_mm` trở đi, align với candidate theo `s` chung (đoạn overlap).

### C2. Blend theo s + guard hình học (Phase 5)

- Giữ công thức trọng số hiện có (gần xe ưu tiên previous, xa ưu tiên current, scale theo confidence) nhưng chạy trên miền `s` đã align thay vì index.
- Ngoài đoạn overlap: chỉ lấy từ **candidate** (không append đuôi previous nếu candidate ngắn hơn — thay bằng giữ candidate và ghi `normalization_metrics.truncated_prev=true`), trừ khi candidate ngắn hơn `min_path_length_mm` thì trả invalid để manager xử lý hold/recovery.
- Không blend khi `candidate.trajectory_kind != previous.trajectory_kind` — passthrough với `normalization_mode = "kind_mismatch_passthrough"`; quyết định transition thuộc manager.
- Sau blend kiểm: max heading step giữa 2 điểm liên tiếp, max curvature, min length. Vi phạm → fit lại đoạn lỗi bằng cubic Hermite (helper smoothing đã có trong codebase cho transition) hoặc trả invalid kèm metric.
- Output `normalization_metrics`: `lateral_dev_mean/max`, `heading_dev_max`, `overlap_ratio`, `blend_w_cur_range` — publish gọn trong `/avs/lane_state` (chỉ vài số, không phải array).

### C3. Manager deviation metric tổng hợp (Phase 6)

Thay `calculate_path_deviation` bằng struct metric tính trên miền s chung:

```cpp
struct PathDeviationMetrics {
    double lateral_rms_mm;
    double heading_rms_rad;
    double curvature_max_delta;
    double overlap_ratio;        // đoạn s chung / chiều dài committed còn lại
    bool   topology_changed;     // target_lane_id / trajectory_kind đổi
};
```

Chính sách (mỗi ngưỡng là parameter):

- REPLAN chỉ khi: intent (enum,seq) mới; maneuver hoàn tất; committed invalid thật (quá hold window / quá ngắn / vượt curvature-heading limit); blocked bởi rule; hoặc `topology_changed && lateral_rms > ngưỡng && candidate.confidence >= min_commit_confidence`.
- Deviation lớn nhưng confidence thấp → KHÔNG commit; `HOLD` + `hold_reason = "low_confidence_high_deviation"`, tăng bộ đếm; quá `low_conf_hold_frames` → RECOVERY.
- HOLD khi dropout ngắn hạn / candidate khác biệt nhẹ / maneuver candidate tạm mất (đồng bộ bộ đếm dropout với Plan A — một nguồn duy nhất).
- Hold window theo cả frame VÀ quãng đường còn lại (`remaining_s_mm` cạn → không hold nữa).

### C4. Không đổi

- `/avs/control_error` schema và cách chiếu lookahead/theta/curvature (`publish_control_error_from_trajectory`) — mọi cải thiện ổn định phải đến từ trajectory tốt hơn, không từ đổi công thức projection.
- Gate direct-IPM Phase 7 giữ nguyên điều kiện match.

## 3. Thiết Kế Unit Test

Vị trí: `test/decision_system/test_plan_c_stability.py`. Test normalizer/manager thuần hình học nên chạy được qua harness không cần ROS.

| Test | Setup | Assertion |
|---|---|---|
| `test_progress_estimation_on_straight` | committed thẳng 5m, ego đã tiến 1.2m (path mới bắt đầu trễ) | `progress_s_mm ≈ 1200 ± 100` |
| `test_blend_aligned_not_by_index` | previous 30 điểm, candidate 20 điểm lệch dọc 500mm nhưng cùng đường | blended path bám đường thật; so với blend-theo-index cũ phải hết lệch ảo |
| `test_small_jitter_smoothed` | 2 frame giống nhau lệch lateral 20–50mm | output lệch < input; không snap; `epsilon_x` jitter giảm so baseline |
| `test_no_blind_tail_append_on_topology_change` | previous follow_main dài, candidate turn ngắn khác hướng | KHÔNG có điểm nào của đuôi previous trong output; mode `kind_mismatch_passthrough` |
| `test_blend_respects_confidence` | candidate confidence 0.1 vs 0.9, cùng lệch 100mm | w_cur thấp hơn rõ rệt khi confidence thấp (đo qua output gần previous hơn) |
| `test_post_blend_continuity_guard` | candidate có điểm outlier tạo gãy heading > ngưỡng | output không còn gãy (đã re-fit) hoặc invalid + metric ghi rõ |
| `test_deviation_metrics_composite` | 2 path cùng lateral nhưng khác heading mạnh | `heading_rms` bắt được khác biệt mà lateral_rms ≈ 0 |
| `test_low_confidence_high_deviation_holds` | candidate lệch 900mm, confidence 0.1 | `HOLD`, không `COMMIT_NEW`; sau `low_conf_hold_frames` → RECOVERY |
| `test_high_confidence_topology_change_commits` | candidate lệch lớn, confidence cao, target lane đổi | `COMMIT_NEW`, `replan_reason` rõ |
| `test_hold_window_by_remaining_distance` | committed còn 300mm, dropout | không hold path cạn — vào recovery/replan |
| `test_dropout_hold_then_recovery` (regression Phase 6 cũ) | dropout > hold window | ENTER_RECOVERY như hiện tại |
| `test_no_replan_storm_on_noise` | 30 frame nhiễu ±40mm | tổng số `COMMIT_NEW` == 1 (lần đầu) |

Replay metric test (dùng hạ tầng simulator có sẵn): chạy `follow_main_curve.json` và `follow_main_straight.json`, assert jitter `epsilon_x_mm`/`theta_rad` **không tăng** so baseline lưu trước khi sửa; mục tiêu giảm ≥ 20% trên curve (soft target — báo cáo số, không fail test nếu chỉ đạt một phần, nhưng phải giải thích).

## 4. Điều Kiện Ràng Buộc Hoàn Thành

- [~] Baseline jitter/replan-count: **báo cáo forward đã lưu** ở `../local_post_inference_simulator/stability_baseline_metrics.md` (13 fixture live, 2026-07-05); jitter trọng tâm ≈0 (curve 0.0mm, straight 0.2mm). **Không có** baseline "trước khi sửa" lịch sử (không lưu tại thời điểm implement C) → không so sánh before/after được; regression-freedom chứng minh bằng live 13/13 pass thay thế.
- [x] Toàn bộ test bảng trên pass; test cũ + test Plan A/B pass nguyên trạng. *(verified 2026-07-05: `pytest -q test/decision_system` 93/93.)*
- [x] Gate chung README pass: `colcon build` clean + `pytest` 93/93 + gtest 11/11 + `AVS_REQUIRE_LIVE_ROS=1 pytest -m ros` **13/13** (vượt 6/6 yêu cầu). *(verified 2026-07-05.)*
- [x] Mọi lần `COMMIT_NEW` trong replay đều có `replan_reason` thuộc danh sách enum đã định nghĩa. *(test `test_replan_reason_enum_matches_control_node`/`_harness` pass.)*
- [x] Không còn magic number 800/50/5 trong manager — tất cả thành parameter có tên. *(2026-07-05: 800/50 = `replan_lateral_rms_mm`/`hold_lateral_rms_mm`; hai dropout-window `5` gộp về `maneuver_dropout_hold_frames` — trước đó `trajectory_manager.hpp:78` còn hard-code `5`, nay dùng chung một nguồn theo C3.)*
- [~] Diff logic (control_node.cpp + trajectory_manager.hpp) — **chờ user duyệt qua bước commit** (user tự commit sau khi review, theo yêu cầu phiên làm việc này).

## 5. Rủi Ro

- Thay metric deviation đổi điểm quyết định replan → fixture cũ có thể fail: xử lý từng case, phân biệt "fixture kỳ vọng behavior cũ sai" vs "code mới sai".
- Progress estimation sai trên path cong gắt có thể cắt nhầm đoạn gần xe → test riêng trên turn trajectory trước khi bật cho maneuver.
- Blend theo s tốn CPU hơn blend index — đo thời gian callback trên Pi 5 (mục tiêu: không tăng quá ~1ms/frame; nếu vượt, giảm số điểm resample).
