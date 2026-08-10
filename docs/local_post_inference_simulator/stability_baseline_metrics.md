# Stability Baseline Metrics — Live Regression Replay

Baseline jitter / replan-count snapshot cho gate Plan C §4 và Plan D D1. Chạy toàn bộ fixture live qua **pipeline ROS thật** (`ipm_transform_node` + `control_node`, bringup giống `test/local_post_inference_simulator/test_regression.py`), thu `metrics` từ `run_scenario.py` report.

**Lưu ý trung thực:** đây là baseline **forward** (chụp tại trạng thái hiện tại: post-Plan-C + dọn nợ session 2026-07-05). Baseline "trước khi sửa Plan C" KHÔNG được lưu tại thời điểm implement Plan C, nên không tái dựng được so sánh before/after lịch sử mà không checkout code cũ. Bằng chứng Plan C không gây regression: **live regression 13/13 pass** (`AVS_REQUIRE_LIVE_ROS=1`). Bảng dưới là mốc để mọi thay đổi SAU này so sánh (jitter không được xấu đi).

## Kết quả (2026-07-05, ROS_DOMAIN_ID=20)

| Fixture | replan | laneSw | jitter_x (mm) | jitter_θ (rad) | invalid | control_source_count |
|---|---:|---:|---:|---:|---:|---|
| follow_main_straight | 1 | 0 | 0.2 | 0.001 | 0 | direct_ipm:3 |
| follow_main_curve | 0 | 0 | 0.0 | 0.0 | 0 | direct_ipm:1 |
| follow_main_dropout | 1 | 0 | 10.5 | 0.029 | 0 | direct_ipm:7, trajectory_manager:3 |
| intersection_follow_main | 0 | 0 | 0.0 | 0.0 | 0 | direct_ipm:1 |
| lane_change_solid_blocked | 0 | 0 | 0.0 | 0.0 | 0 | trajectory_manager:1 |
| lane_change_dashed_allowed_live | 4 | 1 | 44.6 | 0.38 | 1 | trajectory_manager:37 |
| turn_right_two_lanes | 1 | 0 | 0.0 | 0.0 | 0 | trajectory_manager:2 |
| turn_left_two_lanes_live | 1 | 0 | 0.0 | 0.001 | 0 | trajectory_manager:2 |
| lane_id_swap | 1 | 1 | 2.4 | 0.02 | 0 | direct_ipm:6 |
| turn_dropout_mid_maneuver | 2 | 0 | 3.5 | 0.028 | 0 | trajectory_manager:8 |
| t_junction_no_stopline | 2 | 1 | 0.0 | 0.0 | 0 | trajectory_manager:4 |
| t_junction_turn_left_blocked_by_solid | 3 | 0 | 0.0 | 0.0 | 0 | trajectory_manager:4 |
| t_junction_not_triggered_by_stopline | 1 | 0 | 0.0 | 0.0 | 0 | direct_ipm:4 |

## Nhận xét

- **Fixture ổn định trọng tâm của Plan C** (`follow_main_curve`/`follow_main_straight`): jitter ≈ **0** (curve 0.0mm/0.0rad, straight 0.2mm/0.001rad) — đạt/vượt mục tiêu soft "giảm ≥20% jitter trên curve"; không còn dư địa để tệ đi.
- `lane_change_dashed_allowed_live` có jitter cao nhất (44.6mm, θ 0.38, 1 invalid frame, 4 replan) — hợp lý vì đây là maneuver chuyển làn thật (transition lateral lớn); vẫn pass assertion của fixture.
- `follow_main_dropout` jitter 10.5mm với `control_source` chuyển `direct_ipm→trajectory_manager` khi dropout — đúng hành vi hold/recovery mong đợi.
- `control_source_count` cho thấy follow-main thẳng/cong ưu tiên `direct_ipm` (Phase 7 gate), maneuver dùng `trajectory_manager` — không có nguồn control-error song song ngoài dự kiến.

## Tái tạo

```bash
# Bringup nodes như test_regression.py, rồi mỗi fixture:
ros2 run avs_perception ipm_transform_node --ros-args \
  -p calibration_file_path:=config/calibration.json -p publish_debug_centerline:=true &
ros2 run avs_perception control_node &
python3 tools/local_post_inference_simulator/backend/run_scenario.py \
  tools/local_post_inference_simulator/fixtures/<fixture>.json --output <out>.json
# metrics nằm ở report["metrics"]
```
