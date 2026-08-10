# Kế Hoạch Phát Triển Decision & Trajectory Planning

Folder này chứa các plan triển khai tiếp theo để đạt mục tiêu lớn nhất: **xe rẽ đúng, chuyển làn đúng, đi đúng làn theo luật chọn lane trong `../architecture/decision_sys.md`**.

Các plan được lập dựa trên khảo sát code thực tế (branch `decision_trajectory_refactor`, commit `74ef914`) đối chiếu với:

- `../architecture/decision_sys.md` — luật chọn lane (source-of-truth nghiệp vụ)
- `../architecture/decision_sys_implementation_plan.md`
- `../architecture/trajectory_planning_memory_proposal.md`
- `../architecture/trajectory_planning_memory_implementation_plan.md`
- `../architecture/decision_trajectory_refactor_roadmap.md` — roadmap refactor (nguồn phase tham chiếu chính)

## Ràng Buộc Toàn Cục (áp dụng cho MỌI plan)

1. **Contract `/avs/control_error` đóng băng tuyệt đối**: không đổi tên field, đơn vị, dấu, ngữ nghĩa của `lane_state`, `target_label`, `epsilon_x_mm`, `epsilon_y_mm`, `theta_rad`, `curvature_inv_mm`, `lookahead_d_mm`. Không đổi cách tính theo hướng làm lệch tương thích với controller tầng thấp (Pure Pursuit/PD/ESP32).
2. **Mọi thay đổi vào `control_node.cpp` hoặc `ipm_transform_node.cpp` phải được liệt kê cụ thể (hàm/dòng/hành vi đổi) và được user duyệt trước khi code.** Plan chỉ mô tả thay đổi dự kiến; implement từng phần sau khi duyệt.
3. Không dùng `stop-line` cho kích hoạt rẽ / phát hiện giao lộ / T-junction / chuyển làn.
4. Mỗi frame chỉ publish đúng một `active trajectory`; không có nguồn control error song song không kiểm soát.
5. Label constants: `turn-lane = 20` theo model 22 class hiện tại (KHÔNG phải 17 của model 19 class cũ, cũng không phải 10); mọi label mới gom về constant, sync `config/label_mapping.json` + simulator docs.
6. Test/fixture trước, refactor sau: không đổi behavior khi chưa có test bảo vệ behavior đó.

## Gate Hoàn Thành Chung (điều kiện bắt buộc trước khi báo "xong" một phase/feature)

Mỗi phase trong mọi plan chỉ được coi là hoàn thành khi **tất cả** các mục sau pass:

```bash
cd ros2_ws
colcon --log-base log_user build --symlink-install --packages-select avs_perception --build-base build_user --install-base install_user
```

```bash
pytest -q test/decision_system
AVS_REQUIRE_LIVE_ROS=1 pytest -m ros test/local_post_inference_simulator/test_regression.py   # khi phase đổi behavior runtime
```

Cộng thêm:

- Unit test MỚI được thiết kế trong plan của phase đó pass (không chỉ test cũ).
- Không có test cũ nào bị sửa expected-value để "cho pass" mà không có giải thích được user duyệt.
- Schema `/avs/control_error` không đổi (kiểm bằng test contract hiện có `test_phase1_to_phase5_contract.py`).
- Debug `/avs/lane_state` đủ để giải thích quyết định của phase (theo checklist roadmap).
- Báo cáo ngắn: cái gì đổi, test nào cover, metric trước/sau (nếu phase có metric).
- **Bảng bằng chứng bắt buộc khi báo "xong" phase/plan**: mỗi plan phải định nghĩa sẵn bảng metric trước/sau đo trên fixture cụ thể (xem mẫu §4 `plan_E_ipm_horizon_and_far_field.md`) + tên gtest/pytest mới kèm output thật + diff anchor (hàm/dòng đã đổi đối chiếu với danh sách dự kiến trong plan). Không có bảng bằng chứng điền đủ = chưa xong.

## Hiện Trạng Đã Khảo Sát (2026-07-03)

Đã có và hoạt động:

- Các khối `PathObservationBuilder`, `TrajectoryPlanner`, `TrajectoryNormalizer`, `TrajectoryManager` nằm trong `control_node.cpp` (3661 dòng).
- Debug field `replan_reason`, `dropout_hold_counter`, `trajectory_confidence`, `normalization_mode`, `control_source` đã publish qua `/avs/lane_state`. **Đính chính (2026-07-03, verify lại code hiện tại)**: `manager_action`, `hold_reason`, `commit_allowed`, `candidate_trajectory_kind` CHƯA tồn tại trong `control_node.cpp` (không có trong `TrajectoryManager::Decision`, không có trong `publish_lane_state`) — đây là field MỚI cần thêm ở Plan A mục A5, không phải field có sẵn cần "bảo đảm được set".
- 42 unit test pass (`test/decision_system`), 6/6 fixture live regression pass, mapping phase↔fixture trong `../local_post_inference_simulator/scenario_refactor_mapping.md`.
- Bug turn-lane 10 vs 17 trong IPM đã fix (dùng `LABEL_TURN_LANE`).

Chưa đúng roadmap (căn cứ code, có anchor dòng trong từng plan):

| Vấn đề | Roadmap phase | Plan xử lý |
|---|---|---|
| Nhánh `FOLLOW_MAIN` bỏ qua pending intent; maneuver kích hoạt bằng khoảng cách `turn_proximity_mm_` trong state machine | Phase 1 | Plan A |
| Mất turn-lane 1 frame → reset ngay `current_intent_` về `FOLLOW_MAIN`, không có hold window | Phase 2 | Plan A |
| Manager so intent bằng string; deviation = mean |Δx| theo index; replan khi deviation lớn bất kể confidence | Phase 6 | Plan C |
| Normalizer blend theo index (`s = i*100mm`), append đuôi path dài hơn một cách mù quáng, không có progress alignment | Phase 5 | Plan C |
| Luật chọn turn-lane gần/xa, T-junction, marking gate cần audit độ phủ so với `decision_sys.md` | Phase 4, 8 | Plan B |
| Còn 2 magic `!= 17` (`control_node.cpp:266`, `:3090`); Python harness 1666 dòng mirror logic C++; chưa tách file | Phase 3, 10 | Plan D |
| Regression chưa có: dropout giữa maneuver, ID swap, 2 turn-lane cùng phía khác khoảng cách | Phase 9 | Plan B, D |

## Danh Sách Plan Và Thứ Tự Thực Hiện

| # | File | Nội dung | Phụ thuộc |
|---|---|---|---|
| A | `plan_A_intent_driven_planning.md` | Planner sinh candidate theo intent ở mọi frame; intent latch + hold window | — |
| B | `plan_B_lane_rule_conformance.md` | Audit + hoàn thiện luật chọn turn-lane gần/xa, T-junction, marking gate theo `decision_sys.md` | A |
| C | `plan_C_normalizer_manager_stability.md` | Normalizer arc-length, manager deviation metric tổng hợp + replan policy | A (nên sau B) |
| D | `plan_D_regression_and_debt.md` | Mở rộng replay regression, dọn magic label, chính sách harness, tách file (Phase 10) | A, B, C |
| E | `plan_E_ipm_horizon_and_far_field.md` | IPM horizon clipping / valid-region (root-fix BEV & path lệch trên cong/rẽ ở ngã 4); guard reject lane phi lý. **E1+E2 XONG 2026-07-06** (bằng chứng §4 trong plan); fixture đa-frame + assertions crossroads XONG cùng ngày (regression live 15 passed, 0 skipped); E3 tùy chọn chờ số liệu | độc lập; E2 nên sau A |
| F | `plan_F_solid_yellow_legality_gate.md` | Gate hợp lệ lane theo solid-yellow (signed side test + PCA/β-band, memory hold, auto-return qua internal intent override). **F1+F2+F3 CODE XONG 2026-07-18** (61 gtest, 96 pytest nguyên trạng); còn mở: fixture thật có yellow + quyết định §9 về vượt ngược solid-yellow (bị marking gate chặn chủ đích) | A; độc lập B/C |

Lý do thứ tự: A sửa lỗi kiến trúc chặn trực tiếp mục tiêu "rẽ đúng/chuyển làn đúng" (intent bị nuốt), B đảm bảo đúng LUẬT, C đảm bảo ỔN ĐỊNH, D khóa lại bằng regression và trả nợ kỹ thuật. Không bắt đầu bằng tách file (D) — đúng khuyến cáo roadmap.
