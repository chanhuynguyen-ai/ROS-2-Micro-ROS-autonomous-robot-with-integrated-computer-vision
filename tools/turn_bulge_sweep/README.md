# turn_bulge_sweep

Dò tham số tạo hình đường nối khi rẽ (`TrajectoryPlanner::turn_lateral_bulge_mult`
và `turn_bezier_handle_scale_mult`) bằng **chính header planner thật** —
không có bản sao logic nào ở đây, nên số đo ra khớp với path mà `control_node`
sinh trên xe.

## Build

Cần header sinh `label_mapping.hpp`, tức là đã `colcon build` package
`avs_perception` ít nhất một lần.

```bash
cd /home/goln/SimpleSysIDV
g++ -std=c++17 -O2 -o /tmp/turn_bulge_sweep \
    tools/turn_bulge_sweep/turn_bulge_sweep.cpp \
    -Iros2_ws/src/avs_perception/include \
    -Iros2_ws/build_user/avs_perception/include
```

## Dùng

```bash
# Bảng quét mặc định (bulge 0.00 → 0.80, bước 0.05)
/tmp/turn_bulge_sweep

# Xem hình dạng path ở một giá trị cụ thể (ASCII BEV, +x phải / +y lên như dashboard)
/tmp/turn_bulge_sweep --plot 0.40

# Quét khoảng khác / đổi handle scale / rẽ trái
/tmp/turn_bulge_sweep --from 0.2 --to 0.6 --step 0.02
/tmp/turn_bulge_sweep --handle 1.2
/tmp/turn_bulge_sweep --intent left

# Dùng frame telemetry thật thay vì fixture dựng sẵn
/tmp/turn_bulge_sweep --telemetry /path/to/frame.json
```

File telemetry là JSON đúng schema `/avs/telemetry_realworld` mà
`PathObservationBuilder::build` ăn vào (`objects[]` với `label`, `waypoints`).
Fixture mặc định là frame rẽ phải capture từ xe thật — cùng geometry với hai
gtest guard trong `decision_trajectory_test.cpp`.

## Đọc bảng

| cột | ý nghĩa |
| --- | --- |
| `belly_mm` | độ sâu bụng cong về **phía ngoài** cua (phía tiếp tuyến đầu lệch tới) |
| `inside_mm` | phần lấn sang **góc trong** — muốn ~0, khác 0 nghĩa là bụng bị đảo phía |
| `outward_mm` | path vòng ra xa xe bao nhiêu về phía ngược hướng cua |
| `min_radius_mm` | bán kính cua nhỏ nhất trên path = đỉnh yêu cầu đánh lái (lớn = mượt hơn) |
| `length_mm` | tổng chiều dài path |
| `FOLDS` | path mất tiến độ tiến (y giảm) — **không bao giờ ship giá trị này** |

## Số đo 2026-08-04 (fixture rẽ phải, handle=1.5)

| bulge | belly_mm | outward_mm | min_radius_mm | ghi chú |
| --- | --- | --- | --- | --- |
| 0.00 | 148 | 21 | 236 | chỉ có handle scale, gần như bám chord |
| 0.20 | 244 | 63 | 214 | |
| 0.40 | 356 | 126 | 205 | **mặc định đang ship** |
| 0.60 | 481 | 210 | 171 | |
| 0.78 | ~543 | ~290 | ~157 | giá trị cũ — vòng ra ngoài rất rộng |
| 0.80 | 609 | 299 | 143 | **FOLDS** — vách đứng, không dùng được |

Vách `FOLDS` ở 0.80 là với `handle=1.5`; đổi `--handle` thì vị trí vách đổi
theo, phải quét lại. Handle scale một mình gập path ở khoảng 2.5–2.6x nên
`turn_lateral_bulge_mult` mới là knob nên chỉnh.

## Đổi giá trị

Chỉnh live trên xe (`control_node` đọc lại param mỗi frame telemetry, không
cần restart):

```bash
ros2 param set /control_node turn_lateral_bulge_mult 0.35
ros2 param set /control_node turn_bezier_handle_scale_mult 1.5
ros2 param get /control_node turn_lateral_bulge_mult
```

Chốt giá trị: sửa mặc định ở
`ros2_ws/src/avs_perception/include/avs_perception/trajectory_planner.hpp`
(hai static public đầu class `TrajectoryPlanner` — cũng là default của hai
ROS param), rồi `colcon build --packages-select avs_perception`.

Sau khi đổi `turn_bezier_handle_scale_mult`, vị trí vách `FOLDS` của bulge đổi
theo — quét lại bằng `--handle <giá trị mới>` trước khi chọn bulge.
