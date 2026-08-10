# Quan sát cú rẽ trên xe thật

Bộ đôi script để đo cú rẽ trên sa bàn, thay cho `turn_monitor2.py` (ad-hoc,
sống trong scratchpad phiên 2026-08-05 và đã mất).

- `turn_observe.py` — chạy TRONG container `avs_perception` trên Pi, ghi jsonl
  và in sự kiện latch ngay lúc chạy.
- `turn_analyze.py` — chạy trên laptop, tách từng cú rẽ và in phân bố số đo.

## Vì sao giữ nguyên hình học

`config/record_run.py` (bản cũ trên Pi) vứt `active_trajectory_points` và
`debug_trajectories` cho nhẹ file. Nhưng path commit của frame **ngay trước lúc
latch đóng** chính là `observed` — đầu vào thật của `extend_to_turn_angle`.
Không có nó thì một cú rẽ hỏng không chạy lại offline được, và
`extend_to_turn_angle` có 7 nhánh `return pts` nên đoán nhánh nào là vô ích
(xem `skills/decision_trajectory_SKILL/SKILL.md` mục 3b-bis).

`turn_observe.py` giữ `active_trajectory_points` cùng
`debug_trajectories[stage=candidate|committed]`. Bỏ `normalized` vì suy ra được
và là stage nặng nhất. ~14FPS × cỡ vài KB/frame — một lượt 2 phút cỡ vài MB.

## Quy trình một lượt chạy

```bash
# 1. Chép script lên Pi (chỉ khi sửa)
scp tools/turn_observation/turn_observe.py pi@raspi5.local:~/SimpleSysIDV/config/

# 2. Ghi lại tham số đang chạy — không có nó thì lượt đo không truy lại được
ssh pi@raspi5.local 'docker exec avs_perception_container bash -c \
  "source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=20 \
   ros2 param dump /control_node"' > runNN_params.yaml

# 3. Bắt đầu ghi (chạy nền), rồi cho xe chạy
ssh pi@raspi5.local 'docker exec avs_perception_container bash -c \
  "source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=20 \
   python3 /workspace/config/turn_observe.py /workspace/config/runNN.jsonl 150"'

# 4. Ra lệnh intent (từ Pi hoặc laptop)
ssh pi@raspi5.local 'curl -X POST "http://localhost:8000/api/route_intent?intent=turn_left"'

# 5. Kéo về và phân tích
scp pi@raspi5.local:~/SimpleSysIDV/config/runNN.jsonl .
python3 tools/turn_observation/turn_analyze.py runNN.jsonl
python3 tools/turn_observation/turn_analyze.py runNN.jsonl --episodes   # trace từng frame
```

## Đọc kết quả

`turn_analyze.py` in bốn mục A–D ứng với bốn thay đổi cần kiểm chứng, kèm số đo
"trước fix" để so sánh trực tiếp:

| Mục | Đo cái gì | Trước fix |
|-----|-----------|-----------|
| A | `path_front_y` lúc tiếp cận — neo turn vào xe hay vào lane bên kia giao lộ | cụm 500-600mm chiếm 13-14% frame |
| B | `turned_deg` và % path tiêu thụ lúc nhả latch | nhả ở 59.5-62.9° sau khi chỉ đi 15-43% path |
| C | Số lần skip-to-runout, góc cuối cùng | một cú đi 2175/2191mm, ra ở 118° |
| D | Góc latch tự báo vs góc xoay THẬT đo từ `/odom_raw` | 80° thật đọc thành 59-70 (phải) / 113-125 (trái) |

Mỗi mục in **phân bố** (min/p25/median/p75/max), không in trung bình. Lý do nằm
trong memory `turn-latch-open-loop-drift`: các ngưỡng sai trước đây đều do nhìn
một con số tóm tắt thay vì nhìn hai cụm tách nhau.

`[D]` là mục quan trọng nhất vì nó là **đối chứng độc lập**: `turn_observe.py`
tự tích phân yaw từ quaternion `/odom_raw` chứ không đọc lại con số của
`control_node`. Hai cột lệch nhau = datum của gate sai, không phải xe rẽ sai.

Nếu `frame có yaw đo` < 50% thì `/odom_raw` không tới (ESP32 cần reset sau khi
micro-ROS agent khởi động lại — xem memory `esp32-needs-reset-after-agent-restart`),
và toàn bộ mục D vô nghĩa.
