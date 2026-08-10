# 📖 Hướng Dẫn Vận Hành & Điều Khiển Hệ Thống AVS Robot

Tài liệu này tổng hợp toàn bộ các câu lệnh dùng để setup mạng, quản lý Docker, chạy các node nhận diện/điều khiển trên xe (Raspberry Pi 5) cũng như cách sử dụng RViz và các công cụ Live Plot trên máy trạm (Ubuntu).

---

## 1. Kết Nối Mạng (Raspberry Pi)
Đảm bảo xe Raspberry Pi và máy tính Ubuntu cùng kết nối chung một mạng Wi-Fi để có thể đồng bộ ROS 2 (Domain ID `20`).

```bash
# Xem danh sách các mạng wifi có sẵn
nmcli dev wifi list

# Kết nối vào wifi (Thay thế tên và mật khẩu cho phù hợp)
sudo nmcli dev wifi connect "Dat1502" password "15022004" ifname wlan0
# hoặc
sudo nmcli dev wifi connect "MNghia" password "12345678" ifname wlan0
```

---

## 2. Quản Lý Docker & Dashboard (Raspberry Pi)
Tất cả hệ thống chạy trong Docker. Đường dẫn gốc trên Pi là `/home/pi/SimpleSysIDV`.

**Khởi động các container:**
```bash
cd ~/SimpleSysIDV
# Chạy tất cả các dịch vụ (ẩn dưới nền)
docker compose -f docker-compose.prod.yml up -d

# Hoặc khởi động chỉ định cụ thể các container
docker compose -f docker-compose.prod.yml up -d avs_perception video_publisher web_dashboard avs_dashboard
```

**Xem Logs hệ thống:**
```bash
# Xem log nhận diện và điều khiển cốt lõi
sudo docker compose -f docker-compose.prod.yml logs -f avs_perception

# Xem log của giao diện Dashboard API
docker compose -f docker-compose.prod.yml logs -f avs_dashboard
```

**Khởi động lại Dashboard khi cập nhật code:**
```bash
cd ~/SimpleSysIDV
docker compose -f docker-compose.prod.yml restart avs_dashboard
```

---

## 3. Môi Trường ROS 2 Cơ Bản (Trên Raspberry Pi)

Để tương tác với ROS 2, bạn cần `exec` vào trong container `avs_perception_container`.

**Mở bash shell trong container:**
```bash
docker exec -it avs_perception_container bash

# Load môi trường ROS 2
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=20
```

**Các lệnh kiểm tra cơ bản (Chạy trực tiếp từ bên ngoài Pi):**

*Liệt kê Topic và Node:*
```bash
docker exec -it avs_perception_container bash -lc "
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 node list && ros2 topic list -t
"
```

*Đọc tần số (Hz) của các Topic quan trọng:*
```bash
docker exec -it avs_perception_container bash -lc "
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20 &&
for t in /camera/image_raw /avs/telemetry /avs/telemetry_realworld /avs/lane_state /avs/control_error /odom_raw
do
  echo ''
  echo ===== \$t =====
  timeout 6 ros2 topic hz \$t || true
done
"
```

*Đọc data thô của một Topic:*
```bash
# Xem Lane Target
docker exec -it avs_perception_container bash -lc "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20 && ros2 topic echo /lane_target"

# Xem Vận tốc điều khiển
docker exec -it avs_perception_container bash -lc "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20 && ros2 topic echo /cmd_vel"
```

---

## 4. Các Lệnh Điều Khiển & Nhận Diện Làn (Raspberry Pi)

Dưới đây là các lệnh chạy từng Node nhận diện và bộ điều khiển (Controllers). Lệnh được gói trong `docker exec` để có thể paste chạy thẳng trên Terminal của Pi.

### Khẩn Cấp (Phanh Xe Bằng Lệnh Dòng Lệnh)
```bash
docker exec -it avs_perception_container bash -lc "
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20 &&
timeout 5 ros2 topic pub -r 30 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
"
```

### Chạy xe thẳng với Vận Tốc Tự Do (Thử động cơ)
```bash
docker exec -it avs_perception_container bash -lc "
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
"
```

### Nhận Diện Làn (Lane Parser Node)
Xử lý ảnh ra đường dẫn mục tiêu `/lane_target`.
```bash
docker exec -it avs_perception_container bash -lc "
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 run avs_controlsystem lane_parser_node --ros-args \
  -p target_class:=main-lane \
  -p image_width:=640.0 -p image_height:=480.0 \
  -p lookahead_ratio:=0.70 -p near_ratio:=0.85 \
  -p min_points:=8
"
```

### Điều Khiển Nhận Diện Lỗi & Tính Toán Vận Tốc (cmdvel_from_control_error)
Tính toán `/cmd_vel` dựa trên `/avs/control_error`.
```bash
docker exec -it avs_perception_container bash -lc "
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 run avs_perception cmdvel_from_control_error_node --ros-args \
  -p control_error_topic:=/avs/control_error \
  -p lane_state_topic:=/avs/lane_state \
  -p cmd_vel_topic:=/cmd_vel \
  -p debug_topic:=/avs/controller_debug \
  -p enable_motion:=true \
  -p error_timeout:=3.0 -p stop_on_timeout:=true \
  -p v_max:=0.20 -p v_min:=0.055 -p v_turn_max:=0.080 -p omega_max:=0.36 \
  -p kp_x:=0.32 -p kp_theta:=0.26 -p kd_x:=0.018 -p kd_theta:=0.022 \
  -p error_filter_alpha:=0.045 -p v_rate_limit:=0.12 -p omega_rate_limit:=0.18 \
  -p slow_error_gain:=4.2 -p theta_weight:=0.75 -p wheel_mix_factor:=0.18 \
  -p inner_min_fraction:=0.48 -p epsilon_x_deadband_m:=0.018 -p theta_deadband_rad:=0.020 \
  -p max_abs_epsilon_x_m:=0.75 -p max_abs_theta_rad:=1.20 -p invert_angular:=false
"
```

### Điều Khiển Bám Làn Cơ Bản (LiDAR Follower)
```bash
docker exec -it avs_perception_container bash -lc "
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 run avs_controlsystem lane_lidar_follower_node --ros-args \
  -p lane_target_topic:=/lane_target \
  -p scan_topic:=/scan \
  -p cmd_vel_topic:=/cmd_vel \
  -p v_max:=0.1 -p v_min:=0.05 -p omega_max:=0.30 \
  -p kp_y:=0.6 -p kp_heading:=0.2 -p kd_heading:=0.4 \
  -p lane_lost_timeout:=2.0 -p search_omega:=0.08 \
  -p emergency_distance:=0.18 -p stop_distance:=0.32 -p slow_distance:=0.70 \
  -p invert_angular:=false
"
```

### Điều Khiển Pure Pursuit kết hợp PD (Nâng Cao)
```bash
docker exec -it avs_perception_container bash -lc "
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 run avs_controlsystem pur_persuit_pd_mainlane_following --ros-args \
  -p control_error_topic:=/avs/control_error \
  -p cmd_vel_topic:=/cmd_vel \
  -p debug_topic:=/avs/pur_persuit_pd_mainlane_debug \
  -p enable_motion:=true \
  -p error_timeout_s:=1.5 \
  -p v_max:=0.10 -p v_min:=0.05 -p v_turn_min:=0.05 \
  -p k_c:=0.02 -p k_pp:=1.20 -p k_theta:=0.2 -p kd_lateral:=0.4 -p kd_theta:=0.02 \
  -p omega_max:=0.45 
"
```

### Bộ Điều Khiển Vượt Cản / Góc Gắt (Start Turn PP+PD có Log)
```bash
docker exec -it avs_perception_container bash -lc "
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 run avs_controlsystem pur_persuit_pd_mainlane_following_logger --ros-args \
  -p control_error_topic:=/avs/control_error \
  -p cmd_vel_topic:=/cmd_vel \
  -p debug_topic:=/avs/pur_persuit_pd_mainlane_debug \
  -p odom_topic:=/odom \
  -p use_odom:=true -p enable_logging:=true \
  -p run_data_dir:=/workspace/ros2_ws/src/avs_controlsystem/run_data \
  -p save_image_period_s:=2.0 -p csv_flush_period_s:=1.0 \
  -p enable_motion:=true \
  -p error_timeout_s:=1.5 \
  -p v_max:=0.10 -p v_min:=0.5 -p v_turn_min:=0.03 \
  -p k_c:=0.20 -p k_pp:=0.60 -p k_theta:=0.2 -p kd_lateral:=0.4 -p kd_theta:=0.2 \
  -p omega_max:=0.3 \
  -p wheel_separation_m:=0.135 -p max_delta_v:=0.085 \
  -p delta_v_rate_limit:=0.16 -p inner_wheel_min_fraction:=0.45 \
  -p filter_alpha:=0.18 -p derivative_alpha:=0.25 -p v_rate_limit:=0.12 \
  -p ld_min_m:=0.14 -p ld_max_m:=0.85 -p invert_angular:=false
"
```

### Hệ Điều Khiển Nâng Cao (Launch Files)
**1. Cascade Controller V1:**
```bash
docker exec -it avs_perception_container bash -lc '
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 launch avs_cascadecontrol cascade_controller_v1.launch.py \
  enable_cmd:=true cmd_vel_topic:=/cmd_vel invert_angular:=false allow_cmd_vel_conflict:=false
'
```
*(Nếu cần, có script tắt: `docker exec -it avs_perception_container bash -lc '/workspace/ros2_ws/start_cascade_lane.sh'`)*

**2. PD Backstepping Controller:**
```bash
docker exec -it avs_perception_container bash -lc '
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=20 &&
ros2 launch avs_pdbackstepingcontrol pd_backsteping_control.launch.py \
  enable_cmd:=true cmd_vel_topic:=/cmd_vel invert_angular:=false allow_cmd_vel_conflict:=false
'
```

**3. Mở công cụ gỡ lỗi (RQT Graph):**
*(Yêu cầu đã config X11 Forwarding / DISPLAY)*
```bash
docker exec -it avs_perception_container bash -lc "
source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=20 && export ROS_LOCALHOST_ONLY=0 &&
ros2 run rqt_graph rqt_graph
"
```

---

## 5. Trực Quan Hoá & Vẽ Đồ Thị Live (Trên UBUNTU - PC Trạm)
Lưu ý: Mọi cửa sổ Terminal trên Ubuntu cần cấu hình Môi trường ROS 2 và chung `ROS_DOMAIN_ID=20`.

```bash
# Lệnh phải chạy trước tiên trên mỗi cửa sổ terminal ở Ubuntu:
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
```

### 5.1 Đọc Odometry và Hiện Rviz:
```bash
# Chạy script chuyển đổi odom -> Transform tf
python3 ~/odom_raw_to_tf.py

# Các alias tiện ích bật rviz (nếu có cấu hình trong .bashrc)
start_rviz_car
reset_rviz_car
stop_rviz_car
```

### 5.2 Vẽ biểu đồ Live Topic Cơ Bản
```bash
# Vẽ biểu đồ sử dụng script mặc định
python3 ~/ros_topic_live_plot.py

# Vẽ biểu đồ sử dụng alias
plot_robot

# Hoặc dùng công cụ vẽ Topics nâng cao với tham số tuỳ biến
python3 ~/plot_robot_topics_live.py \
  --window 60 \
  --plot-hz 5 \
  --track-width 0.135 \
  --csv ~/avs_robot_live_log.csv
```

### 5.3 Vẽ biểu đồ chi tiết Cascade Control
```bash
# Mở cửa sổ Live Plot giao diện (Cascade)
python3 ~/cascade_plot_logger.py --window-s 120 --log-hz 20 --plot-hz 5

# Nếu muốn record dữ liệu ngầm và KHÔNG mở UI Matplotlib
python3 ~/cascade_plot_logger.py --no-live --duration-s 60 --log-hz 20 --plot-hz 2 --raw-jsonl
```

### 5.4 Vẽ biểu đồ chi tiết Pure Pursuit & PD
```bash
# Live Plot chung
python3 ~/pp_pd_plot_logger.py --window-s 120 --log-hz 20 --plot-hz 5
  
# Chỉ tập trung vẽ PD Controller (lấy dữ liệu từ /avs/main_following_pd_debug)
python3 ~/pp_pd_plot_logger.py --preferred-controller pd

# Chỉ tập trung vẽ Pure Pursuit (lấy dữ liệu từ /avs/pur_persuit_mainlane_debug)
python3 ~/pp_pd_plot_logger.py --preferred-controller pp

# Chỉ tập trung vẽ Pure Pursuit + PD (lấy dữ liệu từ /avs/pur_persuit_pd_mainlane_debug)
python3 ~/pp_pd_plot_logger.py --preferred-controller pppd
```

---

## 6. Chuyển Mã Nguồn & Triển Khai Dữ Liệu (Từ Ubuntu -> Raspberry Pi)
Đảm bảo bạn đứng ở thư mục gốc trên Ubuntu chứa Code: `cd /home/bluedstar/AVS_Robot_Control_Center`

**Đồng bộ các ROS 2 package:**
```bash
scp -r ./SimpleRobot/ros2_ws/src/avs_cascadecontrol pi@raspi5.local:/home/pi/SimpleSysIDV/ros2_ws/src/
scp -r ./SimpleRobot/ros2_ws/src/avs_controlsystem pi@raspi5.local:/home/pi/SimpleSysIDV/ros2_ws/src/
```

**Đồng bộ Dashboard Backend (bỏ qua Git & Node modules):**
```bash
rsync -avz --exclude 'web/node_modules' --exclude '.git' \
  ./SimpleRobot/avs_dashboard_system/ \
  pi@raspi5.local:/home/pi/SimpleSysIDV/avs_dashboard_system/
```

**Đồng bộ Dashboard Frontend (Web tĩnh Build ra):**
```bash
scp -r ./SimpleRobot/avs_dashboard_system/web/dist/* \
  pi@raspi5.local:/home/pi/SimpleSysIDV/web_dashboard/frontend/avs_advanced/
```

---

## 7. Quản Trị Version Control (Git Push trên Raspberry Pi)
Nếu code được chỉnh sửa trực tiếp trên Raspberry Pi và cần đẩy lên Github nhánh chính:

```bash
cd /home/pi/SimpleSysIDV

# Gắn link Repo
git remote set-url origin https://github.com/blueDstar/AVS-System.git 2>/dev/null || \
git remote add origin https://github.com/blueDstar/AVS-System.git

# Kiểm tra trạng thái Git
git status
git branch -vv

# Thêm/Commit code và Push
git add -A
git commit -m "Update AVS system code"

# Kéo update về trước để tránh rẽ nhánh
git pull origin main --rebase

# Push code lên
git push origin main
# (Sử dụng ép push --force-with-lease nếu phát sinh rẽ nhánh nhưng cực kỳ cẩn thận)
# git push --force-with-lease origin main
```

> **Lưu ý File `.gitignore`:** Đảm bảo thư mục Pi có chứa file `.gitignore` để tránh upload cache Docker, build logs... Lệnh tạo nhanh:
> ```bash
> cat > .gitignore << 'EOF'
> ros2_ws/build/
> ros2_ws/install/
> ros2_ws/log/
> build/
> install/
> log/
> __pycache__/
> *.pyc
> *.so
> *.mp4
> *.avi
> *.bag
> *.db3
> EOF
> ```
