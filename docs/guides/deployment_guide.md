# Hướng dẫn triển khai hệ thống lên Raspberry Pi 5 (Production Target)

Tài liệu này hướng dẫn từng bước đưa pipeline nhận diện AVS (`ncnn_inference_node` -> `ipm_transform_node` -> `control_node`) từ Laptop phát triển lên **Raspberry Pi 5**, chạy bằng Docker thực tế và xem kết quả nhận diện qua **web dashboard local**.

Phạm vi hiện tại: pipeline dừng ở `/avs/control_error` + `/avs/lane_state` (xem CLAUDE.md). Repo **không còn** node điều khiển (`pure_pursuit_node`/`cmdvel_from_control_error_node`) — các node đó đã bị gỡ khỏi workspace. Guide này chỉ triển khai phần vision/decision + dashboard giám sát.

**Ngoại lệ — micro-ROS agent:** `docker-compose.prod.yml` có service `micro_ros_agent` (image `microros/micro-ros-agent:humble`) làm cầu Serial `/dev/ttyUSB0` ↔ ROS2 cho ESP32. Đây **không** phải node trong workspace, chỉ là container hạ tầng. Khởi động bằng `scripts/start_agent_rpi5.sh` — xem §5.1.

---

## 1. Chuẩn bị tài nguyên trên Laptop

### 1.1 Build image ARM64 (nếu chưa có hoặc code vừa đổi)

Image production build **không Vulkan**, NCNN CPU (Pha H — xem `docs/optimize/ncnn_inference_latency_plan.md` §6.8):

```bash
cd /home/goln/SimpleSysIDV
sudo docker buildx build \
  --platform linux/arm64 \
  -t avs_perception:arm64 \
  -f docker/Dockerfile \
  -o type=docker,dest=avs_perception_arm64.tar .
```

Build qua QEMU mất khoảng 10–15 phút. Không dùng `avs_perception_arm64_vulkan.tar` cho production — image đó chỉ để benchmark Pha D/H, Vulkan bị loại khỏi production path.

Nếu chỉ sửa code C++/Python trong `ros2_ws/` hoặc `web_dashboard/` mà **không** đổi `docker/Dockerfile`, không cần build lại image: các thư mục này được bind-mount và biên dịch lại ngay trên Pi mỗi lần container khởi động.

### 1.2 Checklist trước khi sync

Đảm bảo thư mục dự án có đủ:

1. **Docker image ARM64:** `avs_perception_arm64.tar` (thư mục gốc repo).
2. **Mã nguồn ROS2:** `ros2_ws/`.
3. **Model:** `models/best_ncnn_model/` (FP32, dùng làm FP16 CPU runtime — default hiện tại) và `models/best_ncnn_model_int8/` (rollback, xem §4.3).
4. **Cấu hình:** `docker-compose.prod.yml`, `config/config.json`.
5. **Frontend/backend dashboard:** `web_dashboard/`.

---

## 2. Sao chép dự án sang Raspberry Pi 5

Bật Raspberry Pi 5, kết nối cùng mạng Wi-Fi/LAN với Laptop.

### Bước 2.1: Lấy địa chỉ IP hoặc hostname của Pi 5

```bash
hostname -I
```

Có thể dùng IP trực tiếp (`192.168.1.100`) hoặc mDNS hostname (`raspi5.local`, `goln-raspi5.local` tuỳ máy) nếu Avahi hoạt động.

### Bước 2.2: Sao chép dự án bằng `rsync`

Loại bỏ build artifacts, venv, `.git`, và các thư mục không cần trên Pi:

```bash
rsync -avz --no-owner --no-group \
  --exclude="ros2_ws/build" \
  --exclude="ros2_ws/install" \
  --exclude="ros2_ws/log" \
  --exclude="ros2_ws/build_user" \
  --exclude="ros2_ws/install_user" \
  --exclude="ros2_ws/log_user" \
  --exclude="ncnn-src" \
  --exclude="build" \
  --exclude="install" \
  --exclude="log" \
  --exclude="docs" \
  --exclude="skills" \
  --exclude=".venv" \
  --exclude=".git" \
  --exclude="config/calibration.json" \
  --exclude=".agents" \
  --exclude=".claude" \
  --exclude=".codegraph" \
  --exclude=".codex" \
  --exclude=".tokensave" \
  --exclude="terminal-run" \
  --exclude="avs_perception_arm64_vulkan.tar" \
  /home/goln/SimpleSysIDV/ pi@raspi5.local:~/SimpleSysIDV/
```

`test/` **giữ lại** nếu bạn cần chạy `mode: "video"` (xem §4.1) làm smoke test trước khi chuyển sang camera thật; có thể loại bỏ (`--exclude="test"`) khi chỉ chạy camera live.

---

## 3. Cài đặt và thiết lập trên Raspberry Pi 5

```bash
ssh pi@raspi5.local
cd ~/SimpleSysIDV
```

### Bước 3.1: Nạp Docker image ARM64 vào Pi 5

```bash
sudo docker load -i avs_perception_arm64.tar
```

Kiểm tra: `sudo docker images` phải thấy `avs_perception:arm64`.

### Bước 3.2: Thiết lập quyền truy cập USB camera

```bash
sudo usermod -aG video $USER
```

Cần logout/login lại (hoặc `newgrp video`) để áp dụng group mới.

### Bước 3.3 (khuyến nghị): udev symlink ổn định cho camera

USB camera thường tạo nhiều `/dev/video*` (capture + metadata device). Nên gán symlink cố định thay vì phụ thuộc `/dev/video0` (index có thể đổi khi cắm lại):

```bash
sudo nano /etc/udev/rules.d/99-usb-camera.rules
```

```text
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", ATTR{index}=="0", SYMLINK+="video_source"
```

`ATTR{index}=="0"` bắt buộc — index 0 là capture device, index khác là metadata, không mở được để capture.

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/video_source   # phải trỏ -> /dev/video0 (không phải video1)
```

Chi tiết đầy đủ (vendor/product ID theo từng xe): `skills/camera_SKILL/SKILL.md`.

---

## 4. Cấu hình chế độ chạy

### Bước 4.1: Chọn nguồn input — `config/config.json`

```json
{
  "mode": "camera",
  "camera_device": "/dev/video_source",
  "video_path": "/workspace/test/test_video/video_test1.mp4",
  "prob_threshold": 0.25,
  "nms_threshold": 0.45,
  "loop": true,
  "fps_override": 0.0,
  "camera_width": 640,
  "camera_height": 480,
  "camera_fps": 30
}
```

- `mode: "camera"` — chạy camera USB thực tế (mục tiêu của guide này).
- `mode: "video"` — phát video test (`test/test_video/`), dùng để smoke test trước khi có camera gắn sẵn.
- `camera_device`: dùng symlink `/dev/video_source` (Bước 3.3) nếu đã cấu hình udev; nếu chưa, dùng `/dev/video0` trực tiếp nhưng chấp nhận rủi ro đổi index.

### Bước 4.2: Runtime inference mặc định — FP16 CPU

`docker-compose.prod.yml` đã chốt default production (Pha C–H, xem `docs/optimize/ncnn_inference_latency_plan.md` §7): model FP32 (`models/best_ncnn_model/`) chạy dưới `use_fp16_packed/storage/arithmetic=true`, `use_int8_inference=false`, `use_vulkan_compute=false`. Không cần sửa gì để dùng candidate này — đây đã là default khi `up`.

### Bước 4.3: Rollback INT8 (chỉ khi FP16 CPU có vấn đề)

Nếu cần rollback, sửa lệnh `ncnn_inference_node` trong `docker-compose.prod.yml` (dòng `command` của service `avs_perception`), thêm override:

```text
-p model_param_path:=/workspace/models/best_ncnn_model_int8/model.ncnn.param \
-p model_bin_path:=/workspace/models/best_ncnn_model_int8/model.ncnn.bin \
-p use_int8_inference:=true \
-p use_fp16_packed:=false -p use_fp16_storage:=false -p use_fp16_arithmetic:=false
```

Không xoá cấu hình FP16 gốc, chỉ tạm ghi đè khi cần so sánh/debug.

---

## 5. Chạy hệ thống trên Raspberry Pi 5

```bash
cd ~/SimpleSysIDV
sudo docker compose -f docker-compose.prod.yml up -d
```

Bốn container khởi động:

| Container | Vai trò |
|---|---|
| `avs_perception_container` | Clean build cũ, `colcon build --symlink-install` (Release) trên ARM64, sau đó chạy đồng thời `ncnn_inference_node` (FP16 CPU), `ipm_transform_node`, `control_node`. |
| `video_publisher_container` | Đợi build xong, đọc camera USB (hoặc video test theo `config.json`) và publish `/camera/image_raw`. |
| `web_dashboard_container` | Chạy FastAPI + ROS2 bridge, phục vụ dashboard tại cổng `8000`. |
| `micro_ros_agent` | Cầu Serial `/dev/ttyUSB0` @921600 ↔ ROS2 cho ESP32 (`restart: unless-stopped`). Xem §5.1. |

Build C++ diễn ra ngay trong container mỗi lần start (không cần build sẵn trên Laptop) — chờ vài phút cho lần đầu.

Theo dõi log để xác nhận đúng runtime FP16 CPU đang chạy:

```bash
sudo docker compose -f docker-compose.prod.yml logs -f avs_perception
```

Tìm dòng log dạng:

```text
Applied NCNN Options: vulkan=0, fp16_p=1, fp16_s=1, fp16_a=1, pack=1, int8=0, target_size=320, ...
```

`vulkan=0, int8=0, fp16_*=1` xác nhận đang chạy đúng candidate FP16 CPU.

### 5.1 micro-ROS agent (ESP32) — chỉ được chạy MỘT instance

`/dev/ttyUSB0` là tài nguyên độc quyền. Nếu có nhiều tiến trình cùng `open()` nó, mỗi agent sẽ tự reset DTR/RTS và flush cổng, khiến ESP32 mất session XRCE-DDS liên tục và **hủy đăng ký subscriber `/cmd_vel`**.

Vì vậy agent được khai báo thành service Compose với `container_name: micro_ros_agent` cố định — Compose tự đảm bảo singleton. **Không** dùng `docker run` rời (sinh container tên ngẫu nhiên, mỗi lần chạy thêm một cái).

```bash
cd ~/SimpleSysIDV
./scripts/start_agent_rpi5.sh            # Agent treo / ESP32 mất kết nối
./scripts/start_agent_rpi5.sh --follow   
./scripts/start_agent_rpi5.sh --status   # Xem có bao nhiêu agent, ai giữ cổng
./scripts/start_agent_rpi5.sh --stop     # Tắt hẳn agent (vượt restart policy)
```

Script dọn **mọi** container sinh từ image `microros/micro-ros-agent` (kể cả tên ngẫu nhiên), chờ cổng được nhả, rồi mới `compose up -d micro_ros_agent`. Nếu phát hiện tiến trình **khác** đang giữ cổng, script in tên rồi dừng thay vì kill — xem cảnh báo dưới.

Kiểm tra ai đang giữ cổng:

```bash
sudo fuser -v /dev/ttyUSB0        # kỳ vọng: đúng 1 PID, COMMAND = micro_ros_agent
docker ps -a | grep micro-ros-agent
```

### 5.2 Container Yahboom (joy teleop) — xung đột `/cmd_vel`

`~/ros2_humble.sh` khởi động `yahboomtechnology/ros-humble:4.1.2` chạy `/root/1.sh`. Chuỗi thực thi đã kiểm chứng:

```text
/root/1.sh -> systemctl start supervisor
           -> [program:ChassisServer] -> /root/run_handle.sh
           -> ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py
           -> chỉ 2 node: yahboom_joy + joy_node
```

Nó **không** mở `/dev/ttyUSB0` (không có chassis driver, `yahboomcar_base_node` không tham chiếu `/dev/tty*`). Rủi ro thật nằm ở chỗ khác: image có sẵn `ROS_DOMAIN_ID=20` **trùng domain AVS**, chạy `--net=host`, mount `/dev/input` (tay cầm DragonRise) → nó **publish `/cmd_vel` cạnh tranh** với `control_node` trên đúng topic ESP32 subscribe.

Dùng bản đã làm cứng:

```bash
./scripts/start_ros2_humble_rpi5.sh            # dọn instance cũ + chạy
./scripts/start_ros2_humble_rpi5.sh --status   # xem trạng thái
./scripts/start_ros2_humble_rpi5.sh --stop     # dừng
./scripts/start_ros2_humble_rpi5.sh --prune    # xoá container Exited tồn đọng
```

Khác biệt so với `~/ros2_humble.sh` cũ:

| Vấn đề bản cũ | Bản mới |
|---|---|
| Không `--name`/`--rm` → tích tụ container tên ngẫu nhiên | `--name yahboom_ros` + `--rm`, dọn instance đang chạy trước khi start |
| `-v /tem/.X11-unix:/tmp/.X11-unix` (**typo**) → Docker tạo thư mục rỗng `/tem/.X11-unix` trên host, mount đè → X11 hỏng im lặng | `-v /tmp/.X11-unix:/tmp/.X11-unix`, và chỉ mount khi `DISPLAY` có set |
| `xhost +` mở access control cho mọi client | `xhost +local:root` |
| Mount `/dev/video0` kể cả khi không tồn tại | Chỉ mount thiết bị thực sự có |
| Im lặng khi stack tự hành đang chạy | Cảnh báo rõ xung đột `/cmd_vel` (cảnh báo, **không** chặn — teleop có thể đang là override thủ công/E-stop) |

---

## 6. Giám sát hệ thống qua web dashboard local

Từ Laptop hoặc điện thoại cùng mạng với Pi, mở trình duyệt:

```
http://<IP-hoặc-hostname-Pi>:8000
```

Ví dụ `http://192.168.1.100:8000` hoặc `http://raspi5.local:8000`.

Dashboard hiển thị (subscribe `/camera/image_raw/compressed`, `/avs/telemetry_realworld`, `/avs/lane_state`, `/avs/control_error`):

1. Luồng video nhận diện lane/marking/object thời gian thực.
2. `lane_state` — trạng thái làn hiện tại.
3. `control_error` — sai số lane/heading tính từ `control_node` (đầu ra cuối của pipeline hiện tại; chưa có node điều khiển động cơ trong repo).

---

## 7. Vận hành & Troubleshooting nhanh

```bash
sudo docker compose -f docker-compose.prod.yml ps                          # trạng thái container
sudo docker compose -f docker-compose.prod.yml logs -f                     # log tất cả
sudo docker compose -f docker-compose.prod.yml logs -f video_publisher     # log riêng camera
sudo docker compose -f docker-compose.prod.yml down                        # dừng hệ thống
```

Sau khi sửa code (`ros2_ws/`, `web_dashboard/`) trên Laptop: rsync lại lên Pi rồi `down` -> `up -d` (không cần rebuild image, C++ compile lại trong container).

Sự cố thường gặp — xem chi tiết đầy đủ tại `skills/docker_SKILL/SKILL.md` §7:

| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| Dashboard không có video khi `mode: "camera"` | udev symlink sai device index, hoặc `/dev` chưa mount đúng vào `video_publisher` |
| `FileNotFoundError: install/setup.bash` | Race condition build — bình thường sẽ tự retry, chờ `avs_perception_container` build xong |
| Container `avs_perception` liên tục restart build | Xung đột build artifact x86_64/ARM64 do bind-mount `ros2_ws/` — container tự `rm -rf build install log` mỗi lần start nên thường tự khỏi |

---

## 8. Tham chiếu

- Kiến trúc Docker đầy đủ, camera vendor/product ID theo từng xe, cấu hình V4L2: `skills/docker_SKILL/SKILL.md`, `skills/camera_SKILL/SKILL.md`.
- Quyết định runtime FP16 CPU / rollback INT8 / lý do loại Vulkan: `docs/optimize/ncnn_inference_latency_plan.md` §6–§7.
- Label mapping và các bẫy segmentation: `CLAUDE.md` mục "Label Mapping".


## 9. Cập nhật và tunning tham số 
### 9.1 Cập nhật tham số  turn_lateral_bulge_mult 
- path: /home/goln/SimpleSysIDV/ros2_ws/src/avs_perception/include/avs_perception/trajectory_planner.hpp
- update on pi command:docker exec -it avs_perception_container bash -c "source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash && ros2 param set /control_node turn_lateral_bulge_mult 0.6"