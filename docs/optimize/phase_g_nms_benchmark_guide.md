# Hướng Dẫn Benchmark Pha G: Có Nên Dùng NMS Trên Raspberry Pi 5

Guide này nối tiếp `ncnn_inference_latency_plan.md` §6.7. Mục tiêu: chạy
benchmark **G0 (NMS on)** và **G1 (NMS off + top-30)** bằng `video_test_node`
trên Raspberry Pi 5 thật, dùng cùng video test, cùng model, cùng runtime FP16
CPU. Kết quả dùng để quyết định production default của `enable_nms`.

Phase này **không dùng camera live**. Camera live dài hạn chỉ chạy sau khi đã
chốt NMS, để kiểm tra stability/throttle/jitter.

## 1. Kết Quả Cần Chốt

Sau guide này cần trả lời được:

- `enable_nms` production default là `true` hay `false`.
- Nếu `enable_nms=false`, `max_detections` default có giữ `30` hay cần chỉnh.
- Tắt NMS có giảm p95 `Detection Latency` thật không.
- Phần tiết kiệm ở `NMS/select` có bị mất lại ở `Mask` không.
- Output video NMS off có duplicate lane/mask gây nhiễu quan sát không.

## 2. Điều Kiện Tiên Quyết

- SSH vào được Test Pi: `goln-raspi5@goln-raspi5.local`.
- Code đã có runtime parameters:
  - `enable_nms`
  - `max_detections`
- Video test tồn tại ở repo laptop:
  - `test/test_video/video_test1.mp4`
- Model mặc định tồn tại:
  - `models/best_ncnn_model/model.ncnn.param`
  - `models/best_ncnn_model/model.ncnn.bin`
- Không có production containers đang chạy trên Pi trong lúc benchmark:

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV 2>/dev/null || true
sudo docker compose -f docker-compose.prod.yml down 2>/dev/null || true
```

Đường build mặc định của guide này là **Docker** vì khớp production image
`avs_perception:arm64` và tránh phụ thuộc ROS2/colcon cài native trên host OS.
Đường native chỉ là dự phòng.

## 3. Bẫy Quan Trọng Khi Sync Sang Pi

Không sync toàn bộ repo sang Pi. Các thư mục như `.venv`, `node_modules`,
`.codegraph`, build artifacts và đặc biệt `ncnn/` ở repo laptop không nên đi
sang Pi.

`ncnn/` trong repo laptop có thể là build x86_64. Nếu rsync sang Pi và colcon
link nhầm thư viện này, build ARM64 sẽ lỗi hoặc runtime crash. Vì vậy dùng
allowlist giống `pi_benchmark_guide.md`.

## 4. Bước 1 — Đồng Bộ Code Sang Pi

Chạy trên laptop:

```bash
rsync -avz \
  --include='/ros2_ws/' --include='/ros2_ws/src/' --include='/ros2_ws/src/***' \
  --include='/models/' \
  --include='/models/best_ncnn_model/' --include='/models/best_ncnn_model/***' \
  --include='/test/' --include='/test/test_video/' --include='/test/test_video/***' \
  --include='/tools/' --include='/tools/optimize/' --include='/tools/optimize/***' \
  --include='/config/' --include='/config/***' \
  --include='/docs/' --include='/docs/optimize/' --include='/docs/optimize/results/' \
  --include='/docs/optimize/phase_g_nms_benchmark_guide.md' \
  --exclude='*' \
  /home/goln/SimpleSysIDV/ goln-raspi5@goln-raspi5.local:/home/goln-raspi5/SimpleSysIDV/
```

Lệnh trên sync đúng phần cần cho benchmark:

- source ROS2 package
- model NCNN
- video test
- config sinh `label_mapping.hpp`
- tools power/benchmark
- thư mục results để lưu log/report

Nếu muốn kiểm tra trước khi sync thật, thêm `-n` vào `rsync`.

## 5. Bước 2 — Chọn Đường Build

### Đường A — Docker, Khuyến Nghị

Kiểm tra image production trên Pi:

```bash
ssh goln-raspi5@goln-raspi5.local
docker images | grep avs_perception
docker run --rm avs_perception:arm64 which bash
```

Nếu image chưa có, build/transfer image theo guide Docker nội bộ trước rồi quay
lại bước này.

Build package trong container:

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV
mkdir -p docs/optimize/results test/test_video_output

docker run --rm \
  --network host \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/config:/workspace/config" \
  avs_perception:arm64 \
  bash -c "command -v colcon >/dev/null || (apt-get update && apt-get install -y python3-colcon-common-extensions) && \
           cd /workspace/ros2_ws && rm -rf build install log && \
           colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
```

Lưu ý:

- `config/` phải mount lúc build vì `CMakeLists.txt` sinh `label_mapping.hpp`
  từ `config/label_mapping.json`.
- Build Docker dùng layout `ros2_ws/install`.
- Nếu gặp quyền Docker, thêm `sudo` trước lệnh hoặc cấu hình user vào group
  `docker`.

### Đường B — Native OS, Dự Phòng

Chỉ dùng nếu Pi host đã có ROS2 Humble, colcon và NCNN ARM64 native:

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV/ros2_ws
colcon --log-base log_user build --symlink-install \
  --packages-select avs_perception \
  --build-base build_user --install-base install_user
```

Nếu `colcon: command not found`, quay lại Đường A hoặc cài native toolchain theo
`pi_benchmark_guide.md`.

## 6. Bước 3 — Chuẩn Bị Môi Trường Đo Ổn Định

Chạy trên Pi host, ngoài container:

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV

sudo docker compose -f docker-compose.prod.yml down 2>/dev/null || true
./tools/optimize/pi_power_guard.sh setup 2000
./tools/optimize/pi_power_guard.sh status
```

Nên chạy monitor trong một terminal/tmux riêng:

```bash
cd ~/SimpleSysIDV
./tools/optimize/pi_power_guard.sh monitor ~/phase_g_power_log.txt
```

Nếu không dùng `pi_power_guard.sh`, tối thiểu ghi lại:

```bash
vcgencmd measure_temp
vcgencmd get_throttled
```

`get_throttled=0x0` là trạng thái tốt. Nếu có throttle/undervoltage, cải thiện
nguồn/cooling rồi chạy lại trước khi chốt quyết định.

## 7. Cấu Hình Benchmark Chuẩn

Cả G0 và G1 phải giữ cùng cấu hình:

```text
model: models/best_ncnn_model
video: test/test_video/video_test1.mp4
prob_threshold=0.25
nms_threshold=0.45
use_vulkan_compute=false
use_int8_inference=false
use_fp16_packed=true
use_fp16_storage=true
use_fp16_arithmetic=true
use_packing_layout=true
num_threads=3
target_size=320
decode_non_control_masks=false
max_detections=30
```

Chỉ khác:

- G0: `enable_nms=true`
- G1: `enable_nms=false`

## 8. Bước 4 — Chạy Benchmark G0/G1

Chạy từng run riêng. Không gộp bằng `&&` nếu đang nghi ngờ nguồn/cooling, để dễ
biết run nào gây sự cố.

### Đường A — Docker, Khuyến Nghị

#### G0 — NMS On

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV
mkdir -p docs/optimize/results test/test_video_output

docker run --rm \
  --network host \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/models:/workspace/models" \
  -v "$(pwd)/test:/workspace/test" \
  -v "$(pwd)/docs/optimize/results:/workspace/docs/optimize/results" \
  -e ROS2_INSTALL_DIR=install \
  avs_perception:arm64 \
  bash -lc "source /workspace/ros2_ws/install/setup.bash && \
    ros2 run avs_perception video_test_node \
      --ros-args \
      -p video_path:=/workspace/test/test_video/video_test1.mp4 \
      -p output_path:=/workspace/test/test_video_output/phase_g_g0_nms_on.mp4 \
      -p model_param_path:=/workspace/models/best_ncnn_model/model.ncnn.param \
      -p model_bin_path:=/workspace/models/best_ncnn_model/model.ncnn.bin \
      -p prob_threshold:=0.25 \
      -p nms_threshold:=0.45 \
      -p use_vulkan_compute:=false \
      -p use_int8_inference:=false \
      -p use_fp16_packed:=true \
      -p use_fp16_storage:=true \
      -p use_fp16_arithmetic:=true \
      -p use_packing_layout:=true \
      -p num_threads:=3 \
      -p target_size:=320 \
      -p decode_non_control_masks:=false \
      -p enable_nms:=true \
      -p max_detections:=30" \
  2>&1 | tee docs/optimize/results/phase_g_g0_nms_on.log
```

#### G1 — NMS Off, Top-30

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV
mkdir -p docs/optimize/results test/test_video_output

docker run --rm \
  --network host \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/models:/workspace/models" \
  -v "$(pwd)/test:/workspace/test" \
  -v "$(pwd)/docs/optimize/results:/workspace/docs/optimize/results" \
  -e ROS2_INSTALL_DIR=install \
  avs_perception:arm64 \
  bash -lc "source /workspace/ros2_ws/install/setup.bash && \
    ros2 run avs_perception video_test_node \
      --ros-args \
      -p video_path:=/workspace/test/test_video/video_test1.mp4 \
      -p output_path:=/workspace/test/test_video_output/phase_g_g1_nms_off_top30.mp4 \
      -p model_param_path:=/workspace/models/best_ncnn_model/model.ncnn.param \
      -p model_bin_path:=/workspace/models/best_ncnn_model/model.ncnn.bin \
      -p prob_threshold:=0.25 \
      -p nms_threshold:=0.45 \
      -p use_vulkan_compute:=false \
      -p use_int8_inference:=false \
      -p use_fp16_packed:=true \
      -p use_fp16_storage:=true \
      -p use_fp16_arithmetic:=true \
      -p use_packing_layout:=true \
      -p num_threads:=3 \
      -p target_size:=320 \
      -p decode_non_control_masks:=false \
      -p enable_nms:=false \
      -p max_detections:=30" \
  2>&1 | tee docs/optimize/results/phase_g_g1_nms_off_top30.log
```

### Đường B — Native OS, Dự Phòng

#### G0 — NMS On

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV/ros2_ws
source install_user/setup.bash

ros2 run avs_perception video_test_node \
  --ros-args \
  -p video_path:=/home/goln-raspi5/SimpleSysIDV/test/test_video/video_test1.mp4 \
  -p output_path:=/home/goln-raspi5/SimpleSysIDV/test/test_video_output/phase_g_g0_nms_on.mp4 \
  -p model_param_path:=/home/goln-raspi5/SimpleSysIDV/models/best_ncnn_model/model.ncnn.param \
  -p model_bin_path:=/home/goln-raspi5/SimpleSysIDV/models/best_ncnn_model/model.ncnn.bin \
  -p prob_threshold:=0.25 \
  -p nms_threshold:=0.45 \
  -p use_vulkan_compute:=false \
  -p use_int8_inference:=false \
  -p use_fp16_packed:=true \
  -p use_fp16_storage:=true \
  -p use_fp16_arithmetic:=true \
  -p use_packing_layout:=true \
  -p num_threads:=3 \
  -p target_size:=320 \
  -p decode_non_control_masks:=false \
  -p enable_nms:=true \
  -p max_detections:=30 \
  2>&1 | tee /home/goln-raspi5/SimpleSysIDV/docs/optimize/results/phase_g_g0_nms_on.log
```

#### G1 — NMS Off, Top-30

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV/ros2_ws
source install_user/setup.bash

ros2 run avs_perception video_test_node \
  --ros-args \
  -p video_path:=/home/goln-raspi5/SimpleSysIDV/test/test_video/video_test1.mp4 \
  -p output_path:=/home/goln-raspi5/SimpleSysIDV/test/test_video_output/phase_g_g1_nms_off_top30.mp4 \
  -p model_param_path:=/home/goln-raspi5/SimpleSysIDV/models/best_ncnn_model/model.ncnn.param \
  -p model_bin_path:=/home/goln-raspi5/SimpleSysIDV/models/best_ncnn_model/model.ncnn.bin \
  -p prob_threshold:=0.25 \
  -p nms_threshold:=0.45 \
  -p use_vulkan_compute:=false \
  -p use_int8_inference:=false \
  -p use_fp16_packed:=true \
  -p use_fp16_storage:=true \
  -p use_fp16_arithmetic:=true \
  -p use_packing_layout:=true \
  -p num_threads:=3 \
  -p target_size:=320 \
  -p decode_non_control_masks:=false \
  -p enable_nms:=false \
  -p max_detections:=30 \
  2>&1 | tee /home/goln-raspi5/SimpleSysIDV/docs/optimize/results/phase_g_g1_nms_off_top30.log
```

## 9. Bước 5 — Kiểm Tra Output Trên Pi

Kiểm tra file log và video:

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV
ls -lh docs/optimize/results/phase_g_g0_nms_on.log
ls -lh docs/optimize/results/phase_g_g1_nms_off_top30.log
ls -lh test/test_video_output/phase_g_g0_nms_on.mp4
ls -lh test/test_video_output/phase_g_g1_nms_off_top30.mp4
```

Ghi lại trạng thái sau benchmark:

```bash
cd ~/SimpleSysIDV
./tools/optimize/pi_power_guard.sh status
vcgencmd measure_temp
vcgencmd get_throttled
```

Nếu có throttle/undervoltage hoặc Pi reboot giữa chừng, không chốt kết quả. Sửa
nguồn/cooling rồi chạy lại.

## 10. Bước 6 — Kéo Kết Quả Về Laptop

Trên laptop:

```bash
mkdir -p /home/goln/SimpleSysIDV/docs/optimize/results
mkdir -p /home/goln/SimpleSysIDV/test/test_video_output

rsync -avz \
  goln-raspi5@goln-raspi5.local:/home/goln-raspi5/SimpleSysIDV/docs/optimize/results/ \
  /home/goln/SimpleSysIDV/docs/optimize/results/

rsync -avz \
  goln-raspi5@goln-raspi5.local:/home/goln-raspi5/SimpleSysIDV/test/test_video_output/phase_g_g*.mp4 \
  /home/goln/SimpleSysIDV/test/test_video_output/
```

`docs/optimize/results/` là thư mục kết quả cục bộ. Không commit log/video thô
nếu chưa có lý do rõ; nên tổng hợp thành report markdown.

## 11. Bước 7 — Đọc Kết Quả Log

Trong mỗi log, tìm `PROFILING REPORT`:

```bash
grep -A30 "PROFILING REPORT" docs/optimize/results/phase_g_g0_nms_on.log
grep -A30 "PROFILING REPORT" docs/optimize/results/phase_g_g1_nms_off_top30.log
```

Các dòng quan trọng:

```text
Detection Latency stats:
  - Average
  - P50
  - P95
  - Min
  - Max
Inference Performance
Object Count
Timing breakdown mean / p95:
  - Preprocess
  - Extractor
  - Proposal
  - NMS/select
  - Mask
```

Cách đọc:

- `P95` là metric chính, ưu tiên hơn average.
- `NMS/select` cho biết bỏ NMS tiết kiệm được bao nhiêu.
- `Mask` cho biết NMS off có làm decode mask tăng bù hay không.
- `Object Count` tăng mạnh là dấu hiệu duplicate có thể gây nhiễu downstream.
- FPS chỉ là metric phụ; không chọn config chỉ vì FPS nhỉnh hơn nhưng p95 xấu.

## 12. Bước 8 — Kiểm Tra Video Output

Mở hoặc copy hai file:

```text
test/test_video_output/phase_g_g0_nms_on.mp4
test/test_video_output/phase_g_g1_nms_off_top30.mp4
```

So sánh bằng mắt:

- Lane polygon có bị duplicate dày bất thường không.
- `main-lane` và `turn-lane` có ổn định không.
- Mask có chồng nhiều lớp lên cùng vùng không.
- Sign/vehicle có làm object count tăng bất thường không.
- Có frame nào bản NMS off nhìn nhiễu rõ hơn NMS on không.

## 13. Rule Ra Quyết Định

Chọn `enable_nms=false` làm production default chỉ khi:

- G1 giảm p95 `Detection Latency` rõ so với G0.
- `Mask` mean/p95 của G1 không tăng bù phần tiết kiệm từ `NMS/select`.
- `Object Count` của G1 không tăng tới mức gây nhiễu.
- Video G1 không có duplicate lane/mask gây sai quan sát.
- Pi không throttle/undervoltage trong cả hai run.

Giữ `enable_nms=true` nếu:

- p95 giảm không đáng kể.
- `Mask` tăng mạnh khi NMS off.
- Object count tăng mạnh.
- Video NMS off nhiễu hơn.
- Cần tăng `prob_threshold` cao mới kiểm soát duplicate, làm mất class quan
  trọng.

## 14. Bước 9 — Viết Report Tổng Hợp

Tạo report trên laptop:

```bash
nano /home/goln/SimpleSysIDV/docs/optimize/results/phase_g_nms_benchmark_report.md
```

Mẫu report:

```markdown
# Phase G NMS Benchmark Report

Date:
Hardware: Raspberry Pi 5
Build path: Docker avs_perception:arm64 / Native OS
Power supply:
Cooling:
Power guard cap:
Throttle before:
Throttle after:
Temperature before:
Temperature after:

Model:
Video:
Runtime:
- prob_threshold=0.25
- nms_threshold=0.45
- use_vulkan_compute=false
- use_int8_inference=false
- use_fp16_packed=true
- use_fp16_storage=true
- use_fp16_arithmetic=true
- use_packing_layout=true
- num_threads=3
- target_size=320
- decode_non_control_masks=false
- max_detections=30

## G0: NMS On

- Average latency:
- P50 latency:
- P95 latency:
- FPS:
- Object count avg/max:
- NMS/select mean/p95:
- Mask mean/p95:
- Output video:
- Log:

## G1: NMS Off Top-30

- Average latency:
- P50 latency:
- P95 latency:
- FPS:
- Object count avg/max:
- NMS/select mean/p95:
- Mask mean/p95:
- Output video:
- Log:

## Visual Check

- Duplicate lane/mask:
- Main-lane stability:
- Turn-lane stability:
- Notes:

## Decision

- enable_nms default:
- max_detections default:
- Reason:
```

## 15. Sau Khi Chốt

Nếu chọn NMS off:

- Cập nhật production config/default sang `enable_nms=false`.
- Giữ `max_detections` theo giá trị report đã chốt.
- Chạy lại build package và một run sanity.

Nếu giữ NMS on:

- Giữ default `enable_nms=true`.
- Vẫn giữ param `enable_nms` để debug/benchmark sau này.

Sau đó mới chuyển sang runtime stability camera live dài hạn trên Pi 5: cooling,
nguồn, throttling, governor, `num_threads`, `output_age_ms`, jitter IPM/control.

## 16. Troubleshooting Nhanh

| Triệu chứng | Nguyên nhân khả dĩ | Cách xử lý |
|---|---|---|
| `Unable to find image 'avs_perception:arm64' locally` | Pi chưa load production image | Build/transfer image theo Docker guide, hoặc dùng Đường B native |
| `colcon: command not found` trong container | Image thiếu colcon | Lệnh build Docker đã tự cài; nếu vẫn lỗi, kiểm tra mạng apt trong container |
| `colcon: command not found` ngoài container | Host OS thiếu colcon | Dùng Đường A Docker hoặc cài `python3-colcon-common-extensions` |
| `No rule to make target '/workspace/config/label_mapping.json'` | Thiếu mount `config` lúc build Docker | Thêm `-v "$(pwd)/config:/workspace/config"` rồi build lại |
| `Could not open input video file` | Video chưa sync hoặc sai path container/native | Kiểm tra `test/test_video/video_test1.mp4`; với Docker dùng `/workspace/test/...` |
| `Failed to load NCNN model` | Model chưa sync hoặc sai path | Kiểm tra `models/best_ncnn_model/*.param/*.bin`; với Docker dùng `/workspace/models/...` |
| Không tạo được output video | Thiếu thư mục output hoặc quyền ghi | `mkdir -p test/test_video_output`; nếu file root-owned từ Docker, dùng `sudo chown -R $USER:$USER test/test_video_output` |
| Pi reboot hoặc SSH mất giữa run | Brownout nguồn/cáp hoặc spike tải | Chạy `pi_power_guard.sh setup 2000`, monitor EXT5V, đổi cáp/adapter, chạy từng run riêng |
