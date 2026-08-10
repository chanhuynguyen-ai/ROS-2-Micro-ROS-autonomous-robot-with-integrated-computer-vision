# Hướng Dẫn Benchmark NCNN Inference Trên Raspberry Pi 5 (FP32 / FP16 / INT8 / Vulkan)

Guide này nối tiếp `ncnn_inference_latency_plan.md` sau khi Pha A (harness) và Pha B
(runtime parameters) đã xong. Mục tiêu: chạy benchmark **B0 (INT8, baseline hiện
tại) / B1 (FP32 CPU) / B2 (FP16 CPU)** thật trên phần cứng Raspberry Pi 5, rồi kéo
kết quả về laptop để đánh giá và điền vào bảng quyết định ở `ncnn_inference_latency_plan.md`
§7. Benchmark Vulkan (B3/B4, Pha D) dùng một image Docker riêng — làm theo §13
**sau khi** đã có kết quả B0/B1/B2.

## 1. Vì Sao Không Benchmark Được Trên Laptop

`tools/optimize/run_benchmark_suite.sh` chạy `ros2 run avs_perception ...` trực
tiếp (không qua Docker) và cần NCNN + workspace build native. Số liệu latency/FPS
trên CPU x86_64 của laptop không phản ánh Cortex-A76 của Pi 5, nên phải chạy trên
phần cứng thật.

## 2. Điều Kiện Tiên Quyết

- SSH vào được Test Pi: `goln-raspi5@goln-raspi5.local` (đã cấu hình key trước;
  nếu chưa, `ssh-copy-id goln-raspi5@goln-raspi5.local` từ laptop một lần).
- **Đường build mặc định của guide này là Docker** (§4 Đường A) — dùng đúng
  image production `avs_perception:arm64`, không cần cài ROS2/colcon lên OS của
  Pi. Chỉ cần Docker đã cài trên Pi và image `avs_perception:arm64` đã có
  (`docker images | grep avs_perception`; nếu trống, build+transfer theo
  `skills/docker_SKILL/SKILL.md` §5 trước).
- Đường native (§4 Đường B) là dự phòng, chỉ dùng nếu Docker không khả dụng —
  yêu cầu ROS2 Humble + `python3-colcon-common-extensions` cài native trên OS
  của Pi (base production chỉ dùng ROS2 **bên trong container**, nên OS host
  thường KHÔNG có sẵn `colcon` — đây là nguyên nhân lỗi `colcon: command not
  found` nếu thử build native mà chưa cài).
- Không có `docker compose -f docker-compose.prod.yml up` đang chạy trên Pi trong
  lúc benchmark (production containers sẽ tranh CPU, tranh cổng DDS và làm sai
  số liệu) — dừng bằng `docker compose -f docker-compose.prod.yml down` trước.

## 3. Bẫy Quan Trọng: `ncnn/` Trong Repo Là Build x86_64, Không Dùng Được Trên Pi

Thư mục `ncnn/` ở gốc repo (`ncnn/lib/libncnn.so*`) đang được commit vào git
nhưng là **build x86_64 của laptop** (`file` xác nhận `ELF 64-bit ... x86-64`).
`CMakeLists.txt` của `avs_perception` tự động tìm `ncnn_DIR` theo thứ tự:

1. biến môi trường `ncnn_DIR`
2. `ncnn-src/build_vulkan/install/lib/cmake/ncnn` (build Vulkan đang làm dở trên laptop)
3. `ncnn/lib/cmake/ncnn` (**x86_64, sai kiến trúc nếu rsync sang Pi**)
4. `/usr/lib/cmake/ncnn` hoặc `/usr/local/lib/cmake/ncnn` (NCNN cài native trên máy)

Vì vậy khi đồng bộ code sang Pi **bắt buộc loại trừ `ncnn/` và `ncnn-src/`**
(giống rsync trong `skills/docker_SKILL/SKILL.md` §5.3), nếu không colcon sẽ link
nhầm thư viện x86_64 và build lỗi hoặc crash khi chạy trên ARM64.

## 4. Bước 0 — Chọn Đường Build: Docker (Đường A, khuyến nghị) hay Native OS (Đường B)

### Đường A — Docker (khuyến nghị)

Image production `avs_perception:arm64` đã build sẵn NCNN native ARM64 bên
trong (`docker/Dockerfile`, branch `20240820`, cờ `NCNN_VULKAN=OFF
-DNCNN_ARM_NEON=ON -DCMAKE_INSTALL_PREFIX=/usr`) — dùng đúng image này để
benchmark nghĩa là **cùng NCNN build với production**, không cần cài gì thêm
lên OS của Pi, và tránh hẳn lỗi `colcon: command not found` (colcon chỉ cần có
trong container, không cần trên host). Đây là đường được dùng trong §6/§8 của
guide này.

Kiểm tra trước khi build (Bước 2):

```bash
ssh goln-raspi5@goln-raspi5.local
docker images | grep avs_perception          # image đã load chưa?
docker run --rm avs_perception:arm64 which colcon   # colcon có sẵn trong image chưa?
```

- Nếu dòng đầu **trống** → image chưa có trên Test Pi, build + transfer theo
  `skills/docker_SKILL/SKILL.md` §5 (cross-build trên laptop qua QEMU, `docker
  save` rồi rsync/`docker load` sang Pi) trước khi tiếp tục.
- Nếu dòng thứ hai **không in ra path** (colcon thiếu trong image) — không sao,
  lệnh build ở §6 đã tự cài `python3-colcon-common-extensions` bên trong
  container nếu thiếu, không cần xử lý thủ công ở đây.

### Đường B — Native OS (dự phòng, chỉ dùng nếu Docker không khả dụng)

Nếu Pi không có Docker hoặc không load được image, có thể build NCNN + ROS2
workspace thẳng trên OS của Pi. Kiểm tra trước:

```bash
ssh goln-raspi5@goln-raspi5.local
find /usr /usr/local -maxdepth 6 -iname 'ncnnConfig.cmake' 2>/dev/null
which colcon
```

Nếu **cả hai đều có kết quả** → đã sẵn sàng, sang Bước 1 (dùng nhánh Native ở
§6/§8).

Nếu **thiếu colcon**, cài trước (không có sẵn dù ROS2 Humble base đã cài, vì
đây là gói dev tool riêng, không nằm trong `ros-humble-ros-base`):

```bash
sudo apt-get update && sudo apt-get install -y python3-colcon-common-extensions
```

Nếu **thiếu NCNN native** (`ncnnConfig.cmake` không tìm thấy), build NCNN,
dùng đúng version với Docker production (`20240820`) để không lẫn confound
version khác vào so sánh FP32/FP16 (KHÔNG dùng `ncnn-src` hiện tại trên laptop
— đó là commit mới hơn, đang dùng riêng cho thử nghiệm Vulkan Pha D):

```bash
sudo apt-get install -y build-essential cmake git libopencv-dev
cd ~ && git clone --depth 1 --branch 20240820 https://github.com/Tencent/ncnn.git ncnn-native-build
cd ncnn-native-build && git submodule update --init
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DNCNN_VULKAN=OFF \
      -DNCNN_BUILD_EXAMPLES=OFF \
      -DNCNN_BUILD_TOOLS=OFF \
      -DNCNN_BUILD_BENCHMARK=OFF \
      -DNCNN_SHARED_LIB=ON \
      -DNCNN_ARM_NEON=ON \
      ..
make -j$(nproc)
sudo make install
cd ~ && rm -rf ncnn-native-build   # dọn source sau khi install xong
```

Cài vào `/usr` để `CMakeLists.txt` tự tìm thấy qua fallback thứ 4 ở §3 — không
cần set `ncnn_DIR` thủ công mỗi lần build.

## 5. Bước 1 — Đồng Bộ Code Sang Pi

Repo gốc có nhiều thư mục không liên quan đến benchmark — một số rất nặng và/hoặc
kiến trúc x86_64 (`.venv` ~7.8GB, `.codegraph` ~224MB, `.tokensave` ~193MB,
`avs_perception_arm64.tar` ~541MB Docker image, `node_modules`, `docs`, `web_dashboard`,
`scratch`, `skills`, `.agents`, `.claude`, `.codex`, `scripts/` export tooling, và
`build/`/`install/`/`log/` ở gốc repo — đây là **build x86_64 để lại từ colcon chạy
trên laptop**, khác với `ros2_ws/build*` đã biết, cũng không được sync).

Thay vì liệt kê từng thư mục cần loại trừ (dễ sót thư mục mới phát sinh sau này),
dùng **allowlist** — chỉ định đúng các đường dẫn cần cho benchmark, mọi thứ khác
tự động bị loại:

```bash
rsync -avz \
  --include='/ros2_ws/' --include='/ros2_ws/src/' --include='/ros2_ws/src/***' \
  --include='/models/' \
  --include='/models/best_ncnn_model/' --include='/models/best_ncnn_model/***' \
  --include='/models/best_ncnn_model_int8/' --include='/models/best_ncnn_model_int8/***' \
  --include='/test/' --include='/test/test_video/' --include='/test/test_video/***' \
  --include='/tools/' --include='/tools/optimize/' --include='/tools/optimize/***' \
  --include='/config/' --include='/config/***' \
  --exclude='*' \
  /home/goln/SimpleSysIDV/ goln-raspi5@goln-raspi5.local:/home/goln-raspi5/SimpleSysIDV/
```

Đã kiểm chứng bằng `rsync -n` (dry-run): lệnh trên chỉ đồng bộ đúng
`ros2_ws/src/**` (toàn bộ package `avs_perception`, gồm cả `control_node.cpp`,
`ipm_transform_node.cpp` v.v. vì colcon build cả package chứ không chỉ
`ncnn_inference_node`), `models/best_ncnn_model{,_int8}/**`, `test/test_video/**`,
`tools/optimize/**`, `config/**` (cần cho `LABEL_MAPPING_JSON` trong
`CMakeLists.txt`) — tổng khoảng 60MB, chủ yếu là video test, thay vì hàng GB nếu
sync cả repo.

`.venv/` không nằm trong allowlist nên tự động bị loại — nó chỉ phục vụ dev/training
trên x86_64 (`pyvenv.cfg` có `include-system-site-packages = true`, tức `rclpy`/
`std_msgs` nó dùng thực chất kế thừa từ ROS2 Humble cài hệ thống, không vendor
riêng). `tools/optimize/benchmark_telemetry.py` chỉ cần `rclpy` + `std_msgs`, đã
có sẵn khi Pi cài ROS2 Humble qua apt (§2).

Nếu sau này cần thêm dữ liệu (ví dụ fixture video mới, hoặc muốn xem dashboard
trực tiếp trên Pi trong lúc benchmark), thêm đúng `--include` tương ứng thay vì
chuyển hẳn sang blacklist.

## 6. Bước 2 — Build Trên Pi

### Đường A — Docker (khuyến nghị)

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV
docker run --rm \
  --network host \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/config:/workspace/config" \
  avs_perception:arm64 \
  bash -c "command -v colcon >/dev/null || (apt-get update && apt-get install -y python3-colcon-common-extensions) && \
           cd /workspace/ros2_ws && rm -rf build install log && \
           colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
```

`config/` phải mount **ngay từ bước build**, không chỉ lúc chạy benchmark ở
Bước 4 — `CMakeLists.txt` có target `generate_label_mapping` đọc
`config/label_mapping.json` tại thời điểm build (sinh `label_mapping.hpp`), bỏ
sót mount này sẽ lỗi `No rule to make target
'/workspace/config/label_mapping.json'`.

Container tự cài `colcon` nếu image chưa có sẵn (§4 Đường A đã kiểm tra trước).
Build ghi vào `ros2_ws/build`, `install`, `log` — vì đây là layout mặc định mà
production dùng (`docker-compose.prod.yml`), **không phải** `install_user` như
build native trên laptop. Nếu gặp `permission denied` khi `docker` chạy, thêm
`sudo` trước lệnh (hoặc `sudo usermod -aG docker $USER` rồi đăng nhập lại SSH).

Build chạy dưới quyền root trong container (giống hệt cách production tự build
mỗi lần container khởi động) nên `ros2_ws/build`/`install`/`log` trên Pi sẽ
thuộc sở hữu `root` — không ảnh hưởng benchmark, chỉ cần lưu ý nếu sau này
muốn `rm -rf` thủ công thì dùng `sudo`.

### Đường B — Native OS (dự phòng)

Dùng đúng layout `install_user` mà `tools/optimize/run_benchmark_suite.sh` mặc
định kỳ vọng (`ros2_ws/install_user/setup.bash`), khớp quy ước trong `CLAUDE.md`:

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV/ros2_ws
colcon --log-base log_user build --symlink-install \
  --packages-select avs_perception \
  --build-base build_user --install-base install_user
```

Kiểm tra log build in ra `AVS: Release build — ARM64 optimizations ENABLED
(-O3 -march=armv8.2-a -mtune=cortex-a76)` — nếu in ra dòng x86_64/generic thay
vào đó nghĩa là `CMAKE_SYSTEM_PROCESSOR` không nhận đúng ARM64 (không nên xảy ra
trên Pi 5 thật, nhưng cross-check nếu số liệu bất thường).

## 7. Bước 3 — Chuẩn Bị Môi Trường Đo Ổn Định

Trên Pi (host OS, ngoài container), trước khi benchmark:

```bash
# Dừng production containers nếu đang chạy, tránh tranh CPU
sudo docker compose -f docker-compose.prod.yml down 2>/dev/null

# Chuẩn bị power-safe: governor=performance NHƯNG cap tần số 2.0GHz
# (giảm dòng đỉnh — xem §8.1; mọi preset cùng cap nên so sánh tương đối
# FP32/FP16/INT8 vẫn hợp lệ), đồng thời bật persistent journal.
cd ~/SimpleSysIDV
./tools/optimize/pi_power_guard.sh setup 2000

# Kiểm tra trạng thái (temp, throttle, tần số, điện áp 5V đầu vào)
./tools/optimize/pi_power_guard.sh status
```

`pi_power_guard.sh` nằm trong `tools/optimize/` nên đã được rsync sang Pi ở
Bước 1. Chạy `setup` **không tham số** để bỏ cap (full 2.4GHz) — chỉ dùng khi
sự cố nguồn ở §8.1 đã được giải quyết dứt điểm bằng cáp/adapter mới và cần số
liệu tuyệt đối cuối cùng. Sau khi benchmark xong, trả Pi về mặc định bằng
`./tools/optimize/pi_power_guard.sh restore`.

Lưu ý khi so sánh: các run phải **cùng một mức cap** mới so được với nhau —
`cpu_freq_before/after.scaling_max_freq` trong `metadata_<run_id>.json` ghi
lại mức cap của từng run để đối chiếu.

`tools/optimize/run_benchmark.sh` đã tự ghi nhiệt độ/throttle/CPU util, tần số
CPU (cur/max) và toàn bộ PMIC ADC (`vcgencmd pmic_read_adc`, gồm `EXT5V_V` —
điện áp 5V đầu vào thật) trước-sau vào `metadata_<run_id>.json`, không cần đo
thủ công trong lúc chạy.

## 8. Bước 4 — Chạy Benchmark B0 / B1 / B2

### Đường A — Docker (khuyến nghị)

**Chạy từng preset một lệnh `docker run` riêng (mặc định, khuyến nghị sau sự cố
mất điện §8.1)** thay vì gộp cả 3 preset vào một lệnh `bash -c "... && ... &&
..."` — mỗi lệnh dưới đây là một `docker run` độc lập, chạy xong thì thoát hẳn,
cho Pi nghỉ thật giữa các preset thay vì chỉ 5s cooldown nội bộ của script:

```bash
ssh goln-raspi5@goln-raspi5.local
mkdir -p ~/SimpleSysIDV/docs/optimize/results
cd ~/SimpleSysIDV

# Chạy lệnh này cho TỪNG preset (b0, rồi b1, rồi b2), kiểm tra `ssh` vẫn còn
# vào được Pi bình thường giữa mỗi lần trước khi chạy preset tiếp theo.
docker run --rm \
  --network host \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/models:/workspace/models" \
  -v "$(pwd)/config:/workspace/config" \
  -v "$(pwd)/test:/workspace/test" \
  -v "$(pwd)/tools:/workspace/tools" \
  -v "$(pwd)/docs/optimize/results:/workspace/docs/optimize/results" \
  --device /dev/dri \
  -e ROS2_INSTALL_DIR=install \
  avs_perception:arm64 \
  bash -c "cd /workspace && ./tools/optimize/run_benchmark_suite.sh b0"
```

Đổi `b0` thành `b1` rồi `b2` ở cuối lệnh cho hai lần chạy sau. Giữa mỗi lần,
đợi vài phút và xác nhận Pi vẫn phản hồi (`ssh goln-raspi5@goln-raspi5.local
uptime`) trước khi chạy preset kế tiếp — nếu Pi sập ở một preset cụ thể, bạn
biết ngay preset nào gây ra thay vì mất luôn cả batch.

Nên chạy cùng lúc sampler nhiệt độ/điện áp ở §8.1 trong một `tmux`/`screen`
pane khác trong suốt Bước 4, để nếu sập lại thì có log ngay trước thời điểm đó
thay vì phải suy luận ngược.

Lệnh gộp cả 3 preset trong một `docker run` (dùng `&&`) vẫn có thể dùng **sau
khi** đã xác nhận từng preset chạy ổn định riêng lẻ không gây sập máy:

```bash
docker run --rm \
  --network host \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/models:/workspace/models" \
  -v "$(pwd)/config:/workspace/config" \
  -v "$(pwd)/test:/workspace/test" \
  -v "$(pwd)/tools:/workspace/tools" \
  -v "$(pwd)/docs/optimize/results:/workspace/docs/optimize/results" \
  --device /dev/dri \
  -e ROS2_INSTALL_DIR=install \
  avs_perception:arm64 \
  bash -c "cd /workspace && ./tools/optimize/run_benchmark_suite.sh b0 && \
           ./tools/optimize/run_benchmark_suite.sh b1 && \
           ./tools/optimize/run_benchmark_suite.sh b2"
```

- `mkdir -p ~/SimpleSysIDV/docs/optimize/results` cần chạy trước vì `docs/` không
  nằm trong allowlist rsync ở Bước 1 — chỉ tạo đúng thư mục kết quả, không sync
  cả `docs/`.
- `--device /dev/dri` chỉ cần cho Vulkan (§13) nhưng vô hại ở các preset
  CPU. **Không dùng `-v /dev:/dev` thay thế** — bind-mount chỉ làm node hiện
  ra, device cgroup của Docker vẫn chặn `open()` (`Operation not permitted`).
- `vcgencmd` bên trong container: Test Pi (Ubuntu 22.04) **không có
  `/dev/vchiq`** (node mà `vcgencmd` cần, chỉ Raspberry Pi OS expose mặc
  định — đã xác nhận thực tế 2026-07-08 khi `--device /dev/vchiq` báo `no
  such file or directory`). Vì vậy `temperature/throttled/pmic_adc` trong
  `metadata_*.json` sẽ ghi `"N/A"`/`[]` khi benchmark qua Docker — chấp nhận
  được: nguồn giám sát nhiệt/điện áp chính là `pi_power_guard.sh monitor`
  chạy trên host (§8.1). Nếu Pi nào có `/dev/vchiq` (host RPi OS), thêm
  `--device /dev/vchiq` để metadata có số liệu thật.
- `ROS2_INSTALL_DIR=install` báo cho `run_benchmark_suite.sh` biết cần source
  `ros2_ws/install/setup.bash` (layout container ở Bước 2), khác mặc định
  `install_user` dùng cho native.
- `git_commit` trong metadata sẽ ghi `"unknown"` vì `.git/` không được mount vào
  container — chấp nhận được, chỉ mất thông tin provenance, không ảnh hưởng số
  liệu benchmark.
- Suite tự **so le khởi động** node (inference load model một mình trước,
  10s sau mới khởi động publisher — biện pháp giảm cú bước dòng, xem §8.1);
  thêm `-e BENCH_STAGGER_SEC=20` vào `docker run` nếu muốn giãn hơn. Khoảng
  so le nằm trước warmup nên không ảnh hưởng số liệu đo.

### 8.1 Pi Tự Tắt Nguồn Giữa Chừng — Ghi Nhận Sự Cố Và Cách Giám Sát

Đã ghi nhận trường hợp thực tế: Pi 5 (dùng đúng adapter chính hãng, cáp cắm
chặt) **tự tắt nguồn đột ngột và tái hiện nhiều lần**, luôn xảy ra khi đang
chạy Bước 4 Đường A (không xảy ra ở các bước khác như rsync, build). Đặc điểm
quan sát được:

- Nhiệt độ lúc sập chỉ 45-47°C (`vcgencmd measure_temp`) — **không phải quá
  nhiệt**.
- `vcgencmd get_throttled` đọc `0x0` liên tục đến tận dòng log cuối — firmware
  không kịp set cờ undervoltage trước khi mất điện, tức là điện áp sụp rất
  nhanh/dứt khoát chứ không tụt từ từ.
- Log sampler (xem lệnh bên dưới) bị cắt đột ngột giữa chừng, đôi khi kèm ký
  tự rác ở dòng cuối — dấu hiệu mất điện đột ngột ở tầng vật lý, không phải
  shutdown có kiểm soát (không có log kernel panic/halt).
- `journalctl --list-boots` chỉ thấy đúng 1 boot mỗi lần kiểm tra sau sự cố —
  xác nhận Pi đã reboot hoàn toàn, không phải benchmark bị treo hay mất kết
  nối SSH/VNC đơn thuần (đã loại trừ bằng cách chạy trong `tmux`, vẫn sập).
- File kết quả (`benchmark_<run_id>.csv`, `metadata_<run_id>.json`) tạo ra 0
  byte, đúng ngay thời điểm reboot — benchmark bị cắt ngay khi vừa bắt đầu,
  gợi ý nguyên nhân liên quan đến cú tăng dòng đột ngột lúc
  `video_publisher_node` + `ncnn_inference_node` cùng khởi động, hơn là do
  tải trung bình kéo dài.

**Cập nhật 2026-07-08 — chạy MỘT preset duy nhất vẫn sập.** Điều này loại
trừ giả thuyết "tải tích lũy qua nhiều preset": trigger nằm ở **cú bước tải
(load step) đầu mỗi run**, mà run nào cũng có, nên tách preset không đủ.
Phân tích hệ thống benchmark chỉ ra 3 yếu tố làm cú bước tải này tệ nhất có
thể về mặt điện (đều đã được khắc phục trong script):

- `run_benchmark_suite.sh` (cũ) khởi động `video_publisher_node` và
  `ncnn_inference_node` **đồng thời** → decode video + load model (cấp phát
  bộ nhớ lớn, transform weight) + inference full-thread ập vào cùng lúc.
  **Đã sửa:** node inference khởi động một mình trước, đợi
  `BENCH_STAGGER_SEC` (mặc định 10s) cho model load xong rồi mới khởi động
  publisher — tách một cú bước dòng lớn thành hai cú nhỏ.
- Governor `performance` giữ cả 4 core ở 2.4GHz ngay từ idle → cả biên độ
  dòng đỉnh lẫn độ dốc dI/dt đều tối đa (production không set governor này
  nên chưa từng chạm profile điện đó). **Đã sửa:** §7 giờ dùng
  `pi_power_guard.sh setup 2000` — giữ clock cố định (đo vẫn ổn định) nhưng
  cap 2.0GHz, giảm đáng kể dòng đỉnh (P ~ V²·f, 2.4GHz cần VDD cao hơn).
- Sampler cũ đo `measure_volts core` — đây là rail nội bộ được PMIC giữ ổn
  định đến giây cuối, **không thấy được sụt áp 5V đầu vào**. **Đã sửa:**
  `pi_power_guard.sh monitor` đọc `vcgencmd pmic_read_adc EXT5V_V` (5V input
  thật): khỏe ~5.0–5.2V, nếu tụt về ~4.6V ngay trước khi sập là bằng chứng
  brownout nguồn.

Lưu ý trung thực: phần mềm **không thể tự cắt nguồn Pi** — gốc rễ vẫn là
điện (nghi ngờ hàng đầu: cáp/đầu nối USB-C tiếp xúc kém dưới dòng tải cao,
hoặc adapter xuống cấp — cần thử đổi cáp/adapter khác và/hoặc board Pi 5
khác để khoanh vùng). Các biện pháp phần mềm ở trên chỉ hạ dòng đỉnh xuống
dưới ngưỡng sập và thu bằng chứng; chúng không thay việc sửa phần cứng.

Quy trình chạy an toàn (sau khi đã `setup 2000` ở Bước 3):

1. **Chạy từng preset một lệnh `docker run` riêng** (mặc định ở trên) thay
   vì gộp `b0 && b1 && b2` trong một lệnh.
2. **Chạy sampler song song** trong một `tmux` pane khác (cài `tmux` trước
   nếu chưa có: `sudo apt-get install -y tmux`):
   ```bash
   tmux new -s monitor
   ./tools/optimize/pi_power_guard.sh monitor ~/power_log.txt
   ```
   Ctrl+B rồi D để detach, `tmux attach -t monitor` để xem lại. Nếu sập lại,
   `tail -20 ~/power_log.txt` cho biết EXT5V_V ngay trước lúc mất điện.
3. **Chạy benchmark trong `tmux` riêng** (không gắn trực tiếp phiên SSH/VNC),
   để tách biệt rõ giữa "mất kết nối" và "Pi thật sự mất điện":
   ```bash
   tmux new -s bench
   # paste lệnh docker run (per-preset) ở trên vào đây
   ```
4. Nếu vẫn sập với cap 2000: thử cap thấp hơn (`setup 1500`) để xác nhận
   hướng nguyên nhân (sập biến mất ở cap thấp = chắc chắn brownout nguồn),
   nhưng **không dùng số liệu cap 1500 để chốt quyết định** — chỉ để chẩn
   đoán. Song song, đổi cáp USB-C khác (ưu tiên cáp ngắn, đạt chuẩn 5A/100W)
   rồi đổi hẳn adapter khác (không chỉ cáp) trước khi nghi ngờ lỗi board.
5. Sau sự cố, soi log boot trước bằng `journalctl -k -b -1` (persistent
   journal đã được `setup` bật sẵn; nếu vẫn báo "no persistent journal was
   found", kiểm tra `Storage=` trong `/etc/systemd/journald.conf` — có thể
   đang bị set `volatile`, cần đổi thành `auto` hoặc `persistent`).

### Đường B — Native OS (dự phòng)

Trên Pi, từ gốc repo (`~/SimpleSysIDV`):

```bash
cd ~/SimpleSysIDV
./tools/optimize/run_benchmark_suite.sh b0   # INT8 CPU — baseline hiện tại
./tools/optimize/run_benchmark_suite.sh b1   # FP32 CPU
./tools/optimize/run_benchmark_suite.sh b2   # FP16 CPU
```

(mặc định dùng layout `install_user` từ Bước 2 Đường B; không cần set
`ROS2_INSTALL_DIR`.)

### Chung cho cả hai đường

Không chạy `all` hoặc `b3`/`b4` với image/build CPU của Bước 0/2 — NCNN ở đó
dùng `NCNN_VULKAN=OFF`, nên `use_vulkan_compute=true` sẽ tự fallback về CPU
(xem `yolo26_seg.cpp::set_options`) và số liệu bị dán nhãn "Vulkan" trong khi
thực chất chạy CPU. `run_benchmark_suite.sh` giờ tự phát hiện trường hợp này
(soi log node tìm cảnh báo "compiled without Vulkan support") và **abort có
chủ đích** thay vì ghi số liệu sai. Benchmark Vulkan thật dùng image
`avs_perception:arm64-vulkan` riêng — làm theo §13.

Mỗi lệnh chạy ~300 frame + 30 frame warmup, tự dừng khi đủ hoặc watchdog timeout.
Mỗi run tạo 3 file trong `docs/optimize/results/` trên Pi:
`metadata_<run_id>.json`, `benchmark_<run_id>.csv`, `benchmark_summary_<run_id>.json`.

Script chấp nhận tham số fixture thứ hai (mặc định `video_test1`), truyền sau
tên preset, ví dụ `run_benchmark_suite.sh b1 video_test1` (thêm vào cuối lệnh
tương ứng ở trên) để sẵn sàng chạy trên các fixture khác khi bổ sung (xem §11
— hiện repo chỉ có một video test).

## 9. Bước 5 — Kéo Kết Quả Về Laptop

Trên **laptop**:

```bash
mkdir -p /home/goln/SimpleSysIDV/docs/optimize/results
rsync -avz \
  goln-raspi5@goln-raspi5.local:/home/goln-raspi5/SimpleSysIDV/docs/optimize/results/ \
  /home/goln/SimpleSysIDV/docs/optimize/results/
```

`docs/optimize/results/` đã nằm trong `.gitignore` — kết quả benchmark là dữ
liệu đo cục bộ, không commit trực tiếp (nếu muốn lưu vĩnh viễn, tổng hợp thành
bảng markdown ở Bước 6 rồi commit bảng đó).

## 10. Bước 6 — Tổng Hợp Bảng So Sánh

Trên laptop, dùng script mới `tools/optimize/compare_results.py` để dựng bảng
theo đúng format §7 của `ncnn_inference_latency_plan.md`:

```bash
python3 tools/optimize/compare_results.py docs/optimize/results
```

In ra bảng markdown gồm: mode, fixture, avg inference latency, P95 total
latency, P95 output age, avg FPS, main-lane missing rate, turn-lane present
rate, nhiệt độ trước→sau, trạng thái throttle, run ID. Copy kết quả vào bảng
§7 của plan và đánh giá theo quy tắc đã có trong plan:

- Ưu tiên mode có p95 thấp và ổn định, không chỉ avg FPS cao.
- Nếu FP16 không nhanh hơn đáng kể hoặc `main_lane_missing_rate` /
  `turn_lane_present_rate` xấu hơn INT8/FP32, không chuyển default sang FP16.
- Không chọn INT8 làm production nếu `turn_lane_present_rate` giảm rõ so với FP32.

## 11. Giới Hạn Hiện Tại — Đọc Trước Khi Kết Luận

- **Đường A (Docker) dùng đúng image production** (`avs_perception:arm64`),
  nên không còn rủi ro lệch version NCNN giữa benchmark và production như lo
  ngại trước đây với build native riêng ở Đường B — Đường A là lựa chọn đáng
  tin cậy hơn cho số liệu quyết định production.
- **Chỉ có một fixture video** (`test/test_video/video_test1.mp4`). Plan §5.3
  yêu cầu tối thiểu 3 nhóm (thẳng/ổn định, có `turn-lane`, nhiễu). Kết quả từ
  guide này chỉ đủ để so sánh sơ bộ FP32/FP16/INT8 trên một kịch bản — **chưa
  đủ để chốt production candidate cuối cùng** theo Gate của Pha C
  (`ncnn_inference_latency_plan.md` §6.3). Cần bổ sung fixture trước khi coi
  Pha C là hoàn thành.
- **Vulkan (Pha D) không chạy được với image production** vì cả image Docker
  lẫn NCNN native build ở Bước 0 đều dùng `NCNN_VULKAN=OFF` để khớp production.
  Benchmark Vulkan dùng image riêng `avs_perception:arm64-vulkan`
  (`docker/Dockerfile.vulkan`) — quy trình đầy đủ ở §13. Lưu ý image Vulkan
  dùng NCNN **khác version** với production (commit `b16501a` thay vì
  `20240820`), nên khi so Vulkan vs CPU phải re-run B1/B2 trong chính image
  Vulkan để loại confound version (§13.5).
- Plan §6.3 mô tả "export model FP16 bằng Ultralytics `quantize=16`", nhưng
  code hiện tại (Pha B) đạt FP16 bằng cách bật `use_fp16_packed/storage/arithmetic`
  ở runtime trên model FP32 gốc (`best_ncnn_model`), không cần export file
  `.param/.bin` riêng — đây là cách NCNN khuyến nghị cho tăng tốc FP16 trên
  ARMv8.2 (Cortex-A76 hỗ trợ FP16 arithmetic phần cứng). Không cần làm thêm
  bước export, chỉ cần model FP32 gốc là đủ cho cả B1 và B2.

## 12. Troubleshooting Nhanh

| Triệu chứng | Nguyên nhân khả dĩ | Cách xử lý |
|---|---|---|
| `bash: colcon: command not found` (chạy trực tiếp trên Pi, ngoài Docker) | OS host chỉ có ROS2 base, không có `colcon` (chỉ chạy trong container ở production, chưa từng cài native) | Chuyển sang Đường A (Docker, §4/§6/§8), hoặc cài `sudo apt-get install -y python3-colcon-common-extensions` nếu vẫn muốn build native (Đường B) |
| `docker: command not found` hoặc `permission denied` khi gọi `docker` | Docker chưa cài trên Pi, hoặc user hiện tại chưa thuộc group `docker` | Cài Docker nếu thiếu; nếu chỉ thiếu quyền, thêm `sudo` trước lệnh hoặc `sudo usermod -aG docker $USER` rồi đăng nhập lại SSH |
| `docker: Error response from daemon: pull access denied` hoặc `Unable to find image 'avs_perception:arm64' locally` | Image production chưa được `docker load` trên Test Pi này | Build + transfer image theo `skills/docker_SKILL/SKILL.md` §5, hoặc `docker load -i avs_perception_arm64.tar` nếu đã có sẵn tarball trên Pi |
| `No rule to make target '/workspace/config/label_mapping.json'` (Đường A) | Thiếu `-v "$(pwd)/config:/workspace/config"` trong lệnh `docker run` build ở Bước 2 | Thêm mount `config` (đã có sẵn trong guide này) rồi build lại |
| `find_package(ncnn REQUIRED)` fail (Đường B) | Chưa cài NCNN native, hoặc rsync lỡ mang theo `ncnn/` x86_64 | Xóa `~/SimpleSysIDV/ncnn` trên Pi nếu tồn tại, làm lại Bước 0 Đường B |
| `ncnn_inference_node` crash ngay khi start | Model path sai, hoặc link nhầm `libncnn.so` x86_64 | Đường A: `docker run --rm -v "$(pwd)/ros2_ws:/workspace/ros2_ws" avs_perception:arm64 bash -c "ldd /workspace/ros2_ws/install/avs_perception/lib/avs_perception/ncnn_inference_node \| grep ncnn"`. Đường B: đổi `install` thành `install_user`, bỏ phần `docker run` — xác nhận trỏ tới `/usr/lib/libncnn.so*` |
| Benchmark watchdog timeout "Warmup not completed within 90 seconds" | Node chưa kịp load model / camera video chưa publish | Kiểm tra `video_publisher_node` log, tăng `--warmup`/kiểm tra CPU quá tải do process khác |
| `p95` cao bất thường lần đầu, ổn định lần sau | Chưa warmup đủ / governor chưa chuyển `performance` | Lặp lại Bước 3, kiểm tra `cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor` |
| `throttled_after` khác `0x0` trong metadata | Pi quá nhiệt trong lúc benchmark | Loại kết quả run đó, để nguội, thêm tản nhiệt/quạt trước khi đo lại |
| Pi vẫn sập nguồn dù đã `setup 2000` + so le khởi động | Nguồn/cáp không chịu nổi ngay cả dòng đỉnh đã giảm | Chẩn đoán bằng `setup 1500` (chỉ để xác nhận, không lấy số liệu); xem `~/power_log.txt` cột EXT5V_V; đổi cáp 5A rồi đổi adapter (§8.1) |
| `pmic_adc_before/after` = `[]` hoặc `temperature_before/after` = `"N/A"` trong metadata (Đường A) | `vcgencmd` trong container cần `/dev/vchiq` — Test Pi (Ubuntu 22.04) không có node này nên đây là **hành vi kỳ vọng**; theo dõi nhiệt/điện áp bằng `pi_power_guard.sh monitor` trên host thay thế | Nếu Pi chạy RPi OS (có `/dev/vchiq`): thêm `--device /dev/vchiq` vào `docker run` (lưu ý `-v /dev:/dev` KHÔNG đủ — device cgroup chặn `open()`) |
| `docker: ... error gathering device information while adding custom device "/dev/vchiq": no such file or directory` | Host không có `/dev/vchiq` (Ubuntu không expose node vchiq như RPi OS) | Bỏ cờ `--device /dev/vchiq`, giữ `--device /dev/dri`; metadata mất nhiệt độ (N/A) nhưng benchmark chạy bình thường |
| `MESA: error: Opening /dev/dri/renderD128 failed: Operation not permitted` trong container | Dùng `-v /dev:/dev` thay vì `--device /dev/dri` — device cgroup của Docker chặn `open()` device chưa khai báo, kể cả với root | Thay `-v /dev:/dev` bằng `--device /dev/dri` (thêm `--device /dev/vchiq` cho vcgencmd chỉ khi host có node đó) — xem §13.4 |

## 13. Benchmark Vulkan B3/B4 (Pha D) — Image Riêng `avs_perception:arm64-vulkan`

Chạy phần này **sau khi** đã có kết quả B0/B1/B2 (§8) — B3/B4 chỉ có ý nghĩa
khi so được với CPU baseline. Toàn bộ dùng Đường A (Docker); không có đường
native cho Vulkan.

### 13.1 Vì Sao Cần Image Riêng + Điều Kiện Kernel (Gate — Kiểm Tra Trước Tiên)

GPU Pi 5 là VideoCore VII (V3D 7.1). Driver Vulkan userspace cho nó là Mesa
`v3dv`, chỉ hỗ trợ V3D 7.1 từ **Mesa ≥ 23.3**. Test Pi chạy Ubuntu 22.04
(theo `skills/camera_SKILL/SKILL.md`) — Mesa 23.2 của jammy KHÔNG đủ, và
image production (`ros:humble` = jammy) cũng vậy. Giải pháp:
`docker/Dockerfile.vulkan` build image `avs_perception:arm64-vulkan` mang
**Mesa 25.x từ PPA `ppa:kisak/turtle`** (có build arm64 cho jammy) vào trong
container, kèm NCNN build lại với `NCNN_VULKAN=ON`.

Driver userspace nằm trong container, nhưng **kernel driver `v3d` phải có
trên host** — đây là điều kiện duy nhất phía OS của Pi, và là thứ CHƯA được
xác minh trên Test Pi. Kiểm tra trước khi làm bất cứ bước nào khác:

```bash
ssh goln-raspi5@goln-raspi5.local
uname -r          # cần kernel 6.6+ (HWE/raspi) — kernel cũ không có v3d cho Pi 5
ls -la /dev/dri   # cần thấy renderD128 (và card0/card1)
```

- Nếu `/dev/dri` **có `renderD128`** → gate pass, tiếp tục 13.2.
- Nếu `/dev/dri` **không tồn tại/trống** → kernel chưa có driver GPU cho Pi 5;
  Vulkan không thể chạy dù container có driver gì đi nữa. Phương án: nâng
  kernel HWE (`sudo apt install linux-generic-hwe-22.04` rồi reboot, kiểm tra
  lại) hoặc nâng OS lên Ubuntu 24.04 — cả hai đều là thay đổi hệ thống lớn,
  cân nhắc/backup trước, và dừng Pha D lại nếu chưa sẵn sàng.

Hai điểm cần hiểu trước khi đọc số liệu:

- **NCNN trong image Vulkan là commit `b16501a` (master, 2026-06)**, trùng
  `ncnn-src/` trên laptop đang dùng cho Pha D, KHÔNG phải `20240820` của
  production — master có nhiều fix Vulkan mà bản 2024 không có. Đổi lại, so
  sánh Vulkan-vs-CPU phải thực hiện **trong cùng image Vulkan** (13.5) để
  không lẫn confound version.
- Image đã pin `VK_DRIVER_FILES` về ICD `broadcom` (v3dv), nên **không thể
  âm thầm rơi về llvmpipe** (Vulkan software trên CPU) — nếu driver/kernel
  thiếu, mọi thứ fail rõ ràng thay vì cho ra số liệu "Vulkan" giả.

### 13.2 Build Image Vulkan Trên Laptop + Transfer Sang Pi

Giống quy trình `skills/docker_SKILL/SKILL.md` §5.2, chỉ khác Dockerfile và
tên tag/tar:

```bash
cd /home/goln/SimpleSysIDV
sudo docker buildx build \
  --platform linux/arm64 \
  -t avs_perception:arm64-vulkan \
  -f docker/Dockerfile.vulkan \
  -o type=docker,dest=avs_perception_arm64_vulkan.tar .
```

Build lâu hơn image CPU đáng kể (NCNN Vulkan compile thêm glslang + hàng
trăm shader SPIR-V; qua QEMU có thể 30–60 phút). Sau đó transfer + load:

```bash
rsync -avz --progress avs_perception_arm64_vulkan.tar \
  goln-raspi5@goln-raspi5.local:/home/goln-raspi5/SimpleSysIDV/
ssh goln-raspi5@goln-raspi5.local \
  "cd ~/SimpleSysIDV && sudo docker load -i avs_perception_arm64_vulkan.tar"
```

(`avs_perception_arm64_vulkan.tar` ~nửa GB — không commit vào git, giống
`avs_perception_arm64.tar` hiện tại.)

### 13.3 Build ros2_ws Trong Image Vulkan (Layout Riêng `install_vulkan`)

NCNN version khác → **phải rebuild workspace trong image Vulkan**, và build
vào layout riêng để không đè `ros2_ws/install` đã build bằng image CPU (giữ
khả năng chạy lại B0–B2 production mà không rebuild):

```bash
ssh goln-raspi5@goln-raspi5.local
cd ~/SimpleSysIDV
docker run --rm \
  --network host \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/config:/workspace/config" \
  avs_perception:arm64-vulkan \
  bash -c "command -v colcon >/dev/null || (apt-get update && apt-get install -y python3-colcon-common-extensions) && \
           cd /workspace/ros2_ws && sudo rm -rf build_vulkan install_vulkan log_vulkan 2>/dev/null; \
           colcon --log-base log_vulkan build --symlink-install \
             --build-base build_vulkan --install-base install_vulkan \
             --cmake-args -DCMAKE_BUILD_TYPE=Release"
```

NCNN trong image cài ở `/usr` nên `CMakeLists.txt` tự tìm thấy qua fallback
thứ 4 của §3 — không cần set `ncnn_DIR`.

### 13.4 Verify Vulkan Trong Container Trước Khi Benchmark

```bash
docker run --rm --device /dev/dri avs_perception:arm64-vulkan \
  bash -c "vulkaninfo --summary 2>&1 | grep -iE 'deviceName|driverName|apiVersion' || vulkaninfo --summary 2>&1 | tail -5"
```

Phải dùng `--device /dev/dri`, KHÔNG dùng `-v /dev:/dev` — bind-mount chỉ làm
device node hiện ra trong container, còn **device cgroup của Docker vẫn chặn
`open()`** trên node chưa khai báo qua `--device`, gây lỗi
`MESA: error: Opening /dev/dri/renderD128 failed: Operation not permitted`
(đã gặp thực tế 2026-07-08, kể cả khi chạy root trong container).

- Kỳ vọng: `deviceName` chứa **`V3D 7.1`**, `driverName` là `V3DV Mesa`.
- Nếu báo không tìm thấy GPU → quay lại gate 13.1 (kernel/`/dev/dri`), hoặc
  thiếu `--device /dev/dri`.
- KHÔNG được thấy `llvmpipe` — image đã pin ICD broadcom; nếu vẫn thấy
  llvmpipe nghĩa là image build sai (kiểm tra `VK_DRIVER_FILES` trong
  container: `docker run --rm avs_perception:arm64-vulkan env | grep VK_`).

### 13.5 Chạy B3/B4 + Re-run B1/B2 Cùng Image (Chống Confound Version)

Trước tiên **rsync lại code sang Pi** (chạy lại đúng lệnh rsync Bước 1 §5 —
`tools/optimize/**` nằm trong allowlist): suite/harness có các thay đổi dành
riêng cho Vulkan (guard chống fallback CPU, warmup timeout dài hơn cho lần
compile shader đầu) — chạy bản cũ trên Pi sẽ fail watchdog 90s một cách khó
hiểu.

Lệnh giống §8 Đường A, chỉ đổi **image** và **`ROS2_INSTALL_DIR`**. Vẫn giữ
đủ kỷ luật power của §8.1: `pi_power_guard.sh setup` + `monitor` chạy trước
(GPU V3D ăn dòng trên rail riêng, KHÔNG bị cap CPU giới hạn → tổng dòng có
thể cao hơn các run CPU — theo dõi `EXT5V_V` sát hơn), từng preset một lệnh
riêng, xác nhận Pi còn sống giữa các lần:

```bash
cd ~/SimpleSysIDV
# Chạy lần lượt: b3, b4, rồi b1, b2 (CPU cùng version NCNN để so sánh)
docker run --rm \
  --network host \
  -v "$(pwd)/ros2_ws:/workspace/ros2_ws" \
  -v "$(pwd)/models:/workspace/models" \
  -v "$(pwd)/config:/workspace/config" \
  -v "$(pwd)/test:/workspace/test" \
  -v "$(pwd)/tools:/workspace/tools" \
  -v "$(pwd)/docs/optimize/results:/workspace/docs/optimize/results" \
  --device /dev/dri \
  -e ROS2_INSTALL_DIR=install_vulkan \
  avs_perception:arm64-vulkan \
  bash -c "cd /workspace && ./tools/optimize/run_benchmark_suite.sh b3"
```

- Thứ tự khuyến nghị: `b3` (FP32 Vulkan) → `b4` (FP16 Vulkan) → `b1`, `b2`
  (FP32/FP16 CPU **trong image Vulkan**). Bốn run này cùng NCNN `b16501a`
  nên so trực tiếp được với nhau; còn cặp B1/B2 mới vs B1/B2 cũ (20240820,
  §8) cho biết bản thân version NCNN làm CPU nhanh/chậm đi bao nhiêu.
- Suite có guard tự động cho preset Vulkan: abort nếu NCNN không có Vulkan
  (thay vì ghi số liệu CPU dán nhãn Vulkan), và in các dòng device từ log
  node sau mỗi run — **phải thấy `V3D`**, không phải `llvmpipe`.
- `run_benchmark.sh` ghi `vulkan_available: "yes"` trong metadata khi
  `vulkaninfo` chạy được trong container — check nhanh tính hợp lệ của run.

### 13.6 Đánh Giá — Gate Pha D

Kéo kết quả về laptop và tổng hợp như §9/§10. Theo
`ncnn_inference_latency_plan.md` §6.4:

- Node phải start thành công với Vulkan và chạy đủ 300 frame telemetry.
- **Không chọn Vulkan** nếu `output_age_ms`, p95 latency hoặc jitter xấu hơn
  CPU baseline — kể cả khi avg FPS cao hơn.
- So thêm chất lượng (`main_lane_missing_rate`, `turn_lane_present_rate`,
  `qual_*_poly_area`) giữa Vulkan và CPU cùng image: v3dv là driver trẻ,
  nếu mask/polygon lệch rõ so với CPU thì loại Vulkan bất kể tốc độ.
- Kết quả "Vulkan chậm hơn CPU" là kết quả hợp lệ và đã được plan lường
  trước (§3: "không được giả định chắc chắn nhanh hơn CPU") — ghi nhận vào
  bảng §7 của plan rồi đóng Pha D.

### 13.7 Troubleshooting Riêng Vulkan

| Triệu chứng | Nguyên nhân khả dĩ | Cách xử lý |
|---|---|---|
| `add-apt-repository` fail hoặc apt không tìm thấy `mesa-vulkan-drivers` mới khi build image | PPA `ppa:kisak/turtle` đổi tên/ngừng jammy sau này | Kiểm tra https://launchpad.net/~kisak/+archive/ubuntu/turtle còn publish jammy arm64 không; nếu không còn, cần build Mesa v3dv từ source trong Dockerfile (chưa có sẵn hướng dẫn — ghi nhận lại để làm) |
| `vulkaninfo` trong container: không thấy GPU / `ERROR_INITIALIZATION_FAILED` | Kernel host thiếu driver `v3d` (gate 13.1), thiếu `--device /dev/dri`, hoặc dùng `-v /dev:/dev` thay cho `--device` (bị device cgroup chặn — lỗi kèm `Operation not permitted`) | Kiểm tra `ls /dev/dri` trên host; dùng đúng `--device /dev/dri` (§13.4); nâng kernel HWE 6.6+/OS 24.04 nếu host thiếu `/dev/dri` |
| Suite abort: `NCNN here has no Vulkan support` | Chạy nhầm image CPU (`avs_perception:arm64`), hoặc `ROS2_INSTALL_DIR` trỏ về `install` build bằng image CPU | Dùng image `arm64-vulkan` + `ROS2_INSTALL_DIR=install_vulkan` (13.3/13.5) |
| Log node in `llvmpipe` thay vì `V3D` | ICD pin bị ghi đè (`VK_DRIVER_FILES`/`VK_ICD_FILENAMES` bị unset khi chạy) | Không override 2 biến env đó trong `docker run`; kiểm tra `docker run --rm avs_perception:arm64-vulkan env \| grep VK_` |
| `Watchdog timeout: Warmup not completed` ở b3/b4 dù node đã in `[0 V3D 7.1...]` (GPU nhận đúng) | Lần inference Vulkan đầu tiên phải compile SPIR-V pipeline cho V3D — mất vài phút, vượt watchdog 90s mặc định của CPU; HOẶC bản suite trên Pi là bản cũ chưa có warmup timeout riêng cho Vulkan | Rsync lại `tools/optimize` (Bước 1) — suite mới tự đặt 420s cho preset Vulkan (`BENCH_WARMUP_TIMEOUT` để chỉnh); nếu vẫn timeout ở 420s, kiểm tra GPU hang: `dmesg \| grep -iE 'v3d\|gpu'` trên host — thấy reset/hang → v3dv không kham nổi compute của model này |
| Node crash/segfault khi bật Vulkan (b3/b4) nhưng b1/b2 cùng image chạy tốt | v3dv chưa đủ hoàn thiện cho compute path của NCNN trên model này | Ghi nhận log crash, thử `b3` trước `b4`; nếu cả hai crash → kết luận Vulkan chưa khả thi ở version driver/NCNN này, đóng Pha D với kết quả âm |
| Vulkan chạy được nhưng chậm hơn CPU nhiều | Bình thường với GPU nhỏ + overhead upload/download mỗi frame ở input 320 | Đây là kết quả hợp lệ để kết luận không dùng Vulkan (13.6) |
| Pi sập nguồn trong run Vulkan dù CPU runs ổn | GPU thêm tải dòng trên rail riêng, không bị cap CPU giới hạn | Xem `~/power_log.txt` (EXT5V_V); xử lý phần cứng theo §8.1 — cap CPU không đủ khi GPU cùng chạy |
