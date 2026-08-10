# 🚢 Skill: Docker Deployment & Performance

## 1. Architecture Overview

The AVS system runs as **3 Docker containers** sharing the same base image (`avs_perception`):

| Container | Role | Key Dependencies |
|---|---|---|
| `avs_perception_container` | Builds ROS2 C++ nodes (NCNN inference + video publisher), runs `ncnn_inference_node` | NCNN, OpenCV, cv_bridge |
| `video_publisher_container` | Waits for build completion, runs `video_publisher_node` (camera/video input) | OpenCV V4L2, cv_bridge |
| `web_dashboard_container` | Runs FastAPI web server + ROS2 bridge for UI | FastAPI, Uvicorn, rclpy |

All containers use `network_mode: host` for zero-config ROS2 DDS discovery.

---

## 2. Docker Image

### 2.1 Base Image
```dockerfile
FROM ros:humble-ros-base
```

### 2.2 Key Installed Dependencies
- **Build tools:** `build-essential`, `cmake`, `git`, `pkg-config`
- **Vision:** `libopencv-dev`, `python3-opencv`, `ros-humble-cv-bridge`, `ros-humble-image-transport`
- **AI Runtime:** NCNN (compiled from source, branch `20240820`, installed to `/usr`)
- **Web:** `fastapi`, `uvicorn`, `websockets` (via pip3)
- **NCNN build flags:**
  - `NCNN_VULKAN=OFF` (CPU-only, no GPU)
  - `NCNN_SHARED_LIB=ON`
  - `NCNN_ARM_NEON=ON` (ARM SIMD optimization for Pi 5)
  - `CMAKE_INSTALL_PREFIX=/usr` (system-wide installation)

### 2.3 Entrypoint
`docker/entrypoint.sh` only sources ROS2 Humble base:
```bash
source "/opt/ros/humble/setup.bash"
exec "$@"
```
> **IMPORTANT:** Do NOT source workspace `install/setup.bash` in the entrypoint. Each container manages its own workspace sourcing via its `command`. This prevents race conditions where one container deletes `install/` while another tries to source it.

---

## 3. Compose Files

### 3.1 Development (`docker-compose.yml`)
- Used on the **Acer Nitro 5 laptop** for local testing.
- Contains `build:` directives to build the image from `docker/Dockerfile`.
- Image tag: `avs_perception:latest` (x86_64 native).

### 3.2 Production (`docker-compose.prod.yml`)
- Used on **Raspberry Pi 5** for deployment.
- **No `build:` directives** — uses pre-built ARM64 image only.
- Image tag: `avs_perception:arm64`.

---

## 4. Container Orchestration Details

### 4.1 Build & Start Sequence (Race Condition Prevention)
The `avs_perception_container` compiles ROS2 C++ code at startup:
```bash
cd /workspace/ros2_ws && rm -rf build install log && colcon build --symlink-install && source install/setup.bash && ros2 run avs_perception ncnn_inference_node
```

The `video_publisher_container` uses a **two-phase wait** to prevent race conditions:
```bash
# Phase 1: Wait for old build artifacts to be deleted
while [ -f .../video_publisher_node ]; do sleep 1; done
# Phase 2: Wait for fresh build to complete
while [ ! -f .../video_publisher_node ]; do sleep 2; done
# Phase 3: Source and run
source install/setup.bash && ros2 run avs_perception video_publisher_node
```

> **WARNING:** Without Phase 1, the video_publisher would find the stale executable from a previous run and start immediately, only to crash when `avs_perception_container` deletes `install/` moments later.

### 4.2 Camera Device Access
```yaml
volumes:
  - /dev:/dev  # Mount entire /dev for camera access
```
> **CRITICAL:** Do NOT use `devices:` for camera mapping. Docker's `devices:` directive creates a static device node at container start time. If the camera is reconnected or udev symlinks change, the container won't see the update. Instead, mount `/dev` directly (requires `privileged: true`).

### 4.3 Volume Mounts
| Host Path | Container Path | Purpose |
|---|---|---|
| `./ros2_ws` | `/workspace/ros2_ws` | ROS2 source code (compiled at runtime) |
| `./models` | `/workspace/models` | NCNN model files (.param + .bin) |
| `./test` | `/workspace/test` | Test video files |
| `./config` | `/workspace/config` | Runtime configuration (config.json) |
| `./web_dashboard` | `/workspace/web_dashboard` | Frontend + backend code |
| `/dev` | `/dev` | Camera USB devices (video_publisher only) |

---

## 5. Cross-Compilation & Deployment to Raspberry Pi 5

### 5.1 Prerequisites (Laptop - one-time setup)
Install QEMU emulator for ARM64 cross-compilation:
```bash
sudo apt-get update
sudo apt-get install -y qemu-user-static binfmt-support
```

### 5.2 Build ARM64 Image on Laptop
```bash
cd /home/goln/SimpleSysIDV
sudo docker buildx build \
  --platform linux/arm64 \
  -t avs_perception:arm64 \
  -f docker/Dockerfile \
  -o type=docker,dest=avs_perception_arm64.tar .
```
> **NOTE:** Build takes ~12 minutes via QEMU emulation. The warning `current commit information was not captured` is harmless (caused by sudo + git ownership mismatch).

### 5.3 Transfer to Raspberry Pi 5
Sync source code + image to Pi (exclude heavy development artifacts):
```bash
rsync -avz \
  --exclude '.venv' \
  --exclude 'build' \
  --exclude 'install' \
  --exclude 'log' \
  --exclude '.git' \
  --exclude 'ncnn-src' \
  --exclude 'ncnn' \
  /home/goln/SimpleSysIDV/ \
  <USER>@<HOSTNAME>.local:/home/<USER>/SimpleSysIDV/
```

**Target hosts:**
| Unit | Command |
|---|---|
| Test Pi | `goln-raspi5@goln-raspi5.local:/home/goln-raspi5/SimpleSysIDV/` |
| Prod Pi | `pi@raspi5.local:/home/pi/SimpleSysIDV/` |

### 5.4 Load Image on Raspberry Pi 5
```bash
ssh <USER>@<HOSTNAME>.local
cd ~/SimpleSysIDV
sudo docker load -i avs_perception_arm64.tar
```
Verify: `sudo docker images` should show `avs_perception:arm64`.

### 5.5 Camera udev Setup on Pi 5
Create stable device symlink:
```bash
sudo nano /etc/udev/rules.d/99-usb-camera.rules
```
```text
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", ATTR{index}=="0", SYMLINK+="video_source"
```

> **CRITICAL:** `ATTR{index}=="0"` is mandatory. USB cameras create multiple `/dev/video*` entries — index 0 is the capture device, index 1+ are metadata devices that cannot be opened for video capture.

**Known camera configurations:**
| Unit | Camera | Vendor:Product | V4L2 Settings |
|---|---|---|---|
| Test Pi | HBV HD CAMERA | `0ac8:0346` | MJPEG 640x480 @ 30fps |
| Prod Pi | High-speed camera | TBD | MJPEG up to 120fps |

Activate and verify:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/video_source  # Must show -> /dev/video0 (NOT video1)
```

### 5.6 Start the System
```bash
cd ~/SimpleSysIDV
sudo docker compose -f docker-compose.prod.yml up -d
```

### 5.7 Access Dashboard
From laptop browser (Firefox): `http://<HOSTNAME>.local:8000`

---

## 6. Operational Commands

| Action | Command |
|---|---|
| Start (background) | `sudo docker compose -f docker-compose.prod.yml up -d` |
| Stop | `sudo docker compose -f docker-compose.prod.yml down` |
| View all logs | `sudo docker compose -f docker-compose.prod.yml logs -f` |
| View specific logs | `sudo docker compose -f docker-compose.prod.yml logs -f video_publisher` |
| Check status | `sudo docker compose -f docker-compose.prod.yml ps` |
| Restart after code change | `down` → sync new code → `up -d` (no rebuild needed) |
| Full rebuild on laptop | `sudo docker compose build` |

---

## 7. Troubleshooting & Known Issues

### 7.1 `version` attribute warning
```
WARN: the attribute `version` is obsolete
```
Harmless. Docker Compose v2 ignores this field.

### 7.2 `Published ports are discarded when using host network mode`
Expected behavior. `network_mode: host` shares the host's network stack directly, making explicit port mappings unnecessary (but they don't cause errors).

### 7.3 Camera fails to open (`V4L2 failed to open camera`)
**Symptoms:** Switching to camera mode on the web UI shows no video.
**Diagnosis:** Check logs: `sudo docker compose -f docker-compose.prod.yml logs -f video_publisher`

| Log Message | Cause | Fix |
|---|---|---|
| `can't open camera by index` | udev symlink points to metadata device (e.g., `/dev/video1`) | Add `ATTR{index}=="0"` to udev rule |
| `V4L2 failed to open camera: /dev/video_source` | Symlink doesn't exist inside container | Use `volumes: - /dev:/dev` instead of `devices:` |
| `No such file or directory: /dev/video_source` | udev rule not configured or camera unplugged | Run `sudo udevadm trigger` and verify `ls -l /dev/video_source` |

### 7.4 Build cache conflicts (CMake path errors)
**Cause:** Host build artifacts (x86_64) conflict with container builds (ARM64) because `ros2_ws/` is bind-mounted.
**Fix:** The `avs_perception_container` command runs `rm -rf build install log` before `colcon build` to ensure a clean build every startup.

### 7.5 Race condition: `FileNotFoundError: install/`
**Cause:** `video_publisher_container` or `web_dashboard_container` tries to source `install/setup.bash` while `avs_perception_container` is deleting it.
**Fix:**
- `video_publisher`: Two-phase wait loop (see §4.1).
- `web_dashboard`: Does not source workspace — runs FastAPI directly.
- `entrypoint.sh`: Only sources ROS2 base, never workspace.

### 7.6 Camera V4L2 format configuration
The `video_publisher_node` forces V4L2 capture settings when opening camera devices:
```cpp
cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);   // from config camera_width
cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);  // from config camera_height
cap.set(cv::CAP_PROP_FPS, 30);            // from config camera_fps
```
These values are configurable via `config/config.json` and correspond to the V4L2 command:
```bash
ffplay -f video4linux2 -input_format mjpeg -video_size 640x480 -framerate 30 -i /dev/video0
```

---

## 8. Config Reference (`config/config.json`)

```json
{
  "mode": "video",
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

| Field | Description |
|---|---|
| `mode` | `"camera"` for real-time USB camera, `"video"` for test simulation |
| `camera_device` | V4L2 device path (use udev symlink `/dev/video_source`) |
| `video_path` | Path to test video file inside container |
| `camera_width/height/fps` | V4L2 capture format (MJPEG enforced automatically) |
| `prob_threshold` / `nms_threshold` | YOLO inference thresholds (adjustable via web UI) |