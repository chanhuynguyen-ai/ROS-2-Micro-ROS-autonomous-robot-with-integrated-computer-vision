#  Project: AI Autonomous Vision System (AVS)

## 1. System Context
A real-time autonomous vehicle vision system utilizing **ROS2 Humble**, **Docker**, and **YOLO11n-seg** or **DSUnet**. It focuses on lane and vehicle segmentation for high-speed decision making, optimized specifically for CPU-only execution on the Raspberry Pi 5.

## 2. Software Stack & Programming Languages

- **Primary Languages:**
  - **Python 3.10:** Main language for ROS2 high-level nodes, OpenCV image processing, and AI segmentation logic.
  - **C++ (C++17):** Used for performance-critical tasks, specifically NCNN inference and high-speed ROS2 nodes to prevent memory fragmentation on ARM64.
  - **C/C++:** For firmware development on ESP32 via the micro-ROS framework.
- **Middleware:** ROS2 (Humble Hawksbill).
- **AI Frameworks:** Ultralytics (YOLO26), DSUnet (pytorch), NCNN (Optimized for ARM CPU via NEON/FP16 assembly).
- **Embedded Integration:** micro-ROS (ESP32).
- **Containerization:** Docker & Docker Compose.

## 3. Hardware Environment
### Build Host (Development)
- **Model:** Acer Nitro 5 AN515-57
- **Spec:** 11th Gen Intel i5-11400H, 16GB RAM, RTX 3050 (4GB VRAM).
- **OS:** Ubuntu 22.04.5 LTS (GNOME 42.9).

### Deployment Target (Edge) — Dual Raspberry Pi 5 Setup
Two Raspberry Pi 5 units are used: one for development testing and one for production deployment.

#### Test Unit (Development Testing)
- **Model:** Raspberry Pi 5 (4GB RAM) + Active Cooler.
- **Storage:** 64GB MicroSD (OS via Raspberry Pi Imager).
- **Network:** Hostname: `goln-raspi5.local`, User: `goln-raspi5`.
- **Camera:** HBV HD CAMERA (USB, Vendor: `0ac8`, Product: `0346`), V4L2 MJPEG 640x480 @ 30fps.
- **Controllers:** ESP32 via **micro-ROS** for low-level actuation.

#### Production Unit (Deployment)
- **Model:** Raspberry Pi 5 (4GB RAM) + Active Cooler.
- **Network:** Hostname: `raspi5.local`, User: `pi`.
- **Camera:** High-speed USB camera (up to 120fps capable), V4L2 MJPEG.
- **Controllers:** ESP32 via **micro-ROS** for low-level actuation.

## 4. Development & Operation Principles
To ensure system stability and maintainability, the following guidelines are enforced:
- **Hardware Skills:** Device identification and udev mapping (See `skills/camera_SKILL/SKILL.md`).
- **Deployment Skills:** Performance-driven Docker orchestration (See `skills/docker_SKILL/SKILL.md`).
- **Coding Standards:** Surgical changes and simplicity-first approach based on Karpathy Guidelines (See `skills/karpathy-guidelines/SKILL.md`).
- **Communication Skill:** Distributed ROS2 setup for Pi-to-Laptop streaming (See `skills/data_transport_SKILL/SKILL.md`).
- **Labeling Guidelines:** Lane and marking annotation rules for stable control transitions (See `skills/labeling_SKILL/SKILL.md`).
- **UI Standard:** Web dashboard UI/UX running on localhost via Firefox browser (optimized for Firefox).
- **CPU-Centric Design:** Minimize overhead by keeping the entire vision pipeline (decode, resize, normalize, inference) on the CPU. Do not offload preprocessing to the GPU to avoid API call latency and CPU-GPU memory copy overhead.

## 5. Vision & Perception Logic
### Segmentation Classes:
- **Lanes:** `main-lane` (ego), `other-lane` (adjacent), `turn-lane` (intersection turn).
- **Markings:** `solid-white`, `solid-yellow`, `dashed-white`, `dashed-yellow`, `stop-line` (intersection stop line).
- **Parking:** `parking-slot` (oriented parking slot).
- **Objects:** `vehicle` (all detectable traffic).

### CPU Pipeline Optimization Strategy:
- **Optimized Decoding:** Use CPU-accelerated decoders (`libjpeg-turbo` for MJPEG USB camera streams) to convert compressed frames to BGR/RGB.
- **ARM NEON Preprocessing:** Utilize OpenCV's native ARM NEON SIMD vectorization for fast image resizing and color-space conversions directly on the CPU (under 2ms for 720p/480p).
- **NCNN FP16/INT8 Quantization:** Run model inference using NCNN optimized C++ codes with FP16 precision, utilizing all 4 physical Cortex-A76 cores of the Pi 5. If further latency reduction and higher FPS are needed, utilize INT8 quantization (post-training quantization) with calibration to keep accuracy loss minimal.
- **Thermal & Power Efficiency:** Keeping the VideoCore VII GPU idle lowers power draw and limits heat generation, preventing thermal throttling of the CPU.

## 6. System Roadmap
- [x] Hardware identification & udev setup.
- [x] Dockerization & Inference optimization.
- [ ] Decision-making logic (Twist message generation).