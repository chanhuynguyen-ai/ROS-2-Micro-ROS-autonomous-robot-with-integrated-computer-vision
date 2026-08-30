<div align="center">

# 🤖 ROS 2 Autonomous Vision Robot

### Autonomous Robot with ROS 2, Computer Vision, NCNN and micro-ROS

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)
![C++](https://img.shields.io/badge/C++-17-00599C?logo=cplusplus)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-5C3EE8?logo=opencv)
![Docker](https://img.shields.io/badge/Docker-Containerized-2496ED?logo=docker)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi%205-Edge%20AI-A22846?logo=raspberrypi)

**A ROS 2 autonomous robotics platform integrating real-time computer vision, edge AI inference, control algorithms and embedded hardware.**

</div>

---

## 🚀 Overview

This project develops an autonomous mobile robot using **ROS 2 Humble**, **OpenCV**, **NCNN**, **Raspberry Pi 5**, and **ESP32 / micro-ROS**.

The system processes camera input, performs road and lane segmentation, estimates navigation information, applies autonomous control algorithms, and sends motion commands toward the embedded controller.

The main goal is to build a complete robotics pipeline rather than a standalone AI model:

```text
Camera
  ↓
Computer Vision
  ↓
NCNN Segmentation
  ↓
Lane / Road Understanding
  ↓
IPM + Navigation Error
  ↓
Controller
  ↓
ROS 2 Motion Command
  ↓
micro-ROS
  ↓
ESP32
  ↓
Motor / Steering
```

---

## ✨ Key Features

* ROS 2 Humble modular architecture
* Real-time camera and video processing
* Lane and road segmentation
* NCNN CPU inference
* OpenCV image processing
* Inverse Perspective Mapping
* Multiple autonomous control algorithms
* Raspberry Pi 5 edge deployment
* ESP32 / micro-ROS architecture
* Docker-based development environment

---

## 🏗️ Architecture

```mermaid
flowchart LR

CAM["📷 Camera"]
AI["🧠 NCNN Vision"]
IPM["IPM"]
CTRL["🎛️ Controller"]
ROS["ROS 2 Command"]
MCU["ESP32"]
MOTOR["⚙️ Motor"]

CAM --> AI
AI --> IPM
IPM --> CTRL
CTRL --> ROS
ROS --> MCU
MCU --> MOTOR
```

---

## 👁️ Computer Vision

The perception system is designed for navigation-oriented road understanding.

Supported concepts include:

```text
main-lane
other-lane
turn-lane
solid-white
solid-yellow
dashed-white
dashed-yellow
stop-line
parking-slot
vehicle
```

The vision pipeline uses:

* OpenCV preprocessing
* NCNN neural-network inference
* segmentation masks
* lane extraction
* bird's-eye / IPM transformation
* navigation error estimation

---

## 🧩 ROS 2 Packages

```text
ros2_ws/src/
├── avs_perception
├── avs_controlsystem
├── avs_cascadecontrol
├── avs_hybridcontrol
├── avs_pdbackstepingcontrol
└── yahboomcar_description
```

Main perception executables:

```text
video_publisher_node
ncnn_inference_node
ipm_transform_node
control_node
video_test_node
```

---

## 🎛️ Control Algorithms

The project includes multiple experimental autonomous-control approaches:

* Cascade Control
* Hybrid Control
* PD / Backstepping Control
* General control-system implementation

These controllers can be evaluated using the same perception pipeline.

---

## 🛠️ Tech Stack

| Area            | Technology       |
| --------------- | ---------------- |
| Robotics        | ROS 2 Humble     |
| Computer Vision | OpenCV           |
| AI Inference    | NCNN             |
| Languages       | C++17, Python    |
| Edge Computer   | Raspberry Pi 5   |
| Embedded        | ESP32, micro-ROS |
| Deployment      | Docker           |
| Build           | CMake, Colcon    |

---

## 🚀 Quick Start

Clone the repository:

```bash
git clone https://github.com/chanhuynguyen-ai/ROS-2-Micro-ROS-autonomous-robot-with-integrated-computer-vision.git
cd ROS-2-Micro-ROS-autonomous-robot-with-integrated-computer-vision
```

### Build with ROS 2

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

### Run perception nodes

```bash
ros2 run avs_perception video_publisher_node
ros2 run avs_perception ncnn_inference_node
ros2 run avs_perception ipm_transform_node
ros2 run avs_perception control_node
```

---

## 🐳 Docker

```bash
docker compose build
docker compose up
```

---

## 📈 Project Status

| Module                        | Status         |
| ----------------------------- | -------------- |
| ROS 2 architecture            | ✅              |
| Camera pipeline               | ✅              |
| NCNN inference                | ✅              |
| IPM                           | ✅              |
| Control algorithms            | ✅ Experimental |
| Docker environment            | ✅              |
| Raspberry Pi optimization     | 🚧             |
| micro-ROS / ESP32 integration | 🚧             |
| Full hardware autonomous demo | 📌 Planned     |

---

## 🗺️ Roadmap

* [x] ROS 2 perception pipeline
* [x] NCNN integration
* [x] Lane / road segmentation
* [x] IPM processing
* [x] Multiple control algorithms
* [ ] Finalize production controller
* [ ] Complete micro-ROS integration
* [ ] Benchmark Raspberry Pi 5 performance
* [ ] Add complete autonomous-driving demo
* [ ] Add automated ROS 2 tests

---

## 🎯 Engineering Focus

This project demonstrates the integration of:

**Computer Vision + Edge AI + ROS 2 + Control Systems + Embedded Robotics**

rather than treating each subsystem independently.

The long-term objective is a practical autonomous robot capable of processing visual information and converting it into real-time physical motion.

---

## 👨‍💻 Author

**Nguyen Chan Huy**

Robotics & Artificial Intelligence Engineering

Focus:

`Computer Vision` · `AI Engineering` · `ROS 2` · `Autonomous Robotics`

GitHub: [@chanhuynguyen-ai](https://github.com/chanhuynguyen-ai)

---

<div align="center">

### ⭐ If you find this project useful, consider giving it a star.

**ROS 2 · Computer Vision · Edge AI · Autonomous Robotics**

</div>
