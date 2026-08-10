#  Skill: High-Speed Real-time Data Transport

**Trạng thái: đề xuất/roadmap.** Chưa tìm thấy cấu hình Zenoh/CycloneDDS/`ROS_DOMAIN_ID` multi-machine thật nào trong repo hiện tại — deployment production chạy single-machine trên Pi 5 (`network_mode: host` trong `docker-compose.prod.yml`). Nội dung dưới đây là hướng kỹ thuật đề xuất cho khi cần setup phân tán Pi-to-laptop, không phải mô tả hệ thống đang chạy.

## 1. Middleware Architecture (Distributed ROS2)
Utilize the native distributed nature of **ROS2 Humble** to connect Pi 5 (Publisher) and Laptop (Subscriber).
- **DDS Implementation:** Use **FastDDS** or **CycloneDDS** for reliable UDP-based communication.
- **Zenoh Bridge (Optional):** For ultra-low latency and unstable Wi-Fi, integrate `zenoh-bridge-ros2` to optimize bandwidth by up to 10x compared to standard DDS.

## 2. Bandwidth Optimization (Image Transport)
Avoid sending raw frames (e.g., 1280x720 YUYV) over Wi-Fi as it will cause 1-2s lag.
- **Compressed Topics:** Use `image_transport` with **CompressedImage** (JPEG/PNG) or **Theora** plugins.
- **Sub-sampling:** Send raw inference results (JSON/Protobuf) as small text messages, and only stream video at 15-20 FPS for monitoring.

## 3. Tooling for Laptop Display
- **Foxglove Bridge:** The most efficient way to stream data to a remote UI. It uses WebSockets to bundle ROS2 topics.
- **ROS2 Multi-machine Setup:** Set `ROS_DOMAIN_ID` to the same value on both devices to allow seamless "Plug-and-Play" discovery between the Pi 5 and Acer Nitro 5.

## 4. Success Criteria (Karpathy Style)
- [ ] Latency between Pi 5 detection and Laptop display < 100ms.
- [ ] Zero packet loss for critical steering commands sent to ESP32.