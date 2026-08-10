# SimpleRobot Project Overview

## 1. Mục đích
Tài liệu này tóm tắt toàn bộ nội dung hiện có của dự án `SimpleRobot` và giải thích các thành phần chính đã được xác định trong kho.

## 2. Cấu trúc thư mục hiện tại
- `.git/` — Kho Git, không chứa mã nguồn.
- `.gitignore` — Cấu hình Git.
- `docker-compose.yml` — hiện đang rỗng.
- `README.md` — chỉ chứa tiêu đề `# SimpleRobot`.
- `GEMINI.md` — tài liệu chính mô tả toàn bộ hệ thống.
- `skills/` — chứa các hướng dẫn chuyên môn theo từng mảng chức năng.

## 3. Nội dung chính của dự án
### 3.1 `GEMINI.md`
Đây là file quan trọng nhất trong kho. Nội dung chính gồm:
- Mục tiêu hệ thống: hệ thống thị giác xe tự hành thời gian thực.
- Công nghệ chính: ROS2 Humble, Docker, YOLO11n-seg hoặc DSUnet.
- Mục tiêu xử lý: phân đoạn làn đường và phương tiện.
- Ngôn ngữ sử dụng: Python 3.10, C++17, C/C++ cho ESP32.
- Phần cứng: máy phát triển Ubuntu + laptop, triển khai Raspberry Pi 5 và ESP32.
- Nguyên tắc phát triển: udev mapping, Docker orchestration, coding theo Karpathy, ROS2 phân tán, giám sát Foxglove Studio.
- Lớp nhận diện: `main-lane`, `other-lane`, `solid-white`, `solid-yellow`, `dashed-white`, `dashed-yellow`, `vehicle`.
- Roadmap: đã xong phần setup/hardware/docker, chưa xong phần ra quyết định điều khiển (`Twist message generation`).

### 3.2 `skills/`
Folder này chứa các hướng dẫn theo từng mảng kỹ năng:
- `camera_SKILL/SKILL.md`
  - Mô tả hệ thống thị giác, ROS2, Docker và YOLO11n-seg.
- `docker_SKILL/SKILL.md`
  - Hướng dẫn xây dựng cross-platform với `docker buildx`.
  - Định hướng export mô hình sang NCNN/ONNX.
  - Khuyến nghị ánh xạ thiết bị như `/dev/ai_camera -> /dev/video_source` và `/dev/dri -> /dev/dri`.
- `data_transport_SKILL/SKILL.md`
  - Hướng dẫn ROS2 phân tán giữa Raspberry Pi và laptop.
  - Khuyến nghị sử dụng FastDDS/CycloneDDS, `image_transport` với CompressedImage, và Foxglove Bridge.
  - Giữ `ROS_DOMAIN_ID` cùng một giá trị giữa các thiết bị.
- `karpathy-guidelines/SKILL.md`
  - Nguyên tắc lập trình: nghĩ trước, giữ code đơn giản, thay đổi cẩn thận, và đặt ra tiêu chí rõ ràng.

## 4. Nhận xét quan trọng
- Kho hiện tại không chứa mã nguồn ROS2, không có `Dockerfile`, và `docker-compose.yml` đang trống.
- Dự án là một bản kế hoạch, không phải một ứng dụng đã hoàn thiện.
- Để tiến triển, cần bổ sung:
  - mã ROS2 cho camera/inference và publish topic,
  - Dockerfile và cấu hình Docker Compose,
  - firmware micro-ROS cho ESP32,
  - logic ra quyết định điều khiển.

## 5. Hướng dẫn tiếp theo đề xuất
### 5.1 Nếu bạn muốn triển khai thêm
- Tạo `Dockerfile` cho ROS2 Humble với Python, OpenCV, ROS2 packages.
- Viết node ROS2 Python/C++ đọc camera và chạy inference segmentation.
- Viết node ROS2 publish kết quả segmentation và kế hoạch điều khiển `Twist`.
- Cấu hình ROS2 multi-machine giữa Pi và laptop.
- Viết firmware micro-ROS cho ESP32 nhận lệnh điều khiển.

### 5.2 Nếu bạn muốn tôi hỗ trợ tiếp
Tôi có thể giúp bạn:
- tạo cấu trúc thư mục mẫu cho mã nguồn;
- viết `Dockerfile` và `docker-compose.yml` mẫu;
- lập trình các node ROS2 Python/C++;
- hướng dẫn cấu hình ROS2 mạng và Foxglove;
- xây dựng luồng inference lane/vehicle segmentation.

## 6. Kết luận
`SimpleRobot` hiện tại là bản thiết kế và tài liệu hướng dẫn. Để biến nó thành dự án thực tế, cần bổ sung mã nguồn và cấu hình triển khai.
