# Báo cáo Chi tiết: Các Kỹ thuật Tối ưu hóa Hệ thống (AVS)

Báo cáo này tập trung phân tích chi tiết **Phần III, Mục 9: Các kỹ thuật tối ưu hóa hệ thống** dựa trên mã nguồn thực tế và tệp tin cấu hình dự án AVS. Tài liệu này hệ thống hóa các giải pháp tối ưu hóa từ tầng phần cứng (CPU ARM64), tầng trình biên dịch (GCC flags), tầng suy luận AI (NCNN FP16), tầng lập trình đa luồng (Zero-lag Buffer), cho đến tầng truyền dẫn mạng (DDS và ảnh nén JPEG).

---

## 1. Triết lý Tối ưu hóa CPU-Centric trong AVS

Để hệ thống AVS có thể chạy ổn định ở tốc độ thời gian thực (đạt mốc $30\text{ fps}$) trên máy tính nhúng Raspberry Pi 5 mà không bị suy giảm hiệu năng do quá nhiệt (thermal throttling), hệ thống áp dụng triết lý **CPU-Centric Design**:
- Giữ toàn bộ quy trình từ khâu giải mã ảnh, tiền xử lý, chạy mô hình AI cho đến tính toán hình học thực địa trên các nhân CPU của Pi 5.
- Không chuyển đổi luồng dữ liệu sang GPU VideoCore VII để tránh độ trễ sao chép dữ liệu giữa RAM CPU và VRAM GPU qua các API đồ họa, đồng thời giữ GPU chạy ở chế độ nhàn rỗi (idle) để giảm công suất tiêu thụ và tỏa nhiệt của bo mạch nhúng.

---

## 2. Tối ưu hóa ở Tầng Trình Biên dịch (Compiler Optimization)

Tệp tin cấu hình biên dịch `ros2_ws/src/avs_perception/CMakeLists.txt` tự động nhận diện kiến trúc CPU để áp dụng các cờ tối ưu hóa chuyên sâu nhất của GCC trong chế độ biên dịch `Release`:

```cmake
add_compile_options(
  -O3
  -march=armv8.2-a
  -mtune=cortex-a76
  -ffast-math
  -funroll-loops
)
```

### Phân tích ý nghĩa kỹ thuật của các Cờ biên dịch:
1. **`-O3` (Mức tối ưu hóa cao nhất):** Ép trình biên dịch GCC thực hiện các tối ưu hóa toán học chuyên sâu như: tự động inline các hàm ngắn, phân tích và sắp xếp lại thứ tự biến cục bộ để lưu trữ trên thanh ghi thay vì bộ nhớ RAM, tối ưu hóa các lệnh rẽ nhánh và tự động vector hóa mã nguồn ở mức độ cao.
2. **`-march=armv8.2-a` (Target Architecture):** Chỉ thị GCC sinh mã máy tối ưu cho tập lệnh ARMv8.2-A. Tập lệnh này hỗ trợ tính toán số học dấu phẩy động độ chính xác nửa (**FP16**) bằng phần cứng, giúp thực hiện các phép tính mạng neural nhanh gấp đôi so với FP32 truyền thống.
3. **`-mtune=cortex-a76` (Micro-architecture Tuning):** Tối ưu hóa cấu trúc mã máy (sắp xếp đường ống lệnh pipeline và phân phối thanh ghi) đặc thù cho kiến trúc nhân Cortex-A76 của vi xử lý Broadcom BCM2712 trên Raspberry Pi 5, giảm thiểu tối đa hiện tượng trễ chu kỳ lệnh (stalls).
4. **`-ffast-math` (Toán học siêu tốc):** Cho phép trình biên dịch nới lỏng các quy tắc chặt chẽ của chuẩn IEEE-754 (ví dụ: không kiểm tra các giá trị NaN hay vô cùng Inf, bỏ qua phân biệt dấu của số 0). Nhờ đó, các phép tính ma trận và lượng giác lượng lớn trong IPM và khớp đa thức chạy nhanh hơn từ 15-20%.
5. **`-funroll-loops` (Khai triển vòng lặp):** Trình biên dịch tự động mở rộng các vòng lặp cố định thành chuỗi lệnh tuần tự. Kỹ thuật này làm giảm số lượng lệnh kiểm tra điều kiện và rẽ nhánh (branching instructions), giúp bộ dự báo rẽ nhánh của CPU hoạt động chính xác hơn và tận dụng bộ nhớ đệm lệnh (instruction cache) tốt hơn.

---

## 3. Tối ưu hóa Mô hình AI qua Thư viện NCNN và ARM NEON

Mô hình YOLO Seg được biên dịch và suy luận thông qua thư viện NCNN (đã được cấu hình tối ưu hóa trong container Docker):

1. **Khai thác ARM NEON SIMD:**
   - Thư viện NCNN tích hợp sẵn mã nguồn viết bằng ngôn ngữ assembly tối ưu hóa riêng cho kiến trúc ARM NEON. Khi chạy tính toán tích chập (Convolution), CPU thực thi các lệnh SIMD (Single Instruction Multiple Data) để nhân chập song song nhiều kênh điểm ảnh cùng một lúc, giảm thiểu số chu kỳ CPU cần thiết.
2. **Chế độ Tính toán FP16 (Half-Precision):**
   - NCNN chuyển đổi các trọng số của mô hình từ dạng FP32 ($32$-bit) sang FP16 ($16$-bit). Việc này giảm một nửa kích thước tệp trọng số lưu trữ và giảm một nửa băng thông bộ nhớ RAM khi nạp mô hình, đồng thời tận dụng trực tiếp các thanh ghi vector FP16 trên nhân Cortex-A76 để tăng gấp đôi số phép tính trên một giây (flops).
3. **Phân bổ luồng CPU tối ưu (Cores Threading):**
   - Đặt tham số suy luận `num_threads = 3` hoặc `4`. Raspberry Pi 5 sở hữu 4 nhân Cortex-A76 vật lý đồng nhất. Việc phân bổ 3 luồng cho AI suy luận giúp tận dụng tối đa năng lực xử lý song song, đồng thời chừa lại 1 nhân cho luồng chính của ROS2 xử lý hình học và truyền thông DDS, tránh tình trạng nghẽn cổ chai luồng chính.

---

## 4. Tối ưu hóa Đa luồng và Cơ chế Đọc ảnh Không Trễ (Zero-lag Buffer)

Trong node `video_publisher_node`, việc đọc camera phần cứng có thể bị block do trễ đồng bộ phần cứng hoặc trễ từ driver V4L2 của hệ điều hành. AVS giải quyết triệt để vấn đề này bằng thiết kế **Asynchronous Frame Capture**:

```
                       [Luồng nền: capture_loop]
                                  │
                                  ▼ cv::VideoCapture::read()
                 ┌─────────────────────────────────┐
                 │  Buffer Ghi đè (Kích thước = 1)  │ ◄── Ghi đè frame mới nhất
                 └─────────────────────────────────┘
                                  ▲
                                  │  Consume & Publish
                       [Luồng chính: ROS2 Timer]
```

- Luồng phụ nền `capture_loop` liên tục truy vấn camera và ghi đè khung hình mới nhất vào biến dùng chung `latest_frame_` (kích thước bộ đệm bằng 1).
- Khi luồng chính ROS2 gọi callback phát ảnh, nó sẽ lấy khung hình mới nhất từ bộ đệm này.
- **Hiệu quả:** Nếu node AI xử lý chậm hơn tốc độ phát của camera, các khung hình cũ xếp hàng trong hàng đợi của Driver camera sẽ bị giải phóng ngay lập tức mà không tích lũy thành hàng đợi dài. Xe tự hành luôn nhận được ảnh mới nhất từ thực địa, loại bỏ hoàn toàn hiện tượng trễ tích lũy (image latency buildup) gây đâm va vật cản.

---

## 5. Tối ưu hóa Băng thông DDS và Truyền thông Dashboard

Khi xe chạy ngoài thực địa và giao tiếp không dây (Wifi) với máy tính giám sát, băng thông mạng là một giới hạn vật lý nghiêm trọng. AVS áp dụng các tối ưu hóa băng thông sau:

### 5.1. Nén ảnh JPEG chất lượng 80% (Compressed Image)
- Một luồng ảnh thô kích thước $640 \times 480$ định dạng BGR8 ở tần số $30\text{ fps}$ tiêu tốn băng thông mạng khổng lồ:
  $$\text{Băng thông thô} = 640 \times 480 \times 3 \text{ bytes} \times 30\text{ fps} \approx 27.6\text{ MB/s} \approx 221\text{ Mbps}$$
- Băng thông này vượt quá giới hạn ổn định của kết nối Wifi ngoài thực địa. AVS chỉ truyền luồng ảnh thô cục bộ giữa các node nhúng qua Shared Memory IPC, và nén ảnh sang định dạng JPEG ở chất lượng $80\%$ qua topic `/camera/image_raw/compressed` để truyền về Dashboard:
  $$\text{Băng thông nén JPEG} \approx 1.0 - 1.5\text{ MB/s} \approx 8 - 12\text{ Mbps}$$
- Giúp giảm dung lượng truyền tải đi gần **20 lần**, đảm bảo luồng hình ảnh giám sát luôn mượt mà và không gây nghẽn đường truyền DDS.

### 5.2. Trích xuất biên đa giác thay cho mặt nạ dày đặc (Contour vs Dense Mask)
- Thay vì truyền bản đồ mặt nạ phân vùng nhị phân (Dense Segmentation Mask) kích thước lớn ($320 \times 320$ hoặc $640 \times 480$ bytes) qua mạng ROS2, node AI thực hiện rút trích biên ngoài tối giản bằng thuật toán xấp xỉ biên `cv::CHAIN_APPROX_SIMPLE`.
- Payload truyền tải chuyển đổi từ một ma trận điểm ảnh cồng kềnh sang một danh sách tọa độ các điểm biên thưa. Kích thước gói tin JSON telemetry giảm xuống dưới **$5\text{ KB}$**, giúp giảm thiểu tối đa tải tuần tự hóa chuỗi văn bản (JSON serialization overhead) và tiết kiệm băng thông mạng DDS.
