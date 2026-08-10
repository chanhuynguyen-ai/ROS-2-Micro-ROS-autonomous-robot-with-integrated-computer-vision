# Kế Hoạch Tối Ưu Inference Và Latency

Folder này chứa các plan tối ưu hiệu năng perception runtime, tập trung vào:

- giảm latency end-to-end của `ncnn_inference_node`
- tăng FPS xử lý ổn định trên Raspberry Pi 5
- kiểm chứng lựa chọn NCNN FP32/FP16/INT8 bằng benchmark và chất lượng output
- đánh giá khả năng dùng Vulkan trên Raspberry Pi 5 mà không phá vỡ pipeline ROS2 hiện tại

Plan chính:

- `ncnn_inference_latency_plan.md` — kế hoạch benchmark và triển khai tối ưu NCNN inference, post-processing, camera/ROS pipeline.
- `pi_benchmark_guide.md` — hướng dẫn triển khai benchmark B0/B1/B2 (INT8/FP32/FP16) lên Raspberry Pi 5 thật và kéo kết quả về laptop đánh giá; §13 hướng dẫn benchmark Vulkan B3/B4 (Pha D) bằng image riêng `avs_perception:arm64-vulkan`.
- `results/benchmark_report_20260708.md` — báo cáo phân tích benchmark Pi 5 từ các run B0/B1/B2 (CPU, image production NCNN `20240820`; dữ liệu thô trong `results/unuse_vulkan/`).
- `results/benchmark_report_vulkan_20260708.md` — báo cáo phân tích benchmark Vulkan Pha D (B3/B4 + B1/B2 re-run, image `arm64-vulkan` NCNN `b16501a`; dữ liệu thô trong `results/use_vulkan/`).

> [!NOTE]
> **Selective Mask Decoding (Pha E):** Mặc định, parameter `decode_non_control_masks` được set thành `false` ở cả môi trường runtime và benchmark/offline profiler. Hệ thống sẽ bỏ qua việc giải mã đa giác (mask) cho xe cộ (vehicle) và biển báo (sign-*) để tối ưu performance. Node điều khiển (Control) chỉ sử dụng Bounding Box cho các nhãn này. Nếu bạn cần trích xuất hoặc hiển thị mask cho xe cộ trên dashboard/debug tool, bạn phải truyền cờ `--ros-args -p decode_non_control_masks:=true` (hoặc đặt giá trị này trong node options).

Kết quả benchmark CPU (B0/B1/B2, NCNN `20240820`):

- Benchmark ngày `2026-07-08` được chạy trên Raspberry Pi 5 với governor `performance`, `num_threads = 3`, input `video_test1`, resolution `320`.
- Mức cap CPU thực tế là `1500 MHz`, khác với ví dụ `2000 MHz` trong guide; vì vậy các số liệu tuyệt đối là baseline ở `1.5 GHz`.
- Trong 3 preset đã đo (`INT8 CPU`, `FP32 CPU`, `FP16 CPU`), `FP16 CPU` đang cho kết quả tốt nhất về `inference_latency_ms`, `node_total_latency_ms`, `output_age_ms`, `processing_fps` và `publish_fps`.
- Kết luận chi tiết, bảng số liệu và giới hạn benchmark nằm trong `results/benchmark_report_20260708.md`.

Bảng so sánh nhanh (CPU, NCNN `20240820`):

| Preset | Run ID | Inference mean (ms) | Inference p95 (ms) | Node total mean (ms) | Output age mean (ms) | Processing FPS mean | Publish FPS mean | Main lane missing | Turn lane present | CPU util tổng (%) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `INT8 CPU` | `20260708_114354` | 62.64 | 79.37 | 63.49 | 92.40 | 15.96 | 14.96 | 1.67% | 23.67% | 78.28 |
| `FP32 CPU` | `20260708_114543` | 72.27 | 89.62 | 73.22 | 100.72 | 13.96 | 12.96 | 1.00% | 22.67% | 78.25 |
| `FP16 CPU` | `20260708_114703` | 45.15 | 56.06 | 46.00 | 61.84 | 20.48 | 19.49 | 1.00% | 33.00% | 71.61 |

Kết quả benchmark Vulkan — Pha D (B3/B4 + B1/B2 re-run cùng image, NCNN `b16501a`):

- Chạy cùng ngày `2026-07-08`, cùng điều kiện (cap CPU `1500 MHz`, `video_test1`, `320`, 3 threads) bằng image `avs_perception:arm64-vulkan` (Mesa 25 v3dv, `NCNN_VULKAN=ON`). GPU nhận đúng `V3D 7.1.10.2`, chạy ổn định đủ 300 frame, không crash — kết quả âm dưới đây là số đo thật, không phải lỗi cấu hình.
- **Kết luận Pha D: KHÔNG dùng Vulkan trên Pi 5.** `FP32 Vulkan` chậm hơn `FP32 CPU` cùng version **7.6×**; `FP16 Vulkan` chậm hơn `FP16 CPU` **16.5×** (V3D không có fp16 arithmetic — `fp16-a=0` — nên FP16 còn chậm hơn FP32 trên GPU). Pipeline Vulkan chỉ đạt ~2 FPS trên input 20 FPS, `output_age_ms` ~549–715 ms — loại dứt khoát theo gate §6.4 của plan.
- Chất lượng output Vulkan không suy giảm (main-lane missing 0–0.67%) — v3dv tính đúng, vấn đề thuần túy là tốc độ.
- Phát hiện phụ: NCNN `b16501a` nhanh hơn `20240820` trên CPU (FP16 p95 giảm ~20%, output age mean giảm ~24.5%) — đáng mở việc riêng đánh giá nâng version NCNN production (cần test chất lượng trước).
- Chi tiết trong `results/benchmark_report_vulkan_20260708.md`.

Bảng so sánh nhanh (image `arm64-vulkan`, NCNN `b16501a`):

| Preset | Run ID | Inference mean (ms) | Inference p95 (ms) | Node total mean (ms) | Output age mean (ms) | Processing FPS mean | Publish FPS mean | Main lane missing | Turn lane present | CPU util tổng (%) |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `FP32 Vulkan` | `20260708_142504` | 518.48 | 522.26 | 519.37 | 548.54 | 2.00 | 1.00 | 0.67% | 28.33% | 15.40 |
| `FP16 Vulkan` | `20260708_143027` | 686.85 | 690.95 | 687.75 | 714.87 | 2.00 | 1.00 | 0.00% | 29.67% | 15.18 |
| `FP32 CPU` | `20260708_143757` | 68.38 | 76.58 | 69.22 | 97.20 | 14.87 | 13.87 | 1.00% | 21.33% | 77.46 |
| `FP16 CPU` | `20260708_143853` | 41.59 | 44.87 | 42.52 | 46.68 | 20.47 | 19.50 | 1.33% | 33.67% | 66.53 |

Trạng thái quyết định sau Pha C + Pha D: **`FP16 CPU` là ứng viên default duy nhất còn lại** cho `ncnn_inference_node` trên Pi 5 (INT8 giữ làm fallback, FP32 và Vulkan loại).

Bảng quyết định cuối cùng:

| Runtime | Quyết định | Ghi chú |
|---|---|---|
| `FP16 CPU` | Chọn default candidate | Tốt nhất về latency/FPS trong cả batch production CPU và batch image Vulkan khi chạy CPU mode. |
| `INT8 CPU` | Giữ fallback | Nhanh hơn `FP32 CPU` nhưng thua `FP16 CPU`; chưa đủ cơ sở chất lượng để chọn làm default segmentation/lane. |
| `FP32 CPU` | Loại default | Chậm hơn `FP16 CPU`, không có lợi thế runtime rõ ràng. |
| `FP32 Vulkan` | Loại production | `518.48 ms` inference mean, khoảng `2 FPS`; chậm hơn CPU cùng image khoảng `7.6x`. |
| `FP16 Vulkan` | Loại production | `686.85 ms` inference mean; chậm hơn `FP16 CPU` cùng image khoảng `16.5x` vì V3D không có fp16 arithmetic. |

Ràng buộc chung:

- Không đổi contract `/avs/telemetry`, `/avs/telemetry_realworld`, `/avs/control_error` nếu không có phase riêng được duyệt.
- Không coi FPS model đơn lẻ là thành công nếu `output_age_ms` hoặc jitter toàn pipeline tăng.
- Không dùng INT8 làm default production cho segmentation/lane nếu chưa có benchmark chất lượng mask/polygon/centerline trên dữ liệu AVS.
- Mọi thay đổi tối ưu phải có số liệu trước/sau trên cùng fixture hoặc cùng camera setup.
