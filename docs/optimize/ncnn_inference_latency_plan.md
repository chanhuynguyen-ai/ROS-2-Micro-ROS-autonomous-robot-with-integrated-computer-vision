# Plan Tối Ưu NCNN Inference Và Latency Trên Raspberry Pi 5

## 1. Mục Tiêu

Tối ưu pipeline inference hiện tại để:

- giảm `inference_latency_ms`
- giảm `node_total_latency_ms`
- giảm `output_age_ms`
- tăng `processing_fps` và `publish_fps` ổn định
- giữ chất lượng segmentation đủ tốt cho IPM, centerline và control

Phạm vi chính:

- `ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp`
- `ros2_ws/src/avs_perception/src/yolo26_seg.cpp`
- `ros2_ws/src/avs_perception/include/avs_perception/yolo26_seg.hpp`
- `ros2_ws/src/avs_perception/CMakeLists.txt`
- `models/best_ncnn_model/`
- `models/best_ncnn_model_int8/`
- Docker/build scripts nếu cần rebuild NCNN có Vulkan

## 2. Hiện Trạng Đã Khảo Sát

### 2.1 Runtime hiện tại

Pipeline:

```text
/camera/image_raw
-> ncnn_inference_node
-> YOLO26Seg::detect()
-> mask decode + contour extraction
-> JSON telemetry
-> /avs/telemetry
```

Các metric đã có trong telemetry:

- `input_fps`
- `processing_fps`
- `publish_fps`
- `bridge_latency_ms`
- `inference_latency_ms`
- `post_processing_latency_ms`
- `contour_time_ms`
- `json_finalize_latency_ms`
- `publish_latency_ms`
- `node_processing_latency_ms`
- `node_total_latency_ms`
- `input_age_ms`
- `output_age_ms`

### 2.2 Model và NCNN option hiện tại

`ncnn_inference_node.cpp` đang default tới model INT8:

```text
models/best_ncnn_model_int8/model.ncnn.param
models/best_ncnn_model_int8/model.ncnn.bin
```

`YOLO26Seg` hiện cấu hình:

```text
use_vulkan_compute = false
use_fp16_packed = true
use_fp16_storage = true
use_fp16_arithmetic = true
use_packing_layout = true
use_int8_inference = true
num_threads = 3 trong constructor
num_threads = 4 qua default ROS parameter của ncnn_inference_node
```

NCNN local build hiện tại:

```text
NCNN_VULKAN OFF
NCNN_VULKAN 0 trong ncnn/include/ncnn/platform.h
```

Kết luận:

- Code hiện tại chưa thể dùng Vulkan dù bật `use_vulkan_compute` ở runtime.
- Baseline production hiện tại đang nghiêng sang INT8, trái với hướng cần kiểm chứng FP32/FP16.
- Cần benchmark có kiểm soát trước khi đổi default runtime.

## 3. Cơ Sở Kỹ Thuật

Theo tài liệu Ultralytics hiện hành:

- NCNN là format được khuyến nghị cho Raspberry Pi vì tối ưu cho mobile/embedded ARM.
- Benchmark Ultralytics trên Raspberry Pi 5 với YOLO26n FP32, input 640, báo NCNN khoảng `67.69 ms/im`, chưa tính pre/post-processing.
- NCNN hỗ trợ Vulkan GPU acceleration qua device kiểu `vulkan:0` khi driver và NCNN build hỗ trợ Vulkan.
- Export NCNN hỗ trợ `quantize=16` cho FP16; `32` hoặc không set là FP32.

Theo thông số Raspberry Pi 5:

- CPU: Broadcom BCM2712, quad-core Arm Cortex-A76 2.4GHz.
- GPU: VideoCore VII, hỗ trợ OpenGL ES 3.1 và Vulkan 1.3.

Ý nghĩa cho AVS:

- Vulkan đáng benchmark nhưng không được giả định chắc chắn nhanh hơn CPU cho mọi layer hoặc mọi kích thước input.
- FP16 đáng thử vì có thể giảm bandwidth/model size và tăng tốc, nhưng phải đo chất lượng mask/polygon.
- INT8 không nên là default production nếu chưa có calibration/QAT và test chất lượng đầu ra cho lane segmentation.

Nguồn tham chiếu:

- https://docs.ultralytics.com/integrations/ncnn/
- https://docs.ultralytics.com/guides/raspberry-pi/
- https://www.raspberrypi.com/products/raspberry-pi-5/

## 4. Nguyên Tắc Tối Ưu

1. Đo trước, sửa sau.
2. Tách `model inference latency` khỏi `node end-to-end latency`.
3. Không tối ưu FPS bằng cách tạo backlog.
4. So sánh chất lượng output cùng lúc với tốc độ.
5. Không thay đổi planner/control để bù cho lỗi segmentation do tối ưu inference.
6. Mọi phase phải có rollback path rõ ràng qua ROS parameter hoặc build flag.

## 5. Benchmark Matrix Bắt Buộc

### 5.1 Biến thể model/runtime

| ID | Model | NCNN Vulkan | FP16 option | INT8 option | Mục đích |
|---|---|---:|---:|---:|---|
| B0 | `best_ncnn_model_int8` | off | on hiện tại | on | Baseline repo hiện tại |
| B1 | `best_ncnn_model` | off | off | off | FP32 CPU baseline |
| B2 | `best_ncnn_model` hoặc export FP16 | off | on | off | FP16 CPU |
| B3 | `best_ncnn_model` | on | off | off | FP32 Vulkan |
| B4 | FP16 NCNN | on | on | off | FP16 Vulkan |

Nếu B3/B4 không chạy do driver/build/layer support, ghi nhận là kết quả hợp lệ và fallback về CPU.

### 5.2 Metric cần thu thập

Tối thiểu:

- `input_fps`
- `processing_fps`
- `publish_fps`
- `bridge_latency_ms`
- `inference_latency_ms`
- `post_processing_latency_ms`
- `contour_time_ms`
- `json_finalize_latency_ms`
- `publish_latency_ms`
- `node_total_latency_ms`
- `input_age_ms`
- `output_age_ms`
- CPU utilization per core
- memory usage
- temperature và throttling state

Chất lượng output:

- số object theo class quan trọng: `main-lane`, `other-lane`, `turn-lane`, lane markings, `stop-line`
- diện tích mask trung bình theo class
- số polygon points trung bình
- tỷ lệ frame mất lane chính
- sai lệch centerline/IPM nếu có fixture ground truth hoặc replay đã biết
- regression `turn-lane = 17` không bị ảnh hưởng

### 5.3 Fixture benchmark

Cần tối thiểu 3 nhóm input:

- Video thẳng, ít object, ánh sáng ổn định.
- Video ngã rẽ/ngã tư có `turn-lane`.
- Video nhiễu hơn: blur, ánh sáng kém, nhiều marking hoặc nhiều object.

Mỗi benchmark phải chạy tối thiểu:

- warmup 30 frame không tính vào thống kê
- đo 300 frame hoặc ít nhất 60 giây camera live
- báo trung bình, p50, p90, p95, max cho latency

## 6. Kế Hoạch Triển Khai Theo Pha

## 6.1 Pha A: Chuẩn Hóa Benchmark Harness

Mục tiêu:

- có cách chạy benchmark lặp lại, không phụ thuộc quan sát thủ công dashboard
- xuất CSV/JSON để so sánh trước/sau

Việc cần làm:

- thêm script hoặc mode benchmark đọc telemetry `/avs/telemetry`
- lưu metric theo frame
- thêm summary p50/p90/p95/max
- ghi lại model path, NCNN flags, thread count, input source, resolution, git commit
- thêm checklist vận hành trên Pi 5: governor, nhiệt độ, throttling, `vulkaninfo`

Output:

- `docs/optimize/results/` hoặc `scratch/benchmark_results/` cho kết quả đo local
- bảng baseline B0 trước khi sửa code inference

Gate:

- chạy được baseline hiện tại B0
- có file kết quả chứa đủ metric ở §5.2

## 6.2 Pha B: Đưa NCNN Options Thành Runtime Parameters

Mục tiêu:

- bỏ hardcode inference mode
- benchmark được FP32/FP16/INT8/Vulkan mà không rebuild app mỗi lần

Thay đổi dự kiến:

- thêm ROS parameters:
  - `use_vulkan_compute`
  - `use_fp16_packed`
  - `use_fp16_storage`
  - `use_fp16_arithmetic`
  - `use_packing_layout`
  - `use_int8_inference`
  - `target_size` nếu muốn benchmark `320` vs `416` hoặc `640`
- chỉ set các option trước khi load model
- không gọi `set_num_threads()` mỗi frame nếu parameter không đổi
- log đầy đủ config inference khi node start

Default đề xuất sau pha này:

```text
model_param_path = models/best_ncnn_model/model.ncnn.param
model_bin_path = models/best_ncnn_model/model.ncnn.bin
use_int8_inference = false
use_vulkan_compute = false
use_fp16_* = false cho FP32 baseline, hoặc true nếu FP16 được chọn sau benchmark
num_threads = 3
```

Lưu ý:

- Không đổi default sang Vulkan cho đến khi Pha D pass.
- Không xóa INT8 path; giữ để so sánh và rollback.

Gate:

- build pass
- B0 vẫn chạy được bằng parameter override
- B1 chạy được bằng default hoặc launch config mới
- telemetry/log thể hiện đúng mode đang chạy

## 6.3 Pha C: Benchmark FP32 Và FP16 CPU

Mục tiêu:

- xác định FP32 hay FP16 CPU phù hợp hơn cho production trước khi đưa Vulkan vào.

Việc cần làm:

- export hoặc xác nhận model FP16 NCNN bằng Ultralytics `quantize=16`
- chạy B1 và B2 trên cùng fixture
- so sánh tốc độ và chất lượng output
- kiểm tra các layer có fallback gây chậm bất thường không

Tiêu chí chọn:

- Nếu FP16 nhanh hơn đáng kể và không giảm chất lượng lane/polygon, chọn FP16 CPU làm candidate.
- Nếu FP16 không nhanh hơn hoặc chất lượng mask giảm, giữ FP32 CPU.
- INT8 chỉ được chọn nếu có bằng chứng chất lượng tương đương trên fixture AVS, không chỉ dựa vào FPS.

Gate:

- bảng B1/B2/B0 có đủ metric
- có nhận xét chất lượng output cho lane classes
- chọn một CPU baseline production candidate

## 6.4 Pha D: Rebuild NCNN Có Vulkan Và Benchmark Vulkan

Mục tiêu:

- kiểm chứng Vulkan trên Raspberry Pi 5 bằng pipeline C++/ROS thật, không chỉ bằng Ultralytics Python.

Việc cần làm:

- cài/kiểm tra Vulkan runtime trên Pi 5
- xác minh `vulkaninfo` nhận GPU
- rebuild NCNN với:
  - `NCNN_VULKAN=ON`
  - `NCNN_ARM_NEON=ON`
  - cân nhắc `NCNN_BUILD_TOOLS=ON` cho benchmark/tooling
- cập nhật CMake/Docker path để link đúng NCNN Vulkan build
- bật `use_vulkan_compute=true` qua parameter
- chạy B3/B4

Rủi ro:

- một số layer có thể fallback CPU/GPU transfer overhead làm chậm hơn CPU.
- Vulkan có thể tăng throughput nhưng tăng latency/jitter nếu tranh chấp với display/compositor.
- Driver/container access có thể phức tạp hơn bare-metal.

Gate:

- build xác nhận `NCNN_VULKAN ON`
- node start thành công với Vulkan
- B3/B4 có số liệu hoặc có failure log rõ nguyên nhân
- không chọn Vulkan nếu `output_age_ms`, p95 latency hoặc jitter xấu hơn CPU baseline

## 6.5 Pha E: Tối Ưu Post-processing Mask/Contour/JSON

Mục tiêu:

- giảm latency ngoài model, đặc biệt khi scene có nhiều object/mask.
- chạy tối ưu trên runtime candidate đã chọn sau Pha C/D: `FP16 CPU`, không dùng Vulkan.

Baseline/default cho Pha E:

```text
model_param_path = models/best_ncnn_model/model.ncnn.param
model_bin_path = models/best_ncnn_model/model.ncnn.bin
use_vulkan_compute = false
use_int8_inference = false
use_fp16_packed = true
use_fp16_storage = true
use_fp16_arithmetic = true
use_packing_layout = true
num_threads = 3
```

Rollback path:

- `INT8 CPU`: override model path sang `models/best_ncnn_model_int8/` và `use_int8_inference=true`.
- `FP32 CPU`: giữ `models/best_ncnn_model/`, set `use_fp16_* = false`, `use_int8_inference=false`.
- `Vulkan`: không dùng production path hiện tại; chỉ giữ để tái lập benchmark Pha D nếu cần.

Việc cần làm:

- tách timing trong `YOLO26Seg::detect()`:
  - preprocess
  - extractor
  - proposal decode
  - NMS
  - mask decode
- chỉ decode mask cho class cần publish polygon/control nếu dashboard không cần mọi mask ở full rate
- tránh tạo full-frame `dest_mask` cho mọi object nếu contour ROI đủ dùng
- reserve vector/string trước khi build JSON
- cân nhắc giới hạn polygon points hoặc dùng `cv::approxPolyDP` có kiểm soát
- cân nhắc publish lightweight telemetry cho control và optional rich telemetry cho debug/dashboard

Ràng buộc:

- Không copy planner/control logic sang node inference.
- Không làm mất polygon cần cho IPM.
- Nếu giảm polygon points, phải kiểm tra centerline/IPM không lệch đáng kể.

Gate:

- `post_processing_latency_ms`, `contour_time_ms`, `json_finalize_latency_ms` giảm trên fixture nhiều object
- IPM/control regression pass
- dashboard/debug vẫn đọc được telemetry hoặc có migration plan

## 6.6 Pha F: Tối Ưu Camera/ROS Pipeline

Mục tiêu:

- giảm backlog và copy trước/sau inference.

Việc cần làm:

- xác nhận camera publish resolution/FPS thực tế
- nếu camera đang publish cao hơn `target_size`, cân nhắc crop/resize upstream
- dùng QoS phù hợp real-time:
  - depth 1
  - best effort nếu chấp nhận drop frame
  - không giữ frame cũ khi inference chậm
- kiểm tra `cv_bridge::toCvCopy()` có copy không cần thiết không
- cân nhắc `image_transport` compressed/raw theo CPU budget thực tế
- nếu cần, tách debug image stream khỏi control telemetry stream

Gate:

- `input_age_ms` và `output_age_ms` không tăng dần khi chạy lâu
- khi inference chậm, node drop frame cũ thay vì xử lý backlog
- control vẫn nhận trajectory đều và ổn định

## 6.7 Pha G: Benchmark Có Nên Dùng NMS Không

Mục tiêu:

- quyết định production runtime nên bật hay tắt NMS cho YOLO26 segmentation trên Raspberry Pi 5.
- đo bằng `video_test_node` trước khi chuyển sang chạy camera live dài hạn.

Bối cảnh:

- NMS hiện chạy sau proposal decode trong `YOLO26Seg::detect()`.
- Nếu tắt NMS, CPU có thể giảm phần so sánh IoU, nhưng số object duplicate đi vào `decode_mask()` có thể tăng và làm tổng latency tăng.
- Vì vậy không xóa cứng NMS; cần benchmark A/B với cùng video, cùng model, cùng FP16 CPU config và có giới hạn số detection khi NMS tắt.
- Guide thao tác riêng: [`phase_g_nms_benchmark_guide.md`](./phase_g_nms_benchmark_guide.md).

Việc cần làm:

- expose runtime params cho cả `ncnn_inference_node` và `video_test_node`:
  - `enable_nms` default `true`
  - `max_detections` default đủ an toàn, ví dụ `30` hoặc `40`
  - nếu cần thêm `max_proposals_before_nms` để giới hạn proposal trước bước mask decode
- khi `enable_nms=true`:
  - giữ flow hiện tại: sort proposal -> NMS -> scale/clamp -> selective mask decode
- khi `enable_nms=false`:
  - vẫn sort proposal theo confidence
  - bỏ so sánh IoU
  - chỉ lấy top `max_detections` trước khi decode mask
  - không để toàn bộ proposal vượt threshold đi vào mask decode
- chạy `video_test_node` trên Raspberry Pi 5 với cùng video test:

```bash
cd ros2_ws
source install_user/setup.bash

ros2 run avs_perception video_test_node \
  --ros-args \
  -p video_path:=/home/goln/SimpleSysIDV/test/test_video/video_test1.mp4 \
  -p output_path:=/home/goln/SimpleSysIDV/test/test_video_output/nms_on.mp4 \
  -p use_vulkan_compute:=false \
  -p use_int8_inference:=false \
  -p use_fp16_packed:=true \
  -p use_fp16_storage:=true \
  -p use_fp16_arithmetic:=true \
  -p use_packing_layout:=true \
  -p num_threads:=3 \
  -p decode_non_control_masks:=false \
  -p enable_nms:=true

ros2 run avs_perception video_test_node \
  --ros-args \
  -p video_path:=/home/goln/SimpleSysIDV/test/test_video/video_test1.mp4 \
  -p output_path:=/home/goln/SimpleSysIDV/test/test_video_output/nms_off_top30.mp4 \
  -p use_vulkan_compute:=false \
  -p use_int8_inference:=false \
  -p use_fp16_packed:=true \
  -p use_fp16_storage:=true \
  -p use_fp16_arithmetic:=true \
  -p use_packing_layout:=true \
  -p num_threads:=3 \
  -p decode_non_control_masks:=false \
  -p enable_nms:=false \
  -p max_detections:=30
```

Metrics cần ghi:

- `preprocess_latency_ms`
- `inference_latency_ms`
- `proposal_latency_ms`
- `nms_or_select_latency_ms`
- `mask_latency_ms`
- `node_total_latency_ms`
- FPS xử lý trung bình/p95
- số proposal trước filter, số object publish sau filter
- object count theo class quan trọng: `main-lane`, `other-lane`, `turn-lane`, lane marking, `stop-line`
- kiểm tra output video xem duplicate lane/mask có làm telemetry nhiễu không

Gate:

- Nếu `enable_nms=false` giảm p95 `node_total_latency_ms` rõ ràng, không làm `mask_latency_ms` tăng bù, và không tạo duplicate telemetry gây lệch IPM/control, chọn `enable_nms=false` làm production default.
- Nếu latency giảm không đáng kể, hoặc mask/object count tăng gây nhiễu, giữ `enable_nms=true`.
- Quyết định phải dựa trên Pi 5 thật; kết quả laptop chỉ dùng để sanity check.
- Sau khi chốt NMS, mới chạy runtime stability camera live dài hạn: cooling, power supply, throttling, governor/performance mode, `num_threads`, và jitter IPM/control.

## 6.8 Pha H: Rebuild Image Production Với NCNN CPU Version Mới

Mục tiêu:

- sau khi hoàn tất các pha tối ưu runtime/post-processing/camera và chốt quyết định NMS, rebuild image production **không Vulkan** với NCNN CPU version mới hơn để lấy lợi ích CPU đã thấy trong benchmark image Vulkan.

Bối cảnh benchmark:

- Image `avs_perception:arm64-vulkan` dùng NCNN `b16501a` cho thấy `FP16 CPU` nhanh hơn image production NCNN `20240820`:
  - `inference_latency_ms` mean: `45.15 ms` -> `41.59 ms`
  - `inference_latency_ms` p95: `56.06 ms` -> `44.87 ms`
  - `output_age_ms` mean: `61.84 ms` -> `46.68 ms`
- Lợi ích này đến từ đường CPU/NCNN version mới, không phải Vulkan. Vì vậy production image nên giữ `NCNN_VULKAN=OFF` để giảm dependency và rủi ro driver.

Việc cần làm:

- rebuild NCNN trong image production với version mới đã benchmark hoặc commit được chốt tương đương `b16501a`.
- giữ build flags CPU:
  - `NCNN_VULKAN=OFF`
  - `NCNN_ARM_NEON=ON`
  - `NCNN_SHARED_LIB=ON`
  - Release build, tối ưu ARM64/Cortex-A76 như Docker production hiện tại.
- giữ default runtime là `FP16 CPU` như Pha E:
  - `use_vulkan_compute=false`
  - `use_int8_inference=false`
  - `use_fp16_packed/storage/arithmetic=true`
  - `num_threads=3`
- chạy lại benchmark B1/B2 tối thiểu trên image production mới, cùng fixture/cap CPU, để xác nhận lợi ích không phụ thuộc image Vulkan.
- chạy regression chất lượng mask/polygon/IPM/control trước khi thay image production chính thức.

Gate:

- image production mới build pass trên Pi 5/container ARM64.
- `FP16 CPU` trên image mới không chậm hơn image production cũ và không tạo regression chất lượng.
- rollback image production cũ vẫn sẵn sàng nếu có lỗi runtime.

## 7. Bảng Quyết Định Production Candidate

Sau Pha C/D, production candidate hiện tại được chọn bằng bảng:

| Mode | Avg inference | P95 total | P95 output age | FPS | Quality | Risk | Chọn |
|---|---:|---:|---:|---:|---|---|---|
| INT8 CPU | 62.64 ms | 80.37 ms | 115.62 ms | 15.96 / 14.96 | proxy OK, chưa đủ ground truth | INT8 quality chưa được chứng minh đủ cho lane segmentation | Fallback |
| FP32 CPU | 68.38-72.27 ms | 77.37-90.39 ms | 120.34-124.42 ms | 13.96-14.87 / 12.96-13.87 | proxy OK | chậm hơn FP16 CPU | No |
| FP16 CPU | 41.59-45.15 ms | 45.82-57.03 ms | 61.81-82.32 ms | 20.47-20.48 / 19.49-19.50 | proxy OK, cần thêm fixture chất lượng | candidate chính, cần regression IPM/control | Yes |
| FP32 Vulkan | 518.48 ms | 523.63 ms | 571.52 ms | 2.00 / 1.00 | proxy OK | chậm hơn CPU cùng image khoảng 7.6x | No |
| FP16 Vulkan | 686.85 ms | 691.91 ms | 736.48 ms | 2.00 / 1.00 | proxy OK | chậm hơn FP16 CPU cùng image khoảng 16.5x, V3D không có fp16 arithmetic | No |

Ghi chú:

- Cột FPS ghi `processing_fps / publish_fps`.
- Khoảng số liệu CPU phản ánh hai batch benchmark: image production NCNN `20240820` và image Vulkan khi chạy CPU mode với NCNN `b16501a`.
- Quyết định cuối cùng sau Pha C/D: `FP16 CPU` là default candidate; `INT8 CPU` giữ rollback; Vulkan loại khỏi production path hiện tại.

Quy tắc chọn:

- Ưu tiên mode có p95 thấp và output ổn định, không chỉ avg FPS cao.
- Nếu Vulkan nhanh hơn avg nhưng p95/jitter xấu hơn CPU, không chọn Vulkan cho control loop.
- Nếu INT8 làm giảm chất lượng lane/turn-lane, không dùng production dù nhanh hơn.

## 8. Command Gợi Ý

Build package:

```bash
cd ros2_ws
colcon --log-base log_user build --symlink-install --packages-select avs_perception --build-base build_user --install-base install_user
```

Chạy node với FP16 CPU candidate:

```bash
cd ros2_ws
source install_user/setup.bash
ros2 run avs_perception ncnn_inference_node \
  --ros-args \
  -p model_param_path:=/home/goln/SimpleSysIDV/models/best_ncnn_model/model.ncnn.param \
  -p model_bin_path:=/home/goln/SimpleSysIDV/models/best_ncnn_model/model.ncnn.bin \
  -p use_int8_inference:=false \
  -p use_vulkan_compute:=false \
  -p use_fp16_packed:=true \
  -p use_fp16_storage:=true \
  -p use_fp16_arithmetic:=true \
  -p use_packing_layout:=true \
  -p num_threads:=3
```

Chạy node với FP32 CPU rollback:

```bash
cd ros2_ws
source install_user/setup.bash
ros2 run avs_perception ncnn_inference_node \
  --ros-args \
  -p model_param_path:=/home/goln/SimpleSysIDV/models/best_ncnn_model/model.ncnn.param \
  -p model_bin_path:=/home/goln/SimpleSysIDV/models/best_ncnn_model/model.ncnn.bin \
  -p use_int8_inference:=false \
  -p use_vulkan_compute:=false \
  -p use_fp16_packed:=false \
  -p use_fp16_storage:=false \
  -p use_fp16_arithmetic:=false \
  -p num_threads:=3
```

Chạy node với INT8 baseline hiện tại:

```bash
cd ros2_ws
source install_user/setup.bash
ros2 run avs_perception ncnn_inference_node \
  --ros-args \
  -p model_param_path:=/home/goln/SimpleSysIDV/models/best_ncnn_model_int8/model.ncnn.param \
  -p model_bin_path:=/home/goln/SimpleSysIDV/models/best_ncnn_model_int8/model.ncnn.bin \
  -p use_int8_inference:=true \
  -p use_vulkan_compute:=false
```

Kiểm tra Vulkan trên Pi 5:

```bash
vulkaninfo --summary
```

## 9. Điều Kiện Hoàn Thành Plan

Plan này chỉ được coi là hoàn thành khi có:

- benchmark B0/B1/B2 tối thiểu trên Pi 5
- benchmark B3/B4 hoặc failure report rõ ràng nếu Vulkan không khả dụng
- production candidate được chọn bằng bảng §7
- code runtime option đã build pass
- regression decision/IPM liên quan pass nếu post-processing thay đổi
- Pha H hoàn tất: image production không Vulkan được rebuild với NCNN CPU version mới hoặc có quyết định rõ ràng hoãn nâng version
- benchmark B1/B2 trên image production mới xác nhận `FP16 CPU` không regression so với image cũ
- tài liệu kết quả đo được lưu lại và có command tái lập
