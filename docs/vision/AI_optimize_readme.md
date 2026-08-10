# AI Inference Optimization Review

This document reviews the current NCNN inference pipeline in this repository and proposes a practical optimization plan for Raspberry Pi 5 deployment.

It is intentionally scoped to the code and container setup that exist today in `SimpleSysIDV`, not to a separate standalone demo project.

## Scope

This review focuses on:

- ROS2 Humble perception pipeline
- NCNN-based YOLO26 segmentation inference
- Dockerized deployment
- Raspberry Pi 5 CPU/GPU optimization opportunities

Relevant implementation files:

- `docker/Dockerfile`
- `ros2_ws/src/avs_perception/src/yolo26_seg.cpp`
- `ros2_ws/src/avs_perception/src/ncnn_inference_node.cpp`
- `ros2_ws/src/avs_perception/CMakeLists.txt`

## Current System State

### Runtime Architecture

The current repository runs inference as a ROS2 node, not as a standalone `./yoloncnn` executable.

Current flow:

1. Camera frames are published to `/camera/image_raw`
2. `ncnn_inference_node` subscribes to raw images
3. YOLO26 NCNN inference runs in `YOLO26Seg`
4. Masks are decoded per detection
5. Contours are extracted from binary masks
6. Telemetry is serialized into JSON and published to `/avs/telemetry`

### Container Build State

The Docker image currently:

- installs OpenCV from Debian packages (`libopencv-dev`, `python3-opencv`)
- builds NCNN from source
- disables Vulkan at build time
- disables NCNN tools at build time

From `docker/Dockerfile`:

- `-DNCNN_VULKAN=OFF`
- `-DNCNN_BUILD_TOOLS=OFF`
- `-DNCNN_ARM_NEON=ON`

Implications:

- Vulkan inference is not currently available in the deployed image
- tools such as `ncnnoptimize`, `ncnn2table`, and `ncnn2int8` are not available inside the container
- CPU-side NEON support is enabled in NCNN

### Inference Engine State

From `yolo26_seg.cpp`, the current NCNN options are:

- `use_vulkan_compute = false`
- `use_fp16_packed = true`
- `use_fp16_storage = true`
- `use_fp16_arithmetic = true`
- `use_packing_layout = true`
- `use_int8_inference = true`
- `num_threads = 4`

The current code therefore prefers CPU inference with FP16 packing enabled and INT8 inference enabled, while using all 4 Pi 5 CPU cores.

### Build Optimization State

`ros2_ws/src/avs_perception/CMakeLists.txt` already enables useful Release optimizations for ARM64 builds:

- `-O3`
- `-march=armv8.2-a`
- `-mtune=cortex-a76`
- `-ffast-math`
- `-funroll-loops`

This is a solid baseline. The main remaining gains are likely to come from runtime behavior and algorithmic changes rather than compiler flags alone.

## Likely Bottlenecks

### 1. Per-frame logging overhead

`ncnn_inference_node.cpp` logs at `INFO` level for every received frame and every telemetry publish.

That is unnecessary overhead for a real-time pipeline and can create jitter, especially on edge hardware.

### 2. Full-frame mask decode per detection

`decode_mask()` currently:

1. computes the full 80x80 prototype combination
2. applies sigmoid over the full 80x80 map
3. crops only after full computation
4. resizes the cropped area back to the bounding box

This is a reasonable first implementation, but it does unnecessary work for small detections.

### 3. Contour extraction and JSON serialization

After inference, the node:

- runs `cv::findContours()` for every object mask
- serializes all polygons manually into a JSON string

This cost sits outside the pure model inference time but still affects end-to-end latency and FPS.

### 4. Thread saturation

The engine hardcodes `num_threads = 4`.

On Raspberry Pi 5, using all cores for inference can reduce headroom for:

- ROS2 scheduling
- image transport
- IPM transformation
- control logic
- system services

This can improve isolated inference throughput while worsening total pipeline latency and jitter.

### 5. OpenCV package uncertainty

The current container uses Debian OpenCV packages.

It is plausible that a custom OpenCV build could improve preprocessing performance, but that should be treated as a benchmark question, not an assumption. The current code uses `ncnn::Mat::from_pixels_resize(...)`, so preprocessing is not purely an OpenCV problem anyway.

## Optimization Proposals

## 1. Reduce logging overhead first

### Why

This is low-risk, cheap, and directly relevant to real-time stability.

### Action

- downgrade per-frame `RCLCPP_INFO` logs to `RCLCPP_DEBUG`
- or use throttled logging for periodic observability

### Expected effect

- lower CPU overhead
- less scheduler disturbance
- cleaner profiling data

### Validation

Measure:

- full pipeline latency
- FPS variance
- CPU utilization

before and after log reduction.

## 2. Add stage-level profiling

### Why

Without timing breakdowns, optimization work is mostly guesswork.

### Action

Add timing around:

1. ROS image conversion
2. NCNN extract/inference
3. proposal decoding
4. mask decoding
5. contour extraction
6. JSON serialization
7. publish

### Expected effect

- identifies the dominant bottleneck
- prevents premature optimization in the wrong layer

### Validation

Collect timing summaries across a fixed video or ROS bag and compare medians/p95.

## 3. Make thread count configurable

### Why

`num_threads = 4` is currently a hardcoded policy decision.

That should be a runtime parameter so the system can be tuned under real workload.

### Action

- expose NCNN thread count as a ROS2 parameter
- benchmark `1`, `2`, `3`, and `4`

### Expected effect

- possible reduction in end-to-end latency jitter
- more stable coexistence with other ROS2 nodes

### Validation

Track:

- inference latency
- full pipeline latency
- FPS stability
- system CPU usage
- thermal throttling behavior

## 4. Optimize `decode_mask()` with ROI-bounded computation

### Why

The current implementation computes the full prototype map for every detection, then crops afterward.

For small objects, this is wasted work.

### Action

Refactor mask decode so that:

1. the image-space box is mapped to prototype-space first
2. linear combination is computed only over the needed ROI
3. sigmoid is applied only over that ROI

Optional follow-up:

- parallelize the loop
- evaluate NEON-friendly formulations only if profiling shows this function remains dominant

### Expected effect

- lower CPU time per detected object
- larger gains when many small objects are present

### Validation

Measure:

- isolated `decode_mask()` time
- total frame latency
- output mask equivalence against baseline

## 5. Reassess contour and telemetry payload cost

### Why

The current node turns every binary mask into polygons and then manually serializes them into JSON.

That is expensive and may not always be necessary at full fidelity.

### Action

Benchmark alternatives:

- contour simplification
- minimum contour area threshold
- publish fewer polygon points
- publish raster mask metadata only when polygons are not required
- move from manual string building to a structured JSON library if maintainability becomes a problem

### Expected effect

- reduced CPU overhead after inference
- smaller telemetry payloads
- lower downstream processing cost

### Validation

Measure:

- contour extraction time
- serialization time
- telemetry message size
- downstream consumer compatibility

## 6. Benchmark Vulkan instead of assuming it helps

### Why

Vulkan is currently disabled both at NCNN build time and at runtime.

It may help, or it may add overhead depending on model size, memory transfers, and Pi 5 driver behavior.

### Action

Create a controlled benchmark branch or image variant with:

- `-DNCNN_VULKAN=ON`
- runtime `use_vulkan_compute = true`

### Expected effect

Possible outcomes:

- lower CPU usage
- better throughput
- no gain
- worse latency due to overhead

### Validation

Compare CPU mode vs Vulkan mode using the same:

- model
- input stream
- resolution
- thermal conditions

Do not adopt Vulkan by default unless it improves end-to-end behavior, not just isolated inference time.

## 7. Treat custom OpenCV as a later-stage optimization

### Why

A custom OpenCV build increases build complexity and maintenance cost.

The current code path already uses NCNN-native pixel conversion and resize at the inference boundary, so OpenCV may not be the dominant issue.

### Action

Only pursue a custom OpenCV build after profiling shows preprocessing or image conversion is materially expensive.

If needed, benchmark a custom build with:

- `ENABLE_NEON=ON`
- `WITH_OPENMP=ON` or `WITH_TBB=ON`
- `CMAKE_BUILD_TYPE=Release`

### Expected effect

Potential but uncertain gain.

### Validation

Use measured preprocessing time and total frame latency. Do not justify this change on theoretical grounds alone.

## NCNN Model Optimization Notes

The repository currently loads:

- `/workspace/models/yolo26-best_ncnn_model_int8/model.ncnn.param`
- `/workspace/models/yolo26-best_ncnn_model_int8/model.ncnn.bin`

That suggests the runtime path already targets an INT8 model.

However, the current container does not build NCNN tools, so the quantization and model optimization workflow is not reproducible inside the deployed image.

Recommended clarification:

- document where `model.ncnn.param`, `model.ncnn.bin`, and `model.table` are produced
- document whether `ncnnoptimize` has already been applied to the shipped model
- separate offline model-generation steps from runtime deployment steps

If model generation is part of this repo's intended workflow, a separate document should cover:

1. export
2. optimize
3. calibrate
4. convert to INT8
5. validate accuracy against the deployment dataset

## Recommended Benchmark Plan

Use one fixed input source for all comparisons:

- a recorded ROS bag, or
- a fixed video replay

Keep these constant:

- model files
- camera resolution
- ROS2 graph
- Docker image base
- ambient thermal conditions as much as possible

Benchmark matrix:

1. `num_threads = 1`
2. `num_threads = 2`
3. `num_threads = 3`
4. `num_threads = 4`
5. CPU mode vs Vulkan mode
6. baseline mask decode vs ROI-bounded mask decode
7. baseline telemetry vs reduced polygon payload

Collect:

- inference latency mean / median / p95
- full pipeline latency mean / median / p95
- FPS mean and variance
- CPU utilization per core
- memory usage
- SoC temperature / throttling events
- telemetry payload size

## Recommended Implementation Order

To maximize gain per unit of effort:

1. reduce per-frame logging
2. add profiling breakdowns
3. parameterize thread count
4. benchmark thread settings under real pipeline load
5. optimize `decode_mask()`
6. reduce contour/JSON overhead if still needed
7. benchmark Vulkan
8. consider custom OpenCV only if data justifies it

## Conclusion

The current system already has a reasonable compiler and NCNN baseline for Raspberry Pi 5 CPU inference.

The highest-confidence next steps are not broad rebuild efforts. They are:

- measure the pipeline properly
- remove avoidable runtime overhead
- tune thread usage under realistic ROS2 load
- optimize `decode_mask()` if profiling confirms it is a hotspot

This should produce a more reliable improvement path than assuming Vulkan or a custom OpenCV build will automatically solve the performance problem.
