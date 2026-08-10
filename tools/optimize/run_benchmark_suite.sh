#!/bin/bash
# AVS Benchmark Suite
# Runs benchmarks cleanly with strict process cleanup via traps.

# ROS2_INSTALL_DIR lets this run against either the dev "install_user" layout
# (native, see CLAUDE.md build layout) or a container's default "install"
# layout (docker-compose/docker run always colcon-builds to ros2_ws/install).
ROS2_INSTALL_DIR="${ROS2_INSTALL_DIR:-install_user}"
if [ -f "ros2_ws/${ROS2_INSTALL_DIR}/setup.bash" ]; then
    source "ros2_ws/${ROS2_INSTALL_DIR}/setup.bash" || { echo "[Error] Failed to source ros2_ws/${ROS2_INSTALL_DIR}/setup.bash."; exit 1; }
else
    echo "[Error] Could not find ros2_ws/${ROS2_INSTALL_DIR}/setup.bash. Benchmark environment setup failed."
    exit 1
fi

PIDS_TO_KILL=""

cleanup() {
    echo "[Cleanup] Stopping background ROS nodes..."
    if [ -n "$PIDS_TO_KILL" ]; then
        kill -SIGINT $PIDS_TO_KILL 2>/dev/null
        sleep 2
        kill -9 $PIDS_TO_KILL 2>/dev/null
        wait $PIDS_TO_KILL 2>/dev/null || true
    fi
}

# Ensure cleanup runs on exit, interrupt, or error
trap cleanup EXIT INT TERM

run_bench() {
  local model_dir=$1
  local mode_name=$2
  local ncnn_flags=$3
  local use_fp16_p=$4
  local use_fp16_s=$5
  local use_fp16_a=$6
  local use_int8=$7
  local use_vulkan=$8
  local input_source=${9:-"video_test1"}
  local SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
  local WS_DIR="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"
  local video_path="${WS_DIR}/test/test_video/${input_source}.mp4"

  echo "=== Running $mode_name ==="
  # Staggered startup (power-safety, see docs/optimize/pi_benchmark_guide.md §8.1):
  # start the inference node ALONE first so model load (large allocations +
  # weight transforms) finishes before any frame arrives, then start the video
  # publisher. This splits one compound idle->peak current step (decode +
  # model load + first inference at once) into two smaller ones.
  # Node output goes to a log file so Vulkan presets can be verified against
  # the actual runtime path (not just the preset label) — see check below.
  local NODE_LOG
  NODE_LOG=$(mktemp /tmp/ncnn_inference_node_XXXXXX.log)
  ros2 run avs_perception ncnn_inference_node --ros-args \
    -p model_param_path:="${WS_DIR}/models/$model_dir/model.ncnn.param" \
    -p model_bin_path:="${WS_DIR}/models/$model_dir/model.ncnn.bin" \
    -p use_fp16_packed:=$use_fp16_p -p use_fp16_storage:=$use_fp16_s -p use_fp16_arithmetic:=$use_fp16_a \
    -p use_int8_inference:=$use_int8 -p use_vulkan_compute:=$use_vulkan -p target_size:=320 -p num_threads:=3 \
    > "$NODE_LOG" 2>&1 &
  local NODE_PID=$!
  PIDS_TO_KILL="$NODE_PID"

  # Let model load settle before frames start flowing (BENCH_STAGGER_SEC to tune)
  sleep "${BENCH_STAGGER_SEC:-10}"
  kill -0 $NODE_PID 2>/dev/null || { echo "[Error] Inference node crashed during model load."; tail -30 "$NODE_LOG"; exit 1; }

  if [ "$use_vulkan" == "true" ]; then
    # yolo26_seg.cpp::set_options logs this and silently falls back to CPU when
    # NCNN was built with NCNN_VULKAN=OFF — results would be CPU numbers
    # mislabeled as Vulkan. Abort instead (guide §13: use arm64-vulkan image).
    if grep -q "compiled without Vulkan support" "$NODE_LOG"; then
      echo "[Error] NCNN here has no Vulkan support — '$mode_name' would run on CPU but be labeled Vulkan."
      echo "        Use the avs_perception:arm64-vulkan image (docs/optimize/pi_benchmark_guide.md §13)."
      exit 1
    fi
    # Surface which Vulkan device NCNN picked (V3D = Pi 5 GPU, llvmpipe = CPU
    # software rasterizer — llvmpipe numbers are NOT GPU numbers).
    grep -iE "V3D|llvmpipe" "$NODE_LOG" | head -5
  fi

  ros2 run avs_perception video_publisher_node --ros-args -p video_path:="$video_path" -p loop:=true &
  local PUB_PID=$!

  # Register PIDs for the trap
  PIDS_TO_KILL="$PUB_PID $NODE_PID"

  # Wait for nodes to initialize
  sleep 5

  # Check if nodes are still alive
  kill -0 $PUB_PID 2>/dev/null || { echo "[Error] Publisher node crashed early."; exit 1; }
  kill -0 $NODE_PID 2>/dev/null || { echo "[Error] Inference node crashed early."; exit 1; }
  
  # Vulkan: first inference compiles SPIR-V pipelines for V3D — can take
  # minutes on the Pi 5 GPU, so give warmup far longer than the CPU default
  # (override with BENCH_WARMUP_TIMEOUT if needed).
  local warmup_timeout
  if [ "$use_vulkan" == "true" ]; then
    warmup_timeout="${BENCH_WARMUP_TIMEOUT:-420}"
  else
    warmup_timeout="${BENCH_WARMUP_TIMEOUT:-90}"
  fi

  ./tools/optimize/run_benchmark.sh --frames 300 --warmup 30 --warmup-timeout "$warmup_timeout" --model "$model_dir" --mode "$mode_name" --input-source "$input_source" --resolution "320" --num-threads 3 --ncnn-flags "$ncnn_flags" --decode-non-control-masks "false" --notes "Automated Benchmark $mode_name"
  local BENCH_EXIT_CODE=$?
  
  if [ $BENCH_EXIT_CODE -ne 0 ]; then
      echo "[Error] Benchmark $mode_name failed with exit code $BENCH_EXIT_CODE"
      echo "--- last 30 lines of inference node log ---"
      tail -30 "$NODE_LOG"
      # Cleanup will be handled by the trap since we exit
      exit $BENCH_EXIT_CODE
  fi

  if [ "$use_vulkan" == "true" ]; then
    echo "--- Vulkan device lines from inference node log ---"
    grep -iE "V3D|llvmpipe" "$NODE_LOG" | head -5 || echo "(none logged)"
  fi
  rm -f "$NODE_LOG"

  # Manually clean up after successful run
  kill -SIGINT $NODE_PID $PUB_PID 2>/dev/null
  sleep 2
  kill -9 $NODE_PID $PUB_PID 2>/dev/null
  wait $NODE_PID 2>/dev/null || true
  wait $PUB_PID 2>/dev/null || true
  PIDS_TO_KILL=""
  
  echo "=== Completed $mode_name. Cooling down... ==="
  sleep 5
}

# Second positional arg selects the input fixture (default: video_test1).
# Lets the same suite run against new fixtures (turn/junction, noisy) once
# they're added under test/test_video/ without editing this script.
FIXTURE="${2:-video_test1}"

# Ensure arguments are provided if user wants to run only specific tests
if [ "$1" == "all" ]; then
    run_bench "best_ncnn_model_int8" "INT8 CPU" "int8=true" "false" "false" "false" "true" "false" "$FIXTURE"
    run_bench "best_ncnn_model" "FP32 CPU" "fp32_all_false" "false" "false" "false" "false" "false" "$FIXTURE"
    run_bench "best_ncnn_model" "FP16 CPU" "fp16_packed,fp16_storage,fp16_arithmetic" "true" "true" "true" "false" "false" "$FIXTURE"
    run_bench "best_ncnn_model" "FP32 Vulkan" "vulkan=true" "false" "false" "false" "false" "true" "$FIXTURE"
    run_bench "best_ncnn_model" "FP16 Vulkan" "vulkan=true,fp16_packed,fp16_storage,fp16_arithmetic" "true" "true" "true" "false" "true" "$FIXTURE"
elif [ "$1" == "b0" ]; then
    run_bench "best_ncnn_model_int8" "INT8 CPU" "int8=true" "false" "false" "false" "true" "false" "$FIXTURE"
elif [ "$1" == "b1" ]; then
    run_bench "best_ncnn_model" "FP32 CPU" "fp32_all_false" "false" "false" "false" "false" "false" "$FIXTURE"
elif [ "$1" == "b2" ]; then
    run_bench "best_ncnn_model" "FP16 CPU" "fp16_packed,fp16_storage,fp16_arithmetic" "true" "true" "true" "false" "false" "$FIXTURE"
elif [ "$1" == "b3" ]; then
    run_bench "best_ncnn_model" "FP32 Vulkan" "vulkan=true" "false" "false" "false" "false" "true" "$FIXTURE"
elif [ "$1" == "b4" ]; then
    run_bench "best_ncnn_model" "FP16 Vulkan" "vulkan=true,fp16_packed,fp16_storage,fp16_arithmetic" "true" "true" "true" "false" "true" "$FIXTURE"
else
    echo "Usage: $0 [all | b0 | b1 | b2 | b3 | b4] [fixture_name]"
    echo "  fixture_name: basename under test/test_video/ without .mp4 (default: video_test1)"
    exit 1
fi
