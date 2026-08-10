FROM ros:humble-ros-base

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libopencv-dev \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    libomp-dev \
    protobuf-compiler \
    libprotobuf-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install python web server packages
RUN pip3 install fastapi uvicorn websockets


# Clone and build NCNN from source
WORKDIR /tmp
# Pinned to commit benchmarked in Phase H (docs/optimize/ncnn_inference_latency_plan.md §6.8):
# FP16 CPU on this commit measurably beats production baseline 20240820.
RUN mkdir ncnn && cd ncnn && \
    git init && \
    git fetch --depth 1 https://github.com/Tencent/ncnn.git b16501a65edf03828dae55b4d20ccc6a6bf4a3b4 && \
    git checkout FETCH_HEAD && \
    git submodule update --init --depth 1 && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=/usr \
          -DNCNN_VULKAN=OFF \
          -DNCNN_BUILD_EXAMPLES=OFF \
          -DNCNN_BUILD_TOOLS=OFF \
          -DNCNN_BUILD_BENCHMARK=OFF \
          -DNCNN_SHARED_LIB=ON \
          -DNCNN_ARM_NEON=ON \
          .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && rm -rf ncnn

# Set up workspace
WORKDIR /workspace

# Copy ROS2 workspace
COPY ros2_ws /workspace/ros2_ws

# Setup entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
