#!/bin/bash
set -e

# Source ROS 2 Humble setup
source "/opt/ros/humble/setup.bash"

exec "$@"
