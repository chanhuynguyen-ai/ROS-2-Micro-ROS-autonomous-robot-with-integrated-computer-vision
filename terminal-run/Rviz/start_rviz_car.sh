#!/usr/bin/env bash

WS="$HOME/AVS_Robot_Control_Center/SimpleRobot/ros2_ws"

if [ ! -d "$WS" ]; then
  echo "ERROR: Không thấy workspace: $WS"
  exit 1
fi

SETUP_FILE=""

if [ -f "$WS/install/setup.bash" ]; then
  SETUP_FILE="$WS/install/setup.bash"
elif [ -f "$WS/install_user/setup.bash" ]; then
  SETUP_FILE="$WS/install_user/setup.bash"
elif [ -f "$WS/install_host/setup.bash" ]; then
  SETUP_FILE="$WS/install_host/setup.bash"
else
  echo "ERROR: Không thấy install/setup.bash, install_user/setup.bash hoặc install_host/setup.bash"
  echo "Hãy build workspace trước:"
  echo "  cd $WS"
  echo "  source /opt/ros/humble/setup.bash"
  echo "  colcon build --symlink-install"
  exit 1
fi

ODOM_TF="$HOME/odom_raw_to_tf.py"

# Telemetry -> RViz markers node (lane masks, waypoints, trajectory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LANE_MARKERS="$SCRIPT_DIR/telemetry_to_rviz_markers.py"

if [ ! -f "$ODOM_TF" ]; then
  echo "WARNING: Không thấy $ODOM_TF"
  echo "RViz vẫn chạy được, nhưng sẽ thiếu TF/path từ /odom_raw nếu node này không có."
fi

if [ ! -f "$LANE_MARKERS" ]; then
  echo "WARNING: Không thấy $LANE_MARKERS"
  echo "RViz sẽ thiếu lane masks/waypoints/trajectory markers."
fi

echo "Stopping old RViz car processes..."
pkill -f robot_state_publisher || true
pkill -f joint_state_publisher || true
pkill -f odom_raw_to_tf.py || true
pkill -f telemetry_to_rviz_markers.py || true
pkill -f rviz2 || true

sleep 1

COMMON="cd $WS && source /opt/ros/humble/setup.bash && source $SETUP_FILE && export ROS_DOMAIN_ID=20 && export ROS_LOCALHOST_ONLY=0"

echo "Using workspace: $WS"
echo "Using setup:     $SETUP_FILE"

echo "Starting robot description..."
gnome-terminal --title="car_description" -- bash -lc "
$COMMON
ros2 launch yahboomcar_description description_launch.py
exec bash
"

sleep 2

echo "Starting joint_state_publisher..."
gnome-terminal --title="joint_state_publisher" -- bash -lc "
$COMMON
ros2 run joint_state_publisher joint_state_publisher
exec bash
"

sleep 1

if [ -f "$ODOM_TF" ]; then
  echo "Starting odom_raw_to_tf path node..."
  gnome-terminal --title="odom_path_tf" -- bash -lc "
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
python3 $ODOM_TF
exec bash
"
else
  echo "Skip odom_raw_to_tf.py because file is missing."
fi

sleep 1

# ── Lane Markers Node ────────────────────────────────────────────
# Subscribe /avs/telemetry_realworld -> publish /avs/lane_markers
# Renders lane polygons, waypoints, trajectory as RViz markers
# under the car in base_footprint frame.
if [ -f "$LANE_MARKERS" ]; then
  echo "Starting telemetry -> RViz lane markers node..."
  gnome-terminal --title="lane_markers_rviz" -- bash -lc "
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
python3 $LANE_MARKERS
exec bash
"
else
  echo "Skip telemetry_to_rviz_markers.py because file is missing."
fi

sleep 1

echo "Starting RViz2..."
gnome-terminal --title="rviz_car" -- bash -lc "
$COMMON
rviz2
exec bash
"

echo ""
echo "Started RViz car tools."
echo "Trong RViz đặt:"
echo "  Fixed Frame = odom_frame"
echo "  View Target Frame = odom_frame"
echo "  RobotModel dùng /robot_description"
echo "  Marker path topic = /avs/odom_path_marker"
echo "  Marker arrow topic = /avs/current_heading_arrow"
echo ""
echo "  ── MỚI: Lane Masks / Waypoints / Trajectory ──"
echo "  Add -> By topic -> /avs/lane_markers -> MarkerArray"
echo "  Hoặc Add -> MarkerArray, topic = /avs/lane_markers"
echo "  Markers sẽ hiện các polygon lane (filled), waypoints,"
echo "  trajectory (cyan), và target lookahead (hồng) phía trước xe."
