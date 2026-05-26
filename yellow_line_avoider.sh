#!/bin/bash
set -eo pipefail

REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export IGN_IP="${IGN_IP:-127.0.0.1}"

ARM_POSE="look_for_spill"
ARM_SETTLE_SECONDS="${ARM_SETTLE_SECONDS:-5}"
CAMERA_TOPIC="${YELLOW_LINE_CAMERA_TOPIC:-/top_camera/rgb/preview/image_raw}"
BACK_SPEED="${YELLOW_LINE_BACK_SPEED:-0.12}"
BACK_DURATION="${YELLOW_LINE_BACK_DURATION:-1.8}"

ARM_MOVER_PID=""
RQT_PID=""
OVERLAY_PID=""

node_running() {
  ros2 node list 2>/dev/null | grep -qx "$1"
}

cleanup() {
  echo
  echo "Stopping yellow-line avoider..."
  timeout 2 ros2 topic pub -1 /cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  pkill -f yellow_line_avoider >/dev/null 2>&1 || true
  if [ -n "$RQT_PID" ]; then
    kill "$RQT_PID" >/dev/null 2>&1 || true
  fi
  if [ -n "$OVERLAY_PID" ]; then
    kill "$OVERLAY_PID" >/dev/null 2>&1 || true
  fi
  if [ -n "$ARM_MOVER_PID" ]; then
    kill "$ARM_MOVER_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "═══════════════════════════════════════════"
echo "  Yellow Line Avoider"
echo "  Arm pose:     $ARM_POSE"
echo "  Camera topic: $CAMERA_TOPIC"
echo "  Back speed:   $BACK_SPEED m/s  for  $BACK_DURATION s"
echo "═══════════════════════════════════════════"
echo

if ! node_running "/transform_point"; then
  echo "Starting arm command listener..."
  ros2 run dis_tutorial7 arm_mover_actions.py &
  ARM_MOVER_PID="$!"
  sleep 2
else
  echo "Arm command listener already running."
fi

echo "Moving arm to '$ARM_POSE'..."
ros2 topic pub --times 3 --rate 1 /arm_command std_msgs/msg/String "{data: '$ARM_POSE'}"
echo "Waiting ${ARM_SETTLE_SECONDS}s for the arm to settle..."
sleep "$ARM_SETTLE_SECONDS"

if ! node_running "/robot_state_overlay"; then
  echo "Starting robot_state_overlay..."
  ros2 run task1 robot_state_overlay &
  OVERLAY_PID="$!"
  sleep 1
else
  echo "robot_state_overlay already running."
fi

echo "Opening camera debug window..."
ros2 run rqt_image_view rqt_image_view /yellow_line/debug_image &
RQT_PID="$!"
sleep 1

echo
echo "Starting yellow-line avoider. Press Ctrl+C to stop."
echo "Debug window: /yellow_line/debug_image"
echo "Status:       ros2 topic echo /yellow_line_status"

ros2 run task1 yellow_line_avoider --ros-args \
  -p camera_topic:="$CAMERA_TOPIC" \
  -p back_speed:="$BACK_SPEED" \
  -p back_duration:="$BACK_DURATION"
