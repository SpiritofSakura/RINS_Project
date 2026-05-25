#!/bin/bash
set -eo pipefail

REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

ARM_ARG="${1:-qr}"
case "$ARM_ARG" in
  garage|left|right|qr|spill|up) ;;
  *)
    echo "Usage: $0 [garage|left|right|qr|spill|up]"
    echo "Default: qr (look_for_qr, arm forward)"
    echo "Tip: try 'left' or 'right' if the forward pose does not see the floor line."
    exit 1
    ;;
esac

case "$ARM_ARG" in
  left)  ARM_POSE="look_at_belt_left" ;;
  right) ARM_POSE="look_at_belt_right" ;;
  qr)    ARM_POSE="look_for_qr" ;;
  spill) ARM_POSE="look_for_spill" ;;
  *)     ARM_POSE="$ARM_ARG" ;;
esac

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export IGN_IP="${IGN_IP:-127.0.0.1}"

CAMERA_TOPIC="${BLUE_LINE_CAMERA_TOPIC:-/top_camera/rgb/preview/image_raw}"
DEBUG_VIEW="${BLUE_LINE_DEBUG_VIEW:-true}"
ECHO_STATUS="${BLUE_LINE_ECHO_STATUS:-true}"
ARM_SETTLE_SECONDS="${ARM_SETTLE_SECONDS:-5}"
LINEAR_SPEED="${BLUE_LINE_LINEAR_SPEED:-0.10}"
MIN_BLUE_PIXELS="${BLUE_LINE_MIN_PIXELS:-90}"
BRANCH_TOP="${BLUE_LINE_BRANCH_TOP:-0.42}"
INTERSECTION_CONFIRM="${BLUE_LINE_INTERSECTION_CONFIRM:-0.50}"

ARM_MOVER_PID=""
OVERLAY_PID=""
STATUS_PID=""

node_running() {
  ros2 node list 2>/dev/null | grep -qx "$1"
}

cleanup() {
  echo
  echo "Stopping blue-line test..."
  timeout 2 ros2 topic pub -1 /blue_line_enabled std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub -1 /cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub -1 /manual_control_active std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1 || true

  pkill -f blue_line_explorer >/dev/null 2>&1 || true

  if [ -n "$OVERLAY_PID" ]; then
    kill "$OVERLAY_PID" >/dev/null 2>&1 || true
  fi
  if [ -n "$STATUS_PID" ]; then
    kill "$STATUS_PID" >/dev/null 2>&1 || true
  fi
  if [ -n "$ARM_MOVER_PID" ]; then
    kill "$ARM_MOVER_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "Blue-line test mode"
echo "  arm pose:     $ARM_POSE"
echo "  camera topic: $CAMERA_TOPIC"
echo "  debug view:   $DEBUG_VIEW"
echo "  status echo:  $ECHO_STATUS"
echo "  branch top:   $BRANCH_TOP"
echo

if ! node_running "/transform_point"; then
  echo "Starting arm command listener..."
  ros2 run dis_tutorial7 arm_mover_actions.py &
  ARM_MOVER_PID="$!"
  sleep 2
else
  echo "Arm command listener already running."
fi

if ! node_running "/robot_state_overlay"; then
  echo "Starting robot_state_overlay for RViz feedback..."
  ros2 run task1 robot_state_overlay &
  OVERLAY_PID="$!"
  sleep 1
else
  echo "robot_state_overlay already running."
fi

echo "Moving arm to '$ARM_POSE'..."
ros2 topic pub --times 3 --rate 1 /arm_command std_msgs/msg/String "{data: '$ARM_POSE'}"
echo "Waiting ${ARM_SETTLE_SECONDS}s for the arm to settle..."
sleep "$ARM_SETTLE_SECONDS"

echo
echo "Starting blue-line explorer. Press Ctrl+C to stop."
echo "RViz overlay should show BLUE_LINE_SEARCH / BLUE_LINE_FOLLOW / BLUE_LINE_INTERSECTION."
echo "Debug image topic: /blue_line/debug_image"
echo "Status topic:      /blue_line/status"
echo

if [ "$ECHO_STATUS" = "true" ]; then
  ros2 topic echo /blue_line/status &
  STATUS_PID="$!"
fi

if node_running "/blue_line_explorer"; then
  echo "blue_line_explorer is already running, enabling it with /blue_line_enabled."
  ros2 param set /blue_line_explorer debug_view "$DEBUG_VIEW" >/dev/null 2>&1 || true
  ros2 param set /blue_line_explorer camera_topic "$CAMERA_TOPIC" >/dev/null 2>&1 || true
  ros2 param set /blue_line_explorer min_blue_pixels "$MIN_BLUE_PIXELS" >/dev/null 2>&1 || true
  ros2 param set /blue_line_explorer roi_top_fraction 0.0 >/dev/null 2>&1 || true
  ros2 param set /blue_line_explorer use_blue_dominance true >/dev/null 2>&1 || true
  ros2 param set /blue_line_explorer branch_roi_top_fraction "$BRANCH_TOP" >/dev/null 2>&1 || true
  ros2 param set /blue_line_explorer intersection_confirm_seconds "$INTERSECTION_CONFIRM" >/dev/null 2>&1 || true
  ros2 topic pub -1 /blue_line_enabled std_msgs/msg/Bool "{data: true}"
  while true; do
    sleep 1
  done
else
  ros2 run task1 blue_line_explorer --ros-args \
    -p enabled_on_start:=true \
    -p start_on_patrol_finished:=false \
    -p camera_topic:="$CAMERA_TOPIC" \
    -p debug_view:="$DEBUG_VIEW" \
    -p min_blue_pixels:="$MIN_BLUE_PIXELS" \
    -p roi_top_fraction:=0.0 \
    -p use_blue_dominance:=true \
    -p branch_roi_top_fraction:="$BRANCH_TOP" \
    -p intersection_confirm_seconds:="$INTERSECTION_CONFIRM" \
    -p linear_speed:="$LINEAR_SPEED"
fi
