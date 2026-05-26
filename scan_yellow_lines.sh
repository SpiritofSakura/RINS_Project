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

ARM_MOVER_PID=""

node_running() {
  ros2 node list 2>/dev/null | grep -qx "$1"
}

cleanup() {
  echo
  echo "Yellow line scanner stopped."
  if [ -n "$ARM_MOVER_PID" ]; then
    kill "$ARM_MOVER_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

echo "═══════════════════════════════════════════"
echo "  Yellow Line Map Scanner"
echo "  Arm pose:  $ARM_POSE"
echo "  Map file:  $REPO_DIR/map2.pgm"
echo "═══════════════════════════════════════════"
echo
echo "Drive the robot around room 1 to cover all yellow lines."
echo "Watch the debug window — yellow pixels turn cyan when detected."
echo "Press Ctrl+C when done — the map is saved automatically."
echo

if ! node_running "/transform_point"; then
  echo "Starting arm controller..."
  ros2 run dis_tutorial7 arm_mover_actions.py &
  ARM_MOVER_PID="$!"
  sleep 2
else
  echo "Arm controller already running."
fi

echo "Moving arm to '$ARM_POSE'..."
ros2 topic pub --times 3 --rate 1 /arm_command std_msgs/msg/String "{data: '$ARM_POSE'}"
echo "Waiting ${ARM_SETTLE_SECONDS}s for arm to settle..."
sleep "$ARM_SETTLE_SECONDS"

echo
echo "Starting scanner..."
python3 "$REPO_DIR/scan_yellow_lines.py"
