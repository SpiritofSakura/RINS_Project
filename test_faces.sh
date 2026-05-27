#!/bin/bash
set -eo pipefail

REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"


echo "Face detection test"
echo "  Launching face_test.launch.py (all 3 face nodes + behavior_manager)"
echo

ros2 launch task1 face_test.launch.py
