#!/bin/bash
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export IGN_IP=127.0.0.1

ros2 launch task1 task1.launch.py
