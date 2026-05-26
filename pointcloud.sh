#!/bin/bash
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export GZ_VERSION=harmonic

ros2 run task1 pointcloud_viewer
