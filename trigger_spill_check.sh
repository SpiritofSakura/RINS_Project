#!/bin/bash
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 service call /spill_check std_srvs/srv/Trigger
