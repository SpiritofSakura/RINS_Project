#!/bin/bash
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"

ros2 topic pub --once /spill_check_trigger std_msgs/String "data: check"
ros2 service call /spill_check std_srvs/srv/Trigger "{}"
