#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
source ~/ROS/project/RINS_Project/install/setup.bash

ukaz="$1"

case "$ukaz" in
    patrol_on)
        ros2 topic pub /patrol_command std_msgs/msg/Bool "{data: true}" -1
        ;;
    patrol_off)
        ros2 topic pub /patrol_command std_msgs/msg/Bool "{data: false}" -1
        ;;
    manual_on)
        ros2 topic pub /manual_control_active std_msgs/msg/Bool "{data: true}" -1
        ;;
    manual_off)
        ros2 topic pub /manual_control_active std_msgs/msg/Bool "{data: false}" -1
        ;;
    idle)
        ros2 topic pub /manual_control_active std_msgs/msg/Bool "{data: false}" -1
        ros2 topic pub /patrol_command std_msgs/msg/Bool "{data: false}" -1
        ;;
    *)
        echo "Usage: $0 {patrol_on|patrol_off|manual_on|manual_off|idle}"
        exit 1
        ;;
esac