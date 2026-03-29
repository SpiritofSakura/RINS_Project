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
    target_done)
        ros2 topic pub /target_done std_msgs/msg/Empty "{}" -1
        ;;
    resume_patrol)
        ros2 topic pub /resume_patrol std_msgs/msg/Empty "{}" -1
        ;;
    *)
        echo "Usage: $0 {patrol_on|patrol_off|manual_on|manual_off|idle|target_done|resume_patrol}"
        exit 1
        ;;
esac