#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

pkill -9 -f rmw_zenohd 2>/dev/null || true
sleep 1

ros2 run rmw_zenoh_cpp rmw_zenohd
