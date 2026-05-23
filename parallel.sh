#!/bin/bash

# View-only: shows the derivative + Hough line window without moving the robot.
# Remove --ros-args -p view_only:=False to enable alignment.

source /home/zeta/RINS_Project/install/setup.bash

ros2 run task1 parallel_align --ros-args -p view_only:=True
