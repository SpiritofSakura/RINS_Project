#!/bin/bash
source ~/RINS_Project/install/setup.bash

# Launch the rings test configuration
ros2 launch task1 rings_test.launch.py real_robot:=true display_windows:=true
