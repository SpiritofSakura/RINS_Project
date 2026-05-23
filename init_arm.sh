#!/bin/bash
source /home/zeta/RINS_Project/install/setup.bash
echo "Starting arm controller listener..."
ros2 run dis_tutorial7 arm_mover_actions.py
