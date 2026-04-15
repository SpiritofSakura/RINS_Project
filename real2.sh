#!/bin/bash
ros2 run laser_filters scan_to_scan_filter_chain \
  --ros-args \
  --params-file ~/RINS_Project/src/dis_tutorial3/config/laser_filter_chain.yaml \
  --remap /scan_filtered:=scan_filtered &

ros2 launch turtlebot4_navigation localization.launch.py \
  map:=/home/zeta/RINS_Project/src/dis_tutorial3/maps/map_name.yaml \
  params:=/home/zeta/RINS_Project/src/dis_tutorial3/config/localization_irl.yaml

wait
