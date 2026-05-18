#!/bin/bash
source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export GZ_VERSION=harmonic

colcon build --symlink-install --merge-install
