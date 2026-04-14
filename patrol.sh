#!/bin/bash
ros2 topic pub --once /patrol_command std_msgs/msg/Bool "{data: true}"
