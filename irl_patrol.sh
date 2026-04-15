#!/bin/bash
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /patrol_command std_msgs/msg/Bool "{data: true}"
