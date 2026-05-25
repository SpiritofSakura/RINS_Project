#!/bin/bash
echo "Killing all ROS, Gazebo and project node processes..."

# Project-compiled C++ nodes (cylinder_segmentation etc.) live here, NOT in /opt/ros
PROJECT_PIDS=$(ps aux | grep '/home/zeta/RINS_Project/install' | grep -v grep | awk '{print $2}')

# ROS system processes
ROS_PIDS=$(ps aux | grep '/opt/ros/jazzy' | grep -v grep | awk '{print $2}')

# Gazebo GUI and server
GZ_GUI_PIDS=$(ps aux | grep -E '\bgz\b' | grep -E '\bgui\b' | grep -v grep | awk '{print $2}')
GZ_SERVER_PIDS=$(ps aux | grep -E '\bgz\b' | grep -E '\bserver\b' | grep -v grep | awk '{print $2}')

# Combine and deduplicate
ALL_PIDS=$(echo -e "$PROJECT_PIDS\n$ROS_PIDS\n$GZ_GUI_PIDS\n$GZ_SERVER_PIDS" \
    | sort -u | grep -v '^$' | tr '\n' ' ' | xargs)

if [ -n "$ALL_PIDS" ]; then
    echo "Killing PIDs: $ALL_PIDS"
    echo "$ALL_PIDS" | xargs -r kill -9
else
    echo "No binary ROS/Gazebo processes found."
fi

# Always kill Python3 nodes regardless of the above
echo "Killing Python3 ROS nodes..."
pkill -9 -f python3 2>/dev/null || true

# Kill zenoh router
echo "Killing zenoh router..."
pkill -9 -f rmw_zenohd 2>/dev/null || true

sleep 1

# Stop ROS daemon cleanly after everything else is dead
echo "Stopping ROS daemon..."
ros2 daemon stop 2>/dev/null || true

sleep 1

echo "Verifying — remaining processes:"
REMAINING=$(ps aux | grep -E '/home/zeta/RINS_Project/install|/opt/ros/jazzy|\bgz\b|rmw_zenohd' | grep -v grep)
if [ -z "$REMAINING" ]; then
    echo "All clean."
else
    echo "$REMAINING"
fi
