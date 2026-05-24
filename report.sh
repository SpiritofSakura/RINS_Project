#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOPIC="/report_commands"
MSG_TYPE="std_msgs/msg/String"

usage() {
    echo "Usage: $0 <command>"
    echo ""
    echo "Commands:"
    echo "  make [--no-increment]    Generate an inspection report"
    echo "  clear                    Clear all reports and reset state"
    echo "  images                   Generate a PDF from saved defect images (no ROS needed)"
    echo "  launch                   Start the report_manager node"
}

source_ros() {
    if [ -z "$ROS_DISTRO" ]; then
        source /opt/ros/jazzy/setup.bash 2>/dev/null
    fi
    if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
        source "$SCRIPT_DIR/install/setup.bash"
    fi
}

do_make() {
    source_ros
    MSG="make"
    if [[ "$2" == "--no-increment" ]]; then
        MSG="make --no-increment"
    fi
    ros2 topic pub --once "$TOPIC" "$MSG_TYPE" "data: '$MSG'"
}

do_clear() {
    source_ros
    ros2 topic pub --once "$TOPIC" "$MSG_TYPE" "data: 'clear'"
}

do_images() {
    python3 "$SCRIPT_DIR/generate_image_report.py"
}

do_launch() {
    source_ros
    echo "Starting report_manager..."
    ros2 run task1 report_manager --ros-args -p report_dir:="$SCRIPT_DIR/reports"
}

case "${1:-}" in
    make)   do_make ;;
    clear)  do_clear ;;
    images) do_images ;;
    launch) do_launch ;;
    *)      usage ;;
esac
