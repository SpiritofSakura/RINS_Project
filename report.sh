#!/bin/bash
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"

NOQR="false"
for arg in "$@"; do
    case "$arg" in
        --noqr)
            NOQR="true"
            ;;
    esac
done

echo "Starting report manager — noqr=$NOQR"
ros2 run task1 report_manager --ros-args -p noqr:="$NOQR"
