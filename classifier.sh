#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"
source install/setup.bash

cleanup() {
    echo "Shutting down..." >&2
    kill "$detect_pid" "$classifier_pid" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "Starting tile_detect (debug mode)..."
ros2 run task1 tile_detect --ros-args -p debug_mode:=true &
detect_pid=$!

echo "Starting tile_classifier..."
ros2 run task1 tile_classifier &
classifier_pid=$!

echo "Both nodes running. Press Ctrl+C to stop."
wait
