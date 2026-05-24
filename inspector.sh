#!/bin/bash
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export IGN_IP=127.0.0.1

WS="red"
USE_YAML="false"
USE_ORCH="true"

for arg in "$@"; do
    case "$arg" in
        red|green)
            WS="$arg"
            ;;
        --yaml)
            USE_YAML="true"
            USE_ORCH="false"
            ;;
    esac
done

echo "Starting inspector — workstation=$WS use_yaml=$USE_YAML use_orchestrator=$USE_ORCH"
ros2 launch task1 anomaly_inspection.launch.py \
    workstation:="$WS" \
    use_yaml:="$USE_YAML" \
    use_orchestrator:="$USE_ORCH"
