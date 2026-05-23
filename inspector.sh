#!/bin/bash

# Usage: ./inspector.sh [green|red]
# Reads an existing test_workstation_locations.yaml,
# navigates to the workstation and inspects tiles.

COLOR="${1:-green}"

if [[ "$COLOR" != "green" && "$COLOR" != "red" ]]; then
    echo "Usage: $0 [green|red]"
    exit 1
fi

source /home/zeta/RINS_Project/install/setup.bash

ros2 launch task1 anomaly_inspection.launch.py workstation:="$COLOR"
