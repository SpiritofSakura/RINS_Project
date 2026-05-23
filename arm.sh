#!/bin/bash
CMD="${1:-garage}"

case "$CMD" in
  garage|left|right|qr|spill|up) ;;
  *)
    echo "Usage: $0 [garage|left|right|qr|spill|up]"
    echo "  (default: garage)"
    exit 1
    ;;
esac

# Map short names to full pose names
case "$CMD" in
  left)  POSE="look_at_belt_left" ;;
  right) POSE="look_at_belt_right" ;;
  qr)    POSE="look_for_qr" ;;
  spill) POSE="look_for_spill" ;;
  *)     POSE="$CMD" ;;
esac

source /home/zeta/RINS_Project/install/setup.bash
ros2 topic pub -1 /arm_command std_msgs/String "data: '$POSE'"
