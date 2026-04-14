#!/bin/bash
# Launches the full IRL stack in a tmux session.
# Usage: ./real.sh
# To start patrol after launch: ./patrol.sh

SESSION="rins"

# Kill existing session if running
tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -x 220 -y 50

# Pane 0: Nav2
tmux rename-window -t $SESSION:0 "stack"
tmux send-keys -t $SESSION:0 "bash real1.sh" Enter

# Pane 1: Localization
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "sleep 5 && bash real2.sh" Enter

# Pane 2: task1
tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.2 "sleep 10 && ros2 launch task1 task1.launch.py real_robot:=true" Enter

# Pane 3: RViz
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.3 "sleep 8 && bash real3.sh" Enter

tmux attach-session -t $SESSION
