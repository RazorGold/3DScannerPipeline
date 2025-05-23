#!/bin/bash

SESSION="ros3d"

# Start new tmux session
tmux new-session -d -s $SESSION

# Set up first pane: central_control
tmux send-keys -t $SESSION "source ~/ros_ws/install/setup.bash && ros2 run central_control central_node" C-m

# Split vertically: turntable
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "source ~/ros_ws/install/setup.bash && ros2 run turntable turntable_node" C-m

# Split horizontally: robotic_arm
tmux split-window -h -t $SESSION:0.1
tmux send-keys -t $SESSION "source ~/ros_ws/install/setup.bash && ros2 run robot_arm robot_node" C-m

# Go to pane 0, split horizontally: capture
tmux select-pane -t $SESSION:0.0
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "source ~/ros_ws/install/setup.bash && ros2 run capture capture_node" C-m

# Go to pane 1, split vertically: meshing
tmux select-pane -t $SESSION:0.1
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "source ~/ros_ws/install/setup.bash && ros2 run mesh meshing_node" C-m

# Go to pane 2, split vertically: user_interface
tmux select-pane -t $SESSION:0.2
tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "source ~/ros_ws/install/setup.bash && ros2 run user_interface ui_node" C-m

# Final pane: Flask web server
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd ~/ros_ws/UI && python3 server.py" C-m

# Attach to the session
tmux select-layout tiled
tmux attach -t $SESSION