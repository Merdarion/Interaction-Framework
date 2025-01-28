#!/bin/bash

# Start a new tmux session with a specific name
tmux new-session -d -s bident

# Create the first terminal and run the first command
tmux send-keys -t bident 'ros2 launch ros2_qualisys_driver qualisys.launch.py' C-m

# Create the second terminal and run the second command
tmux split-window -h
tmux send-keys -t bident 'ros2 run llm_response talker' C-m

# Create the third terminal and run the third command
tmux split-window -h
tmux send-keys -t bident 'ros2 run rtsp_stream scene_camera_publisher' C-m

# Select the first pane (to split vertically for kinect_publisher)
tmux select-pane -t 0
tmux split-window -v
tmux send-keys -t bident 'ros2 run kinect_cameras kinect_publisher' C-m

# Select the second pane (to split horizontally for input_fusion)
tmux select-pane -t 2
tmux split-window -h
tmux send-keys -t bident 'ros2 run input_fusion framework' C-m

# Select the third pane (to split horizontally for audio_publisher)
tmux select-pane -t 3
tmux split-window -h
tmux send-keys -t bident 'ros2 run rtsp_stream audio_publisher' C-m

# Select the fourth pane (to split horizontally for actor)
tmux select-pane -t 4
tmux split-window -h
tmux send-keys -t bident 'ros2 run nao_action actor' C-m

# Equalize the pane sizes
tmux select-layout tiled

# Attach to the tmux session to keep it running
tmux attach-session -t bident

