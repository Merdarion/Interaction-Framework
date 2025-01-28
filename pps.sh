#!/bin/bash

# Start a new tmux session with a specific name
tmux new-session -d -s bident

# Create the first terminal and run the first command
tmux send-keys -t bident 'ros2 launch ros2_qualisys_driver qualisys.launch.py' C-m

# Create the first terminal and run the first command
tmux split-window -h
tmux send-keys -t bident 'ros2 run llm_response talker' C-m

# Split the window horizontally for the second terminal
tmux split-window -h
tmux send-keys -t bident 'ros2 run rtsp_stream scene_camera_publisher' C-m

# Select the first pane to split horizontally for the fourth terminal
tmux select-pane -t 0
tmux split-window -v
tmux send-keys -t bident 'ros2 run kinect_cameras kinect_publisher' C-m

# Go to the next pane (created earlier) and split for input fusion
tmux select-pane -t 2
tmux split-window -h
tmux send-keys -t bident 'ros2 run input_fusion pre_programmed_schedule' C-m

# Go to pane 3 and split horizontally for audio_publisher
tmux select-pane -t 3
tmux split-window -h
tmux send-keys -t bident 'ros2 run rtsp_stream audio_publisher' C-m

# Split the next pane for the actor
tmux select-pane -t 4
tmux split-window -h
tmux send-keys -t bident 'ros2 run nao_action actor' C-m

# Equalize the pane sizes
tmux select-layout tiled

# Attach to the tmux session to keep it running
tmux attach-session -t bident
