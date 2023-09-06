#!/bin/bash

tmux new-session -d

tmux send-keys "clear" C-m
tmux send-keys "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"

tmux split-window -h

tmux send-keys "clear" C-m
tmux send-keys "ros2 launch ike_launch ike_nav.launch.xml"

tmux attach-session
