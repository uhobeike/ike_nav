#!/bin/bash

tmux new-session -d

tmux send-keys "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" C-m

tmux split-window -h

tmux send-keys "ros2 launch ike_launch ike_nav.launch.xml" C-m

tmux attach-session
