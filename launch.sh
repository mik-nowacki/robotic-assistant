#!/bin/bash


set -x

tmux new-session -d -s "ros"

tmux send-keys -t "ros" "source /opt/ros/kinetic/setup.bash" C-m
tmux send-keys -t "ros" "source ~/ros_catkin_ws/devel/setup.bash" C-m
tmux send-keys -t "ros" "roscore" C-m

sleep 5

tmux new-window -t "ros" -n "camera"
tmux send-keys -t "ros:camera" "source /opt/ros/kinetic/setup.bash" C-m
tmux send-keys -t "ros:camera" "source ~/ros_catkin_ws/devel/setup.bash" C-m
tmux send-keys -t "ros:camera" "rosrun myCobotROS camera_node.py" C-m

sleep 3

tmux new-window -t "ros" -n "robot"
tmux send-keys -t "ros:robot" "source /opt/ros/kinetic/setup.bash" C-m
tmux send-keys -t "ros:robot" "source ~/ros_catkin_ws/devel/setup.bash" C-m
tmux send-keys -t "ros:robot" "rosrun myCobotROS robot_node.py" C-m

tmux select-window -t "ros:0"
tmux attach-session -t "ros"
