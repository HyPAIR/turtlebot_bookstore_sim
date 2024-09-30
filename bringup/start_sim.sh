#!/bin/bash
# Starts up the turtlebot bookstore sim

SESSION=turtlebot_bookstore_sim

tmux -2 new-session -d -s $SESSION
tmux rename-window -t $SESSION 'gazebo'
tmux new-window -t $SESSION -n 'navigation'
tmux new-window -t $SESSION -n 'door-manager'

tmux select-window -t $SESSION:gazebo
tmux send-keys "ros2 launch turtlebot_bookstore_sim turtlebot_bookstore.launch.py" C-m

tmux select-window -t $SESSION:navigation
tmux send-keys "ros2 launch turtlebot_bookstore_sim navigator.launch.py" C-m

tmux select-window -t $SESSION:door-manager
tmux send-keys "ros2 launch turtlebot_bookstore_sim doors.launch.py door_yaml:=$(ros2 pkg prefix turtlebot_bookstore_sim)/share/turtlebot_bookstore_sim/maps/bookstore_door_map.yaml initial_status_list:=[open,closed,open,closed,open,closed]" C-m