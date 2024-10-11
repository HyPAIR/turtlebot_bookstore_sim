#!/bin/bash
# Starts up the turtlebot bookstore sim

SESSION=turtlebot_bookstore_sim
POLICY_MODE=$1
MONGO_CONNECTION_STRING=$2


tmux -2 new-session -d -s $SESSION
tmux rename-window -t $SESSION 'gazebo'
tmux new-window -t $SESSION -n 'navigation'
tmux new-window -t $SESSION -n 'door-manager'
tmux new-window -t $SESSION -n 'policy-executor'

tmux select-window -t $SESSION:gazebo
tmux send-keys "ros2 launch turtlebot_bookstore_sim turtlebot_bookstore.launch.py" C-m

tmux select-window -t $SESSION:navigation
tmux send-keys "ros2 launch turtlebot_bookstore_sim navigator.launch.py" C-m

tmux select-window -t $SESSION:door-manager
tmux send-keys "ros2 launch turtlebot_bookstore_sim doors.launch.py" C-m

tmux select-window -t $SESSION:policy-executor
# TODO: Deal with DB Collection names here
tmux send-keys "ros2 launch turtlebot_bookstore_sim policy_executor.launch.py db_collection:=bookstore-$POLICY_MODE mode:=$POLICY_MODE db_connection_string:=$MONGO_CONNECTION_STRING" C-m