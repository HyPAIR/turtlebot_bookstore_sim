#!/bin/bash

NODE=$(ros2 node list | grep /policy_executor)
while [ ! -z "${NODE}" ]
do 
    echo "Policy Executor still running..."
    sleep 5s
    NODE=$(ros2 node list | grep /policy_executor)
done

echo "Policy Executor Finished, Terminating." 
# Kill absolutely everything and be extra sure
tmux kill-server
pkill -9 gzserver
pkill -9 gzclient
pkill -9 ros
sleep 10s


