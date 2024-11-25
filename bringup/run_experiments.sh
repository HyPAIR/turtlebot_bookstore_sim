#!/bin/bash
# Runs the simulation 100 times with proper clean up
# The user can select data, initial, or refined modes

MODE=$1
MONGO_CONNECTION_STRING=$2
DOOR_STATUS_FILE=../params/bookstore_door_setups.yaml

for RUN in {0..99..1};
do
    echo "STARTING RUN $(($RUN + 1))/100"
    if [ $MODE == "data" ]
    then
        echo "RUNNING DATA COLLECTION"
        source ./start_sim.sh $MODE $MONGO_CONNECTION_STRING
    else
        echo "RUNNING $MODE EXPERIMENTS"
        source ./start_sim.sh $MODE $MONGO_CONNECTION_STRING $DOOR_STATUS_FILE $RUN
    fi
    sleep 20s # Wait for everything to be properly setup before waiting
    sh ./termination_checker.sh
done
