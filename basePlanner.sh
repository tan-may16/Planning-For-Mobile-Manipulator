#!/bin/bash
export X_INIT=$1
export Y_INIT=$2
export YAW_INIT=$3
export X_GOAL=$4
export Y_GOAL=$5
export YAW_GOAL=$6
export REPLAN_FLAG=$7
export RESOLUTION=$8
export SIZE=$9

# print the start and goal coordinates
echo "Start coordinates: $X_INIT $Y_INIT $YAW_INIT"
echo "Goal coordinates: $X_GOAL $Y_GOAL $YAW_GOAL"

if [[ $REPLAN_FLAG -eq 0 ]]
then
    echo "Starting the environment"
    killall -9 gzclient & killall -9 gzserver & killall -9 rosmaster
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/gazebo_models_worlds_collection/models
    # Launching the PLANNER
    g++ ./Navigation/code/A_star.cpp -o ./Navigation/code/a.out
    ./Navigation/code/a.out $X_INIT $Y_INIT $YAW_INIT $X_GOAL $Y_GOAL $YAW_GOAL $RESOLUTION

    # visualizing the 2D path
    python3 ./Navigation/code/visualize.py --type 3D
    python3 ./Navigation/code/visualize.py --type 2D

    # Launching the robot in gazebo
    roslaunch mobile_manipulator mobile_manipulator_gazebo.launch x:=$X_INIT y:=$Y_INIT yaw:=$YAW_INIT &
    sleep 15

    # performing movement
    rosrun mobile_manipulator planToVel ./Navigation/code/office_3d.txt $RESOLUTION $SIZE 2
fi

if [[ $REPLAN_FLAG -eq 1 ]]
then
    ./Navigation/code/a.out $X_INIT $Y_INIT $YAW_INIT $X_GOAL $Y_GOAL $YAW_GOAL $RESOLUTION

    # visualizing the 2D path
    python3 ./Navigation/code/visualize.py --type 3D
    python3 ./Navigation/code/visualize.py --type 2D

    # performing movement
    rosrun mobile_manipulator planToVel ./Navigation/code/office_2d.txt $RESOLUTION $SIZE 2
fi
