#!/bin/bash
export X_INIT=$1
export Y_INIT=$2
export YAW_INIT=$3
export X_GOAL=$4
export Y_GOAL=$5
export YAW_GOAL=$6

killall -9 gzserver

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/gazebo_models_worlds_collection/models
# Launching the PLANNER
g++ ./Navigation/code/A_star.cpp -o ./Navigation/code/a.out
./Navigation/code/a.out $X_INIT $Y_INIT $YAW_INIT $X_GOAL $Y_GOAL $YAW_GOAL

# visualizing the 2D path
python3 ./Navigation/code/visualize.py --type 3D
python3 ./Navigation/code/visualize.py --type 2D

# Launching the robot in gazebo
roslaunch mobile_manipulator mobile_manipulator_gazebo.launch x:=$X_INIT y:=$Y_INIT yaw:=$YAW_INIT &
sleep 15

# performing movement
rosrun mobile_manipulator planToVel ./Navigation/code/office_3d.txt 2