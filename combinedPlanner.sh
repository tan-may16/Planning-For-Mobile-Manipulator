#!/bin/bash
export X_INIT=$1
export Y_INIT=$2
export YAW_INIT=$3
export X_GOAL=$4
export Y_GOAL=$5
export YAW_GOAL=$6
export THETA_1=$7
export THETA_2=$8
export THETA_3=$9
export THETA_4=${10}
export THETA_5=${11}
replanFlag=0
resolution=10
size=220

./basePlanner.sh $X_INIT $Y_INIT $YAW_INIT $X_GOAL $Y_GOAL $YAW_GOAL $replanFlag $resolution $size

# perform arm movement using RRT Star
# roslaunch mobile_manipulator_moveit_config move_group.launch
mate-terminal --tab --title="moveit" --command="bash -c 'roslaunch mobile_manipulator_moveit_config move_group.launch && sleep 5'" &
sleep 5


mate-terminal --tab --title="armPlanner" -e "bash -c 'rosrun moveit_commander moveit_commander_cmdline.py&& sleep 10'" &
sleep 5
mate-terminal --tab --title="armPlanner_command" -e "bash -c './armPlanner.sh $X_INIT $Y_INIT $YAW_INIT $X_GOAL $Y_GOAL $YAW_GOAL $THETA_1 $THETA_2 $THETA_3 $THETA_4 $THETA_5 && sleep 3'" &
