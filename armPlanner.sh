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

replanFlag=1

xdotool key ctrl+Page_Up
xdotool type "use arm"
xdotool key Return;
xdotool type "rec c"
xdotool key Return;
xdotool type "goal = c"
xdotool key Return;
xdotool type "goal[0] = $THETA_1"
xdotool key Return;
xdotool type "goal[1] = $THETA_2"
xdotool key Return;
xdotool type "goal[2] = $THETA_3"
xdotool key Return;
xdotool type "goal[3] = $THETA_4"
xdotool key Return;
xdotool type "goal[4] = $THETA_5"
xdotool key Return;
xdotool type "go goal"
xdotool key Return;
sleep 10
xdotool type "go c"
xdotool key Return;
sleep 10
xdotool type "exit"
xdotool key Return

# YAW_INIT is converted to negative because the robot is facing the opposite direction in gazebo

export YAW_INIT=$((YAW_INIT+180))
export YAW_INIT=$((YAW_INIT%360))

# YAW_GOAL is converted to negative because the robot is facing the opposite direction in gazebo

export YAW_GOAL=$((YAW_GOAL+180))
export YAW_GOAL=$((YAW_GOAL%360))

./basePlanner.sh $X_GOAL $Y_GOAL $YAW_GOAL $X_INIT $Y_INIT $YAW_INIT $replanFlag
