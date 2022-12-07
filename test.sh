#!/bin/bash
export YAW=$1

# add 180 to the yaw value

export YAW=$((YAW+180))

echo $YAW

# divide by 360 if the value is greater than 360 and take the remainder

export YAW=$((YAW%360))

echo $YAW