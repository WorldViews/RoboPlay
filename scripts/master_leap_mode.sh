#!/usr/bin/env bash

echo "Leap Mode: Master"

killall roslaunch
roslaunch leap_motion leap_robot_gxyzp.launch
