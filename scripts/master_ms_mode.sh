#!/usr/bin/env bash

echo "MS Mode: Master"

killall roslaunch
roslaunch phantomx_rst master_arm.launch &
sleep 10
roslaunch two_robot_control two_robot_control.launch &
