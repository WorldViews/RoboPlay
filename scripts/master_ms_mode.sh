#!/usr/bin/env bash

echo "MS Mode: Master"

killall roslaunch
roslaunch phantomx_rst master_arm.launch &
until rostopic list ; do sleep 1; done
roslaunch two_robot_control two_robot_control.launch &
