#!/usr/bin/env bash

echo "MS Mode: Master"

roslaunch phantomx_rst master_arm.launch
roslaunch two_robot_control two_robot_control.launch
