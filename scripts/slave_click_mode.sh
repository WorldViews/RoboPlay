#!/usr/bin/env bash

echo "Click Mode: Slave"

killall roscore
killall roslaunch
roscore &
roslaunch phantomx_rst arm.launch &
roslaunch rosbridge_server rosbridge_websocket.launch &
roslaunch hit_based_robot_control hit_num_robot_control.launch
