#!/usr/bin/env bash

echo "Leap Mode: Slave"

killall roscore
killall roslaunch
roscore > ./roscore.log &
roslaunch phantomx_rst arm.launch
