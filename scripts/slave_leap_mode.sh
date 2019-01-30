#!/usr/bin/env bash

echo "Leap Mode: Slave"

killall roscore
killall roslaunch
roslaunch phantomx_rst arm.launch
