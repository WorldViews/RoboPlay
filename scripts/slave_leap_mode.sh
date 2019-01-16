#!/usr/bin/env bash

echo "Leap Mode: Slave"

killall roscore
roscore > ./roscore.log &
roslaunch phantomx_rst arm.launch
