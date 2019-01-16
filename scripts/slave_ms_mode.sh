#!/usr/bin/env bash

echo "MS Mode: Slave"

killall roscore
roscore > ./roscore.log &
sleep 5
roslaunch phantomx_rst arm.launch
