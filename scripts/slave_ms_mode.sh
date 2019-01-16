#!/usr/bin/env bash

echo "MS Mode: Slave"

killall roscore
roscore > ./roscore.log &
roslaunch phantomx_rst arm.launch
