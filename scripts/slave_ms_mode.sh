#!/usr/bin/env bash

echo "MS Mode: Slave"

killall roscore
roscore > ./roscore.log &
until rostopic list ; do sleep 1; done
roslaunch phantomx_rst arm.launch
