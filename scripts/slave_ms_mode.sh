#!/usr/bin/env bash

echo "MS Mode: Slave"

killall roscore
killall roslaunch
roslaunch phantomx_rst arm.launch
