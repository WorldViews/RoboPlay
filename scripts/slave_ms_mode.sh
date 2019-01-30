#!/usr/bin/env bash

echo "MS Mode: Slave"

killall roscore
roslaunch phantomx_rst arm.launch
