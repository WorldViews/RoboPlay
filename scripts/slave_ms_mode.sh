#!/usr/bin/env bash

echo "MS Mode: Slave"

killall roscore
roscore > ./roscore.log &
sleep 5
while [ 1 ] ; do
        echo 'begin checking for roscore'
        check_process "roscore" # the thread name
        CHECK_RET = $?
        if [ $CHECK_RET -eq 1 ]; # exists
        then
          roslaunch phantomx_rst arm.launch
        fi
        sleep 10
done
# roslaunch phantomx_rst arm.launch
