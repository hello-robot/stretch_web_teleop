#!/bin/bash

# Usage: ./stop_interface.sh
# TODO: The below command(s) sometimes doesn't fully terminate
# the realsense_ros nodes. Then, when we re-launch the web interface,
# realsense_ros fails to connect, and moving forward it can't
# find the D435 until we power cycle the robot. The face that
# power-cycling is necessary is a bug (e.g., with realsense code),
# but this would not arise if this script properly terminates
# realsense nodes.
screen -S "web_teleop_ros" -X stuff '^C'
t1=$?
sleep 3;
if [[ $t1 -ne 0 ]]; then
    echo "Using pkill"
    sudo pkill screen
fi
pm2 kill
