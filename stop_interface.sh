#!/bin/bash

# Usage: ./stop_interface.sh
screen -S "web_teleop_ros" -X stuff '^C'
if [[ $? -ne 0 ]]; then
    echo "Using pkill"
    sudo pkill screen
fi
pm2 kill
