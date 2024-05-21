#!/bin/bash

# Usage: ./launch_interface.sh -m $HELLO_FLEET_PATH/maps/<map_name>.yaml
MAP_ARG=""
if getopts ":m:" opt && [[ $opt == "m" && -f $OPTARG ]]; then
    echo "Setting map..."
    MAP_ARG="map_yaml:=$OPTARG"
fi

stretch_free_robot_process.py;
./stop_interface.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
sleep 2;
screen -dm -S "web_teleop_ros" ros2 launch stretch_web_teleop web_interface.launch.py $MAP_ARG
sleep 3;
~/ament_ws/src/stretch_web_teleop/start_web_server_and_robot_browser.sh
