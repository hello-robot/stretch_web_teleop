#!/bin/bash

stretch_free_robot_process.py;
sleep 2;
udevadm control --reload-rules && udevadm trigger
source /opt/ros/humble/setup.bash
source ~/ament_ws/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
screen -dm -S "ros" ros2 launch stretch_web_teleop web_interface.launch.py certfile:=${HELLO_FLEET_ID}+6.pem keyfile:=${HELLO_FLEET_ID}+6-key.pem map_yaml:=map.yaml 
sleep 3;
~/ament_ws/src/stretch_web_teleop/start_web_server_and_robot_browser.sh
