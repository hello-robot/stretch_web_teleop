#!/bin/bash

pkill -f python;
sleep 2;
udevadm control --reload-rules && udevadm trigger
source /opt/ros/humble/setup.bash
source /home/hello-robot/ament_ws/install/setup.bash
source /home/hello-robot/web_interface_ws/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
screen -dm -S "ros" ros2 launch stretch_teleop_interface web_interface.launch.py certfile:=${HELLO_FLEET_ID}+6.pem keyfile:=${HELLO_FLEET_ID}+6-key.pem map_yaml:=map.yaml 
sleep 3;
/home/hello-robot/web_interface_ws/src/stretch_teleop_interface/start_web_server_and_robot_browser.sh