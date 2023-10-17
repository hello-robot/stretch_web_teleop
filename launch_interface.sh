#!/bin/bash

pkill -f python;
sleep 2;
sudo udevadm control --reload-rules && sudo udevadm trigger
kill -9 $(sudo lsof -t -i:80)
pkill -f "node ./node_modules/.bin/nodemon server.js"
# ros2 daemon stop;
# ros2 daemon start;
# sudo killall -r ros;
# sudo killall -r start_web_server_and_robot_browser;
source /opt/ros/humble/setup.bash
source /home/hello-robot/ament_ws/install/setup.bash
source /home/hello-robot/hello_robot_ws/install/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
screen -dm -S "ros" ros2 launch stretch_teleop_interface web_interface.launch.py certfile:=stretch-re2-2051-local+6.pem keyfile:=stretch-re2-2051-local+6-key.pem map_yaml:=map.yaml 
sleep 5;
screen -dm -S "server" /home/hello-robot/hello_robot_ws/src/stretch_teleop_interface/start_web_server_and_robot_browser.sh localstorage
# sleep 5;
# export NODE_EXTRA_CA_CERTS="/home/hello-robot/hello_robot_ws/src/stretch_teleop_interface/certificates/rootCA.pem"
# cd /home/hello-robot/hello_robot_ws/src/stretch_teleop_interface && npm run localstorage &
# sudo --preserve-env /home/hello-robot/hello_robot_ws/src/stretch_teleop_interface/node_modules/.bin/nodemon server.js &
# /home/hello-robot/hello_robot_ws/src/stretch_teleop_interface/node_modules/.bin/nodemon start_robot_browser.js
# set +x
# ros2 launch stretch_core d435i_high_resolution.launch.py