#!/bin/bash

export NODE_EXTRA_CA_CERTS="/home/hello-robot/ament_ws/src/stretch_web_teleop/certificates/rootCA.pem"
cd ~/ament_ws/src/stretch_web_teleop && pm2 start npm --name="stretch_web_teleop" -- run localstorage 
cd ~/ament_ws/src/stretch_web_teleop && pm2 start server.js --watch
cd ~/ament_ws/src/stretch_web_teleop && pm2 start start_robot_browser.js --watch 
set +x
